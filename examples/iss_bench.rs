//! Instruction-set simulator throughput on four synthetic programs.
//!
//! `cargo run --release --example iss_bench [-- <steps>]`
//!
//! Each program is a small hand-assembled loop run in SoC mode (caches, timers
//! and the peripheral bank ticking, as in a firmware run) for `steps` steps
//! (default 50 M); `ISS_BENCH_ONLY=<name>` runs one program. Before timing, every program is disassembled and run for a
//! few thousand steps with its expected effect checked, so a broken encoding
//! fails loudly instead of benchmarking a runaway PC.

use std::time::Instant;

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::decoder::decode_bytes;
use bcm55030_emulator::decoder::format::format_instruction;
use bcm55030_emulator::soc::bank::BootMode;

const CODE: u32 = 0x1000;
const BUF: u32 = 0x8000;

// --- encoders (ARCompact, big-endian) --------------------------------------

/// 32-bit ALU form: P=00 `a = b op c`, P=01 `a = b op u6`, P=11 `b = b op c` if cc.
fn alu(sub: u32, b: u32, c: u32, a: u32, p: u32, f: bool) -> u32 {
    (0x04 << 27) | ((b & 7) << 24) | ((p & 3) << 22) | ((sub & 0x3F) << 16) | ((f as u32) << 15)
        | (((b >> 3) & 7) << 12) | ((c & 0x3F) << 6) | (a & 0x3F)
}
/// Conditional register form: `b = b op c` when `cc` holds.
fn alu_cc(sub: u32, b: u32, c: u32, cc: u32, f: bool) -> u32 {
    alu(sub, b, c, cc & 0x1F, 0b11, f)
}
fn ld(a: u32, b: u32, off: i32, zz: u32) -> u32 {
    let s9 = (off as u32) & 0x1FF;
    (0x02 << 27) | ((b & 7) << 24) | ((s9 & 0xFF) << 16) | ((s9 >> 8) << 15) | (((b >> 3) & 7) << 12)
        | ((zz & 3) << 7) | (a & 0x3F)
}
fn st(c: u32, b: u32, off: i32, zz: u32) -> u32 {
    let s9 = (off as u32) & 0x1FF;
    (0x03 << 27) | ((b & 7) << 24) | ((s9 & 0xFF) << 16) | ((s9 >> 8) << 15) | (((b >> 3) & 7) << 12)
        | ((c & 0x3F) << 6) | ((zz & 3) << 1)
}
/// `b[.d] s21`, unconditional, relative to PCL.
fn b(off: i32, delay: bool) -> u32 {
    let d = off as u32;
    ((d >> 1) & 0x3FF) << 17 | ((d >> 11) & 0x3FF) << 6 | (delay as u32) << 5
}
/// `brcc b, c, s9` (register form), relative to PCL. `cc`: 0 EQ, 1 NE, 2 LT, 3 GE.
fn brcc(b: u32, c: u32, off: i32, cc: u32) -> u32 {
    let s9 = (off as u32) & 0x1FF;
    (0x01 << 27) | ((b & 7) << 24) | (((s9 >> 1) & 0x7F) << 17) | (1 << 16) | ((s9 >> 8) << 15)
        | (((b >> 3) & 7) << 12) | ((c & 0x3F) << 6) | (cc & 0xF)
}
/// `lp` with a u6 half-word offset from PCL to LP_END.
fn lp(u6: u32) -> u32 {
    alu(0x28, 0, u6, 0, 0b01, false)
}
/// `b_s s10`, relative to PCL.
fn b_s(off: i32) -> u16 {
    (0x1E << 11) | (((off >> 1) as u16) & 0x1FF)
}

const ADD: u32 = 0x00;
const SUB: u32 = 0x02;
const AND: u32 = 0x04;
const OR: u32 = 0x05;
const XOR: u32 = 0x07;
const MAX: u32 = 0x08;
const MOV: u32 = 0x0A;
const ADD2: u32 = 0x15;
const LIMM: u32 = 62;
const LP_COUNT: u32 = 60;
const CC_NE: u32 = 0x02;

struct Prog {
    name: &'static str,
    bytes: Vec<u8>,
    setup: fn(&mut Cpu),
    check: fn(&Cpu) -> bool,
}

struct Asm(Vec<u8>);
impl Asm {
    fn w(&mut self, w: u32) -> &mut Self {
        self.0.extend_from_slice(&w.to_be_bytes());
        self
    }
    fn h(&mut self, h: u16) -> &mut Self {
        self.0.extend_from_slice(&h.to_be_bytes());
        self
    }
    fn at(&self) -> i32 {
        self.0.len() as i32
    }
}

fn programs() -> Vec<Prog> {
    // (a) ALU mix with flags and a conditional form.
    let mut a = Asm(Vec::new());
    a.w(alu(ADD, 1, 2, 3, 0, true))
        .w(alu(SUB, 3, 1, 4, 0, true))
        .w(alu(AND, 3, 2, 5, 0, false))
        .w(alu_cc(OR, 6, 2, CC_NE, false))
        .w(alu(XOR, 5, 4, 7, 0, true))
        .w(alu(MAX, 1, 2, 8, 0, true))
        .w(alu(ADD, 1, 1, 1, 0b01, false));
    let back = -(a.at() & !3);
    a.h(b_s(back));

    // (b) SRAM loads and stores, cycling through a 256-byte buffer.
    let mut m = Asm(Vec::new());
    m.w(alu(MOV, 10, LIMM, 0, 0, false)).w(BUF);
    let top = m.at();
    m.w(alu(AND, 1, 0x3F, 11, 0b01, false))
        .w(alu(ADD2, 10, 11, 12, 0, false))
        .w(st(1, 12, 0, 0))
        .w(ld(2, 12, 0, 0))
        .w(st(2, 12, 4, 0))
        .w(ld(3, 12, 4, 0))
        .w(ld(4, 12, 1, 1))
        .w(alu(ADD, 1, 1, 1, 0b01, false));
    let back = top - (m.at() & !3);
    m.h(b_s(back));

    // (c) Compare-and-branch and a delayed branch with a filled slot.
    let mut c = Asm(Vec::new());
    c.w(alu(ADD, 1, 1, 1, 0b01, false)); // 0
    c.w(brcc(1, 2, 8, 1)); // 4: brne r1, r2, +8 -> 12
    c.w(alu(MOV, 1, 0, 0, 0b01, false)); // 8: r1 = 0
    c.w(b(-12, true)); // 12: b.d 0
    c.w(alu(ADD, 3, 1, 3, 0b01, false)); // 16: delay slot

    // (d) Zero-overhead loop, re-armed by an outer branch.
    let mut z = Asm(Vec::new());
    z.w(alu(MOV, LP_COUNT, 2, 0, 0, false)); // 0: lp_count = r2
    z.w(lp(6)); // 4: LP_END = PCL(4) + 12 = 16
    z.w(alu(ADD, 1, 1, 1, 0b01, false)); // 8
    z.w(alu(ADD, 3, 1, 3, 0, false)); // 12
    z.h(b_s(-16)); // 16: b_s 0

    vec![
        Prog {
            name: "alu",
            bytes: a.0,
            setup: |cpu| {
                cpu.state.core_regs[1] = 1;
                cpu.state.core_regs[2] = 3;
            },
            check: |cpu| cpu.state.core_regs[1] > 100,
        },
        Prog {
            name: "mem",
            bytes: m.0,
            setup: |_| {},
            check: |cpu| cpu.state.core_regs[1] > 100 && cpu.state.core_regs[3] == cpu.state.core_regs[2],
        },
        Prog {
            name: "branch",
            bytes: c.0,
            setup: |cpu| cpu.state.core_regs[2] = 50,
            check: |cpu| cpu.state.core_regs[3] > 100 && cpu.state.core_regs[1] <= 50,
        },
        Prog {
            name: "zol",
            bytes: z.0,
            setup: |cpu| cpu.state.core_regs[2] = 1000,
            // The body runs lp_count times per pass, the outer branch re-arms it.
            check: |cpu| cpu.state.core_regs[1] > 1000,
        },
    ]
}

fn fresh(p: &Prog) -> Cpu {
    let mut cpu = Cpu::new_bcm55030(BootMode::Cold);
    cpu.mem.load_binary(CODE, &p.bytes);
    cpu.state.pc = CODE;
    (p.setup)(&mut cpu);
    cpu
}

fn main() {
    let steps: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(50_000_000);
    let end = |p: &Prog| CODE + p.bytes.len() as u32;
    let only = std::env::var("ISS_BENCH_ONLY").ok();
    for p in programs() {
        if only.as_deref().is_some_and(|o| o != p.name) {
            continue;
        }
        // Disassemble, then check the expected effect over a short run.
        let mut pc = CODE;
        while pc < end(&p) {
            let d = decode_bytes(pc, &p.bytes, CODE).unwrap_or_else(|e| panic!("{}: {e:?} at {pc:#x}", p.name));
            if std::env::var_os("ISS_BENCH_LIST").is_some() {
                println!("  {:>6} {pc:#06x}  {}", p.name, format_instruction(&d));
            }
            pc += d.total_size();
        }
        let mut cpu = fresh(&p);
        for _ in 0..20_000 {
            cpu.step().unwrap_or_else(|e| panic!("{}: {e:?}", p.name));
            assert!((CODE..end(&p)).contains(&cpu.state.pc), "{}: PC left the program", p.name);
        }
        assert!((p.check)(&cpu), "{}: expected effect not observed", p.name);

        let mut cpu = fresh(&p);
        let t = Instant::now();
        for _ in 0..steps {
            let _ = cpu.step();
        }
        let s = t.elapsed().as_secs_f64();
        assert!((CODE..end(&p)).contains(&cpu.state.pc), "{}: PC left the program", p.name);
        println!(
            "{:>6}: {steps} steps in {s:.3} s = {:.1} M steps/s (insns {})",
            p.name,
            steps as f64 / s / 1e6,
            cpu.state.instruction_count
        );
    }
}
