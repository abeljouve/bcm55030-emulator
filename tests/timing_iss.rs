//! Phase-B integration test: the cycle-accurate contention overlay driving the
//! REAL ISS (decoder + executor + memory), not the standalone Phase-A model.
//!
//! It hand-assembles a genuine ARC 700 poll loop into SRAM, runs the real
//! `Cpu::step`, and asserts that the overlay reports a `BUS_DEADLOCK` on a
//! layout whose loop line and cold line collide in the direct-mapped I-cache,
//! and does NOT report one on a non-colliding layout. This proves the overlay
//! reproduces the layout-dependent fetch-vs-load starvation over the real
//! instruction path.
//!
//! Requires the `timing` feature: `cargo test --features timing --test timing_iss`.
#![cfg(feature = "timing")]

use bcm55030_emulator::cpu::registers::PauseReason;
use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::decoder;
use bcm55030_emulator::soc::bank::BootMode;
use bcm55030_emulator::timing::TimingConfig;

/// `ldb.di r0, [r1]` — uncached byte load, base r1, dest r0, offset 0.
/// major=0x02, B=r1, di=1, aa=0(None), zz=1(byte), x=0, a=r0, s9=0.
const LDB_DI_R0_R1: u32 = 0x1100_0880;

/// Encode an unconditional `b target` (major 0x00, sub bit16=1, NoDelay).
/// The 25-bit signed byte offset is taken from `pcl = pc & !3`.
fn enc_b(pc: u32, target: u32) -> u32 {
    let off = target.wrapping_sub(pc & 0xFFFF_FFFC) as i32;
    assert_eq!(off & 1, 0, "branch offset must be even");
    let raw = ((off >> 1) as u32) & 0x00FF_FFFF; // 24-bit
    let s_low = raw & 0x3FF; // S[10:1]  -> bits[26:17]
    let s_high = (raw >> 10) & 0x3FF; // S[20:11] -> bits[15:6]
    let t = (raw >> 20) & 0xF; // S[24:21] -> bits[3:0]
    (1 << 16) | (s_low << 17) | (s_high << 6) | t
}

fn write_word_be(mem: &mut bcm55030_emulator::memory::Memory, addr: u32, w: u32) {
    mem.load_binary(
        addr,
        &[(w >> 24) as u8, (w >> 16) as u8, (w >> 8) as u8, w as u8],
    );
}

/// Build the poll loop at `base`, cold line at `cold`, and run it with the
/// timing overlay on. Returns `true` if a BUS_DEADLOCK was reported.
fn run_poll_loop(base: u32, cold: u32, max_steps: u64) -> bool {
    // Program:
    //   base+0 : ldb.di r0,[r1]     (r1 = MMIO status address, uncached load)
    //   base+4 : b cold
    //   cold+0 : b base
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    write_word_be(&mut cpu.mem, base, LDB_DI_R0_R1);
    write_word_be(&mut cpu.mem, base + 4, enc_b(base + 4, cold));
    write_word_be(&mut cpu.mem, cold, enc_b(cold, base));

    // Verify the hand-assembled bytes decode as intended.
    {
        use bcm55030_emulator::decoder::instruction::Instruction;
        let d0 = decoder::decode(base, &cpu.mem).expect("decode load");
        match d0.inst {
            Instruction::Load { cache_bypass, .. } => assert!(cache_bypass, "load must be .di"),
            other => panic!("base+0 not a load: {other:?}"),
        }
        let d1 = decoder::decode(base + 4, &cpu.mem).expect("decode b cold");
        match d1.inst {
            Instruction::Branch { offset, cc: None, link: false, .. } => {
                assert_eq!(((base + 4) & !3).wrapping_add(offset as u32), cold, "b cold target");
            }
            other => panic!("base+4 not an uncond branch: {other:?}"),
        }
        let d2 = decoder::decode(cold, &cpu.mem).expect("decode b base");
        match d2.inst {
            Instruction::Branch { offset, cc: None, link: false, .. } => {
                assert_eq!((cold & !3).wrapping_add(offset as u32), base, "b base target");
            }
            other => panic!("cold not an uncond branch: {other:?}"),
        }
    }

    // r1 = an MMIO status address (outside the 512 KB SRAM window → uncached).
    cpu.state.core_regs[1] = 0x0100_1040;
    cpu.state.pc = base;
    cpu.enable_timing(TimingConfig::default());

    for _ in 0..max_steps {
        cpu.step().expect("step");
        if cpu.state.halted {
            break;
        }
    }
    cpu.state.pause_reason == PauseReason::BusDeadlock
}

#[test]
fn overlay_reports_deadlock_on_colliding_layout() {
    // base line set = (0x1000>>5)&127 = 0; cold = base + 4 KB → same set 0.
    let base = 0x0000_1000;
    let cold = 0x0000_2000; // +4096 → same direct-mapped set
    assert!(
        run_poll_loop(base, cold, 200_000),
        "colliding poll loop must report BUS_DEADLOCK on the real ISS"
    );
}

#[test]
fn overlay_passes_on_non_colliding_layout() {
    // cold in a different set (one line over) → no eviction, no starvation.
    let base = 0x0000_1000;
    let cold = 0x0000_1080; // set 4, distinct from base's set 0
    assert!(
        !run_poll_loop(base, cold, 50_000),
        "non-colliding poll loop must NOT report a deadlock"
    );
}

#[test]
fn debug_ld_reads_one_at_the_hang() {
    // At a reported deadlock the load is left outstanding, so DEBUG.LD
    // (aux 0x05 bit 31) reads 1 — the silicon ground-truth signal.
    let base = 0x0000_1000;
    let cold = 0x0000_2000;
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    write_word_be(&mut cpu.mem, base, LDB_DI_R0_R1);
    write_word_be(&mut cpu.mem, base + 4, enc_b(base + 4, cold));
    write_word_be(&mut cpu.mem, cold, enc_b(cold, base));
    cpu.state.core_regs[1] = 0x0100_1040;
    cpu.state.pc = base;
    cpu.enable_timing(TimingConfig::default());
    for _ in 0..200_000 {
        cpu.step().expect("step");
        if cpu.state.halted {
            break;
        }
    }
    assert_eq!(cpu.state.pause_reason, PauseReason::BusDeadlock);
    let debug = cpu.state.read_aux_reg(0x05).unwrap();
    assert_ne!(debug & (1 << 31), 0, "DEBUG.LD must be 1 while the load is starved");
}
