//! `arc_isa::flat::FlatMachine` and the emulator agree on plain memory.
//!
//! The same short random programs run on the self-contained domain and on a
//! flat-memory `Cpu` (no caches, no MMIO, interrupts off), step by step. The
//! architectural state must match after every step; only the timer counters,
//! which exist in the emulator alone, are left out. A data access outside the
//! memory ends the case: the emulator's flat test memory reads 0 there but
//! faults on a write, which is not a property of the core.

use arc_isa::dom::{AuxReg, Dom, Flag};
use arc_isa::flat::FlatMachine;
use bcm55030_emulator::cpu::registers::DelayState;
use bcm55030_emulator::cpu::Cpu;

const MEM: usize = 0x10000;
const CODE: u32 = 0x1000;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u64) -> u64 {
        ((self.next() >> 32) * n) >> 32
    }
}

fn alu(sub: u32, b: u32, c: u32, a: u32, p: u32) -> u32 {
    (0x04 << 27) | ((b & 7) << 24) | ((p & 3) << 22) | ((sub & 0x3F) << 16) | (((b >> 3) & 7) << 12)
        | ((c & 0x3F) << 6) | (a & 0x3F)
}

fn word(rng: &mut Rng) -> u32 {
    match rng.below(6) {
        0 => {
            let d = ((rng.below(32) as i32 - 16) * 2) as u32;
            ((d >> 1) & 0x3FF) << 17 | ((d >> 11) & 0x3FF) << 6 | (rng.below(2) as u32) << 5
        }
        1 => alu(0x28, 0, rng.below(12) as u32, 0, 1),
        2 => alu(0x0A, 60, 1 + rng.below(4) as u32, 0, 1),
        _ => rng.next() as u32,
    }
}

fn compare(step: usize, f: &mut FlatMachine, c: &Cpu) {
    let s = &c.state;
    assert_eq!(f.regs, s.core_regs, "step {step}: registers");
    assert_eq!(f.pc, s.pc, "step {step}: pc");
    for (flag, v) in [(Flag::Z, s.flag_z), (Flag::N, s.flag_n), (Flag::C, s.flag_c), (Flag::V, s.flag_v),
                      (Flag::E1, s.flag_e1), (Flag::E2, s.flag_e2), (Flag::A1, s.flag_a1),
                      (Flag::A2, s.flag_a2), (Flag::AE, s.flag_ae), (Flag::DE, s.flag_de),
                      (Flag::U, s.flag_u), (Flag::L, s.flag_l), (Flag::H, s.flag_h)] {
        assert_eq!(f.flag(flag), v, "step {step}: {flag:?}");
    }
    for (a, v) in [(AuxReg::LpStart, s.aux_lp_start), (AuxReg::LpEnd, s.aux_lp_end),
                   (AuxReg::Status32L1, s.aux_status32_l1), (AuxReg::Status32L2, s.aux_status32_l2),
                   (AuxReg::IrqLv12, s.aux_irq_lv12), (AuxReg::Bta, s.aux_bta),
                   (AuxReg::IntVectorBase, s.aux_int_vector_base)] {
        assert_eq!(f.aux(a), v, "step {step}: {a:?}");
    }
    let delay = match s.delay_state {
        DelayState::DelaySlot { target, is_link } => Some((target, is_link)),
        DelayState::None => None,
    };
    assert_eq!(f.delay, delay, "step {step}: delay slot");
    assert_eq!((f.halted, f.sleeping), (s.halted, s.sleeping), "step {step}: halted/sleeping");
}

#[test]
fn flat_machine_matches_the_emulator_on_plain_memory() {
    let mut rng = Rng(0xF1A7_0001);
    let (mut steps_run, mut out_of_memory) = (0u64, 0u64);
    for _ in 0..20_000 {
        let code: Vec<u8> = (0..1 + rng.below(8)).flat_map(|_| word(&mut rng).to_be_bytes()).collect();
        let mut f = FlatMachine::new(MEM);
        let mut c = Cpu::new(MEM);
        f.mem[CODE as usize..CODE as usize + code.len()].copy_from_slice(&code);
        c.mem.load_binary(CODE, &code);
        let pc = CODE + 2 * rng.below(8) as u32;
        let start = CODE + 2 * rng.below(16) as u32;
        let end = CODE + 2 * rng.below(16) as u32;
        let count = rng.below(4) as u32;
        let mut regs = [0u32; 64];
        for r in regs.iter_mut() {
            *r = if rng.below(4) != 0 { CODE + 4 * rng.below(256) as u32 } else { rng.next() as u32 };
        }
        regs[60] = count;
        f.regs = regs;
        c.state.core_regs = regs;
        f.pc = pc;
        c.state.pc = pc;
        f.set_aux(AuxReg::LpStart, start);
        f.set_aux(AuxReg::LpEnd, end);
        f.set_aux(AuxReg::IntVectorBase, 0x2000);
        c.state.aux_lp_start = start;
        c.state.aux_lp_end = end;
        c.state.aux_int_vector_base = 0x2000;
        for step in 0..1 + rng.below(16) as usize {
            let rf = f.step();
            let rc = c.step();
            let outside = |r: &Result<(), arc_isa::exception::Exception>| matches!(r,
                Err(arc_isa::exception::Exception::MemoryError { address, .. }) if *address as usize >= MEM);
            if outside(&rf) || outside(&rc) {
                out_of_memory += 1;
                break;
            }
            assert_eq!(rf, rc, "step {step}: result");
            compare(step, &mut f, &c);
            steps_run += 1;
            if rf.is_err() || f.halted || f.sleeping {
                break;
            }
        }
        if f.mem[..] != c.mem.sram_snapshot()[..MEM] && out_of_memory == 0 {
            panic!("memory differs");
        }
    }
    assert!(steps_run > 50_000, "only {steps_run} steps compared");
    assert!(out_of_memory < 3_000, "{out_of_memory} of 20000 cases left the memory");
}
