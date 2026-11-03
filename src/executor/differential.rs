//! Differential tests: the generic semantics of `arc_isa::sem`, run on
//! `EmuDom`, against the frozen executor in `executor::legacy`. Every case
//! compares the result (exception variant and address included) and the
//! whole `CpuState`, field by field.

use std::collections::HashMap;

use arc_isa::condition::ConditionCode;
use arc_isa::decoder::instruction::{AluOp, Instruction, Operand};
use arc_isa::dom::{Dom, Flag};
use arc_isa::sem::{self, regs};

use crate::cpu::exception::Exception;

use super::emu_dom::EmuDom;
use super::legacy;
use crate::cpu::registers::CpuState;
use crate::memory::Memory;

pub(crate) struct Rng(u64);
impl Rng {
    pub(crate) fn new(seed: u64) -> Self {
        Self(seed | 1)
    }
    pub(crate) fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub(crate) fn bool(&mut self) -> bool {
        self.next() & 1 != 0
    }
    /// Half uniform, half from values that sit on edges.
    pub(crate) fn word(&mut self) -> u32 {
        const EDGES: [u32; 16] = [
            0, 1, 2, 31, 32, 33, 0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, 0xFFFF, 0x8000, 0xFF,
            0x80, 3, 7, 0x1F,
        ];
        let r = self.next();
        if r & 1 == 0 { (r >> 32) as u32 } else { EDGES[(r >> 8) as usize % EDGES.len()] }
    }
}

/// A state with random registers, flags (user mode included) and PC.
pub(crate) fn random_state(rng: &mut Rng) -> CpuState {
    let mut s = CpuState::new();
    for r in s.core_regs.iter_mut() {
        *r = rng.word();
    }
    s.flag_z = rng.bool();
    s.flag_n = rng.bool();
    s.flag_c = rng.bool();
    s.flag_v = rng.bool();
    s.flag_e1 = rng.bool();
    s.flag_e2 = rng.bool();
    s.flag_a1 = rng.bool();
    s.flag_a2 = rng.bool();
    s.flag_l = rng.bool();
    s.flag_u = rng.next().is_multiple_of(4);
    s.pc = (rng.next() as u32) & 0x0007_FFFE;
    s
}

/// Lines of the two states' `Debug` output that differ.
pub(crate) fn state_diff(a: &CpuState, b: &CpuState) -> Vec<String> {
    let fa = format!("{a:#?}");
    let fb = format!("{b:#?}");
    fa.lines()
        .zip(fb.lines())
        .filter(|(x, y)| x != y)
        .map(|(x, y)| format!("legacy {} | generic {}", x.trim(), y.trim()))
        .collect()
}

#[test]
fn conditions_exhaustive() {
    let mut s = CpuState::new();
    let mut m = Memory::new(0x100);
    for q in 0..16u8 {
        let cc = ConditionCode::from_u8(q).unwrap();
        for bits in 0..16u8 {
            let (z, n, c, v) = (bits & 8 != 0, bits & 4 != 0, bits & 2 != 0, bits & 1 != 0);
            (s.flag_z, s.flag_n, s.flag_c, s.flag_v) = (z, n, c, v);
            let want = legacy::cond::evaluate(cc, z, n, c, v);
            let got = regs::cond_holds(&mut EmuDom::new(&mut s, &mut m), cc);
            assert_eq!(got, want, "{cc:?} Z{z} N{n} C{c} V{v}");
        }
    }
}

#[test]
fn core_register_reads_and_writes() {
    let mut rng = Rng::new(0xC0DE_0001);
    let mut m = Memory::new(0x100);
    for _ in 0..200 {
        let base = random_state(&mut rng);
        for user in [false, true] {
            for r in 0..64u8 {
                let mut a = base.clone();
                let mut b = base.clone();
                a.flag_u = user;
                b.flag_u = user;
                assert_eq!(
                    regs::read_core_reg(&mut EmuDom::new(&mut b, &mut m), r),
                    legacy::regs::read_core_reg(&a, r),
                    "read r{r} U={user}"
                );
                let v = rng.word();
                let want = legacy::regs::write_core_reg(&mut a, r, v);
                let got = regs::write_core_reg(&mut EmuDom::new(&mut b, &mut m), r, v);
                assert_eq!(got, want, "write r{r} U={user}");
                let d = state_diff(&a, &b);
                assert!(d.is_empty(), "write r{r} U={user}: {d:?}");
            }
        }
    }
}

#[test]
fn status32_read_and_write() {
    let mut rng = Rng::new(0xC0DE_0002);
    let mut m = Memory::new(0x100);
    for _ in 0..20_000 {
        let base = random_state(&mut rng);
        let mut a = base.clone();
        let mut b = base.clone();
        assert_eq!(
            regs::status32(&mut EmuDom::new(&mut b, &mut m)),
            legacy::regs::status32(&a),
        );
        let v = rng.word();
        legacy::regs::set_status32(&mut a, v);
        regs::set_status32(&mut EmuDom::new(&mut b, &mut m), v);
        let d = state_diff(&a, &b);
        assert!(d.is_empty(), "set_status32({v:#x}): {d:?}");
        // Every flag the trait names maps onto the field the executor uses.
        let mut e = EmuDom::new(&mut b, &mut m);
        for f in Flag::ALL {
            let bit = e.bit(v, f.status32_bit() as u8);
            assert_eq!(e.flag(f), bit, "{f:?}");
        }
    }
}

/// Cases per differential module: `ARC_DIFF_CASES`, or 200 000.
fn cases() -> u64 {
    std::env::var("ARC_DIFF_CASES").ok().and_then(|s| s.parse().ok()).unwrap_or(200_000)
}

/// Registers that exercise special handling (ILINK, BLINK, LP_COUNT, r61-r63)
/// as often as ordinary ones.
fn pick_reg(rng: &mut Rng) -> u8 {
    const SPECIAL: [u8; 10] = [26, 27, 28, 29, 30, 31, 60, 61, 62, 63];
    if rng.bool() { SPECIAL[rng.next() as usize % SPECIAL.len()] } else { (rng.next() % 26) as u8 }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Form {
    R,
    I,
    N,
}

fn pick_operand(rng: &mut Rng, allow_none: bool) -> (Operand, Form) {
    match rng.next() % if allow_none { 3 } else { 2 } {
        0 => (Operand::Reg(pick_reg(rng)), Form::R),
        1 => (Operand::Imm(rng.word()), Form::I),
        _ => (Operand::None, Form::N),
    }
}

fn pick_cc(rng: &mut Rng) -> Option<ConditionCode> {
    let q = rng.next() % 17;
    if q == 16 { None } else { ConditionCode::from_u8(q as u8) }
}

fn assert_same(what: &dyn std::fmt::Debug, ra: &Result<(), Exception>, rb: &Result<(), Exception>,
               a: &CpuState, b: &CpuState) {
    assert_eq!(ra, rb, "{what:?}: result");
    if a != b {
        panic!("{what:?}: state differs: {:?}", state_diff(a, b));
    }
}

#[test]
fn alu_matches_legacy() {
    let ops: Vec<AluOp> = (0..64u8).filter_map(AluOp::from_u8).collect();
    let mut rng = Rng::new(0xA1_0001);
    let mut m = Memory::new(0x100);
    let mut cover: HashMap<(u8, bool, bool, Form, Form, Form), u64> = HashMap::new();
    for _ in 0..cases() {
        let op = ops[rng.next() as usize % ops.len()];
        let f = rng.bool();
        let cc = pick_cc(&mut rng);
        let (dst, fd) = pick_operand(&mut rng, true);
        let (src1, f1) = pick_operand(&mut rng, true);
        let (src2, f2) = pick_operand(&mut rng, true);
        let base = random_state(&mut rng);
        let (mut a, mut b) = (base.clone(), base);
        let ra = legacy::alu::execute_alu(op, dst, src1, src2, f, cc, &mut a);
        let rb = sem::alu::alu(&mut EmuDom::new(&mut b, &mut m), op, dst, src1, src2, f, cc);
        assert_same(&(op, dst, src1, src2, f, cc), &ra, &rb, &a, &b);
        *cover.entry((op as u8, f, cc.is_some(), fd, f1, f2)).or_default() += 1;
    }
    // Every op x .f x cc-present x operand forms must have been exercised.
    let cells = ops.len() * 2 * 2 * 27;
    assert_eq!(cover.len(), cells, "ALU coverage: {} of {cells} cells", cover.len());
}

/// Random encodings through the real decoder, executed by the frozen executor
/// and by the live one (which delegates to the generic semantics step by step).
#[test]
fn execute_random_encodings_matches_legacy() {
    let mut rng = Rng::new(0xE0_0001);
    let base_pc = 0x1000u32;
    let (mut decoded, mut rejected) = (0u64, 0u64);
    for _ in 0..cases() {
        let w = rng.next();
        let bytes = w.to_be_bytes();
        let Ok(dec) = crate::decoder::decode_bytes(base_pc, &bytes, base_pc) else {
            rejected += 1;
            continue;
        };
        // Memory-touching and AUX instructions get their own differential.
        if matches!(dec.inst, Instruction::Load { .. } | Instruction::Store { .. }
            | Instruction::LoadAux { .. } | Instruction::StoreAux { .. }) {
            continue;
        }
        decoded += 1;
        let mut base = random_state(&mut rng);
        base.pc = base_pc;
        let (mut a, mut b) = (base.clone(), base);
        let (mut ma, mut mb) = (Memory::new(0x100), Memory::new(0x100));
        let ra = legacy::execute(&dec, &mut a, &mut ma);
        let rb = super::execute(&dec, &mut b, &mut mb);
        assert_same(&dec.inst, &ra, &rb, &a, &b);
    }
    assert!(decoded > cases() / 4, "decoded {decoded}, rejected {rejected}");
}
