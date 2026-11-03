//! Differential tests: the generic semantics of `arc_isa::sem`, run on
//! `EmuDom`, against the frozen executor in `executor::legacy`. Every case
//! compares the result (exception variant and address included) and the
//! whole `CpuState`, field by field.

use std::collections::HashMap;

use arc_isa::condition::ConditionCode;
use arc_isa::decoder::instruction::{
    AluOp, BrCompareKind, DecodedInstruction, DelayMode, ExtArithOp, Instruction, Operand, SingleOp,
    ZeroOp,
};
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
    /// Uniform in `0..n`, from the high bits: the low bits of consecutive
    /// outputs are correlated, and selecting on them leaves combinations
    /// unreachable.
    pub(crate) fn below(&mut self, n: u64) -> u64 {
        ((self.next() >> 32) * n) >> 32
    }
    pub(crate) fn bool(&mut self) -> bool {
        self.next() >> 63 != 0
    }
    /// Half uniform, half from values that sit on edges.
    pub(crate) fn word(&mut self) -> u32 {
        const EDGES: [u32; 16] = [
            0, 1, 2, 31, 32, 33, 0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, 0xFFFF, 0x8000, 0xFF,
            0x80, 3, 7, 0x1F,
        ];
        let r = self.next();
        if r >> 63 == 0 { (r >> 16) as u32 } else { EDGES[((r >> 32) % EDGES.len() as u64) as usize] }
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
    s.flag_u = rng.below(4) == 0;
    s.pc = (rng.next() as u32) & 0x0007_FFFE;
    s.aux_status32_l1 = rng.word();
    s.aux_status32_l2 = rng.word();
    s.aux_bta = rng.word();
    s.aux_bta_l1 = rng.word();
    s.aux_bta_l2 = rng.word();
    s.aux_lp_start = rng.word() & !1;
    s.aux_lp_end = rng.word() & !1;
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
    if rng.bool() { SPECIAL[rng.below(SPECIAL.len() as u64) as usize] } else { rng.below(26) as u8 }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Form {
    R,
    I,
    N,
}

fn pick_operand(rng: &mut Rng, allow_none: bool) -> (Operand, Form) {
    match rng.below(if allow_none { 3 } else { 2 }) {
        0 => (Operand::Reg(pick_reg(rng)), Form::R),
        1 => (Operand::Imm(rng.word()), Form::I),
        _ => (Operand::None, Form::N),
    }
}

fn pick_cc(rng: &mut Rng) -> Option<ConditionCode> {
    let q = rng.below(17);
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
        let op = ops[rng.below(ops.len() as u64) as usize];
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

const SINGLE_OPS: [SingleOp; 13] = [
    SingleOp::Asl, SingleOp::Asr, SingleOp::Lsr, SingleOp::Ror, SingleOp::Rrc, SingleOp::Sexb,
    SingleOp::Sexw, SingleOp::Extb, SingleOp::Extw, SingleOp::Abs, SingleOp::Not, SingleOp::Rlc,
    SingleOp::Ex,
];

const EXT_OPS: [ExtArithOp; 20] = [
    ExtArithOp::Asl, ExtArithOp::Lsr, ExtArithOp::Asr, ExtArithOp::Ror, ExtArithOp::Adds,
    ExtArithOp::Subs, ExtArithOp::Divaw, ExtArithOp::Asls, ExtArithOp::Asrs, ExtArithOp::Addsdw,
    ExtArithOp::Subsdw, ExtArithOp::Swap, ExtArithOp::Norm, ExtArithOp::Sat16, ExtArithOp::Rnd16,
    ExtArithOp::Abssw, ExtArithOp::Abss, ExtArithOp::Negsw, ExtArithOp::Negs, ExtArithOp::Normw,
];

fn check_single(op: SingleOp, dst: Operand, src: Operand, f: bool, cc: Option<ConditionCode>,
                base: CpuState, m: &mut Memory) {
    let (mut a, mut b) = (base.clone(), base);
    let ra = legacy::single_op::execute_single_op(op, dst, src, f, cc, &mut a);
    let rb = sem::single_op::single_op(&mut EmuDom::new(&mut b, m), op, dst, src, f, cc);
    assert_same(&(op, dst, src, f, cc), &ra, &rb, &a, &b);
}

#[allow(clippy::too_many_arguments)]
fn check_ext(op: ExtArithOp, dst: Operand, src1: Operand, src2: Operand, f: bool,
             cc: Option<ConditionCode>, base: CpuState, m: &mut Memory) {
    let (mut a, mut b) = (base.clone(), base);
    let ra = legacy::extended::execute_ext_arith(op, dst, src1, src2, f, cc, &mut a);
    let rb = sem::extended::ext_arith(&mut EmuDom::new(&mut b, m), op, dst, src1, src2, f, cc);
    assert_same(&(op, dst, src1, src2, f, cc), &ra, &rb, &a, &b);
}

#[test]
fn single_op_matches_legacy() {
    let mut rng = Rng::new(0x51_0001);
    let mut m = Memory::new(0x100);
    let mut cover: HashMap<(usize, bool, bool, Form, Form), u64> = HashMap::new();
    for _ in 0..cases() {
        let i = rng.below(SINGLE_OPS.len() as u64) as usize;
        let f = rng.bool();
        let cc = pick_cc(&mut rng);
        let (dst, fd) = pick_operand(&mut rng, true);
        let (src, fs) = pick_operand(&mut rng, true);
        let base = random_state(&mut rng);
        check_single(SINGLE_OPS[i], dst, src, f, cc, base, &mut m);
        *cover.entry((i, f, cc.is_some(), fd, fs)).or_default() += 1;
    }
    let cells = SINGLE_OPS.len() * 2 * 2 * 9;
    assert_eq!(cover.len(), cells, "single-op coverage: {} of {cells}", cover.len());
    // Exhaustive on the extension operations' input width.
    let mut s = CpuState::new();
    for op in [SingleOp::Sexb, SingleOp::Sexw, SingleOp::Extb, SingleOp::Extw] {
        for v in 0..=0xFFFFu32 {
            for hi in [0, 0xABCD_0000] {
                s.core_regs[0] = v | hi;
                check_single(op, Operand::Reg(1), Operand::Reg(0), true, None, s.clone(), &mut m);
            }
        }
    }
}

#[test]
fn ext_arith_matches_legacy() {
    let mut rng = Rng::new(0xE7_0001);
    let mut m = Memory::new(0x100);
    let mut cover: HashMap<(usize, bool, bool, Form, Form, Form), u64> = HashMap::new();
    for _ in 0..cases() {
        let i = rng.below(EXT_OPS.len() as u64) as usize;
        let f = rng.bool();
        let cc = pick_cc(&mut rng);
        let (dst, fd) = pick_operand(&mut rng, true);
        let (src1, f1) = pick_operand(&mut rng, true);
        let (src2, f2) = pick_operand(&mut rng, true);
        let base = random_state(&mut rng);
        check_ext(EXT_OPS[i], dst, src1, src2, f, cc, base, &mut m);
        *cover.entry((i, f, cc.is_some(), fd, f1, f2)).or_default() += 1;
    }
    let cells = EXT_OPS.len() * 2 * 2 * 27;
    assert_eq!(cover.len(), cells, "ext-arith coverage: {} of {cells}", cover.len());
    // Every shift count 0..=40 on every barrel operation, carry in either way.
    let mut s = CpuState::new();
    for op in [ExtArithOp::Asl, ExtArithOp::Lsr, ExtArithOp::Asr, ExtArithOp::Ror] {
        for count in 0..=40u32 {
            for v in [0, 1, 0x8000_0000, 0xFFFF_FFFF, 0x1234_5678, 0x8765_4321] {
                for c in [false, true] {
                    s.core_regs[0] = v;
                    s.core_regs[1] = count;
                    s.flag_c = c;
                    check_ext(op, Operand::Reg(2), Operand::Reg(0), Operand::Reg(1), true, None,
                              s.clone(), &mut m);
                }
            }
        }
    }
    // NORM/NORMW on the values that sit on their edges.
    let mut edges = vec![0u32, 1, 0xFFFF_FFFF, 0xFFFF_FFFE, 0x7FFF_FFFF, 0x8000_0000];
    for i in 0..32 {
        edges.push(1 << i);
        edges.push(!(1u32 << i));
        edges.push((1u32 << i).wrapping_neg());
    }
    for op in [ExtArithOp::Norm, ExtArithOp::Normw] {
        for &v in &edges {
            s.core_regs[0] = v;
            check_ext(op, Operand::Reg(2), Operand::Reg(0), Operand::None, true, None, s.clone(), &mut m);
        }
    }
}

fn pick_delay(rng: &mut Rng) -> DelayMode {
    if rng.bool() { DelayMode::Delay } else { DelayMode::NoDelay }
}

/// A decoded instruction at the state's PC, as the legacy control executors
/// read their PC and length from it.
fn at_pc(inst: Instruction, s: &CpuState, rng: &mut Rng) -> DecodedInstruction {
    let size = if rng.bool() { 2 } else { 4 };
    DecodedInstruction { inst, size, has_limm: size == 4 && rng.bool(), pc: s.pc }
}

#[test]
fn control_matches_legacy() {
    const KINDS: [BrCompareKind; 8] = [
        BrCompareKind::Breq, BrCompareKind::Brne, BrCompareKind::Brlt, BrCompareKind::Brge,
        BrCompareKind::Brlo, BrCompareKind::Brhs, BrCompareKind::Bbit0, BrCompareKind::Bbit1,
    ];
    let mut rng = Rng::new(0xB2_0001);
    let mut m = Memory::new(0x100);
    let mut cover: HashMap<(u8, bool, bool, bool), u64> = HashMap::new();
    for _ in 0..cases() {
        let base = random_state(&mut rng);
        let (mut a, mut b) = (base.clone(), base);
        let cc = pick_cc(&mut rng);
        let delay = pick_delay(&mut rng);
        let link = rng.bool();
        let offset = (rng.next() as i32) >> 10 & !1;
        let which = rng.below(6) as u8;
        let (ra, rb) = match which {
            0 => {
                let dec = at_pc(Instruction::Branch { offset, cc, delay, link }, &a, &mut rng);
                let ra = legacy::branch::execute_branch(&dec, offset, cc, delay, link, &mut a);
                let rb = sem::branch::branch(&mut EmuDom::new(&mut b, &mut m), offset, cc, delay, link,
                                             dec.total_size());
                (ra, rb)
            }
            1 => {
                let kind = KINDS[rng.below(KINDS.len() as u64) as usize];
                let (src1, _) = pick_operand(&mut rng, false);
                let (src2, _) = pick_operand(&mut rng, false);
                let dec = at_pc(Instruction::BranchCompare { kind, src1, src2, offset, delay }, &a, &mut rng);
                let ra = legacy::branch::execute_branch_compare(kind, src1, src2, offset, delay, &dec, &mut a);
                let rb = sem::branch::branch_compare(&mut EmuDom::new(&mut b, &mut m), kind, src1, src2,
                                                     offset, delay);
                (ra, rb)
            }
            2 => {
                let (target, _) = pick_operand(&mut rng, false);
                let flag_restore = rng.bool();
                let dec = at_pc(Instruction::Jump { target, cc, delay, link, flag_restore }, &a, &mut rng);
                let ra = legacy::branch::execute_jump(&dec, target, cc, delay, link, flag_restore, &mut a);
                let rb = sem::branch::jump(&mut EmuDom::new(&mut b, &mut m), target, cc, delay, link,
                                           flag_restore, dec.total_size());
                (ra, rb)
            }
            3 => {
                let zop = match rng.below(7) {
                    0 => ZeroOp::Nop,
                    1 => ZeroOp::Sleep { u6: rng.below(64) as u8 },
                    2 => ZeroOp::Swi,
                    3 => ZeroOp::Brk,
                    4 => ZeroOp::Trap { param: rng.below(64) as u8 },
                    5 => ZeroOp::Rtie,
                    _ => ZeroOp::Sync,
                };
                let ra = legacy::special::execute_zero_op(&zop, &mut a);
                let rb = sem::special::zero_op(&mut EmuDom::new(&mut b, &mut m), &zop);
                (ra, rb)
            }
            4 => {
                let off = (rng.next() as u32) & 0x0FFE;
                let dec = at_pc(Instruction::Loop { offset: off, cc }, &a, &mut rng);
                let ra = legacy::special::execute_loop(off, cc, &dec, &mut a);
                let rb = sem::special::lp(&mut EmuDom::new(&mut b, &mut m), off, cc, dec.total_size());
                (ra, rb)
            }
            _ => {
                let (src, _) = pick_operand(&mut rng, false);
                let src = if rng.below(4) == 0 { Operand::Imm(rng.word() | 1) } else { src };
                let ra = legacy::special::execute_flag(src, cc, &mut a);
                let rb = sem::special::flag(&mut EmuDom::new(&mut b, &mut m), src, cc);
                (ra, rb)
            }
        };
        assert_same(&(which, cc, delay, link, offset), &ra, &rb, &a, &b);
        *cover.entry((which, cc.is_some(), delay == DelayMode::Delay, link)).or_default() += 1;
    }
    assert_eq!(cover.len(), 6 * 8, "control coverage: {} of 48", cover.len());
}
