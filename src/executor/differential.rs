//! Differential tests: the generic semantics of `arc_isa::sem`, run on
//! `EmuDom`, against the frozen executor in `executor::legacy`. Every case
//! compares the result (exception variant and address included) and the
//! whole `CpuState`, field by field.

use std::collections::HashMap;

use arc_isa::condition::ConditionCode;
use arc_isa::decoder::instruction::{
    AluOp, BrCompareKind, DataSize, DecodedInstruction, DelayMode, ExtArithOp, Instruction, Operand,
    SingleOp, WritebackMode, ZeroOp,
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

/// Loads, stores, `lr` and `sr`, on two SoC memories that evolve in lockstep:
/// same initial SRAM, same instruction stream, one run by the frozen executor
/// and one by the generic semantics. Besides the result and the `CpuState`,
/// each case compares the misaligned-access log and the MMIO history; the
/// D-cache and SRAM are compared every few cases and at the end (a divergence
/// persists, so the window names the culprit to within a few instructions).
#[test]
fn memory_and_aux_match_legacy() {
    use crate::memory::SRAM_SIZE;
    use crate::soc::bank::BootMode;
    use std::collections::VecDeque;

    const DATA: u32 = 0x0001_0000;
    const DATA_LEN: u32 = 0x0006_0000;
    // Unclaimed MMIO: reads give 0, writes are dropped, both are logged.
    const MMIO: u32 = 0x0300_0000;
    const AUX_CHOICES: [u32; 24] = [
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0A, 0x0B, 0x0C, 0x10, 0x11, 0x19, 0x21, 0x22,
        0x25, 0x43, 0x47, 0x48, 0x4A, 0x58, 0x59, 0x5B, 0x101,
    ];

    let mut rng = Rng::new(0x3E_0001);
    let mut ma = Memory::new_soc(SRAM_SIZE, BootMode::Cold);
    let mut mb = Memory::new_soc(SRAM_SIZE, BootMode::Cold);
    let mut sram = vec![0u8; SRAM_SIZE];
    for b in sram.iter_mut() {
        *b = rng.next() as u8;
    }
    ma.load_binary(0, &sram);
    mb.load_binary(0, &sram);

    let sizes = [DataSize::Word, DataSize::HalfWord, DataSize::Byte];
    let wbs = [WritebackMode::None, WritebackMode::PreWrite, WritebackMode::PostWrite, WritebackMode::Scaled];
    let mut recent: VecDeque<String> = VecDeque::new();
    let mut cover: HashMap<(u8, u8, u8, bool), u64> = HashMap::new();
    let n = cases();
    for i in 0..n {
        let mut base = random_state(&mut rng);
        base.flag_u = rng.below(8) == 0;
        // Base register r1 points at SRAM data (any alignment), the vector
        // area (the NCO aperture for `.di` stores) or unclaimed MMIO.
        let region = rng.below(3) as u8;
        base.core_regs[1] = match region {
            0 => DATA + rng.below(DATA_LEN as u64) as u32,
            1 => rng.below(0x80) as u32,
            _ => MMIO + rng.below(0x100) as u32,
        };
        base.core_regs[2] = rng.below(64) as u32;
        let kind = rng.below(4) as u8;
        let size = sizes[rng.below(3) as usize];
        let wb = wbs[rng.below(4) as usize];
        let di = rng.bool();
        let sx = rng.bool();
        let off = if rng.bool() { Operand::Reg(2) } else { Operand::Imm(rng.below(64) as u32) };
        let reg = |rng: &mut Rng| if rng.below(4) == 0 { pick_reg(rng) } else { 3 + rng.below(20) as u8 };
        let inst = match kind {
            0 => Instruction::Load {
                dst: if rng.below(8) == 0 { Operand::None } else { Operand::Reg(reg(&mut rng)) },
                base: Operand::Reg(1), offset: off, data_size: size, sign_extend: sx,
                writeback: wb, cache_bypass: di,
            },
            1 => Instruction::Store {
                src: Operand::Reg(reg(&mut rng)), base: Operand::Reg(1), offset: off,
                data_size: size, writeback: wb, cache_bypass: di,
            },
            _ => {
                let a = if rng.below(4) == 0 { rng.word() & 0x7FF } else {
                    AUX_CHOICES[rng.below(AUX_CHOICES.len() as u64) as usize]
                };
                let src_or_dst = Operand::Reg(reg(&mut rng));
                let addr = if rng.bool() { Operand::Imm(a) } else {
                    base.core_regs[4] = a;
                    Operand::Reg(4)
                };
                if kind == 2 {
                    Instruction::LoadAux { dst: src_or_dst, addr }
                } else {
                    Instruction::StoreAux { src: src_or_dst, addr }
                }
            }
        };
        let dec = DecodedInstruction { inst: inst.clone(), size: 4, has_limm: false, pc: base.pc };
        let (mut a, mut b) = (base.clone(), base);
        let ra = legacy::execute(&dec, &mut a, &mut ma);
        let rb = super::execute(&dec, &mut b, &mut mb);
        recent.push_back(format!("#{i} {inst:?}"));
        if recent.len() > 16 {
            recent.pop_front();
        }
        assert_eq!(ra, rb, "{inst:?}: result");
        if a != b {
            panic!("{inst:?}: state differs: {:?}", state_diff(&a, &b));
        }
        assert_eq!(ma.misaligned_data.count, mb.misaligned_data.count, "{inst:?}: misaligned count");
        assert_eq!(ma.misaligned_data.hits.last(), mb.misaligned_data.hits.last(), "{inst:?}: misaligned log");
        {
            let (ba, bb) = (ma.bank().unwrap().read(), mb.bank().unwrap().read());
            assert_eq!(ba.mmio_history.len(), bb.mmio_history.len(), "{inst:?}: MMIO history length");
            assert_eq!(format!("{:?}", ba.mmio_history.back()), format!("{:?}", bb.mmio_history.back()),
                       "{inst:?}: MMIO history");
        }
        if i % 64 == 63 || i + 1 == n {
            assert_eq!(format!("{:?}", ma.dcache_snapshot()), format!("{:?}", mb.dcache_snapshot()),
                       "D-cache differs; last instructions: {recent:?}");
        }
        if i % 8192 == 8191 || i + 1 == n {
            assert!(ma.sram_snapshot() == mb.sram_snapshot(), "SRAM differs; last instructions: {recent:?}");
        }
        *cover.entry((kind, region, size as u8, di)).or_default() += 1;
    }
    assert_eq!(cover.len(), 4 * 3 * 3 * 2, "memory/AUX coverage: {} of 72", cover.len());
}

/// The register-level AUX API of `CpuState` (now the generic semantics over
/// `StateDom`) against the frozen one, on every address below 0x800.
#[test]
fn cpu_state_aux_api_matches_legacy() {
    let mut rng = Rng::new(0xA0_0001);
    for _ in 0..50 {
        let base = random_state(&mut rng);
        for addr in 0..0x800u32 {
            let (mut a, mut b) = (base.clone(), base.clone());
            assert_eq!(b.read_aux_reg(addr), legacy::regs::read_aux_reg(&a, addr), "read {addr:#x}");
            let v = rng.word();
            let ra = legacy::regs::write_aux_reg(&mut a, addr, v);
            let rb = b.write_aux_reg(addr, v);
            assert_same(&("write", addr, v), &ra, &rb, &a, &b);
        }
    }
}
