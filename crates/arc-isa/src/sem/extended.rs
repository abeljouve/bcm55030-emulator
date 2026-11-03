//! Extension operations: barrel shifts, NORM/NORMW, SWAP (major 0x05), and
//! the saturating/DSP group, which this core does not implement.

use crate::condition::ConditionCode;
use crate::decoder::instruction::{ExtArithOp, Operand};
use crate::dom::{Dom, Flag};
use crate::exception::Exception;

use super::regs::{cond_holds, operand, write_dest};

#[inline]
pub fn ext_arith<D: Dom>(
    d: &mut D,
    op: ExtArithOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
) -> Result<(), Exception> {
    // The saturating/DSP group is not implemented on this core: it faults
    // whatever its condition. INFERRED for the false-condition case: only
    // unconditional forms were measured.
    if op.is_absent_on_core() {
        return Err(Exception::InstructionError { address: d.pc() });
    }
    // One copy of the body, guarded: no condition is "always".
    let g = match cc {
        None => d.kb(true),
        Some(cc) => cond_holds(d, cc),
    };
    d.when(g, move |d| body(d, op, dst, src1, src2, set_flags))
}

#[inline(always)]
fn body<D: Dom>(
    d: &mut D,
    op: ExtArithOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
) -> Result<(), Exception> {
    let a = operand(d, src1)?;
    let b = operand(d, src2)?;
    match op {
        ExtArithOp::Asl | ExtArithOp::Lsr | ExtArithOp::Asr | ExtArithOp::Ror => {
            let (r, c) = barrel_shift(d, op, a, b);
            write_dest(d, dst, r)?;
            if set_flags {
                set_zn(d, r);
                d.set_flag(Flag::C, c);
            }
        }
        ExtArithOp::Norm | ExtArithOp::Normw => {
            // The destination gets the normalisation count, but Z and N
            // describe the SOURCE (NORMW: its low half). C and V are kept.
            // Observed on silicon.
            let (r, z, n) = if op == ExtArithOp::Norm {
                // Leading sign bits minus one: clz of the value with its sign
                // folded away, minus 1 (0 and all-ones give 31).
                let n = d.bit(a, 31);
                let na = d.not(a);
                let x = d.ite(n, na, a);
                let lz = d.clz(x);
                let one = d.k(1);
                let r = d.sub(lz, one);
                let zero = d.k(0);
                let z = d.eq(a, zero);
                (r, z, n)
            } else {
                // Same on the low half: clz counts 16 extra zeros, hence 17.
                let n = d.bit(a, 15);
                let na = d.not(a);
                let x = d.ite(n, na, a);
                let x16 = d.zext(x, 16);
                let lz = d.clz(x16);
                let k17 = d.k(17);
                let r = d.sub(lz, k17);
                let a16 = d.zext(a, 16);
                let zero = d.k(0);
                let z = d.eq(a16, zero);
                (r, z, n)
            };
            write_dest(d, dst, r)?;
            if set_flags {
                d.set_flag(Flag::Z, z);
                d.set_flag(Flag::N, n);
            }
        }
        ExtArithOp::Swap => {
            let k16 = d.k(16);
            let hi = d.shl(a, k16);
            let lo = d.lshr(a, k16);
            let r = d.or(hi, lo);
            write_dest(d, dst, r)?;
            if set_flags {
                set_zn(d, r);
            }
        }
        _ => unreachable!("{op:?} faults before it computes"),
    }
    Ok(())
}

#[inline(always)]
fn set_zn<D: Dom>(d: &mut D, r: D::V) {
    let zero = d.k(0);
    let z = d.eq(r, zero);
    let n = d.bit(r, 31);
    d.set_flag(Flag::Z, z);
    d.set_flag(Flag::N, n);
}

/// Multi-bit shift through the barrel shifter. The count is taken modulo 32
/// (a count of 32 shifts by 0, 33 by 1). C is the last bit shifted out, and is
/// cleared when the masked count is 0. V is never written. Observed on silicon
/// for register counts 0..=63 on all four operations; the immediate (u6)
/// forms go through the same operation and are assumed to match (INFERRED).
///
/// Every carry below is bit 0 of a shift by an out-of-range amount when the
/// masked count is 0 (32, or 0xFFFFFFFF), which `bvlshr` turns into 0: the
/// zero-count carry falls out without a branch.
#[inline(always)]
fn barrel_shift<D: Dom>(d: &mut D, op: ExtArithOp, a: D::V, count: D::V) -> (D::V, D::B) {
    let m31 = d.k(31);
    let s = d.and(count, m31);
    let k32 = d.k(32);
    let one = d.k(1);
    let s_minus_1 = d.sub(s, one);
    let last_right = d.lshr(a, s_minus_1);
    match op {
        ExtArithOp::Asl => {
            let r = d.shl(a, s);
            let back = d.sub(k32, s);
            let out = d.lshr(a, back);
            let c = d.bit(out, 0);
            (r, c)
        }
        ExtArithOp::Lsr => {
            let r = d.lshr(a, s);
            (r, d.bit(last_right, 0))
        }
        ExtArithOp::Asr => {
            let r = d.ashr(a, s);
            (r, d.bit(last_right, 0))
        }
        ExtArithOp::Ror => {
            let lo = d.lshr(a, s);
            let back = d.sub(k32, s);
            let hi = d.shl(a, back);
            let r = d.or(lo, hi);
            (r, d.bit(last_right, 0))
        }
        _ => unreachable!(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flat::FlatMachine;

    fn run(op: ExtArithOp, a: u32, b: u32, flags_in: (bool, bool, bool, bool)) -> (u32, [bool; 4]) {
        let mut m = FlatMachine::new(0);
        m.regs[0] = a;
        m.regs[1] = b;
        m.set_flag(Flag::Z, flags_in.0);
        m.set_flag(Flag::N, flags_in.1);
        m.set_flag(Flag::C, flags_in.2);
        m.set_flag(Flag::V, flags_in.3);
        ext_arith(&mut m, op, Operand::Reg(2), Operand::Reg(0), Operand::Reg(1), true, None).unwrap();
        (m.regs[2], [m.flag(Flag::Z), m.flag(Flag::N), m.flag(Flag::C), m.flag(Flag::V)])
    }

    #[test]
    fn shift_count_is_taken_modulo_32() {
        for count in [0u32, 1, 31, 32, 33, 63] {
            let s = count & 31;
            let f = (false, false, false, false);
            assert_eq!(run(ExtArithOp::Asl, 5, count, f).0, 5u32 << s);
            assert_eq!(run(ExtArithOp::Lsr, 0xA000_0000, count, f).0, 0xA000_0000u32 >> s);
            assert_eq!(run(ExtArithOp::Asr, 0x8000_0000, count, f).0, ((0x8000_0000u32 as i32) >> s) as u32);
            assert_eq!(run(ExtArithOp::Ror, 0x1234_5678, count, f).0, 0x1234_5678u32.rotate_right(s));
        }
    }

    #[test]
    fn shift_carry_is_last_bit_out_and_cleared_by_a_zero_count() {
        for op in [ExtArithOp::Asl, ExtArithOp::Lsr, ExtArithOp::Asr, ExtArithOp::Ror] {
            for count in [0, 32] {
                let (_, f) = run(op, 0xFFFF_FFFF, count, (false, false, true, true));
                assert!(!f[2], "{op:?} count {count}: C must be cleared");
                assert!(f[3], "{op:?} count {count}: V must be kept");
            }
        }
        let f = (false, false, false, false);
        assert!(run(ExtArithOp::Asl, 0x8000_0000, 1, f).1[2]);
        assert!(!run(ExtArithOp::Asl, 0x0000_0001, 32 + 31, f).1[2]);
        assert!(run(ExtArithOp::Lsr, 0x0000_0002, 2, f).1[2]);
        assert!(run(ExtArithOp::Asr, 0x0000_0001, 33, f).1[2]);
        assert!(run(ExtArithOp::Ror, 0x0000_0001, 1, f).1[2]);
    }

    #[test]
    fn norm_flags_describe_the_source() {
        // (source, count, Z, N): C and V preset to 1 must survive.
        for (src, count, z, n) in [
            (0u32, 31, true, false),
            (0x8000_0000, 0, false, true),
            (0xFFFF_FFFF, 31, false, true),
            (1, 30, false, false),
        ] {
            let (r, f) = run(ExtArithOp::Norm, src, 0, (false, false, true, true));
            assert_eq!((r, f), (count, [z, n, true, true]), "norm.f {src:#x}");
        }
        for (src, count, z, n) in [
            (0xABCD_0000u32, 15, true, false),
            (0x0000_8000, 0, false, true),
            (0x0000_FFFF, 15, false, true),
            (0x0000_0001, 14, false, false),
        ] {
            let (r, f) = run(ExtArithOp::Normw, src, 0, (false, false, true, true));
            assert_eq!((r, f), (count, [z, n, true, true]), "normw.f {src:#x}");
        }
    }

    #[test]
    fn saturating_group_faults() {
        for op in [
            ExtArithOp::Adds, ExtArithOp::Subs, ExtArithOp::Divaw, ExtArithOp::Asls,
            ExtArithOp::Asrs, ExtArithOp::Addsdw, ExtArithOp::Subsdw, ExtArithOp::Sat16,
            ExtArithOp::Rnd16, ExtArithOp::Abss, ExtArithOp::Abssw, ExtArithOp::Negs,
            ExtArithOp::Negsw,
        ] {
            let mut m = FlatMachine::new(0);
            m.pc = 0x100;
            m.regs[2] = 0xDEAD;
            let r = ext_arith(&mut m, op, Operand::Reg(2), Operand::Reg(0), Operand::Reg(1), true, None);
            assert_eq!(r, Err(Exception::InstructionError { address: 0x100 }), "{op:?}");
            assert_eq!(m.regs[2], 0xDEAD, "{op:?} must not write");
        }
    }
}
