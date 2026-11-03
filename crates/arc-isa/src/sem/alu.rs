//! Two-operand ALU operations (major opcode 0x04, sub-opcodes 0x00-0x1D).

use crate::condition::ConditionCode;
use crate::decoder::instruction::{AluOp, Operand};
use crate::dom::{Dom, Flag};
use crate::exception::Exception;

use super::regs::{cond_holds, operand, write_dest};

/// Execute `op` with the operands as decoded. Flags are written only with
/// `set_flags`; a false condition does nothing, reads included.
#[inline]
pub fn alu<D: Dom>(
    d: &mut D,
    op: AluOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
) -> Result<(), Exception> {
    // An unimplemented opcode faults whatever its condition. INFERRED for the
    // false-condition case: only unconditional forms were measured.
    if op.is_absent_on_core() {
        return Err(Exception::InstructionError { address: d.pc() });
    }
    match cc {
        None => body(d, op, dst, src1, src2, set_flags),
        Some(cc) => {
            let g = cond_holds(d, cc);
            d.when(g, |d| body(d, op, dst, src1, src2, set_flags))
        }
    }
}

/// Result of one operation: the value written, and the flags it defines.
/// `zn` overrides the value Z and N are taken from.
struct Out<V, B> {
    r: V,
    c: Option<B>,
    v: Option<B>,
    zn: Option<V>,
}

#[inline(always)]
fn body<D: Dom>(
    d: &mut D,
    op: AluOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
) -> Result<(), Exception> {
    let a = operand(d, src1)?;
    let b = operand(d, src2)?;
    let o = compute(d, op, a, b);
    write_dest(d, dst, o.r)?;
    if set_flags {
        let zn = o.zn.unwrap_or(o.r);
        let zero = d.k(0);
        let z = d.eq(zn, zero);
        let n = d.bit(zn, 31);
        d.set_flag(Flag::Z, z);
        d.set_flag(Flag::N, n);
        if let Some(c) = o.c {
            d.set_flag(Flag::C, c);
        }
        if let Some(v) = o.v {
            d.set_flag(Flag::V, v);
        }
    }
    Ok(())
}

/// Signed overflow of `a + b = r`: both operands differ in sign from the result.
#[inline(always)]
fn add_overflow<D: Dom>(d: &mut D, a: D::V, b: D::V, r: D::V) -> D::B {
    let x = d.xor(a, r);
    let y = d.xor(b, r);
    let t = d.and(x, y);
    d.bit(t, 31)
}

/// Signed overflow of `a - b = r`: the operands differ in sign, and so do `a` and `r`.
#[inline(always)]
fn sub_overflow<D: Dom>(d: &mut D, a: D::V, b: D::V, r: D::V) -> D::B {
    let x = d.xor(a, b);
    let y = d.xor(a, r);
    let t = d.and(x, y);
    d.bit(t, 31)
}

#[inline(always)]
fn add_full<D: Dom>(d: &mut D, a: D::V, b: D::V) -> Out<D::V, D::B> {
    let r = d.add(a, b);
    let c = d.ult(r, a);
    let v = add_overflow(d, a, b, r);
    Out { r, c: Some(c), v: Some(v), zn: None }
}

/// `a - b`; C is the borrow (set when `a < b` unsigned).
#[inline(always)]
fn sub_full<D: Dom>(d: &mut D, a: D::V, b: D::V) -> Out<D::V, D::B> {
    let r = d.sub(a, b);
    let c = d.ult(a, b);
    let v = sub_overflow(d, a, b, r);
    Out { r, c: Some(c), v: Some(v), zn: None }
}

/// `1 << (b & 31)`.
#[inline(always)]
fn bit_mask<D: Dom>(d: &mut D, b: D::V) -> D::V {
    let m31 = d.k(31);
    let n = d.and(b, m31);
    let one = d.k(1);
    d.shl(one, n)
}

#[inline(always)]
fn logic<D: Dom>(r: D::V) -> Out<D::V, D::B> {
    Out { r, c: None, v: None, zn: None }
}

#[inline(always)]
fn compute<D: Dom>(d: &mut D, op: AluOp, a: D::V, b: D::V) -> Out<D::V, D::B> {
    match op {
        AluOp::Add => add_full(d, a, b),
        AluOp::Adc => {
            let cin = d.flag(Flag::C);
            let ci = d.b2v(cin);
            let r1 = d.add(a, b);
            let r = d.add(r1, ci);
            let c1 = d.ult(r1, a);
            let c2 = d.ult(r, r1);
            let c = d.bor(c1, c2);
            let v = add_overflow(d, a, b, r);
            Out { r, c: Some(c), v: Some(v), zn: None }
        }
        AluOp::Sub | AluOp::Cmp => sub_full(d, a, b),
        AluOp::Sbc => {
            // A - B - C, with C as the borrow in.
            let cin = d.flag(Flag::C);
            let bi = d.b2v(cin);
            let r1 = d.sub(a, b);
            let r = d.sub(r1, bi);
            let b1 = d.ult(a, b);
            let b2 = d.ult(r1, bi);
            let c = d.bor(b1, b2);
            let v = sub_overflow(d, a, b, r);
            Out { r, c: Some(c), v: Some(v), zn: None }
        }
        AluOp::And | AluOp::Tst => {
            let r = d.and(a, b);
            logic::<D>(r)
        }
        AluOp::Or => {
            let r = d.or(a, b);
            logic::<D>(r)
        }
        AluOp::Bic => {
            let nb = d.not(b);
            let r = d.and(a, nb);
            logic::<D>(r)
        }
        AluOp::Xor => {
            let r = d.xor(a, b);
            logic::<D>(r)
        }
        // MAX/MIN: Z, N and V come from the comparison a - b; C is set when
        // the second operand is the one selected, ties included. Observed on
        // silicon (C is not the borrow of a - b).
        AluOp::Max => {
            let cmp = d.sub(a, b);
            let v = sub_overflow(d, a, b, cmp);
            let b_lt_a = d.slt(b, a);
            let b_sel = d.bnot(b_lt_a);
            let r = d.ite(b_sel, b, a);
            Out { r, c: Some(b_sel), v: Some(v), zn: Some(cmp) }
        }
        AluOp::Min => {
            let cmp = d.sub(a, b);
            let v = sub_overflow(d, a, b, cmp);
            let a_lt_b = d.slt(a, b);
            let b_sel = d.bnot(a_lt_b);
            let r = d.ite(b_sel, b, a);
            Out { r, c: Some(b_sel), v: Some(v), zn: Some(cmp) }
        }
        AluOp::Mov => logic::<D>(b),
        AluOp::Rcmp | AluOp::Rsub => sub_full(d, b, a),
        AluOp::Bset => {
            let m = bit_mask(d, b);
            let r = d.or(a, m);
            logic::<D>(r)
        }
        AluOp::Bclr => {
            let m = bit_mask(d, b);
            let nm = d.not(m);
            let r = d.and(a, nm);
            logic::<D>(r)
        }
        AluOp::Btst => {
            let m = bit_mask(d, b);
            let r = d.and(a, m);
            logic::<D>(r)
        }
        AluOp::Bxor => {
            let m = bit_mask(d, b);
            let r = d.xor(a, m);
            logic::<D>(r)
        }
        AluOp::Bmsk => {
            // Mask of the low (b & 31) + 1 bits.
            let m31 = d.k(31);
            let n = d.and(b, m31);
            let sh = d.sub(m31, n);
            let ones = d.k(u32::MAX);
            let mask = d.lshr(ones, sh);
            let r = d.and(a, mask);
            logic::<D>(r)
        }
        AluOp::Add1 | AluOp::Add2 | AluOp::Add3 => {
            let s = match op {
                AluOp::Add1 => 1,
                AluOp::Add2 => 2,
                _ => 3,
            };
            let sh = d.k(s);
            let bs = d.shl(b, sh);
            add_full(d, a, bs)
        }
        AluOp::Sub1 | AluOp::Sub2 | AluOp::Sub3 => {
            let s = match op {
                AluOp::Sub1 => 1,
                AluOp::Sub2 => 2,
                _ => 3,
            };
            let sh = d.k(s);
            let bs = d.shl(b, sh);
            sub_full(d, a, bs)
        }
        AluOp::Mpy | AluOp::Mpyh | AluOp::Mpyu | AluOp::Mpyhu => {
            unreachable!("{op:?} faults before it computes")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flat::FlatMachine;

    #[test]
    fn minmax_flags_match_silicon() {
        // (op, src1, src2, result, Z, N, C, V), measured on silicon.
        type Case = (AluOp, u32, u32, u32, bool, bool, bool, bool);
        const CASES: &[Case] = &[
            (AluOp::Min, 0, 0, 0, true, false, true, false),
            (AluOp::Max, 0, 0, 0, true, false, true, false),
            (AluOp::Min, 1, 0xFFFF_FFFF, 0xFFFF_FFFF, false, false, true, false),
            (AluOp::Min, 0xFFFF_FFFF, 1, 0xFFFF_FFFF, false, true, false, false),
            (AluOp::Max, 1, 0xFFFF_FFFF, 1, false, false, false, false),
            (AluOp::Max, 0xFFFF_FFFF, 1, 1, false, true, true, false),
            (AluOp::Min, 0x8000_0000, 0x7FFF_FFFF, 0x8000_0000, false, false, false, true),
            (AluOp::Min, 0x7FFF_FFFF, 0x8000_0000, 0x8000_0000, false, true, true, true),
            (AluOp::Max, 0x8000_0000, 0x7FFF_FFFF, 0x7FFF_FFFF, false, false, true, true),
            (AluOp::Max, 0x7FFF_FFFF, 0x8000_0000, 0x7FFF_FFFF, false, true, false, true),
            (AluOp::Min, 0x8000_0000, 0xFFFF_FFFF, 0x8000_0000, false, true, false, false),
            (AluOp::Min, 0xFFFF_FFFF, 0x8000_0000, 0x8000_0000, false, false, true, false),
            (AluOp::Max, 0x8000_0000, 0xFFFF_FFFF, 0xFFFF_FFFF, false, true, true, false),
            (AluOp::Max, 0xFFFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, false, false, false, false),
            (AluOp::Min, 0x7FFF_FFFF, 0, 0, false, false, true, false),
            (AluOp::Min, 0, 0x7FFF_FFFF, 0, false, true, false, false),
            (AluOp::Max, 0x7FFF_FFFF, 0, 0x7FFF_FFFF, false, false, false, false),
            (AluOp::Max, 0, 0x7FFF_FFFF, 0x7FFF_FFFF, false, true, true, false),
        ];
        for &(op, a, b, res, z, n, c, v) in CASES {
            let mut m = FlatMachine::new(0);
            m.regs[0] = a;
            m.regs[1] = b;
            alu(&mut m, op, Operand::Reg(2), Operand::Reg(0), Operand::Reg(1), true, None).unwrap();
            assert_eq!(
                (m.regs[2], m.flag(Flag::Z), m.flag(Flag::N), m.flag(Flag::C), m.flag(Flag::V)),
                (res, z, n, c, v),
                "{op:?} {a:#x},{b:#x}"
            );
        }
    }

    #[test]
    fn multiply_faults_without_writing() {
        for op in [AluOp::Mpy, AluOp::Mpyh, AluOp::Mpyu, AluOp::Mpyhu] {
            let mut m = FlatMachine::new(0);
            m.pc = 0x200;
            m.regs[0] = 3;
            m.regs[1] = 7;
            let r = alu(&mut m, op, Operand::Reg(8), Operand::Reg(0), Operand::Reg(1), false, None);
            assert_eq!(r, Err(Exception::InstructionError { address: 0x200 }), "{op:?}");
            assert_eq!(m.regs[8], 0, "{op:?}");
        }
    }
}
