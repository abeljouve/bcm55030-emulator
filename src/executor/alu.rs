use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::instruction::{AluOp, Operand};

use super::multiply;
use super::{resolve_value, write_dest};

pub fn execute_alu(
    op: AluOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
    state: &mut CpuState,
) -> Result<(), Exception> {
    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let a = resolve_value(src1, state)?;
    let b = resolve_value(src2, state)?;

    let (result, carry, overflow) = compute_alu(op, a, b, state.flag_c);

    write_dest(dst, result, state)?;

    if set_flags {
        state.flag_z = result == 0;
        state.flag_n = (result >> 31) != 0;
        if let Some(c) = carry {
            state.flag_c = c;
        }
        if let Some(v) = overflow {
            state.flag_v = v;
        }
    }

    Ok(())
}

fn compute_alu(op: AluOp, a: u32, b: u32, carry_in: bool) -> (u32, Option<bool>, Option<bool>) {
    match op {
        AluOp::Add => {
            let (result, carry) = a.overflowing_add(b);
            let overflow = ((a ^ result) & (b ^ result)) >> 31 != 0;
            (result, Some(carry), Some(overflow))
        }
        AluOp::Adc => {
            let c = carry_in as u32;
            let (r1, c1) = a.overflowing_add(b);
            let (result, c2) = r1.overflowing_add(c);
            let carry = c1 || c2;
            let overflow = ((a ^ result) & (b ^ result)) >> 31 != 0;
            (result, Some(carry), Some(overflow))
        }
        AluOp::Sub | AluOp::Cmp => {
            // ARC convention: C = borrow (C=1 when A < B unsigned)
            let (result, borrow) = a.overflowing_sub(b);
            let carry = borrow;
            let overflow = ((a ^ b) & (a ^ result)) >> 31 != 0;
            (result, Some(carry), Some(overflow))
        }
        AluOp::Sbc => {
            // SBC: A - B - C (subtract with carry/borrow)
            // ARC convention: C = borrow, so borrow_in = carry_in
            let borrow_in = carry_in as u32;
            let (r1, b1) = a.overflowing_sub(b);
            let (result, b2) = r1.overflowing_sub(borrow_in);
            let carry = b1 || b2;
            let overflow = ((a ^ b) & (a ^ result)) >> 31 != 0;
            (result, Some(carry), Some(overflow))
        }
        AluOp::And | AluOp::Tst => (a & b, None, None),
        AluOp::Or => (a | b, None, None),
        AluOp::Bic => (a & !b, None, None),
        AluOp::Xor => (a ^ b, None, None),
        AluOp::Max => {
            // Flags from internal comparison (a - b)
            let (cmp_result, borrow) = a.overflowing_sub(b);
            let overflow = ((a ^ b) & (a ^ cmp_result)) >> 31 != 0;
            if (a as i32) >= (b as i32) {
                (a, Some(borrow), Some(overflow))
            } else {
                (b, Some(borrow), Some(overflow))
            }
        }
        AluOp::Min => {
            // Flags from internal comparison (a - b)
            let (cmp_result, borrow) = a.overflowing_sub(b);
            let overflow = ((a ^ b) & (a ^ cmp_result)) >> 31 != 0;
            if (a as i32) <= (b as i32) {
                (a, Some(borrow), Some(overflow))
            } else {
                (b, Some(borrow), Some(overflow))
            }
        }
        AluOp::Mov => (b, None, None),
        AluOp::Rcmp => {
            // ARC convention: C = borrow
            let (result, borrow) = b.overflowing_sub(a);
            let carry = borrow;
            let overflow = ((b ^ a) & (b ^ result)) >> 31 != 0;
            (result, Some(carry), Some(overflow))
        }
        AluOp::Rsub => {
            // ARC convention: C = borrow
            let (result, borrow) = b.overflowing_sub(a);
            let carry = borrow;
            let overflow = ((b ^ a) & (b ^ result)) >> 31 != 0;
            (result, Some(carry), Some(overflow))
        }
        AluOp::Bset => (a | (1u32 << (b & 31)), None, None),
        AluOp::Bclr => (a & !(1u32 << (b & 31)), None, None),
        AluOp::Btst => (a & (1u32 << (b & 31)), None, None),
        AluOp::Bxor => (a ^ (1u32 << (b & 31)), None, None),
        AluOp::Bmsk => {
            let width = (b & 31) + 1;
            let mask = if width >= 32 {
                0xFFFFFFFF
            } else {
                (1u32 << width) - 1
            };
            (a & mask, None, None)
        }
        AluOp::Add1 => add_shifted(a, b, 1),
        AluOp::Add2 => add_shifted(a, b, 2),
        AluOp::Add3 => add_shifted(a, b, 3),
        AluOp::Sub1 => sub_shifted(a, b, 1),
        AluOp::Sub2 => sub_shifted(a, b, 2),
        AluOp::Sub3 => sub_shifted(a, b, 3),
        AluOp::Mpy => (multiply::mpy(a, b), None, None),
        AluOp::Mpyh => (multiply::mpyh(a, b), None, None),
        AluOp::Mpyu => (multiply::mpyu(a, b), None, None),
        AluOp::Mpyhu => (multiply::mpyhu(a, b), None, None),
    }
}

fn add_shifted(a: u32, b: u32, shift: u32) -> (u32, Option<bool>, Option<bool>) {
    let shifted = b << shift;
    let (result, carry) = a.overflowing_add(shifted);
    let overflow = ((a ^ result) & (shifted ^ result)) >> 31 != 0;
    (result, Some(carry), Some(overflow))
}

fn sub_shifted(a: u32, b: u32, shift: u32) -> (u32, Option<bool>, Option<bool>) {
    let shifted = b << shift;
    // ARC convention: C = borrow
    let (result, borrow) = a.overflowing_sub(shifted);
    let carry = borrow;
    let overflow = ((a ^ shifted) & (a ^ result)) >> 31 != 0;
    (result, Some(carry), Some(overflow))
}
