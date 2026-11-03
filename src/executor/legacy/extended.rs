#![allow(dead_code, clippy::all)]
use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::instruction::{ExtArithOp, Operand};

use super::{resolve_value, write_dest};

pub fn execute_ext_arith(
    op: ExtArithOp,
    dst: Operand,
    src1: Operand,
    src2: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
    state: &mut CpuState,
) -> Result<(), Exception> {
    // The saturating/DSP group is not implemented on this core: it faults
    // whatever its condition. INFERRED for the false-condition case: only
    // unconditional forms were measured.
    if op.is_absent_on_core() {
        return Err(Exception::InstructionError { address: state.pc });
    }

    if let Some(cc) = cc {
        if !super::cond::evaluate(cc, state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let a = resolve_value(src1, state)?;
    let b = resolve_value(src2, state)?;

    match op {
        ExtArithOp::Asl | ExtArithOp::Lsr | ExtArithOp::Asr | ExtArithOp::Ror => {
            let (result, carry) = barrel_shift(op, a, b);
            write_dest(dst, result, state)?;
            if set_flags {
                state.flag_z = result == 0;
                state.flag_n = (result >> 31) != 0;
                state.flag_c = carry;
            }
        }
        ExtArithOp::Norm | ExtArithOp::Normw => {
            // The destination gets the normalisation count, but Z and N
            // describe the SOURCE (NORMW: its low half). C and V are kept.
            // Observed on silicon.
            let (result, src_zero, src_neg) = if op == ExtArithOp::Norm {
                (norm32(a), a == 0, (a >> 31) != 0)
            } else {
                let v16 = a as u16;
                (norm16(v16), v16 == 0, (v16 >> 15) != 0)
            };
            write_dest(dst, result, state)?;
            if set_flags {
                state.flag_z = src_zero;
                state.flag_n = src_neg;
            }
        }
        ExtArithOp::Swap => {
            let result = a.rotate_left(16);
            write_dest(dst, result, state)?;
            if set_flags {
                state.flag_z = result == 0;
                state.flag_n = (result >> 31) != 0;
            }
        }
        _ => unreachable!("{op:?} faults before it computes"),
    }

    Ok(())
}

/// Multi-bit shift through the barrel shifter. The count is taken modulo 32
/// (a count of 32 shifts by 0, 33 by 1). C is the last bit shifted out, and is
/// cleared when the masked count is 0. V is never written. Observed on silicon
/// for register counts 0..=63 on all four operations; the immediate (u6)
/// forms go through the same operation and are assumed to match (INFERRED).
fn barrel_shift(op: ExtArithOp, a: u32, count: u32) -> (u32, bool) {
    let s = count & 31;
    if s == 0 {
        return (a, false);
    }
    match op {
        ExtArithOp::Asl => (a << s, (a >> (32 - s)) & 1 != 0),
        ExtArithOp::Lsr => (a >> s, (a >> (s - 1)) & 1 != 0),
        ExtArithOp::Asr => (((a as i32) >> s) as u32, (a >> (s - 1)) & 1 != 0),
        ExtArithOp::Ror => (a.rotate_right(s), (a >> (s - 1)) & 1 != 0),
        _ => unreachable!(),
    }
}

/// Leading sign bits minus one: how far `v` can shift left before its sign
/// changes. 0 and all-ones give 31.
fn norm32(v: u32) -> u32 {
    if v >> 31 == 0 {
        v.leading_zeros().saturating_sub(1).min(31)
    } else {
        (!v).leading_zeros().saturating_sub(1).min(31)
    }
}

fn norm16(v: u16) -> u32 {
    if v >> 15 == 0 {
        v.leading_zeros().saturating_sub(1).min(15)
    } else {
        (!v).leading_zeros().saturating_sub(1).min(15)
    }
}

