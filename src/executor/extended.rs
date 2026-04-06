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
    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let a = resolve_value(src1, state)?;
    let b = resolve_value(src2, state)?;

    let result = match op {
        ExtArithOp::Asl => {
            let shift = b & 31;
            if shift == 0 { a } else { a << shift }
        }
        ExtArithOp::Lsr => {
            let shift = b & 31;
            if shift == 0 { a } else { a >> shift }
        }
        ExtArithOp::Asr => {
            let shift = b & 31;
            ((a as i32) >> shift) as u32
        }
        ExtArithOp::Ror => a.rotate_right(b & 31),
        ExtArithOp::Adds => (a as i32).saturating_add(b as i32) as u32,
        ExtArithOp::Subs => (a as i32).saturating_sub(b as i32) as u32,
        ExtArithOp::Addsdw => {
            let ah = (a >> 16) as i16;
            let al = a as i16;
            let bh = (b >> 16) as i16;
            let bl = b as i16;
            let rh = ah.saturating_add(bh) as u16;
            let rl = al.saturating_add(bl) as u16;
            ((rh as u32) << 16) | (rl as u32)
        }
        ExtArithOp::Subsdw => {
            let ah = (a >> 16) as i16;
            let al = a as i16;
            let bh = (b >> 16) as i16;
            let bl = b as i16;
            let rh = ah.saturating_sub(bh) as u16;
            let rl = al.saturating_sub(bl) as u16;
            ((rh as u32) << 16) | (rl as u32)
        }
        ExtArithOp::Divaw => {
            // Division assist: if src1 >= 0 then (src1<<1) - src2 else (src1<<1) + src2
            if (a as i32) >= 0 {
                (a << 1).wrapping_sub(b)
            } else {
                (a << 1).wrapping_add(b)
            }
        }
        ExtArithOp::Asls => {
            // Shift left with saturation; negative shift = right shift
            let shift = if src2 == Operand::None { 1i32 } else { b as i32 };
            if shift < 0 {
                ((a as i32) >> (-shift).min(31)) as u32
            } else {
                let s = shift.min(31) as u32;
                let val = a as i32;
                let result = (val as i64) << s;
                if result > i32::MAX as i64 {
                    0x7FFFFFFFu32
                } else if result < i32::MIN as i64 {
                    0x80000000u32
                } else {
                    result as i32 as u32
                }
            }
        }
        ExtArithOp::Asrs => {
            // Shift right with saturation; negative shift = left shift
            let shift = if src2 == Operand::None { 1i32 } else { b as i32 };
            if shift < 0 {
                let s = (-shift).min(31) as u32;
                let val = a as i32;
                let result = (val as i64) << s;
                if result > i32::MAX as i64 {
                    0x7FFFFFFFu32
                } else if result < i32::MIN as i64 {
                    0x80000000u32
                } else {
                    result as i32 as u32
                }
            } else {
                ((a as i32) >> shift.min(31)) as u32
            }
        }
        ExtArithOp::Sat16 => {
            let v = a as i32;
            if v > 0x7FFF {
                0x7FFF
            } else if v < -0x8000 {
                0xFFFF8000u32
            } else {
                a & 0xFFFF
            }
        }
        ExtArithOp::Rnd16 => {
            let sum = (a as i32 as i64) + 0x8000;
            let shifted = (sum >> 16) as i32;
            if shifted > 0x7FFF {
                0x7FFF
            } else if shifted < -0x8000 {
                0xFFFF8000u32
            } else {
                shifted as u32
            }
        }
        ExtArithOp::Abss => {
            let v = a as i32;
            if v == i32::MIN {
                0x7FFFFFFFu32
            } else {
                v.unsigned_abs()
            }
        }
        ExtArithOp::Abssw => {
            let v = (a as i16) as i32;
            let abs_v = if v == i16::MIN as i32 {
                i16::MAX as i32
            } else {
                v.abs()
            };
            abs_v as u32
        }
        ExtArithOp::Negs => {
            let v = a as i32;
            if v == i32::MIN {
                0x7FFFFFFFu32
            } else {
                (-v) as u32
            }
        }
        ExtArithOp::Negsw => {
            let v = (a as i16) as i32;
            let neg = -v;
            if neg > i16::MAX as i32 {
                i16::MAX as u32
            } else {
                neg as u16 as u32
            }
        }
        ExtArithOp::Norm => {
            if a == 0 || a == 0xFFFFFFFF {
                31
            } else if (a >> 31) == 0 {
                a.leading_zeros() - 1
            } else {
                (!a).leading_zeros() - 1
            }
        }
        ExtArithOp::Normw => {
            let v16 = (a & 0xFFFF) as u16;
            if v16 == 0 || v16 == 0xFFFF {
                15
            } else if (v16 >> 15) == 0 {
                v16.leading_zeros() as u32 - 1
            } else {
                (!v16).leading_zeros() as u32 - 1
            }
        }
        ExtArithOp::Swap => ((a & 0xFFFF) << 16) | ((a >> 16) & 0xFFFF),
    };

    write_dest(dst, result, state)?;

    if set_flags {
        state.flag_z = result == 0;
        state.flag_n = (result >> 31) != 0;
    }

    Ok(())
}
