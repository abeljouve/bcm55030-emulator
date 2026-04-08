use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::fields;
use crate::decoder::instruction::{Operand, SingleOp};
use crate::memory::Memory;

use super::{resolve_value, write_dest};

pub fn execute_single_op(
    op: SingleOp,
    dst: Operand,
    src: Operand,
    set_flags: bool,
    cc: Option<ConditionCode>,
    state: &mut CpuState,
    mem: &mut Memory,
) -> Result<(), Exception> {
    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    // EX: atomic exchange with memory
    if matches!(op, SingleOp::Ex) {
        let addr = resolve_value(src, state)?;
        let reg_val = resolve_value(dst, state)?;
        let mem_val = mem.read_word(addr)?;
        mem.write_word(addr, reg_val)?;
        write_dest(dst, mem_val, state)?;
        return Ok(());
    }

    let val = resolve_value(src, state)?;
    let carry_in = state.flag_c;
    let mut overflow: Option<bool> = None;

    let (result, carry) = match op {
        SingleOp::Asl => {
            let c = (val >> 31) != 0;
            (val << 1, Some(c))
        }
        SingleOp::Asr => {
            let c = (val & 1) != 0;
            (((val as i32) >> 1) as u32, Some(c))
        }
        SingleOp::Lsr => {
            let c = (val & 1) != 0;
            (val >> 1, Some(c))
        }
        SingleOp::Ror => {
            let c = (val & 1) != 0;
            (val.rotate_right(1), Some(c))
        }
        SingleOp::Rrc => {
            let c = (val & 1) != 0;
            let result = (val >> 1) | ((carry_in as u32) << 31);
            (result, Some(c))
        }
        SingleOp::Sexb => (fields::sign_extend(val & 0xFF, 8) as u32, None),
        SingleOp::Sexw => (fields::sign_extend(val & 0xFFFF, 16) as u32, None),
        SingleOp::Extb => (val & 0xFF, None),
        SingleOp::Extw => (val & 0xFFFF, None),
        SingleOp::Abs => {
            let signed = val as i32;
            if signed == i32::MIN {
                overflow = Some(true);
                (0x80000000u32, Some(true))
            } else if signed < 0 {
                ((-signed) as u32, Some(true))
            } else {
                (val, Some(false))
            }
        }
        SingleOp::Not => (!val, None),
        SingleOp::Rlc => {
            let c = (val >> 31) != 0;
            let result = (val << 1) | (carry_in as u32);
            (result, Some(c))
        }
        SingleOp::Ex => unreachable!(),
    };

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
