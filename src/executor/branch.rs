use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::{CpuState, DelayState, REG_BLINK};
use crate::decoder::instruction::*;

use super::resolve_value;

pub fn execute_branch(
    decoded: &DecodedInstruction,
    offset: i32,
    cc: Option<ConditionCode>,
    delay: DelayMode,
    link: bool,
    state: &mut CpuState,
) -> Result<(), Exception> {
    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    // All ARC branch offsets are relative to PCL (word-aligned PC)
    let pcl = decoded.pc & !3;
    let target = (pcl as i64 + offset as i64) as u32;
    let next_pc = decoded.pc + decoded.total_size();

    match delay {
        DelayMode::Delay => {
            // For BL.D, blink is set later in step() when we know the delay slot size.
            // For B.D (no link), nothing to do here.
            state.delay_state = DelayState::DelaySlot { target, is_link: link };
        }
        DelayMode::NoDelay => {
            if link {
                state.write_core_reg(REG_BLINK, next_pc)?;
            }
            state.pc = target;
            state.pc_written = true;
        }
    }

    Ok(())
}

pub fn execute_branch_compare(
    kind: BrCompareKind,
    src1: Operand,
    src2: Operand,
    offset: i32,
    delay: DelayMode,
    decoded: &DecodedInstruction,
    state: &mut CpuState,
) -> Result<(), Exception> {
    let a = resolve_value(src1, state)?;
    let b = resolve_value(src2, state)?;

    let taken = match kind {
        BrCompareKind::Breq => a == b,
        BrCompareKind::Brne => a != b,
        BrCompareKind::Brlt => (a as i32) < (b as i32),
        BrCompareKind::Brge => (a as i32) >= (b as i32),
        BrCompareKind::Brlo => a < b,
        BrCompareKind::Brhs => a >= b,
        BrCompareKind::Bbit0 => (a >> (b & 31)) & 1 == 0,
        BrCompareKind::Bbit1 => (a >> (b & 31)) & 1 == 1,
    };

    if !taken {
        return Ok(());
    }

    // All ARC branch offsets are relative to PCL (word-aligned PC)
    let pcl = decoded.pc & !3;
    let target = (pcl as i64 + offset as i64) as u32;

    match delay {
        DelayMode::Delay => {
            // BRcc never has link, is_link always false
            state.delay_state = DelayState::DelaySlot { target, is_link: false };
        }
        DelayMode::NoDelay => {
            state.pc = target;
            state.pc_written = true;
        }
    }

    Ok(())
}

pub fn execute_jump(
    decoded: &DecodedInstruction,
    target_op: Operand,
    cc: Option<ConditionCode>,
    delay: DelayMode,
    link: bool,
    flag_restore: bool,
    state: &mut CpuState,
) -> Result<(), Exception> {
    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let target = resolve_value(target_op, state)?;
    let next_pc = decoded.pc + decoded.total_size();

    // J.F [ILINK1/2]: restore STATUS32 from saved level
    if flag_restore {
        if let Operand::Reg(r) = target_op {
            let saved_status = if r == 29 {
                state.aux_status32_l1
            } else {
                state.aux_status32_l2
            };
            state.set_status32(saved_status);
        }
    }

    match delay {
        DelayMode::Delay => {
            // For JL.D, blink is set later in step() when we know the delay slot size.
            state.delay_state = DelayState::DelaySlot { target, is_link: link };
        }
        DelayMode::NoDelay => {
            if link {
                state.write_core_reg(REG_BLINK, next_pc)?;
            }
            state.pc = target;
            state.pc_written = true;
        }
    }

    Ok(())
}
