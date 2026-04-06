use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::instruction::*;

use super::resolve_value;

pub fn execute_zero_op(zop: &ZeroOp, state: &mut CpuState) -> Result<(), Exception> {
    match zop {
        ZeroOp::Nop => Ok(()),
        ZeroOp::Sleep { .. } => {
            state.sleeping = true;
            Ok(())
        }
        ZeroOp::Swi => Err(Exception::Trap { param: 0 }),
        ZeroOp::Brk => {
            state.halted = true;
            Ok(())
        }
        ZeroOp::Trap { param } => Err(Exception::Trap { param: *param }),
        ZeroOp::Rtie => {
            if state.flag_u {
                return Err(Exception::PrivilegeViolation {
                    address: state.pc,
                });
            }
            if state.flag_ae {
                // Return from level 2 exception
                let saved = state.aux_status32_l2;
                state.set_status32(saved);
                state.pc = state.aux_eret;
                state.pc_written = true;
                state.aux_bta = state.aux_bta_l2;
            } else {
                // Return from level 1 exception
                let saved = state.aux_status32_l1;
                state.set_status32(saved);
                state.pc = state.aux_eret;
                state.pc_written = true;
                state.aux_bta = state.aux_bta_l1;
            }
            Ok(())
        }
        ZeroOp::Sync => Ok(()),
    }
}

pub fn execute_loop(
    offset: u32,
    cc: Option<ConditionCode>,
    decoded: &DecodedInstruction,
    state: &mut CpuState,
) -> Result<(), Exception> {
    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let next_pc = decoded.pc + decoded.total_size();
    state.aux_lp_start = next_pc;
    state.aux_lp_end = decoded.pc.wrapping_add(offset);

    Ok(())
}

pub fn execute_flag(
    src: Operand,
    cc: Option<ConditionCode>,
    state: &mut CpuState,
) -> Result<(), Exception> {
    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            return Ok(());
        }
    }

    let val = resolve_value(src, state)?;

    // Bit 0 = H (halt)
    if val & 1 != 0 {
        if state.flag_u {
            return Err(Exception::PrivilegeViolation {
                address: state.pc,
            });
        }
        state.halted = true;
        state.set_status32(val);
        return Ok(());
    }

    if state.flag_u {
        // User mode: only Z, N, C, V can be written
        state.flag_z = (val >> 11) & 1 != 0;
        state.flag_n = (val >> 10) & 1 != 0;
        state.flag_c = (val >> 9) & 1 != 0;
        state.flag_v = (val >> 8) & 1 != 0;
    } else {
        state.set_status32(val);
    }

    Ok(())
}
