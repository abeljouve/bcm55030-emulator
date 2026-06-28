use crate::cpu::condition::ConditionCode;
use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::decoder::instruction::*;

use super::resolve_value;

pub fn execute_zero_op(zop: &ZeroOp, state: &mut CpuState) -> Result<(), Exception> {
    match zop {
        ZeroOp::Nop => Ok(()),
        ZeroOp::Sleep { u6 } => {
            // SLEEP operand can selectively enable interrupt levels
            // Bits[4:3] of u6 control E1/E2 enable during sleep
            if *u6 != 0 {
                let mask = *u6 as u32;
                // Bit 3 = E1 enable, Bit 4 = E2 enable
                state.flag_e1 = (mask >> 3) & 1 != 0;
                state.flag_e2 = (mask >> 4) & 1 != 0;
            }
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
            // OBSERVED (silicon): the BCM55030 ARC700 core does NOT implement
            // `rtie`. Executing it raises vec=2 (Instruction Error), exactly
            // as a non-existent opcode would. The interrupt-return idiom on
            // this core is `j[.f] [ilink1/2]` (see executor/branch.rs and the
            // decode32.rs ILINK handling). Two independent silicon bisects
            // (2026-05-18 and 2026-06-28) showed an ISR ending in `rtie`
            // faulting vec=2 (`ABCDdQ` then vec=2), while the same ISR ending
            // in `j [ilink]` returned cleanly. A 4 MB scan of the reference flash
            // dump (reference-fw-dump.bin) found ZERO `rtie` opcodes
            // (all 4 byte-orderings) but 8x `j.f [ilink1]` (0x20208740) and
            // 11x `j.f [ilink2]` (0x20208780): reference's sole interrupt-return
            // idiom is `j.f [ilink1/2]`; it never uses `rtie`.
            // See the design notes.
            //
            // The ZeroOp::Rtie decode variant is intentionally kept so the
            // disassembler still LABELS the opcode "rtie" (format.rs); only
            // EXECUTION must fault, matching silicon.
            Err(Exception::InstructionError { address: state.pc })
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
    // ISA: "aux_reg[LP_END] = cPCL + s13" — the LP target is computed
    // from cPCL (current PC longword = PC & ~3), NOT raw PC. For LP at a
    // half-word-aligned address (e.g. 0x...c2 after a 16-bit instruction),
    // the offset must be added to the 4-byte-aligned PC, otherwise
    // LP_END is off by 2 and the loop body runs only once.
    let pcl = decoded.pc & 0xFFFFFFFC;

    if let Some(cc) = cc {
        if !cc.evaluate(state.flag_z, state.flag_n, state.flag_c, state.flag_v) {
            // Condition not met: skip the loop body by jumping to lp_end.
            // ARC 700 ISA: "If the condition is not satisfied, no loop is
            // set up and a branch is made to the target of the LP instruction"
            state.pc = pcl.wrapping_add(offset);
            state.pc_written = true;
            return Ok(());
        }
    }

    // ISA: LP clears the L bit (re-enables zero-overhead loops)
    state.flag_l = false;

    let next_pc = decoded.pc + decoded.total_size();
    state.aux_lp_start = next_pc;
    state.aux_lp_end = pcl.wrapping_add(offset);

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
            return Err(Exception::PrivilegeViolation { address: state.pc });
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::registers::REG_ILINK1;

    #[test]
    fn rtie_faults_instruction_error_vec2() {
        // OBSERVED (silicon): the BCM55030 ARC700 does NOT implement `rtie`;
        // executing it raises vec=2 (Instruction Error). It must NOT restore
        // STATUS32/PC. See the design notes.
        let mut state = CpuState::new();
        state.pc = 0x1234;
        // Even with level-1 IRQ context primed, rtie must fault, not return.
        state.flag_a1 = true;
        state.aux_status32_l1 = 0xDEAD_BEEF;
        state.core_regs[REG_ILINK1 as usize] = 0x9999;

        let result = execute_zero_op(&ZeroOp::Rtie, &mut state);
        assert_eq!(
            result,
            Err(Exception::InstructionError { address: 0x1234 }),
            "rtie must raise InstructionError"
        );
        // Confirm InstructionError maps to vector 2 (the silicon-observed vec).
        assert_eq!(
            Exception::InstructionError { address: 0x1234 }.vector_number(),
            0x02,
            "InstructionError must be vector 2"
        );
        // And it must NOT have executed the return (PC untouched, no restore).
        assert!(!state.pc_written, "rtie must not write PC");
        assert_eq!(state.pc, 0x1234, "rtie must not change PC");
    }
}
