//! PC-indexed hooks, run before the instruction at that PC.

use std::collections::HashMap;

use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::memory::Memory;

/// Result of hook execution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HookAction {
    /// Skip the current instruction (hook handled everything)
    Skip,
    /// Continue normal execution (hook was informational)
    Continue,
    /// Pause the CPU before executing the instruction at this PC.
    /// The step loop sets `state.paused = true` and returns without
    /// advancing PC — the next `cpu.step()` call will still see the
    /// breakpoint PC. A UI / MCP `CpuCommand::StepOne` clears `paused`
    /// before calling `step()`, stepping through the breakpoint.
    Pause,
}

/// Function signature for custom hooks.
pub type HookFn = fn(&mut CpuState, &mut Memory) -> Result<HookAction, Exception>;

/// A hook registered at a specific PC address.
#[derive(Clone, Copy)]
pub enum Hook {
    /// Return immediately to caller (J [blink]). No return value.
    ReturnImmediate,
    /// Set r0 to value and return to caller (J [blink]).
    ReturnValue(u32),
    /// Log a message when this address is hit. Continues execution.
    Log(&'static str),
    /// Custom function. Gets full access to CPU state and memory.
    Custom(HookFn),
    /// UI / MCP breakpoint: pause before executing the instruction.
    /// Installed by `CpuCommand::SetBreakpoint` and removed by
    /// `RemoveBreakpoint`. Address-generic — the hook itself carries
    /// no firmware knowledge (the contributor guide).
    Breakpoint,
}

pub type HookTable = HashMap<u32, Hook>;

/// Execute a hook. Called from Cpu::step() when PC matches a registered hook.
pub fn execute_hook(
    hook: Hook,
    state: &mut CpuState,
    mem: &mut Memory,
) -> Result<HookAction, Exception> {
    match hook {
        Hook::ReturnImmediate => {
            state.pc = state.core_regs[31]; // REG_BLINK = 31
            state.instruction_count += 1;
            Ok(HookAction::Skip)
        }
        Hook::ReturnValue(val) => {
            state.core_regs[0] = val;
            state.pc = state.core_regs[31];
            state.instruction_count += 1;
            Ok(HookAction::Skip)
        }
        Hook::Log(msg) => {
            crate::vlog!("[Hook] {} at PC=0x{:05X}, blink=0x{:05X}, insn={}",
                msg, state.pc, state.core_regs[31], state.instruction_count);
            Ok(HookAction::Continue)
        }
        Hook::Custom(f) => f(state, mem),
        Hook::Breakpoint => Ok(HookAction::Pause),
    }
}
