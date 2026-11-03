//! Architectural sequencing of one step, around the execution of an
//! instruction: the delay slot, the zero-overhead loop, the BLINK of a
//! delayed link branch, the PC update, and the entry into an exception.
//!
//! A step is, in order:
//!
//! 1. [`begin`]: consume a pending delay slot; outside a delay slot, at
//!    LP_END with LP_COUNT non-zero and loops enabled, count down, and jump
//!    back to LP_START unless the count reached zero. A jump ends the step:
//!    no instruction executes.
//! 2. decode the instruction at the PC (the caller's: fetching is platform);
//! 3. [`link_slot`]: the instruction in the delay slot of `bl.d`/`jl.d` sees
//!    BLINK already set to the address after itself;
//! 4. `begin_insn`, then execute;
//! 5. [`finish`]: after a delay-slot instruction, go to the branch target;
//!    otherwise to the next instruction, unless the instruction moved the PC
//!    itself without leaving a delay slot pending.

use crate::dom::{AuxReg, Dom, Flag};
use crate::exception::Exception;
use crate::regs::{REG_BLINK, REG_ILINK2, REG_LP_COUNT};

use super::regs::{status32, write_core_reg};

/// Pending delay slot (target, links) and whether the zero-overhead loop
/// jumped back, which ends the step.
#[inline(always)]
pub fn begin<D: Dom>(d: &mut D) -> (Option<(D::V, bool)>, D::B) {
    let delay = d.take_delay();
    if delay.is_some() {
        // No loop-back from a delay slot.
        let no = d.kb(false);
        return (delay, no);
    }
    let l = d.flag(Flag::L);
    let enabled = d.bnot(l);
    let pc = d.k(d.pc());
    let end = d.aux(AuxReg::LpEnd);
    let at_end = d.eq(pc, end);
    let count = d.reg(REG_LP_COUNT);
    let zero = d.k(0);
    let exhausted = d.eq(count, zero);
    let live = d.bnot(exhausted);
    let here = d.band(enabled, at_end);
    let g = d.band(here, live);
    let one = d.k(1);
    let next = d.sub(count, one);
    let next_zero = d.eq(next, zero);
    let again = d.bnot(next_zero);
    let jump = d.band(g, again);
    // The guarded bodies cannot fail.
    let _ = d.when(g, |d| {
        d.set_reg(REG_LP_COUNT, next);
        Ok(())
    });
    let _ = d.when(jump, |d| {
        let start = d.aux(AuxReg::LpStart);
        d.set_pc(start);
        Ok(())
    });
    (delay, jump)
}

/// In the delay slot of a linking branch, BLINK is the address after the slot.
#[inline(always)]
pub fn link_slot<D: Dom>(d: &mut D, delay: Option<(D::V, bool)>, next_pc: u32)
    -> Result<(), Exception> {
    if let Some((_, true)) = delay {
        let r = d.k(next_pc);
        write_core_reg(d, REG_BLINK, r)?;
    }
    Ok(())
}

/// Where execution continues after the instruction.
#[inline(always)]
pub fn finish<D: Dom>(d: &mut D, delay: Option<(D::V, bool)>, next_pc: u32) {
    match delay {
        Some((target, _)) => d.set_pc(target),
        None => {
            let pending = d.delay_pending();
            let written = d.pc_written();
            let not_written = d.bnot(written);
            let sequential = d.bor(pending, not_written);
            let _ = d.when(sequential, |d| {
                let n = d.k(next_pc);
                d.set_pc(n);
                Ok(())
            });
        }
    }
}

/// Enter the handler for a synchronous exception at `slot` of the vector
/// table, resuming later at `resume`. Measured on silicon for Instruction
/// Error, SWI and TRAP_S (slot 2): the vector is fetched from the table in
/// memory at INT_VECTOR_BASE; ILINK2 receives the resume address and
/// STATUS32_L2 the current STATUS32, which itself is left as it was;
/// AUX_IRQ_LV12 bit 1 is set; any pending delay slot is dropped.
#[inline]
pub fn exception_entry<D: Dom>(d: &mut D, slot: u32, resume: D::V) {
    let _ = d.take_delay();
    let st = status32(d);
    d.set_aux(AuxReg::Status32L2, st);
    d.set_reg(REG_ILINK2, resume);
    let lv12 = d.aux(AuxReg::IrqLv12);
    let l2 = d.k(1 << 1);
    let lv12 = d.or(lv12, l2);
    d.set_aux(AuxReg::IrqLv12, lv12);
    let base = d.aux(AuxReg::IntVectorBase);
    let off = d.k(slot * 8);
    let vector = d.add(base, off);
    d.set_pc(vector);
}
