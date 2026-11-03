//! Frozen copy of `Cpu::step` (and its exception entry), with the frozen
//! executor, kept as the reference the generic step sequencing is checked
//! against. Only names are changed.
#![allow(dead_code, clippy::all)]

use super::*;
use crate::cpu::registers::REG_LP_COUNT;

/// `AUX_IRQ_LV12` bit 1: set on entry to a level-2 handler.
const LV12_L2: u32 = 1 << 1;

impl Cpu {
    pub(crate) fn legacy_step(&mut self) -> Result<(), Exception> {
        match self.legacy_step_raw() {
            Ok(()) => Ok(()),
            Err(e) => self.legacy_take_exception(e),
        }
    }

    fn legacy_step_raw(&mut self) -> Result<(), Exception> {
        if self.state.halted || self.state.paused {
            return Ok(());
        }

        // Hook dispatch — only for UI breakpoints / watchpoints. The
        // old 35 SoC hooks are gone.
        if !self.hooks.is_empty() {
            if let Some(&hook) = self.hooks.get(&self.state.pc) {
                match hooks::execute_hook(hook, &mut self.state, &mut self.mem)? {
                    HookAction::Skip => return Ok(()),
                    HookAction::Continue => {}
                    HookAction::Pause => {
                        self.state.paused = true;
                        self.state.pause_reason =
                            crate::cpu::registers::PauseReason::Breakpoint(self.state.pc);
                        return Ok(());
                    }
                }
            }
        }

        // When sleeping, only tick timers and check interrupts.
        if self.state.sleeping {
            self.tick_timers_and_bank();
            self.check_uart_irq();
            if self.check_interrupts() {
                self.state.sleeping = false;
            }
            return Ok(());
        }

        // Save and clear delay state
        let delay_info = match self.state.delay_state {
            DelayState::DelaySlot { target, is_link } => {
                self.state.delay_state = DelayState::None;
                Some((target, is_link))
            }
            DelayState::None => None,
        };

        // Zero-overhead loop check (not during delay slots)
        if delay_info.is_none() && !self.state.flag_l {
            let lp_count = self.state.core_regs[REG_LP_COUNT as usize];
            if self.state.pc == self.state.aux_lp_end && lp_count > 0 {
                let new_count = lp_count - 1;
                self.state.core_regs[REG_LP_COUNT as usize] = new_count;
                if new_count > 0 {
                    self.state.pc = self.state.aux_lp_start;
                    return Ok(());
                }
            }
        }

        // Fetch and decode. A fault leaves the PC on the faulting
        // instruction (the executor never moves it before failing); the
        // `inspect_err`s record what the exception entry needs, on the error path
        // only, so the hot path keeps its shape.
        let decoded = decoder::decode(self.state.pc, &self.mem).inspect_err(|_| {
            self.fault_site = FaultSite::Decode(delay_info.map(|(target, _)| target));
        })?;
        let next_pc = self.state.pc + decoded.total_size();

        if self.trace {
            let src = self
                .debug_info
                .as_ref()
                .and_then(|di| di.lookup(self.state.pc));
            if let Some(loc) = src {
                eprintln!(
                    "[TRACE] PC=0x{:08X} size={} Z={} N={} C={} V={} {:?} @{}:{}",
                    self.state.pc,
                    decoded.total_size(),
                    self.state.flag_z as u8,
                    self.state.flag_n as u8,
                    self.state.flag_c as u8,
                    self.state.flag_v as u8,
                    decoded.inst,
                    loc.file,
                    loc.line
                );
            } else {
                eprintln!(
                    "[TRACE] PC=0x{:08X} size={} Z={} N={} C={} V={} {:?}",
                    self.state.pc,
                    decoded.total_size(),
                    self.state.flag_z as u8,
                    self.state.flag_n as u8,
                    self.state.flag_c as u8,
                    self.state.flag_v as u8,
                    decoded.inst
                );
            }
        }

        // For BL.D/JL.D: set blink to address AFTER the delay slot
        if let Some((_target, true)) = delay_info {
            self.state.write_core_reg(REG_BLINK, next_pc)?;
        }

        // Update CPU context on the bank for watchpoint / trace output.
        // Also set the .di flag from the current instruction so MMIO
        // history entries record whether the access used cache bypass.
        if let Some(ref bank) = self.bank {
            let mut guard = bank.write();
            guard.update_cpu_context(
                self.state.pc,
                self.state.core_regs[31],
                self.state.instruction_count,
            );
            guard.current_di = match &decoded.inst {
                Instruction::Store { cache_bypass, .. }
                | Instruction::Load { cache_bypass, .. } => *cache_bypass,
                _ => false,
            };
        }

        // Timing overlay: capture the load's effective address / SYNC BEFORE
        // execute, because a writeback (`.aw`/`.ab`) load mutates its base
        // register and would give the wrong post-execute EA (and hence a wrong
        // MMIO classification at the SRAM boundary).
        #[cfg(feature = "timing")]
        let timing_pre = if self.timing_shadow.is_some() {
            Some(Self::timing_load_info(&decoded.inst, &self.state))
        } else {
            None
        };

        // Execute
        self.state.pc_written = false;
        self.state.link_executed = false;
        crate::executor::legacy::execute(&decoded, &mut self.state, &mut self.mem).inspect_err(|_| {
            self.fault_site = FaultSite::Execute(delay_info.map_or(next_pc, |(target, _)| target));
        })?;

        // SR trace: log AUX register writes and record in history buffer.
        if let Instruction::StoreAux { src, addr } = &decoded.inst {
            let addr_val = executor::resolve_value(*addr, &self.state).unwrap_or(0);
            let val = executor::resolve_value(*src, &self.state).unwrap_or(0);
            if self.trace_sr {
                eprintln!(
                    "[SR] PC=0x{:05X} sr [0x{:X}], 0x{:08X}",
                    self.state.pc, addr_val, val
                );
            }
            if let Some(ref bank) = self.bank {
                bank.write().record_aux_write(addr_val, val);
            }
        }

        // PC update logic
        if let Some((target, _is_link)) = delay_info {
            self.state.pc = target;
        } else if matches!(self.state.delay_state, DelayState::DelaySlot { .. }) {
            self.state.pc = next_pc;
        } else if !self.state.pc_written {
            self.state.pc = next_pc;
        }

        self.state.instruction_count += 1;

        // Cycle-accurate cache + bus-contention overlay (opt-in `timing`
        // feature). Purely observational: the instruction has already executed
        // functionally above; the shadow only accounts for contention and, if
        // the fetch-vs-load starvation invariant fires, halts the CPU with a
        // BUS_DEADLOCK report. `decoded.pc` is the instruction address and
        // `self.state.pc` is the next fetch address (the fetch-ahead origin).
        #[cfg(feature = "timing")]
        if let Some((load, is_sync)) = timing_pre {
            let insn_pc = decoded.pc;
            let next_pc = self.state.pc;
            // Disjoint field borrows: the shadow and memory are separate fields.
            let shadow = self.timing_shadow.as_mut().unwrap();
            let hit_deadlock = shadow.on_instruction(insn_pc, load, next_pc, is_sync, &self.mem);
            self.state.debug_load_pending = shadow.load_pending;
            if hit_deadlock {
                if let Some(rep) = shadow.deadlock {
                    eprintln!(
                        "[BUS_DEADLOCK] pc=0x{:08X} load_ea=0x{:08X} mmio={} \
aliasing_fetch_line={:?} set_index={:?} starve={} cyc={} \
(DEBUG.LD=1: load starved by fixed-priority fetch stream)",
                        rep.pc,
                        rep.load_ea,
                        rep.load_mmio,
                        rep.aliasing_fetch_line.map(|l| format!("0x{l:08X}")),
                        rep.set_index,
                        rep.starve_cycles,
                        rep.cycle,
                    );
                }
                self.state.halted = true;
                self.state.pause_reason = crate::cpu::registers::PauseReason::BusDeadlock;
                return Ok(());
            }
        }

        // Spin detection (see `Cpu::step_raw`).
        if self.bank.is_some() {
            if self.state.pc == self.tight_loop_last_pc {
                self.tight_loop_count += 1;
                if self.tight_loop_count >= TIGHT_LOOP_THRESHOLD {
                    if self.tight_loop_halts {
                        // Harness mode: terminate this isolated run instead of
                        // rebooting the SoC. No log spam (spins are expected
                        // under fuzzing), no shared-state mutation.
                        self.state.halted = true;
                        return Ok(());
                    }
                    match self.classify_spin() {
                        SpinVerdict::WatchdogArmed => {
                            // The reset is real, but it belongs to the timer.
                            // Skip the simulated time the CPU would burn
                            // spinning so the host doesn't execute millions of
                            // no-op iterations, then let the timer fire it on
                            // its own terms (and at its own count).
                            self.fast_forward_armed_watchdog();
                            self.tight_loop_count = 0;
                        }
                        SpinVerdict::Interruptible => {
                            // An ISR can still break this. Keep executing —
                            // that is what the machine does. Re-arm the
                            // counter so the classification is re-checked
                            // periodically (the firmware may arm a watchdog,
                            // or mask interrupts, while spinning) without
                            // re-running it every single step.
                            self.tight_loop_count = 0;
                        }
                        SpinVerdict::Hung => {
                            // Nothing can ever move the PC again. Stop with
                            // the state INTACT and say so: registers, SRAM and
                            // the shadow call stack are the evidence, and a
                            // reboot would destroy all three.
                            eprintln!(
                                "[BCM55030] PC=0x{:08X} spinning with no watchdog armed and \
                                 interrupts masked -- permanent hang, halting with state intact",
                                self.state.pc
                            );
                            self.state.halted = true;
                            self.state.pause_reason =
                                crate::cpu::registers::PauseReason::SpinNoWatchdog(self.state.pc);
                            return Ok(());
                        }
                    }
                }
            } else {
                self.tight_loop_count = 0;
                self.tight_loop_last_pc = self.state.pc;
            }
        }

        // Shadow call stack: push on BL/JL, pop on J [blink].
        if let Some((_, true)) = delay_info {
            if self.shadow_call_stack.len() < 64 {
                self.shadow_call_stack.push(next_pc);
            }
        } else if self.state.link_executed {
            if self.shadow_call_stack.len() < 64 {
                let blink = self.state.core_regs[REG_BLINK as usize];
                self.shadow_call_stack.push(blink);
            }
        } else if self.state.pc_written {
            let blink = self.state.core_regs[REG_BLINK as usize];
            if self.state.pc == blink && !self.shadow_call_stack.is_empty() {
                self.shadow_call_stack.pop();
            }
        }

        if self.profiling_enabled {
            if let Some(&frame_pc) = self.shadow_call_stack.last() {
                *self.function_profile.entry(frame_pc).or_insert(0) += 1;
            }
        }

        // Watchpoint trap — if any read/write during executor set a
        // hit, pause the CPU. The next step() runs past the
        // watchpoint only after the UI clears `paused`.
        if let Some((wp_addr, wp_mode)) = self.mem.watchpoints.take_hit() {
            self.state.paused = true;
            self.state.pause_reason =
                crate::cpu::registers::PauseReason::Watch(wp_addr, wp_mode);
        }

        // Timers + peripheral bank
        self.tick_timers_and_bank();
        self.check_uart_irq();
        self.check_interrupts();

        Ok(())
    }

    #[cold]
    #[inline(never)]
    fn legacy_take_exception(&mut self, e: Exception) -> Result<(), Exception> {
        if self.stop_on_exception || !e.is_vectored() {
            return Err(e);
        }
        let fault_pc = self.state.pc;
        let resume = match self.fault_site {
            FaultSite::Execute(resume) => resume,
            FaultSite::Decode(Some(target)) => target,
            FaultSite::Decode(None) => fault_pc.wrapping_add(self.legacy_undecodable_size()),
        };
        // From the vector table in SRAM at INT_VECTOR_BASE, never through the
        // NCO alias that serves interrupt vectors: with both a `j @limm` NCO
        // channel 2 and an SRAM slot 2 installed, silicon took the SRAM one.
        let vector = self
            .state
            .aux_int_vector_base
            .wrapping_add(e.vector_number() as u32 * 8);
        // A fault on the vector itself would re-enter forever: there is
        // nothing left to observe, so stop with the state intact instead.
        if fault_pc == vector {
            return Err(e);
        }
        self.state.delay_state = DelayState::None;
        self.state.aux_status32_l2 = self.state.status32();
        self.state.core_regs[REG_ILINK2 as usize] = resume;
        self.state.aux_irq_lv12 |= LV12_L2;
        self.state.pc = vector;
        self.exceptions_taken += 1;
        if self.trace {
            eprintln!(
                "[EXC] {:?} at PC=0x{:08X} -> 0x{:08X}, ILINK2=0x{:08X}",
                e, fault_pc, vector, self.state.core_regs[REG_ILINK2 as usize]
            );
        }
        self.last_exception = Some((fault_pc, e));
        Ok(())
    }

    /// Size of an instruction the decoder rejected: the major opcode still
    /// says whether it is a 16- or a 32-bit encoding. INFERRED: a LIMM such an
    /// encoding might carry is not counted.
    fn legacy_undecodable_size(&self) -> u32 {
        match self.mem.fetch_half(self.state.pc) {
            Ok(hw) if (hw >> 11) >= 0x0C => 2,
            _ => 4,
        }
    }
}
