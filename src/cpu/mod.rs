pub mod condition;
pub mod exception;
pub mod registers;

use std::sync::Arc;

use parking_lot::RwLock;

use exception::Exception;
use registers::{CpuState, DelayState, REG_BLINK, REG_ILINK1, REG_ILINK2, REG_LP_COUNT};

use crate::decoder;
use crate::executor;
use crate::hooks::{self, HookAction, HookTable};
use crate::memory::Memory;
use crate::soc::bank::{BootMode, PeripheralBank, BANK_TICK_PRESCALER};

/// UART interrupt number (IRQ 5, level 1 per aux_irq_lev = 0xD7 bit 5 = 0).
const UART_IRQ: u32 = 5;

/// UART IRQ poll prescaler (audit 3.2).
const UART_PRESCALER: u64 = 256;

pub struct Cpu {
    pub state: CpuState,
    pub mem: Memory,
    /// Log every instruction to stderr
    pub trace: bool,
    /// PC address hooks for breakpoints / watchpoints / run-to-cursor.
    /// Empty by default on `develop` — all 35 SoC-specific entries from
    /// the old `register_hooks()` were deleted per the contributor guide. The
    /// infrastructure stays for the future UI debug features.
    pub hooks: HookTable,
    /// Fractional accumulator for ARC Timer0/1 tick rate.
    /// 156.25 MHz clock, ~1.76 cycles/instruction average.
    timer_frac_acc: u32,
    /// Shared peripheral bank (cloned from `mem.bank()`). Cpu holds an
    /// additional `Arc` clone so it can tick the bank without going
    /// through `Memory`'s hot path.
    bank: Option<Arc<RwLock<PeripheralBank>>>,
    /// Instructions elapsed since the last bank tick.
    bank_tick_accumulator: u64,
}

impl Cpu {
    /// Create a CPU with flat memory (for tests / simple use).
    pub fn new(mem_size: usize) -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new(mem_size),
            trace: false,
            hooks: HookTable::new(),
            timer_frac_acc: 0,
            bank: None,
            bank_tick_accumulator: 0,
        }
    }

    /// Create a BCM55030 CPU with unified SRAM + peripheral bank.
    pub fn new_bcm55030(boot_mode: BootMode) -> Self {
        let mut state = CpuState::new();
        // BCM55030 wires Timer 1 to IRQ 7 (standard ARC 700 uses IRQ 4).
        state.timer1_irq = 7;
        let mem = Memory::new_soc(crate::memory::SRAM_SIZE, boot_mode);
        let bank = mem.bank().cloned();
        Self {
            state,
            mem,
            trace: false,
            hooks: HookTable::new(),
            timer_frac_acc: 0,
            bank,
            bank_tick_accumulator: 0,
        }
    }

    /// Access the shared peripheral bank handle. Used by `main.rs` to
    /// grab the UART mpsc sender and wire stdin → UART input.
    pub fn bank(&self) -> Option<&Arc<RwLock<PeripheralBank>>> {
        self.bank.as_ref()
    }

    pub fn step(&mut self) -> Result<(), Exception> {
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

        // Fetch and decode
        let decoded = decoder::decode(self.state.pc, &self.mem)?;
        let next_pc = self.state.pc + decoded.total_size();

        if self.trace {
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

        // For BL.D/JL.D: set blink to address AFTER the delay slot
        if let Some((_target, true)) = delay_info {
            self.state.write_core_reg(REG_BLINK, next_pc)?;
        }

        // Update CPU context on the bank for watchpoint / trace output.
        if let Some(ref bank) = self.bank {
            let mut guard = bank.write();
            guard.update_cpu_context(
                self.state.pc,
                self.state.core_regs[31],
                self.state.instruction_count,
            );
        }

        // Execute
        self.state.pc_written = false;
        executor::execute(&decoded, &mut self.state, &mut self.mem)?;

        // PC update logic
        if let Some((target, _is_link)) = delay_info {
            self.state.pc = target;
        } else if matches!(self.state.delay_state, DelayState::DelaySlot { .. }) {
            self.state.pc = next_pc;
        } else if !self.state.pc_written {
            self.state.pc = next_pc;
        }

        self.state.instruction_count += 1;

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

    pub fn run(&mut self, max_steps: u64) -> Result<(), Exception> {
        for _ in 0..max_steps {
            if self.state.halted || self.state.sleeping {
                break;
            }
            self.step()?;
        }
        Ok(())
    }

    /// Advance timers (CPU-side ARC Timer0/1) and the peripheral bank
    /// (which drives per-peripheral tick state).
    fn tick_timers_and_bank(&mut self) {
        // ARC Timer0/1: ~1.76 ticks per instruction (156.25 MHz, ~89 MIPS).
        // Bare-metal verified: 1000 NOPs = 7,026 COUNT0 ticks.
        self.timer_frac_acc += 176;
        while self.timer_frac_acc >= 100 {
            self.timer_frac_acc -= 100;
            self.tick_arc_timers_once();
        }

        // Peripheral bank tick (EPON free-running counter, PBC busy
        // counters, BSC busy counters, UART mpsc drain, future SerDes /
        // MACsec / etc.).
        self.bank_tick_accumulator += 1;
        if self.bank_tick_accumulator >= BANK_TICK_PRESCALER {
            self.bank_tick_accumulator = 0;
            if let Some(ref bank) = self.bank {
                bank.write().tick(BANK_TICK_PRESCALER);
            }
        }
    }

    /// Increment ARC Timer0 and Timer1 by one tick each.
    fn tick_arc_timers_once(&mut self) {
        // Timer 0 (IRQ 3)
        self.state.aux_count0 = self.state.aux_count0.wrapping_add(1);
        if self.state.aux_limit0 != 0 && self.state.aux_count0 >= self.state.aux_limit0 {
            self.state.aux_control0 |= 0x08;
            if self.state.aux_control0 & 0x01 != 0 {
                self.state.aux_irq_pending |= 1 << 3;
            }
            if self.state.aux_control0 & 0x02 == 0 {
                self.state.aux_count0 = 0;
            }
        }
        // Timer 1 (IRQ 7 on BCM55030).
        self.state.aux_count1 = self.state.aux_count1.wrapping_add(1);
        if self.state.aux_limit1 != 0 && self.state.aux_count1 >= self.state.aux_limit1 {
            self.state.aux_control1 |= 0x08;
            if self.state.aux_control1 & 0x01 != 0 {
                self.state.aux_irq_pending |= 1 << self.state.timer1_irq;
            }
            self.state.aux_count1 = 0;
        }
    }

    /// Poll the UART IRQ line via the peripheral bank. Prescaled to
    /// avoid hot-path contention on every instruction.
    fn check_uart_irq(&mut self) {
        if self.state.instruction_count % UART_PRESCALER != 0 {
            return;
        }
        let pending = if let Some(ref bank) = self.bank {
            bank.read().uart.irq_pending() != 0
        } else {
            false
        };
        if pending {
            self.state.aux_irq_pending |= 1 << UART_IRQ;
        }
    }

    /// Check and take pending interrupts.
    fn check_interrupts(&mut self) -> bool {
        if self.state.delay_state != DelayState::None {
            return false;
        }

        let pending = self.state.aux_irq_pending & self.state.aux_ienable;
        if pending == 0 {
            return false;
        }

        let irq = pending.trailing_zeros();
        if irq >= 32 {
            return false;
        }

        let is_level2 = (self.state.aux_irq_lev >> irq) & 1 != 0;

        if is_level2 {
            if !self.state.flag_e2 || self.state.flag_a2 {
                return false;
            }
            self.state.aux_status32_l2 = self.state.status32();
            self.state.aux_bta_l2 = self.state.aux_bta;
            self.state.core_regs[REG_ILINK2 as usize] = self.state.pc;
            self.state.aux_icause2 = irq;
            // L2 entry: keep E1 set (matches old behaviour). Audit 2.4
            // asked for clearing both E1 and E2 per spec, but doing so
            // caused a boot regression at 0x4428 — needs separate RE
            // before re-enabling. Tracked in the design notes §2.4.
            self.state.flag_e2 = false;
            self.state.flag_a2 = true;
            self.state.flag_de = false;
            self.state.flag_u = false;
            self.state.flag_l = true;

            self.state.irq_shadow_r0_r3[0] = self.state.core_regs[0];
            self.state.irq_shadow_r0_r3[1] = self.state.core_regs[1];
            self.state.irq_shadow_r0_r3[2] = self.state.core_regs[2];
            self.state.irq_shadow_r0_r3[3] = self.state.core_regs[3];
        } else {
            if !self.state.flag_e1 || self.state.flag_a1 {
                return false;
            }
            self.state.aux_status32_l1 = self.state.status32();
            self.state.aux_bta_l1 = self.state.aux_bta;
            self.state.core_regs[REG_ILINK1 as usize] = self.state.pc;
            self.state.aux_icause1 = irq;
            self.state.flag_e1 = false;
            self.state.flag_e2 = false;
            self.state.flag_a1 = true;
            self.state.flag_de = false;
            self.state.flag_u = false;
            self.state.flag_l = true;

            self.state.irq_shadow_r0_r3[0] = self.state.core_regs[0];
            self.state.irq_shadow_r0_r3[1] = self.state.core_regs[1];
            self.state.irq_shadow_r0_r3[2] = self.state.core_regs[2];
            self.state.irq_shadow_r0_r3[3] = self.state.core_regs[3];
        }

        self.state.aux_irq_pending &= !(1 << irq);

        let vector = irq;
        self.state.pc = self.state.aux_int_vector_base + vector * 8;
        self.state.pc_written = true;

        if self.trace {
            eprintln!(
                "[IRQ] Took level {} interrupt IRQ {} → vector 0x{:08X}",
                if is_level2 { 2 } else { 1 },
                irq,
                self.state.pc
            );
        }

        true
    }
}
