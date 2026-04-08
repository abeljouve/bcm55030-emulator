pub mod condition;
pub mod exception;
pub mod registers;

use exception::Exception;
use registers::{CpuState, DelayState, REG_BLINK, REG_ILINK1, REG_ILINK2, REG_LP_COUNT};

use crate::decoder;
use crate::executor;
use crate::hooks::{self, HookAction, HookTable};
use crate::memory::Memory;

/// UART interrupt number (IRQ 5, level 1 per aux_irq_lev = 0xD7 bit 5 = 0).
const UART_IRQ: u32 = 5;

/// UART IRQ prescaler: check every N instructions.
/// On real hardware, the UART is baud-rate limited (~5760 bytes/sec at 57600 baud).
/// Without throttling, the ISR drains the TX ring buffer instantly.
const UART_PRESCALER: u64 = 256;

pub struct Cpu {
    pub state: CpuState,
    pub mem: Memory,
    /// Log every instruction to stderr
    pub trace: bool,
    /// PC address hooks for SoC-specific behavior (boot ROM intercepts, stubs, etc.)
    pub hooks: HookTable,
}

impl Cpu {
    /// Create a CPU with flat memory (for tests / simple use).
    pub fn new(mem_size: usize) -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new(mem_size),
            trace: false,
            hooks: HookTable::new(),
        }
    }

    /// Create a BCM55030 CPU with Harvard architecture (separate ICCM/DCCM + MMIO).
    pub fn new_bcm55030() -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new_harvard(
                crate::memory::ICCM_SIZE,
                crate::memory::DCCM_SIZE,
            ),
            trace: false,
            hooks: HookTable::new(),
        }
    }

    pub fn step(&mut self) -> Result<(), Exception> {
        if self.state.halted {
            return Ok(());
        }

        // Hook dispatch — all SoC-specific behavior (boot ROM, stubs, milestones)
        // is injected via hooks. The core ARC700 step loop has no SoC knowledge.
        if !self.hooks.is_empty() {
            if let Some(&hook) = self.hooks.get(&self.state.pc) {
                match hooks::execute_hook(hook, &mut self.state, &mut self.mem)? {
                    HookAction::Skip => return Ok(()),
                    HookAction::Continue => {}
                }
            }
        }

        // When sleeping, only tick timers and check interrupts.
        if self.state.sleeping {
            self.tick_timers();
            let uart_pending = if let Some(mmio) = self.mem.mmio() {
                mmio.uart.irq_pending()
            } else {
                false
            };
            if uart_pending {
                self.state.aux_irq_pending |= 1 << UART_IRQ;
            }
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
                self.state.pc, decoded.total_size(),
                self.state.flag_z as u8, self.state.flag_n as u8,
                self.state.flag_c as u8, self.state.flag_v as u8,
                decoded.inst
            );
        }

        // For BL.D/JL.D: set blink to address AFTER the delay slot
        if let Some((_target, true)) = delay_info {
            self.state.write_core_reg(REG_BLINK, next_pc)?;
        }

        // Update MMIO PC context for unhandled register logging
        if let Some(mut mmio) = self.mem.mmio() {
            mmio.current_pc = self.state.pc;
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


        // Timer tick
        self.tick_timers();

        // UART peripheral IRQ
        self.check_uart_irq();

        // Check for pending interrupts
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

    /// Advance timers by one tick.
    /// ARC 700 timers always count (no start/stop bit).
    fn tick_timers(&mut self) {
        // BCM55030 EPON MAC free-running timer at SYSREG+0x050
        const HW_TIMER_PRESCALER: u64 = 64;
        if self.state.instruction_count % HW_TIMER_PRESCALER == 0 {
            if let Some(mut mmio) = self.mem.mmio() {
                mmio.timer_counter = mmio.timer_counter.wrapping_add(1);
            }
        }

        // ARC Timer0/1 prescaler
        const TIMER_PRESCALER: u64 = 128;
        if self.state.instruction_count % TIMER_PRESCALER != 0 {
            return;
        }

        // Timer 0 (IRQ 3)
        self.state.aux_count0 = self.state.aux_count0.wrapping_add(1);
        if self.state.aux_limit0 != 0 && self.state.aux_count0 >= self.state.aux_limit0 {
            self.state.aux_control0 |= 0x08; // IP bit
            if self.state.aux_control0 & 0x05 != 0 {
                self.state.aux_irq_pending |= 1 << 3;
            }
            if self.state.aux_control0 & 0x02 == 0 {
                self.state.aux_count0 = 0;
            }
        }
        // Timer 1 (IRQ 4)
        self.state.aux_count1 = self.state.aux_count1.wrapping_add(1);
        if self.state.aux_limit1 != 0 && self.state.aux_count1 >= self.state.aux_limit1 {
            self.state.aux_control1 |= 0x08; // IP bit
            if self.state.aux_control1 & 0x05 != 0 {
                self.state.aux_irq_pending |= 1 << 4;
            }
            if self.state.aux_control1 & 0x02 == 0 {
                self.state.aux_count1 = 0;
            }
        }
    }

    /// Set UART IRQ pending bit if the UART peripheral needs service.
    fn check_uart_irq(&mut self) {
        if self.state.instruction_count % UART_PRESCALER != 0 {
            return;
        }
        let pending = if let Some(mmio) = self.mem.mmio() {
            mmio.uart.irq_pending()
        } else {
            false
        };
        if pending {
            self.state.aux_irq_pending |= 1 << UART_IRQ;
        }
    }

    /// Check and take pending interrupts.
    /// Returns true if an interrupt was taken (PC changed to vector).
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
            self.state.flag_e2 = false;
            self.state.flag_a2 = true;
            self.state.flag_de = false;
            self.state.flag_u = false;
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
        }

        self.state.aux_irq_pending &= !(1 << irq);

        let vector = 16 + irq;
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
