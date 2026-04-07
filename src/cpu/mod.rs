pub mod condition;
pub mod exception;
pub mod registers;

use exception::Exception;
use registers::{CpuState, DelayState, REG_BLINK, REG_ILINK1, REG_ILINK2, REG_LP_COUNT};

use crate::decoder;
use crate::executor;
use crate::memory::{Memory, DCCM_SIZE, ICCM_SIZE};

/// UART interrupt number (IRQ 5, level 1 per aux_irq_lev = 0xD7 bit 5 = 0).
/// The bootloader's UART ISR at 0x4348 ends with J.F [ILINK1] (level 1 RTIE),
/// so the UART IRQ must be level 1.
const UART_IRQ: u32 = 5;

pub struct Cpu {
    pub state: CpuState,
    pub mem: Memory,
    /// Log every instruction to stderr
    pub trace: bool,
}

impl Cpu {
    /// Create a CPU with flat memory (for tests / simple use).
    pub fn new(mem_size: usize) -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new(mem_size),
            trace: false,
        }
    }

    /// Create a BCM55030 CPU with Harvard architecture (separate ICCM/DCCM + MMIO).
    pub fn new_bcm55030() -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new_harvard(ICCM_SIZE, DCCM_SIZE),
            trace: false,
        }
    }

    pub fn step(&mut self) -> Result<(), Exception> {
        if self.state.halted {
            return Ok(());
        }

        // When sleeping, only tick timers and check interrupts.
        // An interrupt wakes the CPU from SLEEP.
        if self.state.sleeping {
            self.tick_timers();
            self.check_uart_irq();
            if self.check_interrupts() {
                self.state.sleeping = false;
                // PC was set to interrupt vector by check_interrupts
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
                // new_count == 0: last iteration done, fall through
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
        // (must happen before executing the delay slot, per ISA spec)
        if let Some((_target, true)) = delay_info {
            self.state.write_core_reg(REG_BLINK, next_pc)?;
        }

        // Execute
        self.state.pc_written = false;
        executor::execute(&decoded, &mut self.state, &mut self.mem)?;

        // PC update logic
        if let Some((target, _is_link)) = delay_info {
            // Completed delay slot: jump to saved branch target
            self.state.pc = target;
        } else if matches!(self.state.delay_state, DelayState::DelaySlot { .. }) {
            // Branch set up a delay slot: advance to delay slot instruction
            self.state.pc = next_pc;
        } else if !self.state.pc_written {
            // PC not explicitly set by executor: normal advance
            self.state.pc = next_pc;
        }
        // else: PC explicitly set by executor (branch/jump NoDelay, RTIE)

        self.state.instruction_count += 1;

        // Timer tick (simple: increment once per instruction)
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

    /// Advance timers by one tick (called every TIMER_PRESCALER instructions).
    /// ARC 700 timers always count (no start/stop bit).
    /// BCM55030 timer CONTROL: bit 0 = IE (interrupt enable when count reaches limit),
    /// bit 1 = NH (not halted), bit 3 = IP (interrupt pending, write-1-to-clear).
    /// Note: the bootloader writes CONTROL1 = 4 (bit 2), so we treat bit 2 as IE
    /// for BCM55030 compatibility (non-standard).
    fn tick_timers(&mut self) {
        // BCM55030 EPON MAC free-running timer at SYSREG+0x050.
        // This is independent of the ARC Timer1 — it's a hardware counter
        // read by timer1_get_current_value (0x45E4) for delay loops.
        const HW_TIMER_PRESCALER: u64 = 64;
        if self.state.instruction_count % HW_TIMER_PRESCALER == 0 {
            if let Some(mut mmio) = self.mem.mmio() {
                mmio.timer_counter = mmio.timer_counter.wrapping_add(1);
            }
        }

        // ARC timer prescaler: on real BCM55030, the timer clock is slower than the CPU.
        const TIMER_PRESCALER: u64 = 128;
        if self.state.instruction_count % TIMER_PRESCALER != 0 {
            return;
        }

        // Timer 0 (IRQ 3)
        self.state.aux_count0 = self.state.aux_count0.wrapping_add(1);
        if self.state.aux_limit0 != 0 && self.state.aux_count0 >= self.state.aux_limit0 {
            self.state.aux_control0 |= 0x08; // IP bit
            if self.state.aux_control0 & 0x05 != 0 {
                // IE: bit 0 (standard) or bit 2 (BCM55030)
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
                // IE: bit 0 (standard) or bit 2 (BCM55030)
                self.state.aux_irq_pending |= 1 << 4;
            }
            if self.state.aux_control1 & 0x02 == 0 {
                self.state.aux_count1 = 0;
            }
        }
    }

    /// Set UART IRQ pending bit if the UART peripheral needs service.
    fn check_uart_irq(&mut self) {
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
        // Don't take interrupts during delay slots
        if self.state.delay_state != DelayState::None {
            return false;
        }

        // Combine hardware pending with software hint
        let pending = self.state.aux_irq_pending & self.state.aux_ienable;
        if pending == 0 {
            return false;
        }

        // Find highest priority (lowest numbered) pending interrupt
        let irq = pending.trailing_zeros();
        if irq >= 32 {
            return false;
        }

        // Determine interrupt level from AUX_IRQ_LEV (bit set = level 2, clear = level 1)
        let is_level2 = (self.state.aux_irq_lev >> irq) & 1 != 0;

        if is_level2 {
            // Level 2: check E2 enabled and not already in level 2
            if !self.state.flag_e2 || self.state.flag_a2 {
                return false;
            }
            // Save state
            self.state.aux_status32_l2 = self.state.status32();
            self.state.aux_bta_l2 = self.state.aux_bta;
            self.state.core_regs[REG_ILINK2 as usize] = self.state.pc;
            self.state.aux_icause2 = irq;
            // Update STATUS32
            self.state.flag_e2 = false;
            self.state.flag_a2 = true;
            self.state.flag_de = false;
            self.state.flag_u = false;
        } else {
            // Level 1: check E1 enabled and not already in level 1
            if !self.state.flag_e1 || self.state.flag_a1 {
                return false;
            }
            // Save state
            self.state.aux_status32_l1 = self.state.status32();
            self.state.aux_bta_l1 = self.state.aux_bta;
            self.state.core_regs[REG_ILINK1 as usize] = self.state.pc;
            self.state.aux_icause1 = irq;
            // Update STATUS32
            self.state.flag_e1 = false;
            self.state.flag_e2 = false;
            self.state.flag_a1 = true;
            self.state.flag_de = false;
            self.state.flag_u = false;
        }

        // Clear pending bit (edge-triggered)
        self.state.aux_irq_pending &= !(1 << irq);

        // Jump to interrupt vector: base + vector_number * 8
        // IRQ N uses vector (16 + N) for ARC 700
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
