pub mod condition;
pub mod exception;
pub mod registers;

use exception::Exception;
use registers::{CpuState, DelayState, REG_BLINK, REG_LP_COUNT};

use crate::decoder;
use crate::executor;
use crate::memory::{Memory, DCCM_SIZE, ICCM_SIZE};

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
        if self.state.halted || self.state.sleeping {
            return Ok(());
        }

        // BCM55030 UART intercept: the bootloader's UART is interrupt-driven
        // (ring buffer + TX interrupt handler). Since we don't deliver interrupts,
        // the buffer fills and uart_send_byte_blocking loops forever.
        // Intercept: write character directly to stdout and return immediately.
        if self.mem.is_harvard() && self.state.pc == 0x42F4 {
            // uart_send_byte_blocking(char): r0 = character, blink = return addr
            let ch = self.state.core_regs[0] as u8;
            use std::io::Write;
            let _ = std::io::stdout().lock().write_all(&[ch]);
            let _ = std::io::stdout().lock().flush();
            // Simulate return: PC = blink (r31)
            self.state.pc = self.state.core_regs[REG_BLINK as usize];
            self.state.instruction_count += 1;
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
    fn tick_timers(&mut self) {
        // Timer 0
        if self.state.aux_control0 & 1 != 0 {
            // Timer enabled (bit 0 of CONTROL)
            self.state.aux_count0 = self.state.aux_count0.wrapping_add(1);
            if self.state.aux_limit0 != 0 && self.state.aux_count0 >= self.state.aux_limit0 {
                // Set interrupt pending (bit 3 of CONTROL = IP)
                self.state.aux_control0 |= 0x08;
                // Auto-reload if not halted mode (bit 1 of CONTROL = NH)
                if self.state.aux_control0 & 0x02 == 0 {
                    self.state.aux_count0 = 0;
                }
            }
        }
        // Timer 1
        if self.state.aux_control1 & 1 != 0 {
            self.state.aux_count1 = self.state.aux_count1.wrapping_add(1);
            if self.state.aux_limit1 != 0 && self.state.aux_count1 >= self.state.aux_limit1 {
                self.state.aux_control1 |= 0x08;
                if self.state.aux_control1 & 0x02 == 0 {
                    self.state.aux_count1 = 0;
                }
            }
        }
    }
}
