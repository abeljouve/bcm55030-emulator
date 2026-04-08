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

/// UART IRQ prescaler: check every N instructions.
/// On real hardware, the UART is baud-rate limited (~5760 bytes/sec at 57600 baud).
/// Without throttling, the ISR drains the TX ring buffer instantly, causing the
/// bootloader's tx_idle flush check (FUN_0x4428) to see an already-empty buffer
/// and loop forever. A prescaler of 256 provides enough delay for the software
/// to observe tx_idle=0 before the ISR finishes draining.
const UART_PRESCALER: u64 = 256;

pub struct Cpu {
    pub state: CpuState,
    pub mem: Memory,
    /// Log every instruction to stderr
    pub trace: bool,
    /// Trace firmware init only (activated after firmware_main_loop entry)
    pub trace_firmware_init: bool,
}

impl Cpu {
    /// Create a CPU with flat memory (for tests / simple use).
    pub fn new(mem_size: usize) -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new(mem_size),
            trace: false,
            trace_firmware_init: false,
        }
    }

    /// Create a BCM55030 CPU with Harvard architecture (separate ICCM/DCCM + MMIO).
    pub fn new_bcm55030() -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new_harvard(ICCM_SIZE, DCCM_SIZE),
            trace: false,
            trace_firmware_init: false,
        }
    }

    pub fn step(&mut self) -> Result<(), Exception> {
        if self.state.halted {
            return Ok(());
        }

        // Boot ROM "start app" intercept at ICCM address 0x32000.
        // The bootloader computes this address (0x31FB4 + 0x4C) and calls JL [r0].
        // On real hardware, the boot ROM has code here. We intercept and load firmware.
        if self.mem.is_harvard() && self.state.pc == self.mem.iccm_base + 0x32000 {
            if self.boot_rom_start_app() {
                return Ok(());
            }
        }

        // Boot ROM function intercepts for firmware.
        // The BCM55030 mask ROM (burned in silicon, not readable) provides functions
        // called by firmware's startup stub at 0x00-0xD8. The ICCM is filled with
        // J_S [blink] (0x7EE0) after the app binary, so these addresses already
        // return immediately. We intercept them explicitly to:
        // - Document what the real boot ROM does at each address
        // - Implement the one critical function (0x74B60) that must NOT return
        if self.state.pc >= 0x4E000 {
            if self.mem.is_harvard() {
                if self.boot_rom_intercept() {
                    return Ok(());
                }
            }
        }

        // When sleeping, only tick timers and check interrupts.
        // An interrupt wakes the CPU from SLEEP.
        if self.state.sleeping {
            self.tick_timers();
            // No prescaler during SLEEP — check every step for wakeup
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

        // Prevent firmware_main_loop from "returning" to the startup code.
        // The function has do{}while(true) and should never return. But on the
        // emulator, some init error path causes it to exit. The startup code at
        // 0x98 would branch to halt_loop, then restart via stack corruption.
        // We redirect back to firmware_main_loop to retry the init.
        if self.mem.app_size.is_some() && self.state.pc == 0x98 {
            // Just re-enter firmware_main_loop
            self.state.pc = 0x20C;
            return Ok(());
        }

        // SerDes register read intercept: hw_peripheral_read_dispatch returns 0 for
        // bus types it doesn't handle (type 0x00 = direct SerDes access, not SPI).
        // On real hardware, the BCM55030 has dedicated SerDes register buses.
        // We return 0xFF (all flags set) to satisfy calibration/ready checks.
        if self.mem.app_size.is_some() && self.state.pc == 0x12CA4 {
            let bus_type = (self.state.core_regs[0] >> 8) & 0xFF;
            if bus_type != 0x06 { // not SPI → unhandled bus type
                self.state.core_regs[0] = 0xFF;
                self.state.pc = self.state.core_regs[REG_BLINK as usize];
                self.state.instruction_count += 1;
                return Ok(());
            }
        }
        // Intercept serdes_hw_ready_flag (0x1E6C) — always return 1 (ready).
        // On real hardware, the boot ROM hw_init sets this flag. The emulator stubs
        // that function, so the flag never gets set.
        if self.mem.app_size.is_some() && self.state.pc == 0x1E6C {
            self.state.core_regs[0] = 1;
            self.state.pc = self.state.core_regs[REG_BLINK as usize];
            self.state.instruction_count += 1;
            return Ok(());
        }
        // Also intercept serdes_reg_write (0x12CD8) for non-SPI bus types — just skip
        if self.mem.app_size.is_some() && self.state.pc == 0x12CD8 {
            let bus_type = (self.state.core_regs[0] >> 8) & 0xFF;
            if bus_type != 0x06 {
                self.state.pc = self.state.core_regs[REG_BLINK as usize];
                self.state.instruction_count += 1;
                return Ok(());
            }
        }

        // Firmware init progress milestones — lightweight, always active in Harvard mode
        if self.mem.is_harvard() && self.mem.app_size.is_some() {
            match self.state.pc {
                0x0020C => eprintln!("[Firmware Init] ★ firmware_main_loop ENTRY at insn {} (caller blink=0x{:05X})",
                    self.state.instruction_count, self.state.core_regs[REG_BLINK as usize]),
                0x000F8 => eprintln!("[Firmware Init] → firmware_update_check_and_trigger at insn {}", self.state.instruction_count),
                0x084FC => eprintln!("[Firmware Init] → firmware_update_trigger (REBOOT!) at insn {}", self.state.instruction_count),
                0x33CF8 => eprintln!("[Firmware Init] → system_reboot_infinite_loop at insn {}", self.state.instruction_count),
                0x3BB30 => eprintln!("[Firmware Init] → epon_link_init at insn {}", self.state.instruction_count),
                0x01B98 => eprintln!("[Firmware Init] → serdes_hw_init_lanes_and_dma at insn {}", self.state.instruction_count),
                0x16138 => eprintln!("[Firmware Init] → cli_uart_init at insn {}", self.state.instruction_count),
                0x3C224 => eprintln!("[Firmware Init] → serdes_config_fds_init at insn {}", self.state.instruction_count),
                0x128E8 => eprintln!("[Firmware Init] → serdes_load_config_and_reinit at insn {}", self.state.instruction_count),
                0x1366C => eprintln!("[Firmware Init]   → serdes_init_all_lanes_hw at insn {}", self.state.instruction_count),
                0x14AB0 => eprintln!("[Firmware Init]   → mpcp_send_RegisterReq_with_speed at insn {}", self.state.instruction_count),
                0x13F94 => eprintln!("[Firmware Init]   → serdes_lane2_init_pon_rx at insn {}", self.state.instruction_count),
                0x14670 => eprintln!("[Firmware Init]   → serdes_lane0_reinit_rate_change at insn {}", self.state.instruction_count),
                0x046D0 => eprintln!("[Firmware Init] → sfp_serial_bus_read_and_configure at insn {}", self.state.instruction_count),
                0x09834 => eprintln!("[Firmware Init] → epon_runtime_full_init at insn {}", self.state.instruction_count),
                0x02750 => eprintln!("[Firmware Init] → irq_setup_vector_and_enable at insn {}", self.state.instruction_count),
                0x16014 => {
                    static SEEN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
                    if !SEEN.swap(true, std::sync::atomic::Ordering::Relaxed) {
                        eprintln!("[Firmware Init] → cli_poll_and_process_input (MAIN LOOP!) at insn {}", self.state.instruction_count);
                    }
                }
                _ => {}
            }
        }

        // Fetch and decode
        let decoded = decoder::decode(self.state.pc, &self.mem)?;
        let next_pc = self.state.pc + decoded.total_size();

        if self.trace || self.trace_firmware_init {
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
        // (no debug)
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

    /// Boot ROM "start app" function: the bootloader has already staged firmware
    /// code in DCCM (copied from flash). We detect this by checking if DCCM
    /// contains non-zero data at the app staging area, copy it to ICCM,
    /// reconfigure memory bases to 0x20000000, and jump to the entry point.
    /// Returns true if app was loaded, false if not a start_app scenario.
    fn boot_rom_start_app(&mut self) -> bool {
        // The bootloader stages firmware in DCCM at offset 0x32000 (copied from flash).
        // Detect it by matching the first 4 bytes with the firmware IVT signature.
        let staging: usize = 0x32000;
        let firmware_signature: [u8; 4] = [0x21, 0x4A, 0x00, 0x00];
        {
            let b0 = self.mem.read_byte(self.mem.dccm_base + staging as u32).unwrap_or(0);
            let b1 = self.mem.read_byte(self.mem.dccm_base + staging as u32 + 1).unwrap_or(0);
            let b2 = self.mem.read_byte(self.mem.dccm_base + staging as u32 + 2).unwrap_or(0);
            let b3 = self.mem.read_byte(self.mem.dccm_base + staging as u32 + 3).unwrap_or(0);
            if [b0, b1, b2, b3] != firmware_signature {
                return false;
            }
        }

        // Determine app size by matching DCCM staging data against flash sections.
        // Multiple TKF sections share the same IVT signature (0x214A0000), so we
        // compare 64 bytes of DCCM content against each flash section to find the
        // correct one (the one the bootloader actually DMA'd).
        let app_size = {
            let mmio = match self.mem.mmio() {
                Some(m) => m,
                None => return false,
            };
            let flash = &mmio.pbc.flash.data;

            // Read first 64 bytes from DCCM staging area for matching
            let match_len = 64;
            let mut staging_bytes = vec![0u8; match_len];
            for i in 0..match_len {
                staging_bytes[i] = self.mem.read_byte(self.mem.dccm_base + (staging + i) as u32).unwrap_or(0);
            }

            // Search flash for matching content at known TKF offsets
            let mut found_size = 0usize;
            for &header_off in &[0x120000usize, 0x1A0000, 0x270000] {
                let code_off = header_off + 0x27;
                if code_off + match_len > flash.len() { continue; }
                if flash[code_off..code_off + match_len] == staging_bytes[..] {
                    // Found matching section — determine size by scanning for erased block
                    let max_size = flash.len() - code_off;
                    let mut size = 0;
                    while size < max_size {
                        let block_end = (size + 256).min(max_size);
                        if flash[code_off + size..code_off + block_end].iter().all(|&b| b == 0xFF) {
                            break;
                        }
                        size += 256;
                    }
                    found_size = size;
                    eprintln!("[Boot ROM] Matched flash section at 0x{:06X}, size {} bytes", header_off, size);
                    break;
                }
            }
            found_size
        };

        if app_size == 0 {
            eprintln!("[Boot ROM] Firmware signature found in DCCM but not in flash");
            return false;
        }

        eprintln!(
            "[Boot ROM] Firmware detected in DCCM at 0x{:05X}, {} bytes from flash",
            staging, app_size
        );

        // Copy firmware from DCCM staging area to a buffer
        let mut app_code = vec![0u8; app_size];
        for i in 0..app_size {
            app_code[i] = self.mem.read_byte(self.mem.dccm_base + (staging + i) as u32).unwrap_or(0);
        }

        eprintln!(
            "[Boot ROM] Loading firmware: DCCM 0x{:05X}, {} bytes (0x{:X})",
            staging, app_size, app_size
        );

        // Load firmware into ICCM (overwriting bootloader)
        self.mem.load_iccm(0, &app_code);
        self.mem.app_size = Some(app_size);

        // If firmware was staged at a non-zero offset, also copy it to DCCM at offset 0
        if staging != 0 {
            self.mem.load_binary(0, &app_code);
        }

        // Fill remaining ICCM with J_S [blink] (0x7EE0)
        let fill_start = (app_size + 1) & !1;
        if fill_start < ICCM_SIZE {
            let mut fill = vec![0u8; ICCM_SIZE - fill_start];
            for chunk in fill.chunks_exact_mut(2) {
                chunk[0] = 0x7E;
                chunk[1] = 0xE0;
            }
            self.mem.load_iccm(fill_start as u32, &fill);
        }

        // Firmware runs with ICCM/DCCM at base 0 (same as bootloader).
        // The Ghidra base 0x20000000 is just an analysis offset.
        // The flash binary uses absolute addresses in the 0x00000000 range.
        self.mem.iccm_base = 0;
        self.mem.dccm_base = 0;

        // Protect PCL-relative literal pool constants from event_table_clear corruption.
        // See memory.rs for detailed explanation.
        self.mem.protect_firmware_literals();

        // Reset CPU state for firmware
        self.state = CpuState::new();
        self.state.core_regs[28] = 0x10800; // SP = top of stack
        self.state.aux_ienable = 0xFFFFFFFF;
        self.state.flag_e1 = true;
        self.state.flag_e2 = true;

        // Jump to firmware entry point (address 0 = first instruction in ICCM)
        self.state.pc = 0;
        eprintln!(
            "[Boot ROM] ICCM/DCCM base=0, firmware {} bytes, entry=0x00000000",
            app_size
        );

        true
    }

    /// Boot ROM function intercepts for firmware.
    ///
    /// The BCM55030 mask ROM provides C runtime and hardware init functions that
    /// firmware's startup stub calls via JL with LIMM operands. These 4 functions
    /// were identified by searching the firmware binary for all JL instructions
    /// targeting addresses >= 0x4E000 (beyond the 319KB firmware code).
    ///
    /// On real hardware, these live in the mask ROM burned into silicon.
    /// The boot ROM is not readable from the data bus (Harvard architecture:
    /// ICCM = instruction bus only, mem/rf reads the data bus).
    ///
    /// Note: the addresses below are the POST-RELOCATION values. The bootloader
    /// patches the LIMM operands during app loading (flash values differ from
    /// runtime values, e.g. flash 0x79450 → runtime 0x79190).
    ///
    /// Returns true if the PC matched a boot ROM address and was handled.
    fn boot_rom_intercept(&mut self) -> bool {
        match self.state.pc {
            // boot_rom_hw_init — early hardware initialization
            // Called from app startup at PC=0x68, before SP/GP are set (all regs = 0).
            // Pre-relocation LIMM: 0x79450 (Firmware), 0x79190 (Diag)
            // Post-relocation: 0x79190 (if bootloader LIMM patching ran)
            // On real hardware: PLL, clocks, pin mux, SerDes lanes, memory controller.
            // Safe to stub as no-op: the emulator doesn't need PLL/clock configuration.
            0x79190 | 0x79450 => {
                eprintln!("[Boot ROM] 0x{:05X}: boot_rom_hw_init — early HW init (stub, return to 0x{:05X})",
                    self.state.pc, self.state.core_regs[REG_BLINK as usize]);
                self.state.pc = self.state.core_regs[REG_BLINK as usize];
                true
            }

            // boot_rom_crt_main (CRITICAL — must NOT return)
            // Called from app startup at PC=0x90, after SP=0x32000, GP=0x7E400, FP=0.
            // Pre-relocation LIMM: 0x74E24 (Firmware), 0x74B60 (Diag)
            // Post-relocation: 0x74B60 (if bootloader LIMM patching ran)
            // On real hardware: C runtime init (.data copy, .bss clear, heap, call main).
            // .data is already in DCCM (loaded by boot_rom_start_app).
            // We clear .bss (from end of binary to end of DCCM) and jump to main.
            0x74B60 | 0x74E24 => {
                // Clear .bss: from end of loaded binary to end of DCCM
                let bss_start = self.mem.app_size.unwrap_or(0) as u32;
                let bss_end = DCCM_SIZE as u32;
                if bss_start < bss_end {
                    let zeros = vec![0u8; (bss_end - bss_start) as usize];
                    self.mem.load_binary(bss_start, &zeros);
                    eprintln!(
                        "[Boot ROM] 0x{:05X}: boot_rom_crt_main — BSS cleared 0x{:X}-0x{:X} ({} bytes), jumping to firmware_main_loop (0x20C)",
                        self.state.pc, bss_start, bss_end, bss_end - bss_start
                    );
                } else {
                    eprintln!("[Boot ROM] 0x{:05X}: boot_rom_crt_main — jumping to firmware_main_loop (0x20C)", self.state.pc);
                }
                self.state.pc = 0x20C; // firmware_main_loop entry point

                // Set hardware ready flags that the real boot ROM hw_init would set.
                // These are checked by firmware init loops (serdes_hw_ready_flag at 0x1E6C).
                // DCCM 0x7E207 = SerDes hardware ready flag (byte, non-zero = ready)
                let _ = self.mem.write_byte(0x7E207, 1);

                true
            }

            // 0x78F54 — boot_rom_exception_handler_1
            // Called from exception wrapper at PC=0xB4 (push blink, JL, pop blink, ret).
            // On real hardware: handles a specific CPU exception type (likely memory
            // error or machine check). The wrapper at 0xB4 is a default handler that
            // the boot ROM installs in the IVT before the app starts.
            // Firmware replaces exception vectors via hw_auxreg_init_exception_vectors()
            // with its own handlers at 0x33BD0/0x33BD4 during init, so this handler
            // is only active during the brief startup period.
            // Safe to stub as no-op: no exceptions should occur during startup.
            // Pre-relocation: 0x79214 (Firmware), 0x78F54 (Diag)
            0x78F54 | 0x79214 => {
                eprintln!("[Boot ROM] 0x{:05X}: boot_rom_exception_handler_1 (stub, return to 0x{:05X})",
                    self.state.pc, self.state.core_regs[REG_BLINK as usize]);
                self.state.pc = self.state.core_regs[REG_BLINK as usize];
                true
            }

            // boot_rom_exception_handler_2
            // Called from exception wrapper at PC=0xC8 (same pattern as 0xB4).
            // On real hardware: handles a second CPU exception type (likely privilege
            // violation or instruction error). Same lifecycle as handler_1 — active
            // only during startup, replaced by firmware's own handlers at init.
            // Safe to stub as no-op.
            // Pre-relocation: 0x79238 (Firmware), 0x78F78 (Diag)
            0x78F78 | 0x79238 => {
                eprintln!("[Boot ROM] 0x{:05X}: boot_rom_exception_handler_2 (stub, return to 0x{:05X})",
                    self.state.pc, self.state.core_regs[REG_BLINK as usize]);
                self.state.pc = self.state.core_regs[REG_BLINK as usize];
                true
            }

            _ => false,
        }
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
    /// Throttled by UART_PRESCALER to simulate baud-rate limited TX.
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
