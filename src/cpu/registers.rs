use crate::cpu::exception::Exception;

// Special register indices
pub const REG_GP: u8 = 26;
pub const REG_FP: u8 = 27;
pub const REG_SP: u8 = 28;
pub const REG_ILINK1: u8 = 29;
pub const REG_ILINK2: u8 = 30;
pub const REG_BLINK: u8 = 31;
pub const REG_LP_COUNT: u8 = 60;
pub const REG_RESERVED: u8 = 61;
pub const REG_LIMM: u8 = 62;
pub const REG_PCL: u8 = 63;

// Auxiliary register addresses
pub const AUX_STATUS: u32 = 0x00;
pub const AUX_SEMAPHORE: u32 = 0x01;
pub const AUX_LP_START: u32 = 0x02;
pub const AUX_LP_END: u32 = 0x03;
pub const AUX_IDENTITY: u32 = 0x04;
pub const AUX_DEBUG: u32 = 0x05;
pub const AUX_PC: u32 = 0x06;
pub const AUX_STATUS32: u32 = 0x0A;
pub const AUX_STATUS32_L1: u32 = 0x0B;
pub const AUX_STATUS32_L2: u32 = 0x0C;
pub const AUX_COUNT0: u32 = 0x21;
pub const AUX_CONTROL0: u32 = 0x22;
pub const AUX_LIMIT0: u32 = 0x23;
pub const AUX_INT_VECTOR_BASE: u32 = 0x25;
pub const AUX_MACMODE: u32 = 0x41;
pub const AUX_IRQ_LV12: u32 = 0x43;
pub const AUX_COUNT1: u32 = 0x100;
pub const AUX_CONTROL1: u32 = 0x101;
pub const AUX_LIMIT1: u32 = 0x102;
pub const AUX_IRQ_LEV: u32 = 0x200;
pub const AUX_IRQ_HINT: u32 = 0x201;
pub const AUX_ERET: u32 = 0x400;
pub const AUX_ERBTA: u32 = 0x401;
pub const AUX_ERSTATUS: u32 = 0x402;
pub const AUX_ECR: u32 = 0x403;
pub const AUX_EFA: u32 = 0x404;
pub const AUX_ICAUSE1: u32 = 0x40A;
pub const AUX_ICAUSE2: u32 = 0x40B;
/// `IENABLE` in the generic ARC 700 definition — **not implemented on this
/// integration**. Kept as a named address so the disassembler can label it and
/// so the absence is documented where a reader looks for the register. There is
/// deliberately no `aux_ienable` field and no read/write arm: the address falls
/// through to the generic absent-aux handling (read = `IDENTITY_VALUE`, write
/// discarded). -- OBSERVED, DATASHEET §6.1.
pub const AUX_IENABLE: u32 = 0x40C;
pub const AUX_ITRIGGER: u32 = 0x40D;
pub const AUX_XPU: u32 = 0x410;
pub const AUX_BTA: u32 = 0x412;
pub const AUX_BTA_L1: u32 = 0x413;
pub const AUX_BTA_L2: u32 = 0x414;
pub const AUX_IRQ_PULSE_CANCEL: u32 = 0x415;
pub const AUX_IRQ_PENDING: u32 = 0x416;

/// BCM55030 IDENTITY register value (verified against real hardware).
/// ARCVER=0x24, build=0xB4. Non-existent aux regs also return this value.
pub const IDENTITY_VALUE: u32 = 0x00B40124;

// NOTE: a disassembler label `Auxregs0028` = STATUS32 (AUX 0x0A * 4 = byte
// offset 0x28). There is NO BCM55030-custom AUX at number 0x28.

// BCR addresses
pub const AUX_BCR_VER: u32 = 0x60;
pub const AUX_BTA_LINK_BUILD: u32 = 0x63;
pub const AUX_EA_BUILD: u32 = 0x65;
pub const AUX_VECBASE_AC_BUILD: u32 = 0x68;
pub const AUX_RF_BUILD: u32 = 0x6E;
pub const AUX_TIMER_BUILD: u32 = 0x75;
pub const AUX_DCCM_BUILD: u32 = 0x74;
pub const AUX_ICCM_BUILD: u32 = 0x78;
pub const AUX_MULTIPLY_BUILD: u32 = 0x7B;
pub const AUX_SWAP_BUILD: u32 = 0x7C;
pub const AUX_NORM_BUILD: u32 = 0x7D;
pub const AUX_MINMAX_BUILD: u32 = 0x7E;
pub const AUX_BARREL_BUILD: u32 = 0x7F;
pub const AUX_D_CACHE_BUILD: u32 = 0x72;
pub const AUX_I_CACHE_BUILD: u32 = 0x77;

// STATUS32 bit positions
pub const S32_H: u32 = 0;
pub const S32_E1: u32 = 1;
pub const S32_E2: u32 = 2;
pub const S32_A1: u32 = 3;
pub const S32_A2: u32 = 4;
pub const S32_AE: u32 = 5;
pub const S32_DE: u32 = 6;
pub const S32_U: u32 = 7;
pub const S32_V: u32 = 8;
pub const S32_C: u32 = 9;
pub const S32_N: u32 = 10;
pub const S32_Z: u32 = 11;
pub const S32_L: u32 = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DelayState {
    None,
    /// About to execute a delay slot instruction; target is where we jump after.
    /// is_link: if true, blink must be set to (delay_slot_pc + delay_slot_size)
    /// before executing the delay slot.
    DelaySlot { target: u32, is_link: bool },
}

/// Why the CPU last paused. Populated by the step loop when the
/// `paused` flag is raised and consumed by the UI / MCP worker.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PauseReason {
    None,
    /// PC matched a `Hook::Breakpoint` entry.
    Breakpoint(u32),
    /// Memory access matched a watchpoint entry.
    Watch(u32, crate::memory::WatchMode),
    /// User-initiated pause via `CpuCommand::Pause`.
    UserPause,
    /// `state.halted` was observed (CPU reached SLEEP / FLAG 1 etc.).
    Halted,
    /// The PC stopped moving with NO watchdog armed and no interrupt able to
    /// reach the core: a permanent hang. Carries the spinning PC. The state is
    /// left intact deliberately — this used to be reported as a warm reboot,
    /// which destroyed exactly the SRAM/registers needed to diagnose it.
    SpinNoWatchdog(u32),
    /// The optional timing overlay (`timing` feature) detected a fetch-vs-load
    /// bus-arbitration deadlock: an uncached load starved by a fixed-priority
    /// instruction-fetch miss stream.
    #[cfg(feature = "timing")]
    BusDeadlock,
}

#[derive(Debug, Clone)]
pub struct CpuState {
    // Core registers r0-r59, r60 (LP_COUNT)
    // r61 is reserved, r62 is LIMM indicator, r63 is PCL
    pub core_regs: [u32; 64],

    // Program counter (address of next instruction to fetch)
    pub pc: u32,

    // STATUS32 flags
    pub flag_z: bool,
    pub flag_n: bool,
    pub flag_c: bool,
    pub flag_v: bool,
    pub flag_e1: bool,
    pub flag_e2: bool,
    pub flag_a1: bool,
    pub flag_a2: bool,
    pub flag_ae: bool,
    pub flag_de: bool,
    pub flag_u: bool,  // user mode
    pub flag_l: bool,  // loop disable
    pub flag_h: bool,  // halted

    // Auxiliary registers
    pub aux_lp_start: u32,
    pub aux_lp_end: u32,
    pub aux_semaphore: u32,
    pub aux_status32_l1: u32,
    pub aux_status32_l2: u32,
    pub aux_int_vector_base: u32,
    pub aux_macmode: u32,
    pub aux_irq_lv12: u32,
    pub aux_irq_lev: u32,
    pub aux_irq_hint: u32,

    // Timer 0
    pub aux_count0: u32,
    pub aux_control0: u32,
    pub aux_limit0: u32,

    // Timer 1
    pub aux_count1: u32,
    pub aux_control1: u32,
    pub aux_limit1: u32,

    // Timer 1 IRQ line number. ARC 700 default per Table 22 is 4, but the
    // timer IRQ wiring is SoC-integration-level config — BCM55030 wires
    // Timer 1 to IRQ 7 (see `hw_timer_reset_and_set_auxreg7` which installs
    // the tick-counter handler at vec 7, and AUX_IRQ_LEV bit 7 set in HW).
    pub timer1_irq: u32,

    // Exception registers
    pub aux_eret: u32,
    pub aux_erbta: u32,
    pub aux_erstatus: u32,
    pub aux_ecr: u32,
    pub aux_efa: u32,
    pub aux_icause1: u32,
    pub aux_icause2: u32,
    // No `aux_ienable`: aux 0x40C is unimplemented on the BCM55030 (§6.1).
    // There is no per-line interrupt enable mask on this device; see
    // `AUX_IENABLE` above and `Cpu::check_interrupts`.
    pub aux_itrigger: u32,
    pub aux_xpu: u32,
    pub aux_bta: u32,
    pub aux_bta_l1: u32,
    pub aux_bta_l2: u32,
    pub aux_irq_pulse_cancel: u32,
    pub aux_irq_pending: u32,

    // Delay slot state
    pub delay_state: DelayState,

    // Halted / sleeping
    pub halted: bool,
    pub sleeping: bool,

    /// UI / MCP worker pause flag. Set by `Hook::Breakpoint`, watchpoint
    /// hits, or an explicit `CpuCommand::Pause`. The step loop treats
    /// `paused` like `halted`: it returns early without advancing PC.
    pub paused: bool,

    /// Populated when `paused` transitions to `true`. The UI reads this
    /// to display why the CPU stopped.
    pub pause_reason: PauseReason,

    // Instruction counter
    pub instruction_count: u64,

    /// Set by executor when PC is explicitly written (branch/jump/RTIE).
    /// Used by step() to distinguish "branch to same address" from "no branch".
    pub pc_written: bool,

    /// Set by executor on BL/JL (link=true, no delay) for shadow call stack.
    pub link_executed: bool,

    /// Fast IRQ shadow set for r0..r3. Saved on IRQ entry, restored on RTIE.
    pub irq_shadow_r0_r3: [u32; 4],

    /// DC_CTRL (aux 0x48) shadow: the authoritative copy lives in the DCache
    /// model in Memory, but CpuState needs a copy for read_aux_reg/write_aux_reg.
    /// Synced to DCache via executor post-processing on StoreAux(0x48).
    /// BCM55030 reset value: 0xC2 (cache enabled, IM=1, LM=1, bit1=1).
    pub aux_dc_ctrl: u32,

    /// DEBUG.LD (aux 0x05 bit 31, "Load Pending"): set by the optional
    /// cycle-accurate timing overlay (the `timing` feature) while a load is
    /// outstanding on the modelled bus. Always `false` in the default build —
    /// nothing writes it — so a DEBUG read returns the same value as before.
    pub debug_load_pending: bool,

}

impl CpuState {
    pub fn new() -> Self {
        let mut state = Self {
            core_regs: [0u32; 64],
            pc: 0,
            flag_z: false,
            flag_n: false,
            flag_c: false,
            flag_v: false,
            flag_e1: false,
            flag_e2: false,
            flag_a1: false,
            flag_a2: false,
            flag_ae: false,
            flag_de: false,
            flag_u: false,
            flag_l: false,
            flag_h: false,
            aux_lp_start: 0,
            aux_lp_end: 0,
            aux_semaphore: 0,
            aux_status32_l1: 0,
            aux_status32_l2: 0,
            aux_int_vector_base: 0,
            aux_macmode: 0,
            aux_irq_lv12: 0,
            aux_irq_lev: 0,
            aux_irq_hint: 0,
            aux_count0: 0,
            aux_control0: 0,
            aux_limit0: 0,
            aux_count1: 0,
            aux_control1: 0,
            aux_limit1: 0,
            timer1_irq: 4,
            aux_eret: 0,
            aux_erbta: 0,
            aux_erstatus: 0,
            aux_ecr: 0,
            aux_efa: 0,
            aux_icause1: 0,
            aux_icause2: 0,
            aux_itrigger: 0,
            aux_xpu: 0,
            aux_bta: 0,
            aux_bta_l1: 0,
            aux_bta_l2: 0,
            aux_irq_pulse_cancel: 0,
            aux_irq_pending: 0,
            delay_state: DelayState::None,
            halted: false,
            sleeping: false,
            paused: false,
            pause_reason: PauseReason::None,
            instruction_count: 0,
            pc_written: false,
            link_executed: false,
            irq_shadow_r0_r3: [0u32; 4],
            aux_dc_ctrl: 0xC2, // BCM55030 reset: cache enabled, IM=1, LM=1
            debug_load_pending: false,
        };
        // Set default IDENTITY: ARC 700 v1 (ARCVER = 0x31)
        state.core_regs[REG_PCL as usize] = 0;
        state
    }

    /// Build STATUS32 value from individual flags
    pub fn status32(&self) -> u32 {
        ((self.flag_h as u32) << S32_H)
            | ((self.flag_e1 as u32) << S32_E1)
            | ((self.flag_e2 as u32) << S32_E2)
            | ((self.flag_a1 as u32) << S32_A1)
            | ((self.flag_a2 as u32) << S32_A2)
            | ((self.flag_ae as u32) << S32_AE)
            | ((self.flag_de as u32) << S32_DE)
            | ((self.flag_u as u32) << S32_U)
            | ((self.flag_v as u32) << S32_V)
            | ((self.flag_c as u32) << S32_C)
            | ((self.flag_n as u32) << S32_N)
            | ((self.flag_z as u32) << S32_Z)
            | ((self.flag_l as u32) << S32_L)
    }

    /// Restore STATUS32 from a packed value
    pub fn set_status32(&mut self, val: u32) {
        self.flag_h = (val >> S32_H) & 1 != 0;
        self.flag_e1 = (val >> S32_E1) & 1 != 0;
        self.flag_e2 = (val >> S32_E2) & 1 != 0;
        self.flag_a1 = (val >> S32_A1) & 1 != 0;
        self.flag_a2 = (val >> S32_A2) & 1 != 0;
        self.flag_ae = (val >> S32_AE) & 1 != 0;
        self.flag_de = (val >> S32_DE) & 1 != 0;
        self.flag_u = (val >> S32_U) & 1 != 0;
        self.flag_v = (val >> S32_V) & 1 != 0;
        self.flag_c = (val >> S32_C) & 1 != 0;
        self.flag_n = (val >> S32_N) & 1 != 0;
        self.flag_z = (val >> S32_Z) & 1 != 0;
        self.flag_l = (val >> S32_L) & 1 != 0;
    }

    /// Read a core register with constraint checking
    pub fn read_core_reg(&self, index: u8) -> Result<u32, Exception> {
        match index {
            0..=28 => Ok(self.core_regs[index as usize]),
            // ILINK1/ILINK2: not accessible in user mode
            29 | 30 => {
                if self.flag_u {
                    Err(Exception::PrivilegeViolation { address: self.pc })
                } else {
                    Ok(self.core_regs[index as usize])
                }
            }
            31..=60 => Ok(self.core_regs[index as usize]),
            // r61 is reserved
            61 => Err(Exception::InstructionError { address: self.pc }),
            // r62 = LIMM indicator (should be resolved before reaching here)
            62 => Ok(0),
            // r63 = PCL: PC 32-bit aligned (bottom 2 bits = 0)
            63 => Ok(self.pc & 0xFFFFFFFC),
            _ => Err(Exception::InstructionError { address: self.pc }),
        }
    }

    /// Write a core register with constraint checking
    pub fn write_core_reg(&mut self, index: u8, value: u32) -> Result<(), Exception> {
        match index {
            0..=28 => {
                self.core_regs[index as usize] = value;
                Ok(())
            }
            // ILINK1/ILINK2: not writable in user mode
            29 | 30 => {
                if self.flag_u {
                    Err(Exception::PrivilegeViolation { address: self.pc })
                } else {
                    self.core_regs[index as usize] = value;
                    Ok(())
                }
            }
            31..=60 => {
                self.core_regs[index as usize] = value;
                Ok(())
            }
            // r61 reserved
            61 => Err(Exception::InstructionError { address: self.pc }),
            // r62 as destination = discard result
            62 => Ok(()),
            // r63 PCL is read-only
            63 => Err(Exception::InstructionError { address: self.pc }),
            _ => Err(Exception::InstructionError { address: self.pc }),
        }
    }

    /// Read an auxiliary register
    pub fn read_aux_reg(&self, addr: u32) -> Result<u32, Exception> {
        match addr {
            AUX_STATUS => {
                // Legacy STATUS: Z,N,C,V,E2,E1,H,R | PC[25:2]
                let flags = ((self.flag_z as u32) << 31)
                    | ((self.flag_n as u32) << 30)
                    | ((self.flag_c as u32) << 29)
                    | ((self.flag_v as u32) << 28)
                    | ((self.flag_e2 as u32) << 27)
                    | ((self.flag_e1 as u32) << 26)
                    | ((self.flag_h as u32) << 25);
                Ok(flags | ((self.pc >> 2) & 0x03FFFFFF))
            }
            AUX_SEMAPHORE => Ok(self.aux_semaphore),
            AUX_LP_START => Ok(self.aux_lp_start),
            AUX_LP_END => Ok(self.aux_lp_end),
            AUX_IDENTITY => Ok(IDENTITY_VALUE),
            // bit 22 set (verified on real HW); bit 31 = DEBUG.LD (Load
            // Pending), driven only by the optional `timing` overlay — false
            // (and thus absent) in the default build.
            AUX_DEBUG => Ok(0x00400000 | ((self.debug_load_pending as u32) << 31)),
            AUX_PC => Ok(self.pc & 0xFFFFFFFE),
            AUX_STATUS32 => Ok(self.status32()),
            AUX_STATUS32_L1 => Ok(self.aux_status32_l1),
            AUX_STATUS32_L2 => Ok(self.aux_status32_l2),
            AUX_COUNT0 => Ok(self.aux_count0),
            AUX_CONTROL0 => Ok(self.aux_control0),
            AUX_LIMIT0 => Ok(self.aux_limit0),
            AUX_INT_VECTOR_BASE => Ok(self.aux_int_vector_base),
            AUX_MACMODE => Ok(self.aux_macmode),
            AUX_IRQ_LV12 => Ok(self.aux_irq_lv12),
            AUX_COUNT1 => Ok(self.aux_count1),
            AUX_CONTROL1 => Ok(self.aux_control1),
            AUX_LIMIT1 => Ok(self.aux_limit1),
            AUX_IRQ_LEV => Ok(self.aux_irq_lev),
            AUX_IRQ_HINT => Ok(self.aux_irq_hint),
            AUX_ERET => Ok(self.aux_eret),
            AUX_ERBTA => Ok(self.aux_erbta),
            AUX_ERSTATUS => Ok(self.aux_erstatus),
            AUX_ECR => Ok(self.aux_ecr),
            AUX_EFA => Ok(self.aux_efa),
            AUX_ICAUSE1 => Ok(self.aux_icause1),
            AUX_ICAUSE2 => Ok(self.aux_icause2),
            // AUX_IENABLE (0x40C) intentionally absent — see the `_` arm.
            AUX_ITRIGGER => Ok(self.aux_itrigger),
            AUX_XPU => Ok(self.aux_xpu),
            AUX_BTA => Ok(self.aux_bta),
            AUX_BTA_L1 => Ok(self.aux_bta_l1),
            AUX_BTA_L2 => Ok(self.aux_bta_l2),
            AUX_IRQ_PENDING => Ok(self.aux_irq_pending),
            // BCR registers (read-only build config)
            // All values verified against real BCM55030 hardware.
            AUX_BCR_VER => Ok(0x02),
            AUX_BTA_LINK_BUILD => Ok(0x00),       // absent on real HW
            AUX_EA_BUILD => Ok(0x00),              // absent on real HW
            AUX_VECBASE_AC_BUILD => Ok(0x00),      // absent on real HW
            AUX_RF_BUILD => Ok(0x01),              // register file v1
            AUX_TIMER_BUILD => Ok(0x0303),         // v3, T0+T1
            AUX_DCCM_BUILD => Ok(0x00),            // no DCCM (unified SRAM)
            AUX_ICCM_BUILD => Ok(0x00),            // no ICCM (unified SRAM)
            // Cache control registers. The geometry is in `cache.rs`: D-cache
            // 4 KB / 32 B lines, I-cache 4 KB / 1-way / 128 sets / 32 B lines,
            // the latter measured by execution on silicon (see that file).
            // The labels here used to read "D-cache 4KB 8-way, I-cache 8KB
            // 8-way" — both way counts and the I-cache size were wrong, and
            // contradicted `cache.rs` sitting next to them. The VALUES below
            // were always right; only the comments were not.
            0x47 => Ok(0x00),                      // IC_CTRL
            0x48 => Ok(self.aux_dc_ctrl),          // DC_CTRL (shadow, synced to DCache in Memory)
            0x49 => Ok(0x00),                      // CACHE_BYPASS
            AUX_D_CACHE_BUILD => Ok(0x00013001),   // 4KB D-cache present
            AUX_I_CACHE_BUILD => Ok(0x00023001),   // 4KB I-cache present
            AUX_MULTIPLY_BUILD => Ok(0x00),        // absent on real HW
            AUX_SWAP_BUILD => Ok(0x01),
            AUX_NORM_BUILD => Ok(0x02),
            AUX_MINMAX_BUILD => Ok(0x00),          // absent on real HW
            AUX_BARREL_BUILD => Ok(0x00),          // absent on real HW
            // BCM55030-specific BCRs (verified on real HW)
            0x62 => Ok(0x01),                      // unknown BCM55030 BCR
            0x67 => Ok(0x05),                      // unknown (possibly IRQ config)
            0x69 => Ok(0x00FC0001),                // contains UART address prefix
            // Other BCR registers (0x60-0x7F, 0xC0-0xFF): return 0
            0x60..=0x7F => Ok(0x00),
            0xC0..=0xFF => Ok(0x00),
            // Non-existent aux regs return IDENTITY on real HW (not exception).
            // The emulator uses struct fields for internal interrupt/exception
            // state, so this only affects firmware LR reads of unknown regs.
            _ => Ok(IDENTITY_VALUE),
        }
    }

    /// Write an auxiliary register
    pub fn write_aux_reg(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        // User mode cannot write most aux regs
        if self.flag_u {
            match addr {
                // AUX_IENABLE is not listed: an absent address is not a
                // privileged register. A user-mode write to it is absorbed
                // like any other absent aux. -- INFERRED: user mode is never
                // entered in practice, so neither behaviour has been measured;
                // consistency with the other absent addresses is the tie-breaker.
                AUX_STATUS32 | AUX_STATUS32_L1 | AUX_STATUS32_L2 | AUX_INT_VECTOR_BASE
                | AUX_ITRIGGER | AUX_IRQ_LEV | AUX_IRQ_HINT
                | AUX_ERET | AUX_ERBTA | AUX_ERSTATUS | AUX_EFA | AUX_IRQ_PULSE_CANCEL => {
                    return Err(Exception::PrivilegeViolation { address: self.pc });
                }
                _ => {}
            }
        }
        match addr {
            AUX_SEMAPHORE => {
                self.aux_semaphore = val;
                Ok(())
            }
            AUX_LP_START => {
                self.aux_lp_start = val & 0xFFFFFFFE;
                Ok(())
            }
            AUX_LP_END => {
                self.aux_lp_end = val & 0xFFFFFFFE;
                Ok(())
            }
            AUX_STATUS32 => {
                // STATUS32 is read-only via SR; use FLAG instruction to modify
                Err(Exception::InstructionError { address: self.pc })
            }
            AUX_STATUS32_L1 => {
                self.aux_status32_l1 = val;
                Ok(())
            }
            AUX_STATUS32_L2 => {
                self.aux_status32_l2 = val;
                Ok(())
            }
            AUX_COUNT0 => {
                self.aux_count0 = val;
                Ok(())
            }
            AUX_CONTROL0 => {
                // IP bit (bit 3) is read-only; preserve it
                self.aux_control0 = (val & !0x08) | (self.aux_control0 & 0x08);
                Ok(())
            }
            AUX_LIMIT0 => {
                self.aux_limit0 = val;
                Ok(())
            }
            AUX_INT_VECTOR_BASE => {
                self.aux_int_vector_base = val & 0xFFFFFC00;
                Ok(())
            }
            AUX_MACMODE => {
                self.aux_macmode = val;
                Ok(())
            }
            AUX_IRQ_LV12 => {
                // Sticky clear: write 1 to clear
                self.aux_irq_lv12 &= !val;
                Ok(())
            }
            AUX_COUNT1 => {
                self.aux_count1 = val;
                Ok(())
            }
            AUX_CONTROL1 => {
                // IP bit (bit 3) is read-only; preserve it
                self.aux_control1 = (val & !0x08) | (self.aux_control1 & 0x08);
                Ok(())
            }
            AUX_LIMIT1 => {
                self.aux_limit1 = val;
                Ok(())
            }
            AUX_IRQ_LEV => {
                self.aux_irq_lev = val;
                Ok(())
            }
            AUX_IRQ_HINT => {
                self.aux_irq_hint = val & 0x1F;
                Ok(())
            }
            AUX_ERET => {
                self.aux_eret = val;
                Ok(())
            }
            AUX_ERBTA => {
                self.aux_erbta = val;
                Ok(())
            }
            AUX_ERSTATUS => {
                self.aux_erstatus = val;
                Ok(())
            }
            AUX_EFA => {
                self.aux_efa = val;
                Ok(())
            }
            // AUX_IENABLE (0x40C): unimplemented — the write is discarded by
            // the `_` arm, exactly like any other absent auxiliary address.
            AUX_ITRIGGER => {
                self.aux_itrigger = val;
                Ok(())
            }
            AUX_XPU => {
                self.aux_xpu = val;
                Ok(())
            }
            AUX_BTA_L1 => {
                self.aux_bta_l1 = val;
                Ok(())
            }
            AUX_BTA_L2 => {
                self.aux_bta_l2 = val;
                Ok(())
            }
            AUX_IRQ_PULSE_CANCEL => {
                self.aux_irq_pulse_cancel = val;
                Ok(())
            }
            // Cache control registers
            0x47 => Ok(()),  // DC_IVDC: write handled by executor post-processing
            0x48 => {
                // DC_CTRL: update shadow register. Executor syncs to DCache.
                // RW mask 0xE7 applied per hardware verification.
                // Only bits 0, 1, 2, 5, 6, 7 are writable.
                self.aux_dc_ctrl = val & 0xE7;
                Ok(())
            }
            0x49 => Ok(()),  // DC_LDL / CACHE_BYPASS: ignored for now
            // IC_IVIC (aux 0x10): side effect handled by executor (invalidates
            // entire I-cache on any write). No state stored here.
            0x10 => Ok(()),
            // Read-only registers — writes silently ignored on real HW
            AUX_STATUS | AUX_IDENTITY | AUX_DEBUG | AUX_PC | AUX_ECR | AUX_ICAUSE1
            | AUX_ICAUSE2 | AUX_IRQ_PENDING | AUX_BTA => Ok(()),
            // BCR registers are read-only — writes silently ignored
            0x60..=0x7F | 0xC0..=0xFF => Ok(()),
            // Non-existent aux regs: writes silently absorbed on real HW
            _ => Ok(()),
        }
    }
}
