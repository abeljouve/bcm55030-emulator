use arc_isa::decoder::instruction::DelayMode;
use arc_isa::dom::{AuxReg, Flag};

use crate::cpu::exception::Exception;
use crate::cpu::state_dom::StateDom;

pub use arc_isa::regs::*;

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

#[derive(Debug, Clone, PartialEq, Eq)]
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
    pub aux_mulhi: u32,
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

    // Interrupt cause, kept internally (ICAUSE1/2 are not readable here).
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
            aux_mulhi: 0x2,
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

    /// The `CpuState` field holding flag `f`.
    #[inline(always)]
    pub fn flag_mut(&mut self, f: Flag) -> &mut bool {
        match f {
            Flag::Z => &mut self.flag_z,
            Flag::N => &mut self.flag_n,
            Flag::C => &mut self.flag_c,
            Flag::V => &mut self.flag_v,
            Flag::E1 => &mut self.flag_e1,
            Flag::E2 => &mut self.flag_e2,
            Flag::A1 => &mut self.flag_a1,
            Flag::A2 => &mut self.flag_a2,
            Flag::AE => &mut self.flag_ae,
            Flag::DE => &mut self.flag_de,
            Flag::U => &mut self.flag_u,
            Flag::L => &mut self.flag_l,
            Flag::H => &mut self.flag_h,
        }
    }

    /// The `CpuState` field holding auxiliary register `a`.
    #[inline(always)]
    pub fn aux_mut(&mut self, a: AuxReg) -> &mut u32 {
        match a {
            AuxReg::Semaphore => &mut self.aux_semaphore,
            AuxReg::LpStart => &mut self.aux_lp_start,
            AuxReg::LpEnd => &mut self.aux_lp_end,
            AuxReg::Status32L1 => &mut self.aux_status32_l1,
            AuxReg::Status32L2 => &mut self.aux_status32_l2,
            AuxReg::Count0 => &mut self.aux_count0,
            AuxReg::Control0 => &mut self.aux_control0,
            AuxReg::Limit0 => &mut self.aux_limit0,
            AuxReg::IntVectorBase => &mut self.aux_int_vector_base,
            AuxReg::Mulhi => &mut self.aux_mulhi,
            AuxReg::Macmode => &mut self.aux_macmode,
            AuxReg::IrqLv12 => &mut self.aux_irq_lv12,
            AuxReg::Count1 => &mut self.aux_count1,
            AuxReg::Control1 => &mut self.aux_control1,
            AuxReg::Limit1 => &mut self.aux_limit1,
            AuxReg::IrqLev => &mut self.aux_irq_lev,
            AuxReg::IrqHint => &mut self.aux_irq_hint,
            AuxReg::Itrigger => &mut self.aux_itrigger,
            AuxReg::Xpu => &mut self.aux_xpu,
            AuxReg::IrqPulseCancel => &mut self.aux_irq_pulse_cancel,
            AuxReg::IrqPending => &mut self.aux_irq_pending,
            AuxReg::DcCtrl => &mut self.aux_dc_ctrl,
            AuxReg::Bta => &mut self.aux_bta,
            AuxReg::BtaL1 => &mut self.aux_bta_l1,
            AuxReg::BtaL2 => &mut self.aux_bta_l2,
        }
    }

    /// DEBUG as this emulator reads it: bit 22 (measured on silicon), and bit
    /// 31 (DEBUG.LD, load pending), driven only by the optional timing overlay.
    #[inline(always)]
    pub fn debug_aux(&self) -> u32 {
        0x0040_0000 | ((self.debug_load_pending as u32) << 31)
    }

    /// The pending delay slot, consumed.
    #[inline(always)]
    pub fn take_delay(&mut self) -> Option<(u32, bool)> {
        match self.delay_state {
            DelayState::DelaySlot { target, is_link } => {
                self.delay_state = DelayState::None;
                Some((target, is_link))
            }
            DelayState::None => None,
        }
    }

    /// A taken branch: with a delay slot it is left pending, otherwise the PC
    /// moves now and a link writes BLINK.
    #[inline(always)]
    pub fn take_branch(&mut self, target: u32, delay: DelayMode, link: Option<u32>) {
        match delay {
            DelayMode::Delay => {
                self.delay_state = DelayState::DelaySlot { target, is_link: link.is_some() };
            }
            DelayMode::NoDelay => {
                if let Some(ret) = link {
                    self.core_regs[REG_BLINK as usize] = ret;
                    self.link_executed = true;
                }
                self.pc = target;
                self.pc_written = true;
            }
        }
    }

    /// Read an auxiliary register, as `lr` would (without the platform's
    /// cache probes, which need the memory).
    pub fn read_aux_reg(&mut self, addr: u32) -> Result<u32, Exception> {
        arc_isa::sem::aux::read(&mut StateDom(self), addr)
    }

    /// Write an auxiliary register, as `sr` would (without the platform's
    /// cache side effects, which need the memory).
    pub fn write_aux_reg(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        arc_isa::sem::aux::write(&mut StateDom(self), addr, val)
    }
}
