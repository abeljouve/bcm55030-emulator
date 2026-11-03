#![allow(dead_code, clippy::all)]
use crate::cpu::exception::Exception;
use crate::cpu::registers::*;

pub fn status32(state: &CpuState) -> u32 {
    ((state.flag_h as u32) << S32_H)
        | ((state.flag_e1 as u32) << S32_E1)
        | ((state.flag_e2 as u32) << S32_E2)
        | ((state.flag_a1 as u32) << S32_A1)
        | ((state.flag_a2 as u32) << S32_A2)
        | ((state.flag_ae as u32) << S32_AE)
        | ((state.flag_de as u32) << S32_DE)
        | ((state.flag_u as u32) << S32_U)
        | ((state.flag_v as u32) << S32_V)
        | ((state.flag_c as u32) << S32_C)
        | ((state.flag_n as u32) << S32_N)
        | ((state.flag_z as u32) << S32_Z)
        | ((state.flag_l as u32) << S32_L)
}

pub fn set_status32(state: &mut CpuState, val: u32) {
    state.flag_h = (val >> S32_H) & 1 != 0;
    state.flag_e1 = (val >> S32_E1) & 1 != 0;
    state.flag_e2 = (val >> S32_E2) & 1 != 0;
    state.flag_a1 = (val >> S32_A1) & 1 != 0;
    state.flag_a2 = (val >> S32_A2) & 1 != 0;
    state.flag_ae = (val >> S32_AE) & 1 != 0;
    state.flag_de = (val >> S32_DE) & 1 != 0;
    state.flag_u = (val >> S32_U) & 1 != 0;
    state.flag_v = (val >> S32_V) & 1 != 0;
    state.flag_c = (val >> S32_C) & 1 != 0;
    state.flag_n = (val >> S32_N) & 1 != 0;
    state.flag_z = (val >> S32_Z) & 1 != 0;
    state.flag_l = (val >> S32_L) & 1 != 0;
}

pub fn read_core_reg(state: &CpuState, index: u8) -> Result<u32, Exception> {
    match index {
        0..=28 => Ok(state.core_regs[index as usize]),
        // ILINK1/ILINK2: not accessible in user mode
        29 | 30 => {
            if state.flag_u {
                Err(Exception::PrivilegeViolation { address: state.pc })
            } else {
                Ok(state.core_regs[index as usize])
            }
        }
        31..=60 => Ok(state.core_regs[index as usize]),
        // r61 is reserved
        61 => Err(Exception::InstructionError { address: state.pc }),
        // r62 = LIMM indicator (should be resolved before reaching here)
        62 => Ok(0),
        // r63 = PCL: PC 32-bit aligned (bottom 2 bits = 0)
        63 => Ok(state.pc & 0xFFFFFFFC),
        _ => Err(Exception::InstructionError { address: state.pc }),
    }
}

pub fn write_core_reg(state: &mut CpuState, index: u8, value: u32) -> Result<(), Exception> {
    match index {
        0..=28 => {
            state.core_regs[index as usize] = value;
            Ok(())
        }
        // ILINK1/ILINK2: not writable in user mode
        29 | 30 => {
            if state.flag_u {
                Err(Exception::PrivilegeViolation { address: state.pc })
            } else {
                state.core_regs[index as usize] = value;
                Ok(())
            }
        }
        31..=60 => {
            state.core_regs[index as usize] = value;
            Ok(())
        }
        // r61 reserved
        61 => Err(Exception::InstructionError { address: state.pc }),
        // r62 as destination = discard result
        62 => Ok(()),
        // r63 PCL is read-only
        63 => Err(Exception::InstructionError { address: state.pc }),
        _ => Err(Exception::InstructionError { address: state.pc }),
    }
}

pub fn read_aux_reg(state: &CpuState, addr: u32) -> Result<u32, Exception> {
    match addr {
        AUX_STATUS => {
            // Legacy STATUS: Z,N,C,V,E2,E1,H,R | PC[25:2]
            let flags = ((state.flag_z as u32) << 31)
                | ((state.flag_n as u32) << 30)
                | ((state.flag_c as u32) << 29)
                | ((state.flag_v as u32) << 28)
                | ((state.flag_e2 as u32) << 27)
                | ((state.flag_e1 as u32) << 26)
                | ((state.flag_h as u32) << 25);
            Ok(flags | ((state.pc >> 2) & 0x03FFFFFF))
        }
        AUX_SEMAPHORE => Ok(state.aux_semaphore),
        AUX_LP_START => Ok(state.aux_lp_start),
        AUX_LP_END => Ok(state.aux_lp_end),
        AUX_IDENTITY => Ok(IDENTITY_VALUE),
        // bit 22 set (verified on real HW); bit 31 = DEBUG.LD (Load
        // Pending), driven only by the optional `timing` overlay — false
        // (and thus absent) in the default build.
        AUX_DEBUG => Ok(0x00400000 | ((state.debug_load_pending as u32) << 31)),
        AUX_PC => Ok(state.pc & 0xFFFFFFFE),
        AUX_STATUS32 => Ok(status32(state)),
        AUX_STATUS32_L1 => Ok(state.aux_status32_l1),
        AUX_STATUS32_L2 => Ok(state.aux_status32_l2),
        AUX_COUNT0 => Ok(state.aux_count0),
        AUX_CONTROL0 => Ok(state.aux_control0),
        AUX_LIMIT0 => Ok(state.aux_limit0),
        AUX_INT_VECTOR_BASE => Ok(state.aux_int_vector_base),
        AUX_MULHI => Ok(state.aux_mulhi),
        AUX_MACMODE => Ok(state.aux_macmode),
        AUX_IRQ_LV12 => Ok(state.aux_irq_lv12),
        AUX_COUNT1 => Ok(state.aux_count1),
        AUX_CONTROL1 => Ok(state.aux_control1),
        AUX_LIMIT1 => Ok(state.aux_limit1),
        AUX_IRQ_LEV => Ok(state.aux_irq_lev),
        AUX_IRQ_HINT => Ok(state.aux_irq_hint),
        // ERET..EFA, ICAUSE1/2, AUX_IENABLE and BTA, BTA_L1/L2 are
        // absent: they fall through to the `_` arm.
        AUX_ITRIGGER => Ok(state.aux_itrigger),
        AUX_XPU => Ok(state.aux_xpu),
        AUX_IRQ_PENDING => Ok(state.aux_irq_pending),
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
        0x47 => Ok(0x00),                      // DC_IVDC (write-only command)
        0x48 => Ok(state.aux_dc_ctrl),          // DC_CTRL (shadow, synced to DCache in Memory)
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

pub fn write_aux_reg(state: &mut CpuState, addr: u32, val: u32) -> Result<(), Exception> {
    // User mode cannot write most aux regs
    if state.flag_u {
        match addr {
            // AUX_IENABLE is not listed: an absent address is not a
            // privileged register. A user-mode write to it is absorbed
            // like any other absent aux. -- INFERRED: user mode is never
            // entered in practice, so neither behaviour has been measured;
            // consistency with the other absent addresses is the tie-breaker.
            AUX_STATUS32 | AUX_STATUS32_L1 | AUX_STATUS32_L2 | AUX_INT_VECTOR_BASE
            | AUX_ITRIGGER | AUX_IRQ_LEV | AUX_IRQ_HINT | AUX_IRQ_PULSE_CANCEL => {
                return Err(Exception::PrivilegeViolation { address: state.pc });
            }
            _ => {}
        }
    }
    match addr {
        AUX_SEMAPHORE => {
            state.aux_semaphore = val;
            Ok(())
        }
        AUX_LP_START => {
            state.aux_lp_start = val & 0xFFFFFFFE;
            Ok(())
        }
        AUX_LP_END => {
            state.aux_lp_end = val & 0xFFFFFFFE;
            Ok(())
        }
        AUX_STATUS32 => {
            // STATUS32 is read-only via SR; use FLAG instruction to modify
            Err(Exception::InstructionError { address: state.pc })
        }
        AUX_STATUS32_L1 => {
            state.aux_status32_l1 = val;
            Ok(())
        }
        AUX_STATUS32_L2 => {
            state.aux_status32_l2 = val;
            Ok(())
        }
        AUX_COUNT0 => {
            state.aux_count0 = val;
            Ok(())
        }
        AUX_CONTROL0 => {
            // IP bit (bit 3) is read-only; preserve it
            state.aux_control0 = (val & !0x08) | (state.aux_control0 & 0x08);
            Ok(())
        }
        AUX_LIMIT0 => {
            state.aux_limit0 = val;
            Ok(())
        }
        AUX_INT_VECTOR_BASE => {
            state.aux_int_vector_base = val & 0xFFFFFC00;
            Ok(())
        }
        AUX_MULHI => {
            state.aux_mulhi = val;
            Ok(())
        }
        AUX_MACMODE => {
            state.aux_macmode = val;
            Ok(())
        }
        AUX_IRQ_LV12 => {
            // Sticky clear: write 1 to clear
            state.aux_irq_lv12 &= !val;
            Ok(())
        }
        AUX_COUNT1 => {
            state.aux_count1 = val;
            Ok(())
        }
        AUX_CONTROL1 => {
            // IP bit (bit 3) is read-only; preserve it
            state.aux_control1 = (val & !0x08) | (state.aux_control1 & 0x08);
            Ok(())
        }
        AUX_LIMIT1 => {
            state.aux_limit1 = val;
            Ok(())
        }
        AUX_IRQ_LEV => {
            state.aux_irq_lev = val;
            Ok(())
        }
        AUX_IRQ_HINT => {
            state.aux_irq_hint = val & 0x1F;
            Ok(())
        }
        // ERET..EFA, ICAUSE1/2, AUX_IENABLE and BTA, BTA_L1/L2 are
        // absent: the write is discarded by the `_` arm.
        AUX_ITRIGGER => {
            state.aux_itrigger = val;
            Ok(())
        }
        AUX_XPU => {
            state.aux_xpu = val;
            Ok(())
        }
        AUX_IRQ_PULSE_CANCEL => {
            state.aux_irq_pulse_cancel = val;
            Ok(())
        }
        // Cache control registers
        0x47 => Ok(()),  // DC_IVDC: write handled by executor post-processing
        0x48 => {
            // DC_CTRL: update shadow register. Executor syncs to DCache.
            // RW mask 0xE7 applied per hardware verification.
            // Only bits 0, 1, 2, 5, 6, 7 are writable.
            state.aux_dc_ctrl = val & 0xE7;
            Ok(())
        }
        0x49 => Ok(()),  // DC_LDL / CACHE_BYPASS: ignored for now
        // IC_IVIC (aux 0x10): side effect handled by executor (invalidates
        // entire I-cache on any write). No state stored here.
        0x10 => Ok(()),
        // Read-only registers — writes silently ignored on real HW
        AUX_STATUS | AUX_IDENTITY | AUX_DEBUG | AUX_PC | AUX_IRQ_PENDING => Ok(()),
        // BCR registers are read-only — writes silently ignored
        0x60..=0x7F | 0xC0..=0xFF => Ok(()),
        // Non-existent aux regs: writes silently absorbed on real HW
        _ => Ok(()),
    }
}
