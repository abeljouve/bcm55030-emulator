//! Auxiliary registers: `lr`, `sr`, and the architectural behaviour of each
//! address (storage, write masks, read-only and absent addresses, privilege).
//! Platform registers (cache probes and controls) answer through the domain's
//! hooks.

use crate::decoder::instruction::Operand;
use crate::dom::{AuxReg, Dom, Flag};
use crate::exception::Exception;
use crate::regs::*;

use super::regs::{operand, status32, write_dest};

/// Every auxiliary address whose `lr` or `sr` does anything but read IDENTITY
/// and absorb the write, as inclusive ranges. Outside these, an address
/// behaves as absent (checked by a test), so a symbolic `lr`/`sr` with an
/// unknown number can be written exactly as a case split over this list.
pub const DEFINED: &[(u32, u32)] = &[
    (AUX_STATUS, AUX_PC),
    (AUX_STATUS32, AUX_STATUS32_L2),
    (AUX_MULHI, AUX_MULHI),
    (AUX_COUNT0, AUX_LIMIT0),
    (AUX_INT_VECTOR_BASE, AUX_INT_VECTOR_BASE),
    (AUX_MACMODE, AUX_MACMODE),
    (AUX_IRQ_LV12, AUX_IRQ_LV12),
    (0x47, 0x49),
    (0x60, 0x7F),
    (0xC0, 0xFF),
    (AUX_COUNT1, AUX_LIMIT1),
    (AUX_IRQ_LEV, AUX_IRQ_HINT),
    (AUX_ITRIGGER, AUX_ITRIGGER),
    (AUX_XPU, AUX_XPU),
    (AUX_IRQ_PULSE_CANCEL, AUX_IRQ_PENDING),
];

/// `lr dst, [addr]`.
#[inline]
pub fn lr<D: Dom>(d: &mut D, dst: Operand, addr: Operand) -> Result<(), Exception> {
    let a = operand(d, addr)?;
    // The only use of `as_const` in the semantics: dispatch on the register number.
    let v = match d.as_const(a) {
        Some(n) => read(d, n)?,
        None => d.aux_read_dynamic(a)?,
    };
    write_dest(d, dst, v)
}

/// `sr src, [addr]`.
#[inline]
pub fn sr<D: Dom>(d: &mut D, src: Operand, addr: Operand) -> Result<(), Exception> {
    let a = operand(d, addr)?;
    let v = operand(d, src)?;
    match d.as_const(a) {
        Some(n) => {
            write(d, n, v)?;
            d.aux_write_hook(n, v)
        }
        None => d.aux_write_dynamic(a, v),
    }
}

/// Auxiliary register `n`, as `lr` reads it.
pub fn read<D: Dom>(d: &mut D, n: u32) -> Result<D::V, Exception> {
    if let Some(r) = d.aux_read_hook(n) {
        return r;
    }
    let v = match n {
        AUX_STATUS => {
            // Legacy STATUS: Z, N, C, V, E2, E1, H in bits 31..25, PC[25:2] below.
            let mut acc = d.k((d.pc() >> 2) & 0x03FF_FFFF);
            for (f, bit) in [(Flag::Z, 31), (Flag::N, 30), (Flag::C, 29), (Flag::V, 28),
                             (Flag::E2, 27), (Flag::E1, 26), (Flag::H, 25)] {
                let b = d.flag(f);
                let v = d.b2v(b);
                let s = d.k(bit);
                let x = d.shl(v, s);
                acc = d.or(acc, x);
            }
            acc
        }
        AUX_SEMAPHORE => d.aux(AuxReg::Semaphore),
        AUX_LP_START => d.aux(AuxReg::LpStart),
        AUX_LP_END => d.aux(AuxReg::LpEnd),
        AUX_IDENTITY => d.k(IDENTITY_VALUE),
        // Bit 22 is set (measured on silicon). Bit 31, DEBUG.LD, is only
        // driven by a timing model, through the read hook.
        AUX_DEBUG => d.k(0x0040_0000),
        AUX_PC => d.k(d.pc() & !1),
        AUX_STATUS32 => status32(d),
        AUX_STATUS32_L1 => d.aux(AuxReg::Status32L1),
        AUX_STATUS32_L2 => d.aux(AuxReg::Status32L2),
        AUX_COUNT0 => d.aux(AuxReg::Count0),
        AUX_CONTROL0 => d.aux(AuxReg::Control0),
        AUX_LIMIT0 => d.aux(AuxReg::Limit0),
        AUX_INT_VECTOR_BASE => d.aux(AuxReg::IntVectorBase),
        AUX_MULHI => d.aux(AuxReg::Mulhi),
        AUX_MACMODE => d.aux(AuxReg::Macmode),
        AUX_IRQ_LV12 => d.aux(AuxReg::IrqLv12),
        AUX_COUNT1 => d.aux(AuxReg::Count1),
        AUX_CONTROL1 => d.aux(AuxReg::Control1),
        AUX_LIMIT1 => d.aux(AuxReg::Limit1),
        AUX_IRQ_LEV => d.aux(AuxReg::IrqLev),
        AUX_IRQ_HINT => d.aux(AuxReg::IrqHint),
        // ERET..EFA, ICAUSE1/2, AUX_IENABLE and BTA, BTA_L1/L2 are absent on
        // this core: they fall through to IDENTITY below.
        AUX_ITRIGGER => d.aux(AuxReg::Itrigger),
        AUX_XPU => d.aux(AuxReg::Xpu),
        AUX_IRQ_PENDING => d.aux(AuxReg::IrqPending),
        // Build configuration registers (measured on silicon).
        AUX_BCR_VER => d.k(0x02),
        AUX_BTA_LINK_BUILD => d.k(0x00),
        AUX_EA_BUILD => d.k(0x00),
        AUX_VECBASE_AC_BUILD => d.k(0x00),
        AUX_RF_BUILD => d.k(0x01),
        AUX_TIMER_BUILD => d.k(0x0303),
        AUX_DCCM_BUILD => d.k(0x00),
        AUX_ICCM_BUILD => d.k(0x00),
        0x47 => d.k(0x00), // DC_IVDC: write-only command
        0x48 => d.aux(AuxReg::DcCtrl),
        0x49 => d.k(0x00), // CACHE_BYPASS
        AUX_D_CACHE_BUILD => d.k(0x0001_3001),
        AUX_I_CACHE_BUILD => d.k(0x0002_3001),
        AUX_MULTIPLY_BUILD => d.k(0x00),
        AUX_SWAP_BUILD => d.k(0x01),
        AUX_NORM_BUILD => d.k(0x02),
        AUX_MINMAX_BUILD => d.k(0x00),
        AUX_BARREL_BUILD => d.k(0x00),
        0x62 => d.k(0x01),
        0x67 => d.k(0x05),
        0x69 => d.k(0x00FC_0001),
        0x60..=0x7F | 0xC0..=0xFF => d.k(0x00),
        // An address the core does not implement reads IDENTITY, it does not fault.
        _ => d.k(IDENTITY_VALUE),
    };
    Ok(v)
}

/// Write auxiliary register `n` as `sr` does, before the platform hook.
pub fn write<D: Dom>(d: &mut D, n: u32, v: D::V) -> Result<(), Exception> {
    // Privileged in user mode. An absent address is not privileged: its
    // write is absorbed like any other (INFERRED: user mode is never entered
    // in practice; consistency with the absent addresses decides).
    if matches!(
        n,
        AUX_STATUS32 | AUX_STATUS32_L1 | AUX_STATUS32_L2 | AUX_INT_VECTOR_BASE | AUX_ITRIGGER
            | AUX_IRQ_LEV | AUX_IRQ_HINT | AUX_IRQ_PULSE_CANCEL
    ) {
        let u = d.flag(Flag::U);
        d.fault_if(u, Exception::PrivilegeViolation { address: d.pc() })?;
    }
    let masked = |d: &mut D, a: AuxReg, mask: u32| {
        let m = d.k(mask);
        let x = d.and(v, m);
        d.set_aux(a, x);
    };
    // Timer CONTROL: the interrupt-pending bit (3) is read-only.
    let keep_ip = |d: &mut D, a: AuxReg| {
        let old = d.aux(a);
        let ip = d.k(0x08);
        let not_ip = d.k(!0x08);
        let kept = d.and(old, ip);
        let new = d.and(v, not_ip);
        let x = d.or(new, kept);
        d.set_aux(a, x);
    };
    match n {
        AUX_SEMAPHORE => d.set_aux(AuxReg::Semaphore, v),
        AUX_LP_START => masked(d, AuxReg::LpStart, !1),
        AUX_LP_END => masked(d, AuxReg::LpEnd, !1),
        // STATUS32 is written by FLAG, not by `sr`.
        AUX_STATUS32 => return Err(Exception::InstructionError { address: d.pc() }),
        AUX_STATUS32_L1 => d.set_aux(AuxReg::Status32L1, v),
        AUX_STATUS32_L2 => d.set_aux(AuxReg::Status32L2, v),
        AUX_COUNT0 => d.set_aux(AuxReg::Count0, v),
        AUX_CONTROL0 => keep_ip(d, AuxReg::Control0),
        AUX_LIMIT0 => d.set_aux(AuxReg::Limit0, v),
        AUX_INT_VECTOR_BASE => masked(d, AuxReg::IntVectorBase, 0xFFFF_FC00),
        AUX_MULHI => d.set_aux(AuxReg::Mulhi, v),
        AUX_MACMODE => d.set_aux(AuxReg::Macmode, v),
        // Write one to clear.
        AUX_IRQ_LV12 => {
            let old = d.aux(AuxReg::IrqLv12);
            let nv = d.not(v);
            let x = d.and(old, nv);
            d.set_aux(AuxReg::IrqLv12, x);
        }
        AUX_COUNT1 => d.set_aux(AuxReg::Count1, v),
        AUX_CONTROL1 => keep_ip(d, AuxReg::Control1),
        AUX_LIMIT1 => d.set_aux(AuxReg::Limit1, v),
        AUX_IRQ_LEV => d.set_aux(AuxReg::IrqLev, v),
        AUX_IRQ_HINT => masked(d, AuxReg::IrqHint, 0x1F),
        AUX_ITRIGGER => d.set_aux(AuxReg::Itrigger, v),
        AUX_XPU => d.set_aux(AuxReg::Xpu, v),
        AUX_IRQ_PULSE_CANCEL => d.set_aux(AuxReg::IrqPulseCancel, v),
        // DC_CTRL: only bits 0, 1, 2, 5, 6, 7 are writable.
        0x48 => masked(d, AuxReg::DcCtrl, 0xE7),
        // Commands with no stored state (DC_IVDC, IC_IVIC, ...), read-only
        // registers, build configuration and absent addresses: the write is
        // absorbed; side effects belong to the platform hook.
        _ => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flat::FlatMachine;

    fn defined(n: u32) -> bool {
        DEFINED.iter().any(|&(lo, hi)| (lo..=hi).contains(&n))
    }

    #[test]
    fn addresses_outside_the_defined_list_behave_as_absent() {
        let mut addrs: Vec<u32> = (0..0x2000).collect();
        addrs.extend([0x1_0000, 0x7FFF_FFFF, 0x8000_0000, u32::MAX]);
        for user in [false, true] {
            let mut m = FlatMachine::new(0);
            m.pc = 0x1234;
            m.set_flag(Flag::U, user);
            for (i, a) in AuxReg::ALL.iter().enumerate() {
                m.set_aux(*a, 0x1111_0000 + i as u32);
            }
            for &n in addrs.iter().filter(|&&n| !defined(n)) {
                assert_eq!(read(&mut m, n), Ok(IDENTITY_VALUE), "read {n:#x}");
                let before = m.clone();
                assert_eq!(write(&mut m, n, 0xA5A5_A5A5), Ok(()), "write {n:#x}");
                assert_eq!(m, before, "write {n:#x} changed the state");
            }
        }
    }
}
