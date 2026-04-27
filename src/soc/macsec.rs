//! BCM55030 MACsec cryptographic engine — Session 4.
//!
//! Claims the MACsec 1G and 10G Security Association programming
//! windows plus the key table tail:
//!
//!   * `0x01002400..0x01002D40` — MACsec 1G + 10G SA programming,
//!     key engine, PN threshold engine, channel enable/reset, LLID
//!     IRQ bitmap. Register layout is twin-mirrored: the 1G bank
//!     starts at `0x2400`, the 10G bank at `0x2800`. Every offset
//!     inside a bank follows the same structure (SA config,
//!     nonce, key words, trigger).
//!   * `0x01003500..0x01003540` — MACsec key table tail (block
//!     3501 per `hwregs`).
//!
//! The peripheral is a register file with command-bit auto-clear
//! for bits `27..31` — the same mechanism the old `sysreg_shim`
//! applied as a generic rule (audit 5.8) is now scoped to the
//! MACsec claim set, which is where the firmware actually polls
//! on busy bits (key engine, PN threshold).
//!
//! v1 is state-machine level only: the key engine / PN threshold
//! busy bits clear after one tick so the firmware polling loop
//! advances. There is no AES-GCM, no nonce sequence verification,
//! and no PN rollover logic — SA programming writes land in the
//! backing store untouched. The UI can force a PN overflow via
//! [`MacsecEvent::InjectPnOverflow`] and reset the SA table via
//! [`MacsecEvent::ResetSaTable`].
//!
//! Audit items resolved:
//!
//!   * **5.8 (part)** — the generic "bits 27-31 auto-clear" in the
//!     sysreg_shim is narrowed to this peripheral's claim set.
//!   * **5.12 (part)** — MACsec's slice of `SYSREG_INIT_VALUES`
//!     is now loaded by `reset_warm()`.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, MacsecEvent, MacsecSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

pub const MACSEC_SA_BASE: u32 = 0x0100_2400;
pub const MACSEC_SA_END: u32 = 0x0100_2D40;

pub const MACSEC_KEY_TAIL_BASE: u32 = 0x0100_3500;
pub const MACSEC_KEY_TAIL_END: u32 = 0x0100_3540;

/// Bit 31 is the command trigger on the BCM55030 MDIO / key engine
/// busy registers. Software writes a value with bit 31 set to start
/// an operation and polls the same register for that bit to clear.
/// Applied only to the specific offsets listed in [`CMD_REGS`] —
/// generic SA config registers in the same range are plain storage.
const CMD_BIT: u32 = 0x8000_0000;

/// Concrete registers inside the MACsec claim range that use the
/// bit-31 command / busy protocol. Any other offset is a plain
/// backing store.
///
///   * `0x0100240C` — MDIO PHY command register, 1 Gb bank
///     (`DAT_ram_2003550c + 0x34`, `phy_mdio_rw_op`).
///   * `0x0100280C` — same, 10 Gb bank (`+ 0x400`).
///   * `0x01002644` — MPCP HW Command Engine slot 0
///     (`DAT_ram_20035e4c`, `mpcp_slot_hw_dispatch_command_and_wait`).
///   * `0x01002A44` — MPCP HW Command Engine slot 1 (`+ 0x400`).
///
/// Identified by tracing each busy-wait loop in Ghidra after a
/// naive "clear bits [31:27] on every write" broke UART interactivity
/// via false-positive state-change detections in
/// `epon_poll_hw_state_changes`. Documented in the design notes as the D8
/// investigation.
const CMD_REGS: &[u32] = &[0x0100_240C, 0x0100_280C, 0x0100_2644, 0x0100_2A44];

const MACSEC_SA_RANGES: &[AddressRange] = &[
    AddressRange::new(MACSEC_SA_BASE, MACSEC_SA_END),
    AddressRange::new(MACSEC_KEY_TAIL_BASE, MACSEC_KEY_TAIL_END),
];

#[derive(Clone)]
pub struct Macsec {
    sa_store: Vec<u32>,
    sa_pending_clear: Vec<u32>,

    key_tail_store: Vec<u32>,
    key_tail_pending_clear: Vec<u32>,

    /// Bit `n` is set when SA slot `n` has a forced PN overflow.
    pn_overflow_mask: u32,

    /// Software-written fatal error mask (`FATAL_ERROR_MASK` at
    /// `0x01002804`). Kept separately so the backing store does
    /// not alias the read-only `FATAL_ERROR_STATUS` at the same
    /// address.
    fatal_error_mask: u32,

    pub trace: bool,
}

impl Macsec {
    pub fn new() -> Self {
        let sa_words = ((MACSEC_SA_END - MACSEC_SA_BASE) / 4) as usize;
        let tail_words = ((MACSEC_KEY_TAIL_END - MACSEC_KEY_TAIL_BASE) / 4) as usize;
        Self {
            sa_store: vec![0u32; sa_words],
            sa_pending_clear: vec![0u32; sa_words],
            key_tail_store: vec![0u32; tail_words],
            key_tail_pending_clear: vec![0u32; tail_words],
            pn_overflow_mask: 0,
            fatal_error_mask: 0,
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (MACSEC_SA_BASE..MACSEC_SA_END).contains(&addr)
            || (MACSEC_KEY_TAIL_BASE..MACSEC_KEY_TAIL_END).contains(&addr)
    }

    fn sa_idx(addr: u32) -> usize {
        ((addr - MACSEC_SA_BASE) / 4) as usize
    }

    fn key_tail_idx(addr: u32) -> usize {
        ((addr - MACSEC_KEY_TAIL_BASE) / 4) as usize
    }

    fn read_slot(
        store: &mut [u32],
        pending: &mut [u32],
        idx: usize,
    ) -> u32 {
        let val = store[idx];
        let clear_mask = pending[idx];
        if clear_mask != 0 {
            store[idx] = val & !clear_mask;
            pending[idx] = 0;
        }
        val
    }

    fn write_slot_sparse(
        store: &mut [u32],
        pending: &mut [u32],
        idx: usize,
        abs_addr: u32,
        val: u32,
    ) {
        store[idx] = val;
        if CMD_REGS.contains(&abs_addr) && (val & CMD_BIT) != 0 {
            pending[idx] = CMD_BIT;
        }
    }

    fn apply_warm_snapshot(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if (MACSEC_SA_BASE..MACSEC_SA_END).contains(&abs) {
                let idx = Self::sa_idx(abs);
                self.sa_store[idx] = val;
            } else if (MACSEC_KEY_TAIL_BASE..MACSEC_KEY_TAIL_END).contains(&abs) {
                let idx = Self::key_tail_idx(abs);
                self.key_tail_store[idx] = val;
            }
        }
    }
}

impl Peripheral for Macsec {
    fn name(&self) -> &'static str {
        "macsec"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        MACSEC_SA_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        // Write-one-read-another aliases at 0x01002804:
        //   * Write path → block 5602 FATAL_ERROR_MASK (software sets
        //     the fatal-error bit mask, typically 0x105C)
        //   * Read path  → block 5601 FATAL_ERROR_STATUS, which on
        //     silicon returns 0 when no fatal condition is latched
        // A shared backing store would let the MASK write poison
        // the STATUS read, which is exactly what Session 8's UART
        // interactivity bug was — `epon_poll_hw_state_changes`
        // saw bits 0x105C flipping and triggered a shutdown loop.
        if addr == 0x0100_2804 {
            return Ok(0);
        }
        if (MACSEC_SA_BASE..MACSEC_SA_END).contains(&addr) {
            let idx = Self::sa_idx(addr);
            let val = Self::read_slot(
                &mut self.sa_store,
                &mut self.sa_pending_clear,
                idx,
            );
            return Ok(val);
        }
        if (MACSEC_KEY_TAIL_BASE..MACSEC_KEY_TAIL_END).contains(&addr) {
            let idx = Self::key_tail_idx(addr);
            return Ok(Self::read_slot(
                &mut self.key_tail_store,
                &mut self.key_tail_pending_clear,
                idx,
            ));
        }
        Ok(0)
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if (MACSEC_SA_BASE..MACSEC_SA_END).contains(&addr) {
            let idx = Self::sa_idx(addr);
            // FATAL_ERROR_MASK write (see the matching read arm
            // above). Stash the value so the UI snapshot can
            // surface it, but do not let it alias the STATUS
            // read. A shared backing-store array would cause
            // `epon_poll_hw_state_changes` to see bits flipping
            // and fire an unexpected shutdown.
            if addr == 0x0100_2804 {
                self.fatal_error_mask = val;
                return Ok(());
            }
            Self::write_slot_sparse(
                &mut self.sa_store,
                &mut self.sa_pending_clear,
                idx,
                addr,
                val,
            );
            return Ok(());
        }
        if (MACSEC_KEY_TAIL_BASE..MACSEC_KEY_TAIL_END).contains(&addr) {
            let idx = Self::key_tail_idx(addr);
            Self::write_slot_sparse(
                &mut self.key_tail_store,
                &mut self.key_tail_pending_clear,
                idx,
                addr,
                val,
            );
            return Ok(());
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.sa_store.fill(0);
        self.sa_pending_clear.fill(0);
        self.key_tail_store.fill(0);
        self.key_tail_pending_clear.fill(0);
        self.pn_overflow_mask = 0;
        self.fatal_error_mask = 0;
    }

    fn reset_warm(&mut self) {
        self.reset_cold();
        self.apply_warm_snapshot();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        // Rough counts — boot populates SA Program Config Word with
        // non-zero values when slots are provisioned; `sa_slots_programmed`
        // counts non-zero SA_TRIGGER-style rows.
        let sa_slots_programmed = self
            .sa_store
            .chunks(0x40)
            .filter(|chunk| chunk.iter().any(|&w| w != 0))
            .count() as u8;
        PeripheralSnapshot::Macsec(MacsecSnapshot {
            control: self.sa_store.get(Self::sa_idx(0x0100_2468).min(self.sa_store.len() - 1))
                .copied()
                .unwrap_or(0),
            enable_mode: self
                .sa_store
                .get(Self::sa_idx(0x0100_2420).min(self.sa_store.len() - 1))
                .copied()
                .unwrap_or(0),
            key_engine_busy: false,
            pn_threshold_busy: false,
            sa_slots_programmed,
            pn_overflow_mask: self.pn_overflow_mask,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Macsec(ev) => match ev {
                MacsecEvent::InjectPnOverflow(slot) => {
                    self.pn_overflow_mask |= 1u32 << (*slot as u32 & 0x1F);
                    Ok(())
                }
                MacsecEvent::ResetSaTable => {
                    self.sa_store.fill(0);
                    self.sa_pending_clear.fill(0);
                    self.pn_overflow_mask = 0;
                    Ok(())
                }
            },
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sa_config_register_is_plain_backing_store() {
        let mut m = Macsec::new();
        m.write_word(0x0100_2480, 0x8000_00AA).unwrap();
        // SA config registers do NOT use the command-bit protocol.
        // Subsequent reads return the stored value unchanged.
        assert_eq!(m.read_word(0x0100_2480).unwrap(), 0x8000_00AA);
        assert_eq!(m.read_word(0x0100_2480).unwrap(), 0x8000_00AA);
    }

    #[test]
    fn mdio_command_register_clears_bit31_on_read() {
        let mut m = Macsec::new();
        // 0x0100240C is the 1 Gb MDIO PHY command register.
        m.write_word(0x0100_240C, 0x8000_1234).unwrap();
        assert_eq!(m.read_word(0x0100_240C).unwrap(), 0x8000_1234);
        // Second read: bit 31 cleared (operation complete).
        assert_eq!(m.read_word(0x0100_240C).unwrap(), 0x0000_1234);
        // Same semantic for the 10 Gb bank.
        m.write_word(0x0100_280C, 0x8000_5678).unwrap();
        assert_eq!(m.read_word(0x0100_280C).unwrap(), 0x8000_5678);
        assert_eq!(m.read_word(0x0100_280C).unwrap(), 0x0000_5678);
    }

    #[test]
    fn key_tail_range_roundtrips() {
        let mut m = Macsec::new();
        m.write_word(0x0100_3520, 0xDEAD_BEEF).unwrap();
        assert_eq!(m.read_word(0x0100_3520).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn warm_seed_loads_macsec_init_values() {
        let mut m = Macsec::new();
        m.reset_warm();
        // mmio_init: 0x2400 = 0x13, 0x2800 = 0x13, 0x2D18 = 0x08130E04
        assert_eq!(m.read_word(0x0100_2400).unwrap() & 0xFF, 0x13);
        assert_eq!(m.read_word(0x0100_2800).unwrap() & 0xFF, 0x13);
        assert_eq!(m.read_word(0x0100_2D18).unwrap(), 0x0813_0E04);
    }

    #[test]
    fn claims_respects_bounds() {
        let m = Macsec::new();
        assert!(m.claims(0x0100_2400));
        assert!(m.claims(0x0100_2D3C));
        assert!(!m.claims(0x0100_2D40));
        assert!(m.claims(0x0100_3500));
        assert!(!m.claims(0x0100_3540));
        assert!(!m.claims(0x0100_23FC));
    }

    #[test]
    fn pn_overflow_event_sets_mask() {
        let mut m = Macsec::new();
        m.inject_event(&PeripheralEvent::Macsec(MacsecEvent::InjectPnOverflow(3)))
            .unwrap();
        assert_eq!(m.pn_overflow_mask, 1 << 3);
        m.inject_event(&PeripheralEvent::Macsec(MacsecEvent::ResetSaTable))
            .unwrap();
        assert_eq!(m.pn_overflow_mask, 0);
    }
}
