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

/// Mask covering the command bits that auto-clear on the next read.
/// The firmware issues commands by writing a value with bits 27..31
/// set (busy + cmd trigger) and then polls the same register for
/// those bits to clear. Modelled as a pending-clear shadow store.
const CMD_BIT_MASK: u32 = 0xF800_0000;

const MACSEC_SA_RANGES: &[AddressRange] = &[
    AddressRange::new(MACSEC_SA_BASE, MACSEC_SA_END),
    AddressRange::new(MACSEC_KEY_TAIL_BASE, MACSEC_KEY_TAIL_END),
];

pub struct Macsec {
    sa_store: Vec<u32>,
    sa_pending_clear: Vec<u32>,

    key_tail_store: Vec<u32>,
    key_tail_pending_clear: Vec<u32>,

    /// Bit `n` is set when SA slot `n` has a forced PN overflow.
    pn_overflow_mask: u32,

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

    fn write_slot(
        store: &mut [u32],
        pending: &mut [u32],
        idx: usize,
        val: u32,
    ) {
        store[idx] = val;
        let cmd_bits = val & CMD_BIT_MASK;
        if cmd_bits != 0 {
            pending[idx] = cmd_bits;
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
        if (MACSEC_SA_BASE..MACSEC_SA_END).contains(&addr) {
            let idx = Self::sa_idx(addr);
            return Ok(Self::read_slot(
                &mut self.sa_store,
                &mut self.sa_pending_clear,
                idx,
            ));
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
            Self::write_slot(
                &mut self.sa_store,
                &mut self.sa_pending_clear,
                idx,
                val,
            );
            return Ok(());
        }
        if (MACSEC_KEY_TAIL_BASE..MACSEC_KEY_TAIL_END).contains(&addr) {
            let idx = Self::key_tail_idx(addr);
            Self::write_slot(
                &mut self.key_tail_store,
                &mut self.key_tail_pending_clear,
                idx,
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
    fn sa_write_readback_with_command_bit_autoclear() {
        let mut m = Macsec::new();
        m.write_word(0x0100_2480, 0x8000_00AA).unwrap();
        assert_eq!(m.read_word(0x0100_2480).unwrap(), 0x8000_00AA);
        // After one read, command bits 27..31 drop out.
        assert_eq!(m.read_word(0x0100_2480).unwrap(), 0x0000_00AA);
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
