//! BCM55030 MACsec cryptographic engine.
//!
//! Claims:
//!
//!   * `0x01002A00..0x01002D00` — the tail of the twin-mirrored SA
//!     programming window. The claim used to start at `0x01002400` and
//!     run to `0x01002D40`, which was three quarters fiction: the SerDes
//!     lane window is tested first on every dispatch chain and takes
//!     everything below `0x01002A00`, and the block at `0x01002D00` is
//!     the UNI-side Ethernet port (`soc::uni`). Only this slice was ever
//!     reached, so only this slice is claimed — and the tests that
//!     asserted on the unreachable part were passing for the wrong
//!     reason, and stamping a MACsec name on a UNI register besides.
//!   * `0x01003500..0x01003540` — MACsec key table tail (block
//!     3501 per `hwregs`).
//!
//! The peripheral is a register file with command-bit auto-clear for
//! bits `27..31`, scoped to the MACsec claim set — which is where the
//! firmware actually polls on busy bits (key engine, PN threshold).
//!
//! v1 is state-machine level only: the key engine / PN threshold
//! busy bits clear after one tick so the firmware polling loop
//! advances. There is no AES-GCM, no nonce sequence verification,
//! and no PN rollover logic — SA programming writes land in the
//! backing store untouched. The UI can force a PN overflow via
//! [`MacsecEvent::InjectPnOverflow`] and reset the SA table via
//! [`MacsecEvent::ResetSaTable`].

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, MacsecEvent, MacsecSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

pub const MACSEC_SA_BASE: u32 = 0x0100_2A00;
pub const MACSEC_SA_END: u32 = 0x0100_2D00;

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
///   * `0x01002A44` — MPCP HW command engine, second bank (`+ 0x400`).
///     Its busy-wait has no timeout, so the bit must always clear.
///
/// The three command registers below `0x01002A00` (`0x0100240C`,
/// `0x0100280C`, `0x01002644`) are served by
/// `serdes::is_cmd_bit_register`; the claim no longer reaches down
/// there at all.
///
/// Identified by tracing each busy-wait loop after a naive "clear bits
/// [31:27] on every write" broke UART interactivity via false-positive
/// state-change detections in the hardware state poller.
const CMD_REGS: &[u32] = &[0x0100_2A44];

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

    fn apply_silicon_power_on(&mut self) {
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

    // The split word at `0x01002804` is not handled here any more: it
    // sits below `0x01002A00`, so this peripheral was never reached at
    // it. The one servant is `serdes::REG_ZERO_READ_WORKAROUND`, where
    // the two names it used to carry are retracted along with it.
    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
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
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        // Silicon power-on snapshot already applied in `reset_cold`.
        self.reset_cold();
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
            // The mirrored offsets of the second bank: the first bank
            // is served by the SerDes lane window, so `+0x400` is the
            // only copy this peripheral can show.
            control: self
                .sa_store
                .get(Self::sa_idx(0x0100_2A68))
                .copied()
                .unwrap_or(0),
            enable_mode: self
                .sa_store
                .get(Self::sa_idx(0x0100_2A20))
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

    /// Rebuilt on `0x01002A80`. The old form used `0x01002480`, an
    /// address the bank never routes here — it passed while proving
    /// nothing about the peripheral under test.
    #[test]
    fn sa_config_register_is_plain_backing_store() {
        let mut m = Macsec::new();
        m.write_word(0x0100_2A80, 0x8000_00AA).unwrap();
        // SA config registers do NOT use the command-bit protocol.
        // Subsequent reads return the stored value unchanged.
        assert_eq!(m.read_word(0x0100_2A80).unwrap(), 0x8000_00AA);
        assert_eq!(m.read_word(0x0100_2A80).unwrap(), 0x8000_00AA);
    }

    /// `0x01002A44` is the only command register this peripheral is ever
    /// asked to serve: the addresses below `0x01002A00` are taken by the
    /// SerDes lane window, which every dispatch chain tests first.
    #[test]
    fn command_register_clears_bit31_on_read() {
        let mut m = Macsec::new();
        m.write_word(0x0100_2A44, 0x8000_1234).unwrap();
        assert_eq!(m.read_word(0x0100_2A44).unwrap(), 0x8000_1234);
        // Second read: bit 31 cleared (operation complete).
        assert_eq!(m.read_word(0x0100_2A44).unwrap(), 0x0000_1234);
    }

    #[test]
    fn key_tail_range_roundtrips() {
        let mut m = Macsec::new();
        m.write_word(0x0100_3520, 0xDEAD_BEEF).unwrap();
        assert_eq!(m.read_word(0x0100_3520).unwrap(), 0xDEAD_BEEF);
    }

    /// The reset table names no address inside the slice this
    /// peripheral serves, so a warm start leaves it zero. The old form
    /// of this test asserted on `0x2400`, `0x2800` and `0x2D18` — three
    /// addresses the bank routes to the SerDes and to the UNI port —
    /// and so put a MACsec name on a UNI register while passing.
    #[test]
    fn the_warm_seed_names_nothing_inside_the_slice_this_peripheral_serves() {
        let mut m = Macsec::new();
        m.reset_warm();
        for &(off, _) in crate::soc::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            assert!(
                !(MACSEC_SA_BASE..MACSEC_SA_END).contains(&abs),
                "{abs:#010X} would seed this peripheral; update the test, not the claim"
            );
        }
        assert_eq!(m.read_word(0x0100_2A44).unwrap(), 0);
        // The key tail is seeded, and that part is unchanged.
        assert_eq!(m.read_word(0x0100_3500).unwrap(), 0x0001_0000);
    }

    /// The window this peripheral never served is no longer claimed:
    /// everything below `0x01002A00` goes to the SerDes lane window and
    /// the classifier ports, and `0x01002D00` upward is the UNI port.
    #[test]
    fn macsec_no_longer_claims_the_window_it_never_served() {
        let m = Macsec::new();
        assert!(!m.claims(0x0100_2400));
        assert!(!m.claims(0x0100_2480));
        assert!(!m.claims(0x0100_2800));
        assert!(!m.claims(0x0100_2D18));
        assert!(!m.claims(0x0100_2D3C));
        // What it does serve.
        assert!(m.claims(0x0100_2A00));
        assert!(m.claims(0x0100_2A44));
        assert!(m.claims(0x0100_2CFC));
        assert!(!m.claims(0x0100_2D00));
        assert!(m.claims(0x0100_3500));
        assert!(!m.claims(0x0100_3540));
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
