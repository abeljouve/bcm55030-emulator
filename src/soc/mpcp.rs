//! BCM55030 MPCP / Multi-Point Control Protocol.
//!
//! Carves the MPCP-owned register regions out of the sysreg_shim
//! residual store. The MPCP controller's HW blocks are documented
//! in `hwregs` (blocks 17, 21, 22, 28, 52, 61, 69), but several
//! of them are physically co-located with the EPON MAC, the
//! SerDes, and the MACsec engine — the same 32-bit word in the
//! `0x01000000`–`0x01002000` address window has multiple logical
//! names. For the peripherals that already claim an overlapping
//! range (SerDes for block 22 `0x01000180..`, MACsec for block 61
//! `0x01002400..`), this file does not re-claim — the SerDes /
//! MACsec backing stores serve the MPCP writes correctly.
//!
//! This peripheral claims the remaining MPCP-owned addresses:
//!
//! | Block | Name                     | Range                                  |
//! |:------|:-------------------------|:---------------------------------------|
//! |   69  | Queue-to-Pin Map         | `0x01000120`..`0x01000140`             |
//! |   52  | TX Rate Configuration    | `0x01000320`..`0x01000328`             |
//! |   28  | Slot Table               | `0x01001180`..`0x010011C0`             |
//! | 21+51 | Dir/Lane + BW End / MAC  | `0x01001268`..`0x01001390`             |
//!
//! Block 17 "MPCP Speed Configuration" is an SRAM data structure —
//! not MMIO, not modelled here. Block 22 "MPCP LLID Control" is
//! inside the SerDes lane configuration range; the SerDes peripheral
//! round-trips it correctly. Block 61 "MPCP Slot HW Command Engine"
//! (`0x01002644`, stride `0x400`) is inside the MACsec SA programming
//! range and is served by the MACsec command-bit shadow mechanism.
//!
//! Most of it is a plain backing store — writes land in an internal
//! `Vec<u32>` keyed by the claim region, reads round-trip the
//! stored value. Warm snapshot pre-seeds from `mmio_init.rs`.
//!
//! ## The indirect window at `0x010012C0`
//!
//! Block 21 is *not* flat. The word at `+0x54` (`0x010012BC`) selects
//! which table the window above it addresses, and the window is shared:
//! `hwregs` already lists two different registers at the same offset
//! `+0x58` — a per-lane bandwidth window and the per-LLID MAC table.
//!
//! -- OBSERVED that software arms `+0x54` immediately before every
//! access to the window, and that the value it arms distinguishes the
//! tables: the MAC path writes `0`, both for the store and for the
//! read-back, while the bandwidth path writes `1` or `3`. Modelled flat,
//! the second write lands on the first one's word, and the MAC read-back
//! returns a bandwidth descriptor.
//!
//! -- OBSERVED that silicon does not alias them: with the window
//! modelled flat the source address of every upstream MPCP frame carries
//! a bandwidth descriptor where its low four bytes belong, which no peer
//! would answer — and real hardware running this firmware does register.
//! The cold-boot register snapshot in `mmio_init` corroborates the shape
//! from the other side: it captured `+0x54 = 3` together with the
//! window word `0x00060000`, i.e. a bandwidth descriptor read while the
//! bandwidth table was armed.
//!
//! -- INFERRED, and falsifiable: that the selector banks the *whole*
//! window rather than some sub-range of it, and that it banks by the low
//! two bits. Only `0`, `1` and `3` are ever armed by this firmware.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, Peripheral, PeripheralSnapshot,
};

const REGION_COUNT: usize = 4;

#[derive(Clone, Copy)]
struct Region {
    start: u32,
    end: u32,
}

const REGIONS: [Region; REGION_COUNT] = [
    Region { start: 0x0100_0120, end: 0x0100_0140 }, // block 69 queue pin
    Region { start: 0x0100_0320, end: 0x0100_0328 }, // block 52 TX rate
    Region { start: 0x0100_1180, end: 0x0100_11C0 }, // block 28 slot table
    Region { start: 0x0100_1268, end: 0x0100_1390 }, // blocks 21 + 51
];

const MPCP_RANGES: &[AddressRange] = &[
    AddressRange::new(0x0100_0120, 0x0100_0140),
    AddressRange::new(0x0100_0320, 0x0100_0328),
    AddressRange::new(0x0100_1180, 0x0100_11C0),
    AddressRange::new(0x0100_1268, 0x0100_1390),
];

/// Word that selects which table the window above it addresses.
const TABLE_SELECT: u32 = 0x0100_12BC;
/// First word of the banked window; it runs to the end of region 3.
const WINDOW_START: u32 = 0x0100_12C0;
/// Banks the selector can name. Software arms `0`, `1` and `3`.
const BANK_COUNT: usize = 4;

#[derive(Clone)]
pub struct Mpcp {
    stores: [Vec<u32>; REGION_COUNT],
    /// The shared window, once per bank the selector can name.
    banks: [Vec<u32>; BANK_COUNT],
    /// Table currently armed by [`TABLE_SELECT`].
    bank: usize,
    pub trace: bool,
}

impl Mpcp {
    pub fn new() -> Self {
        let stores = std::array::from_fn(|i| {
            let r = REGIONS[i];
            vec![0u32; ((r.end - r.start) / 4) as usize]
        });
        let window_words = ((REGIONS[3].end - WINDOW_START) / 4) as usize;
        let banks = std::array::from_fn(|_| vec![0u32; window_words]);
        Self { stores, banks, bank: 0, trace: false }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        let word = addr & !0x3;
        REGIONS.iter().any(|r| (r.start..r.end).contains(&word))
    }

    /// Index into the banked window, for addresses that live in it.
    fn window_index(addr: u32) -> Option<usize> {
        let word = addr & !0x3;
        (WINDOW_START..REGIONS[3].end)
            .contains(&word)
            .then(|| ((word - WINDOW_START) / 4) as usize)
    }

    fn locate(&self, addr: u32) -> Option<(usize, usize)> {
        let word = addr & !0x3;
        for (i, r) in REGIONS.iter().enumerate() {
            if (r.start..r.end).contains(&word) {
                let idx = ((word - r.start) / 4) as usize;
                return Some((i, idx));
            }
        }
        None
    }

    /// Drive a value into the block-52 TX-rate store at `addr` without
    /// going through the normal write path. Used by the bank (OLT-gated)
    /// to mirror the OLT model's HW-captured timestamp into `0x01000320`
    /// — the registration-independent RX-decode proof the firmware polls.
    /// No-op when `addr` is outside the claimed regions, so a disabled
    /// OLT never reaches this.
    pub fn poke_tx_rate(&mut self, addr: u32, val: u32) {
        if let Some(idx) = Self::window_index(addr) {
            self.banks[self.bank][idx] = val;
        } else if let Some((region, idx)) = self.locate(addr) {
            self.stores[region][idx] = val;
        }
    }

    /// Read-only peek of the block-52 TX-rate store (for snapshot/peek).
    pub fn peek_tx_rate(&self, addr: u32) -> u32 {
        if let Some(idx) = Self::window_index(addr) {
            return self.banks[self.bank][idx];
        }
        match self.locate(addr) {
            Some((region, idx)) => self.stores[region][idx],
            None => 0,
        }
    }

    /// Seed from the cold-boot register snapshot.
    ///
    /// The selector goes in first: the snapshot was taken with one table
    /// armed, so its window words describe *that* table and belong in
    /// that bank alone. Seeding every bank would invent three readings
    /// the capture never took.
    fn apply_silicon_power_on(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            if 0x0100_0000 + off == TABLE_SELECT {
                self.write_word(TABLE_SELECT, val).ok();
            }
        }
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if abs == TABLE_SELECT {
                continue;
            }
            if let Some(idx) = Self::window_index(abs) {
                self.banks[self.bank][idx] = val;
            } else if let Some((region, i)) = self.locate(abs) {
                self.stores[region][i] = val;
            }
        }
    }
}

impl Peripheral for Mpcp {
    fn name(&self) -> &'static str {
        "mpcp"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        MPCP_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        if let Some(idx) = Self::window_index(addr) {
            return Ok(self.banks[self.bank][idx]);
        }
        match self.locate(addr) {
            Some((region, idx)) => Ok(self.stores[region][idx]),
            None => Ok(0),
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if let Some(idx) = Self::window_index(addr) {
            self.banks[self.bank][idx] = val;
            return Ok(());
        }
        if (addr & !0x3) == TABLE_SELECT {
            self.bank = (val as usize) & (BANK_COUNT - 1);
        }
        if let Some((region, idx)) = self.locate(addr) {
            self.stores[region][idx] = val;
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        for store in &mut self.stores {
            store.fill(0);
        }
        for bank in &mut self.banks {
            bank.fill(0);
        }
        self.bank = 0;
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        // Silicon power-on snapshot already applied in `reset_cold`.
        self.reset_cold();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::empty(self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn claims_cover_the_four_regions() {
        let m = Mpcp::new();
        assert!(m.claims(0x0100_0120));
        assert!(m.claims(0x0100_013C));
        assert!(!m.claims(0x0100_0140));

        assert!(m.claims(0x0100_0320));
        assert!(m.claims(0x0100_0324));
        assert!(!m.claims(0x0100_0328));

        assert!(m.claims(0x0100_1180));
        assert!(m.claims(0x0100_11BC));
        assert!(!m.claims(0x0100_11C0));

        assert!(m.claims(0x0100_1268));
        assert!(m.claims(0x0100_138C));
        assert!(!m.claims(0x0100_1390));
    }

    #[test]
    fn round_trip_store() {
        let mut m = Mpcp::new();
        m.write_word(0x0100_1180, 0x8000_0000).unwrap();
        assert_eq!(m.read_word(0x0100_1180).unwrap(), 0x8000_0000);
        m.write_word(0x0100_12BC, 0x0000_0003).unwrap();
        assert_eq!(m.read_word(0x0100_12BC).unwrap(), 0x0000_0003);
    }

    #[test]
    fn warm_snapshot_seeds_from_mmio_init() {
        let mut m = Mpcp::new();
        m.reset_warm();
        // mmio_init: 0x1180 = 0x80000000, 0x1240 -> outside MPCP
        // 0x12BC = 0x00000003, 0x12C0 = 0x00060000
        assert_eq!(m.read_word(0x0100_1180).unwrap(), 0x8000_0000);
        assert_eq!(m.read_word(0x0100_12BC).unwrap(), 0x0000_0003);
        assert_eq!(m.read_word(0x0100_12C0).unwrap(), 0x0006_0000);
    }

    /// The window is shared and the selector tells the tables apart.
    ///
    /// This is the shape the firmware relies on: it stores a MAC half
    /// under selector 0, a bandwidth descriptor under selector 3 at the
    /// same offset, and reads its MAC back afterwards. Flat, the second
    /// store eats the first.
    #[test]
    fn the_selector_keeps_two_tables_apart_at_one_offset() {
        let mut m = Mpcp::new();

        // A placeholder MAC, low half then high half, as software stores it.
        let (mac_lo, mac_hi) = (0x0203_0405, 0x0000_0201);

        m.write_word(0x0100_12BC, 0).unwrap();
        m.write_word(0x0100_12C0, mac_lo).unwrap();
        m.write_word(0x0100_12C4, mac_hi).unwrap();

        m.write_word(0x0100_12BC, 3).unwrap();
        m.write_word(0x0100_12C0, 0x0006_0000).unwrap();
        assert_eq!(m.read_word(0x0100_12C0).unwrap(), 0x0006_0000);

        m.write_word(0x0100_12BC, 0).unwrap();
        assert_eq!(m.read_word(0x0100_12C0).unwrap(), mac_lo);
        assert_eq!(m.read_word(0x0100_12C4).unwrap(), mac_hi);
    }

    /// Words below the window are not banked — the selector itself reads
    /// back whatever was last armed, whichever table that names.
    #[test]
    fn the_selector_word_itself_is_not_banked() {
        let mut m = Mpcp::new();
        m.write_word(0x0100_12BC, 1).unwrap();
        m.write_word(0x0100_1284, 0xDEAD_BEEF).unwrap();
        m.write_word(0x0100_12BC, 0).unwrap();
        assert_eq!(m.read_word(0x0100_12BC).unwrap(), 0);
        assert_eq!(m.read_word(0x0100_1284).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn claims_does_not_overlap_other_peripherals() {
        let m = Mpcp::new();
        // Sparse EPON MAC claim.
        assert!(!m.claims(0x0100_0000));
        assert!(!m.claims(0x0100_0030));
        // SerDes range.
        assert!(!m.claims(0x0100_0180));
        assert!(!m.claims(0x0100_01C0));
        // PBC range.
        assert!(!m.claims(0x0100_0200));
        // MACsec range.
        assert!(!m.claims(0x0100_2400));
        // EPON LLID IRQ window.
        assert!(!m.claims(0x0100_1404));
    }
}
