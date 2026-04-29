//! BCM55030 MPCP / Multi-Point Control Protocol — D3.
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
//! Block 17 "MPCP Speed Configuration" is an SRAM data structure
//! (runtime `0x0007E6CC`) — not MMIO, not modelled here. Block 22
//! "MPCP LLID Control" is inside the SerDes lane configuration
//! range; the SerDes peripheral round-trips it correctly.
//! Block 61 "MPCP Slot HW Command Engine" (runtime `0x01002644`,
//! stride `0x400`) is inside the MACsec SA programming range and
//! is served by the MACsec command-bit shadow mechanism.
//!
//! All runtime base addresses were extracted by dumping SRAM
//! after a full `boot_to_prompt_warm` run and reading the
//! firmware-initialised DAT_* pointers. See deferral D3 in
//! `the design notes` for the RE notes.
//!
//! v1 is a plain backing store — writes land in an internal
//! `Vec<u32>` keyed by the claim region, reads round-trip the
//! stored value. Warm snapshot pre-seeds from `mmio_init.rs`. A
//! future session can layer MPCP semantics on top once the CLI
//! exposes MPCP state.

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

#[derive(Clone)]
pub struct Mpcp {
    stores: [Vec<u32>; REGION_COUNT],
    pub trace: bool,
}

impl Mpcp {
    pub fn new() -> Self {
        let stores = std::array::from_fn(|i| {
            let r = REGIONS[i];
            vec![0u32; ((r.end - r.start) / 4) as usize]
        });
        Self { stores, trace: false }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        let word = addr & !0x3;
        REGIONS.iter().any(|r| (r.start..r.end).contains(&word))
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

    fn apply_silicon_power_on(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if let Some((region, idx)) = self.locate(abs) {
                self.stores[region][idx] = val;
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
        match self.locate(addr) {
            Some((region, idx)) => Ok(self.stores[region][idx]),
            None => Ok(0),
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
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
