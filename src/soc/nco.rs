//! BCM55030 Numerically Controlled Oscillator (NCO) — D1.
//!
//! Owns the single `NCO_TX_MODE_SECONDARY` register at
//! `0x01000F80`, identified by dumping SRAM after a full boot and
//! reading the runtime-initialised `DAT_ram_2003e924` pointer
//! (hwregs block 23). The firmware uses this register to toggle
//! dual-TX mode on the SerDes TX path:
//!
//! | Bit | Name                     | Semantics                   |
//! |:----|:-------------------------|:----------------------------|
//! | `9` | `NCO_TX_MODE_SECONDARY`  | Set in mode 2               |
//! | `14`| `NCO_DUAL_TX_ENABLE`     | Enable dual-TX mode         |
//!
//! v1 is a plain backing store. The future `CLK_READY_FLAG` at
//! `0x00FC1017` mentioned in `the design notes` section 12 is not
//! claimed here because the boot trace shows it is not touched
//! by the warm boot path; it will migrate in when a firmware
//! path that reads it is identified.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{AddressRange, Peripheral, PeripheralSnapshot};

pub const REG_NCO_TX_MODE: u32 = 0x0100_0F80;

const NCO_RANGES: &[AddressRange] =
    &[AddressRange::new(REG_NCO_TX_MODE, REG_NCO_TX_MODE + 4)];

pub struct Nco {
    tx_mode: u32,
    pub trace: bool,
}

impl Nco {
    pub fn new() -> Self {
        Self { tx_mode: 0, trace: false }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (REG_NCO_TX_MODE..REG_NCO_TX_MODE + 4).contains(&addr)
    }
}

impl Peripheral for Nco {
    fn name(&self) -> &'static str {
        "nco"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        NCO_RANGES
    }

    fn read_word(&mut self, _addr: u32) -> Result<u32, Exception> {
        Ok(self.tx_mode)
    }

    fn write_word(&mut self, _addr: u32, val: u32) -> Result<(), Exception> {
        self.tx_mode = val;
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.tx_mode = 0;
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::empty(self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_tx_mode() {
        let mut n = Nco::new();
        assert_eq!(n.read_word(REG_NCO_TX_MODE).unwrap(), 0);
        n.write_word(REG_NCO_TX_MODE, 0x0000_4000).unwrap();
        assert_eq!(n.read_word(REG_NCO_TX_MODE).unwrap(), 0x0000_4000);
    }

    #[test]
    fn claims_covers_word_only() {
        let n = Nco::new();
        assert!(n.claims(0x0100_0F80));
        assert!(n.claims(0x0100_0F83));
        assert!(!n.claims(0x0100_0F84));
        assert!(!n.claims(0x0100_0F7C));
    }
}
