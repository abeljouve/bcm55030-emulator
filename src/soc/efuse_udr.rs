//! BCM55030 eFuse / UDR serial bus — Session 6.
//!
//! Claims the three-register I²C-like bit-bang window at
//! `0x01000040`, `0x01000048`, `0x0100004C`. The BCM55030
//! firmware uses this window to read the on-chip eFuse blob
//! (80 bytes) and to drive an auxiliary UDR debug bus via bit
//! banging on SCL / SDA lines.
//!
//! The old `sysreg_shim` emulated the bit-bang responses inline:
//!
//!   * `0x048` — read returns the stored value OR'd with bit 31
//!     (SDA high) unless bit 4 of the stored value is set
//!     (SDA driven low).
//!   * `0x04C` — read always returns the stored value OR'd with
//!     bit 31.
//!   * `0x040` — side-effect register; writing a value with bit
//!     `0x8000` set clears the SCL edge-counter. Writing `0x04C`
//!     with bit 0 rising counts an SCL edge.
//!
//! This peripheral preserves that exact behaviour — resolves
//! audit 5.11 without drifting HW fidelity, and removes the last
//! special arms from `sysreg_shim`.
//!
//! The eFuse blob itself lives in SRAM at runtime `0x00031F18`
//! (80 bytes). It is not modelled as MMIO here — the firmware
//! already copies it into SRAM during its own init sequence.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, EfuseEvent, EfuseSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

pub const REG_I2C_UDR_CLK_RESET: u32 = 0x0100_0040;
pub const REG_I2C_UDR_SDA: u32 = 0x0100_0048;
pub const REG_I2C_UDR_SCL: u32 = 0x0100_004C;

const EFUSE_RANGES: &[AddressRange] = &[AddressRange::new(0x0100_0040, 0x0100_0050)];

/// Command-bit mask matching the BCM55030 protocol — any write
/// with bits `[31:27]` set is held in a shadow register and
/// cleared on the next read of the same register. The old sysreg
/// shim applied this across the whole aperture; now every
/// peripheral owns its own clear semantic.
const CMD_BIT_MASK: u32 = 0xF800_0000;

pub struct EfuseUdr {
    reg_clk_reset: u32,
    reg_sda: u32,
    reg_scl: u32,
    clk_reset_clear: u32,
    sda_clear: u32,
    scl_clear: u32,
    clock_toggles: u32,
    efuse_snapshot: Vec<u8>,
    pub trace: bool,
}

impl EfuseUdr {
    pub fn new() -> Self {
        Self {
            reg_clk_reset: 0,
            reg_sda: 0,
            reg_scl: 0,
            clk_reset_clear: 0,
            sda_clear: 0,
            scl_clear: 0,
            clock_toggles: 0,
            efuse_snapshot: vec![0xFFu8; 80],
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        // Match the whole 32-bit word for each register so half /
        // byte sub-offsets (0x41..0x43, 0x49..0x4B, 0x4D..0x4F) all
        // route here instead of slipping through to sysreg.
        let word = addr & !0x3;
        matches!(word, REG_I2C_UDR_CLK_RESET | REG_I2C_UDR_SDA | REG_I2C_UDR_SCL)
    }

    fn apply_warm_snapshot(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            match abs {
                REG_I2C_UDR_CLK_RESET => self.reg_clk_reset = val,
                REG_I2C_UDR_SDA => self.reg_sda = val,
                REG_I2C_UDR_SCL => self.reg_scl = val,
                _ => {}
            }
        }
    }
}

impl Peripheral for EfuseUdr {
    fn name(&self) -> &'static str {
        "efuse_udr"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        EFUSE_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        match addr {
            REG_I2C_UDR_CLK_RESET => {
                let val = self.reg_clk_reset;
                if self.clk_reset_clear != 0 {
                    self.reg_clk_reset &= !self.clk_reset_clear;
                    self.clk_reset_clear = 0;
                }
                Ok(val)
            }
            REG_I2C_UDR_SDA => {
                let base = self.reg_sda;
                if self.sda_clear != 0 {
                    self.reg_sda &= !self.sda_clear;
                    self.sda_clear = 0;
                }
                // SDA driven high unless bit 4 of stored value
                // forces it low. Command-bit auto-clear applies
                // to the base value before the SDA overlay.
                if base & 0x10 != 0 {
                    Ok(base & !0x8000_0000)
                } else {
                    Ok(base | 0x8000_0000)
                }
            }
            REG_I2C_UDR_SCL => {
                let base = self.reg_scl;
                if self.scl_clear != 0 {
                    self.reg_scl &= !self.scl_clear;
                    self.scl_clear = 0;
                }
                Ok(base | 0x8000_0000)
            }
            _ => Ok(0),
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let cmd_bits = val & CMD_BIT_MASK;
        match addr {
            REG_I2C_UDR_CLK_RESET => {
                // Bit 0x8000 rising edge clears the SCL edge counter.
                if (val & 0x8000) != 0 && (self.reg_clk_reset & 0x8000) == 0 {
                    self.clock_toggles = 0;
                }
                self.reg_clk_reset = val;
                if cmd_bits != 0 {
                    self.clk_reset_clear = cmd_bits;
                }
            }
            REG_I2C_UDR_SDA => {
                self.reg_sda = val;
                if cmd_bits != 0 {
                    self.sda_clear = cmd_bits;
                }
            }
            REG_I2C_UDR_SCL => {
                // Bit 0 rising edge = SCL clock pulse.
                if (val & 1) != 0 && (self.reg_scl & 1) == 0 {
                    self.clock_toggles = self.clock_toggles.wrapping_add(1);
                }
                self.reg_scl = val;
                if cmd_bits != 0 {
                    self.scl_clear = cmd_bits;
                }
            }
            _ => {}
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.reg_clk_reset = 0;
        self.reg_sda = 0;
        self.reg_scl = 0;
        self.clk_reset_clear = 0;
        self.sda_clear = 0;
        self.scl_clear = 0;
        self.clock_toggles = 0;
    }

    fn reset_warm(&mut self) {
        self.reset_cold();
        self.apply_warm_snapshot();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Efuse(EfuseSnapshot {
            udr_status: self.reg_sda,
            clock_toggles: self.clock_toggles,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Efuse(ev) => match ev {
                EfuseEvent::SetSnapshot(bytes) => {
                    self.efuse_snapshot = bytes.iter().take(80).copied().collect();
                    while self.efuse_snapshot.len() < 80 {
                        self.efuse_snapshot.push(0xFF);
                    }
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
    fn sda_high_when_not_driven_low() {
        let mut e = EfuseUdr::new();
        e.write_word(REG_I2C_UDR_SDA, 0x0000_0000).unwrap();
        assert_eq!(e.read_word(REG_I2C_UDR_SDA).unwrap() & 0x8000_0000, 0x8000_0000);
    }

    #[test]
    fn sda_low_when_bit4_set() {
        let mut e = EfuseUdr::new();
        e.write_word(REG_I2C_UDR_SDA, 0x0000_0010).unwrap();
        assert_eq!(e.read_word(REG_I2C_UDR_SDA).unwrap() & 0x8000_0000, 0);
    }

    #[test]
    fn scl_always_high_bit() {
        let mut e = EfuseUdr::new();
        assert_eq!(e.read_word(REG_I2C_UDR_SCL).unwrap() & 0x8000_0000, 0x8000_0000);
    }

    #[test]
    fn clock_toggles_count_scl_edges() {
        let mut e = EfuseUdr::new();
        e.write_word(REG_I2C_UDR_SCL, 0).unwrap();
        e.write_word(REG_I2C_UDR_SCL, 1).unwrap();
        e.write_word(REG_I2C_UDR_SCL, 0).unwrap();
        e.write_word(REG_I2C_UDR_SCL, 1).unwrap();
        assert_eq!(e.clock_toggles, 2);
        // 0x040 bit 0x8000 rising clears the counter.
        e.write_word(REG_I2C_UDR_CLK_RESET, 0x8000).unwrap();
        assert_eq!(e.clock_toggles, 0);
    }

    #[test]
    fn claims_only_three_addresses() {
        let e = EfuseUdr::new();
        assert!(e.claims(REG_I2C_UDR_CLK_RESET));
        assert!(e.claims(REG_I2C_UDR_SDA));
        assert!(e.claims(REG_I2C_UDR_SCL));
        assert!(!e.claims(0x0100_0044));
        assert!(!e.claims(0x0100_0050));
    }
}
