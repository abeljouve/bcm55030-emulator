//! BCM55030 eFuse / UDR serial bus.
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
//! This peripheral preserves that exact behaviour and removes the
//! last special arms from `sysreg_shim`.
//!
//! ## The scan-out
//!
//! The 640-bit array is shifted out on bit 31 of `0x04C` once bit 0 of
//! `0x048` (read enable) is set: the first bit is present at once, each
//! clock pulse brings the next. Bits come most significant first, from
//! the last byte of the 80-byte array down to the first. Observed on
//! hardware: a reader that follows that order reproduces the array a
//! device prints. Outside a scan-out the data line reads high.
//!
//! The array ships as all ones, a synthetic placeholder that reads the
//! same as the idle data line: a real device's contents are loaded at
//! run time ([`EfuseUdr::load_array`]).

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

/// Bytes in the fuse array (640 bits).
pub const ARRAY_LEN: usize = 80;
/// `0x048` bit 0: read enable, the scan-out is running.
const READ_EN: u32 = 1 << 0;
/// Bit 31 of `0x048` / `0x04C`: the handshake flag and the serial data.
const LINE: u32 = 1 << 31;

#[derive(Clone)]
pub struct EfuseUdr {
    reg_clk_reset: u32,
    reg_sda: u32,
    reg_scl: u32,
    clk_reset_clear: u32,
    sda_clear: u32,
    scl_clear: u32,
    clock_toggles: u32,
    /// The fuse array, as a device prints it (byte 0 first).
    array: [u8; ARRAY_LEN],
    /// Bits shifted out since read enable was set.
    shift_pos: usize,
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
            array: [0xFF; ARRAY_LEN],
            shift_pos: 0,
            trace: false,
        }
    }

    /// Replace the fuse array with a device's own, exactly [`ARRAY_LEN`]
    /// bytes in the order a device prints them.
    pub fn load_array(&mut self, bytes: &[u8]) -> Result<(), String> {
        let array: [u8; ARRAY_LEN] = bytes
            .try_into()
            .map_err(|_| format!("eFuse array must be {ARRAY_LEN} bytes, got {}", bytes.len()))?;
        self.array = array;
        Ok(())
    }

    /// The serial data line: the array bit under the shift position while a
    /// scan-out runs, high otherwise and past the end of the array.
    fn data_line(&self) -> bool {
        if self.reg_sda & READ_EN == 0 || self.shift_pos >= ARRAY_LEN * 8 {
            return true;
        }
        let byte = self.array[ARRAY_LEN - 1 - self.shift_pos / 8];
        byte & (0x80 >> (self.shift_pos % 8)) != 0
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        // Match the whole 32-bit word for each register so half /
        // byte sub-offsets (0x41..0x43, 0x49..0x4B, 0x4D..0x4F) all
        // route here instead of slipping through to sysreg.
        let word = addr & !0x3;
        matches!(word, REG_I2C_UDR_CLK_RESET | REG_I2C_UDR_SDA | REG_I2C_UDR_SCL)
    }

    fn apply_silicon_power_on(&mut self) {
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
                Ok(if self.data_line() { base | LINE } else { base & !LINE })
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
                // Read enable rising: the scan-out starts at the first bit.
                if val & READ_EN != 0 && self.reg_sda & READ_EN == 0 {
                    self.shift_pos = 0;
                }
                self.reg_sda = val;
                if cmd_bits != 0 {
                    self.sda_clear = cmd_bits;
                }
            }
            REG_I2C_UDR_SCL => {
                // Bit 0 rising edge = SCL clock pulse.
                if (val & 1) != 0 && (self.reg_scl & 1) == 0 {
                    self.clock_toggles = self.clock_toggles.wrapping_add(1);
                    if self.reg_sda & READ_EN != 0 {
                        self.shift_pos = self.shift_pos.saturating_add(1);
                    }
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
        // The array itself is fuses: a reset does not change it.
        self.shift_pos = 0;
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        // Silicon power-on snapshot already applied in `reset_cold`.
        self.reset_cold();
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
                    // A short input leaves the rest at the placeholder value.
                    let mut array = [0xFFu8; ARRAY_LEN];
                    for (dst, src) in array.iter_mut().zip(bytes.iter()) {
                        *dst = *src;
                    }
                    self.array = array;
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
    fn data_line_high_outside_a_scan_out() {
        let mut e = EfuseUdr::new();
        assert_eq!(e.read_word(REG_I2C_UDR_SCL).unwrap() & LINE, LINE);
    }

    fn pulse(e: &mut EfuseUdr) {
        let v = e.read_word(REG_I2C_UDR_SCL).unwrap();
        e.write_word(REG_I2C_UDR_SCL, v | 1).unwrap();
        let v = e.read_word(REG_I2C_UDR_SCL).unwrap();
        e.write_word(REG_I2C_UDR_SCL, v & !1).unwrap();
    }

    fn data_bit(e: &mut EfuseUdr) -> u8 {
        (e.read_word(REG_I2C_UDR_SCL).unwrap() >> 31) as u8
    }

    /// The scan-out a reader performs: enable, preamble, start handshake,
    /// 640 bits most significant first into a buffer filled from its last
    /// byte down, trailer.
    fn scan_out(e: &mut EfuseUdr) -> [u8; ARRAY_LEN] {
        e.write_word(REG_I2C_UDR_SDA, 0).unwrap();
        e.write_word(REG_I2C_UDR_CLK_RESET, 0x8000).unwrap();
        for _ in 0..16 {
            pulse(e);
        }
        e.write_word(REG_I2C_UDR_SDA, 0x10).unwrap();
        pulse(e);
        assert_eq!(e.read_word(REG_I2C_UDR_SDA).unwrap() & LINE, 0);
        e.write_word(REG_I2C_UDR_SDA, 0).unwrap();
        pulse(e);
        assert_eq!(e.read_word(REG_I2C_UDR_SDA).unwrap() & LINE, LINE);
        e.write_word(REG_I2C_UDR_SDA, READ_EN).unwrap();
        let mut buf = [0u8; ARRAY_LEN];
        for bit in 0..ARRAY_LEN * 8 {
            if bit != 0 {
                pulse(e);
            }
            buf[ARRAY_LEN - 1 - bit / 8] |= data_bit(e) << (7 - bit % 8);
        }
        e.write_word(REG_I2C_UDR_SDA, 0).unwrap();
        for _ in 0..16 {
            pulse(e);
        }
        e.write_word(REG_I2C_UDR_CLK_RESET, 0).unwrap();
        buf
    }

    #[test]
    fn scan_out_returns_the_loaded_array() {
        let mut e = EfuseUdr::new();
        let array: [u8; ARRAY_LEN] = core::array::from_fn(|i| (i as u8).wrapping_mul(37) ^ 0x5A);
        e.load_array(&array).unwrap();
        assert_eq!(scan_out(&mut e), array);
        // A second scan-out starts again from the first bit.
        assert_eq!(scan_out(&mut e), array);
    }

    #[test]
    fn placeholder_array_by_default_and_kept_across_reset() {
        let mut e = EfuseUdr::new();
        assert_eq!(scan_out(&mut e), [0xFF; ARRAY_LEN]);
        let array = [0xA5; ARRAY_LEN];
        e.load_array(&array).unwrap();
        e.reset_cold();
        assert_eq!(scan_out(&mut e), array);
    }

    #[test]
    fn load_array_refuses_a_wrong_length() {
        let mut e = EfuseUdr::new();
        assert!(e.load_array(&[0; ARRAY_LEN - 1]).is_err());
        assert!(e.load_array(&[0; ARRAY_LEN + 1]).is_err());
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
