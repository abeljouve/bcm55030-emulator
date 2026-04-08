pub mod uart;
pub mod spi_flash;
pub mod pbc;

use crate::cpu::exception::Exception;
use uart::SimpleUart;
use pbc::PeripheralBusController;

/// UART base address in the SoC MMIO space.
/// Hardware base pointer is 0x00FC0FE8; data register at +0x28, IER at +0x2C.
const UART_BASE: u32 = 0x00FC1010;
const UART_SIZE: u32 = 0x10; // +0x00 through +0x0F (data, IER, baud_lo, baud_hi)

/// Peripheral Bus Controller (SPI + MDIO) base address
const PBC_BASE: u32 = 0x010001F0;
const PBC_SIZE: u32 = 0x50; // +0x00 through +0x4F

/// BCM55030 EPON MAC register block (includes Chip ID, revision, EPON regs)
/// The `reg N` CLI command reads offset N*4 from this base.
const SYSREG_BASE: u32 = 0x01000000;
const SYSREG_SIZE: u32 = 0x1F0; // +0x000 through +0x1EF (up to PBC_BASE)

/// SerDes lane status registers (firmware scans these at startup)
const SERDES_BASE: u32 = 0x224A0000;
const SERDES_SIZE: u32 = 0x0800; // 256 lanes × 8 bytes

/// MMIO controller — dispatches memory-mapped I/O accesses to peripherals
pub struct MmioController {
    pub uart: SimpleUart,
    pub pbc: PeripheralBusController,
    pub trace: bool,
    /// BCM55030 EPON MAC timer counter at SYSREG+0x050.
    /// Read by timer1_get_current_value (0x45E4) as a 16-bit hardware counter.
    /// Incremented each time Timer1 interrupt fires.
    pub timer_counter: u16,
    /// BCM55030 EPON MAC / system register storage for read-write registers.
    /// The bootloader writes config values (e.g., FDS recovery bitmap at +0x24)
    /// and reads them back later. Uninitialized entries default to 0.
    sysreg_store: [u32; SYSREG_SIZE as usize / 4],
    /// I2C bit-bang state for SFP EEPROM bus (SYSREG+0x48/0x4C).
    /// Counts clock toggles to simulate NAK (no SFP module present).
    i2c_clock_toggles: u32,
}

impl MmioController {
    pub fn new() -> Self {
        Self {
            uart: SimpleUart::new(),
            pbc: PeripheralBusController::new(),
            trace: false,
            timer_counter: 0,
            sysreg_store: [0; SYSREG_SIZE as usize / 4],
            i2c_clock_toggles: 0,
        }
    }

    #[inline]
    fn is_uart(addr: u32) -> bool {
        addr >= UART_BASE && addr < UART_BASE + UART_SIZE
    }

    #[inline]
    fn is_pbc(addr: u32) -> bool {
        addr >= PBC_BASE && addr < PBC_BASE + PBC_SIZE
    }

    #[inline]
    fn is_sysreg(addr: u32) -> bool {
        addr >= SYSREG_BASE && addr < SYSREG_BASE + SYSREG_SIZE
    }

    /// BCM55030 EPON MAC / system register reads.
    /// Hardware-defined registers return fixed values; all others return
    /// the last value written (read-write storage for firmware config).
    fn sysreg_read_word(&self, offset: u32) -> u32 {
        match offset {
            0x000 => 0x47010203, // reg 0x00: CHIP_ID (BCM4701)
            0x004 => 0xB2110816, // reg 0x01: CHIP_REV / bond options
            0x00C => 0x0114B820, // reg 0x03: LLID_CAPTURE_MASK
            0x018 => 0x00000006, // reg 0x06: LLID_ACTIVE_BITMAP
            0x030 => 0x0000FFFF, // reg 0x0C: RX_GRANT_MASK
            0x050 => self.timer_counter as u32, // Timer counter
            0x1E0 => 0x45504F4E, // reg 0x78: EPON signature ("EPON")
            0x048 => {
                // I2C status register for SFP EEPROM bit-bang bus.
                // Bit 31 = SDA input line. Bit 4 = ACK enable (set by firmware).
                // The firmware sets bit 4 before checking for ACK (bit 31=0),
                // then clears it and waits for SDA high (bit 31=1).
                // We simulate an always-ACKing SFP EEPROM:
                // - When bit 4 is set: return bit 31=0 (ACK from slave)
                // - When bit 4 is clear: return bit 31=1 (SDA released high)
                let base = self.sysreg_store[0x048 / 4];
                if base & 0x10 != 0 {
                    base & !0x80000000 // bit 4 set → ACK: SDA low
                } else {
                    base | 0x80000000  // bit 4 clear → SDA high (idle/stop)
                }
            }
            0x04C => {
                // I2C clock/data register. Bit 0 = SCL, bit 31 = SDA (data in).
                // For data reads, return SDA=1 (all 0xFF EEPROM data = blank SFP).
                let base = self.sysreg_store[0x04C / 4];
                base | 0x80000000 // SDA high = data bit 1 (0xFF bytes)
            }
            _ => {
                // Read-write storage for firmware-configured registers
                let idx = (offset / 4) as usize;
                if idx < self.sysreg_store.len() {
                    self.sysreg_store[idx]
                } else {
                    0
                }
            }
        }
    }

    /// BCM55030 EPON MAC / system register writes.
    fn sysreg_write_word(&mut self, offset: u32, val: u32) {
        // Track I2C clock toggles on bit 0 of register 0x4C
        if offset == 0x04C {
            let old = self.sysreg_store[0x04C / 4];
            if (val & 1) != 0 && (old & 1) == 0 {
                self.i2c_clock_toggles += 1;
            }
        }
        // Reset I2C state when start condition is initiated (bit 15 set on 0x40)
        if offset == 0x040 {
            let old = self.sysreg_store[0x040 / 4];
            if (val & 0x8000) != 0 && (old & 0x8000) == 0 {
                self.i2c_clock_toggles = 0;
            }
        }

        let idx = (offset / 4) as usize;
        if idx < self.sysreg_store.len() {
            // Auto-clear command/busy bits (27-31) on write.
            // Many EPON MAC registers use upper bits as write-1-to-trigger commands
            // that the hardware clears after processing. The firmware polls these bits
            // and expects them to be cleared. We simulate instant completion.
            let stored = val & 0x07FFFFFF; // clear bits 27-31 for reads
            self.sysreg_store[idx] = stored;
        }
    }

    #[inline]
    fn is_serdes(addr: u32) -> bool {
        addr >= SERDES_BASE && addr < SERDES_BASE + SERDES_SIZE
    }

    // ---------- byte ----------

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        if Self::is_uart(addr) {
            return self.uart.read_byte(addr - UART_BASE);
        }
        if Self::is_pbc(addr) {
            let offset = addr - PBC_BASE;
            let word_offset = offset & !3;
            let byte_idx = offset & 3;
            let word = self.pbc.read_word(word_offset);
            // Big-endian: byte 0 is MSB
            return Ok((word >> (24 - byte_idx * 8)) as u8);
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            let word_offset = offset & !3;
            let byte_idx = offset & 3;
            let word = self.sysreg_read_word(word_offset);
            return Ok((word >> (24 - byte_idx * 8)) as u8);
        }
        if Self::is_serdes(addr) {
            if self.trace {
                eprintln!("[MMIO] read  byte  0x{:08X} → 0x01 (serdes)", addr);
            }
            return Ok(1);
        }
        if self.trace {
            eprintln!("[MMIO] read  byte  0x{:08X} → 0x00", addr);
        }
        Ok(0)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if Self::is_uart(addr) {
            return self.uart.write_byte(addr - UART_BASE, val);
        }
        if self.trace {
            eprintln!("[MMIO] write byte  0x{:08X} = 0x{:02X}", addr, val);
        }
        Ok(())
    }

    // ---------- halfword (big-endian) ----------

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        if Self::is_uart(addr) {
            let hi = self.uart.read_byte(addr - UART_BASE)? as u16;
            let lo = self.uart.read_byte(addr + 1 - UART_BASE)? as u16;
            return Ok((hi << 8) | lo);
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            let word_offset = offset & !3;
            let half_idx = (offset >> 1) & 1;
            let word = self.sysreg_read_word(word_offset);
            return Ok((word >> (16 - half_idx * 16)) as u16);
        }
        if Self::is_serdes(addr) {
            if self.trace {
                eprintln!("[MMIO] read  half  0x{:08X} → 0x0001 (serdes)", addr);
            }
            return Ok(1);
        }
        if self.trace {
            eprintln!("[MMIO] read  half  0x{:08X} → 0x0000", addr);
        }
        Ok(0)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        if Self::is_uart(addr) {
            self.uart.write_byte(addr - UART_BASE, (val >> 8) as u8)?;
            self.uart.write_byte(addr + 1 - UART_BASE, val as u8)?;
            return Ok(());
        }
        if self.trace {
            eprintln!("[MMIO] write half  0x{:08X} = 0x{:04X}", addr, val);
        }
        Ok(())
    }

    // ---------- word (big-endian) ----------

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        if Self::is_uart(addr) {
            return self.uart.read_word(addr - UART_BASE);
        }
        if Self::is_pbc(addr) {
            let offset = addr - PBC_BASE;
            let val = self.pbc.read_word(offset);
            if self.trace {
                eprintln!("[MMIO] read  word  0x{:08X} → 0x{:08X} (pbc+0x{:02X})", addr, val, offset);
            }
            return Ok(val);
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            let val = self.sysreg_read_word(offset);
            if self.trace {
                eprintln!("[MMIO] read  word  0x{:08X} → 0x{:08X} (sysreg+0x{:02X})", addr, val, offset);
            }
            return Ok(val);
        }
        if Self::is_serdes(addr) {
            if self.trace {
                eprintln!("[MMIO] read  word  0x{:08X} → 0x00000001 (serdes)", addr);
            }
            return Ok(1);
        }
        if self.trace {
            eprintln!("[MMIO] read  word  0x{:08X} → 0x00000000", addr);
        }
        Ok(0)
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if Self::is_uart(addr) {
            return self.uart.write_word(addr - UART_BASE, val);
        }
        if Self::is_pbc(addr) {
            let offset = addr - PBC_BASE;
            if self.trace {
                eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X} (pbc+0x{:02X})", addr, val, offset);
            }
            self.pbc.write_word(offset, val);
            return Ok(());
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            self.sysreg_write_word(offset, val);
            if self.trace {
                eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X} (sysreg+0x{:02X})", addr, val, offset);
            }
            return Ok(());
        }
        if self.trace {
            eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X}", addr, val);
        }
        Ok(())
    }
}
