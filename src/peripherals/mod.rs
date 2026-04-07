pub mod uart;
pub mod spi_flash;
pub mod pbc;

use crate::cpu::exception::Exception;
use uart::SimpleUart;
use pbc::PeripheralBusController;

/// UART base address in the SoC MMIO space
const UART_BASE: u32 = 0x00FC1014;
const UART_SIZE: u32 = 0x40; // +0x00 through +0x3F

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
}

impl MmioController {
    pub fn new() -> Self {
        Self {
            uart: SimpleUart::new(),
            pbc: PeripheralBusController::new(),
            trace: false,
            timer_counter: 0,
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
    /// The `reg N` CLI command reads at offset N*4 from base 0x01000000.
    fn sysreg_read_word(&self, offset: u32) -> u32 {
        match offset {
            0x000 => 0x47010203, // reg 0x00: CHIP_ID (BCM4701)
            0x004 => 0xB2110816, // reg 0x01: CHIP_REV / bond options
            0x00C => 0x0114B820, // reg 0x03: LLID_CAPTURE_MASK
            0x018 => 0x00000006, // reg 0x06: LLID_ACTIVE_BITMAP
            0x030 => 0x0000FFFF, // reg 0x0C: RX_GRANT_MASK
            0x050 => self.timer_counter as u32, // Timer counter (read by timer1_get_current_value)
            0x1E0 => 0x45504F4E, // reg 0x78: EPON signature ("EPON")
            _ => 0,
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
            if self.trace {
                eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X} (sysreg+0x{:02X})", addr, val, addr - SYSREG_BASE);
            }
            return Ok(());
        }
        if self.trace {
            eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X}", addr, val);
        }
        Ok(())
    }
}
