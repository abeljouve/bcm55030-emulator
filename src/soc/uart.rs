//! BCM55030 UART — SFP+ console port peripheral.
//!
//! MMIO aperture: `0x00FC1000..0x00FC1100`. A single 16550-like controller
//! mirrored 8× every `0x20` bytes (verified on real hardware). Per-channel
//! registers: `DATA = +0x10`, `IER/STATUS = +0x14`, `BAUD_LO = +0x18`,
//! `BAUD_HI = +0x1C`. Per-channel offsets `0x00..0x0F` are unclaimed.
//!
//! Session 1 notes:
//! - `held_pre_firmware` / `firmware_loaded` / `FIRMWARE_BASE` handling **removed**.
//!   The hardware has no concept of firmware. Bytes typed during the
//!   bootloader are consumed by the bootloader's own CLI prompt
//!   (`FFFF/>`) just like on real silicon. To drive firmware, type after
//!   seeing the `2000/>` prompt.
//! - Input arrives via the bank's mpsc channel; `PeripheralBank::tick`
//!   drains the receiver and calls [`Uart::push_rx_byte`] for each byte.
//! - TX bytes go to `stdout` (CLI mode) and are mirrored into a bounded
//!   `tx_log` ring so the future UI can render the console output in a
//!   panel without racing with stdout.

use std::collections::VecDeque;
use std::io::{self, Write};

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, Peripheral, PeripheralError, PeripheralEvent, PeripheralSnapshot,
    UartEvent, UartSnapshot,
};

pub const UART_RANGE_START: u32 = 0x00FC1000;
pub const UART_RANGE_END: u32 = 0x00FC1100;
const UART_CHANNEL_SIZE: u32 = 0x20;
/// First defined register offset within a channel (DATA register).
const UART_REG_OFFSET: u32 = 0x10;

/// Bounded ring depth for the TX log shown in the UI panel.
const TX_LOG_CAPACITY: usize = 16 * 1024;

const UART_RANGES: &[AddressRange] = &[AddressRange::new(UART_RANGE_START, UART_RANGE_END)];

#[derive(Clone)]
pub struct Uart {
    ier: u8,
    baud_div_lo: u8,
    baud_div_hi: u8,
    rx_queue: VecDeque<u8>,
    /// Bounded ring of bytes written by the firmware for the UI panel.
    /// Oldest entries are dropped when the ring fills up.
    tx_log: VecDeque<u8>,
    /// When true, TX bytes are also echoed to `stdout`. Enabled by
    /// default for CLI mode; the UI can disable it once its own panel
    /// is wired up.
    pub stdout_passthrough: bool,
}

impl Uart {
    pub fn new() -> Self {
        Self {
            ier: 0,
            baud_div_lo: 0,
            baud_div_hi: 0,
            rx_queue: VecDeque::new(),
            tx_log: VecDeque::with_capacity(TX_LOG_CAPACITY),
            stdout_passthrough: true,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (UART_RANGE_START..UART_RANGE_END).contains(&addr)
    }

    /// Map an absolute UART address to the register offset within a
    /// single channel (`0x00..0x0F`). Returns `None` for the unclaimed
    /// lower half of the channel.
    fn reg_offset(addr: u32) -> Option<u32> {
        let channel_off = (addr - UART_RANGE_START) % UART_CHANNEL_SIZE;
        if channel_off >= UART_REG_OFFSET {
            Some(channel_off - UART_REG_OFFSET)
        } else {
            None
        }
    }

    /// Push a byte received from the mpsc channel (stdin / UI / MCP).
    pub fn push_rx_byte(&mut self, b: u8) {
        self.rx_queue.push_back(b);
    }

    /// True if the UART should raise its IRQ line (`IRQ 5`, level 1).
    pub fn irq_pending(&self) -> u32 {
        let tx = self.ier & 0x40 != 0;
        let rx = (self.ier & 0x04 != 0) && !self.rx_queue.is_empty();
        if tx || rx {
            1u32 << 5
        } else {
            0
        }
    }

    /// Compose the IER/STATUS register with live hardware bits overlaid.
    /// Bit 5 = RX buffer empty, bit 7 = TX complete (always 1 in v1
    /// because TX goes to stdout synchronously; audit 6.3 is noted).
    fn read_ier(&self) -> u8 {
        let mut val = self.ier;
        if self.rx_queue.is_empty() {
            val |= 0x20;
        } else {
            val &= !0x20;
        }
        val |= 0x80;
        val
    }

    /// Raw IER with software-managed bits only.
    pub fn ier(&self) -> u8 {
        self.ier
    }

    fn read_reg_byte(&mut self, reg: u32) -> u8 {
        match reg {
            0x00 => self.rx_queue.pop_front().unwrap_or(0),
            0x04 => self.read_ier(),
            0x08 => self.baud_div_lo,
            0x0C => self.baud_div_hi,
            _ => 0,
        }
    }

    fn write_reg_byte(&mut self, reg: u32, val: u8) {
        match reg {
            0x00 => {
                self.log_tx_byte(val);
                if self.stdout_passthrough {
                    let stdout = io::stdout();
                    let mut handle = stdout.lock();
                    let _ = handle.write_all(&[val]);
                    let _ = handle.flush();
                }
            }
            0x04 => {
                self.ier = val;
            }
            0x08 => self.baud_div_lo = val,
            0x0C => self.baud_div_hi = val,
            _ => {}
        }
    }

    fn log_tx_byte(&mut self, b: u8) {
        if self.tx_log.len() == TX_LOG_CAPACITY {
            self.tx_log.pop_front();
        }
        self.tx_log.push_back(b);
    }

    /// Current TX log contents (UI/test observation).
    pub fn tx_log_bytes(&self) -> Vec<u8> {
        self.tx_log.iter().copied().collect()
    }

    /// Clear the TX log ring (UI action).
    pub fn clear_tx_log(&mut self) {
        self.tx_log.clear();
    }
}

impl Peripheral for Uart {
    fn name(&self) -> &'static str {
        "uart"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        UART_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        if let Some(reg) = Self::reg_offset(addr) {
            let b = self.read_reg_byte(reg);
            Ok((b as u32) << 24)
        } else {
            Ok(0)
        }
    }

    fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        let Some(reg) = Self::reg_offset(addr) else {
            return Ok(0);
        };
        let b: u8 = match reg {
            // Side-effect-free peek of the RX FIFO head. Real
            // `read_word(0x00)` pops the front byte — `peek_word`
            // must not.
            0x00 => self.rx_queue.front().copied().unwrap_or(0),
            0x04 => self.read_ier(),
            0x08 => self.baud_div_lo,
            0x0C => self.baud_div_hi,
            _ => 0,
        };
        Ok((b as u32) << 24)
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if let Some(reg) = Self::reg_offset(addr) {
            self.write_reg_byte(reg, (val >> 24) as u8);
        }
        Ok(())
    }

    fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        match Self::reg_offset(addr) {
            Some(reg) => Ok(self.read_reg_byte(reg)),
            None => Ok(0),
        }
    }

    fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if let Some(reg) = Self::reg_offset(addr) {
            self.write_reg_byte(reg, val);
        }
        Ok(())
    }

    fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        let hi = match Self::reg_offset(addr) {
            Some(reg) => self.read_reg_byte(reg) as u16,
            None => 0u16,
        };
        let lo = match Self::reg_offset(addr + 1) {
            Some(reg) => self.read_reg_byte(reg) as u16,
            None => 0u16,
        };
        Ok((hi << 8) | lo)
    }

    fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        if let Some(reg) = Self::reg_offset(addr) {
            self.write_reg_byte(reg, (val >> 8) as u8);
        }
        if let Some(reg) = Self::reg_offset(addr + 1) {
            self.write_reg_byte(reg, val as u8);
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.ier = 0;
        self.baud_div_lo = 0;
        self.baud_div_hi = 0;
        self.rx_queue.clear();
        // tx_log preserved — it's an observational buffer, not hardware.
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        let tail_len = self.tx_log.len().min(2048);
        let skip = self.tx_log.len() - tail_len;
        let tail: Vec<u8> = self.tx_log.iter().skip(skip).copied().collect();
        PeripheralSnapshot::Uart(UartSnapshot {
            ier: self.ier,
            baud_divisor: ((self.baud_div_hi as u16) << 8) | (self.baud_div_lo as u16),
            rx_queue_len: self.rx_queue.len(),
            tx_log_tail: tail,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Uart(ev) => match ev {
                UartEvent::Bytes(bytes) => {
                    for &b in bytes {
                        self.push_rx_byte(b);
                    }
                    Ok(())
                }
                UartEvent::Break => Ok(()),
                UartEvent::ClearTxLog => {
                    self.clear_tx_log();
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
    fn uart_tx_byte_logged() {
        let mut u = Uart::new();
        u.stdout_passthrough = false;
        // DATA register of channel 0 = 0x00FC1010
        u.write_byte(0x00FC1010, b'A').unwrap();
        u.write_byte(0x00FC1010, b'B').unwrap();
        assert_eq!(u.tx_log_bytes(), vec![b'A', b'B']);
    }

    #[test]
    fn uart_rx_fifo_pop() {
        let mut u = Uart::new();
        u.push_rx_byte(b'X');
        u.push_rx_byte(b'Y');
        assert_eq!(u.read_byte(0x00FC1010).unwrap(), b'X');
        assert_eq!(u.read_byte(0x00FC1010).unwrap(), b'Y');
        assert_eq!(u.read_byte(0x00FC1010).unwrap(), 0);
    }

    #[test]
    fn uart_claims_mirror() {
        let u = Uart::new();
        assert!(u.claims(0x00FC1010));
        assert!(u.claims(0x00FC1030));
        assert!(u.claims(0x00FC10F0));
        assert!(!u.claims(0x00FC1100));
    }

    #[test]
    fn uart_irq_pending_tx_and_rx() {
        let mut u = Uart::new();
        assert_eq!(u.irq_pending(), 0);
        u.ier = 0x40; // TXIE
        assert_eq!(u.irq_pending(), 1 << 5);
        u.ier = 0x04; // RXIE
        assert_eq!(u.irq_pending(), 0); // no data
        u.push_rx_byte(b'!');
        assert_eq!(u.irq_pending(), 1 << 5);
    }
}
