use std::io::{self, Write};

use crate::cpu::exception::Exception;

/// Simple UART controller (bootloader UART)
/// Base address: 0x00FC1014
/// Offsets relative to base:
///   +0x00: Control (write 0xA4 to init)
///   +0x04: Baud divisor low
///   +0x08: Baud divisor high
///   +0x28: TX data (write byte → serial out)
///   +0x2C: Status register (bit 7 = TX complete, bit 5 = RX ready)
pub struct SimpleUart {
    control: u32,
    baud_div_lo: u32,
    baud_div_hi: u32,
    rx_buffer: Option<u8>,
}

impl SimpleUart {
    pub fn new() -> Self {
        Self {
            control: 0,
            baud_div_lo: 0,
            baud_div_hi: 0,
            rx_buffer: None,
        }
    }

    /// Check stdin for a pending byte (non-blocking)
    fn poll_rx(&mut self) {
        if self.rx_buffer.is_some() {
            return;
        }
        // Try non-blocking read from stdin
        // On Unix, stdin in raw mode could be non-blocking; here we just skip
        // since we can't easily do non-blocking stdin without termios.
        // RX will be available when explicitly fed.
    }

    /// Feed a byte into the RX buffer (for external injection)
    pub fn feed_rx(&mut self, byte: u8) {
        self.rx_buffer = Some(byte);
    }

    pub fn read_byte(&mut self, offset: u32) -> Result<u8, Exception> {
        match offset {
            0x00 => Ok(self.control as u8),
            0x04 => Ok(self.baud_div_lo as u8),
            0x08 => Ok(self.baud_div_hi as u8),
            0x28 => Ok(0), // TX data register (read returns 0)
            0x2C => {
                // Status register
                self.poll_rx();
                let mut status: u8 = 0x80; // TX complete (bit 7) always set
                if self.rx_buffer.is_some() {
                    status |= 0x20; // RX ready (bit 5)
                }
                Ok(status)
            }
            _ => Ok(0),
        }
    }

    pub fn read_word(&mut self, offset: u32) -> Result<u32, Exception> {
        match offset {
            0x00 => Ok(self.control),
            0x04 => Ok(self.baud_div_lo),
            0x08 => Ok(self.baud_div_hi),
            0x2C => {
                self.poll_rx();
                let mut status: u32 = 0x80;
                if self.rx_buffer.is_some() {
                    status |= 0x20;
                }
                Ok(status)
            }
            _ => Ok(0),
        }
    }

    pub fn write_byte(&mut self, offset: u32, val: u8) -> Result<(), Exception> {
        match offset {
            0x00 => {
                self.control = val as u32;
            }
            0x04 => {
                self.baud_div_lo = val as u32;
            }
            0x08 => {
                self.baud_div_hi = val as u32;
            }
            0x28 => {
                // TX data — output to stdout
                let stdout = io::stdout();
                let mut handle = stdout.lock();
                let _ = handle.write_all(&[val]);
                let _ = handle.flush();
            }
            _ => {}
        }
        Ok(())
    }

    pub fn write_word(&mut self, offset: u32, val: u32) -> Result<(), Exception> {
        match offset {
            0x00 => {
                self.control = val;
            }
            0x04 => {
                self.baud_div_lo = val;
            }
            0x08 => {
                self.baud_div_hi = val;
            }
            0x28 => {
                // TX data — output low byte to stdout
                let stdout = io::stdout();
                let mut handle = stdout.lock();
                let _ = handle.write_all(&[val as u8]);
                let _ = handle.flush();
            }
            _ => {}
        }
        Ok(())
    }
}
