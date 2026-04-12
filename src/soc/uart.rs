use std::collections::VecDeque;
use std::io::{self, Write};

use crate::cpu::exception::Exception;

/// BCM55030 UART controller (interrupt-driven, ring buffer architecture)
///
/// Hardware register map (byte-accessible, 4-byte aligned):
///   Base pointer in bootloader: 0x00FC0FE8
///   0x00FC1010 (base+0x28): Data register (TX write → stdout, RX read ← queue)
///   0x00FC1014 (base+0x2C): IER/Status register
///   0x00FC1018 (base+0x30): Baud divisor low byte
///   0x00FC101C (base+0x34): Baud divisor high byte
///
/// IER/Status register bits:
///   0: Error flag (W1C)
///   1: TX holding register empty status
///   2: RX IRQ enable (RXIE) — software R/W
///   5: RX buffer empty — hardware-generated on read (1=empty, 0=data available)
///   6: TX empty IRQ enable (TXIE) — software R/W
///   7: TX IRQ pending — hardware-generated on read (= TXIE, since TX is always ready)
///
/// The bootloader's UART ISR at 0x4348 (boot_uart_rx_handler) uses this controller:
///   - RX: reads data register when bit 5 = 0, stores in ring buffer at 0xF968
///   - TX: reads from ring buffer at 0xFA68, writes to data register when bit 7 = 1
///   - uart_tx_putchar (0x427C) sets TXIE (bit 6) when there's data to send
///   - uart_rx_getchar (0x4218) sets RXIE (bit 2) after reading from ring buffer
pub struct SimpleUart {
    /// IER/status register (software-managed bits; bits 5 and 7 are overlaid on read)
    ier: u8,
    baud_div_lo: u8,
    baud_div_hi: u8,
    /// Pending RX data from stdin, fed into the data register
    pub rx_queue: VecDeque<u8>,
    /// Stdin bytes that arrived during the bootloader phase (mem.app_size
    /// is None). The bootloader's UART ISR consumes the live rx_queue
    /// then discards the bytes when no CLI prompt is active, so we hold a
    /// parallel copy here. The `firmware_cli_poll_hook` drains this back into
    /// rx_queue on the first call to `cli_poll_and_process_input`, once
    /// firmware's CLI is ready to consume input.
    pub held_pre_firmware: VecDeque<u8>,
}

impl SimpleUart {
    pub fn new() -> Self {
        Self {
            ier: 0,
            baud_div_lo: 0,
            baud_div_hi: 0,
            rx_queue: VecDeque::new(),
            held_pre_firmware: VecDeque::new(),
        }
    }

    /// Read the IER/status register with hardware-generated bits overlaid.
    /// bit 5: RX buffer empty (1 = no data, 0 = data available)
    /// bit 7: TX IRQ pending (= TXIE bit 6, since TX holding register is always empty)
    fn read_ier(&self) -> u8 {
        let mut val = self.ier;
        // bit 5: reflect actual RX data availability
        if self.rx_queue.is_empty() {
            val |= 0x20;
        } else {
            val &= !0x20;
        }
        // bit 7: TX complete — always 1 since TX is instant (stdout).
        // On real HW this clears while a byte is being shifted out, then
        // re-asserts.  Polled firmware checks this before writing DATA.
        val |= 0x80;
        val
    }

    /// Get the raw IER register value (software-managed bits only).
    pub fn ier(&self) -> u8 {
        self.ier
    }

    /// Clear specific IER bits.
    pub fn ier_clear(&mut self, mask: u8) {
        self.ier &= !mask;
    }

    /// Set specific IER bits.
    pub fn ier_set(&mut self, mask: u8) {
        self.ier |= mask;
    }

    /// Check if the UART should generate an IRQ.
    /// Returns true when TX or RX needs ISR service.
    pub fn irq_pending(&self) -> bool {
        // TX: TXIE (bit 6) set — ISR needs to drain the software ring buffer
        let tx = self.ier & 0x40 != 0;
        // RX: RXIE (bit 2) set AND data available in hardware RX queue
        let rx = (self.ier & 0x04 != 0) && !self.rx_queue.is_empty();
        tx || rx
    }

    /// Read a byte from a UART register.
    /// Offsets are relative to UART_BASE (0x00FC1010):
    ///   0x00 = data, 0x04 = IER, 0x08 = baud_lo, 0x0C = baud_hi
    pub fn read_byte(&mut self, offset: u32) -> Result<u8, Exception> {
        match offset {
            0x00 => {
                // Data register: pop one byte from RX queue
                Ok(self.rx_queue.pop_front().unwrap_or(0))
            }
            0x04 => Ok(self.read_ier()),
            0x08 => Ok(self.baud_div_lo),
            0x0C => Ok(self.baud_div_hi),
            _ => Ok(0),
        }
    }

    pub fn read_word(&mut self, offset: u32) -> Result<u32, Exception> {
        // Registers are byte-wide at 4-byte boundaries.
        // Word read returns the byte value at the MSB (big-endian).
        let b = self.read_byte(offset)?;
        Ok((b as u32) << 24)
    }

    /// Write a byte to a UART register.
    pub fn write_byte(&mut self, offset: u32, val: u8) -> Result<(), Exception> {
        match offset {
            0x00 => {
                // Data register: TX byte → stdout
                let stdout = io::stdout();
                let mut handle = stdout.lock();
                let _ = handle.write_all(&[val]);
                let _ = handle.flush();
            }
            0x04 => {
                // IER/Status register — store all bits as-is.
                // Bits 5 and 7 are overridden on read, so stale values don't matter.
                self.ier = val;
            }
            0x08 => self.baud_div_lo = val,
            0x0C => self.baud_div_hi = val,
            _ => {}
        }
        Ok(())
    }

    pub fn write_word(&mut self, offset: u32, val: u32) -> Result<(), Exception> {
        // Word write: use the MSB byte (big-endian)
        self.write_byte(offset, (val >> 24) as u8)
    }
}
