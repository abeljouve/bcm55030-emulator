/// MX25L3235E SPI NOR Flash model — 4 MB (32 Mbit)
/// JEDEC ID: C2 20 16 (Macronix)

const FLASH_SIZE: usize = 4 * 1024 * 1024; // 4 MB

// SPI NOR flash commands
const CMD_RDSR: u8 = 0x05;   // Read Status Register
const CMD_WREN: u8 = 0x06;   // Write Enable
const CMD_WRDI: u8 = 0x04;   // Write Disable
const CMD_WRSR: u8 = 0x01;   // Write Status Register
const CMD_READ: u8 = 0x03;   // Read Data
const CMD_FAST_READ: u8 = 0x0B; // Fast Read
const CMD_PP: u8 = 0x02;     // Page Program
const CMD_SE: u8 = 0x20;     // Sector Erase (4 KB)
const CMD_BE: u8 = 0xD8;     // Block Erase (64 KB)
const CMD_CE: u8 = 0xC7;     // Chip Erase
const CMD_RDID: u8 = 0x9F;   // Read JEDEC ID
const CMD_REMS: u8 = 0x90;   // Read Electronic Manufacturer/Device ID
const CMD_CP: u8 = 0xAD;     // Continuously Program

// Status register bits
const SR_WIP: u8 = 0x01;     // Write In Progress
const SR_WEL: u8 = 0x02;     // Write Enable Latch

// JEDEC ID for MX25L3235E
const JEDEC_MANUFACTURER: u8 = 0xC2;
const JEDEC_MEMORY_TYPE: u8 = 0x20;
const JEDEC_CAPACITY: u8 = 0x16;
const DEVICE_ID: u8 = 0x15;

pub struct SpiFlash {
    pub data: Vec<u8>,
    status: u8,
}

impl SpiFlash {
    pub fn new() -> Self {
        Self {
            data: vec![0xFF; FLASH_SIZE],
            status: 0x00, // not busy, write disabled
        }
    }

    /// Execute a FIFO-based SPI command.
    /// `tx` contains the bytes sent to the flash (opcode + address + data).
    /// Returns response bytes of length `rx_len`.
    pub fn execute_fifo_command(&mut self, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if tx.is_empty() {
            return vec![0; rx_len];
        }

        let opcode = tx[0];
        let mut rx = vec![0u8; rx_len];

        match opcode {
            CMD_RDID => {
                // Returns manufacturer, memory type, capacity
                if rx_len >= 1 { rx[0] = JEDEC_MANUFACTURER; }
                if rx_len >= 2 { rx[1] = JEDEC_MEMORY_TYPE; }
                if rx_len >= 3 { rx[2] = JEDEC_CAPACITY; }
            }
            CMD_REMS => {
                // TX: 0x90, addr[23:16], addr[15:8], addr[7:0]
                // Response depends on addr[0]: 0 → MFR,DEV; 1 → DEV,MFR
                let addr_lsb = if tx.len() >= 4 { tx[3] & 1 } else { 0 };
                if addr_lsb == 0 {
                    if rx_len >= 1 { rx[0] = JEDEC_MANUFACTURER; }
                    if rx_len >= 2 { rx[1] = DEVICE_ID; }
                } else {
                    if rx_len >= 1 { rx[0] = DEVICE_ID; }
                    if rx_len >= 2 { rx[1] = JEDEC_MANUFACTURER; }
                }
            }
            CMD_RDSR => {
                if rx_len >= 1 { rx[0] = self.status; }
            }
            CMD_WREN => {
                self.status |= SR_WEL;
            }
            CMD_WRDI => {
                self.status &= !SR_WEL;
            }
            CMD_WRSR => {
                if self.status & SR_WEL != 0 && tx.len() >= 2 {
                    // Only writable bits (block protect, etc.) — keep WIP/WEL managed internally
                    self.status = (self.status & (SR_WIP | SR_WEL)) | (tx[1] & 0xFC);
                }
                self.status &= !SR_WEL;
            }
            CMD_READ => {
                if tx.len() >= 4 {
                    let addr = Self::extract_addr(tx);
                    for i in 0..rx_len {
                        let flash_addr = (addr as usize + i) % FLASH_SIZE;
                        rx[i] = self.data[flash_addr];
                    }
                }
            }
            CMD_FAST_READ => {
                // Same as READ but with 1 dummy byte after address
                if tx.len() >= 5 {
                    let addr = Self::extract_addr(tx);
                    for i in 0..rx_len {
                        let flash_addr = (addr as usize + i) % FLASH_SIZE;
                        rx[i] = self.data[flash_addr];
                    }
                }
            }
            CMD_PP => {
                if self.status & SR_WEL != 0 && tx.len() >= 4 {
                    let addr = Self::extract_addr(tx);
                    // Page program: can only change 1→0 within a 256-byte page
                    for i in 4..tx.len() {
                        let flash_addr = (addr as usize + (i - 4)) % FLASH_SIZE;
                        self.data[flash_addr] &= tx[i]; // can only clear bits
                    }
                }
                self.status &= !SR_WEL;
            }
            CMD_SE => {
                if self.status & SR_WEL != 0 && tx.len() >= 4 {
                    let addr = Self::extract_addr(tx) as usize;
                    let sector_base = addr & !0xFFF; // 4KB aligned
                    let end = (sector_base + 4096).min(FLASH_SIZE);
                    self.data[sector_base..end].fill(0xFF);
                }
                self.status &= !SR_WEL;
            }
            CMD_BE => {
                if self.status & SR_WEL != 0 && tx.len() >= 4 {
                    let addr = Self::extract_addr(tx) as usize;
                    let block_base = addr & !0xFFFF; // 64KB aligned
                    let end = (block_base + 65536).min(FLASH_SIZE);
                    self.data[block_base..end].fill(0xFF);
                }
                self.status &= !SR_WEL;
            }
            CMD_CE => {
                if self.status & SR_WEL != 0 {
                    self.data.fill(0xFF);
                }
                self.status &= !SR_WEL;
            }
            CMD_CP => {
                // Continuously Program mode — handled at higher level
                // For FIFO, treat as page program with 2-byte data
                if self.status & SR_WEL != 0 && tx.len() >= 4 {
                    let addr = Self::extract_addr(tx);
                    for i in 4..tx.len() {
                        let flash_addr = (addr as usize + (i - 4)) % FLASH_SIZE;
                        self.data[flash_addr] &= tx[i];
                    }
                }
            }
            _ => {
                // Unknown command — ignore
            }
        }

        rx
    }

    /// Read flash data for DMA transfer
    pub fn dma_read(&self, addr: u32, len: usize) -> Vec<u8> {
        let mut result = vec![0xFF; len];
        let start = (addr as usize) % FLASH_SIZE;
        for i in 0..len {
            result[i] = self.data[(start + i) % FLASH_SIZE];
        }
        result
    }

    /// Write flash data from DMA transfer
    pub fn dma_write(&mut self, addr: u32, data: &[u8]) {
        let start = (addr as usize) % FLASH_SIZE;
        for (i, &byte) in data.iter().enumerate() {
            let flash_addr = (start + i) % FLASH_SIZE;
            self.data[flash_addr] &= byte; // page program semantics
        }
    }

    fn extract_addr(tx: &[u8]) -> u32 {
        ((tx[1] as u32) << 16) | ((tx[2] as u32) << 8) | (tx[3] as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jedec_id() {
        let mut flash = SpiFlash::new();
        let rx = flash.execute_fifo_command(&[CMD_RDID], 3);
        assert_eq!(rx, vec![0xC2, 0x20, 0x16]);
    }

    #[test]
    fn test_rems() {
        let mut flash = SpiFlash::new();
        let rx = flash.execute_fifo_command(&[CMD_REMS, 0, 0, 0], 2);
        assert_eq!(rx[0], 0xC2);
        assert_eq!(rx[1], 0x15);
    }

    #[test]
    fn test_status_register() {
        let mut flash = SpiFlash::new();
        // Initial status: 0
        let rx = flash.execute_fifo_command(&[CMD_RDSR], 1);
        assert_eq!(rx[0], 0x00);

        // Write enable
        flash.execute_fifo_command(&[CMD_WREN], 0);
        let rx = flash.execute_fifo_command(&[CMD_RDSR], 1);
        assert_eq!(rx[0], SR_WEL);

        // Write disable
        flash.execute_fifo_command(&[CMD_WRDI], 0);
        let rx = flash.execute_fifo_command(&[CMD_RDSR], 1);
        assert_eq!(rx[0], 0x00);
    }

    #[test]
    fn test_read_erased() {
        let flash = SpiFlash::new();
        let data = flash.dma_read(0, 4);
        assert_eq!(data, vec![0xFF, 0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn test_page_program_and_read() {
        let mut flash = SpiFlash::new();
        flash.execute_fifo_command(&[CMD_WREN], 0);
        flash.execute_fifo_command(&[CMD_PP, 0x00, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF], 0);
        let data = flash.dma_read(0, 4);
        assert_eq!(data, vec![0xDE, 0xAD, 0xBE, 0xEF]);
    }
}
