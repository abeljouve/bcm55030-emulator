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

#[derive(Clone)]
pub struct SpiFlash {
    pub data: Vec<u8>,
    status: u8,
    /// Set when flash contents are modified (PP, SE, BE, CE, DMA write).
    /// Used to decide whether to persist flash to disk on exit.
    pub dirty: bool,
    /// Snapshot of `data` at firmware-load time. Used by the GUI
    /// memory viewer to highlight bytes that have been modified
    /// since the firmware was loaded, and by the persistence path
    /// to show the user whether anything needs saving.
    ///
    /// `None` until a firmware is loaded (fresh boot with no
    /// image → nothing to diff against).
    pub baseline: Option<Vec<u8>>,
    /// SST-style Auto-Address-Increment Program (AAI / opcode 0xAD)
    /// internal address pointer. `Some(addr)` while the chip is in AAI
    /// mode, i.e. between the AAI start burst (`[AD, a2, a1, a0, d0, d1]`,
    /// tx_len=6) and the WRDI that terminates it. Each AAI continuation
    /// burst (`[AD, di, di+1]`, tx_len=3) writes 2 bytes at this address
    /// and advances it by 2 — there is no address in the continuation
    /// command, the chip remembers it. Cleared by `CMD_WRDI` and by
    /// `reset_volatile`.
    ///
    /// Evidence: the reference firmware `fds_read_llid_flag_byte` (the decompiler
    /// 0x20011d74) and reference bootloader `spi_flash_write_command`
    /// (rt 0x4b54) both emit AAI start (`SPI_STATUS=0x61`) followed by
    /// AAI continuations (`SPI_STATUS=0x31`) and terminate with WRDI
    /// (`SPI_STATUS=0x11`). See bug
    /// the design notes
    /// section D3.
    aai_addr: Option<u32>,
}

impl SpiFlash {
    pub fn new() -> Self {
        Self {
            data: vec![0xFF; FLASH_SIZE],
            status: 0x00, // not busy, write disabled
            dirty: false,
            baseline: None,
            aai_addr: None,
        }
    }

    /// Reset volatile chip state (status latch, AAI mode). Flash array
    /// `data` is non-volatile and intentionally preserved across power
    /// cycles — callers that want a fresh chip must reseat `data`.
    pub fn reset_volatile(&mut self) {
        self.status = 0x00;
        self.aai_addr = None;
    }

    /// Capture the current `data` contents as the baseline for
    /// future diffs. Called after a firmware load so the memory
    /// viewer can highlight writes as they happen. Clears
    /// `dirty` because the on-disk image is now the same as the
    /// in-memory image.
    pub fn capture_baseline(&mut self) {
        self.baseline = Some(self.data.clone());
        self.dirty = false;
    }

    /// `true` when the byte at `offset` differs from the
    /// captured baseline. Returns `false` when no baseline has
    /// been captured yet.
    #[inline]
    pub fn is_byte_modified(&self, offset: usize) -> bool {
        match self.baseline.as_ref() {
            Some(b) => {
                offset < b.len() && offset < self.data.len() && self.data[offset] != b[offset]
            }
            None => false,
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
                // WRDI also terminates SST AAI mode. Real chips exit the
                // AAI state machine and accept the next command as a
                // normal one-shot opcode.
                self.aai_addr = None;
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
                    for i in 4..tx.len() {
                        let flash_addr = (addr as usize + (i - 4)) % FLASH_SIZE;
                        self.data[flash_addr] &= tx[i];
                    }
                    self.dirty = true;
                }
                self.status &= !SR_WEL;
            }
            CMD_SE => {
                if self.status & SR_WEL != 0 && tx.len() >= 4 {
                    let addr = Self::extract_addr(tx) as usize;
                    let sector_base = addr & !0xFFF;
                    if sector_base < FLASH_SIZE {
                        let end = (sector_base + 4096).min(FLASH_SIZE);
                        self.data[sector_base..end].fill(0xFF);
                        self.dirty = true;
                    }
                }
                self.status &= !SR_WEL;
            }
            CMD_BE => {
                if self.status & SR_WEL != 0 && tx.len() >= 4 {
                    let addr = Self::extract_addr(tx) as usize;
                    let block_base = addr & !0xFFFF;
                    if block_base < FLASH_SIZE {
                        let end = (block_base + 65536).min(FLASH_SIZE);
                        self.data[block_base..end].fill(0xFF);
                        self.dirty = true;
                    }
                }
                self.status &= !SR_WEL;
            }
            CMD_CE => {
                if self.status & SR_WEL != 0 {
                    self.data.fill(0xFF);
                    self.dirty = true;
                }
                self.status &= !SR_WEL;
            }
            CMD_CP => {
                // SST-style Auto-Address-Increment Program (AAI). Two
                // distinct burst shapes, decided by tx length — see
                // `aai_addr` doc on `SpiFlash`.
                //
                // - tx_len == 6: AAI START. tx = [AD, a2, a1, a0, d0, d1].
                //   Extract the 24-bit address from tx[1..4], write the
                //   2 data bytes (tx[4..6]), and arm `aai_addr` at
                //   `addr + 2` for subsequent continuations.
                // - tx_len == 3: AAI CONTINUATION. tx = [AD, di, di+1].
                //   No address — the chip auto-increments. Write the 2
                //   data bytes at the current `aai_addr` and advance.
                //
                // Both bursts require WEL set. **WEL stays set across
                // an AAI sequence** on real silicon and is only cleared
                // by WRDI — the the reference firmware relies on this
                // (their AAI-exit WRDI loop polls WEL waiting for it to
                // clear). Do NOT clear WEL in this handler.
                if self.status & SR_WEL == 0 {
                    // No write-enable → real chip ignores the command.
                } else if tx.len() == 6 {
                    let addr = Self::extract_addr(tx);
                    let mut cur = addr as usize % FLASH_SIZE;
                    self.data[cur] &= tx[4];
                    cur = (cur + 1) % FLASH_SIZE;
                    self.data[cur] &= tx[5];
                    cur = (cur + 1) % FLASH_SIZE;
                    self.aai_addr = Some(cur as u32);
                    self.dirty = true;
                } else if tx.len() == 3 {
                    if let Some(addr) = self.aai_addr {
                        let mut cur = addr as usize % FLASH_SIZE;
                        self.data[cur] &= tx[1];
                        cur = (cur + 1) % FLASH_SIZE;
                        self.data[cur] &= tx[2];
                        cur = (cur + 1) % FLASH_SIZE;
                        self.aai_addr = Some(cur as u32);
                        self.dirty = true;
                    }
                    // Continuation without a prior start → ignored.
                }
                // Other tx lengths are not part of the AAI protocol;
                // ignored.
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

    /// Write flash data from DMA transfer (page program semantics)
    /// After a write operation, WEL is automatically cleared (like real hardware).
    pub fn dma_write(&mut self, addr: u32, data: &[u8]) {
        let start = (addr as usize) % FLASH_SIZE;
        for (i, &byte) in data.iter().enumerate() {
            let flash_addr = (start + i) % FLASH_SIZE;
            self.data[flash_addr] &= byte;
        }
        self.status &= !SR_WEL;
        self.dirty = true;
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

    /// SST AAI: a 4-byte program issued as one start burst + one
    /// continuation burst, exactly as the reference firmware's
    /// `flash_clear_lane_direction_record` (writing 4 zero bytes) emits
    /// it. Pre-D3-fix this stayed `[0xFF; 4]` because the continuation
    /// was silently dropped. See bug
    /// the design notes D3.
    #[test]
    fn test_aai_program_4_bytes_zero() {
        let mut flash = SpiFlash::new();
        flash.execute_fifo_command(&[CMD_WREN], 0);
        // Start: AAI at addr 0x100000, data bytes 0x00 0x00
        flash.execute_fifo_command(&[CMD_CP, 0x10, 0x00, 0x00, 0x00, 0x00], 0);
        // Continuation: 2 more zero bytes (no address)
        flash.execute_fifo_command(&[CMD_CP, 0x00, 0x00], 0);
        let data = flash.dma_read(0x100000, 4);
        assert_eq!(data, vec![0x00, 0x00, 0x00, 0x00]);
        // WRDI terminates AAI.
        flash.execute_fifo_command(&[CMD_WRDI], 0);
    }

    /// AAI keeps WEL set across the burst chain; only WRDI clears it.
    /// The the reference firmware's AAI-exit loop polls WEL waiting for clear.
    #[test]
    fn test_aai_keeps_wel_until_wrdi() {
        let mut flash = SpiFlash::new();
        flash.execute_fifo_command(&[CMD_WREN], 0);
        assert_eq!(
            flash.execute_fifo_command(&[CMD_RDSR], 1)[0] & SR_WEL,
            SR_WEL
        );
        flash.execute_fifo_command(&[CMD_CP, 0x00, 0x10, 0x00, 0xAA, 0xBB], 0);
        // WEL still set after AAI start.
        assert_eq!(
            flash.execute_fifo_command(&[CMD_RDSR], 1)[0] & SR_WEL,
            SR_WEL
        );
        flash.execute_fifo_command(&[CMD_CP, 0xCC, 0xDD], 0);
        // WEL still set after continuation.
        assert_eq!(
            flash.execute_fifo_command(&[CMD_RDSR], 1)[0] & SR_WEL,
            SR_WEL
        );
        flash.execute_fifo_command(&[CMD_WRDI], 0);
        assert_eq!(
            flash.execute_fifo_command(&[CMD_RDSR], 1)[0] & SR_WEL,
            0
        );
        // Data landed correctly.
        let data = flash.dma_read(0x001000, 4);
        assert_eq!(data, vec![0xAA, 0xBB, 0xCC, 0xDD]);
    }

    /// Continuation without a prior start is a no-op.
    #[test]
    fn test_aai_continuation_without_start_ignored() {
        let mut flash = SpiFlash::new();
        flash.execute_fifo_command(&[CMD_WREN], 0);
        flash.execute_fifo_command(&[CMD_CP, 0x11, 0x22], 0);
        let data = flash.dma_read(0, 0x10);
        assert_eq!(data, vec![0xFF; 0x10]);
    }

    /// AAI start without WEL is a no-op.
    #[test]
    fn test_aai_without_wel_ignored() {
        let mut flash = SpiFlash::new();
        // No WREN.
        flash.execute_fifo_command(&[CMD_CP, 0x00, 0x00, 0x00, 0xAA, 0xBB], 0);
        let data = flash.dma_read(0, 4);
        assert_eq!(data, vec![0xFF, 0xFF, 0xFF, 0xFF]);
    }

    /// AND-semantics: AAI cannot turn `0`-bits back to `1`. Two writes
    /// to the same address combine bit-clears.
    #[test]
    fn test_aai_and_semantics() {
        let mut flash = SpiFlash::new();
        // First write: program 0xF0 0x0F at 0x200000.
        flash.execute_fifo_command(&[CMD_WREN], 0);
        flash.execute_fifo_command(&[CMD_CP, 0x20, 0x00, 0x00, 0xF0, 0x0F], 0);
        flash.execute_fifo_command(&[CMD_WRDI], 0);
        // Second write at same addr: program 0xAA 0x55. AND semantics →
        // result = (0xF0 & 0xAA), (0x0F & 0x55) = 0xA0, 0x05.
        flash.execute_fifo_command(&[CMD_WREN], 0);
        flash.execute_fifo_command(&[CMD_CP, 0x20, 0x00, 0x00, 0xAA, 0x55], 0);
        flash.execute_fifo_command(&[CMD_WRDI], 0);
        let data = flash.dma_read(0x200000, 2);
        assert_eq!(data, vec![0xA0, 0x05]);
    }

    /// Volatile reset clears status latch and any in-flight AAI burst.
    #[test]
    fn test_reset_volatile_clears_aai_and_status() {
        let mut flash = SpiFlash::new();
        flash.execute_fifo_command(&[CMD_WREN], 0);
        flash.execute_fifo_command(&[CMD_CP, 0x00, 0x00, 0x00, 0xAA, 0xBB], 0);
        assert!(flash.aai_addr.is_some());
        assert_ne!(flash.status & SR_WEL, 0);
        // Data already programmed survives reset (non-volatile array).
        let before = flash.dma_read(0, 2);
        flash.reset_volatile();
        assert!(flash.aai_addr.is_none());
        assert_eq!(flash.status, 0);
        let after = flash.dma_read(0, 2);
        assert_eq!(before, after);
    }
}
