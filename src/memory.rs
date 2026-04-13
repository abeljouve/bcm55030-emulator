use std::cell::RefCell;

use crate::cache::{DCache, ICache, IC_LINE_SIZE};
use crate::cpu::exception::Exception;
use crate::soc::mmio::MmioController;

/// BCM55030 unified SRAM size: 512 KB
/// Hardware confirmed: BCR 0x74 (ICCM) = 0, BCR 0x78 (DCCM) = 0.
/// The BCM55030 has unified 512 KB SRAM with I-cache/D-cache, no ICCM/DCCM.
pub const SRAM_SIZE: usize = 512 * 1024;

pub struct Memory {
    /// Primary data store (unified SRAM in SoC mode, flat memory for tests).
    /// In SoC mode, both instruction fetch and data access read/write this store.
    data: Vec<u8>,

    /// MMIO peripheral controller (SoC mode only).
    /// Uses RefCell for interior mutability: reads may have side effects
    /// (e.g. UART status register clears on read).
    mmio: Option<RefCell<MmioController>>,

    /// SRAM base address (SoC mode). Data/instruction access at addr in
    /// [dccm_base, dccm_base + sram_size) reads/writes SRAM.
    /// Default 0 for bootloader, 0x20000000 for firmware.
    pub dccm_base: u32,

    /// Size of the loaded app binary (used for BSS clearing in boot ROM CRT init)
    pub app_size: Option<usize>,

    /// Runtime base address where the firmware binary is loaded in SRAM.
    /// `0` until firmware loads. Firmware is loaded at `0x32000` matching real BCM55030
    /// hardware: bootloader stays at `0..0xA800`, firmware at `0x32000..`.
    /// Validated via `mem/rm 0x32000` returning the firmware IVT signature on real HW.
    pub app_load_base: u32,

    /// SRAM write watchpoint address (temporary diagnostic).
    /// When set, logs the first write to this word-aligned address with full context.
    pub dccm_watchpoint: Option<u32>,

    /// ARC700 D-cache: 8 KB, 4-way set-associative, 32-byte lines.
    /// Present in SoC mode only. LD/ST data accesses go through it via
    /// read_*_data / write_*_data. Direct read_*/write_* methods are
    /// cache-coherent (peek/update cache if line present) for hook use.
    dcache: Option<DCache>,

    /// ARC700 I-cache: 16 KB, 4-way set-associative, 32-byte lines.
    /// Instruction fetch goes through it. DMA writes invalidate covered lines.
    icache: Option<RefCell<ICache>>,
}

impl Memory {
    /// Create a flat memory (single address space, for tests).
    pub fn new(size: usize) -> Self {
        Self {
            data: vec![0u8; size],
            mmio: None,
            dccm_base: 0,
            app_size: None,
            app_load_base: 0,
            dccm_watchpoint: None,
            dcache: None,
            icache: None,
        }
    }

    /// Create a SoC memory with unified SRAM + MMIO controller.
    /// BCM55030 has 512 KB unified SRAM (no separate ICCM/DCCM).
    pub fn new_soc(sram_size: usize) -> Self {
        Self {
            data: vec![0u8; sram_size],
            mmio: Some(RefCell::new(MmioController::new())),
            dccm_base: 0,
            app_size: None,
            app_load_base: 0,
            dccm_watchpoint: None,
            dcache: Some(DCache::new()),
            icache: Some(RefCell::new(ICache::new())),
        }
    }

    /// Check DCCM write watchpoint. Logs writes that overlap with the watched word.
    fn check_watchpoint(&self, off: usize, size: usize) {
        if let Some(wp) = self.dccm_watchpoint {
            let wp_off = wp as usize;
            if off < wp_off + 4 && off + size > wp_off {
                let old = if wp_off + 3 < self.data.len() {
                    ((self.data[wp_off] as u32) << 24) | ((self.data[wp_off+1] as u32) << 16)
                    | ((self.data[wp_off+2] as u32) << 8) | (self.data[wp_off+3] as u32)
                } else { 0 };
                let (pc, blink) = self.mmio.as_ref()
                    .map(|m| { let b = m.borrow(); (b.current_pc, b.current_blink) })
                    .unwrap_or((0, 0));
                eprintln!(
                    "[WATCHPOINT] DCCM 0x{:05X} ({}B) hits watch 0x{:05X}, old=0x{:08X}, PC=0x{:05X}, blink=0x{:05X}",
                    off, size, wp_off, old, pc, blink
                );
            }
        }
    }

    pub fn is_soc(&self) -> bool {
        self.mmio.is_some()
    }

    pub fn sram_size(&self) -> usize {
        self.data.len()
    }

    /// Load a binary blob into SRAM (unified memory).
    pub fn load_binary(&mut self, addr: u32, binary: &[u8]) {
        let start = addr as usize;
        let end = start + binary.len();
        assert!(end <= self.data.len(), "binary exceeds memory size");
        self.data[start..end].copy_from_slice(binary);
    }

    /// Access the MMIO controller (SoC mode only).
    pub fn mmio(&self) -> Option<std::cell::RefMut<'_, MmioController>> {
        self.mmio.as_ref().map(|rc| rc.borrow_mut())
    }

    // ========== Flat mode bounds check ==========

    fn check_bounds(&self, addr: u32, size: u32) -> Result<(), Exception> {
        let end = (addr as u64) + (size as u64);
        if end > self.data.len() as u64 {
            return Err(Exception::MemoryError {
                address: addr,
                is_write: false,
            });
        }
        Ok(())
    }

    // ========== Data reads (SRAM in SoC mode, flat otherwise) ==========

    /// Check if an address falls in the SRAM range and return the offset.
    #[inline]
    fn sram_offset(&self, addr: u32) -> Option<usize> {
        if addr >= self.dccm_base {
            let off = (addr - self.dccm_base) as usize;
            if off < self.data.len() {
                return Some(off);
            }
        }
        None
    }

    /// Read byte from backing store (SRAM/MMIO) without checking D-cache.
    /// Used for .di bypass path and instruction fetch.
    fn read_byte_backing(&self, addr: u32) -> Result<u8, Exception> {
        if self.mmio.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                return Ok(self.data[off]);
            }
            if let Some(ref mmio) = self.mmio {
                return mmio.borrow_mut().read_byte(addr);
            }
            return Ok(0);
        }
        self.check_bounds(addr, 1)?;
        Ok(self.data[addr as usize])
    }

    /// Read byte — cache-coherent. If the address is in the D-cache,
    /// returns the cached value. Used by hooks and internal code that must
    /// see the same memory view as firmware LD instructions.
    pub fn read_byte(&self, addr: u32) -> Result<u8, Exception> {
        if self.mmio.is_some() {
            if let Some(ref dc) = self.dcache {
                if let Some(val) = dc.peek_byte(addr) {
                    return Ok(val);
                }
            }
            if let Some(off) = self.sram_offset(addr) {
                return Ok(self.data[off]);
            }
            // MMIO
            if let Some(ref mmio) = self.mmio {
                return mmio.borrow_mut().read_byte(addr);
            }
            // Unmapped address (boot ROM data, etc.) — return 0
            return Ok(0);
        }
        // Flat mode
        self.check_bounds(addr, 1)?;
        Ok(self.data[addr as usize])
    }

    /// Cache-coherent halfword read. Checks D-cache first, then backing store.
    pub fn read_half(&self, addr: u32) -> Result<u16, Exception> {
        // Check D-cache for coherence (hooks must see cached firmware data)
        if let Some(ref dc) = self.dcache {
            if let (Some(hi), Some(lo)) = (dc.peek_byte(addr), dc.peek_byte(addr + 1)) {
                return Ok(((hi as u16) << 8) | (lo as u16));
            }
        }
        if self.mmio.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                if off + 1 < self.data.len() {
                    return Ok(((self.data[off] as u16) << 8) | (self.data[off + 1] as u16));
                }
            }
            if addr & 1 != 0 {
                return Ok(0);
            }
            if let Some(ref mmio) = self.mmio {
                return mmio.borrow_mut().read_half(addr);
            }
            return Ok(0);
        }
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        self.check_bounds(addr, 2)?;
        let a = addr as usize;
        Ok(((self.data[a] as u16) << 8) | (self.data[a + 1] as u16))
    }

    /// Cache-coherent word read. Checks D-cache first, then backing store.
    pub fn read_word(&self, addr: u32) -> Result<u32, Exception> {
        // Check D-cache for coherence (hooks must see cached firmware data)
        if let Some(ref dc) = self.dcache {
            if let (Some(b0), Some(b1), Some(b2), Some(b3)) = (
                dc.peek_byte(addr),
                dc.peek_byte(addr.wrapping_add(1)),
                dc.peek_byte(addr.wrapping_add(2)),
                dc.peek_byte(addr.wrapping_add(3)),
            ) {
                return Ok(((b0 as u32) << 24) | ((b1 as u32) << 16)
                    | ((b2 as u32) << 8) | (b3 as u32));
            }
        }
        if self.mmio.is_some() {
            if addr & 3 != 0 {
                if let Some(off) = self.sram_offset(addr) {
                    if off + 3 < self.data.len() {
                        return Ok(((self.data[off] as u32) << 24)
                            | ((self.data[off + 1] as u32) << 16)
                            | ((self.data[off + 2] as u32) << 8)
                            | (self.data[off + 3] as u32));
                    }
                }
                return Ok(0);
            }
            if let Some(off) = self.sram_offset(addr) {
                if off + 3 < self.data.len() {
                    return Ok(((self.data[off] as u32) << 24)
                        | ((self.data[off + 1] as u32) << 16)
                        | ((self.data[off + 2] as u32) << 8)
                        | (self.data[off + 3] as u32));
                }
            }
            if let Some(ref mmio) = self.mmio {
                return mmio.borrow_mut().read_word(addr);
            }
            return Ok(0);
        }
        if addr & 3 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        self.check_bounds(addr, 4)?;
        let a = addr as usize;
        Ok(((self.data[a] as u32) << 24)
            | ((self.data[a + 1] as u32) << 16)
            | ((self.data[a + 2] as u32) << 8)
            | (self.data[a + 3] as u32))
    }

    // ========== Data writes (SRAM in SoC mode, flat otherwise) ==========

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if self.mmio.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                self.check_watchpoint(off, 1);
                self.data[off] = val;
                // Update D-cache for coherence (hooks/DMA write to SRAM directly)
                if let Some(ref mut dc) = self.dcache {
                    dc.write_byte(addr, val);
                }
                return Ok(());
            }
            if let Some(ref mmio) = self.mmio {
                return mmio.borrow_mut().write_byte(addr, val);
            }
            // Unmapped write — absorb silently
            return Ok(());
        }
        self.check_bounds(addr, 1)?;
        self.data[addr as usize] = val;
        Ok(())
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        if self.mmio.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                if off + 1 < self.data.len() {
                    self.check_watchpoint(off, 2);
                    self.data[off] = (val >> 8) as u8;
                    self.data[off + 1] = val as u8;
                    // Update D-cache for coherence
                    if let Some(ref mut dc) = self.dcache {
                        dc.write_byte(addr, (val >> 8) as u8);
                        dc.write_byte(addr + 1, val as u8);
                    }
                    return Ok(());
                }
            }
            if addr & 1 != 0 {
                return Ok(()); // Unmapped misaligned write — absorb
            }
            if let Some(ref mmio) = self.mmio {
                return mmio.borrow_mut().write_half(addr, val);
            }
            // Unmapped write — absorb silently
            return Ok(());
        }
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        self.check_bounds(addr, 2)?;
        let a = addr as usize;
        self.data[a] = (val >> 8) as u8;
        self.data[a + 1] = val as u8;
        Ok(())
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if self.mmio.is_some() {
            if addr & 3 != 0 {
                // Misaligned SRAM writes: perform byte-wise fixup
                if let Some(off) = self.sram_offset(addr) {
                    if off + 3 < self.data.len() {
                        self.check_watchpoint(off, 4);
                        self.data[off] = (val >> 24) as u8;
                        self.data[off + 1] = (val >> 16) as u8;
                        self.data[off + 2] = (val >> 8) as u8;
                        self.data[off + 3] = val as u8;
                        // Update D-cache for coherence
                        if let Some(ref mut dc) = self.dcache {
                            dc.write_byte(addr, (val >> 24) as u8);
                            dc.write_byte(addr + 1, (val >> 16) as u8);
                            dc.write_byte(addr + 2, (val >> 8) as u8);
                            dc.write_byte(addr + 3, val as u8);
                        }
                        return Ok(());
                    }
                }
                return Ok(()); // Unmapped misaligned write — absorb
            }
            if let Some(off) = self.sram_offset(addr) {
                if off + 3 < self.data.len() {
                    self.check_watchpoint(off, 4);
                    self.data[off] = (val >> 24) as u8;
                    self.data[off + 1] = (val >> 16) as u8;
                    self.data[off + 2] = (val >> 8) as u8;
                    self.data[off + 3] = val as u8;
                    // Update D-cache for coherence
                    if let Some(ref mut dc) = self.dcache {
                        dc.write_byte(addr, (val >> 24) as u8);
                        dc.write_byte(addr + 1, (val >> 16) as u8);
                        dc.write_byte(addr + 2, (val >> 8) as u8);
                        dc.write_byte(addr + 3, val as u8);
                    }
                    return Ok(());
                }
            }
            if let Some(ref mmio) = self.mmio {
                mmio.borrow_mut().write_word(addr, val)?;
                self.apply_pending_dma();
                return Ok(());
            }
            // Unmapped write — absorb silently
            return Ok(());
        }
        if addr & 3 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        self.check_bounds(addr, 4)?;
        let a = addr as usize;
        self.data[a] = (val >> 24) as u8;
        self.data[a + 1] = (val >> 16) as u8;
        self.data[a + 2] = (val >> 8) as u8;
        self.data[a + 3] = val as u8;
        Ok(())
    }

    /// Apply pending DMA transfers from the PBC.
    /// DMA writes bypass the D-cache, so we invalidate affected cache lines
    /// to prevent stale reads.
    fn apply_pending_dma(&mut self) {
        if let Some(ref mmio) = self.mmio {
            // Apply flash -> DCCM reads
            let writes = mmio.borrow_mut().pbc.take_pending_dma();
            for dma_write in writes {
                let start = dma_write.dccm_addr as usize;
                let end = start + dma_write.data.len();
                if end <= self.data.len() {
                    self.data[start..end].copy_from_slice(&dma_write.data);
                    // Invalidate D-cache lines covering the DMA range
                    if let Some(ref mut dc) = self.dcache {
                        let base = dma_write.dccm_addr & !0x1F;
                        let last = (dma_write.dccm_addr + dma_write.data.len() as u32).saturating_sub(1);
                        let mut addr = base;
                        while addr <= (last & !0x1F) {
                            dc.invalidate_line(addr);
                            addr += 32;
                        }
                    }
                    // Invalidate I-cache lines too (DMA modifies instructions)
                    if let Some(ref ic_cell) = self.icache {
                        let mut ic = ic_cell.borrow_mut();
                        let line_mask = (IC_LINE_SIZE as u32) - 1;
                        let base = dma_write.dccm_addr & !line_mask;
                        let last = (dma_write.dccm_addr + dma_write.data.len() as u32).saturating_sub(1);
                        let mut addr = base;
                        while addr <= (last & !line_mask) {
                            ic.invalidate_line(addr);
                            addr += IC_LINE_SIZE as u32;
                        }
                    }
                }
            }

            // Apply DCCM -> flash writes
            let flash_writes = mmio.borrow_mut().pbc.take_pending_flash_writes();
            for fw in flash_writes {
                let start = fw.dccm_addr as usize;
                let end = start + fw.length;
                if end <= self.data.len() {
                    let data = self.data[start..end].to_vec();
                    mmio.borrow_mut().pbc.complete_flash_write(fw.flash_addr, &data);
                }
            }
        }
    }

    // ========== D-cache data access (LD/ST instructions) ==========
    //
    // These methods route through the D-cache when:
    //   - cache_bypass is false (no .di flag on the instruction)
    //   - D-cache is present and enabled (DC_CTRL bit 0 = 0)
    // Otherwise they fall through to direct SRAM/MMIO access.
    //
    // Instruction fetch and DMA use the non-cached read_*/write_* methods above.

    /// Check if the D-cache is present and enabled.
    fn dcache_enabled(&self) -> bool {
        self.dcache.as_ref().map_or(false, |dc| dc.is_enabled())
    }

    /// Read a 32-byte cache line from the backing store (SRAM or MMIO),
    /// bypassing the D-cache. Used for cache fills on miss.
    fn read_line_from_backing(&self, line_addr: u32) -> Result<[u8; 32], Exception> {
        let mut data = [0u8; 32];
        if let Some(off) = self.sram_offset(line_addr) {
            // SRAM path — fast bulk copy
            let end = (off + 32).min(self.data.len());
            let count = end - off;
            data[..count].copy_from_slice(&self.data[off..end]);
        } else if let Some(ref mmio) = self.mmio {
            // MMIO path — byte-by-byte (each read may have side effects)
            for i in 0..32u32 {
                data[i as usize] = mmio.borrow_mut().read_byte(line_addr + i)?;
            }
        }
        Ok(data)
    }

    /// Write a 32-byte evicted dirty cache line back to the backing store.
    fn writeback_line(&mut self, line_addr: u32, data: &[u8; 32]) -> Result<(), Exception> {
        if let Some(off) = self.sram_offset(line_addr) {
            // SRAM path — fast bulk copy
            let end = (off + 32).min(self.data.len());
            let count = end - off;
            self.data[off..end].copy_from_slice(&data[..count]);
        } else if let Some(ref mmio) = self.mmio {
            for i in 0..32u32 {
                mmio.borrow_mut().write_byte(line_addr + i, data[i as usize])?;
            }
        }
        Ok(())
    }

    /// Ensure a cache line containing `addr` is loaded. Fills on miss,
    /// writes back any evicted dirty line.
    fn ensure_cache_line(&mut self, addr: u32) -> Result<(), Exception> {
        if self.dcache.as_ref().unwrap().contains(addr) {
            return Ok(());
        }
        // Miss: read line from backing store
        let line_addr = addr & !0x1F;
        let line_data = self.read_line_from_backing(line_addr)?;

        // Fill cache (may evict a dirty line)
        let evicted = self.dcache.as_mut().unwrap().fill_line(addr, &line_data);
        if let Some(ev) = evicted {
            self.writeback_line(ev.addr, &ev.data)?;
        }
        Ok(())
    }

    /// Data byte read through D-cache. Used by LD instructions.
    /// When cache_bypass (.di) is set, reads from backing store directly.
    pub fn read_byte_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u8, Exception> {
        if cache_bypass || !self.dcache_enabled() {
            return self.read_byte_backing(addr);
        }
        self.ensure_cache_line(addr)?;
        Ok(self.dcache.as_mut().unwrap().read_byte(addr).unwrap())
    }

    /// Data halfword read through D-cache. Used by LD.H instructions.
    pub fn read_half_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u16, Exception> {
        if cache_bypass || addr & 1 != 0 || !self.dcache_enabled() {
            // .di bypass or misaligned: read from backing store directly
            let b0 = self.read_byte_backing(addr)?;
            let b1 = self.read_byte_backing(addr.wrapping_add(1))?;
            return Ok(((b0 as u16) << 8) | (b1 as u16));
        }
        self.ensure_cache_line(addr)?;
        let dc = self.dcache.as_mut().unwrap();
        let hi = dc.read_byte(addr).unwrap() as u16;
        let lo = dc.read_byte(addr + 1).unwrap() as u16;
        Ok((hi << 8) | lo)
    }

    /// Data word read through D-cache. Used by LD instructions.
    pub fn read_word_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u32, Exception> {
        if cache_bypass || addr & 3 != 0 || !self.dcache_enabled() {
            // .di bypass or misaligned: read from backing store directly
            let b0 = self.read_byte_backing(addr)? as u32;
            let b1 = self.read_byte_backing(addr.wrapping_add(1))? as u32;
            let b2 = self.read_byte_backing(addr.wrapping_add(2))? as u32;
            let b3 = self.read_byte_backing(addr.wrapping_add(3))? as u32;
            return Ok((b0 << 24) | (b1 << 16) | (b2 << 8) | b3);
        }
        self.ensure_cache_line(addr)?;
        let dc = self.dcache.as_mut().unwrap();
        let b0 = dc.read_byte(addr).unwrap() as u32;
        let b1 = dc.read_byte(addr + 1).unwrap() as u32;
        let b2 = dc.read_byte(addr + 2).unwrap() as u32;
        let b3 = dc.read_byte(addr + 3).unwrap() as u32;
        Ok((b0 << 24) | (b1 << 16) | (b2 << 8) | b3)
    }

    /// Data byte write through D-cache. Used by ST instructions.
    /// HW-faithful write-back, write-allocate: data stays in cache until
    /// eviction/invalidation. With .di (cache_bypass), writes directly to
    /// backing store without touching the cache (scan7b test 7 verified:
    /// .di stores do NOT update or invalidate cached copies).
    pub fn write_byte_data(&mut self, addr: u32, val: u8, cache_bypass: bool) -> Result<(), Exception> {
        if cache_bypass || !self.dcache_enabled() {
            return self.write_byte(addr, val);
        }
        self.ensure_cache_line(addr)?;
        self.dcache.as_mut().unwrap().write_byte(addr, val);
        Ok(())
    }

    /// Data halfword write through D-cache. Used by ST.H instructions.
    pub fn write_half_data(&mut self, addr: u32, val: u16, cache_bypass: bool) -> Result<(), Exception> {
        if cache_bypass || addr & 1 != 0 || !self.dcache_enabled() {
            return self.write_half(addr, val);
        }
        self.ensure_cache_line(addr)?;
        let dc = self.dcache.as_mut().unwrap();
        dc.write_byte(addr, (val >> 8) as u8);
        dc.write_byte(addr + 1, val as u8);
        Ok(())
    }

    /// Data word write through D-cache. Used by ST instructions.
    pub fn write_word_data(&mut self, addr: u32, val: u32, cache_bypass: bool) -> Result<(), Exception> {
        if cache_bypass || addr & 3 != 0 || !self.dcache_enabled() {
            return self.write_word(addr, val);
        }
        self.ensure_cache_line(addr)?;
        let dc = self.dcache.as_mut().unwrap();
        dc.write_byte(addr, (val >> 24) as u8);
        dc.write_byte(addr + 1, (val >> 16) as u8);
        dc.write_byte(addr + 2, (val >> 8) as u8);
        dc.write_byte(addr + 3, val as u8);
        // Write-through for SRAM
        if let Some(off) = self.sram_offset(addr) {
            if off + 3 < self.data.len() {
                self.data[off] = (val >> 24) as u8;
                self.data[off + 1] = (val >> 16) as u8;
                self.data[off + 2] = (val >> 8) as u8;
                self.data[off + 3] = val as u8;
            }
        }
        Ok(())
    }

    // ========== D-cache control (DC_CTRL / DC_IVDC) ==========

    /// Read DC_CTRL (aux 0x48) from the D-cache model.
    /// Returns the BCM55030 reset value 0xC2 if no cache is present.
    pub fn dcache_read_ctrl(&self) -> u32 {
        self.dcache.as_ref().map_or(0xC2, |dc| dc.read_dc_ctrl())
    }

    /// Sync DC_CTRL (aux 0x48) write to the D-cache model.
    pub fn dcache_sync_ctrl(&mut self, val: u32) {
        if let Some(ref mut dc) = self.dcache {
            dc.write_dc_ctrl(val);
        }
    }

    /// Invalidate entire D-cache (DC_IVDC aux 0x47 write).
    /// Dirty lines are flushed to backing store if IM=1 in DC_CTRL.
    pub fn dcache_invalidate_all(&mut self) -> Result<(), Exception> {
        let evicted = if let Some(ref mut dc) = self.dcache {
            dc.invalidate_all()
        } else {
            Vec::new()
        };
        for ev in evicted {
            self.writeback_line(ev.addr, &ev.data)?;
        }
        Ok(())
    }

    /// Invalidate a single D-cache line (DC_IVDL aux 0x4A write).
    pub fn dcache_invalidate_line(&mut self, addr: u32) -> Result<(), Exception> {
        let evicted = if let Some(ref mut dc) = self.dcache {
            dc.invalidate_line(addr)
        } else {
            None
        };
        if let Some(ev) = evicted {
            self.writeback_line(ev.addr, &ev.data)?;
        }
        Ok(())
    }

    /// Set DC_RAM_ADDR (aux 0x58) on the D-cache for direct probe.
    pub fn dcache_set_ram_addr(&mut self, addr: u32) {
        if let Some(ref mut dc) = self.dcache {
            dc.set_ram_addr(addr);
        }
    }

    /// Read DC_TAG (aux 0x59) — returns (line_base | valid_bit) for probe address.
    pub fn dcache_read_tag(&self) -> u32 {
        self.dcache.as_ref().map_or(0, |dc| dc.read_tag())
    }

    /// Read DC_DATA (aux 0x5B) — returns 32-bit word at probe address.
    pub fn dcache_read_data(&self) -> u32 {
        self.dcache.as_ref().map_or(0, |dc| dc.read_data())
    }

    // ========== I-cache control ==========

    /// Invalidate entire I-cache (IC_IVIC aux 0x10).
    pub fn icache_invalidate_all(&self) {
        if let Some(ref ic) = self.icache {
            ic.borrow_mut().invalidate_all();
        }
    }

    /// Invalidate single I-cache line (IC_IVIL aux 0x19).
    pub fn icache_invalidate_line(&self, addr: u32) {
        if let Some(ref ic) = self.icache {
            ic.borrow_mut().invalidate_line(addr);
        }
    }

    /// Ensure an I-cache line containing `addr` is loaded from SRAM.
    /// Called on I-cache miss during instruction fetch.
    fn icache_fill(&self, addr: u32) {
        if let Some(ref ic_cell) = self.icache {
            let mut ic = ic_cell.borrow_mut();
            if ic.contains(addr) {
                return;
            }
            // Read line from SRAM (no MMIO for instruction fetch)
            let line_addr = addr & !((IC_LINE_SIZE as u32) - 1);
            let mut data = [0u8; IC_LINE_SIZE];
            if let Some(off) = self.sram_offset(line_addr) {
                let end = (off + IC_LINE_SIZE).min(self.data.len());
                let count = end - off;
                data[..count].copy_from_slice(&self.data[off..end]);
            }
            ic.fill_line(addr, &data);
        }
    }

    // ========== Instruction fetch (goes through I-cache in SoC mode) ==========

    pub fn fetch_half(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        if self.mmio.is_some() {
            // Try I-cache first
            if let Some(ref ic_cell) = self.icache {
                let enabled = ic_cell.borrow().is_enabled();
                if enabled {
                    // Fast path: hit
                    {
                        let ic = ic_cell.borrow();
                        if let Some(val) = ic.peek_half(addr) {
                            return Ok(val);
                        }
                    }
                    // Miss: fill from SRAM and retry
                    self.icache_fill(addr);
                    let ic = ic_cell.borrow();
                    if let Some(val) = ic.peek_half(addr) {
                        return Ok(val);
                    }
                }
            }
            // Fallback: direct SRAM read (I-cache disabled or fill failed)
            if let Some(a) = self.sram_offset(addr) {
                if a + 1 < self.data.len() {
                    return Ok(((self.data[a] as u16) << 8) | (self.data[a + 1] as u16));
                }
            }
            return Err(Exception::MemoryError { address: addr, is_write: false });
        }
        self.check_bounds(addr, 2)?;
        let a = addr as usize;
        Ok(((self.data[a] as u16) << 8) | (self.data[a + 1] as u16))
    }

    pub fn fetch_word(&self, addr: u32) -> Result<u32, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        if self.mmio.is_some() {
            // Try I-cache first. A 32-bit word spanning a 32B line boundary
            // requires two line fetches — handle that case via halfwords.
            if let Some(ref ic_cell) = self.icache {
                let enabled = ic_cell.borrow().is_enabled();
                if enabled {
                    let line_mask = (IC_LINE_SIZE as u32) - 1;
                    let line_end = (addr & !line_mask) + IC_LINE_SIZE as u32;
                    if addr + 4 <= line_end {
                        // Word fits in one line
                        {
                            let ic = ic_cell.borrow();
                            if let Some(val) = ic.peek_word(addr) {
                                return Ok(val);
                            }
                        }
                        self.icache_fill(addr);
                        let ic = ic_cell.borrow();
                        if let Some(val) = ic.peek_word(addr) {
                            return Ok(val);
                        }
                    } else {
                        // Cross-line: fetch halfwords separately
                        let hi = self.fetch_half(addr)? as u32;
                        let lo = self.fetch_half(addr + 2)? as u32;
                        return Ok((hi << 16) | lo);
                    }
                }
            }
            // Fallback: direct SRAM read
            if let Some(a) = self.sram_offset(addr) {
                if a + 3 < self.data.len() {
                    return Ok(((self.data[a] as u32) << 24)
                        | ((self.data[a + 1] as u32) << 16)
                        | ((self.data[a + 2] as u32) << 8)
                        | (self.data[a + 3] as u32));
                }
            }
            return Err(Exception::MemoryError { address: addr, is_write: false });
        }
        self.check_bounds(addr, 4)?;
        let a = addr as usize;
        Ok(((self.data[a] as u32) << 24)
            | ((self.data[a + 1] as u32) << 16)
            | ((self.data[a + 2] as u32) << 8)
            | (self.data[a + 3] as u32))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_big_endian_word() {
        let mut mem = Memory::new(16);
        mem.write_word(0, 0xDEADBEEF).unwrap();
        assert_eq!(mem.data[0], 0xDE);
        assert_eq!(mem.data[1], 0xAD);
        assert_eq!(mem.data[2], 0xBE);
        assert_eq!(mem.data[3], 0xEF);
        assert_eq!(mem.read_word(0).unwrap(), 0xDEADBEEF);
    }

    #[test]
    fn test_big_endian_half() {
        let mut mem = Memory::new(16);
        mem.write_half(0, 0xCAFE).unwrap();
        assert_eq!(mem.data[0], 0xCA);
        assert_eq!(mem.data[1], 0xFE);
        assert_eq!(mem.read_half(0).unwrap(), 0xCAFE);
    }

    #[test]
    fn test_byte() {
        let mut mem = Memory::new(16);
        mem.write_byte(5, 0x42).unwrap();
        assert_eq!(mem.read_byte(5).unwrap(), 0x42);
    }

    #[test]
    fn test_misaligned_word() {
        let mem = Memory::new(16);
        assert!(mem.read_word(1).is_err());
        assert!(mem.read_word(2).is_err());
        assert!(mem.read_word(3).is_err());
    }

    #[test]
    fn test_misaligned_half() {
        let mem = Memory::new(16);
        assert!(mem.read_half(1).is_err());
    }

    #[test]
    fn test_load_binary() {
        let mut mem = Memory::new(16);
        mem.load_binary(0, &[0x20, 0x00, 0x08, 0x00]);
        assert_eq!(mem.read_word(0).unwrap(), 0x20000800);
    }

    #[test]
    fn test_unified_sram() {
        let mut mem = Memory::new_soc(1024);
        // Write data to SRAM
        mem.load_binary(0, &[0xAA, 0xBB, 0xCC, 0xDD]);
        // With unified SRAM, fetch and data read see the same backing store
        assert_eq!(mem.fetch_half(0).unwrap(), 0xAABB);
        assert_eq!(mem.fetch_word(0).unwrap(), 0xAABBCCDD);
        assert_eq!(mem.read_half(0).unwrap(), 0xAABB);
        assert_eq!(mem.read_word(0).unwrap(), 0xAABBCCDD);
    }

    #[test]
    fn test_soc_mmio_stub() {
        let mem = Memory::new_soc(1024);
        // MMIO reads return 0 for unstubbed addresses
        assert_eq!(mem.read_byte(0x00FC0000).unwrap(), 0);
        assert_eq!(mem.read_word(0xDE000000).unwrap(), 0);
    }

    // ========== D-cache integration tests ==========

    // ========== D-cache integration tests ==========
    //
    // Use new_soc(4096) so SRAM covers addresses 0x000-0xFFF.

    #[test]
    fn test_dcache_sram_write_through() {
        let mut mem = Memory::new_soc(4096);
        // Write 0xDEADBEEF to SRAM address 0x100 through D-cache (no .di)
        mem.write_word_data(0x100, 0xDEADBEEF, false).unwrap();

        // Read back through cache — should return cached value
        assert_eq!(mem.read_word_data(0x100, false).unwrap(), 0xDEADBEEF);

        // Write-through: SRAM backing store is also updated
        // .di bypass reads from raw SRAM and sees the correct value
        assert_eq!(mem.read_word_data(0x100, true).unwrap(), 0xDEADBEEF);

        // Direct read (cache-coherent) also sees it
        assert_eq!(mem.read_word(0x100).unwrap(), 0xDEADBEEF);
    }

    #[test]
    fn test_dcache_bypass_writes_to_sram() {
        let mut mem = Memory::new_soc(4096);
        // Write with .di — bypasses cache, goes directly to SRAM
        mem.write_word_data(0x200, 0xCAFEBABE, true).unwrap();

        // SRAM has the value
        assert_eq!(mem.read_word(0x200).unwrap(), 0xCAFEBABE);

        // Read without .di fills cache from SRAM
        assert_eq!(mem.read_word_data(0x200, false).unwrap(), 0xCAFEBABE);
    }

    #[test]
    fn test_dcache_invalidate_flushes_dirty() {
        let mut mem = Memory::new_soc(4096);
        // Write to SRAM through cache (write-through: also updates SRAM)
        mem.write_word_data(0x300, 0x12345678, false).unwrap();
        // All reads see the value (write-through keeps SRAM in sync)
        assert_eq!(mem.read_word(0x300).unwrap(), 0x12345678);
        assert_eq!(mem.read_word_data(0x300, true).unwrap(), 0x12345678);

        // Invalidate all (IM=1 by default → flush dirty lines)
        mem.dcache_invalidate_all().unwrap();

        // Now SRAM should have the value (flushed from cache)
        assert_eq!(mem.read_word(0x300).unwrap(), 0x12345678);

        // Cache is empty — reading without .di fills fresh from SRAM
        assert_eq!(mem.read_word_data(0x300, false).unwrap(), 0x12345678);
    }

    #[test]
    fn test_dcache_disable_bypasses_all() {
        let mut mem = Memory::new_soc(4096);
        // Disable cache via DC_CTRL (bit 0 = 1 → disabled)
        mem.dcache_sync_ctrl(0xC3);

        // Write without .di — should go directly to SRAM (cache disabled)
        mem.write_word_data(0x100, 0xAAAAAAAA, false).unwrap();
        assert_eq!(mem.read_word(0x100).unwrap(), 0xAAAAAAAA);

        // Re-enable cache
        mem.dcache_sync_ctrl(0xC2);

        // Now writes go to cache
        mem.write_word_data(0x100, 0xBBBBBBBB, false).unwrap();
        // Direct read is cache-coherent — sees the new cached value
        assert_eq!(mem.read_word(0x100).unwrap(), 0xBBBBBBBB);
        assert_eq!(mem.read_word_data(0x100, false).unwrap(), 0xBBBBBBBB);
    }

    #[test]
    fn test_dcache_byte_and_half() {
        let mut mem = Memory::new_soc(4096);
        // Byte write through cache
        mem.write_byte_data(0x200, 0xAB, false).unwrap();
        assert_eq!(mem.read_byte_data(0x200, false).unwrap(), 0xAB);
        // Direct read is cache-coherent
        assert_eq!(mem.read_byte(0x200).unwrap(), 0xAB);

        // Half write through cache (same 16-byte line as 0x200)
        mem.write_half_data(0x202, 0xCDEF, false).unwrap();
        assert_eq!(mem.read_half_data(0x202, false).unwrap(), 0xCDEF);
        assert_eq!(mem.read_half(0x202).unwrap(), 0xCDEF);

        // Flush to SRAM backing store
        mem.dcache_invalidate_all().unwrap();
        assert_eq!(mem.read_byte(0x200).unwrap(), 0xAB);
        assert_eq!(mem.read_half(0x202).unwrap(), 0xCDEF);
    }

    #[test]
    fn test_dcache_flat_mode_no_cache() {
        let mut mem = Memory::new(1024);
        // Flat mode (no DCache) — all accesses go directly to memory
        mem.write_word_data(0x0, 0xDEADBEEF, false).unwrap();
        assert_eq!(mem.read_word_data(0x0, false).unwrap(), 0xDEADBEEF);
        // Direct read also sees it (no cache layer)
        assert_eq!(mem.read_word(0x0).unwrap(), 0xDEADBEEF);
    }

    #[test]
    fn test_dcache_dma_stale_then_bypass() {
        let mut mem = Memory::new_soc(4096);
        // Fill cache with SRAM data at address 0x100
        mem.load_binary(0x100, &[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(mem.read_word_data(0x100, false).unwrap(), 0x11223344);

        // Direct SRAM overwrite (simulating DMA without cache invalidation)
        // load_binary writes to raw SRAM, does NOT update cache
        mem.data[0x100] = 0xAA;
        mem.data[0x101] = 0xBB;
        mem.data[0x102] = 0xCC;
        mem.data[0x103] = 0xDD;

        // Cache still holds the OLD value (stale) — non-.di read hits cache
        assert_eq!(mem.read_word_data(0x100, false).unwrap(), 0x11223344);

        // .di bypass reads raw SRAM — sees the NEW DMA value
        assert_eq!(mem.read_word_data(0x100, true).unwrap(), 0xAABBCCDD);

        // Cache-coherent read_word sees the cached (stale) value
        assert_eq!(mem.read_word(0x100).unwrap(), 0x11223344);
    }
}
