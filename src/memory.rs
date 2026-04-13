use std::cell::RefCell;
use std::sync::Arc;

use parking_lot::RwLock;

use crate::cache::{DCache, ICache, IC_LINE_SIZE};
use crate::cpu::exception::Exception;
use crate::soc::bank::{BootMode, PeripheralBank};
use crate::soc::peripheral::DatapathOp;

/// BCM55030 unified SRAM: 512 KB. No ICCM/DCCM.
pub const SRAM_SIZE: usize = 512 * 1024;

pub struct Memory {
    /// Unified SRAM (SoC mode) / flat memory (tests). **Lock-free**:
    /// the CPU hot path touches `data` directly without acquiring any
    /// lock. Only MMIO accesses go through the peripheral bank lock.
    data: Vec<u8>,

    /// Shared peripheral bank. `None` in flat test mode.
    bank: Option<Arc<RwLock<PeripheralBank>>>,

    /// SRAM base addr. Access in [dccm_base, dccm_base+sram_size) hits SRAM.
    pub dccm_base: u32,

    pub dccm_watchpoint: Option<u32>,

    /// D-cache: 4 KB, 2-way, 32 B lines. LD/ST path via `read_*_data`/`write_*_data`.
    dcache: Option<DCache>,

    /// I-cache: 4 KB, 1-way, 32 B lines. Instruction fetch path.
    icache: Option<RefCell<ICache>>,

    /// Audit 2.2: policy for MMIO reads / writes that do not match any
    /// peripheral claim. `false` (default) returns zero so existing
    /// code that never probes unmapped addresses stays unaffected.
    /// `true` returns [`Exception::MemoryError`] to surface hidden
    /// firmware accesses — enable with `--unmapped-exception` to
    /// discover new unmodelled registers.
    pub unmapped_exception: bool,
}

impl Memory {
    /// Create a flat memory (single address space, for tests).
    pub fn new(size: usize) -> Self {
        Self {
            data: vec![0u8; size],
            bank: None,
            dccm_base: 0,
            dccm_watchpoint: None,
            dcache: None,
            icache: None,
            unmapped_exception: false,
        }
    }

    /// Create a SoC memory with unified SRAM + peripheral bank.
    /// BCM55030 has 512 KB unified SRAM (no separate ICCM/DCCM).
    pub fn new_soc(sram_size: usize, boot_mode: BootMode) -> Self {
        let bank = Arc::new(RwLock::new(PeripheralBank::new(boot_mode)));
        Self {
            data: vec![0u8; sram_size],
            bank: Some(bank),
            dccm_base: 0,
            dccm_watchpoint: None,
            dcache: Some(DCache::new()),
            icache: Some(RefCell::new(ICache::new())),
            unmapped_exception: false,
        }
    }

    /// Access the shared peripheral bank handle. Callers that need to
    /// read / write the bank acquire the lock themselves.
    pub fn bank(&self) -> Option<&Arc<RwLock<PeripheralBank>>> {
        self.bank.as_ref()
    }

    fn check_watchpoint(&self, off: usize, size: usize) {
        if let Some(wp) = self.dccm_watchpoint {
            let wp_off = wp as usize;
            if off < wp_off + 4 && off + size > wp_off {
                let old = if wp_off + 3 < self.data.len() {
                    ((self.data[wp_off] as u32) << 24)
                        | ((self.data[wp_off + 1] as u32) << 16)
                        | ((self.data[wp_off + 2] as u32) << 8)
                        | (self.data[wp_off + 3] as u32)
                } else {
                    0
                };
                let (pc, blink) = if let Some(ref bank) = self.bank {
                    let g = bank.read();
                    (g.current_pc, g.current_blink)
                } else {
                    (0, 0)
                };
                eprintln!(
                    "[WATCHPOINT] DCCM 0x{:05X} ({}B) hits watch 0x{:05X}, old=0x{:08X}, PC=0x{:05X}, blink=0x{:05X}",
                    off, size, wp_off, old, pc, blink
                );
            }
        }
    }

    pub fn is_soc(&self) -> bool {
        self.bank.is_some()
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

    fn read_byte_backing(&self, addr: u32) -> Result<u8, Exception> {
        if self.bank.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                return Ok(self.data[off]);
            }
            if let Some(ref bank) = self.bank {
                return bank.write().read_byte(addr);
            }
            return Ok(0);
        }
        self.check_bounds(addr, 1)?;
        Ok(self.data[addr as usize])
    }

    pub fn read_byte(&self, addr: u32) -> Result<u8, Exception> {
        if self.bank.is_some() {
            if let Some(ref dc) = self.dcache {
                if let Some(val) = dc.peek_byte(addr) {
                    return Ok(val);
                }
            }
            if let Some(off) = self.sram_offset(addr) {
                return Ok(self.data[off]);
            }
            if let Some(ref bank) = self.bank {
                return bank.write().read_byte(addr);
            }
            return Ok(0);
        }
        self.check_bounds(addr, 1)?;
        Ok(self.data[addr as usize])
    }

    pub fn read_half(&self, addr: u32) -> Result<u16, Exception> {
        if let Some(ref dc) = self.dcache {
            if let (Some(hi), Some(lo)) = (dc.peek_byte(addr), dc.peek_byte(addr + 1)) {
                return Ok(((hi as u16) << 8) | (lo as u16));
            }
        }
        if self.bank.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                if off + 1 < self.data.len() {
                    return Ok(((self.data[off] as u16) << 8) | (self.data[off + 1] as u16));
                }
            }
            if addr & 1 != 0 {
                return Ok(0);
            }
            if let Some(ref bank) = self.bank {
                return bank.write().read_half(addr);
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

    pub fn read_word(&self, addr: u32) -> Result<u32, Exception> {
        if let Some(ref dc) = self.dcache {
            if let (Some(b0), Some(b1), Some(b2), Some(b3)) = (
                dc.peek_byte(addr),
                dc.peek_byte(addr.wrapping_add(1)),
                dc.peek_byte(addr.wrapping_add(2)),
                dc.peek_byte(addr.wrapping_add(3)),
            ) {
                return Ok(((b0 as u32) << 24)
                    | ((b1 as u32) << 16)
                    | ((b2 as u32) << 8)
                    | (b3 as u32));
            }
        }
        if self.bank.is_some() {
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
            if let Some(ref bank) = self.bank {
                return bank.write().read_word(addr);
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

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if self.bank.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                self.check_watchpoint(off, 1);
                self.data[off] = val;
                if let Some(ref mut dc) = self.dcache {
                    dc.write_byte(addr, val);
                }
                return Ok(());
            }
            if let Some(ref bank) = self.bank {
                bank.write().write_byte(addr, val)?;
                self.drain_datapath()?;
                return Ok(());
            }
            return Ok(());
        }
        self.check_bounds(addr, 1)?;
        self.data[addr as usize] = val;
        Ok(())
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        if self.bank.is_some() {
            if let Some(off) = self.sram_offset(addr) {
                if off + 1 < self.data.len() {
                    self.check_watchpoint(off, 2);
                    self.data[off] = (val >> 8) as u8;
                    self.data[off + 1] = val as u8;
                    if let Some(ref mut dc) = self.dcache {
                        dc.write_byte(addr, (val >> 8) as u8);
                        dc.write_byte(addr + 1, val as u8);
                    }
                    return Ok(());
                }
            }
            if addr & 1 != 0 {
                return Ok(());
            }
            if let Some(ref bank) = self.bank {
                bank.write().write_half(addr, val)?;
                self.drain_datapath()?;
                return Ok(());
            }
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
        if self.bank.is_some() {
            if addr & 3 != 0 {
                if let Some(off) = self.sram_offset(addr) {
                    if off + 3 < self.data.len() {
                        self.check_watchpoint(off, 4);
                        self.data[off] = (val >> 24) as u8;
                        self.data[off + 1] = (val >> 16) as u8;
                        self.data[off + 2] = (val >> 8) as u8;
                        self.data[off + 3] = val as u8;
                        if let Some(ref mut dc) = self.dcache {
                            dc.write_byte(addr, (val >> 24) as u8);
                            dc.write_byte(addr + 1, (val >> 16) as u8);
                            dc.write_byte(addr + 2, (val >> 8) as u8);
                            dc.write_byte(addr + 3, val as u8);
                        }
                        return Ok(());
                    }
                }
                return Ok(());
            }
            if let Some(off) = self.sram_offset(addr) {
                if off + 3 < self.data.len() {
                    self.check_watchpoint(off, 4);
                    self.data[off] = (val >> 24) as u8;
                    self.data[off + 1] = (val >> 16) as u8;
                    self.data[off + 2] = (val >> 8) as u8;
                    self.data[off + 3] = val as u8;
                    if let Some(ref mut dc) = self.dcache {
                        dc.write_byte(addr, (val >> 24) as u8);
                        dc.write_byte(addr + 1, (val >> 16) as u8);
                        dc.write_byte(addr + 2, (val >> 8) as u8);
                        dc.write_byte(addr + 3, val as u8);
                    }
                    return Ok(());
                }
            }
            if let Some(ref bank) = self.bank {
                bank.write().write_word(addr, val)?;
                self.drain_datapath()?;
                return Ok(());
            }
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

    /// Pull pending `DatapathOp`s from the bank and apply them to SRAM
    /// / flash. Called after any MMIO write that might have triggered a
    /// DMA transfer (PBC flash DMA is the only current source).
    fn drain_datapath(&mut self) -> Result<(), Exception> {
        let ops = if let Some(ref bank) = self.bank {
            bank.write().take_pending_datapath()
        } else {
            return Ok(());
        };
        for op in ops {
            self.apply_datapath_op(op)?;
        }
        Ok(())
    }

    fn apply_datapath_op(&mut self, op: DatapathOp) -> Result<(), Exception> {
        match op {
            DatapathOp::SramWrite { sram_addr, data } => {
                let start = sram_addr as usize;
                let end = start + data.len();
                if end <= self.data.len() {
                    self.data[start..end].copy_from_slice(&data);
                    if let Some(ref mut dc) = self.dcache {
                        let base = sram_addr & !0x1F;
                        let last = (sram_addr + data.len() as u32).saturating_sub(1);
                        let mut addr = base;
                        while addr <= (last & !0x1F) {
                            dc.invalidate_line(addr);
                            addr += 32;
                        }
                    }
                    if let Some(ref ic_cell) = self.icache {
                        let mut ic = ic_cell.borrow_mut();
                        let line_mask = (IC_LINE_SIZE as u32) - 1;
                        let base = sram_addr & !line_mask;
                        let last = (sram_addr + data.len() as u32).saturating_sub(1);
                        let mut addr = base;
                        while addr <= (last & !line_mask) {
                            ic.invalidate_line(addr);
                            addr += IC_LINE_SIZE as u32;
                        }
                    }
                }
            }
            DatapathOp::FlashWrite {
                peripheral,
                flash_addr,
                sram_addr,
                length,
            } => {
                let start = sram_addr as usize;
                let end = start + length;
                if end <= self.data.len() {
                    let data = self.data[start..end].to_vec();
                    if let Some(ref bank) = self.bank {
                        bank.write().complete_flash_write(peripheral, flash_addr, &data);
                    }
                }
            }
        }
        Ok(())
    }

    // D-cache data path: LD/ST route here when cache_bypass=false and DC is enabled.

    fn dcache_enabled(&self) -> bool {
        self.dcache.as_ref().map_or(false, |dc| dc.is_enabled())
    }

    fn read_line_from_backing(&self, line_addr: u32) -> Result<[u8; 32], Exception> {
        let mut data = [0u8; 32];
        if let Some(off) = self.sram_offset(line_addr) {
            let end = (off + 32).min(self.data.len());
            let count = end - off;
            data[..count].copy_from_slice(&self.data[off..end]);
        } else if let Some(ref bank) = self.bank {
            let mut guard = bank.write();
            for i in 0..32u32 {
                data[i as usize] = guard.read_byte(line_addr + i)?;
            }
        }
        Ok(data)
    }

    fn writeback_line(&mut self, line_addr: u32, data: &[u8; 32]) -> Result<(), Exception> {
        if let Some(off) = self.sram_offset(line_addr) {
            let end = (off + 32).min(self.data.len());
            let count = end - off;
            self.data[off..end].copy_from_slice(&data[..count]);
        } else if let Some(ref bank) = self.bank {
            let mut guard = bank.write();
            for i in 0..32u32 {
                guard.write_byte(line_addr + i, data[i as usize])?;
            }
        }
        Ok(())
    }

    fn ensure_cache_line(&mut self, addr: u32) -> Result<(), Exception> {
        if self.dcache.as_ref().unwrap().contains(addr) {
            return Ok(());
        }
        let line_addr = addr & !0x1F;
        let line_data = self.read_line_from_backing(line_addr)?;
        let evicted = self.dcache.as_mut().unwrap().fill_line(addr, &line_data);
        if let Some(ev) = evicted {
            self.writeback_line(ev.addr, &ev.data)?;
        }
        Ok(())
    }

    pub fn read_byte_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u8, Exception> {
        if cache_bypass || !self.dcache_enabled() {
            return self.read_byte_backing(addr);
        }
        self.ensure_cache_line(addr)?;
        Ok(self.dcache.as_mut().unwrap().read_byte(addr).unwrap())
    }

    pub fn read_half_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u16, Exception> {
        if cache_bypass || addr & 1 != 0 || !self.dcache_enabled() {
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

    pub fn read_word_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u32, Exception> {
        // Audit 2.1 / deferral D5 resolved 2026-04-13: bare-metal
        // scan `tmp/hello-bare/scan_misalign.c` on a live BCM55030
        // confirmed the CPU silently fixes up misaligned word /
        // half reads and writes at any byte offset — no exception
        // is raised. The byte-by-byte path below is HW-faithful,
        // not a workaround.
        if cache_bypass || addr & 3 != 0 || !self.dcache_enabled() {
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

    pub fn write_byte_data(&mut self, addr: u32, val: u8, cache_bypass: bool) -> Result<(), Exception> {
        if cache_bypass || !self.dcache_enabled() {
            return self.write_byte(addr, val);
        }
        self.ensure_cache_line(addr)?;
        self.dcache.as_mut().unwrap().write_byte(addr, val);
        Ok(())
    }

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

    pub fn dcache_read_ctrl(&self) -> u32 {
        self.dcache.as_ref().map_or(0xC2, |dc| dc.read_dc_ctrl())
    }

    pub fn dcache_sync_ctrl(&mut self, val: u32) {
        if let Some(ref mut dc) = self.dcache {
            dc.write_dc_ctrl(val);
        }
    }

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

    pub fn dcache_set_ram_addr(&mut self, addr: u32) {
        if let Some(ref mut dc) = self.dcache {
            dc.set_ram_addr(addr);
        }
    }

    pub fn dcache_read_tag(&self) -> u32 {
        self.dcache.as_ref().map_or(0, |dc| dc.read_tag())
    }

    pub fn dcache_read_data(&self) -> u32 {
        self.dcache.as_ref().map_or(0, |dc| dc.read_data())
    }

    // ========== I-cache control ==========

    pub fn icache_invalidate_all(&self) {
        if let Some(ref ic) = self.icache {
            ic.borrow_mut().invalidate_all();
        }
    }

    pub fn icache_invalidate_line(&self, addr: u32) {
        if let Some(ref ic) = self.icache {
            ic.borrow_mut().invalidate_line(addr);
        }
    }

    fn icache_fill(&self, addr: u32) {
        if let Some(ref ic_cell) = self.icache {
            let mut ic = ic_cell.borrow_mut();
            if ic.contains(addr) {
                return;
            }
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

    // ========== Instruction fetch ==========

    pub fn fetch_half(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        if self.bank.is_some() {
            if let Some(ref ic_cell) = self.icache {
                let enabled = ic_cell.borrow().is_enabled();
                if enabled {
                    {
                        let ic = ic_cell.borrow();
                        if let Some(val) = ic.peek_half(addr) {
                            return Ok(val);
                        }
                    }
                    self.icache_fill(addr);
                    let ic = ic_cell.borrow();
                    if let Some(val) = ic.peek_half(addr) {
                        return Ok(val);
                    }
                }
            }
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
        if self.bank.is_some() {
            if let Some(ref ic_cell) = self.icache {
                let enabled = ic_cell.borrow().is_enabled();
                if enabled {
                    let line_mask = (IC_LINE_SIZE as u32) - 1;
                    let line_end = (addr & !line_mask) + IC_LINE_SIZE as u32;
                    if addr + 4 <= line_end {
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
                        let hi = self.fetch_half(addr)? as u32;
                        let lo = self.fetch_half(addr + 2)? as u32;
                        return Ok((hi << 16) | lo);
                    }
                }
            }
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
        assert_eq!(mem.read_word(0).unwrap(), 0xDEADBEEF);
    }

    #[test]
    fn test_big_endian_half() {
        let mut mem = Memory::new(16);
        mem.write_half(0, 0xCAFE).unwrap();
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
    }

    #[test]
    fn test_load_binary() {
        let mut mem = Memory::new(16);
        mem.load_binary(0, &[0x20, 0x00, 0x08, 0x00]);
        assert_eq!(mem.read_word(0).unwrap(), 0x20000800);
    }

    #[test]
    fn test_unified_sram_soc_warm() {
        let mut mem = Memory::new_soc(1024, BootMode::Warm);
        mem.load_binary(0, &[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(mem.fetch_half(0).unwrap(), 0xAABB);
        assert_eq!(mem.fetch_word(0).unwrap(), 0xAABBCCDD);
        assert_eq!(mem.read_half(0).unwrap(), 0xAABB);
        assert_eq!(mem.read_word(0).unwrap(), 0xAABBCCDD);
    }

    #[test]
    fn test_dcache_sram_write_through() {
        let mut mem = Memory::new_soc(4096, BootMode::Warm);
        mem.write_word_data(0x100, 0xDEADBEEF, false).unwrap();
        assert_eq!(mem.read_word_data(0x100, false).unwrap(), 0xDEADBEEF);
        assert_eq!(mem.read_word_data(0x100, true).unwrap(), 0xDEADBEEF);
        assert_eq!(mem.read_word(0x100).unwrap(), 0xDEADBEEF);
    }

    #[test]
    fn test_dcache_bypass_writes_to_sram() {
        let mut mem = Memory::new_soc(4096, BootMode::Warm);
        mem.write_word_data(0x200, 0xCAFEBABE, true).unwrap();
        assert_eq!(mem.read_word(0x200).unwrap(), 0xCAFEBABE);
        assert_eq!(mem.read_word_data(0x200, false).unwrap(), 0xCAFEBABE);
    }

    #[test]
    fn test_dcache_invalidate_flushes_dirty() {
        let mut mem = Memory::new_soc(4096, BootMode::Warm);
        mem.write_word_data(0x300, 0x12345678, false).unwrap();
        assert_eq!(mem.read_word(0x300).unwrap(), 0x12345678);
        mem.dcache_invalidate_all().unwrap();
        assert_eq!(mem.read_word(0x300).unwrap(), 0x12345678);
    }
}
