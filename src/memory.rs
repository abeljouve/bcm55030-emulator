use std::cell::{Cell, RefCell};
use std::sync::Arc;

use parking_lot::RwLock;

use crate::cache::{DCache, DCacheLineInfo, DcacheSaveState, ICache, IcacheSaveState, IC_LINE_SIZE};
use crate::cpu::exception::Exception;
use crate::soc::bank::{BootMode, PeripheralBank};
use crate::soc::peripheral::DatapathOp;

/// BCM55030 unified SRAM: 512 KB. No ICCM/DCCM.
pub const SRAM_SIZE: usize = 512 * 1024;

/// Access direction a watchpoint triggers on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WatchMode {
    Read,
    Write,
    ReadWrite,
}

/// A single watchpoint entry. Half-open range `[addr, addr + size)`.
#[derive(Clone, Copy, Debug)]
pub struct Watchpoint {
    pub addr: u32,
    pub size: u32,
    pub mode: WatchMode,
}

impl Watchpoint {
    /// Does the access `[access_addr, access_addr + access_size)`
    /// overlap this watchpoint's range, *and* match its configured
    /// direction?
    #[inline]
    pub fn matches(&self, access_addr: u32, access_size: u32, access: WatchMode) -> bool {
        let a_end = access_addr.saturating_add(access_size);
        let w_end = self.addr.saturating_add(self.size);
        let overlaps = access_addr < w_end && self.addr < a_end;
        let mode_match = matches!(
            (self.mode, access),
            (WatchMode::ReadWrite, _)
                | (WatchMode::Read, WatchMode::Read)
                | (WatchMode::Write, WatchMode::Write)
        );
        overlaps && mode_match
    }
}

/// Watchpoint list + last hit. The check path uses interior
/// mutability on `hit` so `read_*` helpers (`&self`) can record a
/// trap without taking `&mut self`.
#[derive(Default, Debug)]
pub struct WatchpointTable {
    pub entries: Vec<Watchpoint>,
    /// Most recent hit: `(access_addr, access_mode)`. Cleared by
    /// `Cpu::step` after transitioning to the paused state via
    /// `take_hit()`.
    hit: Cell<Option<(u32, WatchMode)>>,
}

impl WatchpointTable {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn add(&mut self, wp: Watchpoint) -> usize {
        self.entries.push(wp);
        self.entries.len() - 1
    }

    pub fn remove(&mut self, index: usize) {
        if index < self.entries.len() {
            self.entries.remove(index);
        }
    }

    pub fn clear(&mut self) {
        self.entries.clear();
        self.hit.set(None);
    }

    /// Consume the last hit (if any). Called by `Cpu::step` after the
    /// executor returns, to decide whether to pause.
    #[inline]
    pub fn take_hit(&self) -> Option<(u32, WatchMode)> {
        self.hit.replace(None)
    }

    /// Scan the table for a matching entry and record a hit through
    /// interior mutability. The fast path is the empty-table case —
    /// callers must gate the call behind `!self.is_empty()` so
    /// read_* helpers pay nothing when no watchpoints are set.
    pub fn check(&self, addr: u32, size: u32, access: WatchMode) -> bool {
        for wp in &self.entries {
            if wp.matches(addr, size, access) {
                self.hit.set(Some((addr, access)));
                return true;
            }
        }
        false
    }
}

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

    /// UI / MCP watchpoints. Default empty — the hot path only pays
    /// a single `is_empty()` branch when no watchpoints are set.
    pub watchpoints: WatchpointTable,
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
            watchpoints: WatchpointTable::new(),
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
            watchpoints: WatchpointTable::new(),
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

    /// Clone the full backing byte buffer (SRAM in SoC mode, flat
    /// memory in tests). Used by the UI / MCP worker to refresh its
    /// disassembly-panel SRAM view on demand.
    pub fn sram_snapshot(&self) -> Vec<u8> {
        self.data.clone()
    }

    pub fn restore_sram(&mut self, data: &[u8]) {
        let len = data.len().min(self.data.len());
        self.data[..len].copy_from_slice(&data[..len]);
    }

    pub fn save_cache_state(&self) -> (Option<DcacheSaveState>, Option<IcacheSaveState>) {
        let dc = self.dcache.as_ref().map(|dc| dc.save_state());
        let ic = self.icache.as_ref().map(|ic| ic.borrow().save_state());
        (dc, ic)
    }

    pub fn restore_cache_state(
        &mut self,
        dc: Option<DcacheSaveState>,
        ic: Option<IcacheSaveState>,
    ) {
        if let (Some(ref mut cache), Some(state)) = (&mut self.dcache, dc) {
            cache.restore_state(state);
        }
        if let (Some(ref cache_cell), Some(state)) = (&self.icache, ic) {
            cache_cell.borrow_mut().restore_state(state);
        }
    }

    /// Borrow a slice of the backing buffer without cloning. Returns
    /// `None` if the requested range exits the buffer.
    pub fn sram_slice(&self, start: u32, len: u32) -> Option<&[u8]> {
        let s = start as usize;
        let e = s.checked_add(len as usize)?;
        if e > self.data.len() {
            return None;
        }
        Some(&self.data[s..e])
    }

    /// Snapshot every physical D-cache line. Empty in flat mode (no
    /// cache). Callers use this to populate the "D-cache state"
    /// sub-tab of the memory viewer.
    pub fn dcache_snapshot(&self) -> Vec<DCacheLineInfo> {
        self.dcache
            .as_ref()
            .map(DCache::snapshot_lines)
            .unwrap_or_default()
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
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 1, WatchMode::Read);
        }
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
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 2, WatchMode::Read);
        }
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
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 4, WatchMode::Read);
        }
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
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 1, WatchMode::Write);
        }
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
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 2, WatchMode::Write);
        }
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
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 4, WatchMode::Write);
        }
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
    pub fn drain_datapath_public(&mut self) -> Result<(), Exception> {
        self.drain_datapath()
    }

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
                        // A DMA SRAM write must not evict a *dirty*
                        // D-cache line: the CPU's pending write shadows
                        // SRAM (HW-verified, scan7b test 7). Evicting it
                        // dropped the firmware's cached saved-return
                        // slot when a flash-read destination shared a
                        // 32-byte line with a live stack word
                        // (bug emu-reboot-halt-and-transfer-model-
                        // divergences D2 → j [blink=0] → reboot). Clean
                        // lines are still invalidated so a later cached
                        // read refills from the new SRAM bytes.
                        let base = sram_addr & !0x1F;
                        let last = (sram_addr + data.len() as u32).saturating_sub(1);
                        let mut addr = base;
                        while addr <= (last & !0x1F) {
                            dc.dma_sram_overwrite(addr);
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
            DatapathOp::CacheInvalidate { addr: inv_addr } => {
                if let Some(ref mut dc) = self.dcache {
                    dc.invalidate_line(inv_addr & !0x1F);
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

    /// MMIO space (outside SRAM) is uncacheable — the D-cache only
    /// covers SRAM at 0x00000000..0x0007FFFF. Loads/stores to MMIO
    /// addresses bypass the D-cache regardless of the `.di` flag.
    #[inline]
    fn is_mmio(&self, addr: u32) -> bool {
        self.sram_offset(addr).is_none()
    }

    /// Mirror a `.di` (uncached) store that lands in the low-memory
    /// NCO/IVT aperture into the NCO channel table. The hardware
    /// aliases the 16-channel NCO table over the ARC interrupt-vector
    /// range on a separate physical bus: `.di` stores update the NCO
    /// (the ARC interrupt unit fetches its vector from there), while
    /// plain reads / instruction fetch still see SRAM. The SRAM write
    /// is kept (the caller still performs it) so I-cache coherence of
    /// any code executed from `0x0..0x80` is unchanged — only the
    /// interrupt-vector source is added. Evidence: Ghidra
    /// `nco_write_channel` @0x5a18 / `hw_install_irq_vector_2`
    /// @0x20042d00 plate comments; "NCO table IS the ARC IVT" RE
    /// swarm (live slot0 = `j @0x150`).
    #[inline]
    fn nco_ivt_mirror(&self, addr: u32, val: u32, size: u8) {
        if let Some(ref bank) = self.bank {
            bank.write().nco.ivt_di_store(addr, val, size);
        }
    }

    pub fn read_byte_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u8, Exception> {
        if cache_bypass || self.is_mmio(addr) || !self.dcache_enabled() {
            return self.read_byte_backing(addr);
        }
        self.ensure_cache_line(addr)?;
        Ok(self.dcache.as_mut().unwrap().read_byte(addr).unwrap())
    }

    pub fn read_half_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u16, Exception> {
        if cache_bypass || addr & 1 != 0 || self.is_mmio(addr) || !self.dcache_enabled() {
            if self.sram_offset(addr).is_none() {
                if let Some(ref bank) = self.bank {
                    return bank.write().read_half(addr);
                }
                return Ok(0);
            }
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
        if cache_bypass || addr & 3 != 0 || self.is_mmio(addr) || !self.dcache_enabled() {
            if self.sram_offset(addr).is_none() {
                if let Some(ref bank) = self.bank {
                    return bank.write().read_word(addr);
                }
                return Ok(0);
            }
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
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 1, WatchMode::Write);
        }
        if cache_bypass && crate::soc::nco::Nco::in_ivt_aperture(addr) {
            self.nco_ivt_mirror(addr, val as u32, 1);
        }
        if cache_bypass || self.is_mmio(addr) || !self.dcache_enabled() {
            return self.write_byte(addr, val);
        }
        self.ensure_cache_line(addr)?;
        self.dcache.as_mut().unwrap().write_byte(addr, val);
        Ok(())
    }

    pub fn write_half_data(&mut self, addr: u32, val: u16, cache_bypass: bool) -> Result<(), Exception> {
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 2, WatchMode::Write);
        }
        if cache_bypass && crate::soc::nco::Nco::in_ivt_aperture(addr) {
            self.nco_ivt_mirror(addr, val as u32, 2);
        }
        if cache_bypass || addr & 1 != 0 || self.is_mmio(addr) || !self.dcache_enabled() {
            return self.write_half(addr, val);
        }
        self.ensure_cache_line(addr)?;
        let dc = self.dcache.as_mut().unwrap();
        dc.write_byte(addr, (val >> 8) as u8);
        dc.write_byte(addr + 1, val as u8);
        Ok(())
    }

    pub fn write_word_data(&mut self, addr: u32, val: u32, cache_bypass: bool) -> Result<(), Exception> {
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 4, WatchMode::Write);
        }
        if cache_bypass && crate::soc::nco::Nco::in_ivt_aperture(addr) {
            self.nco_ivt_mirror(addr, val, 4);
        }
        if cache_bypass || addr & 3 != 0 || self.is_mmio(addr) || !self.dcache_enabled() {
            return self.write_word(addr, val);
        }
        self.ensure_cache_line(addr)?;
        let dc = self.dcache.as_mut().unwrap();
        dc.write_byte(addr, (val >> 24) as u8);
        dc.write_byte(addr + 1, (val >> 16) as u8);
        dc.write_byte(addr + 2, (val >> 8) as u8);
        dc.write_byte(addr + 3, val as u8);
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

    /// Single-line I-cache invalidate. Retained as a Memory API but
    /// currently unused: `IC_IVIL` is a HW-verified no-op on BCM55030
    /// (only `IC_IVIC` flushes), and DMA coherence invalidates lines
    /// directly on the cache in `apply_datapath_op`.
    #[allow(dead_code)]
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
            // Only fill from SRAM-backed addresses. A fetch to an
            // address outside SRAM has no defined contents on real
            // silicon — the prior unconditional zero-fill silently
            // turned out-of-range fetches into `b .` (opcode
            // `0x00000000`), which the tight-loop watchdog then
            // converted into a warm reboot, hiding the firmware's
            // actual jump-to-garbage symptom. Returning early here
            // lets `fetch_half`/`fetch_word` fall through to the
            // SRAM-bounds check and raise `Exception::MemoryError`,
            // which vectors to the firmware's instruction-error
            // handler and surfaces the faulting PC.
            let Some(off) = self.sram_offset(line_addr) else {
                return;
            };
            let mut data = [0u8; IC_LINE_SIZE];
            let end = (off + IC_LINE_SIZE).min(self.data.len());
            let count = end - off;
            data[..count].copy_from_slice(&self.data[off..end]);
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

    // Cached writes are write-back: the dirty value lives in the D-cache
    // and SRAM stays stale until a flush. Cached reads (`read_word_data`
    // with `cache_bypass=false`) and cache-peek reads (`read_word`) see
    // the new value. Uncached reads (`read_word_data` with
    // `cache_bypass=true`) reach SRAM directly and therefore see the
    // pre-write contents — scan7b test 7 on real HW confirms `.di` loads
    // never consult the cache.
    #[test]
    fn test_dcache_writes_are_write_back() {
        let mut mem = Memory::new_soc(4096, BootMode::Warm);
        mem.write_word_data(0x100, 0xDEADBEEF, false).unwrap();

        // Cached and cache-peek paths see the new value.
        assert_eq!(mem.read_word_data(0x100, false).unwrap(), 0xDEADBEEF);
        assert_eq!(mem.read_word(0x100).unwrap(), 0xDEADBEEF);

        // Uncached read bypasses the cache → sees pre-write SRAM (0).
        assert_eq!(mem.read_word_data(0x100, true).unwrap(), 0);

        // Invalidate with IM=1 flushes dirty → SRAM now has the value.
        mem.dcache_invalidate_all().unwrap();
        assert_eq!(mem.read_word_data(0x100, true).unwrap(), 0xDEADBEEF);
        assert_eq!(mem.read_word(0x100).unwrap(), 0xDEADBEEF);
    }

    // Direct Memory-API repro of the alleged coherence bug from
    // the design notes. The TODO note
    // claimed a cached write of `0xDEADBEEF` to addr X followed by
    // a cached write to addr X+4 (same 32 B line) would make a
    // subsequent cached read of X return X+4's value instead of the
    // seed. These probes force each of the interesting configurations
    // and assert that the cached read of X is always the seed.
    #[test]
    fn test_dcache_adjacent_words_no_alias() {
        let mut mem = Memory::new_soc(16 * 1024, BootMode::Warm);
        let canary: u32 = 0x3A30;
        let ticks: u32 = 0x3A34;
        assert_eq!(canary & !0x1F, ticks & !0x1F, "must share one 32 B line");

        // 1. Pre-warmed line: BSS-style zero, then seed, then adjacent.
        for a in (0x3A20u32..0x3A40).step_by(4) {
            mem.write_word_data(a, 0, false).unwrap();
        }
        mem.write_word_data(canary, 0xDEADBEEF, false).unwrap();
        mem.write_word_data(ticks, 0xAA, false).unwrap();
        assert_eq!(mem.read_word_data(canary, false).unwrap(), 0xDEADBEEF);
        assert_eq!(mem.read_word_data(ticks, false).unwrap(), 0xAA);

        // 2. Cold line: line isn't touched before the seed.
        let mut mem = Memory::new_soc(16 * 1024, BootMode::Warm);
        mem.write_word_data(canary, 0xDEADBEEF, false).unwrap();
        mem.write_word_data(ticks, 0xAA, false).unwrap();
        assert_eq!(mem.read_word_data(canary, false).unwrap(), 0xDEADBEEF);

        // 3. Byte-level seed then word-level adjacent write. Exercises
        //    the mix of write_byte_data / write_word_data that firmware
        //    compilers emit when packing structs.
        let mut mem = Memory::new_soc(16 * 1024, BootMode::Warm);
        for (i, b) in [0xDE, 0xAD, 0xBE, 0xEF].iter().enumerate() {
            mem.write_byte_data(canary + i as u32, *b, false).unwrap();
        }
        mem.write_word_data(ticks, 0xAA, false).unwrap();
        assert_eq!(mem.read_word_data(canary, false).unwrap(), 0xDEADBEEF);

        // 4. Force an eviction of the canary line between the seed
        //    and the adjacent write by hammering the same set with
        //    three other tags. RR replacement guarantees the victim
        //    cycles and the canary line gets flushed to SRAM.
        let mut mem = Memory::new_soc(128 * 1024, BootMode::Warm);
        mem.write_word_data(canary, 0xDEADBEEF, false).unwrap();
        let stride: u32 = 64 * 32;
        for k in 1..=3 {
            mem.write_word_data(canary + k * stride, 0x1111_0000 | k, false)
                .unwrap();
        }
        // Cached read must refill from SRAM (evicted line was flushed).
        assert_eq!(mem.read_word_data(canary, false).unwrap(), 0xDEADBEEF);
        mem.write_word_data(ticks, 0xAA, false).unwrap();
        assert_eq!(mem.read_word_data(canary, false).unwrap(), 0xDEADBEEF);
        assert_eq!(mem.read_word_data(ticks, false).unwrap(), 0xAA);
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

    // Regression: bug emu-reboot-halt-and-transfer-model-divergences
    // D2. A DMA SRAM write (flash_memcpy's MDIO read into a stack
    // local) must NOT evict a *dirty* D-cache line sharing the same
    // 32-byte line — that line holds a caller's CPU-written saved
    // `blink`. Pre-fix the line was evicted, the restore-millicode
    // `ld blink` read SRAM (0), `j [blink=0]` → PC=0 → reboot.
    #[test]
    fn dma_sram_write_preserves_dirty_saved_blink_line() {
        let mut mem = Memory::new_soc(0x8000, BootMode::Warm);
        let slot = 0x1F7Cu32; // saved-blink stack slot
        let saved_blink = 0x0003_60BEu32;
        // Caller `st blink,[sp,N]`: cached, write-back → dirty line,
        // SRAM still stale.
        mem.write_word_data(slot, saved_blink, false).unwrap();
        assert_eq!(mem.read_word_data(slot, false).unwrap(), saved_blink);
        assert_eq!(mem.read_word_data(slot, true).unwrap(), 0x0000_0000);

        // flash_memcpy head read: PBC flash/MDIO DMA writes 4 flash
        // bytes into a stack local in the SAME 32-byte line.
        mem.apply_datapath_op(DatapathOp::SramWrite {
            sram_addr: 0x1F70,
            data: vec![0xFF, 0xFF, 0xFF, 0xFF],
        })
        .unwrap();

        // Silicon-faithful (scan7b test 7): the dirty saved-blink
        // survives — CPU's pending write shadows the DMA SRAM write.
        assert_eq!(
            mem.read_word_data(slot, false).unwrap(),
            saved_blink,
            "DMA SRAM write must not evict the dirty saved-blink line"
        );
        // The DMA bytes did land in SRAM at the real destination.
        assert_eq!(mem.read_word_data(0x1F70, true).unwrap(), 0xFFFF_FFFF);
    }

    #[test]
    fn dma_sram_write_refreshes_clean_line() {
        let mut mem = Memory::new_soc(0x8000, BootMode::Warm);
        let a = 0x2000u32;
        mem.apply_datapath_op(DatapathOp::SramWrite {
            sram_addr: a,
            data: vec![0x11, 0x22, 0x33, 0x44],
        })
        .unwrap();
        // Clean cached line (read allocates, not dirty).
        assert_eq!(mem.read_word_data(a, false).unwrap(), 0x1122_3344);
        mem.apply_datapath_op(DatapathOp::SramWrite {
            sram_addr: a,
            data: vec![0xAA, 0xBB, 0xCC, 0xDD],
        })
        .unwrap();
        assert_eq!(
            mem.read_word_data(a, false).unwrap(),
            0xAABB_CCDD,
            "clean line must refill from the freshly DMA-written SRAM"
        );
    }
}
