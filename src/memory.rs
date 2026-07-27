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

    /// Policy for MMIO reads / writes that do not match any
    /// peripheral claim. `false` (default) returns zero so existing
    /// code that never probes unmapped addresses stays unaffected.
    /// `true` returns [`Exception::MemoryError`] to surface hidden
    /// firmware accesses — enable with `--unmapped-exception` to
    /// discover new unmodelled registers.
    pub unmapped_exception: bool,

    /// UI / MCP watchpoints. Default empty — the hot path only pays
    /// a single `is_empty()` branch when no watchpoints are set.
    pub watchpoints: WatchpointTable,

    /// Optional data-read trace (data-table discovery). When `Some`,
    /// every `read_*_data` whose address falls in `[lo,hi)` records
    /// `(addr, size_bytes, value)`. `None` by default — the hot path pays a
    /// single `is_some()` branch. The range filter naturally excludes MMIO
    /// (peripheral addresses are outside the SRAM data window).
    pub read_trace: Option<ReadTrace>,

    /// Misaligned CPU data accesses seen so far (see [`MisalignedDataLog`]
    /// and `data_effective_addr`). Always on: the align-down is silent by
    /// construction, so without a counter this entire bug class stays
    /// invisible to every behavioural gate. Costs one compare per data
    /// access on the aligned (overwhelmingly common) path.
    pub misaligned_data: MisalignedDataLog,
}

/// One misaligned CPU data access recorded by `data_effective_addr`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MisalignedDataHit {
    /// PC of the faulting load/store (0 in flat test mode — no bank).
    pub pc: u32,
    /// Address the instruction asked for.
    pub addr: u32,
    /// Address the core actually drove (`addr` with the low bits cleared).
    pub ea: u32,
    /// Access width in bytes: 2 or 4.
    pub width: u8,
    pub is_write: bool,
}

/// Running tally of misaligned CPU word / half-word data accesses.
///
/// Each hit is silent corruption on real silicon: the core reads or writes a
/// *different* location than the instruction named, with no exception. The
/// emulator reproduces that faithfully (see `data_effective_addr`), which by
/// itself would make the whole class undetectable — hence this log. Firmware
/// that never issues a misaligned data access reports `count == 0`.
#[derive(Default, Clone, Debug)]
pub struct MisalignedDataLog {
    /// Total misaligned data accesses. Not capped.
    pub count: u64,
    /// The first [`MISALIGNED_LOG_CAP`] hits, for attribution.
    pub hits: Vec<MisalignedDataHit>,
}

/// How many misaligned hits are retained with full detail. The counter keeps
/// running past this; only the detail list stops growing.
pub const MISALIGNED_LOG_CAP: usize = 64;

/// Captured firmware data-region reads (see [`Memory::read_trace`]). The
/// emulator is the address oracle: running a reference function with this enabled
/// yields the exact `(addr, size, value)` of every table/global load it
/// issues — used to populate the `.data` region with real contents.
#[derive(Default, Clone)]
pub struct ReadTrace {
    pub lo: u32,
    pub hi: u32,
    pub hits: Vec<(u32, u8, u32)>,
}

/// The SRAM image before anything writes it.
///
/// OBSERVED (verified against real hardware): silicon SRAM does **not** come up
/// zeroed — a probe of three regions read ~98% arbitrary bytes. Zero-filling is a
/// modelling choice known to be wrong, and wrong in a direction that matters: it
/// makes every read-before-write return 0, a very special value. A uniform
/// non-zero fill is barely better — it still gives every such read the same one.
///
/// `ARC700_SRAM_FILL=noise` fills with a deterministic PRNG, which is what the
/// hardware looks like. Determinism is deliberate: runs must stay reproducible.
/// The default stays 0 so no existing comparison shifts under anyone.
fn sram_initial(size: usize) -> Vec<u8> {
    if std::env::var("ARC700_SRAM_FILL").map(|v| v.trim().eq_ignore_ascii_case("noise")) == Ok(true)
    {
        // xorshift32, fixed seed -- same bytes every run, on every host.
        let mut x: u32 = 0x1357_9BDF;
        let mut v = Vec::with_capacity(size);
        for _ in 0..size {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            v.push((x >> 24) as u8);
        }
        return v;
    }
    vec![sram_fill(); size]
}

/// The byte SRAM holds before anything writes it, when the fill is UNIFORM.
///
/// `ARC700_SRAM_FILL=0xA5` (any u8, hex or decimal) sets it; see
/// [`sram_initial`] for why the default of 0 is a choice and not an observation,
/// and for the `noise` mode that matches measured hardware.
pub fn sram_fill() -> u8 {
    match std::env::var("ARC700_SRAM_FILL") {
        Ok(s) => {
            let t = s.trim();
            let parsed = t
                .strip_prefix("0x")
                .or_else(|| t.strip_prefix("0X"))
                .map(|h| u8::from_str_radix(h, 16))
                .unwrap_or_else(|| t.parse::<u8>());
            parsed.unwrap_or(0)
        }
        Err(_) => 0,
    }
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
            read_trace: None,
            misaligned_data: MisalignedDataLog::default(),
        }
    }

    /// Create a SoC memory with unified SRAM + peripheral bank.
    /// BCM55030 has 512 KB unified SRAM (no separate ICCM/DCCM).
    ///
    /// SRAM starts at [`sram_fill`], which is 0 by default and settable with
    /// `ARC700_SRAM_FILL` — see that function for why the default is a modelling
    /// CHOICE and not a measurement.
    pub fn new_soc(sram_size: usize, boot_mode: BootMode) -> Self {
        let bank = Arc::new(RwLock::new(PeripheralBank::new(boot_mode)));
        Self {
            data: sram_initial(sram_size),
            bank: Some(bank),
            dccm_base: 0,
            dccm_watchpoint: None,
            dcache: Some(DCache::new()),
            icache: Some(RefCell::new(ICache::new())),
            unmapped_exception: false,
            watchpoints: WatchpointTable::new(),
            read_trace: None,
            misaligned_data: MisalignedDataLog::default(),
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

    // ========== Observer / introspection accessors ==========
    //
    // `read_byte` / `read_half` / `read_word` / `write_byte` / `write_half` /
    // `write_word` back the debug and introspection views (MCP `read_memory` /
    // `write_memory`, the UI memory panel, offline analysis tools).
    //
    // They deliberately do NOT apply the §6.2 align-down that the CPU data port
    // applies in `data_effective_addr`. This asymmetry is intentional: a
    // debugger asked for the bytes at address N must report the bytes at
    // address N. Do not "fix" the misaligned branches below to match the CPU —
    // the very tools used to validate the emulator read through here, and
    // rounding would make their diagnostics lie about memory contents.
    //
    // `write_word_data` / `write_half_data` reuse `write_word` / `write_half`
    // as their backing-store writer, but only ever with an address that
    // `data_effective_addr` has already masked.

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
                        // SRAM (verified against real hardware). Evicting
                        // it would drop a firmware's cached saved-return
                        // slot when a flash-read destination shares a
                        // 32-byte line with a live stack word
                        // (→ j [blink=0] → reboot). Clean lines are still
                        // invalidated so a later cached read refills from
                        // the new SRAM bytes.
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
    /// interrupt-vector source is added. The NCO table aliasing the ARC
    /// interrupt-vector range is verified against real hardware.
    #[inline]
    fn nco_ivt_mirror(&self, addr: u32, val: u32, size: u8) {
        if let Some(ref bank) = self.bank {
            bank.write().nco.ivt_di_store(addr, val, size);
        }
    }

    /// Record a data-region read in the active trace (see [`ReadTrace`]).
    /// Gated by `read_trace.is_some()` + the `[lo,hi)` range; off by default.
    #[inline]
    fn trace_data_read(&mut self, addr: u32, size: u8, val: u32) {
        if let Some(rt) = self.read_trace.as_mut() {
            if addr >= rt.lo && addr < rt.hi {
                rt.hits.push((addr, size, val));
            }
        }
    }

    /// Effective address of a CPU data access — the address the core actually
    /// drives on the memory port.
    ///
    /// OBSERVED (verified against real hardware, §6.2 "Misaligned accesses"):
    /// this ARC700 integration neither traps nor performs a byte-wise fixup on
    /// a misaligned word / half-word access. The core silently clears the low
    /// address bits before performing the transfer:
    ///
    /// ```text
    /// word effective address      = address & ~3
    /// half-word effective address = address & ~1
    /// ```
    ///
    /// The mask is **per width**, not a single `& ~3`: `ldw base+3` transfers
    /// the half-word at `base+2` (`(base+3) & ~1`), *not* the one at `base+0`.
    /// Byte accesses have no alignment concept — `width == 1` yields an
    /// identity mask, so they are unaffected.
    ///
    /// This is an address-path property of the core, upstream of any cache or
    /// bus decision, so it applies uniformly to cached, `.di` (uncached) and
    /// MMIO accesses. Hence it is applied here, at the single entry point of
    /// the data port, ahead of the `cache_bypass || is_mmio || !dcache_enabled`
    /// dispatch — a peripheral therefore only ever sees an already-masked
    /// address. (INFERRED for the `.di` and MMIO legs: §6.2's probes used an
    /// SRAM base, so neither leg is directly measured. `.DI` is defined purely
    /// in terms of cache participation — "the cache is bypassed and the data is
    /// loaded directly from or stored directly to the memory" — and says
    /// nothing about address formation, so the two are orthogonal axes.)
    ///
    /// Deliberately NOT applied to:
    ///
    /// - **The `.ab` / `.aw` address write-back.** ISA Table 52 defines
    ///   "memory address used" and "register value write-back" as two separate
    ///   quantities; the write-back is the arithmetic `REG + offset` and is
    ///   never a function of the memory address. Masking here rather than in
    ///   the executor's `compute_ea` keeps the two decoupled by construction.
    /// - **`.as` scaled addressing**, which forms `REG + (offset << size)`
    ///   *before* this rule applies. Masking downstream of the executor's EA
    ///   computation composes correctly: a scaled access off an unaligned base
    ///   still gets masked here.
    /// - **Instruction fetch** (`fetch_half` / `fetch_word`) — a different port
    ///   with its own alignment rules. `addr & 3 == 2` is routine and
    ///   legitimate there because ARCompact mixes 16- and 32-bit encodings.
    /// - **The observer helpers** (`read_word` / `write_half` / …), which back
    ///   the debug and introspection views. Those must report memory exactly as
    ///   it is; a debugger that rounded down would lie about its contents.
    #[inline]
    fn data_effective_addr(&mut self, addr: u32, width: u32, is_write: bool) -> u32 {
        let ea = addr & !(width - 1);
        if ea != addr {
            self.record_misaligned(addr, ea, width as u8, is_write);
        }
        ea
    }

    /// Record a misaligned data access. Cold: correct firmware never gets here.
    #[cold]
    fn record_misaligned(&mut self, addr: u32, ea: u32, width: u8, is_write: bool) {
        self.misaligned_data.count += 1;
        if self.misaligned_data.hits.len() >= MISALIGNED_LOG_CAP {
            return;
        }
        let pc = self
            .bank
            .as_ref()
            .map(|b| b.read().current_pc)
            .unwrap_or(0);
        self.misaligned_data.hits.push(MisalignedDataHit {
            pc,
            addr,
            ea,
            width,
            is_write,
        });
    }

    pub fn read_byte_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u8, Exception> {
        let r = self.read_byte_data_inner(addr, cache_bypass);
        if self.read_trace.is_some() {
            if let Ok(v) = r {
                self.trace_data_read(addr, 1, v as u32);
            }
        }
        r
    }

    fn read_byte_data_inner(&mut self, addr: u32, cache_bypass: bool) -> Result<u8, Exception> {
        if cache_bypass || self.is_mmio(addr) || !self.dcache_enabled() {
            return self.read_byte_backing(addr);
        }
        self.ensure_cache_line(addr)?;
        Ok(self.dcache.as_mut().unwrap().read_byte(addr).unwrap())
    }

    pub fn read_half_data(&mut self, addr: u32, cache_bypass: bool) -> Result<u16, Exception> {
        // §6.2: half-word effective address = address & ~1.
        let addr = self.data_effective_addr(addr, 2, false);
        let r = self.read_half_data_inner(addr, cache_bypass);
        if self.read_trace.is_some() {
            if let Ok(v) = r {
                // Record the *effective* address: this trace is used as the
                // address oracle, so it must report what the core drove.
                self.trace_data_read(addr, 2, v as u32);
            }
        }
        r
    }

    fn read_half_data_inner(&mut self, addr: u32, cache_bypass: bool) -> Result<u16, Exception> {
        debug_assert_eq!(addr & 1, 0, "caller must pass a §6.2 effective address");
        if cache_bypass || self.is_mmio(addr) || !self.dcache_enabled() {
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
        // §6.2: word effective address = address & ~3.
        let addr = self.data_effective_addr(addr, 4, false);
        let r = self.read_word_data_inner(addr, cache_bypass);
        if self.read_trace.is_some() {
            if let Ok(v) = r {
                // Record the *effective* address (see `read_half_data`).
                self.trace_data_read(addr, 4, v);
            }
        }
        r
    }

    // WRONG per silicon characterization: a comment here used to claim the CPU
    // performs a byte-wise fixup of a misaligned access, and a dedicated
    // byte-by-byte branch implemented it.
    //
    // previously believed: the CPU silently fixes up misaligned word / half
    // reads and writes at any byte offset with no exception, so a byte-by-byte
    // path was HW-faithful. That inferred a fixup from the mere absence of a
    // trap.
    //
    // Verified against real hardware: there is no exception, but the low address
    // bits are silently *cleared* (§6.2, align-down) — the transfer lands
    // elsewhere, it is not byte-wise fixed up. The byte-wise model was "more
    // correct" than the hardware, which made the resulting silent corruption
    // invisible here. The mask now lives in `data_effective_addr`, so by the
    // time control reaches this function the address is aligned.
    fn read_word_data_inner(&mut self, addr: u32, cache_bypass: bool) -> Result<u32, Exception> {
        debug_assert_eq!(addr & 3, 0, "caller must pass a §6.2 effective address");
        if cache_bypass || self.is_mmio(addr) || !self.dcache_enabled() {
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
        // §6.2: half-word effective address = address & ~1. Masked before the
        // watchpoint check and the NCO mirror so both see the address the core
        // actually drove.
        let addr = self.data_effective_addr(addr, 2, true);
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 2, WatchMode::Write);
        }
        if cache_bypass && crate::soc::nco::Nco::in_ivt_aperture(addr) {
            self.nco_ivt_mirror(addr, val as u32, 2);
        }
        if cache_bypass || self.is_mmio(addr) || !self.dcache_enabled() {
            return self.write_half(addr, val);
        }
        self.ensure_cache_line(addr)?;
        let dc = self.dcache.as_mut().unwrap();
        dc.write_byte(addr, (val >> 8) as u8);
        dc.write_byte(addr + 1, val as u8);
        Ok(())
    }

    pub fn write_word_data(&mut self, addr: u32, val: u32, cache_bypass: bool) -> Result<(), Exception> {
        // §6.2: word effective address = address & ~3. Note this also repairs a
        // fabrication on the MMIO leg: a misaligned MMIO store used to fall
        // into `write_word`'s unaligned branch, find no SRAM backing, and
        // return `Ok(())` — silently dropped, never reaching the peripheral nor
        // the MMIO write stream. It now lands on the aligned register.
        let addr = self.data_effective_addr(addr, 4, true);
        if !self.watchpoints.is_empty() {
            self.watchpoints.check(addr, 4, WatchMode::Write);
        }
        if cache_bypass && crate::soc::nco::Nco::in_ivt_aperture(addr) {
            self.nco_ivt_mirror(addr, val, 4);
        }
        if cache_bypass || self.is_mmio(addr) || !self.dcache_enabled() {
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
        // Disabling the cache does NOT write dirty lines back. Setting
        // `DC_CTRL.DC` clears the enable flag and nothing else: the pending
        // dirty lines are simply lost, and the next access — which now takes
        // the bypass path — reads the stale SRAM word. Software that means to
        // keep what it wrote must issue a write-back invalidate (`DC_IVDC`
        // with `IM=1`) *before* disabling; a well-behaved boot loader does
        // exactly that. -- OBSERVED, DATASHEET §5.3 (and §5.3.1, §10.7), which calls
        // out compiler-saved frame pointers held across a disable window as
        // the case this bites.
        //
        // WRONG per silicon characterization: this used to flush every dirty
        // line on the enabled->disabled transition.
        //
        // previously believed: "On real ARC700 turning the D-cache off leaves
        // memory coherent — dirty lines are written back so later
        // uncached/bypassed accesses see the CPU's writes."
        //
        // That was inferred, not measured. On real hardware, disabling the cache
        // does not write back. Flushing here made the model more forgiving than
        // the hardware, which is the one direction a model must never err. In the
        // shipping configuration both boot loaders disable the D-cache and never
        // re-enable it (§7 step 3), so at runtime the cache is empty and there is
        // nothing to lose on such a store anyway.
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

    // NOTE: despite the name, this pins the *observer* family in flat test
    // mode, not the ARC700 misalignment rule. Flat mode has no peripheral bank
    // and takes `read_word`'s strict path, which rejects a misaligned request
    // rather than inventing a value. The CPU data port behaves completely
    // differently (§6.2 align-down) — see `misaligned_*_data_*` below.
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
    // pre-write contents — verified on real hardware: `.di` loads
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

    // Direct Memory-API repro of a hypothesized coherence bug: a
    // cached write of `0xDEADBEEF` to addr X followed by
    // a cached write to addr X+4 (same 32 B line) would supposedly make a
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

    // DATASHEET §5.3: disabling the D-cache does NOT write dirty lines back.
    // The enable flag clears, the dirty lines are lost, and the next access —
    // bypassing the cache now — reads the stale SRAM word. §5.3 names this
    // exact case: "compiler-saved frame pointers held on the stack across a
    // cache-disable window".
    //
    // The emulator used to flush here, inferred from a firmware fault that was
    // really caused by entering with the D-cache enabled — a state no boot path
    // produces. Modelling a write-back the hardware does not perform hides every
    // instance of this bug class.
    #[test]
    fn dc_ctrl_disable_strands_dirty_lines() {
        let mut mem = Memory::new_soc(0x8000, BootMode::Warm);
        let slot = 0x1F88u32;
        let saved_fp = 0x0003_1FB8u32;
        // CPU `st fp,[sp,N]`: cached write-back -> dirty. The uncached
        // (bypass) view still sees stale SRAM (0).
        mem.write_word_data(slot, saved_fp, false).unwrap();
        assert_eq!(mem.read_word_data(slot, true).unwrap(), 0x0000_0000);
        // Firmware disables the D-cache (DC_CTRL = 0xC3, DC bit set).
        mem.dcache_sync_ctrl(0xC3);
        // SRAM never received the write, and every load now bypasses.
        assert_eq!(mem.read_word_data(slot, true).unwrap(), 0x0000_0000);
        assert_eq!(mem.read_word_data(slot, false).unwrap(), 0x0000_0000);
    }

    // The other half of §5.3: the write-back invalidate (`DC_IVDC` with IM=1)
    // is what makes SRAM coherent, and software must issue it *before* the
    // disable. A well-behaved boot loader does (flush-invalidate then
    // `sr 0xC3,[DC_CTRL]`), which is why its hand-off survives.
    #[test]
    fn write_back_invalidate_before_disable_preserves_the_line() {
        let mut mem = Memory::new_soc(0x8000, BootMode::Warm);
        let slot = 0x1F88u32;
        let saved_fp = 0x0003_1FB8u32;
        mem.write_word_data(slot, saved_fp, false).unwrap();
        // DC_IVDC with IM=1: flush, then invalidate.
        mem.dcache_invalidate_all().unwrap();
        mem.dcache_sync_ctrl(0xC3);
        assert_eq!(mem.read_word_data(slot, true).unwrap(), saved_fp);
        assert_eq!(mem.read_word_data(slot, false).unwrap(), saved_fp);
    }

    // Regression: a DMA SRAM write (a flash/MDIO DMA read into a stack
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

        // A flash/MDIO DMA head read writes 4 flash bytes into a stack
        // local in the SAME 32-byte line.
        mem.apply_datapath_op(DatapathOp::SramWrite {
            sram_addr: 0x1F70,
            data: vec![0xFF, 0xFF, 0xFF, 0xFF],
        })
        .unwrap();

        // Silicon-faithful: the dirty saved-blink survives — the CPU's
        // pending write shadows the DMA SRAM write.
        assert_eq!(
            mem.read_word_data(slot, false).unwrap(),
            saved_blink,
            "DMA SRAM write must not evict the dirty saved-blink line"
        );
        // The DMA bytes did land in SRAM at the real destination.
        assert_eq!(mem.read_word_data(0x1F70, true).unwrap(), 0xFFFF_FFFF);
    }

    // ===================================================================
    // ISA characterization §6.2 "Misaligned accesses"
    //
    // Test vector and result table for §6.2. Bytes
    // `80 FF 00 AA 11 22 33 44` at an aligned base:
    //
    //   | Request                  | Observed result/effect              |
    //   |--------------------------|-------------------------------------|
    //   | ld base+1, +2, or +3     | 0x80FF00AA from base+0              |
    //   | ldw base+1               | 0x000080FF from base+0              |
    //   | ldw base+3               | 0x000000AA from base+2              |
    //   | st base+1 or base+2      | overwrites the word at base+0       |
    //   | stw base+1               | overwrites the half-word at base+0  |
    //
    // The core does not trap and does not perform a byte-wise fixup; it
    // silently clears the low address bits. This is silent corruption, and
    // these tests are the only behavioural gate on it — a boot-path write diff
    // cannot see this class (no misaligned access executes on the boot path),
    // so if these go, the model goes back to being wrong with nothing to catch
    // it.
    // ===================================================================

    /// The oracle's own byte vector.
    const ORACLE_BYTES: [u8; 8] = [0x80, 0xFF, 0x00, 0xAA, 0x11, 0x22, 0x33, 0x44];
    /// An aligned base, as the oracle specifies.
    const ORACLE_BASE: u32 = 0x10;

    fn oracle_mem() -> Memory {
        let mut mem = Memory::new(64);
        mem.load_binary(ORACLE_BASE, &ORACLE_BYTES);
        mem
    }

    fn oracle_soc() -> Memory {
        let mut mem = Memory::new_soc(0x8000, BootMode::Warm);
        mem.load_binary(ORACLE_BASE, &ORACLE_BYTES);
        mem
    }

    // Row 1: `ld base+1`, `+2`, `+3` -> 0x80FF00AA from base+0.
    #[test]
    fn misaligned_word_load_reads_the_word_at_base() {
        let mut mem = oracle_mem();
        let base = ORACLE_BASE;
        assert_eq!(mem.read_word_data(base, false).unwrap(), 0x80FF_00AA);
        for off in 1..=3u32 {
            assert_eq!(
                mem.read_word_data(base + off, false).unwrap(),
                0x80FF_00AA,
                "ld base+{off} must transfer the word at base+0 (address & ~3)",
            );
        }
    }

    // Rows 2-3: `ldw base+1` -> 0x000080FF from base+0;
    //           `ldw base+3` -> 0x000000AA from base+2.
    #[test]
    fn misaligned_half_load_masks_per_width_not_to_the_word() {
        let mut mem = oracle_mem();
        let base = ORACLE_BASE;
        assert_eq!(mem.read_half_data(base, false).unwrap(), 0x80FF);
        assert_eq!(
            mem.read_half_data(base + 1, false).unwrap(),
            0x80FF,
            "ldw base+1 -> half-word at base+0",
        );
        // The load-bearing row: the mask is `& ~1`, so base+3 -> base+2. A
        // single `& ~3` shared by both widths would satisfy every other row in
        // this file and silently fail only here.
        assert_eq!(
            mem.read_half_data(base + 3, false).unwrap(),
            0x00AA,
            "ldw base+3 -> half-word at base+2, NOT base+0",
        );
        assert_eq!(mem.read_half_data(base + 2, false).unwrap(), 0x00AA);
    }

    // Row 4: `st base+1` or `base+2` overwrites the word at base+0.
    #[test]
    fn misaligned_word_store_overwrites_the_word_at_base() {
        for off in [1u32, 2, 3] {
            let mut mem = oracle_mem();
            let base = ORACLE_BASE;
            mem.write_word_data(base + off, 0xDEAD_BEEF, false).unwrap();
            assert_eq!(
                mem.read_word_data(base, false).unwrap(),
                0xDEAD_BEEF,
                "st base+{off} must overwrite the word at base+0",
            );
            // No byte-wise spill into the neighbouring word: the transfer
            // moved wholesale to base+0, it did not straddle.
            assert_eq!(
                mem.read_word_data(base + 4, false).unwrap(),
                0x1122_3344,
                "st base+{off} must not disturb the next word",
            );
        }
    }

    // Row 5: `stw base+1` overwrites the half-word at base+0. Plus the
    // per-width counterpart: `stw base+3` hits base+2.
    #[test]
    fn misaligned_half_store_masks_per_width() {
        let base = ORACLE_BASE;

        let mut mem = oracle_mem();
        mem.write_half_data(base + 1, 0xBEEF, false).unwrap();
        assert_eq!(
            mem.read_word_data(base, false).unwrap(),
            0xBEEF_00AA,
            "stw base+1 -> half-word at base+0; the rest of the word is intact",
        );

        let mut mem = oracle_mem();
        mem.write_half_data(base + 3, 0xBEEF, false).unwrap();
        assert_eq!(
            mem.read_word_data(base, false).unwrap(),
            0x80FF_BEEF,
            "stw base+3 -> half-word at base+2, NOT base+0",
        );
    }

    // The mask is an address-path property of the core, upstream of the
    // cache/bus decision — so it holds identically on the cached and the `.di`
    // (uncached) paths, which take different branches in `*_data`.
    #[test]
    fn misaligned_mask_holds_on_cached_and_di_paths() {
        for cache_bypass in [false, true] {
            let mut mem = oracle_soc();
            let base = ORACLE_BASE;
            assert_eq!(
                mem.read_word_data(base + 3, cache_bypass).unwrap(),
                0x80FF_00AA,
                "ld base+3 (.di={cache_bypass}) -> word at base+0",
            );
            assert_eq!(
                mem.read_half_data(base + 3, cache_bypass).unwrap(),
                0x00AA,
                "ldw base+3 (.di={cache_bypass}) -> half-word at base+2",
            );

            let mut mem = oracle_soc();
            mem.write_word_data(base + 1, 0xDEAD_BEEF, cache_bypass).unwrap();
            assert_eq!(
                mem.read_word_data(base, cache_bypass).unwrap(),
                0xDEAD_BEEF,
                "st base+1 (.di={cache_bypass}) -> word at base+0",
            );
        }
    }

    // Aligned traffic must be bit-for-bit unchanged by the mask, and must not
    // be counted as misaligned.
    #[test]
    fn aligned_data_accesses_are_unchanged() {
        let mut mem = oracle_mem();
        let base = ORACLE_BASE;
        assert_eq!(mem.read_word_data(base, false).unwrap(), 0x80FF_00AA);
        assert_eq!(mem.read_word_data(base + 4, false).unwrap(), 0x1122_3344);
        assert_eq!(mem.read_half_data(base, false).unwrap(), 0x80FF);
        assert_eq!(mem.read_half_data(base + 2, false).unwrap(), 0x00AA);
        assert_eq!(mem.read_half_data(base + 6, false).unwrap(), 0x3344);

        mem.write_word_data(base, 0xCAFE_BABE, false).unwrap();
        assert_eq!(mem.read_word_data(base, false).unwrap(), 0xCAFE_BABE);
        mem.write_half_data(base + 4, 0x1234, false).unwrap();
        assert_eq!(mem.read_half_data(base + 4, false).unwrap(), 0x1234);

        assert_eq!(
            mem.misaligned_data.count, 0,
            "aligned traffic must not be reported as misaligned",
        );
    }

    // Byte accesses have no alignment concept: every byte address is legal and
    // the mask must be an identity there.
    #[test]
    fn byte_data_accesses_are_unaffected() {
        let mut mem = oracle_mem();
        let base = ORACLE_BASE;
        for (i, expect) in ORACLE_BYTES.iter().enumerate() {
            assert_eq!(
                mem.read_byte_data(base + i as u32, false).unwrap(),
                *expect,
                "ldb base+{i} must read the byte at base+{i}",
            );
        }
        // A byte store at an odd address stays exactly there.
        mem.write_byte_data(base + 3, 0x5A, false).unwrap();
        assert_eq!(mem.read_byte_data(base + 3, false).unwrap(), 0x5A);
        assert_eq!(mem.read_word_data(base, false).unwrap(), 0x80FF_005A);
        assert_eq!(mem.misaligned_data.count, 0);
    }

    // The observer / introspection family must stay faithful: it reports the
    // bytes actually at the requested address, and does NOT round down. The
    // tools that validate the emulator read through here; if they rounded,
    // their diagnostics would lie about memory contents.
    #[test]
    fn observer_reads_stay_faithful_and_do_not_round() {
        let mem = oracle_soc();
        let base = ORACLE_BASE;
        assert_eq!(
            mem.read_word(base + 1).unwrap(),
            0xFF00_AA11,
            "observer read_word(base+1) reports the bytes at base+1",
        );
        assert_eq!(mem.read_word(base + 3).unwrap(), 0xAA11_2233);
        assert_eq!(
            mem.read_half(base + 1).unwrap(),
            0xFF00,
            "observer read_half(base+1) reports the bytes at base+1",
        );
        // Contrast with the CPU data port at the same addresses.
        let mut mem = mem;
        assert_eq!(mem.read_word_data(base + 1, false).unwrap(), 0x80FF_00AA);
        assert_eq!(mem.read_half_data(base + 1, false).unwrap(), 0x80FF);
    }

    // The align-down is silent by construction, so the diagnostic log is what
    // turns this bug class from invisible into measurable.
    #[test]
    fn misaligned_data_accesses_are_logged() {
        let mut mem = oracle_mem();
        let base = ORACLE_BASE;
        let _ = mem.read_word_data(base + 1, false).unwrap();
        let _ = mem.read_half_data(base + 3, false).unwrap();
        mem.write_word_data(base + 2, 0, false).unwrap();
        let _ = mem.read_byte_data(base + 1, false).unwrap(); // not misaligned

        assert_eq!(mem.misaligned_data.count, 3);
        let hits = &mem.misaligned_data.hits;
        assert_eq!(hits.len(), 3);
        assert_eq!((hits[0].addr, hits[0].ea, hits[0].width, hits[0].is_write), (base + 1, base, 4, false));
        assert_eq!((hits[1].addr, hits[1].ea, hits[1].width, hits[1].is_write), (base + 3, base + 2, 2, false));
        assert_eq!((hits[2].addr, hits[2].ea, hits[2].width, hits[2].is_write), (base + 2, base, 4, true));
    }

    // The read trace is used as an address oracle for reconstructing data
    // regions, so it must record the address the core actually drove.
    #[test]
    fn read_trace_records_the_effective_address() {
        let mut mem = oracle_mem();
        let base = ORACLE_BASE;
        mem.read_trace = Some(ReadTrace { lo: 0, hi: 0x40, hits: Vec::new() });
        let _ = mem.read_word_data(base + 3, false).unwrap();
        let _ = mem.read_half_data(base + 3, false).unwrap();
        let hits = &mem.read_trace.as_ref().unwrap().hits;
        assert_eq!(hits[0], (base, 4, 0x80FF_00AA), "word trace records base+0");
        assert_eq!(hits[1], (base + 2, 2, 0x00AA), "half trace records base+2");
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
