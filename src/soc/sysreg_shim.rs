//! Transitional SYSREG fallback — hosts the legacy hardcoded arms from
//! the old `MmioController::sysreg_read_word` / `sysreg_write_word` plus
//! a word-wide backing store. Everything in `0x01000000..0x01003800`
//! that has not yet been carved into its own peripheral file is handled
//! here.
//!
//! **This file shrinks as sessions land.** Session 2 carves SerDes out,
//! Session 3 carves EPON MAC + MPCP, Session 4 carves MACsec + DMA,
//! Session 6 carves timer + NCO + clock_pll + efuse_udr + fatal_filter.
//! Once all subsystems are modelled natively, `sysreg_shim` should be
//! deleted entirely along with `SYSREG_INIT_VALUES`.
//!
//! Session 1 intentionally preserves the existing stub behaviour —
//! including the generic "bits 27–31 auto-clear" mechanism — so the
//! firmware boot-to-prompt path is not regressed by the refactor.
//! Audit 5.8 remains open here until each peripheral claims its
//! command-bit registers individually.

use std::collections::{HashMap, HashSet};

use crate::cpu::exception::Exception;
use crate::soc::peripheral::AddressRange;

pub const SYSREG_BASE: u32 = 0x01000000;
pub const SYSREG_SIZE: u32 = 0x3800;
pub const SYSREG_END: u32 = SYSREG_BASE + SYSREG_SIZE;

pub const SYSREG_RANGES: &[AddressRange] = &[AddressRange::new(SYSREG_BASE, SYSREG_END)];

/// Aggregated trace entry for `--dump-mmio-trace` / cold-boot catalog.
#[derive(Default, Clone, Debug)]
pub struct ShimTraceEntry {
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: u32,
    pub last_write_value: u32,
    pub first_pc: u32,
    pub first_insn: u64,
}

#[derive(Clone)]
pub struct SysregShim {
    pub trace: bool,
    sysreg_store: Vec<u32>,
    unhandled_logged: HashSet<u32>,
    pub mmio_trace: Option<HashMap<u32, ShimTraceEntry>>,
    pub current_pc: u32,
    pub current_insn: u64,
}

impl SysregShim {
    pub fn new() -> Self {
        let num = (SYSREG_SIZE / 4) as usize;
        Self {
            trace: false,
            sysreg_store: vec![0u32; num],
            unhandled_logged: HashSet::new(),
            mmio_trace: Some(HashMap::new()),
            current_pc: 0,
            current_insn: 0,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (SYSREG_BASE..SYSREG_END).contains(&addr)
    }

    #[inline]
    pub fn update_cpu_context(&mut self, pc: u32, insn: u64) {
        self.current_pc = pc;
        self.current_insn = insn;
    }

    /// Cold reset. Zeroes the residual backing store, then applies
    /// the silicon power-on snapshot from `SYSREG_INIT_VALUES`. The
    /// snapshot reflects values observed by `hardware probing` immediately
    /// after stage-1 hands off to stage-2 — i.e. true silicon
    /// defaults, not a post-boot snapshot.
    pub fn reset_cold(&mut self) {
        for slot in &mut self.sysreg_store {
            *slot = 0;
        }
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let idx = (off / 4) as usize;
            if idx < self.sysreg_store.len() {
                self.sysreg_store[idx] = val;
            }
        }
    }

    /// Warm reset — silicon power-on values are already applied by
    /// `reset_cold`. The cold/warm distinction lives in
    /// `boot_from_flash` (which pre-sets `STATUS32.E1/E2` only in
    /// warm mode).
    pub fn reset_warm(&mut self) {
        self.reset_cold();
    }

    /// No-op tick. The shim no longer owns any tick-driven state
    /// — Session 6 migrated the EPON free-running counter to
    /// `src/soc/timer.rs`.
    pub fn tick(&mut self, _cpu_instructions: u64) {}

    fn log_unhandled_read(&mut self, offset: u32, val: u32) {
        let aligned = offset & !3;
        if self.unhandled_logged.insert(aligned) {
            let abs = SYSREG_BASE + aligned;
            match super::mmio_blocks::lookup(abs) {
                Some(info) => crate::vlog!(
                    "[MMIO] UNHANDLED READ  sysreg+0x{:04X} (0x{:08X}) → 0x{:08X}  at PC=0x{:05X}  [#{} {}::{}]",
                    aligned, abs, val, self.current_pc, info.block_id, info.block_name, info.reg_name
                ),
                None => crate::vlog!(
                    "[MMIO] UNHANDLED READ  sysreg+0x{:04X} (0x{:08X}) → 0x{:08X}  at PC=0x{:05X}",
                    aligned, abs, val, self.current_pc
                ),
            }
        }
        if let Some(ref mut trace) = self.mmio_trace {
            let entry = trace.entry(aligned).or_insert_with(|| ShimTraceEntry {
                first_pc: self.current_pc,
                first_insn: self.current_insn,
                ..Default::default()
            });
            entry.reads += 1;
            entry.last_read_value = val;
        }
    }

    fn log_unhandled_write(&mut self, offset: u32, val: u32) {
        let aligned = offset & !3;
        let key = aligned | 0x8000_0000;
        if self.unhandled_logged.insert(key) {
            let abs = SYSREG_BASE + aligned;
            match super::mmio_blocks::lookup(abs) {
                Some(info) => crate::vlog!(
                    "[MMIO] UNHANDLED WRITE sysreg+0x{:04X} (0x{:08X}) = 0x{:08X}  at PC=0x{:05X}  [#{} {}::{}]",
                    aligned, abs, val, self.current_pc, info.block_id, info.block_name, info.reg_name
                ),
                None => crate::vlog!(
                    "[MMIO] UNHANDLED WRITE sysreg+0x{:04X} (0x{:08X}) = 0x{:08X}  at PC=0x{:05X}",
                    aligned, abs, val, self.current_pc
                ),
            }
        }
        if let Some(ref mut trace) = self.mmio_trace {
            let entry = trace.entry(aligned).or_insert_with(|| ShimTraceEntry {
                first_pc: self.current_pc,
                first_insn: self.current_insn,
                ..Default::default()
            });
            entry.writes += 1;
            entry.last_write_value = val;
        }
    }

    fn store_read(&self, offset: u32) -> u32 {
        let idx = (offset / 4) as usize;
        if idx < self.sysreg_store.len() {
            self.sysreg_store[idx]
        } else {
            0
        }
    }

    fn sysreg_read_word(&mut self, offset: u32) -> u32 {
        // Pure residual fallback. Deferral D7 bisected the
        // previously-required bits `[31:27]` auto-clear and
        // identified a single dependent register (`0x01000160`),
        // now owned by `epon_mac.rs::REG_MPCP_CMD_LATCH`. The
        // shim is a plain backing store for everything else.
        // Audit 5.8 finally resolved.
        let val = self.store_read(offset);
        self.log_unhandled_read(offset, val);
        val
    }

    fn sysreg_write_word(&mut self, offset: u32, val: u32) {
        self.log_unhandled_write(offset, val);
        let idx = (offset / 4) as usize;
        if idx < self.sysreg_store.len() {
            self.sysreg_store[idx] = val;
        }
    }

    // -------- public access surface used by the bank --------

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let off = addr - SYSREG_BASE;
        let val = self.sysreg_read_word(off);
        if self.trace {
            eprintln!(
                "[MMIO] read  word  0x{:08X} → 0x{:08X} (sysreg+0x{:04X})",
                addr, val, off
            );
        }
        Ok(val)
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let off = addr - SYSREG_BASE;
        self.sysreg_write_word(off, val);
        if self.trace {
            eprintln!(
                "[MMIO] write word  0x{:08X} = 0x{:08X} (sysreg+0x{:04X})",
                addr, val, off
            );
        }
        Ok(())
    }

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        let off = addr - SYSREG_BASE;
        let word_off = off & !3;
        let half_idx = (off >> 1) & 1;
        let word = self.sysreg_read_word(word_off);
        Ok((word >> (16 - half_idx * 16)) as u16)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        let off = addr - SYSREG_BASE;
        let word_off = off & !3;
        let half_idx = (off >> 1) & 1;
        let idx = (word_off / 4) as usize;
        if idx < self.sysreg_store.len() {
            let shift = 16 - half_idx * 16;
            let mask = !(0xFFFFu32 << shift);
            self.sysreg_store[idx] = (self.sysreg_store[idx] & mask) | ((val as u32) << shift);
        }
        Ok(())
    }

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        let off = addr - SYSREG_BASE;
        let word_off = off & !3;
        let byte_idx = off & 3;
        let word = self.sysreg_read_word(word_off);
        Ok((word >> (24 - byte_idx * 8)) as u8)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        let off = addr - SYSREG_BASE;
        let word_off = off & !3;
        let byte_idx = off & 3;
        let idx = (word_off / 4) as usize;
        if idx < self.sysreg_store.len() {
            let shift = 24 - byte_idx * 8;
            let mask = !(0xFFu32 << shift);
            self.sysreg_store[idx] =
                (self.sysreg_store[idx] & mask) | ((val as u32) << shift);
        }
        Ok(())
    }
}
