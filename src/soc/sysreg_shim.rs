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

pub struct SysregShim {
    pub trace: bool,
    pub timer_counter: u16,
    sysreg_store: Vec<u32>,
    sysreg_pending_clear: Vec<u32>,
    i2c_clock_toggles: u32,
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
            timer_counter: 0,
            sysreg_store: vec![0u32; num],
            sysreg_pending_clear: vec![0u32; num],
            i2c_clock_toggles: 0,
            unhandled_logged: HashSet::new(),
            mmio_trace: None,
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

    /// Cold reset. Leaves timer counter + snapshot-derived regs at 0.
    pub fn reset_cold(&mut self) {
        for slot in &mut self.sysreg_store {
            *slot = 0;
        }
        for slot in &mut self.sysreg_pending_clear {
            *slot = 0;
        }
        self.i2c_clock_toggles = 0;
        self.timer_counter = 0;
    }

    /// Warm reset — apply `SYSREG_INIT_VALUES` on top of cold reset.
    pub fn reset_warm(&mut self) {
        self.reset_cold();
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let idx = (off / 4) as usize;
            if idx < self.sysreg_store.len() {
                self.sysreg_store[idx] = val;
            }
        }
    }

    /// Advance the EPON MAC free-running counter. Called by
    /// `PeripheralBank::tick`. Prescaler is coarse — the actual ratio to
    /// the CPU clock is unverified (audit 3.1); Session 6 refines it.
    pub fn tick(&mut self, _cpu_instructions: u64) {
        self.timer_counter = self.timer_counter.wrapping_add(1);
    }

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

    fn store_read(&mut self, offset: u32) -> u32 {
        let idx = (offset / 4) as usize;
        if idx < self.sysreg_store.len() {
            let val = self.sysreg_store[idx];
            let clear_mask = self.sysreg_pending_clear[idx];
            if clear_mask != 0 {
                self.sysreg_store[idx] = val & !clear_mask;
                self.sysreg_pending_clear[idx] = 0;
            }
            val
        } else {
            0
        }
    }

    fn sysreg_read_word(&mut self, offset: u32) -> u32 {
        match offset {
            // ── EPON MAC core ──
            0x000 => 0x47010203, // CHIP_ID
            0x004 => 0xB2110816, // CHIP_REV
            0x00C => 0x0114B820, // LLID_CAPTURE_MASK
            0x018 => 0x00000006, // LLID_ACTIVE_BITMAP
            0x030 => 0x0000FFFF, // RX_GRANT_MASK
            0x050 => self.timer_counter as u32,
            0x048 => {
                let base = self.sysreg_store[0x048 / 4];
                if base & 0x10 != 0 {
                    base & !0x80000000
                } else {
                    base | 0x80000000
                }
            }
            0x04C => {
                let base = self.sysreg_store[0x04C / 4];
                base | 0x80000000
            }
            // BSC registers 0x140/0x14C/0x150 are NOT routed here — the
            // BscI2c peripheral claims them first in the bank router.
            // SerDes lane lock registers 0x194/0x1D4 likewise now route
            // to the SerDes peripheral (Session 2, audit 5.3).
            0x1E0 => 0x45504F4E, // EPON signature ("EPON")
            o if (o & 0x1FF) == 0x1D8 && (0x15D8..=0x1FD8).contains(&o) => 0,
            o @ 0x1400..=0x3FFF if (o.wrapping_sub(0x143C)) % 0x200 == 0 => {
                self.store_read(offset) | 0x100
            }
            0x1404 | 0x1604 | 0x1804 | 0x1A04 | 0x1C04 | 0x1E04 => 0,
            0x2804 => 0,
            0x0064 => {
                let val = self.store_read(offset);
                (val & 0xFFFF_0000) | 0x0000_FFFF
            }
            0x3604 => 0,
            _ => {
                let val = self.store_read(offset);
                self.log_unhandled_read(offset, val);
                val
            }
        }
    }

    fn sysreg_write_word(&mut self, offset: u32, val: u32) {
        match offset {
            0x000 | 0x004 | 0x00C | 0x018 | 0x030 | 0x050 | 0x040 | 0x048 | 0x04C
            | 0x1E0 | 0x2804 | 0x3604 | 0x1404 | 0x1604 | 0x1804
            | 0x1A04 | 0x1C04 | 0x1E04 => {}
            o if (0x1400..=0x3FFF).contains(&o) && (o.wrapping_sub(0x143C)) % 0x200 == 0 => {}
            _ => self.log_unhandled_write(offset, val),
        }

        if offset == 0x04C {
            let old = self.sysreg_store[0x04C / 4];
            if (val & 1) != 0 && (old & 1) == 0 {
                self.i2c_clock_toggles += 1;
            }
        }
        if offset == 0x040 {
            let old = self.sysreg_store[0x040 / 4];
            if (val & 0x8000) != 0 && (old & 0x8000) == 0 {
                self.i2c_clock_toggles = 0;
            }
        }

        // LLID rx/tx config anchors (LLID 0/31): HW clears bit 0 on write-back.
        let val = if matches!(offset, 0x043C | 0x04B8 | 0x0D00 | 0x0D7C) {
            val & !0x0001
        } else {
            val
        };

        let idx = (offset / 4) as usize;
        if idx < self.sysreg_store.len() {
            self.sysreg_store[idx] = val;
            let cmd_bits = val & 0xF800_0000;
            if cmd_bits != 0 {
                self.sysreg_pending_clear[idx] = cmd_bits;
            }
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
