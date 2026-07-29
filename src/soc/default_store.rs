//! Residual register store for MMIO addresses not claimed by any
//! peripheral. Stores writes verbatim and returns them on subsequent
//! reads. NO magic — no auto-clear, no side effects, no forced bits.
//! This is deliberately a dumb store so that audit item 5.8 (generic
//! bits 27-31 auto-clear) can never regress.
//!
//! When a peripheral lands, it claims a range from the residual store by
//! declaring it via `Peripheral::address_ranges()`. The residual store
//! shrinks as the peripheral coverage grows, and is expected to go to
//! zero once all 15 subsystems are modelled.

use std::collections::HashMap;

use crate::cpu::exception::Exception;
use crate::soc::peripheral::AddressRange;

/// Trace entry for unhandled residual accesses — used by
/// `--dump-mmio-trace` to build the cold-boot inventory of registers the
/// firmware touches that are not yet modelled by a peripheral.
#[derive(Default, Clone, Debug)]
pub struct ResidualTraceEntry {
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: u32,
    pub last_write_value: u32,
    pub first_pc: u32,
    pub first_insn: u64,
    /// Bit 0=byte, bit 1=half, bit 2=word.
    pub access_widths: u8,
}

/// Dumb register store for an MMIO range. Used as the default fallback
/// when no peripheral claims an address inside its covered window.
#[derive(Clone)]
pub struct DefaultRegisterStore {
    /// Covered range. All accesses must fall inside; the bank guarantees
    /// this by only routing unclaimed addresses here.
    range: AddressRange,
    /// Backing storage indexed by `(addr - range.start) / 4`.
    store: Vec<u32>,
    /// Optional access trace for `--dump-mmio-trace`.
    pub trace: Option<HashMap<u32, ResidualTraceEntry>>,
    /// Set of offsets already logged via `vlog!` (first-access logging).
    logged: std::collections::HashSet<u32>,
    /// Current CPU context for trace entries.
    pub current_pc: u32,
    pub current_insn: u64,
}

impl DefaultRegisterStore {
    pub fn new(range: AddressRange) -> Self {
        let num = ((range.end - range.start) / 4) as usize;
        Self {
            range,
            store: vec![0u32; num],
            trace: None,
            logged: std::collections::HashSet::new(),
            current_pc: 0,
            current_insn: 0,
        }
    }

    #[inline]
    pub fn range(&self) -> AddressRange {
        self.range
    }

    #[inline]
    pub fn contains(&self, addr: u32) -> bool {
        self.range.contains(addr)
    }

    fn idx(&self, addr: u32) -> usize {
        ((addr - self.range.start) / 4) as usize
    }

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let idx = self.idx(addr);
        let val = self.store.get(idx).copied().unwrap_or(0);
        self.log_read(addr & !3, val, 4);
        Ok(val)
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let idx = self.idx(addr);
        if let Some(slot) = self.store.get_mut(idx) {
            *slot = val;
        }
        self.log_write(addr & !3, val, 4);
        Ok(())
    }

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        let word_addr = addr & !3;
        let byte_idx = addr & 3;
        let word = self.store.get(self.idx(word_addr)).copied().unwrap_or(0);
        let val = (word >> (24 - byte_idx * 8)) as u8;
        self.log_read(word_addr, word, 1);
        Ok(val)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        let word_addr = addr & !3;
        let byte_idx = addr & 3;
        let idx = self.idx(word_addr);
        if let Some(slot) = self.store.get_mut(idx) {
            let shift = 24 - byte_idx * 8;
            let mask = !(0xFFu32 << shift);
            *slot = (*slot & mask) | ((val as u32) << shift);
            let new = *slot;
            self.log_write(word_addr, new, 1);
        }
        Ok(())
    }

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        let word_addr = addr & !3;
        let half_idx = (addr >> 1) & 1;
        let word = self.store.get(self.idx(word_addr)).copied().unwrap_or(0);
        let val = (word >> (16 - half_idx * 16)) as u16;
        self.log_read(word_addr, word, 2);
        Ok(val)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        let word_addr = addr & !3;
        let half_idx = (addr >> 1) & 1;
        let idx = self.idx(word_addr);
        if let Some(slot) = self.store.get_mut(idx) {
            let shift = 16 - half_idx * 16;
            let mask = !(0xFFFFu32 << shift);
            *slot = (*slot & mask) | ((val as u32) << shift);
            let new = *slot;
            self.log_write(word_addr, new, 2);
        }
        Ok(())
    }

    fn log_read(&mut self, word_addr: u32, val: u32, width: u32) {
        if self.logged.insert(word_addr) {
            crate::vlog!(
                "[MMIO] UNHANDLED READ  0x{:08X} → 0x{:08X}  at PC=0x{:05X}",
                word_addr, val, self.current_pc
            );
        }
        if let Some(ref mut trace) = self.trace {
            let entry = trace.entry(word_addr).or_insert_with(|| ResidualTraceEntry {
                first_pc: self.current_pc,
                first_insn: self.current_insn,
                ..Default::default()
            });
            entry.reads += 1;
            entry.last_read_value = val;
            entry.access_widths |= match width {
                1 => 1,
                2 => 2,
                4 => 4,
                _ => 0,
            };
        }
    }

    fn log_write(&mut self, word_addr: u32, val: u32, width: u32) {
        let key = word_addr | 0x8000_0000;
        if self.logged.insert(key) {
            crate::vlog!(
                "[MMIO] UNHANDLED WRITE 0x{:08X} = 0x{:08X}  at PC=0x{:05X}",
                word_addr, val, self.current_pc
            );
        }
        if let Some(ref mut trace) = self.trace {
            let entry = trace.entry(word_addr).or_insert_with(|| ResidualTraceEntry {
                first_pc: self.current_pc,
                first_insn: self.current_insn,
                ..Default::default()
            });
            entry.writes += 1;
            entry.last_write_value = val;
            entry.access_widths |= match width {
                1 => 1,
                2 => 2,
                4 => 4,
                _ => 0,
            };
        }
    }

    /// Load a slice of `(offset_from_range_start, value)` pairs. Used by
    /// peripherals that migrate their `reset_warm()` state out of
    /// `mmio_init.rs` — and by the residual store itself when loaded
    /// with the remaining unclaimed offsets.
    pub fn load_init_slice(&mut self, pairs: &[(u32, u32)]) {
        for &(off, val) in pairs {
            let idx = (off / 4) as usize;
            if idx < self.store.len() {
                self.store[idx] = val;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn store_roundtrip_word() {
        let mut s = DefaultRegisterStore::new(AddressRange::new(0x1000_0000, 0x1000_0100));
        s.write_word(0x1000_0004, 0xDEADBEEF).unwrap();
        assert_eq!(s.read_word(0x1000_0004).unwrap(), 0xDEADBEEF);
    }

    #[test]
    fn store_no_autoclear_of_high_bits() {
        let mut s = DefaultRegisterStore::new(AddressRange::new(0x1000_0000, 0x1000_0100));
        s.write_word(0x1000_0000, 0xF800_0042).unwrap();
        // Guard: bits 27-31 must NOT be auto-cleared.
        assert_eq!(s.read_word(0x1000_0000).unwrap(), 0xF800_0042);
        assert_eq!(s.read_word(0x1000_0000).unwrap(), 0xF800_0042);
    }

    #[test]
    fn store_byte_and_half() {
        let mut s = DefaultRegisterStore::new(AddressRange::new(0x1000_0000, 0x1000_0100));
        s.write_word(0x1000_0008, 0x11223344).unwrap();
        assert_eq!(s.read_byte(0x1000_0008).unwrap(), 0x11);
        assert_eq!(s.read_byte(0x1000_0009).unwrap(), 0x22);
        assert_eq!(s.read_half(0x1000_000A).unwrap(), 0x3344);
    }
}
