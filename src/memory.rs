use std::cell::RefCell;

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

    pub fn read_byte(&self, addr: u32) -> Result<u8, Exception> {
        if self.mmio.is_some() {
            // SoC mode: route by address
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

    pub fn read_half(&self, addr: u32) -> Result<u16, Exception> {
        if self.mmio.is_some() {
            // SoC mode: check SRAM first, then MMIO, then unmapped
            if let Some(off) = self.sram_offset(addr) {
                // Misaligned SRAM reads: perform byte-wise (fixup behavior).
                // The ARC 700 raises a MisalignedAccess exception, but the firmware's
                // exception handler fixes up the access. We emulate this directly.
                if off + 1 < self.data.len() {
                    return Ok(((self.data[off] as u16) << 8) | (self.data[off + 1] as u16));
                }
            }
            if addr & 1 != 0 {
                // Unmapped misaligned — return 0
                return Ok(0);
            }
            if let Some(ref mmio) = self.mmio {
                return mmio.borrow_mut().read_half(addr);
            }
            // Unmapped address — return 0
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
        if self.mmio.is_some() {
            // SoC mode: check SRAM first, then MMIO, then unmapped
            if addr & 3 != 0 {
                // Misaligned SRAM reads: perform byte-wise fixup
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
            // Unmapped address — return 0
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
            // Misaligned SRAM writes: perform byte-wise fixup.
            // The ARC 700 raises MisalignedAccess, but the firmware's exception
            // handler fixes up the access. We emulate this directly.
            if let Some(off) = self.sram_offset(addr) {
                if off + 1 < self.data.len() {
                    self.check_watchpoint(off, 2);
                    self.data[off] = (val >> 8) as u8;
                    self.data[off + 1] = val as u8;
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

    /// Apply pending DMA transfers from the PBC
    fn apply_pending_dma(&mut self) {
        if let Some(ref mmio) = self.mmio {
            // Apply flash -> DCCM reads
            let writes = mmio.borrow_mut().pbc.take_pending_dma();
            for dma_write in writes {
                let start = dma_write.dccm_addr as usize;
                let end = start + dma_write.data.len();
                if end <= self.data.len() {
                    self.data[start..end].copy_from_slice(&dma_write.data);
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

    // ========== Instruction fetch (SRAM in SoC mode, flat otherwise) ==========

    pub fn fetch_half(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        if self.mmio.is_some() {
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
}
