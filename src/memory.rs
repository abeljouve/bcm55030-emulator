use std::cell::RefCell;
use std::collections::HashSet;

use crate::cpu::exception::Exception;
use crate::soc::mmio::MmioController;

/// BCM55030 DCCM size: 512 KB
pub const DCCM_SIZE: usize = 512 * 1024;
/// BCM55030 ICCM size: 512 KB
pub const ICCM_SIZE: usize = 512 * 1024;

pub struct Memory {
    /// Primary data store.
    /// - Flat mode: single unified memory (for tests).
    /// - Harvard mode: DCCM (data closely coupled memory).
    data: Vec<u8>,

    /// Instruction memory (Harvard mode only).
    /// When present, `fetch_half`/`fetch_word` read from here.
    iccm: Option<Vec<u8>>,

    /// MMIO peripheral controller (Harvard mode only).
    /// Uses RefCell for interior mutability: reads may have side effects
    /// (e.g. UART status register clears on read).
    mmio: Option<RefCell<MmioController>>,

    /// ICCM base address (Harvard mode). Instruction fetch at PC in
    /// [iccm_base, iccm_base + iccm_size) reads from ICCM.
    /// Default 0 for bootloader, 0x20000000 for firmware.
    pub iccm_base: u32,

    /// DCCM base address (Harvard mode). Data access at addr in
    /// [dccm_base, dccm_base + dccm_size) reads/writes DCCM.
    /// Default 0 for bootloader, 0x20000000 for firmware.
    pub dccm_base: u32,

    /// Size of the loaded app binary (used for BSS clearing in boot ROM CRT init)
    pub app_size: Option<usize>,

    /// Protect the firmware code section in DCCM from event_table_clear corruption.
    ///
    /// The firmware firmware's event groups have entry tables that overlap with the code
    /// section in DCCM (since ICCM and DCCM both contain the firmware binary, but code
    /// runs from ICCM while data is in DCCM). The compiler places PCL-relative literal
    /// pool constants (pointers, table bases) between code sequences.
    ///
    /// `event_table_clear_all_counter_entries()` writes 0 to counter slots within
    /// these entry tables, inadvertently zeroing PCL-relative constants. On real
    /// hardware, the FDS scan rebuilds these counters from flash records, restoring
    /// the overlapping constants. But the scan itself needs these constants to locate
    /// flash regions — creating a circular dependency on first boot.
    ///
    /// We break this cycle by suppressing zero-writes to non-zero halfwords in the
    /// firmware code section (0 to app_size). This preserves PCL-relative constants while
    /// still allowing legitimate data writes (non-zero values).
    ///
    /// Set to app_size after firmware binary is loaded. 0 = protection disabled.
    firmware_code_protect_end: usize,

    /// COW tracking for the firmware code section.
    /// On ARC 700 with overlapping ICCM/DCCM, data reads go to DCCM while
    /// instruction fetch goes to ICCM. Both are initialized with the same binary.
    /// If the firmware (or a cascading MMIO bug) writes to a DCCM address in the
    /// code section, the literal pool values at that address diverge from ICCM.
    ///
    /// This set tracks which word-aligned offsets in the code section have been
    /// explicitly written. For reads:
    /// - Written addresses → return DCCM value (firmware's intent)
    /// - Unwritten addresses → return ICCM value (pristine literal pools)
    ///
    /// This makes PCL-relative literal pools resilient to accidental corruption
    /// from overlapping data structures or wrong code paths due to incomplete MMIO.
    code_section_written: HashSet<usize>,
}

impl Memory {
    /// Create a flat memory (single address space, for tests).
    pub fn new(size: usize) -> Self {
        Self {
            data: vec![0u8; size],
            iccm: None,
            mmio: None,
            iccm_base: 0,
            dccm_base: 0,
            app_size: None,
            firmware_code_protect_end: 0,
            code_section_written: HashSet::new(),
        }
    }

    /// Create a Harvard-architecture memory with separate ICCM, DCCM, and MMIO.
    pub fn new_harvard(iccm_size: usize, dccm_size: usize) -> Self {
        Self {
            data: vec![0u8; dccm_size],
            iccm: Some(vec![0u8; iccm_size]),
            mmio: Some(RefCell::new(MmioController::new())),
            iccm_base: 0,
            dccm_base: 0,
            app_size: None,
            firmware_code_protect_end: 0,
            code_section_written: HashSet::new(),
        }
    }

    /// Enable protection of the firmware code section against event_table_clear corruption.
    /// Called after firmware binary is loaded to DCCM, before execution starts.
    pub fn protect_firmware_literals(&mut self) {
        if let Some(size) = self.app_size {
            self.firmware_code_protect_end = size;
            eprintln!("[BCM55030] DCCM code section protection: 0x0000-0x{:04X}", size);
        }
    }

    pub fn is_harvard(&self) -> bool {
        self.iccm.is_some()
    }

    pub fn dccm_size(&self) -> usize {
        self.data.len()
    }

    /// Load a binary blob into the primary data store (flat) or DCCM (Harvard).
    pub fn load_binary(&mut self, addr: u32, binary: &[u8]) {
        let start = addr as usize;
        let end = start + binary.len();
        assert!(end <= self.data.len(), "binary exceeds memory size");
        self.data[start..end].copy_from_slice(binary);
    }

    /// Load a binary blob into ICCM (Harvard mode).
    /// In flat mode, falls back to load_binary.
    pub fn load_iccm(&mut self, addr: u32, binary: &[u8]) {
        if let Some(ref mut iccm) = self.iccm {
            let start = addr as usize;
            let end = start + binary.len();
            assert!(end <= iccm.len(), "binary exceeds ICCM size");
            iccm[start..end].copy_from_slice(binary);
        } else {
            self.load_binary(addr, binary);
        }
    }

    /// Access the MMIO controller (Harvard mode only).
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

    // ========== Data reads (DCCM in Harvard, flat otherwise) ==========

    /// Check if an address falls in the DCCM range and return the offset.
    #[inline]
    fn dccm_offset(&self, addr: u32) -> Option<usize> {
        if addr >= self.dccm_base {
            let off = (addr - self.dccm_base) as usize;
            if off < self.data.len() {
                return Some(off);
            }
        }
        None
    }

    pub fn read_byte(&self, addr: u32) -> Result<u8, Exception> {
        if self.iccm.is_some() {
            // Harvard mode: route by address
            if let Some(off) = self.dccm_offset(addr) {
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
        if self.iccm.is_some() {
            // Harvard mode: check DCCM first, then MMIO, then unmapped
            if let Some(off) = self.dccm_offset(addr) {
                // Misaligned DCCM reads: perform byte-wise (fixup behavior).
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
        if self.iccm.is_some() {
            // Harvard mode: check DCCM first, then MMIO, then unmapped
            if addr & 3 != 0 {
                // Misaligned DCCM reads: perform byte-wise fixup
                if let Some(off) = self.dccm_offset(addr) {
                    if off + 3 < self.data.len() {
                        return Ok(((self.data[off] as u32) << 24)
                            | ((self.data[off + 1] as u32) << 16)
                            | ((self.data[off + 2] as u32) << 8)
                            | (self.data[off + 3] as u32));
                    }
                }
                return Ok(0);
            }
            if let Some(off) = self.dccm_offset(addr) {
                if off + 3 < self.data.len() {
                    // COW: for unwritten code section words, return ICCM (pristine literal pools)
                    if off < self.firmware_code_protect_end && !self.code_section_written.contains(&(off & !3)) {
                        if let Some(ref iccm) = self.iccm {
                            if off + 3 < iccm.len() {
                                return Ok(((iccm[off] as u32) << 24)
                                    | ((iccm[off + 1] as u32) << 16)
                                    | ((iccm[off + 2] as u32) << 8)
                                    | (iccm[off + 3] as u32));
                            }
                        }
                    }
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

    // ========== Data writes (DCCM in Harvard, flat otherwise) ==========

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if self.iccm.is_some() {
            if let Some(off) = self.dccm_offset(addr) {
                if off < self.firmware_code_protect_end {
                    self.code_section_written.insert(off & !3);
                }
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
        if self.iccm.is_some() {
            // Misaligned DCCM writes: perform byte-wise fixup.
            // The ARC 700 raises MisalignedAccess, but the firmware's exception
            // handler fixes up the access. We emulate this directly.
            if let Some(off) = self.dccm_offset(addr) {
                if off + 1 < self.data.len() {
                    // Protect firmware code section: suppress zero-writes to non-zero halfwords.
                    // This preserves PCL-relative literal pool constants that overlap
                    // with event counter table entries.
                    if val == 0 && off < self.firmware_code_protect_end {
                        let existing = ((self.data[off] as u16) << 8) | (self.data[off + 1] as u16);
                        if existing != 0 {
                            return Ok(()); // Suppress: would zero a code-section constant
                        }
                    }
                    if off < self.firmware_code_protect_end {
                        self.code_section_written.insert(off & !3);
                    }
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
        if self.iccm.is_some() {
            if addr & 3 != 0 {
                // Misaligned DCCM writes: perform byte-wise fixup
                if let Some(off) = self.dccm_offset(addr) {
                    if off + 3 < self.data.len() {
                        if off < self.firmware_code_protect_end {
                            self.code_section_written.insert(off & !3);
                        }
                        self.data[off] = (val >> 24) as u8;
                        self.data[off + 1] = (val >> 16) as u8;
                        self.data[off + 2] = (val >> 8) as u8;
                        self.data[off + 3] = val as u8;
                        return Ok(());
                    }
                }
                return Ok(()); // Unmapped misaligned write — absorb
            }
            if let Some(off) = self.dccm_offset(addr) {
                if off + 3 < self.data.len() {
                    if off < self.firmware_code_protect_end {
                        self.code_section_written.insert(off & !3);
                    }
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

    // ========== Instruction fetch (ICCM in Harvard, flat otherwise) ==========

    /// Check if an address falls in the ICCM range and return the offset.
    #[inline]
    fn iccm_offset(&self, addr: u32) -> Option<usize> {
        if let Some(ref iccm) = self.iccm {
            if addr >= self.iccm_base {
                let off = (addr - self.iccm_base) as usize;
                if off < iccm.len() {
                    return Some(off);
                }
            }
        }
        None
    }

    pub fn fetch_half(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        if let Some(ref iccm) = self.iccm {
            if let Some(a) = self.iccm_offset(addr) {
                if a + 1 < iccm.len() {
                    return Ok(((iccm[a] as u16) << 8) | (iccm[a + 1] as u16));
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
        if let Some(ref iccm) = self.iccm {
            if let Some(a) = self.iccm_offset(addr) {
                if a + 3 < iccm.len() {
                    return Ok(((iccm[a] as u32) << 24)
                        | ((iccm[a + 1] as u32) << 16)
                        | ((iccm[a + 2] as u32) << 8)
                        | (iccm[a + 3] as u32));
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
    fn test_harvard_separate_buses() {
        let mut mem = Memory::new_harvard(1024, 1024);
        // Write to ICCM
        mem.load_iccm(0, &[0xAA, 0xBB, 0xCC, 0xDD]);
        // Write to DCCM at same address
        mem.load_binary(0, &[0x11, 0x22, 0x33, 0x44]);
        // Fetch reads ICCM
        assert_eq!(mem.fetch_half(0).unwrap(), 0xAABB);
        assert_eq!(mem.fetch_word(0).unwrap(), 0xAABBCCDD);
        // Data reads DCCM
        assert_eq!(mem.read_half(0).unwrap(), 0x1122);
        assert_eq!(mem.read_word(0).unwrap(), 0x11223344);
    }

    #[test]
    fn test_harvard_mmio_stub() {
        let mem = Memory::new_harvard(1024, 1024);
        // MMIO reads return 0 for unstubbed addresses
        assert_eq!(mem.read_byte(0x00FC0000).unwrap(), 0);
        assert_eq!(mem.read_word(0xDE000000).unwrap(), 0);
    }
}
