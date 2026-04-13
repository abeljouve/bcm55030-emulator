use std::collections::{HashMap, HashSet};

use crate::cpu::exception::Exception;
use super::uart::SimpleUart;
use super::pbc::PeripheralBusController;
use super::sfp_eeprom;

/// BSC I2C controller state (SFP EEPROM at sysreg 0x140/0x14C/0x150).
#[derive(Default)]
pub struct BscI2cState {
    pub base_addr: u16,
    pub pending_read_off: u16,
    pub read_ready: bool,
    /// 0 = A0h ID page, 1 = A2h DDM page.
    pub device: u8,
}

/// One unhandled MMIO access stat (aggregated per address).
#[derive(Default, Clone)]
pub struct MmioTraceEntry {
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: u32,
    pub last_write_value: u32,
    pub first_pc: u32,
    pub first_insn: u64,
}

/// UART MMIO block: 0x00FC1000-0x00FC10FF (256 bytes).
/// Single controller mirrored 8× every 0x20 bytes (verified on real HW).
/// Registers per channel: DATA=+0x10, STATUS/CTL=+0x14, BAUD_LO=+0x18, BAUD_HI=+0x1C.
const UART_RANGE_START: u32 = 0x00FC1000;
const UART_RANGE_SIZE: u32 = 0x100; // 8 channels × 0x20
/// Per-channel register stride
const UART_CHANNEL_SIZE: u32 = 0x20;
/// Offset within a channel where the known registers start (DATA at 0x10)
const UART_REG_OFFSET: u32 = 0x10;

/// Peripheral Bus Controller (SPI + MDIO) base address
const PBC_BASE: u32 = 0x010001F0;
const PBC_SIZE: u32 = 0x50; // +0x00 through +0x4F

/// BCM55030 EPON MAC / SoC register block.
/// Covers all MMIO registers from CHIP_ID through Channel Config Register.
/// Resolved from 85 hwregs base pointers: the full MMIO space spans
/// 0x01000000 to 0x010037B4. We round up to 0x3800.
const SYSREG_BASE: u32 = 0x01000000;
const SYSREG_SIZE: u32 = 0x3800;

/// SerDes lane status registers (firmware scans these at startup)
const SERDES_BASE: u32 = 0x224A0000;
const SERDES_SIZE: u32 = 0x0800; // 256 lanes × 8 bytes

/// MMIO controller — dispatches memory-mapped I/O accesses to peripherals
pub struct MmioController {
    pub uart: SimpleUart,
    pub pbc: PeripheralBusController,
    pub trace: bool,
    /// Current CPU PC — set by the CPU step loop before MMIO access.
    /// Used to provide context in unhandled register warnings.
    pub current_pc: u32,
    /// Current blink (caller return address) — set by the CPU step loop.
    /// Used by the watchpoint to identify which function called the writer.
    pub current_blink: u32,
    /// BCM55030 EPON MAC timer counter at SYSREG+0x050.
    /// Read by timer1_get_current_value (0x45E4) as a 16-bit hardware counter.
    /// Incremented each time Timer1 interrupt fires.
    pub timer_counter: u16,
    /// BCM55030 SoC register storage for read-write registers.
    /// Covers the full MMIO space (0x01000000-0x010037FF). Uninitialized
    /// entries default to 0. PBC addresses (0x1F0-0x23F) are handled
    /// separately by the PBC dispatcher (checked first).
    sysreg_store: Vec<u32>,
    /// Bits pending auto-clear: when the firmware writes command bits (27-31) to a
    /// register, the hardware clears them after processing. We clear them on the next
    /// read (simulating instant completion).
    sysreg_pending_clear: Vec<u32>,
    /// I2C bit-bang state for SYSREG+0x48/0x4C. Despite the old name this
    /// block is NOT the SFP I2C controller — that lives at
    /// SYSREG+0x140/0x14C/0x150 (see `bsc`). This bit-bang is likely the
    /// eFuse UDR bus (64-byte OTP reads via `serial_bus_read_80bytes`).
    /// Counts clock toggles to simulate a benign empty response.
    i2c_clock_toggles: u32,
    /// BSC I2C state machine for SFP EEPROM reads (SYSREG+0x140/0x14C/0x150).
    pub bsc: BscI2cState,
    /// Track which unhandled SYSREG offsets have been logged (first-access only).
    /// Prevents flooding from polling loops while showing every unique register.
    unhandled_logged: HashSet<u32>,
    /// Optional aggregated trace of all unhandled MMIO accesses.
    /// Enabled via `--dump-mmio-trace`. Indexed by sysreg offset (word-aligned).
    /// Used to inventory which registers a CLI command touches (Phase 2 prep).
    pub mmio_trace: Option<HashMap<u32, MmioTraceEntry>>,
    /// Current CPU instruction count, for trace timestamps.
    pub current_insn: u64,
}

impl MmioController {
    pub fn new() -> Self {
        let num_entries = SYSREG_SIZE as usize / 4;
        let mut sysreg_store = vec![0u32; num_entries];
        // Pre-populate from live HW snapshot (post-boot state captured 2026-04-10
        // on running ONU). 304 non-zero registers. See src/soc/mmio_init.rs and
        // docs/hw_snapshot_full.txt for the source data.
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let idx = (off / 4) as usize;
            if idx < sysreg_store.len() {
                sysreg_store[idx] = val;
            }
        }
        Self {
            uart: SimpleUart::new(),
            pbc: PeripheralBusController::new(),
            trace: false,
            current_pc: 0,
            current_blink: 0,
            timer_counter: 0,
            sysreg_store,
            sysreg_pending_clear: vec![0u32; num_entries],
            i2c_clock_toggles: 0,
            bsc: BscI2cState::default(),
            unhandled_logged: HashSet::new(),
            mmio_trace: None,
            current_insn: 0,
        }
    }

    #[inline]
    fn is_uart(addr: u32) -> bool {
        addr >= UART_RANGE_START && addr < UART_RANGE_START + UART_RANGE_SIZE
    }

    /// Map an absolute UART address to the uart.rs register offset (0x00-0x0C).
    /// Returns None for per-channel offsets below 0x10 (unknown registers).
    fn uart_reg_offset(addr: u32) -> Option<u32> {
        let channel_off = (addr - UART_RANGE_START) % UART_CHANNEL_SIZE;
        if channel_off >= UART_REG_OFFSET {
            Some(channel_off - UART_REG_OFFSET)
        } else {
            None // Unknown register in lower part of channel
        }
    }

    #[inline]
    fn is_pbc(addr: u32) -> bool {
        addr >= PBC_BASE && addr < PBC_BASE + PBC_SIZE
    }

    #[inline]
    fn is_sysreg(addr: u32) -> bool {
        addr >= SYSREG_BASE && addr < SYSREG_BASE + SYSREG_SIZE
    }

    /// Sysreg reads. Fixed values for HW regs; rest via store_read with cmd-bit auto-clear.
    fn sysreg_read_word(&mut self, offset: u32) -> u32 {
        match offset {
            // ── EPON MAC core (0x000-0x1EF) ──────────────────────────────
            0x000 => 0x47010203, // CHIP_ID (BCM4701)
            0x004 => 0xB2110816, // CHIP_REV / bond options
            0x00C => 0x0114B820, // LLID_CAPTURE_MASK
            0x018 => 0x00000006, // LLID_ACTIVE_BITMAP
            0x030 => 0x0000FFFF, // RX_GRANT_MASK
            0x050 => self.timer_counter as u32, // Free-running timer counter
            0x048 => {
                // I2C status register for SFP EEPROM bit-bang bus.
                // Bit 31 = SDA input line. Bit 4 = ACK enable (set by firmware).
                let base = self.sysreg_store[0x048 / 4];
                if base & 0x10 != 0 {
                    base & !0x80000000 // bit 4 set → ACK: SDA low
                } else {
                    base | 0x80000000  // bit 4 clear → SDA high (idle/stop)
                }
            }
            0x04C => {
                // I2C clock/data register. Bit 0 = SCL, bit 31 = SDA (data in).
                let base = self.sysreg_store[0x04C / 4];
                base | 0x80000000 // SDA high = data bit 1 (0xFF bytes)
            }
            // ── BSC I2C (SFP EEPROM) — 0x140/0x14C/0x150 ─────────────────
            0x140 => self.store_read(offset),
            0x14C => {
                if self.bsc.read_ready {
                    let w = sfp_eeprom::read_word(self.bsc.device, self.bsc.pending_read_off);
                    self.bsc.read_ready = false;
                    w
                } else {
                    self.store_read(offset)
                }
            }
            // 0x150: bit 31 busy flag, auto-clear on read.
            0x150 => {
                let idx = 0x150 / 4;
                let val = self.sysreg_store[idx];
                self.sysreg_store[idx] = val & !0x80000000;
                val & !0x80000000
            }
            0x194 | 0x1D4 => {
                // SerDes lane link lock: force bits 1,3 (locked) to break polling loops.
                let base = self.sysreg_store[(offset / 4) as usize];
                base | 0x0A
            }
            0x1E0 => 0x45504F4E, // EPON signature ("EPON")

            // HW counter result regs (stride 0x200 base 0x15D8): 0 on quiescent ONU.
            o if (o & 0x1FF) == 0x1D8 && (0x15D8..=0x1FD8).contains(&o) => 0,

            // DMA Channel Queue Drain (base 0x143C, stride 0x200): force bit 8 = drain done.
            o @ 0x1400..=0x3FFF if (o.wrapping_sub(0x143C)) % 0x200 == 0 => {
                self.store_read(offset) | 0x100
            }

            // LLID interrupt status: force 0 to prevent false state changes.
            0x1404 | 0x1604 | 0x1804 | 0x1A04 | 0x1C04 | 0x1E04 => 0,

            0x2804 => 0, // Fatal error status — quiescent ONU.

            // MDIO data reg: force low 16 = 0xFFFF (no PHY pulldown).
            0x0064 => {
                let val = self.store_read(offset);
                (val & 0xFFFF_0000) | 0x0000_FFFF
            }

            // SerDes error status — W1C, force 0 to prevent macsec_hw_session_init loop.
            0x3604 => 0,

            _ => {
                self.log_unhandled_read(offset);
                self.store_read(offset)
            }
        }
    }

    /// Read from the read-write store with auto-clear for command bits (27-31).
    #[inline]
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

    /// Log the first read from an unhandled SYSREG offset.
    /// Shows PC context for Ghidra reverse engineering.
    /// Also accumulates into the optional `mmio_trace` map.
    fn log_unhandled_read(&mut self, offset: u32) {
        let aligned = offset & !3;
        let idx = (aligned / 4) as usize;
        let val = if idx < self.sysreg_store.len() { self.sysreg_store[idx] } else { 0 };
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
            let entry = trace.entry(aligned).or_insert_with(|| MmioTraceEntry {
                first_pc: self.current_pc,
                first_insn: self.current_insn,
                ..Default::default()
            });
            entry.reads += 1;
            entry.last_read_value = val;
        }
    }

    /// Log the first write to an unhandled SYSREG offset.
    /// Also accumulates into the optional `mmio_trace` map.
    fn log_unhandled_write(&mut self, offset: u32, val: u32) {
        let aligned = offset & !3;
        // Use offset | 0x80000000 to distinguish write logs from read logs in the set
        let key = aligned | 0x80000000;
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
            let entry = trace.entry(aligned).or_insert_with(|| MmioTraceEntry {
                first_pc: self.current_pc,
                first_insn: self.current_insn,
                ..Default::default()
            });
            entry.writes += 1;
            entry.last_write_value = val;
        }
    }

    /// BCM55030 SoC register writes.
    fn sysreg_write_word(&mut self, offset: u32, val: u32) {
        // Log unhandled writes (offsets without explicit read handlers)
        match offset {
            0x000 | 0x004 | 0x00C | 0x018 | 0x030 | 0x050 |
            0x040 | 0x048 | 0x04C | 0x194 | 0x1D4 | 0x1E0 | 0x2804 | 0x3604 |
            0x140 | 0x14C | 0x150 |
            0x1404 | 0x1604 | 0x1804 | 0x1A04 | 0x1C04 | 0x1E04 => {}
            o if (0x1400..=0x3FFF).contains(&o) && (o.wrapping_sub(0x143C)) % 0x200 == 0 => {}
            _ => self.log_unhandled_write(offset, val),
        }

        // BSC I2C write 0x140: bits[18-26]=word_idx+0x100, bits[27-31]=cmd, bit 0=A0/A2 select.
        if offset == 0x140 {
            let param_5 = (val >> 18) & 0x1FF;
            let cmd_hi = (val >> 27) & 0x1F;
            if (cmd_hi & 1) != 0 && param_5 >= 0x100 {
                let word_idx = (param_5 - 0x100) as u16;
                self.bsc.pending_read_off = self.bsc.base_addr.wrapping_add(word_idx * 4);
                self.bsc.read_ready = true;
            }
            self.bsc.device = (val & 0x1) as u8;
        }
        if offset == 0x14C {
            self.bsc.base_addr = (val & 0xFFFF) as u16;
        }

        // Track I2C clock toggles on bit 0 of register 0x4C
        if offset == 0x04C {
            let old = self.sysreg_store[0x04C / 4];
            if (val & 1) != 0 && (old & 1) == 0 {
                self.i2c_clock_toggles += 1;
            }
        }
        // Reset I2C state when start condition is initiated (bit 15 set on 0x40)
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
            let cmd_bits = val & 0xF8000000;
            if cmd_bits != 0 {
                self.sysreg_pending_clear[idx] = cmd_bits;
            }
        }
    }

    #[inline]
    fn is_serdes(addr: u32) -> bool {
        addr >= SERDES_BASE && addr < SERDES_BASE + SERDES_SIZE
    }

    // ---------- byte ----------

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        if Self::is_uart(addr) {
            return match Self::uart_reg_offset(addr) {
                Some(off) => self.uart.read_byte(off),
                None => Ok(0), // unknown per-channel register
            };
        }
        if Self::is_pbc(addr) {
            let offset = addr - PBC_BASE;
            let word_offset = offset & !3;
            let byte_idx = offset & 3;
            let word = self.pbc.read_word(word_offset);
            return Ok((word >> (24 - byte_idx * 8)) as u8);
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            let word_offset = offset & !3;
            let byte_idx = offset & 3;
            let word = self.sysreg_read_word(word_offset);
            return Ok((word >> (24 - byte_idx * 8)) as u8);
        }
        if Self::is_serdes(addr) {
            if self.trace {
                eprintln!("[MMIO] read  byte  0x{:08X} → 0x01 (serdes)", addr);
            }
            return Ok(1);
        }
        if self.trace {
            eprintln!("[MMIO] read  byte  0x{:08X} → 0x00", addr);
        }
        Ok(0)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if Self::is_uart(addr) {
            return match Self::uart_reg_offset(addr) {
                Some(off) => self.uart.write_byte(off, val),
                None => Ok(()), // unknown per-channel register
            };
        }
        if Self::is_sysreg(addr) {
            // Byte write to sysreg: read-modify-write the containing word
            let offset = addr - SYSREG_BASE;
            let word_offset = offset & !3;
            let byte_idx = offset & 3;
            let idx = (word_offset / 4) as usize;
            if idx < self.sysreg_store.len() {
                let shift = 24 - byte_idx * 8;
                let mask = !(0xFFu32 << shift);
                self.sysreg_store[idx] = (self.sysreg_store[idx] & mask) | ((val as u32) << shift);
            }
            return Ok(());
        }
        if self.trace {
            eprintln!("[MMIO] write byte  0x{:08X} = 0x{:02X}", addr, val);
        }
        Ok(())
    }

    // ---------- halfword (big-endian) ----------

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        if Self::is_uart(addr) {
            let hi = match Self::uart_reg_offset(addr) {
                Some(off) => self.uart.read_byte(off)? as u16,
                None => 0u16,
            };
            let lo = match Self::uart_reg_offset(addr + 1) {
                Some(off) => self.uart.read_byte(off)? as u16,
                None => 0u16,
            };
            return Ok((hi << 8) | lo);
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            let word_offset = offset & !3;
            let half_idx = (offset >> 1) & 1;
            let word = self.sysreg_read_word(word_offset);
            return Ok((word >> (16 - half_idx * 16)) as u16);
        }
        if Self::is_serdes(addr) {
            if self.trace {
                eprintln!("[MMIO] read  half  0x{:08X} → 0x0001 (serdes)", addr);
            }
            return Ok(1);
        }
        if self.trace {
            eprintln!("[MMIO] read  half  0x{:08X} → 0x0000", addr);
        }
        Ok(0)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        if Self::is_uart(addr) {
            if let Some(off) = Self::uart_reg_offset(addr) {
                self.uart.write_byte(off, (val >> 8) as u8)?;
            }
            if let Some(off) = Self::uart_reg_offset(addr + 1) {
                self.uart.write_byte(off, val as u8)?;
            }
            return Ok(());
        }
        if Self::is_sysreg(addr) {
            // Halfword write to sysreg: read-modify-write the containing word
            let offset = addr - SYSREG_BASE;
            let word_offset = offset & !3;
            let half_idx = (offset >> 1) & 1;
            let idx = (word_offset / 4) as usize;
            if idx < self.sysreg_store.len() {
                let shift = 16 - half_idx * 16;
                let mask = !(0xFFFFu32 << shift);
                self.sysreg_store[idx] = (self.sysreg_store[idx] & mask) | ((val as u32) << shift);
            }
            return Ok(());
        }
        if self.trace {
            eprintln!("[MMIO] write half  0x{:08X} = 0x{:04X}", addr, val);
        }
        Ok(())
    }

    // ---------- word (big-endian) ----------

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        if Self::is_uart(addr) {
            return match Self::uart_reg_offset(addr) {
                Some(off) => self.uart.read_word(off),
                None => Ok(0),
            };
        }
        if Self::is_pbc(addr) {
            let offset = addr - PBC_BASE;
            let val = self.pbc.read_word(offset);
            if self.trace {
                eprintln!("[MMIO] read  word  0x{:08X} → 0x{:08X} (pbc+0x{:02X})", addr, val, offset);
            }
            return Ok(val);
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            let val = self.sysreg_read_word(offset);
            if self.trace {
                eprintln!("[MMIO] read  word  0x{:08X} → 0x{:08X} (sysreg+0x{:04X})", addr, val, offset);
            }
            return Ok(val);
        }
        if Self::is_serdes(addr) {
            if self.trace {
                eprintln!("[MMIO] read  word  0x{:08X} → 0x00000001 (serdes)", addr);
            }
            return Ok(1);
        }
        if self.trace {
            eprintln!("[MMIO] read  word  0x{:08X} → 0x00000000", addr);
        }
        Ok(0)
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if Self::is_uart(addr) {
            return match Self::uart_reg_offset(addr) {
                Some(off) => self.uart.write_word(off, val),
                None => Ok(()),
            };
        }
        if Self::is_pbc(addr) {
            let offset = addr - PBC_BASE;
            if self.trace {
                eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X} (pbc+0x{:02X})", addr, val, offset);
            }
            self.pbc.write_word(offset, val);
            return Ok(());
        }
        if Self::is_sysreg(addr) {
            let offset = addr - SYSREG_BASE;
            self.sysreg_write_word(offset, val);
            if self.trace {
                eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X} (sysreg+0x{:04X})", addr, val, offset);
            }
            return Ok(());
        }
        if self.trace {
            eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X}", addr, val);
        }
        Ok(())
    }
}
