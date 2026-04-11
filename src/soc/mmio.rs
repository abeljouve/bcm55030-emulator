use std::collections::{HashMap, HashSet};

use crate::cpu::exception::Exception;
use super::uart::SimpleUart;
use super::pbc::PeripheralBusController;
use super::sfp_eeprom;

/// BSC I2C (Broadcom Serial Controller) state for SFP EEPROM reads.
///
/// The BSC is exposed to the firmware via three MMIO registers at
/// sysreg offsets `0x140`, `0x14C`, `0x150` (base `0x010000F8 + 0x48/0x54/0x58`).
/// `dpoe_lane_read_bytes_from_table` @ the decompiler 0x20032e44 drives them via
/// three helpers:
///   - `dpoe_lane_cmd_config_and_wait` @ 0x20032c68 writes 0x140 and polls
///     bits [27-31] until they auto-clear.
///   - `dpoe_lane_cmd_write_and_wait` @ 0x20032d44 writes 0x150 and polls
///     bit 31 until it clears. Carries an encoded byte address in bits
///     [3-13] when used in "set address" mode (`param_2 == 2`).
///   - `dpoe_get_lane_state_field_from_table` @ 0x20032ccc calls
///     cmd_config with `param_4 = 1` and `param_5 = word_idx + 0x100`,
///     then reads the 32-bit result from 0x14C.
///
/// To emulate the controller faithfully enough for `access/read 4 <inst>
/// <addr> <len>`, we track the pending read word index, the base byte
/// offset set via cmd_write_and_wait, and the current device (A0h vs A2h).
#[derive(Default)]
pub struct BscI2cState {
    /// Base byte offset within the SFP EEPROM, as set by a
    /// `dpoe_lane_cmd_write_and_wait(..., param_2 = 2, param_4 = addr)` call
    /// (write to 0x150 with bit 1 set).
    pub base_addr: u16,
    /// Byte offset of the next word the firmware expects to read from 0x14C.
    /// Updated every time `dpoe_get_lane_state_field_from_table` writes 0x140
    /// with the `word_idx + 0x100` length field and the `param_4 = 1` cmd bit.
    pub pending_read_off: u16,
    /// True when a 0x14C read should return an SFP word.
    pub read_ready: bool,
    /// Selected EEPROM device: 0 = A0h (ID page), 1 = A2h (DDM page).
    /// Set from the "lanes" field (bits 16-17) of the 0x140 command word.
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

/// UART base address in the SoC MMIO space.
/// Hardware base pointer is 0x00FC0FE8; data register at +0x28, IER at +0x2C.
const UART_BASE: u32 = 0x00FC1010;
const UART_SIZE: u32 = 0x10; // +0x00 through +0x0F (data, IER, baud_lo, baud_hi)

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
        addr >= UART_BASE && addr < UART_BASE + UART_SIZE
    }

    #[inline]
    fn is_pbc(addr: u32) -> bool {
        addr >= PBC_BASE && addr < PBC_BASE + PBC_SIZE
    }

    #[inline]
    fn is_sysreg(addr: u32) -> bool {
        addr >= SYSREG_BASE && addr < SYSREG_BASE + SYSREG_SIZE
    }

    /// BCM55030 SoC register reads.
    ///
    /// Hardware-defined registers return fixed/computed values. All others use a
    /// read-write store with auto-clear for command bits (27-31), simulating
    /// instant hardware completion of write-triggered operations.
    ///
    /// The full register map was resolved from 85 hwregs base pointers via Ghidra.
    /// Major clusters:
    ///   0x000-0x1EF  EPON MAC core (CHIP_ID, timers, I2C, link lock, EPON sig)
    ///   0x1F0-0x23F  PBC (handled separately, checked before sysreg)
    ///   0x240-0xFFF  SerDes Speed/PHY, MACsec, MPCP LLID, misc
    ///   0xFF8-0x13FF EPON MAC extended (timing, grants, slots, counters, LLIDs)
    ///   0x13DC-0x15FF DMA/IRQ controller, channel drain, mailbox, counters
    ///   0x2100-0x27FF DMA status, Lane HW, Fatal Error, MDIO, SerDes MDIO
    ///   0x2B00-0x37FF MACsec Control, VLAN, Filter, Lane IRQ, Channel Config
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
            //
            // Command/config register 0x140: after a firmware write with
            // cmd bits at [27-31], the hardware clears those bits when the
            // operation completes. `sysreg_pending_clear` already handles
            // the generic 0xF8000000 mask, so the default store_read path
            // behaves correctly here.
            0x140 => self.store_read(offset),
            // Data register 0x14C: returns the 32-bit word at the byte
            // offset set up by the previous 0x140 + 0x150 command sequence.
            // The firmware reads this register once per
            // `dpoe_get_lane_state_field_from_table` call, which bumps the
            // word index itself — we only honour `read_ready` to avoid
            // leaking data when the firmware is polling.
            0x14C => {
                if self.bsc.read_ready {
                    let w = sfp_eeprom::read_word(self.bsc.device, self.bsc.pending_read_off);
                    self.bsc.read_ready = false;
                    w
                } else {
                    self.store_read(offset)
                }
            }
            // Write/trigger register 0x150: bit 31 is the "busy" flag.
            // Auto-clear it on read to simulate instant completion.
            0x150 => {
                let idx = 0x150 / 4;
                let val = self.sysreg_store[idx];
                self.sysreg_store[idx] = val & !0x80000000;
                val & !0x80000000
            }
            0x194 | 0x1D4 => {
                // SerDes lane link lock status registers.
                // Bits 1,3 = link lock indicators for sub-lanes.
                // Return "all locked" to prevent infinite polling loops.
                let base = self.sysreg_store[(offset / 4) as usize];
                base | 0x0A
            }
            0x1E0 => 0x45504F4E, // EPON signature ("EPON")

            // ── HW counter result registers (stats/fifo, stats/epon, …) ──
            //
            // `hw_chan_latch_and_read_hw_counter` @ the decompiler 0x2000de6c writes
            // a `(group, chan, field)` selector to base+0x8 then reads the
            // 32-bit counter result from base+0xC. Base is
            // `0x010015cc + group * 0x200`, so the result registers live at
            // 0x010015D8, 0x010017D8, 0x010019D8, 0x01001BD8, 0x01001DD8,
            // 0x01001FD8.
            //
            // Real HW returns 0 for every (chan, field) on a quiescent ONU
            // (no traffic, no FIFO fill, no error counts). Our default
            // `store_read` path leaks whatever the firmware (or init code)
            // last wrote to the same offset, which produces nonsense values
            // like 30 in `stats/fifo`. Returning 0 matches real HW.
            o if (o & 0x1FF) == 0x1D8 && (0x15D8..=0x1FD8).contains(&o) => 0,

            // ── DMA/IRQ cluster (0x13DC-0x15FF) ─────────────────────────
            // DMA Channel Queue Drain Register: base 0x143C, stride 0x200.
            // epon_rx_queue_wait_drain_done polls bit 8 (0x100) until set.
            o @ 0x1400..=0x3FFF if (o.wrapping_sub(0x143C)) % 0x200 == 0 => {
                self.store_read(offset) | 0x100 // bit 8 = drain complete
            }

            // ── LLID interrupt status registers (W1C semantics) ─────────
            // The polled SerDes block at +0x1404+N*0x200 contains per-LLID
            // interrupt status. Real HW returns 0 (`mem/rm 0x1001404..0x1001E04`)
            // because the events being tracked don't occur on a quiescent ONU.
            // Our store-and-return default would return whatever the firmware
            // wrote to these (e.g., 0x100), causing `epon_poll_hw_state_changes`
            // to detect false positives and trigger `system_shutdown_and_flush`.
            //
            // Phase 1 minimal fix: return 0 for the 6 specific polled addresses
            // (entries 9-14 of the descriptor table at .data 0x7ED90, reg
            // indices 0x501/0x581/0x601/0x681/0x701/0x781).
            //
            // To extend in Phase 2 if other LLID intr regs need the same.
            0x1404 | 0x1604 | 0x1804 | 0x1A04 | 0x1C04 | 0x1E04 => 0,

            // ── Fatal Error Status (0x2804) ──────────────────────────────
            // hw_check_fatal_error_status reads base 0x010027B8 + 0x4C = offset 0x2804.
            // Write side (error mask) shares the same address. Return 0 = no errors.
            0x2804 => 0,

            // ── MDIO Clause 22/45 Controller data register (0x0064) ─────
            //
            // `mdio_bus_read_reg` @ ram:20033420 writes a read command to
            // this same register then reads back (val & 0xFFFF) as the PHY
            // response. `mdio_bus_write_reg` @ ram:200332a8 writes the
            // write-data variant but does not check the response.
            //
            // Real BCM55030 has no PHYs wired to the clause 22/45 MDIO
            // bus when used as a standalone ONU (the Device module does
            // not expose an external MDIO bus). `mdio/read X Y` on live
            // hardware returns `ffff` for every (phy, reg) — the standard
            // "no PHY pulldown" value.
            //
            // Preserve the command bits (so the firmware's subsequent
            // polls of the same register read what it wrote for the
            // trigger/ack) and force bits [15:0] to 0xFFFF so a read
            // response returns the no-PHY pattern.
            0x0064 => {
                let val = self.store_read(offset);
                (val & 0xFFFF_0000) | 0x0000_FFFF
            }

            // ── SerDes Error Status (0x3604) ─────────────────────────────
            // serdes_check_error_status @ 0x20011940 reads (val & 0xFFFF0).
            // The firmware writes 0x000FFFF0 to clear errors then reads back.
            // On real HW these bits are W1C — reading after the clear returns 0.
            // Without this stub, our store returns the written 0xFFFF0, and
            // `epon_llid_mka_tick_all_channels` calls the heavy
            // `macsec_hw_session_init` every iteration (~370k insns/loop),
            // so the cli_poll loop only runs ~5 times/second and the second
            // command typed at the prompt gets dropped.
            0x3604 => 0,

            // ── Default: read-write store with auto-clear ────────────────
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

        // ── BSC I2C state machine writes ────────────────────────────────
        //
        // 0x140 command/config register encoding (from
        // `dpoe_lane_cmd_config_and_wait` @ the decompiler 0x20032c68):
        //   bits [0-6]:   param_2 & 0x7f
        //   bits [16-17]: (param_3 & 3) << 16  — EEPROM device select
        //   bits [18-26]: (param_5 & 0x1ff) << 18
        //   bits [27-31]: param_4 << 27  — command bits, auto-cleared
        //
        // `dpoe_get_lane_state_field_from_table` @ 0x20032ccc calls
        // cmd_config with `param_4 = 1` and `param_5 = word_idx + 0x100`,
        // so the trigger for "prepare read" is:
        //   (val >> 27) & 1 = 1 and ((val >> 18) & 0x1ff) >= 0x100
        // In that case the word index is `((val >> 18) & 0xff)` and we
        // schedule a read of 4 SFP bytes at `base_addr + word_idx*4`.
        if offset == 0x140 {
            let param_5 = (val >> 18) & 0x1FF;
            let cmd_hi = (val >> 27) & 0x1F;
            if (cmd_hi & 1) != 0 && param_5 >= 0x100 {
                let word_idx = (param_5 - 0x100) as u16;
                self.bsc.pending_read_off = self.bsc.base_addr.wrapping_add(word_idx * 4);
                self.bsc.read_ready = true;
            }
            // The CLI `inst` parameter (0 = A0h, 1 = A2h) is propagated as
            // bit 0 of the descriptor's `desc[2:4]` field, which lands in
            // bits[0-6] of the 0x140 write (`param_2 & 0x7f`). On this
            // hardware bit 0 toggles between 0x50 (A0h) and 0x51 (A2h).
            self.bsc.device = (val & 0x1) as u8;
        }
        // Writes to 0x14C set the base byte offset for subsequent reads.
        // `dpoe_set_lane_state_field_in_table` @ the decompiler 0x20032d08 issues
        // `st.di r4,[r1,0x54]` with `r4 = CLI addr`, then calls cmd_config.
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

        // ── LLID config registers (stats/epon) ───────────────────────────
        //
        // The LLID rx/tx config registers are at:
        //   0x010003BC + llid * 4   (1G mode, used by hw_pon_get_llid_tx_config_1g
        //                            @ the decompiler 0x2003e0c8) — wait, base is 0x0100043C
        //   0x01000D00 + llid * 4   (10G mode, used by hw_pon_get_llid_rx_config_10g
        //                            @ the decompiler 0x2003e870)
        //
        // The boot-time init writes 0x00017FFF (low 16 = 0x7FFF = 32767) to
        // the registers for LLID 0 and LLID 31 (the channel-zero and
        // channel-31 anchors); LLIDs 1-30 stay at 0. On real hardware those
        // two registers read back with bit 0 of the low 16 cleared (0x7FFE
        // = 32766), so the `mpcp_get_llid_rx_config_by_speed` getter prints
        // 32766 instead of 32767. The other 30 LLIDs are unaffected.
        //
        // Mask bit 0 of the low 16 on write so the stored value matches
        // what real HW returns to a subsequent read.
        let val = if matches!(offset, 0x043C | 0x04B8 | 0x0D00 | 0x0D7C) {
            val & !0x0001
        } else {
            val
        };

        let idx = (offset / 4) as usize;
        if idx < self.sysreg_store.len() {
            self.sysreg_store[idx] = val;
            // Mark bits 27-31 for auto-clear on next read.
            // Hardware command registers use these bits as write-1-to-trigger that
            // the hardware clears after processing. The firmware polls until cleared.
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
            return self.uart.read_byte(addr - UART_BASE);
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
            return self.uart.write_byte(addr - UART_BASE, val);
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
            let hi = self.uart.read_byte(addr - UART_BASE)? as u16;
            let lo = self.uart.read_byte(addr + 1 - UART_BASE)? as u16;
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
            self.uart.write_byte(addr - UART_BASE, (val >> 8) as u8)?;
            self.uart.write_byte(addr + 1 - UART_BASE, val as u8)?;
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
            return self.uart.read_word(addr - UART_BASE);
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
            return self.uart.write_word(addr - UART_BASE, val);
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
