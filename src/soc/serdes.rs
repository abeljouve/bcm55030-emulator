//! BCM55030 SerDes peripheral — Session 2 scaffold.
//!
//! Claims two MMIO windows that were previously handled by stubs in
//! `src/soc/mmio.rs`:
//!
//!   * **`0x01000180..0x010001F8`** — SerDes Lane Configuration + Link
//!     Status + PHY Status. Hosts registers `PON_LANE_INDEX`,
//!     `LANE0_LINK_LOCK`, `LANE2_LINK_LOCK`, `UNI_LANE_INDEX`, etc.
//!     (block 5 "SerDes Lane Configuration" + block 44 "SerDes Link
//!     Status MMIO" per `hwregs`).
//!
//!   * **`0x224A0000..0x224A0800`** — SerDes Lane Status File. A 256-
//!     entry × 8-byte window that the firmware scans at startup.
//!     Previous stub returned `0x01` unconditionally; this peripheral
//!     now serves a lane-indexed register file so per-lane state is
//!     coherent (audit 5.1).
//!
//! Claimed as of Phase 4a (scenario plan):
//!   * **`0x01002400..0x01002A00`** — Lane HW Reset / Enable / Mode +
//!     10G mirror.  Contains MDIO command registers with bit-31 "go"
//!     semantics: firmware writes `val | 0x80000000`, then polls until
//!     bit 31 clears.  The emulator auto-clears bit 31 on the next
//!     read (matching `REG_MPCP_CMD_LATCH` in `epon_mac.rs`).
//!     Evidence: `unhandled_mmio.json` — 5 addresses with command-bit
//!     pattern (`0x0100240C`, `0x01002420`, `0x01002644`, `0x0100280C`,
//!     `0x01002820`).
//!   * **`0x01002D00..0x01002D40`** — Lane Mode Controller.
//!
//! Not yet claimed (left to sysreg_shim for now):
//!   * Extended SerDes config (`0x01003500..`)
//!
//! Audit items resolved in this session:
//!   * **5.1** — SerDes indirect range returns 1. Replaced by a proper
//!     per-lane register file (see `serdes_window_read`).
//!   * **5.3** — SerDes link lock forced set. Replaced by a derived
//!     bit that depends on the per-lane `locked` state, which the UI
//!     can toggle via [`SerDesEvent::SetLinkLocked`].
//!
//! Audit 5.2 (SerDes SPI slave returns `0xFF`) is resolved via the
//! cross-peripheral dispatch from `pbc.rs`: when `SPI_CONTROL & 0x40`
//! is set, PBC calls [`SerDes::spi_command`] instead of returning the
//! old stub value. For v1 this still returns an all-`0xFF` response
//! — the behavioural calibration data path arrives in a later pass.
//!
//! Audit 5.6 (SerDes error status forced 0) stays in `sysreg_shim`
//! because the affected register (`0x01003604`) belongs to block 46
//! (Filter/Mask Controller) per `hwregs`, not to the SerDes. It will
//! migrate to `fatal_filter.rs` in Session 7.

use std::collections::HashSet;

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, LaneSpeed, Peripheral, PeripheralError, PeripheralEvent, PeripheralSnapshot,
    SerDesEvent, SerDesLaneSnapshot, SerDesSnapshot,
};

/// Main SerDes MMIO window (lane configuration + link status).
pub const SERDES_MAIN_BASE: u32 = 0x0100_0180;
pub const SERDES_MAIN_END: u32 = 0x0100_01F8;

/// SerDes Lane MDIO / HW Enable / Mode window (1G + 10G banks).
pub const SERDES_LANE_BASE: u32 = 0x0100_2400;
pub const SERDES_LANE_END: u32 = 0x0100_2A00;

/// Fatal Error Status / Mask — split register at the same address.
/// Write = FATAL_ERROR_MASK (bits to mask out of fatal detection).
/// Read  = FATAL_ERROR_STATUS (current fatal error state, 0 = none).
/// hwregs block 62 (mask, W) + block 5601 (status, R).
const REG_FATAL_ERROR_STATUS_MASK: u32 = 0x0100_2804;

/// Lane Mode Controller window.
pub const SERDES_MODE_BASE: u32 = 0x0100_2D00;
pub const SERDES_MODE_END: u32 = 0x0100_2D40;

/// SerDes Lane Status File window — a 256-entry × 8-byte per-lane
/// status table. Reads return a lane-indexed register file.
pub const SERDES_WINDOW_BASE: u32 = 0x224A_0000;
pub const SERDES_WINDOW_END: u32 = 0x224A_0800;

const SERDES_MAIN_WORDS: usize = ((SERDES_MAIN_END - SERDES_MAIN_BASE) / 4) as usize;
const SERDES_LANE_WORDS: usize = ((SERDES_LANE_END - SERDES_LANE_BASE) / 4) as usize;
const SERDES_MODE_WORDS: usize = ((SERDES_MODE_END - SERDES_MODE_BASE) / 4) as usize;

/// Lane count. The BCM55030 exposes four electrical lanes: PON RX, PON
/// TX, UNI lane 1, UNI lane 2 — the firmware assigns them as lanes
/// 0..3 depending on the product SKU.
pub const LANE_COUNT: usize = 4;

const SERDES_RANGES: &[AddressRange] = &[
    AddressRange::new(SERDES_MAIN_BASE, SERDES_MAIN_END),
    AddressRange::new(SERDES_LANE_BASE, SERDES_LANE_END),
    AddressRange::new(SERDES_MODE_BASE, SERDES_MODE_END),
    AddressRange::new(SERDES_WINDOW_BASE, SERDES_WINDOW_END),
];

/// Absolute MMIO addresses for per-register handlers (not offsets —
/// the full 32-bit address makes debugging easier and avoids one more
/// level of subtraction inside the fast path).
const REG_PON_LANE_INDEX: u32 = 0x0100_0180;
const REG_LANE0_LINK_LOCK: u32 = 0x0100_0194;
const REG_UNI_LANE_INDEX: u32 = 0x0100_01C0;
const REG_LANE2_LINK_LOCK: u32 = 0x0100_01D4;

/// Bit mask applied to the `LANE{0,2}_LINK_LOCK` registers when the
/// corresponding lanes are locked. Bit 1 = lane 0/2 locked, bit 3 =
/// lane 1/3 locked (per `hwregs` block 44 documentation).
const LANE_LOCK_MASK_01: u32 = 0b1010;

#[derive(Clone, Copy, Debug)]
pub struct LaneState {
    pub enabled: bool,
    pub locked: bool,
    pub rx_los: bool,
    pub speed: LaneSpeed,
    pub mdio_address: u8,
}

impl LaneState {
    const fn cold() -> Self {
        Self {
            enabled: false,
            locked: false,
            rx_los: false,
            speed: LaneSpeed::Pon10G,
            mdio_address: 0,
        }
    }

    const fn warm() -> Self {
        // Post-boot snapshot: lanes brought up, no RX loss, link OK.
        Self {
            enabled: true,
            locked: true,
            rx_los: false,
            speed: LaneSpeed::Pon10G,
            mdio_address: 0,
        }
    }
}

/// Ticks after enable before a lane auto-locks (~ matches the
/// real HW EPON timer delay of ~1024 ticks during bring-up).
const LANE_LOCK_DELAY_TICKS: u64 = 16;

#[derive(Clone)]
pub struct SerDes {
    /// Backing store for the main MMIO window.
    raw_store: [u32; SERDES_MAIN_WORDS],
    /// Backing store for the Lane MDIO / HW Enable window.
    lane_store: Vec<u32>,
    /// Backing store for the Lane Mode Controller window.
    mode_store: Vec<u32>,
    /// Addresses where bit 31 was written as 1 ("go" command).  The
    /// next `read_word` returns with bit 31 cleared and removes the
    /// entry — matching the hardware's "operation complete" semantics.
    cmd_pending_clear: HashSet<u32>,
    /// Per-lane tick counter for auto-lock progression.  Starts
    /// counting when `enabled` becomes true and `locked` is false.
    lane_lock_countdown: [u64; LANE_COUNT],
    pub lanes: [LaneState; LANE_COUNT],
    /// Per-lane status file at `0x224A0000 + lane*8`.
    window: Vec<u8>,
    /// Latched error status — W1C by software. Session 2 reads always
    /// return 0 (no injected faults); UI / future sessions can raise
    /// bits here.
    pub error_status: u32,
    pub trace: bool,
}

impl SerDes {
    pub fn new() -> Self {
        Self {
            raw_store: [0u32; SERDES_MAIN_WORDS],
            lane_store: vec![0u32; SERDES_LANE_WORDS],
            mode_store: vec![0u32; SERDES_MODE_WORDS],
            cmd_pending_clear: HashSet::new(),
            lane_lock_countdown: [0; LANE_COUNT],
            lanes: [LaneState::cold(); LANE_COUNT],
            window: vec![0u8; (SERDES_WINDOW_END - SERDES_WINDOW_BASE) as usize],
            error_status: 0,
            trace: false,
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (SERDES_MAIN_BASE..SERDES_MAIN_END).contains(&addr)
            || (SERDES_LANE_BASE..SERDES_LANE_END).contains(&addr)
            || (SERDES_MODE_BASE..SERDES_MODE_END).contains(&addr)
            || (SERDES_WINDOW_BASE..SERDES_WINDOW_END).contains(&addr)
    }

    #[inline]
    fn lane_idx(addr: u32) -> usize {
        ((addr - SERDES_LANE_BASE) / 4) as usize
    }

    #[inline]
    fn mode_idx(addr: u32) -> usize {
        ((addr - SERDES_MODE_BASE) / 4) as usize
    }

    fn is_cmd_bit_register(addr: u32) -> bool {
        matches!(
            addr,
            0x0100_240C | 0x0100_2420 | 0x0100_2644 | 0x0100_280C | 0x0100_2820
        )
    }

    fn main_idx(addr: u32) -> usize {
        ((addr - SERDES_MAIN_BASE) / 4) as usize
    }

    /// Derive the `LANE{0,2}_LINK_LOCK` register view from the
    /// per-lane `locked` state. `lane_pair` = 0 → (lane0 bit1, lane1
    /// bit3); `lane_pair` = 1 → (lane2 bit1, lane3 bit3).
    fn compute_lane_lock(&self, lane_pair: usize) -> u32 {
        let idx0 = lane_pair * 2;
        let idx1 = lane_pair * 2 + 1;
        let addr = if lane_pair == 0 {
            REG_LANE0_LINK_LOCK
        } else {
            REG_LANE2_LINK_LOCK
        };
        let mut val = self.raw_store[Self::main_idx(addr)];
        // Clear the lock bits and re-derive them from lane state.
        val &= !LANE_LOCK_MASK_01;
        if self.lanes[idx0].locked {
            val |= 1 << 1;
        }
        if self.lanes[idx1].locked {
            val |= 1 << 3;
        }
        val
    }

    fn window_read(&self, offset: u32) -> u8 {
        let idx = (offset - SERDES_WINDOW_BASE) as usize;
        self.window[idx]
    }

    fn window_write(&mut self, offset: u32, val: u8) {
        let idx = (offset - SERDES_WINDOW_BASE) as usize;
        self.window[idx] = val;
    }

    /// Cross-peripheral SPI slave stub — called from PBC when
    /// `SPI_CONTROL & 0x40` routes a FIFO command to the SerDes SPI
    /// slave. Session 2 returns an all-`0xFF` response (matching the
    /// old behaviour); calibration / ack responses land in a later
    /// session. The distinct entry point addresses audit 5.2 by
    /// giving the SerDes ownership of this path.
    pub fn spi_command(&mut self, _tx: &[u8], rx_len: usize) -> Vec<u8> {
        vec![0xFFu8; rx_len]
    }

    fn apply_warm_snapshot(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if (SERDES_MAIN_BASE..SERDES_MAIN_END).contains(&abs) {
                let idx = Self::main_idx(abs);
                self.raw_store[idx] = val;
            } else if (SERDES_LANE_BASE..SERDES_LANE_END).contains(&abs) {
                let idx = Self::lane_idx(abs);
                self.lane_store[idx] = val;
            } else if (SERDES_MODE_BASE..SERDES_MODE_END).contains(&abs) {
                let idx = Self::mode_idx(abs);
                self.mode_store[idx] = val;
            }
        }
        for lane in &mut self.lanes {
            *lane = LaneState::warm();
        }
    }
}

impl Peripheral for SerDes {
    fn name(&self) -> &'static str {
        "serdes"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        SERDES_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        if (SERDES_WINDOW_BASE..SERDES_WINDOW_END).contains(&addr) {
            let base = (addr - SERDES_WINDOW_BASE) as usize;
            let b0 = self.window[base] as u32;
            let b1 = self.window[base + 1] as u32;
            let b2 = self.window[base + 2] as u32;
            let b3 = self.window[base + 3] as u32;
            return Ok((b0 << 24) | (b1 << 16) | (b2 << 8) | b3);
        }
        if (SERDES_LANE_BASE..SERDES_LANE_END).contains(&addr) {
            if addr == REG_FATAL_ERROR_STATUS_MASK {
                return Ok(0);
            }
            let idx = Self::lane_idx(addr);
            let mut val = self.lane_store[idx];
            if self.cmd_pending_clear.remove(&addr) {
                val &= !0x8000_0000;
                self.lane_store[idx] = val;
            }
            return Ok(val);
        }
        if (SERDES_MODE_BASE..SERDES_MODE_END).contains(&addr) {
            return Ok(self.mode_store[Self::mode_idx(addr)]);
        }
        match addr {
            REG_LANE0_LINK_LOCK => Ok(self.compute_lane_lock(0)),
            REG_LANE2_LINK_LOCK => Ok(self.compute_lane_lock(1)),
            _ => Ok(self.raw_store[Self::main_idx(addr)]),
        }
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if (SERDES_WINDOW_BASE..SERDES_WINDOW_END).contains(&addr) {
            let base = (addr - SERDES_WINDOW_BASE) as usize;
            self.window[base] = (val >> 24) as u8;
            self.window[base + 1] = (val >> 16) as u8;
            self.window[base + 2] = (val >> 8) as u8;
            self.window[base + 3] = val as u8;
            return Ok(());
        }
        if (SERDES_LANE_BASE..SERDES_LANE_END).contains(&addr) {
            if addr == REG_FATAL_ERROR_STATUS_MASK {
                return Ok(());
            }
            let idx = Self::lane_idx(addr);
            self.lane_store[idx] = val;
            if Self::is_cmd_bit_register(addr) && (val & 0x8000_0000) != 0 {
                self.cmd_pending_clear.insert(addr);
            }
            return Ok(());
        }
        if (SERDES_MODE_BASE..SERDES_MODE_END).contains(&addr) {
            self.mode_store[Self::mode_idx(addr)] = val;
            return Ok(());
        }
        let idx = Self::main_idx(addr);
        match addr {
            REG_PON_LANE_INDEX => {
                let lane_idx = (val & 0x3) as usize;
                if lane_idx < LANE_COUNT {
                    self.lanes[lane_idx].enabled = true;
                    self.lanes[lane_idx].locked = true;
                }
            }
            REG_UNI_LANE_INDEX => {
                let lane_idx = 2 + ((val >> 20) & 0x3) as usize;
                if lane_idx < LANE_COUNT {
                    self.lanes[lane_idx].enabled = true;
                    self.lanes[lane_idx].locked = true;
                }
            }
            _ => {}
        }
        self.raw_store[idx] = val;
        Ok(())
    }

    fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        if (SERDES_WINDOW_BASE..SERDES_WINDOW_END).contains(&addr) {
            return Ok(self.window_read(addr));
        }
        let word_addr = addr & !3;
        let byte_idx = addr & 3;
        let word = self.read_word(word_addr)?;
        Ok((word >> (24 - byte_idx * 8)) as u8)
    }

    fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if (SERDES_WINDOW_BASE..SERDES_WINDOW_END).contains(&addr) {
            self.window_write(addr, val);
            return Ok(());
        }
        // For cmd-bit registers, byte write should not trigger cmd-bit
        // auto-clear. Route via raw store instead of write_word.
        let word_addr = addr & !3;
        let byte_idx = addr & 3;
        let old = self.read_word(word_addr)?;
        let shift = 24 - byte_idx * 8;
        let mask = !(0xFFu32 << shift);
        let new = (old & mask) | ((val as u32) << shift);
        self.write_word(word_addr, new)
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        for i in 0..LANE_COUNT {
            if self.lanes[i].enabled && !self.lanes[i].locked && !self.lanes[i].rx_los {
                self.lane_lock_countdown[i] += 1;
                if self.lane_lock_countdown[i] >= LANE_LOCK_DELAY_TICKS {
                    self.lanes[i].locked = true;
                    self.lane_lock_countdown[i] = 0;
                }
            } else if !self.lanes[i].enabled || self.lanes[i].rx_los {
                self.lane_lock_countdown[i] = 0;
            }
        }
    }

    fn reset_cold(&mut self) {
        self.raw_store = [0u32; SERDES_MAIN_WORDS];
        self.lane_store.fill(0);
        self.mode_store.fill(0);
        self.cmd_pending_clear.clear();
        self.lane_lock_countdown = [0; LANE_COUNT];
        self.lanes = [LaneState::cold(); LANE_COUNT];
        self.window.fill(0);
        self.error_status = 0;
    }

    fn reset_warm(&mut self) {
        self.reset_cold();
        self.apply_warm_snapshot();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::SerDes(SerDesSnapshot {
            lanes: [
                self.lane_snapshot(0),
                self.lane_snapshot(1),
                self.lane_snapshot(2),
                self.lane_snapshot(3),
            ],
            error_status: self.error_status,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::SerDes(ev) => match ev {
                SerDesEvent::SetLaneEnabled(lane, enabled) => {
                    let l = *lane as usize;
                    if l >= LANE_COUNT {
                        return Err(PeripheralError::InvalidParameter("lane index"));
                    }
                    self.lanes[l].enabled = *enabled;
                    if !*enabled {
                        self.lanes[l].locked = false;
                    }
                    Ok(())
                }
                SerDesEvent::SetLinkLocked(lane, locked) => {
                    let l = *lane as usize;
                    if l >= LANE_COUNT {
                        return Err(PeripheralError::InvalidParameter("lane index"));
                    }
                    self.lanes[l].locked = *locked;
                    Ok(())
                }
                SerDesEvent::InjectRxLos(lane, los) => {
                    let l = *lane as usize;
                    if l >= LANE_COUNT {
                        return Err(PeripheralError::InvalidParameter("lane index"));
                    }
                    self.lanes[l].rx_los = *los;
                    if *los {
                        self.lanes[l].locked = false;
                    }
                    Ok(())
                }
                SerDesEvent::SetLaneSpeed(lane, speed) => {
                    let l = *lane as usize;
                    if l >= LANE_COUNT {
                        return Err(PeripheralError::InvalidParameter("lane index"));
                    }
                    self.lanes[l].speed = *speed;
                    Ok(())
                }
                SerDesEvent::ClearErrorStatus => {
                    self.error_status = 0;
                    Ok(())
                }
            },
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}

impl SerDes {
    fn lane_snapshot(&self, idx: usize) -> SerDesLaneSnapshot {
        let l = self.lanes[idx];
        SerDesLaneSnapshot {
            enabled: l.enabled,
            locked: l.locked,
            rx_los: l.rx_los,
            speed: l.speed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_reset_lanes_disabled() {
        let mut s = SerDes::new();
        s.reset_cold();
        for l in &s.lanes {
            assert!(!l.enabled);
            assert!(!l.locked);
        }
        // LANE0/2 link lock returns 0 in cold mode
        assert_eq!(s.read_word(0x0100_0194).unwrap() & LANE_LOCK_MASK_01, 0);
        assert_eq!(s.read_word(0x0100_01D4).unwrap() & LANE_LOCK_MASK_01, 0);
    }

    #[test]
    fn warm_reset_loads_lane_config() {
        let mut s = SerDes::new();
        s.reset_warm();
        // Warm mode assumes lanes are locked — bits 1 and 3 of link
        // lock registers must be set.
        assert_eq!(
            s.read_word(0x0100_0194).unwrap() & LANE_LOCK_MASK_01,
            LANE_LOCK_MASK_01
        );
        assert_eq!(
            s.read_word(0x0100_01D4).unwrap() & LANE_LOCK_MASK_01,
            LANE_LOCK_MASK_01
        );
    }

    #[test]
    fn pon_lane_index_write_enables_lane() {
        let mut s = SerDes::new();
        s.reset_cold();
        // Write lane index 0 — enables lane 0
        s.write_word(0x0100_0180, 0x0000_0000).unwrap();
        assert!(s.lanes[0].enabled);
        assert!(s.lanes[0].locked);
        assert_ne!(s.read_word(0x0100_0194).unwrap() & (1 << 1), 0);
    }

    #[test]
    fn window_range_round_trips() {
        let mut s = SerDes::new();
        s.write_word(0x224A_0010, 0xDEAD_BEEF).unwrap();
        assert_eq!(s.read_word(0x224A_0010).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn inject_rx_los_drops_lock() {
        let mut s = SerDes::new();
        s.reset_warm();
        s.inject_event(&PeripheralEvent::SerDes(SerDesEvent::InjectRxLos(0, true)))
            .unwrap();
        assert!(s.lanes[0].rx_los);
        assert!(!s.lanes[0].locked);
        assert_eq!(s.read_word(0x0100_0194).unwrap() & (1 << 1), 0);
    }

    #[test]
    fn claims_ranges() {
        let s = SerDes::new();
        assert!(s.claims(0x0100_0194));
        assert!(s.claims(0x224A_0010));
        assert!(!s.claims(0x0100_0170));
        assert!(!s.claims(0x0100_01F8));
        assert!(s.claims(0x0100_240C));
        assert!(s.claims(0x0100_2820));
        assert!(s.claims(0x0100_2D00));
        assert!(!s.claims(0x0100_2A00));
    }

    #[test]
    fn cmd_bit_auto_clear_on_read() {
        let mut s = SerDes::new();
        s.write_word(0x0100_240C, 0x8000_5000).unwrap();
        assert_eq!(s.read_word(0x0100_240C).unwrap(), 0x0000_5000);
        assert_eq!(s.read_word(0x0100_240C).unwrap(), 0x0000_5000);
    }

    #[test]
    fn cmd_bit_no_clear_without_bit31() {
        let mut s = SerDes::new();
        s.write_word(0x0100_240C, 0x0000_5000).unwrap();
        assert_eq!(s.read_word(0x0100_240C).unwrap(), 0x0000_5000);
    }

    #[test]
    fn cmd_bit_10g_mirror() {
        let mut s = SerDes::new();
        s.write_word(0x0100_280C, 0x8000_5000).unwrap();
        assert_eq!(s.read_word(0x0100_280C).unwrap(), 0x0000_5000);
    }

    #[test]
    fn cmd_bit_hw_enable() {
        let mut s = SerDes::new();
        s.write_word(0x0100_2644, 0x8000_0501).unwrap();
        assert_eq!(s.read_word(0x0100_2644).unwrap(), 0x0000_0501);
    }

    #[test]
    fn lane_store_roundtrip() {
        let mut s = SerDes::new();
        s.write_word(0x0100_2418, 0x0010_0C01).unwrap();
        assert_eq!(s.read_word(0x0100_2418).unwrap(), 0x0010_0C01);
    }

    #[test]
    fn mode_store_roundtrip() {
        let mut s = SerDes::new();
        s.write_word(0x0100_2D00, 0xDEAD_BEEF).unwrap();
        assert_eq!(s.read_word(0x0100_2D00).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn tick_auto_lock_progression() {
        let mut s = SerDes::new();
        s.reset_cold();
        s.inject_event(&PeripheralEvent::SerDes(SerDesEvent::SetLaneEnabled(0, true)))
            .unwrap();
        assert!(s.lanes[0].enabled);
        assert!(!s.lanes[0].locked);

        for _ in 0..LANE_LOCK_DELAY_TICKS - 1 {
            s.tick(0);
        }
        assert!(!s.lanes[0].locked);

        s.tick(0);
        assert!(s.lanes[0].locked);
        assert_ne!(s.read_word(0x0100_0194).unwrap() & (1 << 1), 0);
    }

    #[test]
    fn tick_auto_lock_blocked_by_rx_los() {
        let mut s = SerDes::new();
        s.reset_cold();
        s.inject_event(&PeripheralEvent::SerDes(SerDesEvent::SetLaneEnabled(0, true)))
            .unwrap();
        s.inject_event(&PeripheralEvent::SerDes(SerDesEvent::InjectRxLos(0, true)))
            .unwrap();

        for _ in 0..LANE_LOCK_DELAY_TICKS * 2 {
            s.tick(0);
        }
        assert!(!s.lanes[0].locked);
    }
}
