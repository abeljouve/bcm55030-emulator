//! BCM55030 SerDes peripheral.
//!
//! Claims the SerDes MMIO windows:
//!
//!   * **`0x01000174..0x010001F8`** — SerDes Lane Configuration + Link
//!     Status + PHY Status. One register file with the two directions
//!     a fixed `0x40` apart; hosts `PON_LANE_INDEX`, the lane-present
//!     masks, the link-arm registers, `UNI_LANE_INDEX`, etc. (block 5
//!     "SerDes Lane Configuration" + block 44 "SerDes Link Status
//!     MMIO" per `hwregs`).
//!
//!   * **`0x224A0000..0x224A0800`** — SerDes Lane Status File. A 256-
//!     entry × 8-byte window that the firmware scans at startup, served
//!     as a lane-indexed register file so per-lane state is coherent.
//!
//!   * **`0x01002400..0x01002A00`** — Lane HW Reset / Enable / Mode +
//!     10G mirror. Contains indirect command registers with bit-31
//!     "go" semantics: firmware writes `val | 0x80000000`, then polls
//!     until bit 31 clears. The emulator auto-clears bit 31 on the
//!     next read. Three addresses carry that handshake: `0x0100240C`,
//!     `0x01002644`, `0x0100280C`. `+0x20` in each bank does **not**
//!     — see `is_cmd_bit_register`.
//!
//!   * **`0x01002D00..0x01002D40`** — Lane Mode Controller.
//!
//! SerDes link lock is a derived bit that depends on the per-lane
//! `locked` state, which the UI can toggle via
//! [`SerDesEvent::SetLinkLocked`].
//!
//! The SerDes SPI slave is reached via cross-peripheral dispatch from
//! `pbc.rs`: when `SPI_CONTROL & 0x40` is set, PBC calls
//! [`SerDes::spi_command`]. This still returns an all-`0xFF` response —
//! the behavioural calibration data path is not modelled yet.
//!
//! SerDes error status (`0x01003604`) is not claimed here: it belongs
//! to block 46 (Filter/Mask Controller) per `hwregs`, not to the
//! SerDes.

use std::collections::HashSet;

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, LaneSpeed, Peripheral, PeripheralError, PeripheralEvent, PeripheralSnapshot,
    SerDesEvent, SerDesLaneSnapshot, SerDesSnapshot,
};

/// Main SerDes MMIO window (lane configuration + link status).
///
/// The base is the start of the lane configuration file, not of the
/// first register with a handler: software addresses this block as one
/// file with the transmit direction at a fixed `+0x40` from the receive
/// one (index `+0x0C`/`+0x4C`, link arm `+0x24`/`+0x64`, config word
/// `+0x38`/`+0x78`). Starting the window three words later would leave
/// the head of that file with a different servant.
pub const SERDES_MAIN_BASE: u32 = 0x0100_0174;
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
const REG_LANE01_PRESENT_MASK: u32 = 0x0100_0194;
const REG_UNI_LANE_INDEX: u32 = 0x0100_01C0;
const REG_LANE23_PRESENT_MASK: u32 = 0x0100_01D4;

/// Link-arm registers, one per direction (`+0x24` / `+0x64` from the
/// lane configuration base). Software arms `(1 << 24) << lane` and
/// reads the same word back to learn whether the lane came up.
///
/// -- OBSERVED on silicon: the armed bit **never** holds, on either
/// register, including on a lane that is registered and transmitting.
/// The read-back is always zero for these four bits. A model that let
/// them round-trip would report a link the hardware never reported.
const REG_LANE01_LINK_ARM: u32 = 0x0100_0198;
const REG_LANE23_LINK_ARM: u32 = 0x0100_01D8;

/// The four lane bits inside a link-arm register.
const LANE_ARM_MASK: u32 = 0x0F00_0000;

/// Bit mask applied to the lane-present registers when the
/// corresponding lanes are locked. Bit 1 = lane 0/2, bit 3 = lane 1/3.
///
/// -- INFERRED, and contradicted by hardware: those registers read a
/// constant `0xF` and `0x3` there, and the second one does not move
/// when the far end stops transmitting — so they carry which lanes
/// exist, not whether they are up. Deriving the bits from lane state
/// is kept for now because nothing measures the difference; the name
/// no longer claims it is a lock.
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
    /// Latched error status — W1C by software. Reads return 0 with no
    /// injected faults; the UI can raise bits here.
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

    /// Registers whose bit 31 is a "go/busy" handshake: firmware writes
    /// it set, then spins on the same address until it reads back clear.
    ///
    /// OBSERVED: `+0x20` in each bank is **not** one of them. It holds the
    /// VLAN EtherType, which is written as `0x8100_XXXX` — bit 31 belongs
    /// to the 0x8100 tag protocol identifier, not to a command. Clearing
    /// it made the firmware read back `0x0100_XXXX`, and the read-modify-
    /// write that followed persisted the truncated value.
    fn is_cmd_bit_register(addr: u32) -> bool {
        matches!(addr, 0x0100_240C | 0x0100_2644 | 0x0100_280C)
    }

    /// Return a descriptive per-lane MDIO tag for MMIO history entries.
    /// Per-lane MDIO controller base = 0x010023D8 + lane*0x400.
    /// CMD at base+0x34, DATA3..DATA0 at base+0x38..+0x44.
    pub fn mdio_peripheral_tag(addr: u32) -> &'static str {
        if !(SERDES_LANE_BASE..SERDES_LANE_END).contains(&addr) {
            return "serdes";
        }
        let off = addr - SERDES_LANE_BASE;
        // 1G bank: lanes 0-1 at offset 0x00 + lane*0x400
        // 10G bank: lanes 0-1 at offset 0x400 + lane*0x400
        // Per-lane MDIO registers at lane_base+0x0C..+0x1C
        // Lane 0 1G: CMD=0x240C DATA3=0x2410 DATA2=0x2414 DATA1=0x2418 DATA0=0x241C
        // Lane 0 1G HW_EN: 0x2420
        // Lane 1 1G: CMD=0x2644 (stride 0x238 in the 1G bank)
        // Lane 0 10G: CMD=0x280C DATA3=0x2810..
        // Lane 0 10G HW_EN: 0x2820
        match addr {
            // 1G lane 0 MDIO
            0x0100_240C => "serdes_l0_1g_mdio_cmd",
            0x0100_2410 => "serdes_l0_1g_mdio_data3",
            0x0100_2414 => "serdes_l0_1g_mdio_data2",
            0x0100_2418 => "serdes_l0_1g_mdio_data1",
            0x0100_241C => "serdes_l0_1g_mdio_data0",
            0x0100_2420 => "serdes_l0_1g_vlan_ethertype",
            // 1G lane 1 MDIO (stride 0x238 from lane 0)
            0x0100_2644 => "serdes_l1_1g_mdio_cmd",
            0x0100_2648 => "serdes_l1_1g_mdio_data3",
            0x0100_264C => "serdes_l1_1g_mdio_data2",
            0x0100_2650 => "serdes_l1_1g_mdio_data1",
            0x0100_2654 => "serdes_l1_1g_mdio_data0",
            0x0100_2658 => "serdes_l1_1g_hw_enable",
            // 10G lane 0 MDIO
            0x0100_280C => "serdes_l0_10g_mdio_cmd",
            0x0100_2810 => "serdes_l0_10g_mdio_data3",
            0x0100_2814 => "serdes_l0_10g_mdio_data2",
            0x0100_2818 => "serdes_l0_10g_mdio_data1",
            0x0100_281C => "serdes_l0_10g_mdio_data0",
            0x0100_2820 => "serdes_l0_10g_vlan_ethertype",
            // 10G lane 1 MDIO
            0x0100_2A44 => "serdes_l1_10g_mdio_cmd",
            0x0100_2A48 => "serdes_l1_10g_mdio_data3",
            0x0100_2A4C => "serdes_l1_10g_mdio_data2",
            0x0100_2A50 => "serdes_l1_10g_mdio_data1",
            0x0100_2A54 => "serdes_l1_10g_mdio_data0",
            _ => {
                // Generic lane-region fallback with 0x400 stride
                let lane = off / 0x400;
                match lane {
                    0 => "serdes_lane0",
                    1 => "serdes_lane1",
                    2 => "serdes_lane2",
                    3 => "serdes_lane3",
                    _ => "serdes_lane",
                }
            }
        }
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
            REG_LANE01_PRESENT_MASK
        } else {
            REG_LANE23_PRESENT_MASK
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
    /// slave. Returns an all-`0xFF` response; calibration / ack
    /// responses are not modelled yet.
    pub fn spi_command(&mut self, _tx: &[u8], rx_len: usize) -> Vec<u8> {
        vec![0xFFu8; rx_len]
    }

    fn apply_silicon_power_on(&mut self) {
        // Silicon power-on values land in the backing stores; lanes
        // remain disabled/unlocked because silicon shows
        // LANE_BUS_EN=0 and LINK_LOCK_01/23=0 at power-on (verified via
        // hardware probing). Firmware programs lane enables itself
        // during init.
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
            REG_LANE01_PRESENT_MASK => Ok(self.compute_lane_lock(0)),
            REG_LANE23_PRESENT_MASK => Ok(self.compute_lane_lock(1)),
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
            // The arm bits do not stick: keep the rest of the word, drop
            // the four lane bits so a read-back never reports a link.
            REG_LANE01_LINK_ARM | REG_LANE23_LINK_ARM => {
                self.raw_store[idx] = val & !LANE_ARM_MASK;
                return Ok(());
            }
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
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        // Silicon power-on snapshot already covers warm reset state.
        self.reset_cold();
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
    fn warm_reset_silicon_power_on_lanes_unlocked() {
        let mut s = SerDes::new();
        s.reset_warm();
        // Silicon power-on shows LANE_BUS_EN=0 and LINK_LOCK_01/23=0
        // — lanes are unlocked at power-on. Firmware brings them up
        // itself.
        for l in &s.lanes {
            assert!(!l.enabled);
            assert!(!l.locked);
        }
        assert_eq!(s.read_word(0x0100_0194).unwrap() & LANE_LOCK_MASK_01, 0);
        assert_eq!(s.read_word(0x0100_01D4).unwrap() & LANE_LOCK_MASK_01, 0);
        // LANE01_RESET / LANE_BUS_EN must read zero — bit 15 of
        // PON_MODE freezes silicon when set, so any non-zero seed
        // here is a regression of this bug.
        assert_eq!(s.read_word(0x0100_0180).unwrap(), 0);
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
        // Silicon power-on shows lanes unlocked. Bring lane 0 up
        // first (as firmware would), then inject RX LOS.
        s.inject_event(&PeripheralEvent::SerDes(SerDesEvent::SetLaneEnabled(0, true)))
            .unwrap();
        s.inject_event(&PeripheralEvent::SerDes(SerDesEvent::SetLinkLocked(0, true)))
            .unwrap();
        assert!(s.lanes[0].locked);
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

    /// Arming a lane must not manufacture a link the hardware never
    /// reports — and must not swallow the rest of the word either, which
    /// is the negative control that tells the two apart.
    #[test]
    fn link_arm_bits_never_latch_but_the_word_round_trips() {
        for addr in [REG_LANE01_LINK_ARM, REG_LANE23_LINK_ARM] {
            let mut s = SerDes::new();
            s.reset_cold();
            for lane in 0..LANE_COUNT {
                s.write_word(addr, (1 << 24) << lane).unwrap();
                assert_eq!(s.read_word(addr).unwrap() & LANE_ARM_MASK, 0);
            }
            s.write_word(addr, 0x0F00_1234).unwrap();
            assert_eq!(s.read_word(addr).unwrap(), 0x0000_1234);
        }
    }

    #[test]
    fn main_window_starts_at_the_lane_config_base() {
        let s = SerDes::new();
        assert!(s.claims(0x0100_0174));
        assert!(!s.claims(0x0100_0170));
    }

    /// The VLAN EtherType register keeps bit 31: the value written is a
    /// tag protocol identifier (`0x8100`), not a command. A firmware that
    /// reads it back and rewrites the result must see what it wrote.
    #[test]
    fn vlan_ethertype_keeps_bit31_across_read_modify_write() {
        for addr in [0x0100_2420u32, 0x0100_2820] {
            let mut s = SerDes::new();
            s.write_word(addr, 0x8100_88A8).unwrap();
            // First read is the one the firmware branches on...
            assert_eq!(s.read_word(addr).unwrap(), 0x8100_88A8);
            // ...and the read-modify-write that follows must not truncate.
            let v = s.read_word(addr).unwrap();
            s.write_word(addr, v).unwrap();
            assert_eq!(s.read_word(addr).unwrap(), 0x8100_88A8);
        }
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
