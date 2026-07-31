//! BCM55030 EPON MAC peripheral.
//!
//! Hosts the core EPON MAC register surface. The peripheral owns a
//! **sparse** claim set rather than one contiguous MMIO range, because
//! the BCM55030 interleaves EPON MAC registers with unrelated
//! subsystems (I2C/eFuse control at `0x040/0x048/0x04C`, the
//! free-running counter at `0x050`, BSC at `0x140..0x158`, SerDes at
//! `0x180..0x1F8`, PBC at `0x1F0..0x240`) inside the same
//! `0x01000000..0x01002000` window. A contiguous range would steal
//! those offsets.
//!
//! The peripheral claims, by predicate:
//!
//!   * Sparse core registers — CHIP_ID, CHIP_REV, LLID masks, active
//!     bitmap, grant masks, IRQ mask, EPON status, active flags, plus
//!     the `0x0064` special half-`FFFF` read arm.
//!   * `0x01000400..0x01000E80` — LLID grant / enable tables, per-LLID
//!     config (anchors 0x043C, 0x04B8, 0x0D00, 0x0D7C clear bit 0 on
//!     write).
//!   * `0x01001400..0x01002000` — LLID IRQ status + counter stats +
//!     queue drain. Six stride-`0x200` blocks, one per LLID slot:
//!       - `0x1X04` IRQ status — always 0, W1C.
//!       - `0x1XD8` counter result slot — always 0.
//!       - `0x1X3C` queue drain — bit 8 permanently set with per-offset
//!         auto-clear.
//!
//! Behaviour notes:
//!
//!   * LLID IRQ status is forced to 0, backed by per-LLID IRQ state.
//!   * DMA queue drain bit 8 is driven by real per-LLID state with a
//!     targeted auto-clear.
//!   * LLID 0/31 anchor registers clear bit 0 on write.
//!
//! Not owned here:
//!
//!   * `0x01000080..0x010000BC` — queue priority / DPoE flavour
//!     registers (generic backing store elsewhere).
//!   * `0x01002000..0x01002800` — MACsec / DMA.
//!   * `0x01002804` — fatal error aggregator; lives in `macsec.rs`,
//!     inside the MACsec 10G SA programming bank.
//!   * `0x01002820..0x01003600` — fatal + filter tail.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, EponEvent, EponMacSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

pub const CHIP_ID_VALUE: u32 = 0x47010203;
pub const CHIP_REV_VALUE: u32 = 0xB2110816;
/// Silicon power-on default for LLID_CAPTURE_MASK / PON_MODE — zero,
/// observed on hardware. Bit 15 gates into an unclocked PCS domain, so
/// a non-zero reset value would freeze the bus on real silicon.
pub const LLID_CAPTURE_MASK_RESET: u32 = 0x0000_0000;
/// Silicon power-on default for LLID_ACTIVE_BITMAP — zero, observed on
/// hardware. Firmware programs the active bitmap during init.
pub const LLID_ACTIVE_RESET: u32 = 0x0000_0000;
pub const RX_GRANT_MASK_RESET: u32 = 0x0000_FFFF;
/// Silicon power-on default for LLID_COUNTER_MASK at +0x024.
pub const LLID_COUNTER_MASK_RESET: u32 = 0x0007_7FF7;
/// Silicon power-on default for MPCP_CMD_LATCH / IND_CMD at +0x160.
pub const MPCP_CMD_LATCH_RESET: u32 = 0x0001_5000;
pub const EPON_SIG_VALUE: u32 = 0x4550_4F4E; // "EPON"

/// Number of active LLID slots the firmware scans in the stride-0x200
/// counter / IRQ / drain blocks. The HW exposes up to 16 slots but
/// the firmware only enables six on the BCM55030 reference design.
pub const LLID_SLOT_COUNT: usize = 6;

const EPON_LLID_BASE: u32 = 0x0100_1400;
const EPON_LLID_STRIDE: u32 = 0x0000_0200;
const EPON_LLID_TOP: u32 = EPON_LLID_BASE + (LLID_SLOT_COUNT as u32) * EPON_LLID_STRIDE; // 0x0100_2000

/// Report frame template: length in the high half, a timestamp delta in
/// the low half. The firmware writes both from the registration exchange.
const REG_REPORT_FRAME_PARAMS: u32 = 0x0100_04D8;
const REG_REPORT_FRAME_LEN: u32 = 0x0100_052C;

const EPON_TABLE_BASE: u32 = 0x0100_0400;
const EPON_TABLE_END: u32 = 0x0100_0E80;

const REG_CHIP_ID: u32 = 0x0100_0000;
const REG_CHIP_REV: u32 = 0x0100_0004;
const REG_LLID_CAPTURE_MASK: u32 = 0x0100_000C;
const REG_LLID_ACTIVE_BITMAP: u32 = 0x0100_0018;
const REG_LLID_MASK_CONTROL: u32 = 0x0100_0020;
const REG_LLID_COUNTER_MASK: u32 = 0x0100_0024;
const REG_TX_GRANT_MASK: u32 = 0x0100_0028;
const REG_RX_GRANT_MASK: u32 = 0x0100_0030;
const REG_IRQ_MASK: u32 = 0x0100_0034;
const REG_EPON_STATUS: u32 = 0x0100_0044;
const REG_ACTIVE_FLAGS: u32 = 0x0100_0054;
const REG_MDIO_COMMAND: u32 = 0x0100_0060;
/// 1G PHY link status. Bit 1 = link change (W1C).
const REG_1G_LINK_STATUS: u32 = 0x0100_0410;
const REG_HW_STATE_STATUS: u32 = 0x0100_0E04;
const REG_SPECIAL_0064: u32 = 0x0100_0064;
/// MPCP-adjacent command latch. The firmware writes a value with bits
/// `[31:27]` set (command opcode) and polls the register for those bits
/// to clear. The access pattern matches MPCP LLID control (block 22)
/// but the offset is not documented in `hwregs`.
// MPCP CMD LATCH moved to LaneBus mpcp_bus (lane 8) in bank.rs.
/// EPON discovery status. Bit 2 = OLT discovery detected, bit 1 =
/// discovery change. W1C for bits[2:1]. `mpcp_epon_get_status` returns
/// true when `(val & 6) == 4` (bit 2 set, bit 1 clear).
const REG_DISCOVERY_STATUS: u32 = 0x0100_1040;

/// LLID 0/31 anchor registers — the HW clears bit 0 on every write.
/// The firmware programs the rest of the bitfields; bit 0 is the
/// "write to latch" trigger that should never read back as 1.
const LLID_ANCHOR_ADDRS: [u32; 4] = [
    0x0100_043C,
    0x0100_04B8,
    0x0100_0D00,
    0x0100_0D7C,
];

/// Address carveouts inside the table / LLID windows that belong to
/// **other** peripherals. `claims()` excludes them so routing still
/// reaches SerDes / PBC / BSC first.
///
/// The table window `0x0400..0x0E80` is EPON-only on real HW — no
/// peripherals live inside it. The LLID window `0x1400..0x2000` is
/// likewise EPON-only. This stays here as documentation, not code.
///
/// The sparse core claim set is narrow by construction, so nothing
/// needs excluding there either.

#[derive(Clone)]
pub struct EponMac {
    /// Backing store for every EPON-owned word. Indexed by
    /// `store_index(addr)` — sparse registers get a dedicated slot,
    /// the two windowed ranges get contiguous slices.
    table_store: Vec<u32>,
    llid_store: Vec<u32>,

    /// Per-LLID IRQ pending bit — raised via `InjectLlidInterrupt`.
    /// The firmware's polling loops W1C these registers, so reads
    /// return the latched value and writes clear matching bits.
    llid_irq_pending: [u32; LLID_SLOT_COUNT],

    /// Per-LLID queue drain counter. Bit 8 of each drain register
    /// reads back as 1 whenever the slot has entries to drain; the
    /// firmware clears it by writing 1 to bit 8. We always report
    /// "has entries" to keep the polling loop progressing — the
    /// auto-clear resets it the instant the firmware polls.
    llid_drain_flag: [bool; LLID_SLOT_COUNT],

    /// Frames that have arrived in each queue since reset, indexed by
    /// `(channel, queue)`. The datapath reports arrivals here; nothing
    /// in this peripheral invents them.
    queue_arrivals: std::collections::HashMap<(usize, u32), u64>,
    /// Last command written to each channel's counter-latch port, with
    /// the busy bit already cleared.
    latch_cmd: [u32; LLID_SLOT_COUNT],
    /// Counter reads whose selector names something this model cannot
    /// tell apart. Counted rather than silently answered with zero.
    pub unattributed_counter_reads: u64,

    llid_capture_mask: u32,
    llid_active_bitmap: u32,
    llid_mask_control: u32,
    llid_counter_mask: u32,
    tx_grant_mask: u32,
    rx_grant_mask: u32,
    irq_mask: u32,
    epon_status: u32,
    active_flags: u32,
    link_status_1g: u32,
    hw_state_status: u32,
    discovery_status: u32,
    /// REG_SPECIAL_0064 backing store. Silicon power-on is zero
    /// (observed on hardware); firmware programs this register during
    /// its init.
    special_0064: u32,

    pub trace: bool,
}

impl EponMac {
    pub fn new() -> Self {
        let table_words = ((EPON_TABLE_END - EPON_TABLE_BASE) / 4) as usize;
        let llid_words = ((EPON_LLID_TOP - EPON_LLID_BASE) / 4) as usize;
        let mut me = Self {
            table_store: vec![0u32; table_words],
            llid_store: vec![0u32; llid_words],
            llid_irq_pending: [0; LLID_SLOT_COUNT],
            llid_drain_flag: [true; LLID_SLOT_COUNT],
            queue_arrivals: std::collections::HashMap::new(),
            latch_cmd: [0; LLID_SLOT_COUNT],
            unattributed_counter_reads: 0,
            llid_capture_mask: LLID_CAPTURE_MASK_RESET,
            llid_active_bitmap: LLID_ACTIVE_RESET,
            llid_mask_control: 0,
            llid_counter_mask: LLID_COUNTER_MASK_RESET,
            tx_grant_mask: 0,
            rx_grant_mask: RX_GRANT_MASK_RESET,
            irq_mask: 0,
            epon_status: 0,
            active_flags: 0,
            link_status_1g: 0,
            hw_state_status: 0,
            discovery_status: 0,
            special_0064: 0,
            trace: false,
        };
        me.apply_silicon_power_on();
        me
    }

    /// Write a raw value into the LLID backing store at the given
    /// absolute MMIO address. Used by the OLT emulator to inject
    /// bitmap bits and mailbox data into the EPON MAC's address space
    /// without going through the normal write_word side-effect path.
    /// Set the 1G PHY link change bit (bit 1 of REG_1G_LINK_STATUS).
    pub fn set_1g_link_change_bit(&mut self) {
        self.link_status_1g |= 0x2;
    }

    /// Set the 10G PHY link status bit (bit 6 of REG_HW_STATE_STATUS).
    ///
    /// bit6 is a sticky LEVEL latch, NOT a transient pulse. On real
    /// silicon it is the 10G PCS 64b/66b block-lock status: it re-latches
    /// every block while a valid downstream stream is present and the
    /// lane-3 RX path is up. The firmware W1C-clears it each tick and
    /// checks whether it re-latched on the next tick — a CONTINUOUS lock
    /// re-asserts, a dropped lock does not.
    ///
    /// The bank calls this every tick (OLT-gated) while the OLT model is
    /// broadcasting a valid downstream + lane-3 RX is up, so bit6
    /// re-asserts as a level. There is NO auto-clear: the level holds
    /// until the W1C write clears the latch history (it then re-asserts
    /// on the next tick if the stream is still present) or the bank stops
    /// driving it (stream gone). DO-NOT-FAKE: the VALUE comes from the
    /// modelled downstream-present condition, never from a firmware write.
    pub fn set_phy_link_status_bit(&mut self) {
        self.hw_state_status |= 0x40;
    }

    /// Clear the 10G PHY link status bit6 level — called by the bank when
    /// the OLT model is no longer broadcasting a valid downstream, so the
    /// PCS block-lock drops.
    pub fn clear_phy_link_status_bit(&mut self) {
        self.hw_state_status &= !0x40;
    }

    /// Set bit 2 of REG_DISCOVERY_STATUS (OLT discovery detected).
    /// Called by the bank tick when the OLT is enabled and link is up.
    pub fn set_discovery_status_bit(&mut self) {
        self.discovery_status |= 0x4;
    }

    pub fn poke_llid_store(&mut self, addr: u32, val: u32) {
        if (EPON_LLID_BASE..EPON_LLID_TOP).contains(&addr) {
            let idx = Self::llid_idx(addr);
            self.llid_store[idx] = val;
        }
    }

    pub fn poke_table_store(&mut self, addr: u32, val: u32) {
        if (EPON_TABLE_BASE..EPON_TABLE_END).contains(&addr) {
            let idx = Self::table_idx(addr);
            self.table_store[idx] = val;
        }
    }

    /// Read a raw value from the LLID backing store.
    pub fn peek_llid_store(&self, addr: u32) -> u32 {
        if (EPON_LLID_BASE..EPON_LLID_TOP).contains(&addr) {
            let idx = Self::llid_idx(addr);
            self.llid_store[idx]
        } else {
            0
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        // Word-align the address so half / byte sub-offsets (e.g.
        // `0x01000001`) route to the same peripheral as the word.
        // Sparse single-register claims break under byte / half
        // access otherwise — the trait default `read_half` path
        // passes the word-aligned base but some routing layers
        // forward the raw address.
        let word = addr & !0x3;
        match word {
            REG_CHIP_ID | REG_CHIP_REV | REG_LLID_CAPTURE_MASK | REG_LLID_ACTIVE_BITMAP
            | REG_LLID_MASK_CONTROL | REG_LLID_COUNTER_MASK | REG_TX_GRANT_MASK
            | REG_RX_GRANT_MASK | REG_IRQ_MASK | REG_EPON_STATUS | REG_ACTIVE_FLAGS
            | REG_MDIO_COMMAND | REG_SPECIAL_0064
            | REG_1G_LINK_STATUS | REG_HW_STATE_STATUS | REG_DISCOVERY_STATUS => true,
            _ => {
                (EPON_TABLE_BASE..EPON_TABLE_END).contains(&addr)
                    || (EPON_LLID_BASE..EPON_LLID_TOP).contains(&addr)
            }
        }
    }

    fn table_idx(addr: u32) -> usize {
        ((addr - EPON_TABLE_BASE) / 4) as usize
    }

    fn llid_idx(addr: u32) -> usize {
        ((addr - EPON_LLID_BASE) / 4) as usize
    }

    /// The queue pointer register, as the hardware presents it.
    ///
    /// The same address is a selector on write and a pointer on read.
    /// Software writes `(queue & 0x1F) | (mode << 5)` to say which queue
    /// and which of the two pointers it wants, then reads bits [26:9]
    /// back. Inside that field the pointer sits at bits [15:6], which is
    /// how the readback prints `0:028:00` for a queue starting at 0x028.
    ///
    /// A queue nothing has consumed reads back at its own start address:
    /// both pointers begin at the head of the ring. The start comes from
    /// the queue's own configuration word — the same one the readback
    /// prints as `S` — so this is derived, never stored.
    ///
    /// ⚠ Bit 8 stays **derived**. Software writes this word with bit 8
    /// clear, and a model that stored the written value back would leave
    /// the firmware spinning on a drain-done bit that never comes.

    /// The report template the firmware programmed, read without side
    /// effects: `(length, length_plus_four)`.
    ///
    /// The firmware computes both from fields the far end put in its
    /// REGISTER frame and writes them here; the MAC is what turns them
    /// into a frame. Reading them back is what makes a report an oracle
    /// rather than a frame the model invented.
    pub fn report_template(&self) -> (u16, u16) {
        let head = self.peek_word(REG_REPORT_FRAME_PARAMS).unwrap_or(0);
        let tail = self.peek_word(REG_REPORT_FRAME_LEN).unwrap_or(0);
        ((head >> 16) as u16, tail as u16)
    }

    /// Record frames arriving in a queue. Called by the datapath, which
    /// is the only thing that knows a frame landed; this peripheral just
    /// holds the number so software can latch it.
    pub fn record_queue_arrivals(&mut self, slot: u8, frames: u64) {
        if frames == 0 {
            return;
        }
        let chan = (slot >> 5) as usize;
        let queue = (slot & 0x1F) as u32;
        *self.queue_arrivals.entry((chan, queue)).or_insert(0) += frames;
    }

    /// Arrivals recorded for a queue, for tests and inspection.
    pub fn queue_arrivals(&self, chan: usize, queue: u32) -> u64 {
        self.queue_arrivals.get(&(chan, queue)).copied().unwrap_or(0)
    }

    /// The counter-latch command port.
    ///
    /// Software writes `((extra & 7) << 9) | ((queue & 0x1F) << 4) | op`
    /// here — `0xC` latches a counter for reading, `0x4` writes one — and
    /// then **spins on this same address** until bit 31 clears. That wait
    /// has no timeout, so the busy bit is never left set, whatever the
    /// command asked for.
    fn write_latch_cmd(&mut self, slot: usize, val: u32) {
        self.latch_cmd[slot] = val & !0x8000_0000;
    }

    /// The latched counter word.
    ///
    /// The queue comes from the command written just before, bits [8:4].
    ///
    /// ⚠ The selector at bits [11:9] chooses **which** counter of the
    /// several a queue has, and which is which is **not established** —
    /// the readback prints six per channel and none of them is pinned to
    /// a quantity. This model knows exactly one number about a queue: how
    /// many frames arrived in it. It returns that for every selector, and
    /// counts the reads it cannot attribute, so the ambiguity is visible
    /// rather than hidden behind a plausible zero.
    fn peek_latched_counter(&self, slot: usize) -> u32 {
        let queue = (self.latch_cmd[slot] >> 4) & 0x1F;
        self.queue_arrivals.get(&(slot, queue)).copied().unwrap_or(0) as u32
    }

    /// Same value, but counted. `peek` must stay free of side effects —
    /// the bank peeks a word before writing it.
    fn latched_counter(&mut self, slot: usize) -> u32 {
        self.unattributed_counter_reads += 1;
        self.peek_latched_counter(slot)
    }

    fn queue_pointer_word(&self, slot: usize, addr: u32) -> u32 {
        let idx = Self::llid_idx(addr);
        let selector = self.llid_store[idx] & 0x3F;
        let queue = selector & 0x1F;
        // Queue configuration words live at +0x80 in the same slot, one
        // per queue; the start address is their low eleven bits.
        let cfg_addr = EPON_LLID_BASE + (slot as u32) * EPON_LLID_STRIDE + 0x80 + queue * 4;
        let start = self.llid_store[Self::llid_idx(cfg_addr)] & 0x7FF;
        let pointer_field = (start << 6) & 0x3_FFFF;
        let drain = if self.llid_drain_flag[slot] { 0x100 } else { 0 };
        (pointer_field << 9) | drain | selector
    }

    /// Classify an LLID-window address into `(slot, offset_within_slot)`.
    fn llid_slot(addr: u32) -> (usize, u32) {
        let rel = addr - EPON_LLID_BASE;
        let slot = (rel / EPON_LLID_STRIDE) as usize;
        let within = rel % EPON_LLID_STRIDE;
        (slot, within)
    }

    fn apply_silicon_power_on(&mut self) {
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if !self.claims(abs) {
                continue;
            }
            // Route through our own side-effect-free backing store.
            self.write_store_no_side_effects(abs, val);
        }
    }

    fn write_store_no_side_effects(&mut self, addr: u32, val: u32) {
        match addr {
            REG_CHIP_ID | REG_CHIP_REV | REG_MDIO_COMMAND
            | REG_HW_STATE_STATUS => {
                // Fixed / read-only values — ignore power-on seed.
            }
            REG_SPECIAL_0064 => self.special_0064 = val,
            REG_LLID_CAPTURE_MASK => self.llid_capture_mask = val,
            REG_LLID_ACTIVE_BITMAP => self.llid_active_bitmap = val,
            REG_LLID_MASK_CONTROL => self.llid_mask_control = val,
            REG_LLID_COUNTER_MASK => self.llid_counter_mask = val,
            REG_TX_GRANT_MASK => self.tx_grant_mask = val,
            REG_RX_GRANT_MASK => self.rx_grant_mask = val,
            REG_IRQ_MASK => self.irq_mask = val,
            REG_EPON_STATUS => self.epon_status = val,
            REG_ACTIVE_FLAGS => self.active_flags = val,
            _ if (EPON_TABLE_BASE..EPON_TABLE_END).contains(&addr) => {
                let idx = Self::table_idx(addr);
                self.table_store[idx] = val;
            }
            _ if (EPON_LLID_BASE..EPON_LLID_TOP).contains(&addr) => {
                let idx = Self::llid_idx(addr);
                self.llid_store[idx] = val;
            }
            _ => {}
        }
    }
}

impl Peripheral for EponMac {
    fn name(&self) -> &'static str {
        "epon_mac"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        // Cosmetic — the bank dispatches via `claims()`, which is a
        // predicate. We still return the two large windows so logs
        // and snapshots know the peripheral "owns" this area.
        const RANGES: &[AddressRange] = &[
            AddressRange::new(EPON_TABLE_BASE, EPON_TABLE_END),
            AddressRange::new(EPON_LLID_BASE, EPON_LLID_TOP),
        ];
        RANGES
    }

    fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        // Side-effect-free probe — same match arms as `read_word`
        // minus the MPCP-command-latch auto-clear. Anything not
        // listed falls back to `Ok(0)` so callers see a pure
        // zero rather than triggering a mutation.
        match addr {
            REG_CHIP_ID => return Ok(CHIP_ID_VALUE),
            REG_CHIP_REV => return Ok(CHIP_REV_VALUE),
            REG_LLID_CAPTURE_MASK => return Ok(self.llid_capture_mask),
            REG_LLID_ACTIVE_BITMAP => return Ok(self.llid_active_bitmap),
            REG_LLID_MASK_CONTROL => return Ok(self.llid_mask_control),
            REG_LLID_COUNTER_MASK => return Ok(self.llid_counter_mask),
            REG_TX_GRANT_MASK => return Ok(self.tx_grant_mask),
            REG_RX_GRANT_MASK => return Ok(self.rx_grant_mask),
            REG_IRQ_MASK => return Ok(self.irq_mask),
            REG_EPON_STATUS => return Ok(self.epon_status),
            REG_ACTIVE_FLAGS => return Ok(self.active_flags),
            REG_MDIO_COMMAND => return Ok(0),
            REG_SPECIAL_0064 => return Ok(self.special_0064),
            REG_1G_LINK_STATUS => return Ok(self.link_status_1g),
            REG_HW_STATE_STATUS => return Ok(self.hw_state_status),
            REG_DISCOVERY_STATUS => return Ok(self.discovery_status),
            _ => {}
        }
        if (EPON_LLID_BASE..EPON_LLID_TOP).contains(&addr) {
            let (slot, within) = Self::llid_slot(addr);
            if slot < LLID_SLOT_COUNT {
                match within {
                    0x04 => return Ok(self.llid_irq_pending[slot]),
                    0x3C => return Ok(self.queue_pointer_word(slot, addr)),
                    0x1D8 => return Ok(self.peek_latched_counter(slot)),
                    _ => {
                        let idx = Self::llid_idx(addr);
                        return Ok(self.llid_store[idx]);
                    }
                }
            }
            let idx = Self::llid_idx(addr);
            return Ok(self.llid_store[idx]);
        }
        if (EPON_TABLE_BASE..EPON_TABLE_END).contains(&addr) {
            let idx = Self::table_idx(addr);
            return Ok(self.table_store[idx]);
        }
        Ok(0)
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        // Sparse core first.
        match addr {
            REG_CHIP_ID => return Ok(CHIP_ID_VALUE),
            REG_CHIP_REV => return Ok(CHIP_REV_VALUE),
            REG_LLID_CAPTURE_MASK => return Ok(self.llid_capture_mask),
            REG_LLID_ACTIVE_BITMAP => return Ok(self.llid_active_bitmap),
            REG_LLID_MASK_CONTROL => return Ok(self.llid_mask_control),
            REG_LLID_COUNTER_MASK => return Ok(self.llid_counter_mask),
            REG_TX_GRANT_MASK => return Ok(self.tx_grant_mask),
            REG_RX_GRANT_MASK => return Ok(self.rx_grant_mask),
            REG_IRQ_MASK => return Ok(self.irq_mask),
            REG_EPON_STATUS => return Ok(self.epon_status),
            REG_ACTIVE_FLAGS => return Ok(self.active_flags),
            REG_MDIO_COMMAND => return Ok(0),
            REG_1G_LINK_STATUS => return Ok(self.link_status_1g),
            REG_HW_STATE_STATUS => return Ok(self.hw_state_status),
            REG_SPECIAL_0064 => return Ok(self.special_0064),
            REG_DISCOVERY_STATUS => return Ok(self.discovery_status),
            _ => {}
        }

        // Windowed reads.
        if (EPON_LLID_BASE..EPON_LLID_TOP).contains(&addr) {
            let (slot, within) = Self::llid_slot(addr);
            if slot < LLID_SLOT_COUNT {
                match within {
                    0x04 => return Ok(self.llid_irq_pending[slot]),
                    0x3C => return Ok(self.queue_pointer_word(slot, addr)),
                    0x1D8 => return Ok(self.latched_counter(slot)),
                    _ => {
                        let idx = Self::llid_idx(addr);
                        return Ok(self.llid_store[idx]);
                    }
                }
            }
            let idx = Self::llid_idx(addr);
            return Ok(self.llid_store[idx]);
        }

        if (EPON_TABLE_BASE..EPON_TABLE_END).contains(&addr) {
            let idx = Self::table_idx(addr);
            return Ok(self.table_store[idx]);
        }

        Ok(0)
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        match addr {
            REG_CHIP_ID | REG_CHIP_REV => return Ok(()), // read-only
            REG_LLID_CAPTURE_MASK => {
                self.llid_capture_mask = val;
                return Ok(());
            }
            REG_LLID_ACTIVE_BITMAP => {
                self.llid_active_bitmap = val;
                return Ok(());
            }
            REG_LLID_MASK_CONTROL => {
                self.llid_mask_control = val;
                return Ok(());
            }
            REG_LLID_COUNTER_MASK => {
                self.llid_counter_mask = val;
                return Ok(());
            }
            REG_TX_GRANT_MASK => {
                self.tx_grant_mask = val;
                return Ok(());
            }
            REG_RX_GRANT_MASK => {
                self.rx_grant_mask = val;
                return Ok(());
            }
            REG_IRQ_MASK => {
                self.irq_mask = val;
                return Ok(());
            }
            REG_EPON_STATUS => {
                // EPON_STATUS bit 0 is a "latch counters" trigger that
                // auto-clears; bits [2:1] are status. Preserve bits
                // [2:1] and discard bit 0.
                self.epon_status = val & !0x1;
                return Ok(());
            }
            REG_ACTIVE_FLAGS => {
                self.active_flags = val;
                return Ok(());
            }
            REG_MDIO_COMMAND => return Ok(()),
            REG_1G_LINK_STATUS => {
                // W1C: bits written as 1 are cleared.
                self.link_status_1g &= !val;
                return Ok(());
            }
            REG_HW_STATE_STATUS => {
                // W1C: bits written as 1 are cleared.
                self.hw_state_status &= !val;
                return Ok(());
            }
            REG_SPECIAL_0064 => {
                self.special_0064 = val;
                return Ok(());
            }
            REG_DISCOVERY_STATUS => {
                // Bit 0: latch trigger (write-through, readback confirms done).
                // Bits[2:1]: W1C (write 1 to clear status).
                self.discovery_status = (self.discovery_status & !(val & 0x6))
                    | (val & !0x6);
                return Ok(());
            }
            _ => {}
        }

        if (EPON_LLID_BASE..EPON_LLID_TOP).contains(&addr) {
            let (slot, within) = Self::llid_slot(addr);
            if slot < LLID_SLOT_COUNT {
                match within {
                    0x04 => {
                        // W1C — clear any bit the firmware writes 1 to.
                        self.llid_irq_pending[slot] &= !val;
                        return Ok(());
                    }
                    0x1D4 => {
                        // Counter-latch command. The firmware spins on
                        // this same address for bit 31 with no timeout,
                        // so the busy bit never survives the write.
                        self.write_latch_cmd(slot, val);
                        return Ok(());
                    }
                    0x3C => {
                        // Queue drain — writing bit 8 clears the flag.
                        if val & 0x100 != 0 {
                            self.llid_drain_flag[slot] = false;
                        }
                        let idx = Self::llid_idx(addr);
                        self.llid_store[idx] = val & !0x100;
                        return Ok(());
                    }
                    _ => {}
                }
            }
            let idx = Self::llid_idx(addr);
            self.llid_store[idx] = val;
            return Ok(());
        }

        if (EPON_TABLE_BASE..EPON_TABLE_END).contains(&addr) {
            // LLID anchor registers clear bit 0 on write.
            let final_val = if LLID_ANCHOR_ADDRS.contains(&addr) {
                val & !0x1
            } else {
                val
            };
            let idx = Self::table_idx(addr);
            self.table_store[idx] = final_val;
            return Ok(());
        }

        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        // Re-arm queue drain flags every tick so the firmware's
        // polling loop always sees new data — this keeps parity with
        // the old shim arm that unconditionally set bit 8.
        for flag in &mut self.llid_drain_flag {
            *flag = true;
        }
        // bit6 of REG_HW_STATE_STATUS is a sticky LEVEL latch driven by
        // the bank (OLT-gated), not a transient pulse — no auto-clear
        // here. The bank re-asserts it every tick while the downstream
        // stream is present; the firmware's W1C clear is honored by
        // write_word, and the level re-asserts on the next bank tick.
    }

    fn reset_cold(&mut self) {
        self.table_store.fill(0);
        self.llid_store.fill(0);
        self.llid_irq_pending = [0; LLID_SLOT_COUNT];
        self.llid_drain_flag = [true; LLID_SLOT_COUNT];
        self.queue_arrivals.clear();
        self.latch_cmd = [0; LLID_SLOT_COUNT];
        self.unattributed_counter_reads = 0;
        self.llid_capture_mask = LLID_CAPTURE_MASK_RESET;
        self.llid_active_bitmap = LLID_ACTIVE_RESET;
        self.llid_mask_control = 0;
        self.llid_counter_mask = LLID_COUNTER_MASK_RESET;
        self.tx_grant_mask = 0;
        self.rx_grant_mask = RX_GRANT_MASK_RESET;
        self.irq_mask = 0;
        self.epon_status = 0;
        self.active_flags = 0;
        self.link_status_1g = 0;
        self.hw_state_status = 0;
        self.discovery_status = 0;
        self.special_0064 = 0;
        // Apply silicon power-on snapshot (covers table_store /
        // llid_store entries that aren't reset by individual fields).
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        // Silicon power-on snapshot already covers warm reset state —
        // the only difference is `STATUS32.E1/E2`, set in
        // `boot_from_flash`.
        self.reset_cold();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::EponMac(EponMacSnapshot {
            chip_id: CHIP_ID_VALUE,
            chip_rev: CHIP_REV_VALUE,
            llid_active_bitmap: self.llid_active_bitmap,
            llid_capture_mask: self.llid_capture_mask,
            rx_grant_mask: self.rx_grant_mask,
            tx_grant_mask: self.tx_grant_mask,
            irq_mask: self.irq_mask,
            llid_irq_pending: self.llid_irq_pending,
        })
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Epon(ev) => match ev {
                EponEvent::SetLlidActive(llid, active) => {
                    let bit = 1u32 << (*llid as u32);
                    if *active {
                        self.llid_active_bitmap |= bit;
                    } else {
                        self.llid_active_bitmap &= !bit;
                    }
                    Ok(())
                }
                EponEvent::InjectLlidInterrupt(llid) => {
                    let slot = (*llid as usize) % LLID_SLOT_COUNT;
                    self.llid_irq_pending[slot] |= 1u32 << (*llid as u32 & 0x1F);
                    Ok(())
                }
                EponEvent::ResetCounters => {
                    // Counter results are synthetic zero; nothing to
                    // clear today. Still acknowledge the event so the
                    // UI can bind the button.
                    Ok(())
                }
            },
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chip_id_and_rev_are_fixed() {
        let mut m = EponMac::new();
        assert_eq!(m.read_word(REG_CHIP_ID).unwrap(), CHIP_ID_VALUE);
        assert_eq!(m.read_word(REG_CHIP_REV).unwrap(), CHIP_REV_VALUE);
        // Writes are silently dropped.
        m.write_word(REG_CHIP_ID, 0xDEADBEEF).unwrap();
        assert_eq!(m.read_word(REG_CHIP_ID).unwrap(), CHIP_ID_VALUE);
    }

    #[test]
    fn llid_anchor_bit0_clears_on_write() {
        let mut m = EponMac::new();
        m.write_word(0x0100_043C, 0x0001_7FFF).unwrap();
        assert_eq!(m.read_word(0x0100_043C).unwrap(), 0x0001_7FFE);
        m.write_word(0x0100_0D00, 0xFFFF_FFFF).unwrap();
        assert_eq!(m.read_word(0x0100_0D00).unwrap(), 0xFFFF_FFFE);
    }

    #[test]
    fn llid_irq_status_w1c() {
        let mut m = EponMac::new();
        m.inject_event(&PeripheralEvent::Epon(EponEvent::InjectLlidInterrupt(1)))
            .unwrap();
        let addr = EPON_LLID_BASE + EPON_LLID_STRIDE + 0x04;
        assert!(m.read_word(addr).unwrap() != 0);
        m.write_word(addr, 0xFFFF_FFFF).unwrap();
        assert_eq!(m.read_word(addr).unwrap(), 0);
    }

    #[test]
    fn bit6_is_a_sticky_level_not_a_50_tick_pulse() {
        // bit6 of 0x01000E04 is a LEVEL latch. Once set it does NOT
        // auto-clear after N ticks. It only clears on a W1C write or an
        // explicit level-drop from the bank.
        let mut m = EponMac::new();
        assert_eq!(m.read_word(REG_HW_STATE_STATUS).unwrap() & 0x40, 0);
        m.set_phy_link_status_bit();
        assert_eq!(m.read_word(REG_HW_STATE_STATUS).unwrap() & 0x40, 0x40);
        // Many ticks pass — the level holds (no auto-clear).
        for _ in 0..200 {
            m.tick(64);
        }
        assert_eq!(
            m.read_word(REG_HW_STATE_STATUS).unwrap() & 0x40,
            0x40,
            "bit6 must stay set as a level, not pulse away after 50 ticks"
        );
        // W1C clears the latch history.
        m.write_word(REG_HW_STATE_STATUS, 0x40).unwrap();
        assert_eq!(m.read_word(REG_HW_STATE_STATUS).unwrap() & 0x40, 0);
        // The bank re-asserts it next tick while the stream is present
        // (modelled by calling set_phy_link_status_bit again) — continuous.
        m.set_phy_link_status_bit();
        assert_eq!(m.read_word(REG_HW_STATE_STATUS).unwrap() & 0x40, 0x40);
        // Explicit level drop (downstream gone).
        m.clear_phy_link_status_bit();
        assert_eq!(m.read_word(REG_HW_STATE_STATUS).unwrap() & 0x40, 0);
    }

    #[test]
    fn queue_drain_bit8_sticky_until_acked() {
        let mut m = EponMac::new();
        let addr = EPON_LLID_BASE + 0x3C;
        assert_eq!(m.read_word(addr).unwrap() & 0x100, 0x100);
        // Firmware acks bit 8 → flag clears.
        m.write_word(addr, 0x100).unwrap();
        assert_eq!(m.read_word(addr).unwrap() & 0x100, 0);
        // Tick re-arms it.
        m.tick(64);
        assert_eq!(m.read_word(addr).unwrap() & 0x100, 0x100);
    }

    /// A queue nothing has arrived in reads zero — and that zero is a
    /// measurement, not a hardcoded answer: the same port returns the
    /// arrivals once the datapath reports some.
    ///
    /// This replaces a test that asserted the port is *always* zero. It
    /// passed for the wrong reason: two read paths returned a literal
    /// zero, so the assertion could not fail and the port had no
    /// denominator at all.
    #[test]
    fn a_latched_counter_answers_the_arrivals_of_the_selected_queue() {
        let mut m = EponMac::new();
        m.reset_cold();
        let cmd = EPON_LLID_BASE + 0x1D4;
        let data = EPON_LLID_BASE + 0x1D8;

        // Nothing has arrived anywhere yet.
        for queue in [0x00u32, 0x0F, 0x10] {
            m.write_word(cmd, (queue << 4) | 0xC).unwrap();
            assert_eq!(m.read_word(data).unwrap(), 0);
        }

        // The datapath reports three frames into queue 0x10 and one into
        // 0x0F; each queue answers for itself.
        m.record_queue_arrivals(0x10, 3);
        m.record_queue_arrivals(0x0F, 1);

        m.write_word(cmd, (0x10 << 4) | 0xC).unwrap();
        assert_eq!(m.read_word(data).unwrap(), 3);
        m.write_word(cmd, (0x0F << 4) | 0xC).unwrap();
        assert_eq!(m.read_word(data).unwrap(), 1);
        m.write_word(cmd, 0xC).unwrap();
        assert_eq!(m.read_word(data).unwrap(), 0, "queue 0 saw nothing");
    }

    /// The wait on the command port has no timeout, so the busy bit must
    /// never survive a write — whatever the command asked for.
    #[test]
    fn the_counter_latch_never_stays_busy() {
        let mut m = EponMac::new();
        m.reset_cold();
        for val in [0x8000_000Cu32, 0x8000_0FFC, 0xFFFF_FFFF, 0x0000_0004] {
            m.write_word(EPON_LLID_BASE + 0x1D4, val).unwrap();
            assert_eq!(m.read_word(EPON_LLID_BASE + 0x1D4).unwrap() & 0x8000_0000, 0);
        }
    }

    /// Peeking must not move a counter of reads — the bank peeks a word
    /// before writing it.
    #[test]
    fn peeking_the_counter_port_has_no_side_effect() {
        let mut m = EponMac::new();
        m.reset_cold();
        m.record_queue_arrivals(0x10, 2);
        m.write_word(EPON_LLID_BASE + 0x1D4, (0x10 << 4) | 0xC).unwrap();
        let before = m.unattributed_counter_reads;
        assert_eq!(m.peek_word(EPON_LLID_BASE + 0x1D8).unwrap(), 2);
        assert_eq!(m.unattributed_counter_reads, before);
    }

    #[test]
    fn cold_reset_silicon_power_on_defaults() {
        let mut m = EponMac::new();
        m.reset_cold();
        // Silicon: RX_GRANT_MASK = 0x0000FFFF
        assert_eq!(m.read_word(REG_RX_GRANT_MASK).unwrap(), 0x0000_FFFF);
        // Silicon: LLID_CAPTURE_MASK / PON_MODE = 0 (bit 15 would
        // freeze silicon if set — see emu-sysreg-reset-values-wrong).
        assert_eq!(m.read_word(REG_LLID_CAPTURE_MASK).unwrap(), 0);
        // Silicon: LLID_ACTIVE_BITMAP = 0
        assert_eq!(m.read_word(REG_LLID_ACTIVE_BITMAP).unwrap(), 0);
        // Silicon: LLID_COUNTER_MASK = 0x00077FF7
        assert_eq!(m.read_word(REG_LLID_COUNTER_MASK).unwrap(), 0x0007_7FF7);
        // MPCP_CMD_LATCH moved to LaneBus mpcp_bus (lane 8).
        // Silicon: SPECIAL_0064 = 0
        assert_eq!(m.read_word(REG_SPECIAL_0064).unwrap(), 0);
    }

    #[test]
    fn claims_predicate_excludes_other_peripherals() {
        let m = EponMac::new();
        // eFuse / I2C UDR — must stay in sysreg.
        assert!(!m.claims(0x0100_0040));
        assert!(!m.claims(0x0100_0048));
        assert!(!m.claims(0x0100_004C));
        // EPON free-running counter — sysreg/timer.
        assert!(!m.claims(0x0100_0050));
        // BSC I²C — its own peripheral.
        assert!(!m.claims(0x0100_0140));
        // SerDes window.
        assert!(!m.claims(0x0100_0180));
        // PBC SPI.
        assert!(!m.claims(0x0100_0200));
        // MACsec.
        assert!(!m.claims(0x0100_2400));
        // EPON-owned.
        assert!(m.claims(0x0100_0000));
        assert!(m.claims(0x0100_0030));
        assert!(m.claims(0x0100_043C));
        assert!(m.claims(0x0100_1404));
        // 0x0100_2804 is owned by MACsec.
        assert!(!m.claims(0x0100_2804));
    }

    /// The queue pointer register answers with the queue's own start
    /// address, cadred the way the readback path decodes it: bits [26:9]
    /// of the word, pointer at [15:6] inside that field.
    ///
    /// Real hardware prints `Rd:0:028:00` for a queue whose start is
    /// `0x028` and `Rd:0:038:00` for one starting at `0x038`; a model
    /// returning the stored selector printed `0:000:00` for both.
    #[test]
    fn a_queue_pointer_reads_back_at_the_start_of_its_queue() {
        let mut m = EponMac::new();
        m.reset_cold();
        for (queue, start, end) in [(0x0Eu32, 0x000u32, 0x027u32), (0x0F, 0x028, 0x037), (0x10, 0x038, 0x047)] {
            // The queue configuration word, as software programs it.
            m.write_word(0x0100_1480 + queue * 4, (end << 12) | start).unwrap();
            // Select that queue, read pointer 0 then pointer 1.
            for mode in 0..2u32 {
                m.write_word(0x0100_143C, queue | (mode << 5)).unwrap();
                let word = m.read_word(0x0100_143C).unwrap();
                let field = (word >> 9) & 0x3_FFFF;
                assert_eq!((field & 0xFFC0) >> 6, start, "queue {queue:#04x} pointer {mode}");
                assert_eq!(field & 0x3F, 0);
            }
        }
    }

    /// Bit 8 is derived, never stored. Software writes this word with
    /// bit 8 clear; storing that back would leave the firmware spinning
    /// on a drain-done bit that never arrives — its wait has no timeout.
    #[test]
    fn the_drain_done_bit_survives_a_selector_write() {
        let mut m = EponMac::new();
        m.reset_cold();
        m.write_word(0x0100_143C, 0x0F).unwrap();
        assert_ne!(m.read_word(0x0100_143C).unwrap() & 0x100, 0);
    }
}
