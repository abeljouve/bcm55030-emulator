//! BCM55030 EPON MAC peripheral — Session 3.
//!
//! Hosts the core EPON MAC register surface that was previously
//! hardcoded inside `sysreg_shim`. The peripheral owns a **sparse**
//! claim set rather than one contiguous MMIO range, because the
//! BCM55030 interleaves EPON MAC registers with unrelated subsystems
//! (I²C/eFuse control at `0x040/0x048/0x04C`, the free-running
//! counter at `0x050`, BSC at `0x140..0x158`, SerDes at `0x180..0x1F8`,
//! PBC at `0x1F0..0x240`) inside the same `0x01000000..0x01002000`
//! window. A contiguous range would steal those offsets.
//!
//! The peripheral claims, by predicate:
//!
//!   * Sparse core registers — CHIP_ID, CHIP_REV, LLID masks, active
//!     bitmap, grant masks, IRQ mask, EPON status, active flags,
//!     plus the `0x0064` special half-`FFFF` read arm. Note: the
//!     `0x01002804` register previously claimed here as a fatal-
//!     error aggregator moved to `macsec.rs` in Session 4 — it sits
//!     inside the MACsec 10G SA programming bank.
//!   * `0x01000400..0x01000E80` — LLID grant / enable tables,
//!     per-LLID config (anchors 0x043C, 0x04B8, 0x0D00, 0x0D7C
//!     clear bit 0 on write — audit 5.9).
//!   * `0x01001400..0x01002000` — LLID IRQ status + counter stats +
//!     queue drain. Six stride-`0x200` blocks, one per LLID slot:
//!       - `0x1X04` IRQ status — always 0, W1C (audit 5.4).
//!       - `0x1XD8` counter result slot — always 0.
//!       - `0x1X3C` queue drain — bit 8 permanently set with
//!         per-offset auto-clear (audit 5.7).
//!   * `0x01002804` — fatal error aggregator, always 0 W1C (audit
//!     5.5 partial; the rest lands in Session 7 `fatal_filter.rs`).
//!
//! Audit items resolved in this session:
//!
//!   * **5.4** — LLID IRQ status forced to 0 moved from a generic
//!     shim arm into a proper peripheral with per-LLID IRQ state.
//!   * **5.5 (part)** — Fatal error register `0x2804` returns 0.
//!   * **5.7** — DMA queue drain bit 8 is now driven by real per-LLID
//!     state with targeted auto-clear, not a blanket sysreg arm.
//!   * **5.9** — LLID 0/31 anchor registers clear bit 0 on write in
//!     the peripheral that owns them.
//!   * **5.12 (part)** — `SYSREG_INIT_VALUES` residual shrinks: every
//!     offset in the EPON MAC claim set is now loaded by this
//!     peripheral's `reset_warm()`, not by the shim.
//!
//! Not yet owned (left to `sysreg_shim` or future sessions):
//!
//!   * `0x01000080..0x010000BC` — queue priority / DPoE flavour
//!     registers. No special semantics needed yet, backing store
//!     is enough (sysreg generic path).
//!   * `0x01002000..0x01002800` — MACsec / DMA (Session 4).
//!   * `0x01002820..0x01003600` — fatal + filter tail (Session 7).

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, EponEvent, EponMacSnapshot, Peripheral, PeripheralError, PeripheralEvent,
    PeripheralSnapshot,
};

pub const CHIP_ID_VALUE: u32 = 0x47010203;
pub const CHIP_REV_VALUE: u32 = 0xB2110816;
/// Silicon power-on default for LLID_CAPTURE_MASK / PON_MODE — zero on
/// hardware (verified 2026-04-29 via hardware probing). Bit 15 of this
/// register gates into an unclocked PCS domain, so a non-zero reset
/// value would freeze the bus on real silicon. See
/// `the design notes`.
pub const LLID_CAPTURE_MASK_RESET: u32 = 0x0000_0000;
/// Silicon power-on default for LLID_ACTIVE_BITMAP — zero on hardware
/// (verified 2026-04-29 via hardware probing). Reference firmware programs the
/// active bitmap during init.
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
/// MPCP-adjacent command latch. The firmware writes a value with
/// bits `[31:27]` set (command opcode) and polls the register for
/// those bits to clear. Identified by bisecting the sysreg residual
/// auto-clear region during deferral D7 — this is the one register
/// in the whole `0x01000000..0x01003800` window that still depends
/// on the command-bit semantic. Not yet in `hwregs`; the access
/// pattern matches MPCP LLID control (block 22) but the offset is
/// not documented.
const REG_MPCP_CMD_LATCH: u32 = 0x0100_0160;
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

    /// MPCP command-latch register at `0x01000160`. Bits `[31:27]`
    /// hold the current opcode; any non-zero value auto-clears on
    /// the next read so the firmware polling loop progresses.
    mpcp_cmd_latch: u32,
    mpcp_cmd_pending_clear: u32,

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
    /// REG_SPECIAL_0064 backing store. Silicon power-on is zero —
    /// the previous shim returned 0x5382_FFFF unconditionally, which
    /// did not match real silicon (verified 2026-04-29 via hardware probing).
    /// Reference firmware programs this register during its init.
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
            mpcp_cmd_latch: MPCP_CMD_LATCH_RESET,
            mpcp_cmd_pending_clear: 0,
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
    pub fn set_phy_link_status_bit(&mut self) {
        self.hw_state_status |= 0x40;
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
            | REG_MDIO_COMMAND | REG_SPECIAL_0064 | REG_MPCP_CMD_LATCH
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
            REG_MPCP_CMD_LATCH => self.mpcp_cmd_latch = val,
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
            REG_MPCP_CMD_LATCH => return Ok(self.mpcp_cmd_latch),
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
                    0x3C => {
                        let idx = Self::llid_idx(addr);
                        let base = self.llid_store[idx] & !0x100;
                        let set = if self.llid_drain_flag[slot] { 0x100 } else { 0 };
                        return Ok(base | set);
                    }
                    0x1D8 => return Ok(0),
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
            REG_MPCP_CMD_LATCH => {
                let val = self.mpcp_cmd_latch;
                if self.mpcp_cmd_pending_clear != 0 {
                    self.mpcp_cmd_latch &= !self.mpcp_cmd_pending_clear;
                    self.mpcp_cmd_pending_clear = 0;
                }
                return Ok(val);
            }
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
                    0x3C => {
                        // Queue drain register — bit 8 reflects the
                        // per-slot "has entries" flag, rest of the
                        // bits come from the backing store so the
                        // firmware's other config bits round-trip.
                        let idx = Self::llid_idx(addr);
                        let base = self.llid_store[idx] & !0x100;
                        let set = if self.llid_drain_flag[slot] { 0x100 } else { 0 };
                        return Ok(base | set);
                    }
                    0x1D8 => return Ok(0), // counter result — always 0
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
            REG_MPCP_CMD_LATCH => {
                self.mpcp_cmd_latch = val;
                let cmd_bits = val & 0xF800_0000;
                if cmd_bits != 0 {
                    self.mpcp_cmd_pending_clear = cmd_bits;
                }
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
    }

    fn reset_cold(&mut self) {
        self.table_store.fill(0);
        self.llid_store.fill(0);
        self.llid_irq_pending = [0; LLID_SLOT_COUNT];
        self.llid_drain_flag = [true; LLID_SLOT_COUNT];
        self.mpcp_cmd_latch = MPCP_CMD_LATCH_RESET;
        self.mpcp_cmd_pending_clear = 0;
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
        // `boot_from_flash`. Closes deferral D6.
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

    #[test]
    fn counter_result_slots_always_zero() {
        let mut m = EponMac::new();
        for slot in 0..LLID_SLOT_COUNT as u32 {
            let addr = EPON_LLID_BASE + slot * EPON_LLID_STRIDE + 0x1D8;
            assert_eq!(m.read_word(addr).unwrap(), 0);
        }
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
        // Silicon: MPCP_CMD_LATCH = 0x00015000
        assert_eq!(m.read_word(REG_MPCP_CMD_LATCH).unwrap(), 0x0001_5000);
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
        // MACsec (Session 4).
        assert!(!m.claims(0x0100_2400));
        // EPON-owned.
        assert!(m.claims(0x0100_0000));
        assert!(m.claims(0x0100_0030));
        assert!(m.claims(0x0100_043C));
        assert!(m.claims(0x0100_1404));
        // 0x0100_2804 moved to MACsec in Session 4.
        assert!(!m.claims(0x0100_2804));
    }
}
