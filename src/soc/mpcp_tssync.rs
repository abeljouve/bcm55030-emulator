//! BCM55030 MPCP timestamp-sync (NCO slave loop) register block.
//!
//! Models the MPCP TS-sync control/status registers that the firmware's
//! TS-sync state machine drives and reads back. On real silicon these
//! registers live in the EPON MAC's MPCP timing block; they are NOT a
//! dumb RAM store — three of them are HW-driven status reads:
//!
//! | Addr         | Name (firmware contract)                | R/W on silicon |
//! |:-------------|:----------------------------------------|:---------------|
//! | `0x01000300` | TS-sync arm (`MPCP_TS_BLK+0x18`, bit0)  | firmware R/W   |
//! | `0x01000304` | HW OLT-lock (`MPCP_TS_BLK+0x1c`, bit0)  | FW arms, HW latches |
//! | `0x01000314` | freq-error high word (firmware writes)  | firmware R/W   |
//! | `0x01000318` | acquisition/tracking window             | firmware R/W   |
//! | `0x0100031C` | acquisition/tracking mode               | firmware R/W   |
//! | `0x010000B4` | live OLT MPCP timestamp (HW captured)    | HW status read |
//! | `0x01000D88` | local NCO TX phase-offset correction    | firmware R/W   |
//! | `0x01000D8C` | local NCO TX timestamp (HW free-running) | HW status read |
//!
//! `0x01000320` (HW-captured OLT timestamp) and `0x01000324` (rate) are
//! claimed by `mpcp.rs` (block 52 TX rate) — this peripheral does NOT
//! re-claim them. The bank drives the OLT-derived value of `0x01000320`
//! into the `mpcp` backing store via `Mpcp::poke_tx_rate`, mirroring the
//! `epon_mac` bitmap-poke pattern, so `mpcp` stays the single owner.
//!
//! # HW-faithfulness / OLT-gating (CRITICAL)
//!
//! When the OLT model is DISABLED (the normal boot path and the
//! differential harness), this peripheral behaves EXACTLY like the
//! residual backing store it replaces: a plain read/write store seeded
//! from `SYSREG_INIT_VALUES` (so `0x010000B4` reads `0xFFFFFFFF`,
//! `0x01000D8C` reads `0x00C93C2E`, the rest read 0 until written).
//! No OLT-driven advance, no `0x01000304` bit0 latch. The bank only
//! calls the `poke_*` driver methods when `olt.config.enabled`, so the
//! boot-diff and real-dump boot are byte-identical to the prior model.
//!
//! When the OLT model is ENABLED, the bank drives:
//!   * `0x010000B4` / `0x01000D8C` from the OLT's advancing
//!     `mpcp_timestamp` / a monotonic local NCO counter;
//!   * `0x01000304` bit0 (OLT-lock) = 1 only once the OLT model has
//!     broadcast ≥1 downstream GATE — modelling HW timestamp lock off
//!     the recovered downstream, NOT a firmware write to `0x01000300`.
//!
//! All firmware-written registers (`0x01000300`, `0x01000314`,
//! `0x01000318`, `0x0100031C`, `0x01000D88`, and the firmware-arm bit of
//! `0x01000304`) round-trip through the backing store in BOTH modes.
//!
//! Datasheet basis: §14.6 (MPCP TX-rate `0x01000320`); the bit-level
//! TS-sync semantics of `0x01000304`/`0x010000B4` are an INFERRED
//! contract taken from the firmware's expected behaviour (marked
//! below), not yet verified against silicon.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{AddressRange, Peripheral, PeripheralSnapshot};

// OBSERVED: MPCP_TS_BLK = 0x010002E8, +0x18 = 0x01000300.
pub const REG_TS_ARM: u32 = 0x0100_0300; // MPCP_TS_BLK + 0x18, bit0 = firmware arm
pub const REG_TS_LOCK: u32 = 0x0100_0304; // MPCP_TS_BLK + 0x1c, bit0 = HW OLT-lock
pub const REG_TS_FREQ_ERR: u32 = 0x0100_0314; // freq-error high word (firmware write)
pub const REG_TS_WINDOW: u32 = 0x0100_0318; // MPCP_TS_BLK + 0x30, acquisition/tracking window
pub const REG_TS_MODE: u32 = 0x0100_031C; // MPCP_TS_BLK + 0x34, acquisition/tracking mode
pub const REG_CAPTURED_TS: u32 = 0x0100_00B4; // live OLT MPCP timestamp (HW captured)
pub const REG_NCO_TX_OFFSET: u32 = 0x0100_0D88; // local NCO TX phase-offset correction (firmware write)
pub const REG_NCO_TX_TS: u32 = 0x0100_0D8C; // local NCO TX timestamp (HW free-running)

const CLAIMED: [u32; 8] = [
    REG_CAPTURED_TS,
    REG_TS_ARM,
    REG_TS_LOCK,
    REG_TS_FREQ_ERR,
    REG_TS_WINDOW,
    REG_TS_MODE,
    REG_NCO_TX_OFFSET,
    REG_NCO_TX_TS,
];

const TSSYNC_RANGES: &[AddressRange] = &[
    AddressRange::new(REG_CAPTURED_TS, REG_CAPTURED_TS + 4),
    AddressRange::new(REG_TS_ARM, REG_TS_MODE + 4),
    AddressRange::new(REG_NCO_TX_OFFSET, REG_NCO_TX_TS + 4),
];

#[derive(Clone)]
pub struct MpcpTsSync {
    // Backing store fields — round-trip firmware writes in both modes.
    ts_arm: u32,
    /// Bit0 = firmware-arm latch (firmware writes 1, reads back), bits
    /// above are reserved. The HW OLT-lock indication is OR-ed in at
    /// read time from `hw_lock` (see `read_word`).
    ts_lock_fw: u32,
    ts_freq_err: u32,
    ts_window: u32,
    ts_mode: u32,
    captured_ts: u32,
    nco_tx_offset: u32,
    nco_tx_ts: u32,

    /// HW OLT-lock level — set by the bank (OLT-gated) once the OLT model
    /// has broadcast a downstream GATE. OR-ed into `0x01000304` bit0 on
    /// read. INFERRED contract (HW sets bit0 when synced).
    hw_lock: bool,

    pub trace: bool,
}

impl MpcpTsSync {
    pub fn new() -> Self {
        let mut me = Self {
            ts_arm: 0,
            ts_lock_fw: 0,
            ts_freq_err: 0,
            ts_window: 0,
            ts_mode: 0,
            captured_ts: 0,
            nco_tx_offset: 0,
            nco_tx_ts: 0,
            hw_lock: false,
            trace: false,
        };
        me.apply_silicon_power_on();
        me
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        CLAIMED.contains(&(addr & !0x3))
    }

    fn apply_silicon_power_on(&mut self) {
        // Seed from SYSREG_INIT_VALUES so the OLT-disabled read values
        // match the residual store this peripheral replaces.
        // OBSERVED (mmio_init.rs): 0x00B4 = 0xFFFFFFFF, 0x0D8C = 0x00C93C2E;
        // the other TS-sync offsets read zero on silicon.
        for &(off, val) in super::mmio_init::SYSREG_INIT_VALUES {
            let abs = 0x0100_0000 + off;
            if self.claims(abs) {
                self.store_seed(abs, val);
            }
        }
        self.hw_lock = false;
    }

    fn store_seed(&mut self, addr: u32, val: u32) {
        match addr & !0x3 {
            REG_TS_ARM => self.ts_arm = val,
            REG_TS_LOCK => self.ts_lock_fw = val,
            REG_TS_FREQ_ERR => self.ts_freq_err = val,
            REG_TS_WINDOW => self.ts_window = val,
            REG_TS_MODE => self.ts_mode = val,
            REG_CAPTURED_TS => self.captured_ts = val,
            REG_NCO_TX_OFFSET => self.nco_tx_offset = val,
            REG_NCO_TX_TS => self.nco_tx_ts = val,
            _ => {}
        }
    }

    // ── OLT-gated driver methods (called by bank ONLY when OLT enabled) ──

    /// Drive the live OLT MPCP timestamp (`0x010000B4`) from the OLT
    /// model's advancing `mpcp_timestamp`.
    pub fn drive_captured_ts(&mut self, ts: u32) {
        self.captured_ts = ts;
    }

    /// Drive the local NCO TX timestamp (`0x01000D8C`) — a monotonic
    /// counter in the bank-tick domain.
    pub fn drive_nco_tx_ts(&mut self, ts: u32) {
        self.nco_tx_ts = ts;
    }

    /// Set the HW OLT-lock level (`0x01000304` bit0). Driven by the
    /// bank once the OLT model has broadcast a downstream GATE — models
    /// HW timestamp lock off the recovered downstream, never a firmware
    /// write to `0x01000300`.
    pub fn set_hw_lock(&mut self, locked: bool) {
        self.hw_lock = locked;
    }

    /// Current firmware-written captured-TS value (for snapshot/peek).
    pub fn captured_ts(&self) -> u32 {
        self.captured_ts
    }
    pub fn nco_tx_ts(&self) -> u32 {
        self.nco_tx_ts
    }
    pub fn hw_lock(&self) -> bool {
        self.hw_lock
    }
}

impl Peripheral for MpcpTsSync {
    fn name(&self) -> &'static str {
        "mpcp_tssync"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        TSSYNC_RANGES
    }

    fn peek_word(&self, addr: u32) -> Result<u32, Exception> {
        match addr & !0x3 {
            REG_TS_ARM => Ok(self.ts_arm),
            REG_TS_LOCK => Ok(self.ts_lock_fw | if self.hw_lock { 1 } else { 0 }),
            REG_TS_FREQ_ERR => Ok(self.ts_freq_err),
            REG_TS_WINDOW => Ok(self.ts_window),
            REG_TS_MODE => Ok(self.ts_mode),
            REG_CAPTURED_TS => Ok(self.captured_ts),
            REG_NCO_TX_OFFSET => Ok(self.nco_tx_offset),
            REG_NCO_TX_TS => Ok(self.nco_tx_ts),
            _ => Ok(0),
        }
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        // Pure read — same as peek (no read-side effects). The HW
        // OLT-lock level is OR-ed into 0x01000304 bit0.
        self.peek_word(addr)
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        match addr & !0x3 {
            REG_TS_ARM => self.ts_arm = val,
            REG_TS_LOCK => {
                // Firmware arm write. Store the firmware-written bits; the
                // HW OLT-lock level is a separate input OR-ed in on read and
                // is NOT created by this write (DO-NOT-FAKE: lock must come
                // from the OLT model, never from the firmware arming the
                // register).
                self.ts_lock_fw = val;
            }
            REG_TS_FREQ_ERR => self.ts_freq_err = val,
            REG_TS_WINDOW => self.ts_window = val,
            REG_TS_MODE => self.ts_mode = val,
            REG_CAPTURED_TS => self.captured_ts = val,
            REG_NCO_TX_OFFSET => self.nco_tx_offset = val,
            REG_NCO_TX_TS => self.nco_tx_ts = val,
            _ => {}
        }
        Ok(())
    }

    fn tick(&mut self, _cpu_instructions: u64) {}

    fn reset_cold(&mut self) {
        self.ts_arm = 0;
        self.ts_lock_fw = 0;
        self.ts_freq_err = 0;
        self.ts_window = 0;
        self.ts_mode = 0;
        self.captured_ts = 0;
        self.nco_tx_offset = 0;
        self.nco_tx_ts = 0;
        self.hw_lock = false;
        self.apply_silicon_power_on();
    }

    fn reset_warm(&mut self) {
        self.reset_cold();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::empty(self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn claims_only_the_eight_tssync_registers() {
        let m = MpcpTsSync::new();
        for a in &CLAIMED {
            assert!(m.claims(*a));
            assert!(m.claims(*a | 0x3)); // byte/half sub-offset
        }
        // 0x01000320/0x01000324 belong to mpcp.rs, not here.
        assert!(!m.claims(0x0100_0320));
        assert!(!m.claims(0x0100_0324));
        // Adjacent unrelated addresses.
        assert!(!m.claims(0x0100_0308));
        assert!(!m.claims(0x0100_0D84));
        assert!(!m.claims(0x0100_0D90));
        assert!(!m.claims(0x0100_00B0));
    }

    #[test]
    fn olt_disabled_reads_match_silicon_seed() {
        // With no OLT driving, reads must match the residual store seed:
        // 0x010000B4 = 0xFFFFFFFF, 0x01000D8C = 0x00C93C2E, others 0.
        let mut m = MpcpTsSync::new();
        assert_eq!(m.read_word(REG_CAPTURED_TS).unwrap(), 0xFFFF_FFFF);
        assert_eq!(m.read_word(REG_NCO_TX_TS).unwrap(), 0x00C9_3C2E);
        assert_eq!(m.read_word(REG_TS_ARM).unwrap(), 0);
        assert_eq!(m.read_word(REG_TS_LOCK).unwrap(), 0);
        assert_eq!(m.read_word(REG_TS_FREQ_ERR).unwrap(), 0);
        assert_eq!(m.read_word(REG_TS_WINDOW).unwrap(), 0);
        assert_eq!(m.read_word(REG_TS_MODE).unwrap(), 0);
        assert_eq!(m.read_word(REG_NCO_TX_OFFSET).unwrap(), 0);
    }

    #[test]
    fn firmware_written_registers_round_trip() {
        let mut m = MpcpTsSync::new();
        m.write_word(REG_TS_ARM, 0x1).unwrap();
        assert_eq!(m.read_word(REG_TS_ARM).unwrap(), 0x1);
        m.write_word(REG_TS_FREQ_ERR, 0xDEAD_BEEF).unwrap();
        assert_eq!(m.read_word(REG_TS_FREQ_ERR).unwrap(), 0xDEAD_BEEF);
        m.write_word(REG_TS_WINDOW, 0x1C0).unwrap();
        assert_eq!(m.read_word(REG_TS_WINDOW).unwrap(), 0x1C0);
        m.write_word(REG_TS_MODE, 0xC).unwrap();
        assert_eq!(m.read_word(REG_TS_MODE).unwrap(), 0xC);
        m.write_word(REG_NCO_TX_OFFSET, 0x1234).unwrap();
        assert_eq!(m.read_word(REG_NCO_TX_OFFSET).unwrap(), 0x1234);
    }

    #[test]
    fn firmware_arming_lock_does_not_create_hw_lock() {
        // DO-NOT-FAKE: firmware writing 1 to 0x01000304 must NOT make the
        // HW OLT-lock bit read back unless the OLT model drove it.
        let mut m = MpcpTsSync::new();
        m.write_word(REG_TS_LOCK, 0x1).unwrap();
        // Firmware-written bit reads back (it round-trips), but that is the
        // firmware's own arm bit. The HW lock input is independent.
        assert_eq!(m.read_word(REG_TS_LOCK).unwrap() & 1, 1);
        // Clear the firmware bit; with no HW lock the register reads 0.
        m.write_word(REG_TS_LOCK, 0x0).unwrap();
        assert_eq!(m.read_word(REG_TS_LOCK).unwrap(), 0);
        assert!(!m.hw_lock());
    }

    #[test]
    fn hw_lock_level_sets_bit0_on_read() {
        let mut m = MpcpTsSync::new();
        m.write_word(REG_TS_LOCK, 0x0).unwrap();
        m.set_hw_lock(true);
        assert_eq!(m.read_word(REG_TS_LOCK).unwrap() & 1, 1);
        m.set_hw_lock(false);
        assert_eq!(m.read_word(REG_TS_LOCK).unwrap() & 1, 0);
    }

    #[test]
    fn olt_drives_captured_and_nco_timestamps() {
        let mut m = MpcpTsSync::new();
        m.drive_captured_ts(0x0001_0000);
        m.drive_nco_tx_ts(0x0002_0000);
        assert_eq!(m.read_word(REG_CAPTURED_TS).unwrap(), 0x0001_0000);
        assert_eq!(m.read_word(REG_NCO_TX_TS).unwrap(), 0x0002_0000);
    }
}
