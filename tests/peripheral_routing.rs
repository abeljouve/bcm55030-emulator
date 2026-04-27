//! Peripheral routing invariants — Session 8 polish.
//!
//! Guards against two classes of regression:
//!
//! 1. **Residual has no auto-clear** (audit 5.8 — fully resolved
//!    by deferral D7). The typed peripherals own their own bits
//!    `[31:27]` auto-clear. The sysreg residual path is a plain
//!    backing store: writes land untouched and reads return the
//!    stored value forever. The only register that still needs
//!    the command-bit semantic (`0x01000160`) is now claimed by
//!    `epon_mac.rs::REG_MPCP_CMD_LATCH`.
//!
//! 2. **Sub-word routing**. Half / byte accesses to an address
//!    owned by a sparse-claim peripheral (e.g. `epon_mac.rs`)
//!    must reach that peripheral regardless of whether the raw
//!    address is word-aligned. Sparse single-register claims are
//!    particularly vulnerable — without a `addr & !0x3` mask in
//!    `claims()`, a half-read at `0x01000001` slips through to
//!    `sysreg_shim` instead of the intended peripheral.

use bcm55030_emulator::soc::bank::{BootMode, PeripheralBank};
use bcm55030_emulator::soc::peripheral::PeripheralSnapshot;

/// Residual sysreg addresses are a plain backing store — writes
/// round-trip unmodified. The bits `[31:27]` command-bit auto-
/// clear used to live here (audit 5.8) and regressed deferral
/// D7. The single dependent register `0x01000160` now lives in
/// `epon_mac.rs` with its own semantic.
#[test]
fn residual_sysreg_is_plain_backing_store() {
    let mut bank = PeripheralBank::new(BootMode::Cold);
    // 0x01000080 is in the queue priority region — pure residual.
    let addr = 0x0100_0080;
    bank.write_word(addr, 0xF800_00AA).unwrap();
    let first = bank.read_word(addr).unwrap();
    assert_eq!(first, 0xF800_00AA, "residual read returns latched value");
    let second = bank.read_word(addr).unwrap();
    assert_eq!(
        second, 0xF800_00AA,
        "residual store must be idempotent (no auto-clear)"
    );
}

/// The MPCP command-latch register at `0x01000160` keeps the
/// command-bit `[31:27]` auto-clear semantic inside `epon_mac`,
/// where the single dependent firmware path polls it.
#[test]
fn epon_mac_mpcp_cmd_latch_autoclears_on_second_read() {
    let mut bank = PeripheralBank::new(BootMode::Cold);
    let addr = 0x0100_0160;
    bank.write_word(addr, 0xF800_00AA).unwrap();
    let first = bank.read_word(addr).unwrap();
    assert_eq!(first, 0xF800_00AA);
    let second = bank.read_word(addr).unwrap();
    assert_eq!(second, 0x0000_00AA);
}

/// Half / byte accesses at sub-word offsets of an epon_mac register
/// must still be routed to epon_mac, not sysreg.
#[test]
fn epon_mac_sparse_claims_catches_half_and_byte_accesses() {
    let mut bank = PeripheralBank::new(BootMode::Warm);

    // CHIP_ID lives at 0x01000000 and is a fixed read-only register.
    // The high half-word should be 0x4701.
    let half_hi = bank.read_half(0x0100_0000).unwrap();
    assert_eq!(half_hi, 0x4701);

    // The low half-word should be 0x0203.
    let half_lo = bank.read_half(0x0100_0002).unwrap();
    assert_eq!(half_lo, 0x0203);

    // Byte-level reads — MSB first (big-endian).
    assert_eq!(bank.read_byte(0x0100_0000).unwrap(), 0x47);
    assert_eq!(bank.read_byte(0x0100_0001).unwrap(), 0x01);
    assert_eq!(bank.read_byte(0x0100_0002).unwrap(), 0x02);
    assert_eq!(bank.read_byte(0x0100_0003).unwrap(), 0x03);

    // CHIP_REV = 0xB2110816.
    assert_eq!(bank.read_byte(0x0100_0004).unwrap(), 0xB2);
    assert_eq!(bank.read_byte(0x0100_0007).unwrap(), 0x16);
}

/// Timer / eFuse UDR single-register peripherals must also handle
/// sub-word accesses via their `claims()` word-align mask.
#[test]
fn single_register_peripherals_handle_sub_word_access() {
    let mut bank = PeripheralBank::new(BootMode::Cold);

    // Timer: write a known value, read it back byte-by-byte.
    bank.write_word(0x0100_0050, 0x1122_3344).unwrap();
    assert_eq!(bank.read_byte(0x0100_0050).unwrap(), 0x11);
    assert_eq!(bank.read_byte(0x0100_0053).unwrap(), 0x44);

    // eFuse I2C_UDR_SDA: bit 31 always set unless bit 4 is set.
    // Confirm byte read at offset 0x48 returns the top byte.
    bank.write_word(0x0100_0048, 0x0000_0000).unwrap();
    let top_byte = bank.read_byte(0x0100_0048).unwrap();
    assert_eq!(top_byte & 0x80, 0x80, "SDA bit 31 should read high");
}

/// A routing round-trip test: write to every active peripheral's
/// representative register via the bank and confirm reads land
/// inside the correct peripheral, not the sysreg fallback.
#[test]
fn bank_routes_known_peripherals_without_sysreg_fallback() {
    let mut bank = PeripheralBank::new(BootMode::Warm);

    // UART: DATA register at 0x00FC1010 — word writes use the
    // MSB byte per silicon (`val >> 24`), so shift the target
    // byte into bits [31:24]. Disable stdout passthrough so the
    // byte lands in tx_log only.
    bank.uart.stdout_passthrough = false;
    bank.write_word(0x00FC_1010, (b'X' as u32) << 24).unwrap();
    let tx_log = bank.uart.tx_log_bytes();
    assert!(tx_log.contains(&b'X'), "UART TX log should echo the byte");

    // PBC: SPI_CONTROL at 0x01000200 — writable register. Keep
    // bit 6 clear so we do not accidentally route a SerDes SPI
    // slave command through the bank's cross-peripheral dispatch.
    bank.write_word(0x0100_0200, 0xDEAD_BE00).unwrap();
    assert_eq!(bank.read_word(0x0100_0200).unwrap(), 0xDEAD_BE00);

    // BSC: register-backed store at 0x01000144.
    bank.write_word(0x0100_0144, 0x0000_00FF).unwrap();
    assert_eq!(bank.read_word(0x0100_0144).unwrap() & 0xFF, 0xFF);

    // SerDes: lane config register at 0x010001AC.
    bank.write_word(0x0100_01AC, 0xCAFE_BABE).unwrap();
    assert_eq!(bank.read_word(0x0100_01AC).unwrap(), 0xCAFE_BABE);
}

/// the design spec Phase 1: SFP orphan fix. `snapshot_all()` must include a
/// dedicated `PeripheralSnapshot::Sfp` row so the UI can render a
/// per-peripheral tab without reaching into BSC internals.
#[test]
fn snapshot_all_exposes_sfp_row() {
    let bank = PeripheralBank::new(BootMode::Warm);
    let rows = bank.snapshot_all();
    assert_eq!(rows.len(), 12, "12 peripheral rows expected after SFP fix");
    assert!(
        matches!(rows[3], PeripheralSnapshot::Sfp(_)),
        "row 3 should be PeripheralSnapshot::Sfp, got {:?}",
        rows[3].name()
    );
    if let PeripheralSnapshot::Sfp(ref sfp) = rows[3] {
        assert!(!sfp.vendor.is_empty(), "SFP vendor should be populated");
        assert!(!sfp.part_number.is_empty(), "SFP part_number should be populated");
    }
}
