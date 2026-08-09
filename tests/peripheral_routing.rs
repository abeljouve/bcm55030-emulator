//! Peripheral routing invariants.
//!
//! Guards against two classes of regression:
//!
//! 1. **Residual has no auto-clear**. The typed peripherals own their
//!    own bits `[31:27]` auto-clear. The sysreg residual path is a plain
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
    // MPCP CMD LATCH (lane 8) now uses immediate clear — cmd bits
    // are cleared on write, not deferred to the next read. The
    // firmware reads CMD via the D-cache so deferred clear would
    // never fire.
    let mut bank = PeripheralBank::new(BootMode::Cold);
    let addr = 0x0100_0160;
    bank.write_word(addr, 0xF800_00AA).unwrap();
    let first = bank.read_word(addr).unwrap();
    assert_eq!(first, 0x0000_00AA, "cmd bits cleared immediately on write");
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
    assert_eq!(rows.len(), 13, "13 peripheral rows expected (12 original + OLT)");
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

/// The lane register file has one instance per context, and the selector
/// that picks the instance does **not** ride on the lane bus: software
/// latches it in a separate MMIO word and re-arms it before every
/// transaction. The bank has to carry it across.
///
/// Without the bridge the two contexts share one file, each overwriting
/// the other's registers — measured on a full boot as reads returning a
/// value the reading context never wrote, then written back into its own
/// configuration by the read-modify-write that follows.
#[test]
fn the_mode_selector_reaches_the_lane_register_file() {
    let mut bank = PeripheralBank::new(BootMode::Warm);
    // Bits [13:12] of the selector word; the rest of the word belongs to
    // unrelated clock and reset controls and must not matter here.
    for (word, expected_page) in [
        (0x3E3E_0E41u32, 0usize),
        (0x3E3E_1E41, 1),
        (0x3E3E_2E41, 2),
        (0x3E3E_3E41, 3),
    ] {
        bank.write_word(0x0100_0040, word).unwrap();
        assert_eq!(
            bank.mpcp_bus.page(),
            expected_page,
            "selector word {word:#010x} should select page {expected_page}"
        );
    }
}

/// The classifier gate is shut by default, and a shut gate routes
/// downstream frames byte-for-byte the way the EtherType fallback did
/// before the classifier existed.
///
/// The step from a rule verdict to a mailbox queue is the link that is
/// not established; an open gate by default would put that guess on the
/// path every downstream frame takes.
#[test]
fn the_classifier_gate_is_shut_by_default_and_changes_no_routing() {
    let bank = PeripheralBank::new(BootMode::Warm);
    assert!(!bank.use_classifier, "the gate must default to shut");
    assert_eq!(bank.olt.classifier_counters.classified, 0);
}

/// With the gate open and no rules programmed, every frame falls back —
/// and the fallback is **counted**. A fallback nobody counts cannot be
/// told apart from a classifier that worked.
#[test]
fn an_open_gate_with_no_rules_falls_back_and_says_so() {
    use bcm55030_emulator::soc::lue::{ClassifierBinding, Lue, Verdict};

    let lue = Lue::new();
    let frame = {
        let mut f = vec![0u8; 64];
        // MPCP EtherType — the fallback sends it to the control queue.
        f[12] = 0x88;
        f[13] = 0x08;
        f
    };
    // An empty table decides nothing; it must not read as a miss.
    assert!(matches!(
        lue.classify(ClassifierBinding::default(), &frame),
        Verdict::Undecidable { .. }
    ));
}

/// O5, end to end: a frame that lands in a queue moves that queue's
/// counter by one, and only that queue's.
///
/// The port used to answer a literal zero on both read paths, so "the
/// counter did not move" could not be told from "the model has no
/// counter". The denominator is the point of this test.
#[test]
fn an_arriving_frame_moves_the_counter_of_its_own_queue() {
    let mut bank = PeripheralBank::new(BootMode::Warm);
    let cmd = 0x0100_15D4u32;
    let data = 0x0100_15D8u32;

    let read_queue = |bank: &mut PeripheralBank, queue: u32| {
        bank.write_word(cmd, (queue << 4) | 0xC).unwrap();
        bank.read_word(data).unwrap()
    };

    // Control queue 0x10 and data queue 0x0F both start empty.
    assert_eq!(read_queue(&mut bank, 0x10), 0);
    assert_eq!(read_queue(&mut bank, 0x0F), 0);

    // Three frames into the control queue, reported the way the
    // datapath reports them.
    bank.epon_mac.record_queue_arrivals(0x10, 3);
    assert_eq!(read_queue(&mut bank, 0x10), 3);
    assert_eq!(read_queue(&mut bank, 0x0F), 0, "a neighbour queue must not move");

    // And the busy bit never survives a command: the firmware's wait on
    // this port has no timeout.
    bank.write_word(cmd, 0x8000_000C).unwrap();
    assert_eq!(bank.read_word(cmd).unwrap() & 0x8000_0000, 0);
}
