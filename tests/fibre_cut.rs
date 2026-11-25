//! A cut fibre, as this SoC sees it.
//!
//! The peer's side of a cut is tested in the `epon-olt` crate. What is tested
//! here is what the bank derives from the light being gone: the 10G link-loss
//! latch at `0x01000E04` bit 6 behaves as on a unit without fibre for as long
//! as the cut lasts, and the timestamp lock at `0x01000304` bit 0 falls with
//! the light.

use bcm55030_emulator::soc::bank::{BootMode, PeripheralBank};
use epon_olt::clock::WireDuration;

const REG_HW_STATE_STATUS: u32 = 0x0100_0E04;
const LINK_LOSS: u32 = 0x40;
const REG_TS_LOCK: u32 = 0x0100_0304;
const HW_LOCK: u32 = 0x01;

/// A bank whose line is up and whose peer has broadcast a GATE.
fn receiving_bank() -> PeripheralBank {
    let mut bank = PeripheralBank::new(BootMode::Cold);
    bank.olt.link_up_delay = 0;
    bank.olt.set_link_up(true);
    for _ in 0..4 {
        bank.tick(64);
    }
    assert!(bank.olt.has_broadcast_gate());
    bank
}

fn link_loss(bank: &mut PeripheralBank) -> bool {
    bank.read_word(REG_HW_STATE_STATUS).unwrap() & LINK_LOSS != 0
}

/// Clear the latch the way software does, then let one tick pass.
fn clear_and_tick(bank: &mut PeripheralBank) {
    bank.write_word(REG_HW_STATE_STATUS, LINK_LOSS).unwrap();
    bank.tick(64);
}

#[test]
fn a_cut_latches_link_loss_until_the_light_is_back() {
    let mut bank = receiving_bank();
    clear_and_tick(&mut bank);
    assert!(!link_loss(&mut bank), "a receiving unit's clear holds");
    assert!(bank.olt.downstream_lit());

    bank.olt.cut_fibre(WireDuration::from_ms(2));
    bank.tick(64);
    assert!(!bank.olt.downstream_lit());
    assert!(link_loss(&mut bank), "the light is gone");
    // As on a unit without fibre: it comes back after every clear.
    for _ in 0..5 {
        clear_and_tick(&mut bank);
        assert!(link_loss(&mut bank));
    }

    // A tick is under 25 µs of link time at the default scale: a few
    // hundred take the line past the end of the cut.
    for _ in 0..400 {
        bank.tick(64);
    }
    assert!(bank.olt.downstream_lit());
    // The latch keeps what it saw during the cut until software clears it;
    // after that the clear holds again.
    assert!(link_loss(&mut bank));
    clear_and_tick(&mut bank);
    assert!(!link_loss(&mut bank));
}

#[test]
fn the_timestamp_lock_falls_with_the_light() {
    let mut bank = receiving_bank();
    assert_eq!(bank.read_word(REG_TS_LOCK).unwrap() & HW_LOCK, HW_LOCK);
    bank.olt.cut_fibre(WireDuration::from_ms(2));
    bank.tick(64);
    assert_eq!(bank.read_word(REG_TS_LOCK).unwrap() & HW_LOCK, 0);
    for _ in 0..400 {
        bank.tick(64);
    }
    assert_eq!(bank.read_word(REG_TS_LOCK).unwrap() & HW_LOCK, HW_LOCK);
}

#[test]
fn nothing_crosses_a_cut_fibre() {
    let mut bank = receiving_bank();
    let (down_before, _) = bank.olt.lost_in_the_dark();
    bank.olt.cut_fibre(WireDuration::from_ms(1500));
    // Past the next keepalive and the next discovery GATE (804 ms and 1 s of
    // link time), both sent into the cut, and short of its end.
    let pending = bank.olt.total_pending_count();
    for _ in 0..60_000 {
        bank.tick(64);
    }
    let (down_after, _) = bank.olt.lost_in_the_dark();
    assert!(down_after > down_before, "the peer kept sending into the cut");
    assert!(bank.olt.total_pending_count() <= pending, "and nothing reached the mailbox");
}
