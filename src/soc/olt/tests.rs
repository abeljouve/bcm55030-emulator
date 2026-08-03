//! What this module is responsible for: the mailbox, the capture of what the
//! firmware transmits, and the bridge between bank ticks and link time.
//!
//! The protocol itself — discovery, registration, the lifetime of a
//! registration, the cadences — belongs to the peer and is tested in the
//! `epon-olt` crate, against a clock no CPU has to advance.

use super::*;

use mailbox::{ALIGN_PAD, CMD_STATUS_BASE, DATA_BASE, STRIDE};
use types::{EtherType, MIN_FRAME_LEN};

const ONU_MAC: MacAddr = MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
/// Firmware submits on channel block 1, not block 0.
const TX_BLOCK: u32 = 1;

fn live_olt() -> Olt {
    let mut olt = Olt::new();
    olt.link_up_delay = 0;
    olt.set_link_up(true);
    olt
}

/// A peer on a dark line: it sends nothing of its own, so what reaches the
/// mailbox is only what a test put there.
fn quiet_olt() -> Olt {
    let mut olt = Olt::new();
    olt.link_up_delay = 0;
    olt
}

/// Advance `ticks` bank ticks and load whatever arrived, the way the bank
/// does every tick.
fn run(olt: &mut Olt, ticks: u64) {
    for _ in 0..ticks {
        olt.tick(0);
        olt.load_frames_into_mailbox(None);
    }
}

/// Advance far enough for a frame to have crossed the fibre.
fn cross(olt: &mut Olt) {
    let crossing = FibreConfig::downstream().propagation.as_ps();
    let ticks = crossing / (TICK_PS * olt.config.time_scale.max(1)) + 2;
    run(olt, ticks);
}

/// A registration request shaped the way an ONU sends one.
fn onu_request() -> mpcp::RegisterReqBody {
    mpcp::RegisterReqBody {
        flag: mpcp::RegisterReqFlag::Register,
        pending_grants: 16,
        discovery_information: 0x0011,
        laser_on: 32,
        laser_off: 32,
    }
}

fn onu_header(opcode: mpcp::Opcode) -> mpcp::Header {
    mpcp::Header {
        dst: MacAddr::MPCP_MULTICAST,
        src: ONU_MAC,
        opcode,
        timestamp: 0,
    }
}

/// Split a frame the way the firmware pushes it: alignment padding, the
/// frame, then zero fill to the expected word count.
fn submission_words(frame: &[u8]) -> Vec<u32> {
    let mut raw = vec![0u8; ALIGN_PAD];
    raw.extend_from_slice(frame);
    raw.resize(mailbox::word_count(frame.len()) * 4, 0);
    raw.chunks(4)
        .map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

/// Drive a frame through the transmit port exactly as the firmware does.
fn submit(olt: &mut Olt, frame: &[u8]) {
    let cmd_addr = CMD_STATUS_BASE + TX_BLOCK * STRIDE;
    let data_addr = DATA_BASE + TX_BLOCK * STRIDE;
    olt.observe_write(
        cmd_addr,
        mailbox::Command::Write { channel: 0, len: frame.len(), mpcp: true }.encode(),
    );
    for word in submission_words(frame) {
        olt.observe_write(data_addr, word);
    }
}

fn frame_with_ethertype(ethertype: EtherType) -> Vec<u8> {
    types::FrameWriter::ethernet(MacAddr::ZERO, ONU_MAC, ethertype).pad_to(MIN_FRAME_LEN)
}

#[test]
fn a_fresh_model_is_idle_with_the_line_armed() {
    let olt = Olt::new();
    assert_eq!(olt.mpcp_state(), OltMpcpState::Idle);
    assert!(!olt.link_up, "the link rises only after the delay");
    assert_eq!(olt.link_up_delay, LINK_UP_DELAY_TICKS);
}

// ── The two clocks ──────────────────────────────────────────────────

#[test]
fn a_tick_is_worth_a_fixed_amount_of_link_time() {
    let mut olt = live_olt();
    run(&mut olt, 1000);
    assert_eq!(olt.ticks_elapsed(), 1000);
    assert_eq!(olt.wire_now().as_ps(), 1000 * TICK_PS * DEFAULT_TIME_SCALE);
}

/// The scale multiplies how much link time a tick buys, so a timestamp
/// difference still equals the interval that produced it. Scaling the
/// intervals instead would shrink every duration measured between two frames.
#[test]
fn the_timestamp_advances_at_the_wires_rate_whatever_the_scale() {
    let elapsed_quanta = |scale: u64, ticks: u64| {
        let mut olt = live_olt();
        olt.config.time_scale = scale;
        run(&mut olt, ticks);
        (olt.mpcp_timestamp, olt.wire_now())
    };
    // Same link time, different numbers of ticks: the same timestamp.
    let (fast, fast_wire) = elapsed_quanta(32, 1000);
    let (slow, slow_wire) = elapsed_quanta(1, 32_000);
    assert_eq!(fast_wire, slow_wire);
    assert_eq!(fast, slow);
    // And it is the count the wire clock says, not a per-tick accumulation.
    assert_eq!(fast, fast_wire.mpcp_timestamp());
}

// ── Uplink capture ──────────────────────────────────────────────────

#[test]
fn a_submission_is_reassembled_into_the_frame_that_was_sent() {
    let mut olt = live_olt();
    assert!(!olt.real_tx_seen());

    let sent = mpcp::register_req(onu_header(mpcp::Opcode::RegisterReq), onu_request());
    submit(&mut olt, &sent);
    assert!(olt.real_tx_seen());
    assert_eq!(olt.tx_dropped(), 0);

    // It reaches the peer once it has crossed, not before.
    assert!(olt.tx_log().is_empty(), "still on the line");
    cross(&mut olt);
    let captured = &olt.tx_log().back().expect("logged").data;
    assert_eq!(captured, &sent, "the frame must survive the round trip");
    assert_eq!(olt.get_onu_mac(), ONU_MAC.octets());
}

#[test]
fn observing_a_transmit_descriptor_does_not_consume_it() {
    let mut olt = live_olt();
    let cmd = mailbox::Command::Write { channel: 0, len: 64, mpcp: true }.encode();
    assert!(!olt.write_cmd(CMD_STATUS_BASE, cmd));
}

#[test]
fn an_abandoned_submission_is_counted_not_merged() {
    let mut olt = live_olt();
    let frame = mpcp::register_req(onu_header(mpcp::Opcode::RegisterReq), onu_request());

    olt.observe_write(
        CMD_STATUS_BASE + TX_BLOCK * STRIDE,
        mailbox::Command::Write { channel: 0, len: frame.len(), mpcp: true }.encode(),
    );
    olt.observe_write(DATA_BASE + TX_BLOCK * STRIDE, 0);
    assert!(!olt.real_tx_seen());

    submit(&mut olt, &frame);
    cross(&mut olt);

    assert_eq!(olt.tx_dropped(), 1);
    assert_eq!(&olt.tx_log().back().expect("logged").data, &frame);
}

#[test]
fn a_submission_before_the_link_rises_is_still_captured() {
    // The peer is always present; only the link state gates traffic.
    let mut olt = Olt::new();
    submit(&mut olt, &mpcp::register_req(onu_header(mpcp::Opcode::RegisterReq), onu_request()));
    assert!(olt.real_tx_seen());
}

/// The first frame the firmware actually transmits rewinds whatever a
/// synthesized one reached, so the first real request meets the first real
/// answer.
#[test]
fn a_captured_frame_rewinds_a_synthetically_registered_state_machine() {
    let mut olt = live_olt();
    let req = mpcp::register_req(onu_header(mpcp::Opcode::RegisterReq), onu_request());
    // Where the synthesized path leaves the model: a handshake underway
    // without the firmware having transmitted anything.
    olt.handle_tx_frame(&req);
    olt.handle_tx_frame(&req);
    cross(&mut olt);
    assert_eq!(olt.mpcp_state(), OltMpcpState::Discovery);

    submit(&mut olt, &req);
    cross(&mut olt);
    assert_eq!(olt.mpcp_state(), OltMpcpState::Idle, "the handshake was rewound");
}

// ── Mailbox ─────────────────────────────────────────────────────────

#[test]
fn control_plane_frames_share_one_slot() {
    let mut olt = quiet_olt();
    for _ in 0..3 {
        olt.inject_raw_frame(frame_with_ethertype(EtherType::SlowProtocol));
    }
    olt.inject_raw_frame(frame_with_ethertype(EtherType::Mpcp));
    cross(&mut olt);

    let control = Slot::CONTROL.0;
    assert_eq!(olt.mailbox_pending.get(&control).map(|q| q.len()), Some(4));

    assert!(olt.write_cmd(CMD_STATUS_BASE, mailbox::Command::Read { slot: Slot::CONTROL }.encode()));
    assert!(!olt.mailbox_fifo.is_empty());
    assert_eq!(olt.mailbox_pending.get(&control).map(|q| q.len()), Some(3));
}

#[test]
fn the_bitmap_bit_tracks_its_queue() {
    let mut olt = quiet_olt();
    olt.inject_raw_frame(frame_with_ethertype(EtherType::SlowProtocol));
    olt.inject_raw_frame(frame_with_ethertype(EtherType::Mpcp));
    cross(&mut olt);

    let (index, bit) = Slot::CONTROL.bitmap_position();
    let read = mailbox::Command::Read { slot: Slot::CONTROL }.encode();
    assert_ne!(olt.mailbox_bitmap[index] & (1 << bit), 0);

    olt.write_cmd(CMD_STATUS_BASE, read);
    assert_ne!(olt.mailbox_bitmap[index] & (1 << bit), 0, "one frame is left");

    olt.write_cmd(CMD_STATUS_BASE, read);
    assert_eq!(olt.mailbox_bitmap[index] & (1 << bit), 0, "the queue drained");
}

#[test]
fn reading_an_empty_slot_is_still_intercepted() {
    let mut olt = quiet_olt();
    olt.inject_raw_frame(frame_with_ethertype(EtherType::Eapol));
    cross(&mut olt);

    assert!(olt.write_cmd(CMD_STATUS_BASE, mailbox::Command::Read { slot: Slot::CONTROL }.encode()));
    assert!(olt.mailbox_fifo.is_empty());
    assert_eq!(olt.read_cmd_status(CMD_STATUS_BASE), Some(0));
    assert_eq!(olt.mailbox_pending.get(&Slot::EAPOL.0).map(|q| q.len()), Some(1));
}

#[test]
fn a_frame_read_back_signals_data_ready_then_drains() {
    let mut olt = quiet_olt();
    olt.inject_raw_frame(frame_with_ethertype(EtherType::Mpcp));
    cross(&mut olt);
    olt.write_cmd(CMD_STATUS_BASE, mailbox::Command::Read { slot: Slot::CONTROL }.encode());

    assert_eq!(olt.read_cmd_status(CMD_STATUS_BASE), Some(mailbox::STATUS_DATA_READY));
    while olt.read_cmd_status(CMD_STATUS_BASE) == Some(mailbox::STATUS_DATA_READY) {
        olt.read_data(DATA_BASE);
    }
    assert!(olt.frame_consumed);
    assert_eq!(olt.read_data(DATA_BASE), Some(0));
}

/// Nothing draining the queue must cost frames, not build a backlog. A
/// downstream that stores everything delivers a burst of frames whose windows
/// closed long ago, and reports a link that never lost anything.
#[test]
fn an_undrained_queue_loses_frames_rather_than_hoarding_them() {
    let mut olt = live_olt();
    // Ten GATEs a second of link time, so the run does not have to be long.
    olt.config.gate_interval_ms = 100;
    run(&mut olt, 200_000);

    assert!(olt.total_pending_count() <= MAILBOX_DEPTH);
    assert!(olt.counters().frames_sent > MAILBOX_DEPTH as u64);
    assert!(olt.dropped_downstream() > 0, "an undrained downstream must lose frames");
}

#[test]
fn a_frame_without_a_full_header_is_ignored() {
    let mut olt = live_olt();
    olt.on_tx_frame(&[0x00; 10]);
    cross(&mut olt);
    assert!(olt.tx_log().is_empty());
}

// ── Reset ───────────────────────────────────────────────────────────

#[test]
fn cold_reset_preserves_config() {
    let mut olt = live_olt();
    olt.config.mac = MacAddr::new([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
    run(&mut olt, 5000);
    assert!(olt.counters().oam_keepalives_sent > 0);

    olt.reset_cold();

    assert_eq!(olt.config.mac.octets(), [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
    assert_eq!(olt.mpcp_state(), OltMpcpState::Idle);
    assert_eq!(olt.counters().oam_keepalives_sent, 0);
    assert_eq!(olt.wire_now(), WireInstant::ZERO);
}

/// The peer does not reboot because this SoC does: a live line stays live
/// and re-delivers its link-change edge.
#[test]
fn cold_reset_keeps_the_line_live_and_re_delivers_the_edge() {
    let mut olt = Olt::new();
    run(&mut olt, LINK_UP_DELAY_TICKS);
    assert!(olt.link_up);

    olt.reset_cold();

    assert!(!olt.link_up);
    assert_eq!(olt.link_up_delay, LINK_UP_DELAY_TICKS);
    run(&mut olt, LINK_UP_DELAY_TICKS);
    assert!(olt.link_up);
    assert!(olt.link_change_pending);
}

/// Enabling before a firmware load, which cold-resets every peripheral,
/// must not leave an enabled peer on a dead link.
#[test]
fn enabling_before_the_firmware_load_keeps_the_link() {
    let mut olt = Olt::new();
    olt.reset_cold();
    run(&mut olt, LINK_UP_DELAY_TICKS);
    assert!(olt.link_up);
}

/// An explicit link-down is an instruction, not an accident.
#[test]
fn cold_reset_does_not_resurrect_an_explicitly_downed_link() {
    let mut olt = Olt::new();
    olt.link_up_delay = 0;
    olt.set_link_up(false);

    olt.reset_cold();

    assert_eq!(olt.link_up_delay, 0);
    assert!(!olt.link_up);
}

#[test]
fn cold_reset_preserves_the_trace_flag() {
    let mut olt = Olt::new();
    olt.trace = true;
    olt.reset_cold();
    assert!(olt.trace);
}

/// The configured LLID must be the one actually granted, or setting it is a
/// no-op that reads back as if it had worked.
#[test]
fn the_configured_llid_is_the_one_granted() {
    let mut olt = live_olt();
    olt.set_assigned_llid(Llid(0x3C67));
    let req = mpcp::register_req(onu_header(mpcp::Opcode::RegisterReq), onu_request());
    for _ in 0..=epon_olt::peer::IGNORED_REGISTER_REQUESTS {
        olt.on_tx_frame(&req);
        cross(&mut olt);
    }
    // Long enough for the peer's turnaround plus the crossing back.
    run(&mut olt, 200);

    let frame = olt
        .rx_log()
        .iter()
        .rev()
        .find(|f| f.description == "REGISTER")
        .expect("a REGISTER was sent")
        .data
        .clone();
    match mpcp::Pdu::parse(&frame).expect("parses").body {
        mpcp::Body::Register { llid, flag, .. } => {
            assert_eq!(llid, Llid(0x3C67));
            assert_eq!(flag, mpcp::RegisterFlag::Ack.as_u8());
        }
        other => panic!("unexpected body {other:?}"),
    }
    assert_eq!(olt.assigned_llid(), 0x3C67);
    assert_eq!(olt.pending_llid_update, Some(0x3C67));
}

/// A cold reset keeps the configured LLID rather than falling back.
#[test]
fn cold_reset_keeps_the_configured_llid() {
    let mut olt = live_olt();
    olt.set_assigned_llid(Llid(0x3C67));
    olt.reset_cold();
    assert_eq!(olt.assigned_llid(), 0x3C67);
}
