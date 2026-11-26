//! A frame arriving on the subscriber port, followed to where it stops.
//!
//! The port's own gates have unit tests next to them. What this file
//! asks is the question those cannot: once a frame is past them, does it
//! reach the firmware? That answer is not the port's — it is the
//! classifier's, and the point of the test is that the model says "no"
//! for a reason it can name, and says "yes" the moment a rule names a
//! queue.
//!
//! The ingress reads instance 0 table 1, the pair a rule naming a CPU
//! queue was measured to work in. With no such rule there, the first
//! test below is what silicon measured: frames go in, nothing comes
//! out, and every one of them is counted. The second is the same port
//! once a rule names the queue the firmware polls.
//!
//! ⚠ One thing this file must not be read as saying. A frame that stops
//! here stops because **no verdict named a queue**, and against the rule
//! set a running device carries that verdict is *undecidable* — the
//! engine refuses operators it has not established rather than guessing
//! them. So the model declines to route the frame; it does not claim the
//! hardware would drop it. Those are different statements, and only the
//! first one is being made.

use bcm55030_emulator::soc::bank::PeripheralBank;
use bcm55030_emulator::soc::lue::{
    Action, ActionPayload, Clause, ClassifierBinding, Entry, Field, Link, Op, CMD_GO, CMD_WRITE,
};
use bcm55030_emulator::soc::peripheral::PeripheralEvent;
use bcm55030_emulator::soc::uni::{UniEvent, UniRxSignal};
use bcm55030_emulator::soc::bank::BootMode;

/// The queue the firmware polls for frames it terminates itself.
const CPU_QUEUE: u16 = 0x10;

fn ethertype_frame(et: u16) -> Vec<u8> {
    let mut f = vec![0u8; 64];
    f[12] = (et >> 8) as u8;
    f[13] = et as u8;
    f
}

/// A rule that matches one EtherType and names a queue for it, written
/// through the port the way software writes it.
fn program_queue_rule(bank: &mut PeripheralBank, instance: usize, table: u8, index: u16, et: u16, queue: u16) {
    let entry = Entry::Clause {
        clause: Clause {
            field: Field::EtherType,
            op: Op::Eq,
            shift: 0x30,
            anchor: 0,
            operand: et as u64,
        },
        link: Link::Terminal(Action {
            priority: 1,
            payload: ActionPayload::Selected { sel: 2, sub: 0, field: queue, wide: true },
        }),
    };
    let port = &mut bank.lue.inst[instance];
    for (i, w) in entry.encode().to_regs().iter().enumerate() {
        port.write_data(i, *w);
    }
    port.write_cmd(CMD_GO | CMD_WRITE | ((table as u32) << 12) | index as u32);
}

fn bank_with_live_port() -> PeripheralBank {
    let mut bank = PeripheralBank::new(BootMode::Cold);
    bank.inject_event(&PeripheralEvent::Uni(UniEvent::SetArmed(true)));
    bank.inject_event(&PeripheralEvent::Uni(UniEvent::SetRxSignal(
        UniRxSignal::BaseX { autoneg: true },
    )));
    bank
}

fn offer(bank: &mut PeripheralBank, frame: Vec<u8>) {
    bank.inject_event(&PeripheralEvent::Uni(UniEvent::ReceiveFrame(frame)));
}

/// What silicon measured, reproduced: the port takes the frames and
/// the firmware never sees one, because the instance this ingress is
/// bound to names no queue.
#[test]
fn with_no_rule_naming_a_queue_the_frames_stop_at_the_classifier() {
    let mut bank = bank_with_live_port();
    bank.use_classifier = true;
    let before = bank.olt.total_pending_count();

    for _ in 0..5 {
        offer(&mut bank, ethertype_frame(0x8809));
    }
    bank.tick(1);

    assert_eq!(
        bank.olt.total_pending_count(),
        before,
        "no rule names a queue, so nothing may reach the mailbox"
    );
    // And the frames are accounted for rather than lost quietly: the
    // port took all five, and the classifier is where they stopped.
    let snap = bank.uni.counters;
    assert_eq!(snap.rx_offered, 5);
    assert_eq!(snap.rx_accepted, 5);
    assert_eq!(snap.rx_dropped_unarmed, 0);
    assert_eq!(snap.rx_dropped_no_signal, 0);
    assert_eq!(bank.uni.rx_queued(), 0, "the datapath took them from the port");
}

/// The positive control. Give the bound instance a rule that names the
/// queue the firmware polls, and the same frame arrives — which is what
/// says the first test measured the rule set and not a severed path.
#[test]
fn a_rule_naming_the_cpu_queue_carries_the_frame_to_the_mailbox() {
    let mut bank = bank_with_live_port();
    bank.use_classifier = true;
    let ClassifierBinding { instance, table } = bank.uni_classifier_binding;
    program_queue_rule(&mut bank, instance, table, 0x20, 0x8809, CPU_QUEUE);

    let before = bank.olt.total_pending_count();
    offer(&mut bank, ethertype_frame(0x8809));
    bank.tick(1);

    assert_eq!(
        bank.olt.total_pending_count(),
        before + 1,
        "a rule naming the queue is what carries the frame"
    );
    assert_eq!(bank.uni.counters.rx_accepted, 1);
}

/// The rule belongs to one instance, and binding the ingress to the
/// other one is not a detail: it decides whether the frame moves.
#[test]
fn the_binding_decides_which_rule_set_the_ingress_meets() {
    let mut bank = bank_with_live_port();
    bank.use_classifier = true;
    // Put the rule in one instance and read the ingress with the other.
    let ClassifierBinding { instance: bound, table } = bank.uni_classifier_binding;
    program_queue_rule(&mut bank, 1 - bound, table, 0x20, 0x8809, CPU_QUEUE);

    let before = bank.olt.total_pending_count();
    offer(&mut bank, ethertype_frame(0x8809));
    bank.tick(1);
    assert_eq!(
        bank.olt.total_pending_count(),
        before,
        "the rule is in the instance this ingress is not bound to"
    );

    // Move the binding and the same frame lands.
    bank.uni_classifier_binding = ClassifierBinding { instance: 1 - bound, table };
    offer(&mut bank, ethertype_frame(0x8809));
    bank.tick(1);
    assert_eq!(bank.olt.total_pending_count(), before + 1);
}

/// The table is part of the binding too: the same rule one table over is
/// not one this ingress meets.
#[test]
fn a_rule_in_another_table_of_the_bound_instance_is_not_seen() {
    let mut bank = bank_with_live_port();
    bank.use_classifier = true;
    let ClassifierBinding { instance, table } = bank.uni_classifier_binding;
    assert_eq!((instance, table), (0, 1), "the pair measured to carry a CPU rule");
    program_queue_rule(&mut bank, instance, 8, 0x20, 0x8809, CPU_QUEUE);

    let before = bank.olt.total_pending_count();
    offer(&mut bank, ethertype_frame(0x8809));
    bank.tick(1);
    assert_eq!(bank.olt.total_pending_count(), before);
}

/// With the classifier gate shut there is no rule set to consult, and
/// the frame is dropped rather than routed by the EtherType fallback —
/// that fallback is the fibre's history, and this port has none.
#[test]
fn a_shut_classifier_gate_drops_the_frame_instead_of_guessing() {
    let mut bank = bank_with_live_port();
    assert!(!bank.use_classifier, "the gate is shut by default");
    let before = bank.olt.total_pending_count();

    offer(&mut bank, ethertype_frame(0x8809));
    bank.tick(1);

    assert_eq!(
        bank.olt.total_pending_count(),
        before,
        "with no rule set, there is nothing to route by"
    );
    assert_eq!(bank.uni.counters.rx_accepted, 1, "the port still took it");
}

/// A frame offered to a port that is not up never becomes the
/// datapath's problem.
#[test]
fn an_unarmed_port_hands_the_datapath_nothing() {
    let mut bank = PeripheralBank::new(BootMode::Cold);
    bank.use_classifier = true;
    let ClassifierBinding { instance, table } = bank.uni_classifier_binding;
    program_queue_rule(&mut bank, instance, table, 0x20, 0x8809, CPU_QUEUE);

    let before = bank.olt.total_pending_count();
    offer(&mut bank, ethertype_frame(0x8809));
    bank.tick(1);

    assert_eq!(bank.olt.total_pending_count(), before);
    assert_eq!(bank.uni.counters.rx_dropped_unarmed, 1);
    assert_eq!(bank.uni.counters.rx_accepted, 0);
}
