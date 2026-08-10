//! The rule set a running system actually programs, against the frames a
//! peer actually sends.
//!
//! Every other classifier test builds a rule it also wrote. This one
//! starts from a readback of six rules taken off a running device and
//! asks the engine what it makes of them, which is the only question
//! that says whether the classifier can route anything at all.
//!
//! The fixture is built from the same constructors the model uses, then
//! checked against the words the readback returned — so a change to the
//! codec breaks the anchor rather than quietly moving both sides.

use bcm55030_emulator::soc::lue::{
    Action, ActionPayload, Clause, Engine, EngineCounters, Entry, Field, Link, Lue, Op, RuleStart,
    Verdict, CMD_GO, CMD_WRITE,
};
use bcm55030_emulator::soc::olt::mpcp::{self, GateFlags, Header, Opcode};
use bcm55030_emulator::soc::olt::oam::{self, Flags, InfoTlv, Oui};
use bcm55030_emulator::soc::olt::types::MacAddr;

/// Instance 1: the one the readback path addresses.
const CMD: u32 = 0x0100_280C;
const DATA: u32 = 0x0100_2810;
/// The table the readback path addresses.
const TABLE: u8 = 3;

/// A clause with no anchor, which is all but one of them.
fn clause(field: u8, op: Op, shift: u8, operand: u64) -> Clause {
    Clause { field: Field::from_code(field), op, shift, anchor: 0, operand }
}

/// The terminal action every one of the six rules ends with, at the two
/// priorities the readback shows.
fn terminal(priority: u8) -> Link {
    Link::Terminal(Action { priority, payload: ActionPayload::Simple { type_code: 6 } })
}

/// One rule: where it starts, and the clauses it chains.
struct Rule {
    index: u16,
    clauses: Vec<Clause>,
    action: Link,
}

/// The six rules a device returned, in the order it printed them.
///
/// Field code `0x02` is EtherType. `0x03`, `0x04`, `0x0F` and `0x10` are
/// the codes with no established frame field — which is the whole point
/// of this fixture.
fn live_rules() -> Vec<Rule> {
    vec![
        Rule {
            index: 0x2E,
            clauses: vec![clause(0x0F, Op::Eq, 0x38, 0)],
            action: terminal(2),
        },
        Rule {
            index: 0x2F,
            clauses: vec![
                clause(0x03, Op::NotExists, 0x3F, 0),
                clause(0x04, Op::NotExists, 0x3F, 0),
                clause(0x02, Op::Eq, 0x00, 0x8180),
            ],
            action: terminal(2),
        },
        Rule {
            index: 0x32,
            clauses: vec![
                clause(0x03, Op::NotExists, 0x3F, 0),
                clause(0x04, Op::NotExists, 0x3F, 0),
                clause(0x02, Op::Eq, 0x00, 0x888E),
            ],
            action: terminal(2),
        },
        Rule {
            index: 0x35,
            clauses: vec![clause(0x0F, Op::Ge, 0x38, 0x20)],
            action: terminal(0),
        },
        Rule {
            index: 0x36,
            clauses: vec![
                clause(0x02, Op::Eq, 0x30, 0x8808),
                clause(0x03, Op::NotExists, 0x3F, 0),
                clause(0x04, Op::NotExists, 0x3F, 0),
                clause(0x10, Op::Ge, 0x30, 2),
                clause(0x10, Op::Le, 0x30, 6),
            ],
            action: terminal(2),
        },
        Rule {
            index: 0x3B,
            clauses: vec![
                clause(0x02, Op::Eq, 0x30, 0x8809),
                clause(0x03, Op::NotExists, 0x3F, 0),
                clause(0x04, Op::NotExists, 0x3F, 0),
                // The one clause with an anchor, and the only value of it
                // that occurs on a rule this table holds.
                Clause {
                    field: Field::from_code(0x10),
                    op: Op::Eq,
                    shift: 0x30,
                    anchor: 8,
                    operand: 0x0300,
                },
            ],
            action: terminal(2),
        },
    ]
}

/// Program the fixture the way software does: data registers, then the
/// command word with the go bit.
fn program(lue: &mut Lue, index: u16, entry: Entry) {
    for (i, w) in entry.encode().to_regs().iter().enumerate() {
        lue.write_word(DATA + i as u32 * 4, *w).unwrap();
    }
    lue.write_word(CMD, CMD_GO | CMD_WRITE | ((TABLE as u32) << 12) | index as u32)
        .unwrap();
}

fn programmed() -> Lue {
    let mut lue = Lue::new();
    for rule in live_rules() {
        let last = rule.clauses.len() - 1;
        for (step, clause) in rule.clauses.iter().enumerate() {
            let link = if step == last { rule.action } else { Link::AndNext };
            program(&mut lue, rule.index + step as u16, Entry::Clause { clause: *clause, link });
        }
    }
    lue
}

fn starts() -> Vec<RuleStart> {
    live_rules()
        .iter()
        .map(|r| RuleStart { table: TABLE, index: r.index })
        .collect()
}

// ── Frames the peer really sends ────────────────────────────────────

fn olt_mac() -> MacAddr {
    MacAddr::new([0x02, 0x00, 0x00, 0x00, 0x00, 0x01])
}

fn onu_mac() -> MacAddr {
    MacAddr::new([0x02, 0x00, 0x00, 0x00, 0x00, 0x02])
}

fn header(opcode: Opcode) -> Header {
    Header { dst: MacAddr::MPCP_MULTICAST, src: olt_mac(), opcode, timestamp: 0x0001_0000 }
}

/// A discovery GATE: EtherType 0x8808, opcode 2 — the frame rule 0x36
/// was written for.
fn gate_frame() -> Vec<u8> {
    mpcp::gate(
        header(Opcode::Gate),
        GateFlags { grant_count: 1, discovery: true, force_report: 0 },
        None,
        Some(mpcp::DiscoveryWindow { sync_time: 0x0020, information: 0x0011 }),
    )
}

/// A REGISTER: same EtherType, opcode 5 — also inside rule 0x36's range.
fn register_frame() -> Vec<u8> {
    mpcp::bare(header(Opcode::Register))
}

/// An OAM Information PDU: EtherType 0x8809, subtype 0x03 — the frame
/// rule 0x3B was written for.
fn oam_frame() -> Vec<u8> {
    oam::information(
        onu_mac(),
        olt_mac(),
        Flags::local_stable(),
        InfoTlv {
            is_local: true,
            oam_version: 0x01,
            revision: 0,
            state: 0,
            configuration: 0,
            max_pdu_size: 1518,
            oui: Oui([0x00, 0x00, 0x00]),
            vendor_specific: [0; 4],
        },
    )
}

// ── The fixture is anchored to what hardware returned ───────────────

/// The clause headers and action words a device returned. Recomputed
/// here from the constructors: if the codec drifts, this breaks before
/// any conclusion is drawn from it.
#[test]
fn the_fixture_reproduces_the_words_the_readback_returned() {
    let expected: Vec<(u16, Vec<u32>)> = vec![
        (0x2E, vec![0xF038_00F1, 0x0270_0000]),
        (0x2F, vec![0xF03F_0036, 0xF03F_0046, 0xF000_0021, 0x0270_0000]),
        (0x32, vec![0xF03F_0036, 0xF03F_0046, 0xF000_0021, 0x0270_0000]),
        (0x35, vec![0xF038_00F4, 0x0070_0000]),
        (
            0x36,
            vec![
                0xF030_0021, 0xF03F_0036, 0xF03F_0046, 0xF030_0104, 0xF030_0103, 0x0270_0000,
            ],
        ),
        (
            0x3B,
            vec![0xF030_0021, 0xF03F_0036, 0xF03F_0046, 0xF030_2101, 0x0270_0000],
        ),
    ];

    for (rule, (index, words)) in live_rules().iter().zip(expected) {
        assert_eq!(rule.index, index);
        let headers: Vec<u32> = rule.clauses.iter().map(|c| c.header()).collect();
        let action = match rule.action {
            Link::Terminal(a) => a.encode(),
            Link::AndNext => unreachable!("a rule ends with an action"),
        };
        let mut got = headers;
        got.push(action);
        assert_eq!(got, words, "rule {index:#06X}");
    }
}

/// The six rules pack the table with no gap and no overlap: each starts
/// where the one below it ends. That is what makes "a rule is a run of
/// consecutive indices" a reading of the table rather than an assumption
/// about it — and it is how the engine finds rule starts at all.
#[test]
fn the_six_rules_tile_the_table_end_to_end() {
    let rules = live_rules();
    let mut next = rules[0].index;
    for rule in &rules {
        assert_eq!(rule.index, next, "rule {:#06X} does not follow the previous one", rule.index);
        next += rule.clauses.len() as u16;
    }
    assert_eq!(next, 0x3F, "the six rules should occupy 0x2E..0x3E");

    let lue = programmed();
    assert_eq!(lue.inst[1].entry_count(TABLE), 17, "17 entries, as the readback counted");
    let found = lue.inst[1].rule_starts(TABLE);
    let expected: Vec<u16> = rules.iter().map(|r| r.index).collect();
    assert_eq!(found, expected, "the engine must find exactly these six rule starts");
}

// ── What the engine makes of them ───────────────────────────────────

/// ⛔ **The frontier.** Not one of the six rules can be evaluated: every
/// one of them tests a field code with no established frame field
/// (`0x03`, `0x04`, `0x0F`, `0x10`), and one also carries an anchor the
/// model refuses to act on.
///
/// So switching the classifier on changes no routing whatsoever — every
/// frame comes back undecidable and falls through to the EtherType
/// fallback. This test exists so that stays a measured fact: the day a
/// field code is established, it fails, and that is the signal.
#[test]
fn every_live_rule_is_undecidable_so_no_frame_can_be_routed_by_one() {
    let lue = programmed();
    let rules = starts();

    for (name, frame) in [
        ("GATE", gate_frame()),
        ("REGISTER", register_frame()),
        ("OAM information", oam_frame()),
    ] {
        let mut c = EngineCounters::default();
        let verdict = Engine::classify(&lue.inst[1], &rules, &frame, &mut c);
        assert!(
            matches!(verdict, Verdict::Undecidable { .. }),
            "{name}: expected undecidable, got {verdict:?}"
        );
        assert_eq!(c.matched, 0, "{name}: nothing may match while a rule cannot be read");
        assert_eq!(
            c.undecidable_field, 1,
            "{name}: the refusal must be the unestablished field code"
        );
        assert!(c.verdicts_accounted_for(), "{name}: {c:?}");
    }
}

/// Rule by rule, so the reason is attributed rather than aggregated.
///
/// Every one of the six stops on the same thing: a field code with no
/// established frame field. **One** gap, not two — the anchor used to be
/// a second refusal, and is not one any more.
#[test]
fn every_rule_stops_on_the_same_single_gap() {
    let lue = programmed();
    let frame = gate_frame();

    for start in starts() {
        let mut c = EngineCounters::default();
        let verdict = Engine::evaluate_rule(&lue.inst[1], start, &frame, &mut c);
        assert!(
            matches!(verdict, Verdict::Undecidable { .. }),
            "rule {:#06X}: {verdict:?}",
            start.index
        );
        assert_eq!(c.undecidable_field, 1, "rule {:#06X}", start.index);
        assert_eq!(c.undecidable_window, 0, "the window is no longer a refusal");
        assert_eq!(c.undecidable_operator, 0, "rule {:#06X}", start.index);
    }

    // The shape of rule 0x3B's last clause, over a field the model can
    // read: the anchor no longer stops it, and it discriminates.
    for (ethertype_word, expected) in [(0x0300u16, true), (0x0100, false)] {
        let mut lue = Lue::new();
        program(
            &mut lue,
            0x100,
            Entry::Clause {
                clause: Clause {
                    field: Field::EtherType,
                    op: Op::Eq,
                    shift: 0x30,
                    anchor: 8,
                    operand: 0x0300,
                },
                link: terminal(2),
            },
        );
        let mut probe = vec![0u8; 64];
        probe[12] = (ethertype_word >> 8) as u8;
        probe[13] = ethertype_word as u8;

        let mut c = EngineCounters::default();
        let verdict = Engine::classify(
            &lue.inst[1],
            &[RuleStart { table: TABLE, index: 0x100 }],
            &probe,
            &mut c,
        );
        if expected {
            assert!(matches!(verdict, Verdict::Match { .. }), "{ethertype_word:#06x}: {verdict:?}");
        } else {
            assert_eq!(verdict, Verdict::NoMatch, "{ethertype_word:#06x}");
        }
        assert_eq!(c.undecidable_window, 0);
    }
}

/// A rule the model *can* read, deciding a real frame's queue — with the
/// negative control next to it.
///
/// This is the one path where every link holds at once: an established
/// field (`0x00`, the destination address), an operator the model acts
/// on, and an action that names a queue. The rule is the one a running
/// system programs into instance 0 / table 2, reproduced here from the
/// constructors.
///
/// Two frames differing only in their destination: one matches and is
/// routed by the rule, the other does not and falls back. Without the
/// second, "the classifier routed it" cannot be told from "the
/// classifier routes everything".
#[test]
fn a_readable_rule_routes_a_frame_and_leaves_its_neighbour_alone() {
    use bcm55030_emulator::soc::lue::ClassifierBinding;
    use bcm55030_emulator::soc::olt::Olt;
    use bcm55030_emulator::soc::peripheral::Peripheral;

    const BROADCAST: MacAddr = MacAddr::new([0xFF; 6]);
    // Instance 0, table 2 — where the destination-address rule lives.
    let binding = ClassifierBinding { instance: 0, table: 2 };
    // The queue result, carrying the queue the receive path polls for
    // everything that is not control-plane.
    let action = Action {
        priority: 1,
        payload: ActionPayload::Selected { sel: 2, sub: 0, field: 0x0F, wide: true },
    };
    assert_eq!(action.destination_queue(), Some(0x0F));

    let entry = Entry::Clause {
        clause: Clause {
            field: Field::from_code(0x00),
            op: Op::Eq,
            // 49 bits, which is how the rule is programmed: a 48-bit
            // address with the bit above it required to be clear.
            shift: 0x0F,
            anchor: 0,
            operand: 0x0000_FFFF_FFFF_FFFF,
        },
        link: Link::Terminal(action),
    };
    let mut lue = Lue::new();
    for (i, w) in entry.encode().to_regs().iter().enumerate() {
        lue.write_word(0x0100_2410 + i as u32 * 4, *w).unwrap();
    }
    lue.write_word(0x0100_240C, CMD_GO | CMD_WRITE | (2 << 12) | 0x7F).unwrap();

    // A frame the rule covers, and one that differs only in where it is
    // addressed.
    let frame = |dst: MacAddr| {
        let mut f = dst.octets().to_vec();
        f.extend_from_slice(&onu_mac().octets());
        // IPv4 — an EtherType no control-plane rule claims, so the
        // fallback and the rule cannot agree by accident.
        f.extend_from_slice(&[0x08, 0x00]);
        f.resize(60, 0);
        f
    };

    let mut olt = Olt::new();
    olt.set_link_up(true);

    let settle = |olt: &mut Olt, lue: &Lue| {
        for _ in 0..256 {
            olt.tick(0);
            olt.load_frames_into_mailbox(Some((lue, binding)));
        }
    };
    // The peer talks on its own schedule; take the baseline after it has
    // started so its frames are not counted as this test's.
    settle(&mut olt, &lue);
    let before = olt.classifier_counters;

    olt.inject_raw_frame(frame(BROADCAST));
    settle(&mut olt, &lue);
    let after_match = olt.classifier_counters;
    assert_eq!(
        after_match.matched,
        before.matched + 1,
        "the broadcast frame should be routed by the rule: {after_match:?}"
    );

    olt.inject_raw_frame(frame(onu_mac()));
    settle(&mut olt, &lue);
    let after_miss = olt.classifier_counters;
    assert_eq!(
        after_miss.matched, after_match.matched,
        "a frame the rule does not cover must not be routed by it"
    );
    assert_eq!(
        after_miss.no_match,
        after_match.no_match + 1,
        "and it must be counted as a miss, not lost: {after_miss:?}"
    );
    assert!(after_miss.decisions_accounted_for(), "{after_miss:?}");
}

/// ⛔ **The trap this whole file exists to keep shut.** Most selected
/// actions are not queue results at all, and reading their field as a
/// queue would route frames by a number that is not one.
///
/// Over a full boot, a live rule set programs 84 selected actions and
/// **not one** of them is the queue result: 64 carry one neighbouring
/// result and 20 another. Their fields run 0x01..0x1F — a range that
/// looks exactly like a queue number and is not.
#[test]
fn a_selected_action_that_is_not_the_queue_result_names_no_queue() {
    // The two the live rule set actually contains.
    for (sub, field) in [(3u8, 0x04u16), (5, 0x01)] {
        let action = Action {
            priority: 1,
            payload: ActionPayload::Selected { sel: 2, sub, field, wide: true },
        };
        assert_eq!(
            action.destination_queue(),
            None,
            "sub-index {sub} is a different result, not a queue"
        );
    }
    // And the one that is.
    let queue = Action {
        priority: 1,
        payload: ActionPayload::Selected { sel: 2, sub: 0, field: 0x10, wide: true },
    };
    assert_eq!(queue.destination_queue(), Some(0x10));
    // Nothing outside the selected form names a queue either.
    for payload in [
        ActionPayload::Simple { type_code: 6 },
        ActionPayload::Broadcast,
        ActionPayload::Raw(0x2100_0010),
    ] {
        assert_eq!(Action { priority: 1, payload }.destination_queue(), None, "{payload:?}");
    }
}

/// ⛔ **The second frontier, and the one that matters for routing.**
///
/// Even a rule this table holds that *did* match would name no queue:
/// all six end in the same action type, and the model's only path from a
/// verdict to a mailbox slot needs a `Selected` action whose field is the
/// slot. There is not one in this table.
///
/// So the queue-selecting rules are not the ones the readback path
/// prints — which makes the instance/table binding, not the verdict-to-
/// queue step, the thing to establish next.
#[test]
fn no_rule_in_this_table_names_a_queue() {
    for rule in live_rules() {
        let Link::Terminal(action) = rule.action else {
            unreachable!("a rule ends with an action");
        };
        assert!(
            !matches!(action.payload, ActionPayload::Selected { .. }),
            "rule {:#06X} names a queue after all: {:?}",
            rule.index,
            action.payload
        );
        assert!(
            matches!(action.payload, ActionPayload::Simple { type_code: 6 }),
            "rule {:#06X}: {:?}",
            rule.index,
            action.payload
        );
    }
}
