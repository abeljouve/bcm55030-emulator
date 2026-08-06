//! Matching a frame against the rule tables.
//!
//! A rule is a run of entries at consecutive indices in one table. Each
//! carries one clause; an entry whose fourth word is the link word says
//! another clause follows and **both** must match. The first entry whose
//! fourth word is something else ends the rule, and that word is its
//! first action word.
//!
//! Three things this engine deliberately refuses to do:
//!
//! * **Guess a field.** Only three field codes have a frame field behind
//!   them. The other codes a live rule set uses are real but unexplained,
//!   and a clause over one of them returns [`Verdict::Undecidable`] —
//!   never a match, never a silent miss.
//! * **Evaluate a non-zero anchor.** Two anchor values occur and neither
//!   is explained. Decoding one is safe; acting on one is not.
//! * **Fall through.** A frame that matches nothing is a miss with a
//!   counter, not an implicit default.

use super::port::LuePort;
use super::rule::{Action, Clause, Entry, Field, Link, Op};

/// Which frame field a clause reads.
///
/// Only the three codes with a provenance are here. Everything else is
/// undecidable by construction rather than by omission.
fn extract_field(field: Field, frame: &[u8]) -> Option<u64> {
    if frame.len() < 14 {
        return None;
    }
    let be = |b: &[u8]| b.iter().fold(0u64, |acc, &x| (acc << 8) | x as u64);
    match field {
        // -- OBSERVED: every clause using this code is compared against
        // an EtherType value in a live rule set.
        Field::EtherType => Some(be(&frame[12..14])),
        // -- INFERRED: the two address codes are established elsewhere in
        // the classifier documentation but no observed clause uses them,
        // so nothing here has ever exercised this path.
        Field::Observed(0x00) | Field::Unknown(0x00) => Some(be(&frame[0..6])),
        Field::Observed(0x01) | Field::Unknown(0x01) => Some(be(&frame[6..12])),
        _ => None,
    }
}

/// Compare, on the low `compared_bits` of both sides.
///
/// Returns `None` for an operator the model will not act on.
fn compare(op: Op, value: u64, operand: u64, bits: u32) -> Option<bool> {
    let mask = if bits >= 64 { u64::MAX } else { (1u64 << bits) - 1 };
    let (v, o) = (value & mask, operand & mask);
    match op {
        Op::Never => Some(false),
        Op::Always => Some(true),
        Op::Eq => Some(v == o),
        Op::Ne => Some(v != o),
        Op::Le => Some(v <= o),
        Op::Ge => Some(v >= o),
        // -- INFERRED and not acted on: "exists" is a statement about
        // whether the field is present in the frame, and this engine has
        // no notion of an absent field. Answering would be inventing one.
        Op::Exists | Op::NotExists => None,
        Op::Unknown(_) => None,
    }
}

/// What the classifier concluded.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// No rule matched. Per the specification this is a drop, not a
    /// default queue — the caller decides, and counts.
    NoMatch,
    /// A rule could neither match nor be ruled out, because part of it
    /// is not established. Never silently treated as either.
    Undecidable { reason: &'static str },
    Match { priority: u8, action: Action },
}

/// Why the engine refused, and how often. Every refusal is counted:
/// a path that drops a decision without incrementing something reads as
/// a working classifier.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EngineCounters {
    pub frames_classified: u64,
    pub matched: u64,
    pub no_match: u64,
    /// A clause over a field code with no established frame field.
    pub undecidable_field: u64,
    /// A clause with a non-zero anchor.
    pub undecidable_anchor: u64,
    /// A clause with an operator the model will not act on.
    pub undecidable_operator: u64,
    /// An entry the decoder could not read as a clause.
    pub undecidable_entry: u64,
    /// Rules that ran off the end of their table without terminating.
    pub unterminated_rules: u64,
}

impl EngineCounters {
    /// Every frame classified left exactly one verdict behind.
    pub fn verdicts_accounted_for(&self) -> bool {
        self.frames_classified
            == self.matched
                + self.no_match
                + self.undecidable_field
                + self.undecidable_anchor
                + self.undecidable_operator
                + self.undecidable_entry
                + self.unterminated_rules
    }
}

/// How far a rule may run before the engine calls it unterminated.
const MAX_RULE_LEN: u16 = 64;

/// Where a rule starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RuleStart {
    pub table: u8,
    pub index: u16,
}

pub struct Engine;

impl Engine {
    /// Evaluate one rule against a frame.
    ///
    /// ⚠ Order and precedence are a **contradiction carried forward, not
    /// settled**: one specification says every rule is evaluated and the
    /// highest priority wins, another says evaluation stops at the first
    /// match. The hardware leans towards the first reading — each action
    /// word carries its own four priority bits, which a stop-at-first
    /// design would not need. [`Engine::classify`] takes the highest
    /// priority; `a_rule_set_where_the_two_readings_disagree` exhibits a
    /// case where the two answers differ.
    pub fn evaluate_rule(
        port: &LuePort,
        start: RuleStart,
        frame: &[u8],
        counters: &mut EngineCounters,
    ) -> Verdict {
        let mut all_matched = true;
        for step in 0..MAX_RULE_LEN {
            let index = start.index.wrapping_add(step);
            let Some(quad) = port.entry(start.table, index) else {
                counters.unterminated_rules += 1;
                return Verdict::Undecidable { reason: "rule ran past the last entry" };
            };
            let Entry::Clause { clause, link } = Entry::decode(quad) else {
                counters.undecidable_entry += 1;
                return Verdict::Undecidable { reason: "entry is not a clause" };
            };

            match Self::test_clause(&clause, frame) {
                Err(reason) => {
                    match reason {
                        "field has no established frame field" => counters.undecidable_field += 1,
                        "clause anchor is not understood" => counters.undecidable_anchor += 1,
                        _ => counters.undecidable_operator += 1,
                    }
                    return Verdict::Undecidable { reason };
                }
                Ok(matched) => all_matched &= matched,
            }

            match link {
                Link::AndNext => continue,
                Link::Terminal(action) => {
                    return if all_matched {
                        counters.matched += 1;
                        Verdict::Match { priority: action.priority, action }
                    } else {
                        counters.no_match += 1;
                        Verdict::NoMatch
                    };
                }
            }
        }
        counters.unterminated_rules += 1;
        Verdict::Undecidable { reason: "rule longer than any observed one" }
    }

    /// One clause against one frame. `Err` carries why it could not be
    /// decided — the caller turns that into a counter.
    fn test_clause(clause: &Clause, frame: &[u8]) -> Result<bool, &'static str> {
        if clause.anchor != 0 {
            return Err("clause anchor is not understood");
        }
        let value = extract_field(clause.field, frame)
            .ok_or("field has no established frame field")?;
        compare(clause.op, value, clause.operand, clause.compared_bits())
            .ok_or("operator is not one the model acts on")
    }

    /// Evaluate every rule and take the highest priority that matched.
    ///
    /// An undecidable rule stops the whole classification: a verdict
    /// reached by skipping the rule that could not be read would be a
    /// guess wearing a result's clothes.
    pub fn classify(
        port: &LuePort,
        rules: &[RuleStart],
        frame: &[u8],
        counters: &mut EngineCounters,
    ) -> Verdict {
        counters.frames_classified += 1;
        let mut per_rule = EngineCounters::default();
        let mut best: Option<(u8, Action)> = None;
        for start in rules {
            match Self::evaluate_rule(port, *start, frame, &mut per_rule) {
                Verdict::Match { priority, action } => {
                    if best.is_none_or(|(p, _)| priority > p) {
                        best = Some((priority, action));
                    }
                }
                Verdict::NoMatch => {}
                Verdict::Undecidable { reason } => {
                    // Carry the per-rule tally so the reason survives.
                    counters.undecidable_field += per_rule.undecidable_field;
                    counters.undecidable_anchor += per_rule.undecidable_anchor;
                    counters.undecidable_operator += per_rule.undecidable_operator;
                    counters.undecidable_entry += per_rule.undecidable_entry;
                    counters.unterminated_rules += per_rule.unterminated_rules;
                    return Verdict::Undecidable { reason };
                }
            }
        }
        match best {
            Some((priority, action)) => {
                counters.matched += 1;
                Verdict::Match { priority, action }
            }
            None => {
                counters.no_match += 1;
                Verdict::NoMatch
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::port::{Quad, CMD_GO, CMD_WRITE};
    use super::super::rule::{ActionPayload, Field, LINK_AND_NEXT};
    use super::*;
    use crate::soc::olt::mailbox::Slot;
    use crate::soc::olt::types::EtherType;

    /// Program a rule into a port the way software does: data registers
    /// first, then the command word.
    fn program(port: &mut LuePort, table: u8, index: u16, entry: Entry) {
        for (i, w) in entry.encode().to_regs().iter().enumerate() {
            port.write_data(i, *w);
        }
        port.write_cmd(CMD_GO | CMD_WRITE | ((table as u32) << 12) | index as u32);
    }

    fn frame_with_ethertype(et: u16) -> Vec<u8> {
        let mut f = vec![0u8; 64];
        f[12] = (et >> 8) as u8;
        f[13] = et as u8;
        f
    }

    fn ethertype_rule(et: u16, priority: u8) -> Entry {
        Entry::Clause {
            clause: Clause {
                field: Field::EtherType,
                op: Op::Eq,
                shift: 0x30,
                anchor: 0,
                operand: et as u64,
            },
            link: Link::Terminal(Action {
                priority,
                payload: ActionPayload::Simple { type_code: 6 },
            }),
        }
    }

    #[test]
    fn a_single_clause_rule_matches_its_ethertype_and_nothing_else() {
        let mut port = LuePort::new();
        program(&mut port, 3, 0x40, ethertype_rule(0x8809, 2));
        let rules = [RuleStart { table: 3, index: 0x40 }];
        let mut c = EngineCounters::default();

        let v = Engine::classify(&port, &rules, &frame_with_ethertype(0x8809), &mut c);
        assert!(matches!(v, Verdict::Match { priority: 2, .. }));

        let v = Engine::classify(&port, &rules, &frame_with_ethertype(0x8808), &mut c);
        assert_eq!(v, Verdict::NoMatch);
        assert!(c.verdicts_accounted_for(), "{c:?}");
    }

    /// Chained clauses are an AND: the link word says another follows,
    /// and a frame has to satisfy both.
    #[test]
    fn chained_clauses_must_all_match() {
        let mut port = LuePort::new();
        program(&mut port, 3, 0x10, Entry::Clause {
            clause: Clause {
                field: Field::EtherType,
                op: Op::Eq,
                shift: 0x30,
                anchor: 0,
                operand: 0x8809,
            },
            link: Link::AndNext,
        });
        program(&mut port, 3, 0x11, Entry::Clause {
            clause: Clause {
                field: Field::from_code(0x00),
                op: Op::Eq,
                shift: 0x10,
                anchor: 0,
                operand: 0x0180_C200_0001,
            },
            link: Link::Terminal(Action {
                priority: 1,
                payload: ActionPayload::Simple { type_code: 6 },
            }),
        });
        let rules = [RuleStart { table: 3, index: 0x10 }];
        let mut c = EngineCounters::default();

        let mut good = frame_with_ethertype(0x8809);
        good[0..6].copy_from_slice(&[0x01, 0x80, 0xC2, 0x00, 0x00, 0x01]);
        assert!(matches!(
            Engine::classify(&port, &rules, &good, &mut c),
            Verdict::Match { .. }
        ));

        // Right EtherType, wrong address: the chain fails.
        let bad = frame_with_ethertype(0x8809);
        assert_eq!(Engine::classify(&port, &rules, &bad, &mut c), Verdict::NoMatch);
    }

    /// A field code nobody has explained must not be guessed at, and the
    /// refusal must leave a number behind. This is the state a live rule
    /// set is actually in: its clauses use codes `0x03`, `0x04`, `0x0F`
    /// and `0x10`, none of which is pinned to a frame field.
    #[test]
    fn an_unexplained_field_is_undecidable_and_counted() {
        let mut port = LuePort::new();
        for code in [0x03u8, 0x04, 0x0F, 0x10] {
            program(&mut port, 3, 0x20, Entry::Clause {
                clause: Clause {
                    field: Field::from_code(code),
                    op: Op::Eq,
                    shift: 0x3F,
                    anchor: 0,
                    operand: 0,
                },
                link: Link::Terminal(Action {
                    priority: 0,
                    payload: ActionPayload::Simple { type_code: 6 },
                }),
            });
            let mut c = EngineCounters::default();
            let v = Engine::classify(
                &port,
                &[RuleStart { table: 3, index: 0x20 }],
                &frame_with_ethertype(0x8809),
                &mut c,
            );
            assert!(matches!(v, Verdict::Undecidable { .. }), "field {code:#04x}");
            assert_eq!(c.undecidable_field, 1);
            assert!(c.verdicts_accounted_for(), "{c:?}");
        }
    }

    /// The anchor is decoded but never acted on.
    #[test]
    fn a_non_zero_anchor_is_undecidable_and_counted() {
        let mut port = LuePort::new();
        program(&mut port, 3, 0x30, Entry::Clause {
            clause: Clause {
                field: Field::EtherType,
                op: Op::Eq,
                shift: 0x30,
                anchor: 8,
                operand: 0x8809,
            },
            link: Link::Terminal(Action {
                priority: 0,
                payload: ActionPayload::Simple { type_code: 6 },
            }),
        });
        let mut c = EngineCounters::default();
        let v = Engine::classify(
            &port,
            &[RuleStart { table: 3, index: 0x30 }],
            &frame_with_ethertype(0x8809),
            &mut c,
        );
        assert!(matches!(v, Verdict::Undecidable { .. }));
        assert_eq!(c.undecidable_anchor, 1);
    }

    /// An empty table decides nothing — it does not quietly match.
    #[test]
    fn an_empty_table_is_undecidable_not_a_miss() {
        let port = LuePort::new();
        let mut c = EngineCounters::default();
        let v = Engine::classify(
            &port,
            &[RuleStart { table: 3, index: 0 }],
            &frame_with_ethertype(0x8809),
            &mut c,
        );
        assert!(matches!(v, Verdict::Undecidable { .. }));
        assert_eq!(c.unterminated_rules, 1);
        assert_eq!(c.matched, 0, "an empty table must never match");
    }

    /// The two readings of the specification disagree on this rule set:
    /// stop-at-first would return priority 1, highest-priority returns 3.
    /// The engine takes the second, and this is the case that would show
    /// it up if that turns out to be wrong.
    #[test]
    fn a_rule_set_where_the_two_readings_disagree() {
        let mut port = LuePort::new();
        program(&mut port, 3, 0x50, ethertype_rule(0x8809, 1));
        program(&mut port, 3, 0x51, ethertype_rule(0x8809, 3));
        let mut c = EngineCounters::default();
        let v = Engine::classify(
            &port,
            &[RuleStart { table: 3, index: 0x50 }, RuleStart { table: 3, index: 0x51 }],
            &frame_with_ethertype(0x8809),
            &mut c,
        );
        assert_eq!(
            v,
            Verdict::Match {
                priority: 3,
                action: Action { priority: 3, payload: ActionPayload::Simple { type_code: 6 } }
            },
            "highest priority wins; stop-at-first would have said 1"
        );
    }

    /// What the engine concludes, next to what the EtherType fallback
    /// concludes. They are different questions — the fallback always
    /// answers, the engine only answers when it can — and this is the
    /// comparison the classifier exists to make.
    #[test]
    fn the_verdict_and_the_ethertype_fallback_are_compared_not_conflated() {
        let mut port = LuePort::new();
        program(&mut port, 3, 0x60, ethertype_rule(0x8809, 2));
        let rules = [RuleStart { table: 3, index: 0x60 }];
        let mut c = EngineCounters::default();

        let control = frame_with_ethertype(EtherType::SlowProtocol.as_u16());
        assert_eq!(Slot::for_frame(&control), Slot::CONTROL);
        assert!(matches!(
            Engine::classify(&port, &rules, &control, &mut c),
            Verdict::Match { .. }
        ));

        // The fallback routes this one too; the rule set does not cover
        // it, and the engine says so rather than inventing a queue.
        let data = frame_with_ethertype(0x0800);
        assert_eq!(Slot::for_frame(&data), Slot::DATA);
        assert_eq!(Engine::classify(&port, &rules, &data, &mut c), Verdict::NoMatch);
        assert!(c.verdicts_accounted_for(), "{c:?}");
    }

    /// The port must round-trip whatever it is given, so an entry the
    /// decoder cannot read is a refusal, not a crash.
    #[test]
    fn an_entry_that_is_not_a_clause_is_undecidable() {
        let mut port = LuePort::new();
        for (i, w) in Quad::new([0x0123_4567, 0, 0, LINK_AND_NEXT]).to_regs().iter().enumerate() {
            port.write_data(i, *w);
        }
        port.write_cmd(CMD_GO | CMD_WRITE | (3 << 12) | 0x70);
        let mut c = EngineCounters::default();
        let v = Engine::classify(
            &port,
            &[RuleStart { table: 3, index: 0x70 }],
            &frame_with_ethertype(0x8809),
            &mut c,
        );
        assert!(matches!(v, Verdict::Undecidable { .. }));
        assert_eq!(c.undecidable_entry, 1);
    }
}
