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
//! * **Guess an operator.** Four of the sixteen operator codes are
//!   established by their operands; software emits four more whose
//!   meaning is not. Acting on one of those is worst where it looks most
//!   harmless — a rule whose clauses all carry an unexplained code would
//!   match *every* frame and route it, which is not a missing answer but
//!   a confident wrong one. Exactly one unexplained code is acted on, in
//!   exactly one place: see [`OP_TAG_ABSENCE`].
//! * **Fall through.** A frame that matches nothing is a miss with a
//!   counter, not an implicit default.
//!
//! Not every clause is a comparison. One operator is read as asking
//! whether the frame carries a tag at all, and that is answered from the
//! frame's shape rather than from a value — see [`field_presence`], which
//! decides the case where an open question does not matter and refuses
//! the case where it does.
//!
//! A clause compares over a window, not over the whole field: `shift`
//! and `anchor` are its two ends, and both are honoured here. See
//! [`super::rule::Clause::anchor`] for what establishes that, and for
//! the one thing about it that is still open.

use super::port::LuePort;
use super::rule::{Action, Clause, Field, Op, RuleError};
use epon_olt::types::ETHERNET_HEADER_LEN;

/// Which frame field a clause reads.
///
/// Only the three codes with a provenance are here. Everything else is
/// undecidable by construction rather than by omission.
fn extract_field(field: Field, frame: &[u8]) -> Option<u64> {
    if frame.len() < ETHERNET_HEADER_LEN {
        return None;
    }
    let be = |b: &[u8]| b.iter().fold(0u64, |acc, &x| (acc << 8) | x as u64);
    match field {
        // -- OBSERVED: every clause using this code is compared against
        // an EtherType value in a live rule set.
        Field::EtherType => Some(be(&frame[12..14])),
        // -- OBSERVED, and it does not rest on a documentation table: the
        // rules that route compare this code against 01:80:C2:00:00:01
        // and 01:80:C2:00:00:02, which are group addresses. A group
        // address cannot be a source address in any Ethernet frame, so
        // the code is the destination.
        Field::Observed(0x00) | Field::Unknown(0x00) => Some(be(&frame[0..6])),
        // -- INFERRED: the complementary code, compared elsewhere against
        // a unicast address. Consistent with the source address, and not
        // pinned by anything the way the code above is.
        Field::Observed(0x01) | Field::Unknown(0x01) => Some(be(&frame[6..12])),
        // -- INFERRED, and falsifiable the same way the extractor code is:
        // the frame's length in bytes.
        //
        // The rule set pins the shape hard. One table holds eighty rules
        // built from one four-rule group repeated: `== 0x40`,
        // `>= 0x41 && <= 0x7F`, `>= 0x80`, `>= 0x100` — 64, 65..127, 128,
        // 256, the size buckets every Ethernet counter block keeps. Each
        // group ends in a different result index, counting up across the
        // groups, which is what a bucketed counter looks like and is not
        // what a routing table looks like: not one of the eighty names a
        // queue.
        //
        // Read as anything but a length, those four comparands are a
        // coincidence four times over.
        Field::Observed(FRAME_LENGTH) | Field::Unknown(FRAME_LENGTH) => Some(frame.len() as u64),
        Field::Observed(DEFAULT_EXTRACTOR) | Field::Unknown(DEFAULT_EXTRACTOR) => {
            // A tagged frame moves everything after the type field, and
            // the model has no established offset to move it by.
            if field_presence(Field::from_code(TAG_FIELD_CODES[0]), frame) != Presence::Absent {
                return None;
            }
            frame.get(ETHERNET_HEADER_LEN..ETHERNET_HEADER_LEN + 2).map(be)
        }
        // The link index. Its meaning IS established — see [`LINK_INDEX`]
        // — and it is still refused, because the value is link metadata
        // and this function is handed frame bytes. Spelled out rather
        // than left to the catch-all so the refusal reads as a decision.
        Field::Observed(LINK_INDEX) | Field::Unknown(LINK_INDEX) => None,
        _ => None,
    }
}

/// The one extractor the firmware programs on its boot path, and so the
/// only one of the seven a clause can name and be answered.
///
/// Extractors are slots, not fields: a slot is allocated, its descriptor
/// is written, and the allocator hands back `slot + 0x10` for a rule to
/// use as a field code. The inverse exists and closes it — the release
/// path computes `slot = code - 0x10`. Slot 0 is set aside at start-up:
/// its allocation bit is marked before any allocation runs and its
/// descriptor is written once per bank. Measured over a whole boot, the
/// descriptor block takes **exactly two writes**, both of the same word,
/// one per bank, from one instruction.
///
/// ⚠ "Set aside" is not "unreachable". The mark covers two of the three
/// templates, and a caller asking for a field no wider than the slot-0
/// scan threshold starts its search at slot 0 — as does a diagnostic
/// command the shipped image exposes. What is established is narrower and
/// is what the model needs: **on the boot path this descriptor is written
/// once and never rewritten.**
///
/// ⛔ **What the descriptor selects is NOT established**, and the one
/// thing that used to be called pinned has since been contradicted. The
/// claim was that the descriptor's width is 16 bits, read off the width
/// column of the software-side table that turns a wire field code into a
/// header field code. That table has since been read out of a live
/// device in full, and **not one of its extractor rows is 16 bits wide**
/// — the rows that name an extractor rather than a hardware field carry
/// widths of 0x14 and 0x20, never 0x10. So either the row was misread or
/// the width does not come from that table at all.
///
/// ⛔ The remaining columns of that table are **not** imported here, and
/// must not be: it is a software translation table, indexed before
/// anything is written, and the hardware never reads it. Its column names
/// are inferred, not measured.
///
/// -- INFERRED: this model reads those sixteen bits as the two octets
/// **after** the type field. What the image gives is only this — the
/// value is 16 bits wide, it is bounded to `[2, 6]` under one EtherType,
/// and its top octet is compared against two constants under another.
/// Mapping those numbers onto named protocol fields is a lookup in a
/// standard, not evidence about this silicon, and it is deliberately not
/// the argument here.
///
/// The argument is that the reading is **falsifiable and was tested**:
/// under a wrong offset the two rules using this code can never match, so
/// a boot would route nothing by them. Run against the peer, they match
/// 203 of 596 frames, every one to the queue the EtherType fallback
/// picks independently and none to a different one. That is an outcome
/// this reading could have failed and did not.
///
/// -- A corroboration that costs nothing and was not sought: the same
/// full read of the translation table shows **six** rows naming an
/// extractor rather than a hardware field. Six plus the reserved slot 0
/// is seven, which is exactly the width of the `0x10..=0x16` slot range
/// this constant sits at the bottom of. The range closes.
const DEFAULT_EXTRACTOR: u8 = 0x10;

/// The code the size-bucket rules compare against frame-length boundaries.
///
/// Unlike [`DEFAULT_EXTRACTOR`] this one is not an extractor slot — it is
/// outside the `0x10..=0x16` slot range, so it names a field the hardware
/// knows on its own.
const FRAME_LENGTH: u8 = 0x24;

/// The code the per-link tables compare against a link index.
///
/// -- OBSERVED, and from software's own code rather than from a table of
/// field names: the batch-commit helper **appends this clause itself**.
/// When a batch is finalised and the first byte of its header is zero, it
/// pushes one more clause — this code, equality, operand `header[2] + 1`
/// — before handing the batch to the rule writer. `header[2]` is the
/// link index the batch is being programmed for, so the operand is that
/// index plus one, and nothing else in a batch can produce this code.
///
/// The rule set corroborates the reading independently and was not
/// consulted to reach it. Instance 0 tables 4 through 7 each hold three
/// rules naming *other* operands and one terminal rule naming their own:
/// the table whose terminal is `== 1` carries rules for 2, 3 and 4, the
/// one with `== 2` carries 1, 3 and 4, and so on. That is a per-link
/// discrimination table, one table per link, and it fixes the table index
/// too: `header[2] + 4`. Seeding a persistent configuration record so
/// that a batch is programmed with `header[2] = 0` put the new rule in
/// table 4 with the clause `== 1` — what this reading predicts, and what
/// a wrong one could not have produced.
///
/// ⛔ **The engine still refuses it, and deliberately.** The value is not
/// in the frame: it is the index of the link the frame arrived on, and
/// [`Engine::classify`] is handed bytes and nothing else. Two things
/// would have to be settled before answering it — which index a frame
/// carrying the broadcast link identifier takes, and whether the operand
/// is the index plus one or a separate numbering that merely starts at
/// one. Guessing either would turn a refusal into a wrong match.
///
/// The denominator of what answering it would unblock is measured: over a
/// boot plus four injected frames, binding the classifier to instance 0
/// tables 4 through 8 yields **592 field refusals out of 600 frames** and
/// not one verdict. Every one of those refusals is this code.
///
/// ⛔ **The identification is back to being a hypothesis.** The argument
/// above turns on the commit helper emitting *this code*, and it reads
/// the `0x0F` at that site as a header field code. There are **two code
/// spaces** — see [`super::rule::Field`] — and nothing at that site says
/// which one the byte belongs to. Read as a wire code instead, it
/// translates to header code `0x0A`, and then the clause the helper
/// appends is not this code at all and the corroboration below is about
/// some other field.
///
/// ⚠ Two things inherit the doubt and must move with it if it resolves:
/// the per-link table story in [`super::ClassifierBinding`], and the
/// table index `header[2] + 4` that goes with it. Nothing is renamed or
/// removed on the strength of a hypothesis — this is a flag, not a
/// retraction.
const LINK_INDEX: u8 = 0x0F;

/// The EtherTypes that mark a tagged frame.
///
/// -- OBSERVED: they are the two halves of one configurable register that
/// sits in the same lane file as the classifier port, whose power-on
/// value holds exactly these two. A frame whose type field is neither
/// carries no tag at all, which is the only case [`field_presence`] has
/// to decide.
const TAG_ETHERTYPES: [u16; 2] = [0x8100, 0x88A8];

/// The two field codes that name a tag.
///
/// ⛔ **Which is which is not established, and neither is the rule the
/// hardware uses to tell them apart.** Selecting by tag EtherType and
/// selecting by tag position are both consistent with every instruction
/// in the image; the argument that once pinned the first was refuted, and
/// the second is ruled out for a pair of codes by a rule that asks for
/// one absent and the other equal to a tag value at once. This model
/// encodes neither.
///
/// -- OBSERVED that absence is a state of its own and not a value: on one
/// path software emits [`OP_TAG_ABSENCE`] for a tag whose configured
/// value is zero and equality for any other value, so "not there" and
/// "there, and zero" are two different questions to this hardware.
const TAG_FIELD_CODES: [u8; 2] = [0x03, 0x04];

/// The one unexplained operator code this engine acts on — and only over
/// the two tag field codes.
///
/// -- INFERRED that it means "the frame does not carry this field". The
/// argument is **not** the one this file used to make; that one said the
/// configured value was zero, which reads just as well as a wildcard and
/// therefore separated nothing. What does separate the two polarities:
/// every rule carrying this code is one of the rules that capture the
/// control plane, and control-plane frames on this link are untagged. So
/// under "absent" those rules capture what they were written for, and
/// under the opposite polarity **every one of them is dead** and the
/// control plane is captured by nothing.
///
/// ⚠ Falsifier, and it is a single frame: a tagged frame that a rule
/// carrying this code should still match. Under the reading here that
/// rule cannot match it; under the other one it must.
///
/// ⛔ All fifteen sites that emit this code carry one of
/// [`TAG_FIELD_CODES`]. Over any other field code the engine refuses,
/// rather than letting an unexplained operator become an unconditional
/// match on a field whose presence it cannot even ask about.
const OP_TAG_ABSENCE: u8 = 6;

/// Whether a frame carries the field a clause names.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Presence {
    Present,
    Absent,
    /// The model cannot say. Never collapsed into either of the others:
    /// the two answers are opposite verdicts.
    Unknown,
}

/// Does this frame carry that field?
///
/// The interesting case is the tag codes. On an **untagged** frame both
/// are absent, whichever one is the service tag and whichever the
/// customer tag — so the attribution that is not established does not
/// have to be, and every rule that only asks for their absence becomes
/// decidable. On a **tagged** frame it does have to be, and this returns
/// [`Presence::Unknown`] rather than picking one.
///
/// That is the whole shape of it: decide where the open question does not
/// matter, refuse where it does.
///
/// Only [`TAG_FIELD_CODES`] reach here from a clause — the caller refuses
/// [`OP_TAG_ABSENCE`] over anything else before asking. The other arms
/// serve the tag lookup [`extract_field`] does, and answer `Unknown`
/// rather than a guess for everything the model has no notion of presence
/// for.
fn field_presence(field: Field, frame: &[u8]) -> Presence {
    if frame.len() < ETHERNET_HEADER_LEN {
        return Presence::Unknown;
    }
    let code = field.code();
    if !TAG_FIELD_CODES.contains(&code) {
        // A frame always carries its addresses and its type field; for
        // anything else the model has no notion of presence to offer.
        return match code {
            0x00 | 0x01 | 0x02 => Presence::Present,
            _ => Presence::Unknown,
        };
    }
    let ethertype = ((frame[12] as u16) << 8) | frame[13] as u16;
    if TAG_ETHERTYPES.contains(&ethertype) {
        Presence::Unknown
    } else {
        Presence::Absent
    }
}

/// Bring a value down to a clause's compare window: `bits` bits starting
/// at `anchor`.
fn window_shift(x: u64, anchor: u32, bits: u32) -> u64 {
    let mask = if bits >= 64 { u64::MAX } else { (1u64 << bits) - 1 };
    (x >> anchor.min(63)) & mask
}

/// The rival convention: mask the window's bits and leave them where they
/// are, rather than right-aligning both sides.
///
/// Kept because it makes the equivalence testable rather than merely
/// argued — see `the_masking_convention_does_not_change_any_ordered_verdict`.
/// It is `window_shift` followed by a left shift, and since
/// `anchor + bits == 64 - shift <= 64` that shift never overflows, so it
/// is strictly monotone and preserves every ordered verdict as well as
/// the equalities.
#[cfg(test)]
fn window_place(x: u64, anchor: u32, bits: u32) -> u64 {
    window_shift(x, anchor, bits) << anchor.min(63)
}

/// Compare, over the clause's window.
///
/// Returns `None` for an operator the model will not act on — which is
/// every code whose ordering is not established by its operands. A rule
/// whose clauses all carry such a code would otherwise match every frame
/// and route it, and that is the failure mode this refusal exists for:
/// not a missing answer, a confident wrong one.
fn compare(op: Op, value: u64, operand: u64, anchor: u32, bits: u32) -> Option<bool> {
    let (v, o) = (window_shift(value, anchor, bits), window_shift(operand, anchor, bits));
    match op {
        Op::Eq => Some(v == o),
        Op::Ne => Some(v != o),
        Op::Le => Some(v <= o),
        Op::Ge => Some(v >= o),
        // Emitted by software, meaning not established. The one exception
        // is [`OP_TAG_ABSENCE`], which never reaches this function: it is
        // answered from the frame's shape in [`Engine::test_clause`],
        // never from a value.
        Op::Observed(_) | Op::Unknown(_) => None,
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
    /// A rule matched, and here is **everything** it says to do. A rule
    /// carries a list — forward, edit a tag, pick a queue — and only one
    /// entry of that list names a queue. Returning a single action would
    /// hide it: the queue result is the last of the three in every rule
    /// software builds for its user-facing port.
    Match { priority: u8, actions: Vec<Action> },
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
    /// A clause asking whether a field is there, on a frame where the
    /// answer depends on which of two codes names which tag.
    pub undecidable_presence: u64,
    /// A clause whose two window boundaries leave no bits to compare.
    pub undecidable_window: u64,
    /// A clause with an operator the model will not act on.
    pub undecidable_operator: u64,
    /// An entry the decoder could not read as a clause.
    pub undecidable_entry: u64,
    /// Rules that ran off the end of their table without terminating.
    pub unterminated_rules: u64,
    /// The bound table held no rules at all.
    pub no_rules: u64,
}

impl EngineCounters {
    /// Every frame classified left exactly one verdict behind.
    pub fn verdicts_accounted_for(&self) -> bool {
        self.frames_classified
            == self.matched
                + self.no_match
                + self.undecidable_field
                + self.undecidable_presence
                + self.undecidable_window
                + self.undecidable_operator
                + self.undecidable_entry
                + self.unterminated_rules
                + self.no_rules
    }

    /// Fold another tally into this one, field by field.
    pub fn add(&mut self, other: &Self) {
        self.frames_classified += other.frames_classified;
        self.matched += other.matched;
        self.no_match += other.no_match;
        self.undecidable_field += other.undecidable_field;
        self.undecidable_presence += other.undecidable_presence;
        self.undecidable_window += other.undecidable_window;
        self.undecidable_operator += other.undecidable_operator;
        self.undecidable_entry += other.undecidable_entry;
        self.unterminated_rules += other.unterminated_rules;
        self.no_rules += other.no_rules;
    }
}

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
        let rule = match port.rule(start.table, start.index).result {
            Ok(rule) => rule,
            Err(RuleError::NotAClause) => {
                counters.undecidable_entry += 1;
                return Verdict::Undecidable { reason: "entry is not a clause" };
            }
            Err(RuleError::RanPastTheEnd) => {
                counters.unterminated_rules += 1;
                return Verdict::Undecidable { reason: "rule ran past the last entry" };
            }
            Err(RuleError::TooLong) => {
                counters.unterminated_rules += 1;
                return Verdict::Undecidable { reason: "rule longer than any observed one" };
            }
        };

        let mut all_matched = true;
        for clause in &rule.clauses {
            match Self::test_clause(clause, frame) {
                Err(reason) => {
                    match reason {
                        "field has no established frame field" => counters.undecidable_field += 1,
                        "field presence is not established" => counters.undecidable_presence += 1,
                        "clause window is empty" => counters.undecidable_window += 1,
                        _ => counters.undecidable_operator += 1,
                    }
                    return Verdict::Undecidable { reason };
                }
                Ok(matched) => all_matched &= matched,
            }
        }

        if all_matched {
            counters.matched += 1;
            // Every action of a rule carries the same batch tag, so the
            // first one's stands for the rule's.
            let priority = rule.actions.first().map_or(0, |a| a.priority);
            Verdict::Match { priority, actions: rule.actions }
        } else {
            counters.no_match += 1;
            Verdict::NoMatch
        }
    }

    /// One clause against one frame. `Err` carries why it could not be
    /// decided — the caller turns that into a counter.
    pub(super) fn test_clause(clause: &Clause, frame: &[u8]) -> Result<bool, &'static str> {
        // Presence is not a comparison: no window, no operand, no value.
        // The one unexplained operator this engine acts on, and only over
        // the two field codes every one of its emission sites carries.
        if clause.op.code() == OP_TAG_ABSENCE {
            if !TAG_FIELD_CODES.contains(&clause.field.code()) {
                return Err("operator is not one the model acts on");
            }
            return match field_presence(clause.field, frame) {
                Presence::Present => Ok(false),
                Presence::Absent => Ok(true),
                Presence::Unknown => Err("field presence is not established"),
            };
        }
        // Two boundaries that meet or cross leave nothing to compare.
        // No observed clause is shaped that way; one that is describes
        // no comparison, and answering would be inventing one.
        if clause.compared_bits() == 0 {
            return Err("clause window is empty");
        }
        let value = extract_field(clause.field, frame)
            .ok_or("field has no established frame field")?;
        compare(
            clause.op,
            value,
            clause.operand,
            clause.anchor as u32,
            clause.compared_bits(),
        )
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
        let mut best: Option<(u8, Vec<Action>)> = None;
        for start in rules {
            match Self::evaluate_rule(port, *start, frame, &mut per_rule) {
                Verdict::Match { priority, actions } => {
                    if best.as_ref().is_none_or(|(p, _)| priority > *p) {
                        best = Some((priority, actions));
                    }
                }
                Verdict::NoMatch => {}
                Verdict::Undecidable { reason } => {
                    // Carry the per-rule tally so the reason survives.
                    counters.undecidable_field += per_rule.undecidable_field;
                    counters.undecidable_presence += per_rule.undecidable_presence;
                    counters.undecidable_window += per_rule.undecidable_window;
                    counters.undecidable_operator += per_rule.undecidable_operator;
                    counters.undecidable_entry += per_rule.undecidable_entry;
                    counters.unterminated_rules += per_rule.unterminated_rules;
                    return Verdict::Undecidable { reason };
                }
            }
        }
        match best {
            Some((priority, actions)) => {
                counters.matched += 1;
                Verdict::Match { priority, actions }
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
    use super::super::rule::{ActionPayload, Entry, Field, Link, Rule, LINK_AND_NEXT};
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

    /// A field a frame does not carry must not be guessed at, and the
    /// refusal must leave a number behind. This is the state a live rule
    /// set is actually in for codes `0x03`, `0x04` and `0x0F`.
    ///
    /// The three are refused for two different reasons, and the
    /// difference matters: `0x03`/`0x04` are refused because *which tag
    /// is which* is unexplained, `0x0F` because its meaning IS explained
    /// (see [`LINK_INDEX`]) and the value simply is not in the bytes.
    /// Both are refusals; only the second one names its missing input.
    #[test]
    fn a_field_the_frame_does_not_carry_is_undecidable_and_counted() {
        let mut port = LuePort::new();
        // `0x03` and `0x04` have a presence but no value: a rule may ask
        // whether a tag is there, and this model will not invent what is
        // in it. `0x0F` is the link index, which is link metadata.
        for code in [0x03u8, 0x04, 0x0F] {
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

    /// The extractor the firmware programs, against the two rules that
    /// use it — the test the reading could fail.
    ///
    /// One rule bounds it to `[2, 6]` under EtherType `0x8808`, the other
    /// selects its top octet with an anchor of 8 under `0x8809`. Read as
    /// the two octets after the type field, both discriminate; the
    /// negative controls are what makes that a measurement rather than a
    /// restatement.
    #[test]
    fn the_default_extractor_reads_the_two_octets_after_the_type_field() {
        let mut frame = frame_with_ethertype(0x8808);
        for (word, opcode_in_range) in [(2u16, true), (6, true), (1, false), (7, false)] {
            frame[14] = (word >> 8) as u8;
            frame[15] = word as u8;
            let lower = Clause {
                field: Field::from_code(0x10),
                op: Op::Ge,
                shift: 0x30,
                anchor: 0,
                operand: 2,
            };
            let upper = Clause { op: Op::Le, operand: 6, ..lower };
            let got = Engine::test_clause(&lower, &frame).unwrap()
                && Engine::test_clause(&upper, &frame).unwrap();
            assert_eq!(got, opcode_in_range, "opcode {word}");
        }

        // The anchored form: the top octet only, whatever sits under it.
        let subtype = Clause {
            field: Field::from_code(0x10),
            op: Op::Eq,
            shift: 0x30,
            anchor: 8,
            operand: 0x0300,
        };
        let mut slow = frame_with_ethertype(0x8809);
        for (hi, lo, expected) in [(0x03u8, 0x00u8, true), (0x03, 0xFF, true), (0x01, 0x00, false)]
        {
            slow[14] = hi;
            slow[15] = lo;
            assert_eq!(Engine::test_clause(&subtype, &slow), Ok(expected), "{hi:#04x}{lo:02x}");
        }

        // A tagged frame moves the window by an amount nothing pins, and
        // the model must refuse rather than read the wrong two octets.
        let mut tagged = frame_with_ethertype(0x8100);
        tagged[14] = 0;
        tagged[15] = 2;
        assert!(Engine::test_clause(&subtype, &tagged).is_err());

        // And the extractors that are allocated but never programmed
        // stay unreadable — one measured descriptor, not seven.
        for code in [0x11u8, 0x12, 0x16] {
            let c = Clause { field: Field::from_code(code), ..subtype };
            assert!(Engine::test_clause(&c, &slow).is_err(), "field {code:#04x}");
        }
    }

    /// The two window ends, against the clauses that establish them.
    ///
    /// These are address matches software programs, and each decodes to
    /// a textbook mapping and to nothing else — which is what pins the
    /// window to `[63 - shift : anchor]` rather than to any other
    /// reading of the same two numbers.
    #[test]
    fn the_window_ends_reproduce_the_address_blocks_they_encode() {
        // IPv4 multicast: the 01:00:5E block with its low 23 bits free.
        let v4 = Clause {
            field: Field::from_code(0x00),
            op: Op::Eq,
            shift: 0,
            anchor: 23,
            operand: 0x0000_0100_5E00_0100,
        };
        assert_eq!(v4.compared_bits(), 41);
        // IPv6 multicast: the 33:33 prefix with its low 32 bits free.
        let v6 = Clause { anchor: 32, operand: 0x0000_3333_0000_0100, ..v4 };
        assert_eq!(v6.compared_bits(), 32);
        // The group bit of the first octet, on its own.
        let group = Clause { shift: 23, anchor: 40, operand: 0x0000_0100_0000_0000, ..v4 };
        assert_eq!(group.compared_bits(), 1);

        let mac = |m: [u8; 6]| m.iter().fold(0u64, |a, &b| (a << 8) | b as u64);
        for (clause, addr, expected, what) in [
            (v4, [0x01, 0x00, 0x5E, 0x00, 0x00, 0x00], true, "block start"),
            (v4, [0x01, 0x00, 0x5E, 0x7F, 0xFF, 0xFF], true, "block end"),
            (v4, [0x01, 0x00, 0x5E, 0x80, 0x00, 0x00], false, "just past the block"),
            (v4, [0x01, 0x00, 0x5F, 0x00, 0x00, 0x00], false, "neighbouring prefix"),
            (v6, [0x33, 0x33, 0x00, 0x00, 0x00, 0x01], true, "IPv6 block"),
            (v6, [0x33, 0x33, 0xFF, 0xFF, 0xFF, 0xFF], true, "IPv6 block end"),
            (v6, [0x33, 0x34, 0x00, 0x00, 0x00, 0x00], false, "IPv6 neighbour"),
            (group, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00], true, "group bit set"),
            (group, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00], false, "group bit clear"),
            (group, [0xFF; 6], true, "broadcast is a group address"),
        ] {
            let got = compare(
                clause.op,
                mac(addr),
                clause.operand,
                clause.anchor as u32,
                clause.compared_bits(),
            );
            assert_eq!(got, Some(expected), "{what}: {addr:02X?}");
        }
    }

    /// The anchor a live rule set actually carries: it selects the
    /// subtype octet of a slow-protocol frame out of the sixteen bits
    /// after the EtherType, which is the only reading under which the
    /// rule discriminates anything.
    #[test]
    fn an_anchored_clause_selects_the_octet_it_was_written_for() {
        let subtype = Clause {
            // The two octets after the EtherType, whatever the hardware
            // calls the code — this test is about the window, not the
            // field.
            field: Field::EtherType,
            op: Op::Eq,
            shift: 0x30,
            anchor: 8,
            operand: 0x0300,
        };
        assert_eq!(subtype.compared_bits(), 8, "one octet, not two");
        let test = |v: u64| {
            compare(subtype.op, v, subtype.operand, subtype.anchor as u32, subtype.compared_bits())
        };
        // OAM is subtype 0x03; the octet under it is the flags, and the
        // rule must not care what it holds.
        assert_eq!(test(0x0300), Some(true));
        assert_eq!(test(0x0350), Some(true), "the flags octet is outside the window");
        assert_eq!(test(0x03FF), Some(true));
        // A different slow protocol must not match.
        assert_eq!(test(0x0100), Some(false), "LACP is subtype 0x01");
        assert_eq!(test(0x0200), Some(false));
    }

    /// An anchor of zero must leave every existing clause exactly as it
    /// was: the window's low end simply sits at the bottom.
    #[test]
    fn a_zero_anchor_compares_the_same_bits_it_always_did() {
        for shift in [0u8, 0x10, 0x30, 0x38, 0x3F] {
            let c = Clause {
                field: Field::EtherType,
                op: Op::Eq,
                shift,
                anchor: 0,
                operand: 0,
            };
            assert_eq!(c.compared_bits(), 64 - shift as u32);
        }
    }

    /// Two ends that meet describe no comparison. Nothing observed is
    /// shaped that way, and answering anyway would invent a result.
    #[test]
    fn an_empty_window_is_undecidable_and_counted() {
        let mut port = LuePort::new();
        program(&mut port, 3, 0x30, Entry::Clause {
            clause: Clause {
                field: Field::EtherType,
                op: Op::Eq,
                shift: 0x30,
                anchor: 0x30,
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
        assert_eq!(c.undecidable_window, 1);
        assert!(c.verdicts_accounted_for(), "{c:?}");
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
                actions: vec![Action {
                    priority: 3,
                    payload: ActionPayload::Simple { type_code: 6 }
                }]
            },
            "highest priority wins; stop-at-first would have said 1"
        );
    }

    /// A rule whose action list does not fit in the clause entry: the
    /// first action takes the last clause's link word and the rest spill
    /// into the entry that follows. The queue result is the **third**
    /// action, so a model that reads only the first cannot see it.
    ///
    /// The three actions are the ones software builds for its user-facing
    /// port; only the last names a queue.
    #[test]
    fn the_queue_result_is_found_past_the_first_action_word() {
        let queue = Action {
            priority: 1,
            payload: ActionPayload::Selected { sel: 2, sub: 0, field: 0x010, wide: true },
        };
        let rule = Rule {
            clauses: vec![Clause {
                field: Field::EtherType,
                op: Op::Eq,
                shift: 0x30,
                anchor: 0,
                operand: 0x8808,
            }],
            actions: vec![
                Action { priority: 1, payload: ActionPayload::Simple { type_code: 7 } },
                Action { priority: 1, payload: ActionPayload::Simple { type_code: 3 } },
                queue,
            ],
        };
        assert_eq!(rule.destination_queue(), Some(0x10));

        let mut port = LuePort::new();
        for (step, quad) in rule.encode().iter().enumerate() {
            for (i, w) in quad.to_regs().iter().enumerate() {
                port.write_data(i, *w);
            }
            port.write_cmd(CMD_GO | CMD_WRITE | (8 << 12) | (0x34 + step as u32));
        }
        // Two entries: one clause, one spill. The spill entry must not be
        // mistaken for a second rule.
        assert_eq!(port.entry_count(8), 2);
        assert_eq!(port.rule_starts(8), vec![0x34]);

        let mut c = EngineCounters::default();
        let v = Engine::classify(
            &port,
            &[RuleStart { table: 8, index: 0x34 }],
            &frame_with_ethertype(0x8808),
            &mut c,
        );
        let Verdict::Match { actions, .. } = v else { panic!("expected a match, got {v:?}") };
        assert_eq!(actions.len(), 3, "the whole list survives classification");
        assert_eq!(actions.iter().find_map(Action::destination_queue), Some(0x10));
        // The negative control: the first action names no queue at all.
        assert_eq!(actions[0].destination_queue(), None);
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

    /// The size-bucket field reads the frame's own length.
    ///
    /// The boundaries are the ones the live rule set compares against —
    /// 64, 65..127, 128, 256 — so a frame of each size must land in its
    /// own bucket and nowhere else. Read as anything but a length, the
    /// four comparands stop lining up.
    #[test]
    fn the_size_bucket_field_is_the_frame_length() {
        for len in [ETHERNET_HEADER_LEN, 0x40, 0x41, 0x7F, 0x80, 0x100] {
            let frame = vec![0u8; len];
            assert_eq!(
                extract_field(Field::from_code(FRAME_LENGTH), &frame),
                Some(len as u64),
                "length {len}"
            );
        }
        // Below an Ethernet header there is no frame to measure.
        assert_eq!(extract_field(Field::from_code(FRAME_LENGTH), &[0u8; 8]), None);
    }

    /// The operators software emits but nothing explains are refused, and
    /// the refusal is counted rather than silently turned into a verdict.
    ///
    /// This is the behaviour that matters most. Two of these codes used
    /// to answer unconditionally — one always false, one always true —
    /// and the second is the dangerous one: a rule whose every clause
    /// carried it matched every frame and routed it.
    #[test]
    fn the_operators_the_model_will_not_act_on_are_refused_and_counted() {
        let frame = frame_with_ethertype(0x8809);
        for code in [0u8, 5, 7] {
            let mut port = LuePort::new();
            program(&mut port, 3, 0x80, Entry::Clause {
                clause: Clause {
                    field: Field::EtherType,
                    op: Op::from_code(code),
                    shift: 0x30,
                    anchor: 0,
                    operand: 0x8809,
                },
                link: Link::Terminal(Action {
                    priority: 0,
                    payload: ActionPayload::simple(6).unwrap(),
                }),
            });
            let mut c = EngineCounters::default();
            let v = Engine::classify(&port, &[RuleStart { table: 3, index: 0x80 }], &frame, &mut c);
            assert!(matches!(v, Verdict::Undecidable { .. }), "operator {code}: {v:?}");
            assert_eq!(c.undecidable_operator, 1, "operator {code}");
            assert_eq!(c.matched, 0, "operator {code} must never match");
            assert!(c.verdicts_accounted_for(), "operator {code}: {c:?}");
        }

        // The negative control: the same clause under an established
        // operator does decide, and decides yes. Without it, "refused" is
        // indistinguishable from "the fixture never matched anything".
        let mut port = LuePort::new();
        program(&mut port, 3, 0x80, ethertype_rule(0x8809, 2));
        let mut c = EngineCounters::default();
        let v = Engine::classify(&port, &[RuleStart { table: 3, index: 0x80 }], &frame, &mut c);
        assert!(matches!(v, Verdict::Match { .. }), "{v:?}");
        assert_eq!(c.undecidable_operator, 0);
    }

    /// The one unexplained operator the engine does act on is confined to
    /// the two field codes every one of its emission sites carries.
    #[test]
    fn the_tag_absence_operator_applies_only_to_the_tag_field_codes() {
        let untagged = frame_with_ethertype(0x8809);
        for code in TAG_FIELD_CODES {
            let clause = Clause {
                field: Field::from_code(code),
                op: Op::from_code(OP_TAG_ABSENCE),
                shift: 0x3F,
                anchor: 0,
                operand: 0,
            };
            assert_eq!(Engine::test_clause(&clause, &untagged), Ok(true), "code {code:#04x}");
        }

        // Any other field code, same operator: refused. Before this, the
        // presence path answered `Present` for the address and type
        // codes, which turned the clause into an unconditional false and
        // an unexplained operator into a decided verdict.
        for code in [0x00u8, 0x01, 0x02, 0x10, 0x24, 0x0F] {
            let clause = Clause {
                field: Field::from_code(code),
                op: Op::from_code(OP_TAG_ABSENCE),
                shift: 0x3F,
                anchor: 0,
                operand: 0,
            };
            assert_eq!(
                Engine::test_clause(&clause, &untagged),
                Err("operator is not one the model acts on"),
                "code {code:#04x}"
            );
        }

        // And on a tagged frame the tag codes go back to being refused,
        // because which of the two names which tag is not established.
        for tpid in TAG_ETHERTYPES {
            let tagged = frame_with_ethertype(tpid);
            let clause = Clause {
                field: Field::from_code(TAG_FIELD_CODES[0]),
                op: Op::from_code(OP_TAG_ABSENCE),
                shift: 0x3F,
                anchor: 0,
                operand: 0,
            };
            assert_eq!(
                Engine::test_clause(&clause, &tagged),
                Err("field presence is not established"),
                "{tpid:#06x}"
            );
        }
    }

    /// The two masking conventions cannot disagree — proved by exhausting
    /// them rather than by arguing it in a comment.
    ///
    /// Right-aligning both sides and masking them in place differ by a
    /// left shift that never overflows, which is monotone, so it
    /// preserves the ordered operators as well as the equalities. This
    /// was carried as an open question for a long time; it is not one.
    #[test]
    fn the_masking_convention_does_not_change_any_ordered_verdict() {
        let values: [u64; 8] = [
            0,
            1,
            0x8809,
            0x0180_C200_0001,
            0x0000_FFFF_FFFF_FFFF,
            0xFFFF_FFFF_FFFF_FFFF,
            0x1234_5678_9ABC_DEF0,
            0x8000_0000_0000_0000,
        ];
        let mut pairs = 0u32;
        for shift in [0u8, 8, 0x10, 0x23, 0x30, 0x38, 0x3E] {
            for anchor in [0u8, 1, 5, 8, 0x17, 0x20, 0x28] {
                let clause = Clause {
                    field: Field::EtherType,
                    op: Op::Eq,
                    shift,
                    anchor,
                    operand: 0,
                };
                let bits = clause.compared_bits();
                if bits == 0 {
                    continue;
                }
                let a = clause.anchor as u32;
                assert!(a + bits <= 64, "the window must fit in 64 bits");
                for &value in &values {
                    for &operand in &values {
                        for op in [Op::Eq, Op::Ne, Op::Le, Op::Ge] {
                            let right = compare(op, value, operand, a, bits)
                                .expect("an established operator always decides");
                            let (v, o) =
                                (window_place(value, a, bits), window_place(operand, a, bits));
                            let placed = match op {
                                Op::Eq => v == o,
                                Op::Ne => v != o,
                                Op::Le => v <= o,
                                Op::Ge => v >= o,
                                other => panic!("not an established operator: {other:?}"),
                            };
                            assert_eq!(
                                right, placed,
                                "shift {shift:#04x} anchor {anchor:#04x} {op:?} \
                                 {value:#018x} vs {operand:#018x}"
                            );
                            pairs += 1;
                        }
                    }
                }
            }
        }
        assert!(pairs > 1000, "the sweep must actually sweep: {pairs}");
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
