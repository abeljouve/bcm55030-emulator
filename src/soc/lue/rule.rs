//! Rule entries: what the four words of a table entry mean.
//!
//! An entry is either a **clause** — a field, a comparison, and a
//! 64-bit operand — or something this decoder does not recognise, which
//! stays [`Entry::Raw`]. The port must round-trip any 128 bits it is
//! given: tables the readback path never touches carry a format nothing
//! here has ever seen, and losing them would be a silent corruption.
//!
//! Clause header layout, all four fields confirmed against the readback
//! of a live rule set:
//!
//! ```text
//!  31    28 27  22 21   16 15    10 9      4 3    0
//! +--------+------+-------+--------+--------+------+
//! |  0xF   |  0   | shift | anchor | field  |  op  |
//! +--------+------+-------+--------+--------+------+
//! ```
//!
//! `shift` says how much of the operand is compared: the field is
//! materialised right-aligned in 64 bits and the low `64 - shift` bits
//! take part. The operand spans two words, most significant first —
//! which puts its high half at the *higher* address, same inversion as
//! [`Quad`].

use super::port::Quad;

/// Marker in the top nibble of a clause header.
const CLAUSE_TAG: u32 = 0xF;

/// Word that means "another clause follows, and both must match".
pub const LINK_AND_NEXT: u32 = 0x1000_0000;

/// Comparison operator, four bits.
///
/// -- INFERRED, strongly: the four values a live rule set uses
/// (`1`, `3`, `4`, `6`) line up with the operator table of the classifier
/// specification. The other four are **predictions**, and falsifiable:
/// nothing observed exercises them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Never,
    Eq,
    Ne,
    Le,
    Ge,
    Exists,
    NotExists,
    Always,
    /// Operator codes are four bits wide but only eight are named.
    Unknown(u8),
}

impl Op {
    pub fn from_code(code: u8) -> Self {
        match code & 0xF {
            0 => Op::Never,
            1 => Op::Eq,
            2 => Op::Ne,
            3 => Op::Le,
            4 => Op::Ge,
            5 => Op::Exists,
            6 => Op::NotExists,
            7 => Op::Always,
            other => Op::Unknown(other),
        }
    }

    pub fn code(self) -> u8 {
        match self {
            Op::Never => 0,
            Op::Eq => 1,
            Op::Ne => 2,
            Op::Le => 3,
            Op::Ge => 4,
            Op::Exists => 5,
            Op::NotExists => 6,
            Op::Always => 7,
            Op::Unknown(n) => n & 0xF,
        }
    }
}

/// Which field of the frame a clause looks at, six bits.
///
/// ⛔ The hardware **renumbers**: these are not the field codes of the
/// classifier specification. Five codes appear in a live rule set, and
/// only one of them is pinned to a frame field by evidence — `0x02`,
/// which is compared against `0x8809`, `0x8808`, `0x888E` and `0x8180`.
/// The rest keep their number and say so.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Field {
    /// Compared against EtherType values in every observed use.
    EtherType,
    /// Codes seen in a live rule set whose frame field is not
    /// established. Kept distinct from `Unknown` so that "seen but
    /// unexplained" does not read as "never seen".
    Observed(u8),
    Unknown(u8),
}

impl Field {
    /// Codes a live rule set uses, beyond the one that is pinned.
    const OBSERVED_CODES: [u8; 4] = [0x03, 0x04, 0x0F, 0x10];

    pub fn from_code(code: u8) -> Self {
        let code = code & 0x3F;
        if code == 0x02 {
            Field::EtherType
        } else if Self::OBSERVED_CODES.contains(&code) {
            Field::Observed(code)
        } else {
            Field::Unknown(code)
        }
    }

    pub fn code(self) -> u8 {
        match self {
            Field::EtherType => 0x02,
            Field::Observed(n) | Field::Unknown(n) => n & 0x3F,
        }
    }
}

/// One comparison.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clause {
    pub field: Field,
    pub op: Op,
    /// Bits [21:16]. The high end of the compare window: the bit above
    /// it and everything over it are don't-care.
    pub shift: u8,
    /// Bits [15:10]. Where the compare window starts.
    ///
    /// -- OBSERVED that it is the window's **low end**: software emits
    /// it paired with a comparand shifted up by the same amount, so a
    /// clause with `anchor 23` against an address block leaves the low
    /// 23 bits out of the comparison, and one with `anchor 8` against a
    /// two-octet field compares only the upper octet. Every observed
    /// clause decodes to something coherent under that reading and to
    /// nothing under an anchor of zero.
    ///
    /// ⛔ **The window's high end is NOT established.** Two readings
    /// survive every clause the firmware emits:
    ///
    /// * `[63 - shift : anchor]`, width `64 - shift - anchor` — what
    ///   this model implements.
    /// * `[anchor + 63 - shift : anchor]`, width `64 - shift` — a
    ///   fixed-width window slid up by the anchor.
    ///
    /// They agree on every observed clause because each one either has
    /// `shift == 0` (both windows then reach the top) or compares a
    /// field too narrow to reach the bits where they differ. An earlier
    /// reading of this file claimed the first was established by three
    /// arguments; an adversarial re-check found all three
    /// non-discriminating, and one of them read two mutually exclusive
    /// branches as if both ran. Do not restore that claim.
    ///
    /// What separates them: a clause over a field that can carry bits
    /// above `anchor + 64 - shift` — field `0x1B` is tested at
    /// `shift 0x3A, anchor 5`, so a value with bit 6 set answers
    /// differently under the two.
    ///
    /// ⛔ Also not established: whether the hardware masks the bits
    /// below the anchor in place or right-aligns both sides. The two
    /// agree on `==` and `!=`, and no observed clause pairs a non-zero
    /// anchor with an ordered operator. This model right-aligns; see
    /// [`super::engine`] for the experiment.
    pub anchor: u8,
    pub operand: u64,
}

impl Clause {
    /// Width of the compare window, in bits.
    pub fn compared_bits(&self) -> u32 {
        64u32
            .saturating_sub(self.shift as u32)
            .saturating_sub(self.anchor as u32)
    }

    pub fn header(&self) -> u32 {
        (CLAUSE_TAG << 28)
            | ((self.shift as u32 & 0x3F) << 16)
            | ((self.anchor as u32 & 0x3F) << 10)
            | ((self.field.code() as u32) << 4)
            | self.op.code() as u32
    }

    pub fn from_header(header: u32, operand: u64) -> Option<Self> {
        if header >> 28 != CLAUSE_TAG {
            return None;
        }
        Some(Self {
            field: Field::from_code(((header >> 4) & 0x3F) as u8),
            op: Op::from_code((header & 0xF) as u8),
            shift: ((header >> 16) & 0x3F) as u8,
            anchor: ((header >> 10) & 0x3F) as u8,
            operand,
        })
    }
}

/// What follows a clause inside its entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Link {
    /// Another clause follows at the next index; both must match.
    AndNext,
    /// The rule ends here and this word is its first action word.
    Terminal(Action),
}

/// An action word.
///
/// The encoding is fully known. Of the meanings, exactly one is
/// established and it is the one routing needs: see
/// [`Action::destination_queue`]. The rest are carried, not named.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Action {
    /// Bits [27:24].
    ///
    /// ⛔ Called a priority here because that is how the engine uses it,
    /// but software sets it from a **per-batch tag** chosen by whoever
    /// programmed the rule, not from anything frame-shaped. Whether the
    /// hardware treats it as a precedence at all is **not established**.
    /// What would settle it: two overlapping rules with different tags
    /// and one frame that matches both.
    pub priority: u8,
    pub payload: ActionPayload,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActionPayload {
    /// The `type == 0` encoding: top nibble `0xE`, no other field.
    Broadcast,
    /// Types 2..=7, carried as `type ^ 1` in bits [22:20].
    Simple { type_code: u8 },
    /// Types 0x08..=0x5B: a selector in the top nibble, a sub-index, and
    /// a field whose width depends on the selector.
    Selected { sel: u8, sub: u8, field: u16, wide: bool },
    /// Anything else. Kept verbatim so it round-trips.
    Raw(u32),
}

/// Selector nibble of the group the queue result belongs to.
const SELECTED_QUEUE_SEL: u8 = 2;
/// Sub-index within that group. Software's result codes run from this
/// one upwards through the group, so the neighbours are other results
/// entirely — not other queues.
const SELECTED_QUEUE_SUB: u8 = 0;

impl Action {
    /// The queue this action sends a frame to, if it names one.
    ///
    /// -- OBSERVED, and the only action meaning that is. Three things,
    /// none of which is a specification read off and called hardware:
    ///
    /// * The encoding is re-derived from the image: the action encoder
    ///   dispatches on the type through a table whose every entry was
    ///   recomputed, and this one lands in the group and sub-index used
    ///   here.
    /// * At every call site that emits this type, the value handed to it
    ///   is written `pin & 0x1F`, and those pin numbers are the ones the
    ///   receive path polls its queues by. Queue numbers are gathered
    ///   elsewhere into `1 << (pin & 0x1F)` bitmaps, which is where the
    ///   five bits come from.
    /// * Read back over a boot, the rules carrying this type are exactly
    ///   the ones software builds for its user-facing port, and all of
    ///   them name the same queue — the one the control path dequeues.
    ///
    /// ⚠ The published result-code table this type lines up with was
    /// **not** used to establish it: an adversarial re-check found the
    /// name column to be a read-off, and the "nothing left over" closure
    /// argument to be an artifact of the window it was counted in. What
    /// survived is the structure and the call sites, which is what the
    /// three points above rest on.
    ///
    /// ⛔ Every **other** `Selected` action is a different result — a
    /// scheduling index, a VLAN edit, a bitmask — and returning a queue
    /// for one of those would route frames by a number that is not a
    /// queue. A live rule set is full of them: of 84 selected actions
    /// counted over a boot, **none** was this one.
    pub fn destination_queue(&self) -> Option<u8> {
        match self.payload {
            ActionPayload::Selected { sel, sub, field, .. }
                if sel == SELECTED_QUEUE_SEL && sub == SELECTED_QUEUE_SUB =>
            {
                Some((field & 0x1F) as u8)
            }
            _ => None,
        }
    }

    pub fn encode(&self) -> u32 {
        let prio = ((self.priority as u32) & 0xF) << 24;
        match self.payload {
            ActionPayload::Broadcast => 0xE000_0000 | prio,
            ActionPayload::Simple { type_code } => {
                (((type_code ^ 1) as u32 & 0x7) << 20) | prio
            }
            ActionPayload::Selected { sel, sub, field, wide } => {
                let shift = if wide { 12 } else { 16 };
                ((sel as u32 & 0xF) << 28)
                    | (field as u32)
                    | ((sub as u32 & 0xF) << shift)
                    | prio
            }
            ActionPayload::Raw(w) => w,
        }
    }

    pub fn decode(word: u32) -> Self {
        let priority = ((word >> 24) & 0xF) as u8;
        let sel = (word >> 28) as u8;
        // Bits [23:20] outside the priority field, for the simple form.
        let simple = ((word >> 20) & 0x7) as u8;
        let payload = if sel == 0xE && word & 0x00FF_FFFF == 0 && (word >> 20) & 0xF == 0 {
            ActionPayload::Broadcast
        } else if sel == 0 && word & 0x000F_FFFF == 0 && simple != 0 {
            ActionPayload::Simple { type_code: simple ^ 1 }
        } else if (2..=9).contains(&sel) {
            // The two widths differ in where the sub-index sits; a
            // selector below 8 is the wide form.
            let wide = sel < 8;
            let shift = if wide { 12 } else { 16 };
            let sub = ((word >> shift) & 0xF) as u8;
            let field = (word & ((1 << shift) - 1)) as u16;
            ActionPayload::Selected { sel, sub, field, wide }
        } else {
            return Self { priority, payload: ActionPayload::Raw(word) };
        };
        let candidate = Self { priority, payload };
        // Only claim a structured reading if it reproduces the word.
        if candidate.encode() == word {
            candidate
        } else {
            Self { priority, payload: ActionPayload::Raw(word) }
        }
    }
}

/// A decoded table entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Entry {
    Clause { clause: Clause, link: Link },
    /// Not a clause, or not one this decoder recognises. The words are
    /// kept exactly.
    Raw(Quad),
}

impl Entry {
    pub fn decode(quad: Quad) -> Self {
        let operand = ((quad.words[1] as u64) << 32) | quad.words[2] as u64;
        match Clause::from_header(quad.words[0], operand) {
            Some(clause) => {
                let link = if quad.words[3] == LINK_AND_NEXT {
                    Link::AndNext
                } else {
                    Link::Terminal(Action::decode(quad.words[3]))
                };
                Entry::Clause { clause, link }
            }
            None => Entry::Raw(quad),
        }
    }

    pub fn encode(&self) -> Quad {
        match self {
            Entry::Clause { clause, link } => Quad::new([
                clause.header(),
                (clause.operand >> 32) as u32,
                clause.operand as u32,
                match link {
                    Link::AndNext => LINK_AND_NEXT,
                    Link::Terminal(a) => a.encode(),
                },
            ]),
            Entry::Raw(q) => *q,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The eight distinct clauses a live rule set contains, built from
    /// the constructors rather than pasted.
    fn observed_clauses() -> Vec<Clause> {
        vec![
            Clause { field: Field::from_code(0x02), op: Op::Eq, shift: 0x30, anchor: 0, operand: 0x8809 },
            Clause { field: Field::from_code(0x02), op: Op::Eq, shift: 0x30, anchor: 0, operand: 0x8808 },
            Clause { field: Field::from_code(0x02), op: Op::Eq, shift: 0x00, anchor: 0, operand: 0x888E },
            Clause { field: Field::from_code(0x03), op: Op::NotExists, shift: 0x3F, anchor: 0, operand: 0 },
            Clause { field: Field::from_code(0x04), op: Op::NotExists, shift: 0x3F, anchor: 0, operand: 0 },
            Clause { field: Field::from_code(0x10), op: Op::Eq, shift: 0x30, anchor: 8, operand: 0x0300 },
            Clause { field: Field::from_code(0x0F), op: Op::Ge, shift: 0x38, anchor: 0, operand: 0x20 },
            Clause { field: Field::from_code(0x10), op: Op::Le, shift: 0x30, anchor: 0, operand: 6 },
        ]
    }

    /// The headers those clauses have in the readback of a live rule
    /// set. This is the anchor: without it the codec would only be
    /// checked against itself.
    const OBSERVED_HEADERS: [u32; 8] = [
        0xF030_0021, 0xF030_0021, 0xF000_0021, 0xF03F_0036,
        0xF03F_0046, 0xF030_2101, 0xF038_00F4, 0xF030_0103,
    ];

    #[test]
    fn clause_headers_match_the_ones_hardware_returned() {
        for (clause, expected) in observed_clauses().iter().zip(OBSERVED_HEADERS) {
            assert_eq!(clause.header(), expected, "for {clause:?}");
        }
    }

    #[test]
    fn every_observed_clause_round_trips() {
        for clause in observed_clauses() {
            let entry = Entry::Clause { clause, link: Link::AndNext };
            assert_eq!(Entry::decode(entry.encode()), entry);
        }
    }

    /// Six operators and five field codes, all reached through the
    /// public constructors.
    #[test]
    fn operator_and_field_codes_round_trip() {
        for code in 0..16u8 {
            assert_eq!(Op::from_code(code).code(), code);
        }
        for code in 0..64u8 {
            assert_eq!(Field::from_code(code).code(), code);
        }
        assert_eq!(Field::from_code(0x02), Field::EtherType);
        assert!(matches!(Field::from_code(0x10), Field::Observed(0x10)));
        assert!(matches!(Field::from_code(0x3F), Field::Unknown(0x3F)));
    }

    /// `shift` is how much of the operand is *not* compared: a shift of
    /// zero compares all 64 bits, and `0x30` compares sixteen. That is
    /// the only reading under which both observed EtherType clauses make
    /// sense at once.
    #[test]
    fn the_shift_field_says_how_wide_the_comparison_is() {
        let wide = observed_clauses()[2];
        assert_eq!(wide.compared_bits(), 64);
        let narrow = observed_clauses()[0];
        assert_eq!(narrow.compared_bits(), 16);
    }

    /// A live rule set ends every one of its six rules with the same
    /// action type at two different priorities — so the four priority
    /// bits are measured, not assumed.
    #[test]
    fn the_two_observed_action_words_decode_to_one_type_at_two_priorities() {
        let a = Action::decode(0x0270_0000);
        let b = Action::decode(0x0070_0000);
        assert_eq!(a.payload, ActionPayload::Simple { type_code: 6 });
        assert_eq!(b.payload, ActionPayload::Simple { type_code: 6 });
        assert_eq!(a.priority, 2);
        assert_eq!(b.priority, 0);
        assert_eq!(a.encode(), 0x0270_0000);
        assert_eq!(b.encode(), 0x0070_0000);
    }

    /// Anything the decoder cannot explain must survive untouched: the
    /// tables the readback path never reaches hold a format nothing here
    /// has seen.
    #[test]
    fn an_unrecognised_entry_survives_verbatim() {
        for words in [
            [0x0123_4567u32, 0x89AB_CDEF, 0x0000_0000, 0xFFFF_FFFF],
            [0x0000_0000, 0, 0, 0],
            [0xE000_0000, 0xDEAD_BEEF, 0xCAFE_F00D, 0x5A5A_5A5A],
        ] {
            let q = Quad::new(words);
            let entry = Entry::decode(q);
            assert!(matches!(entry, Entry::Raw(_)));
            assert_eq!(entry.encode(), q);
        }
    }

    /// Every action word must round-trip, decoded or not — an action the
    /// decoder misreads would be written back wrong.
    #[test]
    fn every_action_word_round_trips_even_undecoded() {
        for word in [
            0x0270_0000u32, 0x0070_0000, 0x0F60_0000, 0xE000_0000, 0xE300_0000,
            0x2001_2345, 0x8F01_2345, 0x1000_0000, 0xFFFF_FFFF, 0x0000_0001,
        ] {
            assert_eq!(Action::decode(word).encode(), word, "for {word:#010x}");
        }
    }

    /// A terminal entry carries the action; a linked one carries the
    /// link word and nothing else.
    #[test]
    fn a_rule_chains_until_a_word_that_is_not_the_link() {
        let clause = observed_clauses()[0];
        let linked = Entry::Clause { clause, link: Link::AndNext };
        assert_eq!(linked.encode().words[3], LINK_AND_NEXT);

        let terminal = Entry::Clause {
            clause,
            link: Link::Terminal(Action {
                priority: 2,
                payload: ActionPayload::Simple { type_code: 6 },
            }),
        };
        assert_eq!(terminal.encode().words[3], 0x0270_0000);
        assert_eq!(Entry::decode(terminal.encode()), terminal);
    }
}
