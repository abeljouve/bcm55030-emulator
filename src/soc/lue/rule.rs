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

/// Does this word carry the clause tag?
///
/// Deliberately weaker than [`Clause::from_header`], which also refuses
/// the bits nothing ever writes. This one only **bounds** — it answers
/// "could an action word be here", and no action word can carry the tag —
/// while the other one **decodes**. Keeping the two predicates in one
/// place is what stops them drifting apart in silence.
#[inline]
pub const fn looks_like_clause_tag(word: u32) -> bool {
    word >> 28 == CLAUSE_TAG
}

/// Bits [27:22] of a clause header. Nothing observed ever sets them, so a
/// word that does is not a clause this decoder can round-trip.
const CLAUSE_RESERVED: u32 = 0x0FC0_0000;

/// Word that means "another clause follows, and both must match".
pub const LINK_AND_NEXT: u32 = 0x1000_0000;

/// Comparison operator, four bits.
///
/// Four codes are established by their operands — the values a rule
/// compares against say what the comparison is, without any table being
/// read off. The other four codes software emits are **not** explained,
/// and are kept apart from the ones it never emits at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    /// Code 1. Established by its operands: a reserved control multicast
    /// address, and the EtherType values rules compare against.
    Eq,
    /// Code 2. Established: the same constants appear under [`Op::Eq`]
    /// elsewhere, so the two codes ask the same question either way up.
    Ne,
    /// Code 3, an upper bound. The direction comes from the operand pairs
    /// themselves — each site carrying this code pairs with a code-4 site
    /// whose value is lower, and the two bracket a range.
    Le,
    /// Code 4, the lower bound and the other half of those pairs.
    Ge,
    /// Codes 0, 5, 6 and 7. Software **emits all four**, in one boot, and
    /// what they mean is not established. Kept distinct from
    /// [`Op::Unknown`] so that "seen but unexplained" does not read as
    /// "never seen" — the same idiom as [`Field::Observed`].
    Observed(u8),
    /// Codes 8 through 15: never seen at all.
    Unknown(u8),
}

/// The four operator codes software emits whose meaning is not
/// established.
///
/// All four appear in one boot, which is why they are a list here rather
/// than a gap: "not explained" and "not emitted" are different states,
/// and the second one is the only one that would justify a guess.
pub const OBSERVED_OP_CODES: [u8; 4] = [0, 5, 6, 7];

impl Op {
    pub const fn from_code(code: u8) -> Self {
        match code & 0xF {
            1 => Op::Eq,
            2 => Op::Ne,
            3 => Op::Le,
            4 => Op::Ge,
            0 | 5 | 6 | 7 => Op::Observed(code & 0xF),
            other => Op::Unknown(other),
        }
    }

    pub const fn code(self) -> u8 {
        match self {
            Op::Eq => 1,
            Op::Ne => 2,
            Op::Le => 3,
            Op::Ge => 4,
            Op::Observed(n) | Op::Unknown(n) => n & 0xF,
        }
    }

    /// Whether the operand ordering of this operator is established.
    pub const fn is_established(self) -> bool {
        matches!(self, Op::Eq | Op::Ne | Op::Le | Op::Ge)
    }
}

/// Which field of the frame a clause looks at, six bits.
///
/// ⛔ **There are two code spaces, not one renumbered space.** The code
/// that travels on the wire when a rule is programmed from outside is not
/// the code that ends up in a clause header; software translates between
/// them before it writes. This enum consumes the **header** code — the
/// one the hardware reads — which is the right side of that boundary and
/// the only side this model ever sees. Nothing here should be matched
/// against a code taken off the wire.
///
/// Eleven distinct codes appear in a live rule set. One is pinned to a
/// frame field by evidence — `0x02`, compared against `0x8809`, `0x8808`,
/// `0x888E` and `0x8180`. The rest keep their number and say so; what
/// [`super::engine`] is willing to answer, and why, is documented there
/// code by code.
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
    ///
    /// `0x00` and `0x01` are here because rules do compare them against
    /// addresses; `0x00` is pinned by the group addresses it is matched
    /// against — see [`super::engine`]. `0x24` is the most used code of
    /// the whole set — eighty of the hundred-odd rules carry it — and
    /// [`super::engine`] reads it as the frame length; leaving it out of
    /// this list made `from_code` call it unseen while the engine
    /// answered it, which is two files disagreeing about the same number.
    /// The rest are seen and unexplained.
    const OBSERVED_CODES: [u8; 10] =
        [0x00, 0x01, 0x03, 0x04, 0x0F, 0x10, 0x18, 0x19, 0x1B, 0x24];

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
    /// The pairing is not an inference over hard-coded constants any
    /// more. The one site that builds a clause from a descriptor loads
    /// one byte of it, passes it as the left-shift that positions the
    /// comparand, loads the **same** byte again and passes it as this
    /// field — two loads of one offset, four instructions apart. Whatever
    /// the anchor is, it is the position software put the comparand at.
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
    /// above `anchor + 64 - shift`. Field `0x1B` is the one — software
    /// emits it at `shift 0x3A, anchor 5` from one site and at
    /// `shift 0x39, anchor 6` from another, so the two readings assign
    /// that one field widths of 6 and 7 bits respectively, and a value
    /// with bit 6 set answers differently under them.
    ///
    /// ⚠ A decomposition of `shift` into a field width plus a narrowing
    /// term has been proposed as a second separator. It is **not** one,
    /// and it is not written here: it rests on the width column of a
    /// software-side table and on names given to that table by
    /// inference, not on anything the hardware consumes. The
    /// hardware is handed `shift` and `anchor`; where software got them
    /// is a fact about software.
    ///
    /// One thing does lean, and it is a design argument rather than an
    /// observation, so it settles nothing: every hard-coded pair software
    /// emits sums to at most `0x3F`, and it spends shift budget to test a
    /// single bit at a variable position — a bound the first reading
    /// needs and the second has no use for.
    ///
    /// -- What **is** now cheap: a live device assembles and prints
    /// clauses on demand. A start / clause / result / add / print
    /// sequence takes the four numbers in the clear and returns the words
    /// it built, so every question about the **encoding** is one experiment's
    /// work. Questions about the **evaluation** are not: nothing reads a
    /// classification verdict back, so the two window readings above stay
    /// separated only by an effect no instrument can see yet.
    ///
    /// -- Settled, and no longer a question: whether the hardware masks
    /// the bits below the anchor in place or right-aligns both sides
    /// **cannot change a verdict**. Right-aligning is
    /// `w(x) = (x >> anchor) & mask`; masking in place is
    /// `w(x) << anchor`. Since `anchor + bits == 64 - shift <= 64`, that
    /// left shift never overflows, and a left shift by a constant with no
    /// overflow is strictly monotone — so it preserves `==`, `!=`, `<=`
    /// and `>=` alike. This model right-aligns because it is cheaper.
    pub anchor: u8,
    pub operand: u64,
}

impl Clause {
    /// How many bits actually take part in the comparison: `64 - shift`
    /// from the top, less the `anchor` bits cut off the bottom.
    pub fn compared_bits(&self) -> u32 {
        64u32
            .saturating_sub(self.shift as u32)
            .saturating_sub(self.anchor as u32)
    }

    /// The compare window as `(top bit, bottom bit)`, both inclusive.
    ///
    /// Purely the two header numbers rearranged, with no notion of which
    /// field they apply to — so a display can show the window without
    /// knowing anything this model has not established.
    pub fn window(&self) -> (u32, u32) {
        (63u32.saturating_sub(self.shift as u32), self.anchor as u32)
    }

    pub fn header(&self) -> u32 {
        (CLAUSE_TAG << 28)
            | ((self.shift as u32 & 0x3F) << 16)
            | ((self.anchor as u32 & 0x3F) << 10)
            | ((self.field.code() as u32) << 4)
            | self.op.code() as u32
    }

    pub fn from_header(header: u32, operand: u64) -> Option<Self> {
        if !looks_like_clause_tag(header) {
            return None;
        }
        // Bits [27:22] are never written by anything observed, and this
        // decoder has nowhere to put them. A word that carries them would
        // re-encode without them, which is a silent corruption of an
        // entry the port promised to round-trip; refusing it keeps the
        // words verbatim in [`Entry::Raw`] instead.
        if header & CLAUSE_RESERVED != 0 {
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
    /// The rule ends here and this word is its **first** action word. The
    /// rest, if any, are in the entries that follow — see [`Rule`].
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
    ///
    /// -- One rival reading is now dead. "Bits [27:24] carry the action
    /// type" would explain the field without any notion of precedence,
    /// and it is refuted by two words that decode to the **same**
    /// `(sel 2, sub 0)` result with different values here: one read back
    /// off a running device, one assembled on demand through the device's
    /// own rule builder. Same result, two values — so the field is not
    /// the result's identity.
    ///
    /// -- And the experiment that would settle precedence is now cheap:
    /// the same rule builder takes the tag as a parameter, so two
    /// overlapping rules at two tags can be installed in one experiment.
    pub priority: u8,
    pub payload: ActionPayload,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActionPayload {
    /// Top nibble `0xE`, no other field.
    ///
    /// ⛔ The name is older than the evidence and does not survive it:
    /// nothing establishes what this word does, and the type code it was
    /// once attributed to (0) has **no measured word at all** — see
    /// [`ActionPayload::simple`]. Carried and round-tripped, not named.
    Broadcast,
    /// Carried as `type_code ^ 1` in bits [22:20].
    ///
    /// Only `2..=7` have a measured word behind them; build those through
    /// [`ActionPayload::simple`], which refuses the rest.
    Simple { type_code: u8 },
    /// Types 0x08..=0x5B: a selector in the top nibble, a sub-index, and
    /// a field whose width depends on the selector.
    ///
    /// Software walks the type in groups of fourteen — eleven "wide"
    /// entries then three "narrow" ones — and the group number lands in
    /// the top nibble as `2 + group` or `8 + group`. Six groups gives
    /// selectors `2..=7` wide and `8..=0xD` narrow.
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

/// Widest queue number any emission site can produce.
///
/// ⛔ This is an observation about the **emitter**, not a field width:
/// every call site writes `idx & 0x1F`, so five bits is what software can
/// express. The field itself is twelve bits wide, and what the hardware
/// does with a wider value is not established — which is why a wider one
/// is refused rather than folded down into this range.
const QUEUE_FIELD_MAX: u16 = 0x1F;

impl ActionPayload {
    /// A [`ActionPayload::Simple`] payload for a type code that has a
    /// measured word behind it, or `None`.
    ///
    /// Types `2..=7` are the ones a live rule set contains. Types 0 and 1
    /// have **no measured word**, and the encoding is not merely unknown
    /// for them, it is contradictory: type 1 would encode to all zeros —
    /// which the spill scan of [`Rule::decode`] reads as padding — and
    /// type 0 to `0x0010_0000`, while the word the name
    /// [`ActionPayload::Broadcast`] was attached to is `0xE000_0000`.
    /// Inventing one would put a word in a table that nothing ever wrote.
    pub const fn simple(type_code: u8) -> Option<Self> {
        if matches!(type_code, 2..=7) {
            Some(ActionPayload::Simple { type_code })
        } else {
            None
        }
    }
}

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
    ///   is written `idx & 0x1F`, and those index values are the ones the
    ///   receive path polls its queues by. Queue numbers are gathered
    ///   elsewhere into `1 << (idx & 0x1F)` bitmaps, which is where the
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
    /// ⛔ Every **other** `Selected` action is a different result, and
    /// returning a queue for one of those would route frames by a number
    /// that is not a queue. What those other results *do* is not
    /// established — they are other sub-indices of the same group, and
    /// nothing names them. A live rule set is full of them: under one
    /// boot-long stimulus, 84 selected actions were counted and **none**
    /// of the 84 was this one. That count belongs to that stimulus; it is
    /// not a property of every rule set, and two words of this very type
    /// have since been seen elsewhere.
    ///
    /// ⛔ A field wider than [`QUEUE_FIELD_MAX`] is **refused, not
    /// truncated**: masking it would alias queue 32 onto queue 0 and route
    /// frames somewhere no rule asked for.
    pub fn destination_queue(&self) -> Option<u8> {
        match self.payload {
            ActionPayload::Selected { sel, sub, field, .. }
                if sel == SELECTED_QUEUE_SEL
                    && sub == SELECTED_QUEUE_SUB
                    && field <= QUEUE_FIELD_MAX =>
            {
                Some(field as u8)
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
        } else if (2..=0xD).contains(&sel) {
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

/// A whole rule: a chain of clauses, then the actions to take.
///
/// A rule does **not** fit in one entry, and it does not occupy a whole
/// number of them either. Software lays it out like this:
///
/// ```text
///  entry n     [ clause  | operand hi | operand lo | link 0x10000000 ]
///  entry n+1   [ clause  | operand hi | operand lo | ACTION 0        ]  <- last clause
///  entry n+2   [ ACTION 1| ACTION 2   | 0          | 0               ]  <- spill + padding
/// ```
///
/// The link word of **every** clause is written, then the first action
/// word overwrites the last one's; the remaining actions run on through
/// the following entries in word order and are zero-padded out to a whole
/// entry. So a rule spans `clauses + ceil((actions - 1) / 4)` entries.
///
/// -- OBSERVED, from the builder itself rather than from the shape of a
/// table: it writes the link word into every clause, backs up by one
/// word, writes the action words, and pads with
/// `(4 - ((actions - 1) & 3)) & 3` zeros. A second function computes the
/// entry count the allocator reserves, and it is the same number by a
/// different route.
///
/// ⛔ **The padding is not a terminator.** With `actions % 4 == 1` there
/// is none: one action fits in the link word and spills nothing at all,
/// five fill a spill entry exactly. What bounds the list in that case is
/// [`Rule::decode`]'s other stop — an entry carrying the clause tag,
/// which no action word ever can (see [`ActionPayload::Selected`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rule {
    pub clauses: Vec<Clause>,
    pub actions: Vec<Action>,
}

/// Why a run of entries is not a rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RuleError {
    /// An entry in the clause chain is not a clause.
    NotAClause,
    /// The chain reached an index nothing was ever written to.
    RanPastTheEnd,
    /// The chain ran longer than any rule this hardware can hold.
    TooLong,
}

/// What reading a rule produced, and how many entries it covered.
///
/// The span is reported **even when the read failed**, so a caller
/// walking a table does not turn one broken rule into several.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RuleRead {
    pub result: Result<Rule, RuleError>,
    pub span: u16,
}

/// How far a rule may run before the decoder calls it corrupt. Well above
/// anything observed; it is there so a damaged table cannot loop.
pub const MAX_RULE_ENTRIES: u16 = 64;

impl Rule {
    /// The entries this rule occupies, in index order.
    pub fn encode(&self) -> Vec<Quad> {
        if self.clauses.is_empty() {
            return Vec::new();
        }
        let mut words: Vec<u32> = Vec::with_capacity(self.clauses.len() * 4 + 4);
        for clause in &self.clauses {
            words.push(clause.header());
            words.push((clause.operand >> 32) as u32);
            words.push(clause.operand as u32);
            words.push(LINK_AND_NEXT);
        }
        if !self.actions.is_empty() {
            // The first action word takes the last clause's link word.
            words.truncate(words.len() - 1);
            words.extend(self.actions.iter().map(Action::encode));
            while words.len() % 4 != 0 {
                words.push(0);
            }
        }
        words
            .chunks_exact(4)
            .map(|c| Quad::new([c[0], c[1], c[2], c[3]]))
            .collect()
    }

    /// Read the rule starting at `start`. `fetch` answers with the entry
    /// at an index, or `None` for one nothing ever wrote.
    pub fn decode(mut fetch: impl FnMut(u16) -> Option<Quad>, start: u16) -> RuleRead {
        let mut clauses = Vec::new();
        let mut actions = Vec::new();
        let mut span = 0u16;

        let fail = |result, span| RuleRead { result: Err(result), span };
        loop {
            if span >= MAX_RULE_ENTRIES {
                return fail(RuleError::TooLong, span);
            }
            let Some(quad) = fetch(start.wrapping_add(span)) else {
                return fail(RuleError::RanPastTheEnd, span);
            };
            span += 1;
            let Entry::Clause { clause, link } = Entry::decode(quad) else {
                return fail(RuleError::NotAClause, span);
            };
            clauses.push(clause);
            match link {
                Link::AndNext => continue,
                Link::Terminal(first) => {
                    actions.push(first);
                    break;
                }
            }
        }

        // Actions past the first, in word order, until the padding — or
        // until an entry that is a clause, which is the next rule.
        'spill: while span < MAX_RULE_ENTRIES {
            let Some(quad) = fetch(start.wrapping_add(span)) else { break };
            // The wider of the two clause predicates, on purpose: this
            // bounds the action list, and an entry carrying the tag ends
            // it whether or not this decoder can read it as a clause.
            // Using the narrower one would swallow the next rule's first
            // entry into this rule's action list.
            if looks_like_clause_tag(quad.words[0]) {
                break;
            }
            span += 1;
            for word in quad.words {
                if word == 0 {
                    break 'spill;
                }
                actions.push(Action::decode(word));
            }
        }

        RuleRead { result: Ok(Self { clauses, actions }), span }
    }

    /// The queue this rule sends a matching frame to.
    ///
    /// A rule carries several actions and at most one of them names a
    /// queue; what the others do is not established, and they are spoken
    /// of by type code here rather than named. Looking only at the first
    /// would miss the queue — it is the **last** of the three in every
    /// rule software builds for its user-facing port.
    pub fn destination_queue(&self) -> Option<u8> {
        self.actions.iter().find_map(Action::destination_queue)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The operator code the tag clauses carry. Unexplained, so it goes
    /// through the numeric constructor rather than wearing a name.
    const TAG_ABSENCE: Op = Op::from_code(6);

    /// The eight distinct clauses a live rule set contains, built from
    /// the constructors rather than pasted.
    fn observed_clauses() -> Vec<Clause> {
        vec![
            Clause { field: Field::from_code(0x02), op: Op::Eq, shift: 0x30, anchor: 0, operand: 0x8809 },
            Clause { field: Field::from_code(0x02), op: Op::Eq, shift: 0x30, anchor: 0, operand: 0x8808 },
            Clause { field: Field::from_code(0x02), op: Op::Eq, shift: 0x00, anchor: 0, operand: 0x888E },
            Clause { field: Field::from_code(0x03), op: TAG_ABSENCE, shift: 0x3F, anchor: 0, operand: 0 },
            Clause { field: Field::from_code(0x04), op: TAG_ABSENCE, shift: 0x3F, anchor: 0, operand: 0 },
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

    /// Every operator code keeps its number, and the three classes stay
    /// apart: established, emitted-but-unexplained, never seen.
    #[test]
    fn all_sixteen_operator_codes_round_trip() {
        for code in 0..16u8 {
            assert_eq!(Op::from_code(code).code(), code, "code {code}");
        }
        for code in [1u8, 2, 3, 4] {
            assert!(Op::from_code(code).is_established(), "code {code}");
        }
        for code in OBSERVED_OP_CODES {
            assert_eq!(Op::from_code(code), Op::Observed(code), "code {code}");
            assert!(!Op::from_code(code).is_established(), "code {code}");
        }
        for code in 8..16u8 {
            assert_eq!(Op::from_code(code), Op::Unknown(code), "code {code}");
            assert!(!Op::from_code(code).is_established(), "code {code}");
        }
    }

    /// Field codes keep their number too, and the most used code of the
    /// whole set is on the seen list rather than the unseen one.
    #[test]
    fn field_codes_round_trip_and_the_seen_ones_are_marked() {
        for code in 0..64u8 {
            assert_eq!(Field::from_code(code).code(), code);
        }
        assert_eq!(Field::from_code(0x02), Field::EtherType);
        assert!(matches!(Field::from_code(0x10), Field::Observed(0x10)));
        assert!(matches!(Field::from_code(0x3F), Field::Unknown(0x3F)));
        // The frame-length code: the engine answers it, so this file must
        // not call it unseen.
        assert!(matches!(Field::from_code(0x24), Field::Observed(0x24)));
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

    /// A clause word assembled on demand by a live device, reproduced bit
    /// for bit from the four numbers that were typed into it.
    ///
    /// The device takes `field`, `operator`, `anchor`, `operand`, `shift`
    /// in the clear and prints the words it built. That makes the header
    /// layout a construction rather than an interpretation: the same four
    /// numbers in, the same word out.
    #[test]
    fn the_header_layout_reproduces_a_word_a_device_assembled_on_demand() {
        let built = Clause {
            field: Field::from_code(0x02),
            op: Op::Eq,
            shift: 0x30,
            anchor: 0,
            operand: 0x0806,
        };
        assert_eq!(built.header(), 0xF030_0021);
        assert_eq!(built.compared_bits(), 16, "64 - 0x30");
        assert_eq!(built.window(), (63 - 0x30, 0));
        assert_eq!(
            Clause::from_header(built.header(), built.operand),
            Some(built)
        );

        // And the action the same experiment attached to it: a queue result.
        let action = Action::decode(0x2800_000F);
        assert_eq!(
            action.payload,
            ActionPayload::Selected { sel: 2, sub: 0, field: 0x0F, wide: true }
        );
        assert_eq!(action.destination_queue(), Some(0x0F));
        assert_eq!(action.encode(), 0x2800_000F);
    }

    /// A header carrying bits nothing ever writes is **not** decoded as a
    /// clause — because decoding it would drop them, and the port's whole
    /// contract is that 128 bits it cannot read come back unchanged.
    #[test]
    fn a_clause_header_with_reserved_bits_is_not_decoded_as_a_clause() {
        let corrupt = 0xF330_0021u32;
        assert!(looks_like_clause_tag(corrupt), "it does carry the tag");
        assert_eq!(Clause::from_header(corrupt, 0x8809), None);

        let q = Quad::new([corrupt, 0, 0x8809, LINK_AND_NEXT]);
        let entry = Entry::decode(q);
        assert!(matches!(entry, Entry::Raw(_)), "got {entry:?}");
        assert_eq!(entry.encode(), q, "the words must survive bit for bit");

        // The negative control: the same word without those bits is a
        // clause, and decodes to the one a live device assembled.
        let clean = corrupt & !CLAUSE_RESERVED;
        assert_eq!(clean, 0xF030_0021);
        assert!(matches!(Entry::decode(Quad::new([clean, 0, 0x8809, LINK_AND_NEXT])),
            Entry::Clause { .. }));
    }

    /// The two measured queue-result words, built from the constructors:
    /// same selector, same sub-index, two different values in [27:24].
    ///
    /// This is the pending half of
    /// [`the_two_observed_action_words_decode_to_one_type_at_two_priorities`]
    /// — that one shows one *simple* type at two values of the field,
    /// this one shows one *selected* result at two values of it. Together
    /// they leave no room for the field to be the action's identity.
    #[test]
    fn the_two_selected_action_words_differ_only_in_their_tag_and_field() {
        let readback = Action {
            priority: 1,
            payload: ActionPayload::Selected { sel: 2, sub: 0, field: 0x10, wide: true },
        };
        let assembled = Action {
            priority: 8,
            payload: ActionPayload::Selected { sel: 2, sub: 0, field: 0x0F, wide: true },
        };
        assert_eq!(readback.encode(), 0x2100_0010);
        assert_eq!(assembled.encode(), 0x2800_000F);
        assert_eq!(Action::decode(readback.encode()), readback);
        assert_eq!(Action::decode(assembled.encode()), assembled);

        // Same result, two tags — so the tag is not the result.
        let (
            ActionPayload::Selected { sel: s1, sub: b1, .. },
            ActionPayload::Selected { sel: s2, sub: b2, .. },
        ) = (readback.payload, assembled.payload)
        else {
            panic!("both are selected actions");
        };
        assert_eq!((s1, b1), (s2, b2));
        assert_ne!(readback.priority, assembled.priority);
        // And both name a queue, which no other sub-index does.
        assert_eq!(readback.destination_queue(), Some(0x10));
        assert_eq!(assembled.destination_queue(), Some(0x0F));
    }

    /// A queue field wider than anything an emitter can produce is
    /// refused. Masking it would alias queue 32 onto queue 0 and route a
    /// frame to a queue no rule named.
    #[test]
    fn a_queue_field_wider_than_five_bits_is_refused_not_aliased() {
        for field in [0x20u16, 0x21, 0x3F, 0x100, 0xFFF] {
            let action = Action {
                priority: 1,
                payload: ActionPayload::Selected { sel: 2, sub: 0, field, wide: true },
            };
            assert_eq!(action.destination_queue(), None, "field {field:#05x}");
            // It still round-trips: refusing to read it is not dropping it.
            assert_eq!(Action::decode(action.encode()), action, "field {field:#05x}");
        }
        // The boundary, from both sides.
        let inside = Action {
            priority: 1,
            payload: ActionPayload::Selected { sel: 2, sub: 0, field: 0x1F, wide: true },
        };
        assert_eq!(inside.destination_queue(), Some(0x1F));
    }

    /// The two type codes with no measured word have no encoding here.
    #[test]
    fn the_action_types_without_a_measured_word_have_no_encoding() {
        assert_eq!(ActionPayload::simple(0), None);
        assert_eq!(ActionPayload::simple(1), None);
        for type_code in 2..=7u8 {
            let payload = ActionPayload::simple(type_code)
                .unwrap_or_else(|| panic!("type {type_code} has a measured word"));
            let action = Action { priority: 1, payload };
            assert_eq!(Action::decode(action.encode()), action, "type {type_code}");
        }
        for type_code in [8u8, 9, 0xFF] {
            assert_eq!(ActionPayload::simple(type_code), None, "type {type_code}");
        }
    }

    /// The spill scan's stop-on-zero must stay: a type-1 word would be
    /// all zeros, which is exactly what padding looks like. This pins the
    /// reason the constructor above refuses it.
    #[test]
    fn a_zero_word_ends_the_action_list_rather_than_decoding() {
        let rule = Rule {
            clauses: vec![observed_clauses()[0]],
            actions: vec![
                Action { priority: 0, payload: ActionPayload::simple(6).unwrap() },
                Action {
                    priority: 1,
                    payload: ActionPayload::Selected { sel: 2, sub: 0, field: 0x10, wide: true },
                },
            ],
        };
        let quads = rule.encode();
        assert_eq!(quads.len(), 2);
        assert_eq!(quads[1].words[2..], [0, 0], "padded out with zeros");
        let read = Rule::decode(|i| quads.get(i as usize).copied(), 0);
        assert_eq!(read.result, Ok(rule));
    }
}
