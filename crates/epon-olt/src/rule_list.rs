//! Classifier rule lists, as they travel on the extended-OAM channel.
//!
//! One rule list update is a run of variables under leaf
//! [`leaf::RULE_LIST`](crate::extended::leaf::RULE_LIST), each one an entry,
//! followed by an action variable that puts them into effect. Two lists are
//! addressed, distinguished by the first octet of every entry's value.
//!
//! What this module is *not*: it does not decide which entries a receiver
//! will accept, nor what a type code means. Those are properties of whatever
//! firmware is listening, and a sender that enforced them could no longer
//! measure a refusal — which is the only observable a receiver offers here.

use crate::extended::{leaf, Container, Descriptor, Length, Variable};

/// Byte 4 of a rule-list entry: which list it addresses, or what to do to
/// them both.
///
/// ⛔ This is byte 4. The type code is byte 5. The two are adjacent nibble
/// pairs in a hex string and nothing in the bytes tells them apart, which is
/// why they are separate fields here: a receiver that refuses an unknown
/// sub-operation was observed to stop reading the rest of the message, not
/// just the entry, so the inversion costs everything and is silent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubOp {
    Mark,
    Reset,
    /// Entries of the list a rule matches on.
    MatchEntries,
    /// Entries of the list a rule acts through.
    ActionEntries,
}

impl SubOp {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Mark => 0,
            Self::Reset => 1,
            Self::MatchEntries => 2,
            Self::ActionEntries => 3,
        }
    }

    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Mark),
            1 => Some(Self::Reset),
            2 => Some(Self::MatchEntries),
            3 => Some(Self::ActionEntries),
            _ => None,
        }
    }
}

/// The octet a [`SubOp::Reset`] entry carries after its sub-operation.
///
/// Position established by the frame that was exercised; nothing observed
/// gives it a second value, so it is written out rather than parameterised.
const RESET_TRAILER: u8 = 0x00;

/// The single octet the apply action carries.
const APPLY_BODY: u8 = 0x00;

/// An entry of the list a rule matches on.
///
/// Field names say where each octet sits and what is known to be done with
/// it. `type_code` stays a number: what a given code selects is a property of
/// the receiver, not of this encoding.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MatchEntry {
    /// Byte 5. A code; only part of its space has been seen.
    pub type_code: u8,
    /// Byte 6. Position established, meaning not.
    pub byte6: u8,
    /// Bytes 7 and 8, each subtracted from a base the type selects.
    pub bases: (u8, u8),
    /// Byte 9. Values 1 to 4 observed.
    pub selector: u8,
    /// Byte 11 onward. Byte 10 carries its length, so it is not stored
    /// separately: a count that can disagree with what follows it is a
    /// second source of truth.
    pub bits: Vec<u8>,
    /// Anything past the bits. Kept so a message round-trips rather than
    /// being silently shortened.
    pub trailing: Vec<u8>,
}

impl MatchEntry {
    /// Octets of an entry before its bit field: sub-operation, type, byte 6,
    /// two bases, selector, and the bit-field length.
    const FIXED_LEN: usize = 7;

    pub fn new(type_code: u8, byte6: u8, bases: (u8, u8), selector: u8, bits: Vec<u8>) -> Self {
        Self { type_code, byte6, bases, selector, bits, trailing: Vec::new() }
    }

    fn encode_into(&self, out: &mut Vec<u8>) {
        out.push(SubOp::MatchEntries.as_u8());
        out.push(self.type_code);
        out.push(self.byte6);
        out.push(self.bases.0);
        out.push(self.bases.1);
        out.push(self.selector);
        out.push(self.bits.len().min(u8::MAX as usize) as u8);
        out.extend_from_slice(&self.bits);
        out.extend_from_slice(&self.trailing);
    }

    /// Decode from the sub-operation octet onward.
    fn decode(value: &[u8]) -> Option<Self> {
        if value.len() < Self::FIXED_LEN {
            return None;
        }
        let declared = value[6] as usize;
        let available = value.len() - Self::FIXED_LEN;
        // A declared length longer than what is there is not silently
        // stretched: the entry keeps what arrived, and the length is
        // recomputed from it on the way out.
        let take = declared.min(available);
        Some(Self {
            type_code: value[1],
            byte6: value[2],
            bases: (value[3], value[4]),
            selector: value[5],
            bits: value[Self::FIXED_LEN..Self::FIXED_LEN + take].to_vec(),
            trailing: value[Self::FIXED_LEN + take..].to_vec(),
        })
    }
}

/// The four octets an action entry may carry after its type code.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActionParameter {
    /// Bytes 6 and 7, big-endian.
    pub tag: u16,
    /// Byte 8.
    pub a: u8,
    /// Byte 9.
    pub b: u8,
}

impl ActionParameter {
    /// The only value of `tag` a receiver was observed to read past.
    ///
    /// Not enforced on the way out: sending another value is how one finds
    /// out what the receiver does with it.
    pub const REQUIRED_TAG: u16 = 3;

    pub const fn new(a: u8, b: u8) -> Self {
        Self { tag: Self::REQUIRED_TAG, a, b }
    }
}

/// An entry of the list a rule acts through.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ActionEntry {
    /// Byte 5. Stays numeric: what a code selects is decided by the
    /// receiver, from a table this crate has no business holding.
    pub type_code: u8,
    pub parameter: Option<ActionParameter>,
    pub trailing: Vec<u8>,
}

impl ActionEntry {
    /// Sub-operation and type code.
    const FIXED_LEN: usize = 2;
    /// Tag, then two octets.
    const PARAMETER_LEN: usize = 4;

    pub fn new(type_code: u8, parameter: Option<ActionParameter>) -> Self {
        Self { type_code, parameter, trailing: Vec::new() }
    }

    fn encode_into(&self, out: &mut Vec<u8>) {
        out.push(SubOp::ActionEntries.as_u8());
        out.push(self.type_code);
        if let Some(p) = self.parameter {
            out.extend_from_slice(&p.tag.to_be_bytes());
            out.push(p.a);
            out.push(p.b);
        }
        out.extend_from_slice(&self.trailing);
    }

    fn decode(value: &[u8]) -> Option<Self> {
        if value.len() < Self::FIXED_LEN {
            return None;
        }
        let type_code = value[1];
        let rest = &value[Self::FIXED_LEN..];
        if rest.len() >= Self::PARAMETER_LEN {
            return Some(Self {
                type_code,
                parameter: Some(ActionParameter {
                    tag: u16::from_be_bytes([rest[0], rest[1]]),
                    a: rest[2],
                    b: rest[3],
                }),
                trailing: rest[Self::PARAMETER_LEN..].to_vec(),
            });
        }
        Some(Self { type_code, parameter: None, trailing: rest.to_vec() })
    }
}

/// One variable of a rule-list update.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RuleEntry {
    Mark,
    Reset,
    Match(MatchEntry),
    Action(ActionEntry),
}

impl RuleEntry {
    pub fn sub_op(&self) -> SubOp {
        match self {
            Self::Mark => SubOp::Mark,
            Self::Reset => SubOp::Reset,
            Self::Match(_) => SubOp::MatchEntries,
            Self::Action(_) => SubOp::ActionEntries,
        }
    }

    /// The bytes from the sub-operation octet onward — byte 4 of the TLV.
    pub fn encode_value(&self) -> Vec<u8> {
        let mut out = Vec::new();
        match self {
            // The sub-operation octet is the whole entry: a mark has
            // nothing to carry.
            Self::Mark => out.push(SubOp::Mark.as_u8()),
            Self::Reset => {
                out.push(SubOp::Reset.as_u8());
                out.push(RESET_TRAILER);
            }
            Self::Match(m) => m.encode_into(&mut out),
            Self::Action(a) => a.encode_into(&mut out),
        }
        out
    }

    /// Read an entry from the bytes `encode_value` produces.
    pub fn decode_value(value: &[u8]) -> Option<Self> {
        let sub_op = SubOp::from_u8(*value.first()?)?;
        match sub_op {
            SubOp::Mark => Some(Self::Mark),
            SubOp::Reset => Some(Self::Reset),
            SubOp::MatchEntries => MatchEntry::decode(value).map(Self::Match),
            SubOp::ActionEntries => ActionEntry::decode(value).map(Self::Action),
        }
    }

    /// The whole variable, address included. Built here so the declared
    /// length and the value it describes cannot drift apart.
    pub fn variable(&self) -> Variable {
        let value = self.encode_value();
        let length = Length::of_value(value.len())
            .expect("a rule-list entry is between 1 and 127 bytes");
        Variable::Container(Container {
            descriptor: Descriptor::attribute(leaf::RULE_LIST),
            length,
            value,
        })
    }

    /// Read an entry back out of a variable, if that is what it is.
    pub fn from_variable(v: &Variable) -> Option<Self> {
        let c = v.as_container()?;
        if c.descriptor != Descriptor::attribute(leaf::RULE_LIST) {
            return None;
        }
        Self::decode_value(&c.value)
    }
}

/// The variable that puts the accumulated entries into effect.
fn apply_variable() -> Variable {
    Variable::Container(Container {
        descriptor: Descriptor::action(leaf::RULE_LIST_APPLY),
        length: Length::Bytes(1),
        value: vec![APPLY_BODY],
    })
}

fn is_apply(v: &Variable) -> bool {
    v.as_container()
        .is_some_and(|c| c.descriptor == Descriptor::action(leaf::RULE_LIST_APPLY))
}

/// A whole update to a receiver's rule lists.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RuleListUpdate {
    pub reset: bool,
    pub mark: bool,
    pub match_entries: Vec<MatchEntry>,
    pub action_entries: Vec<ActionEntry>,
    pub apply: bool,
}

impl RuleListUpdate {
    /// The variables of the update, in the order a receiver was observed to
    /// require.
    ///
    /// ⛔ That order is a property of a *receiver*, not of the format. Nothing
    /// in the bytes marks an entry as coming before another, which is why
    /// [`Self::from_variables`] accepts them in any order: a parser that
    /// enforced this would refuse a valid message from equipment that emits
    /// it differently, and would be measuring itself.
    pub fn variables(&self) -> Vec<Variable> {
        let mut out = Vec::new();
        if self.reset {
            out.push(RuleEntry::Reset.variable());
        }
        if self.mark {
            out.push(RuleEntry::Mark.variable());
        }
        for m in &self.match_entries {
            out.push(RuleEntry::Match(m.clone()).variable());
        }
        for a in &self.action_entries {
            out.push(RuleEntry::Action(a.clone()).variable());
        }
        if self.apply {
            out.push(apply_variable());
        }
        out
    }

    /// Read an update back from a variable list, whatever order it is in.
    ///
    /// Returns `None` if any variable is neither a rule-list entry nor the
    /// apply action: a partially understood message would report as a
    /// complete one.
    pub fn from_variables(vars: &[Variable]) -> Option<Self> {
        let mut out = Self::default();
        for v in vars {
            if is_apply(v) {
                out.apply = true;
                continue;
            }
            match RuleEntry::from_variable(v)? {
                RuleEntry::Mark => out.mark = true,
                RuleEntry::Reset => out.reset = true,
                RuleEntry::Match(m) => out.match_entries.push(m),
                RuleEntry::Action(a) => out.action_entries.push(a),
            }
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extended::Branch;

    fn a_match() -> MatchEntry {
        MatchEntry::new(0x06, 0x01, (0x0C, 0x02), 3, vec![0xFF, 0x0F])
    }

    fn an_action() -> ActionEntry {
        ActionEntry::new(0x03, Some(ActionParameter::new(0x10, 0x00)))
    }

    fn update() -> RuleListUpdate {
        RuleListUpdate {
            reset: true,
            mark: true,
            match_entries: vec![a_match(), MatchEntry::new(0x12, 0, (0, 0), 1, vec![0x01])],
            action_entries: vec![an_action(), ActionEntry::new(0x08, None)],
            apply: true,
        }
    }

    /// The whole TLV, address included, as it goes on the wire.
    fn tlv(entry: &RuleEntry) -> Vec<u8> {
        let Variable::Container(c) = entry.variable() else {
            panic!("an entry is a container");
        };
        let mut out = vec![c.descriptor.branch.as_u8()];
        out.extend_from_slice(&c.descriptor.leaf.to_be_bytes());
        out.push(c.length.as_u8());
        out.extend_from_slice(&c.value);
        out
    }

    /// The inversion this type exists to prevent: byte 4 is the
    /// sub-operation and byte 5 is the type code, and no public constructor
    /// can produce them the other way round.
    #[test]
    fn sub_operation_and_type_code_are_separate_fields() {
        let bytes = tlv(&RuleEntry::Match(MatchEntry::new(0x06, 0, (0, 0), 1, vec![])));
        assert_eq!(bytes[4], SubOp::MatchEntries.as_u8());
        assert_eq!(bytes[5], 0x06);
        assert_eq!(&bytes[4..6], &[0x02, 0x06]);

        // Same two numbers on an action entry, and they land the same way
        // round: the sub-operation is never the payload's type.
        let bytes = tlv(&RuleEntry::Action(ActionEntry::new(0x02, None)));
        assert_eq!(bytes[4], SubOp::ActionEntries.as_u8());
        assert_eq!(bytes[5], 0x02);
    }

    #[test]
    fn the_fixed_entries_encode_the_bytes_the_receiver_reads() {
        let d7 = Branch::Attribute.as_u8();
        let d9 = Branch::Action.as_u8();
        assert_eq!(tlv(&RuleEntry::Mark), vec![d7, 0x05, 0x01, 0x01, 0x00]);
        assert_eq!(tlv(&RuleEntry::Reset), vec![d7, 0x05, 0x01, 0x02, 0x01, 0x00]);

        let Variable::Container(c) = apply_variable() else {
            panic!("the apply action is a container");
        };
        assert_eq!(c.descriptor, Descriptor::action(leaf::RULE_LIST_APPLY));
        assert_eq!(c.descriptor.branch.as_u8(), d9);
        assert_eq!(c.length.as_u8(), 1);
        assert_eq!(c.value, vec![0x00]);
    }

    #[test]
    fn a_match_entry_lays_its_octets_out_where_they_were_measured() {
        let bytes = tlv(&RuleEntry::Match(a_match()));
        // branch, leaf, length, then the value.
        assert_eq!(&bytes[..3], &[Branch::Attribute.as_u8(), 0x05, 0x01]);
        assert_eq!(bytes[3] as usize, bytes.len() - 4, "declared length is the value's");
        assert_eq!(bytes[4], SubOp::MatchEntries.as_u8());
        assert_eq!(bytes[5], 0x06, "type code");
        assert_eq!(bytes[6], 0x01, "byte 6");
        assert_eq!(&bytes[7..9], &[0x0C, 0x02], "the two bases");
        assert_eq!(bytes[9], 3, "selector");
        assert_eq!(bytes[10], 2, "length of the bit field");
        assert_eq!(&bytes[11..], &[0xFF, 0x0F]);
    }

    #[test]
    fn an_action_parameter_sits_where_the_receiver_looks_for_it() {
        let bytes = tlv(&RuleEntry::Action(an_action()));
        assert_eq!(bytes[4], SubOp::ActionEntries.as_u8());
        assert_eq!(bytes[5], 0x03, "type code");
        assert_eq!(
            u16::from_be_bytes([bytes[6], bytes[7]]),
            ActionParameter::REQUIRED_TAG
        );
        assert_eq!(&bytes[8..10], &[0x10, 0x00]);
    }

    #[test]
    fn every_entry_round_trips_through_its_variable() {
        for entry in [
            RuleEntry::Mark,
            RuleEntry::Reset,
            RuleEntry::Match(a_match()),
            RuleEntry::Match(MatchEntry::new(0x21, 0xFF, (1, 2), 4, vec![])),
            RuleEntry::Action(an_action()),
            RuleEntry::Action(ActionEntry::new(0x00, None)),
        ] {
            let v = entry.variable();
            assert_eq!(RuleEntry::from_variable(&v), Some(entry.clone()));
            assert_eq!(RuleEntry::decode_value(&entry.encode_value()), Some(entry));
        }
    }

    #[test]
    fn a_rule_list_update_round_trips() {
        let update = update();
        let vars = update.variables();
        assert_eq!(RuleListUpdate::from_variables(&vars), Some(update));
    }

    #[test]
    fn the_update_emits_its_entries_in_the_order_the_receiver_requires() {
        let vars = update().variables();
        let ops: Vec<Option<SubOp>> = vars
            .iter()
            .map(|v| RuleEntry::from_variable(v).map(|e| e.sub_op()))
            .collect();
        assert_eq!(
            ops,
            vec![
                Some(SubOp::Reset),
                Some(SubOp::Mark),
                Some(SubOp::MatchEntries),
                Some(SubOp::MatchEntries),
                Some(SubOp::ActionEntries),
                Some(SubOp::ActionEntries),
                // The apply action is not a rule-list entry at all.
                None,
            ]
        );
        assert!(is_apply(vars.last().expect("apply goes last")));
    }

    /// The negative control of the test above: the order is the sender's
    /// choice, not something the format states, so the parser must not
    /// depend on it.
    #[test]
    fn the_parser_accepts_the_entries_in_any_order() {
        let update = update();
        let mut reversed = update.variables();
        reversed.reverse();
        assert_eq!(
            RuleListUpdate::from_variables(&reversed),
            Some(RuleListUpdate {
                // Entry order within each list is preserved, and only the
                // relative order of the two lists is free.
                match_entries: update.match_entries.iter().rev().cloned().collect(),
                action_entries: update.action_entries.iter().rev().cloned().collect(),
                ..update.clone()
            })
        );

        // Interleaved, which no sender does and every parser must survive.
        let mixed = vec![
            RuleEntry::Action(an_action()).variable(),
            RuleEntry::Match(a_match()).variable(),
            apply_variable(),
            RuleEntry::Reset.variable(),
        ];
        let parsed = RuleListUpdate::from_variables(&mixed).expect("parses");
        assert!(parsed.reset && parsed.apply && !parsed.mark);
        assert_eq!(parsed.match_entries, vec![a_match()]);
        assert_eq!(parsed.action_entries, vec![an_action()]);
    }

    #[test]
    fn a_variable_that_is_not_a_rule_list_entry_is_refused() {
        let stranger = Variable::Container(Container {
            descriptor: Descriptor::attribute(leaf::FIRMWARE_INFO),
            length: Length::Bytes(1),
            value: vec![0],
        });
        assert_eq!(RuleEntry::from_variable(&stranger), None);
        assert_eq!(RuleListUpdate::from_variables(&[stranger]), None);
        // An address, which carries no value at all.
        let address = Variable::Address {
            descriptor: Descriptor::attribute(leaf::RULE_LIST),
            parameters: Vec::new(),
        };
        assert_eq!(RuleEntry::from_variable(&address), None);
    }

    #[test]
    fn an_unknown_sub_operation_is_refused_rather_than_guessed() {
        for bad in [4u8, 5, 0x80, 0xFF] {
            assert_eq!(SubOp::from_u8(bad), None);
            assert_eq!(RuleEntry::decode_value(&[bad, 0, 0]), None);
        }
        assert_eq!(RuleEntry::decode_value(&[]), None);
        // A match entry shorter than its fixed part decodes to nothing
        // rather than to an entry with invented octets.
        assert_eq!(
            RuleEntry::decode_value(&[SubOp::MatchEntries.as_u8(), 1, 2, 3]),
            None
        );
    }

    /// A declared bit-field length longer than what arrived does not stretch
    /// the entry past its bytes, and the length written back out is the one
    /// the bits actually have.
    #[test]
    fn a_bit_field_longer_than_its_bytes_is_not_invented() {
        let value = vec![SubOp::MatchEntries.as_u8(), 0x06, 0, 0, 0, 1, 0x40, 0xAA];
        let Some(RuleEntry::Match(m)) = RuleEntry::decode_value(&value) else {
            panic!("a match entry");
        };
        assert_eq!(m.bits, vec![0xAA]);
        assert!(m.trailing.is_empty());
        assert_eq!(RuleEntry::Match(m).encode_value()[6], 1);
    }
}
