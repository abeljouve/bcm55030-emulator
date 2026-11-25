//! A scripted exchange on the organization-specific channel.
//!
//! The peer's own state machine covers what the standards define: discovery,
//! registration, keepalives. What an operator's equipment does after that is
//! provisioning, and it is specific to the link — so it is not modelled here
//! but read from a script, one step per line, and played once OAM discovery
//! has converged.
//!
//! The script is reactive: it sends a message, waits for the answer, and
//! decides what happens when the answer is wrong or does not come. That is
//! what separates it from a list of frames sent at fixed instants, which
//! keeps going whatever the far end did.
//!
//! ```text
//! # comment
//! ignore 001000/08
//! send   001000/01 D6 0000 0100 D7 0006
//! expect 001000/02 D6 0000 0100 D7 0006 ?? within 500 mismatch stop timeout stop
//! wait   250
//! send   AABBCC/10 0000 0001
//! expect AABBCC/* 80 within 2000 mismatch deregister-after 10 timeout deregister-after 0
//! deregister
//! ```
//!
//! - `send <oui>/<code> <bytes>` puts one extended OAMPDU on the line. The
//!   addresses and the flags are the peer's own, as for any frame it builds;
//!   the bytes follow the code octet untouched.
//! - `expect <oui>/<code|*> <pattern> within <ms>` waits for an extended
//!   OAMPDU under that OUI. The pattern is matched against the bytes that
//!   follow the code octet, as a prefix; `??` matches any octet.
//!   `mismatch <reaction>` says what a frame under that OUI which is not the
//!   expected one leads to, `timeout <reaction>` what silence leads to. Both
//!   default to `stop`.
//! - `wait <ms>` lets link time pass.
//! - `deregister` tears the registration down.
//! - `cut <ms>` takes the light off the fibre for that long, both ways: what
//!   the peer sends does not reach the far end, and what the far end sends
//!   does not reach the peer. The step ends when the light comes back. The
//!   peer is not told; it finds out by what stops arriving, and its own
//!   watchdog (see [`crate::peer`]) tears down a registration whose grants go
//!   unanswered — with a REGISTER the cut fibre loses.
//! - `ignore <oui>/<code|*>` names messages the far end sends on its own
//!   initiative. They are counted and never judged.
//! - `oam-destination multicast|unicast` says where the peer addresses its
//!   OAMPDUs, for the whole run: the slow-protocol group address, or the
//!   ONU's own once it is known. Equipment differs on this, and an answer
//!   built from the request carries the choice back.
//! - `reregistration replay|once` says what a registration that follows a
//!   teardown does to the script, whatever tore the first one down.
//!   - `replay`, the default: the script is played again from its first
//!     step, once OAM has converged on the new registration. A new
//!     registration is a new link to provision. A script that a reaction
//!     ended (`stop`, `deregister-after`) stays ended.
//!   - `once`: the script is played on the first registration only, and a
//!     teardown neither stops nor restarts it.
//!
//!   `deregister` and `cut` steps are faults injected into the run, not
//!   provisioning: a replay skips one that an earlier pass already played,
//!   or every registration would end at the same step. A script that cuts
//!   the fibre twice therefore cuts it once per registration, in order.
//!
//! A reaction is `continue`, `stop`, or `deregister-after <ms>`. Every
//! duration is link time, in milliseconds.
//!
//! The script only starts once the link is registered and OAM discovery has
//! converged. A teardown in the middle of a replayed script ends that pass:
//! the step under way is recorded as interrupted, and the next registration
//! starts from the top.

use std::fmt;

use crate::clock::{WireDuration, WireInstant};
use crate::oam::Oui;

/// An OUI and, optionally, the octet under it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Selector {
    pub oui: Oui,
    /// `None` selects every code under the OUI.
    pub code: Option<u8>,
}

impl Selector {
    pub fn selects(&self, oui: Oui, code: u8) -> bool {
        self.oui == oui && self.code.is_none_or(|c| c == code)
    }
}

impl fmt::Display for Selector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.code {
            Some(code) => write!(f, "{}/{code:02X}", self.oui),
            None => write!(f, "{}/*", self.oui),
        }
    }
}

/// Octets to find at the start of a body. `None` matches any octet.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Pattern(pub Vec<Option<u8>>);

impl Pattern {
    /// True when `body` starts with the pattern. A body shorter than the
    /// pattern does not match: an octet that is not there is not any octet.
    pub fn matches(&self, body: &[u8]) -> bool {
        self.0.len() <= body.len()
            && self.0.iter().zip(body).all(|(want, got)| want.is_none_or(|w| w == *got))
    }
}

/// What a step that did not go as written leads to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reaction {
    /// Carry on with the next step.
    Continue,
    /// Play nothing more.
    Stop,
    /// Tear the registration down after this long, then play nothing more.
    Deregister { after: WireDuration },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Action {
    Send { oui: Oui, code: u8, body: Vec<u8> },
    Expect {
        selector: Selector,
        pattern: Pattern,
        within: WireDuration,
        on_mismatch: Reaction,
        on_timeout: Reaction,
    },
    Wait(WireDuration),
    Deregister,
    /// Take the light off the fibre for this long.
    Cut(WireDuration),
}

impl Action {
    /// A fault injected into the run rather than an exchange: played on the
    /// first pass that reaches it, skipped by the replays after it.
    pub fn once_per_run(&self) -> bool {
        matches!(self, Self::Deregister | Self::Cut(_))
    }
}

/// What a registration that follows a teardown does to the script.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Reregistration {
    /// Play the script again from its first step.
    #[default]
    Replay,
    /// Leave the script as the teardown found it.
    Once,
}

/// One line of the script, with where it came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Step {
    /// One-based line of the source text.
    pub line: usize,
    pub action: Action,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Script {
    pub steps: Vec<Step>,
    pub ignored: Vec<Selector>,
    /// Where OAMPDUs are addressed, when the script says: `true` for the
    /// slow-protocol group address. `None` leaves the peer as configured.
    pub multicast_oam: Option<bool>,
    pub reregistration: Reregistration,
}

/// A line that could not be read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub line: usize,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for ParseError {}

impl Script {
    pub fn parse(text: &str) -> Result<Self, ParseError> {
        let mut script = Self::default();
        for (index, raw) in text.lines().enumerate() {
            let line = index + 1;
            let err = |message: String| ParseError { line, message };
            let content = raw.split('#').next().unwrap_or("");
            let mut words = content.split_whitespace();
            let Some(keyword) = words.next() else { continue };
            let rest: Vec<&str> = words.collect();
            let action = match keyword {
                "ignore" => {
                    let [selector] = rest[..] else {
                        return Err(err("ignore takes one <oui>/<code>".into()));
                    };
                    script.ignored.push(parse_selector(selector).map_err(err)?);
                    continue;
                }
                "oam-destination" => {
                    script.multicast_oam = Some(match rest[..] {
                        ["multicast"] => true,
                        ["unicast"] => false,
                        _ => return Err(err("oam-destination is `multicast` or `unicast`".into())),
                    });
                    continue;
                }
                "reregistration" => {
                    script.reregistration = match rest[..] {
                        ["replay"] => Reregistration::Replay,
                        ["once"] => Reregistration::Once,
                        _ => return Err(err("reregistration is `replay` or `once`".into())),
                    };
                    continue;
                }
                "send" => {
                    let Some((selector, bytes)) = rest.split_first() else {
                        return Err(err("send takes <oui>/<code> and the bytes".into()));
                    };
                    let selector = parse_selector(selector).map_err(err)?;
                    let Some(code) = selector.code else {
                        return Err(err("send needs a code, not *".into()));
                    };
                    let body = parse_pattern(bytes)
                        .map_err(err)?
                        .0
                        .into_iter()
                        .collect::<Option<Vec<u8>>>()
                        .ok_or_else(|| err("send cannot carry ??".into()))?;
                    Action::Send { oui: selector.oui, code, body }
                }
                "expect" => parse_expect(&rest).map_err(err)?,
                "wait" => {
                    let [ms] = rest[..] else {
                        return Err(err("wait takes a duration in milliseconds".into()));
                    };
                    Action::Wait(parse_ms(ms).map_err(err)?)
                }
                "deregister" if rest.is_empty() => Action::Deregister,
                "cut" => {
                    let [ms] = rest[..] else {
                        return Err(err("cut takes a duration in milliseconds".into()));
                    };
                    let span = parse_ms(ms).map_err(err)?;
                    if span == WireDuration::ZERO {
                        return Err(err("a cut lasts at least a millisecond".into()));
                    }
                    Action::Cut(span)
                }
                other => return Err(err(format!("unknown step `{other}`"))),
            };
            script.steps.push(Step { line, action });
        }
        Ok(script)
    }

    fn ignores(&self, oui: Oui, code: u8) -> bool {
        self.ignored.iter().any(|s| s.selects(oui, code))
    }
}

fn parse_selector(word: &str) -> Result<Selector, String> {
    let (oui, code) = word
        .split_once('/')
        .ok_or_else(|| format!("`{word}` is not <oui>/<code>"))?;
    let oui = Oui::parse(oui).ok_or_else(|| format!("`{oui}` is not an OUI"))?;
    let code = match code {
        "*" => None,
        hex => Some(
            u8::from_str_radix(hex, 16).map_err(|_| format!("`{hex}` is not a code octet"))?,
        ),
    };
    Ok(Selector { oui, code })
}

fn parse_ms(word: &str) -> Result<WireDuration, String> {
    word.parse::<u64>()
        .map(WireDuration::from_ms)
        .map_err(|_| format!("`{word}` is not a duration in milliseconds"))
}

/// Octets written as hexadecimal pairs, in words of any length; `??` for an
/// octet left free.
fn parse_pattern(words: &[&str]) -> Result<Pattern, String> {
    let mut out = Vec::new();
    for word in words {
        if word.len() % 2 != 0 || !word.is_ascii() {
            return Err(format!("`{word}` is not a whole number of octets"));
        }
        for pair in word.as_bytes().chunks(2) {
            let pair = std::str::from_utf8(pair).expect("ASCII checked above");
            out.push(match pair {
                "??" => None,
                hex => Some(
                    u8::from_str_radix(hex, 16).map_err(|_| format!("`{hex}` is not an octet"))?,
                ),
            });
        }
    }
    Ok(Pattern(out))
}

fn parse_expect(words: &[&str]) -> Result<Action, String> {
    let (selector, rest) =
        words.split_first().ok_or("expect takes <oui>/<code> first")?;
    let selector = parse_selector(selector)?;
    let within_at = rest
        .iter()
        .position(|w| *w == "within")
        .ok_or("expect needs `within <ms>`")?;
    let pattern = parse_pattern(&rest[..within_at])?;
    let mut tail = rest[within_at + 1..].iter();
    let within = parse_ms(tail.next().ok_or("`within` needs a duration")?)?;
    let mut on_mismatch = Reaction::Stop;
    let mut on_timeout = Reaction::Stop;
    while let Some(word) = tail.next() {
        let slot = match *word {
            "mismatch" => &mut on_mismatch,
            "timeout" => &mut on_timeout,
            other => return Err(format!("`{other}` is neither `mismatch` nor `timeout`")),
        };
        *slot = match *tail.next().ok_or("a reaction is missing")? {
            "continue" => Reaction::Continue,
            "stop" => Reaction::Stop,
            "deregister-after" => Reaction::Deregister {
                after: parse_ms(tail.next().ok_or("`deregister-after` needs a duration")?)?,
            },
            other => return Err(format!("`{other}` is not a reaction")),
        };
    }
    Ok(Action::Expect { selector, pattern, within, on_mismatch, on_timeout })
}

/// How one step ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    Sent,
    Waited,
    /// The expected frame arrived.
    Matched,
    /// A frame under the expected OUI arrived and was not the expected one.
    Mismatched { oui: Oui, code: u8 },
    /// Nothing under the expected OUI arrived in time.
    TimedOut,
    /// The registration was torn down, by a step or by a reaction.
    Deregistered,
    /// The light went off; the step ends when it comes back.
    Cut,
    /// The registration was torn down while the step was under way, by
    /// something other than the step itself.
    Interrupted,
    /// A fault an earlier pass already injected.
    Skipped,
}

impl fmt::Display for Verdict {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Sent => f.write_str("sent"),
            Self::Waited => f.write_str("waited"),
            Self::Matched => f.write_str("matched"),
            Self::Mismatched { oui, code } => write!(f, "mismatched ({oui}/{code:02X})"),
            Self::TimedOut => f.write_str("timed out"),
            Self::Deregistered => f.write_str("deregistered"),
            Self::Cut => f.write_str("fibre cut"),
            Self::Interrupted => f.write_str("interrupted by a teardown"),
            Self::Skipped => f.write_str("skipped (played on an earlier registration)"),
        }
    }
}

/// What happened at one step, and when.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Outcome {
    /// Index of the step in [`Script::steps`].
    pub step: usize,
    /// Line of the script the step was read from.
    pub line: usize,
    pub at: WireInstant,
    pub verdict: Verdict,
    /// Which pass over the script this was: one more for every registration
    /// the script was replayed on.
    pub registration: u32,
}

/// What the player asks of the peer that hosts it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Command {
    Send { oui: Oui, code: u8, body: Vec<u8> },
    Deregister,
    /// Take the light off the fibre from now until `until`.
    Cut { until: WireInstant },
    /// Call [`Player::resume`] again at this instant.
    WakeAt(WireInstant),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// Not started: discovery has not converged yet.
    Idle,
    /// The step at the cursor is to be started.
    Ready,
    Waiting { until: WireInstant },
    /// The fibre is cut until this instant.
    Dark { until: WireInstant },
    Expecting { deadline: WireInstant },
    /// A reaction asked for a teardown at this instant.
    Leaving { at: WireInstant },
    Done,
}

/// A script being played.
///
/// It decides and does not act: each call returns what the peer has to do,
/// which keeps it free of the peer's clock, scheduler and frame builders —
/// and lets it be driven by a test with none of them present.
#[derive(Clone, Debug)]
pub struct Player {
    script: Script,
    cursor: usize,
    state: State,
    outcomes: Vec<Outcome>,
    /// Passes started so far.
    registration: u32,
    /// A reaction ended the script. Nothing replays it.
    halted: bool,
    /// Steps already played, for the ones played once per run.
    played: Vec<bool>,
    /// Frames the script was told to ignore.
    pub ignored_seen: u64,
    /// Extended frames that arrived while no step was waiting for one, or
    /// under an OUI the waiting step does not name.
    pub unsolicited_seen: u64,
}

impl Player {
    pub fn new(script: Script) -> Self {
        Self {
            played: vec![false; script.steps.len()],
            script,
            cursor: 0,
            state: State::Idle,
            outcomes: Vec::new(),
            registration: 0,
            halted: false,
            ignored_seen: 0,
            unsolicited_seen: 0,
        }
    }

    pub fn started(&self) -> bool {
        self.state != State::Idle
    }

    pub fn finished(&self) -> bool {
        self.state == State::Done
    }

    pub fn outcomes(&self) -> &[Outcome] {
        &self.outcomes
    }

    pub fn steps(&self) -> &[Step] {
        &self.script.steps
    }

    /// What a registration that follows a teardown does to the script.
    pub fn reregistration(&self) -> Reregistration {
        self.script.reregistration
    }

    /// Passes started so far: 1 on the first registration, one more for
    /// every registration the script was replayed on.
    pub fn registration(&self) -> u32 {
        self.registration
    }

    /// Start a pass. A call while a pass is under way, or after the last one
    /// ended, changes nothing.
    pub fn start(&mut self, now: WireInstant) -> Vec<Command> {
        if self.state != State::Idle {
            return Vec::new();
        }
        self.registration += 1;
        self.state = State::Ready;
        self.resume(now)
    }

    /// The registration the script was playing on was torn down at `now`.
    ///
    /// Replayed, the pass ends here: a step waiting on the link is recorded
    /// as interrupted, and the next [`Self::start`] plays from the top. A
    /// teardown a reaction had asked for, or one that arrives while such a
    /// teardown is pending, ends the script for good.
    pub fn on_teardown(&mut self, now: WireInstant) {
        if self.script.reregistration == Reregistration::Once || self.state == State::Idle {
            return;
        }
        match self.state {
            State::Waiting { .. } | State::Dark { .. } | State::Expecting { .. } => {
                self.record(now, Verdict::Interrupted);
            }
            State::Leaving { .. } => {
                self.record(now, Verdict::Interrupted);
                self.halted = true;
            }
            State::Idle | State::Ready | State::Done => {}
        }
        if self.halted {
            self.state = State::Done;
            return;
        }
        self.cursor = 0;
        self.state = State::Idle;
    }

    fn record(&mut self, at: WireInstant, verdict: Verdict) {
        let line = self.script.steps[self.cursor].line;
        let registration = self.registration;
        self.outcomes.push(Outcome { step: self.cursor, line, at, verdict, registration });
    }

    fn react(&mut self, reaction: Reaction, now: WireInstant) -> Vec<Command> {
        match reaction {
            Reaction::Continue => {
                self.cursor += 1;
                self.state = State::Ready;
                self.resume(now)
            }
            Reaction::Stop => {
                self.state = State::Done;
                self.halted = true;
                Vec::new()
            }
            Reaction::Deregister { after } => {
                let at = now + after;
                self.state = State::Leaving { at };
                self.resume(now)
            }
        }
    }

    /// Carry on from `now`: run every step that is due, up to one that waits.
    pub fn resume(&mut self, now: WireInstant) -> Vec<Command> {
        let mut commands = Vec::new();
        loop {
            match self.state {
                State::Idle | State::Done => return commands,
                State::Waiting { until } => {
                    if now < until {
                        commands.push(Command::WakeAt(until));
                        return commands;
                    }
                    self.record(until, Verdict::Waited);
                    self.cursor += 1;
                    self.state = State::Ready;
                }
                State::Dark { until } => {
                    if now < until {
                        commands.push(Command::WakeAt(until));
                        return commands;
                    }
                    self.cursor += 1;
                    self.state = State::Ready;
                }
                State::Expecting { deadline } => {
                    if now < deadline {
                        commands.push(Command::WakeAt(deadline));
                        return commands;
                    }
                    let Action::Expect { on_timeout, .. } = self.script.steps[self.cursor].action
                    else {
                        unreachable!("only an expect step waits for a frame");
                    };
                    self.record(deadline, Verdict::TimedOut);
                    commands.extend(self.react(on_timeout, deadline));
                    return commands;
                }
                State::Leaving { at } => {
                    if now < at {
                        commands.push(Command::WakeAt(at));
                        return commands;
                    }
                    self.record(at, Verdict::Deregistered);
                    self.state = State::Done;
                    self.halted = true;
                    commands.push(Command::Deregister);
                    return commands;
                }
                State::Ready => {
                    let Some(step) = self.script.steps.get(self.cursor) else {
                        self.state = State::Done;
                        return commands;
                    };
                    if step.action.once_per_run() {
                        if self.played[self.cursor] {
                            self.record(now, Verdict::Skipped);
                            self.cursor += 1;
                            continue;
                        }
                        self.played[self.cursor] = true;
                    }
                    match step.action.clone() {
                        Action::Send { oui, code, body } => {
                            self.record(now, Verdict::Sent);
                            commands.push(Command::Send { oui, code, body });
                            self.cursor += 1;
                        }
                        Action::Wait(span) => self.state = State::Waiting { until: now + span },
                        Action::Cut(span) => {
                            let until = now + span;
                            self.record(now, Verdict::Cut);
                            commands.push(Command::Cut { until });
                            self.state = State::Dark { until };
                        }
                        Action::Expect { within, .. } => {
                            self.state = State::Expecting { deadline: now + within };
                        }
                        Action::Deregister => {
                            self.record(now, Verdict::Deregistered);
                            commands.push(Command::Deregister);
                            self.cursor += 1;
                            // Replayed, the teardown ends this pass: nothing
                            // after it is played on the registration it ended.
                            if self.script.reregistration == Reregistration::Replay {
                                return commands;
                            }
                        }
                    }
                }
            }
        }
    }

    /// An extended OAMPDU arrived: `body` is what follows its code octet.
    pub fn on_upstream(&mut self, oui: Oui, code: u8, body: &[u8], now: WireInstant) -> Vec<Command> {
        if self.script.ignores(oui, code) {
            self.ignored_seen += 1;
            return Vec::new();
        }
        let State::Expecting { deadline } = self.state else {
            self.unsolicited_seen += 1;
            return Vec::new();
        };
        // A frame that lands after the deadline is late, whatever it holds:
        // the timeout is decided first, at the instant it was due.
        if now >= deadline {
            let mut commands = self.resume(now);
            commands.extend(self.on_upstream(oui, code, body, now));
            return commands;
        }
        let Action::Expect { selector, ref pattern, on_mismatch, .. } =
            self.script.steps[self.cursor].action
        else {
            unreachable!("only an expect step waits for a frame");
        };
        if selector.oui != oui {
            self.unsolicited_seen += 1;
            return Vec::new();
        }
        if selector.selects(oui, code) && pattern.matches(body) {
            self.record(now, Verdict::Matched);
            self.cursor += 1;
            self.state = State::Ready;
            return self.resume(now);
        }
        self.record(now, Verdict::Mismatched { oui, code });
        self.react(on_mismatch, now)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An identifier no container on this link uses.
    const OUI: Oui = Oui([0xAA, 0xBB, 0xCC]);
    const OTHER_OUI: Oui = Oui([0xAA, 0xBB, 0xCD]);
    const REQUEST: u8 = 0x10;
    const ANSWER: u8 = 0x11;

    fn at(ms: u64) -> WireInstant {
        WireInstant::ZERO + WireDuration::from_ms(ms)
    }

    fn script(text: &str) -> Script {
        Script::parse(text).expect("the fixture parses")
    }

    fn verdicts(player: &Player) -> Vec<Verdict> {
        player.outcomes().iter().map(|o| o.verdict.clone()).collect()
    }

    /// One request, and an answer whose second octet is left free.
    const REQUEST_AND_ANSWER: &str = "\
        send   AABBCC/10 0102\n\
        expect AABBCC/11 80?? within 100 mismatch deregister-after 10 timeout deregister-after 2000\n\
        send   AABBCC/10 03\n";

    #[test]
    fn a_line_is_read_into_the_step_it_spells() {
        let s = script("# heading\n\nignore AABBCC/08\nsend AA-BB-CC/10 01 0203 # tail\nwait 5\nderegister\n");
        assert_eq!(s.ignored, vec![Selector { oui: OUI, code: Some(0x08) }]);
        assert_eq!(
            s.steps,
            vec![
                Step { line: 4, action: Action::Send { oui: OUI, code: REQUEST, body: vec![1, 2, 3] } },
                Step { line: 5, action: Action::Wait(WireDuration::from_ms(5)) },
                Step { line: 6, action: Action::Deregister },
            ]
        );
    }

    #[test]
    fn the_destination_of_oampdus_is_left_alone_unless_the_script_says() {
        assert_eq!(script("wait 1\n").multicast_oam, None);
        assert_eq!(script("oam-destination multicast\n").multicast_oam, Some(true));
        assert_eq!(script("oam-destination unicast\n").multicast_oam, Some(false));
        assert_eq!(Script::parse("oam-destination broadcast\n").unwrap_err().line, 1);
    }

    #[test]
    fn an_expect_line_carries_its_pattern_and_both_reactions() {
        let s = script("expect AABBCC/* 80 ?? 01 within 250 timeout continue mismatch deregister-after 10");
        assert_eq!(
            s.steps[0].action,
            Action::Expect {
                selector: Selector { oui: OUI, code: None },
                pattern: Pattern(vec![Some(0x80), None, Some(0x01)]),
                within: WireDuration::from_ms(250),
                on_mismatch: Reaction::Deregister { after: WireDuration::from_ms(10) },
                on_timeout: Reaction::Continue,
            }
        );
    }

    #[test]
    fn a_line_that_cannot_be_read_names_itself() {
        for (text, line) in [
            ("send AABBCC/10 0\n", 1),
            ("wait 5\nsend AABBCC/* 01\n", 2),
            ("send AABBCC/10 ??\n", 1),
            ("expect AABBCC/11 01\n", 1),
            ("expect AABBCC/11 within 5 mismatch later\n", 1),
            ("resend AABBCC/10 01\n", 1),
        ] {
            assert_eq!(Script::parse(text).unwrap_err().line, line, "{text:?}");
        }
    }

    #[test]
    fn a_pattern_is_a_prefix_and_a_short_body_does_not_match() {
        let p = Pattern(vec![Some(0x80), None]);
        assert!(p.matches(&[0x80, 0x55, 0x00]));
        assert!(!p.matches(&[0x80]));
        assert!(!p.matches(&[0x00, 0x55]));
        assert!(Pattern::default().matches(&[]));
    }

    #[test]
    fn nothing_is_played_before_the_start() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        assert!(p.resume(at(50)).is_empty());
        assert!(p.on_upstream(OUI, ANSWER, &[0x80, 0], at(60)).is_empty());
        assert!(!p.started());
    }

    #[test]
    fn the_expected_answer_lets_the_next_step_go() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        assert_eq!(
            p.start(at(0)),
            vec![
                Command::Send { oui: OUI, code: REQUEST, body: vec![1, 2] },
                Command::WakeAt(at(100)),
            ]
        );
        assert_eq!(
            p.on_upstream(OUI, ANSWER, &[0x80, 0x7F, 0x00], at(3)),
            vec![Command::Send { oui: OUI, code: REQUEST, body: vec![3] }]
        );
        assert_eq!(verdicts(&p), vec![Verdict::Sent, Verdict::Matched, Verdict::Sent]);
        assert!(p.finished());
        // The deadline of the answered step still fires; it must change nothing.
        assert!(p.resume(at(100)).is_empty());
    }

    #[test]
    fn a_wrong_answer_is_refused_at_its_own_delay() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        assert_eq!(p.on_upstream(OUI, ANSWER, &[0x00, 0x00], at(3)), vec![Command::WakeAt(at(13))]);
        assert!(p.resume(at(12)).contains(&Command::WakeAt(at(13))));
        assert_eq!(p.resume(at(13)), vec![Command::Deregister]);
        assert_eq!(
            verdicts(&p),
            vec![Verdict::Sent, Verdict::Mismatched { oui: OUI, code: ANSWER }, Verdict::Deregistered]
        );
        assert!(p.finished());
    }

    #[test]
    fn the_right_octets_under_the_wrong_code_are_a_wrong_answer() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        p.on_upstream(OUI, REQUEST, &[0x80, 0x00], at(3));
        assert_eq!(verdicts(&p)[1], Verdict::Mismatched { oui: OUI, code: REQUEST });
    }

    #[test]
    fn silence_is_decided_at_the_deadline_not_when_the_loop_got_there() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        // The host comes back late; the timeout is still stamped where it fell.
        assert_eq!(p.resume(at(180)), vec![Command::WakeAt(at(2100))]);
        assert_eq!(
            p.outcomes()[1],
            Outcome { step: 1, line: 2, at: at(100), verdict: Verdict::TimedOut, registration: 1 }
        );
        assert_eq!(p.resume(at(2100)), vec![Command::Deregister]);
    }

    #[test]
    fn a_frame_after_the_deadline_is_late_whatever_it_holds() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        p.on_upstream(OUI, ANSWER, &[0x80, 0x00], at(150));
        assert_eq!(verdicts(&p)[1], Verdict::TimedOut);
        assert_eq!(p.unsolicited_seen, 1);
    }

    #[test]
    fn a_timeout_that_continues_plays_the_rest() {
        let mut p = Player::new(script(
            "send AABBCC/10 01\nexpect AABBCC/11 within 40 timeout continue\nsend AABBCC/10 02\n",
        ));
        p.start(at(0));
        assert_eq!(p.resume(at(40)), vec![Command::Send { oui: OUI, code: REQUEST, body: vec![2] }]);
        assert_eq!(verdicts(&p), vec![Verdict::Sent, Verdict::TimedOut, Verdict::Sent]);
    }

    #[test]
    fn what_the_script_ignores_or_does_not_wait_for_is_counted_not_judged() {
        let mut p = Player::new(script(
            "ignore AABBCC/08\nwait 10\nsend AABBCC/10 01\nexpect AABBCC/11 within 100\n",
        ));
        assert_eq!(p.start(at(0)), vec![Command::WakeAt(at(10))]);
        // While waiting, nothing is expected.
        assert!(p.on_upstream(OUI, ANSWER, &[], at(2)).is_empty());
        p.resume(at(10));
        // Ignored, and another container's traffic: neither ends the step.
        assert!(p.on_upstream(OUI, 0x08, &[], at(12)).is_empty());
        assert!(p.on_upstream(OTHER_OUI, ANSWER, &[], at(13)).is_empty());
        p.on_upstream(OUI, ANSWER, &[], at(14));
        assert_eq!((p.ignored_seen, p.unsolicited_seen), (1, 2));
        assert_eq!(verdicts(&p), vec![Verdict::Waited, Verdict::Sent, Verdict::Matched]);
    }

    #[test]
    fn a_deregister_step_played_once_tears_down_and_the_script_goes_on() {
        let mut p = Player::new(script("reregistration once\nderegister\nwait 5\n"));
        assert_eq!(p.start(at(0)), vec![Command::Deregister, Command::WakeAt(at(5))]);
        p.on_teardown(at(0));
        p.resume(at(5));
        assert_eq!(verdicts(&p), vec![Verdict::Deregistered, Verdict::Waited]);
        assert!(p.finished());
        // And a registration after it plays nothing.
        assert!(p.start(at(10)).is_empty());
    }

    #[test]
    fn a_cut_takes_the_light_off_and_the_step_ends_when_it_comes_back() {
        let s = script("cut 3000\nsend AABBCC/10 01\n");
        assert_eq!(s.steps[0].action, Action::Cut(WireDuration::from_ms(3000)));
        for bad in ["cut\n", "cut 0\n", "cut 3 s\n"] {
            assert_eq!(Script::parse(bad).unwrap_err().line, 1, "{bad:?}");
        }
        let mut p = Player::new(s);
        assert_eq!(
            p.start(at(10)),
            vec![Command::Cut { until: at(3010) }, Command::WakeAt(at(3010))]
        );
        assert!(p.resume(at(3009)).contains(&Command::WakeAt(at(3010))));
        assert_eq!(p.resume(at(3010)), vec![Command::Send { oui: OUI, code: REQUEST, body: vec![1] }]);
        assert_eq!(verdicts(&p), vec![Verdict::Cut, Verdict::Sent]);
        assert_eq!(p.outcomes()[0].at, at(10));
    }

    #[test]
    fn a_replay_cuts_the_fibre_once_per_registration_in_order() {
        let mut p = Player::new(script("send AABBCC/10 01\ncut 3000\ncut 120000\n"));
        p.start(at(0));
        // The cut takes the registration down before the light is back.
        p.on_teardown(at(2000));
        let second = p.start(at(5000));
        assert_eq!(
            second,
            vec![
                Command::Send { oui: OUI, code: REQUEST, body: vec![1] },
                Command::Cut { until: at(125_000) },
                Command::WakeAt(at(125_000)),
            ]
        );
        p.on_teardown(at(6000));
        assert_eq!(
            p.start(at(130_000)),
            vec![Command::Send { oui: OUI, code: REQUEST, body: vec![1] }]
        );
        assert_eq!(
            verdicts(&p),
            vec![
                Verdict::Sent, Verdict::Cut, Verdict::Interrupted,
                Verdict::Sent, Verdict::Skipped, Verdict::Cut, Verdict::Interrupted,
                Verdict::Sent, Verdict::Skipped, Verdict::Skipped,
            ]
        );
        assert!(p.finished());
    }

    #[test]
    fn what_a_reregistration_does_is_read_from_the_header() {
        assert_eq!(script("wait 1\n").reregistration, Reregistration::Replay);
        assert_eq!(script("reregistration once\n").reregistration, Reregistration::Once);
        assert_eq!(script("reregistration replay\n").reregistration, Reregistration::Replay);
        assert_eq!(Script::parse("reregistration twice\n").unwrap_err().line, 1);
    }

    #[test]
    fn a_new_registration_replays_the_script_from_the_top() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        p.on_upstream(OUI, ANSWER, &[0x80, 0x00], at(3));
        assert!(p.finished());
        p.on_teardown(at(50));
        assert!(!p.finished() && !p.started(), "waiting for the next registration");
        assert_eq!(
            p.start(at(1000)),
            vec![
                Command::Send { oui: OUI, code: REQUEST, body: vec![1, 2] },
                Command::WakeAt(at(1100)),
            ]
        );
        assert_eq!(p.registration(), 2);
        let passes: Vec<u32> = p.outcomes().iter().map(|o| o.registration).collect();
        assert_eq!(passes, vec![1, 1, 1, 2]);
    }

    #[test]
    fn a_teardown_under_a_step_interrupts_it_and_the_next_pass_starts_over() {
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        p.on_teardown(at(40));
        assert_eq!(p.outcomes()[1].verdict, Verdict::Interrupted);
        assert_eq!(p.outcomes()[1].at, at(40));
        // The deadline of the interrupted step fires into nothing.
        assert!(p.resume(at(100)).is_empty());
        assert_eq!(p.outcomes().len(), 2);
        p.start(at(500));
        assert_eq!(p.outcomes()[2], Outcome {
            step: 0,
            line: 1,
            at: at(500),
            verdict: Verdict::Sent,
            registration: 2,
        });
    }

    #[test]
    fn a_script_a_reaction_ended_is_not_replayed() {
        // Stopped by a mismatch.
        let mut p = Player::new(script("send AABBCC/10 01\nexpect AABBCC/11 80 within 100\n"));
        p.start(at(0));
        p.on_upstream(OUI, ANSWER, &[0x00], at(3));
        p.on_teardown(at(10));
        assert!(p.start(at(1000)).is_empty());
        assert!(p.finished());
        // Torn down by its own reaction.
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        p.on_upstream(OUI, ANSWER, &[0x00, 0x00], at(3));
        assert_eq!(p.resume(at(13)), vec![Command::Deregister]);
        p.on_teardown(at(13));
        assert!(p.start(at(1000)).is_empty());
        // Torn down by something else while that teardown was pending.
        let mut p = Player::new(script(REQUEST_AND_ANSWER));
        p.start(at(0));
        p.on_upstream(OUI, ANSWER, &[0x00, 0x00], at(3));
        p.on_teardown(at(5));
        assert_eq!(p.outcomes().last().map(|o| o.verdict.clone()), Some(Verdict::Interrupted));
        assert!(p.start(at(1000)).is_empty());
        assert!(p.resume(at(13)).is_empty(), "the pending teardown is not sent any more");
    }

    #[test]
    fn a_replay_skips_the_teardown_an_earlier_pass_injected() {
        let mut p = Player::new(script("send AABBCC/10 01\nderegister\nsend AABBCC/10 02\n"));
        assert_eq!(
            p.start(at(0)),
            vec![Command::Send { oui: OUI, code: REQUEST, body: vec![1] }, Command::Deregister]
        );
        // Nothing after the teardown is played on the registration it ended.
        p.on_teardown(at(0));
        assert_eq!(
            p.start(at(1000)),
            vec![
                Command::Send { oui: OUI, code: REQUEST, body: vec![1] },
                Command::Send { oui: OUI, code: REQUEST, body: vec![2] },
            ]
        );
        assert_eq!(
            verdicts(&p),
            vec![Verdict::Sent, Verdict::Deregistered, Verdict::Sent, Verdict::Skipped, Verdict::Sent]
        );
        assert!(p.finished());
    }
}
