//! The event scheduler the peer's loop runs on.
//!
//! Everything the peer does on its own initiative — a discovery GATE, a
//! keepalive, the expiry of a registration — is an event with a due time.
//! Nothing is decided by "has enough elapsed since the last one", which is
//! how a model ends up doing work in proportion to how often it is polled
//! rather than to how much link time has passed.
//!
//! Rescheduling is by cancel-and-arm rather than by mutating the heap: an
//! armed timer holds a generation, and an entry whose generation is stale is
//! discarded when it surfaces. That keeps the ordering total and the run
//! reproducible — the same sequence of arms and cancels always yields the
//! same sequence of firings.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::clock::{WireDuration, WireInstant};

/// One armed timer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Entry<E> {
    at: WireInstant,
    /// Breaks ties between events armed for the same instant, in arming
    /// order, so a run never depends on how the heap happens to be shaped.
    seq: u64,
    generation: u64,
    event: E,
}

impl<E: Eq> Ord for Entry<E> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.at.cmp(&other.at).then(self.seq.cmp(&other.seq))
    }
}

impl<E: Eq> PartialOrd for Entry<E> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// A due-time-ordered queue of events.
///
/// `E` is the caller's event kind. Each distinct value is its own timer: arming
/// one that is already armed replaces it rather than adding a second firing.
#[derive(Clone, Debug)]
pub struct Scheduler<E> {
    heap: BinaryHeap<Reverse<Entry<E>>>,
    /// Current generation per event kind. An entry that surfaces with an
    /// older generation was cancelled or re-armed, and is dropped.
    generations: Vec<(E, u64)>,
    next_seq: u64,
}

impl<E: Copy + Eq> Default for Scheduler<E> {
    fn default() -> Self {
        Self { heap: BinaryHeap::new(), generations: Vec::new(), next_seq: 0 }
    }
}

impl<E: Copy + Eq> Scheduler<E> {
    pub fn new() -> Self {
        Self::default()
    }

    fn generation(&mut self, event: E) -> &mut u64 {
        if let Some(index) = self.generations.iter().position(|(e, _)| *e == event) {
            return &mut self.generations[index].1;
        }
        self.generations.push((event, 0));
        &mut self.generations.last_mut().expect("just pushed").1
    }

    /// Arm `event` for `at`, replacing any previous arming of it.
    pub fn arm_at(&mut self, event: E, at: WireInstant) {
        let generation = {
            let g = self.generation(event);
            *g += 1;
            *g
        };
        let seq = self.next_seq;
        self.next_seq += 1;
        self.heap.push(Reverse(Entry { at, seq, generation, event }));
    }

    /// Arm `event` for `delay` after `now`.
    pub fn arm_in(&mut self, event: E, now: WireInstant, delay: WireDuration) {
        self.arm_at(event, now + delay);
    }

    /// Disarm `event`. A firing already in the heap is discarded when it
    /// surfaces.
    pub fn cancel(&mut self, event: E) {
        *self.generation(event) += 1;
    }

    /// True when `event` has a live arming.
    pub fn is_armed(&mut self, event: E) -> bool {
        let generation = *self.generation(event);
        self.heap
            .iter()
            .any(|Reverse(e)| e.event == event && e.generation == generation)
    }

    /// When `event` is next due, if it is armed.
    pub fn due_at(&mut self, event: E) -> Option<WireInstant> {
        let generation = *self.generation(event);
        self.heap
            .iter()
            .filter(|Reverse(e)| e.event == event && e.generation == generation)
            .map(|Reverse(e)| e.at)
            .min()
    }

    /// Pop the next event due at or before `now`, with the instant it was due
    /// at — not `now`. A peer that acts on an event acts at the moment the
    /// event was scheduled for, however late the loop got round to it.
    pub fn pop_due(&mut self, now: WireInstant) -> Option<(WireInstant, E)> {
        while let Some(Reverse(entry)) = self.heap.peek().copied() {
            if entry.at > now {
                return None;
            }
            self.heap.pop();
            if *self.generation(entry.event) == entry.generation {
                return Some((entry.at, entry.event));
            }
        }
        None
    }

    /// Instant the next live event is due at.
    pub fn next_due(&mut self) -> Option<WireInstant> {
        let live: Vec<_> = self.heap.iter().map(|Reverse(e)| (e.event, e.generation, e.at)).collect();
        live.into_iter()
            .filter(|(event, generation, _)| *self.generation(*event) == *generation)
            .map(|(_, _, at)| at)
            .min()
    }

    pub fn clear(&mut self) {
        self.heap.clear();
        self.generations.clear();
        self.next_seq = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Ev {
        Gate,
        Keepalive,
        Lifetime,
    }

    fn at(ms: u64) -> WireInstant {
        WireInstant::from_ps(WireDuration::from_ms(ms).as_ps())
    }

    #[test]
    fn events_come_out_in_due_order_not_arming_order() {
        let mut s = Scheduler::new();
        s.arm_at(Ev::Lifetime, at(60_000));
        s.arm_at(Ev::Gate, at(1000));
        s.arm_at(Ev::Keepalive, at(804));

        assert_eq!(s.pop_due(at(2000)), Some((at(804), Ev::Keepalive)));
        assert_eq!(s.pop_due(at(2000)), Some((at(1000), Ev::Gate)));
        assert_eq!(s.pop_due(at(2000)), None);
        assert_eq!(s.next_due(), Some(at(60_000)));
    }

    #[test]
    fn an_event_fires_at_its_due_time_however_late_the_loop_is() {
        let mut s = Scheduler::new();
        s.arm_at(Ev::Gate, at(1000));
        // The loop only gets here at t+5 s; the GATE still belongs to t+1 s,
        // and a periodic timer re-armed from it must not inherit the delay.
        assert_eq!(s.pop_due(at(5000)), Some((at(1000), Ev::Gate)));
    }

    #[test]
    fn re_arming_replaces_rather_than_duplicates() {
        let mut s = Scheduler::new();
        s.arm_at(Ev::Gate, at(1000));
        s.arm_at(Ev::Gate, at(2000));
        assert_eq!(s.pop_due(at(9000)), Some((at(2000), Ev::Gate)));
        assert_eq!(s.pop_due(at(9000)), None);
    }

    #[test]
    fn a_cancelled_event_never_fires() {
        let mut s = Scheduler::new();
        s.arm_at(Ev::Lifetime, at(60_000));
        assert!(s.is_armed(Ev::Lifetime));
        s.cancel(Ev::Lifetime);
        assert!(!s.is_armed(Ev::Lifetime));
        assert_eq!(s.pop_due(at(120_000)), None);
        assert_eq!(s.next_due(), None);
    }

    #[test]
    fn ties_break_in_arming_order() {
        let mut s = Scheduler::new();
        s.arm_at(Ev::Gate, at(500));
        s.arm_at(Ev::Keepalive, at(500));
        assert_eq!(s.pop_due(at(500)), Some((at(500), Ev::Gate)));
        assert_eq!(s.pop_due(at(500)), Some((at(500), Ev::Keepalive)));
    }

    #[test]
    fn due_at_reports_the_live_arming() {
        let mut s = Scheduler::new();
        s.arm_at(Ev::Gate, at(1000));
        assert_eq!(s.due_at(Ev::Gate), Some(at(1000)));
        s.arm_at(Ev::Gate, at(400));
        assert_eq!(s.due_at(Ev::Gate), Some(at(400)));
        s.cancel(Ev::Gate);
        assert_eq!(s.due_at(Ev::Gate), None);
    }
}
