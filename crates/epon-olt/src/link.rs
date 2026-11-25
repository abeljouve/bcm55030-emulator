//! A peer with fibre either side of it.
//!
//! This is the whole composition: whoever drives it hands up frames and takes
//! down frames, and everything between — the peer's own loop, the travel time,
//! the finite queues — happens here. A host embedding the peer next to a
//! simulated ONU and a host running it against real equipment differ only in
//! what advances the clock and what the two ends of this type are wired to.

use crate::clock::WireInstant;
use crate::fibre::{Fibre, FibreConfig, InFlight};
use crate::peer::{Peer, PeerConfig};

/// The peer and both directions of the link it sits on.
#[derive(Clone)]
pub struct Link {
    pub peer: Peer,
    /// Peer to ONU.
    pub downstream: Fibre,
    /// ONU to peer.
    pub upstream: Fibre,
}

impl Default for Link {
    fn default() -> Self {
        Self::new(PeerConfig::default(), FibreConfig::downstream(), FibreConfig::upstream())
    }
}

impl Link {
    pub fn new(peer: PeerConfig, downstream: FibreConfig, upstream: FibreConfig) -> Self {
        Self {
            peer: Peer::new(peer),
            downstream: Fibre::new(downstream),
            upstream: Fibre::new(upstream),
        }
    }

    /// Run the link up to `now`.
    ///
    /// Arrivals first, then the peer's own events, then what it decided goes
    /// on the line — in that order, so a frame that lands can still be
    /// answered within the same step rather than the next one. The peer is
    /// brought up to each arrival before the frame is taken, so a cut it
    /// decided on the way is already on the line when the frame is judged.
    pub fn advance_to(&mut self, now: WireInstant) {
        while let Some(due) = self.upstream.next_arrival().filter(|t| *t <= now) {
            self.peer.advance_to(due);
            self.collect_cuts();
            if let Some(landed) = self.upstream.pop_arrived(due) {
                self.peer.deliver(&landed.frame, landed.arrives_at);
                self.collect_cuts();
            }
        }
        self.peer.advance_to(now);
        self.collect_cuts();
        for out in self.peer.take_downstream() {
            self.downstream.send(out.frame, out.at);
        }
    }

    /// Put a frame from the ONU end on the upstream. Returns false when it was
    /// dropped because the upstream was full.
    pub fn send_upstream(&mut self, frame: Vec<u8>, at: WireInstant) -> bool {
        self.upstream.send(frame, at)
    }

    /// Take the next frame that has reached the ONU end by `now`.
    pub fn poll_downstream(&mut self, now: WireInstant) -> Option<InFlight> {
        self.collect_cuts();
        self.downstream.pop_arrived(now)
    }

    /// Cut the fibre from `from` until `until`, in both directions.
    ///
    /// The peer is not told: nothing on the line says it was cut, and the
    /// peer finds out the way an OLT does, by what stops arriving.
    pub fn cut(&mut self, from: WireInstant, until: WireInstant) {
        self.downstream.cut(from, until);
        self.upstream.cut(from, until);
    }

    /// Put on the line the cuts the peer's script asked for.
    pub fn collect_cuts(&mut self) {
        for cut in self.peer.take_fibre_cuts() {
            self.cut(cut.from, cut.until);
        }
    }

    /// True while light crosses the fibre at `at`.
    pub fn lit_at(&self, at: WireInstant) -> bool {
        self.downstream.lit_at(at)
    }

    /// Frames lost because their crossing met a cut, both directions.
    pub fn lost_dark(&self) -> u64 {
        self.downstream.lost_dark + self.upstream.lost_dark
    }

    /// Bring the link up or down. Down empties both directions: a frame on a
    /// dark fibre does not arrive later, it is gone.
    pub fn set_link(&mut self, up: bool, at: WireInstant) {
        self.peer.set_link(up, at);
        if !up {
            self.downstream.clear();
            self.upstream.clear();
        }
    }

    /// Frames lost because a direction was full.
    pub fn dropped(&self) -> u64 {
        self.downstream.dropped_full + self.upstream.dropped_full
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::{WireDuration, WireInstant};
    use crate::onu::OnuResponder;
    use crate::peer::REGISTRATION_LIFETIME;
    use crate::types::MacAddr;

    fn at(ms: u64) -> WireInstant {
        WireInstant::from_ps(WireDuration::from_ms(ms).as_ps())
    }

    const ONU: MacAddr = MacAddr::new([0x02, 0x00, 0x00, 0x01, 0x02, 0x03]);

    /// Co-simulate the peer and a responder up to `until`, stepping by `step`.
    ///
    /// The step is the responder's reaction time: it only hears what has
    /// landed by the time the loop comes round. Which is the whole reason it
    /// is a parameter here — see the test that varies it.
    fn co_simulate(link: &mut Link, onu: &mut OnuResponder, until: WireInstant, step: WireDuration) {
        let mut now = WireInstant::ZERO;
        while now <= until {
            link.advance_to(now);
            while let Some(landed) = link.poll_downstream(now) {
                onu.deliver(&landed.frame, landed.arrives_at);
            }
            onu.advance_to(now);
            for (sent_at, frame) in onu.take_output() {
                // A frame cannot enter the line before the loop got to it.
                link.send_upstream(frame, sent_at.max(now));
            }
            now += step;
        }
    }

    #[test]
    fn a_frame_the_peer_sends_reaches_the_far_end_later() {
        let mut link = Link::default();
        link.set_link(true, at(0));
        link.advance_to(at(0));
        // The GATE was decided at t=0 but is still travelling.
        assert!(link.poll_downstream(at(0)).is_none());
        let landed = link.poll_downstream(at(1)).expect("lands within a millisecond");
        assert!(landed.arrives_at > at(0));
    }

    /// The oracle of the whole exercise. The peer's timer is a round minute;
    /// what the far end measures is that minute plus what the exchange costs.
    /// Neither figure is written down — the minute is configured, the rest is
    /// produced by the model, and here it comes out under a millisecond.
    #[test]
    fn what_the_far_end_observes_is_the_minute_plus_the_exchange() {
        let mut link = Link::default();
        let mut onu = OnuResponder::new(ONU);
        link.set_link(true, at(0));
        // A responder that hears the instant a frame lands: 100 µs of loop.
        co_simulate(&mut link, &mut onu, at(62_000), WireDuration::from_us(100));

        assert_eq!(link.peer.counters.registrations, 1);
        assert_eq!(link.peer.counters.deregistrations, 1);
        let up = onu.registered_at.expect("the grant landed");
        let down = onu.deregistered_at.expect("the teardown landed");

        let observed = down - up;
        assert!(observed > REGISTRATION_LIFETIME, "observed {observed}");
        let overhead = observed.saturating_sub(REGISTRATION_LIFETIME);
        assert!(
            overhead.as_ps() < WireDuration::from_ms(1).as_ps(),
            "two crossings of 20 km and one turnaround cannot account for more \
             than a millisecond, got {overhead}"
        );
    }

    /// And the counterweight: the gap the far end measures is dominated by how
    /// long the far end takes to answer, not by the fibre. A responder that
    /// only looks every 100 ms shifts the observed lifetime by about that
    /// much — which is why an observed figure is not a constant of the peer.
    #[test]
    fn the_far_ends_own_latency_lands_in_the_figure_it_measures() {
        let measure = |step: WireDuration| {
            let mut link = Link::default();
            let mut onu = OnuResponder::new(ONU);
            link.set_link(true, at(0));
            co_simulate(&mut link, &mut onu, at(62_000), step);
            let up = onu.registered_at.expect("the grant landed");
            let down = onu.deregistered_at.expect("the teardown landed");
            (down - up).saturating_sub(REGISTRATION_LIFETIME)
        };
        let prompt = measure(WireDuration::from_us(100));
        let sluggish = measure(WireDuration::from_ms(100));
        assert!(
            sluggish > prompt + WireDuration::from_ms(10),
            "a sluggish far end must show a longer life: {prompt} vs {sluggish}"
        );
    }

    #[test]
    fn a_burst_nothing_drains_is_lost_rather_than_stored() {
        let mut link = Link::default();
        link.set_link(true, at(0));
        // Nothing at the far end reads for a minute, while the peer keeps to
        // its cadence.
        link.advance_to(at(60_000));
        assert!(link.peer.counters.frames_sent > link.downstream.config.depth as u64);
        assert!(link.downstream.dropped_full > 0, "an undrained queue must lose frames");
        assert!(link.downstream.in_flight() <= link.downstream.config.depth);
    }

    /// The whole chain a cut sets off, with an end that keeps its own
    /// watchdog: the peer's grants go unanswered and it tears the link down,
    /// its REGISTER is lost on the dark fibre, the far end gives its LLID up
    /// when GATEs stop, and once the light is back it registers again and
    /// the script is played again without cutting a second time.
    #[test]
    fn a_cut_tears_the_link_down_and_the_light_coming_back_brings_it_up_again() {
        let mut link = Link::default();
        let mut onu = OnuResponder::new(ONU);
        link.peer.set_script(crate::script::Script::parse("wait 500\ncut 3000\nwait 10\n").expect("parses"));
        link.set_link(true, at(0));
        co_simulate(&mut link, &mut onu, at(30_000), WireDuration::from_ms(1));

        let peer = &link.peer;
        assert_eq!(peer.counters.registrations, 2, "{:?}", peer.counters);
        assert_eq!(peer.counters.mpcp_timeouts, 1);
        assert_eq!(onu.counters.watchdog_expiries, 1);
        assert_eq!(onu.counters.deregisters_seen, 0, "the peer's teardown was lost on the line");
        assert!(link.downstream.lost_dark > 0 && link.upstream.lost_dark > 0);
        assert_eq!(link.downstream.cuts().len(), 1, "cut once, not on every registration");
        let player = peer.script().expect("scripted");
        assert_eq!(player.registration(), 2);
        assert!(player.finished());
        use crate::script::Verdict;
        let verdicts: Vec<Verdict> = player.outcomes().iter().map(|o| o.verdict.clone()).collect();
        assert_eq!(
            verdicts,
            vec![
                Verdict::Waited, Verdict::Cut, Verdict::Interrupted,
                Verdict::Waited, Verdict::Skipped, Verdict::Waited,
            ]
        );
        assert!(peer.counters.register_requests_accounted_for());
        assert!(peer.counters.upstream_mpcpdus_accounted_for());
    }

    /// A cut shorter than the watchdog loses frames and nothing else.
    #[test]
    fn a_cut_shorter_than_the_watchdog_leaves_the_registration_up() {
        let mut link = Link::default();
        let mut onu = OnuResponder::new(ONU);
        link.peer.set_script(crate::script::Script::parse("cut 300\n").expect("parses"));
        link.set_link(true, at(0));
        co_simulate(&mut link, &mut onu, at(20_000), WireDuration::from_ms(1));
        assert_eq!(link.peer.counters.registrations, 1);
        assert_eq!(link.peer.counters.mpcp_timeouts, 0);
        assert_eq!(onu.counters.watchdog_expiries, 0);
        assert!(link.lost_dark() > 0);
    }

    #[test]
    fn a_dark_fibre_carries_nothing_that_was_already_on_it() {
        let mut link = Link::default();
        link.set_link(true, at(0));
        link.advance_to(at(0));
        assert!(link.downstream.in_flight() > 0);
        link.set_link(false, at(0));
        assert_eq!(link.downstream.in_flight(), 0);
        assert!(link.poll_downstream(at(10_000)).is_none());
    }
}
