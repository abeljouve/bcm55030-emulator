//! The span of fibre between the two ends, in one direction.
//!
//! A frame handed to a fibre does not appear at the other end at the instant
//! it was built. It is clocked out at the line rate, travels, and lands in a
//! queue of finite depth. Modelling that is what separates a duration the peer
//! decided from the duration the far end observes: the two differ by exactly
//! what happens here, and by nothing that is written into a constant.
//!
//! Two properties are load-bearing:
//!
//! - **Order is preserved.** Jitter is added to the arrival instant, never
//!   allowed to move a frame ahead of one sent before it. A fibre does not
//!   reorder, so neither does this.
//! - **Depth is finite.** What does not fit is dropped and counted. A queue
//!   that never loses anything turns a burst into a backlog that is delivered
//!   late but delivered, which is the one thing a real downstream never does.

use std::collections::VecDeque;

use crate::clock::{WireDuration, WireInstant};

/// Line rate of the downstream, in bits per second.
pub const RATE_10G: u64 = 10_312_500_000;
/// Line rate of the upstream, in bits per second.
///
/// The link is asymmetric: 10 Gbit/s down, 1 Gbit/s up.
pub const RATE_1G: u64 = 1_250_000_000;

/// Preamble, start-of-frame delimiter and inter-frame gap, in bytes. A frame
/// occupies the line for longer than its own length.
const FRAMING_OVERHEAD: u64 = 8 + 12;

/// How a direction of the link behaves.
#[derive(Clone, Copy, Debug)]
pub struct FibreConfig {
    /// One-way propagation delay. About 5 µs per kilometre of fibre.
    pub propagation: WireDuration,
    /// Upper bound of the extra delay applied per frame, drawn uniformly.
    /// Models what varies between one crossing and the next.
    pub jitter: WireDuration,
    /// Line rate, which sets how long a frame takes to clock out.
    pub rate_bps: u64,
    /// Frames the far end can hold before it starts losing them.
    pub depth: usize,
    /// Seed of the jitter draw. Fixed, so a run reproduces exactly.
    pub seed: u64,
}

impl FibreConfig {
    /// Distance light covers in fibre, at roughly 5 µs per kilometre.
    pub const fn propagation_for_km(km: u64) -> WireDuration {
        WireDuration::from_ps(km * 5 * crate::clock::PS_PER_US)
    }

    /// Downstream: 10 Gbit/s, the depth of a control-frame queue.
    pub fn downstream() -> Self {
        Self {
            propagation: Self::propagation_for_km(DEFAULT_DISTANCE_KM),
            jitter: WireDuration::ZERO,
            rate_bps: RATE_10G,
            depth: DEFAULT_DEPTH,
            seed: 0x2545_F491_4F6C_DD1D,
        }
    }

    /// Upstream: 1 Gbit/s.
    pub fn upstream() -> Self {
        Self { rate_bps: RATE_1G, ..Self::downstream() }
    }
}

/// Distance modelled by default. A drop within a few tens of kilometres is
/// the ordinary case; the figure matters only through its delay.
pub const DEFAULT_DISTANCE_KM: u64 = 20;

/// Frames a direction holds before it starts dropping.
///
/// Deep enough that the ordinary exchange never touches it, shallow enough
/// that a burst nothing is draining is lost rather than stored.
pub const DEFAULT_DEPTH: usize = 32;

/// A frame in flight, with the instant it lands.
#[derive(Clone, Debug)]
pub struct InFlight {
    pub arrives_at: WireInstant,
    pub frame: Vec<u8>,
}

/// One direction of the link.
#[derive(Clone, Debug)]
pub struct Fibre {
    pub config: FibreConfig,
    /// Frames in flight, ordered by arrival, which is also send order.
    queue: VecDeque<InFlight>,
    rng: Rng,
    /// Arrival of the last frame accepted, so order survives jitter.
    last_arrival: WireInstant,
    pub sent: u64,
    pub delivered: u64,
    /// Frames refused because the direction was full.
    pub dropped_full: u64,
}

impl Fibre {
    pub fn new(config: FibreConfig) -> Self {
        Self {
            rng: Rng::new(config.seed),
            config,
            queue: VecDeque::new(),
            last_arrival: WireInstant::ZERO,
            sent: 0,
            delivered: 0,
            dropped_full: 0,
        }
    }

    /// How long `len` bytes occupy the line.
    fn serialization(&self, len: usize) -> WireDuration {
        let bits = (len as u64 + FRAMING_OVERHEAD) * 8;
        WireDuration::from_ps(bits * 1_000_000_000_000 / self.config.rate_bps)
    }

    /// Hand a frame to the line at `at`. Returns false when the far end was
    /// full and the frame was dropped.
    pub fn send(&mut self, frame: Vec<u8>, at: WireInstant) -> bool {
        self.sent += 1;
        if self.queue.len() >= self.config.depth {
            self.dropped_full += 1;
            return false;
        }
        let jitter = if self.config.jitter.as_ps() == 0 {
            WireDuration::ZERO
        } else {
            WireDuration::from_ps(self.rng.below(self.config.jitter.as_ps() + 1))
        };
        let earliest = at + self.config.propagation + jitter + self.serialization(frame.len());
        // A fibre does not reorder: a frame cannot land before one that was
        // sent ahead of it, whatever the jitter drew.
        let arrives_at = earliest.max(self.last_arrival);
        self.last_arrival = arrives_at;
        self.queue.push_back(InFlight { arrives_at, frame });
        true
    }

    /// Take the next frame that has landed by `now`, with the instant it did.
    pub fn pop_arrived(&mut self, now: WireInstant) -> Option<InFlight> {
        let arrived = self.queue.front().is_some_and(|f| f.arrives_at <= now);
        if !arrived {
            return None;
        }
        self.delivered += 1;
        self.queue.pop_front()
    }

    /// Frames on the line, neither delivered nor dropped.
    pub fn in_flight(&self) -> usize {
        self.queue.len()
    }

    /// Instant the next frame lands, if there is one.
    pub fn next_arrival(&self) -> Option<WireInstant> {
        self.queue.front().map(|f| f.arrives_at)
    }

    /// Drop everything on the line. What a link going down does.
    pub fn clear(&mut self) {
        self.queue.clear();
        self.last_arrival = WireInstant::ZERO;
    }

    /// Round trip across this direction and back, for a frame of `len` bytes:
    /// what an exchange costs on top of whatever the far end decided.
    pub fn one_way_for(&self, len: usize) -> WireDuration {
        self.config.propagation + self.serialization(len)
    }
}

/// xorshift64*, so a jitter draw is reproducible and costs nothing.
#[derive(Clone, Copy, Debug)]
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }

    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, bound: u64) -> u64 {
        if bound == 0 {
            0
        } else {
            self.next() % bound
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(us: u64) -> WireInstant {
        WireInstant::from_ps(WireDuration::from_us(us).as_ps())
    }

    fn frame(len: usize) -> Vec<u8> {
        vec![0u8; len]
    }

    #[test]
    fn a_frame_lands_after_travel_and_serialization() {
        let mut f = Fibre::new(FibreConfig::downstream());
        assert!(f.send(frame(60), at(0)));
        // Nothing has landed while it is still on its way.
        assert!(f.pop_arrived(at(50)).is_none());
        let landed = f.pop_arrived(at(1000)).expect("lands");
        // 20 km is 100 µs; 80 bytes at 10 Gbit/s is well under a microsecond.
        let travel = landed.arrives_at - at(0);
        assert!(travel > WireDuration::from_us(100), "{travel}");
        assert!(travel < WireDuration::from_us(101), "{travel}");
        assert_eq!(f.delivered, 1);
    }

    #[test]
    fn the_upstream_clocks_out_eight_times_slower() {
        let down = Fibre::new(FibreConfig::downstream());
        let up = Fibre::new(FibreConfig::upstream());
        let d = down.one_way_for(60).as_ps() - down.config.propagation.as_ps();
        let u = up.one_way_for(60).as_ps() - up.config.propagation.as_ps();
        assert_eq!(u / d, RATE_10G / RATE_1G);
    }

    #[test]
    fn a_full_direction_drops_rather_than_stores() {
        let mut f = Fibre::new(FibreConfig { depth: 2, ..FibreConfig::downstream() });
        assert!(f.send(frame(60), at(0)));
        assert!(f.send(frame(60), at(1)));
        assert!(!f.send(frame(60), at(2)));
        assert_eq!(f.dropped_full, 1);
        assert_eq!(f.sent, 3);
        assert_eq!(f.in_flight(), 2);
        // Draining makes room again.
        f.pop_arrived(at(1000)).expect("lands");
        assert!(f.send(frame(60), at(1000)));
    }

    #[test]
    fn jitter_never_reorders() {
        let mut f = Fibre::new(FibreConfig {
            jitter: WireDuration::from_us(500),
            ..FibreConfig::downstream()
        });
        for _ in 0..64 {
            f.send(frame(60), at(0));
        }
        let mut previous = WireInstant::ZERO;
        let mut seen = 0;
        while let Some(landed) = f.pop_arrived(at(100_000)) {
            assert!(landed.arrives_at >= previous, "reordered");
            previous = landed.arrives_at;
            seen += 1;
        }
        assert_eq!(seen, FibreConfig::downstream().depth);
    }

    #[test]
    fn the_same_seed_draws_the_same_delays() {
        let config = FibreConfig {
            jitter: WireDuration::from_us(500),
            ..FibreConfig::downstream()
        };
        let arrivals = |c| {
            let mut f = Fibre::new(c);
            let mut out = Vec::new();
            for i in 0..8 {
                f.send(frame(60), at(i * 1000));
            }
            while let Some(l) = f.pop_arrived(at(1_000_000)) {
                out.push(l.arrives_at);
            }
            out
        };
        assert_eq!(arrivals(config), arrivals(config));
        // And a different seed draws differently, or the jitter is not doing
        // anything.
        assert_ne!(arrivals(config), arrivals(FibreConfig { seed: 7, ..config }));
    }

    #[test]
    fn no_jitter_means_a_fixed_delay() {
        let mut f = Fibre::new(FibreConfig::downstream());
        f.send(frame(60), at(0));
        f.send(frame(60), at(1000));
        let a = f.pop_arrived(at(1_000_000)).expect("lands");
        let b = f.pop_arrived(at(1_000_000)).expect("lands");
        assert_eq!(b.arrives_at - a.arrives_at, WireDuration::from_us(1000));
    }
}
