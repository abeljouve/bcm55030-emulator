//! Wire time.
//!
//! The peer runs on one time base and one only: picoseconds of link time,
//! counted from the moment the model started. Every interval it schedules and
//! every timestamp it puts on the wire is derived from that one quantity, so
//! a duration measured between two frames means the same thing as the constant
//! that produced it.
//!
//! This is deliberately not a wall clock. What drives it is left to whoever
//! owns the loop: a host running the peer against real equipment advances it
//! from a monotonic clock, and a host running it beside a simulated ONU
//! advances it from that simulation's own notion of elapsed time. Both see the
//! same peer, because the peer never asks where the time came from.

use std::fmt;
use std::ops::{Add, AddAssign, Sub};

/// Duration of one MPCP time quantum (IEEE 802.3 clause 64: 16 ns).
///
/// The value is the quantum the link actually runs at, measured off the
/// downstream, not the nominal 16 000 ps.
pub const TQ_PS: u64 = 16_007;

/// Picoseconds in a millisecond.
pub const PS_PER_MS: u64 = 1_000_000_000;
/// Picoseconds in a microsecond.
pub const PS_PER_US: u64 = 1_000_000;

/// An elapsed span of wire time, in picoseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct WireDuration(pub u64);

impl WireDuration {
    pub const ZERO: Self = Self(0);

    pub const fn from_ps(ps: u64) -> Self {
        Self(ps)
    }

    pub const fn from_us(us: u64) -> Self {
        Self(us * PS_PER_US)
    }

    pub const fn from_ms(ms: u64) -> Self {
        Self(ms * PS_PER_MS)
    }

    pub const fn as_ps(self) -> u64 {
        self.0
    }

    pub const fn as_ms(self) -> u64 {
        self.0 / PS_PER_MS
    }

    /// Milliseconds with three decimals, which is the resolution anything
    /// derived from a frame exchange is worth quoting at.
    pub fn as_ms_f64(self) -> f64 {
        self.0 as f64 / PS_PER_MS as f64
    }

    /// Time quanta this span is worth.
    pub const fn as_tq(self) -> u64 {
        self.0 / TQ_PS
    }

    pub const fn saturating_sub(self, other: Self) -> Self {
        Self(self.0.saturating_sub(other.0))
    }
}

impl fmt::Display for WireDuration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.3} ms", self.as_ms_f64())
    }
}

impl Add for WireDuration {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

/// A point on the wire clock, in picoseconds since the model started.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct WireInstant(pub u64);

impl WireInstant {
    pub const ZERO: Self = Self(0);

    pub const fn from_ps(ps: u64) -> Self {
        Self(ps)
    }

    pub const fn as_ps(self) -> u64 {
        self.0
    }

    /// The 32-bit MPCP timestamp this instant carries, in time quanta.
    ///
    /// Deriving it here rather than accumulating it per tick is what keeps a
    /// timestamp difference equal to the wall time between two frames: there
    /// is no rounding to drift out of, only one division.
    pub const fn mpcp_timestamp(self) -> u32 {
        (self.0 / TQ_PS) as u32
    }

    pub const fn saturating_sub(self, other: Self) -> WireDuration {
        WireDuration(self.0.saturating_sub(other.0))
    }
}

impl Add<WireDuration> for WireInstant {
    type Output = Self;
    fn add(self, rhs: WireDuration) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign<WireDuration> for WireInstant {
    fn add_assign(&mut self, rhs: WireDuration) {
        self.0 += rhs.0;
    }
}

impl Sub for WireInstant {
    type Output = WireDuration;
    fn sub(self, rhs: Self) -> WireDuration {
        WireDuration(self.0.saturating_sub(rhs.0))
    }
}

impl fmt::Display for WireInstant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "t+{:.3} ms", self.0 as f64 / PS_PER_MS as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_timestamp_difference_is_the_elapsed_time() {
        // One second of link time is worth this many quanta, and the
        // difference of the two timestamps must come back to the same figure
        // rather than to an accumulation of rounded steps.
        let start = WireInstant::from_ps(0);
        let end = start + WireDuration::from_ms(1000);
        let quanta = end.mpcp_timestamp().wrapping_sub(start.mpcp_timestamp());
        assert_eq!(quanta as u64, WireDuration::from_ms(1000).as_tq());
        assert_eq!(quanta, 62_472_668);
    }

    #[test]
    fn the_timestamp_wraps_where_thirty_two_bits_end() {
        let wrap = WireInstant::from_ps(TQ_PS << 32);
        assert_eq!(wrap.mpcp_timestamp(), 0);
        let just_before = WireInstant::from_ps((TQ_PS << 32) - TQ_PS);
        assert_eq!(just_before.mpcp_timestamp(), u32::MAX);
    }

    #[test]
    fn durations_convert_both_ways() {
        assert_eq!(WireDuration::from_ms(1).as_ps(), PS_PER_MS);
        assert_eq!(WireDuration::from_us(1000), WireDuration::from_ms(1));
        assert_eq!(WireDuration::from_ms(60_000).as_ms(), 60_000);
    }
}
