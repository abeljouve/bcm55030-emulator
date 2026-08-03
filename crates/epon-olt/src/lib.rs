//! An EPON OLT that runs its own loop.
//!
//! The far end of a passive optical network, as a library: MPCP discovery and
//! registration (IEEE 802.3 clause 64), OAM discovery and keepalives (clause
//! 57), the organization-specific channel their extensions are carried on, a
//! delay model for the fibre between the two ends, and the scheduler all of it
//! runs on.
//!
//! What it deliberately does not have is a host. The peer owns a clock it does
//! not advance, emits frames it does not transmit, and consumes frames it does
//! not receive. Three calls join it to whatever does:
//!
//! ```
//! use epon_olt::{clock::{WireDuration, WireInstant}, link::Link};
//!
//! let mut link = Link::default();
//! link.set_link(true, WireInstant::ZERO);
//! link.advance_to(WireInstant::ZERO + WireDuration::from_ms(5));
//! // Frames the peer decided to send are on their way to the far end.
//! assert!(link.peer.counters.gates_sent > 0);
//! ```
//!
//! - [`clock`] wire time: the one time base everything is expressed in
//! - [`sched`] the due-time queue the peer's loop runs on
//! - [`peer`] the peer itself: state machine, timers, counters
//! - [`fibre`] one direction of link: travel time, jitter, finite depth
//! - [`link`] a peer with fibre either side of it
//! - [`onu`] a minimal responder, to exercise the peer with nothing else present
//! - [`types`] Ethernet primitives shared by the protocol modules
//! - [`mpcp`] MPCPDU encode/decode (clause 64)
//! - [`oam`] OAMPDU encode/decode (clause 57)
//! - [`extended`] organization-specific OAMPDUs and their variables
//! - [`decode`] frame dissection, for a packet view or a log
//! - [`state`] MPCP registration state

pub mod clock;
pub mod decode;
pub mod extended;
pub mod fibre;
pub mod link;
pub mod mpcp;
pub mod oam;
pub mod onu;
pub mod peer;
pub mod sched;
pub mod state;
pub mod types;

pub use clock::{WireDuration, WireInstant};
pub use fibre::{Fibre, FibreConfig};
pub use link::Link;
pub use peer::{Counters, Emitted, LoggedFrame, Peer, PeerConfig};
pub use state::MpcpState;
pub use types::{EtherType, Llid, MacAddr};
