//! MPCP registration state, as the OLT sees it.

use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize)]
pub enum MpcpState {
    /// Waiting for a REGISTER_REQ.
    #[default]
    Idle,
    /// A REGISTER_REQ arrived; the REGISTER response is pending.
    Discovery,
    /// REGISTER sent, waiting for the REGISTER_ACK that closes the handshake.
    WaitAck,
    /// The link is registered.
    Registered,
}

impl MpcpState {
    /// True once discovery has started, whether or not it has completed.
    pub fn is_active(self) -> bool {
        self != Self::Idle
    }
}

impl fmt::Display for MpcpState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Self::Idle => "idle",
            Self::Discovery => "discovery",
            Self::WaitAck => "wait_ack",
            Self::Registered => "registered",
        };
        f.write_str(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_is_the_only_inactive_state() {
        assert!(!MpcpState::default().is_active());
        for s in [
            MpcpState::Discovery,
            MpcpState::WaitAck,
            MpcpState::Registered,
        ] {
            assert!(s.is_active(), "{s}");
        }
    }
}
