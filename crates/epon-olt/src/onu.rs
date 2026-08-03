//! A minimal ONU, enough to exercise the peer.
//!
//! This is not a model of any particular equipment and must never be used as
//! one: it does the smallest thing that keeps a link alive — slave to the
//! discovery window, ask to register, acknowledge the grant, answer OAM — so
//! that the peer can be exercised end to end with nothing else present.
//!
//! Its value is as a control. When the peer misbehaves against real firmware,
//! running the same peer against this tells you whether the fault is in the
//! peer at all.

use crate::clock::{WireDuration, WireInstant};
use crate::mpcp::{self, RegisterAckFlag, RegisterFlag, RegisterReqBody, RegisterReqFlag};
use crate::oam;
use crate::types::{EtherType, Llid, MacAddr};

/// How often the responder sends its own Information OAMPDU.
pub const KEEPALIVE_INTERVAL: WireDuration = WireDuration::from_ms(1000);
/// Windows this responder can use, as bits of the discovery information
/// field. Whatever a GATE offers is narrowed to this before being echoed.
pub const WINDOWS_SUPPORTED: u16 = 0x0011;
/// Laser on and off times this responder declares.
pub const LASER_TQ: u8 = 32;
/// Delay between a discovery GATE and the request it provokes. An ONU waits a
/// random part of the window so several do not collide; one waits a fixed
/// part, which is enough here and keeps the run reproducible.
pub const REQUEST_DELAY: WireDuration = WireDuration::from_us(200);

/// What the responder has been told.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OnuCounters {
    /// Discovery GATEs — the only kind this responder answers.
    pub gates_seen: u64,
    /// GATEs without the discovery flag. This responder has nothing to
    /// do with one, and neither has the firmware: both drop it at the
    /// first test. Counting it is what tells "none arrived" apart from
    /// "they arrived and went nowhere".
    pub gates_normal_seen: u64,
    pub registers_seen: u64,
    pub deregisters_seen: u64,
    pub oam_seen: u64,
    pub extended_seen: u64,
    pub requests_sent: u64,
    pub acks_sent: u64,
}

/// A responder that keeps a link alive and nothing more.
#[derive(Clone)]
pub struct OnuResponder {
    pub mac: MacAddr,
    llid: Option<Llid>,
    discovery: oam::Discovery,
    next_keepalive: Option<WireInstant>,
    /// Discovery information from the last GATE, to echo back narrowed.
    discovery_information: u16,
    out: Vec<(WireInstant, Vec<u8>)>,
    pub counters: OnuCounters,
    /// Instant the LLID grant last landed, which is where the far end starts
    /// counting a registration's life from.
    pub registered_at: Option<WireInstant>,
    /// Instant the teardown landed.
    pub deregistered_at: Option<WireInstant>,
    /// How long each completed registration lasted, as measured here — grant
    /// landing to teardown landing. This is the figure to compare against the
    /// peer's own timer; the difference is what the exchange cost.
    pub registration_lives: Vec<WireDuration>,
}

impl OnuResponder {
    pub fn new(mac: MacAddr) -> Self {
        Self {
            mac,
            llid: None,
            discovery: oam::Discovery::default(),
            next_keepalive: None,
            discovery_information: 0,
            out: Vec::new(),
            counters: OnuCounters::default(),
            registered_at: None,
            deregistered_at: None,
            registration_lives: Vec::new(),
        }
    }

    pub fn llid(&self) -> Option<Llid> {
        self.llid
    }

    /// Frames the responder wants to send, with the instant it decided to.
    pub fn take_output(&mut self) -> Vec<(WireInstant, Vec<u8>)> {
        std::mem::take(&mut self.out)
    }

    /// Run the responder's own timers up to `now`.
    pub fn advance_to(&mut self, now: WireInstant) {
        while let Some(due) = self.next_keepalive.filter(|d| *d <= now) {
            let frame = self.build_info(due);
            self.out.push((due, frame));
            self.discovery.on_sent();
            self.next_keepalive = Some(due + KEEPALIVE_INTERVAL);
        }
    }

    /// Take a frame that reached the ONU end at `at`.
    pub fn deliver(&mut self, frame: &[u8], at: WireInstant) {
        match EtherType::of_frame(frame) {
            Some(EtherType::Mpcp) => self.on_mpcp(frame, at),
            Some(EtherType::SlowProtocol) => self.on_oam(frame, at),
            _ => {}
        }
    }

    fn on_mpcp(&mut self, frame: &[u8], at: WireInstant) {
        let Some(pdu) = mpcp::Pdu::parse(frame) else { return };
        match pdu.body {
            mpcp::Body::Gate { flags, .. } if !flags.discovery => {
                self.counters.gates_normal_seen += 1;
            }
            mpcp::Body::Gate { flags, .. } if flags.discovery => {
                self.counters.gates_seen += 1;
                if let Some(window) = discovery_window(frame) {
                    self.discovery_information = window;
                }
                if self.llid.is_some() {
                    return;
                }
                // Answer inside the window the GATE opened.
                let request = mpcp::register_req(
                    mpcp::Header {
                        dst: MacAddr::MPCP_MULTICAST,
                        src: self.mac,
                        opcode: mpcp::Opcode::RegisterReq,
                        timestamp: pdu.header.timestamp,
                    },
                    RegisterReqBody {
                        flag: RegisterReqFlag::Register,
                        pending_grants: 0,
                        // Which of the offered windows this end can use: what
                        // the GATE offered, narrowed to what it supports.
                        discovery_information: self.discovery_information & WINDOWS_SUPPORTED,
                        laser_on: LASER_TQ,
                        laser_off: LASER_TQ,
                    },
                );
                self.out.push((at + REQUEST_DELAY, request));
                self.counters.requests_sent += 1;
            }
            mpcp::Body::Register { llid, flag, sync_time } => {
                self.counters.registers_seen += 1;
                match RegisterFlag::from_u8(flag) {
                    Some(RegisterFlag::Ack) => {
                        self.llid = Some(llid);
                        self.registered_at = Some(at);
                        let ack = mpcp::register_ack(
                            mpcp::Header {
                                dst: MacAddr::MPCP_MULTICAST,
                                src: self.mac,
                                opcode: mpcp::Opcode::RegisterAck,
                                timestamp: pdu.header.timestamp,
                            },
                            RegisterAckFlag::Ack,
                            llid,
                            sync_time,
                        );
                        self.out.push((at, ack));
                        self.counters.acks_sent += 1;
                        if self.next_keepalive.is_none() {
                            self.next_keepalive = Some(at);
                        }
                    }
                    // 1 and 2 both tear the registration down.
                    Some(RegisterFlag::Reregister | RegisterFlag::Deregister) => {
                        self.counters.deregisters_seen += 1;
                        self.deregistered_at = Some(at);
                        if let Some(up) = self.registered_at {
                            self.registration_lives.push(at - up);
                        }
                        self.llid = None;
                        self.discovery.reset();
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    fn on_oam(&mut self, frame: &[u8], at: WireInstant) {
        if crate::extended::Pdu::parse(frame).is_some() {
            self.counters.extended_seen += 1;
            return;
        }
        let Some(pdu) = oam::Pdu::parse(frame) else { return };
        self.counters.oam_seen += 1;
        if pdu.code == oam::Code::Information {
            self.discovery.on_peer_information(pdu.flags);
        } else {
            self.discovery.on_peer_flags(pdu.flags);
        }
        if self.next_keepalive.is_none() {
            self.next_keepalive = Some(at);
        }
    }

    fn build_info(&self, at: WireInstant) -> Vec<u8> {
        let _ = at;
        oam::information(
            MacAddr::SLOW_PROTOCOL_MULTICAST,
            self.mac,
            self.discovery.flags(),
            oam::InfoTlv {
                is_local: true,
                oam_version: 0x01,
                revision: 0x0000,
                state: 0x00,
                // Passive: it answers, it does not start anything.
                configuration: 0x14,
                max_pdu_size: 0x0600,
                oui: oam::Oui::DPOE,
                vendor_specific: [0; 4],
            },
        )
    }
}

/// The discovery information a GATE offers, which sits after the grant.
fn discovery_window(frame: &[u8]) -> Option<u16> {
    const INFORMATION_OFFSET: usize = 29;
    let bytes = frame.get(INFORMATION_OFFSET..INFORMATION_OFFSET + 2)?;
    Some(u16::from_be_bytes([bytes[0], bytes[1]]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::link::Link;

    fn at(ms: u64) -> WireInstant {
        WireInstant::from_ps(WireDuration::from_ms(ms).as_ps())
    }

    #[test]
    fn the_responder_registers_against_the_peer() {
        let mut link = Link::default();
        let mut onu = OnuResponder::new(MacAddr::new([0x02, 0, 0, 1, 2, 3]));
        link.set_link(true, at(0));

        for ms in 0..10_000 {
            let now = at(ms);
            link.advance_to(now);
            while let Some(landed) = link.poll_downstream(now) {
                onu.deliver(&landed.frame, landed.arrives_at);
            }
            onu.advance_to(now);
            for (sent_at, frame) in onu.take_output() {
                link.send_upstream(frame, sent_at.max(now));
            }
            // The grant is not a registration until the acknowledgement has
            // travelled back, so keep going for a moment after it lands.
            if link.peer.counters.registrations > 0 {
                break;
            }
        }
        assert_eq!(onu.llid(), Some(link.peer.assigned_llid()));
        assert_eq!(link.peer.counters.registrations, 1);
        assert!(link.peer.counters.register_requests_accounted_for());
    }
}
