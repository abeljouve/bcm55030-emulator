//! Frame dissection: a protocol-neutral tree of named fields.
//!
//! One dissector serves every consumer — the MCP tools, the GUI packet view —
//! so a frame reads the same way everywhere. Each field carries its byte
//! range, which is what lets a view highlight the bytes behind a selection.

use crate::{extended, mpcp, oam, types};
use types::{EtherType, MacAddr};

/// One row of the dissection tree.
#[derive(Clone, Debug, serde::Serialize, schemars::JsonSchema)]
pub struct Field {
    /// Nesting level: 0 for a protocol layer, deeper for its contents.
    pub depth: u8,
    pub name: String,
    pub value: String,
    /// Byte range in the frame, when the field comes from the wire.
    pub offset: Option<usize>,
    pub len: usize,
}

impl Field {
    fn layer(name: impl Into<String>, value: impl Into<String>, offset: usize, len: usize) -> Self {
        Self { depth: 0, name: name.into(), value: value.into(), offset: Some(offset), len }
    }

    fn at(depth: u8, name: impl Into<String>, value: impl Into<String>, offset: usize, len: usize) -> Self {
        Self { depth, name: name.into(), value: value.into(), offset: Some(offset), len }
    }

    fn note(depth: u8, name: impl Into<String>, value: impl Into<String>) -> Self {
        Self { depth, name: name.into(), value: value.into(), offset: None, len: 0 }
    }
}

/// A dissected frame.
#[derive(Clone, Debug, serde::Serialize, schemars::JsonSchema)]
pub struct Dissection {
    pub dst: String,
    pub src: String,
    pub ethertype: String,
    /// Innermost protocol recognised: `MPCP`, `OAM`, `OAM/extended`, …
    pub protocol: String,
    /// One-line summary, in the spirit of a packet list.
    pub summary: String,
    pub fields: Vec<Field>,
}

/// Dissect an Ethernet frame as far as it is understood.
pub fn dissect(frame: &[u8]) -> Dissection {
    let dst = MacAddr::from_slice(frame).unwrap_or(MacAddr::ZERO);
    let src = MacAddr::from_slice(frame.get(6..).unwrap_or(&[])).unwrap_or(MacAddr::ZERO);
    let ethertype = EtherType::of_frame(frame);

    let mut fields = vec![
        Field::layer("Ethernet", format!("{} bytes", frame.len()), 0, frame.len().min(14)),
        Field::at(1, "Destination", dst.to_string(), 0, 6),
        Field::at(1, "Source", src.to_string(), 6, 6),
    ];
    if let Some(et) = ethertype {
        fields.push(Field::at(
            1,
            "EtherType",
            format!("{et} (0x{:04X})", et.as_u16()),
            types::ETHERTYPE_OFFSET,
            2,
        ));
    }

    let (protocol, summary) = match ethertype {
        Some(EtherType::Mpcp) => dissect_mpcp(frame, &mut fields),
        Some(EtherType::SlowProtocol) => dissect_oam(frame, &mut fields),
        Some(other) => (other.to_string(), format!("{other} frame")),
        None => ("(truncated)".into(), "frame shorter than an Ethernet header".into()),
    };

    Dissection {
        dst: dst.to_string(),
        src: src.to_string(),
        ethertype: ethertype.map(|e| e.to_string()).unwrap_or_else(|| "-".into()),
        protocol,
        summary,
        fields,
    }
}

fn dissect_mpcp(frame: &[u8], fields: &mut Vec<Field>) -> (String, String) {
    let Some(pdu) = mpcp::Pdu::parse(frame) else {
        return ("MPCP".into(), "malformed MPCPDU".into());
    };
    let opcode = pdu.header.opcode;
    fields.push(Field::layer("MPCP", opcode.to_string(), 14, frame.len().saturating_sub(14)));
    fields.push(Field::at(1, "Opcode", format!("{opcode} ({})", opcode.as_u16()), 14, 2));
    fields.push(Field::at(1, "Timestamp", format!("0x{:08X}", pdu.header.timestamp), 16, 4));

    let summary = match pdu.body {
        mpcp::Body::Gate { flags, .. } => {
            fields.push(Field::at(1, "Grants", flags.grant_count.to_string(), 20, 1));
            fields.push(Field::at(1, "Discovery", flags.discovery.to_string(), 20, 1));
            if frame.len() >= 27 {
                let start = u32::from_be_bytes([frame[21], frame[22], frame[23], frame[24]]);
                let length = u16::from_be_bytes([frame[25], frame[26]]);
                fields.push(Field::at(1, "Grant start", format!("0x{start:08X}"), 21, 4));
                fields.push(Field::at(1, "Grant length", format!("{length} TQ (0x{length:04X})"), 25, 2));
                if frame.len() >= 31 {
                    let sync = u16::from_be_bytes([frame[27], frame[28]]);
                    let info = u16::from_be_bytes([frame[29], frame[30]]);
                    fields.push(Field::at(1, "Sync time", format!("{sync} TQ"), 27, 2));
                    fields.push(Field::at(1, "Discovery information", format!("0x{info:04X}"), 29, 2));
                }
                format!(
                    "GATE, {} grant, window {length} TQ{}",
                    flags.grant_count,
                    if flags.discovery { ", discovery" } else { "" }
                )
            } else {
                "GATE".into()
            }
        }
        mpcp::Body::Register { llid, flag, sync_time } => {
            let name = mpcp::RegisterFlag::from_u8(flag)
                .map(|f| format!("{f:?}"))
                .unwrap_or_else(|| format!("0x{flag:02X}"));
            fields.push(Field::at(1, "Assigned LLID", llid.to_string(), 20, 2));
            fields.push(Field::at(1, "Flags", format!("{name} ({flag})"), 22, 1));
            fields.push(Field::at(1, "Sync time", format!("{sync_time} TQ"), 23, 2));
            if frame.len() >= 28 {
                fields.push(Field::at(1, "Echoed laser on", format!("{} TQ", frame[26]), 26, 1));
                fields.push(Field::at(1, "Echoed laser off", format!("{} TQ", frame[27]), 27, 1));
            }
            format!("REGISTER, LLID {llid}, {name}")
        }
        mpcp::Body::RegisterAck { flag, echoed_port, echoed_sync_time } => {
            fields.push(Field::at(1, "Flags", format!("{flag:?} ({})", flag.as_u8()), 20, 1));
            fields.push(Field::at(1, "Echoed port", echoed_port.to_string(), 21, 2));
            fields.push(Field::at(1, "Echoed sync time", format!("{echoed_sync_time} TQ"), 23, 2));
            format!("REGISTER_ACK, {flag:?}, port {echoed_port}")
        }
        mpcp::Body::RegisterReq(body) => {
            fields.push(Field::at(1, "Flags", format!("{:?} ({})", body.flag, body.flag.as_u8()), 20, 1));
            fields.push(Field::at(1, "Pending grants", body.pending_grants.to_string(), 21, 1));
            fields.push(Field::at(
                1,
                "Discovery information",
                format!("0x{:04X}", body.discovery_information),
                22,
                2,
            ));
            fields.push(Field::at(1, "Laser on", format!("{} TQ", body.laser_on), 24, 1));
            fields.push(Field::at(1, "Laser off", format!("{} TQ", body.laser_off), 25, 1));
            format!("REGISTER_REQ, {:?}, {} pending", body.flag, body.pending_grants)
        }
        mpcp::Body::Empty => opcode.to_string(),
    };
    ("MPCP".into(), summary)
}

fn dissect_oam(frame: &[u8], fields: &mut Vec<Field>) -> (String, String) {
    let Some(pdu) = oam::Pdu::parse(frame) else {
        return ("OAM".into(), "malformed OAMPDU".into());
    };
    let flags = pdu.flags;
    fields.push(Field::layer("OAM", pdu.code.to_string(), 14, frame.len().saturating_sub(14)));
    fields.push(Field::at(1, "Subtype", format!("0x{:02X}", pdu.subtype), 14, 1));
    fields.push(Field::at(1, "Flags", format!("0x{:04X}", flags.as_u16()), 15, 2));
    for (name, set) in [
        ("Link fault", flags.link_fault),
        ("Dying gasp", flags.dying_gasp),
        ("Critical event", flags.critical_event),
        ("Local evaluating", flags.local_evaluating),
        ("Local stable", flags.local_stable),
        ("Remote evaluating", flags.remote_evaluating),
        ("Remote stable", flags.remote_stable),
    ] {
        if set {
            fields.push(Field::at(2, name, "set", 15, 2));
        }
    }
    fields.push(Field::at(1, "Code", format!("{} (0x{:02X})", pdu.code, pdu.code.as_u8()), 17, 1));

    // An organization-specific PDU carries the extended channel.
    if let Some(ext) = extended::Pdu::parse(frame) {
        fields.push(Field::layer("OAM extended", ext.opcode.to_string(), 18, frame.len().saturating_sub(18)));
        fields.push(Field::at(1, "OUI", ext.oui.to_string(), 18, 3));
        fields.push(Field::at(1, "Opcode", format!("{} (0x{:02X})", ext.opcode, ext.opcode.as_u8()), 21, 1));
        for v in &ext.variables {
            fields.push(Field::note(1, "Variable", v.descriptor().to_string()));
            match v {
                extended::Variable::Address { parameters, .. } => {
                    fields.push(Field::note(2, "Asked for", "address only".to_string()));
                    if !parameters.is_empty() {
                        fields.push(Field::note(2, "Parameters", hex(parameters)));
                    }
                }
                extended::Variable::Container(c) => {
                    fields.push(Field::note(2, "Length", describe_length(c.length)));
                    if !c.value.is_empty() {
                        fields.push(Field::note(2, "Value", hex(&c.value)));
                    }
                }
            }
        }
        let summary = if ext.variables.is_empty() {
            format!("{} {}", ext.oui, ext.opcode)
        } else {
            format!(
                "{} {} — {}",
                ext.oui,
                ext.opcode,
                ext.variables
                    .iter()
                    .map(|v| v.descriptor().to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        };
        return ("OAM/extended".into(), summary);
    }

    // Otherwise walk the information TLVs.
    let mut tlvs = oam::Pdu::tlv_bytes(frame);
    let mut offset = 18usize;
    while tlvs.len() >= 2 && tlvs[0] != 0 {
        let kind = tlvs[0];
        let len = tlvs[1] as usize;
        let name = match kind {
            1 => "Local information",
            2 => "Remote information",
            0xFE => "Organization specific",
            _ => "TLV",
        };
        fields.push(Field::at(1, name, format!("{len} bytes"), offset, len.max(2)));
        if kind == 1 || kind == 2 {
            // Body layout after type and length: version(1), revision(2),
            // state(1), configuration(1), max PDU(2), OUI(3), vendor(4).
            if let Some(v) = tlvs.get(2..16) {
                let max_pdu = u16::from_be_bytes([v[5], v[6]]);
                fields.push(Field::at(2, "OAM version", v[0].to_string(), offset + 2, 1));
                fields.push(Field::at(2, "Revision", u16::from_be_bytes([v[1], v[2]]).to_string(), offset + 3, 2));
                fields.push(Field::at(2, "State", format!("0x{:02X}", v[3]), offset + 5, 1));
                fields.push(Field::at(2, "Configuration", format!("0x{:02X}", v[4]), offset + 6, 1));
                fields.push(Field::at(2, "Max OAMPDU size", format!("{max_pdu} (0x{max_pdu:04X})"), offset + 7, 2));
                fields.push(Field::at(2, "OUI", oam::Oui([v[7], v[8], v[9]]).to_string(), offset + 9, 3));
            }
        }
        let step = len.max(2);
        if step > tlvs.len() {
            break;
        }
        tlvs = &tlvs[step..];
        offset += step;
    }
    ("OAM".into(), format!("{} flags 0x{:04X}", pdu.code, flags.as_u16()))
}

fn describe_length(l: extended::Length) -> String {
    match l {
        extended::Length::Full => format!("full, {} bytes (encoded 0)", extended::Length::FULL_WIDTH),
        extended::Length::Bytes(n) => format!("{n} bytes"),
        extended::Length::Status(s) => format!("status 0x{s:02X}, no value"),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::oam::{Flags, InfoTlv, Oui};

    const ONU: MacAddr = MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
    const PEER: MacAddr = MacAddr::new([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);

    fn field<'a>(d: &'a Dissection, name: &str) -> Option<&'a Field> {
        d.fields.iter().find(|f| f.name == name)
    }

    #[test]
    fn a_gate_shows_its_window_and_discovery_bit() {
        let frame = mpcp::gate(
            mpcp::Header {
                dst: MacAddr::MPCP_MULTICAST,
                src: PEER,
                opcode: mpcp::Opcode::Gate,
                timestamp: 0x1234,
            },
            mpcp::GateFlags { grant_count: 1, discovery: true, force_report: 0 },
            Some(mpcp::Grant { start_time: 9, length: 8244 }),
            Some(mpcp::DiscoveryWindow { sync_time: 32, information: 0x0030 }),
        );
        let d = dissect(&frame);
        assert_eq!(d.protocol, "MPCP");
        assert!(d.summary.contains("8244 TQ"), "{}", d.summary);
        assert!(d.summary.contains("discovery"));
        assert_eq!(field(&d, "Grant length").unwrap().offset, Some(25));
        assert_eq!(field(&d, "Sync time").unwrap().value, "32 TQ");
        assert_eq!(field(&d, "Discovery information").unwrap().value, "0x0030");
    }

    #[test]
    fn a_register_names_its_flag_rather_than_printing_a_number() {
        let frame = mpcp::register(
            mpcp::Header { dst: ONU, src: PEER, opcode: mpcp::Opcode::Register, timestamp: 0 },
            types::Llid(0x3C67),
            mpcp::RegisterFlag::Ack,
            mpcp::RegisterBody {
                sync_time: 32,
                echoed_pending_grants: 0,
                echoed_laser_on: 32,
                echoed_laser_off: 32,
            },
        );
        let d = dissect(&frame);
        assert!(d.summary.contains("LLID 15463"), "{}", d.summary);
        assert!(field(&d, "Flags").unwrap().value.contains("Ack"));
        assert_eq!(field(&d, "Echoed laser on").unwrap().value, "32 TQ");
    }

    #[test]
    fn oam_flags_expand_into_the_bits_that_are_set() {
        let frame = oam::information(
            MacAddr::SLOW_PROTOCOL_MULTICAST,
            PEER,
            Flags::converged(),
            InfoTlv {
                is_local: true,
                oam_version: 1,
                revision: 1,
                state: 0,
                configuration: 5,
                max_pdu_size: 0x0600,
                oui: Oui::DPOE,
                vendor_specific: [0; 4],
            },
        );
        let d = dissect(&frame);
        assert_eq!(d.protocol, "OAM");
        assert!(field(&d, "Local stable").is_some());
        assert!(field(&d, "Remote stable").is_some());
        assert!(field(&d, "Link fault").is_none(), "unset bits stay out");
        assert_eq!(field(&d, "Max OAMPDU size").unwrap().value, "1536 (0x0600)");
        assert_eq!(field(&d, "OUI").unwrap().value, "00-10-00");
    }

    #[test]
    fn an_extended_pdu_lists_its_variables() {
        let frame = extended::get_response(
            extended::Header {
                dst: ONU,
                src: PEER,
                flags: Flags::converged(),
                oui: Oui::DPOE,
                opcode: extended::Opcode::GetResponse,
            },
            &[extended::Container {
                descriptor: extended::Descriptor::attribute(extended::leaf::FIRMWARE_INFO),
                length: extended::Length::Bytes(12),
                value: vec![0xAB; 12],
            }],
        );
        let d = dissect(&frame);
        assert_eq!(d.protocol, "OAM/extended");
        assert!(d.summary.contains("get-response"), "{}", d.summary);
        assert!(d.summary.contains("attribute/0x0003"), "{}", d.summary);
        assert_eq!(field(&d, "Length").unwrap().value, "12 bytes");
    }

    #[test]
    fn a_truncated_frame_dissects_without_panicking() {
        for n in 0..14 {
            let d = dissect(&vec![0u8; n]);
            assert_eq!(d.protocol, "(truncated)");
        }
    }

    #[test]
    fn every_field_offset_stays_inside_the_frame() {
        let frame = mpcp::register_req(
            mpcp::Header {
                dst: MacAddr::MPCP_MULTICAST,
                src: ONU,
                opcode: mpcp::Opcode::RegisterReq,
                timestamp: 0,
            },
            mpcp::RegisterReqBody {
                    flag: mpcp::RegisterReqFlag::Register,
                    pending_grants: 16,
                    discovery_information: 0x0011,
                    laser_on: 32,
                    laser_off: 32,
                },
        );
        for f in dissect(&frame).fields {
            if let Some(off) = f.offset {
                assert!(off + f.len <= frame.len(), "{} runs past the frame", f.name);
            }
        }
    }
}

#[cfg(test)]
mod panel_shape {
    use super::*;

    /// What a packet list renders: every frame must yield a protocol name
    /// and a summary, and every dissection must have at least the Ethernet
    /// layer, or a row would come out blank.
    #[test]
    fn every_frame_the_model_builds_renders_a_row() {
        let peer = MacAddr::new([0x02, 0, 0, 0, 0, 1]);
        let onu = MacAddr::new([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        let frames: Vec<Vec<u8>> = vec![
            mpcp::gate(
                mpcp::Header { dst: MacAddr::MPCP_MULTICAST, src: peer, opcode: mpcp::Opcode::Gate, timestamp: 1 },
                mpcp::GateFlags { grant_count: 1, discovery: true, force_report: 0 },
                Some(mpcp::Grant { start_time: 1, length: 8244 }),
                Some(mpcp::DiscoveryWindow { sync_time: 32, information: 0x30 }),
            ),
            mpcp::register(
                mpcp::Header { dst: onu, src: peer, opcode: mpcp::Opcode::Register, timestamp: 2 },
                types::Llid(1),
                mpcp::RegisterFlag::Ack,
                mpcp::RegisterBody { sync_time: 32, echoed_pending_grants: 0, echoed_laser_on: 32, echoed_laser_off: 32 },
            ),
            mpcp::register_req(
                mpcp::Header { dst: MacAddr::MPCP_MULTICAST, src: onu, opcode: mpcp::Opcode::RegisterReq, timestamp: 3 },
                mpcp::RegisterReqBody {
                    flag: mpcp::RegisterReqFlag::Register,
                    pending_grants: 16,
                    discovery_information: 0x0011,
                    laser_on: 32,
                    laser_off: 32,
                },
            ),
            mpcp::register_ack(
                mpcp::Header { dst: MacAddr::MPCP_MULTICAST, src: onu, opcode: mpcp::Opcode::RegisterAck, timestamp: 4 },
                mpcp::RegisterAckFlag::Ack,
                types::Llid(1),
                32,
            ),
            oam::information(
                MacAddr::SLOW_PROTOCOL_MULTICAST,
                peer,
                oam::Flags::converged(),
                oam::InfoTlv {
                    is_local: true, oam_version: 1, revision: 1, state: 0,
                    configuration: 5, max_pdu_size: 0x0600,
                    oui: oam::Oui::DPOE, vendor_specific: [0; 4],
                },
            ),
            extended::get_request(
                extended::Header {
                    dst: onu, src: peer, flags: oam::Flags::converged(),
                    oui: oam::Oui::DPOE, opcode: extended::Opcode::GetRequest,
                },
                &[extended::Descriptor::attribute(extended::leaf::FIRMWARE_INFO)],
            ),
        ];
        for f in frames {
            let d = dissect(&f);
            assert!(!d.protocol.is_empty());
            assert!(!d.summary.is_empty(), "{:?}", d.protocol);
            assert!(!d.dst.is_empty() && !d.src.is_empty());
            assert!(d.fields.iter().any(|x| x.name == "Ethernet"));
            for x in &d.fields {
                if let Some(o) = x.offset {
                    assert!(o + x.len <= f.len(), "{} runs past {}", x.name, f.len());
                }
            }
        }
    }
}
