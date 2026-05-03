use crate::soc::bank::MmioHistoryEntry;

#[derive(Clone, Debug, serde::Serialize)]
pub struct DetectedPattern {
    pub pattern_type: String,
    pub address: u32,
    pub secondary_address: Option<u32>,
    pub count: usize,
    pub value: Option<u32>,
    pub first_pc: u32,
    pub last_pc: u32,
    pub first_insn: u64,
    pub last_insn: u64,
}

pub fn detect_patterns(history: &[MmioHistoryEntry]) -> Vec<DetectedPattern> {
    let mut patterns = Vec::new();
    if history.len() < 3 {
        return patterns;
    }

    detect_busy_wait(history, &mut patterns);
    detect_write_then_poll(history, &mut patterns);
    detect_command_bit(history, &mut patterns);

    patterns.sort_by_key(|p| p.first_insn);
    patterns
}

fn detect_busy_wait(history: &[MmioHistoryEntry], out: &mut Vec<DetectedPattern>) {
    let mut i = 0;
    while i < history.len() {
        if history[i].direction != "read" {
            i += 1;
            continue;
        }
        let addr = history[i].address;
        let val = history[i].value;
        let start = i;
        let mut j = i + 1;
        while j < history.len()
            && history[j].direction == "read"
            && history[j].address == addr
            && history[j].value == val
        {
            j += 1;
        }
        let run = j - start;
        if run >= 5 {
            out.push(DetectedPattern {
                pattern_type: "busy_wait".into(),
                address: addr,
                secondary_address: None,
                count: run,
                value: Some(val),
                first_pc: history[start].pc,
                last_pc: history[j - 1].pc,
                first_insn: history[start].insn,
                last_insn: history[j - 1].insn,
            });
        }
        i = j;
    }
}

fn detect_write_then_poll(history: &[MmioHistoryEntry], out: &mut Vec<DetectedPattern>) {
    let mut i = 0;
    while i + 1 < history.len() {
        if history[i].direction != "write" {
            i += 1;
            continue;
        }
        let addr = history[i].address;
        let write_insn = history[i].insn;
        let write_pc = history[i].pc;
        let mut j = i + 1;
        let mut read_count = 0;
        while j < history.len()
            && history[j].direction == "read"
            && history[j].address == addr
        {
            read_count += 1;
            j += 1;
        }
        if read_count >= 3 {
            out.push(DetectedPattern {
                pattern_type: "write_then_poll".into(),
                address: addr,
                secondary_address: None,
                count: read_count,
                value: Some(history[i].value),
                first_pc: write_pc,
                last_pc: history[j - 1].pc,
                first_insn: write_insn,
                last_insn: history[j - 1].insn,
            });
        }
        i = j;
    }
}

fn detect_command_bit(history: &[MmioHistoryEntry], out: &mut Vec<DetectedPattern>) {
    let mut i = 0;
    while i + 1 < history.len() {
        if history[i].direction != "write" || history[i].value & 0x8000_0000 == 0 {
            i += 1;
            continue;
        }
        let addr = history[i].address;
        let write_insn = history[i].insn;
        let write_pc = history[i].pc;
        let mut j = i + 1;
        let mut poll_count = 0;
        while j < history.len()
            && history[j].direction == "read"
            && history[j].address == addr
        {
            poll_count += 1;
            if history[j].value & 0x8000_0000 == 0 {
                j += 1;
                break;
            }
            j += 1;
        }
        if poll_count >= 2 {
            out.push(DetectedPattern {
                pattern_type: "command_bit".into(),
                address: addr,
                secondary_address: None,
                count: poll_count,
                value: Some(history[i].value),
                first_pc: write_pc,
                last_pc: history[j - 1].pc,
                first_insn: write_insn,
                last_insn: history[j - 1].insn,
            });
        }
        i = j;
    }
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct TimelineEvent {
    pub event_type: String,
    pub block: Option<String>,
    pub address: u32,
    pub from_insn: u64,
    pub to_insn: u64,
    pub access_count: usize,
    pub summary: String,
}

pub fn build_timeline(
    history: &[MmioHistoryEntry],
    from_insn: Option<u64>,
    to_insn: Option<u64>,
) -> Vec<TimelineEvent> {
    let filtered: Vec<&MmioHistoryEntry> = history
        .iter()
        .filter(|e| {
            from_insn.map_or(true, |f| e.insn >= f) && to_insn.map_or(true, |t| e.insn <= t)
        })
        .collect();

    if filtered.is_empty() {
        return Vec::new();
    }

    let mut events = Vec::new();
    let mut i = 0;

    while i < filtered.len() {
        let periph = filtered[i].peripheral;
        let start_insn = filtered[i].insn;
        let mut j = i + 1;
        while j < filtered.len()
            && filtered[j].peripheral == periph
            && filtered[j].insn.saturating_sub(filtered[j - 1].insn) < 50
        {
            j += 1;
        }
        let burst_len = j - i;
        if burst_len >= 3 {
            let addr = filtered[i].address;
            let same_addr_reads = filtered[i..j]
                .iter()
                .filter(|e| e.address == addr && e.direction == "read")
                .count();
            let same_val_reads = filtered[i..j]
                .iter()
                .filter(|e| e.address == addr && e.direction == "read" && e.value == filtered[i].value)
                .count();

            let event_type = if same_val_reads >= 3 {
                "busy_wait"
            } else if same_addr_reads >= 3 {
                "polling"
            } else {
                "burst"
            };

            let summary = format!(
                "{} × {} access to {} ({})",
                burst_len, periph,
                if burst_len > 1 && filtered[i].address != filtered[j - 1].address {
                    format!("0x{:08X}..0x{:08X}", filtered[i].address, filtered[j - 1].address)
                } else {
                    format!("0x{:08X}", filtered[i].address)
                },
                event_type,
            );

            events.push(TimelineEvent {
                event_type: event_type.into(),
                block: Some(periph.to_string()),
                address: filtered[i].address,
                from_insn: start_insn,
                to_insn: filtered[j - 1].insn,
                access_count: burst_len,
                summary,
            });
        }
        i = j.max(i + 1);
    }

    events
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(insn: u64, addr: u32, val: u32, dir: &'static str) -> MmioHistoryEntry {
        MmioHistoryEntry {
            insn,
            pc: 0x100,
            blink: 0,
            address: addr,
            value: val,
            direction: dir,
            width: "word",
            peripheral: "test",
            di: false,
        }
    }

    #[test]
    fn busy_wait_detection() {
        let h: Vec<MmioHistoryEntry> = (0..8).map(|i| entry(i, 0x1000, 0x42, "read")).collect();
        let patterns = detect_patterns(&h);
        assert_eq!(patterns.len(), 1);
        assert_eq!(patterns[0].pattern_type, "busy_wait");
        assert_eq!(patterns[0].count, 8);
    }

    #[test]
    fn write_then_poll_detection() {
        let mut h = vec![entry(0, 0x2000, 0xFF, "write")];
        for i in 1..=5 {
            h.push(entry(i, 0x2000, 0x01, "read"));
        }
        let patterns = detect_patterns(&h);
        assert!(patterns.iter().any(|p| p.pattern_type == "write_then_poll"));
    }

    #[test]
    fn command_bit_detection() {
        let mut h = vec![entry(0, 0x3000, 0x8000_0001, "write")];
        h.push(entry(1, 0x3000, 0x8000_0001, "read"));
        h.push(entry(2, 0x3000, 0x8000_0001, "read"));
        h.push(entry(3, 0x3000, 0x0000_0001, "read"));
        let patterns = detect_patterns(&h);
        assert!(patterns.iter().any(|p| p.pattern_type == "command_bit"));
    }

    #[test]
    fn timeline_burst() {
        let h: Vec<MmioHistoryEntry> = (0..6).map(|i| entry(i * 2, 0x1000 + (i as u32 % 3) * 4, 0, "read")).collect();
        let tl = build_timeline(&h, None, None);
        assert!(!tl.is_empty());
        assert_eq!(tl[0].access_count, 6);
    }
}
