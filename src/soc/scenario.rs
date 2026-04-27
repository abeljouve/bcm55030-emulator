//! MMIO scenario engine — override registry, scheduled events, and
//! MMIO watchpoints for HW scenario programming.
//!
//! Everything here models **hardware stimulus**, not firmware patching.
//! `ScenarioEffect` has no variant that touches CPU state (registers,
//! PC, SRAM).  Rule 1 (no firmware hooks) is preserved.
//!
//! `peek_word` (GUI 60 Hz polling, MCP `peek_mmio`) does NOT consume
//! one-shot overrides or fire triggers.  Only CPU-driven accesses do.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

/// What an override does when the address is hit.
#[derive(Clone, Debug)]
pub enum OverrideSpec {
    /// Return `value` on every read, forever.
    StaticRead { value: u32 },
    /// Return `value` for the next `remaining` reads, then expire.
    OneShotRead { value: u32, remaining: u32 },
    /// On write, zero out the bits set in `mask` before forwarding to
    /// the peripheral.  Useful for keeping command bits clear.
    MaskedWriteIgnore { mask: u32 },
}

/// Per-address override entry with an optional human-readable label.
#[derive(Clone, Debug)]
pub struct MmioOverride {
    pub spec: OverrideSpec,
    pub label: Option<String>,
}

/// Registry of active MMIO overrides, keyed by word-aligned address.
#[derive(Clone, Debug, Default)]
pub struct MmioOverrideRegistry {
    overrides: HashMap<u32, MmioOverride>,
}

impl MmioOverrideRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.overrides.is_empty()
    }

    pub fn set(&mut self, addr: u32, spec: OverrideSpec, label: Option<String>) {
        let aligned = addr & !3;
        self.overrides.insert(aligned, MmioOverride { spec, label });
    }

    pub fn remove(&mut self, addr: u32) -> bool {
        self.overrides.remove(&(addr & !3)).is_some()
    }

    pub fn clear(&mut self) {
        self.overrides.clear();
    }

    /// Side-effect-free probe — returns the override value (if any)
    /// **without** decrementing one-shot counters.  Used by `peek_word`.
    pub fn peek_read(&self, addr: u32) -> Option<u32> {
        let aligned = addr & !3;
        match &self.overrides.get(&aligned)?.spec {
            OverrideSpec::StaticRead { value } => Some(*value),
            OverrideSpec::OneShotRead { value, remaining } if *remaining > 0 => Some(*value),
            _ => None,
        }
    }

    /// Consuming read — returns the override value (if any) and
    /// decrements one-shot counters.  Expired entries are removed.
    pub fn try_read(&mut self, addr: u32) -> Option<u32> {
        let aligned = addr & !3;
        let entry = self.overrides.get_mut(&aligned)?;
        match &mut entry.spec {
            OverrideSpec::StaticRead { value } => Some(*value),
            OverrideSpec::OneShotRead { value, remaining } => {
                if *remaining == 0 {
                    return None;
                }
                let v = *value;
                *remaining -= 1;
                if *remaining == 0 {
                    self.overrides.remove(&aligned);
                }
                Some(v)
            }
            OverrideSpec::MaskedWriteIgnore { .. } => None,
        }
    }

    /// Check write overrides — returns `Some(masked_val)` if the write
    /// should be modified, `None` if the write should proceed normally.
    pub fn filter_write(&self, addr: u32, val: u32) -> Option<u32> {
        let aligned = addr & !3;
        let entry = self.overrides.get(&aligned)?;
        match &entry.spec {
            OverrideSpec::MaskedWriteIgnore { mask } => Some(val & !mask),
            _ => None,
        }
    }

    /// Iterate over all active overrides (for MCP `list_mmio_overrides`).
    pub fn iter(&self) -> impl Iterator<Item = (u32, &MmioOverride)> {
        self.overrides.iter().map(|(&addr, ov)| (addr, ov))
    }

    pub fn len(&self) -> usize {
        self.overrides.len()
    }
}

// ── JSON parsing helpers ─────────────────────────────────────────────

pub fn parse_hex_or_dec(v: &serde_json::Value) -> Option<u32> {
    if let Some(n) = v.as_u64() {
        return Some(n as u32);
    }
    if let Some(s) = v.as_str() {
        let s = s.trim();
        if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
            return u32::from_str_radix(hex, 16).ok();
        }
        return s.parse::<u32>().ok();
    }
    None
}

pub fn parse_trigger(v: &serde_json::Value) -> Option<ScenarioTrigger> {
    let ty = v.get("type")?.as_str()?;
    match ty {
        "at_instruction" => {
            let n = v.get("n")?.as_u64()?;
            Some(ScenarioTrigger::AtInstruction(n))
        }
        "on_mmio_read" => {
            let addr = parse_hex_or_dec(v.get("address")?)?;
            let occ = v.get("occurrence")?.as_u64()? as u32;
            Some(ScenarioTrigger::OnMmioRead { address: addr, occurrence: occ })
        }
        "on_mmio_write" => {
            let addr = parse_hex_or_dec(v.get("address")?)?;
            let occ = v.get("occurrence")?.as_u64()? as u32;
            Some(ScenarioTrigger::OnMmioWrite { address: addr, occurrence: occ })
        }
        _ => None,
    }
}

pub fn parse_effect(v: &serde_json::Value) -> Option<ScenarioEffect> {
    let ty = v.get("type")?.as_str()?;
    match ty {
        "set_override" => {
            let addr = parse_hex_or_dec(v.get("address")?)?;
            let value = parse_hex_or_dec(v.get("value")?)?;
            let mode = v.get("mode").and_then(|m| m.as_str()).unwrap_or("static");
            let spec = match mode {
                "oneshot" => {
                    let count = v.get("count").and_then(|c| c.as_u64()).unwrap_or(1) as u32;
                    OverrideSpec::OneShotRead { value, remaining: count }
                }
                "mask" => OverrideSpec::MaskedWriteIgnore { mask: value },
                _ => OverrideSpec::StaticRead { value },
            };
            let label = v.get("label").and_then(|l| l.as_str()).map(String::from);
            Some(ScenarioEffect::SetOverride { address: addr, spec, label })
        }
        "remove_override" => {
            let addr = parse_hex_or_dec(v.get("address")?)?;
            Some(ScenarioEffect::RemoveOverride { address: addr })
        }
        "write_mmio" => {
            let addr = parse_hex_or_dec(v.get("address")?)?;
            let value = parse_hex_or_dec(v.get("value")?)?;
            Some(ScenarioEffect::WriteMmio { address: addr, value })
        }
        "pause" => Some(ScenarioEffect::Pause),
        _ => None,
    }
}

// ── Scenario engine (Phase 2) ────────────────────────────────────────

/// When an event fires.
#[derive(Clone, Debug)]
pub enum ScenarioTrigger {
    /// Fire at CPU instruction count N.
    AtInstruction(u64),
    /// Fire on the N-th read of `address`.
    OnMmioRead { address: u32, occurrence: u32 },
    /// Fire on the N-th write to `address`.
    OnMmioWrite { address: u32, occurrence: u32 },
}

/// What happens when an event fires.  No variant touches CPU state.
#[derive(Clone, Debug)]
pub enum ScenarioEffect {
    SetOverride { address: u32, spec: OverrideSpec, label: Option<String> },
    RemoveOverride { address: u32 },
    WriteMmio { address: u32, value: u32 },
    Pause,
}

/// A scheduled event with a unique ID.
#[derive(Clone, Debug)]
pub struct ScheduledEvent {
    pub id: u32,
    pub trigger: ScenarioTrigger,
    pub effect: ScenarioEffect,
    pub label: Option<String>,
    pub fired: bool,
}

/// MMIO watchpoint with an associated action.
#[derive(Clone, Debug)]
pub struct MmioWatchpoint {
    pub id: u32,
    pub address: u32,
    pub size: u32,
    pub mode: MmioWatchMode,
    pub action: MmioWatchAction,
    pub label: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MmioWatchMode {
    Read,
    Write,
    ReadWrite,
}

#[derive(Clone, Debug)]
pub enum MmioWatchAction {
    Pause,
    Fire(ScenarioEffect),
}

/// Deferred writes from scenario effects.  The caller must drain
/// these after the scenario engine processes an access.
#[derive(Clone, Debug)]
pub struct DeferredWrite {
    pub address: u32,
    pub value: u32,
}

/// Central scenario engine — owns overrides, scheduled events, and
/// MMIO watchpoints.
#[derive(Clone, Debug, Default)]
pub struct ScenarioEngine {
    pub overrides: MmioOverrideRegistry,

    events: Vec<ScheduledEvent>,
    next_event_id: u32,

    /// Instruction-triggered events, sorted by instruction count.
    insn_queue: BTreeMap<u64, Vec<usize>>,
    /// Read-triggered events, keyed by address.
    read_triggers: HashMap<u32, Vec<usize>>,
    /// Write-triggered events, keyed by address.
    write_triggers: HashMap<u32, Vec<usize>>,
    /// Per-address read/write counters for occurrence tracking.
    read_counts: HashMap<u32, u32>,
    write_counts: HashMap<u32, u32>,

    watchpoints: Vec<MmioWatchpoint>,
    next_wp_id: u32,

    /// Pause requested by a scenario effect.
    pub pause_requested: bool,

    /// Deferred writes from effects (processed by caller).
    pub deferred_writes: Vec<DeferredWrite>,
}

impl ScenarioEngine {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.overrides.is_empty()
            && self.events.is_empty()
            && self.watchpoints.is_empty()
    }

    // ── Events ──────────────────────────────────────────────────────

    pub fn schedule(&mut self, trigger: ScenarioTrigger, effect: ScenarioEffect, label: Option<String>) -> u32 {
        let id = self.next_event_id;
        self.next_event_id += 1;
        let idx = self.events.len();
        self.events.push(ScheduledEvent {
            id,
            trigger: trigger.clone(),
            effect,
            label,
            fired: false,
        });
        match &trigger {
            ScenarioTrigger::AtInstruction(n) => {
                self.insn_queue.entry(*n).or_default().push(idx);
            }
            ScenarioTrigger::OnMmioRead { address, .. } => {
                self.read_triggers.entry(*address & !3).or_default().push(idx);
            }
            ScenarioTrigger::OnMmioWrite { address, .. } => {
                self.write_triggers.entry(*address & !3).or_default().push(idx);
            }
        }
        id
    }

    pub fn cancel(&mut self, id: u32) -> bool {
        if let Some(ev) = self.events.iter_mut().find(|e| e.id == id && !e.fired) {
            ev.fired = true;
            true
        } else {
            false
        }
    }

    pub fn pending_events(&self) -> impl Iterator<Item = &ScheduledEvent> {
        self.events.iter().filter(|e| !e.fired)
    }

    /// Process instruction-triggered events.  Call from `bank.tick()`.
    pub fn tick(&mut self, cpu_instructions: u64) {
        let to_fire: Vec<usize> = self.insn_queue
            .range(..=cpu_instructions)
            .flat_map(|(_, idxs)| idxs.iter().copied())
            .filter(|&idx| !self.events[idx].fired)
            .collect();

        for idx in to_fire {
            self.fire_event(idx);
        }

        // Remove processed instruction triggers.
        let keys: Vec<u64> = self.insn_queue
            .range(..=cpu_instructions)
            .map(|(&k, _)| k)
            .collect();
        for k in keys {
            self.insn_queue.remove(&k);
        }
    }

    /// Process a MMIO read access.
    pub fn on_mmio_read(&mut self, addr: u32) {
        let aligned = addr & !3;
        let count = self.read_counts.entry(aligned).or_insert(0);
        *count += 1;
        let current = *count;

        if let Some(idxs) = self.read_triggers.get(&aligned).cloned() {
            for idx in idxs {
                if self.events[idx].fired {
                    continue;
                }
                if let ScenarioTrigger::OnMmioRead { occurrence, .. } = self.events[idx].trigger {
                    if current == occurrence {
                        self.fire_event(idx);
                    }
                }
            }
        }

        self.check_watchpoints(aligned, MmioWatchMode::Read);
    }

    /// Process a MMIO write access.
    pub fn on_mmio_write(&mut self, addr: u32) {
        let aligned = addr & !3;
        let count = self.write_counts.entry(aligned).or_insert(0);
        *count += 1;
        let current = *count;

        if let Some(idxs) = self.write_triggers.get(&aligned).cloned() {
            for idx in idxs {
                if self.events[idx].fired {
                    continue;
                }
                if let ScenarioTrigger::OnMmioWrite { occurrence, .. } = self.events[idx].trigger {
                    if current == occurrence {
                        self.fire_event(idx);
                    }
                }
            }
        }

        self.check_watchpoints(aligned, MmioWatchMode::Write);
    }

    fn fire_event(&mut self, idx: usize) {
        self.events[idx].fired = true;
        let effect = self.events[idx].effect.clone();
        self.apply_effect(&effect);
    }

    fn apply_effect(&mut self, effect: &ScenarioEffect) {
        match effect {
            ScenarioEffect::SetOverride { address, spec, label } => {
                self.overrides.set(*address, spec.clone(), label.clone());
            }
            ScenarioEffect::RemoveOverride { address } => {
                self.overrides.remove(*address);
            }
            ScenarioEffect::WriteMmio { address, value } => {
                self.deferred_writes.push(DeferredWrite {
                    address: *address,
                    value: *value,
                });
            }
            ScenarioEffect::Pause => {
                self.pause_requested = true;
            }
        }
    }

    // ── MMIO Watchpoints ────────────────────────────────────────────

    pub fn add_watchpoint(
        &mut self,
        address: u32,
        size: u32,
        mode: MmioWatchMode,
        action: MmioWatchAction,
        label: Option<String>,
    ) -> u32 {
        let id = self.next_wp_id;
        self.next_wp_id += 1;
        self.watchpoints.push(MmioWatchpoint {
            id,
            address: address & !3,
            size: size.max(4),
            mode,
            action,
            label,
        });
        id
    }

    pub fn remove_watchpoint(&mut self, id: u32) -> bool {
        let len_before = self.watchpoints.len();
        self.watchpoints.retain(|wp| wp.id != id);
        self.watchpoints.len() < len_before
    }

    pub fn watchpoints(&self) -> &[MmioWatchpoint] {
        &self.watchpoints
    }

    fn check_watchpoints(&mut self, addr: u32, access: MmioWatchMode) {
        let actions: Vec<MmioWatchAction> = self.watchpoints
            .iter()
            .filter(|wp| {
                let matches_mode = wp.mode == MmioWatchMode::ReadWrite || wp.mode == access;
                let matches_addr = addr >= wp.address && addr < wp.address + wp.size;
                matches_mode && matches_addr
            })
            .map(|wp| wp.action.clone())
            .collect();

        for action in actions {
            match action {
                MmioWatchAction::Pause => {
                    self.pause_requested = true;
                }
                MmioWatchAction::Fire(effect) => {
                    self.apply_effect(&effect);
                }
            }
        }
    }

    /// Take and clear the pause flag.
    pub fn take_pause(&mut self) -> bool {
        let p = self.pause_requested;
        self.pause_requested = false;
        p
    }

    /// Take and clear deferred writes.
    pub fn take_deferred_writes(&mut self) -> Vec<DeferredWrite> {
        std::mem::take(&mut self.deferred_writes)
    }

    /// Load a JSON scenario file.  Returns the number of entries loaded.
    pub fn load_file(&mut self, path: &Path) -> Result<usize, String> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| format!("read {}: {}", path.display(), e))?;
        self.load_json(&content)
    }

    /// Load scenario from a JSON string.
    pub fn load_json(&mut self, json: &str) -> Result<usize, String> {
        let doc: serde_json::Value = serde_json::from_str(json)
            .map_err(|e| format!("parse: {e}"))?;
        let events = doc.get("events")
            .and_then(|v| v.as_array())
            .ok_or_else(|| "missing 'events' array".to_string())?;
        let mut loaded = 0;
        for (i, entry) in events.iter().enumerate() {
            let tool = entry.get("tool").and_then(|v| v.as_str())
                .ok_or_else(|| format!("event[{i}]: missing 'tool'"))?;
            let params = entry.get("params")
                .ok_or_else(|| format!("event[{i}]: missing 'params'"))?;
            match tool {
                "set_mmio_override" => {
                    let addr = parse_hex_or_dec(params.get("address")
                        .ok_or_else(|| format!("event[{i}]: missing address"))?)
                        .ok_or_else(|| format!("event[{i}]: bad address"))?;
                    let value = parse_hex_or_dec(params.get("value")
                        .ok_or_else(|| format!("event[{i}]: missing value"))?)
                        .ok_or_else(|| format!("event[{i}]: bad value"))?;
                    let mode = params.get("mode").and_then(|m| m.as_str()).unwrap_or("static");
                    let spec = match mode {
                        "oneshot" => {
                            let count = params.get("count").and_then(|c| c.as_u64()).unwrap_or(1) as u32;
                            OverrideSpec::OneShotRead { value, remaining: count }
                        }
                        "mask" => OverrideSpec::MaskedWriteIgnore { mask: value },
                        _ => OverrideSpec::StaticRead { value },
                    };
                    let label = params.get("label").and_then(|l| l.as_str()).map(String::from);
                    self.overrides.set(addr, spec, label);
                }
                "schedule_event" => {
                    let trigger = parse_trigger(params.get("trigger")
                        .ok_or_else(|| format!("event[{i}]: missing trigger"))?)
                        .ok_or_else(|| format!("event[{i}]: bad trigger"))?;
                    let effect = parse_effect(params.get("effect")
                        .ok_or_else(|| format!("event[{i}]: missing effect"))?)
                        .ok_or_else(|| format!("event[{i}]: bad effect"))?;
                    let label = params.get("label").and_then(|l| l.as_str()).map(String::from);
                    self.schedule(trigger, effect, label);
                }
                other => {
                    return Err(format!("event[{i}]: unsupported tool '{other}'"));
                }
            }
            loaded += 1;
        }
        Ok(loaded)
    }

    pub fn clear_all(&mut self) {
        self.overrides.clear();
        self.events.clear();
        self.insn_queue.clear();
        self.read_triggers.clear();
        self.write_triggers.clear();
        self.read_counts.clear();
        self.write_counts.clear();
        self.watchpoints.clear();
        self.pause_requested = false;
        self.deferred_writes.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn static_override_persists() {
        let mut reg = MmioOverrideRegistry::new();
        reg.set(0x0100_240C, OverrideSpec::StaticRead { value: 0x5000 }, None);
        assert_eq!(reg.try_read(0x0100_240C), Some(0x5000));
        assert_eq!(reg.try_read(0x0100_240C), Some(0x5000));
        assert_eq!(reg.try_read(0x0100_240C), Some(0x5000));
        assert_eq!(reg.len(), 1);
    }

    #[test]
    fn oneshot_expires() {
        let mut reg = MmioOverrideRegistry::new();
        reg.set(
            0x0100_240C,
            OverrideSpec::OneShotRead { value: 0x5000, remaining: 2 },
            None,
        );
        assert_eq!(reg.try_read(0x0100_240C), Some(0x5000));
        assert_eq!(reg.try_read(0x0100_240C), Some(0x5000));
        assert_eq!(reg.try_read(0x0100_240C), None);
        assert!(reg.is_empty());
    }

    #[test]
    fn peek_does_not_consume() {
        let mut reg = MmioOverrideRegistry::new();
        reg.set(
            0x0100_240C,
            OverrideSpec::OneShotRead { value: 0xAA, remaining: 1 },
            None,
        );
        assert_eq!(reg.peek_read(0x0100_240C), Some(0xAA));
        assert_eq!(reg.peek_read(0x0100_240C), Some(0xAA));
        assert_eq!(reg.try_read(0x0100_240C), Some(0xAA));
        assert_eq!(reg.peek_read(0x0100_240C), None);
    }

    #[test]
    fn masked_write() {
        let reg = MmioOverrideRegistry::new();
        assert_eq!(reg.filter_write(0x0100_240C, 0x8000_5000), None);

        let mut reg = MmioOverrideRegistry::new();
        reg.set(
            0x0100_240C,
            OverrideSpec::MaskedWriteIgnore { mask: 0x8000_0000 },
            None,
        );
        assert_eq!(reg.filter_write(0x0100_240C, 0x8000_5000), Some(0x0000_5000));
    }

    #[test]
    fn address_alignment() {
        let mut reg = MmioOverrideRegistry::new();
        reg.set(0x0100_240F, OverrideSpec::StaticRead { value: 42 }, None);
        assert_eq!(reg.try_read(0x0100_240C), Some(42));
    }

    #[test]
    fn remove_works() {
        let mut reg = MmioOverrideRegistry::new();
        reg.set(0x0100_0000, OverrideSpec::StaticRead { value: 1 }, None);
        assert!(reg.remove(0x0100_0000));
        assert!(!reg.remove(0x0100_0000));
        assert!(reg.is_empty());
    }

    // ── ScenarioEngine tests ────────────────────────────────────────

    #[test]
    fn engine_instruction_trigger() {
        let mut eng = ScenarioEngine::new();
        eng.schedule(
            ScenarioTrigger::AtInstruction(100),
            ScenarioEffect::SetOverride {
                address: 0x0100_240C,
                spec: OverrideSpec::StaticRead { value: 0x5000 },
                label: None,
            },
            None,
        );
        eng.tick(50);
        assert!(eng.overrides.is_empty());
        eng.tick(100);
        assert_eq!(eng.overrides.try_read(0x0100_240C), Some(0x5000));
    }

    #[test]
    fn engine_mmio_read_trigger() {
        let mut eng = ScenarioEngine::new();
        eng.schedule(
            ScenarioTrigger::OnMmioRead { address: 0x0100_240C, occurrence: 3 },
            ScenarioEffect::Pause,
            None,
        );
        eng.on_mmio_read(0x0100_240C);
        assert!(!eng.pause_requested);
        eng.on_mmio_read(0x0100_240C);
        assert!(!eng.pause_requested);
        eng.on_mmio_read(0x0100_240C);
        assert!(eng.take_pause());
    }

    #[test]
    fn engine_mmio_write_trigger() {
        let mut eng = ScenarioEngine::new();
        eng.schedule(
            ScenarioTrigger::OnMmioWrite { address: 0x0100_0050, occurrence: 1 },
            ScenarioEffect::SetOverride {
                address: 0x0100_0050,
                spec: OverrideSpec::StaticRead { value: 0xDEAD },
                label: None,
            },
            Some("timer override".into()),
        );
        eng.on_mmio_write(0x0100_0050);
        assert_eq!(eng.overrides.try_read(0x0100_0050), Some(0xDEAD));
    }

    #[test]
    fn engine_cancel_event() {
        let mut eng = ScenarioEngine::new();
        let id = eng.schedule(
            ScenarioTrigger::AtInstruction(50),
            ScenarioEffect::Pause,
            None,
        );
        assert!(eng.cancel(id));
        eng.tick(50);
        assert!(!eng.pause_requested);
    }

    #[test]
    fn engine_deferred_write() {
        let mut eng = ScenarioEngine::new();
        eng.schedule(
            ScenarioTrigger::OnMmioRead { address: 0x0100_0060, occurrence: 1 },
            ScenarioEffect::WriteMmio { address: 0x0100_0064, value: 0xBEEF },
            None,
        );
        eng.on_mmio_read(0x0100_0060);
        let writes = eng.take_deferred_writes();
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].address, 0x0100_0064);
        assert_eq!(writes[0].value, 0xBEEF);
    }

    #[test]
    fn engine_watchpoint_pause() {
        let mut eng = ScenarioEngine::new();
        eng.add_watchpoint(0x0100_240C, 4, MmioWatchMode::Write, MmioWatchAction::Pause, None);
        eng.on_mmio_read(0x0100_240C);
        assert!(!eng.pause_requested);
        eng.on_mmio_write(0x0100_240C);
        assert!(eng.take_pause());
    }

    #[test]
    fn engine_watchpoint_fire_effect() {
        let mut eng = ScenarioEngine::new();
        eng.add_watchpoint(
            0x0100_0060,
            4,
            MmioWatchMode::Read,
            MmioWatchAction::Fire(ScenarioEffect::SetOverride {
                address: 0x0100_0060,
                spec: OverrideSpec::OneShotRead { value: 0, remaining: 1 },
                label: None,
            }),
            Some("MDIO busy-clear on read".into()),
        );
        eng.on_mmio_read(0x0100_0060);
        assert_eq!(eng.overrides.peek_read(0x0100_0060), Some(0));
    }
}
