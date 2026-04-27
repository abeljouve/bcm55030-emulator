//! MMIO override registry for HW scenario programming.
//!
//! Overrides intercept MMIO reads/writes **before** the normal peripheral
//! dispatch chain.  This models external HW stimulus — "the PHY drove
//! this value on the bus" — not firmware patching.  `ScenarioEffect`
//! deliberately has no variant that touches CPU state (registers, PC,
//! SRAM).  Rule 1 (no firmware hooks) is preserved.
//!
//! `peek_word` (GUI 60 Hz polling, MCP `peek_mmio`) does NOT consume
//! one-shot overrides.  Only `read_word` / `write_word` (CPU-driven
//! accesses with side effects) decrement the remaining counter.

use std::collections::HashMap;

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
}
