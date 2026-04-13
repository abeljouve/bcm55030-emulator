//! Synthesized quiescent-ONU alarm state. See `the design notes`.

use crate::memory::Memory;

/// Base address of the persistent alarm pending-bit mask in DCCM.
/// Stored as the literal `0x0001100c` at firmware the decompiler `0x20033ABC`.
pub const ALARM_MASK_BASE: u32 = 0x1100C;

/// Base address of the GPIO channel-head pointer table in DCCM.
/// Stored as the literal `0x000030c0` at firmware the decompiler `0x2000FFDC`.
pub const GPIO_HEAD_TABLE_BASE: u32 = 0x30C0;

/// Synthetic GPIO chan-15 node. Sits in SRAM gap between firmware .bss and top.
const GPIO_CHAN15_NODE_ADDR: u32 = 0x0007FFE0;

/// Persistent alarm opcodes (bit 0 each) asserted on a quiescent ONU.
/// Verified against the live HW capture `cli_outputs/level0/alm_info.txt`.
pub const PERSISTENT_OPCODES: [u16; 7] = [23, 28, 64, 131, 193, 199, 201];

/// Mirror of `hw_irq_opcode_to_handler_index @ the decompiler 0x20033924`.
/// Returns the handler-index (used to offset into the 147-entry u64 mask
/// array at `ALARM_MASK_BASE`) or `None` if the opcode is unmapped.
fn opcode_to_handler_index(opcode: u16) -> Option<u32> {
    let op = opcode as u32;
    if op > 0x142 {
        if (0x200..0x29E).contains(&op) {
            if op < 0x21D {
                return Some(op.wrapping_sub(0x1AE) & 0xFFFF);
            }
            return Some(op.wrapping_sub(0x211) & 0xFFFF);
        }
        if (0x300..0x301).contains(&op) {
            return Some(op.wrapping_sub(0x273) & 0xFFFF);
        }
        if (0x1000..0x1005).contains(&op) {
            return Some(op.wrapping_sub(0xF72) & 0xFFFF);
        }
        return None;
    }
    if op < 0x25 {
        return Some(op.wrapping_sub(1) & 0xFFFF);
    }
    if (0x40..0x52).contains(&op) {
        return Some(op.wrapping_sub(0x1C) & 0xFFFF);
    }
    if (0x80..0x8A).contains(&op) {
        return Some(op.wrapping_sub(0x4A) & 0xFFFF);
    }
    if (0xC0..0xCE).contains(&op) {
        return Some(op.wrapping_sub(0x80) & 0xFFFF);
    }
    if (0x100..0x101).contains(&op) {
        return Some(op.wrapping_sub(0xB2) & 0xFFFF);
    }
    None
}

pub struct AlarmModel {
    /// u32 addresses holding bit 0 of each persistent opcode's mask.
    persistent_word_addrs: [u32; 7],
    seeded: bool,
    tick_prescaler: u64,
}

impl Default for AlarmModel {
    fn default() -> Self {
        let mut addrs = [0u32; 7];
        for (i, &op) in PERSISTENT_OPCODES.iter().enumerate() {
            let idx = opcode_to_handler_index(op).expect("persistent opcode must map");
            addrs[i] = ALARM_MASK_BASE + idx * 8 + 4;
        }
        Self {
            persistent_word_addrs: addrs,
            seeded: false,
            tick_prescaler: 64,
        }
    }
}

impl AlarmModel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Seed on first tick once `chan_task_descriptor_init` has zeroed the head
    /// table, then re-assert bits each subsequent tick to counter firmware clears.
    pub fn tick(&mut self, mem: &mut Memory, insn_count: u64) {
        if insn_count % self.tick_prescaler != 0 {
            return;
        }

        if !self.seeded {
            if insn_count < 1_000_000 {
                return;
            }
            let first = mem.read_word(GPIO_HEAD_TABLE_BASE).unwrap_or(0xFFFFFFFF);
            if first != 0 {
                return;
            }
            self.seed(mem);
            self.seeded = true;
            return;
        }

        // Post-seed: re-assert any bit that was cleared since the last tick.
        for &addr in &self.persistent_word_addrs {
            let val = mem.read_word(addr).unwrap_or(0);
            if val & 0x0000_0001 == 0 {
                let _ = mem.write_word(addr, val | 0x0000_0001);
            }
        }

        // Re-assert the GPIO chan-15 head pointer if the firmware emptied it.
        let head_addr = GPIO_HEAD_TABLE_BASE + 15 * 4;
        if mem.read_word(head_addr).unwrap_or(0) == 0 {
            self.write_chan15_node(mem);
            let _ = mem.write_word(head_addr, GPIO_CHAN15_NODE_ADDR);
        }
    }

    fn seed(&mut self, mem: &mut Memory) {
        for &addr in &self.persistent_word_addrs {
            let val = mem.read_word(addr).unwrap_or(0);
            let _ = mem.write_word(addr, val | 0x0000_0001);
        }
        self.write_chan15_node(mem);
        let _ = mem.write_word(GPIO_HEAD_TABLE_BASE + 15 * 4, GPIO_CHAN15_NODE_ADDR);
    }

    /// Layout: `{next=0, chan=15, prio_lo=5, 0, 0}` — matches real chan_dispatch.
    fn write_chan15_node(&self, mem: &mut Memory) {
        let node = GPIO_CHAN15_NODE_ADDR;
        let _ = mem.write_word(node, 0);
        let _ = mem.write_byte(node + 4, 15);
        let _ = mem.write_byte(node + 5, 5);
        let _ = mem.write_byte(node + 6, 0);
        let _ = mem.write_byte(node + 7, 0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn persistent_opcodes_map_to_distinct_indices() {
        let mut seen = std::collections::HashSet::new();
        for &op in &PERSISTENT_OPCODES {
            let idx = opcode_to_handler_index(op).expect("must map");
            assert!(seen.insert(idx), "opcode {op} duplicated index {idx}");
        }
    }

    #[test]
    fn opcode_indices_match_investigation_table() {
        // From the design notes:
        assert_eq!(opcode_to_handler_index(23), Some(22));
        assert_eq!(opcode_to_handler_index(28), Some(27));
        assert_eq!(opcode_to_handler_index(64), Some(36));
        assert_eq!(opcode_to_handler_index(131), Some(57));
        assert_eq!(opcode_to_handler_index(193), Some(65));
        assert_eq!(opcode_to_handler_index(199), Some(71));
        assert_eq!(opcode_to_handler_index(201), Some(73));
    }
}
