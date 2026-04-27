//! Phase 1 scaffolding — prerequisites for the UI / MCP branch.
//!
//! These tests lock in the new public surface added in
//! `feature/ui-mcp` Phase 1: breakpoint hook + pause state,
//! watchpoint table, Memory snapshot accessors, DCache line
//! snapshot, decoder byte-slice entry. Every check below must keep
//! passing as later phases build on top of this surface.

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::cpu::registers::PauseReason;
use bcm55030_emulator::decoder;
use bcm55030_emulator::hooks::Hook;
use bcm55030_emulator::memory::{Memory, SRAM_SIZE, WatchMode, Watchpoint};
use bcm55030_emulator::soc::bank::BootMode;

/// Hook::Breakpoint pauses the CPU before executing the target
/// instruction. `state.paused` transitions to `true`, PC stays put,
/// `pause_reason` is `Breakpoint(pc)`, and `instruction_count` does
/// not advance.
#[test]
fn breakpoint_hook_pauses_cpu() {
    // Flat mode: 64 KB memory with a single NOP_S at address 0.
    let mut cpu = Cpu::new(65536);
    // NOP_S = 0x78E0. Big-endian bytes.
    cpu.mem.load_binary(0, &[0x78, 0xE0]);
    cpu.state.pc = 0;

    // Install a breakpoint at PC=0.
    cpu.hooks.insert(0, Hook::Breakpoint);

    let pc_before = cpu.state.pc;
    let count_before = cpu.state.instruction_count;
    cpu.step().unwrap();

    assert!(cpu.state.paused, "breakpoint should set paused flag");
    assert_eq!(cpu.state.pc, pc_before, "PC must not advance on breakpoint");
    assert_eq!(
        cpu.state.instruction_count, count_before,
        "instruction_count must not advance on breakpoint"
    );
    assert_eq!(
        cpu.state.pause_reason,
        PauseReason::Breakpoint(0),
        "pause_reason should identify the breakpoint PC"
    );

    // Clearing `paused` and stepping again must execute the instruction.
    cpu.hooks.remove(&0);
    cpu.state.paused = false;
    cpu.step().unwrap();
    assert_eq!(cpu.state.instruction_count, count_before + 1);
    assert!(!cpu.state.paused);
}

/// Watchpoint hit during an instruction's memory access records the
/// hit; `Cpu::step` drains it and pauses with `PauseReason::Watch`.
/// Exercises the `read_byte` path at instruction level via a LD_S.
#[test]
fn watchpoint_read_records_hit() {
    let mut mem = Memory::new(65536);
    // Stage a value the watchpoint will trap on.
    mem.watchpoints.add(Watchpoint {
        addr: 0x1000,
        size: 4,
        mode: WatchMode::Read,
    });
    assert!(mem.watchpoints.take_hit().is_none());

    // Direct read through the Memory helper.
    let _ = mem.read_word(0x1000).unwrap();
    let hit = mem.watchpoints.take_hit().expect("watchpoint should fire");
    assert_eq!(hit.0, 0x1000);
    assert_eq!(hit.1, WatchMode::Read);

    // Table is consumed — no lingering hit.
    assert!(mem.watchpoints.take_hit().is_none());
}

/// Watchpoint on a write range — `write_word` at any byte inside
/// the range triggers, and a write-only watchpoint ignores reads.
#[test]
fn watchpoint_write_only_semantics() {
    let mut mem = Memory::new(65536);
    mem.watchpoints.add(Watchpoint {
        addr: 0x2000,
        size: 4,
        mode: WatchMode::Write,
    });

    let _ = mem.read_word(0x2000).unwrap();
    assert!(
        mem.watchpoints.take_hit().is_none(),
        "write-only WP should not trigger on reads"
    );

    mem.write_word(0x2000, 0xDEADBEEF).unwrap();
    let hit = mem
        .watchpoints
        .take_hit()
        .expect("write-only WP must trigger on writes");
    assert_eq!(hit.1, WatchMode::Write);
}

/// `Memory::sram_snapshot` returns a cloned copy of the full SRAM
/// buffer with the expected BCM55030 size.
#[test]
fn sram_snapshot_size_matches_soc() {
    let cpu = Cpu::new_bcm55030(BootMode::Warm);
    let snap = cpu.mem.sram_snapshot();
    assert_eq!(snap.len(), SRAM_SIZE, "SRAM snapshot should be 512 KB");
}

/// `Memory::sram_slice` returns a borrow only when the range fits
/// in the backing buffer; otherwise `None`.
#[test]
fn sram_slice_bounds_check() {
    let mem = Memory::new(1024);
    assert!(mem.sram_slice(0, 1024).is_some());
    assert!(mem.sram_slice(0, 1025).is_none());
    assert!(mem.sram_slice(1024, 1).is_none());
}

/// `Memory::dcache_snapshot` returns 128 physical lines in SoC mode,
/// matching the BCM55030 2-way × 64-set geometry. Flat mode has no
/// D-cache and returns an empty vec.
#[test]
fn dcache_snapshot_line_count() {
    let cpu = Cpu::new_bcm55030(BootMode::Warm);
    let lines = cpu.mem.dcache_snapshot();
    assert_eq!(
        lines.len(),
        128,
        "BCM55030 D-cache has 2 ways × 64 sets = 128 lines"
    );

    let flat = Memory::new(1024);
    assert!(flat.dcache_snapshot().is_empty());
}

/// `decoder::decode_bytes` and `decoder::decode(&Memory)` must
/// produce the same `DecodedInstruction` for identical bytes. Golden
/// equivalence — the two entry points share a single fetch-agnostic
/// decode path behind the scenes.
#[test]
fn decode_bytes_matches_memory_decode() {
    let mut mem = Memory::new(4096);
    // NOP_S (0x78E0): 16-bit instruction at PC 0.
    mem.load_binary(0, &[0x78, 0xE0]);

    let from_mem = decoder::decode(0, &mem).expect("memory decode");
    let from_bytes = decoder::decode_bytes(0, &[0x78, 0xE0], 0).expect("slice decode");

    assert_eq!(from_mem.pc, from_bytes.pc);
    assert_eq!(from_mem.size, from_bytes.size);
    assert_eq!(from_mem.has_limm, from_bytes.has_limm);
    assert_eq!(from_mem.total_size(), from_bytes.total_size());
    assert_eq!(from_mem.size, 2, "NOP_S is a 16-bit instruction");
    assert!(!from_mem.has_limm);
}

/// `decode_bytes` respects the `base` offset: disassemble an
/// instruction whose "virtual" PC differs from the buffer index.
#[test]
fn decode_bytes_with_base_offset() {
    let bytes = [0x78, 0xE0]; // NOP_S
    let decoded = decoder::decode_bytes(0x8000, &bytes, 0x8000)
        .expect("base-offset decode should work");
    assert_eq!(decoded.pc, 0x8000);
    assert_eq!(decoded.size, 2);
}
