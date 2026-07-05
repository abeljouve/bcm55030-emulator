//! Regression for the design notes.
//!
//! `reset_soc_in_place` (used by the GUI/MCP `load_firmware` and
//! `Reset` paths) reuses the same `Memory` — and therefore the same
//! I/D-caches — across resets. Before the fix it zeroed SRAM but never
//! invalidated the caches, so a second firmware load ran the new
//! bootloader's `0x0..0x80` IVT/code against STALE I-cache lines from
//! the previous run: garbage fetch → PC=0xFFFFFFFF crash at insn 511,
//! diverging from silicon (and from the CLI, which uses a fresh `Cpu`).
//!
//! Silicon has no "previous life" across a power-on / watchdog reset:
//! the caches are invalid after reset, and DATASHEET §5.4 mandates the
//! boot DMA broadcast cache invalidations. This test pins that.

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::decoder;
use bcm55030_emulator::decoder::instruction::{Instruction, Operand};
use bcm55030_emulator::soc::bank::BootMode;

#[test]
fn reset_soc_in_place_invalidates_stale_icache_for_ivt_region() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);

    // "Previous life": a firmware whose code/IVT lives at 0x0..0x80.
    // Fill SRAM and prime the I-cache by fetching the reset slot.
    let prev = [0xAAu8; 0x80];
    cpu.mem.load_binary(0, &prev);
    let primed = cpu.mem.fetch_word(0x0).unwrap();
    assert_eq!(primed, 0xAAAA_AAAA, "I-cache should hold the prev image");

    // Power-on / watchdog reset, then the next firmware's boot DMA
    // overwrites 0x0..0x80 with a different IVT image.
    cpu.reset_soc_in_place(BootMode::Warm);
    let next = [0x55u8; 0x80];
    cpu.mem.load_binary(0, &next);

    // Silicon fetches the freshly-DMA'd bytes. A stale I-cache line
    // would still return 0xAAAAAAAA → the insn-511 garbage crash.
    let fetched = cpu.mem.fetch_word(0x0).unwrap();
    assert_eq!(
        fetched, 0x5555_5555,
        "post-reset fetch must see the new image, not a stale I-cache line"
    );

    // And a slot deeper into the IVT alias (e.g. IRQ-5 vector @ 0x28).
    let irq5 = cpu.mem.fetch_word(0x28).unwrap();
    assert_eq!(irq5, 0x5555_5555, "IVT slot fetch must not be stale");
}

/// Bug point (b): an interrupt-vector slot in the reference 8-byte form
/// `2020 0f80 <hi16> <lo16>` is the ARCompact `j @<absolute>` opcode
/// pair. The emulator vectors an IRQ to `AUX 0x25 base + N*8` and
/// then fetches/executes the slot — so that slot must decode as a
/// jump whose target is the absolute `<hi:lo>`, and executing it must
/// land PC there. Live-silicon anchor: slot0 = `2020 0f80 0000 0150`
/// = `j @0x150` (`project_dma_256k_chunk_rootcause`).
#[test]
fn ivt_slot_j_limm_decodes_and_executes_to_absolute_target() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);

    // slot @ 0x0 = j @0x00000200  (20 20 0f 80 | 00 00 02 00, big-endian)
    let img: [u8; 8] = [0x20, 0x20, 0x0f, 0x80, 0x00, 0x00, 0x02, 0x00];
    cpu.mem.load_binary(0x0, &img);

    let d = decoder::decode(0x0, &cpu.mem).unwrap();
    assert_eq!(d.total_size(), 8, "j @limm is a 4-byte op + 4-byte LIMM");
    match d.inst {
        Instruction::Jump { target, link, .. } => {
            assert!(!link, "interrupt vector slot is `j`, not `jl`");
            assert_eq!(
                target,
                Operand::Imm(0x0000_0200),
                "j @limm target must be the absolute <hi:lo> word"
            );
        }
        other => panic!("IVT slot did not decode as Jump: {other:?}"),
    }

    // Execute it from PC=0 (no IRQ/delay interference: a fresh Cpu has
    // E1/E2 clear and no pending lines). PC must reach the vector.
    cpu.state.pc = 0x0;
    cpu.step().unwrap();
    assert_eq!(
        cpu.state.pc, 0x0000_0200,
        "executing the IVT `j @limm` slot must jump to the absolute target"
    );
}

/// Bug points (b)+(c) end-to-end: a `.di` store sequence into the
/// low-memory aperture (exactly what `nco_write_channel` /
/// `hw_install_irq_vector_2` emit) programs the NCO/IVT channel, and
/// the ARC interrupt unit then vectors that IRQ to the channel's
/// `j @<absolute>` target — NOT to the SRAM IVT slot. Pre-fix the
/// emulator had no NCO aperture and vectored through SRAM only.
#[test]
fn di_programmed_nco_slot_is_used_for_interrupt_vectoring() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);

    // arc700-rt-style stale bytes sit in the SRAM IVT slot for IRQ-5
    // (@ 0x28): if the model wrongly used SRAM the PC would land here.
    cpu.mem.load_binary(0x28, &[0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0]);

    // `nco_write_channel(5, 0x0003_F108)`: four 16-bit `.di` field
    // stores (CONFIG / PRESCALE / FREQ_HI / FREQ_LO).
    let base = 5u32 * 8;
    cpu.mem.write_half_data(base, 0x2020, true).unwrap();
    cpu.mem.write_half_data(base + 2, 0x0F80, true).unwrap();
    cpu.mem.write_half_data(base + 4, 0x0003, true).unwrap();
    cpu.mem.write_half_data(base + 6, 0xF108, true).unwrap();

    // A 16-bit NOP_S at PC=0; executing it then takes the pending IRQ.
    cpu.mem.load_binary(0x0, &[0x78, 0xE0]);
    cpu.state.pc = 0x0;
    cpu.state.aux_ienable = 0xFFFF_FFFF;
    cpu.state.aux_irq_lev = 0; // IRQ-5 = level 1
    cpu.state.flag_e1 = true;
    cpu.state.aux_irq_pending = 1 << 5;

    cpu.step().unwrap();

    assert_eq!(
        cpu.state.pc, 0x0003_F108,
        "IRQ-5 must vector to the .di-programmed NCO channel target, \
         not the stale SRAM IVT slot (0xDEADBEEF)"
    );
}

/// Bug point (d): I-cache coherence between code fetched from the
/// `0x0..0x80` IVT alias and a later `.di` (cache-bypass) rewrite.
/// DATASHEET §5.4: a `.di` store updates SRAM only and does NOT
/// invalidate the I-cache — execution keeps using the stale line
/// until an explicit `IC_IVIC` (the reference `nco_commit_fence`). This
/// pins both halves: stale-until-IVIC, then fresh-after-IVIC.
#[test]
fn di_store_to_ivt_is_stale_until_ic_ivic() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);

    // Prime: image A at the IVT alias, fetched into the I-cache.
    cpu.mem.load_binary(0x0, &[0xAAu8; 0x40]);
    assert_eq!(cpu.mem.fetch_word(0x0).unwrap(), 0xAAAA_AAAA);

    // `.di` store of a new vector word (cache_bypass = true). SRAM is
    // updated; the I-cache line is intentionally NOT touched.
    cpu.mem
        .write_word_data(0x0, 0x5555_5555, true)
        .unwrap();

    // Faithful silicon behaviour: instruction fetch still serves the
    // stale cached bytes (this is exactly why the bootloader's
    // executed `0x0..0x80` code matters on silicon).
    assert_eq!(
        cpu.mem.fetch_word(0x0).unwrap(),
        0xAAAA_AAAA,
        ".di store must NOT invalidate the I-cache (stale until IC_IVIC)"
    );

    // IC_IVIC commit fence → the rewrite becomes visible to fetch.
    cpu.mem.icache_invalidate_all();
    assert_eq!(
        cpu.mem.fetch_word(0x0).unwrap(),
        0x5555_5555,
        "after IC_IVIC the freshly .di-written vector must be fetched"
    );
}
