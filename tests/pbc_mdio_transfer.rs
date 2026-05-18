//! Closes hypotheses (b) wrong-dest and (c) wrong-length of
//! `the design notes` D2,
//! and re-checks (a) dirty-line preservation through the *real* PBC
//! MMIO path (not just `apply_datapath_op` directly).
//!
//! Drives the exact register sequence `hw_mdio_read_command` emits:
//!   *0x0100022C (REG_DMA_ADDR,      base+0x3C) = src  (flash addr)
//!   *0x01000230 (REG_DMA_DATA_ADDR, base+0x40) = dst  (RAM addr)
//!   *0x01000228 (REG_DMA_CTRL,      base+0x38) = (len<<4)|1   "go"
//! and asserts the PBC delivers exactly `len` bytes from `flash[src]`
//! to `RAM[dst]`, leaving the adjacent words untouched.

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::soc::bank::BootMode;

const REG_DMA_CTRL: u32 = 0x0100_0228;
const REG_DMA_ADDR: u32 = 0x0100_022C;
const REG_DMA_DATA_ADDR: u32 = 0x0100_0230;

fn preload_flash(cpu: &Cpu, addr: usize, bytes: &[u8]) {
    let bank = cpu.mem.bank().unwrap();
    let mut g = bank.write();
    g.pbc.flash.data[addr..addr + bytes.len()].copy_from_slice(bytes);
}

#[test]
fn pbc_mdio_read_delivers_exact_len_to_exact_dst() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    // Flash payload at 0x90000 (the toxic read's effective src);
    // the byte right after must NOT be transferred for len=4.
    preload_flash(&cpu, 0x9_0000, &[0xDE, 0xAD, 0xBE, 0xEF, 0x99]);

    let dst = 0x0003_1F7Cu32;
    // CPU-written sentinels in the neighbouring words (direct SRAM).
    cpu.mem.write_word_data(dst - 4, 0xA1A1_A1A1, true).unwrap();
    cpu.mem.write_word_data(dst + 4, 0xB2B2_B2B2, true).unwrap();

    // hw_mdio_read_command(dst, src=0x90000, len=4)
    cpu.mem.write_word(REG_DMA_ADDR, 0x9_0000).unwrap();
    cpu.mem.write_word(REG_DMA_DATA_ADDR, dst).unwrap();
    cpu.mem.write_word(REG_DMA_CTRL, (4 << 4) | 1).unwrap();
    cpu.mem.drain_datapath_public().unwrap();

    // (b)+(c): exactly 4 bytes, exactly at dst, neighbours intact.
    assert_eq!(
        cpu.mem.read_word_data(dst, true).unwrap(),
        0xDEAD_BEEF,
        "MDIO must deliver flash[src] to exactly dst"
    );
    assert_eq!(
        cpu.mem.read_word_data(dst - 4, true).unwrap(),
        0xA1A1_A1A1,
        "word below dst must be untouched (no under-run)"
    );
    assert_eq!(
        cpu.mem.read_word_data(dst + 4, true).unwrap(),
        0xB2B2_B2B2,
        "word above dst must be untouched (no over-run / wrong len)"
    );
}

#[test]
fn pbc_mdio_read_len2_transfers_exactly_two_bytes() {
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    preload_flash(&cpu, 0x9_0000, &[0x12, 0x34, 0x56, 0x78]);
    let dst = 0x0004_0000u32;
    cpu.mem.write_word_data(dst, 0x0000_0000, true).unwrap();

    cpu.mem.write_word(REG_DMA_ADDR, 0x9_0000).unwrap();
    cpu.mem.write_word(REG_DMA_DATA_ADDR, dst).unwrap();
    cpu.mem.write_word(REG_DMA_CTRL, (2 << 4) | 1).unwrap();
    cpu.mem.drain_datapath_public().unwrap();

    // Only the first 2 bytes written; bytes 2..4 keep their value.
    assert_eq!(cpu.mem.read_word_data(dst, true).unwrap(), 0x1234_0000);
}

#[test]
fn pbc_mdio_read_preserves_dirty_line_through_real_path() {
    // (a) re-checked via the real PBC MMIO path: a caller's cached
    // (dirty) saved-blink in the same 32-byte line as the MDIO dst
    // survives the transfer — the j [blink=0] reboot is gone.
    let mut cpu = Cpu::new_bcm55030(BootMode::Warm);
    preload_flash(&cpu, 0x9_0000, &[0xFF, 0xFF, 0xFF, 0xFF]);

    let blink_slot = 0x0003_1F7Cu32;
    let saved_blink = 0x0003_60BEu32;
    // Caller `st blink,[sp,N]` — cached, write-back → dirty line.
    cpu.mem.write_word_data(blink_slot, saved_blink, false).unwrap();

    // flash_memcpy head read into a stack local in the SAME line.
    cpu.mem.write_word(REG_DMA_ADDR, 0x9_0000).unwrap();
    cpu.mem.write_word(REG_DMA_DATA_ADDR, 0x0003_1F70).unwrap();
    cpu.mem.write_word(REG_DMA_CTRL, (4 << 4) | 1).unwrap();
    cpu.mem.drain_datapath_public().unwrap();

    assert_eq!(
        cpu.mem.read_word_data(blink_slot, false).unwrap(),
        saved_blink,
        "dirty saved-blink must survive the PBC MDIO DMA write"
    );
}
