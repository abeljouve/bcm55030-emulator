# CLAUDE.md — contributor & agent guide

Big-endian **ARC700 / ARCompact** CPU emulator in Rust for the Broadcom
**BCM55030** SoC: a unified 512 KiB SRAM with I-cache (4 KiB, direct-mapped,
32 B lines) and D-cache (4 KiB, 2-way, 32 B lines), MMIO peripherals,
zero-overhead loops, and delay slots. It boots and runs firmware you supply;
see [README.md](README.md) for architecture and usage.

## The one invariant: model the hardware, not a specific firmware

- The emulator reproduces **observed silicon behaviour**. Fixes are surgical and
  grounded in what the hardware actually does — never in what a particular
  firmware image happens to need. No firmware-specific hooks, no "make image X
  boot" shortcuts.
- Peripheral behaviour, reset values and register semantics come from black-box
  reverse engineering of real hardware and from public standards (IEEE 802.3
  EPON/MPCP, SFF-8472/8024/8079, the ARCompact/ARC700 ISA reference). See
  [PROVENANCE.md](PROVENANCE.md).
- **No device identity is baked in.** Serial numbers, MACs, SFP EEPROM and eFUSE
  contents ship as synthetic placeholders; real values are loaded at runtime.
  Keep it that way — do not commit captured identity or firmware.

## Layout

```
src/cpu       core: registers, execution state, delay slots
src/decoder   16/32-bit ARCompact decoder + formatter
src/executor  instruction semantics (ALU, extended, memory, special)
src/cache     I/D-cache model
src/memory    SRAM / flat memory, MMIO routing
src/soc       on-chip peripherals (one module per block)
src/emu       engine, worker thread, hooks, snapshots
src/mcp       embedded MCP server (rmcp + axum)
src/ui        egui/eframe GUI (feature `ui`)
```

## Build & test

```bash
cargo build --release                 # CLI (arc700), MCP included
cargo build --release --features ui   # GUI (arc700-gui)
cargo test                            # unit + integration; keep it green
```

## Conventions

- Comments distinguish what is **observed** (verified against real hardware or
  the ISA spec) from what is **inferred** (derived, not yet verified). Preserve
  that distinction when editing.
- Keep every `cargo test` green; a peripheral change that alters MMIO behaviour
  should come with a test. Prefer small, reviewable changes.
- English only, in code, comments and commits.
