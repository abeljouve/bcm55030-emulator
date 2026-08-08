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
src/soc/olt   EPON peer model, split by concern (see below)
src/emu       engine, worker thread, hooks, snapshots
src/mcp       embedded MCP server (rmcp + axum)
src/ui        egui/eframe GUI (feature `ui`)
```

### The OLT peer, and where it lives

The equipment on the far end of the fibre is **not** a peripheral. It lives in
`crates/epon-olt`, runs its own loop on its own clock, and knows nothing about
this SoC — which is what lets the same engine run with no CPU present at all:

```bash
cargo run -p epon-olt --bin olt -- check     # peer + minimal responder
cargo run -p epon-olt --bin olt -- run       # peer with nothing answering
cargo run -p epon-olt --bin olt -- dissect <hex>
```

| module | holds |
|---|---|
| `clock.rs` | wire time: one base, picoseconds of link time |
| `sched.rs` | the due-time queue the peer's loop runs on |
| `peer.rs` | the peer: state machine, timers, counters |
| `fibre.rs` | one direction of link: travel, jitter, finite depth |
| `link.rs` | a peer with fibre either side of it |
| `onu.rs` | a minimal responder, to exercise the peer on its own |
| `types.rs` | `MacAddr`, `EtherType`, `Llid`, `FrameWriter` |
| `mpcp.rs` | MPCPDU encode/decode (clause 64) |
| `oam.rs` | OAMPDU encode/decode (clause 57) |
| `extended.rs` | organization-specific OAMPDUs and their variables |
| `decode.rs` | frame dissection, shared by the MCP tools and the GUI |

What stays in `src/soc/olt` is the join: the frame-queue mailbox, and the
conversion between the two clocks. Rules for working on it:

- **Intervals are link time, never ticks.** A tick is a host quantity. The one
  place they meet is `TICK_PS × time_scale`, and the scale multiplies the clock
  rather than dividing the intervals, so a duration measured between two frames
  still equals the interval that produced it.
- **Nothing decides "has enough elapsed".** Work is scheduled with a due time
  and fires at that instant, not when the loop got round to it.
- **Queues are finite and losses are counted.** A queue that never drops turns
  a burst into a backlog delivered late, which no real downstream does.
- **Parse before deciding, and count every refusal.** A path that discards a
  frame without incrementing something is a fault that will be read as a clean
  link.
- Keep protocol logic out of `bank.rs`: the bank routes MMIO and owns the
  datapath, the peer owns frame semantics.

## Build & test

```bash
cargo build --release                 # CLI (arc700), MCP included
cargo build --release --features ui   # GUI (arc700-gui)
cargo build --release -p epon-olt     # the peer on its own (bin: olt)
cargo test --workspace                # unit + integration; keep it green
```

## Conventions

- Comments distinguish what is **observed** (verified against real hardware or
  the ISA spec) from what is **inferred** (derived, not yet verified). Preserve
  that distinction when editing.
- Keep every `cargo test` green; a peripheral change that alters MMIO behaviour
  should come with a test. Prefer small, reviewable changes.
- English only, in code, comments and commits.

### Types, not magic numbers

Protocol fields get named types — an enum for a code point, a struct for a
flags octet, a constant for an offset. Do not paste hex blobs: a literal frame
or word array in a test hides what it encodes and silently rots when a field
moves. Build fixtures from the same constructors the model uses, so a test that
still passes proves the round trip rather than the copy.

Bit layouts belong in one place, next to their `encode`/`decode` pair, and
should round-trip in a test.

### Comments

Say what the code cannot. Skip the ones that restate the line below, and keep
what explains a choice, a constraint, or a mechanism that is not visible from
the signature. A short note beats a paragraph.

Keep comments to what the emulator itself needs. Findings, measurements and
external context live in the workshop repository, not in this one.
