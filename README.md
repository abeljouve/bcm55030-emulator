# BCM55030 SoC Emulator

A Rust emulator for the **Broadcom BCM55030**, a big-endian SoC built around an
**ARC700 / ARCompact** core. It models the CPU together with the on-chip
peripherals (UART, timers, SPI flash, I²C/BSC, eFUSE, SerDes, EPON MAC, MPCP,
MACsec, DMA, NCO, SFP EEPROM, …) so that firmware can be booted and inspected
entirely in software.

The emulator carries **no firmware knowledge**: no vendor images or boot ROMs
are compiled in, and the on-chip identity (serial numbers, MACs, SFP EEPROM,
eFUSE) ships only as synthetic placeholders. Everything device-specific — the
flash image, symbols, annotations, real EEPROM/eFUSE contents — is loaded from
the outside at runtime. See [`PROVENANCE.md`](PROVENANCE.md) for how the model
was derived.

Two binaries are shipped:

- **`arc700`** — headless CLI (UART on stdin/stdout) with an embedded MCP server.
- **`arc700-gui`** — an `eframe`/`egui` GUI with an interactive disassembler,
  peripheral inspector, and the same embedded MCP server.

## Build

```bash
cargo build --release                 # CLI, MCP server included
cargo build --release --features ui   # GUI + MCP server
cargo test                            # unit + integration tests
```

The MCP server (`rmcp` + `axum`) is compiled into every build by default. The
only optional feature is `ui` (the `egui`/`eframe` GUI).

## CLI (`arc700`)

Loads a flash image into the SPI-flash peripheral, performs the 64 KiB DMA boot
into SRAM, and runs the CPU.

```bash
cargo run --release -- firmware.bin
cargo run --release -- firmware.bin --trace --break 0x5B3C
cargo run --release -- firmware.bin --mcp-port 3001
```

Selected flags:

| Flag | Effect |
|------|--------|
| `--entry <ADDR>` | Entry point (hex, default `0x0000`) |
| `--max-cycles <N>` | Instruction budget (default: unlimited) |
| `--trace` / `--trace-from-insn <N>` | Log each instruction on stderr |
| `--trace-mmio` / `--trace-mmio-seq <FILE>` | Log MMIO accesses (text / JSON Lines) |
| `--break <ADDR>` / `--watch-dccm <ADDR>` | Breakpoint / DCCM watchpoint |
| `--cold-boot` / `--warm-boot` | Boot mode (default: warm) |
| `--dccm-dump <FILE>` | Dump SRAM to a file on exit |
| `--persist-flash` | Write modified flash back to `<firmware>.persist` |
| `--debug-elf <FILE>` | Load DWARF info to annotate `--trace` with source file/line |
| `--scenario <FILE>` | Apply a JSON scenario at startup |
| `--mcp-port <PORT>` | Start an MCP server on `PORT` (enables worker mode) |

Interactive mode turns on automatically when stdin is a TTY: keystrokes are fed
to the firmware's UART RX in real time.

## GUI (`arc700-gui`)

```bash
cargo run --release --features ui --bin arc700-gui                 # GUI + MCP on :3000
cargo run --release --features ui --bin arc700-gui -- --no-mcp     # GUI only
```

Panels: an interactive disassembler with Ghidra-style branch arcs and symbol
resolution; a register grid (r0–r31, special and Aux registers, STATUS32 flags);
a hex/ASCII memory viewer for SRAM/Flash/D-cache with dirty-byte highlighting; a
peripheral inspector with per-block sub-tabs and event injection; a UART
terminal; and an MCP activity log.

## MCP server (Model Context Protocol)

Both binaries embed an HTTP MCP server (default port `3000`) exposing 30+ tools
to drive and inspect the emulator: run/step/reset, register and memory
read/write, MMIO peek/override, breakpoints and watchpoints, snapshots,
peripheral event injection, and disassembly. This makes the emulator scriptable
from any MCP-capable client.

## Architecture

```
src/
├── cpu/        ARC700 core: registers, execution state, delay slots
├── decoder/    16/32-bit ARCompact instruction decoder + formatter
├── executor/   instruction semantics (ALU, extended, memory, special)
├── cache/      I/D-cache model
├── memory/     SRAM / flat memory, MMIO routing
├── soc/        on-chip peripherals (uart, timer, spi_flash, bsc_i2c, efuse,
│               serdes, epon_mac, mpcp, macsec, dma, nco, sfp_eeprom, olt, …)
├── emu/        emulator engine, worker thread, hooks, snapshots
├── mcp/        embedded MCP server (rmcp + axum)
└── ui/         egui/eframe GUI (feature `ui`)
```

## Fidelity

The ARC700 core is modelled faithfully (including this SoC's quirks, e.g. it has
no `rtie`). The SoC peripherals reproduce the register-level behaviour firmware
observes, but the **analog / clock-domain** behaviour of the SerDes / PCS is
simplified. See [`FIDELITY.md`](FIDELITY.md) for the known divergences.

## License

Licensed under the **GNU Affero General Public License v3.0** — see
[`LICENSE`](LICENSE). The bundled JetBrains Mono font is under the SIL Open Font
License 1.1 (see [`assets/fonts/OFL.txt`](assets/fonts/OFL.txt)).

This is an independent reverse-engineering project. It is not affiliated with or
endorsed by the SoC manufacturer; product names are used only to identify the
hardware being modelled. See [`PROVENANCE.md`](PROVENANCE.md).
