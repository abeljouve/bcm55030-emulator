//! MCP tool handler for the ARC700 emulator. Read-only (phase 4)
//! plus bank-side mutations (phase 5a). CpuCommand-backed
//! mutations (cpu_run, set_breakpoint, write_register, ...)
//! land in phase 5b.
//!
//! Tools follow the the design spec §MCP Server §Tools categories:
//! firmware / cpu / memory / disassembly / peripherals / flash /
//! annotations / breakpoints.
//!
//! the contributor guide: nothing in this module carries firmware-specific
//! constants. Symbols are fetched from `handle.annotations` which
//! is user-loaded at runtime.

use std::collections::HashMap;

use rmcp::handler::server::tool::ToolRouter;
use rmcp::handler::server::wrapper::{Json, Parameters};
use rmcp::model::{
    ProtocolVersion, ServerCapabilities, ServerInfo,
};
use rmcp::{tool, tool_handler, tool_router, ServerHandler};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::emu::{EmulatorHandle, RunState};
use crate::soc::peripheral::{
    AlarmEvent, FatalFilterEvent, PbcEvent, PeripheralEvent, PeripheralSnapshot, SerDesEvent,
    UartEvent,
};

/// Tool handler — one instance is cheap (all state lives behind
/// `Arc`s on the `EmulatorHandle`). `StreamableHttpService` clones
/// this per session via the factory closure in `server.rs`.
#[derive(Clone)]
pub struct EmulatorHandler {
    handle: EmulatorHandle,
    // Consumed by the `#[tool_handler]` macro expansion; the
    // compiler can't see that through the macro hygiene
    // boundary, so silence the false-positive `dead_code`.
    #[allow(dead_code)]
    tool_router: ToolRouter<Self>,
}

impl EmulatorHandler {
    pub fn new(handle: EmulatorHandle) -> Self {
        Self {
            handle,
            tool_router: Self::tool_router(),
        }
    }
}

// ---------- Tool request / response DTOs ---------------------------------

#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
pub struct ReadRegistersParams {
    /// Optional list of register names. Each name may be a core
    /// register (r0..r31, sp, fp, gp, blink, lp_count, ilink1,
    /// ilink2), pc, or status32. When absent, returns every core
    /// register + pc + status32 + selected aux registers.
    pub names: Option<Vec<String>>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReadRegistersResult {
    pub values: HashMap<String, u32>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct FlagsResult {
    pub z: bool,
    pub n: bool,
    pub c: bool,
    pub v: bool,
    pub e1: bool,
    pub e2: bool,
    pub u: bool,
    pub h: bool,
    pub l: bool,
    pub de: bool,
    pub status32: u32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CpuStateResult {
    pub pc: u32,
    pub halted: bool,
    pub sleeping: bool,
    pub paused: bool,
    pub instruction_count: u64,
    pub run_state: String,
    pub pause_reason: String,
}

#[derive(Debug, Serialize, JsonSchema, Default)]
pub struct FirmwareInfoResult {
    pub loaded: bool,
    pub path: Option<String>,
    pub boot_mode: Option<String>,
    pub entry_point: Option<u32>,
    pub flash_size: Option<usize>,
    pub flash_loaded: Option<bool>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct PeekMmioParams {
    pub address: u32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PeekMmioResult {
    pub address: u32,
    pub value: u32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ReadFlashParams {
    pub offset: u32,
    pub length: u32,
}

// ---------- Phase 5a mutation DTOs ---------------------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct WriteMmioParams {
    pub address: u32,
    pub value: u32,
    /// Access width: `"byte"`, `"half"`, or `"word"` (default).
    pub width: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct OkResult {
    pub ok: bool,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SendUartInputParams {
    /// UTF-8 text. Each byte is pushed through the bank's mpsc
    /// UART RX channel — the same path the headless stdin loop
    /// uses.
    pub data: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SendUartInputResult {
    pub bytes_sent: usize,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct WriteFlashParams {
    pub offset: u32,
    /// Hex string of bytes to write (no separators, upper- or
    /// lower-case). Byte count = hex string length / 2.
    pub hex: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct WriteFlashResult {
    pub bytes_written: usize,
    pub offset: u32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct FlashPathParams {
    pub path: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct AddSymbolParams {
    pub address: u32,
    pub name: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct AddCommentParams {
    pub address: u32,
    pub comment: String,
}

/// Dispatcher-style tagged union for `inject_peripheral_event`.
/// Phase 5a covers the variants the test harness needs; the full
/// `PeripheralEvent` surface is wired in phase 7 as the inspector
/// tabs land.
#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct InjectPeripheralEventParams {
    /// Peripheral to target. One of: `uart`, `alarm`, `serdes`,
    /// `fatal_filter`, `pbc`.
    pub peripheral: String,
    /// Event variant name, matching `PeripheralEvent` sub-enums
    /// (e.g. `ForcePending`, `InjectRxLos`, `ClearTxLog`).
    pub event: String,
    /// Variant-specific parameters encoded as a free-form JSON
    /// object. Fields consumed per (peripheral, event) pair —
    /// see the error message on a bad call for supported keys.
    #[serde(default)]
    pub params: serde_json::Value,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MmioTraceEntryJson {
    pub address: u32,
    pub peripheral: String,
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: u32,
    pub last_write_value: u32,
    pub first_pc: u32,
    pub first_insn: u64,
    pub access_widths: u8,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DumpMmioTraceResult {
    pub enabled: bool,
    pub entries: Vec<MmioTraceEntryJson>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReadFlashResult {
    pub offset: u32,
    pub length: u32,
    /// Hex-encoded bytes. Upper-case, no separators.
    pub hex: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PeripheralEntry {
    pub index: usize,
    pub name: String,
    /// Debug-format dump of the peripheral snapshot. Phase 4
    /// ships this as a single string because `PeripheralSnapshot`
    /// does not yet derive `Serialize`; phases 5+ wire proper
    /// JSON projection as peripherals get their inspector tabs.
    pub debug: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ListPeripheralsResult {
    pub peripherals: Vec<PeripheralEntry>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct UartBufferResult {
    pub bytes_len: usize,
    pub ascii: String,
    pub hex: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct BreakpointsResult {
    pub breakpoints: Vec<u32>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SymbolsResult {
    pub symbols: HashMap<String, String>,
}

// ---------- Tool implementations -----------------------------------------

#[tool_router]
impl EmulatorHandler {
    #[tool(
        name = "get_firmware_info",
        description = "Return metadata about the currently loaded firmware, or `loaded=false` if nothing has been loaded yet."
    )]
    async fn get_firmware_info(&self) -> Json<FirmwareInfoResult> {
        let guard = self.handle.firmware_info.lock();
        let out = match guard.as_ref() {
            None => FirmwareInfoResult::default(),
            Some(info) => FirmwareInfoResult {
                loaded: true,
                path: Some(info.path.display().to_string()),
                boot_mode: Some(format!("{:?}", info.boot_mode)),
                entry_point: Some(info.entry_point),
                flash_size: Some(info.flash_size),
                flash_loaded: Some(info.flash_loaded),
            },
        };
        Json(out)
    }

    #[tool(
        name = "read_registers",
        description = "Read one or more CPU registers from the latest emulator snapshot. Supports r0..r63, pc, sp, fp, gp, blink, lp_count, ilink1, ilink2, status32."
    )]
    async fn read_registers(
        &self,
        Parameters(params): Parameters<ReadRegistersParams>,
    ) -> Json<ReadRegistersResult> {
        let snap = self.handle.snapshot.lock().clone();
        let mut out = HashMap::new();
        match params.names {
            Some(names) => {
                for n in names {
                    if let Some(v) = reg_by_name(&snap.cpu, &n) {
                        out.insert(n, v);
                    }
                }
            }
            None => {
                for i in 0..32usize {
                    out.insert(format!("r{}", i), snap.cpu.core_regs[i]);
                }
                out.insert("pc".into(), snap.cpu.pc);
                out.insert("sp".into(), snap.cpu.core_regs[28]);
                out.insert("fp".into(), snap.cpu.core_regs[27]);
                out.insert("gp".into(), snap.cpu.core_regs[26]);
                out.insert("blink".into(), snap.cpu.core_regs[31]);
                out.insert("lp_count".into(), snap.cpu.core_regs[60]);
                out.insert("status32".into(), snap.cpu.flags.status32);
                out.insert("ienable".into(), snap.cpu.aux.ienable);
                out.insert("ipending".into(), snap.cpu.aux.ipending);
            }
        }
        Json(ReadRegistersResult { values: out })
    }

    #[tool(
        name = "read_flags",
        description = "Return the decomposed STATUS32 flag set (Z, N, C, V, E1, E2, U, H, L, DE) plus the raw status32 value."
    )]
    async fn read_flags(&self) -> Json<FlagsResult> {
        let snap = self.handle.snapshot.lock();
        let f = snap.cpu.flags;
        Json(FlagsResult {
            z: f.z,
            n: f.n,
            c: f.c,
            v: f.v,
            e1: f.e1,
            e2: f.e2,
            u: f.u,
            h: f.h,
            l: f.l,
            de: f.de,
            status32: f.status32,
        })
    }

    #[tool(
        name = "get_cpu_state",
        description = "Return the headline CPU state: PC, run state, pause reason, instruction counter, halted / sleeping / paused flags."
    )]
    async fn get_cpu_state(&self) -> Json<CpuStateResult> {
        let snap = self.handle.snapshot.lock();
        Json(CpuStateResult {
            pc: snap.cpu.pc,
            halted: snap.cpu.halted,
            sleeping: snap.cpu.sleeping,
            paused: snap.cpu.paused,
            instruction_count: snap.cpu.instruction_count,
            run_state: format!("{:?}", snap.run_state),
            pause_reason: format!("{:?}", snap.pause_reason),
        })
    }

    #[tool(
        name = "list_peripherals",
        description = "Return every peripheral snapshot published in the most recent emulator frame. Debug-format string per entry in phase 4; proper JSON projections land in phases 5+."
    )]
    async fn list_peripherals(&self) -> Json<ListPeripheralsResult> {
        let snap = self.handle.snapshot.lock();
        let peripherals = snap
            .peripherals
            .iter()
            .enumerate()
            .map(|(index, p)| PeripheralEntry {
                index,
                name: p.name().to_string(),
                debug: format!("{:?}", p),
            })
            .collect();
        Json(ListPeripheralsResult { peripherals })
    }

    #[tool(
        name = "peek_mmio",
        description = "Side-effect-free MMIO word probe. Returns the current value of an MMIO register without triggering FIFO pops, IRQ latch clears, or busy-bit transitions. Use `read_mmio` (phase 5) for side-effectful reads."
    )]
    async fn peek_mmio(
        &self,
        Parameters(params): Parameters<PeekMmioParams>,
    ) -> Json<PeekMmioResult> {
        let value = self
            .handle
            .bank
            .read()
            .peek_word(params.address)
            .unwrap_or(0);
        Json(PeekMmioResult {
            address: params.address,
            value,
        })
    }

    #[tool(
        name = "list_breakpoints",
        description = "Return every active CPU breakpoint address installed via `set_breakpoint` (phase 5)."
    )]
    async fn list_breakpoints(&self) -> Json<BreakpointsResult> {
        let snap = self.handle.snapshot.lock();
        Json(BreakpointsResult {
            breakpoints: snap.breakpoints.clone(),
        })
    }

    #[tool(
        name = "list_symbols",
        description = "Return the user-loaded symbol map (address → name). The emulator ships empty; symbols are loaded at runtime via `add_symbol` (phase 5)."
    )]
    async fn list_symbols(&self) -> Json<SymbolsResult> {
        let ann = self.handle.annotations.read();
        let symbols = ann
            .symbols
            .iter()
            .map(|(addr, name)| (format!("0x{:08X}", addr), name.clone()))
            .collect();
        Json(SymbolsResult { symbols })
    }

    #[tool(
        name = "get_uart_buffer",
        description = "Return the UART TX log (everything the firmware has printed since boot). Decoded as lossy UTF-8 plus a hex dump."
    )]
    async fn get_uart_buffer(&self) -> Json<UartBufferResult> {
        let bytes = self.handle.bank.read().uart.tx_log_bytes();
        let ascii = String::from_utf8_lossy(&bytes).to_string();
        let hex = bytes
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<String>();
        Json(UartBufferResult {
            bytes_len: bytes.len(),
            ascii,
            hex,
        })
    }

    #[tool(
        name = "read_flash",
        description = "Read `length` bytes from offset `offset` of the 4 MB SPI flash image. Returns a hex-encoded dump."
    )]
    async fn read_flash(
        &self,
        Parameters(params): Parameters<ReadFlashParams>,
    ) -> Json<ReadFlashResult> {
        let guard = self.handle.bank.read();
        let flash = &guard.pbc.flash.data;
        let start = params.offset as usize;
        let end = start.saturating_add(params.length as usize).min(flash.len());
        let slice = if start < flash.len() {
            &flash[start..end]
        } else {
            &[][..]
        };
        let hex = slice
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<String>();
        Json(ReadFlashResult {
            offset: params.offset,
            length: slice.len() as u32,
            hex,
        })
    }

    // ---------- Phase 5a mutation tools ----------------------------------

    #[tool(
        name = "write_mmio",
        description = "Side-effectful MMIO write. Goes through the normal peripheral routing path — FIFO pushes, command-bit auto-clears, DMA triggers all fire. `width` is `byte` / `half` / `word` (default)."
    )]
    async fn write_mmio(
        &self,
        Parameters(params): Parameters<WriteMmioParams>,
    ) -> Json<OkResult> {
        let mut guard = self.handle.bank.write();
        let width = params
            .width
            .as_deref()
            .unwrap_or("word")
            .to_ascii_lowercase();
        let _ = match width.as_str() {
            "byte" => guard.write_byte(params.address, params.value as u8),
            "half" => guard.write_half(params.address, params.value as u16),
            _ => guard.write_word(params.address, params.value),
        };
        Json(OkResult { ok: true })
    }

    #[tool(
        name = "send_uart_input",
        description = "Push bytes into the bank's UART RX mpsc channel — the same path stdin uses in the headless entry point. Each code point is sent byte-by-byte so the CPU's UART IRQ path sees them as normal RX traffic."
    )]
    async fn send_uart_input(
        &self,
        Parameters(params): Parameters<SendUartInputParams>,
    ) -> Json<SendUartInputResult> {
        let mut sent = 0usize;
        for b in params.data.bytes() {
            if self.handle.uart_tx.send(b).is_ok() {
                sent += 1;
            } else {
                break;
            }
        }
        Json(SendUartInputResult { bytes_sent: sent })
    }

    #[tool(
        name = "write_flash",
        description = "Directly write raw bytes into the PBC flash backing store. Bypasses the SPI controller — intended for test harness use. Input is a hex string (no separators)."
    )]
    async fn write_flash(
        &self,
        Parameters(params): Parameters<WriteFlashParams>,
    ) -> Json<WriteFlashResult> {
        let bytes = match decode_hex(&params.hex) {
            Ok(b) => b,
            Err(_) => {
                return Json(WriteFlashResult {
                    bytes_written: 0,
                    offset: params.offset,
                });
            }
        };
        let mut guard = self.handle.bank.write();
        let flash = &mut guard.pbc.flash.data;
        let start = params.offset as usize;
        let end = (start + bytes.len()).min(flash.len());
        let written = end.saturating_sub(start);
        if written > 0 {
            flash[start..end].copy_from_slice(&bytes[..written]);
            guard.pbc.flash.dirty = true;
        }
        Json(WriteFlashResult {
            bytes_written: written,
            offset: params.offset,
        })
    }

    #[tool(
        name = "load_flash_from_file",
        description = "Replace the PBC flash image with the contents of a host-side file. Convenience wrapper around `PbcEvent::LoadFlashFromFile`."
    )]
    async fn load_flash_from_file(
        &self,
        Parameters(params): Parameters<FlashPathParams>,
    ) -> Json<OkResult> {
        let event = PeripheralEvent::Pbc(PbcEvent::LoadFlashFromFile(params.path.into()));
        let ok = self.handle.bank.write().inject_event(&event);
        Json(OkResult { ok })
    }

    #[tool(
        name = "dump_flash_to_file",
        description = "Write the current PBC flash image out to a host-side file. Convenience wrapper around `PbcEvent::DumpFlashToFile`."
    )]
    async fn dump_flash_to_file(
        &self,
        Parameters(params): Parameters<FlashPathParams>,
    ) -> Json<OkResult> {
        let event = PeripheralEvent::Pbc(PbcEvent::DumpFlashToFile(params.path.into()));
        let ok = self.handle.bank.write().inject_event(&event);
        Json(OkResult { ok })
    }

    #[tool(
        name = "add_symbol",
        description = "Add an address → name binding to the user symbol table. The emulator ships empty; this is the only way symbols get into the disassembly output."
    )]
    async fn add_symbol(
        &self,
        Parameters(params): Parameters<AddSymbolParams>,
    ) -> Json<OkResult> {
        let mut ann = self.handle.annotations.write();
        ann.symbols.insert(params.address, params.name);
        Json(OkResult { ok: true })
    }

    #[tool(
        name = "add_comment",
        description = "Attach a free-form comment to an address. Rendered as a trailing `; comment` in the disassembly panel (phase 6)."
    )]
    async fn add_comment(
        &self,
        Parameters(params): Parameters<AddCommentParams>,
    ) -> Json<OkResult> {
        let mut ann = self.handle.annotations.write();
        ann.comments.insert(params.address, params.comment);
        Json(OkResult { ok: true })
    }

    #[tool(
        name = "inject_peripheral_event",
        description = "Dispatch a typed `PeripheralEvent` through `bank.inject_event()`. Supports a phase-5a subset covering common test-harness paths: alarm/ForcePending, serdes/InjectRxLos, uart/ClearTxLog, fatal_filter/InjectFatal. Phase 7 expands to every variant."
    )]
    async fn inject_peripheral_event(
        &self,
        Parameters(params): Parameters<InjectPeripheralEventParams>,
    ) -> Json<OkResult> {
        let event = match build_peripheral_event(&params) {
            Some(ev) => ev,
            None => return Json(OkResult { ok: false }),
        };
        let ok = self.handle.bank.write().inject_event(&event);
        Json(OkResult { ok })
    }

    #[tool(
        name = "dump_mmio_trace",
        description = "Return the aggregated MMIO trace catalog. Only populated when the emulator was started with `--dump-mmio-trace`; otherwise `enabled=false` and an empty entries array."
    )]
    async fn dump_mmio_trace(&self) -> Json<DumpMmioTraceResult> {
        let guard = self.handle.bank.read();
        match guard.mmio_trace.as_ref() {
            None => Json(DumpMmioTraceResult {
                enabled: false,
                entries: Vec::new(),
            }),
            Some(map) => {
                let mut entries: Vec<_> = map
                    .iter()
                    .map(|(addr, e)| MmioTraceEntryJson {
                        address: *addr,
                        peripheral: e.peripheral.to_string(),
                        reads: e.reads,
                        writes: e.writes,
                        last_read_value: e.last_read_value,
                        last_write_value: e.last_write_value,
                        first_pc: e.first_pc,
                        first_insn: e.first_insn,
                        access_widths: e.access_widths,
                    })
                    .collect();
                entries.sort_by_key(|e| e.address);
                Json(DumpMmioTraceResult {
                    enabled: true,
                    entries,
                })
            }
        }
    }
}

// ---------- ServerHandler impl -------------------------------------------

#[tool_handler]
impl ServerHandler for EmulatorHandler {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build())
            .with_protocol_version(ProtocolVersion::V_2025_06_18)
            .with_instructions(
                "ARC700 / BCM55030 emulator — read-only MCP tools. Mutation tools land in phase 5.",
            )
    }
}

// ---------- Helpers -------------------------------------------------------

fn reg_by_name(cpu: &crate::emu::snapshot::CpuSnapshot, name: &str) -> Option<u32> {
    let lower = name.to_ascii_lowercase();
    match lower.as_str() {
        "pc" => Some(cpu.pc),
        "status32" => Some(cpu.flags.status32),
        "sp" => Some(cpu.core_regs[28]),
        "fp" => Some(cpu.core_regs[27]),
        "gp" => Some(cpu.core_regs[26]),
        "blink" => Some(cpu.core_regs[31]),
        "lp_count" | "lpcount" => Some(cpu.core_regs[60]),
        "ilink1" => Some(cpu.core_regs[29]),
        "ilink2" => Some(cpu.core_regs[30]),
        "ienable" => Some(cpu.aux.ienable),
        "ipending" => Some(cpu.aux.ipending),
        "identity" => Some(cpu.aux.identity),
        _ => {
            if let Some(rest) = lower.strip_prefix('r') {
                if let Ok(idx) = rest.parse::<usize>() {
                    if idx < 64 {
                        return Some(cpu.core_regs[idx]);
                    }
                }
            }
            None
        }
    }
}

// Keep the import live — the formatter uses `RunState` via
// `format!("{:?}", …)` above, but the explicit use ensures the
// re-export path in the emu module stays exercised.
#[allow(dead_code)]
fn _run_state_guard(r: RunState) {
    let _ = r;
}

// Suppress false-positive unused if `PeripheralSnapshot` ever
// gains explicit imports here.
#[allow(dead_code)]
fn _peripheral_guard(p: PeripheralSnapshot) {
    let _ = p;
}

fn decode_hex(s: &str) -> Result<Vec<u8>, ()> {
    let clean: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if clean.len() % 2 != 0 {
        return Err(());
    }
    let mut out = Vec::with_capacity(clean.len() / 2);
    for chunk in clean.as_bytes().chunks(2) {
        let hi = hex_nibble(chunk[0])?;
        let lo = hex_nibble(chunk[1])?;
        out.push((hi << 4) | lo);
    }
    Ok(out)
}

fn hex_nibble(c: u8) -> Result<u8, ()> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(()),
    }
}

/// Translate the flat JSON envelope into a real `PeripheralEvent`.
/// Phase 5a supports the minimum set the test harness drives;
/// phase 7 expands to every variant as UI inspector tabs land.
fn build_peripheral_event(p: &InjectPeripheralEventParams) -> Option<PeripheralEvent> {
    let obj = p.params.as_object();
    match (p.peripheral.as_str(), p.event.as_str()) {
        ("alarm", "ForcePending") => {
            let opcode = obj
                .and_then(|m| m.get("opcode"))
                .and_then(|v| v.as_u64())?;
            Some(PeripheralEvent::Alarm(AlarmEvent::ForcePending(
                opcode as u16,
            )))
        }
        ("alarm", "ClearPending") => {
            let opcode = obj
                .and_then(|m| m.get("opcode"))
                .and_then(|v| v.as_u64())?;
            Some(PeripheralEvent::Alarm(AlarmEvent::ClearPending(
                opcode as u16,
            )))
        }
        ("alarm", "ClearAll") => Some(PeripheralEvent::Alarm(AlarmEvent::ClearAll)),
        ("serdes", "InjectRxLos") => {
            let lane = obj.and_then(|m| m.get("lane")).and_then(|v| v.as_u64())?;
            let state = obj
                .and_then(|m| m.get("state"))
                .and_then(|v| v.as_bool())?;
            Some(PeripheralEvent::SerDes(SerDesEvent::InjectRxLos(
                lane as u8, state,
            )))
        }
        ("uart", "ClearTxLog") => Some(PeripheralEvent::Uart(UartEvent::ClearTxLog)),
        ("fatal_filter", "InjectFatal") => {
            let mask = obj.and_then(|m| m.get("mask")).and_then(|v| v.as_u64())?;
            Some(PeripheralEvent::FatalFilter(FatalFilterEvent::InjectFatal(
                mask as u32,
            )))
        }
        ("fatal_filter", "ClearFatal") => {
            Some(PeripheralEvent::FatalFilter(FatalFilterEvent::ClearFatal))
        }
        _ => None,
    }
}
