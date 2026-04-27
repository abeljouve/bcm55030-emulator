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
use std::time::Duration;

use rmcp::handler::server::tool::ToolRouter;
use rmcp::handler::server::wrapper::{Json, Parameters};
use rmcp::model::{
    ProtocolVersion, ServerCapabilities, ServerInfo,
};
use rmcp::{tool, tool_handler, tool_router, ServerHandler};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::emu::command::{oneshot, CpuCommand};
use crate::emu::{EmulatorHandle, RunState};
use crate::memory::WatchMode;
use crate::soc::bank::BootMode;
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

// ---------- Scenario override DTOs (Phase 1) -----------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetMmioOverrideParams {
    /// MMIO address (word-aligned).
    pub address: u32,
    /// Value to return on read (for `static` / `oneshot` modes) or
    /// bitmask of bits to suppress on write (`mask` mode).
    pub value: u32,
    /// `"static"` (default), `"oneshot"`, or `"mask"`.
    #[serde(default = "default_override_mode")]
    pub mode: String,
    /// For `oneshot` mode: number of reads before the override expires.
    #[serde(default = "default_oneshot_count")]
    pub count: u32,
    /// Optional human-readable label shown in `list_mmio_overrides`.
    pub label: Option<String>,
}

fn default_override_mode() -> String {
    "static".to_string()
}

fn default_oneshot_count() -> u32 {
    1
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct RemoveMmioOverrideParams {
    pub address: u32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MmioOverrideEntry {
    pub address: u32,
    pub mode: String,
    pub value: u32,
    pub remaining: Option<u32>,
    pub label: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ListMmioOverridesResult {
    pub count: usize,
    pub overrides: Vec<MmioOverrideEntry>,
}

// ---------- Scenario event DTOs (Phase 2) --------------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ScheduleEventParams {
    /// Trigger: `{"type": "at_instruction", "n": 1000}` or
    /// `{"type": "on_mmio_read", "address": "0x0100240C", "occurrence": 1}` or
    /// `{"type": "on_mmio_write", "address": "0x01000050", "occurrence": 3}`.
    pub trigger: serde_json::Value,
    /// Effect: `{"type": "set_override", "address": "0x...", "value": "0x...", "mode": "static"}` or
    /// `{"type": "remove_override", "address": "0x..."}` or
    /// `{"type": "write_mmio", "address": "0x...", "value": "0x..."}` or
    /// `{"type": "pause"}`.
    pub effect: serde_json::Value,
    pub label: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ScheduleEventResult {
    pub ok: bool,
    pub id: Option<u32>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct CancelEventParams {
    pub id: u32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ScheduledEventEntry {
    pub id: u32,
    pub trigger: String,
    pub effect: String,
    pub label: Option<String>,
    pub fired: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ListScheduledEventsResult {
    pub count: usize,
    pub events: Vec<ScheduledEventEntry>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetMmioWatchpointParams {
    pub address: u32,
    /// Size in bytes (default 4 = one word).
    #[serde(default = "default_wp_size")]
    pub size: u32,
    /// `"read"`, `"write"`, or `"rw"` (default).
    #[serde(default = "default_wp_mode")]
    pub mode: String,
    /// `"pause"` (default) or an effect JSON object.
    #[serde(default)]
    pub action: serde_json::Value,
    pub label: Option<String>,
}

fn default_wp_size() -> u32 { 4 }
fn default_wp_mode() -> String { "rw".to_string() }

#[derive(Debug, Serialize, JsonSchema)]
pub struct SetMmioWatchpointResult {
    pub ok: bool,
    pub id: Option<u32>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct RemoveMmioWatchpointParams {
    pub id: u32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MmioWatchpointEntry {
    pub id: u32,
    pub address: u32,
    pub size: u32,
    pub mode: String,
    pub action: String,
    pub label: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ListMmioWatchpointsResult {
    pub count: usize,
    pub watchpoints: Vec<MmioWatchpointEntry>,
}

// ---------- Scenario file DTOs (Phase 3) ---------------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct LoadScenarioParams {
    /// Raw JSON string of the scenario file content.
    pub json: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LoadScenarioResult {
    pub ok: bool,
    pub loaded: usize,
    pub error: Option<String>,
}

// ---------- Phase 5b CpuCommand DTOs -------------------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema, Default)]
pub struct CpuRunParams {
    /// Optional hard cap on the number of instructions to
    /// execute before the worker auto-pauses with
    /// `PauseReason::UserPause`. `None` = unbounded run.
    pub max_insns: Option<u64>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema, Default)]
pub struct CpuStepParams {
    /// Defaults to 1 when absent.
    pub count: Option<u32>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct CpuRunToParams {
    pub address: u32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct CpuResetParams {
    /// `"cold"` or `"warm"` (default).
    pub boot_mode: Option<String>,
    #[serde(default = "default_true")]
    pub keep_breakpoints: bool,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct LoadFirmwareParams {
    /// Host-side path to the flash image (bootloader or full dump).
    pub path: String,
    /// `"cold"` or `"warm"` (default).
    pub boot_mode: Option<String>,
    /// Reset vector / PC after load. Defaults to `0x0`.
    pub entry_point: Option<u32>,
    #[serde(default = "default_true")]
    pub keep_breakpoints: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LoadFirmwareResponse {
    pub ok: bool,
    pub loaded_bytes: usize,
    pub entry_point: u32,
    pub flash_bytes: usize,
    pub error: Option<String>,
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetBreakpointParams {
    pub address: u32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct RemoveBreakpointParams {
    pub address: u32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetWatchpointParams {
    pub address: u32,
    pub size: u32,
    /// `"read"`, `"write"`, or `"rw"`.
    pub mode: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct RemoveWatchpointParams {
    pub index: u32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct WriteRegisterParams {
    pub name: String,
    pub value: u32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct WriteMemoryParams {
    pub address: u32,
    /// Hex-encoded bytes.
    pub hex: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ReadMemoryParams {
    pub address: u32,
    pub length: u32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReadMemoryResult {
    pub address: u32,
    pub length: u32,
    pub hex: String,
    pub ascii: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ReadMmioParams {
    pub address: u32,
    pub width: Option<String>,
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

    // ---------- Scenario override tools (Phase 1) -----------------------

    #[tool(
        name = "set_mmio_override",
        description = "Install an MMIO read/write override. Models external HW stimulus (e.g. 'the PHY drove this value on the bus'). Modes: 'static' (return `value` on every read), 'oneshot' (return `value` for `count` reads then expire), 'mask' (zero out `value` bits on writes). Override is checked BEFORE peripheral dispatch."
    )]
    async fn set_mmio_override(
        &self,
        Parameters(params): Parameters<SetMmioOverrideParams>,
    ) -> Json<OkResult> {
        use crate::soc::scenario::OverrideSpec;
        let spec = match params.mode.as_str() {
            "static" => OverrideSpec::StaticRead { value: params.value },
            "oneshot" => OverrideSpec::OneShotRead {
                value: params.value,
                remaining: params.count,
            },
            "mask" => OverrideSpec::MaskedWriteIgnore { mask: params.value },
            _ => return Json(OkResult { ok: false }),
        };
        self.handle.bank.write().scenario.overrides.set(params.address, spec, params.label);
        Json(OkResult { ok: true })
    }

    #[tool(
        name = "remove_mmio_override",
        description = "Remove a previously installed MMIO override by address."
    )]
    async fn remove_mmio_override(
        &self,
        Parameters(params): Parameters<RemoveMmioOverrideParams>,
    ) -> Json<OkResult> {
        let ok = self.handle.bank.write().scenario.overrides.remove(params.address);
        Json(OkResult { ok })
    }

    #[tool(
        name = "list_mmio_overrides",
        description = "List all active MMIO overrides."
    )]
    async fn list_mmio_overrides(&self) -> Json<ListMmioOverridesResult> {
        let guard = self.handle.bank.read();
        let mut overrides: Vec<MmioOverrideEntry> = guard
            .scenario
            .overrides
            .iter()
            .map(|(addr, ov)| {
                use crate::soc::scenario::OverrideSpec;
                let (mode, value, remaining) = match &ov.spec {
                    OverrideSpec::StaticRead { value } => ("static", *value, None),
                    OverrideSpec::OneShotRead { value, remaining } => {
                        ("oneshot", *value, Some(*remaining))
                    }
                    OverrideSpec::MaskedWriteIgnore { mask } => ("mask", *mask, None),
                };
                MmioOverrideEntry {
                    address: addr,
                    mode: mode.to_string(),
                    value,
                    remaining,
                    label: ov.label.clone(),
                }
            })
            .collect();
        overrides.sort_by_key(|e| e.address);
        Json(ListMmioOverridesResult {
            count: overrides.len(),
            overrides,
        })
    }

    // ---------- Scenario event + watchpoint tools (Phase 2) -------------

    #[tool(
        name = "schedule_event",
        description = "Schedule a scenario event with a trigger and effect. Returns the event ID. Trigger types: 'at_instruction' (fire at instruction N), 'on_mmio_read' (fire on N-th read), 'on_mmio_write' (fire on N-th write). Effects: 'set_override', 'remove_override', 'write_mmio', 'pause'."
    )]
    async fn schedule_event(
        &self,
        Parameters(params): Parameters<ScheduleEventParams>,
    ) -> Json<ScheduleEventResult> {
        let trigger = match parse_trigger(&params.trigger) {
            Some(t) => t,
            None => return Json(ScheduleEventResult { ok: false, id: None }),
        };
        let effect = match parse_effect(&params.effect) {
            Some(e) => e,
            None => return Json(ScheduleEventResult { ok: false, id: None }),
        };
        let id = self.handle.bank.write().scenario.schedule(trigger, effect, params.label);
        Json(ScheduleEventResult { ok: true, id: Some(id) })
    }

    #[tool(
        name = "cancel_event",
        description = "Cancel a scheduled event by ID."
    )]
    async fn cancel_event(
        &self,
        Parameters(params): Parameters<CancelEventParams>,
    ) -> Json<OkResult> {
        let ok = self.handle.bank.write().scenario.cancel(params.id);
        Json(OkResult { ok })
    }

    #[tool(
        name = "list_scheduled_events",
        description = "List all scheduled events (pending and fired)."
    )]
    async fn list_scheduled_events(&self) -> Json<ListScheduledEventsResult> {
        let guard = self.handle.bank.read();
        let events: Vec<ScheduledEventEntry> = guard
            .scenario
            .pending_events()
            .map(|e| ScheduledEventEntry {
                id: e.id,
                trigger: format!("{:?}", e.trigger),
                effect: format!("{:?}", e.effect),
                label: e.label.clone(),
                fired: e.fired,
            })
            .collect();
        Json(ListScheduledEventsResult {
            count: events.len(),
            events,
        })
    }

    #[tool(
        name = "set_mmio_watchpoint",
        description = "Install an MMIO watchpoint. Fires on read/write to the address range. Default action: pause. Can also fire a scenario effect."
    )]
    async fn set_mmio_watchpoint(
        &self,
        Parameters(params): Parameters<SetMmioWatchpointParams>,
    ) -> Json<SetMmioWatchpointResult> {
        use crate::soc::scenario::{MmioWatchAction, MmioWatchMode};
        let mode = match params.mode.as_str() {
            "read" | "r" => MmioWatchMode::Read,
            "write" | "w" => MmioWatchMode::Write,
            "rw" | "readwrite" => MmioWatchMode::ReadWrite,
            _ => return Json(SetMmioWatchpointResult { ok: false, id: None }),
        };
        let action = if params.action.is_null() || params.action.as_str() == Some("pause") {
            MmioWatchAction::Pause
        } else {
            match parse_effect(&params.action) {
                Some(e) => MmioWatchAction::Fire(e),
                None => return Json(SetMmioWatchpointResult { ok: false, id: None }),
            }
        };
        let id = self.handle.bank.write().scenario.add_watchpoint(
            params.address,
            params.size,
            mode,
            action,
            params.label,
        );
        Json(SetMmioWatchpointResult { ok: true, id: Some(id) })
    }

    #[tool(
        name = "remove_mmio_watchpoint",
        description = "Remove an MMIO watchpoint by ID."
    )]
    async fn remove_mmio_watchpoint(
        &self,
        Parameters(params): Parameters<RemoveMmioWatchpointParams>,
    ) -> Json<OkResult> {
        let ok = self.handle.bank.write().scenario.remove_watchpoint(params.id);
        Json(OkResult { ok })
    }

    #[tool(
        name = "list_mmio_watchpoints",
        description = "List all active MMIO watchpoints."
    )]
    async fn list_mmio_watchpoints(&self) -> Json<ListMmioWatchpointsResult> {
        let guard = self.handle.bank.read();
        let watchpoints: Vec<MmioWatchpointEntry> = guard
            .scenario
            .watchpoints()
            .iter()
            .map(|wp| {
                use crate::soc::scenario::MmioWatchMode;
                let mode = match wp.mode {
                    MmioWatchMode::Read => "read",
                    MmioWatchMode::Write => "write",
                    MmioWatchMode::ReadWrite => "rw",
                };
                MmioWatchpointEntry {
                    id: wp.id,
                    address: wp.address,
                    size: wp.size,
                    mode: mode.to_string(),
                    action: format!("{:?}", wp.action),
                    label: wp.label.clone(),
                }
            })
            .collect();
        Json(ListMmioWatchpointsResult {
            count: watchpoints.len(),
            watchpoints,
        })
    }

    // ---------- Scenario file tools (Phase 3) ----------------------------

    #[tool(
        name = "load_scenario",
        description = "Load a JSON scenario file. Each entry in the 'events' array is an MCP tool call (set_mmio_override, schedule_event). Returns the number of entries loaded."
    )]
    async fn load_scenario(
        &self,
        Parameters(params): Parameters<LoadScenarioParams>,
    ) -> Json<LoadScenarioResult> {
        let mut guard = self.handle.bank.write();
        match guard.scenario.load_json(&params.json) {
            Ok(n) => Json(LoadScenarioResult { ok: true, loaded: n, error: None }),
            Err(e) => Json(LoadScenarioResult { ok: false, loaded: 0, error: Some(e) }),
        }
    }

    #[tool(
        name = "clear_scenario",
        description = "Remove all overrides, scheduled events, and MMIO watchpoints."
    )]
    async fn clear_scenario(&self) -> Json<OkResult> {
        self.handle.bank.write().scenario.clear_all();
        Json(OkResult { ok: true })
    }

    // ---------- MMIO trace tools -----------------------------------------

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

    // ---------- Phase 5b CpuCommand-backed tools ------------------------

    #[tool(
        name = "cpu_run",
        description = "Start continuous CPU execution. `max_insns` caps the run length and auto-pauses via `PauseReason::UserPause` when reached. Fire-and-forget: poll `get_cpu_state` for completion."
    )]
    async fn cpu_run(
        &self,
        Parameters(params): Parameters<CpuRunParams>,
    ) -> Json<OkResult> {
        let ok = self
            .handle
            .cpu_cmd
            .send(CpuCommand::Run {
                max_insns: params.max_insns,
            })
            .is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "cpu_pause",
        description = "Pause a running CPU. The worker publishes a Paused snapshot within one instruction."
    )]
    async fn cpu_pause(&self) -> Json<OkResult> {
        let ok = self.handle.cpu_cmd.send(CpuCommand::Pause).is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "cpu_step",
        description = "Execute `count` instructions (default 1) and auto-pause. Poll `get_cpu_state` for completion."
    )]
    async fn cpu_step(
        &self,
        Parameters(params): Parameters<CpuStepParams>,
    ) -> Json<OkResult> {
        let n = params.count.unwrap_or(1);
        let ok = self.handle.cpu_cmd.send(CpuCommand::StepN(n)).is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "cpu_step_over",
        description = "Execute until the instruction at the current `blink` is reached (skip through BL/JL function calls). Implemented by installing a temporary breakpoint at blink and running."
    )]
    async fn cpu_step_over(&self) -> Json<OkResult> {
        let ok = self.handle.cpu_cmd.send(CpuCommand::StepOver).is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "cpu_run_to",
        description = "Run until the CPU reaches `address`, then auto-pause with `PauseReason::UserPause`."
    )]
    async fn cpu_run_to(
        &self,
        Parameters(params): Parameters<CpuRunToParams>,
    ) -> Json<OkResult> {
        let ok = self
            .handle
            .cpu_cmd
            .send(CpuCommand::RunTo {
                address: params.address,
            })
            .is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "cpu_reset",
        description = "Rebuild the CPU via the worker's reset callback. `boot_mode` is `cold` or `warm` (default). `keep_breakpoints=true` re-installs every current breakpoint on the fresh CPU."
    )]
    async fn cpu_reset(
        &self,
        Parameters(params): Parameters<CpuResetParams>,
    ) -> Json<OkResult> {
        let mode = match params
            .boot_mode
            .as_deref()
            .map(|s| s.to_ascii_lowercase())
            .as_deref()
        {
            Some("cold") => BootMode::Cold,
            _ => BootMode::Warm,
        };
        let ok = self
            .handle
            .cpu_cmd
            .send(CpuCommand::Reset {
                boot_mode: mode,
                keep_breakpoints: params.keep_breakpoints,
            })
            .is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "load_firmware",
        description = "Load a flash image (bootloader or full SPI dump) into the PBC SPI flash, perform the 64 KB boot DMA flash → SRAM, and reset the CPU at the given entry point. Mirrors the `src/bin/arc700.rs` CLI boot flow and the UI drag-and-drop path. Requires SoC mode (not flat). `boot_mode` is `cold` or `warm` (default). `entry_point` defaults to 0."
    )]
    async fn load_firmware(
        &self,
        Parameters(params): Parameters<LoadFirmwareParams>,
    ) -> Json<LoadFirmwareResponse> {
        let mode = match params
            .boot_mode
            .as_deref()
            .map(|s| s.to_ascii_lowercase())
            .as_deref()
        {
            Some("cold") => BootMode::Cold,
            _ => BootMode::Warm,
        };
        let entry_point = params.entry_point.unwrap_or(0);
        let (tx, rx) = oneshot::<Result<crate::emu::command::LoadFirmwareResult, String>>();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::LoadFirmware {
                path: params.path.into(),
                mode: crate::emu::command::FirmwareMode::Soc,
                boot_mode: mode,
                flash_path: None,
                entry_point,
                keep_breakpoints: params.keep_breakpoints,
                response: tx,
            })
            .is_err()
        {
            return Json(LoadFirmwareResponse {
                ok: false,
                loaded_bytes: 0,
                entry_point,
                flash_bytes: 0,
                error: Some("cpu_cmd channel closed".into()),
            });
        }
        let result = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(10))
        })
        .await
        .unwrap_or(Err(std::sync::mpsc::RecvTimeoutError::Timeout));
        match result {
            Ok(Ok(r)) => Json(LoadFirmwareResponse {
                ok: true,
                loaded_bytes: r.loaded_bytes,
                entry_point: r.entry_point,
                flash_bytes: r.flash_bytes,
                error: None,
            }),
            Ok(Err(e)) => Json(LoadFirmwareResponse {
                ok: false,
                loaded_bytes: 0,
                entry_point,
                flash_bytes: 0,
                error: Some(e),
            }),
            Err(_) => Json(LoadFirmwareResponse {
                ok: false,
                loaded_bytes: 0,
                entry_point,
                flash_bytes: 0,
                error: Some("timeout waiting for worker".into()),
            }),
        }
    }

    #[tool(
        name = "set_breakpoint",
        description = "Install a CPU breakpoint at `address`. The worker inserts a `Hook::Breakpoint` entry and pauses before executing that PC. Returns the 0-based index in the breakpoint list."
    )]
    async fn set_breakpoint(
        &self,
        Parameters(params): Parameters<SetBreakpointParams>,
    ) -> Json<OkResult> {
        let (tx, rx) = oneshot::<usize>();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::SetBreakpoint {
                address: params.address,
                response: tx,
            })
            .is_err()
        {
            return Json(OkResult { ok: false });
        }
        let ok = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(2)).is_ok()
        })
        .await
        .unwrap_or(false);
        Json(OkResult { ok })
    }

    #[tool(
        name = "remove_breakpoint",
        description = "Remove a CPU breakpoint by address. No-op if none is installed there."
    )]
    async fn remove_breakpoint(
        &self,
        Parameters(params): Parameters<RemoveBreakpointParams>,
    ) -> Json<OkResult> {
        let ok = self
            .handle
            .cpu_cmd
            .send(CpuCommand::RemoveBreakpoint {
                address: params.address,
            })
            .is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "set_watchpoint",
        description = "Trap on memory access. `mode` is `read` / `write` / `rw`. The worker pauses with `PauseReason::Watch` on hit."
    )]
    async fn set_watchpoint(
        &self,
        Parameters(params): Parameters<SetWatchpointParams>,
    ) -> Json<OkResult> {
        let mode = match params.mode.to_ascii_lowercase().as_str() {
            "read" => WatchMode::Read,
            "write" => WatchMode::Write,
            _ => WatchMode::ReadWrite,
        };
        let (tx, rx) = oneshot::<usize>();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::SetWatchpoint {
                addr: params.address,
                size: params.size,
                mode,
                response: tx,
            })
            .is_err()
        {
            return Json(OkResult { ok: false });
        }
        let ok = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(2)).is_ok()
        })
        .await
        .unwrap_or(false);
        Json(OkResult { ok })
    }

    #[tool(
        name = "remove_watchpoint",
        description = "Remove a watchpoint by index into the installed-watchpoint list."
    )]
    async fn remove_watchpoint(
        &self,
        Parameters(params): Parameters<RemoveWatchpointParams>,
    ) -> Json<OkResult> {
        let ok = self
            .handle
            .cpu_cmd
            .send(CpuCommand::RemoveWatchpoint {
                index: params.index as usize,
            })
            .is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "write_register",
        description = "Write a CPU register by name. Supports r0..r63, pc, sp, fp, gp, blink, lp_count, status32, ilink1, ilink2."
    )]
    async fn write_register(
        &self,
        Parameters(params): Parameters<WriteRegisterParams>,
    ) -> Json<OkResult> {
        let (tx, rx) = oneshot::<Result<(), String>>();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::WriteRegister {
                name: params.name,
                value: params.value,
                response: tx,
            })
            .is_err()
        {
            return Json(OkResult { ok: false });
        }
        let ok = tokio::task::spawn_blocking(move || {
            matches!(rx.recv_timeout(Duration::from_secs(2)), Ok(Ok(())))
        })
        .await
        .unwrap_or(false);
        Json(OkResult { ok })
    }

    #[tool(
        name = "write_memory",
        description = "Write bytes to SRAM via the worker. Hex-encoded payload. Routes through `CpuCommand::WriteSram` so the CPU thread stays exclusive over `Memory`."
    )]
    async fn write_memory(
        &self,
        Parameters(params): Parameters<WriteMemoryParams>,
    ) -> Json<OkResult> {
        let bytes = match decode_hex(&params.hex) {
            Ok(b) => b,
            Err(_) => return Json(OkResult { ok: false }),
        };
        let (tx, rx) = oneshot::<Result<(), String>>();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::WriteSram {
                addr: params.address,
                bytes,
                response: tx,
            })
            .is_err()
        {
            return Json(OkResult { ok: false });
        }
        let ok = tokio::task::spawn_blocking(move || {
            matches!(rx.recv_timeout(Duration::from_secs(2)), Ok(Ok(())))
        })
        .await
        .unwrap_or(false);
        Json(OkResult { ok })
    }

    #[tool(
        name = "read_memory",
        description = "Read `length` bytes from SRAM via the worker's `RequestSram` round-trip. Returns hex + ASCII."
    )]
    async fn read_memory(
        &self,
        Parameters(params): Parameters<ReadMemoryParams>,
    ) -> Json<ReadMemoryResult> {
        let (tx, rx) = oneshot::<crate::emu::SramSnapshot>();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::RequestSram { response: tx })
            .is_err()
        {
            return Json(ReadMemoryResult {
                address: params.address,
                length: 0,
                hex: String::new(),
                ascii: String::new(),
            });
        }
        let snap = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(5)).ok()
        })
        .await
        .ok()
        .flatten();
        let Some(snap) = snap else {
            return Json(ReadMemoryResult {
                address: params.address,
                length: 0,
                hex: String::new(),
                ascii: String::new(),
            });
        };
        let start = params.address as usize;
        let end = start
            .saturating_add(params.length as usize)
            .min(snap.bytes.len());
        let slice = if start < snap.bytes.len() {
            &snap.bytes[start..end]
        } else {
            &[][..]
        };
        let hex = slice.iter().map(|b| format!("{:02X}", b)).collect::<String>();
        let ascii = slice
            .iter()
            .map(|b| if b.is_ascii_graphic() || *b == b' ' { *b as char } else { '.' })
            .collect::<String>();
        Json(ReadMemoryResult {
            address: params.address,
            length: slice.len() as u32,
            hex,
            ascii,
        })
    }

    #[tool(
        name = "read_mmio",
        description = "Side-effectful MMIO read. Unlike `peek_mmio` this triggers FIFO pops, IRQ latch clears, and busy-bit transitions. Use `peek_mmio` for the inspector."
    )]
    async fn read_mmio(
        &self,
        Parameters(params): Parameters<ReadMmioParams>,
    ) -> Json<PeekMmioResult> {
        let mut guard = self.handle.bank.write();
        let width = params
            .width
            .as_deref()
            .unwrap_or("word")
            .to_ascii_lowercase();
        let value = match width.as_str() {
            "byte" => guard.read_byte(params.address).unwrap_or(0) as u32,
            "half" => guard.read_half(params.address).unwrap_or(0) as u32,
            _ => guard.read_word(params.address).unwrap_or(0),
        };
        Json(PeekMmioResult {
            address: params.address,
            value,
        })
    }
}

// ---------- ServerHandler impl -------------------------------------------

#[tool_handler]
impl ServerHandler for EmulatorHandler {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build())
            .with_protocol_version(ProtocolVersion::V_2025_06_18)
            .with_instructions(
                "ARC700 / BCM55030 emulator — full MCP tool surface (read + mutation).",
            )
    }

    /// Custom `call_tool` override. `#[tool_handler]` only
    /// injects a default implementation when one is not already
    /// present, so providing our own keeps the macro happy while
    /// letting us tee every request / response pair into the
    /// `handle.event_log` ring — which powers the GUI's
    /// "MCP Activity" panel.
    async fn call_tool(
        &self,
        request: rmcp::model::CallToolRequestParams,
        context: rmcp::service::RequestContext<rmcp::RoleServer>,
    ) -> Result<rmcp::model::CallToolResult, rmcp::ErrorData> {
        let tool_name = request.name.to_string();
        let is_mutation = is_mutation_tool(&tool_name);
        let params_summary = request
            .arguments
            .as_ref()
            .map(|a| truncate(&serde_json::Value::Object(a.clone()).to_string(), 240))
            .unwrap_or_default();
        self.push_log_request(&tool_name, &params_summary, is_mutation);

        let tcc = rmcp::handler::server::tool::ToolCallContext::new(self, request, context);
        let result = Self::tool_router().call(tcc).await;

        let result_summary = match &result {
            Ok(r) => summarise_result(r),
            Err(e) => format!("error: {e:?}"),
        };
        self.push_log_response(&tool_name, &result_summary, is_mutation);
        result
    }
}

impl EmulatorHandler {
    fn push_log_request(&self, tool: &str, params: &str, is_mutation: bool) {
        use crate::emu::event_log::{Direction, EventEntry};
        use std::time::SystemTime;
        let mut log = self.handle.event_log.lock();
        log.in_flight = log.in_flight.saturating_add(1);
        log.push(EventEntry {
            timestamp: SystemTime::now(),
            direction: Direction::Request,
            tool: tool.to_string(),
            params: params.to_string(),
            result: String::new(),
            is_mutation,
        });
    }

    fn push_log_response(&self, tool: &str, result: &str, is_mutation: bool) {
        use crate::emu::event_log::{Direction, EventEntry};
        use std::time::SystemTime;
        let mut log = self.handle.event_log.lock();
        log.in_flight = log.in_flight.saturating_sub(1);
        log.push(EventEntry {
            timestamp: SystemTime::now(),
            direction: Direction::Response,
            tool: tool.to_string(),
            params: String::new(),
            result: result.to_string(),
            is_mutation,
        });
    }
}

/// Short-form summary of a `CallToolResult` for the activity
/// log. Prefers `structured_content` when present; falls back to
/// the first text content. Truncated to keep the ring lean.
fn summarise_result(r: &rmcp::model::CallToolResult) -> String {
    if let Some(v) = r.structured_content.as_ref() {
        return truncate(&v.to_string(), 240);
    }
    if let Some(first) = r.content.first() {
        if let Some(text) = first.as_text() {
            return truncate(&text.text, 240);
        }
    }
    if matches!(r.is_error, Some(true)) {
        "error".to_string()
    } else {
        "ok".to_string()
    }
}

fn truncate(s: &str, max: usize) -> String {
    if s.len() <= max {
        s.to_string()
    } else {
        let mut out = s[..max].to_string();
        out.push('…');
        out
    }
}

/// Whitelist of tools that mutate emulator state. Drives the
/// warning-coloured row style in the GUI activity log.
fn is_mutation_tool(name: &str) -> bool {
    matches!(
        name,
        "load_firmware"
            | "cpu_run"
            | "cpu_pause"
            | "cpu_step"
            | "cpu_step_over"
            | "cpu_run_to"
            | "cpu_reset"
            | "write_register"
            | "write_memory"
            | "write_mmio"
            | "read_mmio"
            | "set_breakpoint"
            | "remove_breakpoint"
            | "set_watchpoint"
            | "remove_watchpoint"
            | "inject_peripheral_event"
            | "send_uart_input"
            | "write_flash"
            | "load_flash_from_file"
            | "dump_flash_to_file"
            | "add_symbol"
            | "add_comment"
            | "set_mmio_override"
            | "remove_mmio_override"
            | "schedule_event"
            | "cancel_event"
            | "set_mmio_watchpoint"
            | "remove_mmio_watchpoint"
    )
}

// ---------- Helpers -------------------------------------------------------

use crate::soc::scenario::{parse_effect, parse_trigger};

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
