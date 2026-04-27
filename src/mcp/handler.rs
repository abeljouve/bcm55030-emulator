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

// ---------- HexU32 — accepts both integer and hex-string in MCP params ---

/// A `u32` that deserializes from either a JSON number or a hex string
/// (`"0x01000E04"`).  Serializes as a decimal number for JSON round-trip,
/// but the `JsonSchema` advertises `oneOf [integer, string]` so the MCP
/// client knows hex is accepted.
#[derive(Debug, Clone, Copy, Default)]
pub struct HexU32(pub u32);

impl From<HexU32> for u32 {
    fn from(h: HexU32) -> u32 { h.0 }
}

impl std::ops::Deref for HexU32 {
    type Target = u32;
    fn deref(&self) -> &u32 { &self.0 }
}

fn deserialize_hex_u32<'de, D: serde::Deserializer<'de>>(de: D) -> Result<u32, D::Error> {
    struct Visitor;
    impl serde::de::Visitor<'_> for Visitor {
        type Value = u32;
        fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
            write!(f, "an integer or a hex string like \"0x01000E04\"")
        }
        fn visit_u64<E: serde::de::Error>(self, v: u64) -> Result<u32, E> {
            Ok(v as u32)
        }
        fn visit_i64<E: serde::de::Error>(self, v: i64) -> Result<u32, E> {
            Ok(v as u32)
        }
        fn visit_str<E: serde::de::Error>(self, s: &str) -> Result<u32, E> {
            let s = s.trim();
            if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
                u32::from_str_radix(hex, 16)
                    .map_err(serde::de::Error::custom)
            } else {
                s.parse::<u32>()
                    .map_err(serde::de::Error::custom)
            }
        }
    }
    de.deserialize_any(Visitor)
}

impl<'de> serde::Deserialize<'de> for HexU32 {
    fn deserialize<D: serde::Deserializer<'de>>(de: D) -> Result<Self, D::Error> {
        deserialize_hex_u32(de).map(HexU32)
    }
}

impl serde::Serialize for HexU32 {
    fn serialize<S: serde::Serializer>(&self, ser: S) -> Result<S::Ok, S::Error> {
        ser.serialize_u32(self.0)
    }
}

impl schemars::JsonSchema for HexU32 {
    fn schema_name() -> std::borrow::Cow<'static, str> { std::borrow::Cow::Borrowed("HexU32") }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        serde_json::from_value(serde_json::json!({
            "description": "A 32-bit unsigned integer. Accepts decimal (16786444) or hex string (\"0x0100240C\").",
            "oneOf": [
                { "type": "integer", "minimum": 0 },
                { "type": "string", "pattern": "^(0[xX])?[0-9a-fA-F]+$" }
            ]
        })).unwrap()
    }
}

// ---------- HexValue — hex-string serialization for MCP responses ----------

/// A `u32` that serializes as a hex string (`"0x01000E04"`) in MCP
/// responses. Deserializes the same as `HexU32` (accepts both integer
/// and hex string) for round-trip compatibility.
#[derive(Debug, Clone, Copy, Default)]
pub struct HexValue(pub u32);

impl From<u32> for HexValue {
    fn from(v: u32) -> Self { HexValue(v) }
}

impl From<HexValue> for u32 {
    fn from(h: HexValue) -> u32 { h.0 }
}

impl serde::Serialize for HexValue {
    fn serialize<S: serde::Serializer>(&self, ser: S) -> Result<S::Ok, S::Error> {
        ser.serialize_str(&format!("0x{:08X}", self.0))
    }
}

impl<'de> serde::Deserialize<'de> for HexValue {
    fn deserialize<D: serde::Deserializer<'de>>(de: D) -> Result<Self, D::Error> {
        deserialize_hex_u32(de).map(HexValue)
    }
}

impl schemars::JsonSchema for HexValue {
    fn schema_name() -> std::borrow::Cow<'static, str> { std::borrow::Cow::Borrowed("HexValue") }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        serde_json::from_value(serde_json::json!({
            "description": "A 32-bit value as hex string (e.g. \"0x01000E04\").",
            "type": "string",
            "pattern": "^0x[0-9A-F]{8}$"
        })).unwrap()
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
    pub values: HashMap<String, HexValue>,
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
    pub status32: HexValue,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CpuStateResult {
    pub pc: HexValue,
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
    pub entry_point: Option<HexValue>,
    pub flash_size: Option<usize>,
    pub flash_loaded: Option<bool>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct PeekMmioParams {
    pub address: HexU32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PeekMmioResult {
    pub address: HexValue,
    pub value: HexValue,
    pub name: Option<String>,
    pub block: Option<String>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ReadFlashParams {
    pub offset: HexU32,
    pub length: HexU32,
}

// ---------- Phase 5a mutation DTOs ---------------------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct WriteMmioParams {
    pub address: HexU32,
    pub value: HexU32,
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
    pub offset: HexU32,
    /// Hex string of bytes to write (no separators, upper- or
    /// lower-case). Byte count = hex string length / 2.
    pub hex: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct WriteFlashResult {
    pub bytes_written: usize,
    pub offset: HexValue,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct FlashPathParams {
    pub path: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct AddSymbolParams {
    pub address: HexU32,
    pub name: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct AddCommentParams {
    pub address: HexU32,
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
    pub address: HexValue,
    pub peripheral: String,
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: HexValue,
    pub last_write_value: HexValue,
    pub first_pc: HexValue,
    pub first_insn: u64,
    pub access_widths: u8,
    pub name: Option<String>,
    pub block: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DumpMmioTraceResult {
    pub enabled: bool,
    pub entries: Vec<MmioTraceEntryJson>,
}

// ---------- Unhandled MMIO DTOs ------------------------------------------

#[derive(Debug, Serialize, JsonSchema)]
pub struct UnhandledMmioEntry {
    pub address: HexValue,
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: HexValue,
    pub last_write_value: HexValue,
    pub first_pc: HexValue,
    pub first_insn: u64,
    pub name: Option<String>,
    pub block: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct UnhandledMmioResult {
    pub count: usize,
    pub entries: Vec<UnhandledMmioEntry>,
}

// ---------- MMIO history DTOs (Phase B) -----------------------------------

#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
pub struct GetMmioHistoryParams {
    /// Return only the last N entries.
    pub last: Option<usize>,
    /// Filter by MMIO address.
    pub address: Option<HexU32>,
    /// Filter: only entries at or after this instruction count.
    pub from_insn: Option<u64>,
    /// Filter: only entries at or before this instruction count.
    pub to_insn: Option<u64>,
    /// Max entries to return (default 100).
    pub limit: Option<usize>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MmioHistoryEntryJson {
    pub insn: u64,
    pub pc: HexValue,
    pub blink: HexValue,
    pub address: HexValue,
    pub value: HexValue,
    pub direction: String,
    pub width: String,
    pub peripheral: String,
    pub name: Option<String>,
    pub block: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MmioHistoryResult {
    pub total_in_buffer: usize,
    pub returned: usize,
    pub entries: Vec<MmioHistoryEntryJson>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetMmioHistorySizeParams {
    pub size: usize,
}

// ---------- Coverage DTOs (Phase C1) --------------------------------------

#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
pub struct GetCoverageParams {
    /// Only return entries at or above this address.
    pub range_start: Option<HexU32>,
    /// Only return entries below this address.
    pub range_end: Option<HexU32>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CoverageEntry {
    pub address: HexValue,
    pub hits: u32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CoverageResult {
    pub total_pcs: usize,
    pub entries: Vec<CoverageEntry>,
}

// ---------- Call stack + profiling DTOs (Phase C2/C3) ---------------------

#[derive(Debug, Serialize, JsonSchema)]
pub struct CallStackResult {
    pub depth: usize,
    pub frames: Vec<HexValue>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct FunctionProfileEntry {
    pub address: HexValue,
    pub instructions: u64,
}

#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
pub struct FunctionProfileParams {
    /// Return top N entries by instruction count (default 20).
    pub top: Option<usize>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct FunctionProfileResult {
    pub enabled: bool,
    pub entries: Vec<FunctionProfileEntry>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetProfilingParams {
    pub enabled: bool,
}

// ---------- ExplainMmio DTOs ---------------------------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ExplainMmioParams {
    pub address: HexU32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ExplainMmioResult {
    pub address: HexValue,
    pub value: HexValue,
    pub name: Option<String>,
    pub block: Option<String>,
    pub access: Option<String>,
    pub description: Option<String>,
    pub has_override: bool,
    pub last_access: Option<LastAccessJson>,
    pub peripheral: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LastAccessJson {
    pub pc: HexValue,
    pub blink: HexValue,
    pub insn: u64,
    pub direction: String,
    pub value: HexValue,
}

// ---------- OLT DTOs -----------------------------------------------------

#[derive(Debug, Serialize, JsonSchema)]
pub struct OltStateResult {
    pub enabled: bool,
    pub mpcp_state: String,
    pub olt_mac: String,
    pub onu_mac: String,
    pub assigned_llid: u16,
    pub mpcp_timestamp: HexValue,
    pub tx_frame_count: usize,
    pub rx_frame_count: usize,
    pub oam_keepalive_count: u64,
    pub gate_count: u64,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct OltConfigParams {
    /// OLT MAC address (e.g. `"00:0A:F7:01:00:01"`).
    pub mac: Option<String>,
    /// Starting LLID for ONU registration.
    pub llid_start: Option<u16>,
    /// OAM keepalive interval in bank ticks.
    pub oam_interval_ticks: Option<u64>,
    /// GATE interval in bank ticks.
    pub gate_interval_ticks: Option<u64>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct OltConfigResult {
    pub mac: String,
    pub llid_start: u16,
    pub oam_interval_ticks: u64,
    pub gate_interval_ticks: u64,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct OltEnableParams {
    /// Set to true to enable, false to disable.
    pub enabled: bool,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct OltInjectFrameParams {
    /// Hex string of raw Ethernet frame bytes.
    pub hex: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct OltFrameLogEntry {
    pub tick: u64,
    pub description: String,
    pub hex: String,
    pub length: usize,
}

#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
pub struct OltFrameLogParams {
    /// Return only the last N entries.
    pub last: Option<usize>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct OltFrameLogResult {
    pub total: usize,
    pub entries: Vec<OltFrameLogEntry>,
}

// ---------- Scenario override DTOs (Phase 1) -----------------------------

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetMmioOverrideParams {
    pub address: HexU32,
    pub value: HexU32,
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
    pub address: HexU32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MmioOverrideEntry {
    pub address: HexValue,
    pub mode: String,
    pub value: HexValue,
    pub remaining: Option<u32>,
    pub label: Option<String>,
    pub name: Option<String>,
    pub block: Option<String>,
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
    /// When true, the event re-arms after firing (fires on every
    /// N-th occurrence).  Default false = one-shot.
    #[serde(default)]
    pub repeat: bool,
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
    pub address: HexU32,
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
    /// Optional condition: `{ "mask": "0xFF", "expect": "0x01" }`.
    /// Watchpoint fires only when `(value & mask) == expect`.
    #[serde(default)]
    pub condition: Option<WatchpointConditionParams>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct WatchpointConditionParams {
    pub mask: HexU32,
    pub expect: HexU32,
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
    pub address: HexValue,
    pub size: u32,
    pub mode: String,
    pub action: String,
    pub label: Option<String>,
    pub name: Option<String>,
    pub block: Option<String>,
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
    pub address: HexU32,
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
    pub entry_point: Option<HexU32>,
    #[serde(default = "default_true")]
    pub keep_breakpoints: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LoadFirmwareResponse {
    pub ok: bool,
    pub loaded_bytes: usize,
    pub entry_point: HexValue,
    pub flash_bytes: usize,
    pub error: Option<String>,
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetBreakpointParams {
    pub address: HexU32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct RemoveBreakpointParams {
    pub address: HexU32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SetWatchpointParams {
    pub address: HexU32,
    pub size: HexU32,
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
    pub value: HexU32,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct WriteMemoryParams {
    pub address: HexU32,
    /// Hex-encoded bytes.
    pub hex: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ReadMemoryParams {
    pub address: HexU32,
    pub length: HexU32,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReadMemoryResult {
    pub address: HexValue,
    pub length: u32,
    pub hex: String,
    pub ascii: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct ReadMmioParams {
    pub address: HexU32,
    pub width: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReadMmioResult {
    pub address: HexValue,
    pub value: HexValue,
    pub width: String,
    pub name: Option<String>,
    pub block: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReadFlashResult {
    pub offset: HexValue,
    pub length: u32,
    /// Hex-encoded bytes. Upper-case, no separators.
    pub hex: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PeripheralEntry {
    pub index: usize,
    pub name: String,
    pub snapshot: serde_json::Value,
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
    pub breakpoints: Vec<HexValue>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SymbolsResult {
    pub symbols: HashMap<String, String>,
}

// ── Named snapshot DTOs ─────────────────────────────────────────────────

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct SaveSnapshotParams {
    pub name: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SaveSnapshotResult {
    pub ok: bool,
    pub name: String,
    pub instruction_count: u64,
    pub pc: HexValue,
    pub timestamp: String,
    pub size_bytes: usize,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct RestoreSnapshotParams {
    pub name: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct RestoreSnapshotResult {
    pub ok: bool,
    pub name: String,
    pub instruction_count: u64,
    pub pc: HexValue,
    pub error: Option<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SnapshotInfoJson {
    pub name: String,
    pub instruction_count: u64,
    pub pc: HexValue,
    pub timestamp: String,
    pub size_bytes: usize,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ListSnapshotsResult {
    pub count: usize,
    pub snapshots: Vec<SnapshotInfoJson>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct DeleteSnapshotParams {
    pub name: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DeleteSnapshotResult {
    pub ok: bool,
    pub name: String,
}

// ── Pattern detection + timeline DTOs (Phase E) ─────────────────────────

#[derive(Debug, Serialize, JsonSchema)]
pub struct PatternEntry {
    pub pattern_type: String,
    pub address: HexValue,
    pub secondary_address: Option<HexValue>,
    pub count: usize,
    pub value: Option<HexValue>,
    pub first_pc: HexValue,
    pub last_pc: HexValue,
    pub first_insn: u64,
    pub last_insn: u64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DetectPatternsResult {
    pub count: usize,
    pub patterns: Vec<PatternEntry>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct GetEventTimelineParams {
    pub from_insn: Option<u64>,
    pub to_insn: Option<u64>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct TimelineEventJson {
    pub event_type: String,
    pub block: Option<String>,
    pub address: HexValue,
    pub from_insn: u64,
    pub to_insn: u64,
    pub access_count: usize,
    pub summary: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct GetEventTimelineResult {
    pub count: usize,
    pub events: Vec<TimelineEventJson>,
}

// ── Diff + bulk symbols DTOs (Phase F) ──────────────────────────────────

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct DiffSnapshotsParams {
    pub a: String,
    pub b: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct RegisterDiff {
    pub name: String,
    pub a: HexValue,
    pub b: HexValue,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DiffSnapshotsResult {
    pub ok: bool,
    pub register_diffs: Vec<RegisterDiff>,
    pub pc_a: HexValue,
    pub pc_b: HexValue,
    pub insn_a: u64,
    pub insn_b: u64,
    pub sram_changed_bytes: usize,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct LoadSymbolsFileParams {
    pub path: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct LoadSymbolsFileResult {
    pub ok: bool,
    pub loaded: usize,
    pub error: Option<String>,
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
                entry_point: Some(HexValue(info.entry_point)),
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
                        out.insert(n, HexValue(v));
                    }
                }
            }
            None => {
                for i in 0..32usize {
                    out.insert(format!("r{}", i), HexValue(snap.cpu.core_regs[i]));
                }
                out.insert("pc".into(), HexValue(snap.cpu.pc));
                out.insert("sp".into(), HexValue(snap.cpu.core_regs[28]));
                out.insert("fp".into(), HexValue(snap.cpu.core_regs[27]));
                out.insert("gp".into(), HexValue(snap.cpu.core_regs[26]));
                out.insert("blink".into(), HexValue(snap.cpu.core_regs[31]));
                out.insert("lp_count".into(), HexValue(snap.cpu.core_regs[60]));
                out.insert("status32".into(), HexValue(snap.cpu.flags.status32));
                out.insert("ienable".into(), HexValue(snap.cpu.aux.ienable));
                out.insert("ipending".into(), HexValue(snap.cpu.aux.ipending));
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
            status32: HexValue(f.status32),
        })
    }

    #[tool(
        name = "get_cpu_state",
        description = "Return the headline CPU state: PC, run state, pause reason, instruction counter, halted / sleeping / paused flags."
    )]
    async fn get_cpu_state(&self) -> Json<CpuStateResult> {
        let snap = self.handle.snapshot.lock();
        Json(CpuStateResult {
            pc: HexValue(snap.cpu.pc),
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
        description = "Return every peripheral snapshot as structured JSON."
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
                snapshot: serde_json::to_value(p).unwrap_or_default(),
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
        let addr = params.address.0;
        let value = self
            .handle
            .bank
            .read()
            .peek_word(addr)
            .unwrap_or(0);
        let (name, block) = reg_lookup(addr);
        Json(PeekMmioResult {
            address: HexValue(addr),
            value: HexValue(value),
            name,
            block,
        })
    }

    #[tool(
        name = "list_breakpoints",
        description = "Return every active CPU breakpoint address installed via `set_breakpoint` (phase 5)."
    )]
    async fn list_breakpoints(&self) -> Json<BreakpointsResult> {
        let snap = self.handle.snapshot.lock();
        Json(BreakpointsResult {
            breakpoints: snap.breakpoints.iter().map(|&a| HexValue(a)).collect(),
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
        let start = params.offset.0 as usize;
        let end = start.saturating_add(params.length.0 as usize).min(flash.len());
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
            offset: HexValue(params.offset.0),
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
            "byte" => guard.write_byte(params.address.0, params.value.0 as u8),
            "half" => guard.write_half(params.address.0, params.value.0 as u16),
            _ => guard.write_word(params.address.0, params.value.0),
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
                    offset: HexValue(params.offset.0),
                });
            }
        };
        let mut guard = self.handle.bank.write();
        let flash = &mut guard.pbc.flash.data;
        let start = params.offset.0 as usize;
        let end = (start + bytes.len()).min(flash.len());
        let written = end.saturating_sub(start);
        if written > 0 {
            flash[start..end].copy_from_slice(&bytes[..written]);
            guard.pbc.flash.dirty = true;
        }
        Json(WriteFlashResult {
            bytes_written: written,
            offset: HexValue(params.offset.0),
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
        ann.symbols.insert(params.address.0, params.name);
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
        ann.comments.insert(params.address.0, params.comment);
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
            "static" => OverrideSpec::StaticRead { value: params.value.0 },
            "oneshot" => OverrideSpec::OneShotRead {
                value: params.value.0,
                remaining: params.count,
            },
            "mask" => OverrideSpec::MaskedWriteIgnore { mask: params.value.0 },
            _ => return Json(OkResult { ok: false }),
        };
        self.handle.bank.write().scenario.overrides.set(params.address.0, spec, params.label);
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
        let ok = self.handle.bank.write().scenario.overrides.remove(params.address.0);
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
                let (name, block) = reg_lookup(addr);
                MmioOverrideEntry {
                    address: HexValue(addr),
                    mode: mode.to_string(),
                    value: HexValue(value),
                    remaining,
                    label: ov.label.clone(),
                    name,
                    block,
                }
            })
            .collect();
        overrides.sort_by_key(|e| e.address.0);
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
        let id = self.handle.bank.write().scenario.schedule_ex(trigger, effect, params.label, params.repeat);
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
            .all_events()
            .iter()
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
        let condition = params.condition.map(|c| {
            crate::soc::scenario::ValueCondition::MaskEqual {
                mask: c.mask.0,
                expect: c.expect.0,
            }
        });
        let id = self.handle.bank.write().scenario.add_watchpoint(
            params.address.0,
            params.size,
            mode,
            action,
            params.label,
            condition,
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
                let (name, block) = reg_lookup(wp.address);
                MmioWatchpointEntry {
                    id: wp.id,
                    address: HexValue(wp.address),
                    size: wp.size,
                    mode: mode.to_string(),
                    action: format!("{:?}", wp.action),
                    label: wp.label.clone(),
                    name,
                    block,
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
                    .map(|(addr, e)| {
                        let (name, block) = reg_lookup(*addr);
                        MmioTraceEntryJson {
                            address: HexValue(*addr),
                            peripheral: e.peripheral.to_string(),
                            reads: e.reads,
                            writes: e.writes,
                            last_read_value: HexValue(e.last_read_value),
                            last_write_value: HexValue(e.last_write_value),
                            first_pc: HexValue(e.first_pc),
                            first_insn: e.first_insn,
                            access_widths: e.access_widths,
                            name,
                            block,
                        }
                    })
                    .collect();
                entries.sort_by_key(|e| e.address.0);
                Json(DumpMmioTraceResult {
                    enabled: true,
                    entries,
                })
            }
        }
    }

    #[tool(
        name = "get_unhandled_mmio",
        description = "Return aggregated MMIO accesses to sysreg-range addresses that no dedicated peripheral claimed (unmodelled registers). Always enabled. Enriched with register names from the hwregs database."
    )]
    async fn get_unhandled_mmio(&self) -> Json<UnhandledMmioResult> {
        let guard = self.handle.bank.read();
        match guard.sysreg.mmio_trace.as_ref() {
            None => Json(UnhandledMmioResult { count: 0, entries: Vec::new() }),
            Some(map) => {
                let mut entries: Vec<_> = map
                    .iter()
                    .map(|(addr, e)| {
                        let (name, block) = reg_lookup(*addr);
                        UnhandledMmioEntry {
                            address: HexValue(*addr),
                            reads: e.reads,
                            writes: e.writes,
                            last_read_value: HexValue(e.last_read_value),
                            last_write_value: HexValue(e.last_write_value),
                            first_pc: HexValue(e.first_pc),
                            first_insn: e.first_insn,
                            name,
                            block,
                        }
                    })
                    .collect();
                entries.sort_by_key(|e| e.address.0);
                Json(UnhandledMmioResult {
                    count: entries.len(),
                    entries,
                })
            }
        }
    }

    #[tool(
        name = "explain_mmio",
        description = "All-in-one MMIO register inspector: current value (side-effect-free peek), register name + block from hwregs DB, owning peripheral, active override status, and last firmware access (PC, blink, direction, value). Single tool call replaces peek + lookup + override check."
    )]
    async fn explain_mmio(
        &self,
        Parameters(params): Parameters<ExplainMmioParams>,
    ) -> Json<ExplainMmioResult> {
        let addr = params.address.0;
        let guard = self.handle.bank.read();
        let value = guard.peek_word(addr).unwrap_or(0);
        let (name, block) = reg_lookup(addr);
        let reg_info = mmio_blocks::lookup(addr);
        let access = reg_info.map(|r| r.access.to_string());
        let description = reg_info.map(|r| r.desc.to_string());
        let has_override = guard.scenario.overrides.peek_read(addr).is_some();
        let peripheral = guard.peripheral_for(addr).map(|s| s.to_string());
        let last_access = guard.last_access.get(&addr).map(|la| LastAccessJson {
            pc: HexValue(la.pc),
            blink: HexValue(la.blink),
            insn: la.insn,
            direction: la.direction.to_string(),
            value: HexValue(la.value),
        });
        Json(ExplainMmioResult {
            address: HexValue(addr),
            value: HexValue(value),
            name,
            block,
            access,
            description,
            has_override,
            last_access,
            peripheral,
        })
    }

    // ---------- MMIO history tools (Phase B) ------------------------------

    #[tool(
        name = "get_mmio_history",
        description = "Return recent MMIO accesses from the ring buffer. Filters: `last` (tail N), `address` (single register), `from_insn`/`to_insn` (instruction range), `limit` (max returned, default 100). Enriched with register names."
    )]
    async fn get_mmio_history(
        &self,
        Parameters(params): Parameters<GetMmioHistoryParams>,
    ) -> Json<MmioHistoryResult> {
        let guard = self.handle.bank.read();
        let total = guard.mmio_history.len();
        let limit = params.limit.unwrap_or(100);
        let iter: Box<dyn Iterator<Item = &crate::soc::bank::MmioHistoryEntry> + '_> =
            if let Some(last_n) = params.last {
                let skip = total.saturating_sub(last_n);
                Box::new(guard.mmio_history.iter().skip(skip))
            } else {
                Box::new(guard.mmio_history.iter())
            };
        let filter_addr = params.address.map(|h| h.0);
        let from = params.from_insn.unwrap_or(0);
        let to = params.to_insn.unwrap_or(u64::MAX);
        let entries: Vec<MmioHistoryEntryJson> = iter
            .filter(|e| {
                if let Some(a) = filter_addr { if e.address != a { return false; } }
                e.insn >= from && e.insn <= to
            })
            .take(limit)
            .map(|e| {
                let (name, block) = reg_lookup(e.address);
                MmioHistoryEntryJson {
                    insn: e.insn,
                    pc: HexValue(e.pc),
                    blink: HexValue(e.blink),
                    address: HexValue(e.address),
                    value: HexValue(e.value),
                    direction: e.direction.to_string(),
                    width: e.width.to_string(),
                    peripheral: e.peripheral.to_string(),
                    name,
                    block,
                }
            })
            .collect();
        let returned = entries.len();
        Json(MmioHistoryResult {
            total_in_buffer: total,
            returned,
            entries,
        })
    }

    #[tool(
        name = "set_mmio_history_size",
        description = "Set the MMIO history ring buffer capacity. 0 = disabled. Default 8192."
    )]
    async fn set_mmio_history_size(
        &self,
        Parameters(params): Parameters<SetMmioHistorySizeParams>,
    ) -> Json<OkResult> {
        let mut guard = self.handle.bank.write();
        guard.mmio_history_max = params.size;
        while guard.mmio_history.len() > params.size {
            guard.mmio_history.pop_front();
        }
        Json(OkResult { ok: true })
    }

    #[tool(
        name = "clear_mmio_history",
        description = "Clear all entries from the MMIO history ring buffer."
    )]
    async fn clear_mmio_history(&self) -> Json<OkResult> {
        self.handle.bank.write().mmio_history.clear();
        Json(OkResult { ok: true })
    }

    // ---------- Coverage + call stack tools (Phase C) ----------------------

    #[tool(
        name = "get_coverage",
        description = "Return the coverage map: (address, hit_count) for every PC the CPU has executed. Optional `range_start`/`range_end` to filter to a specific memory region. Requires the CPU worker to be running."
    )]
    async fn get_coverage(
        &self,
        Parameters(params): Parameters<GetCoverageParams>,
    ) -> Json<CoverageResult> {
        let (tx, rx) = oneshot::<Vec<(u32, u32)>>();
        if self.handle.cpu_cmd.send(CpuCommand::RequestCoverage { response: tx }).is_err() {
            return Json(CoverageResult { total_pcs: 0, entries: Vec::new() });
        }
        let sparse = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(5)).unwrap_or_default()
        })
        .await
        .unwrap_or_default();
        let lo = params.range_start.map(|h| h.0).unwrap_or(0);
        let hi = params.range_end.map(|h| h.0).unwrap_or(u32::MAX);
        let entries: Vec<CoverageEntry> = sparse
            .iter()
            .filter(|(addr, _)| *addr >= lo && *addr < hi)
            .map(|(addr, hits)| CoverageEntry {
                address: HexValue(*addr),
                hits: *hits,
            })
            .collect();
        let total = entries.len();
        Json(CoverageResult {
            total_pcs: total,
            entries,
        })
    }

    #[tool(
        name = "reset_coverage",
        description = "Zero the coverage map so a fresh run can measure from scratch."
    )]
    async fn reset_coverage(&self) -> Json<OkResult> {
        let ok = self.handle.cpu_cmd.send(CpuCommand::ClearCoverage).is_ok();
        Json(OkResult { ok })
    }

    #[tool(
        name = "get_call_stack",
        description = "Return the shadow call stack — a list of return addresses from BL/JL instructions, most recent last. Approximate: tail calls and ISRs may not be tracked. Requires the CPU worker."
    )]
    async fn get_call_stack(&self) -> Json<CallStackResult> {
        let (tx, rx) = oneshot::<Vec<u32>>();
        if self.handle.cpu_cmd.send(CpuCommand::RequestCallStack { response: tx }).is_err() {
            return Json(CallStackResult { depth: 0, frames: Vec::new() });
        }
        let stack = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(2)).unwrap_or_default()
        })
        .await
        .unwrap_or_default();
        let depth = stack.len();
        Json(CallStackResult {
            depth,
            frames: stack.into_iter().map(HexValue).collect(),
        })
    }

    #[tool(
        name = "get_function_profile",
        description = "Return the top N functions by instruction count. Must be enabled first via `set_profiling`. Keyed by return address (top of shadow call stack during execution)."
    )]
    async fn get_function_profile(
        &self,
        Parameters(params): Parameters<FunctionProfileParams>,
    ) -> Json<FunctionProfileResult> {
        let (tx, rx) = oneshot::<Vec<(u32, u64)>>();
        if self.handle.cpu_cmd.send(CpuCommand::RequestFunctionProfile { response: tx }).is_err() {
            return Json(FunctionProfileResult { enabled: false, entries: Vec::new() });
        }
        let entries = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(5)).unwrap_or_default()
        })
        .await
        .unwrap_or_default();
        let top = params.top.unwrap_or(20);
        let entries: Vec<FunctionProfileEntry> = entries
            .into_iter()
            .take(top)
            .map(|(addr, insns)| FunctionProfileEntry {
                address: HexValue(addr),
                instructions: insns,
            })
            .collect();
        Json(FunctionProfileResult {
            enabled: true,
            entries,
        })
    }

    #[tool(
        name = "set_profiling",
        description = "Enable or disable per-function instruction profiling. When disabled, the profile map is cleared. ~5-10% overhead when enabled."
    )]
    async fn set_profiling(
        &self,
        Parameters(params): Parameters<SetProfilingParams>,
    ) -> Json<OkResult> {
        let ok = self.handle.cpu_cmd.send(CpuCommand::SetProfiling { enabled: params.enabled }).is_ok();
        Json(OkResult { ok })
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
                address: params.address.0,
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
        let entry_point_raw = params.entry_point.map(|h| h.0).unwrap_or(0);
        let (tx, rx) = oneshot::<Result<crate::emu::command::LoadFirmwareResult, String>>();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::LoadFirmware {
                path: params.path.into(),
                mode: crate::emu::command::FirmwareMode::Soc,
                boot_mode: mode,
                flash_path: None,
                entry_point: entry_point_raw,
                keep_breakpoints: params.keep_breakpoints,
                response: tx,
            })
            .is_err()
        {
            return Json(LoadFirmwareResponse {
                ok: false,
                loaded_bytes: 0,
                entry_point: HexValue(entry_point_raw),
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
                entry_point: HexValue(r.entry_point),
                flash_bytes: r.flash_bytes,
                error: None,
            }),
            Ok(Err(e)) => Json(LoadFirmwareResponse {
                ok: false,
                loaded_bytes: 0,
                entry_point: HexValue(entry_point_raw),
                flash_bytes: 0,
                error: Some(e),
            }),
            Err(_) => Json(LoadFirmwareResponse {
                ok: false,
                loaded_bytes: 0,
                entry_point: HexValue(entry_point_raw),
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
                address: params.address.0,
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
                address: params.address.0,
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
                addr: params.address.0,
                size: params.size.0,
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
                value: params.value.0,
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
                addr: params.address.0,
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
                address: HexValue(params.address.0),
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
                address: HexValue(params.address.0),
                length: 0,
                hex: String::new(),
                ascii: String::new(),
            });
        };
        let start = params.address.0 as usize;
        let end = start
            .saturating_add(params.length.0 as usize)
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
            address: HexValue(params.address.0),
            length: slice.len() as u32,
            hex,
            ascii,
        })
    }

    // ── Pattern detection + timeline (Phase E) ────────────────────────

    #[tool(
        name = "detect_mmio_patterns",
        description = "Analyze the MMIO history buffer for common HW interaction patterns: busy-wait (same addr/value read ≥5×), write-then-poll (write then ≥3 reads), command-bit (bit 31 set, polled until clear)."
    )]
    async fn detect_mmio_patterns(&self) -> Json<DetectPatternsResult> {
        let guard = self.handle.bank.read();
        let history: Vec<crate::soc::bank::MmioHistoryEntry> =
            guard.mmio_history.iter().cloned().collect();
        drop(guard);

        let raw = crate::soc::analysis::detect_patterns(&history);
        let patterns: Vec<PatternEntry> = raw
            .into_iter()
            .map(|p| PatternEntry {
                pattern_type: p.pattern_type,
                address: HexValue(p.address),
                secondary_address: p.secondary_address.map(HexValue),
                count: p.count,
                value: p.value.map(HexValue),
                first_pc: HexValue(p.first_pc),
                last_pc: HexValue(p.last_pc),
                first_insn: p.first_insn,
                last_insn: p.last_insn,
            })
            .collect();
        Json(DetectPatternsResult {
            count: patterns.len(),
            patterns,
        })
    }

    #[tool(
        name = "get_event_timeline",
        description = "Condense MMIO history into a high-level event timeline. Groups consecutive accesses to the same peripheral into bursts, identifies busy-waits and polling loops."
    )]
    async fn get_event_timeline(
        &self,
        Parameters(params): Parameters<GetEventTimelineParams>,
    ) -> Json<GetEventTimelineResult> {
        let guard = self.handle.bank.read();
        let history: Vec<crate::soc::bank::MmioHistoryEntry> =
            guard.mmio_history.iter().cloned().collect();
        drop(guard);

        let raw = crate::soc::analysis::build_timeline(&history, params.from_insn, params.to_insn);
        let events: Vec<TimelineEventJson> = raw
            .into_iter()
            .map(|e| TimelineEventJson {
                event_type: e.event_type,
                block: e.block,
                address: HexValue(e.address),
                from_insn: e.from_insn,
                to_insn: e.to_insn,
                access_count: e.access_count,
                summary: e.summary,
            })
            .collect();
        Json(GetEventTimelineResult {
            count: events.len(),
            events,
        })
    }

    // ── Diff snapshots + bulk symbols (Phase F) ─────────────────────

    #[tool(
        name = "diff_snapshots",
        description = "Compare two named snapshots. Returns register diffs, SRAM change count, and instruction/PC deltas."
    )]
    async fn diff_snapshots(
        &self,
        Parameters(params): Parameters<DiffSnapshotsParams>,
    ) -> Json<DiffSnapshotsResult> {
        let err = |msg: String| {
            DiffSnapshotsResult {
                ok: false,
                register_diffs: Vec::new(),
                pc_a: HexValue(0),
                pc_b: HexValue(0),
                insn_a: 0,
                insn_b: 0,
                sram_changed_bytes: 0,
                error: Some(msg),
            }
        };
        let (tx, rx) = oneshot();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::DiffSnapshots {
                a: params.a.clone(),
                b: params.b.clone(),
                response: tx,
            })
            .is_err()
        {
            return Json(err("cpu_cmd channel closed".into()));
        }
        let result = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(10))
        })
        .await
        .unwrap_or(Err(std::sync::mpsc::RecvTimeoutError::Timeout));
        match result {
            Ok(Ok(diff)) => Json(DiffSnapshotsResult {
                ok: true,
                register_diffs: diff
                    .register_diffs
                    .into_iter()
                    .map(|d| RegisterDiff {
                        name: d.name,
                        a: HexValue(d.a),
                        b: HexValue(d.b),
                    })
                    .collect(),
                pc_a: HexValue(diff.pc_a),
                pc_b: HexValue(diff.pc_b),
                insn_a: diff.insn_a,
                insn_b: diff.insn_b,
                sram_changed_bytes: diff.sram_changed_bytes,
                error: None,
            }),
            Ok(Err(e)) => Json(err(e)),
            Err(_) => Json(err("timeout".into())),
        }
    }

    #[tool(
        name = "load_symbols_file",
        description = "Bulk-load symbols from a JSON file. Format: `{\"0x1234\": \"name\", ...}` or `[{\"address\": \"0x1234\", \"name\": \"foo\"}, ...]`."
    )]
    async fn load_symbols_file(
        &self,
        Parameters(params): Parameters<LoadSymbolsFileParams>,
    ) -> Json<LoadSymbolsFileResult> {
        let content = match std::fs::read_to_string(&params.path) {
            Ok(c) => c,
            Err(e) => {
                return Json(LoadSymbolsFileResult {
                    ok: false,
                    loaded: 0,
                    error: Some(format!("read {}: {}", params.path, e)),
                })
            }
        };
        let doc: serde_json::Value = match serde_json::from_str(&content) {
            Ok(v) => v,
            Err(e) => {
                return Json(LoadSymbolsFileResult {
                    ok: false,
                    loaded: 0,
                    error: Some(format!("parse: {e}")),
                })
            }
        };

        let mut ann = self.handle.annotations.write();
        let mut count = 0;

        if let Some(obj) = doc.as_object() {
            for (k, v) in obj {
                if let (Some(addr), Some(name)) = (parse_addr_str(k), v.as_str()) {
                    ann.symbols.insert(addr, name.to_string());
                    count += 1;
                }
            }
        } else if let Some(arr) = doc.as_array() {
            for entry in arr {
                let addr = entry
                    .get("address")
                    .and_then(|v| v.as_str())
                    .and_then(parse_addr_str)
                    .or_else(|| entry.get("address").and_then(|v| v.as_u64()).map(|n| n as u32));
                let name = entry.get("name").and_then(|v| v.as_str());
                if let (Some(a), Some(n)) = (addr, name) {
                    ann.symbols.insert(a, n.to_string());
                    count += 1;
                }
            }
        }

        Json(LoadSymbolsFileResult {
            ok: true,
            loaded: count,
            error: None,
        })
    }

    // ── Named snapshots ──────────────────────────────────────────────

    #[tool(
        name = "save_snapshot",
        description = "Save a named snapshot of the entire emulator state (CPU, SRAM, caches, peripherals, scenario). Can be restored later via `restore_snapshot`."
    )]
    async fn save_snapshot(
        &self,
        Parameters(params): Parameters<SaveSnapshotParams>,
    ) -> Json<SaveSnapshotResult> {
        let (tx, rx) = oneshot();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::SaveSnapshot {
                name: params.name.clone(),
                response: tx,
            })
            .is_err()
        {
            return Json(SaveSnapshotResult {
                ok: false,
                name: params.name,
                instruction_count: 0,
                pc: HexValue(0),
                timestamp: String::new(),
                size_bytes: 0,
                error: Some("cpu_cmd channel closed".into()),
            });
        }
        let result = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(10))
        })
        .await
        .unwrap_or(Err(std::sync::mpsc::RecvTimeoutError::Timeout));
        match result {
            Ok(Ok(info)) => Json(SaveSnapshotResult {
                ok: true,
                name: info.name,
                instruction_count: info.instruction_count,
                pc: HexValue(info.pc),
                timestamp: info.timestamp,
                size_bytes: info.size_bytes,
                error: None,
            }),
            Ok(Err(e)) => Json(SaveSnapshotResult {
                ok: false,
                name: params.name,
                instruction_count: 0,
                pc: HexValue(0),
                timestamp: String::new(),
                size_bytes: 0,
                error: Some(e),
            }),
            Err(_) => Json(SaveSnapshotResult {
                ok: false,
                name: params.name,
                instruction_count: 0,
                pc: HexValue(0),
                timestamp: String::new(),
                size_bytes: 0,
                error: Some("timeout".into()),
            }),
        }
    }

    #[tool(
        name = "restore_snapshot",
        description = "Restore a previously saved named snapshot. CPU is paused after restore. SRAM, peripheral registers, and scenario state are rolled back."
    )]
    async fn restore_snapshot(
        &self,
        Parameters(params): Parameters<RestoreSnapshotParams>,
    ) -> Json<RestoreSnapshotResult> {
        let (tx, rx) = oneshot();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::RestoreSnapshot {
                name: params.name.clone(),
                response: tx,
            })
            .is_err()
        {
            return Json(RestoreSnapshotResult {
                ok: false,
                name: params.name,
                instruction_count: 0,
                pc: HexValue(0),
                error: Some("cpu_cmd channel closed".into()),
            });
        }
        let result = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(10))
        })
        .await
        .unwrap_or(Err(std::sync::mpsc::RecvTimeoutError::Timeout));
        match result {
            Ok(Ok(info)) => Json(RestoreSnapshotResult {
                ok: true,
                name: info.name,
                instruction_count: info.instruction_count,
                pc: HexValue(info.pc),
                error: None,
            }),
            Ok(Err(e)) => Json(RestoreSnapshotResult {
                ok: false,
                name: params.name,
                instruction_count: 0,
                pc: HexValue(0),
                error: Some(e),
            }),
            Err(_) => Json(RestoreSnapshotResult {
                ok: false,
                name: params.name,
                instruction_count: 0,
                pc: HexValue(0),
                error: Some("timeout".into()),
            }),
        }
    }

    #[tool(
        name = "list_snapshots",
        description = "List all named snapshots currently held in memory."
    )]
    async fn list_snapshots(&self) -> Json<ListSnapshotsResult> {
        let (tx, rx) = oneshot();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::ListSnapshots { response: tx })
            .is_err()
        {
            return Json(ListSnapshotsResult {
                count: 0,
                snapshots: Vec::new(),
            });
        }
        let list = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(5))
        })
        .await
        .unwrap_or(Err(std::sync::mpsc::RecvTimeoutError::Timeout))
        .unwrap_or_default();
        let mapped: Vec<SnapshotInfoJson> = list
            .into_iter()
            .map(|s| SnapshotInfoJson {
                name: s.name,
                instruction_count: s.instruction_count,
                pc: HexValue(s.pc),
                timestamp: s.timestamp,
                size_bytes: s.size_bytes,
            })
            .collect();
        Json(ListSnapshotsResult {
            count: mapped.len(),
            snapshots: mapped,
        })
    }

    #[tool(
        name = "delete_snapshot",
        description = "Delete a named snapshot from memory."
    )]
    async fn delete_snapshot(
        &self,
        Parameters(params): Parameters<DeleteSnapshotParams>,
    ) -> Json<DeleteSnapshotResult> {
        let (tx, rx) = oneshot();
        if self
            .handle
            .cpu_cmd
            .send(CpuCommand::DeleteSnapshot {
                name: params.name.clone(),
                response: tx,
            })
            .is_err()
        {
            return Json(DeleteSnapshotResult {
                ok: false,
                name: params.name,
            });
        }
        let existed = tokio::task::spawn_blocking(move || {
            rx.recv_timeout(Duration::from_secs(5))
        })
        .await
        .unwrap_or(Err(std::sync::mpsc::RecvTimeoutError::Timeout))
        .unwrap_or(false);
        Json(DeleteSnapshotResult {
            ok: existed,
            name: params.name,
        })
    }

    #[tool(
        name = "read_mmio",
        description = "Side-effectful MMIO read. Unlike `peek_mmio` this triggers FIFO pops, IRQ latch clears, and busy-bit transitions. Use `peek_mmio` for the inspector."
    )]
    async fn read_mmio(
        &self,
        Parameters(params): Parameters<ReadMmioParams>,
    ) -> Json<ReadMmioResult> {
        let addr = params.address.0;
        let mut guard = self.handle.bank.write();
        let width = params
            .width
            .as_deref()
            .unwrap_or("word")
            .to_ascii_lowercase();
        let value = match width.as_str() {
            "byte" => guard.read_byte(addr).unwrap_or(0) as u32,
            "half" => guard.read_half(addr).unwrap_or(0) as u32,
            _ => guard.read_word(addr).unwrap_or(0),
        };
        let (name, block) = reg_lookup(addr);
        Json(ReadMmioResult {
            address: HexValue(addr),
            value: HexValue(value),
            width,
            name,
            block,
        })
    }

    // ── OLT emulator tools ─────────────────────────────────────────

    #[tool(
        name = "olt_get_state",
        description = "Return the current OLT emulator state: MPCP registration state, MAC addresses, LLID, frame counts."
    )]
    async fn olt_get_state(&self) -> Json<OltStateResult> {
        let bank = self.handle.bank.read();
        // Access the OLT snapshot via the snapshot_all vector which
        // includes the OLT as the last entry.
        let snaps = bank.snapshot_all();
        let olt_snap = snaps.iter().find_map(|s| {
            if let PeripheralSnapshot::Olt(ref o) = s {
                Some(o.clone())
            } else {
                None
            }
        });
        match olt_snap {
            Some(s) => Json(OltStateResult {
                enabled: s.enabled,
                mpcp_state: s.mpcp_state,
                olt_mac: s.olt_mac,
                onu_mac: s.onu_mac,
                assigned_llid: s.assigned_llid,
                mpcp_timestamp: HexValue(s.mpcp_timestamp),
                tx_frame_count: s.tx_frame_count,
                rx_frame_count: s.rx_frame_count,
                oam_keepalive_count: s.oam_keepalive_count,
                gate_count: s.gate_count,
            }),
            None => Json(OltStateResult {
                enabled: false,
                mpcp_state: "unknown".into(),
                olt_mac: "00:00:00:00:00:00".into(),
                onu_mac: "00:00:00:00:00:00".into(),
                assigned_llid: 0,
                mpcp_timestamp: HexValue(0),
                tx_frame_count: 0,
                rx_frame_count: 0,
                oam_keepalive_count: 0,
                gate_count: 0,
            }),
        }
    }

    #[tool(
        name = "olt_get_config",
        description = "Return the OLT emulator configuration: MAC address, LLID range, timing parameters."
    )]
    async fn olt_get_config(&self) -> Json<OltConfigResult> {
        let bank = self.handle.bank.read();
        let cfg = &bank.olt.config;
        Json(OltConfigResult {
            mac: format_mac(&cfg.mac),
            llid_start: cfg.llid_start,
            oam_interval_ticks: cfg.oam_interval_ticks,
            gate_interval_ticks: cfg.gate_interval_ticks,
        })
    }

    #[tool(
        name = "olt_set_config",
        description = "Update OLT emulator configuration. Only provided fields are changed."
    )]
    async fn olt_set_config(
        &self,
        Parameters(params): Parameters<OltConfigParams>,
    ) -> Json<OltConfigResult> {
        let mut bank = self.handle.bank.write();
        if let Some(ref mac_str) = params.mac {
            if let Some(mac) = parse_mac_str(mac_str) {
                bank.olt.config.mac = mac;
            }
        }
        if let Some(llid) = params.llid_start {
            bank.olt.config.llid_start = llid;
        }
        if let Some(interval) = params.oam_interval_ticks {
            bank.olt.config.oam_interval_ticks = interval;
        }
        if let Some(interval) = params.gate_interval_ticks {
            bank.olt.config.gate_interval_ticks = interval;
        }
        let cfg = &bank.olt.config;
        Json(OltConfigResult {
            mac: format_mac(&cfg.mac),
            llid_start: cfg.llid_start,
            oam_interval_ticks: cfg.oam_interval_ticks,
            gate_interval_ticks: cfg.gate_interval_ticks,
        })
    }

    #[tool(
        name = "olt_enable",
        description = "Enable or disable OLT emulation. When enabled, the OLT auto-starts MPCP discovery and OAM keepalive."
    )]
    async fn olt_enable(
        &self,
        Parameters(params): Parameters<OltEnableParams>,
    ) -> Json<OkResult> {
        let mut bank = self.handle.bank.write();
        bank.olt.set_enabled(params.enabled);
        if params.enabled {
            bank.olt.set_link_up(true);
        }
        Json(OkResult { ok: true })
    }

    #[tool(
        name = "olt_inject_frame",
        description = "Inject a raw Ethernet frame into the ONU's RX path through the OLT emulator. Provide frame bytes as hex."
    )]
    async fn olt_inject_frame(
        &self,
        Parameters(params): Parameters<OltInjectFrameParams>,
    ) -> Json<OkResult> {
        let bytes = match hex_decode(&params.hex) {
            Some(b) => b,
            None => return Json(OkResult { ok: false }),
        };
        let mut bank = self.handle.bank.write();
        bank.olt.inject_raw_frame(bytes);
        Json(OkResult { ok: true })
    }

    #[tool(
        name = "olt_get_tx_log",
        description = "Return the log of frames captured from ONU TX (ONU → OLT direction)."
    )]
    async fn olt_get_tx_log(
        &self,
        Parameters(params): Parameters<OltFrameLogParams>,
    ) -> Json<OltFrameLogResult> {
        let bank = self.handle.bank.read();
        let log = bank.olt.tx_log();
        let last = params.last.unwrap_or(50);
        let start = log.len().saturating_sub(last);
        let entries: Vec<OltFrameLogEntry> = log
            .iter()
            .skip(start)
            .map(|f| OltFrameLogEntry {
                tick: f.tick,
                description: f.description.clone(),
                hex: bytes_to_hex(&f.data),
                length: f.data.len(),
            })
            .collect();
        Json(OltFrameLogResult {
            total: log.len(),
            entries,
        })
    }

    #[tool(
        name = "olt_get_rx_log",
        description = "Return the log of frames injected to ONU RX (OLT → ONU direction)."
    )]
    async fn olt_get_rx_log(
        &self,
        Parameters(params): Parameters<OltFrameLogParams>,
    ) -> Json<OltFrameLogResult> {
        let bank = self.handle.bank.read();
        let log = bank.olt.rx_log();
        let last = params.last.unwrap_or(50);
        let start = log.len().saturating_sub(last);
        let entries: Vec<OltFrameLogEntry> = log
            .iter()
            .skip(start)
            .map(|f| OltFrameLogEntry {
                tick: f.tick,
                description: f.description.clone(),
                hex: bytes_to_hex(&f.data),
                length: f.data.len(),
            })
            .collect();
        Json(OltFrameLogResult {
            total: log.len(),
            entries,
        })
    }
}

fn format_mac(mac: &[u8; 6]) -> String {
    format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    )
}

fn parse_mac_str(s: &str) -> Option<[u8; 6]> {
    let parts: Vec<&str> = s.split(':').collect();
    if parts.len() != 6 {
        return None;
    }
    let mut mac = [0u8; 6];
    for (i, part) in parts.iter().enumerate() {
        mac[i] = u8::from_str_radix(part, 16).ok()?;
    }
    Some(mac)
}

fn hex_decode(s: &str) -> Option<Vec<u8>> {
    let s = s.trim();
    let s = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    if s.len() % 2 != 0 {
        return None;
    }
    let mut bytes = Vec::with_capacity(s.len() / 2);
    for i in (0..s.len()).step_by(2) {
        let byte = u8::from_str_radix(&s[i..i + 2], 16).ok()?;
        bytes.push(byte);
    }
    Some(bytes)
}

fn bytes_to_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02X}", b)).collect()
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
            | "set_mmio_history_size"
            | "clear_mmio_history"
            | "reset_coverage"
            | "set_profiling"
            | "save_snapshot"
            | "restore_snapshot"
            | "delete_snapshot"
            | "load_symbols_file"
            | "olt_enable"
            | "olt_set_config"
            | "olt_inject_frame"
    )
}

// ---------- Helpers -------------------------------------------------------

use crate::soc::mmio_blocks;
use crate::soc::scenario::{parse_effect, parse_trigger};

fn reg_lookup(addr: u32) -> (Option<String>, Option<String>) {
    match mmio_blocks::lookup(addr) {
        Some(info) => (
            Some(info.reg_name.to_string()),
            Some(info.block_name.to_string()),
        ),
        None => (None, None),
    }
}

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

fn parse_addr_str(s: &str) -> Option<u32> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).ok()
    } else {
        s.parse::<u32>().ok()
    }
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
