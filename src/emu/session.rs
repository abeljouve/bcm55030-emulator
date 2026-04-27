//! Session save / restore. A *session* is a versioned JSON
//! snapshot of everything the user cares about across restarts:
//! the loaded firmware path, breakpoints, watchpoints,
//! annotations, palette, and the view cursors. Applying a
//! session rebuilds the worker state by replaying the snapshot
//! through `CpuCommand`s and by writing directly into
//! `EmulatorHandle.annotations`.
//!
//! The JSON format is hand-rolled (no serde derives) so that the
//! shape stays stable even if the underlying types grow new
//! fields. Missing keys default gracefully.

use std::collections::HashMap;
use std::path::PathBuf;

use serde_json::{json, Value};

use crate::memory::{WatchMode, Watchpoint};

/// Current schema version. Incremented on breaking changes.
pub const SESSION_VERSION: u32 = 1;

/// A saved session. Opaque helper that wraps a JSON document.
#[derive(Clone, Debug, Default)]
pub struct Session {
    pub firmware_path: Option<PathBuf>,
    pub palette: Option<String>,
    pub breakpoints: Vec<u32>,
    pub watchpoints: Vec<Watchpoint>,
    pub symbols: HashMap<u32, String>,
    pub comments: HashMap<u32, String>,
    pub regions: Vec<(String, u32, u32)>,
    pub disasm_view_base: Option<u32>,
    pub disasm_follow_pc: Option<bool>,
    pub memory_cursor: Option<u32>,
}

impl Session {
    /// Serialise the session to pretty-printed JSON.
    pub fn to_json_string(&self) -> String {
        let mut breakpoints: Vec<Value> = self
            .breakpoints
            .iter()
            .map(|a| json!(format!("0x{:08X}", a)))
            .collect();
        breakpoints.sort_by(|a, b| a.as_str().unwrap_or("").cmp(b.as_str().unwrap_or("")));

        let watchpoints: Vec<Value> = self
            .watchpoints
            .iter()
            .map(|w| {
                json!({
                    "addr": format!("0x{:08X}", w.addr),
                    "size": w.size,
                    "mode": watch_mode_to_str(w.mode),
                })
            })
            .collect();

        let symbols: Vec<Value> = self
            .symbols
            .iter()
            .map(|(addr, name)| json!([format!("0x{:08X}", addr), name]))
            .collect();
        let comments: Vec<Value> = self
            .comments
            .iter()
            .map(|(addr, text)| json!([format!("0x{:08X}", addr), text]))
            .collect();
        let regions: Vec<Value> = self
            .regions
            .iter()
            .map(|(label, start, end)| {
                json!([label, format!("0x{:08X}", start), format!("0x{:08X}", end)])
            })
            .collect();

        let view = json!({
            "disasm_view_base": self.disasm_view_base.map(|v| format!("0x{:08X}", v)),
            "disasm_follow_pc": self.disasm_follow_pc,
            "memory_cursor": self.memory_cursor.map(|v| format!("0x{:08X}", v)),
        });

        serde_json::to_string_pretty(&json!({
            "version": SESSION_VERSION,
            "firmware_path": self
                .firmware_path
                .as_ref()
                .map(|p| p.to_string_lossy().into_owned()),
            "palette": self.palette,
            "breakpoints": breakpoints,
            "watchpoints": watchpoints,
            "annotations": {
                "symbols": symbols,
                "comments": comments,
                "regions": regions,
            },
            "view": view,
        }))
        .unwrap_or_else(|_| "{}".to_string())
    }

    /// Parse a session JSON document. Unknown keys are ignored;
    /// invalid entries are skipped individually.
    pub fn from_json_str(s: &str) -> Result<Self, String> {
        let v: Value = serde_json::from_str(s).map_err(|e| e.to_string())?;
        let mut out = Self::default();

        out.firmware_path = v
            .get("firmware_path")
            .and_then(Value::as_str)
            .map(PathBuf::from);
        out.palette = v
            .get("palette")
            .and_then(Value::as_str)
            .map(|s| s.to_string());

        if let Some(arr) = v.get("breakpoints").and_then(Value::as_array) {
            for entry in arr {
                if let Some(s) = entry.as_str() {
                    if let Some(addr) = parse_hex(s) {
                        out.breakpoints.push(addr);
                    }
                }
            }
        }
        if let Some(arr) = v.get("watchpoints").and_then(Value::as_array) {
            for entry in arr {
                let Some(obj) = entry.as_object() else {
                    continue;
                };
                let addr = obj
                    .get("addr")
                    .and_then(Value::as_str)
                    .and_then(parse_hex);
                let size = obj.get("size").and_then(Value::as_u64).map(|v| v as u32);
                let mode = obj
                    .get("mode")
                    .and_then(Value::as_str)
                    .and_then(watch_mode_from_str);
                if let (Some(addr), Some(size), Some(mode)) = (addr, size, mode) {
                    out.watchpoints.push(Watchpoint { addr, size, mode });
                }
            }
        }
        if let Some(ann) = v.get("annotations").and_then(Value::as_object) {
            if let Some(arr) = ann.get("symbols").and_then(Value::as_array) {
                for entry in arr {
                    if let Some(pair) = entry.as_array() {
                        if pair.len() >= 2 {
                            let addr = pair[0].as_str().and_then(parse_hex);
                            let name = pair[1].as_str();
                            if let (Some(addr), Some(name)) = (addr, name) {
                                out.symbols.insert(addr, name.to_string());
                            }
                        }
                    }
                }
            }
            if let Some(arr) = ann.get("comments").and_then(Value::as_array) {
                for entry in arr {
                    if let Some(pair) = entry.as_array() {
                        if pair.len() >= 2 {
                            let addr = pair[0].as_str().and_then(parse_hex);
                            let text = pair[1].as_str();
                            if let (Some(addr), Some(text)) = (addr, text) {
                                out.comments.insert(addr, text.to_string());
                            }
                        }
                    }
                }
            }
            if let Some(arr) = ann.get("regions").and_then(Value::as_array) {
                for entry in arr {
                    if let Some(tup) = entry.as_array() {
                        if tup.len() >= 3 {
                            let label = tup[0].as_str().map(|s| s.to_string());
                            let start = tup[1].as_str().and_then(parse_hex);
                            let end = tup[2].as_str().and_then(parse_hex);
                            if let (Some(label), Some(start), Some(end)) = (label, start, end) {
                                out.regions.push((label, start, end));
                            }
                        }
                    }
                }
            }
        }
        if let Some(view) = v.get("view").and_then(Value::as_object) {
            out.disasm_view_base = view
                .get("disasm_view_base")
                .and_then(Value::as_str)
                .and_then(parse_hex);
            out.disasm_follow_pc = view.get("disasm_follow_pc").and_then(Value::as_bool);
            out.memory_cursor = view
                .get("memory_cursor")
                .and_then(Value::as_str)
                .and_then(parse_hex);
        }
        Ok(out)
    }
}

fn parse_hex(s: &str) -> Option<u32> {
    let t = s.trim().trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(t, 16).ok()
}

fn watch_mode_to_str(mode: WatchMode) -> &'static str {
    match mode {
        WatchMode::Read => "Read",
        WatchMode::Write => "Write",
        WatchMode::ReadWrite => "ReadWrite",
    }
}

fn watch_mode_from_str(s: &str) -> Option<WatchMode> {
    match s {
        "Read" => Some(WatchMode::Read),
        "Write" => Some(WatchMode::Write),
        "ReadWrite" | "Read/Write" => Some(WatchMode::ReadWrite),
        _ => None,
    }
}
