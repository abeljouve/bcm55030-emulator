//! User-loaded emulator annotations. Never preseeded by the binary
//! (the contributor guide). Loaded from JSON at runtime via an MCP tool or a
//! UI file dialog.

use std::collections::HashMap;

/// Annotation bundle owned by `EmulatorHandle`.
#[derive(Default, Clone, Debug)]
pub struct Annotations {
    /// Address → symbolic name. Rendered next to operands in the
    /// disassembly panel and used by MCP `disassemble` to rewrite
    /// branch targets.
    pub symbols: HashMap<u32, String>,

    /// Address → free-form comment. Displayed in the disassembly
    /// panel as a trailing `; comment` annotation.
    pub comments: HashMap<u32, String>,

    /// Named memory regions: `(label, start, end)`. Rendered as
    /// coloured strips in the memory viewer offset gutter.
    pub regions: Vec<(String, u32, u32)>,
}

impl Annotations {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn clear(&mut self) {
        self.symbols.clear();
        self.comments.clear();
        self.regions.clear();
    }

    /// Serialise to the `{"symbols":[[addr,name],...]}` JSON shape.
    /// Only available under the `ui` feature since the GUI is the
    /// sole consumer today. The MCP `add_symbol`/`add_comment`
    /// tools operate on individual entries and do not need this.
    #[cfg(feature = "ui")]
    pub fn to_json_string(&self) -> String {
        use serde_json::{json, Value};
        let symbols: Vec<Value> = self
            .symbols
            .iter()
            .map(|(addr, name)| json!([addr, name]))
            .collect();
        let comments: Vec<Value> = self
            .comments
            .iter()
            .map(|(addr, text)| json!([addr, text]))
            .collect();
        let regions: Vec<Value> = self
            .regions
            .iter()
            .map(|(label, start, end)| json!([label, start, end]))
            .collect();
        serde_json::to_string_pretty(&json!({
            "symbols": symbols,
            "comments": comments,
            "regions": regions,
        }))
        .unwrap_or_else(|_| "{}".to_string())
    }

    /// Parse a file produced by [`to_json_string`]. Unknown keys
    /// are ignored; malformed entries are skipped individually.
    #[cfg(feature = "ui")]
    pub fn from_json_str(s: &str) -> Result<Self, String> {
        use serde_json::Value;
        let v: Value = serde_json::from_str(s).map_err(|e| e.to_string())?;
        let mut out = Self::new();
        if let Some(arr) = v.get("symbols").and_then(Value::as_array) {
            for entry in arr {
                if let Some(pair) = entry.as_array() {
                    if pair.len() >= 2 {
                        if let (Some(addr), Some(name)) =
                            (pair[0].as_u64(), pair[1].as_str())
                        {
                            out.symbols.insert(addr as u32, name.to_string());
                        }
                    }
                }
            }
        }
        if let Some(arr) = v.get("comments").and_then(Value::as_array) {
            for entry in arr {
                if let Some(pair) = entry.as_array() {
                    if pair.len() >= 2 {
                        if let (Some(addr), Some(text)) =
                            (pair[0].as_u64(), pair[1].as_str())
                        {
                            out.comments.insert(addr as u32, text.to_string());
                        }
                    }
                }
            }
        }
        if let Some(arr) = v.get("regions").and_then(Value::as_array) {
            for entry in arr {
                if let Some(tuple) = entry.as_array() {
                    if tuple.len() >= 3 {
                        if let (Some(label), Some(start), Some(end)) = (
                            tuple[0].as_str(),
                            tuple[1].as_u64(),
                            tuple[2].as_u64(),
                        ) {
                            out.regions
                                .push((label.to_string(), start as u32, end as u32));
                        }
                    }
                }
            }
        }
        Ok(out)
    }
}
