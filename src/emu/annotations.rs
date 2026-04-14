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
}
