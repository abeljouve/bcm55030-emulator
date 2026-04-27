//! DWARF source-line lookup for the `--debug-elf` overlay.
//!
//! Loads an ELF, extracts the DWARF debug sections, and builds a
//! `BTreeMap<u32, SourceLocation>` keyed by PC. The consumer (trace
//! output, UI disassembly, MCP `get_source_coverage`) then looks up
//! each PC against this map to annotate hot paths.
//!
//! Only the `.debug_line` program is parsed — not type info, not
//! local variables. For phase H we just need "what Rust source line
//! produced this instruction".

use std::collections::BTreeMap;
use std::path::Path;

use gimli::{EndianSlice, RunTimeEndian};
use object::{Object, ObjectSection};

/// One row from the DWARF line table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceLocation {
    /// Source file path as recorded in DWARF (may be absolute or
    /// relative to the compilation directory).
    pub file: String,
    /// 1-based line number.
    pub line: u32,
    /// Column number, 0 if unknown.
    pub column: u32,
}

/// Fully-materialised debug index. Keyed by PC — each entry marks the
/// start of a statement for that source location. Consecutive PCs
/// between two entries share the previous entry's source location.
pub struct DebugInfo {
    /// `pc → source location` for every row in the DWARF line table.
    pub rows: BTreeMap<u32, SourceLocation>,
}

impl DebugInfo {
    /// Parse an ELF file and build the PC → source index. Returns
    /// `Err` on any I/O or DWARF error with a human-readable message.
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let path = path.as_ref();
        let bytes = std::fs::read(path).map_err(|e| format!("read {}: {e}", path.display()))?;
        Self::parse(&bytes)
    }

    /// Parse DWARF line tables from an already-loaded ELF byte slice.
    pub fn parse(bytes: &[u8]) -> Result<Self, String> {
        let file = object::File::parse(bytes).map_err(|e| format!("parse object: {e}"))?;

        let endian = if file.is_little_endian() {
            RunTimeEndian::Little
        } else {
            RunTimeEndian::Big
        };

        let load_section = |id: gimli::SectionId| -> Result<EndianSlice<'_, RunTimeEndian>, String> {
            let name = id.name();
            let data = match file.section_by_name(name) {
                Some(section) => section
                    .uncompressed_data()
                    .map_err(|e| format!("decompress {name}: {e}"))?,
                None => std::borrow::Cow::Borrowed(&[][..]),
            };
            // SAFETY of the extend here: `file` owns `bytes` for the
            // duration of `parse`, and the returned `EndianSlice`
            // points back into it. We leak the Cow into a Vec if it
            // was Owned so the returned slice stays live — but for
            // our uncompressed case the slice IS `bytes` itself.
            let leaked: &'static [u8] = Box::leak(data.into_owned().into_boxed_slice());
            Ok(EndianSlice::new(leaked, endian))
        };

        let dwarf = gimli::Dwarf::load(load_section).map_err(|e| format!("dwarf load: {e}"))?;

        let mut rows = BTreeMap::new();

        let mut iter = dwarf.units();
        while let Some(header) = iter
            .next()
            .map_err(|e| format!("next unit: {e}"))?
        {
            let unit = dwarf
                .unit(header)
                .map_err(|e| format!("unit: {e}"))?;

            let program = match unit.line_program.clone() {
                Some(p) => p,
                None => continue,
            };

            let comp_dir = unit
                .comp_dir
                .as_ref()
                .map(|d| d.to_string_lossy().into_owned())
                .unwrap_or_default();

            let (lines, sequences) = program
                .sequences()
                .map_err(|e| format!("line program sequences: {e}"))?;

            for sequence in &sequences {
                let mut state_machine = lines.resume_from(sequence);
                while let Some((header, row)) = state_machine
                    .next_row()
                    .map_err(|e| format!("next row: {e}"))?
                {
                    if row.end_sequence() {
                        continue;
                    }
                    let pc = row.address() as u32;
                    let file = match row.file(header) {
                        Some(entry) => {
                            let mut full = String::new();
                            if let Some(dir_attr) = entry.directory(header) {
                                if let Ok(s) = dwarf.attr_string(&unit, dir_attr) {
                                    let dir = s.to_string_lossy().into_owned();
                                    if !dir.is_empty() {
                                        if dir.starts_with('/') || comp_dir.is_empty() {
                                            full.push_str(&dir);
                                        } else {
                                            full.push_str(&comp_dir);
                                            if !comp_dir.ends_with('/') {
                                                full.push('/');
                                            }
                                            full.push_str(&dir);
                                        }
                                        full.push('/');
                                    }
                                }
                            }
                            if let Ok(s) = dwarf.attr_string(&unit, entry.path_name()) {
                                full.push_str(&s.to_string_lossy());
                            }
                            full
                        }
                        None => "<unknown>".to_string(),
                    };
                    let line = row.line().map(|l| l.get() as u32).unwrap_or(0);
                    let column = match row.column() {
                        gimli::ColumnType::Column(c) => c.get() as u32,
                        gimli::ColumnType::LeftEdge => 0,
                    };
                    rows.insert(
                        pc,
                        SourceLocation {
                            file,
                            line,
                            column,
                        },
                    );
                }
            }
        }

        Ok(Self { rows })
    }

    /// Look up the source location for `pc`. Returns the entry at the
    /// largest `pc' ≤ pc`, i.e. the statement containing `pc`.
    pub fn lookup(&self, pc: u32) -> Option<&SourceLocation> {
        self.rows.range(..=pc).next_back().map(|(_, loc)| loc)
    }

    /// Number of distinct PCs in the index.
    pub fn len(&self) -> usize {
        self.rows.len()
    }

    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }
}
