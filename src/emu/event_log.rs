//! Bounded MCP activity log. Populated by every rmcp tool call and
//! resource read; consumed by the UI `MCP Activity Log` panel.

use std::collections::VecDeque;
use std::time::SystemTime;

/// Direction of a logged interaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    /// Agent → emulator: tool call or resource request.
    Request,
    /// Emulator → agent: tool result or resource payload.
    Response,
}

/// One entry in the activity log.
#[derive(Clone, Debug)]
pub struct EventEntry {
    pub timestamp: SystemTime,
    pub direction: Direction,
    pub tool: String,
    /// Truncated summary of the request parameters (JSON-style).
    pub params: String,
    /// Truncated summary of the response payload.
    pub result: String,
    /// `true` for any tool that mutates state (write_register,
    /// cpu_run, set_breakpoint, load_firmware, …). UI colours these
    /// distinctly from read-only calls.
    pub is_mutation: bool,
}

/// Drop-oldest bounded log.
#[derive(Debug)]
pub struct EventLog {
    entries: VecDeque<EventEntry>,
    capacity: usize,
    /// Number of tool calls currently in-flight (request logged,
    /// response not yet). Drives the toolbar spinner.
    pub in_flight: u32,
}

impl EventLog {
    pub fn new(capacity: usize) -> Self {
        Self {
            entries: VecDeque::with_capacity(capacity.min(4096)),
            capacity,
            in_flight: 0,
        }
    }

    /// Append `entry`, evicting the oldest when full.
    pub fn push(&mut self, entry: EventEntry) {
        if self.entries.len() == self.capacity {
            self.entries.pop_front();
        }
        self.entries.push_back(entry);
    }

    pub fn entries(&self) -> &VecDeque<EventEntry> {
        &self.entries
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn clear(&mut self) {
        self.entries.clear();
        self.in_flight = 0;
    }

    pub fn set_capacity(&mut self, capacity: usize) {
        self.capacity = capacity;
        while self.entries.len() > capacity {
            self.entries.pop_front();
        }
    }
}

impl Default for EventLog {
    fn default() -> Self {
        Self::new(1000)
    }
}
