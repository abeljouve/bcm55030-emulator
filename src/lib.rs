pub mod cache;
pub mod cpu;
pub mod decoder;
pub mod executor;
pub mod hooks;
pub mod memory;
pub mod soc;

use std::sync::atomic::{AtomicBool, Ordering};

/// Global verbose flag — controls debug output ([Hook], [MMIO], [Boot ROM], etc.).
/// Disabled by default; enabled via --verbose CLI flag.
pub static VERBOSE: AtomicBool = AtomicBool::new(false);

/// Returns true when verbose debug output is enabled.
#[inline]
pub fn is_verbose() -> bool {
    VERBOSE.load(Ordering::Relaxed)
}

/// Enable verbose debug output.
pub fn set_verbose(enabled: bool) {
    VERBOSE.store(enabled, Ordering::Relaxed);
}

/// Print to stderr only when verbose mode is enabled.
#[macro_export]
macro_rules! vlog {
    ($($arg:tt)*) => {
        if $crate::is_verbose() {
            eprintln!($($arg)*);
        }
    };
}
