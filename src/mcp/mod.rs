//! Integrated rmcp + axum HTTP server for the ARC700 emulator.
//!
//! Gated behind the `mcp` cargo feature. Nothing in this module is
//! compiled into the default build.
//!
//! The module layout follows the phase-4 plan
//! (`the design plan`):
//!
//! - `handler` — `EmulatorHandler` struct + `#[tool_router]` +
//!   `#[tool_handler]` implementations. Phase 4 ships the
//!   read-only subset of the design spec §MCP Server §Tools. Mutations
//!   land in phase 5.
//! - `server` — `spawn_server(handle, port)` starts a dedicated
//!   tokio runtime on a new OS thread, wires `StreamableHttpService`
//!   into an axum router, and binds it to the configured port.

pub mod handler;
pub mod server;

pub use handler::EmulatorHandler;
pub use server::spawn_server;
