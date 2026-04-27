//! Streamable HTTP MCP server bootstrap. Spawns a dedicated
//! tokio runtime on its own OS thread, wires the
//! `StreamableHttpService` into an axum router, binds it to the
//! configured port, and runs until the process exits.
//!
//! `EmulatorHandler` clones are cheap (all state lives behind
//! `Arc`s on the shared `EmulatorHandle`), so the tower factory
//! closure can build a fresh handler per session without
//! contention.

use std::net::SocketAddr;
use std::sync::Arc;
use std::thread::JoinHandle;

use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::streamable_http_server::tower::{
    StreamableHttpServerConfig, StreamableHttpService,
};

use crate::emu::{EmulatorHandle, McpStatus};
use crate::mcp::handler::EmulatorHandler;

/// Spawn the MCP server on a new OS thread with a dedicated
/// multi-threaded tokio runtime. Returns the worker `JoinHandle`
/// — callers usually drop it on the floor (the process exits
/// when main returns).
pub fn spawn_server(handle: EmulatorHandle, port: u16) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("arc700-mcp".to_string())
        .spawn(move || run_server(handle, port))
        .expect("failed to spawn MCP server thread")
}

fn run_server(handle: EmulatorHandle, port: u16) {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .thread_name("arc700-mcp-tokio")
        .build()
        .expect("failed to build tokio runtime for MCP server");

    runtime.block_on(async move {
        let session_manager = Arc::new(LocalSessionManager::default());
        let handler_handle = handle.clone();
        let service = StreamableHttpService::new(
            move || Ok(EmulatorHandler::new(handler_handle.clone())),
            session_manager,
            StreamableHttpServerConfig::default(),
        );

        let addr = SocketAddr::from(([127, 0, 0, 1], port));
        let listener = match tokio::net::TcpListener::bind(addr).await {
            Ok(l) => l,
            Err(e) => {
                eprintln!("[mcp] failed to bind {}: {}", addr, e);
                return;
            }
        };
        let real_addr = listener
            .local_addr()
            .map(|a| a.to_string())
            .unwrap_or_else(|_| addr.to_string());
        {
            let mut status = handle.mcp_status.lock();
            *status = McpStatus {
                listening: Some(real_addr.clone()),
                connected_clients: 0,
            };
        }
        eprintln!("[mcp] listening on http://{}", real_addr);

        let router = axum::Router::new().fallback_service(service);
        if let Err(e) = axum::serve(listener, router).await {
            eprintln!("[mcp] server terminated: {}", e);
        }
    });
}
