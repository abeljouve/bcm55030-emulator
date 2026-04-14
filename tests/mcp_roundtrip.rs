//! Phase 4 — MCP server roundtrip.
//!
//! Spawns the integrated rmcp + axum server on an ephemeral port,
//! connects an in-process rmcp client to it, and drives every
//! read-only tool shipped in phase 4. Gated behind the `mcp`
//! cargo feature so the default build stays free of the rmcp /
//! axum / tokio dep tree.

#![cfg(feature = "mcp")]

use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::{Mutex, RwLock};

use bcm55030_emulator::emu::command::CpuCommand;
use bcm55030_emulator::emu::{
    Annotations, EmulatorHandle, EmulatorSnapshot, EventLog, McpStatus,
};
use bcm55030_emulator::mcp;
use bcm55030_emulator::soc::bank::{BootMode, PeripheralBank};

use rmcp::model::CallToolRequestParams;
use rmcp::transport::StreamableHttpClientTransport;
use rmcp::ServiceExt;

/// Build a minimal `EmulatorHandle` for the test: real
/// `PeripheralBank`, placeholder snapshot populated with a live
/// peripheral projection, empty annotations / event log.
fn build_handle() -> EmulatorHandle {
    let bank = Arc::new(RwLock::new(PeripheralBank::new(BootMode::Warm)));
    let peripherals = bank.read().snapshot_all();
    let mut snap = EmulatorSnapshot::placeholder(BootMode::Warm);
    snap.peripherals = peripherals;
    let (cmd_tx, _cmd_rx) = mpsc::channel::<CpuCommand>();
    let (uart_tx, _uart_rx) = mpsc::channel::<u8>();

    EmulatorHandle {
        bank,
        snapshot: Arc::new(Mutex::new(snap)),
        cpu_cmd: cmd_tx,
        uart_tx,
        annotations: Arc::new(RwLock::new(Annotations::new())),
        event_log: Arc::new(Mutex::new(EventLog::default())),
        mcp_status: Arc::new(Mutex::new(McpStatus::default())),
        firmware_info: Arc::new(Mutex::new(None)),
    }
}

/// Block until the worker thread has populated `mcp_status.listening`
/// with the real bound address. Returns the `host:port` string.
async fn wait_for_server(handle: &EmulatorHandle) -> String {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(addr) = handle.mcp_status.lock().listening.clone() {
            return addr;
        }
        if Instant::now() > deadline {
            panic!("MCP server did not start within 3 s");
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mcp_roundtrip_read_only_tools() {
    let handle = build_handle();
    let _server = mcp::spawn_server(handle.clone(), 0);
    let addr = wait_for_server(&handle).await;
    let uri = format!("http://{}/mcp", addr);
    let transport = StreamableHttpClientTransport::from_uri(uri.as_str());
    let client = ()
        .serve(transport)
        .await
        .expect("rmcp client initialize");

    // 1. list_tools: the 11 phase-4 read-only tools must all be advertised.
    let listed = client.list_tools(None).await.expect("list_tools");
    let names: Vec<String> = listed.tools.iter().map(|t| t.name.to_string()).collect();
    for expected in [
        "get_firmware_info",
        "read_registers",
        "read_flags",
        "get_cpu_state",
        "list_peripherals",
        "peek_mmio",
        "list_breakpoints",
        "list_symbols",
        "get_uart_buffer",
        "read_flash",
    ] {
        assert!(
            names.iter().any(|n| n == expected),
            "list_tools missing {expected}: {names:?}"
        );
    }

    // 2. get_firmware_info: loaded=false, no path.
    let res = client
        .call_tool(CallToolRequestParams::new("get_firmware_info"))
        .await
        .expect("get_firmware_info");
    let text = render_result(&res);
    assert!(text.contains("\"loaded\":false"), "fw info: {}", text);

    // 3. read_registers (no names → full dump includes pc).
    let res = client
        .call_tool(CallToolRequestParams::new("read_registers"))
        .await
        .expect("read_registers");
    let text = render_result(&res);
    assert!(text.contains("\"pc\""), "read_registers: {}", text);
    assert!(text.contains("\"status32\""));

    // 4. peek_mmio 0x01000000 → CHIP_ID 0x47010203.
    let peek_args = serde_json::json!({ "address": 0x01000000u32 })
        .as_object()
        .unwrap()
        .clone();
    let res = client
        .call_tool(CallToolRequestParams::new("peek_mmio").with_arguments(peek_args))
        .await
        .expect("peek_mmio");
    let text = render_result(&res);
    // CHIP_ID = 0x47010203 = 1_191_248_387 decimal (JSON serialises u32 as a
    // base-10 number).
    assert!(
        text.contains("1191248387"),
        "peek_mmio CHIP_ID: {}",
        text
    );

    // 5. list_peripherals: 12 rows with a `sfp` entry at index 3.
    let res = client
        .call_tool(CallToolRequestParams::new("list_peripherals"))
        .await
        .expect("list_peripherals");
    let text = render_result(&res);
    assert!(text.contains("\"sfp\""), "list_peripherals missing sfp: {}", text);
    assert!(text.contains("\"uart\""));
    assert!(text.contains("\"epon_mac\""));

    // 6. read_flash: return first 16 bytes (all zero on a fresh bank).
    let flash_args = serde_json::json!({ "offset": 0u32, "length": 16u32 })
        .as_object()
        .unwrap()
        .clone();
    let res = client
        .call_tool(CallToolRequestParams::new("read_flash").with_arguments(flash_args))
        .await
        .expect("read_flash");
    let text = render_result(&res);
    assert!(text.contains("\"length\":16"), "read_flash: {}", text);

    client.cancel().await.ok();
}

/// rmcp's `CallToolResult.content` is a vec of tagged content
/// blocks. For JSON tool outputs the server packs the serialized
/// response in `content[0].text`. Collapse that down to a single
/// string for grep-style assertions.
fn render_result(res: &rmcp::model::CallToolResult) -> String {
    let mut out = String::new();
    for block in &res.content {
        if let Some(text) = block.as_text() {
            out.push_str(&text.text);
            out.push('\n');
        }
    }
    if let Some(structured) = &res.structured_content {
        out.push_str(&serde_json::to_string(structured).unwrap_or_default());
    }
    out
}
