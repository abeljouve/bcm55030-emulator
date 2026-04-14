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
    // Use the bank's own UART RX mpsc sender — its receiver
    // lives inside the bank, so bytes pushed by the MCP
    // `send_uart_input` tool have a consumer for the full
    // duration of the test (the bank outlives the handle).
    let uart_tx = bank.read().uart_rx_sender();

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

    // ---------- Phase 5a mutation tools ----------------------------------

    // 7. add_symbol → list_symbols reflects it.
    let sym_args = serde_json::json!({
        "address": 0x0150u32,
        "name": "boot_main"
    })
    .as_object()
    .unwrap()
    .clone();
    client
        .call_tool(CallToolRequestParams::new("add_symbol").with_arguments(sym_args))
        .await
        .expect("add_symbol");
    let res = client
        .call_tool(CallToolRequestParams::new("list_symbols"))
        .await
        .expect("list_symbols");
    let text = render_result(&res);
    assert!(
        text.contains("boot_main"),
        "add_symbol round-trip: {}",
        text
    );

    // 8. write_flash → read_flash round-trip.
    let write_args = serde_json::json!({
        "offset": 0x1000u32,
        "hex": "DEADBEEFCAFEBABE"
    })
    .as_object()
    .unwrap()
    .clone();
    client
        .call_tool(CallToolRequestParams::new("write_flash").with_arguments(write_args))
        .await
        .expect("write_flash");
    let read_args = serde_json::json!({ "offset": 0x1000u32, "length": 8u32 })
        .as_object()
        .unwrap()
        .clone();
    let res = client
        .call_tool(CallToolRequestParams::new("read_flash").with_arguments(read_args))
        .await
        .expect("read_flash after write");
    let text = render_result(&res);
    assert!(
        text.contains("DEADBEEFCAFEBABE"),
        "write/read_flash round-trip: {}",
        text
    );

    // 9. send_uart_input — reports byte count.
    let uart_args = serde_json::json!({ "data": "hi\r\n" })
        .as_object()
        .unwrap()
        .clone();
    let res = client
        .call_tool(
            CallToolRequestParams::new("send_uart_input").with_arguments(uart_args),
        )
        .await
        .expect("send_uart_input");
    let text = render_result(&res);
    assert!(
        text.contains("\"bytes_sent\":4"),
        "send_uart_input: {}",
        text
    );

    // 10. inject_peripheral_event alarm/ForcePending — handler
    //     returns ok:true (the bank's alarm_events peripheral
    //     accepts the event).
    let inject_args = serde_json::json!({
        "peripheral": "alarm",
        "event": "ForcePending",
        "params": { "opcode": 0x1234u32 }
    })
    .as_object()
    .unwrap()
    .clone();
    let res = client
        .call_tool(
            CallToolRequestParams::new("inject_peripheral_event").with_arguments(inject_args),
        )
        .await
        .expect("inject_peripheral_event");
    let text = render_result(&res);
    assert!(text.contains("\"ok\":true"), "inject alarm: {}", text);

    // 11. write_mmio: directly set bsc_i2c raw_store[0x144] via
    //     the bank, then verify it was actually written by
    //     re-reading the bank's register store. Use peek_mmio
    //     for round-trip — bsc_i2c doesn't override peek_word
    //     yet so it returns 0; instead check the change landed
    //     via the `read_mmio`-equivalent path (side-effectful
    //     read_word on bsc_i2c register 0x144).
    let write_args = serde_json::json!({
        "address": 0x0100_0144u32,
        "value": 0x0000_00FFu32,
        "width": "word"
    })
    .as_object()
    .unwrap()
    .clone();
    client
        .call_tool(CallToolRequestParams::new("write_mmio").with_arguments(write_args))
        .await
        .expect("write_mmio");
    // bsc_i2c uses a backing store for 0x144 — read via the
    // direct bank handle to confirm it landed.
    let stored = handle.bank.write().read_word(0x0100_0144).unwrap();
    assert_eq!(stored & 0xFF, 0xFF, "write_mmio round-trip via bank");

    // 12. dump_mmio_trace: tracing disabled (no --dump-mmio-trace).
    let res = client
        .call_tool(CallToolRequestParams::new("dump_mmio_trace"))
        .await
        .expect("dump_mmio_trace");
    let text = render_result(&res);
    assert!(
        text.contains("\"enabled\":false"),
        "dump_mmio_trace: {}",
        text
    );

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
