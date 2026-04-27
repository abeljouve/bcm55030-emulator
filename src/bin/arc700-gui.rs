//! `arc700-gui` — GUI binary. Always launches the egui/eframe
//! window and the integrated MCP server. The firmware is loaded
//! from inside the GUI via the "Load firmware…" toolbar button —
//! no command-line positional argument is taken.

use std::process;
use std::sync::mpsc;
use std::sync::Arc;
use std::thread;

use parking_lot::{Mutex, RwLock};

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::emu::command::CpuCommand;
use bcm55030_emulator::emu::{
    cpu_worker, Annotations, EmulatorHandle, EmulatorSnapshot, EventLog, McpStatus,
};
use bcm55030_emulator::soc::bank::BootMode;

const DEFAULT_MCP_PORT: u16 = 3000;

fn usage(prog: &str) {
    eprintln!("BCM55030 ARC 700 Emulator — GUI");
    eprintln!();
    eprintln!("Usage: {prog} [OPTIONS]");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --mcp-port <PORT>   TCP port for the MCP server (default: {DEFAULT_MCP_PORT}, 0 = random)");
    eprintln!("  --no-mcp            Disable the integrated MCP server");
    eprintln!("  --cold-boot         Use cold-boot mode on reset (default: warm)");
    eprintln!("  --olt-enable                Enable EPON OLT emulation");
    eprintln!("  --olt-mac <MAC>             OLT MAC address (AA:BB:CC:DD:EE:FF)");
    eprintln!("  --verbose, -v       Enable [BCM55030] / [Hook] stderr trace");
    eprintln!("  --help, -h          This help");
    eprintln!();
    eprintln!("Firmware is loaded from inside the GUI via the toolbar.");
}

struct GuiConfig {
    mcp_enabled: bool,
    mcp_port: u16,
    boot_mode: BootMode,
    olt_enable: bool,
    olt_mac: Option<[u8; 6]>,
}

fn parse_args() -> GuiConfig {
    let args: Vec<String> = std::env::args().collect();
    let prog = args.first().map(String::as_str).unwrap_or("arc700-gui");
    let mut cfg = GuiConfig {
        mcp_enabled: true,
        mcp_port: DEFAULT_MCP_PORT,
        boot_mode: BootMode::Warm,
        olt_enable: false,
        olt_mac: None,
    };
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--mcp-port" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --mcp-port requires a number");
                    process::exit(1);
                }
                cfg.mcp_port = args[i].parse().unwrap_or_else(|_| {
                    eprintln!("Error: invalid port: {}", args[i]);
                    process::exit(1);
                });
            }
            "--no-mcp" => cfg.mcp_enabled = false,
            "--cold-boot" => cfg.boot_mode = BootMode::Cold,
            "--warm-boot" => cfg.boot_mode = BootMode::Warm,
            "--olt-enable" => cfg.olt_enable = true,
            "--olt-mac" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --olt-mac requires AA:BB:CC:DD:EE:FF");
                    process::exit(1);
                }
                let mac_str = &args[i];
                let parts: Vec<&str> = mac_str.split(':').collect();
                if parts.len() == 6 {
                    let mut mac = [0u8; 6];
                    let mut ok = true;
                    for (j, p) in parts.iter().enumerate() {
                        match u8::from_str_radix(p, 16) {
                            Ok(v) => mac[j] = v,
                            Err(_) => { ok = false; break; }
                        }
                    }
                    if ok {
                        cfg.olt_mac = Some(mac);
                    } else {
                        eprintln!("Error: invalid MAC: {mac_str}");
                        process::exit(1);
                    }
                } else {
                    eprintln!("Error: invalid MAC format: {mac_str}");
                    process::exit(1);
                }
            }
            "--verbose" | "-v" => bcm55030_emulator::set_verbose(true),
            "--help" | "-h" => {
                usage(prog);
                process::exit(0);
            }
            other => {
                eprintln!("Error: unknown option: {other}");
                usage(prog);
                process::exit(1);
            }
        }
        i += 1;
    }
    cfg
}

fn main() {
    let cfg = parse_args();

    // Fresh BCM55030 — no flash loaded. The CPU sits at PC=0 with
    // a zeroed SRAM. The first "Load firmware…" from the GUI fills
    // the SPI flash peripheral and triggers the 64 KB DMA copy.
    let cpu = Cpu::new_bcm55030(cfg.boot_mode);
    let bank = cpu
        .bank()
        .cloned()
        .expect("BCM55030 Cpu must have a peripheral bank");

    // The GUI owns the UART terminal panel, so the worker must
    // not mirror the TX log to the host stdout. Disable the
    // passthrough up-front; `reset_soc_in_place` preserves the
    // bank Arc so this setting survives across reset /
    // load_firmware.
    bank.write().uart.stdout_passthrough = false;

    if cfg.olt_enable {
        let mut b = bank.write();
        b.olt.set_enabled(true);
        if let Some(mac) = cfg.olt_mac {
            b.olt.config.mac = mac;
        }
        b.olt.set_link_up(true);
    }

    let (cmd_tx, cmd_rx) = mpsc::channel::<CpuCommand>();
    let uart_tx = bank.read().uart_rx_sender();
    let peripherals = bank.read().snapshot_all();

    let mut snap = EmulatorSnapshot::placeholder(cfg.boot_mode);
    snap.peripherals = peripherals;

    let handle = EmulatorHandle {
        bank,
        snapshot: Arc::new(Mutex::new(snap)),
        cpu_cmd: cmd_tx,
        uart_tx,
        annotations: Arc::new(RwLock::new(Annotations::new())),
        event_log: Arc::new(Mutex::new(EventLog::default())),
        mcp_status: Arc::new(Mutex::new(McpStatus::default())),
        firmware_info: Arc::new(Mutex::new(None)),
    };

    let handle_for_worker = handle.clone();
    let boot_mode = cfg.boot_mode;
    let worker = thread::Builder::new()
        .name("arc700-cpu-worker".to_string())
        .spawn(move || {
            cpu_worker::run(
                cpu,
                handle_for_worker,
                cmd_rx,
                Box::new(move |_| Cpu::new_bcm55030(boot_mode)),
            );
        })
        .expect("spawn cpu worker");

    let _mcp_thread = if cfg.mcp_enabled {
        Some(bcm55030_emulator::mcp::spawn_server(
            handle.clone(),
            cfg.mcp_port,
        ))
    } else {
        None
    };

    if let Err(e) = bcm55030_emulator::ui::run(handle.clone()) {
        eprintln!("[ui] eframe error: {e}");
    }

    let _ = handle.cpu_cmd.send(CpuCommand::Shutdown);
    let _ = worker.join();
}
