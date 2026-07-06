use std::env;
use std::fs;
use std::os::unix::io::AsRawFd;
use std::process;
use std::sync::mpsc;
use std::sync::mpsc::Sender;
use std::sync::Arc;

use parking_lot::{Mutex, RwLock};

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::soc::bank::BootMode;

fn usage(prog: &str) {
    eprintln!("BCM55030 ARC 700 Emulator");
    eprintln!();
    eprintln!("Usage: {} [OPTIONS] <flash.bin>", prog);
    eprintln!();
    eprintln!("  <flash.bin>                 SPI flash image (bootloader or full 4MB dump)");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --entry <ADDR>              Entry point (hex, default: 0x0000)");
    eprintln!("  --max-cycles <N>            Maximum instructions (default: unlimited)");
    eprintln!("  --trace                     Log each instruction to stderr");
    eprintln!("  --trace-mmio                Log MMIO accesses to stderr");
    eprintln!("  --trace-sr                  Log AUX register writes (sr instructions) to stderr");
    eprintln!("  --verbose, -v               Show debug messages ([Hook], [MMIO], [Boot ROM]...)");
    eprintln!("  --break <ADDR>              Stop at address (hex)");
    eprintln!("  --dccm-dump <FILE>          Dump DCCM to file on exit");
    eprintln!("  --persist-flash             Save modified flash to <flash.bin>.persist on exit");
    eprintln!("  --cold-boot                 Start with zero sysreg / no IENABLE preset");
    eprintln!("  --warm-boot                 Start with SYSREG_INIT_VALUES (default)");
    eprintln!("  --dump-mmio-trace-cold <F>  Cold-boot + dump MMIO trace catalog to <F>");
    eprintln!("  --unmapped-exception        Trap unclaimed MMIO as MemoryError (audit 2.2)");
    eprintln!("  --trace-mmio-seq <FILE>     Write per-access MMIO trace as JSON Lines to FILE");
    eprintln!("  --trace-mmio-range S:E      Restrict --trace-mmio-seq to [S,E) (hex, repeatable)");
    eprintln!("  --scenario <FILE>           Load a JSON scenario file at startup");
    eprintln!("  --sfp-eeprom <FILE>         Load SFP EEPROM pages from a raw dump (256B A0h, or 512B A0h+A2h)");
    eprintln!("  --mcp-port <PORT>           Start MCP server on PORT (enables worker mode)");
    eprintln!("  --olt-enable                Enable EPON OLT emulation");
    eprintln!("  --olt-mac <MAC>             OLT MAC address (AA:BB:CC:DD:EE:FF)");
    eprintln!();
    eprintln!("For the egui GUI, build with --features ui:");
    eprintln!("  cargo run --release --features ui --bin arc700-gui");
}

struct Config {
    flash_path: String,
    entry_point: u32,
    max_cycles: u64,
    trace: bool,
    trace_from_insn: Option<u64>,
    trace_mmio: bool,
    trace_sr: bool,
    verbose: bool,
    breakpoint: Option<u32>,
    dccm_dump: Option<String>,
    persist_flash: bool,
    watch_dccm: Option<u32>,
    dump_mmio_trace: Option<String>,
    /// Audit 2.2: trap unclaimed MMIO reads / writes as
    /// `MemoryError` exceptions instead of returning zero.
    unmapped_exception: bool,
    boot_mode: BootMode,
    /// Path to an ELF file carrying DWARF debug info. Used to
    /// annotate `--trace` output with source file / line.
    debug_elf: Option<String>,
    /// `--trace-mmio-seq <file>`: per-access sequential MMIO trace
    /// written as JSON Lines to this path.
    trace_mmio_seq: Option<String>,
    /// `--trace-mmio-range start:end`: restrict `--trace-mmio-seq`
    /// recording to accesses where `start <= addr < end`. Multiple
    /// flags OR together.
    trace_mmio_ranges: Vec<(u32, u32)>,
    scenario_path: Option<String>,
    sfp_eeprom_path: Option<String>,
    mcp_port: Option<u16>,
    olt_enable: bool,
    olt_mac: Option<[u8; 6]>,
}

fn parse_hex(s: &str) -> Option<u32> {
    let s = s.trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(s, 16).ok()
}

fn parse_mac(s: &str) -> Option<[u8; 6]> {
    let parts: Vec<&str> = s.split(':').collect();
    if parts.len() != 6 {
        return None;
    }
    let mut mac = [0u8; 6];
    for (i, part) in parts.iter().enumerate() {
        mac[i] = u8::from_str_radix(part, 16).ok()?;
    }
    Some(mac)
}

fn parse_args() -> Config {
    let args: Vec<String> = env::args().collect();
    let prog = &args[0];

    let mut cfg = Config {
        flash_path: String::new(),
        entry_point: 0,
        max_cycles: u64::MAX,
        trace: false,
        trace_from_insn: None,
        trace_mmio: false,
        trace_sr: false,
        verbose: false,
        breakpoint: None,
        dccm_dump: None,
        persist_flash: false,
        watch_dccm: None,
        dump_mmio_trace: None,
        unmapped_exception: false,
        boot_mode: BootMode::Warm,
        debug_elf: None,
        trace_mmio_seq: None,
        trace_mmio_ranges: Vec::new(),
        scenario_path: None,
        sfp_eeprom_path: None,
        mcp_port: None,
        olt_enable: false,
        olt_mac: None,
    };

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--entry" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --entry requires an address");
                    process::exit(1);
                }
                cfg.entry_point = parse_hex(&args[i]).unwrap_or_else(|| {
                    eprintln!("Error: invalid hex address: {}", args[i]);
                    process::exit(1);
                });
            }
            "--max-cycles" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --max-cycles requires a number");
                    process::exit(1);
                }
                cfg.max_cycles = args[i].parse().unwrap_or_else(|_| {
                    eprintln!("Error: invalid number: {}", args[i]);
                    process::exit(1);
                });
            }
            "--trace" => cfg.trace = true,
            "--trace-from-insn" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --trace-from-insn requires a number");
                    process::exit(1);
                }
                cfg.trace_from_insn = Some(args[i].parse().unwrap_or_else(|_| {
                    eprintln!("Error: invalid number: {}", args[i]);
                    process::exit(1);
                }));
            }
            "--trace-mmio" => cfg.trace_mmio = true,
            "--trace-sr" => cfg.trace_sr = true,
            "--verbose" | "-v" => cfg.verbose = true,
            "--break" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --break requires an address");
                    process::exit(1);
                }
                cfg.breakpoint = Some(parse_hex(&args[i]).unwrap_or_else(|| {
                    eprintln!("Error: invalid hex address: {}", args[i]);
                    process::exit(1);
                }));
            }
            "--dccm-dump" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --dccm-dump requires a file path");
                    process::exit(1);
                }
                cfg.dccm_dump = Some(args[i].clone());
            }
            "--persist-flash" => cfg.persist_flash = true,
            "--watch-dccm" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --watch-dccm requires an address");
                    process::exit(1);
                }
                cfg.watch_dccm = Some(parse_hex(&args[i]).unwrap_or_else(|| {
                    eprintln!("Error: invalid hex address: {}", args[i]);
                    process::exit(1);
                }));
            }
            "--dump-mmio-trace" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --dump-mmio-trace requires a file path");
                    process::exit(1);
                }
                cfg.dump_mmio_trace = Some(args[i].clone());
            }
            "--dump-mmio-trace-cold" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --dump-mmio-trace-cold requires a file path");
                    process::exit(1);
                }
                cfg.dump_mmio_trace = Some(args[i].clone());
                cfg.boot_mode = BootMode::Cold;
            }
            "--unmapped-exception" => cfg.unmapped_exception = true,
            "--cold-boot" => cfg.boot_mode = BootMode::Cold,
            "--warm-boot" => cfg.boot_mode = BootMode::Warm,
            "--debug-elf" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --debug-elf requires a file path");
                    process::exit(1);
                }
                cfg.debug_elf = Some(args[i].clone());
            }
            "--trace-mmio-seq" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --trace-mmio-seq requires a file path");
                    process::exit(1);
                }
                cfg.trace_mmio_seq = Some(args[i].clone());
            }
            "--trace-mmio-range" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --trace-mmio-range requires start:end (hex)");
                    process::exit(1);
                }
                let spec = &args[i];
                let parts: Vec<&str> = spec.splitn(2, ':').collect();
                if parts.len() != 2 {
                    eprintln!("Error: --trace-mmio-range: expected start:end, got {:?}", spec);
                    process::exit(1);
                }
                let start = parse_hex(parts[0]).unwrap_or_else(|| {
                    eprintln!("Error: --trace-mmio-range: invalid start address {:?}", parts[0]);
                    process::exit(1);
                });
                let end = parse_hex(parts[1]).unwrap_or_else(|| {
                    eprintln!("Error: --trace-mmio-range: invalid end address {:?}", parts[1]);
                    process::exit(1);
                });
                cfg.trace_mmio_ranges.push((start, end));
            }
            "--scenario" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --scenario requires a file path");
                    process::exit(1);
                }
                cfg.scenario_path = Some(args[i].clone());
            }
            "--sfp-eeprom" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --sfp-eeprom requires a file path");
                    process::exit(1);
                }
                cfg.sfp_eeprom_path = Some(args[i].clone());
            }
            "--mcp-port" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --mcp-port requires a number");
                    process::exit(1);
                }
                cfg.mcp_port = Some(args[i].parse().unwrap_or_else(|_| {
                    eprintln!("Error: invalid port: {}", args[i]);
                    process::exit(1);
                }));
            }
            "--olt-enable" => cfg.olt_enable = true,
            "--olt-mac" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --olt-mac requires AA:BB:CC:DD:EE:FF");
                    process::exit(1);
                }
                cfg.olt_mac = Some(parse_mac(&args[i]).unwrap_or_else(|| {
                    eprintln!("Error: invalid MAC address: {}", args[i]);
                    process::exit(1);
                }));
            }
            "--help" | "-h" => {
                usage(prog);
                process::exit(0);
            }
            _ => {
                if args[i].starts_with('-') {
                    eprintln!("Error: unknown option: {}", args[i]);
                    usage(prog);
                    process::exit(1);
                }
                cfg.flash_path = args[i].clone();
            }
        }
        i += 1;
    }

    if cfg.flash_path.is_empty() {
        eprintln!("Error: no flash image specified");
        usage(prog);
        process::exit(1);
    }

    cfg
}

const BOOT_DMA_SIZE: usize = 64 * 1024;

fn setup_raw_terminal() -> Option<libc::termios> {
    unsafe {
        let fd = std::io::stdin().as_raw_fd();
        let flags = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);

        let mut orig: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut orig) != 0 {
            return None;
        }
        let mut raw = orig;
        libc::cfmakeraw(&mut raw);
        raw.c_cc[libc::VMIN] = 0;
        raw.c_cc[libc::VTIME] = 0;
        if libc::tcsetattr(fd, libc::TCSANOW, &raw) != 0 {
            return None;
        }
        bcm55030_emulator::vlog!("[BCM55030] Terminal: raw mode (Ctrl-C to exit)");
        Some(orig)
    }
}

fn restore_terminal(orig: &libc::termios) {
    unsafe {
        let fd = std::io::stdin().as_raw_fd();
        let flags = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK);
        libc::tcsetattr(fd, libc::TCSANOW, orig);
    }
}

fn try_read_stdin() -> Option<u8> {
    unsafe {
        let mut buf = [0u8; 1];
        let n = libc::read(
            std::io::stdin().as_raw_fd(),
            buf.as_mut_ptr() as *mut libc::c_void,
            1,
        );
        if n == 1 { Some(buf[0]) } else { None }
    }
}

enum RunResult {
    Reboot,
    UserExit,
    Breakpoint,
    Exception(bcm55030_emulator::cpu::exception::Exception),
    MaxCycles,
}

fn run_emulator(cpu: &mut Cpu, cfg: &Config, uart_tx: &Sender<u8>) -> RunResult {
    let mut trace_armed = cfg.trace_from_insn.is_some();
    for step in 0..cfg.max_cycles {
        if cpu.state.halted {
            return RunResult::Reboot;
        }

        if trace_armed {
            if let Some(threshold) = cfg.trace_from_insn {
                if cpu.state.instruction_count >= threshold {
                    cpu.trace = true;
                    trace_armed = false;
                    eprintln!("[TRACE] Tracing armed at insn={}", cpu.state.instruction_count);
                }
            }
        }

        // Poll stdin every 1024 steps — push bytes into the UART
        // receive channel. No firmware-specific branching: bytes typed
        // during the bootloader are consumed by the bootloader CLI,
        // bytes typed during firmware execution by the firmware CLI. Hardware-faithful.
        if step % 1024 == 0 {
            while let Some(byte) = try_read_stdin() {
                if byte == 3 {
                    eprintln!("\n[BCM55030] Ctrl-C, stopping");
                    cpu.state.halted = true;
                    return RunResult::UserExit;
                }
                let _ = uart_tx.send(byte);
            }
        }

        if let Some(bp) = cfg.breakpoint {
            if cpu.state.pc == bp {
                eprintln!("[BREAK] Hit breakpoint at 0x{:08X}", bp);
                return RunResult::Breakpoint;
            }
        }

        if let Err(e) = cpu.step() {
            return RunResult::Exception(e);
        }
    }
    RunResult::MaxCycles
}

/// HW boot: DMA 64 KB flash → SRAM, start at entry_point.
fn boot_from_flash(cpu: &mut Cpu, entry_point: u32, mode: BootMode) {
    let copy_size = {
        let bank = cpu.bank().unwrap().read();
        bank.pbc.flash.data.len().min(BOOT_DMA_SIZE)
    };
    let code = {
        let bank = cpu.bank().unwrap().read();
        bank.pbc.flash.data[..copy_size].to_vec()
    };
    cpu.mem.load_binary(0, &code);
    // DATASHEET §5.4: DMA-originated SRAM writes broadcast I/D-cache
    // invalidations. The boot DMA is one such write; invalidate so a
    // reboot (`boot_from_flash` reused) cannot fetch stale `0x0..0x80`.
    cpu.mem.dcache_invalidate_all().ok();
    cpu.mem.icache_invalidate_all();
    bcm55030_emulator::vlog!(
        "[BCM55030] HW DMA: copied {} bytes (0x{:X}) from flash to SRAM",
        copy_size, copy_size
    );

    // Audit 1.1 revised after D6 diagnosis: `aux_ienable` is preset
    // unconditionally because the BCM55030 silicon appears to reset
    // the IRQ enable mask to `0xFFFFFFFF` — the firmware never
    // programs it explicitly and the bootloader's TX path depends
    // on IRQ 5 firing once STATUS32 bits E1 / E2 are set. E1 / E2
    // themselves still only ship pre-set in warm mode because the
    // firmware's FLAG instruction at `uart_enable_interrupts`
    // (runtime `0x59F4`) drives them in both warm and cold.
    cpu.state.aux_ienable = 0xFFFFFFFF;
    if mode == BootMode::Warm {
        cpu.state.flag_e1 = true;
        cpu.state.flag_e2 = true;
    }

    cpu.state.pc = entry_point;
    bcm55030_emulator::vlog!(
        "[BCM55030] Entry: 0x{:08X}, SRAM=512KB, boot DMA=64KB, mode={:?}",
        entry_point, mode
    );
}

fn reset_cpu_for_reboot(cpu: &mut Cpu) {
    use bcm55030_emulator::cpu::registers::CpuState;

    let saved_timer1_irq = cpu.state.timer1_irq;
    cpu.state = CpuState::new();
    cpu.state.timer1_irq = saved_timer1_irq;

    let sram_size = cpu.mem.sram_size();
    let zeros = vec![0u8; sram_size];
    cpu.mem.load_binary(0, &zeros);

    if let Some(bank) = cpu.bank() {
        // Cold reset on reboot — the FLAG 1 chip reset is a hardware
        // event, so all volatile peripheral state zeroes out. Flash
        // contents persist (they live inside the PBC's `SpiFlash`
        // which preserves `data` across `reset_cold`).
        bank.write().reset_cold();
    }
}

fn run_with_mcp(cpu: Cpu, cfg: &Config, port: u16) {
    use bcm55030_emulator::emu::command::CpuCommand;
    use bcm55030_emulator::emu::{
        Annotations, EmulatorHandle, EmulatorSnapshot, EventLog, McpStatus,
    };

    let bank = cpu
        .bank()
        .cloned()
        .expect("BCM55030 Cpu must have a peripheral bank");

    let (cmd_tx, cmd_rx) = mpsc::channel::<CpuCommand>();
    let uart_tx = bank.read().uart_rx_sender();
    let peripherals = bank.read().snapshot_all();

    let mut snap = EmulatorSnapshot::placeholder(cfg.boot_mode);
    snap.peripherals = peripherals;

    let handle = EmulatorHandle {
        bank,
        snapshot: Arc::new(Mutex::new(snap)),
        cpu_cmd: cmd_tx,
        uart_tx: uart_tx.clone(),
        annotations: Arc::new(RwLock::new(Annotations::new())),
        event_log: Arc::new(Mutex::new(EventLog::default())),
        mcp_status: Arc::new(Mutex::new(McpStatus::default())),
        firmware_info: Arc::new(Mutex::new(None)),
    };

    let handle_for_worker = handle.clone();
    let boot_mode = cfg.boot_mode;
    let worker = std::thread::Builder::new()
        .name("arc700-cpu-worker".to_string())
        .spawn(move || {
            bcm55030_emulator::emu::cpu_worker::run(
                cpu,
                handle_for_worker,
                cmd_rx,
                Box::new(move |_| Cpu::new_bcm55030(boot_mode)),
            );
        })
        .expect("spawn cpu worker");

    let _mcp_thread = bcm55030_emulator::mcp::spawn_server(handle.clone(), port);

    if let Some(bp) = cfg.breakpoint {
        let (tx, _rx) = bcm55030_emulator::emu::command::oneshot();
        let _ = handle.cpu_cmd.send(CpuCommand::SetBreakpoint { address: bp, response: tx });
    }

    let max = if cfg.max_cycles == u64::MAX { None } else { Some(cfg.max_cycles) };
    let _ = handle.cpu_cmd.send(CpuCommand::Run { max_insns: max });

    eprintln!("[arc700] MCP server on port {port}, worker running");
    eprintln!("[arc700] Press Ctrl-C to stop");

    let orig_termios = setup_raw_terminal();

    loop {
        while let Some(byte) = try_read_stdin() {
            if byte == 3 {
                eprintln!("\n[arc700] Ctrl-C, shutting down");
                let _ = handle.cpu_cmd.send(CpuCommand::Shutdown);
                let _ = worker.join();
                if let Some(ref orig) = orig_termios {
                    restore_terminal(orig);
                }
                return;
            }
            let _ = uart_tx.send(byte);
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn main() {
    let cfg = parse_args();
    bcm55030_emulator::set_verbose(cfg.verbose);

    let persist_path = format!("{}.persist", cfg.flash_path);
    let (flash_data, using_persisted) = if cfg.persist_flash {
        match fs::read(&persist_path) {
            Ok(b) => {
                bcm55030_emulator::vlog!("[BCM55030] Loading persisted flash: {}", persist_path);
                (b, true)
            }
            Err(_) => {
                let data = match fs::read(&cfg.flash_path) {
                    Ok(b) => b,
                    Err(e) => {
                        eprintln!("Error reading {}: {}", cfg.flash_path, e);
                        process::exit(1);
                    }
                };
                (data, false)
            }
        }
    } else {
        let data = match fs::read(&cfg.flash_path) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("Error reading {}: {}", cfg.flash_path, e);
                process::exit(1);
            }
        };
        (data, false)
    };

    bcm55030_emulator::vlog!(
        "[BCM55030] Flash image: {} ({} bytes{})",
        cfg.flash_path,
        flash_data.len(),
        if using_persisted { ", persisted state" } else { "" }
    );

    let mut cpu = Cpu::new_bcm55030(cfg.boot_mode);

    // --- Acquire UART input sender for stdin → UART wiring ---
    let uart_tx = cpu.bank().unwrap().read().uart_rx_sender();

    // --- Load flash image into SPI flash peripheral ---
    {
        let mut bank = cpu.bank().unwrap().write();
        let flash_size = bank.pbc.flash.data.len();
        let copy_len = flash_data.len().min(flash_size);
        bank.pbc.flash.data[..copy_len].copy_from_slice(&flash_data[..copy_len]);
        bcm55030_emulator::vlog!("[BCM55030] SPI flash: {} bytes loaded", copy_len);
    }

    boot_from_flash(&mut cpu, cfg.entry_point, cfg.boot_mode);

    cpu.trace = cfg.trace;
    cpu.trace_sr = cfg.trace_sr;
    cpu.mem.dccm_watchpoint = cfg.watch_dccm;
    if let Some(ref path) = cfg.debug_elf {
        match bcm55030_emulator::debug_info::DebugInfo::load(path) {
            Ok(di) => {
                bcm55030_emulator::vlog!(
                    "[BCM55030] DWARF debug info loaded: {} entries from {}",
                    di.len(),
                    path
                );
                cpu.debug_info = Some(di);
            }
            Err(e) => {
                eprintln!("Error loading --debug-elf {}: {}", path, e);
                process::exit(1);
            }
        }
    }
    if cfg.trace_mmio {
        let mut bank = cpu.bank().unwrap().write();
        bank.trace = true;
        bank.sysreg.trace = true;
        bank.pbc.trace = true;
    }
    if cfg.dump_mmio_trace.is_some() {
        let mut bank = cpu.bank().unwrap().write();
        bank.sysreg.mmio_trace = Some(std::collections::HashMap::new());
    }
    if cfg.unmapped_exception {
        cpu.bank().unwrap().write().unmapped_exception = true;
        bcm55030_emulator::vlog!("[BCM55030] Unmapped-access policy = Exception (audit 2.2)");
    }
    if let Some(ref path) = cfg.trace_mmio_seq {
        match bcm55030_emulator::soc::bank::SeqTrace::open(path, cfg.trace_mmio_ranges.clone()) {
            Ok(st) => {
                cpu.bank().unwrap().write().seq_trace = Some(st);
                bcm55030_emulator::vlog!(
                    "[BCM55030] Sequential MMIO trace → {} (ranges: {})",
                    path,
                    if cfg.trace_mmio_ranges.is_empty() {
                        "all".to_string()
                    } else {
                        cfg.trace_mmio_ranges
                            .iter()
                            .map(|(s, e)| format!("0x{:X}..0x{:X}", s, e))
                            .collect::<Vec<_>>()
                            .join(", ")
                    }
                );
            }
            Err(e) => {
                eprintln!("Error opening --trace-mmio-seq {}: {}", path, e);
                process::exit(1);
            }
        }
    }

    if let Some(ref path) = cfg.scenario_path {
        let mut bank = cpu.bank().unwrap().write();
        match bank.scenario.load_file(std::path::Path::new(path)) {
            Ok(n) => eprintln!("[scenario] loaded {} entries from {}", n, path),
            Err(e) => {
                eprintln!("Error loading --scenario {}: {}", path, e);
                process::exit(1);
            }
        }
    }

    if let Some(ref path) = cfg.sfp_eeprom_path {
        match std::fs::read(path) {
            Ok(bytes) => {
                let mut bank = cpu.bank().unwrap().write();
                match bank.bsc_i2c.sfp.load_pages(&bytes) {
                    Ok(()) => {
                        eprintln!("[sfp-eeprom] loaded {} bytes from {}", bytes.len(), path)
                    }
                    Err(e) => {
                        eprintln!("Error in --sfp-eeprom {}: {}", path, e);
                        process::exit(1);
                    }
                }
            }
            Err(e) => {
                eprintln!("Error opening --sfp-eeprom {}: {}", path, e);
                process::exit(1);
            }
        }
    }

    // Configure OLT emulation.
    if cfg.olt_enable {
        let mut bank = cpu.bank().unwrap().write();
        bank.olt.set_enabled(true);
        if let Some(mac) = cfg.olt_mac {
            bank.olt.config.mac = mac;
        }
        // Automatically set the link as up so the OLT starts
        // MPCP discovery once the firmware reaches the event loop.
        bank.olt.set_link_up(true);
    }

    if let Some(port) = cfg.mcp_port {
        run_with_mcp(cpu, &cfg, port);
        return;
    }

    let orig_termios = setup_raw_terminal();

    // Safety rail: infinite-loop firmware gets N chances before we give
    // up. Raised from 5 → u32::MAX (audit 8.3 — the old 5-reboot cap
    // was an arbitrary development fence, not hardware behaviour).
    const MAX_REBOOTS: u32 = u32::MAX;
    let mut reboot_count: u32 = 0;
    let final_result;

    loop {
        let result = run_emulator(&mut cpu, &cfg, &uart_tx);
        match result {
            RunResult::Reboot => {
                reboot_count += 1;
                if reboot_count > MAX_REBOOTS {
                    bcm55030_emulator::vlog!("[BCM55030] Max reboots ({}) reached, stopping", MAX_REBOOTS);
                    final_result = result;
                    break;
                }
                bcm55030_emulator::vlog!(
                    "\n[BCM55030] === REBOOT #{} (FLAG 1 = hardware reset) ===\n",
                    reboot_count
                );
                reset_cpu_for_reboot(&mut cpu);
                boot_from_flash(&mut cpu, cfg.entry_point, cfg.boot_mode);
                cpu.trace = cfg.trace;
                cpu.trace_sr = cfg.trace_sr;
                if cfg.trace_mmio {
                    let mut bank = cpu.bank().unwrap().write();
                    bank.trace = true;
                    bank.sysreg.trace = true;
                    bank.pbc.trace = true;
                }
            }
            _ => {
                final_result = result;
                break;
            }
        }
    }

    if let Some(ref orig) = orig_termios {
        restore_terminal(orig);
    }

    if let RunResult::Exception(ref e) = final_result {
        eprintln!("Exception at PC=0x{:08X}: {:?}", cpu.state.pc, e);
    }

    eprintln!();
    eprintln!("=== Final State ===");
    eprintln!("PC: 0x{:08X}", cpu.state.pc);
    eprintln!("Instructions: {}", cpu.state.instruction_count);
    eprintln!(
        "Flags: Z={} N={} C={} V={}",
        cpu.state.flag_z as u8,
        cpu.state.flag_n as u8,
        cpu.state.flag_c as u8,
        cpu.state.flag_v as u8
    );
    eprintln!(
        "STATUS32: 0x{:04X} (E1={} E2={} U={})",
        cpu.state.status32(),
        cpu.state.flag_e1 as u8,
        cpu.state.flag_e2 as u8,
        cpu.state.flag_u as u8
    );

    for i in 0..32 {
        if cpu.state.core_regs[i] != 0 {
            eprintln!("r{:<2}: 0x{:08X}", i, cpu.state.core_regs[i]);
        }
    }
    if cpu.state.core_regs[60] != 0 {
        eprintln!("LP_COUNT (r60): 0x{:08X}", cpu.state.core_regs[60]);
    }

    if cpu.state.halted {
        eprintln!("CPU: HALTED");
    }
    if cpu.state.sleeping {
        eprintln!("CPU: SLEEPING");
    }
    eprintln!("Reboots: {}", reboot_count);

    if let Some(ref path) = cfg.dccm_dump {
        bcm55030_emulator::vlog!("[BCM55030] Dumping DCCM to {}", path);
        let size = cpu.mem.sram_size();
        let mut buf = Vec::with_capacity(size);
        for addr in 0..size {
            buf.push(cpu.mem.read_byte(addr as u32).unwrap_or(0));
        }
        if let Err(e) = fs::write(path, &buf) {
            eprintln!("Error writing DCCM dump: {}", e);
        }
    }

    // Flush the sequential MMIO trace before any other I/O. BufWriter does
    // NOT flush on drop when the process exits, so we must do it explicitly.
    if cfg.trace_mmio_seq.is_some() {
        if let Some(bank) = cpu.bank() {
            let mut b = bank.write();
            if let Some(ref mut st) = b.seq_trace {
                st.flush();
            }
        }
        if let Some(ref path) = cfg.trace_mmio_seq {
            eprintln!("[BCM55030] Sequential MMIO trace flushed → {}", path);
        }
    }

    if let Some(ref path) = cfg.dump_mmio_trace {
        let trace = {
            let mut bank = cpu.bank().unwrap().write();
            bank.sysreg.mmio_trace.take()
        };
        if let Some(trace) = trace {
            let mut entries: Vec<(u32, bcm55030_emulator::soc::sysreg_shim::ShimTraceEntry)> =
                trace.into_iter().collect();
            entries.sort_by_key(|(off, _)| *off);
            let mut text = String::new();
            text.push_str("# Unhandled MMIO accesses (sorted by sysreg offset)\n");
            text.push_str("# offset      addr        reads  writes  last_read   last_write  first_pc  first_insn\n");
            for (off, e) in &entries {
                text.push_str(&format!(
                    "0x{:04X}  0x{:08X}  {:6}  {:6}  0x{:08X}  0x{:08X}  0x{:05X}  {}\n",
                    off,
                    0x01000000u32 + off,
                    e.reads,
                    e.writes,
                    e.last_read_value,
                    e.last_write_value,
                    e.first_pc,
                    e.first_insn,
                ));
            }
            if let Err(err) = fs::write(path, text) {
                eprintln!("Error writing MMIO trace: {}", err);
            } else {
                eprintln!("[BCM55030] MMIO trace ({} unique addresses) → {}", entries.len(), path);
            }
        }
    }

    if cfg.persist_flash {
        let (is_dirty, flash_data) = {
            let bank = cpu.bank().unwrap().read();
            (bank.pbc.flash.dirty, bank.pbc.flash.data.clone())
        };
        if is_dirty {
            match fs::write(&persist_path, &flash_data) {
                Ok(_) => bcm55030_emulator::vlog!("[BCM55030] Flash persisted to {} ({} bytes)", persist_path, flash_data.len()),
                Err(e) => eprintln!("Error persisting flash: {}", e),
            }
        } else {
            bcm55030_emulator::vlog!("[BCM55030] Flash not modified, nothing to persist");
        }
    }
}

