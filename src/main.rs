use std::env;
use std::fs;
use std::os::unix::io::AsRawFd;
use std::process;

use bcm55030_emulator::cpu::Cpu;

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
    eprintln!("  --verbose, -v               Show debug messages ([Hook], [MMIO], [Boot ROM]...)");
    eprintln!("  --break <ADDR>              Stop at address (hex)");
    eprintln!("  --dccm-dump <FILE>          Dump DCCM to file on exit");
    eprintln!("  --persist-flash             Save modified flash to <flash.bin>.persist on exit");
}

struct Config {
    flash_path: String,
    entry_point: u32,
    max_cycles: u64,
    trace: bool,
    trace_from_insn: Option<u64>,
    trace_mmio: bool,
    verbose: bool,
    breakpoint: Option<u32>,
    dccm_dump: Option<String>,
    persist_flash: bool,
    watch_dccm: Option<u32>,
    dump_mmio_trace: Option<String>,
}

fn parse_hex(s: &str) -> Option<u32> {
    let s = s.trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(s, 16).ok()
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
        verbose: false,
        breakpoint: None,
        dccm_dump: None,
        persist_flash: false,
        watch_dccm: None,
        dump_mmio_trace: None,
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

/// Detect the bootloader code size in a flash image.
/// BCM55030 hardware DMA copies exactly 64 KB from SPI flash offset 0 to SRAM.
/// Verified on real hardware via bare-metal memsize test (2026-04-12).
const BOOT_DMA_SIZE: usize = 64 * 1024;

/// Set up terminal in raw mode for interactive CLI.
fn setup_raw_terminal() -> Option<libc::termios> {
    unsafe {
        let fd = std::io::stdin().as_raw_fd();
        // Always set O_NONBLOCK on stdin (needed for piped/non-tty input too)
        let flags = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);

        let mut orig: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut orig) != 0 {
            // Not a terminal (piped stdin) — O_NONBLOCK is set, no raw mode needed
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

/// Result of run_emulator: tells the caller what to do next.
enum RunResult {
    /// CPU halted (FLAG 1) — treat as hardware reset, reboot from flash
    Reboot,
    /// User requested exit (Ctrl-C) — stop completely
    UserExit,
    /// Breakpoint hit — stop for debugging
    Breakpoint,
    /// CPU exception — stop with error
    Exception(bcm55030_emulator::cpu::exception::Exception),
    /// Max cycles reached
    MaxCycles,
}

fn run_emulator(cpu: &mut Cpu, cfg: &Config) -> RunResult {
    let mut trace_armed = cfg.trace_from_insn.is_some();
    for step in 0..cfg.max_cycles {
        if cpu.state.halted {
            return RunResult::Reboot;
        }

        // Late-arm tracing once instruction count crosses the threshold.
        if trace_armed {
            if let Some(threshold) = cfg.trace_from_insn {
                if cpu.state.instruction_count >= threshold {
                    cpu.trace = true;
                    trace_armed = false;
                    eprintln!("[TRACE] Tracing armed at insn={}", cpu.state.instruction_count);
                }
            }
        }

        // Poll stdin every 1024 steps — feed into UART RX queue
        if step % 1024 == 0 {
            // Firmware phase: PC is past the bootloader footprint (0..0xA800).
            let firmware_loaded = cpu.state.pc >= bcm55030_emulator::soc::boot_rom::FIRMWARE_BASE;
            while let Some(byte) = try_read_stdin() {
                if byte == 3 {
                    eprintln!("\n[BCM55030] Ctrl-C, stopping");
                    cpu.state.halted = true;
                    return RunResult::UserExit;
                }
                if let Some(mut mmio) = cpu.mem.mmio() {
                    mmio.uart.rx_queue.push_back(byte);
                    // Pre-firmware bytes need a parallel copy — bootloader ISR drops them.
                    if !firmware_loaded {
                        mmio.uart.held_pre_firmware.push_back(byte);
                    }
                }
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
fn boot_from_flash(cpu: &mut Cpu, entry_point: u32) {
    let copy_size = {
        let mmio = cpu.mem.mmio().unwrap();
        mmio.pbc.flash.data.len().min(BOOT_DMA_SIZE)
    };
    {
        let code = {
            let mmio = cpu.mem.mmio().unwrap();
            mmio.pbc.flash.data[..copy_size].to_vec()
        };
        cpu.mem.load_binary(0, &code);
    }
    bcm55030_emulator::vlog!(
        "[BCM55030] HW DMA: copied {} bytes (0x{:X}) from flash to SRAM",
        copy_size, copy_size
    );

    // Preset IENABLE/E1/E2 — bootloader enables them later but the emulator
    // UART IRQ path needs them from instruction 0.
    cpu.state.aux_ienable = 0xFFFFFFFF;
    cpu.state.flag_e1 = true;
    cpu.state.flag_e2 = true;

    cpu.state.pc = entry_point;
    bcm55030_emulator::vlog!(
        "[BCM55030] Entry: 0x{:08X}, SRAM=512KB, boot DMA=64KB",
        entry_point
    );
}

/// Reset the CPU and DCCM for a fresh boot, preserving SPI flash state.
/// This emulates a hardware reset: all volatile state (registers, DCCM)
/// is cleared, but the SPI flash retains its contents.
fn reset_cpu_for_reboot(cpu: &mut Cpu) {
    use bcm55030_emulator::cpu::registers::CpuState;

    // Reset CPU state (all registers, flags, aux regs). Preserve SoC-integration
    // fields that describe hardware wiring (timer IRQ lines).
    let saved_timer1_irq = cpu.state.timer1_irq;
    cpu.state = CpuState::new();
    cpu.state.timer1_irq = saved_timer1_irq;

    // Clear SRAM (volatile RAM)
    let sram_size = cpu.mem.sram_size();
    let zeros = vec![0u8; sram_size];
    cpu.mem.load_binary(0, &zeros);

    // Reset peripheral state but NOT flash data (non-volatile)
    if let Some(mut mmio) = cpu.mem.mmio() {
        mmio.uart.rx_queue.clear();
        mmio.pbc.reset_state();
    }
}

fn main() {
    let cfg = parse_args();
    bcm55030_emulator::set_verbose(cfg.verbose);

    // Read flash image (or persisted version if available)
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

    // --- Create BCM55030 CPU ---
    let mut cpu = Cpu::new_bcm55030();

    // --- Register BCM55030 SoC hooks ---
    bcm55030_emulator::soc::register_hooks(&mut cpu.hooks);

    // --- Load flash image into SPI flash peripheral ---
    {
        let mut mmio = cpu.mem.mmio().expect("BCM55030 must have MMIO");
        let flash_size = mmio.pbc.flash.data.len();
        let copy_len = flash_data.len().min(flash_size);
        mmio.pbc.flash.data[..copy_len].copy_from_slice(&flash_data[..copy_len]);
        bcm55030_emulator::vlog!("[BCM55030] SPI flash: {} bytes loaded", copy_len);
    }

    // --- Initial boot from flash ---
    boot_from_flash(&mut cpu, cfg.entry_point);

    cpu.trace = cfg.trace;
    cpu.mem.dccm_watchpoint = cfg.watch_dccm;
    if cfg.trace_mmio {
        if let Some(mut mmio) = cpu.mem.mmio() {
            mmio.trace = true;
            mmio.pbc.trace = true;
        }
    }
    if cfg.dump_mmio_trace.is_some() {
        if let Some(mut mmio) = cpu.mem.mmio() {
            mmio.mmio_trace = Some(std::collections::HashMap::new());
        }
    }

    let orig_termios = setup_raw_terminal();

    // FLAG 1 triggers chip reset. Flash persists across reboots.
    const MAX_REBOOTS: u32 = 5;
    let mut reboot_count: u32 = 0;
    let final_result;

    loop {
        let result = run_emulator(&mut cpu, &cfg);
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
                // Reset CPU and DCCM, keep flash state
                reset_cpu_for_reboot(&mut cpu);
                // Re-run boot ROM sequence from (possibly modified) flash
                boot_from_flash(&mut cpu, cfg.entry_point);
                cpu.trace = cfg.trace;
                if cfg.trace_mmio {
                    if let Some(mut mmio) = cpu.mem.mmio() {
                        mmio.trace = true;
                        mmio.pbc.trace = true;
                    }
                }
                // Continue execution loop
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

    // Final state
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

    // DCCM dump
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

    // Unhandled MMIO trace dump
    if let Some(ref path) = cfg.dump_mmio_trace {
        if let Some(mut mmio) = cpu.mem.mmio() {
            if let Some(trace) = mmio.mmio_trace.take() {
                let mut entries: Vec<(u32, bcm55030_emulator::soc::mmio::MmioTraceEntry)> =
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
    }

    // Persist flash if requested and modified
    if cfg.persist_flash {
        let is_dirty = cpu.mem.mmio().map_or(false, |mmio| mmio.pbc.flash.dirty);
        if is_dirty {
            let flash_data: Vec<u8> = cpu.mem.mmio().unwrap().pbc.flash.data.clone();
            match fs::write(&persist_path, &flash_data) {
                Ok(_) => bcm55030_emulator::vlog!("[BCM55030] Flash persisted to {} ({} bytes)", persist_path, flash_data.len()),
                Err(e) => eprintln!("Error persisting flash: {}", e),
            }
        } else {
            bcm55030_emulator::vlog!("[BCM55030] Flash not modified, nothing to persist");
        }
    }
}
