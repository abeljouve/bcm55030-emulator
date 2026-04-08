use std::env;
use std::fs;
use std::os::unix::io::AsRawFd;
use std::process;

use bcm55030_emulator::cpu::Cpu;
use bcm55030_emulator::memory::ICCM_SIZE;

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
    eprintln!("  --break <ADDR>              Stop at address (hex)");
    eprintln!("  --dccm-dump <FILE>          Dump DCCM to file on exit");
    eprintln!("  --persist-flash             Save modified flash to <flash.bin>.persist on exit");
}

struct Config {
    flash_path: String,
    entry_point: u32,
    max_cycles: u64,
    trace: bool,
    trace_mmio: bool,
    breakpoint: Option<u32>,
    dccm_dump: Option<String>,
    persist_flash: bool,
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
        trace_mmio: false,
        breakpoint: None,
        dccm_dump: None,
        persist_flash: false,
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
            "--trace-mmio" => cfg.trace_mmio = true,
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
/// Scans from offset 0 for the first 256-byte aligned block that is all 0xFF (erased).
/// Returns the offset of that block (= end of bootloader code region).
fn detect_bootloader_size(flash: &[u8]) -> usize {
    let block_size = 256;
    let max_scan = flash.len().min(ICCM_SIZE);
    let mut offset = 0;
    while offset < max_scan {
        let end = (offset + block_size).min(max_scan);
        let block = &flash[offset..end];
        if block.iter().all(|&b| b == 0xFF) {
            return offset;
        }
        offset += block_size;
    }
    max_scan
}

/// Set up terminal in raw mode for interactive CLI.
fn setup_raw_terminal() -> Option<libc::termios> {
    unsafe {
        let fd = std::io::stdin().as_raw_fd();
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
        let flags = libc::fcntl(fd, libc::F_GETFL);
        libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
        eprintln!("[BCM55030] Terminal: raw mode (Ctrl-C to exit)");
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
    for step in 0..cfg.max_cycles {
        if cpu.state.halted {
            return RunResult::Reboot;
        }

        // Poll stdin every 1024 steps — feed into UART RX queue
        if step % 1024 == 0 {
            while let Some(byte) = try_read_stdin() {
                if byte == 3 {
                    eprintln!("\n[BCM55030] Ctrl-C, stopping");
                    cpu.state.halted = true;
                    return RunResult::UserExit;
                }
                if let Some(mut mmio) = cpu.mem.mmio() {
                    mmio.uart.rx_queue.push_back(byte);
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

/// Boot ROM emulation: load bootloader from SPI flash into ICCM/DCCM,
/// fill unused ICCM with J_S [blink] stubs, install IRQ vectors,
/// and configure initial CPU state.
fn boot_from_flash(cpu: &mut Cpu, entry_point: u32) {
    // 1. Detect bootloader size in flash
    let boot_size = {
        let mmio = cpu.mem.mmio().unwrap();
        detect_bootloader_size(&mmio.pbc.flash.data)
    };
    eprintln!("[BCM55030] Boot ROM: bootloader detected, {} bytes (0x{:X})", boot_size, boot_size);

    // 2. Copy bootloader from flash to ICCM and DCCM
    {
        let code = {
            let mmio = cpu.mem.mmio().unwrap();
            mmio.pbc.flash.data[..boot_size].to_vec()
        };
        cpu.mem.load_iccm(0, &code);
        cpu.mem.load_binary(0, &code);
    }

    // 3. Fill remaining ICCM with J_S [blink] (0x7EE0)
    {
        let fill_start = (boot_size + 1) & !1;
        if fill_start < ICCM_SIZE {
            let mut fill = vec![0u8; ICCM_SIZE - fill_start];
            for chunk in fill.chunks_exact_mut(2) {
                chunk[0] = 0x7E;
                chunk[1] = 0xE0;
            }
            cpu.mem.load_iccm(fill_start as u32, &fill);
        }
        eprintln!(
            "[BCM55030] Boot ROM: ICCM filled from 0x{:05X} with J_S [blink]",
            fill_start
        );
    }

    // 4. Boot ROM initial state
    cpu.state.core_regs[28] = 0x10800; // SP = top of DCCM stack area
    cpu.state.aux_ienable = 0xFFFFFFFF;
    cpu.state.flag_e1 = true;
    cpu.state.flag_e2 = true;

    // 5. Install IRQ handlers in the IVT
    {
        // IRQ handler at 0xA800 (20 bytes):
        //   st.aw blink,[sp,-4]   = 0x1CFCB7C8
        //   jl 0x8C80             = 0x20220F80 00008C80
        //   ld.ab blink,[sp,4]    = 0x1404341F
        //   rtie                  = 0x246F003F
        let irq_handler: [u8; 20] = [
            0x1C, 0xFC, 0xB7, 0xC8,
            0x20, 0x22, 0x0F, 0x80,
            0x00, 0x00, 0x8C, 0x80,
            0x14, 0x04, 0x34, 0x1F,
            0x24, 0x6F, 0x00, 0x3F,
        ];
        let handler_addr: u32 = 0xA800;
        cpu.mem.load_iccm(handler_addr, &irq_handler);

        // Install J 0xA800 at all IRQ vector entries (IRQ 0-15 at offsets 0x80-0xF8)
        let j_handler: [u8; 8] = [
            0x20, 0x20, 0x0F, 0x80,
            0x00, 0x00, 0xA8, 0x00,
        ];
        for irq in 0..16u32 {
            let vector_offset = (16 + irq) * 8;
            cpu.mem.load_iccm(vector_offset, &j_handler);
        }
        eprintln!(
            "[BCM55030] Boot ROM: IRQ handler at 0x{:04X}, IVT vectors 0x80-0xF8 installed",
            handler_addr
        );

        // UART IRQ 5 (level 1): direct to bootloader's native UART ISR at 0x4348
        let j_uart_isr: [u8; 8] = [
            0x20, 0x20, 0x0F, 0x80,
            0x00, 0x00, 0x43, 0x48,
        ];
        let uart_vector = (16 + 5) * 8;
        cpu.mem.load_iccm(uart_vector, &j_uart_isr);
        eprintln!(
            "[BCM55030] Boot ROM: UART ISR at 0x4348, vector 0x{:02X} (IRQ 5, level 1)",
            uart_vector
        );
    }

    // 6. Set entry point
    cpu.state.pc = entry_point;
    eprintln!(
        "[BCM55030] Entry: 0x{:08X}, ICCM=512KB, DCCM=512KB",
        entry_point
    );
}

/// Reset the CPU and DCCM for a fresh boot, preserving SPI flash state.
/// This emulates a hardware reset: all volatile state (registers, DCCM)
/// is cleared, but the SPI flash retains its contents.
fn reset_cpu_for_reboot(cpu: &mut Cpu) {
    use bcm55030_emulator::cpu::registers::CpuState;

    // Reset CPU state (all registers, flags, aux regs)
    cpu.state = CpuState::new();

    // Clear DCCM (volatile RAM)
    let dccm_size = cpu.mem.dccm_size();
    let zeros = vec![0u8; dccm_size];
    cpu.mem.load_binary(0, &zeros);

    // Reset peripheral state but NOT flash data (non-volatile)
    if let Some(mut mmio) = cpu.mem.mmio() {
        mmio.uart.rx_queue.clear();
        mmio.pbc.reset_state();
    }
}

fn main() {
    let cfg = parse_args();

    // Read flash image (or persisted version if available)
    let persist_path = format!("{}.persist", cfg.flash_path);
    let (flash_data, using_persisted) = if cfg.persist_flash {
        match fs::read(&persist_path) {
            Ok(b) => {
                eprintln!("[BCM55030] Loading persisted flash: {}", persist_path);
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

    eprintln!(
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
        eprintln!("[BCM55030] SPI flash: {} bytes loaded", copy_len);
    }

    // --- Initial boot from flash ---
    boot_from_flash(&mut cpu, cfg.entry_point);

    cpu.trace = cfg.trace;
    if cfg.trace_mmio {
        if let Some(mut mmio) = cpu.mem.mmio() {
            mmio.trace = true;
            mmio.pbc.trace = true;
        }
    }

    let orig_termios = setup_raw_terminal();

    // --- Main execution loop with reboot support ---
    // On real BCM55030 hardware, FLAG 1 (halt) triggers a chip reset which
    // reboots from SPI flash. The flash is non-volatile and retains any
    // modifications made during the previous boot cycle. This is critical
    // for the bootloader's recovery mechanism: first boot writes FDS flags
    // to flash, resets, and the second boot reads the updated flags.
    const MAX_REBOOTS: u32 = 5;
    let mut reboot_count: u32 = 0;
    let final_result;

    loop {
        let result = run_emulator(&mut cpu, &cfg);
        match result {
            RunResult::Reboot => {
                reboot_count += 1;
                if reboot_count > MAX_REBOOTS {
                    eprintln!("[BCM55030] Max reboots ({}) reached, stopping", MAX_REBOOTS);
                    final_result = result;
                    break;
                }
                eprintln!(
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
        eprintln!("[BCM55030] Dumping DCCM to {}", path);
        let size = cpu.mem.dccm_size();
        let mut buf = Vec::with_capacity(size);
        for addr in 0..size {
            buf.push(cpu.mem.read_byte(addr as u32).unwrap_or(0));
        }
        if let Err(e) = fs::write(path, &buf) {
            eprintln!("Error writing DCCM dump: {}", e);
        }
    }

    // Persist flash if requested and modified
    if cfg.persist_flash {
        let is_dirty = cpu.mem.mmio().map_or(false, |mmio| mmio.pbc.flash.dirty);
        if is_dirty {
            let flash_data: Vec<u8> = cpu.mem.mmio().unwrap().pbc.flash.data.clone();
            match fs::write(&persist_path, &flash_data) {
                Ok(_) => eprintln!("[BCM55030] Flash persisted to {} ({} bytes)", persist_path, flash_data.len()),
                Err(e) => eprintln!("Error persisting flash: {}", e),
            }
        } else {
            eprintln!("[BCM55030] Flash not modified, nothing to persist");
        }
    }
}
