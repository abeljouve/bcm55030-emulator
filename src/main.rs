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
}

struct Config {
    flash_path: String,
    entry_point: u32,
    max_cycles: u64,
    trace: bool,
    trace_mmio: bool,
    breakpoint: Option<u32>,
    dccm_dump: Option<String>,
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

fn run_emulator(
    cpu: &mut Cpu,
    cfg: &Config,
) -> Result<(), bcm55030_emulator::cpu::exception::Exception> {
    for step in 0..cfg.max_cycles {
        if cpu.state.halted {
            break;
        }

        // Poll stdin every 1024 steps
        if step % 1024 == 0 {
            while let Some(byte) = try_read_stdin() {
                if byte == 3 {
                    eprintln!("\n[BCM55030] Ctrl-C, stopping");
                    cpu.state.halted = true;
                    return Ok(());
                }
                cpu.rx_queue.push_back(byte);
            }
        }

        if let Some(bp) = cfg.breakpoint {
            if cpu.state.pc == bp {
                eprintln!("[BREAK] Hit breakpoint at 0x{:08X}", bp);
                break;
            }
        }

        cpu.step()?;
    }
    Ok(())
}

fn main() {
    let cfg = parse_args();

    // Read flash image
    let flash_data = match fs::read(&cfg.flash_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("Error reading {}: {}", cfg.flash_path, e);
            process::exit(1);
        }
    };

    eprintln!(
        "[BCM55030] Flash image: {} ({} bytes)",
        cfg.flash_path,
        flash_data.len()
    );

    // --- Create BCM55030 CPU (always Harvard: ICCM + DCCM + MMIO) ---
    let mut cpu = Cpu::new_bcm55030();

    // --- Load flash image into SPI flash peripheral ---
    {
        let mut mmio = cpu.mem.mmio().expect("BCM55030 must have MMIO");
        let flash_size = mmio.pbc.flash.data.len();
        let copy_len = flash_data.len().min(flash_size);
        mmio.pbc.flash.data[..copy_len].copy_from_slice(&flash_data[..copy_len]);
        eprintln!("[BCM55030] SPI flash: {} bytes loaded", copy_len);
    }

    // --- Boot ROM emulation ---
    // The on-chip mask ROM reads the bootloader from SPI flash and copies
    // it into ICCM (code) and DCCM (data/literal pools), then jumps to
    // the entry point.

    // 1. Detect bootloader size in flash (scan for first erased block)
    let boot_size = {
        let mmio = cpu.mem.mmio().unwrap();
        detect_bootloader_size(&mmio.pbc.flash.data)
    };
    eprintln!("[BCM55030] Boot ROM: bootloader detected, {} bytes (0x{:X})", boot_size, boot_size);

    // 2. Copy bootloader from flash to ICCM and DCCM
    {
        // Copy from flash data to a local buffer first to avoid borrow conflict
        let code = {
            let mmio = cpu.mem.mmio().unwrap();
            mmio.pbc.flash.data[..boot_size].to_vec()
        };
        cpu.mem.load_iccm(0, &code);
        cpu.mem.load_binary(0, &code);
    }

    // 3. Fill remaining ICCM with J_S [blink] (0x7EE0)
    // Any call beyond the bootloader returns immediately — emulates
    // boot ROM helper stubs that exist only in the mask ROM.
    {
        let fill_start = (boot_size + 1) & !1; // 16-bit aligned
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
    cpu.state.aux_ienable = 0xFFFFFFFF; // All IRQs enabled
    cpu.state.flag_e1 = true;
    cpu.state.flag_e2 = true;

    // 5. Boot ROM: install IRQ handlers in the IVT
    // The bootloader's IVT entries for IRQs (0x80-0xFF) contain halt handlers.
    // The real boot ROM installs proper IRQ dispatchers. We write a small
    // handler at 0xA800 (in the J_S[blink] fill area) that saves blink,
    // calls the bootloader's callback dispatcher (0x8C80), restores blink,
    // and returns via RTIE.
    {
        // IRQ handler at 0xA800 (20 bytes):
        //   st.aw blink,[sp,-4]   = 0x1CFCB7C8
        //   jl 0x8C80             = 0x20220F80 00008C80
        //   ld.ab blink,[sp,4]    = 0x1404341F
        //   rtie                  = 0x246F003F
        let irq_handler: [u8; 20] = [
            0x1C, 0xFC, 0xB7, 0xC8, // st.aw blink,[sp,-4]
            0x20, 0x22, 0x0F, 0x80, // jl (first half)
            0x00, 0x00, 0x8C, 0x80, // jl 0x8C80 (second half)
            0x14, 0x04, 0x34, 0x1F, // ld.ab blink,[sp,4]
            0x24, 0x6F, 0x00, 0x3F, // rtie
        ];
        let handler_addr: u32 = 0xA800;
        cpu.mem.load_iccm(handler_addr, &irq_handler);

        // Install J 0xA800 at all IRQ vector entries (IRQ 0-15 at offsets 0x80-0xF8)
        // J 0xA800 = 0x20200F80 0000A800
        let j_handler: [u8; 8] = [
            0x20, 0x20, 0x0F, 0x80, // j (first half)
            0x00, 0x00, 0xA8, 0x00, // j 0xA800 (second half)
        ];
        for irq in 0..16u32 {
            let vector_offset = (16 + irq) * 8; // IVT: vectors 16-31 are IRQs
            cpu.mem.load_iccm(vector_offset, &j_handler);
        }
        eprintln!(
            "[BCM55030] Boot ROM: IRQ handler at 0x{:04X}, IVT vectors 0x80-0xF8 installed",
            handler_addr
        );
    }

    // --- Configure and run ---
    cpu.state.pc = cfg.entry_point;
    cpu.trace = cfg.trace;

    if cfg.trace_mmio {
        if let Some(mut mmio) = cpu.mem.mmio() {
            mmio.trace = true;
            mmio.pbc.trace = true;
        }
    }

    eprintln!(
        "[BCM55030] Entry: 0x{:08X}, ICCM=512KB, DCCM=512KB",
        cfg.entry_point
    );

    let orig_termios = setup_raw_terminal();

    let run_result = run_emulator(&mut cpu, &cfg);

    if let Some(ref orig) = orig_termios {
        restore_terminal(orig);
    }

    if let Err(e) = run_result {
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
}
