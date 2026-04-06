use std::env;
use std::fs;
use std::process;

use bcm55030_emulator::cpu::Cpu;

fn usage(prog: &str) {
    eprintln!("Usage: {} [OPTIONS] <binary>", prog);
    eprintln!();
    eprintln!("Modes:");
    eprintln!("  <binary>                    Load raw binary at address 0 (flat mode)");
    eprintln!("  --soc <binary>              BCM55030 SoC mode (Harvard ICCM/DCCM)");
    eprintln!("  --load-firmware <binary>        Load firmware firmware, entry at 0x0150");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --entry <ADDR>              Entry point (hex, default: 0x0000)");
    eprintln!("  --max-cycles <N>            Maximum instructions (default: 1000000)");
    eprintln!("  --trace                     Log each instruction to stderr");
    eprintln!("  --trace-mmio                Log MMIO accesses to stderr");
    eprintln!("  --break <ADDR>              Stop at address (hex)");
    eprintln!("  --dccm-dump <FILE>          Dump DCCM to file on exit");
    eprintln!("  --mem-size <KB>             Flat mode memory size (default: 1024)");
    eprintln!("  --flash <FILE>              Load SPI flash image (4MB, SoC mode)");
}

struct Config {
    binary_path: String,
    soc_mode: bool,
    load_firmware: bool,
    entry_point: u32,
    max_cycles: u64,
    trace: bool,
    trace_mmio: bool,
    breakpoint: Option<u32>,
    dccm_dump: Option<String>,
    mem_size_kb: usize,
    flash_path: Option<String>,
}

fn parse_hex(s: &str) -> Option<u32> {
    let s = s.trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(s, 16).ok()
}

fn parse_args() -> Config {
    let args: Vec<String> = env::args().collect();
    let prog = &args[0];

    let mut cfg = Config {
        binary_path: String::new(),
        soc_mode: false,
        load_firmware: false,
        entry_point: 0,
        max_cycles: 1_000_000,
        trace: false,
        trace_mmio: false,
        breakpoint: None,
        dccm_dump: None,
        mem_size_kb: 1024,
        flash_path: None,
    };

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--soc" => {
                cfg.soc_mode = true;
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --soc requires a binary path");
                    process::exit(1);
                }
                cfg.binary_path = args[i].clone();
            }
            "--load-firmware" => {
                cfg.soc_mode = true;
                cfg.load_firmware = true;
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --load-firmware requires a binary path");
                    process::exit(1);
                }
                cfg.binary_path = args[i].clone();
                cfg.entry_point = 0x0150;
            }
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
            "--mem-size" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --mem-size requires a number");
                    process::exit(1);
                }
                cfg.mem_size_kb = args[i].parse().unwrap_or_else(|_| {
                    eprintln!("Error: invalid number: {}", args[i]);
                    process::exit(1);
                });
            }
            "--flash" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("Error: --flash requires a file path");
                    process::exit(1);
                }
                cfg.flash_path = Some(args[i].clone());
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
                cfg.binary_path = args[i].clone();
            }
        }
        i += 1;
    }

    if cfg.binary_path.is_empty() {
        eprintln!("Error: no binary specified");
        usage(prog);
        process::exit(1);
    }

    cfg
}

fn main() {
    let cfg = parse_args();

    let binary = match fs::read(&cfg.binary_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("Error reading {}: {}", cfg.binary_path, e);
            process::exit(1);
        }
    };

    let mut cpu = if cfg.soc_mode {
        eprintln!(
            "[SoC] BCM55030 mode: ICCM=512KB DCCM=512KB, loading {} ({} bytes)",
            cfg.binary_path,
            binary.len()
        );
        let mut cpu = Cpu::new_bcm55030();
        // Load firmware into ICCM (code) and DCCM (data/literal pools)
        cpu.mem.load_iccm(0, &binary);
        cpu.mem.load_binary(0, &binary);
        // Set initial stack pointer (SP = r28) to top of DCCM stack area
        cpu.state.core_regs[28] = 0x10800;

        // BCM55030 boot ROM emulation.
        // The on-chip mask ROM normally runs before the bootloader:
        // it loads firmware into ICCM/DCCM, writes callback pointers
        // to specific DCCM addresses, then jumps to the entry point.
        //
        // Fill unloaded ICCM with J_S [blink] (0x7EE0) so that any call
        // to code beyond the loaded binary returns immediately. This handles
        // boot ROM helper functions and tail-calls to code that only exists
        // in the full firmware image.
        {
            let bin_len = binary.len();
            // Round up to 16-bit alignment
            let fill_start = (bin_len + 1) & !1;
            let iccm_size = 512 * 1024;
            if fill_start < iccm_size {
                let mut fill = vec![0u8; iccm_size - fill_start];
                // Fill with J_S [blink] = 0x7E 0xE0 (return to caller)
                for chunk in fill.chunks_exact_mut(2) {
                    chunk[0] = 0x7E;
                    chunk[1] = 0xE0;
                }
                cpu.mem.load_iccm(fill_start as u32, &fill);
            }

            eprintln!(
                "[SoC] Boot ROM: ICCM fill from 0x{:05X} with J_S [blink]",
                fill_start
            );
        }

        cpu
    } else {
        let mem_size = cfg.mem_size_kb * 1024;
        let mut cpu = Cpu::new(mem_size);
        cpu.mem.load_binary(0, &binary);
        cpu
    };

    cpu.state.pc = cfg.entry_point;
    cpu.trace = cfg.trace;

    // Configure MMIO trace
    if cfg.trace_mmio {
        if let Some(mut mmio) = cpu.mem.mmio() {
            mmio.trace = true;
            mmio.pbc.trace = true;
        }
    }

    // Load SPI flash image if provided
    if let Some(ref flash_path) = cfg.flash_path {
        let flash_data = fs::read(flash_path).unwrap_or_else(|e| {
            eprintln!("Error reading flash image {}: {}", flash_path, e);
            process::exit(1);
        });
        if let Some(mut mmio) = cpu.mem.mmio() {
            let flash_size = mmio.pbc.flash.data.len();
            let copy_len = flash_data.len().min(flash_size);
            mmio.pbc.flash.data[..copy_len].copy_from_slice(&flash_data[..copy_len]);
            eprintln!("[SoC] SPI flash: loaded {} bytes from {}", copy_len, flash_path);
        }
    } else if cfg.soc_mode {
        // In SoC mode without explicit flash, load the binary into flash at offset 0
        if let Some(mut mmio) = cpu.mem.mmio() {
            let copy_len = binary.len().min(mmio.pbc.flash.data.len());
            mmio.pbc.flash.data[..copy_len].copy_from_slice(&binary[..copy_len]);
            eprintln!("[SoC] SPI flash: loaded firmware binary ({} bytes) at offset 0", copy_len);
        }
    }

    if cfg.soc_mode {
        eprintln!(
            "[SoC] Entry point: 0x{:08X}, max cycles: {}",
            cfg.entry_point, cfg.max_cycles
        );
    }

    // Run with optional breakpoint
    if let Some(bp) = cfg.breakpoint {
        for _ in 0..cfg.max_cycles {
            if cpu.state.halted || cpu.state.sleeping {
                break;
            }
            if cpu.state.pc == bp {
                eprintln!("[BREAK] Hit breakpoint at 0x{:08X}", bp);
                break;
            }
            match cpu.step() {
                Ok(()) => {}
                Err(e) => {
                    eprintln!(
                        "Exception at PC=0x{:08X}: {:?}",
                        cpu.state.pc, e
                    );
                    break;
                }
            }
        }
    } else {
        match cpu.run(cfg.max_cycles) {
            Ok(()) => {}
            Err(e) => {
                eprintln!(
                    "Exception at PC=0x{:08X}: {:?}",
                    cpu.state.pc, e
                );
            }
        }
    }

    // Print final state
    eprintln!();
    eprintln!("=== Final State ===");
    eprintln!("PC: 0x{:08X}", cpu.state.pc);
    eprintln!(
        "Instructions executed: {}",
        cpu.state.instruction_count
    );
    eprintln!(
        "Flags: Z={} N={} C={} V={}",
        cpu.state.flag_z as u8,
        cpu.state.flag_n as u8,
        cpu.state.flag_c as u8,
        cpu.state.flag_v as u8
    );
    eprintln!(
        "STATUS32: 0x{:04X} (U={} E1={} E2={})",
        cpu.state.status32(),
        cpu.state.flag_u as u8,
        cpu.state.flag_e1 as u8,
        cpu.state.flag_e2 as u8
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
        if cpu.mem.is_harvard() {
            eprintln!("[SoC] Dumping DCCM to {}", path);
            // Read DCCM byte by byte into a buffer
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
}
