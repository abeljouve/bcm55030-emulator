pub mod condition;
pub mod exception;
pub mod registers;

use std::collections::HashMap;
use std::sync::Arc;

use parking_lot::RwLock;

use exception::Exception;
use registers::{CpuState, DelayState, REG_BLINK, REG_ILINK1, REG_ILINK2, REG_LP_COUNT};

use crate::debug_info::DebugInfo;
use crate::decoder;
use crate::decoder::instruction::Instruction;
use crate::executor;
use crate::hooks::{self, HookAction, HookTable};
use crate::memory::Memory;
use crate::soc::bank::{BootMode, PeripheralBank, BANK_TICK_PRESCALER};

/// UART interrupt number (IRQ 5, level 1 per aux_irq_lev = 0xD7 bit 5 = 0).
const UART_IRQ: u32 = 5;

/// UART IRQ poll prescaler.
const UART_PRESCALER: u64 = 256;

/// Number of consecutive same-PC steps after which a spin is *recognised*.
///
/// Recognising a spin is NOT the same as deciding what it means — see
/// [`Cpu::classify_spin`]. This threshold only says "the PC has stopped
/// moving"; whether that ends in a watchdog reset, an interrupt, or a
/// permanent hang is decided from the timer control registers.
const TIGHT_LOOP_THRESHOLD: u32 = 64;

/// ARC timer `CONTROL` bit 0 — IE, interrupt enable.
/// (`docs/isa/05-registers.md`, Figure 66.)
const TIMER_CTRL_IE: u32 = 1 << 0;
/// ARC timer `CONTROL` bit 2 — W, "enable watchdog reset signal".
const TIMER_CTRL_W: u32 = 1 << 2;

/// What a stalled PC actually means on this SoC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpinVerdict {
    /// A timer has its W (watchdog) bit set: the chip really will reset, at
    /// the moment that timer reaches its limit — not before.
    WatchdogArmed,
    /// Interrupts can still be delivered, so something can still break the
    /// spin. This is the ordinary "wait here until the ISR runs" idiom.
    Interruptible,
    /// No watchdog, no path for an interrupt: nothing in the machine can ever
    /// change the PC again. Silicon sits here forever.
    Hung,
}

pub struct Cpu {
    pub state: CpuState,
    pub mem: Memory,
    /// Log every instruction to stderr
    pub trace: bool,
    /// Log `sr` (AUX register write) instructions to stderr
    pub trace_sr: bool,
    /// DWARF-based PC → source-location lookup, loaded from a
    /// separate ELF via the `--debug-elf` flag. When set, the trace
    /// output and the UI disassembly panel annotate each instruction
    /// with its corresponding Rust source line.
    pub debug_info: Option<DebugInfo>,
    /// PC address hooks for breakpoints / watchpoints / run-to-cursor.
    /// Empty by default on `develop` — all 35 SoC-specific entries from
    /// the old `register_hooks()` were deleted per the contributor guide. The
    /// infrastructure stays for the future UI debug features.
    pub hooks: HookTable,
    /// Shadow call stack for `get_call_stack` MCP tool. Pushed on BL/JL,
    /// popped on J [blink]. Max depth 64 — deeper nesting truncates.
    pub shadow_call_stack: Vec<u32>,
    /// Per-function instruction counter for `get_function_profile`. Keyed
    /// by call-site PC (top of shadow stack), incremented each step.
    pub function_profile: HashMap<u32, u64>,
    pub profiling_enabled: bool,
    /// Fractional accumulator for ARC Timer0/1 tick rate.
    /// 156.25 MHz clock, ~1.76 cycles/instruction average.
    pub timer_frac_acc: u32,
    /// Shared peripheral bank (cloned from `mem.bank()`). Cpu holds an
    /// additional `Arc` clone so it can tick the bank without going
    /// through `Memory`'s hot path.
    bank: Option<Arc<RwLock<PeripheralBank>>>,
    /// Instructions elapsed since the last bank tick.
    pub bank_tick_accumulator: u64,

    /// Tight-loop (branch-to-self) detection state. When the PC does
    /// not change across consecutive steps, this counter increments.
    /// Once it reaches [`TIGHT_LOOP_THRESHOLD`], the emulator treats
    /// the spin as a watchdog-triggered warm reset — modelling the
    /// BCM55030's Timer 1 watchdog that fires after the firmware
    /// enters `system_reboot_infinite_loop`.
    tight_loop_count: u32,
    tight_loop_last_pc: u32,

    /// When `true`, a detected tight loop HALTS the CPU instead of issuing the
    /// modelled watchdog warm reboot. Default `false` (real-HW behavior: reboot
    /// from flash). The per-function differential harness sets this so a fuzzed
    /// input that drives a function into a spin terminates that isolated run
    /// (reached=false) rather than rebooting the whole SoC into boot code —
    /// which both mutates shared state and (pre-fix) could panic deep in the
    /// boot path, killing the worker and dropping fuzz cases nondeterministically.
    pub tight_loop_halts: bool,

    /// Optional cycle-accurate cache + bus-contention shadow (the `timing`
    /// feature). `None` unless explicitly enabled via [`Cpu::enable_timing`];
    /// with the feature off this field does not exist and the ISS is unchanged.
    #[cfg(feature = "timing")]
    pub timing_shadow: Option<crate::timing::TimingShadow>,
}

impl Cpu {
    /// Create a CPU with flat memory (for tests / simple use).
    pub fn new(mem_size: usize) -> Self {
        Self {
            state: CpuState::new(),
            mem: Memory::new(mem_size),
            trace: false,
            trace_sr: false,
            debug_info: None,
            hooks: HookTable::new(),
            shadow_call_stack: Vec::new(),
            function_profile: HashMap::new(),
            profiling_enabled: false,
            timer_frac_acc: 0,
            bank: None,
            bank_tick_accumulator: 0,
            tight_loop_count: 0,
            tight_loop_last_pc: u32::MAX,
            tight_loop_halts: false,
            #[cfg(feature = "timing")]
            timing_shadow: None,
        }
    }

    /// Create a BCM55030 CPU with unified SRAM + peripheral bank.
    pub fn new_bcm55030(boot_mode: BootMode) -> Self {
        let mut state = CpuState::new();
        // BCM55030 wires Timer 1 to IRQ 7 (standard ARC 700 uses IRQ 4).
        state.timer1_irq = 7;
        let mem = Memory::new_soc(crate::memory::SRAM_SIZE, boot_mode);
        let bank = mem.bank().cloned();
        Self {
            state,
            mem,
            trace: false,
            trace_sr: false,
            debug_info: None,
            hooks: HookTable::new(),
            shadow_call_stack: Vec::new(),
            function_profile: HashMap::new(),
            profiling_enabled: false,
            timer_frac_acc: 0,
            bank,
            bank_tick_accumulator: 0,
            tight_loop_count: 0,
            tight_loop_last_pc: u32::MAX,
            tight_loop_halts: false,
            #[cfg(feature = "timing")]
            timing_shadow: None,
        }
    }

    /// Access the shared peripheral bank handle. Used by `main.rs` to
    /// grab the UART mpsc sender and wire stdin → UART input.
    pub fn bank(&self) -> Option<&Arc<RwLock<PeripheralBank>>> {
        self.bank.as_ref()
    }

    /// Enable the optional cycle-accurate cache + bus-contention timing overlay
    /// (the `timing` feature). Opt-in: with the overlay off the functional ISS
    /// behaves exactly as before. When on, each executed uncached load runs a
    /// contention window and the CPU halts with a `BUS_DEADLOCK` report if the
    /// fetch-vs-load starvation invariant fires.
    #[cfg(feature = "timing")]
    pub fn enable_timing(&mut self, cfg: crate::timing::TimingConfig) {
        self.timing_shadow = Some(crate::timing::TimingShadow::new(cfg));
    }

    /// Extract the timing overlay's per-instruction inputs from a decoded
    /// instruction and the current register state: `Some((ea, mmio))` if it is
    /// an uncached load, and whether it is a SYNC barrier. The effective
    /// address is resolved from the current registers (loop-invariant for the
    /// poll loops the model targets), and "uncached" is `.di` or any address
    /// outside the D-cacheable SRAM window.
    #[cfg(feature = "timing")]
    fn timing_load_info(inst: &Instruction, state: &CpuState) -> (Option<(u32, bool)>, bool) {
        use crate::decoder::instruction::{DataSize, WritebackMode, ZeroOp};
        match inst {
            Instruction::Load {
                base,
                offset,
                cache_bypass,
                writeback,
                data_size,
                ..
            } => {
                let base_val = executor::resolve_value(*base, state).unwrap_or(0);
                let off_val = executor::resolve_value(*offset, state).unwrap_or(0);
                let ea = match writeback {
                    WritebackMode::PostWrite => base_val,
                    WritebackMode::Scaled => {
                        let scale = match data_size {
                            DataSize::Word => 4u32,
                            DataSize::HalfWord => 2,
                            DataSize::Byte => 1,
                        };
                        base_val.wrapping_add(off_val.wrapping_mul(scale))
                    }
                    _ => base_val.wrapping_add(off_val),
                };
                // Any address outside the D-cacheable SRAM window bypasses the
                // D-cache (MMIO / peripheral space) — a wide-latency bus load.
                let mmio = ea >= crate::memory::SRAM_SIZE as u32;
                let uncached = *cache_bypass || mmio;
                (uncached.then_some((ea, mmio)), false)
            }
            Instruction::ZeroOp(ZeroOp::Sync) => (None, true),
            _ => (None, false),
        }
    }

    /// Reset CPU state, SRAM, and peripheral bank **in place**
    /// without creating a new bank `Arc`. Used by the GUI CPU
    /// worker so clones of the bank handle held by the UI and
    /// MCP threads stay valid across a reset. The peripheral
    /// bank's non-volatile state (SPI flash contents, SFP EEPROM,
    /// eFuse snapshot) is preserved.
    /// Clear transient watchdog tight-loop detection state without touching
    /// architectural state (registers, SRAM, peripherals). A harness that
    /// reuses one Cpu across many independent runs must call this between runs
    /// (e.g. after restoring a snapshot) so a prior run's spin history cannot
    /// bleed into the next and make its result depend on execution order.
    pub fn reset_loop_detection(&mut self) {
        self.tight_loop_count = 0;
        self.tight_loop_last_pc = u32::MAX;
    }

    pub fn reset_soc_in_place(&mut self, boot_mode: BootMode) {
        let saved_timer1_irq = self.state.timer1_irq;
        self.state = CpuState::new();
        self.state.timer1_irq = saved_timer1_irq;

        let sram_size = self.mem.sram_size();
        let zeros = vec![0u8; sram_size];
        self.mem.load_binary(0, &zeros);

        // A CPU reset leaves the I/D-caches invalid — silicon has no
        // "previous life" across a power-on or watchdog reset. The
        // `Memory` (and thus the caches) is reused in place by the GUI
        // CPU worker, so without this an earlier firmware's stale lines
        // would survive into the next `load_firmware` and the new
        // bootloader's `0x0..0x80` IVT/code would be fetched stale —
        // diverging from silicon (PC=0xFFFFFFFF crash). Mirrors
        // `warm_reboot_from_flash`, which already does this.
        self.mem.dcache_invalidate_all().ok();
        self.mem.icache_invalidate_all();

        self.tight_loop_count = 0;
        self.tight_loop_last_pc = u32::MAX;

        if let Some(bank) = self.bank.as_ref() {
            let mut guard = bank.write();
            match boot_mode {
                BootMode::Warm => guard.reset_warm(),
                BootMode::Cold => guard.reset_cold(),
            }
        }
    }

    /// Warm reboot from the current flash content — models the
    /// BCM55030 watchdog reset. Reads the first 64 KB from the PBC
    /// SPI flash (which may have been modified by FDS writes), resets
    /// CPU + peripherals, re-DMAs the bootloader into SRAM, and
    /// resumes from PC=0 with interrupts enabled.
    pub fn warm_reboot_from_flash(&mut self) {
        let dma_bytes = if let Some(bank) = self.bank.as_ref() {
            let guard = bank.read();
            let len = guard.pbc.flash.data.len().min(64 * 1024);
            guard.pbc.flash.data[..len].to_vec()
        } else {
            return;
        };

        self.reset_soc_in_place(BootMode::Warm);
        self.mem.load_binary(0, &dma_bytes);
        self.mem.dcache_invalidate_all().ok();
        self.mem.icache_invalidate_all();

        self.state.flag_e1 = true;
        self.state.flag_e2 = true;
        self.shadow_call_stack.clear();
    }

    pub fn step(&mut self) -> Result<(), Exception> {
        if self.state.halted || self.state.paused {
            return Ok(());
        }

        // Hook dispatch — only for UI breakpoints / watchpoints. The
        // old 35 SoC hooks are gone.
        if !self.hooks.is_empty() {
            if let Some(&hook) = self.hooks.get(&self.state.pc) {
                match hooks::execute_hook(hook, &mut self.state, &mut self.mem)? {
                    HookAction::Skip => return Ok(()),
                    HookAction::Continue => {}
                    HookAction::Pause => {
                        self.state.paused = true;
                        self.state.pause_reason =
                            crate::cpu::registers::PauseReason::Breakpoint(self.state.pc);
                        return Ok(());
                    }
                }
            }
        }

        // When sleeping, only tick timers and check interrupts.
        if self.state.sleeping {
            self.tick_timers_and_bank();
            self.check_uart_irq();
            if self.check_interrupts() {
                self.state.sleeping = false;
            }
            return Ok(());
        }

        // Save and clear delay state
        let delay_info = match self.state.delay_state {
            DelayState::DelaySlot { target, is_link } => {
                self.state.delay_state = DelayState::None;
                Some((target, is_link))
            }
            DelayState::None => None,
        };

        // Zero-overhead loop check (not during delay slots)
        if delay_info.is_none() && !self.state.flag_l {
            let lp_count = self.state.core_regs[REG_LP_COUNT as usize];
            if self.state.pc == self.state.aux_lp_end && lp_count > 0 {
                let new_count = lp_count - 1;
                self.state.core_regs[REG_LP_COUNT as usize] = new_count;
                if new_count > 0 {
                    self.state.pc = self.state.aux_lp_start;
                    return Ok(());
                }
            }
        }

        // Fetch and decode
        let decoded = decoder::decode(self.state.pc, &self.mem)?;
        let next_pc = self.state.pc + decoded.total_size();

        if self.trace {
            let src = self
                .debug_info
                .as_ref()
                .and_then(|di| di.lookup(self.state.pc));
            if let Some(loc) = src {
                eprintln!(
                    "[TRACE] PC=0x{:08X} size={} Z={} N={} C={} V={} {:?} @{}:{}",
                    self.state.pc,
                    decoded.total_size(),
                    self.state.flag_z as u8,
                    self.state.flag_n as u8,
                    self.state.flag_c as u8,
                    self.state.flag_v as u8,
                    decoded.inst,
                    loc.file,
                    loc.line
                );
            } else {
                eprintln!(
                    "[TRACE] PC=0x{:08X} size={} Z={} N={} C={} V={} {:?}",
                    self.state.pc,
                    decoded.total_size(),
                    self.state.flag_z as u8,
                    self.state.flag_n as u8,
                    self.state.flag_c as u8,
                    self.state.flag_v as u8,
                    decoded.inst
                );
            }
        }

        // For BL.D/JL.D: set blink to address AFTER the delay slot
        if let Some((_target, true)) = delay_info {
            self.state.write_core_reg(REG_BLINK, next_pc)?;
        }

        // Update CPU context on the bank for watchpoint / trace output.
        // Also set the .di flag from the current instruction so MMIO
        // history entries record whether the access used cache bypass.
        if let Some(ref bank) = self.bank {
            let mut guard = bank.write();
            guard.update_cpu_context(
                self.state.pc,
                self.state.core_regs[31],
                self.state.instruction_count,
            );
            guard.current_di = match &decoded.inst {
                Instruction::Store { cache_bypass, .. }
                | Instruction::Load { cache_bypass, .. } => *cache_bypass,
                _ => false,
            };
        }

        // Timing overlay: capture the load's effective address / SYNC BEFORE
        // execute, because a writeback (`.aw`/`.ab`) load mutates its base
        // register and would give the wrong post-execute EA (and hence a wrong
        // MMIO classification at the SRAM boundary).
        #[cfg(feature = "timing")]
        let timing_pre = if self.timing_shadow.is_some() {
            Some(Self::timing_load_info(&decoded.inst, &self.state))
        } else {
            None
        };

        // Execute
        self.state.pc_written = false;
        self.state.link_executed = false;
        executor::execute(&decoded, &mut self.state, &mut self.mem)?;

        // SR trace: log AUX register writes and record in history buffer.
        if let Instruction::StoreAux { src, addr } = &decoded.inst {
            let addr_val = executor::resolve_value(*addr, &self.state).unwrap_or(0);
            let val = executor::resolve_value(*src, &self.state).unwrap_or(0);
            if self.trace_sr {
                eprintln!(
                    "[SR] PC=0x{:05X} sr [0x{:X}], 0x{:08X}",
                    self.state.pc, addr_val, val
                );
            }
            if let Some(ref bank) = self.bank {
                bank.write().record_aux_write(addr_val, val);
            }
        }

        // PC update logic
        if let Some((target, _is_link)) = delay_info {
            self.state.pc = target;
        } else if matches!(self.state.delay_state, DelayState::DelaySlot { .. }) {
            self.state.pc = next_pc;
        } else if !self.state.pc_written {
            self.state.pc = next_pc;
        }

        self.state.instruction_count += 1;

        // Cycle-accurate cache + bus-contention overlay (opt-in `timing`
        // feature). Purely observational: the instruction has already executed
        // functionally above; the shadow only accounts for contention and, if
        // the fetch-vs-load starvation invariant fires, halts the CPU with a
        // BUS_DEADLOCK report. `decoded.pc` is the instruction address and
        // `self.state.pc` is the next fetch address (the fetch-ahead origin).
        #[cfg(feature = "timing")]
        if let Some((load, is_sync)) = timing_pre {
            let insn_pc = decoded.pc;
            let next_pc = self.state.pc;
            // Disjoint field borrows: the shadow and memory are separate fields.
            let shadow = self.timing_shadow.as_mut().unwrap();
            let hit_deadlock = shadow.on_instruction(insn_pc, load, next_pc, is_sync, &self.mem);
            self.state.debug_load_pending = shadow.load_pending;
            if hit_deadlock {
                if let Some(rep) = shadow.deadlock {
                    eprintln!(
                        "[BUS_DEADLOCK] pc=0x{:08X} load_ea=0x{:08X} mmio={} \
aliasing_fetch_line={:?} set_index={:?} starve={} cyc={} \
(DEBUG.LD=1: load starved by fixed-priority fetch stream)",
                        rep.pc,
                        rep.load_ea,
                        rep.load_mmio,
                        rep.aliasing_fetch_line.map(|l| format!("0x{l:08X}")),
                        rep.set_index,
                        rep.starve_cycles,
                        rep.cycle,
                    );
                }
                self.state.halted = true;
                self.state.pause_reason = crate::cpu::registers::PauseReason::BusDeadlock;
                return Ok(());
            }
        }

        // Spin detection: if the PC is the same as last step, the CPU is
        // spinning on `b .` (branch to self).
        //
        // What happens next is NOT the detector's call — it is decided by the
        // ARC timer control registers, exactly as on silicon
        // (`classify_spin`). Previously this site rebooted the SoC
        // unconditionally after 64 identical steps, which was wrong twice
        // over: it rebooted even when no watchdog was armed (silicon just
        // spins forever), and it did so ~64 instructions in rather than at the
        // ~15.6 M instructions a 100 ms watchdog takes at 156.25 MHz — a
        // reboot cadence ~250 000x too fast. That fabricated reboot loop is
        // divergence D1 of `docs/bugs/emu-reboot-halt-and-transfer-model-divergences.md`,
        // and it is the same failure shape as the D4 icache zero-fill: an
        // emulator-invented reboot standing where the real symptom was, wiping
        // the SRAM an investigator needed to read.
        //
        // Only active in SoC mode (bank present) — flat-mode tests have no
        // timers to consult and may legitimately reach zero-filled memory that
        // decodes as branch-to-self.
        if self.bank.is_some() {
            if self.state.pc == self.tight_loop_last_pc {
                self.tight_loop_count += 1;
                if self.tight_loop_count >= TIGHT_LOOP_THRESHOLD {
                    if self.tight_loop_halts {
                        // Harness mode: terminate this isolated run instead of
                        // rebooting the SoC. No log spam (spins are expected
                        // under fuzzing), no shared-state mutation.
                        self.state.halted = true;
                        return Ok(());
                    }
                    match self.classify_spin() {
                        SpinVerdict::WatchdogArmed => {
                            // The reset is real, but it belongs to the timer.
                            // Skip the simulated time the CPU would burn
                            // spinning so the host doesn't execute millions of
                            // no-op iterations, then let the timer fire it on
                            // its own terms (and at its own count).
                            self.fast_forward_armed_watchdog();
                            self.tight_loop_count = 0;
                        }
                        SpinVerdict::Interruptible => {
                            // An ISR can still break this. Keep executing —
                            // that is what the machine does. Re-arm the
                            // counter so the classification is re-checked
                            // periodically (the firmware may arm a watchdog,
                            // or mask interrupts, while spinning) without
                            // re-running it every single step.
                            self.tight_loop_count = 0;
                        }
                        SpinVerdict::Hung => {
                            // Nothing can ever move the PC again. Stop with
                            // the state INTACT and say so: registers, SRAM and
                            // the shadow call stack are the evidence, and a
                            // reboot would destroy all three.
                            eprintln!(
                                "[BCM55030] PC=0x{:08X} spinning with no watchdog armed and \
                                 interrupts masked -- permanent hang, halting with state intact",
                                self.state.pc
                            );
                            self.state.halted = true;
                            self.state.pause_reason =
                                crate::cpu::registers::PauseReason::SpinNoWatchdog(self.state.pc);
                            return Ok(());
                        }
                    }
                }
            } else {
                self.tight_loop_count = 0;
                self.tight_loop_last_pc = self.state.pc;
            }
        }

        // Shadow call stack: push on BL/JL, pop on J [blink].
        if let Some((_, true)) = delay_info {
            if self.shadow_call_stack.len() < 64 {
                self.shadow_call_stack.push(next_pc);
            }
        } else if self.state.link_executed {
            if self.shadow_call_stack.len() < 64 {
                let blink = self.state.core_regs[REG_BLINK as usize];
                self.shadow_call_stack.push(blink);
            }
        } else if self.state.pc_written {
            let blink = self.state.core_regs[REG_BLINK as usize];
            if self.state.pc == blink && !self.shadow_call_stack.is_empty() {
                self.shadow_call_stack.pop();
            }
        }

        if self.profiling_enabled {
            if let Some(&frame_pc) = self.shadow_call_stack.last() {
                *self.function_profile.entry(frame_pc).or_insert(0) += 1;
            }
        }

        // Watchpoint trap — if any read/write during executor set a
        // hit, pause the CPU. The next step() runs past the
        // watchpoint only after the UI clears `paused`.
        if let Some((wp_addr, wp_mode)) = self.mem.watchpoints.take_hit() {
            self.state.paused = true;
            self.state.pause_reason =
                crate::cpu::registers::PauseReason::Watch(wp_addr, wp_mode);
        }

        // Timers + peripheral bank
        self.tick_timers_and_bank();
        self.check_uart_irq();
        self.check_interrupts();

        Ok(())
    }

    pub fn run(&mut self, max_steps: u64) -> Result<(), Exception> {
        for _ in 0..max_steps {
            if self.state.halted || self.state.sleeping {
                break;
            }
            self.step()?;
        }
        Ok(())
    }

    /// Advance timers (CPU-side ARC Timer0/1) and the peripheral bank
    /// (which drives per-peripheral tick state).
    fn tick_timers_and_bank(&mut self) {
        // ARC Timer0/1: ~1.76 ticks per instruction (156.25 MHz, ~89 MIPS).
        // Bare-metal verified: 1000 NOPs = 7,026 COUNT0 ticks.
        self.timer_frac_acc += 176;
        while self.timer_frac_acc >= 100 {
            self.timer_frac_acc -= 100;
            self.tick_arc_timers_once();
        }

        // Peripheral bank tick (EPON free-running counter, PBC busy
        // counters, BSC busy counters, UART mpsc drain, future SerDes /
        // MACsec / etc.).
        self.bank_tick_accumulator += 1;
        if self.bank_tick_accumulator >= BANK_TICK_PRESCALER {
            self.bank_tick_accumulator = 0;
            if let Some(ref bank) = self.bank {
                bank.write().tick(BANK_TICK_PRESCALER);
            }
            let _ = self.mem.drain_datapath_public();
        }
    }

    /// Is this timer configured as a watchdog (W bit set, and actually
    /// counting toward a non-zero limit)?
    fn timer_is_watchdog(control: u32, limit: u32) -> bool {
        control & TIMER_CTRL_W != 0 && limit != 0
    }

    /// Decide what a stalled PC means, from the machine state alone.
    ///
    /// This is the whole point of the D1 fix: the outcome of a spin is a
    /// property of the timer control registers and the interrupt state, not of
    /// how many host iterations the emulator felt like running.
    pub fn classify_spin(&self) -> SpinVerdict {
        let s = &self.state;
        if Self::timer_is_watchdog(s.aux_control0, s.aux_limit0)
            || Self::timer_is_watchdog(s.aux_control1, s.aux_limit1)
        {
            return SpinVerdict::WatchdogArmed;
        }
        // Can anything still interrupt us? Both the global enables (E1/E2) and
        // an actual source have to be there. A timer with IE set is a source;
        // so is any already-pending or peripheral-driven line.
        let globally_enabled = s.flag_e1 || s.flag_e2;
        let timer_irq_source = (s.aux_control0 & TIMER_CTRL_IE != 0 && s.aux_limit0 != 0)
            || (s.aux_control1 & TIMER_CTRL_IE != 0 && s.aux_limit1 != 0);
        let external_source = s.aux_irq_pending != 0 || self.bank.as_ref().map(|b| b.read().irq_pending != 0).unwrap_or(false);
        if globally_enabled && (timer_irq_source || external_source) {
            SpinVerdict::Interruptible
        } else {
            SpinVerdict::Hung
        }
    }

    /// Skip the simulated time a spinning CPU would burn before its armed
    /// watchdog fires: advance each armed watchdog's COUNT to just below its
    /// LIMIT, so the very next tick fires it through the normal path.
    ///
    /// The reset still happens in `tick_arc_timers_once`, at the count the
    /// firmware programmed — we only decline to execute the ~15.6 M no-op
    /// iterations in between, which no observer can distinguish (the CPU is
    /// provably not changing any state: same PC, `b .`).
    fn fast_forward_armed_watchdog(&mut self) {
        if Self::timer_is_watchdog(self.state.aux_control0, self.state.aux_limit0) {
            self.state.aux_count0 = self.state.aux_limit0.saturating_sub(1);
        }
        if Self::timer_is_watchdog(self.state.aux_control1, self.state.aux_limit1) {
            self.state.aux_count1 = self.state.aux_limit1.saturating_sub(1);
        }
    }

    /// Increment ARC Timer0 and Timer1 by one tick each.
    fn tick_arc_timers_once(&mut self) {
        let mut watchdog_reset = false;
        // Timer 0 (IRQ 3)
        self.state.aux_count0 = self.state.aux_count0.wrapping_add(1);
        if self.state.aux_limit0 != 0 && self.state.aux_count0 >= self.state.aux_limit0 {
            self.state.aux_control0 |= 0x08;
            if self.state.aux_control0 & TIMER_CTRL_IE != 0 {
                self.state.aux_irq_pending |= 1 << 3;
            }
            // W (bit 2): "enable watchdog reset signal"
            // (`docs/isa/05-registers.md` Figure 66). Modelled here rather
            // than faked by the spin detector, so a watchdog resets the chip
            // when the firmware armed one — and ONLY then.
            if self.state.aux_control0 & TIMER_CTRL_W != 0 {
                watchdog_reset = true;
            }
            if self.state.aux_control0 & 0x02 == 0 {
                self.state.aux_count0 = 0;
            }
        }
        // Timer 1 (IRQ 7 on BCM55030).
        self.state.aux_count1 = self.state.aux_count1.wrapping_add(1);
        if self.state.aux_limit1 != 0 && self.state.aux_count1 >= self.state.aux_limit1 {
            self.state.aux_control1 |= 0x08;
            if self.state.aux_control1 & TIMER_CTRL_IE != 0 {
                self.state.aux_irq_pending |= 1 << self.state.timer1_irq;
            }
            if self.state.aux_control1 & TIMER_CTRL_W != 0 {
                watchdog_reset = true;
            }
            self.state.aux_count1 = 0;
        }
        if watchdog_reset && self.bank.is_some() {
            eprintln!(
                "[BCM55030] Timer watchdog (CONTROL.W) expired at PC=0x{:08X} -- warm reset",
                self.state.pc
            );
            self.warm_reboot_from_flash();
        }
    }

    /// Poll peripheral IRQ lines via the bank. Prescaled to avoid
    /// hot-path contention on every instruction.
    fn check_uart_irq(&mut self) {
        if self.state.instruction_count % UART_PRESCALER != 0 {
            return;
        }
        if let Some(ref bank) = self.bank {
            let b = bank.read();
            if b.uart.irq_pending() != 0 {
                self.state.aux_irq_pending |= 1 << UART_IRQ;
            }
            self.state.aux_irq_pending |= b.irq_pending;
        }
    }

    /// Check and take pending interrupts.
    fn check_interrupts(&mut self) -> bool {
        if self.state.delay_state != DelayState::None {
            return false;
        }

        // No per-line enable mask. `AUX_IENABLE` (0x40C) is unimplemented on
        // this integration: a read returns `IDENTITY` and a write is
        // discarded, so nothing gates a line here. Delivery depends only on
        // the peripheral asserting (which is what set the pending bit), the
        // line's level from `AUX_IRQ_LEV` (0x200), and the matching `E1`/`E2`
        // bit of `STATUS32`, all applied below.
        // -- OBSERVED, DATASHEET §6.1: Timer 1 / IRQ 7 was delivered 19 001
        // times in a run whose own `0x40C` readback showed bit 7 clear.
        let pending = self.state.aux_irq_pending;
        if pending == 0 {
            return false;
        }

        let irq = pending.trailing_zeros();
        if irq >= 32 {
            return false;
        }

        let is_level2 = (self.state.aux_irq_lev >> irq) & 1 != 0;

        if is_level2 {
            if !self.state.flag_e2 || self.state.flag_a2 {
                return false;
            }
            self.state.aux_status32_l2 = self.state.status32();
            self.state.aux_bta_l2 = self.state.aux_bta;
            self.state.core_regs[REG_ILINK2 as usize] = self.state.pc;
            self.state.aux_icause2 = irq;
            // L2 entry: keep E1 set (matches observed behaviour). Clearing
            // both E1 and E2 per the generic ARC spec caused a boot
            // regression on this integration, so it is deliberately not done.
            self.state.flag_e2 = false;
            self.state.flag_a2 = true;
            self.state.flag_de = false;
            self.state.flag_u = false;
            self.state.flag_l = true;

            self.state.irq_shadow_r0_r3[0] = self.state.core_regs[0];
            self.state.irq_shadow_r0_r3[1] = self.state.core_regs[1];
            self.state.irq_shadow_r0_r3[2] = self.state.core_regs[2];
            self.state.irq_shadow_r0_r3[3] = self.state.core_regs[3];
        } else {
            if !self.state.flag_e1 || self.state.flag_a1 {
                return false;
            }
            self.state.aux_status32_l1 = self.state.status32();
            self.state.aux_bta_l1 = self.state.aux_bta;
            self.state.core_regs[REG_ILINK1 as usize] = self.state.pc;
            self.state.aux_icause1 = irq;
            self.state.flag_e1 = false;
            self.state.flag_e2 = false;
            self.state.flag_a1 = true;
            self.state.flag_de = false;
            self.state.flag_u = false;
            self.state.flag_l = true;

            self.state.irq_shadow_r0_r3[0] = self.state.core_regs[0];
            self.state.irq_shadow_r0_r3[1] = self.state.core_regs[1];
            self.state.irq_shadow_r0_r3[2] = self.state.core_regs[2];
            self.state.irq_shadow_r0_r3[3] = self.state.core_regs[3];
        }

        self.state.aux_irq_pending &= !(1 << irq);

        // Dual-aperture interrupt-vector fetch. The 16-channel NCO
        // table is aliased over the ARC interrupt-vector range on a
        // separate physical bus: `.di` stores program it (mirrored in
        // `Memory::write_*_data`), and the ARC interrupt unit fetches
        // its vector from the NCO — NOT from SRAM at `base + N*8`.
        // Each installed slot is a `j @<absolute>` (`2020 0f80 hi lo`)
        // so taking the interrupt jumps straight to `<hi:lo>`. The NCO
        // table serving as the ARC interrupt-vector table is verified
        // against real hardware. When the channel was never programmed
        // as a `j @limm` vector the model falls
        // back to the SRAM IVT slot — covering reset, exception
        // vectors, and any not-yet-installed channel.
        let vector = irq;
        let nco_target = self
            .bank
            .as_ref()
            .and_then(|b| b.read().nco.interrupt_vector(vector as u8));
        self.state.pc = match nco_target {
            Some(target) => target,
            None => self.state.aux_int_vector_base + vector * 8,
        };
        self.state.pc_written = true;

        if self.trace {
            eprintln!(
                "[IRQ] Took level {} interrupt IRQ {} → vector 0x{:08X}",
                if is_level2 { 2 } else { 1 },
                irq,
                self.state.pc
            );
        }

        true
    }
}
