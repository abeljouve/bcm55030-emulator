//! CPU worker thread body. Owns the live `Cpu` and drains the
//! `CpuCommand` channel. Publishes `EmulatorSnapshot` updates on
//! three triggers: 16 ms wall-clock elapsed, state transitions, or
//! explicit `Snapshot` / request commands.

use std::collections::HashMap;
use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::time::{Duration, Instant};

use crate::cpu::registers::PauseReason;
use crate::cpu::Cpu;
use crate::emu::command::{CpuCommand, OneshotSender, SpeedLimit};
use crate::emu::named_snapshot::{NamedSnapshot, SnapshotInfo};
use crate::emu::handle::EmulatorHandle;
use crate::emu::snapshot::{
    CpuSnapshot, DcacheSnapshot, EmulatorSnapshot, RunState, SramSnapshot,
};
use crate::hooks::Hook;
use crate::memory::Watchpoint;
use crate::soc::bank::BootMode;

/// Publish cadence while the CPU is running continuously.
const PUBLISH_INTERVAL: Duration = Duration::from_millis(16);

/// Cadence the worker wakes up on while paused, so the UI still
/// gets periodic snapshots (insns/sec decay, fresh timestamps).
const PAUSED_WAKE: Duration = Duration::from_millis(200);

/// Length of a throttle "budget window". Every window the worker
/// is allowed to execute `target_ips * THROTTLE_WINDOW_SECS`
/// instructions before it sleeps until the window elapses.
const THROTTLE_WINDOW: Duration = Duration::from_millis(10);

/// Signature of the constructor callback the worker uses to rebuild
/// `Cpu` on `CpuCommand::Reset`. `main.rs` supplies a closure
/// returning `Cpu::new_bcm55030(boot_mode)`; tests can supply a
/// closure returning a flat-mode `Cpu`.
pub type ResetFn = Box<dyn FnMut(BootMode) -> Cpu + Send>;

/// Worker state. Private — everything outside this module goes
/// through `run`.
struct Worker {
    cpu: Cpu,
    handle: EmulatorHandle,
    rx: Receiver<CpuCommand>,
    reset_fn: ResetFn,

    running: bool,
    remaining_insns: Option<u64>,
    run_to: Option<u32>,

    breakpoints: Vec<u32>,

    step_over_target: Option<u32>,

    last_publish: Instant,
    ips_window_start: Instant,
    ips_window_insns: u64,
    last_insns_per_sec: u32,

    /// Active throttle setting. `Unlimited` means no sleeping.
    speed_limit: SpeedLimit,
    /// Start of the current throttle budget window.
    throttle_window_start: Instant,
    /// Instructions executed inside the current throttle window.
    throttle_window_insns: u32,

    /// Dense coverage histogram indexed by `pc / 2`. Each entry
    /// saturates at `u32::MAX`. 512 KB SRAM / 2 = 256 Ki slots
    /// = 1 MB of u32s; small enough to carry on the worker.
    coverage: Vec<u32>,

    snapshots: HashMap<String, NamedSnapshot>,
}

/// Run the worker loop until a `Shutdown` command arrives or all
/// command senders have been dropped. Blocks the calling thread.
pub fn run(cpu: Cpu, handle: EmulatorHandle, rx: Receiver<CpuCommand>, reset_fn: ResetFn) {
    let now = Instant::now();
    let mut w = Worker {
        cpu,
        handle,
        rx,
        reset_fn,
        running: false,
        remaining_insns: None,
        run_to: None,
        breakpoints: Vec::new(),
        step_over_target: None,
        last_publish: now,
        ips_window_start: now,
        ips_window_insns: 0,
        last_insns_per_sec: 0,
        speed_limit: SpeedLimit::Unlimited,
        throttle_window_start: now,
        throttle_window_insns: 0,
        coverage: vec![0u32; crate::memory::SRAM_SIZE / 2],
        snapshots: HashMap::new(),
    };
    w.publish_snapshot();
    w.main_loop();
}

impl Worker {
    fn main_loop(&mut self) {
        loop {
            if self.drain_commands() {
                return;
            }

            if self.should_step() {
                let pc_before = self.cpu.state.pc;
                if let Err(e) = self.cpu.step() {
                    eprintln!("[cpu_worker] step error: {:?}", e);
                    self.running = false;
                    self.cpu.state.paused = true;
                    self.cpu.state.pause_reason = PauseReason::Halted;
                    self.publish_snapshot();
                    continue;
                }
                self.ips_window_insns += 1;
                let slot = (pc_before >> 1) as usize;
                if slot < self.coverage.len() {
                    self.coverage[slot] = self.coverage[slot].saturating_add(1);
                }
                self.apply_throttle();

                if let Some(n) = self.remaining_insns {
                    if n <= 1 {
                        self.remaining_insns = None;
                        self.running = false;
                        self.cpu.state.paused = true;
                        self.cpu.state.pause_reason = PauseReason::UserPause;
                        self.publish_snapshot();
                        continue;
                    } else {
                        self.remaining_insns = Some(n - 1);
                    }
                }

                if let Some(target) = self.run_to {
                    if self.cpu.state.pc == target {
                        self.run_to = None;
                        self.running = false;
                        self.cpu.state.paused = true;
                        self.cpu.state.pause_reason = PauseReason::UserPause;
                        self.publish_snapshot();
                        continue;
                    }
                }

                if self.cpu.state.paused || self.cpu.state.halted {
                    self.running = false;
                    self.finalize_step_over_if_hit();
                    self.publish_snapshot();
                    continue;
                }

                if self.last_publish.elapsed() >= PUBLISH_INTERVAL {
                    self.recompute_ips();
                    self.publish_snapshot();
                }
            } else {
                match self.rx.recv_timeout(PAUSED_WAKE) {
                    Ok(cmd) => {
                        if !self.handle_cmd(cmd) {
                            return;
                        }
                    }
                    Err(RecvTimeoutError::Timeout) => {
                        self.recompute_ips();
                        self.publish_snapshot();
                    }
                    Err(RecvTimeoutError::Disconnected) => return,
                }
            }
        }
    }

    #[inline]
    fn should_step(&self) -> bool {
        self.running && !self.cpu.state.halted && !self.cpu.state.paused
    }

    /// Drain every already-queued command without blocking. Returns
    /// `true` if the worker must exit (Shutdown or disconnect).
    fn drain_commands(&mut self) -> bool {
        loop {
            match self.rx.try_recv() {
                Ok(cmd) => {
                    if !self.handle_cmd(cmd) {
                        return true;
                    }
                }
                Err(TryRecvError::Empty) => return false,
                Err(TryRecvError::Disconnected) => return true,
            }
        }
    }

    /// Handle a single command. Returns `false` on `Shutdown`.
    fn handle_cmd(&mut self, cmd: CpuCommand) -> bool {
        match cmd {
            CpuCommand::Run { max_insns } => {
                self.cpu.state.paused = false;
                self.cpu.state.pause_reason = PauseReason::None;
                self.remaining_insns = max_insns;
                self.running = true;
                self.ips_window_start = Instant::now();
                self.ips_window_insns = 0;
            }
            CpuCommand::RunTo { address } => {
                self.cpu.state.paused = false;
                self.cpu.state.pause_reason = PauseReason::None;
                self.run_to = Some(address);
                self.running = true;
                self.ips_window_start = Instant::now();
                self.ips_window_insns = 0;
            }
            CpuCommand::Pause => {
                self.running = false;
                self.cpu.state.paused = true;
                if self.cpu.state.pause_reason == PauseReason::None {
                    self.cpu.state.pause_reason = PauseReason::UserPause;
                }
                self.publish_snapshot();
            }
            CpuCommand::StepOne => {
                self.cpu.state.paused = false;
                self.cpu.state.pause_reason = PauseReason::None;
                self.remaining_insns = Some(1);
                self.running = true;
            }
            CpuCommand::StepN(n) => {
                self.cpu.state.paused = false;
                self.cpu.state.pause_reason = PauseReason::None;
                self.remaining_insns = Some(n as u64);
                self.running = true;
            }
            CpuCommand::StepOver => {
                // Install a temporary breakpoint at blink and run.
                let blink = self.cpu.state.core_regs[31];
                if !self.cpu.hooks.contains_key(&blink) {
                    self.cpu.hooks.insert(blink, Hook::Breakpoint);
                    self.step_over_target = Some(blink);
                }
                self.cpu.state.paused = false;
                self.cpu.state.pause_reason = PauseReason::None;
                self.running = true;
            }
            CpuCommand::Reset {
                boot_mode,
                keep_breakpoints,
            } => {
                let saved = if keep_breakpoints {
                    self.breakpoints.clone()
                } else {
                    Vec::new()
                };
                self.reset_cpu(boot_mode);
                self.breakpoints.clear();
                for addr in saved {
                    self.cpu.hooks.insert(addr, Hook::Breakpoint);
                    self.breakpoints.push(addr);
                }
                self.running = false;
                self.step_over_target = None;
                self.run_to = None;
                self.remaining_insns = None;
                self.publish_snapshot();
            }
            CpuCommand::LoadFirmware {
                path,
                mode: _,
                boot_mode,
                flash_path: _,
                entry_point,
                keep_breakpoints,
                response,
            } => {
                let result = self.load_firmware(path, boot_mode, entry_point, keep_breakpoints);
                let _ = response.send(result);
                self.publish_snapshot();
            }
            CpuCommand::SetBreakpoint { address, response } => {
                if !self.breakpoints.contains(&address) {
                    self.breakpoints.push(address);
                    self.cpu.hooks.insert(address, Hook::Breakpoint);
                }
                let _ = response.send(self.breakpoints.len() - 1);
            }
            CpuCommand::RemoveBreakpoint { address } => {
                self.breakpoints.retain(|a| *a != address);
                self.cpu.hooks.remove(&address);
            }
            CpuCommand::SetWatchpoint {
                addr,
                size,
                mode,
                response,
            } => {
                let idx = self.cpu.mem.watchpoints.add(Watchpoint { addr, size, mode });
                let _ = response.send(idx);
            }
            CpuCommand::RemoveWatchpoint { index } => {
                self.cpu.mem.watchpoints.remove(index);
            }
            CpuCommand::WriteRegister {
                name,
                value,
                response,
            } => {
                let result = write_register_by_name(&mut self.cpu, &name, value);
                let _ = response.send(result);
                self.publish_snapshot();
            }
            CpuCommand::WriteSram {
                addr,
                bytes,
                response,
            } => {
                let result = write_sram(&mut self.cpu, addr, &bytes);
                let _ = response.send(result);
            }
            CpuCommand::RequestSram { response } => {
                let snap = SramSnapshot {
                    bytes: self.cpu.mem.sram_snapshot(),
                    timestamp: Instant::now(),
                };
                let _ = response.send(snap);
            }
            CpuCommand::RequestDcache { response } => {
                send_dcache(&self.cpu, response);
            }
            CpuCommand::Snapshot { response } => {
                let snap = self.build_snapshot();
                let _ = response.send(snap);
            }
            CpuCommand::SetSpeed { limit } => {
                self.speed_limit = limit;
                self.throttle_window_start = Instant::now();
                self.throttle_window_insns = 0;
                self.publish_snapshot();
            }
            CpuCommand::RequestCoverage { response } => {
                let sparse: Vec<(u32, u32)> = self
                    .coverage
                    .iter()
                    .enumerate()
                    .filter_map(|(slot, &count)| {
                        if count > 0 {
                            Some(((slot as u32) << 1, count))
                        } else {
                            None
                        }
                    })
                    .collect();
                let _ = response.send(sparse);
            }
            CpuCommand::ClearCoverage => {
                for entry in self.coverage.iter_mut() {
                    *entry = 0;
                }
            }
            CpuCommand::RequestCallStack { response } => {
                let _ = response.send(self.cpu.shadow_call_stack.clone());
            }
            CpuCommand::RequestFunctionProfile { response } => {
                let mut entries: Vec<(u32, u64)> = self.cpu.function_profile.iter().map(|(&k, &v)| (k, v)).collect();
                entries.sort_by(|a, b| b.1.cmp(&a.1));
                let _ = response.send(entries);
            }
            CpuCommand::SetProfiling { enabled } => {
                self.cpu.profiling_enabled = enabled;
                if !enabled {
                    self.cpu.function_profile.clear();
                }
            }
            CpuCommand::DiffSnapshots { a, b, response } => {
                let result = self.diff_named_snapshots(&a, &b);
                let _ = response.send(result);
            }
            CpuCommand::SaveSnapshot { name, response } => {
                let result = self.save_named_snapshot(&name);
                let _ = response.send(result);
            }
            CpuCommand::RestoreSnapshot { name, response } => {
                let result = self.restore_named_snapshot(&name);
                if result.is_ok() {
                    self.publish_snapshot();
                }
                let _ = response.send(result);
            }
            CpuCommand::ListSnapshots { response } => {
                let list: Vec<SnapshotInfo> = self
                    .snapshots
                    .values()
                    .map(|s| SnapshotInfo {
                        name: s.name.clone(),
                        instruction_count: s.instruction_count,
                        pc: s.pc,
                        timestamp: s.timestamp.clone(),
                        size_bytes: s.size_bytes(),
                    })
                    .collect();
                let _ = response.send(list);
            }
            CpuCommand::DeleteSnapshot { name, response } => {
                let existed = self.snapshots.remove(&name).is_some();
                let _ = response.send(existed);
            }
            CpuCommand::Shutdown => {
                self.running = false;
                self.cpu.state.halted = true;
                self.publish_snapshot();
                return false;
            }
        }
        true
    }

    /// Apply the active throttle: if a speed cap is set and the
    /// current budget window has already issued its quota, sleep
    /// until the window elapses, then rewind the window counters.
    ///
    /// Fenêtres dynamiques: pour ≥ 100 ips on utilise des
    /// fenêtres fixes de `THROTTLE_WINDOW` (10 ms) et on laisse
    /// tourner plusieurs instructions par fenêtre. Pour < 100
    /// ips, une fenêtre 10 ms est trop courte (1 insn / 10 ms =
    /// 100 ips minimum), donc on étire la fenêtre à
    /// `1s / target_ips` et on n'exécute qu'une seule insn par
    /// fenêtre — ce qui descend naturellement jusqu'à 1 ips.
    fn apply_throttle(&mut self) {
        let Some(target_ips) = self.speed_limit.as_ips() else {
            return;
        };
        self.throttle_window_insns += 1;
        let (window, budget) = if target_ips >= 100 {
            let budget = ((target_ips as u64)
                * (THROTTLE_WINDOW.as_micros() as u64)
                / 1_000_000)
                .max(1) as u32;
            (THROTTLE_WINDOW, budget)
        } else {
            let micros = (1_000_000u64 / target_ips.max(1) as u64).max(1);
            (Duration::from_micros(micros), 1u32)
        };
        if self.throttle_window_insns < budget {
            return;
        }
        let elapsed = self.throttle_window_start.elapsed();
        if elapsed < window {
            std::thread::sleep(window - elapsed);
        }
        self.throttle_window_start = Instant::now();
        self.throttle_window_insns = 0;
    }

    /// Reset the live `Cpu` for a `Reset` or `LoadFirmware`
    /// command. In SoC mode (bank is Some) we reset **in place**
    /// via `Cpu::reset_soc_in_place` so the bank `Arc` shared with
    /// the UI and MCP threads through `EmulatorHandle` stays
    /// valid. In flat mode (tests) we fall back to the
    /// constructor closure supplied at worker-spawn time.
    fn reset_cpu(&mut self, boot_mode: BootMode) {
        if self.cpu.bank().is_some() {
            self.cpu.reset_soc_in_place(boot_mode);
        } else {
            self.cpu = (self.reset_fn)(boot_mode);
        }
        for entry in self.coverage.iter_mut() {
            *entry = 0;
        }
    }

    /// Implement `CpuCommand::LoadFirmware`: rebuild a fresh
    /// `Cpu`, copy the file into the PBC SPI flash backing store,
    /// perform the 64 KB boot DMA into SRAM, and set the entry
    /// point. Mirrors the sequence `src/bin/arc700.rs` uses for
    /// the CLI path.
    fn load_firmware(
        &mut self,
        path: std::path::PathBuf,
        boot_mode: BootMode,
        entry_point: u32,
        keep_breakpoints: bool,
    ) -> Result<crate::emu::command::LoadFirmwareResult, String> {
        let data = std::fs::read(&path)
            .map_err(|e| format!("reading {}: {e}", path.display()))?;

        let saved_bps = if keep_breakpoints {
            self.breakpoints.clone()
        } else {
            Vec::new()
        };

        self.reset_cpu(boot_mode);
        if let Some(bank) = self.cpu.bank() {
            bank.write().uart.clear_tx_log();
        }
        self.breakpoints.clear();
        for addr in saved_bps {
            self.cpu.hooks.insert(addr, Hook::Breakpoint);
            self.breakpoints.push(addr);
        }

        let (copy_len, flash_bytes) = {
            let bank_arc = match self.cpu.bank() {
                Some(b) => b.clone(),
                None => {
                    return Err(
                        "load_firmware requires SoC mode (bank is None in flat mode)".into(),
                    )
                }
            };
            let mut bank = bank_arc.write();
            let flash_size = bank.pbc.flash.data.len();
            let copy_len = data.len().min(flash_size);
            bank.pbc.flash.data[..copy_len].copy_from_slice(&data[..copy_len]);
            // Capture a baseline after the load so the memory
            // viewer can tint any later write relative to the
            // freshly-loaded image.
            bank.pbc.flash.capture_baseline();
            (copy_len, flash_size)
        };

        // 64 KB DMA flash → SRAM, same as the CLI binary does via
        // `boot_from_flash`.
        const BOOT_DMA_SIZE: usize = 64 * 1024;
        let dma_len = {
            let bank = self.cpu.bank().unwrap().read();
            bank.pbc.flash.data.len().min(BOOT_DMA_SIZE)
        };
        let dma_bytes = {
            let bank = self.cpu.bank().unwrap().read();
            bank.pbc.flash.data[..dma_len].to_vec()
        };
        self.cpu.mem.load_binary(0, &dma_bytes);
        // DATASHEET §5.4: the on-chip DMA engine writes directly into
        // SRAM and the integration layer broadcasts those writes as
        // I/D-cache invalidations. The 64 KB boot DMA is such a write,
        // so the caches must not retain pre-DMA lines for `0x0..0x80`.
        self.cpu.mem.dcache_invalidate_all().ok();
        self.cpu.mem.icache_invalidate_all();

        // See `src/bin/arc700.rs` for the IRQ-mask/flag handling
        // that this mirrors. The silicon resets `IENABLE` to all-
        // ones; the `E1`/`E2` presets only apply in warm mode.
        self.cpu.state.aux_ienable = 0xFFFFFFFF;
        if boot_mode == BootMode::Warm {
            self.cpu.state.flag_e1 = true;
            self.cpu.state.flag_e2 = true;
        }
        self.cpu.state.pc = entry_point;

        self.cpu.state.paused = true;
        self.cpu.state.pause_reason = PauseReason::UserPause;
        self.running = false;
        self.remaining_insns = None;
        self.run_to = None;
        self.step_over_target = None;

        // Publish firmware metadata for the status bar / MCP
        // `get_firmware_info` tool.
        *self.handle.firmware_info.lock() =
            Some(crate::emu::handle::FirmwareInfo {
                path,
                mode: crate::emu::command::FirmwareMode::Soc,
                boot_mode,
                entry_point,
                flash_size: flash_bytes,
                flash_loaded: true,
            });

        Ok(crate::emu::command::LoadFirmwareResult {
            loaded_bytes: copy_len,
            entry_point,
            flash_bytes,
        })
    }

    fn finalize_step_over_if_hit(&mut self) {
        if let Some(target) = self.step_over_target {
            if let PauseReason::Breakpoint(addr) = self.cpu.state.pause_reason {
                if addr == target {
                    self.cpu.hooks.remove(&target);
                    self.step_over_target = None;
                    if !self.breakpoints.contains(&target) {
                        // user didn't have a real breakpoint here
                    } else {
                        // reinstall the user's real breakpoint
                        self.cpu.hooks.insert(target, Hook::Breakpoint);
                    }
                }
            }
        }
    }

    fn recompute_ips(&mut self) {
        let elapsed = self.ips_window_start.elapsed();
        if elapsed >= Duration::from_millis(500) {
            let secs = elapsed.as_secs_f64().max(1e-9);
            self.last_insns_per_sec = ((self.ips_window_insns as f64) / secs) as u32;
            self.ips_window_start = Instant::now();
            self.ips_window_insns = 0;
        }
    }

    fn build_snapshot(&self) -> EmulatorSnapshot {
        let run_state = if self.cpu.state.halted {
            RunState::Halted
        } else if self.cpu.state.paused {
            match self.cpu.state.pause_reason {
                PauseReason::Breakpoint(_) => RunState::Breakpoint,
                _ => RunState::Paused,
            }
        } else if self.cpu.state.sleeping {
            RunState::Sleeping
        } else if self.running {
            RunState::Running
        } else {
            RunState::Paused
        };

        let peripherals = self
            .cpu
            .bank()
            .map(|b| b.read().snapshot_all())
            .unwrap_or_default();

        let boot_mode = self.handle.snapshot.lock().boot_mode;
        let watchpoints = self.cpu.mem.watchpoints.entries.clone();

        EmulatorSnapshot {
            cpu: CpuSnapshot::from_state(&self.cpu.state),
            peripherals,
            run_state,
            boot_mode,
            bank_tick_accumulator: 0,
            insns_per_sec: self.last_insns_per_sec,
            breakpoints: self.breakpoints.clone(),
            watchpoints,
            pause_reason: self.cpu.state.pause_reason,
            speed_limit: self.speed_limit,
            timestamp: Instant::now(),
        }
    }

    fn publish_snapshot(&mut self) {
        let snap = self.build_snapshot();
        *self.handle.snapshot.lock() = snap;
        self.last_publish = Instant::now();
    }

    fn diff_named_snapshots(
        &self,
        a: &str,
        b: &str,
    ) -> Result<crate::emu::named_snapshot::SnapshotDiff, String> {
        use crate::emu::named_snapshot::{SnapshotDiff, SnapshotRegDiff};

        let sa = self
            .snapshots
            .get(a)
            .ok_or_else(|| format!("snapshot '{}' not found", a))?;
        let sb = self
            .snapshots
            .get(b)
            .ok_or_else(|| format!("snapshot '{}' not found", b))?;

        let mut reg_diffs = Vec::new();
        let names = [
            "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12",
            "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24",
            "r25", "gp", "fp", "sp", "ilink1", "ilink2", "blink",
        ];
        for (i, name) in names.iter().enumerate() {
            let va = sa.cpu_state.core_regs[i];
            let vb = sb.cpu_state.core_regs[i];
            if va != vb {
                reg_diffs.push(SnapshotRegDiff {
                    name: name.to_string(),
                    a: va,
                    b: vb,
                });
            }
        }
        if sa.cpu_state.core_regs[60] != sb.cpu_state.core_regs[60] {
            reg_diffs.push(SnapshotRegDiff {
                name: "lp_count".into(),
                a: sa.cpu_state.core_regs[60],
                b: sb.cpu_state.core_regs[60],
            });
        }
        if sa.cpu_state.status32() != sb.cpu_state.status32() {
            reg_diffs.push(SnapshotRegDiff {
                name: "status32".into(),
                a: sa.cpu_state.status32(),
                b: sb.cpu_state.status32(),
            });
        }

        let sram_changed = sa
            .sram
            .iter()
            .zip(sb.sram.iter())
            .filter(|(a, b)| a != b)
            .count();

        Ok(SnapshotDiff {
            register_diffs: reg_diffs,
            pc_a: sa.pc,
            pc_b: sb.pc,
            insn_a: sa.instruction_count,
            insn_b: sb.instruction_count,
            sram_changed_bytes: sram_changed,
        })
    }

    fn save_named_snapshot(&mut self, name: &str) -> Result<SnapshotInfo, String> {
        let sram = self.cpu.mem.sram_snapshot();
        let (dcache, icache) = self.cpu.mem.save_cache_state();
        let bank_state = self
            .cpu
            .bank()
            .map(|b| b.read().capture_for_snapshot());
        let now = {
            let d = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default();
            format!("{}s", d.as_secs())
        };

        let snap = NamedSnapshot {
            name: name.to_string(),
            timestamp: now.clone(),
            instruction_count: self.cpu.state.instruction_count,
            pc: self.cpu.state.pc,
            cpu_state: self.cpu.state.clone(),
            sram,
            dcache,
            icache,
            bank_state,
            timer_frac_acc: self.cpu.timer_frac_acc,
            bank_tick_accumulator: self.cpu.bank_tick_accumulator,
            shadow_call_stack: self.cpu.shadow_call_stack.clone(),
            function_profile: self.cpu.function_profile.clone(),
            profiling_enabled: self.cpu.profiling_enabled,
        };

        let info = SnapshotInfo {
            name: name.to_string(),
            instruction_count: snap.instruction_count,
            pc: snap.pc,
            timestamp: now,
            size_bytes: snap.size_bytes(),
        };

        self.snapshots.insert(name.to_string(), snap);
        Ok(info)
    }

    fn restore_named_snapshot(&mut self, name: &str) -> Result<SnapshotInfo, String> {
        let snap = self
            .snapshots
            .get(name)
            .ok_or_else(|| format!("snapshot '{}' not found", name))?;

        let info = SnapshotInfo {
            name: name.to_string(),
            instruction_count: snap.instruction_count,
            pc: snap.pc,
            timestamp: snap.timestamp.clone(),
            size_bytes: snap.size_bytes(),
        };

        self.cpu.state = snap.cpu_state.clone();
        self.cpu.mem.restore_sram(&snap.sram);

        // Invalidate both caches — SRAM already has correct data, the
        // firmware re-warms caches naturally. Avoids cloning boxed cache
        // arrays through private fields.
        let _ = self.cpu.mem.dcache_invalidate_all();
        self.cpu.mem.icache_invalidate_all();

        if let Some(ref bank_state) = snap.bank_state {
            if let Some(bank) = self.cpu.bank() {
                bank.write().restore_from_snapshot(bank_state.clone());
            }
        }

        self.cpu.timer_frac_acc = snap.timer_frac_acc;
        self.cpu.bank_tick_accumulator = snap.bank_tick_accumulator;
        self.cpu.shadow_call_stack = snap.shadow_call_stack.clone();
        self.cpu.function_profile = snap.function_profile.clone();
        self.cpu.profiling_enabled = snap.profiling_enabled;

        self.running = false;
        self.cpu.state.paused = true;
        self.cpu.state.pause_reason = PauseReason::UserPause;

        Ok(info)
    }
}

fn write_register_by_name(cpu: &mut Cpu, name: &str, value: u32) -> Result<(), String> {
    let lower = name.to_ascii_lowercase();
    match lower.as_str() {
        "pc" => {
            cpu.state.pc = value;
            Ok(())
        }
        "status32" => {
            cpu.state.set_status32(value);
            Ok(())
        }
        "sp" => {
            cpu.state.core_regs[28] = value;
            Ok(())
        }
        "fp" => {
            cpu.state.core_regs[27] = value;
            Ok(())
        }
        "gp" => {
            cpu.state.core_regs[26] = value;
            Ok(())
        }
        "blink" => {
            cpu.state.core_regs[31] = value;
            Ok(())
        }
        "lp_count" | "lpcount" => {
            cpu.state.core_regs[60] = value;
            Ok(())
        }
        _ => {
            if let Some(rest) = lower.strip_prefix('r') {
                if let Ok(idx) = rest.parse::<usize>() {
                    if idx < 64 {
                        cpu.state.core_regs[idx] = value;
                        return Ok(());
                    }
                }
            }
            Err(format!("unknown register: {}", name))
        }
    }
}

fn write_sram(cpu: &mut Cpu, addr: u32, bytes: &[u8]) -> Result<(), String> {
    for (i, b) in bytes.iter().enumerate() {
        cpu.mem
            .write_byte(addr + i as u32, *b)
            .map_err(|e| format!("write_byte failed at 0x{:08X}: {:?}", addr + i as u32, e))?;
    }
    Ok(())
}

fn send_dcache(cpu: &Cpu, response: OneshotSender<DcacheSnapshot>) {
    let lines = cpu.mem.dcache_snapshot();
    let ctrl_raw = cpu.state.aux_dc_ctrl;
    let _ = response.send(DcacheSnapshot {
        lines,
        ctrl_raw,
        timestamp: Instant::now(),
    });
}
