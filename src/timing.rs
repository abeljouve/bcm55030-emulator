//! Cycle-approximate cache + bus-contention timing model.
//!
//! The rest of the emulator is a **functional ISS**: it models cache
//! *contents* exactly but has no notion of cycles, bus arbitration, or
//! fetch/load contention. That is correct for almost everything, but it
//! cannot reproduce one microarchitectural failure class this ARC 700 SoC
//! exhibits: a **fetch-vs-load bus-arbitration starvation** on the blocking,
//! single-outstanding path between the CPU and the unified SRAM.
//!
//! This module is that missing piece, kept deliberately self-contained and
//! **optional**. Phase A (this file's [`MiniCore`] + the `timing-sweep` bin)
//! is a standalone cycle-driven machine that *demonstrates* the mechanism and
//! its layout dependence — it shows the hang is plausible and layout-gated
//! under the assumed microarchitecture below; it does not prove the silicon
//! arbitrates this way. Phase B reuses [`ContentionBus`] + [`TimingConfig`] as
//! an opt-in shadow over the real ISS (behind the `timing` cargo feature).
//!
//! # OBSERVED vs INFERRED
//!
//! OBSERVED silicon:
//!
//! - **Cache geometry** (DATASHEET §5.2/§5.3/§5.4: 4 KB 1-way direct-mapped
//!   I-cache, set = addr[11:5], 32 B line; MMIO bypass).
//! - **The uncached SRAM load latency, 10 clocks exactly** (§5.5.1: a 64-load
//!   dependent chain spans 632 ticks = `1 + 63 × 10 + 1`).
//! - **The MMIO read latency, ≈ 40 clocks** (§5.5.6, differential measurement;
//!   the absolute is accurate to a few clocks, the differential is solid).
//!
//! Still INFERRED — and this is where the model's weight sits:
//!
//! - Everything about the *bus*: that it is a single-outstanding, blocking,
//!   fixed-priority (fetch > load) interconnect toward SRAM. A working
//!   hypothesis about the interconnect, NOT documented anywhere.
//! - The line-fill latency `F` and the "no loop buffer" front-end.
//! - ⚠ **The fetch run-ahead depth** — the *only* assumed number that is
//!   load-bearing. `F` and `M` are occurrence-invariant (the sweep's
//!   sensitivity table: the hang count over 128 sets is unchanged for
//!   `F ∈ {4,8,16}` and `M ∈ {16,32,64}`), so getting them measured changed
//!   no verdict. Run-ahead is a *switch*: `1 → 0 hangs`, `3 → 2`, `6 → 4`.
//!   Until it is measured, this module predicts *that* the mechanism can
//!   happen, never *which* layouts hang.
//!
//! So: a demonstrator of a plausible mechanism, not a silicon-faithful
//! predictor. It shows a mechanism can occur; it does not identify the
//! mechanism behind any particular failure. Do not classify a bug with it.
//!
//! # The mechanism (generic ARC 700 microarchitecture)
//!
//! Three conditions must hold simultaneously; only the third is layout-
//! dependent, which is why the resulting hang is layout-dependent:
//!
//! 1. **A blocking, single-outstanding-transaction bus toward SRAM with a
//!    fixed priority (fetch > load).** One transaction slot; at each
//!    arbitration a pending instruction fetch preempts a pending-but-
//!    ungranted data load. This is structural — there is no programmable
//!    arbiter.
//! 2. **An uncached load outstanding.** Intrinsic for MMIO polls (MMIO never
//!    enters the D-cache), and for D-cache-*disabled* SRAM byte sweeps (every
//!    load is an uncached bus transaction). The D-cache, when enabled,
//!    *collapses* an SRAM load window from every-byte to an occasional
//!    line-fill.
//!
//!    ⚠ That collapse is **not a mitigation available on a shipping device**.
//!    Per §5.3.1 the D-cache is *disabled* in the shipping configuration — the
//!    boot loader turns it off at boot step 3 and nothing re-enables it — so
//!    every data load is an uncached bus transaction and both surfaces (SRAM
//!    sweep and MMIO poll) are live at all times. An earlier revision of this
//!    comment presented "enable the D-cache" as fixing the SRAM-sweep surface;
//!    it would, on a machine that ran with the cache on, and none does.
//! 3. **A sustained instruction-fetch miss stream while (2) holds.** A run-
//!    ahead fetch engine, following predicted control flow, keeps requesting
//!    the next line it needs. On a **direct-mapped** instruction cache, a hot
//!    loop line whose set index collides with a *cold* line the loop also
//!    touches each iteration causes the two to evict each other — a perpetual
//!    miss stream. Fixed priority then starves the load indefinitely.
//!
//! A resident poll loop by itself does **not** deadlock: once its lines are
//! cached the fetch engine quiesces and the load is granted. The hang needs
//! the *coincidence* of (2) with a *self-sustaining* miss stream from (3),
//! and whether that coincidence lands depends on where the loop's lines fall
//! relative to the colliding cold line — i.e. on layout. Interrupts are **not**
//! the trigger on this SoC (the interrupt unit fetches its vector from the NCO
//! table on a separate aperture, not from the low-SRAM I-cache), so the model
//! reproduces the hang with no interrupts at all.
//!
//! # Fidelity caveat
//!
//! Latencies (`F`, `M`, …) are approximate. The goal is to reproduce the
//! *invariant* — a fixed-priority fetch stream starving an outstanding load —
//! and its layout dependence, not cycle-perfect timing. The exact numbers are
//! chosen so the hang verdict flips with set index; sensitivity to them is
//! documented in the sweep bin and the unit tests.

use crate::cache::{ICache, IC_LINE_SIZE, IC_NUM_SETS};
#[cfg(feature = "timing")]
use crate::decoder::instruction::Instruction;
#[cfg(feature = "timing")]
use crate::memory::Memory;

/// Line-fill latency `F` (cycles the fetch master holds the bus per miss).
/// -- INFERRED: no isolated instruction-fetch line-fill measurement exists.
/// Occurrence-invariant over `{4, 8, 16}` (sweep sensitivity table), so the
/// choice does not decide any verdict.
pub const DEFAULT_FETCH_LATENCY: u32 = 8;
/// Uncached MMIO load latency `M` (cycles). `M >> F`: an MMIO poll keeps its
/// load window open far longer than a line fill, which is what lets a fetch
/// miss stream overlap and starve it.
/// -- OBSERVED, DATASHEET §5.5.6: an MMIO word read costs ≈ 40 clocks, about
/// four times an SRAM word load, measured differentially against the same
/// trampoline on an SRAM address (2783 − 854 ticks over 64 accesses). The
/// absolute is accurate to a few clocks; the differential is solid. Was 32
/// (assumed) before the measurement existed — occurrence-invariant over
/// `{16, 32, 64}`, so no verdict moved.
pub const DEFAULT_MMIO_LOAD_LATENCY: u32 = 40;
/// Uncached SRAM load latency (cycles). Every data load is one of these on a
/// shipping device: the D-cache is disabled there (§5.3.1).
/// -- OBSERVED, DATASHEET §5.5.1: the load-use recurrence is **10 clocks
/// exactly** — a 64-load dependent chain spans 632 ticks, decomposing as
/// `1 (instrument) + 63 × 10 + 1`. (The ≈ 9.88 figure once published divided
/// by 64 instead of the 63 dependency edges a 64-load chain actually has.)
pub const DEFAULT_SRAM_LOAD_LATENCY: u32 = 10;
/// INFERRED, and load-bearing: how many distinct lines the front-end reaches
/// along predicted control flow before it quiesces (if all are resident). This
/// is NOT a latency knob — unlike `F`/`M` (which are occurrence-invariant, see
/// the sweep's sensitivity table) it is a **switch that gates whether the hang
/// exists at all**: the deadlock requires `fetch_runahead_lines >=` the hot
/// loop's line footprint so the engine observes the far-side set collision
/// (`runahead=1 -> 0 hangs`, `>= footprint -> hangs`). The datasheet does not
/// specify a fetch-buffer / prefetch depth; this value is assumed.
pub const DEFAULT_FETCH_RUNAHEAD_LINES: u32 = 8;
/// Consecutive cycles a load may be pending-but-ungranted before the model
/// declares a bus deadlock. Sized well above any legitimate fill burst.
pub const DEFAULT_DEADLOCK_WINDOW: u32 = 4096;

/// Tunable timing parameters. Shared by the standalone [`MiniCore`] and the
/// Phase-B ISS overlay so both reproduce the same invariant.
#[derive(Clone, Copy, Debug)]
pub struct TimingConfig {
    /// `F`: line-fill latency (bus cycles per instruction-fetch miss).
    pub fetch_latency: u32,
    /// `M`: uncached MMIO load latency (bus cycles).
    pub mmio_load_latency: u32,
    /// Uncached SRAM load latency (bus cycles).
    pub sram_load_latency: u32,
    /// Run-ahead depth of the fetch engine, in lines.
    pub fetch_runahead_lines: u32,
    /// `W`: deadlock detection window (starved-load cycles).
    pub deadlock_window: u32,
    /// INFERRED forward-progress property. When `true`, the run-ahead fetch
    /// engine keeps requesting the bus for its next needed line **even while
    /// the backend is stalled on a load**, and it re-derives that need from
    /// I-cache residency each cycle — i.e. the front-end has **no loop buffer**:
    /// a loop resident in a decode/loop buffer would feed the stalled backend
    /// without re-fetching, breaking the ping-pong. This "no loop buffer +
    /// re-read every iteration" assumption is what *sustains* the starvation
    /// (the datasheet describes no such buffer for this reduced-config core, so
    /// it is assumed). Consequence: the model can over-declare hangs for a
    /// small self-evicting loop that a buffered machine would survive. When
    /// `false`, the fetch engine quiesces once the backend stalls, so the same
    /// layout is merely *slow* rather than hung — the toggle that shows this
    /// property is what turns slowness into a deadlock.
    pub fetch_runs_ahead_under_stall: bool,
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            fetch_latency: DEFAULT_FETCH_LATENCY,
            mmio_load_latency: DEFAULT_MMIO_LOAD_LATENCY,
            sram_load_latency: DEFAULT_SRAM_LOAD_LATENCY,
            fetch_runahead_lines: DEFAULT_FETCH_RUNAHEAD_LINES,
            deadlock_window: DEFAULT_DEADLOCK_WINDOW,
            fetch_runs_ahead_under_stall: true,
        }
    }
}

/// Which master currently owns the single bus transaction slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusMaster {
    /// Instruction-cache line fill.
    Fetch,
    /// Data load (uncached: MMIO or D-cache-off SRAM).
    Load,
}

#[derive(Clone, Copy, Debug)]
struct BusTxn {
    master: BusMaster,
    /// Cycles remaining before this transaction completes.
    remaining: u32,
    /// Line base (Fetch) or effective address (Load) the transaction serves.
    addr: u32,
}

/// A blocking, single-outstanding-transaction bus with a fixed-priority
/// (fetch > load) arbiter.
///
/// INFERRED — a working hypothesis, NOT datasheet-backed. The BCM55030
/// datasheet documents no bus arbiter, no single-outstanding constraint and no
/// fetch-over-load priority; this structure is the simplest interconnect that
/// reproduces the observed layout-dependent hang. It is the load-bearing
/// assumption of the whole model — treat it as a hypothesis under test, not
/// established silicon.
///
/// Exactly one transaction is in flight at a time;
/// a new one can only start when the slot is free, and at that arbitration
/// point a pending fetch always wins over a pending load.
///
/// This is the shared heart of the timing model — Phase A drives it from a
/// synthetic program, Phase B drives it from the real instruction stream.
pub struct ContentionBus {
    cfg: TimingConfig,
    slot: Option<BusTxn>,
    /// Set on the cycle a fetch is granted while a load was simultaneously
    /// requesting — i.e. the fetch preempted the load at arbitration. Purely
    /// diagnostic (surfaced in the deadlock report).
    pub last_preempt_addr: Option<u32>,
    /// Total granted fetch transactions (diagnostic).
    pub fetch_grants: u64,
    /// Total granted load transactions (diagnostic).
    pub load_grants: u64,
}

/// What completed on a given [`ContentionBus::tick`], if anything.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusCompletion {
    /// Nothing completed this cycle.
    None,
    /// A line fill for the given line base finished — install it in the I-cache.
    FetchDone(u32),
    /// A load for the given effective address finished — retire the consumer.
    LoadDone(u32),
}

impl ContentionBus {
    pub fn new(cfg: TimingConfig) -> Self {
        Self {
            cfg,
            slot: None,
            last_preempt_addr: None,
            fetch_grants: 0,
            load_grants: 0,
        }
    }

    /// Is the bus slot currently occupied?
    #[inline]
    pub fn is_busy(&self) -> bool {
        self.slot.is_some()
    }

    /// Is a load transaction currently occupying the slot?
    #[inline]
    pub fn load_in_flight(&self) -> bool {
        matches!(self.slot, Some(t) if t.master == BusMaster::Load)
    }

    /// Advance the in-flight transaction by one cycle. Returns what completed.
    pub fn tick(&mut self) -> BusCompletion {
        if let Some(txn) = self.slot.as_mut() {
            txn.remaining = txn.remaining.saturating_sub(1);
            if txn.remaining == 0 {
                let done = *txn;
                self.slot = None;
                return match done.master {
                    BusMaster::Fetch => BusCompletion::FetchDone(done.addr),
                    BusMaster::Load => BusCompletion::LoadDone(done.addr),
                };
            }
        }
        BusCompletion::None
    }

    /// Arbitrate for the free slot given the current pending requests. Fixed
    /// priority: a pending fetch always beats a pending load. Does nothing if
    /// the slot is already occupied. Returns the master that was granted, if
    /// any.
    ///
    /// * `fetch_req` — `Some(line_base)` if the fetch engine needs a fill.
    /// * `load_req`  — `Some((ea, mmio))` if the backend has an outstanding
    ///   uncached load; `mmio` selects the load latency.
    pub fn arbitrate(
        &mut self,
        fetch_req: Option<u32>,
        load_req: Option<(u32, bool)>,
    ) -> Option<BusMaster> {
        if self.slot.is_some() {
            return None;
        }
        self.last_preempt_addr = None;
        if let Some(line) = fetch_req {
            // Record a preempt: fetch is taking the slot while a load also
            // wanted it. This is the coincidence that starves the load.
            if load_req.is_some() {
                self.last_preempt_addr = Some(line);
            }
            self.slot = Some(BusTxn {
                master: BusMaster::Fetch,
                remaining: self.cfg.fetch_latency.max(1),
                addr: line,
            });
            self.fetch_grants += 1;
            return Some(BusMaster::Fetch);
        }
        if let Some((ea, mmio)) = load_req {
            let lat = if mmio {
                self.cfg.mmio_load_latency
            } else {
                self.cfg.sram_load_latency
            };
            self.slot = Some(BusTxn {
                master: BusMaster::Load,
                remaining: lat.max(1),
                addr: ea,
            });
            self.load_grants += 1;
            return Some(BusMaster::Load);
        }
        None
    }
}

// ===========================================================================
// Phase A: standalone minimal cycle-driven machine.
//
// A tiny structural simulator — NOT a faithful ARC 700 — whose only purpose
// is to demonstrate the deadlock and its layout dependence. It has a direct-
// mapped instruction cache (the real geometry, via `ICache`), the contention
// bus above, a run-ahead fetch engine following predicted control flow, and a
// backend that stalls on an uncached load. It runs a synthetic poll loop that,
// each iteration, also touches a fixed *cold* line — modelling a poll whose
// body reaches into cold code (the real firmware wedge sits on such a cold
// path, not on a bare poll).
// ===========================================================================

/// One instruction in the synthetic program.
#[derive(Clone, Copy, Debug)]
enum Kind {
    /// Plain ALU op — retires in one cycle, no bus.
    Alu,
    /// Uncached load. `mmio` selects the load latency / D-cache behaviour.
    Load { ea: u32, mmio: bool },
    /// Unconditional branch, predicted (and always) taken to `target`.
    Branch { target: u32 },
    /// One completed loop iteration boundary marker + loop-exit test. When
    /// `iters_done` reaches the target the program ends (PASS).
    LoopBack { target: u32 },
}

#[derive(Clone, Copy, Debug)]
struct Insn {
    size: u32,
    kind: Kind,
}

/// Number of distinct cache lines the cold path walks per iteration. Spacing
/// them one line apart means a swept loop base collides (in set index) with one
/// of `cold_lines` cold lines at several positions — a richer collision map,
/// like the real firmware whose hot loop shares sets with many cold lines.
pub const DEFAULT_COLD_LINES: u32 = 4;

/// The synthetic program: a poll loop that walks a multi-line cold path each
/// iteration.
///
/// ```text
///   base+0  (L0):  Load  STATUS                    (uncached)
///   base+4  (L1):  Alu
///   base+8  (L2):  Branch -> COLD_0                 (predicted taken)
///   COLD_0       :  Alu ; Branch -> COLD_1          (COLD_i = cold + i*32)
///   COLD_1       :  Alu ; Branch -> COLD_2
///   ...
///   COLD_{n-1}   :  Alu ; LoopBack -> L0            (counts one iteration)
/// ```
///
/// `base` is swept across set positions; the cold path is fixed. When
/// `line(base)` lands in the same direct-mapped I-cache set as any cold line,
/// the two evict each other and, while the STATUS load is outstanding, the
/// run-ahead fetch engine keeps missing on the evicted line and (fixed
/// priority) starves the load → deadlock. When there is no set collision, all
/// lines stay resident → the fetch engine quiesces → the load is granted → PASS.
#[derive(Clone, Copy, Debug)]
pub struct PollLoopProgram {
    pub base: u32,
    pub cold: u32,
    pub cold_lines: u32,
    pub status: u32,
    pub status_mmio: bool,
}

impl PollLoopProgram {
    /// Convenience: a program with the default cold-path length.
    pub fn new(base: u32, cold: u32, status: u32, status_mmio: bool) -> Self {
        Self { base, cold, cold_lines: DEFAULT_COLD_LINES, status, status_mmio }
    }

    fn decode(&self, pc: u32) -> Insn {
        if pc == self.base {
            return Insn { size: 4, kind: Kind::Load { ea: self.status, mmio: self.status_mmio } };
        } else if pc == self.base + 4 {
            return Insn { size: 4, kind: Kind::Alu };
        } else if pc == self.base + 8 {
            return Insn { size: 4, kind: Kind::Branch { target: self.cold } };
        }
        // Cold path: COLD_i at cold + i*32, each = Alu then a branch to the next
        // cold line (or LoopBack on the last).
        let line = IC_LINE_SIZE as u32;
        for i in 0..self.cold_lines {
            let ci = self.cold + i * line;
            if pc == ci {
                return Insn { size: 4, kind: Kind::Alu };
            }
            if pc == ci + 4 {
                if i + 1 < self.cold_lines {
                    return Insn { size: 4, kind: Kind::Branch { target: self.cold + (i + 1) * line } };
                } else {
                    return Insn { size: 4, kind: Kind::LoopBack { target: self.base } };
                }
            }
        }
        // Unreached under predicted-taken flow, but decode must be total.
        Insn { size: 4, kind: Kind::Alu }
    }

    /// Follow predicted control flow one instruction: returns the next PC.
    fn predicted_next(&self, pc: u32) -> u32 {
        let insn = self.decode(pc);
        match insn.kind {
            Kind::Branch { target } | Kind::LoopBack { target } => target,
            _ => pc + insn.size,
        }
    }
}

/// Outcome of a [`MiniCore`] run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RunOutcome {
    /// The loop completed the requested number of iterations without starving.
    Pass { cycles: u64, iters: u32 },
    /// The invariant fired: a load was pending-but-ungranted for more than the
    /// deadlock window. Reports the starved load EA, the aliasing fetch line,
    /// and the arbiter state at the hang.
    Deadlock {
        cycles: u64,
        iters: u32,
        starved_load_ea: u32,
        aliasing_fetch_line: Option<u32>,
        starve_cycles: u32,
    },
    /// Neither completion nor deadlock within the hard cycle cap (diagnostic).
    Timeout { cycles: u64, iters: u32 },
}

impl RunOutcome {
    pub fn is_deadlock(&self) -> bool {
        matches!(self, RunOutcome::Deadlock { .. })
    }
    pub fn is_pass(&self) -> bool {
        matches!(self, RunOutcome::Pass { .. })
    }
}

/// Backend execution state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Backend {
    /// Ready to execute the instruction at `exec_pc` once it is fetched.
    Running,
    /// Stalled on an uncached load to `ea` (`mmio` selects latency).
    StalledOnLoad { ea: u32, mmio: bool },
}

/// The standalone minimal machine (Phase A).
pub struct MiniCore {
    prog: PollLoopProgram,
    cfg: TimingConfig,
    ic: ICache,
    bus: ContentionBus,

    /// Instruction the backend is trying to execute/retire.
    exec_pc: u32,
    backend: Backend,

    /// Run-ahead fetch pointer, following predicted control flow. This model
    /// has NO decode/loop buffer: the fetch engine re-derives what it needs
    /// from I-cache residency each cycle, which is what sustains the ping-pong
    /// under a set collision (see `TimingConfig::fetch_runs_ahead_under_stall`).
    fetch_pc: u32,

    cycle: u64,
    iters_done: u32,
    /// Consecutive cycles a load has been pending-but-ungranted.
    starve_cycles: u32,
    /// The most recent fetch line that preempted a pending load.
    last_aliasing_fetch: Option<u32>,

    /// When the D-cache is on, an SRAM poll load is treated as a warm cache hit
    /// (zero bus) — the load window collapses and there is no outstanding load
    /// to starve. MMIO loads ignore this (they always bypass the D-cache).
    dcache_on: bool,
}

impl MiniCore {
    pub fn new(prog: PollLoopProgram, cfg: TimingConfig, dcache_on: bool) -> Self {
        Self {
            prog,
            cfg,
            ic: ICache::new(),
            bus: ContentionBus::new(cfg),
            exec_pc: prog.base,
            backend: Backend::Running,
            fetch_pc: prog.base,
            cycle: 0,
            iters_done: 0,
            starve_cycles: 0,
            last_aliasing_fetch: None,
            dcache_on,
        }
    }

    #[inline]
    fn line_of(addr: u32) -> u32 {
        addr & !((IC_LINE_SIZE as u32) - 1)
    }

    /// Whether this load consults the D-cache (and can therefore hit and cost
    /// no bus). MMIO never does; SRAM does only when the D-cache is on.
    fn load_uses_dcache(&self, mmio: bool) -> bool {
        self.dcache_on && !mmio
    }

    /// Compute the fetch engine's current request, if any. It walks predicted
    /// control flow from `fetch_pc` and returns the first line it needs that is
    /// not resident in the direct-mapped I-cache. It stops (returns `None`,
    /// i.e. quiesces) when it has scanned `fetch_runahead_lines` distinct lines
    /// that are all resident, or when the predicted flow closes a loop with
    /// everything resident. It is gated off entirely by a backend stall unless
    /// the forward-progress property is set.
    ///
    /// The key to the layout dependence: the scan must span the whole hot loop.
    /// When a loop line's set collides with another line the loop touches, the
    /// two evict each other in the direct-mapped cache, so *some* line on the
    /// predicted path is always non-resident — the scan always returns a miss,
    /// the fetch never quiesces, and (fixed priority) the outstanding load is
    /// starved. With no collision the whole loop stays resident → the scan
    /// finds nothing to fetch → quiesce → the load is granted.
    fn fetch_request(&self) -> Option<u32> {
        // Forward-progress property: if the fetch engine is NOT allowed to run
        // ahead under a stall, it quiesces the moment the backend stalls.
        if !self.cfg.fetch_runs_ahead_under_stall
            && matches!(self.backend, Backend::StalledOnLoad { .. })
        {
            return None;
        }

        let mut pc = self.fetch_pc;
        let mut distinct_lines: Vec<u32> = Vec::new();
        let mut visited: Vec<u32> = Vec::new();
        // Hard step cap: a safety bound so the walk always terminates even for
        // a pathological program. Comfortably larger than any real loop.
        for _ in 0..4096 {
            if !self.ic.contains(pc) {
                return Some(Self::line_of(pc));
            }
            let line = Self::line_of(pc);
            if !distinct_lines.contains(&line) {
                distinct_lines.push(line);
                if distinct_lines.len() >= self.cfg.fetch_runahead_lines as usize {
                    // Scanned the full run-ahead window, all resident → quiesce.
                    return None;
                }
            }
            if visited.contains(&pc) {
                // Predicted flow closed a loop with everything resident → quiesce.
                return None;
            }
            visited.push(pc);
            pc = self.prog.predicted_next(pc);
        }
        None
    }

    /// Advance the fetch pointer along predicted flow past any lines that are
    /// now resident, so `fetch_pc` tracks the frontier of what still needs
    /// fetching. Bounded work per cycle.
    fn advance_fetch_pointer(&mut self) {
        for _ in 0..8 {
            if !self.ic.contains(self.fetch_pc) {
                break;
            }
            self.fetch_pc = self.prog.predicted_next(self.fetch_pc);
        }
    }

    /// Run until PASS, deadlock, or the hard cycle cap. `target_iters` loop
    /// iterations constitutes a pass.
    pub fn run(&mut self, target_iters: u32, max_cycles: u64) -> RunOutcome {
        while self.cycle < max_cycles {
            self.cycle += 1;

            // 1. Advance the in-flight bus transaction.
            match self.bus.tick() {
                BusCompletion::FetchDone(line) => {
                    // Install the freshly filled line in the direct-mapped
                    // I-cache (unconditional eviction of the set's occupant).
                    let mut data = [0u8; IC_LINE_SIZE];
                    // Contents are irrelevant to the timing model; use the
                    // line base as a marker so the tag is set correctly.
                    data[0] = (line >> 24) as u8;
                    self.ic.fill_line(line, &data);
                }
                BusCompletion::LoadDone(_ea) => {
                    // The stalled backend's load retired. `exec_pc` was already
                    // advanced past the load at issue time (the load is in
                    // flight; the consumer stalls, then resumes at the next PC).
                    self.backend = Backend::Running;
                }
                BusCompletion::None => {}
            }

            // 2. Compute this cycle's requests.
            let fetch_req = self.fetch_request();
            let load_req = match self.backend {
                Backend::StalledOnLoad { ea, mmio } => Some((ea, mmio)),
                Backend::Running => None,
            };

            // 3. Arbitrate for a free slot (fixed priority fetch > load).
            if let Some(master) = self.bus.arbitrate(fetch_req, load_req) {
                if master == BusMaster::Fetch {
                    if let Some(line) = self.bus.last_preempt_addr {
                        self.last_aliasing_fetch = Some(line);
                    }
                }
            }

            // 4. Deadlock invariant: a load pending-but-ungranted for too long.
            if let Backend::StalledOnLoad { ea, .. } = self.backend {
                if !self.bus.load_in_flight() {
                    self.starve_cycles += 1;
                    if self.starve_cycles > self.cfg.deadlock_window {
                        return RunOutcome::Deadlock {
                            cycles: self.cycle,
                            iters: self.iters_done,
                            starved_load_ea: ea,
                            aliasing_fetch_line: self.last_aliasing_fetch,
                            starve_cycles: self.starve_cycles,
                        };
                    }
                } else {
                    self.starve_cycles = 0;
                }
            } else {
                self.starve_cycles = 0;
            }

            // 5. Backend: retire one instruction if it is fetched and we are
            //    not stalled on a load.
            if matches!(self.backend, Backend::Running) {
                self.advance_fetch_pointer();
                if self.ic.contains(self.exec_pc) {
                    let insn = self.prog.decode(self.exec_pc);
                    match insn.kind {
                        Kind::Alu => {
                            self.exec_pc += insn.size;
                        }
                        Kind::Branch { target } => {
                            self.exec_pc = target;
                            self.redirect_fetch(target);
                        }
                        Kind::LoopBack { target } => {
                            self.iters_done += 1;
                            if self.iters_done >= target_iters {
                                return RunOutcome::Pass {
                                    cycles: self.cycle,
                                    iters: self.iters_done,
                                };
                            }
                            self.exec_pc = target;
                            self.redirect_fetch(target);
                        }
                        Kind::Load { ea, mmio } => {
                            // With the D-cache ON, an SRAM load is a cache hit
                            // (the poll target is warm) — zero bus, no stall, so
                            // there is no outstanding load to starve. The one
                            // cold fill is abstracted away. MMIO always bypasses
                            // the D-cache, so its load always takes the bus.
                            if self.load_uses_dcache(mmio) {
                                self.exec_pc += insn.size;
                            } else {
                                // Issue the uncached load: advance the PC past it
                                // (the load is in flight) and stall the consumer.
                                self.exec_pc += insn.size;
                                self.backend = Backend::StalledOnLoad { ea, mmio };
                            }
                        }
                    }
                }
            }
        }

        RunOutcome::Timeout {
            cycles: self.cycle,
            iters: self.iters_done,
        }
    }

    /// On a resolved (taken) branch, redirect the fetch frontier to the target
    /// if it has run past it speculatively along a different path. Because the
    /// program's predicted flow equals its taken flow (all branches predicted
    /// taken), the fetch pointer normally already tracks the target; this keeps
    /// it consistent when the backend jumps.
    fn redirect_fetch(&mut self, target: u32) {
        // Only pull the fetch pointer back to the target if it is not already
        // within the resident predicted window from the target — cheap and
        // conservative.
        if !self.ic.contains(target) {
            self.fetch_pc = target;
        }
    }

    /// Set index (direct-mapped) of an address's I-cache line.
    pub fn set_index(addr: u32) -> usize {
        ((addr >> 5) & ((IC_NUM_SETS as u32) - 1)) as usize
    }
}

// ===========================================================================
// Phase B: contention overlay on the real ISS.
//
// A `TimingShadow` runs alongside the functional interpreter (behind the
// `timing` cargo feature). It does not change what executes — the ISS still
// runs each instruction atomically and correctly — it only *accounts* for
// cache + bus contention and detects the fetch-vs-load starvation deadlock,
// reusing the same `ContentionBus` the Phase-A model validated.
//
// It maintains its own direct-mapped I-cache residency model and, on each
// uncached load, opens a contention window: the load holds the single bus
// slot for its latency while a run-ahead fetch engine decodes forward along
// predicted control flow. If the executing code is a tight poll loop whose
// lines collide (in set index) with a cold line it touches, the fetch engine
// perpetually misses, fixed priority starves the load, and the invariant
// fires — reproducing a hardware hang in the functional model. A benign
// (non-looping) MMIO
// access fills a bounded set of forward lines once, quiesces, and the load
// is granted, so it does not false-positive.
// ===========================================================================

/// A reported bus deadlock from the Phase-B overlay.
#[cfg(feature = "timing")]
#[derive(Clone, Copy, Debug)]
pub struct DeadlockReport {
    /// PC of the uncached load whose window deadlocked.
    pub pc: u32,
    /// Effective address of the starved load.
    pub load_ea: u32,
    /// Whether the starved load was MMIO.
    pub load_mmio: bool,
    /// The fetch line that was repeatedly preempting the load, if known.
    pub aliasing_fetch_line: Option<u32>,
    /// Model cycle at which the deadlock was declared.
    pub cycle: u64,
    /// Consecutive starved-load cycles at declaration.
    pub starve_cycles: u32,
    /// I-cache set index the collision maps to (aliasing line's set), if known.
    pub set_index: Option<usize>,
}

/// Optional cycle-accurate contention shadow over the ISS. Only meaningful
/// behaviour when the `timing` feature is on; with it off this type is not
/// compiled and the ISS is byte-for-byte unchanged.
#[cfg(feature = "timing")]
pub struct TimingShadow {
    cfg: TimingConfig,
    bus: ContentionBus,
    /// Shadow direct-mapped I-cache residency (real geometry). Estimates the
    /// fetch miss stream; independent of the functional ISS I-cache.
    ic: ICache,
    /// Total model cycles accounted.
    pub cycle: u64,
    /// Drives DEBUG.LD (aux 0x05 bit 31): true while a load is outstanding in
    /// the model. This is a MODEL-produced diagnostic populated only under this
    /// timing shadow — the functional ISS never sets it, and the datasheet aux
    /// table documents no bit-31 for DEBUG. It exists so a future JTAG/OCD
    /// `DEBUG.LD` sample at a real silicon hang can be cross-checked against the
    /// model's "load starved" state; it is NOT an independent silicon readback.
    /// On a deadlock it stays true, so an aux-0x05 read returns bit31=1.
    pub load_pending: bool,
    /// Set when the invariant fires. Once set, the CPU halts with the report.
    pub deadlock: Option<DeadlockReport>,
    /// Count of uncached-load contention windows opened (diagnostic).
    pub windows: u64,
}

#[cfg(feature = "timing")]
impl TimingShadow {
    pub fn new(cfg: TimingConfig) -> Self {
        Self {
            cfg,
            bus: ContentionBus::new(cfg),
            ic: ICache::new(),
            cycle: 0,
            load_pending: false,
            deadlock: None,
            windows: 0,
        }
    }

    #[inline]
    fn line_of(addr: u32) -> u32 {
        addr & !((IC_LINE_SIZE as u32) - 1)
    }

    /// Mark a line resident in the shadow I-cache (direct-mapped: unconditional
    /// eviction of the set's current occupant).
    fn fill(&mut self, line: u32) {
        let mut data = [0u8; IC_LINE_SIZE];
        data[0] = (line >> 24) as u8;
        self.ic.fill_line(line, &data);
    }

    /// Predicted next PC for the fetch-ahead engine: unconditional and backward
    /// (loop) branches are predicted taken; forward conditional branches fall
    /// through; register-indirect jumps are unpredictable (`None` stops the
    /// scan).
    ///
    /// It decodes from **raw SRAM bytes** (`decode_bytes`), NOT `decode(pc,
    /// mem)`: the latter routes through `Memory::fetch_word` and would fill the
    /// FUNCTIONAL I-cache ahead of execution. That is fine for straight code
    /// (same bytes) but would corrupt self-modifying code (the overlay might
    /// pre-fill a line with pre-store bytes) — breaking the observational-only
    /// guarantee. Reading raw SRAM never touches the functional caches.
    fn predicted_next(pc: u32, mem: &Memory) -> Option<u32> {
        let sram = mem.sram_size() as u32;
        if pc >= sram {
            return None; // not SRAM-resident code we can predict
        }
        let len = core::cmp::min(8, sram - pc); // 32-bit insn + optional LIMM
        let bytes = mem.sram_slice(pc, len)?;
        let d = crate::decoder::decode_bytes(pc, bytes, pc).ok()?;
        let size = d.total_size();
        let pcl = pc & 0xFFFF_FFFC;
        match d.inst {
            Instruction::Branch { offset, cc, link, .. } => {
                if link {
                    // Calls: predicted taken to the target, but the return is
                    // unpredictable — for loop detection treat as fall-through
                    // so the scan does not chase into an unrelated callee.
                    Some(pc.wrapping_add(size))
                } else if cc.is_none() || offset < 0 {
                    Some(pcl.wrapping_add(offset as u32))
                } else {
                    Some(pc.wrapping_add(size))
                }
            }
            Instruction::BranchCompare { offset, .. } => {
                if offset < 0 {
                    Some(pcl.wrapping_add(offset as u32))
                } else {
                    Some(pc.wrapping_add(size))
                }
            }
            Instruction::Jump { .. } => None, // register-indirect: unpredictable
            _ => Some(pc.wrapping_add(size)),
        }
    }

    /// The fetch-ahead engine's request: walk predicted control flow from
    /// `start`, returning the first line not resident in the shadow I-cache.
    /// Quiesces (`None`) when it has scanned `fetch_runahead_lines` distinct
    /// resident lines or closed a loop with everything resident. Bounded work.
    fn fetch_request(&self, start: u32, mem: &Memory) -> Option<u32> {
        let mut pc = start;
        let mut distinct: Vec<u32> = Vec::new();
        let mut visited: Vec<u32> = Vec::new();
        for _ in 0..256 {
            if !self.ic.contains(pc) {
                return Some(Self::line_of(pc));
            }
            let line = Self::line_of(pc);
            if !distinct.contains(&line) {
                distinct.push(line);
                if distinct.len() >= self.cfg.fetch_runahead_lines as usize {
                    return None;
                }
            }
            if visited.contains(&pc) {
                return None;
            }
            visited.push(pc);
            match Self::predicted_next(pc, mem) {
                Some(next) => pc = next,
                None => return None,
            }
        }
        None
    }

    /// Account for one executed instruction. `pc` is its address; if it is an
    /// uncached load, `load` is `Some((ea, mmio))`. `next_pc` is the predicted
    /// next fetch address (where the fetch-ahead engine prefetches from during
    /// a load window). `is_sync` marks a SYNC barrier. Returns `true` if a
    /// deadlock was declared this step (the caller should halt).
    pub fn on_instruction(
        &mut self,
        pc: u32,
        load: Option<(u32, bool)>,
        next_pc: u32,
        is_sync: bool,
        mem: &Memory,
    ) -> bool {
        if self.deadlock.is_some() {
            return true;
        }

        // Fetch of the current instruction: keep the shadow I-cache warm so a
        // loop's residency is tracked. A miss costs one fill (accounted).
        if !self.ic.contains(pc) {
            self.cycle += self.cfg.fetch_latency.max(1) as u64;
            self.fill(Self::line_of(pc));
        } else {
            self.cycle += 1;
        }

        match load {
            Some((ea, mmio)) => {
                self.open_load_window(pc, ea, mmio, next_pc, mem);
            }
            None if is_sync => {
                // SYNC waits for the bus to drain (and can therefore also stall
                // if a fetch stream is monopolising it). Model it as a bounded
                // barrier that advances the bus to idle.
                self.drain_bus(next_pc, mem);
            }
            None => {}
        }

        self.deadlock.is_some()
    }

    /// Run the contention window for an uncached load: the load holds the bus
    /// for its latency while the fetch-ahead engine competes (fixed priority
    /// fetch > load). Detects starvation.
    fn open_load_window(&mut self, pc: u32, ea: u32, mmio: bool, next_pc: u32, mem: &Memory) {
        self.windows += 1;
        self.load_pending = true;
        let mut starve: u32 = 0;
        let mut last_alias: Option<u32> = None;
        // Hard cap so a pass terminates quickly; a deadlock is declared well
        // inside this via the starve window.
        let cap = self.cfg.deadlock_window as u64 + self.cfg.mmio_load_latency as u64 * 4 + 64;
        for _ in 0..cap {
            self.cycle += 1;
            match self.bus.tick() {
                BusCompletion::FetchDone(line) => self.fill(line),
                BusCompletion::LoadDone(_) => {
                    self.load_pending = false;
                    return; // load retired → window closes, no deadlock
                }
                BusCompletion::None => {}
            }
            let fetch_req = self.fetch_request(next_pc, mem);
            let load_req = if self.load_pending { Some((ea, mmio)) } else { None };
            if self.bus.arbitrate(fetch_req, load_req) == Some(BusMaster::Fetch) {
                if let Some(l) = self.bus.last_preempt_addr {
                    last_alias = Some(l);
                }
            }
            if self.load_pending && !self.bus.load_in_flight() {
                starve += 1;
                if starve > self.cfg.deadlock_window {
                    self.deadlock = Some(DeadlockReport {
                        pc,
                        load_ea: ea,
                        load_mmio: mmio,
                        aliasing_fetch_line: last_alias,
                        cycle: self.cycle,
                        starve_cycles: starve,
                        set_index: last_alias.map(|l| MiniCore::set_index(l)),
                    });
                    // Leave load_pending = true so DEBUG.LD reads 1 at the hang.
                    return;
                }
            } else {
                starve = 0;
            }
        }
        // Cap reached without completion or a declared deadlock: treat as
        // completed (a very slow but not-starved load) to avoid false hangs.
        self.load_pending = false;
    }

    /// Drain the bus to idle for a SYNC barrier, letting a monopolising fetch
    /// stream (collision) surface as a stall if present.
    fn drain_bus(&mut self, next_pc: u32, mem: &Memory) {
        let cap = self.cfg.deadlock_window as u64 + 64;
        let mut idle_guard: u32 = 0;
        for _ in 0..cap {
            self.cycle += 1;
            match self.bus.tick() {
                BusCompletion::FetchDone(line) => self.fill(line),
                _ => {}
            }
            let fetch_req = self.fetch_request(next_pc, mem);
            self.bus.arbitrate(fetch_req, None);
            if !self.bus.is_busy() && fetch_req.is_none() {
                idle_guard += 1;
                if idle_guard > 2 {
                    return; // bus quiesced
                }
            } else {
                idle_guard = 0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_base(base: u32, cold: u32, status_mmio: bool, dcache_on: bool) -> RunOutcome {
        // status 0x0100_1040 is an MMIO-ish address (Phase A uses it for latency only).
        let prog = PollLoopProgram::new(base, cold, 0x0100_1040, status_mmio);
        let cfg = TimingConfig::default();
        let mut core = MiniCore::new(prog, cfg, dcache_on);
        core.run(64, 2_000_000)
    }

    #[test]
    fn bus_fixed_priority_fetch_beats_load() {
        let mut bus = ContentionBus::new(TimingConfig::default());
        // Both request the free slot: fetch must win.
        let granted = bus.arbitrate(Some(0x1000), Some((0x0100_1040, true)));
        assert_eq!(granted, Some(BusMaster::Fetch));
        assert!(bus.is_busy());
        assert_eq!(bus.last_preempt_addr, Some(0x1000));
    }

    #[test]
    fn bus_grants_load_when_no_fetch() {
        let mut bus = ContentionBus::new(TimingConfig::default());
        let granted = bus.arbitrate(None, Some((0x0100_1040, true)));
        assert_eq!(granted, Some(BusMaster::Load));
        assert!(bus.load_in_flight());
    }

    #[test]
    fn one_line_loop_never_hangs() {
        // A loop whose body and cold line do NOT collide in set index passes,
        // regardless of MMIO — no self-evicting miss stream.
        // Choose base and cold far apart in set index.
        let base = 0x0002_0000; // set index (0x20000>>5)&127 = 0
        let cold = 0x0002_0400; // set index (0x20400>>5)&127 = 32 → different
        assert_ne!(MiniCore::set_index(base), MiniCore::set_index(cold));
        let outcome = run_base(base, cold, true, false);
        assert!(outcome.is_pass(), "non-colliding layout must pass: {outcome:?}");
    }

    #[test]
    fn colliding_layout_deadlocks_on_mmio() {
        // base and cold chosen to land in the SAME direct-mapped set: the loop
        // line and the cold line evict each other every iteration → the MMIO
        // load is starved → deadlock.
        let base = 0x0002_0000;
        let cold = 0x0002_0000 + (IC_NUM_SETS as u32) * (IC_LINE_SIZE as u32); // +4KB → same set
        assert_eq!(MiniCore::set_index(base), MiniCore::set_index(cold));
        let outcome = run_base(base, cold, true, false);
        assert!(outcome.is_deadlock(), "colliding MMIO layout must deadlock: {outcome:?}");
    }

    #[test]
    fn dcache_on_fixes_sram_load_but_not_mmio() {
        // Same colliding layout. With an SRAM load and the D-cache ON, the load
        // window collapses (resident after first touch) → PASS. With MMIO (which
        // bypasses the D-cache) the same layout still deadlocks.
        let base = 0x0002_0000;
        let cold = 0x0002_0000 + (IC_NUM_SETS as u32) * (IC_LINE_SIZE as u32);

        let sram_dcache_on = run_base(base, cold, false, true);
        assert!(sram_dcache_on.is_pass(), "D$-on SRAM must pass: {sram_dcache_on:?}");

        let sram_dcache_off = run_base(base, cold, false, false);
        assert!(sram_dcache_off.is_deadlock(), "D$-off SRAM must deadlock: {sram_dcache_off:?}");

        let mmio_dcache_on = run_base(base, cold, true, true);
        assert!(mmio_dcache_on.is_deadlock(), "MMIO must deadlock even with D$ on: {mmio_dcache_on:?}");
    }

    #[test]
    fn forward_progress_property_toggles_hang() {
        // With the run-ahead-under-stall property OFF, the same colliding MMIO
        // layout is merely slow (fetch quiesces on stall → load granted).
        let base = 0x0002_0000;
        let cold = 0x0002_0000 + (IC_NUM_SETS as u32) * (IC_LINE_SIZE as u32);
        let prog = PollLoopProgram::new(base, cold, 0x0100_1040, true);

        let mut cfg = TimingConfig::default();
        cfg.fetch_runs_ahead_under_stall = false;
        let mut core = MiniCore::new(prog, cfg, false);
        let outcome = core.run(64, 2_000_000);
        assert!(outcome.is_pass(), "quiescing fetch must not deadlock: {outcome:?}");
    }

    #[test]
    fn hang_verdict_flips_across_set_sweep() {
        // Sweep the loop base across all 128 set positions with a fixed cold
        // line; both hang and no-hang must occur (a genuine collision sweep).
        let cold = 0x0003_0000u32;
        let mut hangs = 0u32;
        let mut passes = 0u32;
        for set in 0..IC_NUM_SETS as u32 {
            let base = 0x0002_0000 + set * (IC_LINE_SIZE as u32);
            match run_base(base, cold, true, false) {
                RunOutcome::Deadlock { .. } => hangs += 1,
                RunOutcome::Pass { .. } => passes += 1,
                RunOutcome::Timeout { .. } => {}
            }
        }
        assert!(hangs > 0, "sweep must produce at least one hang");
        assert!(passes > 0, "sweep must produce at least one pass");
    }
}
