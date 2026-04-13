//! Peripheral trait and shared types for the BCM55030 peripheral model.
//!
//! Every MMIO subsystem of the BCM55030 is modelled as a concrete Rust type
//! implementing [`Peripheral`]. The [`PeripheralBank`](super::bank::PeripheralBank)
//! owns them by value, routes MMIO accesses to the correct peripheral based
//! on address ranges, ticks them periodically from the CPU, and exposes
//! snapshots + event injection points for the UI and MCP server.

use crate::cpu::exception::Exception;

/// Contiguous `[start, end)` MMIO address range claimed by a peripheral.
#[derive(Clone, Copy, Debug)]
pub struct AddressRange {
    pub start: u32,
    pub end: u32,
}

impl AddressRange {
    #[inline]
    pub const fn new(start: u32, end: u32) -> Self {
        Self { start, end }
    }

    #[inline]
    pub fn contains(&self, addr: u32) -> bool {
        addr >= self.start && addr < self.end
    }
}

/// Access width for MMIO reads and writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccessWidth {
    Byte,
    Half,
    Word,
}

/// DMA-style operations produced by a peripheral that target SRAM or flash.
///
/// A peripheral never touches [`Memory`](crate::memory::Memory) directly —
/// it describes the desired operation via [`DatapathOp`] and the bank
/// hands the list back to [`Memory`] after the bank lock is released.
/// This keeps the lock-holding critical section short and makes it
/// impossible for a peripheral to recursively acquire the bank lock while
/// poking SRAM.
#[derive(Clone, Debug)]
pub enum DatapathOp {
    /// DMA write into SRAM. Invalidates affected I/D cache lines on the
    /// receiving side.
    SramWrite {
        sram_addr: u32,
        data: Vec<u8>,
    },
    /// DCCM → flash write. The memory layer reads `length` bytes from
    /// `sram_addr` and calls the PBC peripheral back with the data.
    FlashWrite {
        peripheral: PeripheralId,
        flash_addr: u32,
        sram_addr: u32,
        length: usize,
    },
}

/// Opaque identifier for a peripheral inside the bank.
/// Used by [`DatapathOp::FlashWrite`] so the memory layer knows which
/// peripheral to call back with the SRAM data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PeripheralId {
    Pbc,
}

/// Error returned by [`Peripheral::inject_event`] when the peripheral does
/// not recognise the injected event variant.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PeripheralError {
    UnsupportedEvent,
    InvalidParameter(&'static str),
}

/// UI-driven mutation events. One variant per peripheral — each
/// peripheral dispatches on its own sub-enum.
#[derive(Clone, Debug)]
pub enum PeripheralEvent {
    Uart(UartEvent),
    Sfp(SfpEvent),
    Bsc(BscEvent),
    Pbc(PbcEvent),
    SerDes(SerDesEvent),
    Epon(EponEvent),
    Macsec(MacsecEvent),
    Dma(DmaEvent),
    Alarm(AlarmEvent),
}

/// Alarm dispatch UI-driven mutations. Session 5 exposes these as
/// a **test harness**, not a faithful HW path — the real alarm
/// events on silicon arrive through EPON MAC LLID teardown,
/// stats-counter overflow, and GPIO/PMD pin changes. Forcing an
/// opcode manually lets the UI drive the firmware's alarm
/// handlers without the underlying event source.
#[derive(Clone, Debug)]
pub enum AlarmEvent {
    /// Raise the persistent-pending bit for an alarm opcode.
    ForcePending(u16),
    /// Clear the persistent-pending bit for an alarm opcode.
    ClearPending(u16),
    /// Drop every forced-pending opcode back to zero.
    ClearAll,
}

#[derive(Clone, Debug)]
pub enum MacsecEvent {
    /// Force a PN (packet number) overflow on an SA slot index. The
    /// firmware reads the overflow bit during its periodic key
    /// rotation check and expects to see it cleared after
    /// acknowledging.
    InjectPnOverflow(u8),
    /// Wipe all programmed SAs — equivalent to the
    /// `MACSEC_CHANNEL_RESET` write with `0xFFFFFFFF`.
    ResetSaTable,
}

#[derive(Clone, Debug)]
pub enum DmaEvent {
    /// Force a queue-entry ready state on a specific channel. Used
    /// by the UI "inject frame" button.
    InjectQueueEntry(u8),
    /// Inject a bus-error flag on a channel so the firmware fault
    /// recovery path fires.
    InjectBusError(u8),
}

/// EPON MAC UI-driven mutations.
#[derive(Clone, Debug)]
pub enum EponEvent {
    /// Set the active bitmap bit for a given LLID index.
    SetLlidActive(u8, bool),
    /// Raise the per-LLID IRQ pending bit — the firmware reads this
    /// via the IRQ status registers at stride `0x200`.
    InjectLlidInterrupt(u8),
    /// Clear every LLID counter back to zero. The UI surfaces this
    /// as a "reset counters" button.
    ResetCounters,
}

#[derive(Clone, Debug)]
pub enum SerDesEvent {
    SetLaneEnabled(u8, bool),
    SetLinkLocked(u8, bool),
    InjectRxLos(u8, bool),
    SetLaneSpeed(u8, LaneSpeed),
    ClearErrorStatus,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaneSpeed {
    OneGigabit,
    TenGigabit,
    Pon1G,
    Pon10G,
}

#[derive(Clone, Debug)]
pub enum UartEvent {
    /// Inject a sequence of bytes into the UART receive queue. Equivalent
    /// to the mpsc channel pathway but batched.
    Bytes(Vec<u8>),
    /// Emulate a break signal.
    Break,
    /// Clear the TX log ring.
    ClearTxLog,
}

#[derive(Clone, Debug)]
pub enum SfpEvent {
    SetTemperatureC256(i16),
    SetVccUv(u32),
    SetTxBiasUa(u32),
    SetTxPowerUw(u32),
    SetRxPowerUw(u32),
    SetVendorName([u8; 16]),
    SetSerialNumber([u8; 16]),
    SetPartNumber([u8; 16]),
}

#[derive(Clone, Debug)]
pub enum BscEvent {
    ForceNack,
    Reset,
}

#[derive(Clone, Debug)]
pub enum PbcEvent {
    LoadFlashFromFile(std::path::PathBuf),
    DumpFlashToFile(std::path::PathBuf),
    EraseSector(u32),
}

/// Immutable snapshot of a peripheral's state for UI display. Peripherals
/// return a variant-specific payload; the UI dispatches on the enum.
#[derive(Clone, Debug)]
pub enum PeripheralSnapshot {
    Empty { name: &'static str },
    Uart(UartSnapshot),
    Sfp(SfpSnapshot),
    Pbc(PbcSnapshot),
    Bsc(BscSnapshot),
    SerDes(SerDesSnapshot),
    EponMac(EponMacSnapshot),
    Macsec(MacsecSnapshot),
    Dma(DmaSnapshot),
    Alarm(AlarmSnapshot),
}

impl PeripheralSnapshot {
    pub fn empty(name: &'static str) -> Self {
        Self::Empty { name }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Empty { name } => name,
            Self::Uart(_) => "uart",
            Self::Sfp(_) => "sfp",
            Self::Pbc(_) => "pbc",
            Self::Bsc(_) => "bsc_i2c",
            Self::SerDes(_) => "serdes",
            Self::EponMac(_) => "epon_mac",
            Self::Macsec(_) => "macsec",
            Self::Dma(_) => "dma",
            Self::Alarm(_) => "alarm_events",
        }
    }
}

#[derive(Clone, Debug)]
pub struct MacsecSnapshot {
    pub control: u32,
    pub enable_mode: u32,
    pub key_engine_busy: bool,
    pub pn_threshold_busy: bool,
    pub sa_slots_programmed: u8,
    pub pn_overflow_mask: u32,
}

#[derive(Clone, Debug)]
pub struct AlarmSnapshot {
    /// Opcodes currently held pending by the UI test harness. Up
    /// to 32 distinct opcodes can be tracked.
    pub forced_opcodes: Vec<u16>,
    /// Opcodes pulled from upstream event sources this tick. Empty
    /// in v1 — wiring to real source peripherals lands in a later
    /// session (stats overflow, LLID teardown, GPIO pin change).
    pub live_opcodes: Vec<u16>,
}

#[derive(Clone, Debug)]
pub struct DmaSnapshot {
    pub channels_enabled: u32,
    pub channels_busy: u32,
    pub irq_pending_bitmap: u32,
}

#[derive(Clone, Debug)]
pub struct EponMacSnapshot {
    pub chip_id: u32,
    pub chip_rev: u32,
    pub llid_active_bitmap: u32,
    pub llid_capture_mask: u32,
    pub rx_grant_mask: u32,
    pub tx_grant_mask: u32,
    pub irq_mask: u32,
    pub llid_irq_pending: [u32; 6],
}

#[derive(Clone, Debug)]
pub struct SerDesSnapshot {
    pub lanes: [SerDesLaneSnapshot; 4],
    pub error_status: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct SerDesLaneSnapshot {
    pub enabled: bool,
    pub locked: bool,
    pub rx_los: bool,
    pub speed: LaneSpeed,
}

#[derive(Clone, Debug)]
pub struct UartSnapshot {
    pub ier: u8,
    pub baud_divisor: u16,
    pub rx_queue_len: usize,
    pub tx_log_tail: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct SfpSnapshot {
    pub vendor: String,
    pub serial: String,
    pub part_number: String,
    pub temperature_c256: i16,
    pub vcc_uv: u32,
    pub tx_bias_ua: u32,
    pub tx_power_uw: u32,
    pub rx_power_uw: u32,
}

#[derive(Clone, Debug)]
pub struct PbcSnapshot {
    pub flash_dirty: bool,
    pub flash_size: usize,
    pub spi_control: u32,
    pub dma_flash_addr: u32,
    pub dma_data_addr: u32,
}

#[derive(Clone, Debug)]
pub struct BscSnapshot {
    pub busy: bool,
    pub last_device_addr: u8,
    pub last_word_addr: u16,
}

/// Common contract implemented by every MMIO subsystem in the BCM55030
/// model. Peripherals are `Send + Sync` so the bank can live behind an
/// `Arc<RwLock<...>>`, shared between the CPU thread and the UI/MCP
/// threads.
pub trait Peripheral: Send + Sync {
    /// Short canonical identifier used by logs, snapshots and tests.
    fn name(&self) -> &'static str;

    /// Contiguous MMIO ranges this peripheral owns. Must not overlap with
    /// any other peripheral — the bank panics at construction time if
    /// that invariant is violated.
    fn address_ranges(&self) -> &'static [AddressRange];

    /// Word-granular read with side effects. `addr` is absolute.
    fn read_word(&mut self, addr: u32) -> Result<u32, Exception>;

    /// Word-granular write with side effects.
    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception>;

    /// Byte-granular read. Default implementation extracts the byte from
    /// the containing word, which is safe for peripherals whose reads do
    /// NOT have byte-level side effects. Peripherals with FIFO-pop-on-
    /// read semantics (UART) must override this.
    fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        let word_addr = addr & !3;
        let byte_idx = addr & 3;
        let word = self.read_word(word_addr)?;
        Ok((word >> (24 - byte_idx * 8)) as u8)
    }

    /// Halfword-granular read. Default implementation extracts the half
    /// from the containing word. Override for side-effect-bearing reads.
    fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        let word_addr = addr & !3;
        let half_idx = (addr >> 1) & 1;
        let word = self.read_word(word_addr)?;
        Ok((word >> (16 - half_idx * 16)) as u16)
    }

    /// Byte-granular write. Default implementation read-modify-writes the
    /// containing word. NOTE: this calls `read_word` which may trigger
    /// side effects on peripherals that have them. UART must override.
    fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        let word_addr = addr & !3;
        let byte_idx = addr & 3;
        let old = self.read_word(word_addr)?;
        let shift = 24 - byte_idx * 8;
        let mask = !(0xFFu32 << shift);
        let new = (old & mask) | ((val as u32) << shift);
        self.write_word(word_addr, new)
    }

    /// Halfword-granular write. Default implementation read-modify-writes
    /// the containing word.
    fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        let word_addr = addr & !3;
        let half_idx = (addr >> 1) & 1;
        let old = self.read_word(word_addr)?;
        let shift = 16 - half_idx * 16;
        let mask = !(0xFFFFu32 << shift);
        let new = (old & mask) | ((val as u32) << shift);
        self.write_word(word_addr, new)
    }

    /// Periodic tick — invoked by the bank once per
    /// `BANK_TICK_PRESCALER` CPU instructions. Peripherals use this to
    /// advance timers, pop FIFO entries, raise IRQ pending bits, etc.
    fn tick(&mut self, _cpu_instructions: u64) {}

    /// Cold reset — zeroes all volatile state. Non-volatile storage
    /// (flash, EEPROM, efuse snapshot) is preserved. Called on a hard
    /// reboot (FLAG 1) and at startup when `--cold-boot` is passed.
    fn reset_cold(&mut self);

    /// Warm reset — called at startup when `--warm-boot` is passed
    /// (default). Default implementation defers to `reset_cold`;
    /// peripherals that need to load a snapshot of post-init state from
    /// `src/soc/mmio_init.rs` override this method.
    fn reset_warm(&mut self) {
        self.reset_cold();
    }

    /// Snapshot the peripheral's display state for UI rendering. Default
    /// returns [`PeripheralSnapshot::Empty`] — peripherals that have
    /// interesting state should override.
    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::empty(self.name())
    }

    /// Apply a UI-driven state mutation. Default returns
    /// [`PeripheralError::UnsupportedEvent`]; peripherals that accept
    /// injection override this method and dispatch on their sub-enum.
    fn inject_event(&mut self, _event: &PeripheralEvent) -> Result<(), PeripheralError> {
        Err(PeripheralError::UnsupportedEvent)
    }

    /// Does this peripheral have pending datapath operations to apply?
    /// The bank polls this after every `write_word` that modified the
    /// peripheral's state; if true the bank calls
    /// [`take_pending_datapath`](Self::take_pending_datapath).
    fn has_pending_datapath(&self) -> bool {
        false
    }

    /// Drain any pending datapath operations. The bank forwards the
    /// returned `DatapathOp`s to `Memory::apply_datapath` after releasing
    /// the bank lock.
    fn take_pending_datapath(&mut self) -> Vec<DatapathOp> {
        Vec::new()
    }

    /// Is this peripheral currently raising a CPU interrupt?
    /// The bank ORs all peripherals' IRQ masks during `tick()`.
    fn irq_pending(&self) -> u32 {
        0
    }
}

/// Helper: check if an address falls inside any of the ranges.
#[inline]
pub fn ranges_contain(ranges: &[AddressRange], addr: u32) -> bool {
    ranges.iter().any(|r| r.contains(addr))
}
