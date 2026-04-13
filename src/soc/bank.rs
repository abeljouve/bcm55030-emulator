//! Peripheral bank — the central MMIO routing and tick orchestrator.
//!
//! The bank owns the full set of BCM55030 peripheral models by value and
//! routes every MMIO load / store from the CPU to the peripheral that
//! claims the target address. It also exposes the mpsc channel that UI
//! and `main.rs` use to feed bytes into the UART receive path.
//!
//! The bank is designed to live behind `Arc<parking_lot::RwLock<...>>`
//! so the UI thread (read lock at 60 Hz) and the CPU thread (write lock
//! per MMIO access) can share it without contention on the SRAM fast
//! path — which does not touch the bank at all.

use std::collections::HashMap;
use std::sync::mpsc;

use crate::cpu::exception::Exception;
use crate::soc::alarm_events::AlarmEvents;
use crate::soc::bsc_i2c::BscI2c;
use crate::soc::dma::DmaChannelController;
use crate::soc::efuse_udr::EfuseUdr;
use crate::soc::epon_mac::EponMac;
use crate::soc::fatal_filter::FatalFilter;
use crate::soc::macsec::Macsec;
use crate::soc::pbc::Pbc;
use crate::soc::peripheral::{
    DatapathOp, Peripheral, PeripheralEvent, PeripheralId, PeripheralSnapshot,
};
use crate::soc::serdes::SerDes;
use crate::soc::sysreg_shim::SysregShim;
use crate::soc::timer::EponTimer;
use crate::soc::uart::Uart;

/// CPU instruction ticks between bank tick invocations. Higher = less
/// contention but coarser peripheral advancement. The EPON free-running
/// counter assumes 64 — keep the default here.
pub const BANK_TICK_PRESCALER: u64 = 64;

/// Aggregated MMIO trace entry (replaces the one previously owned by
/// `mmio::MmioController`). Used by `--dump-mmio-trace` / cold-boot
/// trace capture.
#[derive(Default, Clone, Debug)]
pub struct MmioTraceEntry {
    pub peripheral: &'static str,
    pub reads: u64,
    pub writes: u64,
    pub last_read_value: u32,
    pub last_write_value: u32,
    pub first_pc: u32,
    pub first_insn: u64,
    /// bit 0 = byte access, bit 1 = half, bit 2 = word
    pub access_widths: u8,
}

/// Boot mode — controls whether peripherals apply the post-boot snapshot
/// in their `reset_warm()` path or start at cold-reset values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootMode {
    Cold,
    Warm,
}

pub struct PeripheralBank {
    pub uart: Uart,
    pub pbc: Pbc,
    pub bsc_i2c: BscI2c,
    pub serdes: SerDes,
    pub epon_mac: EponMac,
    pub macsec: Macsec,
    pub dma: DmaChannelController,
    pub alarm_events: AlarmEvents,
    pub timer: EponTimer,
    pub efuse_udr: EfuseUdr,
    pub fatal_filter: FatalFilter,

    /// Temporary residual-plus-legacy-arms for the SYSREG range
    /// (`0x01000000..0x01003800`). Hosts every stub that has not yet been
    /// carved into its own peripheral file — CHIP_ID, LLID IRQ forced 0,
    /// SerDes forced bits, I²C UDR bit-bang, DMA queue drain, alarm
    /// counters, etc. This struct shrinks as Sessions 2–7 land their
    /// dedicated peripherals.
    pub sysreg: SysregShim,

    uart_rx_sender: mpsc::Sender<u8>,
    uart_rx_receiver: mpsc::Receiver<u8>,

    /// Current CPU context for trace entries and watchpoint messages.
    pub current_pc: u32,
    pub current_blink: u32,
    pub current_insn: u64,

    /// Optional aggregated MMIO trace for `--dump-mmio-trace`.
    pub mmio_trace: Option<HashMap<u32, MmioTraceEntry>>,

    /// Verbose trace of every MMIO access (unbounded — only enable via
    /// `--trace-mmio`). Mirrors the old `MmioController::trace` flag.
    pub trace: bool,

    /// Aggregated IRQ pending mask from peripherals. `Cpu::step()` ORs
    /// this into `aux_irq_pending` after each bank tick. Session 1 only
    /// UART contributes; other peripherals will add their bits once they
    /// land.
    pub irq_pending: u32,

    /// Boot mode — peripherals snapshot this during construction.
    pub boot_mode: BootMode,
}

impl PeripheralBank {
    pub fn new(boot_mode: BootMode) -> Self {
        let (tx, rx) = mpsc::channel::<u8>();
        let mut bank = Self {
            uart: Uart::new(),
            pbc: Pbc::new(),
            bsc_i2c: BscI2c::new(),
            serdes: SerDes::new(),
            epon_mac: EponMac::new(),
            macsec: Macsec::new(),
            dma: DmaChannelController::new(),
            alarm_events: AlarmEvents::new(),
            timer: EponTimer::new(),
            efuse_udr: EfuseUdr::new(),
            fatal_filter: FatalFilter::new(),
            sysreg: SysregShim::new(),
            uart_rx_sender: tx,
            uart_rx_receiver: rx,
            current_pc: 0,
            current_blink: 0,
            current_insn: 0,
            mmio_trace: None,
            trace: false,
            irq_pending: 0,
            boot_mode,
        };
        // Apply the requested reset flavour.
        match boot_mode {
            BootMode::Cold => bank.reset_cold(),
            BootMode::Warm => bank.reset_warm(),
        }
        bank
    }

    /// Return a cloneable sender that pushes bytes into the UART receive
    /// path. The main loop (`main.rs`) and the future UI both acquire a
    /// clone at startup and then feed bytes without touching the bank
    /// directly.
    pub fn uart_rx_sender(&self) -> mpsc::Sender<u8> {
        self.uart_rx_sender.clone()
    }

    /// Drain the mpsc channel into the UART receive queue. Called from
    /// [`tick`] before other peripherals tick, so UART IRQ bits that
    /// arise from newly-received bytes are visible in the same pass.
    fn drain_uart_channel(&mut self) {
        while let Ok(b) = self.uart_rx_receiver.try_recv() {
            self.uart.push_rx_byte(b);
        }
    }

    /// Advance all peripherals by `cpu_instructions` CPU cycles.
    /// Called from `Cpu::step()` once per `BANK_TICK_PRESCALER`.
    pub fn tick(&mut self, cpu_instructions: u64) {
        self.drain_uart_channel();
        self.uart.tick(cpu_instructions);
        self.pbc.tick(cpu_instructions);
        self.bsc_i2c.tick(cpu_instructions);
        self.serdes.tick(cpu_instructions);
        self.epon_mac.tick(cpu_instructions);
        self.macsec.tick(cpu_instructions);
        self.dma.tick(cpu_instructions);
        self.alarm_events.tick(cpu_instructions);
        self.timer.tick(cpu_instructions);
        self.efuse_udr.tick(cpu_instructions);
        self.fatal_filter.tick(cpu_instructions);
        self.sysreg.tick(cpu_instructions);

        // Aggregate IRQ pending bits. UART is the only v1 contributor
        // (IRQ 5, level 1). Other peripherals will add their bits once
        // they land.
        self.irq_pending = self.uart.irq_pending();
    }

    /// Cold reset — zeros volatile state in all peripherals. Called on
    /// FLAG 1 reboot and at startup in `--cold-boot` mode.
    pub fn reset_cold(&mut self) {
        self.uart.reset_cold();
        self.pbc.reset_cold();
        self.bsc_i2c.reset_cold();
        self.serdes.reset_cold();
        self.epon_mac.reset_cold();
        self.macsec.reset_cold();
        self.dma.reset_cold();
        self.alarm_events.reset_cold();
        self.timer.reset_cold();
        self.efuse_udr.reset_cold();
        self.fatal_filter.reset_cold();
        self.sysreg.reset_cold();
        self.irq_pending = 0;
        self.current_pc = 0;
        self.current_blink = 0;
        self.current_insn = 0;
    }

    /// Warm reset — loads the post-boot snapshot on top of cold reset.
    pub fn reset_warm(&mut self) {
        self.uart.reset_warm();
        self.pbc.reset_warm();
        self.bsc_i2c.reset_warm();
        self.serdes.reset_warm();
        self.epon_mac.reset_warm();
        self.macsec.reset_warm();
        self.dma.reset_warm();
        self.alarm_events.reset_warm();
        self.timer.reset_warm();
        self.efuse_udr.reset_warm();
        self.fatal_filter.reset_warm();
        self.sysreg.reset_warm();
        self.irq_pending = 0;
        self.current_pc = 0;
        self.current_blink = 0;
        self.current_insn = 0;
    }

    /// Update the CPU context fields in one shot. Called by `Cpu::step()`
    /// before every MMIO access so trace entries can show the touching
    /// PC / blink / insn count.
    pub fn update_cpu_context(&mut self, pc: u32, blink: u32, insn: u64) {
        self.current_pc = pc;
        self.current_blink = blink;
        self.current_insn = insn;
        self.sysreg.update_cpu_context(pc, insn);
    }

    /// Snapshot each peripheral for UI display. Cheap (shallow clones)
    /// so the UI can call it every frame under a short read lock.
    pub fn snapshot_all(&self) -> Vec<PeripheralSnapshot> {
        vec![
            self.uart.snapshot(),
            self.pbc.snapshot(),
            self.bsc_i2c.snapshot(),
            self.serdes.snapshot(),
            self.epon_mac.snapshot(),
            self.macsec.snapshot(),
            self.dma.snapshot(),
            self.alarm_events.snapshot(),
            self.timer.snapshot(),
            self.efuse_udr.snapshot(),
            self.fatal_filter.snapshot(),
        ]
    }

    /// Dispatch a UI event to the peripheral that understands it.
    pub fn inject_event(&mut self, event: &PeripheralEvent) -> bool {
        let targets: [&mut dyn Peripheral; 11] = [
            &mut self.uart,
            &mut self.pbc,
            &mut self.bsc_i2c,
            &mut self.serdes,
            &mut self.epon_mac,
            &mut self.macsec,
            &mut self.dma,
            &mut self.alarm_events,
            &mut self.timer,
            &mut self.efuse_udr,
            &mut self.fatal_filter,
        ];
        for p in targets {
            if p.inject_event(event).is_ok() {
                return true;
            }
        }
        false
    }

    /// Drain pending datapath operations from every peripheral. The
    /// caller (`Memory::apply_datapath`) applies them to SRAM / flash
    /// after releasing the bank write lock.
    pub fn take_pending_datapath(&mut self) -> Vec<DatapathOp> {
        let mut ops = Vec::new();
        ops.append(&mut self.pbc.take_pending_datapath());
        ops
    }

    /// PBC flash-write callback. `Memory::apply_datapath` collects the
    /// SRAM bytes and hands them back via this method for peripherals
    /// that emitted a `DatapathOp::FlashWrite`.
    pub fn complete_flash_write(
        &mut self,
        peripheral: PeripheralId,
        flash_addr: u32,
        data: &[u8],
    ) {
        match peripheral {
            PeripheralId::Pbc => self.pbc.complete_flash_write(flash_addr, data),
        }
    }

    // -------- MMIO routing --------

    pub fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        if self.uart.claims(addr) {
            return self.uart.read_word(addr);
        }
        if self.pbc.claims(addr) {
            return self.pbc.read_word(addr);
        }
        if self.bsc_i2c.claims(addr) {
            return self.bsc_i2c.read_word(addr);
        }
        if self.serdes.claims(addr) {
            return self.serdes.read_word(addr);
        }
        if self.epon_mac.claims(addr) {
            return self.epon_mac.read_word(addr);
        }
        if self.macsec.claims(addr) {
            return self.macsec.read_word(addr);
        }
        if self.dma.claims(addr) {
            return self.dma.read_word(addr);
        }
        if self.timer.claims(addr) {
            return self.timer.read_word(addr);
        }
        if self.efuse_udr.claims(addr) {
            return self.efuse_udr.read_word(addr);
        }
        if self.fatal_filter.claims(addr) {
            return self.fatal_filter.read_word(addr);
        }
        if self.sysreg.claims(addr) {
            return self.sysreg.read_word(addr);
        }
        if self.trace {
            eprintln!("[MMIO] read  word  0x{:08X} → 0x00000000 (unmapped)", addr);
        }
        Ok(0)
    }

    pub fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        if self.uart.claims(addr) {
            return self.uart.write_word(addr, val);
        }
        if self.pbc.claims(addr) {
            self.pbc.write_word(addr, val)?;
            // Cross-peripheral dispatch: if the write triggered a
            // SerDes SPI slave command, route it to the SerDes now
            // that the PBC write is complete.
            if let Some((tx, rx_len)) = self.pbc.take_pending_spi_serdes() {
                let rx = self.serdes.spi_command(&tx, rx_len);
                self.pbc.complete_spi_serdes(&rx);
            }
            return Ok(());
        }
        if self.bsc_i2c.claims(addr) {
            return self.bsc_i2c.write_word(addr, val);
        }
        if self.serdes.claims(addr) {
            return self.serdes.write_word(addr, val);
        }
        if self.epon_mac.claims(addr) {
            return self.epon_mac.write_word(addr, val);
        }
        if self.macsec.claims(addr) {
            return self.macsec.write_word(addr, val);
        }
        if self.dma.claims(addr) {
            return self.dma.write_word(addr, val);
        }
        if self.timer.claims(addr) {
            return self.timer.write_word(addr, val);
        }
        if self.efuse_udr.claims(addr) {
            return self.efuse_udr.write_word(addr, val);
        }
        if self.fatal_filter.claims(addr) {
            return self.fatal_filter.write_word(addr, val);
        }
        if self.sysreg.claims(addr) {
            return self.sysreg.write_word(addr, val);
        }
        if self.trace {
            eprintln!("[MMIO] write word  0x{:08X} = 0x{:08X} (unmapped)", addr, val);
        }
        Ok(())
    }

    pub fn read_half(&mut self, addr: u32) -> Result<u16, Exception> {
        if self.uart.claims(addr) {
            return self.uart.read_half(addr);
        }
        if self.pbc.claims(addr) {
            return self.pbc.read_half(addr);
        }
        if self.bsc_i2c.claims(addr) {
            return self.bsc_i2c.read_half(addr);
        }
        if self.serdes.claims(addr) {
            return self.serdes.read_half(addr);
        }
        if self.epon_mac.claims(addr) {
            return self.epon_mac.read_half(addr);
        }
        if self.macsec.claims(addr) {
            return self.macsec.read_half(addr);
        }
        if self.dma.claims(addr) {
            return self.dma.read_half(addr);
        }
        if self.timer.claims(addr) {
            return self.timer.read_half(addr);
        }
        if self.efuse_udr.claims(addr) {
            return self.efuse_udr.read_half(addr);
        }
        if self.fatal_filter.claims(addr) {
            return self.fatal_filter.read_half(addr);
        }
        if self.sysreg.claims(addr) {
            return self.sysreg.read_half(addr);
        }
        Ok(0)
    }

    pub fn write_half(&mut self, addr: u32, val: u16) -> Result<(), Exception> {
        if self.uart.claims(addr) {
            return self.uart.write_half(addr, val);
        }
        if self.pbc.claims(addr) {
            return self.pbc.write_half(addr, val);
        }
        if self.bsc_i2c.claims(addr) {
            return self.bsc_i2c.write_half(addr, val);
        }
        if self.serdes.claims(addr) {
            return self.serdes.write_half(addr, val);
        }
        if self.epon_mac.claims(addr) {
            return self.epon_mac.write_half(addr, val);
        }
        if self.macsec.claims(addr) {
            return self.macsec.write_half(addr, val);
        }
        if self.dma.claims(addr) {
            return self.dma.write_half(addr, val);
        }
        if self.timer.claims(addr) {
            return self.timer.write_half(addr, val);
        }
        if self.efuse_udr.claims(addr) {
            return self.efuse_udr.write_half(addr, val);
        }
        if self.fatal_filter.claims(addr) {
            return self.fatal_filter.write_half(addr, val);
        }
        if self.sysreg.claims(addr) {
            return self.sysreg.write_half(addr, val);
        }
        Ok(())
    }

    pub fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        if self.uart.claims(addr) {
            return self.uart.read_byte(addr);
        }
        if self.pbc.claims(addr) {
            return self.pbc.read_byte(addr);
        }
        if self.bsc_i2c.claims(addr) {
            return self.bsc_i2c.read_byte(addr);
        }
        if self.serdes.claims(addr) {
            return self.serdes.read_byte(addr);
        }
        if self.epon_mac.claims(addr) {
            return self.epon_mac.read_byte(addr);
        }
        if self.macsec.claims(addr) {
            return self.macsec.read_byte(addr);
        }
        if self.dma.claims(addr) {
            return self.dma.read_byte(addr);
        }
        if self.timer.claims(addr) {
            return self.timer.read_byte(addr);
        }
        if self.efuse_udr.claims(addr) {
            return self.efuse_udr.read_byte(addr);
        }
        if self.fatal_filter.claims(addr) {
            return self.fatal_filter.read_byte(addr);
        }
        if self.sysreg.claims(addr) {
            return self.sysreg.read_byte(addr);
        }
        Ok(0)
    }

    pub fn write_byte(&mut self, addr: u32, val: u8) -> Result<(), Exception> {
        if self.uart.claims(addr) {
            return self.uart.write_byte(addr, val);
        }
        if self.pbc.claims(addr) {
            return self.pbc.write_byte(addr, val);
        }
        if self.bsc_i2c.claims(addr) {
            return self.bsc_i2c.write_byte(addr, val);
        }
        if self.serdes.claims(addr) {
            return self.serdes.write_byte(addr, val);
        }
        if self.epon_mac.claims(addr) {
            return self.epon_mac.write_byte(addr, val);
        }
        if self.macsec.claims(addr) {
            return self.macsec.write_byte(addr, val);
        }
        if self.dma.claims(addr) {
            return self.dma.write_byte(addr, val);
        }
        if self.timer.claims(addr) {
            return self.timer.write_byte(addr, val);
        }
        if self.efuse_udr.claims(addr) {
            return self.efuse_udr.write_byte(addr, val);
        }
        if self.fatal_filter.claims(addr) {
            return self.fatal_filter.write_byte(addr, val);
        }
        if self.sysreg.claims(addr) {
            return self.sysreg.write_byte(addr, val);
        }
        Ok(())
    }
}
