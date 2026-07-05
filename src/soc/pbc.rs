//! Peripheral Bus Controller — SPI flash FIFO + SPI DMA + SerDes SPI
//! slave stub. MMIO aperture `0x010001F0..0x01000240`.
//!
//! Owns the on-module `SpiFlash` instance by value. All SRAM ↔ flash
//! DMA happens via [`DatapathOp`] emission — the bank drains them after
//! releasing the write lock so `Memory::apply_datapath` can poke SRAM
//! without recursive locking.

use crate::cpu::exception::Exception;
use crate::soc::peripheral::{
    AddressRange, DatapathOp, Peripheral, PeripheralError, PeripheralEvent, PeripheralId,
    PeripheralSnapshot, PbcEvent, PbcSnapshot,
};
use crate::soc::spi_flash::SpiFlash;

/// DMA physical base for DCCM (the DMA engine sees DCCM at this address)
const DMA_DCCM_BASE: u32 = 0xFFF80000;
const DMA_DCCM_SIZE: u32 = 0x80000; // 512 KB

pub const PBC_BASE: u32 = 0x010001F0;
pub const PBC_END: u32 = 0x01000240;

const REG_SPI_CONTROL: u32 = 0x10;
const REG_SPI_STATUS: u32 = 0x1C;
const REG_SPI_FIFO_DATA: u32 = 0x20;
const REG_SPI_FIFO_DATA1: u32 = 0x24;
const REG_SPI_READ_DATA: u32 = 0x2C;
const REG_SPI_READ_DATA1: u32 = 0x30;
const REG_SPI_CONFIG: u32 = 0x34;
const REG_DMA_CTRL: u32 = 0x38;
const REG_DMA_ADDR: u32 = 0x3C;
const REG_DMA_DATA_ADDR: u32 = 0x40;

const PBC_RANGES: &[AddressRange] = &[AddressRange::new(PBC_BASE, PBC_END)];

/// How many bank ticks the SPI busy bit stays set after a command.
/// Audit 6.1: real hardware is slower than a single CPU step — the
/// firmware polling loop must see `busy=1` at least once.
const SPI_BUSY_TICKS: u8 = 2;

#[derive(Clone)]
pub struct Pbc {
    pub flash: SpiFlash,
    pub trace: bool,

    spi_control: u32,
    spi_config: u32,
    spi_fifo: [u32; 2],
    spi_rx: [u32; 2],
    spi_busy_counter: u8,

    dma_ctrl: u32,
    dma_flash_addr: u32,
    dma_data_addr: u32,
    dma_busy_counter: u8,

    /// Pending datapath operations emitted by the last MMIO write. The
    /// bank drains them after releasing the write lock.
    pending_ops: Vec<DatapathOp>,

    /// Pending SerDes SPI slave command — set when `SPI_CONTROL & 0x40`
    /// routed a FIFO command to the SerDes target. The bank dispatches
    /// the stored `(tx, rx_len)` pair to `SerDes::spi_command` and
    /// feeds the response back via [`Pbc::complete_spi_serdes`].
    /// Resolves audit 5.2 — the SerDes slave path is no longer a
    /// hardcoded `0xFF` stub inside PBC.
    pending_spi_serdes: Option<(Vec<u8>, usize)>,

    /// Last DMA transaction type for MMIO history annotation.
    /// Set on REG_DMA_CTRL write based on ADDR and CMD encoding.
    /// Evidence: session 2026-05-05-1430, PBC register analysis.
    pub last_dma_tag: &'static str,

    /// Optional DMA-flash-WRITE recorder (boot-diff DMADIFF instrumentation).
    /// When `Some`, every `complete_flash_write` appends `(flash_addr, data)`.
    /// The DMA write PAYLOAD is read from the SRAM buffer and is NOT in the
    /// MMIO write stream, so this is the only way to diff FDS-record CONTENT
    /// for differential MMIO tracing between two firmware builds. Pure HW-model
    /// observation, not a firmware hook.
    pub dma_write_log: Option<Vec<(u32, Vec<u8>)>>,
}

impl Pbc {
    pub fn new() -> Self {
        Self {
            flash: SpiFlash::new(),
            trace: false,
            spi_control: 0,
            spi_config: 0,
            spi_fifo: [0; 2],
            spi_rx: [0; 2],
            spi_busy_counter: 0,
            dma_ctrl: 0,
            dma_flash_addr: 0,
            dma_data_addr: 0,
            dma_busy_counter: 0,
            pending_ops: Vec::new(),
            pending_spi_serdes: None,
            last_dma_tag: "pbc",
            dma_write_log: None,
        }
    }

    /// Drain the pending SerDes SPI command (if any). Called by the
    /// bank after every `write_word` that may have triggered a FIFO
    /// command routed to the SerDes slave.
    pub fn take_pending_spi_serdes(&mut self) -> Option<(Vec<u8>, usize)> {
        self.pending_spi_serdes.take()
    }

    /// Store the SerDes slave's response bytes into the SPI read
    /// buffer. Called by the bank after dispatching the pending
    /// command returned by [`Pbc::take_pending_spi_serdes`].
    pub fn complete_spi_serdes(&mut self, rx: &[u8]) {
        self.spi_rx = [0; 2];
        for (i, &byte) in rx.iter().enumerate() {
            let word_idx = i / 4;
            let byte_idx = i % 4;
            if word_idx < 2 {
                self.spi_rx[word_idx] |= (byte as u32) << (byte_idx * 8);
            }
        }
    }

    #[inline]
    pub fn claims(&self, addr: u32) -> bool {
        (PBC_BASE..PBC_END).contains(&addr)
    }

    /// DCCM-side callback from `Memory::apply_datapath` with the bytes
    /// read from SRAM to satisfy a `FlashWrite` op.
    pub fn complete_flash_write(&mut self, flash_addr: u32, data: &[u8]) {
        if self.trace {
            eprintln!(
                "[PBC] SPI DMA WRITE: {} bytes to flash 0x{:06X} data={:02X?}",
                data.len(),
                flash_addr,
                &data[..data.len().min(16)]
            );
        }
        if let Some(log) = &mut self.dma_write_log {
            log.push((flash_addr, data.to_vec()));
        }
        self.flash.dma_write(flash_addr, data);
    }

    fn read_reg_word(&mut self, offset: u32) -> u32 {
        match offset {
            REG_SPI_CONTROL => self.spi_control,
            REG_SPI_STATUS => {
                if self.spi_busy_counter > 0 {
                    self.spi_busy_counter -= 1;
                    1
                } else {
                    0
                }
            }
            REG_SPI_FIFO_DATA => self.spi_fifo[0],
            REG_SPI_FIFO_DATA1 => self.spi_fifo[1],
            REG_SPI_READ_DATA => self.spi_rx[0],
            REG_SPI_READ_DATA1 => self.spi_rx[1],
            REG_SPI_CONFIG => self.spi_config,
            REG_DMA_CTRL => {
                if self.dma_busy_counter > 0 {
                    self.dma_busy_counter -= 1;
                    1
                } else {
                    0
                }
            }
            REG_DMA_ADDR => self.dma_flash_addr,
            REG_DMA_DATA_ADDR => self.dma_data_addr,
            _ => {
                if self.trace {
                    eprintln!("[PBC] read  unknown offset 0x{:02X}", offset);
                }
                0
            }
        }
    }

    fn write_reg_word(&mut self, offset: u32, val: u32) {
        match offset {
            REG_SPI_CONTROL => {
                self.spi_control = val;
            }
            REG_SPI_STATUS => {
                if val & 1 != 0 {
                    self.last_dma_tag = if self.spi_control & 0x40 != 0 {
                        "pbc_spi_serdes"
                    } else {
                        "pbc_spi_flash"
                    };
                    self.execute_fifo_command(val);
                    self.spi_busy_counter = SPI_BUSY_TICKS;
                }
            }
            REG_SPI_FIFO_DATA => {
                self.spi_fifo[0] = val;
            }
            REG_SPI_FIFO_DATA1 => {
                self.spi_fifo[1] = val;
            }
            REG_SPI_CONFIG => {
                self.spi_config = val;
            }
            REG_DMA_CTRL => {
                self.dma_ctrl = val;
                if val & 1 != 0 {
                    let is_write = (val & 0x2) != 0;
                    let addr = self.dma_flash_addr;
                    self.last_dma_tag = if addr >= 0x40_0000 {
                        if is_write { "pbc_mdio_write" } else { "pbc_mdio_read" }
                    } else {
                        if is_write { "pbc_spi_write" } else { "pbc_spi_read" }
                    };
                    self.execute_dma(val);
                    self.dma_busy_counter = SPI_BUSY_TICKS;
                }
            }
            REG_DMA_ADDR => {
                self.dma_flash_addr = val;
            }
            REG_DMA_DATA_ADDR => {
                self.dma_data_addr = val;
            }
            _ => {
                if self.trace {
                    eprintln!("[PBC] write unknown offset 0x{:02X} = 0x{:08X}", offset, val);
                }
            }
        }
    }

    fn execute_fifo_command(&mut self, cmd_word: u32) {
        let tx_len = ((cmd_word >> 4) & 0xF) as usize;
        let rx_len = ((cmd_word >> 8) & 0xFF) as usize;

        let mut tx = Vec::with_capacity(tx_len);
        for i in 0..tx_len {
            let word_idx = i / 4;
            let byte_idx = i % 4;
            // The SPI FIFO physically holds only `spi_fifo.len()` words (8 bytes).
            // A `tx_len` field decoded from a garbage/fuzzed command word can index
            // past it; HW cannot read beyond the FIFO, so words past the end read
            // as 0 rather than panicking (index-out-of-bounds would otherwise crash
            // the worker and silently drop fuzz cases -> nondeterministic sweeps).
            let word = self.spi_fifo.get(word_idx).copied().unwrap_or(0);
            let byte = (word >> (byte_idx * 8)) as u8;
            tx.push(byte);
        }

        if self.trace {
            let opcode_str = if !tx.is_empty() {
                format!("opcode=0x{:02X}", tx[0])
            } else {
                "empty".to_string()
            };
            eprintln!(
                "[PBC] SPI FIFO: {} tx_len={} rx_len={} cmd=0x{:04X}",
                opcode_str, tx_len, rx_len, cmd_word
            );
        }

        // SPI_CONTROL bit 6: 0 = main flash, 1 = SerDes SPI slave.
        // Flash commands execute here. SerDes commands are queued in
        // `pending_spi_serdes` and the bank dispatches them to
        // `SerDes::spi_command` after releasing the PBC lock.
        if self.spi_control & 0x40 != 0 {
            self.pending_spi_serdes = Some((tx, rx_len));
            return;
        }
        let rx = self.flash.execute_fifo_command(&tx, rx_len);

        self.spi_rx = [0; 2];
        for (i, &byte) in rx.iter().enumerate() {
            let word_idx = i / 4;
            let byte_idx = i % 4;
            if word_idx < 2 {
                self.spi_rx[word_idx] |= (byte as u32) << (byte_idx * 8);
            }
        }

        if self.trace && rx_len > 0 {
            eprintln!(
                "[PBC] SPI FIFO result: rx[0]=0x{:08X} rx[1]=0x{:08X}",
                self.spi_rx[0], self.spi_rx[1]
            );
        }
    }

    fn execute_dma(&mut self, ctrl: u32) {
        let length = ((ctrl >> 4) & 0x3FFFF) as usize;
        let flash_addr = self.dma_flash_addr;
        let mem_addr = self.dma_data_addr;
        let write_to_flash = (ctrl & 0x2) != 0;

        let sram_addr = if mem_addr >= DMA_DCCM_BASE
            && mem_addr.wrapping_sub(DMA_DCCM_BASE) < DMA_DCCM_SIZE
        {
            Some(mem_addr - DMA_DCCM_BASE)
        } else if (mem_addr as usize) < 0x80000 {
            Some(mem_addr)
        } else {
            None
        };

        if write_to_flash {
            if self.trace {
                eprintln!(
                    "[PBC] SPI DMA WRITE: ctrl=0x{:08X} mem=0x{:08X} → flash=0x{:06X} len={}",
                    ctrl, mem_addr, flash_addr, length
                );
            }
            if let Some(sram_addr) = sram_addr {
                self.pending_ops.push(DatapathOp::FlashWrite {
                    peripheral: PeripheralId::Pbc,
                    flash_addr,
                    sram_addr,
                    length,
                });
            } else if self.trace {
                eprintln!("[PBC] DMA WRITE: unmapped source address 0x{:08X}", mem_addr);
            }
        } else {
            if self.trace {
                eprintln!(
                    "[PBC] SPI DMA READ: ctrl=0x{:08X} flash=0x{:06X} → mem=0x{:08X} len={}",
                    ctrl, flash_addr, mem_addr, length
                );
            }
            let data = self.flash.dma_read(flash_addr, length);
            if let Some(sram_addr) = sram_addr {
                self.pending_ops.push(DatapathOp::SramWrite { sram_addr, data });
            } else if self.trace {
                eprintln!("[PBC] DMA READ: unmapped target address 0x{:08X}", mem_addr);
            }
        }
    }
}

impl Peripheral for Pbc {
    fn name(&self) -> &'static str {
        "pbc"
    }

    fn address_ranges(&self) -> &'static [AddressRange] {
        PBC_RANGES
    }

    fn read_word(&mut self, addr: u32) -> Result<u32, Exception> {
        let off = addr - PBC_BASE;
        Ok(self.read_reg_word(off))
    }

    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), Exception> {
        let off = addr - PBC_BASE;
        self.write_reg_word(off, val);
        Ok(())
    }

    fn read_byte(&mut self, addr: u32) -> Result<u8, Exception> {
        let off = addr - PBC_BASE;
        let word_off = off & !3;
        let byte_idx = off & 3;
        let word = self.read_reg_word(word_off);
        Ok((word >> (24 - byte_idx * 8)) as u8)
    }

    fn tick(&mut self, _cpu_instructions: u64) {
        // Busy bits also decrement on bank tick so they can clear
        // between polling loop iterations.
        if self.spi_busy_counter > 0 {
            self.spi_busy_counter -= 1;
        }
        if self.dma_busy_counter > 0 {
            self.dma_busy_counter -= 1;
        }
    }

    fn reset_cold(&mut self) {
        self.spi_control = 0;
        self.spi_config = 0;
        self.spi_fifo = [0; 2];
        self.spi_rx = [0; 2];
        self.spi_busy_counter = 0;
        self.dma_ctrl = 0;
        self.dma_flash_addr = 0;
        self.dma_data_addr = 0;
        self.dma_busy_counter = 0;
        self.pending_ops.clear();
        self.pending_spi_serdes = None;
        self.last_dma_tag = "pbc";
        // flash `data` array preserved — non-volatile. Reset the volatile
        // chip-side latches (status register, SST AAI in-flight address)
        // so a cold boot starts from a clean state.
        self.flash.reset_volatile();
    }

    fn snapshot(&self) -> PeripheralSnapshot {
        PeripheralSnapshot::Pbc(PbcSnapshot {
            flash_dirty: self.flash.dirty,
            flash_size: self.flash.data.len(),
            spi_control: self.spi_control,
            dma_flash_addr: self.dma_flash_addr,
            dma_data_addr: self.dma_data_addr,
        })
    }

    fn has_pending_datapath(&self) -> bool {
        !self.pending_ops.is_empty()
    }

    fn take_pending_datapath(&mut self) -> Vec<DatapathOp> {
        std::mem::take(&mut self.pending_ops)
    }

    fn inject_event(&mut self, event: &PeripheralEvent) -> Result<(), PeripheralError> {
        match event {
            PeripheralEvent::Pbc(ev) => match ev {
                PbcEvent::LoadFlashFromFile(path) => {
                    match std::fs::read(path) {
                        Ok(data) => {
                            let n = data.len().min(self.flash.data.len());
                            self.flash.data[..n].copy_from_slice(&data[..n]);
                            self.flash.dirty = false;
                            Ok(())
                        }
                        Err(_) => Err(PeripheralError::InvalidParameter("read failed")),
                    }
                }
                PbcEvent::DumpFlashToFile(path) => {
                    std::fs::write(path, &self.flash.data)
                        .map_err(|_| PeripheralError::InvalidParameter("write failed"))
                }
                PbcEvent::EraseSector(addr) => {
                    let base = (*addr as usize) & !0xFFF;
                    let end = (base + 4096).min(self.flash.data.len());
                    self.flash.data[base..end].fill(0xFF);
                    self.flash.dirty = true;
                    Ok(())
                }
            },
            _ => Err(PeripheralError::UnsupportedEvent),
        }
    }
}
