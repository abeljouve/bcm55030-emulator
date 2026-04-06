/// Peripheral Bus Controller (SPI + MDIO)
/// Base address: 0x010001F0
/// Handles SPI flash FIFO commands, DMA transfers, and MDIO PHY access.

use super::spi_flash::SpiFlash;

/// DMA physical base for DCCM (the DMA engine sees DCCM at this address)
const DMA_DCCM_BASE: u32 = 0xFFF80000;
const DMA_DCCM_SIZE: u32 = 0x80000; // 512KB

/// Register offsets from base (0x010001F0)
const REG_SPI_CONTROL: u32 = 0x10;     // RW: speed[2:0], bit 6 = CS mode
const REG_SPI_STATUS: u32 = 0x1C;      // R: bit 0 = busy; W: triggers FIFO command
const REG_SPI_FIFO_DATA: u32 = 0x20;   // RW: FIFO TX/RX data (word 0)
const REG_SPI_FIFO_DATA1: u32 = 0x24;  // RW: FIFO data (word 1, bytes 4-7)
const REG_SPI_READ_DATA: u32 = 0x2C;   // R: FIFO result (word 0)
const REG_SPI_READ_DATA1: u32 = 0x30;  // R: FIFO result (word 1)
const REG_SPI_CONFIG: u32 = 0x34;      // W: SPI configuration
const REG_DMA_CTRL: u32 = 0x38;        // RW: bit 0 = busy/enable; W: triggers DMA
const REG_DMA_ADDR: u32 = 0x3C;        // W: flash address for DMA
const REG_DMA_DATA_ADDR: u32 = 0x40;   // W: memory address for DMA

/// Pending DMA write to DCCM
pub struct DmaWrite {
    pub dccm_addr: u32,
    pub data: Vec<u8>,
}

pub struct PeripheralBusController {
    pub flash: SpiFlash,
    pub trace: bool,

    // SPI registers
    spi_control: u32,
    spi_config: u32,
    spi_fifo: [u32; 2],        // TX FIFO (2 words = 8 bytes max)
    spi_rx: [u32; 2],          // RX buffer (2 words)

    // DMA registers
    dma_ctrl: u32,
    dma_flash_addr: u32,
    dma_data_addr: u32,

    // Pending DMA writes to be applied to DCCM
    pending_dma: Vec<DmaWrite>,
}

impl PeripheralBusController {
    pub fn new() -> Self {
        Self {
            flash: SpiFlash::new(),
            trace: false,
            spi_control: 0,
            spi_config: 0,
            spi_fifo: [0; 2],
            spi_rx: [0; 2],
            dma_ctrl: 0,
            dma_flash_addr: 0,
            dma_data_addr: 0,
            pending_dma: Vec::new(),
        }
    }

    pub fn read_word(&mut self, offset: u32) -> u32 {
        match offset {
            REG_SPI_CONTROL => self.spi_control,
            REG_SPI_STATUS => {
                // Bit 0 = busy — always 0 (operations complete instantly)
                0
            }
            REG_SPI_FIFO_DATA => self.spi_fifo[0],
            REG_SPI_FIFO_DATA1 => self.spi_fifo[1],
            REG_SPI_READ_DATA => self.spi_rx[0],
            REG_SPI_READ_DATA1 => self.spi_rx[1],
            REG_SPI_CONFIG => self.spi_config,
            REG_DMA_CTRL => {
                // Bit 0 = busy — always 0 (DMA completes instantly)
                0
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

    pub fn write_word(&mut self, offset: u32, val: u32) {
        match offset {
            REG_SPI_CONTROL => {
                self.spi_control = val;
            }
            REG_SPI_STATUS => {
                // Writing triggers a FIFO SPI command
                if val & 1 != 0 {
                    self.execute_fifo_command(val);
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
                    self.execute_dma(val);
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

    /// Drain pending DMA writes (to be applied to DCCM by the memory subsystem)
    pub fn take_pending_dma(&mut self) -> Vec<DmaWrite> {
        std::mem::take(&mut self.pending_dma)
    }

    /// Execute a FIFO-based SPI command
    fn execute_fifo_command(&mut self, cmd_word: u32) {
        let tx_len = ((cmd_word >> 4) & 0xF) as usize;
        let rx_len = ((cmd_word >> 8) & 0xFF) as usize;

        // Extract TX bytes from FIFO (little-endian byte order within words)
        let mut tx = Vec::with_capacity(tx_len);
        for i in 0..tx_len {
            let word_idx = i / 4;
            let byte_idx = i % 4;
            let byte = (self.spi_fifo[word_idx] >> (byte_idx * 8)) as u8;
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

        // Execute on flash
        let rx = self.flash.execute_fifo_command(&tx, rx_len);

        // Store result in RX buffer (little-endian byte order within words)
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

    /// Execute a DMA transfer (always reads from flash to memory)
    fn execute_dma(&mut self, ctrl: u32) {
        let length = ((ctrl >> 4) & 0xFFF) as usize;
        let flash_addr = self.dma_flash_addr;
        let mem_addr = self.dma_data_addr;

        if self.trace {
            eprintln!(
                "[PBC] SPI DMA: READ flash=0x{:06X} → mem=0x{:08X} len={}",
                flash_addr, mem_addr, length
            );
        }

        let data = self.flash.dma_read(flash_addr, length);

        // Translate DMA address to DCCM offset
        if mem_addr >= DMA_DCCM_BASE && mem_addr.wrapping_sub(DMA_DCCM_BASE) < DMA_DCCM_SIZE {
            let dccm_addr = mem_addr - DMA_DCCM_BASE;
            self.pending_dma.push(DmaWrite {
                dccm_addr,
                data,
            });
        } else if (mem_addr as usize) < 0x80000 {
            self.pending_dma.push(DmaWrite {
                dccm_addr: mem_addr,
                data,
            });
        } else if self.trace {
            eprintln!(
                "[PBC] DMA: unmapped target address 0x{:08X}",
                mem_addr
            );
        }
    }
}
