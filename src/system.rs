//! # System control and chip management API
//!
//! This module provides general APIs to control the LR2021 chip, including calibration, interrupts, 
//! mode/status management, FIFO operations, and low-level register access. These are the core system 
//! functions required for chip initialization, operation, and debugging across all communication protocols.
//!
//! ## Available Methods
//!
//! ### Status and Information
//! - [`get_status`](Lr2021::get_status) - Read current chip status and interrupt flags
//! - [`get_errors`](Lr2021::get_errors) - Get detailed error information from the chip
//! - [`get_version`](Lr2021::get_version) - Get chip firmware version information
//! - [`get_and_clear_irq`](Lr2021::get_and_clear_irq) - Read interrupt flags and clear them atomically
//! - [`clear_irqs`](Lr2021::clear_irqs) - Clear specific interrupt flags
//!
//! ### Chip Mode and Power Management
//! - [`set_chip_mode`](Lr2021::set_chip_mode) - Set chip operational mode (sleep, standby, FS, TX, RX)
//!
//! ### Calibration
//! - [`calib_fe`](Lr2021::calib_fe) - Run front-end calibration on specified frequencies
//!
//! ### I/O Management
//! - [`set_dio_irq`](Lr2021::set_dio_irq) - Configure a DIO pin for interrupt generation
//! - [`set_dio_function`](Lr2021::set_dio_function) - Configure a DIO pin function
//! - [`set_dio_rf_switch`](Lr2021::set_dio_rf_switch) - Configure a DIO pin to control an RF Switch
//!
//! ### FIFO Operations
//! #### TX FIFO
//! - [`wr_tx_fifo_from`](Lr2021::wr_tx_fifo_from) - Write data to TX FIFO from external buffer
//! - [`wr_tx_fifo`](Lr2021::wr_tx_fifo) - Write data to TX FIFO from internal buffer
//! - [`get_tx_fifo_lvl`](Lr2021::get_tx_fifo_lvl) - Get number of bytes in TX FIFO
//! - [`clear_tx_fifo`](Lr2021::clear_tx_fifo) - Clear all data from TX FIFO
//!
//! #### RX FIFO  
//! - [`rd_rx_fifo_to`](Lr2021::rd_rx_fifo_to) - Read RX FIFO data to external buffer
//! - [`rd_rx_fifo`](Lr2021::rd_rx_fifo) - Read RX FIFO data to internal buffer
//! - [`get_rx_fifo_lvl`](Lr2021::get_rx_fifo_lvl) - Get number of bytes in RX FIFO
//! - [`clear_rx_fifo`](Lr2021::clear_rx_fifo) - Clear all data from RX FIFO
//!
//! ### Patch RAM Operations
//! - [`load_pram`](Lr2021::load_pram) - Load firmware patch into patch RAM
//! - [`get_pram_info`](Lr2021::get_pram_info) - Get information about loaded patch (type/version)
//!
//! ### Register and Memory Access
//! - [`rd_reg`](Lr2021::rd_reg) - Read a 32-bit register value
//! - [`wr_reg`](Lr2021::wr_reg) - Write a 32-bit register value
//! - [`wr_field`](Lr2021::wr_field) - Write to specific bit field in a register
//! - [`rd_mem`](Lr2021::rd_mem) - Read multiple 32-bit words from memory to internal buffer

use embassy_time::Duration;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::cmd::cmd_regmem::{read_reg_mem32_req, write_reg_mem32_cmd, write_reg_mem_mask32_cmd, ReadRegMem32Rsp};

use super::{BusyPin, Lr2021, Lr2021Error};
use super::status::{Intr, Status};

pub use super::cmd::cmd_system::*;
use super::radio::{set_rx_cmd, set_tx_cmd};

/// Chip Mode: Sleep/Standby/Fs/...
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChipMode {
    DeepSleep,
    Sleep(u32),
    Retention(u8,u32),
    StandbyRc,
    StandbyXosc,
    Fs,
    Tx,
    Rx,
}

const PRAM_MW_DATA   : u32 = 0x600DB002;
const PRAM_MW_ADDR   : u32 = 0x800FF8;
const PRAM_INFO_ADDR : u32 = 0x800FFC;
const PRAM_PLD_ADDR  : u32 = 0x801000;

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{
    /// Read status and interrupt from the chip
    #[doc(alias = "system")]
    pub async fn get_status(&mut self) -> Result<(Status,Intr), Lr2021Error> {
        let req = get_status_req();
        let mut rsp = StatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok((rsp.status(), rsp.intr()))
    }

    /// Read status and interrupt from the chip
    #[doc(alias = "system")]
    pub async fn get_errors(&mut self) -> Result<ErrorsRsp, Lr2021Error> {
        let req = get_errors_req();
        let mut rsp = ErrorsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Read status and interrupt from the chip
    #[doc(alias = "system")]
    pub async fn get_version(&mut self) -> Result<VersionRsp, Lr2021Error> {
        let req = get_version_req();
        let mut rsp = VersionRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Read interrupt from the chip and clear them all
    #[doc(alias = "system")]
    pub async fn get_and_clear_irq(&mut self) -> Result<Intr, Lr2021Error> {
        let req = get_and_clear_irq_req();
        let mut rsp = StatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.intr())
    }

    /// Set the RF channel (in Hz)
    #[doc(alias = "system")]
    pub async fn clear_irqs(&mut self, intr: Intr) -> Result<(), Lr2021Error> {
        let req = clear_irq_cmd(intr.value());
        self.cmd_wr(&req).await
    }

    /// Run calibration on up to 3 frequencies on 16b (MSB encode RX Path)
    /// If none, use current frequency
    #[doc(alias = "system")]
    pub async fn calib_fe(&mut self, freqs_4m: &[u16]) -> Result<(), Lr2021Error> {
        let f0 = freqs_4m.first().copied().unwrap_or(0);
        let f1 = freqs_4m.get(1).copied().unwrap_or(0);
        let f2 = freqs_4m.get(2).copied().unwrap_or(0);
        let req = calib_fe_cmd(f0,f1,f2);
        let len = 2 + 2*freqs_4m.len();
        self.cmd_wr(&req[..len]).await
    }

    /// Set Tx power and ramp time
    #[doc(alias = "system")]
    pub async fn set_chip_mode(&mut self, chip_mode: ChipMode) -> Result<(), Lr2021Error> {
        match chip_mode {
            ChipMode::DeepSleep      => self.cmd_wr(&set_sleep_cmd(false, 0)).await,
            ChipMode::Sleep(t)       => self.cmd_wr(&set_sleep_adv_cmd(true, 0, t)).await,
            ChipMode::Retention(r,t) => self.cmd_wr(&set_sleep_adv_cmd(true, r, t)).await,
            ChipMode::StandbyRc   => self.cmd_wr(&set_standby_cmd(StandbyMode::Rc)).await,
            ChipMode::StandbyXosc => self.cmd_wr(&set_standby_cmd(StandbyMode::Xosc)).await,
            ChipMode::Fs => self.cmd_wr(&set_fs_cmd()).await,
            ChipMode::Tx => self.cmd_wr(&set_tx_cmd()).await,
            ChipMode::Rx => self.cmd_wr(&set_rx_cmd()).await,
        }
    }

    /// Configure a pin as IRQ and enable interrupts for this pin
    #[doc(alias = "system")]
    pub async fn set_dio_function(&mut self, dio: DioNum, func: DioFunc, pull_drive: PullDrive) -> Result<(), Lr2021Error> {
        let req = set_dio_function_cmd(dio, func, pull_drive);
        self.cmd_wr(&req).await
    }

    /// Configure a pin as An RF Switch
    /// Each args flags when the IO should be high
    #[doc(alias = "system")]
    pub async fn set_dio_rf_switch(&mut self, dio_num: DioNum, tx_hf: bool, rx_hf: bool, tx_lf: bool, rx_lf: bool, standby: bool) -> Result<(), Lr2021Error> {
        let req = set_dio_rf_switch_config_cmd(dio_num, tx_hf, rx_hf, tx_lf, rx_lf, standby);
        self.cmd_wr(&req).await
    }

    /// Configure a pin as IRQ and enable interrupts for this pin
    #[doc(alias = "system")]
    pub async fn set_dio_irq(&mut self, dio: DioNum, intr_en: Intr) -> Result<(), Lr2021Error> {
        let sleep_pull = if dio==DioNum::Dio5 || dio==DioNum::Dio6 {PullDrive::PullAuto} else {PullDrive::PullUp};
        let req = set_dio_function_cmd(dio, DioFunc::Irq, sleep_pull);
        self.cmd_wr(&req).await?;
        let req = set_dio_irq_config_cmd(dio, intr_en.value());
        self.cmd_wr(&req).await
    }

    /// Write data to the TX FIFO
    /// Check number of bytes available with get_tx_fifo_lvl()
    #[doc(alias = "system")]
    pub async fn wr_tx_fifo_from(&mut self, buffer: &[u8]) -> Result<(), Lr2021Error> {
        self.cmd_data_wr(&[0,2], buffer).await
    }

    /// Write data to the TX FIFO
    /// Check number of bytes available with get_tx_fifo_lvl()
    #[doc(alias = "system")]
    pub async fn wr_tx_fifo(&mut self, len: usize) -> Result<(), Lr2021Error> {
        self.cmd_wr_begin(&[0,2]).await?;
        self.spi
            .transfer_in_place(&mut self.buffer.data_mut()[..len]).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Clear TX Fifo
    #[doc(alias = "system")]
    pub async fn clear_tx_fifo(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&clear_tx_fifo_cmd()).await
    }

    /// Return number of byte in TX FIFO
    #[doc(alias = "system")]
    pub async fn get_tx_fifo_lvl(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_tx_fifo_level_req();
        let mut rsp = TxFifoLevelRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.level())
    }

    /// Read data from the RX FIFO
    #[doc(alias = "system")]
    pub async fn rd_rx_fifo_to(&mut self, buffer: &mut[u8]) -> Result<(), Lr2021Error> {
        self.cmd_data_rw(&[0,1], buffer).await
    }

    /// Read data from the RX FIFO to the local buffer
    #[doc(alias = "system")]
    pub async fn rd_rx_fifo(&mut self, len: usize) -> Result<(), Lr2021Error> {
        self.cmd_wr_begin(&[0,1]).await?;
        self.spi
            .transfer_in_place(&mut self.buffer.data_mut()[..len]).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Return number of byte in RX FIFO
    #[doc(alias = "system")]
    pub async fn get_rx_fifo_lvl(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_rx_fifo_level_req();
        let mut rsp = RxFifoLevelRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.level())
    }

    /// Clear RX Fifo
    #[doc(alias = "system")]
    pub async fn clear_rx_fifo(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&clear_rx_fifo_cmd()).await
    }

    /// Load a patch in ram
    #[doc(alias = "system")]
    pub async fn load_pram(&mut self, patch: &[u8]) -> Result<(), Lr2021Error> {
        let mut addr = PRAM_PLD_ADDR;
        for patch_block in patch.chunks(32) {
            let len = patch_block.len()+5;
            self.buffer_mut()[0] = 1;
            self.buffer_mut()[1] = 4;
            self.buffer_mut()[2] = ((addr>>16) & 0xFF) as u8;
            self.buffer_mut()[3] = ((addr>> 8) & 0xFF) as u8;
            self.buffer_mut()[4] = ( addr      & 0xFF) as u8;
            self.buffer_mut()[5..len].copy_from_slice(patch_block);
            self.cmd_buf_wr(len).await?;
            addr += 128;
        }
        Ok(())
    }

    /// Return type/version of the Patch Ram if loaded. None if no Patch Ram available
    #[doc(alias = "system")]
    pub async fn get_pram_info(&mut self) -> Result<Option<(u8,u8)>, Lr2021Error> {
        // Check the Magic Word has been written, indicating a PAM was loaded
        let pram_mw = self.rd_reg(PRAM_MW_ADDR).await?;
        if pram_mw == PRAM_MW_DATA {
            let pram_info = self.rd_reg(PRAM_INFO_ADDR).await?;
            let pram_version = (pram_info & 0xFF) as u8;
            let pram_kind    = ((pram_info >> 8) & 0xFF) as u8;
            Ok(Some((pram_version, pram_kind)))
        } else {
            Ok(None)
        }
    }

    /// Read a register value
    #[doc(alias = "system")]
    pub async fn rd_reg(&mut self, addr: u32) -> Result<u32, Lr2021Error> {
        let req = read_reg_mem32_req(addr, 1);
        let mut rsp = ReadRegMem32Rsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.value())
    }

    /// Read nb32 qword (max 40) from memory and save them inside local buffer
    #[doc(alias = "system")]
    pub async fn rd_mem(&mut self, addr: u32, nb32: u8) -> Result<(), Lr2021Error> {
        if nb32 > 40 {
            return Err(Lr2021Error::CmdErr);
        }
        let req = read_reg_mem32_req(addr, nb32);
        self.cmd_wr(&req).await?;
        self.wait_ready(Duration::from_millis(1)).await?;
        self.nss.set_low().map_err(|_| Lr2021Error::Pin)?;
        self.buffer.nop();
        let rsp_buf = &mut self.buffer.0[..4*nb32 as usize];
        self.spi
            .transfer_in_place(rsp_buf).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)?;
        self.buffer.cmd_status().check()
    }

    /// Write a register value
    #[doc(alias = "system")]
    pub async fn wr_reg(&mut self, addr: u32, value: u32) -> Result<(), Lr2021Error> {
        let req = write_reg_mem32_cmd(addr, value);
        self.cmd_wr(&req).await
    }

    /// Write a field value
    #[doc(alias = "system")]
    pub async fn wr_field(&mut self, addr: u32, value: u32, pos: u8, width: u8) -> Result<(), Lr2021Error> {
        let mask =
            if width >= 32 {0xFFFFFFFF}
            else { ((1 << width) - 1) << pos };
        let req = write_reg_mem_mask32_cmd(addr, mask, value << pos);
        self.cmd_wr(&req).await
    }

}
