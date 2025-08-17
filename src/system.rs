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

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{
    /// Read status and interrupt from the chip
    pub async fn get_status(&mut self) -> Result<(Status,Intr), Lr2021Error> {
        let req = get_status_req();
        let mut rsp = StatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok((rsp.status(), rsp.intr()))
    }

    /// Read status and interrupt from the chip
    pub async fn get_errors(&mut self) -> Result<ErrorsRsp, Lr2021Error> {
        let req = get_errors_req();
        let mut rsp = ErrorsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Read status and interrupt from the chip
    pub async fn get_version(&mut self) -> Result<VersionRsp, Lr2021Error> {
        let req = get_version_req();
        let mut rsp = VersionRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Read interrupt from the chip and clear them all
    pub async fn get_and_clear_irq(&mut self) -> Result<Intr, Lr2021Error> {
        let req = get_and_clear_irq_req();
        let mut rsp = StatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.intr())
    }

    /// Set the RF channel (in Hz)
    pub async fn clear_irqs(&mut self, intr: Intr) -> Result<(), Lr2021Error> {
        let req = clear_irq_cmd(intr.value());
        self.cmd_wr(&req).await
    }

    /// Run calibration on up to 3 frequencies on 16b (MSB encode RX Path)
    /// If none, use current frequency
    pub async fn calib_fe(&mut self, freqs_4m: &[u16]) -> Result<(), Lr2021Error> {
        let f0 = freqs_4m.first().copied().unwrap_or(0);
        let f1 = freqs_4m.get(1).copied().unwrap_or(0);
        let f2 = freqs_4m.get(2).copied().unwrap_or(0);
        let req = calib_fe_cmd(f0,f1,f2);
        let len = 2 + 2*freqs_4m.len();
        self.cmd_wr(&req[..len]).await
    }

    /// Set Tx power and ramp time
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
    pub async fn set_dio_irq(&mut self, dio: u8, intr_en: Intr) -> Result<(), Lr2021Error> {
        let sleep_pull = if dio > 6 {PullDrive::PullAuto} else {PullDrive::PullUp};
        let req = set_dio_function_cmd(dio, DioFunc::Irq, sleep_pull);
        self.cmd_wr(&req).await?;
        let req = set_dio_irq_config_cmd(dio, intr_en.value());
        self.cmd_wr(&req).await
    }

    /// Write data to the TX FIFO
    /// Check number of bytes available with get_tx_fifo_lvl()
    pub async fn wr_tx_fifo(&mut self, buffer: &mut[u8]) -> Result<(), Lr2021Error> {
        self.cmd_data([0,2], buffer).await
    }

    /// Clear TX Fifo
    pub async fn clear_tx_fifo(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&clear_tx_fifo_cmd()).await
    }

    /// Return number of byte in TX FIFO
    pub async fn get_tx_fifo_lvl(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_tx_fifo_level_req();
        let mut rsp = TxFifoLevelRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.level())
    }

    /// Read data from the RX FIFO
    /// Check number of bytes available with get_rx_fifo_lvl()
    pub async fn rd_rx_fifo(&mut self, buffer: &mut[u8]) -> Result<(), Lr2021Error> {
        self.cmd_data([0,1], buffer).await
    }

    /// Return number of byte in RX FIFO
    pub async fn get_rx_fifo_lvl(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_rx_fifo_level_req();
        let mut rsp = RxFifoLevelRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.level())
    }

    /// Clear RX Fifo
    pub async fn clear_rx_fifo(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&clear_rx_fifo_cmd()).await
    }

    /// Load a patch in ram
    pub async fn load_pram(&mut self, patch: &[u8]) -> Result<(), Lr2021Error> {
        let mut req = [0;128+3];
        let mut addr = 0x801000;
        for patch_block in patch.chunks(32) {
            req[0] = ((addr>>16) & 0xFF) as u8;
            req[1] = ((addr>> 8) & 0xFF) as u8;
            req[2] = ( addr      & 0xFF) as u8;
            req[3..patch_block.len()+3].copy_from_slice(&patch_block);
            self.cmd_data([1,4], &mut req).await?;
            addr += 128;
        }
        Ok(())
    }

    /// Read a register value
    pub async fn rd_reg(&mut self, addr: u32) -> Result<u32, Lr2021Error> {
        let req = read_reg_mem32_req(addr, 1);
        let mut rsp = ReadRegMem32Rsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.value())
    }

    /// Write a register value
    pub async fn wr_reg(&mut self, addr: u32, value: u32) -> Result<(), Lr2021Error> {
        let req = write_reg_mem32_cmd(addr, value);
        self.cmd_wr(&req).await
    }

    /// Write a field value
    pub async fn wr_field(&mut self, addr: u32, value: u32, pos: u8, width: u8) -> Result<(), Lr2021Error> {
        let mask =
            if width >= 32 {0xFFFFFFFF}
            else { ((1 << width) - 1) << pos };
        let req = write_reg_mem_mask32_cmd(addr, mask, value << pos);
        self.cmd_wr(&req).await
    }

}
