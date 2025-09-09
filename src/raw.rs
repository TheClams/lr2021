//! # API related to Raw IQ Operation
//!
//! This module provides an API to capture or send IQ samples.
//! This allows to handle low data-rate custom modulation.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to run a data capture:
//!
//! ```rust,no_run
//! ```
//!
//! ## Available Methods
//! - [`set_iq_capture_fifo`](Lr2021::set_iq_capture_fifo) - Configure IQ Capture feature to save sampled to the RX FIFO
//! - [`set_iq_capture_ram`](Lr2021::set_iq_capture_ram) - Configure IQ Capture feature to save sampled in local memory
//! - [`set_iq_capture_trigger`](Lr2021::set_iq_capture_trigger) - Configure trigger to start and stop capture
//! - [`get_iq_capture_ram_cnt`](Lr2021::get_iq_capture_ram_cnt) - Return number of sample captured
//! - [`get_iq_samples`](Lr2021::get_iq_samples) - Read nb bytes captures in the memory (maximum 255 by read)
//! - [`set_iq_tx_params`](Lr2021::set_iq_tx_params) - Set the Raw IQ format: number of sample, sample rate and mode (IQ, Frequency or phase)
//!

use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_raw::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Configure IQ Capture feature to save sampled to the RX FIFO
    pub async fn set_iq_capture_fifo(&mut self, sample_rate: u32, rx_bw: RxBw, data_sel: CaptureDataSel) -> Result<(), Lr2021Error> {
        let req = set_raw_iq_capture_params_cmd(rx_bw, sample_rate, RamSel::None, CaptureMode::Fifo, data_sel);
        self.cmd_wr(&req).await
    }

    /// Configure IQ Capture feature to save sampled in local memory
    /// The max size correspond to 16kB. Set to false when apatch ram is used
    pub async fn set_iq_capture_ram(&mut self, sample_rate: u32, rx_bw: RxBw, data_sel: CaptureDataSel, max_size: bool) -> Result<(), Lr2021Error> {
        // Keep things simple in term of memory selection: using RAM1 only adds 4kB and will likely be required for other use
        let ram_sel = if max_size {RamSel::Ram2p3} else {RamSel::Ram3};
        let req = set_raw_iq_capture_params_cmd(rx_bw, sample_rate, ram_sel, CaptureMode::Ddmi, data_sel);
        self.cmd_wr(&req).await
    }

    /// Configure trigger to start and stop capture
    pub async fn set_iq_capture_trigger(&mut self, start: CaptureTrigger, stop: CaptureTrigger) -> Result<(), Lr2021Error> {
        let req = set_raw_iq_trigger_cmd(start, stop);
        self.cmd_wr(&req).await
    }

    /// Return number of sample captured inside the RAM
    pub async fn get_iq_capture_ram_cnt(&mut self) -> Result<RawIqDdmiCntRsp, Lr2021Error> {
        let req = get_raw_iq_ddmi_cnt_req();
        let mut rsp = RawIqDdmiCntRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Read nb bytes captures in the memory (maximum 255 by read)
    /// If data is 16b format is LittleEndian (i.e [7:0],[15:8] )
    pub async fn get_iq_samples(&mut self, offset: u16, nb: u8, buffer: &mut[u8]) -> Result<(), Lr2021Error> {
        let req = get_raw_iq_ddmi_data_req(offset, nb);
        self.cmd_data_rw(&req, buffer).await
    }

    /// Set the Raw IQ format: number of sample, sample rate and mode (IQ, Frequency or phase)
    pub async fn set_iq_tx_params(&mut self, nb_samples: u16, sample_rate: u32, mode: TxIqMode) -> Result<(), Lr2021Error> {
        let req = set_raw_iq_tx_params_cmd(nb_samples, sample_rate, mode);
        self.cmd_wr(&req).await
    }

}