use embassy_time::{Duration, Timer};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::cmd::cmd_regmem::write_reg_mem_mask32_cmd;

pub use super::cmd::cmd_common::*;
use super::{BusyPin, Lr2021, Lr2021Error};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set the RF channel (in Hz)
    pub async fn set_rf(&mut self, freq: u32) -> Result<(), Lr2021Error> {
        let req = set_rf_frequency_cmd(freq);
        self.cmd_wr(&req).await
    }

    /// Set the RX Path (LF/HF)
    pub async fn set_rx_path(&mut self, rx_path: RxPath, rx_boost: u8) -> Result<(), Lr2021Error> {
        let req = set_rx_path_adv_cmd(rx_path, rx_boost);
        self.cmd_wr(&req).await
    }

    /// Set the packet type
    pub async fn set_packet_type(&mut self, packet_type: PacketType) -> Result<(), Lr2021Error> {
        let req = set_packet_type_cmd(packet_type);
        self.cmd_wr(&req).await
    }

    /// Set Tx power and ramp time
    pub async fn set_tx_params(&mut self, tx_power: u8, ramp_time: RampTime) -> Result<(), Lr2021Error> {
        let req = set_tx_params_cmd(tx_power, ramp_time);
        self.cmd_wr(&req).await
    }

    /// Configure LF Power Amplifier
    pub async fn set_pa_lf(&mut self, pa_lf_mode: PaLfMode, pa_lf_duty_cycle: u8, pa_lf_slices: u8) -> Result<(), Lr2021Error> {
        let req = set_pa_config_cmd(PaSel::LfPa, pa_lf_mode, pa_lf_duty_cycle, pa_lf_slices);
        self.cmd_wr(&req).await
    }

    /// Configure HF Power Amplifier
    pub async fn set_pa_hf(&mut self) -> Result<(), Lr2021Error> {
        let req = set_pa_config_cmd(PaSel::HfPa, PaLfMode::LfPaFsm, 6, 7);
        self.cmd_wr(&req).await
    }

    /// Set the Fallback mode after TX/RX
    pub async fn set_fallback(&mut self, fallback_mode: FallbackMode) -> Result<(), Lr2021Error> {
        let req = set_rx_tx_fallback_mode_cmd(fallback_mode);
        self.cmd_wr(&req).await
    }

    /// Set chip in TX mode. Set timeout to 0 or to a value longer than the packet duration.
    pub async fn set_tx(&mut self, tx_timeout: u32) -> Result<(), Lr2021Error> {
        let req = set_tx_adv_cmd(tx_timeout);
        self.cmd_wr(&req).await
    }

    /// Set chip in RX mode. A timeout equal to 0 means a single reception, the value 0xFFFFFF is for continuous RX (i.e. always restart reception)
    /// and any other value, the chip will go back to its fallback mode if a reception does not occur before the timeout is elapsed
    pub async fn set_rx(&mut self, rx_timeout: u32, wait_ready: bool) -> Result<(), Lr2021Error> {
        let req = set_rx_adv_cmd(rx_timeout);
        self.cmd_wr(&req).await?;
        if wait_ready {
            self.wait_ready(Duration::from_millis(100)).await?;
        }
        Ok(())
    }

    /// Configure parameters for Channel Activity Detection
    ///  - CAD Timeout is the maximum time spent measuring RSSI in 31.25ns step
    ///  - Threshold in -dBm to determine if a signal is present
    ///  - Exit Mode: controls what happens after CAD (nothing, TX if nothing, RX if something)
    ///  - TRX Timeout is the timeout used in case the exit_mode triggers a TX or RX
    pub async fn set_cad_params(&mut self, cad_timeout: u32, threshold: u8, exit_mode: ExitMode, trx_timeout: u32) -> Result<(), Lr2021Error> {
        let req = set_cad_params_cmd(cad_timeout, threshold, exit_mode, trx_timeout);
        self.cmd_wr(&req).await
    }

    /// Set chip in Channel activity Detection mode
    /// CAD is configured with set_cad_params
    pub async fn set_cad(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&set_cad_cmd()).await
    }

    /// Set chip in CCA (Clear Channel Assesment) for duration (31.25ns)
    pub async fn set_cca(&mut self, duration: u32) -> Result<(), Lr2021Error> {
        let req = set_cca_cmd(duration);
        self.cmd_wr(&req).await
    }

    /// Set chip in CCA (Clear Channel Assesment) for duration (31.25ns)
    pub async fn get_cca_result(&mut self) -> Result<CcaResultRsp, Lr2021Error> {
        let req = get_cca_result_req();
        let mut rsp = CcaResultRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Configure the radio gain manually:
    ///   - Gain 0 enable the automatic gain selection (default setting)
    ///   - Max gain is 13
    pub async fn set_rx_gain(&mut self, gain: u8) -> Result<(), Lr2021Error> {
        let req = set_agc_gain_manual_cmd(gain.min(13));
        self.cmd_wr(&req).await
    }

    /// Clear RX stats
    pub async fn clear_rx_stats(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&reset_rx_stats_cmd()).await
    }

    /// Return length of last packet received
    pub async fn get_rx_pkt_len(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_rx_pkt_length_req();
        let mut rsp = RxPktLengthRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.pkt_length())
    }

    /// Output CRC to the FIFO even when already checked by hardware
    pub async fn force_crc_out(&mut self) -> Result<(), Lr2021Error> {
        let req = write_reg_mem_mask32_cmd(0xF30844, 0x01000000, 0);
        self.cmd_wr(&req).await
    }

    /// Measure RSSI instantaneous
    pub async fn get_rssi_inst(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_rssi_inst_req();
        let mut rsp = RssiInstRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.rssi())
    }

    /// Measure an average RSSI
    /// Note: Digital Front-End settings are changed for the time of the measurement
    pub async fn get_rssi_avg(&mut self, duration: Duration) -> Result<u16, Lr2021Error> {
        // Configure DigFE for accurate RSSI measurement
        let cfg_rssi = self.rd_reg(0xF3014C).await?;
        self.wr_reg(0xF3014C, (cfg_rssi & 0xFFFFF0FF) | (7<<3)).await?;
        Timer::after(duration).await;
        let rssi = self.get_rssi_inst().await?;
        // Restore RSSI settings
        self.wr_reg(0xF3014C, cfg_rssi).await?;
        Ok(rssi)
    }

}
