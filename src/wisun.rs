use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_wisun::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Wisun packet parameters: preamble, Bandwidth, Payload length, Address filtering
    pub async fn set_wisun_modulation(&mut self, mode: WisunMode, rx_bw: RxBw) -> Result<(), Lr2021Error> {
        let req = set_wisun_mode_cmd(mode, rx_bw);
        self.cmd_wr(&req).await
    }

    // TODO: add dedicated struct: two function based on mode_switch, find some good default based on standard
    #[allow(clippy::too_many_arguments)]
    /// Set Wisun packet parameters: preamble, Bandwidth, Payload length, Address filtering
    pub async fn set_wisun_packet(&mut self, fcs_tx: FcsTx, whitening: Whitening, crc_on: CrcOn, mode_switch_tx: ModeSwitchTx, fec_tx: FecTx, frame_len_tx: u16, pbl_len_tx: u8, pbl_detect: Option<u8>) -> Result<(), Lr2021Error> {
        let req = set_wisun_packet_params_cmd(fcs_tx, whitening, crc_on, mode_switch_tx, fec_tx, frame_len_tx, pbl_len_tx, pbl_detect.unwrap_or(255));
        self.cmd_wr(&req).await
    }

    /// Return info about last packet received: length, CRC error per block, RSSI, LQI
    pub async fn get_wisun_packet_status(&mut self) -> Result<WisunPacketStatusRsp, Lr2021Error> {
        let req = get_wisun_packet_status_req();
        let mut rsp = WisunPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_wisun_rx_stats(&mut self) -> Result<WisunRxStatsRsp, Lr2021Error> {
        let req = get_wisun_rx_stats_req();
        let mut rsp = WisunRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return advanced RX stats
    pub async fn get_wisun_rx_stats_adv(&mut self) -> Result<WisunRxStatsRspAdv, Lr2021Error> {
        let req = get_wisun_rx_stats_req();
        let mut rsp = WisunRxStatsRspAdv::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}