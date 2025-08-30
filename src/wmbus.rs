use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_wmbus::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    // TODO: add dedicated struct. Default rxBw to auto, pbl_len to none, address filtering to true ? (add .without_filt ?) and deduce pbl_len from mode ?
    #[allow(clippy::too_many_arguments)]
    /// Set Wmbus packet parameters: preamble, Bandwidth, Payload length, Address filtering
    pub async fn set_wmbus_packet(&mut self, mode: WmbusMode, rx_bw: RxBw, pkt_format_tx: PktFormatTx, addr_filt_en: bool, pld_len: u8, pbl_len_tx: u16, pbl_len_detect: Option<u8>) -> Result<(), Lr2021Error> {
        let req = set_wmbus_params_cmd(mode, rx_bw, pkt_format_tx, addr_filt_en, pld_len, pbl_len_tx, pbl_len_detect.unwrap_or(255));
        self.cmd_wr(&req).await
    }

    /// Configure the node address for address filtering
    pub async fn set_wmbus_address(&mut self, addr: u64) -> Result<(), Lr2021Error> {
        let req = set_wmbus_address_cmd(addr);
        self.cmd_wr(&req).await
    }

    /// Return info about last packet received: length, CRC error per block, RSSI, LQI
    pub async fn get_wmbus_packet_status(&mut self) -> Result<WmbusPacketStatusRsp, Lr2021Error> {
        let req = get_wmbus_packet_status_req();
        let mut rsp = WmbusPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_wmbus_rx_stats(&mut self) -> Result<WmbusRxStatsRsp, Lr2021Error> {
        let req = get_wmbus_rx_stats_req();
        let mut rsp = WmbusRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return advanced RX stats
    pub async fn get_wmbus_rx_stats_adv(&mut self) -> Result<WmbusRxStatsRspAdv, Lr2021Error> {
        let req = get_wmbus_rx_stats_req();
        let mut rsp = WmbusRxStatsRspAdv::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}