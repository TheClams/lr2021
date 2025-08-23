use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_zigbee::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Zigbee packet parameters: preamble, Bandwidth, Payload length, Address filtering, FCS handling (software/Hardware)
    pub async fn set_zigbee_packet(&mut self, mode: ZigbeeMode, rx_bw: RxBw, pld_len: u8, pbl_len_tx: u16, addr_filt_en: bool, fcs_mode: FcsMode) -> Result<(), Lr2021Error> {
        let req = set_zigbee_params_cmd(mode, rx_bw, pld_len, pbl_len_tx, addr_filt_en, fcs_mode);
        self.cmd_wr(&req).await
    }

    /// Sets the zigbee packet length without calling set_zigbee_packet which takes longer
    /// The function set_zigbee_packet must have been called once before !
    pub async fn set_zigbee_packet_len(&mut self, pld_len: u8) -> Result<(), Lr2021Error> {
        let req = set_zigbee_packet_len_cmd(pld_len);
        self.cmd_wr(&req).await
    }

    /// Configure the different Zigbee addresses for filtering in RX.
    /// Frames that don't match the addresses raise an address error IRQ and reception is aborted.
    /// When a packet is received, the destination address and PAN ID are both checked.
    /// Multi-cast is not supported or filtered and must be handled by the host
    pub async fn set_zigbee_address(&mut self, long_dest_addr: u64, short_dest_addr: u16, pan_id: u16, trans_id: u8) -> Result<(), Lr2021Error> {
        let req = set_zigbee_address_cmd(long_dest_addr, short_dest_addr, pan_id, trans_id);
        self.cmd_wr(&req).await
    }

    /// Return length of last packet received
    pub async fn get_zigbee_packet_status(&mut self) -> Result<ZigbeePacketStatusRsp, Lr2021Error> {
        let req = get_zigbee_packet_status_req();
        let mut rsp = ZigbeePacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_zigbee_rx_stats(&mut self) -> Result<ZigbeeRxStatsRsp, Lr2021Error> {
        let req = get_zigbee_rx_stats_req();
        let mut rsp = ZigbeeRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return advanced RX stats
    pub async fn get_zigbee_rx_stats_adv(&mut self) -> Result<ZigbeeRxStatsRspAdv, Lr2021Error> {
        let req = get_zigbee_rx_stats_req();
        let mut rsp = ZigbeeRxStatsRspAdv::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}