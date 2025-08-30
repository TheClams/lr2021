use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_fsk::*;
use super::{BusyPin, Lr2021, Lr2021Error};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Modulation parameters: raw bitrate, pulse shaping, Bandwidth and fdev
    pub async fn set_fsk_modulation(&mut self, bitrate: u32, pulse_shape: PulseShape, rx_bw: RxBw, fdev: u32) -> Result<(), Lr2021Error> {
        let req = set_fsk_modulation_params_cmd(bitrate, pulse_shape, rx_bw, fdev);
        self.cmd_wr(&req).await
    }

    // TODO: add dedicated struct and find a good default set of values
    #[allow(clippy::too_many_arguments)]
    /// Set FLRC packet parameters: preamble, syncword, header implicit/explicit, CRC and packet length (max 511)
    pub async fn set_fsk_packet(&mut self, pbl_len_tx: u16, pbl_len_detect: PblLenDetect, pbl_long: bool, pld_len_unit: PldLenUnit, addr_comp: AddrComp, fsk_pkt_format: FskPktFormat, pld_len: u16, crc: Crc, dc_free: bool) -> Result<(), Lr2021Error> {
        let req = set_fsk_packet_params_cmd(pbl_len_tx, pbl_len_detect, pbl_long, pld_len_unit, addr_comp, fsk_pkt_format, pld_len, crc, dc_free);
        self.cmd_wr(&req).await
    }

    /// Configure syncword
    pub async fn set_fsk_syncword(&mut self, syncword: u64, bit_order: BitOrder, nb_bits: u8) -> Result<(), Lr2021Error> {
        let req = set_fsk_sync_word_cmd(syncword, bit_order, nb_bits);
        self.cmd_wr(&req).await
    }

    /// Return length of last packet received
    pub async fn get_fsk_packet_status(&mut self) -> Result<FskPacketStatusRsp, Lr2021Error> {
        let req = get_fsk_packet_status_req();
        let mut rsp = FskPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_fsk_rx_stats(&mut self) -> Result<FskRxStatsRsp, Lr2021Error> {
        let req = get_fsk_rx_stats_req();
        let mut rsp = FskRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}