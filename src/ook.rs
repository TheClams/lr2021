use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::{cmd::cmd_regmem::{write_reg_mem_mask32_cmd}, RxBw};

pub use super::cmd::cmd_ook::*;
use super::{BusyPin, Lr2021, Lr2021Error, PulseShape};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Modulation parameters: raw bitrate, bandwidth and pulse shaping
    pub async fn set_ook_modulation(&mut self, bitrate: u32, rx_bw: RxBw, pulse_shape: PulseShape) -> Result<(), Lr2021Error> {
        let req = set_ook_modulation_params_cmd(bitrate, pulse_shape, rx_bw);
        self.cmd_wr(&req).await
    }

    /// Set OOK packet parameter: preamble length (TX), Address filtering, header implicit/explicit, payload length, CRC and encoding
    pub async fn set_ook_packet(&mut self, pre_len_tx: u16, addr_comp: AddrComp, pkt_format: PktFormat, pld_len: u16, crc: Crc, encoding: Encoding) -> Result<(), Lr2021Error> {
        let req = set_ook_packet_params_cmd(pre_len_tx, addr_comp, pkt_format, pld_len, crc, encoding);
        self.cmd_wr(&req).await
    }

    /// Set OOK detector: Preamble (pattern/length/repetition), Sync encoding, Start of Frame delimiter
    pub async fn set_ook_detector(&mut self, preamble_pattern: u16, pattern_length: u8, pattern_num_repeats: u8, sw_is_raw: bool, sfd_kind: SfdKind, sfd_length: u8) -> Result<(), Lr2021Error> {
        let req = set_ook_detector_cmd(preamble_pattern, pattern_length, pattern_num_repeats, sw_is_raw, sfd_kind, sfd_length);
        self.cmd_wr(&req).await
    }

    /// Configure syncword (value, length and bit order)
    pub async fn set_ook_syncword(&mut self, syncword: u32, bit_order: BitOrder, nb_bits: u8) -> Result<(), Lr2021Error> {
        let req = set_ook_sync_word_cmd(syncword, bit_order, nb_bits);
        self.cmd_wr(&req).await
    }

    /// Configure OOK polynom and init value
    pub async fn set_ook_crc(&mut self, polynom: u32, init: u32) -> Result<(), Lr2021Error> {
        let req = set_ook_crc_params_cmd(polynom, init);
        self.cmd_wr(&req).await
    }

    /// Configure OOK Detection absolute threshold
    /// Typically add a few dB above the ambiant noise level
    pub async fn set_ook_thr(&mut self, threshold: i8) -> Result<(), Lr2021Error> {
        let req = write_reg_mem_mask32_cmd(0xF30E14, 0x07F00000, (threshold as u32) << 20);
        self.cmd_wr(&req).await
    }

    /// Configure OOK receiver for ADS-B:
    ///  - Modulation = 2Mb/s with 3MHz bandwidth
    ///  - Packet: Fixed payload 11B + 3B CRC with inverted manchester encoding
    ///  - Detector: Pattern 15b 0x285 and no SFD
    pub async fn set_ook_adsb(&mut self) -> Result<(), Lr2021Error>  {
        self.set_ook_modulation(2_000_000, RxBw::Bw3076, PulseShape::None).await?;
        self.set_ook_packet(8, AddrComp::Off, PktFormat::FixedLength, 11, Crc::Crc3Byte, Encoding::ManchesterInv).await?;
        self.set_ook_syncword(0, BitOrder::LsbFirst, 0).await?;
        self.set_ook_detector(0x285, 15, 0, true, SfdKind::FallingEdge, 0).await?; // TODO change true to false (sw_is_raw is unimportant)
        self.set_ook_crc(0x1FFF409, 0).await?;
        Ok(())
    }

    /// Return stats about last packet received: length, RSSI, LQI
    pub async fn get_ook_packet_status(&mut self) -> Result<OokPacketStatusRsp, Lr2021Error> {
        let req = get_ook_packet_status_req();
        let mut rsp = OokPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_ook_rx_stats(&mut self) -> Result<OokRxStatsRsp, Lr2021Error> {
        let req = get_ook_rx_stats_req();
        let mut rsp = OokRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}