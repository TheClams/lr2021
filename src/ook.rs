//! # API related to OOK operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for OOK (On-Off Keying) modulation.
//! The modem is quite generic and handles many different protocols with configurable parameters
//! for modulation, packet structure, detection, and encoding.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for OOK operations (ADS-B example):
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::ook::{AddrComp, PktFormat, Crc, Encoding, BitOrder, SfdKind};
//! use lr2021::{RxBw, PulseShape};
//!
//! // Set packet type to OOK
//! lr2021.set_packet_type(PacketType::Ook).await.expect("Setting packet type");
//!
//! // Configure OOK modulation (2Mbps, 3MHz bandwidth, no pulse shaping)
//! lr2021.set_ook_modulation(
//!     2_000_000,              // Bitrate: 2 Mbps
//!     RxBw::Bw3076,          // RX bandwidth: 3.076 MHz
//!     PulseShape::None       // No pulse shaping
//! ).await.expect("Setting OOK modulation");
//!
//! // Configure packet parameters (ADS-B: fixed 11-byte payload + 3-byte CRC)
//! lr2021.set_ook_packet(
//!     8,                      // TX preamble length
//!     AddrComp::Off,         // No address filtering
//!     PktFormat::FixedLength, // Fixed length packets
//!     11,                     // Payload length: 11 bytes
//!     Crc::Crc3Byte,         // 3-byte CRC
//!     Encoding::ManchesterInv // Inverted Manchester encoding
//! ).await.expect("Setting packet parameters");
//!
//! // Configure detector (ADS-B specific pattern)
//! lr2021.set_ook_detector(
//!     0x285,                  // Preamble pattern
//!     15,                     // Pattern length: 15 bits
//!     0,                      // No pattern repetition
//!     false,                  // Sync word is not raw
//!     SfdKind::FallingEdge,   // Start Frame Delimiter on falling edge
//!     0                       // SFD length
//! ).await.expect("Setting OOK detector");
//!
//! // Alternative: Use pre-configured ADS-B setup
//! lr2021.set_ook_adsb().await.expect("Setting up for ADS-B");
//! ```
//!
//! ## Available Methods
//!
//! ### Core Configuration
//! - [`set_ook_modulation`](Lr2021::set_ook_modulation) - Configure bitrate, bandwidth, and pulse shaping
//! - [`set_ook_packet`](Lr2021::set_ook_packet) - Set packet parameters (length, CRC, encoding, addressing)
//! - [`set_ook_detector`](Lr2021::set_ook_detector) - Configure preamble detection and start frame delimiter
//! - [`set_ook_syncword`](Lr2021::set_ook_syncword) - Configure synchronization word (value, length, bit order)
//! - [`set_ook_crc`](Lr2021::set_ook_crc) - Configure CRC polynomial and initialization value
//! - [`set_ook_thr`](Lr2021::set_ook_thr) - Set detection threshold above noise level
//!
//! ### Pre-configured Protocols
//! - [`set_ook_adsb`](Lr2021::set_ook_adsb) - Configure for ADS-B protocol (2Mbps, Manchester encoding)
//!
//! ### Status and Statistics
//! - [`get_ook_packet_status`](Lr2021::get_ook_packet_status) - Get packet status (length, RSSI, LQI)
//! - [`get_ook_rx_stats`](Lr2021::get_ook_rx_stats) - Get reception statistics

use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::{cmd::cmd_regmem::write_reg_mem_mask32_cmd, radio::PacketType, RxBw};

pub use super::cmd::cmd_ook::*;
use super::{BusyPin, Lr2021, Lr2021Error, PulseShape};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Modulation parameters: raw bitrate, bandwidth and pulse shaping
    #[doc(alias = "ook")]
    pub async fn set_ook_modulation(&mut self, bitrate: u32, rx_bw: RxBw, pulse_shape: PulseShape) -> Result<(), Lr2021Error> {
        let req = set_ook_modulation_params_cmd(bitrate, pulse_shape, rx_bw);
        self.cmd_wr(&req).await
    }

    /// Set OOK packet parameter: preamble length (TX), Address filtering, header implicit/explicit, payload length, CRC and encoding
    #[doc(alias = "ook")]
    pub async fn set_ook_packet(&mut self, pre_len_tx: u16, addr_comp: AddrComp, pkt_format: PktFormat, pld_len: u16, crc: Crc, encoding: Encoding) -> Result<(), Lr2021Error> {
        let req = set_ook_packet_params_cmd(pre_len_tx, addr_comp, pkt_format, pld_len, crc, encoding);
        self.cmd_wr(&req).await
    }

    /// Set OOK detector: Preamble (pattern/length/repetition), Sync encoding, Start of Frame delimiter
    #[doc(alias = "ook")]
    pub async fn set_ook_detector(&mut self, preamble_pattern: u16, pattern_length: u8, pattern_num_repeats: u8, sw_is_raw: bool, sfd_kind: SfdKind, sfd_length: u8) -> Result<(), Lr2021Error> {
        let req = set_ook_detector_cmd(preamble_pattern, pattern_length, pattern_num_repeats, sw_is_raw, sfd_kind, sfd_length);
        self.cmd_wr(&req).await
    }

    /// Configure syncword (value, length and bit order)
    #[doc(alias = "ook")]
    pub async fn set_ook_syncword(&mut self, syncword: u32, bit_order: BitOrder, nb_bits: u8) -> Result<(), Lr2021Error> {
        let req = set_ook_sync_word_cmd(syncword, bit_order, nb_bits);
        self.cmd_wr(&req).await
    }

    /// Configure OOK polynom and init value
    #[doc(alias = "ook")]
    pub async fn set_ook_crc(&mut self, polynom: u32, init: u32) -> Result<(), Lr2021Error> {
        let req = set_ook_crc_params_cmd(polynom, init);
        self.cmd_wr(&req).await
    }

    /// Configure OOK Detection absolute threshold
    /// Typically add a few dB above the ambiant noise level
    #[doc(alias = "ook")]
    pub async fn set_ook_thr(&mut self, threshold: i8) -> Result<(), Lr2021Error> {
        let req = write_reg_mem_mask32_cmd(0xF30E14, 0x07F00000, (threshold as u32) << 20);
        self.cmd_wr(&req).await
    }

    /// Configure OOK receiver for ADS-B:
    ///  - Modulation = 2Mb/s with 3MHz bandwidth
    ///  - Packet: Fixed payload 11B + 3B CRC with inverted manchester encoding
    ///  - Detector: Pattern 15b 0x285 and no SFD
    #[doc(alias = "ook")]
    pub async fn set_ook_adsb(&mut self) -> Result<(), Lr2021Error>  {
        self.set_packet_type(PacketType::Ook).await.expect("Setting packet type to BLE");
        self.set_ook_modulation(2_000_000, RxBw::Bw3076, PulseShape::None).await?;
        self.set_ook_packet(8, AddrComp::Off, PktFormat::FixedLength, 11, Crc::Crc3Byte, Encoding::ManchesterInv).await?;
        self.set_ook_syncword(0, BitOrder::LsbFirst, 0).await?;
        self.set_ook_detector(0x285, 15, 0, false, SfdKind::FallingEdge, 0).await?;
        self.set_ook_crc(0x1FFF409, 0).await?;
        Ok(())
    }

    /// Return stats about last packet received: length, RSSI, LQI
    #[doc(alias = "ook")]
    pub async fn get_ook_packet_status(&mut self) -> Result<OokPacketStatusRsp, Lr2021Error> {
        let req = get_ook_packet_status_req();
        let mut rsp = OokPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    #[doc(alias = "ook")]
    pub async fn get_ook_rx_stats(&mut self) -> Result<OokRxStatsRsp, Lr2021Error> {
        let req = get_ook_rx_stats_req();
        let mut rsp = OokRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}