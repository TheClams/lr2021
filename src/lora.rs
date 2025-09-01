//! # API related to LoRa operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for LoRa (Long Range) communication.
//! LoRa is a Semtech proprietary modulation using chirp spread-spectrum, providing the highest sensitivity modulation
//! supported by the LR2021, ideal for communication over multiple kilometers at low bit-rate.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for LoRa operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::lora::{Sf, LoraBw, LoraCr, Ldro, HeaderType};
//!
//! // Set packet type to LoRa
//! lr2021.set_packet_type(PacketType::Lora).await.expect("Setting packet type");
//!
//! // Configure LoRa modulation (SF5, BW1000, CR 4/5, LDRO off)
//! lr2021.set_lora_modulation(
//!     Sf::Sf5,                    // Spreading Factor 5
//!     LoraBw::Bw1000,            // Bandwidth 1000 kHz
//!     LoraCr::Cr1Ham45Si,        // Coding Rate 4/5
//!     Ldro::Off                  // Low Data Rate Optimization off
//! ).await.expect("Setting LoRa modulation");
//!
//! // Configure packet parameters (8 symbols preamble, 10 byte payload, explicit header with CRC)
//! lr2021.set_lora_packet(
//!     8,                          // Preamble length (symbols)
//!     10,                         // Payload length (bytes)
//!     HeaderType::Explicit,       // Explicit header (includes payload length)
//!     true,                       // CRC enabled
//!     false                       // IQ not inverted
//! ).await.expect("Setting packet parameters");
//! ```
//!
//! ## Available Methods
//!
//! ### Core LoRa Methods
//! - [`set_lora_modulation`](Lr2021::set_lora_modulation) - Configure spreading factor, bandwidth, coding rate, and LDRO
//! - [`set_lora_packet`](Lr2021::set_lora_packet) - Set packet parameters (preamble, payload length, header type, CRC)
//! - [`set_lora_syncword`](Lr2021::set_lora_syncword) - Set syncword using legacy 1-byte format
//! - [`set_lora_syncword_ext`](Lr2021::set_lora_syncword_ext) - Set syncword using extended 2-byte format
//! - [`set_lora_synch_timeout`](Lr2021::set_lora_synch_timeout) - Configure synchronization timeout
//! - [`set_lora_address`](Lr2021::set_lora_address) - Set address filtering parameters
//!
//! ### Status and Statistics
//! - [`get_lora_packet_status`](Lr2021::get_lora_packet_status) - Get basic packet status information
//! - [`get_lora_packet_status_adv`](Lr2021::get_lora_packet_status_adv) - Get advanced packet status with SNR and frequency offset
//! - [`get_lora_rx_stats`](Lr2021::get_lora_rx_stats) - Get reception statistics
//!
//! ### Channel Activity Detection (CAD)
//! - [`set_lora_cad_params`](Lr2021::set_lora_cad_params) - Configure CAD parameters for listen-before-talk
//! - [`set_lora_cad`](Lr2021::set_lora_cad) - Start channel activity detection
//!
//! ### Advanced Features
//! - [`comp_sx127x_en`](Lr2021::comp_sx127x_en) - Enable SX127x compatibility for SF6
//! - [`set_lora_sidedet_cfg`](Lr2021::set_lora_sidedet_cfg) - Configure side-detector for multiple SF detection
//! - [`set_lora_sidedet_syncword`](Lr2021::set_lora_sidedet_syncword) - Configure side-detector syncwords
//! - [`set_lora_preamble_modulation`](Lr2021::set_lora_preamble_modulation) - Enable preamble phase modulation
//!
//! ### Ranging Operations
//! - [`set_ranging_dev_addr`](Lr2021::set_ranging_dev_addr) - Set device address for ranging
//! - [`set_ranging_req_addr`](Lr2021::set_ranging_req_addr) - Set request address for ranging
//! - [`set_ranging_txrx_delay`](Lr2021::set_ranging_txrx_delay) - Set ranging calibration delay
//! - [`set_ranging_params`](Lr2021::set_ranging_params) - Configure ranging parameters (extended/spy mode)
//! - [`get_ranging_status`](Lr2021::get_ranging_status) - Get ranging exchange status
//! - [`get_ranging_result`](Lr2021::get_ranging_result) - Get basic ranging results
//! - [`get_ranging_ext_result`](Lr2021::get_ranging_ext_result) - Get extended ranging results
//! - [`get_ranging_gain`](Lr2021::get_ranging_gain) - Get ranging gain steps (debug)
//! - [`get_ranging_stats`](Lr2021::get_ranging_stats) - Get ranging statistics
//!
//! ### Timing Synchronization
//! - [`set_lora_timing_sync`](Lr2021::set_lora_timing_sync) - Configure timing synchronization mode
//! - [`set_lora_timing_sync_pulse`](Lr2021::set_lora_timing_sync_pulse) - Configure timing sync pulse parameters

use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_lora::*;
pub use super::cmd::cmd_ranging::*;
use super::{
    cmd::cmd_regmem::write_reg_mem_mask32_cmd,
    system::set_additional_reg_to_retain_cmd,
    BusyPin, Lr2021, Lr2021Error
};

#[derive(Debug, Clone, Copy)]
pub struct SidedetCfg(u8);
impl SidedetCfg {
    pub fn new(sf: Sf, ldro: Ldro, inv: bool) -> Self{
        let b = ((sf as u8) << 4) |
            (ldro as u8) << 2 |
            if inv {1} else {0};
        Self(b)
    }

    pub fn to_byte(&self) -> u8 {
        self.0
    }
}


#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Flag status of last exchange
pub enum RangingStatus {
    Invalid = 0, Standard = 1, Extended = 3
}

impl RangingStatus {
    pub fn is_valid(&self) -> bool {
        *self!=RangingStatus::Invalid
    }
}

impl From<u8> for RangingStatus {
    fn from(value: u8) -> Self {
        match value & 3 {
            1 => RangingStatus::Standard,
            3 => RangingStatus::Extended,
            _ => RangingStatus::Invalid,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Frequency estimation during ranging exchange (valid only on responder side)
pub struct RangingFei {
    /// Frequency estimation on first exchange
    pub fei1: i32,
    /// Frequency estimation on second exchange
    pub fei2: i32,
}


#[derive(Debug, Clone, Copy)]
/// Define duration of the TimingSync pulse of the responder
pub enum TimingSyncPulseWidth {
    W1 = 0, W5 = 1, W52 = 2, W520 = 3, W5200 = 4, W52k = 5, W260k = 6, W1024k = 7
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set LoRa Modulation parameters
    #[doc(alias = "lora")]
    pub async fn set_lora_modulation(&mut self, sf: Sf, bw: LoraBw, cr: LoraCr, ldro: Ldro) -> Result<(), Lr2021Error> {
        let req = set_lora_modulation_params_cmd(sf, bw, cr, ldro);
        self.cmd_wr(&req).await
    }

    /// Set LoRa Packet parameters
    #[doc(alias = "lora")]
    pub async fn set_lora_packet(&mut self, pbl_len: u16, payload_len: u8, header_type: HeaderType, crc_en: bool, invert_iq: bool) -> Result<(), Lr2021Error> {
        let req = set_lora_packet_params_cmd(pbl_len, payload_len, header_type, crc_en, invert_iq);
        self.cmd_wr(&req).await
    }

    /// Set LoRa Syncword using legacy (SX127x) 1B notation: 0x34 for public network, 0x12 for private
    #[doc(alias = "lora")]
    pub async fn set_lora_syncword(&mut self, syncword: u8) -> Result<(), Lr2021Error> {
        let req = set_lora_syncword_cmd(syncword);
        self.cmd_wr(&req).await
    }

    /// Set LoRa Syncword, using 2B notation (2 values on 5b each)
    /// Public network is (6,8) and private network is (2,4)
    #[doc(alias = "lora")]
    pub async fn set_lora_syncword_ext(&mut self, s1: u8, s2: u8) -> Result<(), Lr2021Error> {
        let req = set_lora_syncword_extended_cmd(s1, s2);
        self.cmd_wr(&req).await
    }

    /// Set synchronisation timeout
    /// Timeout is given in number of symbol: either the direct value or with mantissa/exponent (like SX126x)
    #[doc(alias = "lora")]
    pub async fn set_lora_synch_timeout(&mut self, timeout: u8, format: TimeoutFormat) -> Result<(), Lr2021Error> {
        let req = set_lora_synch_timeout_cmd(timeout, format);
        self.cmd_wr(&req).await
    }

    /// Set address for address filtering
    /// Length is the address length in number of byte 0 (no address filtering, default) up to 8
    /// Pos is the first byte in the payload the address appears
    #[doc(alias = "lora")]
    pub async fn set_lora_address(&mut self, len: AddrLen, pos: u8, addr: u64) -> Result<(), Lr2021Error> {
        let req = set_lora_address_cmd(len, pos, addr);
        let len = 2 + (len as usize);
        self.cmd_wr(&req[..len]).await
    }

    /// Return Information about last packet received
    #[doc(alias = "lora")]
    pub async fn get_lora_packet_status(&mut self) -> Result<LoraPacketStatusRsp, Lr2021Error> {
        let req = get_lora_packet_status_req();
        let mut rsp = LoraPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return extended Information about last packet received
    #[doc(alias = "lora")]
    pub async fn get_lora_packet_status_adv(&mut self) -> Result<LoraPacketStatusRspAdv, Lr2021Error> {
        let req = get_lora_packet_status_req();
        let mut rsp = LoraPacketStatusRspAdv::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return length of last packet received
    #[doc(alias = "lora")]
    pub async fn get_lora_rx_stats(&mut self) -> Result<LoraRxStatsRsp, Lr2021Error> {
        let req = get_lora_rx_stats_req();
        let mut rsp = LoraRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Set LoRa Channel Activity Detection parameters
    /// - nb_symbols is the number of symbols for detection: between 1 and 15, use 4 for ideal performances.
    /// - pbl_any: set to false when explicitly searching for preamble, and 1 for any LoRa activity. Note that even when set to 0, CAD can still detect non-preamble, just less likely.
    /// - pnr_delta: Value between 0 and 15 to shorten the CAD time when there is obvisouly no LoRa activity. Set to 0 to always listen for the full duration, set to ~10 for optimal performances.
    ///   Higher value increase the chance to miss activity, while lower value will limit the chance to stop CAD early
    /// - exit_mode: Choose what happens after the CAD: fallback mode, RX or TX (for Listen-Before-Talk)
    /// - timeout: defines the timeout for the following RX or TX if exit mode is not CAD_ONLY
    /// - det_peak: control the detection threshold. Use None to let firmware automatically decide the threshold based on the SF/BW/nb_symbols/pnr_delta
    #[doc(alias = "lora")]
    pub async fn set_lora_cad_params(&mut self, nb_symbols: u8, pbl_any: bool, pnr_delta: u8, exit_mode: ExitMode, timeout: u32, det_peak: Option<u8>) -> Result<(), Lr2021Error> {
        let req = set_lora_cad_params_cmd(nb_symbols, pbl_any, pnr_delta, exit_mode, timeout, det_peak.unwrap_or(0));
        let req_s = if det_peak.is_none() {&req[0..8]} else {&req};
        self.cmd_wr(req_s).await
    }

    /// Start a LoRa Channel Activity Detection (CAD)
    #[doc(alias = "lora")]
    pub async fn set_lora_cad(&mut self) -> Result<(), Lr2021Error> {
        let req = set_lora_cad_cmd();
        self.cmd_wr(&req).await
    }

    const ADDR_LORA_PARAM : u32 = 0xF30A14;
    /// Enable compatibility with SX127x for SF6 communication and syncword format
    /// Must be called after each SetLoraModulation
    /// The retention enable allows to define a register slot to save this copatibility mode in retention
    #[doc(alias = "lora")]
    pub async fn comp_sx127x_en(&mut self, ret_en: Option<u8>) -> Result<(), Lr2021Error> {
        let req = write_reg_mem_mask32_cmd(Self::ADDR_LORA_PARAM, 3<<18, 1<<19);
        self.cmd_wr(&req).await?;
        if let Some(slot) = ret_en {
            let req = set_additional_reg_to_retain_cmd(slot, Self::ADDR_LORA_PARAM);
            self.cmd_wr(&req).await?;
        }
        Ok(())
    }

    #[allow(clippy::get_first)]
    /// Configure Side-Detector allowing multiple SF to be detected
    /// Must be called after set_lora_modulation
    /// If cfg is an empty slice, this disabled all side-detector
    #[doc(alias = "lora")]
    pub async fn set_lora_sidedet_cfg(&mut self, cfg: &[SidedetCfg]) -> Result<(), Lr2021Error> {
        let req = [
            0x02, 0x24,
            cfg.get(0).map(|c| c.to_byte()).unwrap_or(0),
            cfg.get(1).map(|c| c.to_byte()).unwrap_or(0),
            cfg.get(2).map(|c| c.to_byte()).unwrap_or(0),
        ];
        let len = cfg.len() + 2;
        self.cmd_wr(&req[..len]).await
    }

    #[allow(clippy::get_first)]
    /// Configure Side-Detector Syncword using basic syncword format
    #[doc(alias = "lora")]
    pub async fn set_lora_sidedet_syncword(&mut self, sw: &[u8]) -> Result<(), Lr2021Error> {
        let req = [
            0x02, 0x25,
            sw.get(0).copied().unwrap_or(0x24),
            sw.get(1).copied().unwrap_or(0x24),
            sw.get(2).copied().unwrap_or(0x24),
        ];
        let len = sw.len() + 2;
        self.cmd_wr(&req[..len]).await
    }

    /// Long preamble can be modulated in phase in order to provide information about how many symbols are left
    /// This allows a receiver to go back to sleep if beginning of the frame starts in a long time
    #[doc(alias = "lora")]
    pub async fn set_lora_preamble_modulation(&mut self, en: bool, dram_ret: u8, wakeup_time: u16, min_sleep_time: u32) -> Result<(), Lr2021Error> {
        let req = config_lora_preamble_modulation_cmd(en, dram_ret, wakeup_time, min_sleep_time);
        self.cmd_wr(&req).await
    }

    /// Set the device address for ranging operation
    /// The device will answer to ranging request only if the request address matches the device address
    /// The length allows to define how many bytes from the address are checked (starting from LSB)
    #[doc(alias = "lora")]
    pub async fn set_ranging_dev_addr(&mut self, addr: u32, length: Option<CheckLength>) -> Result<(), Lr2021Error> {
         let req = set_ranging_addr_cmd(addr, length.unwrap_or(CheckLength::Addr32b));
        self.cmd_wr(&req).await
   }

    /// Set the request address for ranging operation
    #[doc(alias = "lora")]
    pub async fn set_ranging_req_addr(&mut self, addr: u32) -> Result<(), Lr2021Error> {
         let req = set_ranging_req_addr_cmd(addr);
        self.cmd_wr(&req).await
    }

    /// Set the ranging calibration value
    #[doc(alias = "lora")]
    pub async fn set_ranging_txrx_delay(&mut self, delay: u32) -> Result<(), Lr2021Error> {
         let req = set_ranging_tx_rx_delay_cmd(delay);
        self.cmd_wr(&req).await
   }

    /// Set the ranging parameters: Extended/Spy and number of symbols
    /// Extended mode initiate a second exchange with an inverted direction to improve accuracy and provide some relative speed indication
    /// Spy mode allows to estimate distance between two device while they are performing a ranging exchange.
    /// Number of symbols should typically be between 8 to 16 symbols, with 12 being close to optimal performances
    #[doc(alias = "lora")]
    pub async fn set_ranging_params(&mut self, extended: bool, spy_mode: bool, nb_symbols: u8) -> Result<(), Lr2021Error> {
         let req = set_ranging_params_cmd(extended, spy_mode, nb_symbols);
        self.cmd_wr(&req).await
   }

    /// Return info if last ranging exchange was valid
    #[doc(alias = "lora")]
    pub async fn get_ranging_status(&mut self) -> Result<RangingStatus, Lr2021Error> {
        let val = (self.rd_reg(0xF30B74).await? >> 5) as u8;
        Ok(val.into())
    }

    /// Return the result of last ranging exchange (round-trip time of flight and RSSI)
    /// The distance is provided
    #[doc(alias = "lora")]
    pub async fn get_ranging_result(&mut self) -> Result<RangingResultRsp, Lr2021Error> {
        let req = get_ranging_result_req(RangingResKind::LatestRaw);
        let mut rsp = RangingResultRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return the result of last extended ranging exchange (round-trip time of flight and RSSI for both exchange)
    /// The round-trip time of flight can be converted in meter with the formula: rng*150/(2^12*Bandwidth)
    #[doc(alias = "lora")]
    pub async fn get_ranging_ext_result(&mut self) -> Result<RangingExtResultRsp, Lr2021Error> {
        let req = get_ranging_result_req(RangingResKind::ExtendedRaw);
        let mut rsp = RangingExtResultRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return the gain step used during ranging (the second gain step is only valid for extended ranging)
    /// This is mainly for debug, since gain can influence very slightly the results
    #[doc(alias = "lora")]
    pub async fn get_ranging_gain(&mut self) -> Result<RangingGainStepRsp, Lr2021Error> {
        let req = get_ranging_result_req(RangingResKind::GainSteps);
        let mut rsp = RangingGainStepRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return statistics about ranging exchanges
    #[doc(alias = "lora")]
    pub async fn get_ranging_stats(&mut self) -> Result<RangingStatsRsp, Lr2021Error> {
        let req = get_ranging_stats_req();
        let mut rsp = RangingStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Set Lora in Timing Synchronisation mode
    /// The initiator sends a special frame when the dio is asserted
    /// The responder is in reception mode and will assert the DIO a known delay after reception of the TimingSync packet
    #[doc(alias = "lora")]
    pub async fn set_lora_timing_sync(&mut self, mode: TimingSyncMode, dio_num: u8) -> Result<(), Lr2021Error> {
        let req = set_lora_tx_sync_cmd(mode, dio_num);
        self.cmd_wr(&req).await
    }

    /// Configure the LoRa TimingSync Pulse for the initiator (delay and width)
    #[doc(alias = "lora")]
    pub async fn set_lora_timing_sync_pulse(&mut self, delay: u32, width: TimingSyncPulseWidth) -> Result<(), Lr2021Error> {
        let value = ((width as u32) << 29) | (delay & 0x7FF_FFFF) | 0x0800_0000;
        self.wr_reg(0xF30B64, value).await
    }

}