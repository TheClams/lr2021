//! # API related to BLE operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for Bluetooth Low Energy (BLE) communication.
//! It includes methods for setting modulation parameters, configuring BLE channels, transmitting packets, and retrieving statistics.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for BLE operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::ble::{BleMode, ChannelType};
//!
//! // Set packet type to BLE
//! lr2021.set_packet_type(PacketType::Ble).await.expect("Setting packet type to BLE");
//!
//! // Configure BLE modulation (1Mb/s)
//! lr2021.set_ble_modulation(BleMode::Le1mb).await.expect("Setting BLE mode (1Mb/s)");
//!
//! // Configure BLE parameters (advertising channel)
//! lr2021.set_ble_params(
//!     false,                    // CRC not in FIFO
//!     ChannelType::Advertiser,  // Channel type
//!     0x53,                     // Whitening init (for channel 37)
//!     0x555555,                 // CRC init
//!     0x8e89bed6               // Sync word/Access code
//! ).await.expect("Set BLE params");
//! ```
//!
//! ## Available Methods
//!
//! ### Core BLE Methods
//! - [`set_ble_modulation`](Lr2021::set_ble_modulation) - Configure BLE modulation mode (1M, 2M, 500k, 125k)
//! - [`set_ble_params`](Lr2021::set_ble_params) - Set BLE channel parameters (whitening, CRC, sync word)
//! - [`set_ble_tx`](Lr2021::set_ble_tx) - Set PDU length and transmit packet
//! - [`set_ble_tx_pdu_len`](Lr2021::set_ble_tx_pdu_len) - Set PDU length for pin-triggered transmission
//!
//! ### Status and Statistics
//! - [`get_ble_packet_status`](Lr2021::get_ble_packet_status) - Get status of last received packet
//! - [`get_ble_rx_stats`](Lr2021::get_ble_rx_stats) - Get basic reception statistics
//! - [`get_ble_rx_stats_adv`](Lr2021::get_ble_rx_stats_adv) - Get advanced reception statistics
//!
//! ### BLE Direction finding Methods
//! - [`get_ble_cte_info'](Lr2021::get_ble_cte_info) - Return information on Constant Tone Extension
//! - [`get_ble_cte_all_samples'](Lr2021::get_ble_cte_all_samples) - Read all CTE samples
//! - [`get_ble_cte_next_sample'](Lr2021::get_ble_cte_next_sample) - Retrieve the next CTE complex sample stored
//!
use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_ble::*;
use super::{BusyPin, Lr2021, Lr2021Error};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Sampling period of Angle of Arrival data
pub enum AoaSampling {
    Cte1us = 0, Cte2us = 1
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Constant Tone Extension kind from last received header
pub enum CteKind {
    AoA = 0, AoD1us = 1, AoD2us = 2
}

impl From<u8> for CteKind {
    fn from(value: u8) -> Self {
        match value & 3 {
            2 => CteKind::AoD2us,
            1 => CteKind::AoD1us,
            _ => CteKind::AoA,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Info on constant tone extension from last received packet
pub struct CteInfo {
    /// Number of CTE sample stored
    pub nb_sample: u8,
    /// Kind of CTE (extracted from header)
    pub kind: CteKind,
}

const ADDR_CTE_CTRL : u32 = 0xF30C40;
const ADDR_CTE_SMPL : u32 = 0xF30C44;


impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set BLE Mode (1M, 2M, 500k, 125k)
    pub async fn set_ble_modulation(&mut self, mode: BleMode) -> Result<(), Lr2021Error> {
        let req = set_ble_modulation_params_cmd(mode);
        self.cmd_wr(&req).await?;
        // Need extra configuration for coded mode
        if matches!(mode, BleMode::LeCoded500k|BleMode::LeCoded125k) {
            self.cmd_wr(&[0x02,0x30,0x01,0x20,0x00,0x09,0x00]).await?;
        }
        Ok(())
    }

    /// Set BLE parameters: Channel type (advertising, Header16/24b), Whitening & CRC init, SyncWord/AccessCode
    pub async fn set_ble_params(&mut self, crc_in_fifo: bool, channel_type: ChannelType, whit_init: u8, crc_init: u32, syncword: u32) -> Result<(), Lr2021Error> {
        let req = set_ble_channel_params_cmd(crc_in_fifo, channel_type, whit_init, crc_init, syncword);
        self.cmd_wr(&req).await
    }

    /// Set the PDU length and send the packet
    /// PDU must be ready in FIFO
    pub async fn set_ble_tx(&mut self, len: u8) -> Result<(), Lr2021Error> {
        let req = set_ble_tx_cmd(len);
        self.cmd_wr(&req).await
    }

    /// Set the PDU length
    /// Useful compare to set_ble_tx when using a pin trigger to control the exact transmit time
    pub async fn set_ble_tx_pdu_len(&mut self, len: u8) -> Result<(), Lr2021Error> {
        let req = set_ble_tx_pdu_len_cmd(len);
        self.cmd_wr(&req).await
    }

    /// Return length of last packet received
    pub async fn get_ble_packet_status(&mut self) -> Result<BlePacketStatusRsp, Lr2021Error> {
        let req = get_ble_packet_status_req();
        let mut rsp = BlePacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_ble_rx_stats(&mut self) -> Result<BleRxStatsRsp, Lr2021Error> {
        let req = get_ble_rx_stats_req();
        let mut rsp = BleRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return advanced RX stats
    pub async fn get_ble_rx_stats_adv(&mut self) -> Result<BleRxStatsRspAdv, Lr2021Error> {
        let req = get_ble_rx_stats_req();
        let mut rsp = BleRxStatsRspAdv::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Configure BLE Constant Tone Extension handling
    /// Guard time is given with 125ns resolution (i.e. 32 = 4us)
    pub async fn set_ble_cte(&mut self, en: bool, guard: u8, aoa_smpl: AoaSampling) -> Result<(), Lr2021Error> {
        let value = ((aoa_smpl as u32) << 2)
                + if en {8} else {0}
                + ((guard as u32) << 4);
        self.wr_reg_mask(ADDR_CTE_CTRL, 0x03FC, value).await
    }

    /// Return information on Constant Tone Extension
    pub async fn get_ble_cte_info(&mut self) -> Result<CteInfo, Lr2021Error> {
        let reg = self.rd_reg(ADDR_CTE_CTRL).await?;
        let nb_sample = ((reg >> 20) & 0x7F) as u8;
        let kind: CteKind = ((reg >> 28) as u8).into();
        Ok(CteInfo {nb_sample, kind})
    }

    /// Read all CTE samples
    pub async fn get_ble_cte_all_samples(&mut self, buffer: &mut [i8]) -> Result<CteInfo, Lr2021Error> {
        let info = self.get_ble_cte_info().await?;
        // Ensure we start at first address
        self.wr_reg_mask(ADDR_CTE_CTRL, 0x07F000, 0).await?;
        // Ensure number of sample will fit the provided buffer
        for chunk in buffer.chunks_mut(2).take(info.nb_sample as usize) {
            (chunk[0],chunk[1]) = self.get_ble_cte_next_sample().await?;
        }
        Ok(info)
    }

    /// Retrieve the next CTE complex sample stored
    pub async fn get_ble_cte_next_sample(&mut self) -> Result<(i8,i8), Lr2021Error> {
        let reg = self.rd_reg(ADDR_CTE_SMPL).await?;
        let i = (reg & 0x3F) as i8 - if (reg & 0x20)!=0 {64} else {0};
        let q = ((reg >> 8) & 0x3F) as i8 - if (reg & 0x2000)!=0 {64} else {0};
        Ok((i,q))
    }

}
