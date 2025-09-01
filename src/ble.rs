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
//! - [`set_ble_modulation`](Lr2021::set_ble_modulation) - Configure BLE modulation mode (1M, 2M, 500k, 125k)
//! - [`set_ble_params`](Lr2021::set_ble_params) - Set BLE channel parameters (whitening, CRC, sync word)
//! - [`set_ble_tx`](Lr2021::set_ble_tx) - Set PDU length and transmit packet
//! - [`set_ble_tx_pdu_len`](Lr2021::set_ble_tx_pdu_len) - Set PDU length for pin-triggered transmission
//! - [`get_ble_packet_status`](Lr2021::get_ble_packet_status) - Get status of last received packet
//! - [`get_ble_rx_stats`](Lr2021::get_ble_rx_stats) - Get basic reception statistics
//! - [`get_ble_rx_stats_adv`](Lr2021::get_ble_rx_stats_adv) - Get advanced reception statistics

use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_ble::*;
use super::{BusyPin, Lr2021, Lr2021Error};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set BLE Mode (1M, 2M, 500k, 125k)
    #[doc(alias = "ble")]
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
    #[doc(alias = "ble")]
    pub async fn set_ble_params(&mut self, crc_in_fifo: bool, channel_type: ChannelType, whit_init: u8, crc_init: u32, syncword: u32) -> Result<(), Lr2021Error> {
        let req = set_ble_channel_params_cmd(crc_in_fifo, channel_type, whit_init, crc_init, syncword);
        self.cmd_wr(&req).await
    }

    /// Set the PDU length and send the packet
    /// PDU must be ready in FIFO
    #[doc(alias = "ble")]
    pub async fn set_ble_tx(&mut self, len: u8) -> Result<(), Lr2021Error> {
        let req = set_ble_tx_cmd(len);
        self.cmd_wr(&req).await
    }

    /// Set the PDU length
    /// Useful compare to set_ble_tx when using a pin trigger to control the exact transmit time
    #[doc(alias = "ble")]
    pub async fn set_ble_tx_pdu_len(&mut self, len: u8) -> Result<(), Lr2021Error> {
        let req = set_ble_tx_pdu_len_cmd(len);
        self.cmd_wr(&req).await
    }

    /// Return length of last packet received
    #[doc(alias = "ble")]
    pub async fn get_ble_packet_status(&mut self) -> Result<BlePacketStatusRsp, Lr2021Error> {
        let req = get_ble_packet_status_req();
        let mut rsp = BlePacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    #[doc(alias = "ble")]
    pub async fn get_ble_rx_stats(&mut self) -> Result<BleRxStatsRsp, Lr2021Error> {
        let req = get_ble_rx_stats_req();
        let mut rsp = BleRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return advanced RX stats
    #[doc(alias = "ble")]
    pub async fn get_ble_rx_stats_adv(&mut self) -> Result<BleRxStatsRspAdv, Lr2021Error> {
        let req = get_ble_rx_stats_req();
        let mut rsp = BleRxStatsRspAdv::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}
