// Wmbus commands API

use crate::status::Status;
use super::RxBw;

/// WM-Bus mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WmbusMode {
    ModeS = 0,
    ModeT1 = 1,
    ModeT2O2m = 2,
    ModeT2M2o = 3,
    ModeR2 = 4,
    ModeC1 = 5,
    ModeC2O2m = 6,
    ModeC2M2o = 7,
    ModeN4p8 = 8,
    ModeN2p4 = 9,
    ModeN6p4 = 10,
    ModeN19p2 = 11,
    ModeF2 = 12,
}

/// Packet format (A or B)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PktFormatTx {
    FormatA = 0,
    FormatB = 1,
}

/// Configure the wm-bus mode according to EN13757-4 2019
pub fn set_wmbus_params_cmd(wmbus_mode: WmbusMode, rx_bw: RxBw, pkt_format_tx: PktFormatTx, addr_filt_en: bool, pld_len: u8, pbl_len_tx: u16, pbl_len_detect: u8) -> [u8; 10] {
    let mut cmd = [0u8; 10];
    cmd[0] = 0x02;
    cmd[1] = 0x6A;

    cmd[2] |= (wmbus_mode as u8) & 0xF;
    cmd[3] |= rx_bw as u8;
    cmd[4] |= (pkt_format_tx as u8) & 0x1;
    if addr_filt_en { cmd[5] |= 1; }
    cmd[6] |= pld_len;
    cmd[7] |= ((pbl_len_tx >> 8) & 0xFF) as u8;
    cmd[8] |= (pbl_len_tx & 0xFF) as u8;
    cmd[9] |= pbl_len_detect;
    cmd
}

/// Get the Rx statistics for wmbus packets
pub fn get_wmbus_rx_stats_req() -> [u8; 2] {
    [0x02, 0x6C]
}

/// Get Rx packet status informations for wmbus packets
pub fn get_wmbus_packet_status_req() -> [u8; 2] {
    [0x02, 0x6D]
}

/// Set the WM-Bus address for RX filtering (A-field)
pub fn set_wmbus_address_cmd(address: u64) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x6E;

    cmd[2] |= ((address >> 40) & 0xFF) as u8;
    cmd[3] |= ((address >> 32) & 0xFF) as u8;
    cmd[4] |= ((address >> 24) & 0xFF) as u8;
    cmd[5] |= ((address >> 16) & 0xFF) as u8;
    cmd[6] |= ((address >> 8) & 0xFF) as u8;
    cmd[7] |= (address & 0xFF) as u8;
    cmd
}

// Response structs

/// Response for GetWmbusRxStats command
#[derive(Default)]
pub struct WmbusRxStatsRsp([u8; 8]);

impl WmbusRxStatsRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Total number of received packets
    pub fn pkt_rx(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }

    /// Number of received packets with a CRC error
    pub fn crc_error(&self) -> u16 {
        (self.0[5] as u16) |
        ((self.0[4] as u16) << 8)
    }

    /// Number of packets with a length error
    pub fn len_error(&self) -> u16 {
        (self.0[7] as u16) |
        ((self.0[6] as u16) << 8)
    }
}

impl AsMut<[u8]> for WmbusRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetWmbusRxStats command
#[derive(Default)]
pub struct WmbusRxStatsRspAdv([u8; 18]);

impl WmbusRxStatsRspAdv {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Total number of received packets
    pub fn pkt_rx(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }

    /// Number of received packets with a CRC error
    pub fn crc_error(&self) -> u16 {
        (self.0[5] as u16) |
        ((self.0[4] as u16) << 8)
    }

    /// Number of packets with a length error
    pub fn len_error(&self) -> u16 {
        (self.0[7] as u16) |
        ((self.0[6] as u16) << 8)
    }

    /// Number of detections
    pub fn pbl_det(&self) -> u16 {
        (self.0[9] as u16) |
        ((self.0[8] as u16) << 8)
    }

    /// Number of good found syncword
    pub fn sync_ok(&self) -> u16 {
        (self.0[11] as u16) |
        ((self.0[10] as u16) << 8)
    }

    /// Number of failed syncword
    pub fn sync_fail(&self) -> u16 {
        (self.0[13] as u16) |
        ((self.0[12] as u16) << 8)
    }

    /// Number of rtc timeouts
    pub fn timeout(&self) -> u16 {
        (self.0[15] as u16) |
        ((self.0[14] as u16) << 8)
    }

    /// Number of packets received with correct CRC
    pub fn crc_ok(&self) -> u16 {
        (self.0[17] as u16) |
        ((self.0[16] as u16) << 8)
    }
}

impl AsMut<[u8]> for WmbusRxStatsRspAdv {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetWmbusPacketStatus command
#[derive(Default)]
pub struct WmbusPacketStatusRsp([u8; 11]);

impl WmbusPacketStatusRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// The demodulated length field. This can be different from get_rx_packet_length (in mode A)
    pub fn l_field(&self) -> u8 {
        self.0[2]
    }

    /// Length of the last received packet in bytes (including optional data added in the FIFO, crc, ...)
    pub fn pkt_len(&self) -> u16 {
        (self.0[4] as u16) |
        ((self.0[3] as u16) << 8)
    }

    /// Average over last packet received of RSSI. Actual signal power is –rssi_avg/2 (dBm)
    pub fn rssi_avg(&self) -> u16 {
        (((self.0[9] >> 4) & 0x1) as u16) |
        ((self.0[5] as u16) << 1)
    }

    /// Latch RSSI value after syncword detection. Actual signal power is –rssi_sync/2 (dBm)
    pub fn rssi_sync(&self) -> u16 {
        ((self.0[9] & 0x1) as u16) |
        ((self.0[6] as u16) << 1)
    }

    /// Indicates which crc has failed. In A mode, bit0 is the crc of the header, then each bit is the crc of the subsequent chunks
    pub fn crc_err(&self) -> u32 {
        (((self.0[9] >> 6) & 0x1) as u32) |
        ((self.0[8] as u32) << 1) |
        ((self.0[7] as u32) << 9)
    }

    /// Index of detected syncword (0/1)
    pub fn syncword_idx(&self) -> bool {
        (self.0[9] >> 7) & 0x1 != 0
    }

    /// Link quality indicator (0.25dB)
    pub fn lqi(&self) -> u8 {
        self.0[10]
    }
}

impl AsMut<[u8]> for WmbusPacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
