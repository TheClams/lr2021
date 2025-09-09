// Raw commands API

use crate::status::Status;
use super::RxBw;

/// Select which memory to use for ddmi capture
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RamSel {
    None = 0,
    Ram1 = 1,
    Ram1p2 = 2,
    Ram1p23 = 3,
    Ram2 = 4,
    Ram2p3 = 5,
    Ram3 = 6,
}

/// IQ capture mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CaptureMode {
    None = 0,
    Fifo = 1,
    Ddmi = 2,
}

/// Digital front-end selection for IQ capture
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CaptureDataSel {
    Src = 0,
    Dagc = 1,
    ChannelFilter = 2,
    Decimation = 3,
    Mixer = 4,
    Input = 5,
    SrcGain = 6,
}

/// TX mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxIqMode {
    Iq = 0,
    Freq = 1,
    Phase = 2,
}

/// Trigger selection for the Raw IQ capture start
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CaptureTrigger {
    Soft = 0,
    Rssi = 1,
    External = 2,
    Detect = 3,
    RxDone = 4,
    SyncFound = 5,
    HicIrq13 = 6,
    Rtc3 = 7,
}

/// Sets the raw IQ capture parameters. IQ capture can be done either exclusively using the packet type RAW, or in parallel to the use of a demodulator when in any packet type
pub fn set_raw_iq_capture_params_cmd(rx_bw: RxBw, sample_rate: u32, ram_sel: RamSel, capture_mode: CaptureMode, capture_data_sel: CaptureDataSel) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x90;

    cmd[2] |= rx_bw as u8;
    cmd[3] |= ((sample_rate >> 24) & 0xFF) as u8;
    cmd[4] |= ((sample_rate >> 16) & 0xFF) as u8;
    cmd[5] |= ((sample_rate >> 8) & 0xFF) as u8;
    cmd[6] |= (sample_rate & 0xFF) as u8;
    cmd[7] |= ((ram_sel as u8) & 0x7) << 5;
    cmd[7] |= ((capture_mode as u8) & 0x3) << 3;
    cmd[7] |= (capture_data_sel as u8) & 0x7;
    cmd
}

#[allow(clippy::too_many_arguments)]
/// Sets the raw IQ capture parameters. IQ capture can be done either exclusively using the packet type RAW, or in parallel to the use of a demodulator when in any packet type
pub fn set_raw_iq_capture_params_adv_cmd(rx_bw: RxBw, sample_rate: u32, ram_sel: RamSel, capture_mode: CaptureMode, capture_data_sel: CaptureDataSel, capture_delay_us: u16, ddmi_sample_nb: u16, change_dma_arb_size: u8) -> [u8; 13] {
    let mut cmd = [0u8; 13];
    cmd[0] = 0x02;
    cmd[1] = 0x90;

    cmd[2] |= rx_bw as u8;
    cmd[3] |= ((sample_rate >> 24) & 0xFF) as u8;
    cmd[4] |= ((sample_rate >> 16) & 0xFF) as u8;
    cmd[5] |= ((sample_rate >> 8) & 0xFF) as u8;
    cmd[6] |= (sample_rate & 0xFF) as u8;
    cmd[7] |= ((ram_sel as u8) & 0x7) << 5;
    cmd[7] |= ((capture_mode as u8) & 0x3) << 3;
    cmd[7] |= (capture_data_sel as u8) & 0x7;
    cmd[8] |= ((capture_delay_us >> 8) & 0xFF) as u8;
    cmd[9] |= (capture_delay_us & 0xFF) as u8;
    cmd[10] |= ((ddmi_sample_nb >> 8) & 0xFF) as u8;
    cmd[11] |= (ddmi_sample_nb & 0xFF) as u8;
    cmd[12] |= change_dma_arb_size;
    cmd
}

/// Gets the number of captured DDMI sample.
pub fn get_raw_iq_ddmi_cnt_req() -> [u8; 2] {
    [0x02, 0x91]
}

/// Reads out num bytes starting at relative offset from the DDMI RAMs. Note: in case of 16bit IQ samples, the format is LITTLE ENDIAN
pub fn get_raw_iq_ddmi_data_req(offset: u16, num: u8) -> [u8; 5] {
    let mut cmd = [0u8; 5];
    cmd[0] = 0x02;
    cmd[1] = 0x92;

    cmd[2] |= ((offset >> 8) & 0xFF) as u8;
    cmd[3] |= (offset & 0xFF) as u8;
    cmd[4] |= num;
    cmd
}

/// Sets the parameters used for TX. Note: for mode = IQ, tx_sample_num is the number of IQ pairs, so the number of bytes to write into the TX FIFO is 2*tx_sample_num
pub fn set_raw_iq_tx_params_cmd(tx_sample_num: u16, tx_sample_rate: u32, tx_iq_mode: TxIqMode) -> [u8; 9] {
    let mut cmd = [0u8; 9];
    cmd[0] = 0x02;
    cmd[1] = 0x93;

    cmd[2] |= ((tx_sample_num >> 8) & 0xFF) as u8;
    cmd[3] |= (tx_sample_num & 0xFF) as u8;
    cmd[4] |= ((tx_sample_rate >> 24) & 0xFF) as u8;
    cmd[5] |= ((tx_sample_rate >> 16) & 0xFF) as u8;
    cmd[6] |= ((tx_sample_rate >> 8) & 0xFF) as u8;
    cmd[7] |= (tx_sample_rate & 0xFF) as u8;
    cmd[8] |= (tx_iq_mode as u8) & 0x3;
    cmd
}

/// Sets the raw IQ capture trigger parameters
pub fn set_raw_iq_trigger_cmd(trigger_start: CaptureTrigger, trigger_stop: CaptureTrigger) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x94;

    cmd[2] |= ((trigger_start as u8) & 0xF) << 4;
    cmd[2] |= (trigger_stop as u8) & 0xF;
    cmd
}

/// Sets the raw IQ capture trigger parameters
pub fn set_raw_iq_trigger_adv_cmd(trigger_start: CaptureTrigger, trigger_stop: CaptureTrigger, rssi_up: u16, rssi_down: u16) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x94;

    cmd[2] |= ((trigger_start as u8) & 0xF) << 4;
    cmd[2] |= (trigger_stop as u8) & 0xF;
    cmd[3] |= ((rssi_up >> 1) & 0xFF) as u8;
    cmd[5] |= (rssi_up & 0xFF) as u8;
    cmd[4] |= ((rssi_down >> 1) & 0xFF) as u8;
    cmd[5] |= ((rssi_down & 0xFF) << 1) as u8;
    cmd
}

// Response structs

/// Response for GetRawIqDdmiCnt command
#[derive(Default)]
pub struct RawIqDdmiCntRsp([u8; 4]);

impl RawIqDdmiCntRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// High when wrapping occured during capture
    pub fn wrapped(&self) -> bool {
        (self.0[2] >> 7) & 0x1 != 0
    }

    /// Number of captured IQ bytes for ddmi
    pub fn cnt(&self) -> u16 {
        (self.0[3] as u16) |
        (((self.0[2] & 0x7F) as u16) << 8)
    }
}

impl AsMut<[u8]> for RawIqDdmiCntRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRawIqDdmiData command
#[derive(Default)]
pub struct RawIqDdmiDataRsp([u8; 2]);

impl RawIqDdmiDataRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }
    // TODO: Implement accessor for variable length field 'data'
}

impl AsMut<[u8]> for RawIqDdmiDataRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
