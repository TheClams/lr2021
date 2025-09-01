use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::PulseShape;

pub use super::cmd::cmd_bpsk::*;
use super::{BusyPin, Lr2021, Lr2021Error};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Modulation parameters: raw bitrate, pulse shaping, Bandwidth and fdev
    #[doc(alias = "bpsk_tx")]
    pub async fn set_bpsk_modulation(&mut self, bitrate: u32, pulse_shape: PulseShape, diff_mode_en: DiffModeEn, diff_mode_init: bool, diff_mode_parity: bool) -> Result<(), Lr2021Error> {
        let req = set_bpsk_modulation_params_cmd(bitrate, pulse_shape, diff_mode_en, diff_mode_init, diff_mode_parity);
        self.cmd_wr(&req).await
    }

    /// Set FLRC packet parameters: preamble, syncword, header implicit/explicit, CRC and packet length (max 511)
    #[doc(alias = "bpsk_tx")]
    pub async fn set_bpsk_packet(&mut self, pld_len: u8, bpsk_mode: BpskMode, sigfox_msg: SigfoxMsg, sigfox_rank: SigfoxRank) -> Result<(), Lr2021Error> {
        let req = set_bpsk_packet_params_cmd(pld_len, bpsk_mode, sigfox_msg, sigfox_rank);
        self.cmd_wr(&req).await
    }

}