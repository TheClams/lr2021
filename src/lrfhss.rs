use embedded_hal::digital::v2::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_lrfhss::*;
use super::{BusyPin, Lr2021, Lr2021Error};

#[derive(Clone)]
pub struct LrfhssHop {
    /// Frequency
    freq: u32,
    /// Duration of the hop in number of symbol
    len: u16,
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    // TODO: add dedicated struct and find a good default set of values (maybe 2-3 builder method)
    #[allow(clippy::too_many_arguments)]
    /// Prepare the LR-FHSS packet
    #[doc(alias = "lrfhss")]
    pub async fn lrfhss_build_packet(&mut self, sync_header_cnt: u8, cr: LrfhssCr, grid: Grid, hopping: Hopping, bw: u8, sequence: u16, offset: i8, pld: &[u8]) -> Result<(), Lr2021Error> {
        let req = lr_fhss_build_frame_cmd(sync_header_cnt, cr, grid, hopping, bw, sequence, offset);
        self.cmd_data_wr(&req, pld).await
    }

    /// Configure Syncword of LRFHSS packet
    /// Default value is 0x2C0F7995
    #[doc(alias = "lrfhss")]
    pub async fn set_lrfhss_syncword(&mut self, syncword: u32) -> Result<(), Lr2021Error> {
        let req = set_lr_fhss_sync_word_cmd(syncword);
        self.cmd_wr(&req).await
    }

    /// Set the LRFHSS hopping table
    /// The data parameter should be up to 40 pairs (freq (4B), Nb_symbols (2B))
    #[doc(alias = "lrfhss")]
    pub async fn set_lrfhss_hopping(&mut self, hop_en: bool, freq_hz: bool, pkt_length: u16, nb_used_freqs: u8, nb_hopping_blocks: u8, hops: &[LrfhssHop]) -> Result<(), Lr2021Error> {
        let req = write_lr_fhss_hopping_table_cmd(hop_en, freq_hz, pkt_length, nb_used_freqs, nb_hopping_blocks);
        self.cmd_wr_begin(&req).await?;
        for hop in hops {
            self.buffer_mut()[0] = ((hop.freq >> 24) & 0xFF) as u8;
            self.buffer_mut()[1] = ((hop.freq >> 16) & 0xFF) as u8;
            self.buffer_mut()[2] = ((hop.freq >> 8 ) & 0xFF) as u8;
            self.buffer_mut()[3] = ((hop.freq      ) & 0xFF) as u8;
            self.buffer_mut()[4] = ((hop.len >> 8) & 0xFF) as u8;
            self.buffer_mut()[5] = ((hop.len     ) & 0xFF) as u8;
            self.spi.transfer_in_place(&mut self.buffer.data_mut()[..6]).await
                .map_err(|_| Lr2021Error::Spi)?;
        }
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }


}