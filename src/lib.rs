#![no_std]

pub mod status;
pub mod system;
pub mod cmd;
pub mod radio;
pub mod lora;
pub mod ble;
pub mod flrc;
pub mod ook;
pub mod fsk;
pub mod zigbee;
pub mod zwave;
pub mod lrfhss;
pub mod wmbus;
pub mod wisun;
pub mod bpsk_tx;

use core::marker::PhantomData;

use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_hal::digital::v2::{OutputPin, InputPin};
use embedded_hal_async::{digital::Wait, spi::SpiBus};

use status::{CmdStatus, Intr, Status};
pub use cmd::{RxBw, PulseShape}; // Re-export Bandwidth enum as it is used for all packet types

trait Sealed{}
#[allow(private_bounds)]
/// Sealed trait to implement two flavor of the driver where
/// the busy pin can be either a simple input or one implemeting the Wait trait
pub trait BusyPin: Sealed {
    type Pin: InputPin;

    #[allow(async_fn_in_trait)]
    async fn wait_ready(pin: &mut Self::Pin, timeout: Duration) -> Result<(), Lr2021Error>;
}
pub struct BusyBlocking<I> {
    _marker: PhantomData<I>
}
pub struct BusyAsync<I> {
    _marker: PhantomData<I>
}
impl<I> Sealed for BusyBlocking<I> {}
impl<I> Sealed for BusyAsync<I> {}

impl<I: InputPin> BusyPin for BusyBlocking<I> {
    type Pin = I;

    /// Poll busy pin until it goes low
    async fn wait_ready(pin: &mut I, timeout: Duration) -> Result<(), Lr2021Error> {
        let start = Instant::now();
        while pin.is_high().map_err(|_| Lr2021Error::Pin)? {
            if start.elapsed() >= timeout {
                return Err(Lr2021Error::BusyTimeout);
            }
            // Timer::after_micros(5).await;
        }
        Ok(())
    }
}

impl<I: InputPin + Wait> BusyPin for BusyAsync<I> {
    type Pin = I;

    /// Wait for an interrupt on th busy pin to go low (if not already)
    async fn wait_ready(pin: &mut I, timeout: Duration) -> Result<(), Lr2021Error> {
        // Option 1: Use the Wait trait for more efficient waiting
        if pin.is_high().map_err(|_| Lr2021Error::Pin)? {
            match with_timeout(timeout, pin.wait_for_low()).await {
                Ok(_) => Ok(()),
                Err(_) => Err(Lr2021Error::BusyTimeout),
            }
        } else {
            Ok(())
        }
    }
}

/// Size of an the internal buffer set to the largest command (outside those with variable number of parameters)
const BUFFER_SIZE: usize = 256;
/// Command Buffer:
pub struct CmdBuffer ([u8;BUFFER_SIZE+2]);

impl CmdBuffer {
    /// Create a zero initialized buffer
    pub fn new() -> Self {
        CmdBuffer([0;BUFFER_SIZE+2])
    }

    /// Set first two byte to 0 corresponding to the NOP command
    pub fn nop(&mut self) {
        self.0[0] = 0;
        self.0[1] = 0;
    }

    /// Return the first two bytes as a status
    pub fn status(&self) -> Status {
        Status::from_array([self.0[0],self.0[1]])
    }

    /// Update the status from a slice of bytes
    pub fn updt_status(&mut self, bytes: &[u8]) {
        self.0.iter_mut()
            .zip(bytes)
            .take(2)
            .for_each(|(s,&b)| *s = b);
    }

    /// The bits [3:1] contains a CmdStatus
    pub fn cmd_status(&self) -> CmdStatus {
        let bits_cmd = ((self.0[0] >> 1) & 7) as u8;
        bits_cmd.into()
    }

    /// Give read access to the the last 256 bytes
    pub fn data(&self) -> &[u8] {
        &self.0[2..]
    }

    /// Give read/write access to the the last 256 bytes
    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.0[2..]
    }
}



/// LR2021 Device
pub struct Lr2021<O,SPI, M: BusyPin> {
    /// Reset pin  (active low)
    nreset: O,
    /// Busy pin from the LR2021 indicating if the LR2021 is ready to handle commands
    busy: M::Pin,
    /// SPI device
    spi: SPI,
    /// NSS output pin
    nss: O,
    /// Buffer to store SPI commands/response
    buffer: CmdBuffer,
}

/// Error using the LR2021
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Lr2021Error {
    /// Unable to Set/Get a pin level
    Pin,
    /// Unable to use SPI
    Spi,
    /// Last command failed
    CmdFail,
    /// Last command was invalid
    CmdErr,
    /// Timeout while waiting for busy
    BusyTimeout,
    /// Command with invalid size (>18B)
    InvalidSize,
    /// Unknown error
    Unknown,
}

// Create driver with busy pin not implementing wait
impl<I,O,SPI> Lr2021<O,SPI, BusyBlocking<I>> where
    I: InputPin, O: OutputPin, SPI: SpiBus<u8>
{
    /// Create a LR2021 Device with blocking access on the busy pin
    pub fn new_blocking(nreset: O, busy: I, spi: SPI, nss: O) -> Self {
        Self { nreset, busy, spi, nss, buffer: CmdBuffer::new()}
    }

}

// Create driver with busy pin implementing wait
impl<I,O,SPI> Lr2021<O,SPI, BusyAsync<I>> where
    I: InputPin + Wait, O: OutputPin, SPI: SpiBus<u8>
{
    /// Create a LR2021 Device with async busy pin
    pub fn new(nreset: O, busy: I, spi: SPI, nss: O) -> Self {
        Self { nreset, busy, spi, nss, buffer: CmdBuffer::new()}
    }
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Reset the chip
    pub async fn reset(&mut self) -> Result<(), Lr2021Error> {
        self.nreset.set_low().map_err(|_| Lr2021Error::Pin)?;
        Timer::after_millis(10).await;
        self.nreset.set_high().map_err(|_| Lr2021Error::Pin)?;
        Timer::after_millis(10).await;
        Ok(())
    }

    /// Check if the busy pin is high (debug)
    pub fn is_busy(&self) -> bool {
        self.busy.is_high().unwrap_or(false)
    }

    /// Last status (command status, chip mode, interrupt, ...)
    pub fn status(&self) -> Status {
        self.buffer.status()
    }

    /// Read access to internal buffer
    pub fn buffer(&self) -> &[u8] {
        self.buffer.data()
    }

    /// Read/Write access to internal buffer
    pub fn buffer_mut(&mut self) -> &mut [u8] {
        self.buffer.data_mut()
    }

    /// Last captured interrupt status
    /// Note: might be incomplete if last command was less than 6 bytes
    pub fn last_intr(&self) -> Intr {
        Intr::from_slice(&self.buffer.data()[2..6])
    }

    /// Wait for LR2021 to be ready for a command, i.e. busy pin low
    pub async fn wait_ready(&mut self, timeout: Duration) -> Result<(), Lr2021Error> {
        M::wait_ready(&mut self.busy, timeout).await
    }

    /// Write the beginning of a command, allowing to fill with variable length fields
    pub async fn cmd_wr_begin(&mut self, req: &[u8]) -> Result<(), Lr2021Error> {
        if req.len() > BUFFER_SIZE {
            return Err(Lr2021Error::InvalidSize);
        }
        self.wait_ready(Duration::from_millis(100)).await?;
        self.nss.set_low().map_err(|_| Lr2021Error::Pin)?;
        let rsp_buf = &mut self.buffer.0[..req.len()];
        self.spi
            .transfer(rsp_buf, req).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.buffer.cmd_status().check()
    }

    /// Write a command
    pub async fn cmd_wr(&mut self, req: &[u8]) -> Result<(), Lr2021Error> {
        // #[cfg(feature = "defmt")]{defmt::info!("[CMD WR] {:02x}", req);}
        self.cmd_wr_begin(req).await?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Write a command and read response
    /// Rsp must be n bytes where n is the number of expected byte
    pub async fn cmd_rd(&mut self, req: &[u8], rsp: &mut [u8]) -> Result<(), Lr2021Error> {
        self.cmd_wr(req).await?;
        // Wait for busy to go down before reading the response
        // Some command can have large delay: temperature measurement with highest resolution (13b) takes more than 270us
        self.wait_ready(Duration::from_millis(1)).await?;
        // Read response by transfering a buffer starting with two 0 and replacing it by the read bytes
        self.nss.set_low().map_err(|_| Lr2021Error::Pin)?;
        self.spi
            .transfer_in_place(rsp).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)?;
        // #[cfg(feature = "defmt")]{defmt::info!("[CMD RD] {:02x} => {:02x}", req, rsp);}
        // Save the first two bytes from the response to keep the command status
        self.buffer.updt_status(rsp);
        self.buffer.cmd_status().check()
    }

    /// Write a command with vairable length payload
    /// Any feedback data will be available in side the local buffer
    pub async fn cmd_data_wr(&mut self, opcode: &[u8], data: &[u8]) -> Result<(), Lr2021Error> {
        self.cmd_wr_begin(&opcode).await?;
        let rsp = &mut self.buffer.data_mut()[..data.len()];
        self.spi
            .transfer(rsp, data).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Write a command with variable length payload, and save result provided buffer
    pub async fn cmd_data_rw(&mut self, opcode: &[u8], data: &mut [u8]) -> Result<(), Lr2021Error> {
        self.cmd_wr_begin(&opcode).await?;
        self.spi
            .transfer_in_place(data).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Send content of the local buffer as a command
    pub async fn cmd_buf_wr(&mut self, len: usize) -> Result<(), Lr2021Error> {
        self.wait_ready(Duration::from_millis(100)).await?;
        self.nss.set_low().map_err(|_| Lr2021Error::Pin)?;
        self.spi
            .transfer_in_place(&mut self.buffer.data_mut()[..len]).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

    /// Send content of the local buffer as a command and read a response
    pub async fn cmd_buf_rd(&mut self, len: usize, rsp: &mut [u8]) -> Result<(), Lr2021Error> {
        self.cmd_buf_wr(len).await?;
        // Wait for busy to go down before reading the response
        // Some command can have large delay: temperature measurement with highest resolution (13b) takes more than 270us
        self.wait_ready(Duration::from_millis(1)).await?;
        // Read response by transfering a buffer full of 0 and replacing it by the read bytes
        self.nss.set_low().map_err(|_| Lr2021Error::Pin)?;
        self.spi
            .transfer_in_place(rsp).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)?;
        // Save the first two bytes from the response to keep the command status
        self.buffer.updt_status(rsp);
        self.buffer.cmd_status().check()
    }

    /// Wake-up the chip from a sleep mode (Set NSS low until busy goes low)
    pub async fn wake_up(&mut self) -> Result<(), Lr2021Error> {
        self.nss.set_low().map_err(|_| Lr2021Error::Pin)?;
        self.wait_ready(Duration::from_millis(100)).await?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)
    }

}
