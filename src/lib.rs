//! A platform agnostic driver to interface with the  ADS1256 analog-digital converter
//!
//! This driver was built using [`embedded_hal`] traits.
//!
//!
//!
//! # Example
//!   TODO:
//!
//!
//! [ADS1256 datasheet](http://www.ti.com/lit/ds/symlink/ads1256.pdf)

//#![deny(missing_docs)]
extern crate embedded_hal as hal;

use std::error;
use std::fmt::{self, Debug, Display};

//use hal::blocking::spi::{Transfer, Write};
use hal::blocking::delay::DelayUs;

/// ADC reference voltage in volts
const REF_VOLTS: f64 = 2.5;

/// The operation of the ADS1256 is controlled through a set of registers.
/// ADS1256 datasheet,  Table 23.
#[derive(Debug, Copy, Clone)]
pub enum Register {
    STATUS = 0x00,
    MUX = 0x01,
    ADCON = 0x02,
    DRATE = 0x03,
    IO = 0x04,
    OFC0 = 0x05,
    OFC1 = 0x06,
    OFC2 = 0x07,
    FSC0 = 0x08,
    FSC1 = 0x09,
    FSC2 = 0x0A,
}

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

/// The commands control the operation of the ADS1256.
/// CS must stay low during the entire command sequence.
/// See ADS1256 datasheet, Table 24.
#[derive(Debug, Copy, Clone)]
pub enum Command {
    WAKEUP = 0x00,   // Completes SYNC and Exits Standby Mode
    RDATA = 0x01,    // Read Data
    RDATAC = 0x03,   // Read Data Continuously
    SDATAC = 0x0F,   // Stop Read Data Continuously
    RREG = 0x10,     // Read from REG
    WREG = 0x50,     // Write to REG
    SELFCAL = 0xF0,  // Offset and Gain Self-Calibration
    SELFOCAL = 0xF1, // Offset Self-Calibration
    SELFGCAL = 0xF2, // Gain Self-Calibration
    SYSOCAL = 0xF3,  // System Offset Calibration
    SYSGCAL = 0xF4,  // System Gain Calibration
    SYNC = 0xFC,     // Synchronize the A/D Conversion
    STANDBY = 0xFD,  // Begin Standby Mode
    RESET = 0xFE,    // Reset to Power-Up Values
}

impl Command {
    fn bits(self) -> u8 {
        self as u8
    }
}

/// Programmable Gain Amplifier (pga) ads1256 datasheet, p. 16
#[derive(Debug, Copy, Clone)]
pub enum PGA {
    Gain1 = 0b000,
    Gain2 = 0b001,
    Gain4 = 0b010,
    Gain8 = 0b011,
    Gain16 = 0b100,
    Gain32 = 0b101,
    Gain64 = 0b110,
}

impl Default for PGA {
    fn default() -> Self {
        PGA::Gain1
    }
}

impl PGA {
    pub fn bits(self) -> u8 {
        self as u8
    }

    pub fn val(self) -> u8 {
        1 << self as u8
    }
}

/// ADC Sampling rate.
#[derive(Debug, Copy, Clone)]
pub enum SamplingRate {
    Sps30000 = 0b1111_0000,
    Sps15000 = 0b1110_0000,
    Sps7500 = 0b1101_0000,
    Sps3750 = 0b1100_0000,
    Sps2000 = 0b1011_0000,
    Sps1000 = 0b1010_0001,
    Sps500 = 0b1001_0010,
    Sps100 = 0b1000_0010,
    Sps60 = 0b0111_0010,
    Sps50 = 0b0110_0011,
    Sps30 = 0b0101_0011,
    Sps25 = 0b0100_0011,
    Sps15 = 0b0011_0011,
    Sps10 = 0b0010_0011,
    Sps5 = 0b0001_0011,
    Sps2_5 = 0b0000_0011,
}

impl SamplingRate {
    fn bits(self) -> u8 {
        self as u8
    }
}

impl Default for SamplingRate {
    fn default() -> Self {
        SamplingRate::Sps1000
    }
}

/// ADC Channel.
#[derive(Debug, Copy, Clone)]
pub enum Channel {
    AIN0 = 0,
    AIN1 = 1,
    AIN2 = 2,
    AIN3 = 3,
    AIN4 = 4,
    AIN5 = 5,
    AIN6 = 6,
    AIN7 = 7,
    AINCOM = 8,
}

impl Channel {
    fn bits(self) -> u8 {
        self as u8
    }
}

/// Chip configuration.
#[derive(Debug, Copy, Clone)]
pub struct Config {
    pub sampling_rate: SamplingRate,
    pub gain: PGA,
}

impl Config {
    pub fn new(sampling_rate: SamplingRate, gain: PGA) -> Self {
        Config {
            sampling_rate,
            gain,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Config {
            sampling_rate: SamplingRate::Sps1000,
            gain: PGA::Gain1,
        }
    }
}

/// A driver error.
///
/// Note that the `Error` is generic over embedded_hal's Spi, OutputPin, InputPin with no type bounds.
/// The [`Debug`], [`Display`], and [`error::Error`] traits will be added if available.
/// Use a Newtype to add other common bounds if needed.
pub enum Error<ES, EO, EI> {
    /// SPI bus error
    Spi(ES),
    /// GPIO output pin error
    Output(EO),
    /// GPIO input pin error
    Input(EI),
}

impl<ES: Debug, EO: Debug, EI: Debug> Debug for Error<ES, EO, EI> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::Spi(e) => Debug::fmt(e, f),
            Self::Output(e) => Debug::fmt(e, f),
            Self::Input(e) => Debug::fmt(e, f),
        }
    }
}

impl<ES: Display, EO: Display, EI: Display> Display for Error<ES, EO, EI> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::Spi(e) => Display::fmt(e, f),
            Self::Output(e) => Display::fmt(e, f),
            Self::Input(e) => Display::fmt(e, f),
        }
    }
}

impl<ES: error::Error, EO: error::Error, EI: error::Error> error::Error for Error<ES, EO, EI> {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        match self {
            Self::Spi(e) => e.source(),
            Self::Output(e) => e.source(),
            Self::Input(e) => e.source(),
        }
    }
}

impl<ES, EO, EI> Error<ES, EO, EI> {
    fn spi_err(e: ES) -> Error<ES, EO, EI> {
        Self::Spi(e)
    }

    fn output_err(e: EO) -> Error<ES, EO, EI> {
        Self::Output(e)
    }

    fn input_err(e: EI) -> Error<ES, EO, EI> {
        Self::Input(e)
    }
}

/// ADS1256 driver
#[derive(Debug, Default)]
pub struct ADS1256<SPI, CS, RST, DRDY, D> {
    /// Dedicated GPIO pin  that is used to select ADS1256 chip on the SPI bus
    cs_pin: CS,
    /// Dedicated GPIO pin to reset the ADS1256
    reset_pin: RST,
    /// Dedicated GPIO pin to indicate that conversion is ready
    data_ready_pin: DRDY,
    spi: SPI,
    delay: D,
    config: Config,
}

impl<SPI, CS, RST, DRDY, D, ES, EO, EI> ADS1256<SPI, CS, RST, DRDY, D>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = ES> + hal::blocking::spi::Write<u8, Error = ES>,
    CS: hal::digital::v2::OutputPin<Error = EO>,
    RST: hal::digital::v2::OutputPin<Error = EO>,
    DRDY: hal::digital::v2::InputPin<Error = EI>,
    D: DelayUs<u8>,
{
    /// Creates a new driver from a SPI.
    pub fn new(
        spi: SPI,
        cs_pin: CS,
        reset_pin: RST,
        data_ready_pin: DRDY,
        delay: D,
    ) -> Result<Self, Error<ES, EO, EI>> {
        let mut ads1256 = ADS1256 {
            spi,
            cs_pin,
            reset_pin,
            data_ready_pin,
            delay,
            config: Config::default(),
        };

        // stop read data continuously
        ads1256.wait_for_ready()?;
        ads1256.send_command(Command::SDATAC)?;
        ads1256.delay.delay_us(10);
        Ok(ads1256)
    }

    /// Set chip configuration, calls  [`Self::init()`] afterwards.
    pub fn set_config(&mut self, config: &Config) -> Result<(), Error<ES, EO, EI>> {
        self.config = *config;
        self.init()?;
        Ok(())
    }

    /// Initialize the ADC to a configuration.
    pub fn init(&mut self) -> Result<(), Error<ES, EO, EI>> {
        let adcon = self.read_register(Register::ADCON)?;
        // disable clkout and set the gain
        let new_adcon = (adcon & 0x07) | self.config.gain.bits();
        self.write_register(Register::ADCON, new_adcon)?;
        self.write_register(Register::DRATE, self.config.sampling_rate.bits())?;
        self.send_command(Command::SELFCAL)?;
        self.wait_for_ready()?; // wait for calibration to complete
        Ok(())
    }

    /// Returns true if conversion data is ready to  transmit to the host
    pub fn wait_for_ready(&self) -> Result<bool, Error<ES, EO, EI>> {
        self.data_ready_pin.is_low().map_err(Error::input_err)
    }

    /// Read data from specified register.
    pub fn read_register(&mut self, reg: Register) -> Result<u8, Error<ES, EO, EI>> {
        self.cs_pin.set_low().map_err(Error::output_err)?;
        // write
        self.spi
            .write(&[(Command::RREG.bits() | reg.addr()), 0x00])
            .map_err(Error::spi_err)?;
        self.delay.delay_us(10); // t6 delay

        // read
        let mut rx_buf = [0];
        self.spi.transfer(&mut rx_buf).map_err(Error::spi_err)?;
        self.delay.delay_us(5); // t11
        self.cs_pin.set_high().map_err(Error::output_err)?;
        Ok(rx_buf[0])
    }

    /// Write data to specified register.
    pub fn write_register(&mut self, reg: Register, val: u8) -> Result<(), Error<ES, EO, EI>> {
        self.cs_pin.set_low().map_err(Error::output_err)?;

        let mut tx_buf = [(Command::WREG.bits() | reg.addr()), 0x00, val];
        self.spi.transfer(&mut tx_buf).map_err(Error::spi_err)?;
        self.delay.delay_us(5); // t11
        self.cs_pin.set_high().map_err(Error::output_err)?;
        Ok(())
    }

    /// Send a command to the ADC.
    pub fn send_command(&mut self, cmd: Command) -> Result<(), Error<ES, EO, EI>> {
        self.cs_pin.set_low().map_err(Error::output_err)?;
        self.spi.write(&[cmd.bits()]).map_err(Error::spi_err)?;
        self.cs_pin.set_high().map_err(Error::output_err)?;
        Ok(())
    }

    /// Read 24 bit value from ADS1256. Issue this command after DRDY goes low
    fn read_raw_data(&mut self) -> Result<i32, Error<ES, EO, EI>> {
        self.cs_pin.set_low().map_err(Error::output_err)?;
        self.spi
            .write(&[Command::RDATA.bits()])
            .map_err(Error::spi_err)?;
        self.delay.delay_us(10); // t6 delay = 50*0.13=6.5us

        // receive 3 bytes from spi
        let mut buf = [0u8; 3];
        self.spi.transfer(&mut buf).map_err(Error::spi_err)?;
        self.cs_pin.set_high().map_err(Error::output_err)?;

        let mut result: u32 = ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32);
        // sign extension if result is negative
        if (result & 0x800000) != 0 {
            result |= 0xFF000000;
        }
        Ok(result as i32)
    }

    /// Read an ADC channel and returned  24 bit value as i32.
    pub fn read_channel(&mut self, ch1: Channel, ch2: Channel) -> Result<i32, Error<ES, EO, EI>> {
        // wait form data ready pin to be low
        self.wait_for_ready()?;

        // select channel
        self.write_register(Register::MUX, ch1.bits() << 4 | ch2.bits())?;

        // start conversion
        self.send_command(Command::SYNC)?;
        self.delay.delay_us(5); // t11

        self.send_command(Command::WAKEUP)?;
        self.delay.delay_us(1); // t11

        // read channel data
        let adc_code = self.read_raw_data()?;

        Ok(adc_code)
    }

    /// Convert a raw ADC reading to voltage using the current config.
    pub fn convert_to_volt(&self, code: i32) -> f64 {
        (code as f64) / (0x7FFFFF as f64) * (2.0 * REF_VOLTS) / (self.config.gain.val() as f64)
    }
}
