//! Example to communicate with ADS1256 ADC board from Raspberry PI
//! The High-Precision AD/DA board was used for testing.
//! [AD/DA board ]https://www.waveshare.com/wiki/High-Precision_AD/DA_Board
//!

extern crate ads1256;
extern crate linux_embedded_hal as linux_hal;

use linux_hal::gpio_cdev::{Chip, LineRequestFlags};
use linux_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_hal::{Delay, Pin, Spidev};

use ads1256::{Channel, Config, Register, SamplingRate, ADS1256, PGA};

use std::thread;
use std::time::Duration;

fn main() {
    println!("Hello ADS1256 driver..");

    // spidev0.0 is CE0 at GPIO8, BCM pin 24, spidev0.1 is CE1 at GPIO7, BCM pin 26
    // but we use GPIO22, BCM pin 15 for CS
    let mut spi = Spidev::open("/dev/spidev0.1").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_500_000)
        .lsb_first(false)
        .mode(SpiModeFlags::SPI_MODE_1)
        .build();
    spi.configure(&options).unwrap();

    // GPIO chip
    let mut gpio = Chip::new("/dev/gpiochip0").unwrap();

    // CS output pin, GPIO22, BCM pin 15
    let cs_pin = CdevPin::new(gpio.get_line(22)?.request(
        LineRequestFlags::OUTPUT,
        0,
        "ads1256",
    )?)?;

    // RST output pin, GPIO18, BCM pin 12
    let rst_pin = CdevPin::new(gpio.get_line(18)?.request(
        LineRequestFlags::OUTPUT,
        1,
        "ads1256",
    )?)?;

    // DRDY input pin, GPIO22, BCM pin 11
    let drdy_pin = CdevPin::new(gpio.get_line(17)?.request(
        LineRequestFlags::INPUT,
        0,
        "ads1256",
    )?)?;

    // Reset the adc
    rst_pin.set_value(0).unwrap();
    thread::sleep(Duration::from_micros(1)); // t16 delay (0.52us)
    rst_pin.set_value(1).unwrap();

    // Wait for setup
    thread::sleep(Duration::from_millis(200));

    // Create driver instance
    let mut adc = ADS1256::new(spi, cs_pin, rst_pin, drdy_pin, Delay).unwrap();
    let config = Config::new(SamplingRate::Sps30000, PGA::Gain2);
    adc.set_config(&config).unwrap();

    // Read all single ended channels in one-shot mode
    for ch in &[
        Channel::AIN0,
        Channel::AIN1,
        Channel::AIN2,
        Channel::AIN3,
        Channel::AIN4,
        Channel::AIN5,
        Channel::AIN6,
        Channel::AIN7,
    ] {
        let code = adc.read_channel(*ch, Channel::AINCOM).unwrap();
        let in_volt = adc.convert_to_volt(code);
        println!("Channel {:?} : {:#08x}, {:.20} V ", ch, code, in_volt);
    }
}
