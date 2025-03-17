#![no_std]

//! MS5611 pressure sensor driver using I2C.
//!
//! This library supports both synchronous and asynchronous modes of operation.
//! The mode is selected using the `sync` and `async` features. Only one
//! of these features can be enabled at a time.
//!
//! # Features
//!
//! - `sync`: Enables synchronous mode using `embedded_hal` traits.
//! - `async`: Enables asynchronous mode using `embedded_hal_async` traits.
//!
//! # Example
//!
//! ```ignore
//! use ms5611_i2c::{Ms5611, OversamplingRatio};
//! use embedded_hal::blocking::i2c::{Write, WriteRead};
//!
//! // Create an instance of the I2C bus (implementation specific)
//! let i2c = ...;
//!
//! // Create an instance of the MS5611 sensor
//! let mut sensor = Ms5611::new(i2c, None).unwrap();
//!
//! // Reset the sensor
//! sensor.reset().unwrap();
//!
//! // Read a sample with a specific oversampling ratio
//! let sample = sensor.read_sample(OversamplingRatio::Opt4096).unwrap();
//!
//! println!("Pressure: {} mbar, Temperature: {} °C", sample.pressure_mbar, sample.temperature_c);
//! ```
//!
//! # Errors
//!
//! The library defines a custom error type `MS5611Error` which can represent
//! the following errors:
//!
//! - `I2CError`: An error occurred during I2C communication.
//! - `BadChecksum`: The checksum of the PROM data did not match.
//! - `BadProm`: The PROM data is invalid.
//! - `NoProm`: The PROM data could not be read.
//! - `BadAddress`: The I2C address is invalid.
//! - `BadOsr`: The oversampling ratio is invalid.
//!
//! # Structs
//!
//! - `Ms5611`: Represents the MS5611 pressure sensor.
//! - `Ms5611Sample`: Represents a sample of pressure and temperature data.
//! - `Prom`: Represents the factory calibrated data stored in the sensor's ROM.
//!
//! # Enums
//!
//! - `MS5611Error`: Represents the possible errors that can occur.
//! - `OversamplingRatio`: Represents the available oversampling ratios.
//!
//! # Methods
//!
//! - `Ms5611::new`: Creates a new instance of the MS5611 sensor.
//! - `Ms5611::reset`: Resets the sensor.
//! - `Ms5611::read_sample`: Reads a sample of pressure and temperature data.
//! - `OversamplingRatio::get_delay`: Returns the delay required for the given oversampling ratio.
//! - `OversamplingRatio::addr_modifier`: Returns the address modifier for the given oversampling ratio.

#[cfg(all(feature = "sync", feature = "async"))]
compile_error!("Feature 'sync' and 'async' cannot be enabled at the same time");

#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("Either 'sync' or 'async' must be enabled");

use byteorder::{BigEndian, ByteOrder};
#[cfg(feature = "sync")]
use embassy_time::Delay;
use embassy_time::{Duration, Timer};
#[cfg(feature = "sync")]
use embedded_hal::delay::DelayNs;
#[cfg(feature = "sync")]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;
use maybe_async::maybe_async;
#[derive(Debug)]
pub enum MS5611Error {
    I2CError,
    BadChecksum,
    BadProm,
    NoProm,
    BadAddress,
    BadOsr,
}

/// Oversampling ratio
/// See datasheet for more information.
pub enum OversamplingRatio {
    Opt256,
    Opt512,
    Opt1024,
    Opt2048,
    Opt4096,
}

impl OversamplingRatio {
    fn get_delay(&self) -> u32 {
        match *self {
            OversamplingRatio::Opt256 => 1,
            OversamplingRatio::Opt512 => 2,
            OversamplingRatio::Opt1024 => 3,
            OversamplingRatio::Opt2048 => 5,
            OversamplingRatio::Opt4096 => 10,
        }
    }

    fn addr_modifier(&self) -> u8 {
        match *self {
            OversamplingRatio::Opt256 => 0,
            OversamplingRatio::Opt512 => 2,
            OversamplingRatio::Opt1024 => 4,
            OversamplingRatio::Opt2048 => 6,
            OversamplingRatio::Opt4096 => 8,
        }
    }
}

/// Pressure sensor
pub struct Ms5611<I> {
    i2c: I,
    address: u8,
    prom: Prom,
}

enum Ms5611Reg {
    Reset,
    /// Digital pressure value
    D1,
    /// Digital temperature value
    D2,
    /// AdcRead command returns 24-bit result.
    AdcRead,
    /// Prom command returns 16-bit result.
    Prom,
}

impl Ms5611Reg {
    fn addr(&self) -> u8 {
        match *self {
            Ms5611Reg::Reset => 0x1e,
            Ms5611Reg::D1 => 0x40,
            Ms5611Reg::D2 => 0x50,
            Ms5611Reg::AdcRead => 0x00,
            // Valid from 0xa0 to 0xae
            Ms5611Reg::Prom => 0xa0,
        }
    }
}

/// Output from the MS5611.
#[derive(Debug)]
pub struct Ms5611Sample {
    /// Pressure measured in millibars.
    pub pressure_mbar: f32,
    /// Temperature in celsius.
    pub temperature_c: f32,
}

/// Factory calibrated data in device's ROM.
#[derive(Debug)]
struct Prom {
    /// From datasheet, C1.
    pub pressure_sensitivity: u16,
    /// From datasheet, C2.
    pub pressure_offset: u16,
    /// From datasheet, C3.
    pub temp_coef_pressure_sensitivity: u16,
    /// From datasheet, C4.
    pub temp_coef_pressure_offset: u16,
    /// From datasheet, C5.
    pub temp_ref: u16,
    /// From datasheet, C6.
    pub temp_coef_temp: u16,
}

impl<I> Ms5611<I>
where
    I: I2c,
{
    /// Constructs a new `Ms5611<I>`.
    ///
    /// if no address is provided, the default address `0x77` is used.
    ///
    #[maybe_async]
    pub async fn new(mut i2c: I, i2c_addr: Option<u8>) -> Result<Self, MS5611Error>
    where
        I: I2c,
    {
        let address = i2c_addr.unwrap_or(0x77);
        let prom = Self::read_prom(&mut i2c, address).await?;
        Ok(Ms5611 { i2c, address, prom })
    }

    #[maybe_async]
    pub async fn reset(&mut self) -> Result<(), MS5611Error> {
        self.i2c
            .write(self.address, &[Ms5611Reg::Reset.addr()])
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        // Haven't tested for the lower time bound necessary for the chip to
        // start functioning again. But, it does require some amount of sleep.
        let _x = Timer::after(Duration::from_millis(500)).await;
        Ok(())
    }

    #[maybe_async]
    async fn read_prom(i2c: &mut I, address: u8) -> Result<Prom, MS5611Error> {
        let mut crc_check = 0u16;

        // This is the CRC scheme in the MS5611 AN520 (Application Note)
        fn crc_accumulate_byte(crc_check: &mut u16, byte: u8) {
            *crc_check ^= byte as u16;
            for _ in 0..8 {
                if (*crc_check & 0x8000) > 0 {
                    *crc_check = (*crc_check << 1) ^ 0x3000;
                } else {
                    *crc_check <<= 1;
                }
            }
        }

        fn crc_accumulate_buf2(crc_check: &mut u16, buf: &[u8]) {
            crc_accumulate_byte(crc_check, buf[0]);
            crc_accumulate_byte(crc_check, buf[1]);
        }

        let mut buf: [u8; 2] = [0u8; 2];
        // Address reserved for manufacturer. We need it for the CRC.
        i2c.write_read(address, &[Ms5611Reg::Prom.addr()], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c.write_read(address, &[Ms5611Reg::Prom.addr() + 2], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        let pressure_sensitivity = BigEndian::read_u16(&buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c.write_read(address, &[Ms5611Reg::Prom.addr() + 4], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        let pressure_offset = BigEndian::read_u16(&buf);

        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c.write_read(address, &[Ms5611Reg::Prom.addr() + 6], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        let temp_coef_pressure_sensitivity = BigEndian::read_u16(&buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c.write_read(address, &[Ms5611Reg::Prom.addr() + 8], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        let temp_coef_pressure_offset = BigEndian::read_u16(&buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c.write_read(address, &[Ms5611Reg::Prom.addr() + 10], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        let temp_ref = BigEndian::read_u16(&buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c.write_read(address, &[Ms5611Reg::Prom.addr() + 12], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        let temp_coef_temp = BigEndian::read_u16(&buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        i2c.write_read(address, &[Ms5611Reg::Prom.addr() + 14], &mut buf)
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        // CRC is only last 4 bits
        let crc = BigEndian::read_u16(&buf) & 0x000f;
        crc_accumulate_byte(&mut crc_check, buf[0]);
        crc_accumulate_byte(&mut crc_check, 0);

        crc_check >>= 12;

        if crc != crc_check {
            panic!("PROM CRC did not match: {} != {}", crc, crc_check);
        }
        Ok(Prom {
            pressure_sensitivity,
            pressure_offset,
            temp_coef_pressure_sensitivity,
            temp_coef_pressure_offset,
            temp_ref,
            temp_coef_temp,
        })
    }

    #[maybe_async]
    pub async fn read_sample(
        &mut self,
        osr: OversamplingRatio,
    ) -> Result<Ms5611Sample, MS5611Error> {
        // Note: Variable names aren't pretty, but they're consistent with the
        // MS5611 datasheet.
        let mut buf = [0u8; 4];

        self.i2c
            .write(self.address, &[Ms5611Reg::D1.addr() + osr.addr_modifier()])
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        // If we don't delay, the read is all 0s.
        let delay = osr.get_delay();

        if delay > 0 {
            #[cfg(feature = "sync")]
            Delay.delay_ms(delay);
            #[cfg(feature = "async")]
            let _res = Timer::after(Duration::from_millis(delay.into())).await;
        }

        self.i2c
            .write_read(self.address, &[Ms5611Reg::AdcRead.addr()], &mut buf[1..4])
            .await
            .map_err(|_| MS5611Error::I2CError)?;

        // Raw digital pressure
        let d1 = BigEndian::read_i32(&buf);

        self.i2c
            .write(self.address, &[Ms5611Reg::D2.addr() + osr.addr_modifier()])
            .await
            .map_err(|_| MS5611Error::I2CError)?;
        if delay > 0 {
            #[cfg(feature = "sync")]
            Delay.delay_ms(delay);
            #[cfg(feature = "async")]
            let _res = Timer::after(Duration::from_millis(delay.into())).await;
        }
        self.i2c
            .write_read(self.address, &[Ms5611Reg::AdcRead.addr()], &mut buf[1..4])
            .await
            .map_err(|_| MS5611Error::I2CError)?;

        // Raw digital temperature
        let d2 = BigEndian::read_i32(&buf) as i64;

        // Temperature difference from reference
        let dt = d2 - ((self.prom.temp_ref as i64) << 8);

        // Units: celcius * 100
        let mut temperature: i32 = 2000 + (((dt * (self.prom.temp_coef_temp as i64)) >> 23) as i32);

        let mut offset: i64 = ((self.prom.pressure_offset as i64) << 16)
            + ((dt * (self.prom.temp_coef_pressure_offset as i64)) >> 7);
        let mut sens: i64 = ((self.prom.pressure_sensitivity as i64) << 15)
            + ((dt * (self.prom.temp_coef_pressure_sensitivity as i64)) >> 8);

        let mut t2 = 0i32;
        let mut off2 = 0i64;
        let mut sens2 = 0i64;

        //
        // Second order temperature compensation
        //
        // Low temperature (< 20C)
        if temperature < 2000 {
            t2 = ((dt * dt) >> 31) as i32;
            off2 = ((5 * (temperature - 2000).pow(2)) >> 1) as i64;
            sens2 = off2 >> 1;
        }

        // Very low temperature (< -15)
        if temperature < -1500 {
            off2 += 7 * (temperature as i64 + 1500).pow(2);
            sens2 += (11 * (temperature as i64 + 1500).pow(2)) >> 1;
        }

        temperature -= t2;
        offset -= off2;
        sens -= sens2;

        // Units: mbar * 100
        let pressure: i32 = (((((d1 as i64) * sens) >> 21) - offset) >> 15) as i32;

        Ok(Ms5611Sample {
            pressure_mbar: pressure as f32 / 100.0,
            temperature_c: temperature as f32 / 100.0,
        })
    }
}
