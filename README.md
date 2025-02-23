# MS5611 Library for Rust [![Latest Version]][crates.io] [![Documentation]][docs.rs]

[Latest Version]: https://img.shields.io/crates/v/ms5611.svg
[crates.io]: https://crates.io/crates/ms5611-i2c
[Documentation]: https://docs.rs/ms5611-i2c/badge.svg
[docs.rs]: https://docs.rs/ms5611-i2c

A library for the MS5611 barometric pressure sensor. Only supports the i2c
interface (no SPI).
Works with  embassy-hal.
Support sync and async-i2c
Based on https://github.com/braincore/ms5611-rs

## Features

* Per datasheet, computes the second order temperature compensation.
* Validates the PROM's checksum.
* supports embedded_hal_async::i2c::I2c

