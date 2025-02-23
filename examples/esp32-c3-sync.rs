#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{clock::CpuClock, i2c::master::I2c};
use log::info;
use ms5611_i2c::*;

extern crate alloc;

#[esp_hal_embassy::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    info!("Embassy initialized!");

    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default()
            .with_frequency(esp_hal::time::RateExtU32::kHz(400u32)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO6)
    .with_scl(peripherals.GPIO7);
    let mut sensor = Ms5611::new(i2c, None);
    info!("Sensor initialized!");
    loop {
        info!("loop");
        let pressure = sensor.read_sample(ms5611_i2c::OversamplingRatio::Opt512);
        // Do something with the temperature and pressure values
        // For example, print them to a serial console (if available)
        info!("Pressure: {:?}", pressure);
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
