#![no_std]
#![no_main]

use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    main,
    time::{Duration, Instant, Rate},
    i2c::master::{I2c},
    delay::Delay
};


use esp_println::println;
use icm42670::{prelude::*, Address, Icm42670};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let delay = Delay::new();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let i2c = I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO10)
        .with_scl(peripherals.GPIO8);

    let mut icm = Icm42670::new(i2c, Address::Primary).unwrap();

    loop {
        let gyro_norm = icm.gyro_norm().unwrap();
        let accel_norm = icm.accel_norm().unwrap();

        // println!(
        //     "{:+.04},{:+.04},{:+.04}",
        //     gyro_norm.x, gyro_norm.y, gyro_norm.z
        // );

        println!(
            "{:+.04},{:+.04},{:+.04}",
            accel_norm.x, accel_norm.y, accel_norm.z
        );


        delay.delay_millis(100u32);
    }
}