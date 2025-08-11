#![no_std]
#![no_main]

use esp_hal::{
    clock::CpuClock, delay::Delay, gpio::{Level, Output, OutputConfig}, i2c::master::I2c, main, time::{Duration, Instant, Rate}, Blocking
};

use liquid_crystal::{prelude::*};
use liquid_crystal::LiquidCrystal;
use liquid_crystal::I2C;

use esp_println::println;

use icm42670::{accelerometer::vector::F32x3, prelude::*, Address, Icm42670};

use heapless::String;
use core::fmt::Write;

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    println!("{:?}", info);
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

fn read_accelerometer(i2c: &mut I2c<'_, Blocking>) -> Result<F32x3, &'static str> {

    // Set up accelerometer
    let res = Icm42670::new(i2c, Address::Primary);
    if res.is_err() {
        println!("{:?}", res.unwrap_err());
        return Err("Issue initializing accelerometer");
    }
    let mut icm = res.unwrap();

    // read from gyro
    let res = icm.gyro_norm();
    if res.is_err() {
        println!("{:?}", res.unwrap_err());
        return Err("Issue reading from Gyro");
    }
    return Ok(res.unwrap());
}

fn write_lcd(i2c: &mut I2c<'_, Blocking>, message: &str) {

    let mut delay = Delay::new();

    let mut i2c_lcd = I2C::new(i2c, 0x27);
    let mut lcd = LiquidCrystal::new(&mut i2c_lcd, Bus4Bits, LCD16X2);
    lcd.write(&mut delay,Command(Clear))
        .write(&mut delay,Text(message));
}


#[main]
fn main() -> ! {

    println!("Starting...");

    let mut buf: String<64> = String::new();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize I2C
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(200));
    let mut i2c = esp_hal::i2c::master::I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO10)
        .with_scl(peripherals.GPIO8);

    // Set GPIO0 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default());

    loop {
        led.toggle();
        println!("Hello world");

        let res= read_accelerometer(&mut i2c);

        match res {
            Ok(gyro) => {
                buf.clear();
                write!(
                    &mut buf,
                    "G Y: {:+.2}",
                    gyro.y
                ).unwrap();
                write_lcd(&mut i2c, &buf);
            },
            Err(string) => println!("{}", string)
        }


        // write_lcd(&mut i2c, "fef");

        // counter += 1;

        // lcd.write(&mut delay, Command(Clear))
        //     .write(&mut delay, Text(&buf));
        
        // println!(
        //     "ACCEL  =  X: {:+.04} Y: {:+.04} Z: {:+.04}\t\tGYRO  =  X: {:+.04} Y: {:+.04} Z: {:+.04}",
        //     accel_norm.x, accel_norm.y, accel_norm.z, gyro_norm.x, gyro_norm.y, gyro_norm.z
        // );

        // Wait for half a second
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(100) {}
    }
}