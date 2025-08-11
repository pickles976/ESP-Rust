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

use liquid_crystal::{prelude::*};
use liquid_crystal::LiquidCrystal;
use liquid_crystal::I2C;

use esp_println::println;

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {

    let mut delay = Delay::new();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize I2C
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO10)
        .with_scl(peripherals.GPIO8);


    // Init LCD controller (https://github.com/RecursiveError/liquid_crystal/blob/main/exemples/esp32/hello_i2c.rs)
    const LCD_ADDR: u8 = 0x27;
    let mut i2c_lcd = I2C::new(i2c, LCD_ADDR);
    let mut lcd = LiquidCrystal::new(&mut i2c_lcd, Bus4Bits, LCD16X2);
    lcd.begin(&mut delay);
    lcd.write(&mut delay,Command(Clear))
        .write(&mut delay,Text("hello World!"));



    // Set GPIO0 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default());

    loop {
        led.toggle();
        println!("Hello world");
        // Wait for half a second
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }
}