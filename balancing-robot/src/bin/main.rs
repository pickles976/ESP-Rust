#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::delay::Delay;
use esp_hal::time::{Rate};

use liquid_crystal::{prelude::*};
use liquid_crystal::LiquidCrystal;
use liquid_crystal::I2C;

use esp_println::println;

use icm42670::{prelude::*, Address, Icm42670};
use shared_bus::BusManagerSimple;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn hello_world() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

    // Initialize I2C
    let mut delay = Delay::new();

    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let i2c = esp_hal::i2c::master::I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO10)
        .with_scl(peripherals.GPIO8);

    let bus = BusManagerSimple::new(i2c);
    let mut proxy1 = bus.acquire_i2c();
    let mut icm = Icm42670::new(proxy1, Address::Primary).unwrap();

    // // Init LCD controller (https://github.com/RecursiveError/liquid_crystal/blob/main/exemples/esp32/hello_i2c.rs)
    // const LCD_ADDR: u8 = 0x27;
    // let mut i2c_lcd = I2C::new(i2c, LCD_ADDR);
    // let mut lcd = LiquidCrystal::new(&mut i2c_lcd, Bus4Bits, LCD16X2);
    // lcd.begin(&mut delay);
    // lcd.write(&mut delay,Command(Clear))
    //     .write(&mut delay,Text("Goodbye World!"));

    // TODO: Spawn some tasks
    spawner.spawn(hello_world()).ok();

    loop {
        Timer::after(Duration::from_secs(1)).await;
        println!("serial test");
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}
