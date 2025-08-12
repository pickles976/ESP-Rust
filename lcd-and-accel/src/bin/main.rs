#![no_std]
#![no_main]

use esp_hal::{
    clock::{self, CpuClock}, delay::Delay, gpio::{Level, Output, OutputConfig, OutputPin, Pin}, i2c::master::I2c, ledc::channel::Channel, main, time::{Duration, Instant, Rate}, Blocking
};

use esp_hal::ledc::{Ledc, LSGlobalClkSource, LowSpeed};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::peripherals::Peripherals;

use liquid_crystal::{prelude::*};
use liquid_crystal::LiquidCrystal;
use liquid_crystal::I2C;

use esp_println::println;

use icm42670::{accelerometer::vector::F32x3, prelude::*, Address, Icm42670};

use heapless::String;
use core::{fmt::Write, pin};

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    println!("{:?}", info);
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

const LCD_ADDRESS: u8 = 0x27;
const LOOP_PERIOD_MILLIS: u8 = 100;

fn read_accelerometer(i2c: &mut I2c<'_, Blocking>) -> Result<F32x3, &'static str> {
    /**
     * ~3ms
     */

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

fn initialize_lcd(i2c: &mut I2c<'_, Blocking>){
    let mut delay = Delay::new();

    let mut i2c_lcd = I2C::new(i2c, LCD_ADDRESS);
    let mut lcd = LiquidCrystal::new(&mut i2c_lcd, Bus4Bits, LCD16X2);
    lcd.begin(&mut delay);

}

fn write_lcd(i2c: &mut I2c<'_, Blocking>, message: &str) {
    let mut delay = Delay::new();

    let mut i2c_lcd = I2C::new(i2c, LCD_ADDRESS);
    let mut lcd = LiquidCrystal::new(&mut i2c_lcd, Bus4Bits, LCD16X2);

    lcd.write(&mut delay,Command(Clear))
        .write(&mut delay,Text(message));

}

fn set_motor(duty_pct: u8, forward: bool, pwm_channel: &mut Channel<'_, LowSpeed>, forward_pin: &mut Output, backward_pin: &mut Output) {
    if forward {
        forward_pin.set_high();
        backward_pin.set_low();
    } else {
        forward_pin.set_low();
        backward_pin.set_high();
    }
    pwm_channel.set_duty(duty_pct).unwrap();
}

#[main]
fn main() -> ! {

    println!("Starting...");

    let mut buf: String<64> = String::new();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let gyro_offsets: F32x3 = (-0.272630, -0.366749, -0.692388).into();

    // Initialize I2C
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(200));
    let mut i2c = esp_hal::i2c::master::I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO10)
        .with_scl(peripherals.GPIO8);

    // Initialize LCD
    initialize_lcd(&mut i2c);

    // Set GPIO0 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default());

    // Configure LEDC
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // let channel_b = configure_motor_b(&mut ledc, peripherals.GPIO0);
    // channel_b.set_duty(90);

    // MOTOR B
    // GPIO0 motor B pwm
    // GPIO1 motor B forward
    // GPIO3 motor B backward

    let pwm_pin_b = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    let mut forward_b = Output::new(peripherals.GPIO1, Level::High, OutputConfig::default());
    let mut backward_b = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(1),
    }).unwrap();

    let mut pwm_b = ledc.channel(channel::Number::Channel0, pwm_pin_b);
    pwm_b.configure(channel::config::Config {
        timer: &lstimer,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }).unwrap();


    loop {

        let delay_start = Instant::now();

        led.toggle();

        let res= read_accelerometer(&mut i2c);

        match res {
            Ok(gyro) => {
                let gyro_x = gyro.x - gyro_offsets.x;
                buf.clear();
                write!(
                    &mut buf,
                    "G X: {:+.2}",
                    gyro_x
                ).unwrap();
                write_lcd(&mut i2c, &buf);

                if gyro_x > 5.0 {
                    set_motor(80, true, &mut pwm_b, &mut forward_b, &mut backward_b);
                } else if gyro_x < -5.0 {
                    set_motor(80, false, &mut pwm_b, &mut forward_b, &mut backward_b);
                } else {
                    set_motor(0, true, &mut pwm_b, &mut forward_b, &mut backward_b);
                }

            },
            Err(string) => println!("{}", string)
        }

        // lcd.write(&mut delay, Command(Clear))
        //     .write(&mut delay, Text(&buf));
        
        // println!(
        //     "ACCEL  =  X: {:+.04} Y: {:+.04} Z: {:+.04}\t\tGYRO  =  X: {:+.04} Y: {:+.04} Z: {:+.04}",
        //     accel_norm.x, accel_norm.y, accel_norm.z, gyro_norm.x, gyro_norm.y, gyro_norm.z
        // );


        while delay_start.elapsed() < Duration::from_millis(LOOP_PERIOD_MILLIS.into()) {}
    }
}