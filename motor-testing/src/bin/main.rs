#![no_std]
#![no_main]

use esp_hal::{
    clock::{self, CpuClock}, delay::Delay, gpio::{Level, Output, OutputConfig, OutputPin, Pin}, i2c::master::I2c, ledc::channel::Channel, main, time::{Duration, Instant, Rate}, Blocking
};

use esp_hal::ledc::{Ledc, LSGlobalClkSource, LowSpeed};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::channel::{self, ChannelIFace};

use esp_println::println;

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    println!("{:?}", info);
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

const LOOP_PERIOD_MILLIS: u64 = 1000;

fn set_motor(duty_pct: u8, forward: bool, pwm_channel: &mut Channel<'_, LowSpeed>, forward_pin: &mut Output, backward_pin: &mut Output) {
    if forward {
        forward_pin.set_high();
        backward_pin.set_low();
    } else {
        forward_pin.set_low();
        backward_pin.set_high();
    }
    pwm_channel.set_duty(duty_pct);
}

#[main]
fn main() -> ! {

    println!("Starting...");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Configure LEDC
    let mut ledc: Ledc<'_> = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // PWM TIMER
    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(1),
    }).unwrap();

    // MOTOR A
    // GPIO0 motor A pwm
    // GPIO1 motor A forward
    // GPIO3 motor A backward
    let pwm_pin_a = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    let mut forward_a = Output::new(peripherals.GPIO1, Level::High, OutputConfig::default());
    let mut backward_a = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    let mut pwm_a = ledc.channel(channel::Number::Channel0, pwm_pin_a);
    pwm_a.configure(channel::config::Config {
        timer: &lstimer,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }).unwrap();

    // MOTOR B
    // GPIO4 motor B pwm
    // GPIO5 motor B forward
    // GPIO6 motor B backward
    let pwm_pin_b = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let mut forward_b = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());
    let mut backward_b = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());

    let mut pwm_b = ledc.channel(channel::Number::Channel1, pwm_pin_b);
    pwm_b.configure(channel::config::Config {
        timer: &lstimer,
        duty_pct: 0,
        pin_config: channel::config::PinConfig::PushPull,
    }).unwrap();

    let mut duty = 50;

    loop {

        duty += 1;
        println!("{}", duty);

        let delay_start = Instant::now();

        set_motor(duty, false, &mut pwm_a, &mut forward_a, &mut backward_a); // 60
        // set_motor(duty, false, &mut pwm_b, &mut forward_b, &mut backward_b); // 66

        while delay_start.elapsed() < Duration::from_millis(LOOP_PERIOD_MILLIS.into()) {}
    }
}