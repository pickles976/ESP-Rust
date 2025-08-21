#![no_std]
#![no_main]

use esp_hal::{
    clock::{self, CpuClock}, delay::Delay, gpio::{Level, Output, OutputConfig, OutputPin, Pin}, i2c::master::I2c, ledc::channel::Channel, main, time::{Duration, Instant, Rate}, Blocking
};

use esp_hal::ledc::{Ledc, LSGlobalClkSource, LowSpeed};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::channel::{self, ChannelIFace};

use esp_println::println;

use icm42670::{accelerometer::vector::F32x3, prelude::*, Address, Icm42670, AccelOdr, GyroOdr, AccelRange, GyroRange};

use core::{cmp::{max, min}, f32::consts::PI, fmt::Write, pin};

mod kalman_filter;
use kalman_filter::KalmanFilter;

use libm::roundf;

mod pid;
use pid::Controller;

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    println!("{:?}", info);
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

const LOOP_PERIOD_MILLIS: u8 = 3;
const TARGET_ANGLE: f32 = -94.0;

const DUTY_MIN_A: i32 = 70 - 25;
const DUTY_MIN_B: i32 = 66 - 25;
const DUTY_MAX: i32 = 100;

const K_P: f32 = 0.2;
const K_I: f32 = 0.0;
const K_D: f32 = 0.001;

const DEADZONE: f32 = 2.0;

const MAX_ANGLE: f32 = 75.0;


const GYRO_SENSITIVITY: f32 = 131.0;


pub fn lerp_i32(a: i32, b: i32, t: f32) -> i32 {
    roundf((1.0 - t) * a as f32 + t * b as f32) as i32
}


fn init_imu<'a>(i2c: &'a mut I2c<'static, Blocking>) -> Result<Icm42670<&'a mut I2c<'static, Blocking>>, &'static str> {

    let res = Icm42670::new(i2c, Address::Primary);
    if res.is_err() {
        // println!("{:?}", res.unwrap_err());
        return Err("Issue connecting to IMU");
    }

    let mut imu = res.unwrap();

    // Check what the original ODR was
    let res = imu.accel_odr();
    if res.is_err() {
        // println!("{:?}", res.unwrap_err());
        return Err("Issue reading Accelerometer ODR");
    }
    // println!("{:?}", res.unwrap());

    let res = imu.gyro_odr();
    if res.is_err() {
        // println!("{:?}", res.unwrap_err());
        return Err("Issue reading Gyroscope ODR");
    }
    // println!("{:?}", res.unwrap());


    // Set ODR to 800 Hz for fast reading
    let res = imu.set_accel_odr(AccelOdr::Hz1600);
    if res.is_err() {
        // println!("{:?}", res.unwrap_err());
        return Err("Issue setting Accelerometer ODR");
    }

    imu.set_accel_range(AccelRange::G4);

    let res = imu.set_gyro_odr(GyroOdr::Hz1600);
    if res.is_err() {
        // println!("{:?}", res.unwrap_err());
        return Err("Issue setting Gyro ODR");
    }

    imu.set_gyro_range(GyroRange::Deg250);

    return Ok(imu)
}

fn read_imu(imu: &mut Icm42670<&mut I2c<'static, Blocking>>) -> Result<(F32x3, F32x3), &'static str> {
    // ~3ms

    // read from gyro
    let res_gyro = imu.gyro_norm();
    if res_gyro.is_err() {
        // println!("{:?}", res_gyro.unwrap_err());
        return Err("Issue reading from Gyro");
    }

    let res_accel = imu.accel_norm();
    if res_accel.is_err() {
        // println!("{:?}", res_accel.unwrap_err());
        return Err("Issue reading from Accelerometer");
    }

    return Ok((res_gyro.unwrap(), res_accel.unwrap()));
}

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
    
    let gyro_mean: f32 = -0.3;
    let theta_var = 0.0001;
    let gyro_bias = 0.03;
    let r_measure = 0.0002; // Variance in angle measurement from accelerometer
    let mut kf = KalmanFilter::new(theta_var, gyro_bias, r_measure); // tune these params

    let mut controller = Controller::new(TARGET_ANGLE, K_P, K_I, K_D);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize I2C
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(1000));
    let mut i2c = esp_hal::i2c::master::I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO10)
        .with_scl(peripherals.GPIO8);

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


    // Set up accelerometer (consumes accelerometer)
    let mut imu = init_imu(&mut i2c).unwrap();

    let dt = 1.0 / (1000.0 / LOOP_PERIOD_MILLIS as f32);
    let mut counter: i32 = 0;

    let alpha = 0.3;
    let mut acc_angle_last = PI / 2.0;

    loop {

        let delay_start = Instant::now();

        let res= read_imu(&mut imu);

        match res {
            Ok((gyro, accel)) => {

                let gyro_x = (gyro.x - gyro_mean) / GYRO_SENSITIVITY;

                let mut acc_angle = libm::atan2f(accel.y, accel.z); // lying flat

                acc_angle = alpha * acc_angle + (1.0 - alpha) * acc_angle_last;
                acc_angle_last = acc_angle;


                let estimated_angle_radians = kf.update(acc_angle, gyro_x, dt );
                let estimated_angle: f32 = estimated_angle_radians * 180.0 / PI;

                // println!("Angle: {:?}", estimated_angle - TARGET_ANGLE);

                let output = controller.update(estimated_angle, dt);
                let abs_output= output.abs();

                let duty_a = min(DUTY_MAX, lerp_i32(DUTY_MIN_A, DUTY_MAX, abs_output)) as u8;
                let duty_b = min(DUTY_MAX, lerp_i32(DUTY_MIN_B, DUTY_MAX, abs_output)) as u8;

                // println!("PID OUTPUT: {:?}", output);
                // println!("DUTY PCT A: {:?}", duty_a);
                // println!("DUTY PCT B: {:?}", duty_b);

                if (estimated_angle - TARGET_ANGLE).abs() < DEADZONE {
                    set_motor(0, true, &mut pwm_a, &mut forward_a, &mut backward_a);
                    set_motor(0,  true, &mut pwm_b, &mut forward_b, &mut backward_b);
                } else {
                    set_motor(duty_a, output > 0.0, &mut pwm_a, &mut forward_a, &mut backward_a);
                    set_motor(duty_b,  output > 0.0, &mut pwm_b, &mut forward_b, &mut backward_b);
                }

                // println!("{}", output);

                println!("{:?},{:?},{:?},{:?}", 
                    (counter as f32 * dt), // timestamp
                    (acc_angle * 180.0 / PI) - TARGET_ANGLE, // raw angle (degrees)
                    gyro_x, // raw angular velocity (degrees)
                    estimated_angle - TARGET_ANGLE); // Kalman filtered angle (degrees)

            },
            Err(string) => {
                kf.update_no_input(dt);
            }
        }

        // println!("Elapsed: {}", delay_start.elapsed());
        counter += 1;

        while delay_start.elapsed() < Duration::from_millis(LOOP_PERIOD_MILLIS.into()) {}
    }
}