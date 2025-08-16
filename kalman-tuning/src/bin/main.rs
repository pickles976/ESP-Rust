#![no_std]
#![no_main]

use esp_hal::{
    clock::{self, CpuClock}, i2c::master::I2c, main, time::{Duration, Instant, Rate}, Blocking
};

use esp_println::println;

use icm42670::{accelerometer::vector::F32x3, prelude::*, Address, Icm42670, AccelOdr, GyroOdr};

use core::{f32::consts::PI};

use heapless::String;

mod kalman_filter;
use kalman_filter::KalmanFilter;

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Something went wrong! Restarting!");
    println!("{:?}", info);
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

const LOOP_PERIOD_MILLIS: u8 = 10;

fn init_imu(i2c: &mut I2c<'static, Blocking>) -> Result<Icm42670<I2c<'static, Blocking>>, &'static str> {

    let res = Icm42670::new(i2c, Address::Primary);
    if res.is_err() {
        println!("{:?}", res.unwrap_err());
        return Err("Issue connecting to IMU");
    }

    let mut imu = res.unwrap();

    // Check what the original ODR was
    let res = imu.accel_odr();
    if res.is_err() {
        println!("{:?}", res.unwrap_err());
        return Err("Issue reading Accelerometer ODR");
    }
    println!("{:?}", res.unwrap());

    let res = imu.gyro_odr();
    if res.is_err() {
        println!("{:?}", res.unwrap_err());
        return Err("Issue reading Gyroscope ODR");
    }
    println!("{:?}", res.unwrap());


    // Set ODR to 800 Hz for fast reading
    let res = imu.set_accel_odr(AccelOdr::Hz800);
    if res.is_err() {
        println!("{:?}", res.unwrap_err());
        return Err("Issue setting Accelerometer ODR");
    }

    let res = imu.set_gyro_odr(GyroOdr::Hz800);
    if res.is_err() {
        println!("{:?}", res.unwrap_err());
        return Err("Issue setting Gyro ODR");
    }

    return imu
}

fn read_imu(imu: &mut Icm42670<I2c<'static, Blocking>>) -> Result<(F32x3, F32x3), &'static str> {
    // ~3ms

    // read from gyro
    let res_gyro = imu.gyro_norm();
    if res_gyro.is_err() {
        println!("{:?}", res_gyro.unwrap_err());
        return Err("Issue reading from Gyro");
    }

    let res_accel = imu.accel_norm();
    if res_accel.is_err() {
        println!("{:?}", res_accel.unwrap_err());
        return Err("Issue reading from Accelerometer");
    }

    return Ok((res_gyro.unwrap(), res_accel.unwrap()));
}

#[main]
fn main() -> ! {

    println!("Starting...");

    // Static string buffer
    let mut buf: String<64> = String::new();

    let gyro_offsets: F32x3 = (-0.272630, -0.366749, -0.692388).into();
    let x_var = 0.008859;
    let accel_var = 0.000006;
    let mut kf = KalmanFilter::new(gyro_offsets.x, x_var, accel_var); // tune these params

    // Init Peripherals
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize I2C
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = esp_hal::i2c::master::I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO10)
        .with_scl(peripherals.GPIO8);

    // Set up accelerometer (consumes accelerometer)
    let res = init_imu(&mut i2c).unwrap();

    let mut counter: i32 = 0;

    loop {

        let delay_start = Instant::now();

        let res= read_imu(&mut imu);

        match res {
            Ok((gyro, accel)) => {

                // Uncomment to print raw values

                // println!(
                //     "{:+.04},{:+.04},{:+.04}",
                //     gyro.x, gyro.y, gyro.z
                // );

                // println!(
                //     "{:+.04},{:+.04},{:+.04}",
                //     accel.x, accel.y, accel.z
                // );

                let dt = 1.0 / LOOP_PERIOD_MILLIS as f32;
                let gyro_x = gyro.x;

                let acc_angle = libm::atan2f(accel.y, accel.z); // lying flat
                let estimated_angle_radians = kf.update(acc_angle, gyro_x, dt );
                let estimated_angle: f32 = estimated_angle_radians * 180.0 / PI;

                println!("{:?},{:?},{:?},{:?}", 
                    (counter as f32 * dt), // timestamp
                    acc_angle * 180.0 / PI, // raw angle (degrees)
                    gyro_x * 180.0 / PI, // raw angular velocity (degrees)
                    estimated_angle); // Kalman filtered angle (degrees)

            },
            Err(string) => println!("{}", string)
        }

        counter += 1;

        while delay_start.elapsed() < Duration::from_millis(LOOP_PERIOD_MILLIS.into()) {}
    }
}