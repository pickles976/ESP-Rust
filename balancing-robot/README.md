# TODO:

- [x] build and run
- [x] blinky lights
- [x] serial print
- [x] communicate with display
    - [x] get I2C object building
    - [x] try to write a single character to the screen manually
    - [x] pass in to liquid crystal library
- [x] control H-bridge
    - [x] pwm
- [x] Kalman filter
- [x] get it to rotate when not flat
- [x] try connecting GPIO 4,5,6 for motor 2

- [x] solder this shit
- [x] CAD ROBOT BODY
- [x] print robot body

- [x] assemble rig and adjust pinouts so that wheels turn in the right direction (in the direction the robot is leaning)
- [x] set up test rig with string

- [x] implement feedback system for wheel controls
    - [x] PID control loop

- [ ] redesign car

- [ ] make a wheel calibration package
    - [ ] find the duty % when the motor actually turns on
    - [ ] take slow-mo video of speed at 70%, 80%, 90% and 100%
    - [ ] figure out "gear ratio" for small thetas (imagine the wheel is rotating inside a larger wheel with the IMU at the center)

- [ ] make a kalman filter analysis package
    - [ ] record raw rotation data
    - [ ] record raw angular momentum data
    - [ ] record kalman filter
    - [ ] plot all 3 to see delay
    - [ ] figure out how to reduce delay

- [ ] configure gyro

```bash
cargo run --bin balancing-robot
```

https://crates.io/crates/liquidcrystal_i2c-rs

https://github.com/esp-rs/esp-hal/tree/main/examples/peripheral

https://github.com/esp-rs/esp-rust-board?tab=readme-ov-file