# README

Stuff of note:
1. Find the `Sensitivity Scale Factor` for your gyroscope from the IMU datasheet
2. Do sanity tests on your gyro and accelerometer
3. Use pandas to find gyro bias and initial guesses for variance on measurement readings
4. Plot your raw angle, cumulative angle calculated from gyro, and kalman output. Kalman output should be smoother and more stable than raw angle from accelerometer, and should have minimal lag (10ms max)
5. Make sure your elapsed time for the control loop is small. Less than 10ms
6. Calibrate your motors, at what duty to they turn on?
7. Make sure there is a deadzone where motors turn off (2 degrees or so?)
8. Pick a stable point as your desired balance target
9. Use the correct voltage for your motors!!!
10. Be aware of vibrations! Try to prevent these as much as possible in your chassis design


```bash
cargo run --bin balancing-robot
```


https://github.com/esp-rs/esp-hal/tree/main/examples/peripheral

https://github.com/esp-rs/esp-rust-board?tab=readme-ov-file