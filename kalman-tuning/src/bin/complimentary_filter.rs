#![no_std]


pub struct ComplementaryFilter {
    angle: f32,  // Current filtered angle estimate
    rate: f32, 
    alpha: f32,  // Filter coefficient (typically between 0.90 and 0.98)
}

impl ComplementaryFilter {
    pub fn new(alpha: f32) -> Self {
        ComplementaryFilter {
            angle: 0.0,
            rate: 0.0,
            alpha,
        }
    }

    /// Updates the filter with new gyro and accelerometer data.
    ///
    /// - `gyro_x`: Angular velocity in degrees/sec from the gyroscope.
    /// - `accel_angle`: Angle in degrees estimated from the accelerometer.
    /// - `dt`: Time difference in seconds since the last update.
    pub fn update(&mut self, accel_angle: f32, gyro_x: f32, dt: f32) -> f32 {

        self.rate = gyro_x;
        
        // Integrate gyroscope to get angle estimate
        let gyro_angle = self.angle + gyro_x * dt;

        // Apply complementary filter
        self.angle = self.alpha * gyro_angle + (1.0 - self.alpha) * accel_angle;

        self.angle
    }

    pub fn update_no_input(&mut self, dt: f32) -> f32 {
        self.update(self.angle, self.rate, dt)
    }

    pub fn set_angle(&mut self, angle: f32) {
        self.angle = angle;
    }
}
