#![no_std]

pub struct ComplementaryFilter {
    angle: f32, // in radians
    alpha: f32, // complementary filter coefficient
}

impl ComplementaryFilter {
    /// Create a new complementary filter for angle estimation.
    pub fn new(alpha: f32) -> Self {
        Self {
            angle: 0.0,
            alpha,
        }
    }

    /// Update the angle estimate using accelerometer and gyroscope data.
    ///
    /// - `gyro_y`: Gyroscope Y-axis (angle rate) in degrees/sec
    /// - `dt`: Time delta in seconds
    pub fn update(&mut self, angle: f32, gyro_y: f32, dt: f32) -> f32 {

        // Integrate gyro to estimate angle (convert to radians/sec)
        let gyro_angle_rate = gyro_y.to_radians();
        self.angle += gyro_angle_rate * dt;

        // Complementary filter
        self.angle = self.alpha * self.angle + (1.0 - self.alpha) * angle;

        return self.angle
    }


}
