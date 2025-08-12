#![no_std]

pub struct KalmanFilter {
    angle: f32,   // Estimated angle
    bias: f32,    // Estimated gyro bias
    rate: f32,    // Unbiased rate

    p: [[f32; 2]; 2], // Error covariance matrix
    q_angle: f32,     // Process noise variance for angle
    q_bias: f32,      // Process noise variance for gyro bias
    r_measure: f32,   // Measurement noise variance (accelerometer)
}

impl KalmanFilter {
    pub fn new(q_angle: f32, q_bias: f32, r_measure: f32) -> Self {
        KalmanFilter {
            angle: 0.0,
            bias: 0.0,
            rate: 0.0,
            p: [[0.0; 2]; 2],
            q_angle,
            q_bias,
            r_measure,
        }
    }

    /// Call this every timestep to update the angle estimate.
    /// - `new_angle`: angle from accelerometer
    /// - `new_rate`: angular rate from gyroscope
    /// - `dt`: timestep in seconds
    pub fn update(&mut self, new_angle: f32, new_rate: f32, dt: f32) -> f32 {
        // Prediction step
        self.rate = new_rate - self.bias;
        self.angle += dt * self.rate;

        // Update error covariance matrix
        self.p[0][0] += dt * (dt*self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_angle);
        self.p[0][1] -= dt * self.p[1][1];
        self.p[1][0] -= dt * self.p[1][1];
        self.p[1][1] += self.q_bias * dt;

        // Measurement update
        let y = new_angle - self.angle; // innovation
        let s = self.p[0][0] + self.r_measure; // innovation covariance
        let k0 = self.p[0][0] / s;
        let k1 = self.p[1][0] / s;

        // Update angle and bias with Kalman gain
        self.angle += k0 * y;
        self.bias += k1 * y;

        // Update error covariance matrix
        let p00 = self.p[0][0];
        let p01 = self.p[0][1];

        self.p[0][0] -= k0 * p00;
        self.p[0][1] -= k0 * p01;
        self.p[1][0] -= k1 * p00;
        self.p[1][1] -= k1 * p01;

        self.angle
    }

    pub fn get_angle(&self) -> f32 {
        self.angle
    }
}
