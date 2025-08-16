#![no_std]

pub struct Controller {
    target_value: f32,

    k_p: f32,
    k_i: f32, 
    k_d: f32,

    e_t: f32,
    e_i: f32,
    e_d: f32,
}

impl Controller {
    pub fn new(target_value: f32, k_p: f32, k_i: f32, k_d: f32) -> Self {
        Controller {
            target_value: target_value,
            k_p: k_p,
            k_i: k_i,
            k_d: k_d,
            e_t: 0.0,
            e_i: 0.0,
            e_d: 0.0
        }
    }

    pub fn update(&mut self, actual_value: f32, dt: f32) -> f32 {
        let err = self.target_value - actual_value;
        self.e_i += err;
        self.e_d = (err - self.e_t) / dt;
        self.e_t = err;

        let proportional = self.k_p * self.e_t;
        let integration = self.k_i * self.e_i;
        let derivative = self.k_d * self.e_d;

        return proportional + integration + derivative
    }
}