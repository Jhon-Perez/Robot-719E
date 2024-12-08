pub struct Pid {
    k_p: f64,
    k_i: f64,
    k_d: f64,
    integral_threshold: f64,
    prev_error: f64,
    integral: f64,
}

impl Pid {
    pub fn new(k_p: f64, k_i: f64, k_d: f64, integral_threshold: f64) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            integral_threshold,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn output(&mut self, error: f64) -> f64 {
        if error.abs() < self.integral_threshold {
            self.integral += error;
        } else {
            self.integral = 0.0;
        }

        let derivative = error - self.prev_error;
        self.prev_error = error;

        error * self.k_p + self.integral * self.k_i + derivative * self.k_d
    }
}
