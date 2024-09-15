use core::time::Duration;

use vexide::prelude::*;

use crate::Robot;

const DIAMETER: f64 = 3.25;
const GEAR_RATIO: f64 = 1.75;

const DISTANCE_PER_DEGREE: f64 = DIAMETER * core::f64::consts::PI / GEAR_RATIO;

pub trait Pid {
    fn run(&mut self, robot: &mut Robot, target: f64, time_out: u16);
}

struct BasePid {
    k_p: f64,
    k_i: f64,
    k_d: f64,
    integral_threshold: f64,
    prev_error: f64,
    integral: f64,
}

impl BasePid {
    fn new(k_p: f64, k_i: f64, k_d: f64, integral_threshold: f64) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            integral_threshold,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    fn get_output(&mut self, error: f64) -> f64 {
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

pub struct LinearPid {
    base: BasePid,
    target: f64,
}

impl LinearPid {
    pub fn new(k_p: f64, k_i: f64, k_d: f64, integral_threshold: f64) -> Self {
        Self {
            base: BasePid::new(k_p, k_i, k_d, integral_threshold),
            target: 0.0,
        }
    }

    fn get_error(&self, motor: &Motor) -> f64 {
        self.target - motor.position().unwrap().as_degrees() * DISTANCE_PER_DEGREE
    }
}

impl Pid for LinearPid {
    fn run(&mut self, robot: &mut Robot, target: f64, time_out: u16) {
        self.target = target;
        let mut time = 0;

        loop {
            let error = self.get_error(&robot.chassis.left_motors[0]);

            if error.abs() < 1.0 || time > time_out {
                break;
            }

            let power = self.base.get_output(error);

            robot.chassis.set_motors((power, power));

            sleep(Duration::from_millis(10));
            time += 10;
        }

        robot.chassis.set_motors((0.0, 0.0));
    }
}

pub struct AngularPid {
    base: BasePid,
    target: f64,
}

impl AngularPid {
    pub fn new(k_p: f64, k_i: f64, k_d: f64, integral_threshold: f64) -> Self {
        Self {
            base: BasePid::new(k_p, k_i, k_d, integral_threshold),
            target: 0.0,
        }
    }

    fn get_error(&self, imu_sensor: &InertialSensor) -> f64 {
        normalize_angle(self.target - normalize_angle(imu_sensor.heading().unwrap()))
    }
}

impl Pid for AngularPid {
    fn run(&mut self, robot: &mut Robot, target: f64, time_out: u16) {
        self.target = target;
        let mut time = 0;

        loop {
            let error = self.get_error(&robot.imu_sensor);

            if error.abs() < 1.0 || time > time_out {
                break;
            }

            let power = self.base.get_output(error);

            robot.chassis.set_motors((power, -power));

            sleep(Duration::from_millis(10));
            time += 10;
        }
    }
}

fn normalize_angle(mut angle: f64) -> f64 {
    if angle > 180.0 {
        angle -= 360.0;
    } else if angle < -180.0 {
        angle += 360.0;
    }

    angle
}
