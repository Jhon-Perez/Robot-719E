use core::{
    f64::consts::TAU,
    time::Duration,
};

extern crate alloc;

use crate::vector::Vec2;

use alloc::sync::Arc;

use vexide::{
    core::sync::Mutex,
    prelude::{sleep, Float, InertialSensor, RotationSensor},
};

pub struct Pose {
    pub position: Vec2,
    pub heading: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Self {
            position: Vec2::new(x, y),
            heading,
        }
    }
}

const TL: f64 = 0.0;
const TM: f64 = 0.0;

// const RADIUS: f64 = 2.75;

fn get_delta_ticks(theta_curr: f64, theta_prev: f64) -> f64 {
    let delta_theta_clockwise;
    let delta_theta_counter_closewise;

    if theta_curr < theta_prev {
        delta_theta_clockwise = TAU - theta_prev + theta_curr;
        delta_theta_counter_closewise = -(theta_prev - theta_curr);
    } else {
        delta_theta_clockwise = theta_curr - theta_prev;
        delta_theta_counter_closewise = -(TAU - theta_curr + theta_curr);
    }

    if delta_theta_clockwise < delta_theta_counter_closewise.abs() {
        delta_theta_clockwise
    } else {
        delta_theta_counter_closewise
    }
}

pub async fn step_math(
    robot_pose: Arc<Mutex<Pose>>,
    x_rotation: RotationSensor,
    y_rotation: RotationSensor,
    imu_sensor: InertialSensor,
) {
    let mut prev_mid_val = x_rotation.position().unwrap().as_radians();
    let mut prev_left_val = y_rotation.position().unwrap().as_radians();
    let mut prev_theta = 0.0;

    loop {
        let mid_val = y_rotation.position().unwrap().as_radians();
        let left_val = x_rotation.position().unwrap().as_radians();

        let delta_m = get_delta_ticks(prev_mid_val, mid_val);
        let delta_l = get_delta_ticks(prev_left_val, left_val);

        let heading = imu_sensor.heading().unwrap_or_default();
        let theta = heading * TAU / 360.0;
        let delta_theta = theta - prev_theta;

        prev_left_val = left_val;
        prev_mid_val = mid_val;
        prev_theta = theta;

        let (delta_local_x, delta_local_y) = if delta_theta == 0.0 {
            (delta_m, delta_l)
        } else {
            (
                2.0 * (delta_theta / 2.0).sin() * (delta_m / delta_theta + TM),
                2.0 * (delta_theta / 2.0).sin() * (delta_l / delta_theta + TL),
            )
        };

        let avg_theta = theta + (delta_theta / 2.0);

        let polar_radius = (delta_local_x.powi(2) + delta_local_y.powi(2)).sqrt();
        // let polar_theta = (delta_local_y / delta_local_x).atan() - avg_theta;
        let polar_theta = delta_local_y.atan2(delta_local_x) - avg_theta;

        let mut pose = robot_pose.lock().await;
        pose.position += Vec2::from_polar(polar_radius, polar_theta);
        pose.heading = heading;

        drop(pose);

        sleep(Duration::from_millis(10)).await;
    }
}
