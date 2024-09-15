use core::{
    f64::consts::TAU,
    time::Duration,
};

extern crate alloc;

use alloc::sync::Arc;

use vexide::{
    core::sync::Mutex,
    prelude::{sleep, Float, InertialSensor, Position, RotationSensor},
};

pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Self { x, y, heading }
    }
}

const TL: f64 = 0.0;
const TM: f64 = 0.0;

// const RADIUS: f64 = 2.75;

// reformat this when able to
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
    x_rotation: Arc<RotationSensor>,
    y_rotation: Arc<RotationSensor>,
    imu_sensor: Arc<InertialSensor>,
) {
    let mut prev_mid_val = x_rotation.position().unwrap().as_radians();
    let mut prev_left_val = y_rotation.position().unwrap().as_radians();
    let mut prev_theta = 0.0;

    loop {
        let mid_val = y_rotation.position().unwrap().as_radians();
        let left_val = x_rotation.position().unwrap().as_radians();

        let delta_m = get_delta_ticks(prev_mid_val, mid_val);
        let delta_l = get_delta_ticks(prev_left_val, left_val);

        let theta = Position::from_degrees(imu_sensor.heading().unwrap_or_default()).as_radians();
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
        pose.x += polar_radius * polar_theta.cos();
        pose.y += polar_radius * polar_theta.sin();
        pose.heading = theta;

        drop(pose);

        sleep(Duration::from_millis(10)).await;
    }
}
