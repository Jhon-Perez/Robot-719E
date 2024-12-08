use alloc::sync::Arc;
use core::time::Duration;

use vexide::{
    core::{println, sync::Mutex},
    devices::{smart::{imu::InertialError, motor::MotorError}, PortError},
    prelude::{sleep, spawn, Float, InertialSensor, RotationSensor},
};

use crate::vector::Vec2;

#[derive(Copy, Clone)]
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

const RADIUS: f64 = 2.75;

pub enum SensorError {
    PortError(PortError), // for rotation sensor, find better name
    #[allow(dead_code)]
    MotorError(MotorError), // create own error for pid and odom
    InertialSensor(InertialError),
}

impl From<PortError> for SensorError {
    fn from(e: PortError) -> Self {
        Self::PortError(e)
    }
}

impl From<MotorError> for SensorError {
    fn from(e: MotorError) -> Self {
        Self::MotorError(e)
    }
}

impl From<InertialError> for SensorError {
    fn from(e: InertialError) -> Self {
        Self::InertialSensor(e)
    }
}

pub struct Odometry {
    robot_pose: Arc<Mutex<Pose>>,
    x_rotation: RotationSensor,
    y_rotation: RotationSensor,
    imu_sensor: InertialSensor,

    prev_mid_val: f64,
    prev_left_val: f64,
    prev_theta: f64,
}

impl Odometry {
    pub fn new(
        robot_pose: Arc<Mutex<Pose>>,
        x_rotation: RotationSensor,
        y_rotation: RotationSensor,
        imu_sensor: InertialSensor,
    ) -> Self {
        Self {
            robot_pose,
            x_rotation,
            y_rotation,
            imu_sensor,

            prev_mid_val: 0.0,
            prev_left_val: 0.0,
            prev_theta: 0.0,
        }
    }

    async fn step(&mut self) -> Result<(), SensorError> {
        let mid_val = self.y_rotation.position()?.as_radians();
        let left_val = self.x_rotation.position()?.as_radians();

        let delta_m = (mid_val - self.prev_mid_val) * RADIUS;
        let delta_l = (left_val - self.prev_left_val) * RADIUS;
        let heading = 360.0 - self.imu_sensor.heading()?;

        self.prev_mid_val = mid_val;
        self.prev_left_val = left_val;

        let offset = self.step_math(delta_m, delta_l, heading.to_radians());

        let mut pose = self.robot_pose.lock().await;
        pose.position += offset;
        pose.heading = heading;

        println!("(x, y) = {:?}", pose.position);
        println!("ang = {}", pose.heading);

        drop(pose);

        Ok(())
    }

    fn step_math(&mut self, delta_m: f64, delta_l: f64, theta: f64) -> Vec2 {
        let delta_theta = theta - self.prev_theta;

        let delta_local_x;
        let delta_local_y;

        if delta_theta == 0.0 {
            delta_local_x = delta_m;
            delta_local_y = delta_l
        } else {
            delta_local_x = 2.0 * (delta_theta / 2.0).sin() * (delta_m / delta_theta + TM);
            delta_local_y = 2.0 * (delta_theta / 2.0).sin() * (delta_l / delta_theta + TL);
        };

        let avg_theta = theta + (delta_theta / 2.0);

        let polar_radius = (delta_local_x.powi(2) + delta_local_y.powi(2)).sqrt();
        let polar_theta = delta_local_y.atan2(delta_local_x) - avg_theta;

        Vec2::from_polar(polar_radius, polar_theta)
    }
}

pub fn start(mut odom: Odometry) {
    spawn(async move {
        _ = odom.x_rotation.reset_position();
        _ = odom.y_rotation.reset_position();

        loop {
            if let Err(e) = odom.step().await {
                match e {
                    SensorError::PortError(e) => {
                        println!("Port error: {:?}", e);
                    }
                    SensorError::InertialSensor(e) => {
                        println!("Inertial sensor error: {:?}", e);
                    }
                    SensorError::MotorError(_) => unreachable!(),
                }
                return;
            }

            sleep(Duration::from_millis(10)).await;
        }
    })
    .detach();
}
