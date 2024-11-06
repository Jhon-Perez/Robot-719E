use alloc::{boxed::Box, sync::Arc};
use core::{
    f64::consts::PI,
    time::Duration,
};

use vexide::{
    core::{println, sync::Mutex},
    prelude::{sleep, BrakeMode, Float, InertialSensor, Motor},
};

use crate::{mappings::{ControllerMappings, DriveMode}, odometry::Pose, pid::Pid, vector::Vec2};

const DIAMETER: f64 = 3.25;
const GEAR_RATIO: f64 = 36.0 / 48.0;
const DISTANCE_PER_REVOLUTION: f64 = DIAMETER * PI * GEAR_RATIO;

pub enum TargetType {
    Coordinate(Vec2),
    Distance(f64),
    None,
}

pub struct Chassis {
    left_motors: Box<[Motor]>,
    right_motors: Box<[Motor]>,

    // pid controller
    linear: Pid,
    angular: Pid,
    drive: TargetType,
    turn: TargetType,
    pose: Arc<Mutex<Pose>>,
    imu: InertialSensor,
}

impl Chassis {
    pub fn new(
        left_motors: Box<[Motor]>,
        right_motors: Box<[Motor]>,
        linear: Pid,
        angular: Pid,
        imu: InertialSensor
    ) -> Self {
        Self {
            left_motors,
            right_motors,
            linear,
            angular,
            drive: TargetType::None,
            turn: TargetType::None,
            imu,
            pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, 0.0))),
        }
    }

    pub fn differential_drive(&mut self, mappings: &ControllerMappings) -> (f64, f64) {
        let mut power_val = 0.0;
        let mut turn_val = 0.0;
        let mut left_val = 0.0;
        let mut right_val = 0.0;

        match mappings.drive_mode {
            DriveMode::Arcade { arcade } => {
                power_val = arcade.y().unwrap_or_default();
                turn_val = arcade.x().unwrap_or_default();
            }
            DriveMode::SplitArcade { power, turn } => {
                power_val = power.y().unwrap_or_default();
                turn_val = turn.x().unwrap_or_default();
            }
            DriveMode::Tank { left, right } => {
                left_val = left.y().unwrap_or_default();
                right_val = right.y().unwrap_or_default();
            }
        }

        match mappings.drive_mode {
            DriveMode::Tank { .. } => (),
            _ => {
                left_val = get_acceleration(power_val + turn_val, 1);
                right_val = get_acceleration(power_val - turn_val, 1);
            }
        }

        (left_val * Motor::MAX_VOLTAGE, right_val * Motor::MAX_VOLTAGE)
    }

    pub async fn run(&mut self) {
        let mut time = 0u16;

        loop {
            let drive_error = match self.drive {
                TargetType::Coordinate(target) => {
                    let pose = self.pose.lock().await;
                    target.euclidean_distance(&pose.position)
                }
                TargetType::Distance(target) => {
                    target
                        - self.left_motors[0].position().unwrap().as_revolutions()
                            * DISTANCE_PER_REVOLUTION
                }
                TargetType::None => 0.0,
            };

            let turn_error = match self.turn {
                TargetType::Coordinate(target) => {
                    let pose = self.pose.lock().await;
                    (target.y - pose.position.y).atan2(target.x - pose.position.x)
                },
                TargetType::Distance(target) => {
                    // let pose = self.pose.lock().await;
                    normalize_angle(
                        normalize_angle(target) - normalize_angle(self.imu.heading().unwrap())
                    )
                }
                TargetType::None => 0.0,
            };

            if /*(drive_error.abs() < 0.5 && turn_error.abs() < 0.5) ||*/ time > 5000 {
                break;
            }

            let drive_output = self.linear.output(drive_error);
            let turn_output = self.angular.output(turn_error);

            println!("{}", drive_output);
            println!("{}", turn_output);

            self.set_voltage((
                (drive_output + turn_output) * Motor::MAX_VOLTAGE,
                (drive_output - turn_output) * Motor::MAX_VOLTAGE,
            ));

            time += 10;
            sleep(Duration::from_millis(10)).await;
        }

        self.brake(BrakeMode::Coast);
    }

    pub fn set_drive_target(&mut self, target: TargetType) {
        self.drive = target;
    }

    pub fn set_turn_target(&mut self, target: TargetType) {
        self.turn = target;
    }

    pub fn set_voltage(&mut self, power: (f64, f64)) {
        for motor in self.left_motors.iter_mut() {
            motor.set_voltage(power.0).ok();
        }
        for motor in self.right_motors.iter_mut() {
            motor.set_voltage(power.1).ok();
        }
    }

    pub fn brake(&mut self, mode: BrakeMode) {
        for motor in self.left_motors.iter_mut() {
            motor.brake(mode).ok();
        }
        for motor in self.right_motors.iter_mut() {
            motor.brake(mode).ok();
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

fn get_acceleration(power: f64, acceleration: i32) -> f64 {
    if acceleration == 1 {
        return power;
    }

    power.powi(acceleration - 1) / 1.0.powi(acceleration - 1)
        * if acceleration % 2 == 0 {
            power.abs()
        } else {
            power
        }
}
