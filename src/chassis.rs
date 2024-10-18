use alloc::{boxed::Box, sync::Arc};
use core::{
    f64::consts::{FRAC_PI_2, PI},
    time::Duration,
};

use vexide::{
    core::sync::Mutex,
    prelude::{sleep, BrakeMode, Controller, Float, Motor},
};

use crate::{odometry::Pose, pid::Pid, vector::Vec2};

const DIAMETER: f64 = 3.25;
const GEAR_RATIO: f64 = 0.8;
const DISTANCE_PER_REVOLUTION: f64 = DIAMETER * PI * GEAR_RATIO;

#[allow(dead_code)]
#[derive(PartialEq)]
pub enum JoystickType {
    ArcadeLeft,
    ArcadeRight,
    SplitArcade,
    Tank,
}

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

    joystick_type: JoystickType,
    is_cheesy: bool,

    // Cheesy drive implementation, maybe move to its own struct
    quick_stop_accumulator: f64,
    negative_inertia_accumulator: f64,
    prev_turn: f64,
    prev_power: f64,
}

impl Chassis {
    pub fn new(
        left_motors: Box<[Motor]>,
        right_motors: Box<[Motor]>,
        joystick_type: JoystickType,
        linear: Pid,
        angular: Pid,
    ) -> Self {
        Self {
            left_motors,
            right_motors,
            joystick_type,
            linear,
            angular,
            drive: TargetType::None,
            turn: TargetType::None,
            is_cheesy: false,
            quick_stop_accumulator: 0.0,
            negative_inertia_accumulator: 0.0,
            prev_turn: 0.0,
            prev_power: 0.0,
        }
    }

    pub fn differential_drive(&mut self, controller: &Controller) -> (f64, f64) {
        let mut power = 0.0;
        let mut turn = 0.0;
        let mut left = 0.0;
        let mut right = 0.0;

        match self.joystick_type {
            JoystickType::ArcadeLeft => {
                power = controller.left_stick.y().unwrap_or_default();
                turn = controller.left_stick.x().unwrap_or_default();
            }
            JoystickType::ArcadeRight => {
                power = controller.right_stick.y().unwrap_or_default();
                turn = controller.right_stick.x().unwrap_or_default();
            }
            JoystickType::SplitArcade => {
                power = controller.left_stick.y().unwrap_or_default();
                turn = controller.right_stick.x().unwrap_or_default();
            }
            JoystickType::Tank => {
                left = controller.left_stick.y().unwrap_or_default();
                right = controller.right_stick.y().unwrap_or_default();
            }
        }

        if self.is_cheesy {
            (left, right) = self.cheesy_drive(power, turn);
        } else if self.joystick_type != JoystickType::Tank {
            left = get_acceleration(power + turn, 1);
            right = get_acceleration(power - turn, 1);
        }

        (left * Motor::MAX_VOLTAGE, right * Motor::MAX_VOLTAGE)
    }

    pub async fn run(&mut self, pose: Arc<Mutex<Pose>>) {
        let mut time = 0u16;

        loop {
            let drive_error = match self.drive {
                TargetType::Coordinate(target) => {
                    let pose = pose.lock().await;
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
                    let pose = pose.lock().await;
                    (target.y - pose.position.y).atan2(target.x - pose.position.x)
                },
                TargetType::Distance(target) => {
                    let pose = pose.lock().await;
                    target - pose.heading
                }
                TargetType::None => 0.0,
            };

            if (drive_error.abs() < 0.5 && turn_error.abs() < 0.5) || time > 5000 {
                break;
            }

            let drive_output = self.linear.output(drive_error);
            let turn_output = self.angular.output(turn_error);

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

    // Cheesy drive implementation
    const CD_TURN_NON_LINEARITY: f64 = 0.5;
    const CD_NEGATIVE_INERTIA_SCLAR: f64 = 4.0;
    const CD_SENSITIVITY: f64 = 1.0;
    const DRIVER_SLEW: f64 = 0.1;
    const CONTROLLER_DEADZONE: f64 = 0.05;

    fn turn_remapping(&self, turn: f64) -> f64 {
        let denominator = (FRAC_PI_2 * Self::CD_TURN_NON_LINEARITY).sin();
        let first_map_iteration = (FRAC_PI_2 * Self::CD_TURN_NON_LINEARITY * turn).sin();
        (FRAC_PI_2 * Self::CD_TURN_NON_LINEARITY * first_map_iteration).sin() / denominator
    }

    fn update_accumulator(&mut self) {
        if self.negative_inertia_accumulator > 1.0 {
            self.negative_inertia_accumulator -= 1.0;
        } else if self.negative_inertia_accumulator < -1.0 {
            self.negative_inertia_accumulator += 1.0;
        } else {
            self.negative_inertia_accumulator = 0.0;
        }

        if self.quick_stop_accumulator > 1.0 {
            self.quick_stop_accumulator -= 1.0;
        } else if self.quick_stop_accumulator < -1.0 {
            self.quick_stop_accumulator += 1.0;
        } else {
            self.quick_stop_accumulator = 0.0;
        }
    }

    fn cheesy_drive(&mut self, power: f64, turn: f64) -> (f64, f64) {
        let mut turn_in_place = false;

        let linear =
            if power.abs() < Self::CONTROLLER_DEADZONE && turn.abs() > Self::CONTROLLER_DEADZONE {
                turn_in_place = true;
                0.0
            } else if power - self.prev_power > Self::DRIVER_SLEW {
                self.prev_power + Self::DRIVER_SLEW
            } else if power - self.prev_power < -Self::DRIVER_SLEW * 2.0 {
                self.prev_power - Self::DRIVER_SLEW * 2.0
            } else {
                power
            };

        let turn = self.turn_remapping(turn);

        if turn_in_place {
            (turn * turn.abs(), -turn * turn.abs())
        } else {
            let neg_intertia_power = (turn - self.prev_turn) * Self::CD_NEGATIVE_INERTIA_SCLAR;
            self.negative_inertia_accumulator += neg_intertia_power;

            let angular =
                linear.abs() * (turn - self.negative_inertia_accumulator) * Self::CD_SENSITIVITY
                    - self.quick_stop_accumulator;

            self.update_accumulator();

            (linear + angular, linear - angular)
        }
    }
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
