use alloc::vec::Vec;
use core::f32::consts::FRAC_PI_2;

use vexide::prelude::{Controller, Float, Motor};

#[allow(dead_code)]
#[derive(PartialEq)]
pub enum JoystickType {
    ArcadeLeft,
    ArcadeRight,
    SplitArcade,
    Tank,
}

pub struct Chassis {
    pub left_motors: Vec<Motor>,
    pub right_motors: Vec<Motor>,

    joystick_type: JoystickType,
    is_cheesy: bool,

    // Cheesy drive implementation, maybe move to its own struct
    quick_stop_accumulator: f32,
    negative_inertia_accumulator: f32,
    prev_turn: f32,
    prev_power: f32,
}

impl Chassis {
    pub fn new(
        left_motors: Vec<Motor>,
        right_motors: Vec<Motor>,
        joystick_type: JoystickType,
    ) -> Self {
        Self {
            left_motors,
            right_motors,
            joystick_type,
            is_cheesy: false,
            quick_stop_accumulator: 0.0,
            negative_inertia_accumulator: 0.0,
            prev_turn: 0.0,
            prev_power: 0.0,
        }
    }

    pub fn set_joystick_type(&mut self, joystick_type: JoystickType) {
        self.joystick_type = joystick_type;
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

        (
            left as f64 * Motor::MAX_VOLTAGE,
            right as f64 * Motor::MAX_VOLTAGE,
        )
    }

    // Implement error handling
    pub fn set_motors(&mut self, power: (f64, f64)) {
        for motor in self.left_motors.iter_mut() {
            motor.set_voltage(power.0).unwrap();
        }
        for motor in self.right_motors.iter_mut() {
            motor.set_voltage(power.1).unwrap();
        }
    }

    // Cheesy drive implementation
    const CD_TURN_NON_LINEARITY: f32 = 0.5;
    const CD_NEGATIVE_INERTIA_SCLAR: f32 = 4.0;
    const CD_SENSITIVITY: f32 = 1.0;
    const DRIVER_SLEW: f32 = 0.1;
    const CONTROLLER_DEADZONE: f32 = 0.05;

    fn turn_remapping(&self, turn: f32) -> f32 {
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

    fn cheesy_drive(&mut self, power: f32, turn: f32) -> (f32, f32) {
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

fn get_acceleration(power: f32, acceleration: i32) -> f32 {
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
