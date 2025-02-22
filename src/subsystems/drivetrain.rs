use vexide::prelude::{Float, Motor};

use crate::mappings::{ControllerMappings, DriveMode};

fn get_acceleration(power: f64, acceleration: i32) -> f64 {
    if acceleration == 1 {
        return power;
    }

    power.powi(acceleration - 1)
        * if acceleration % 2 == 0 {
            power.abs()
        } else {
            power
        }
}

pub fn differential_drive(mappings: &ControllerMappings) -> (f64, f64) {
    let mut power_val = 0.0;
    let mut turn_val = 0.0;
    let mut left_val = 0.0;
    let mut right_val = 0.0;

    match mappings.drive_mode {
        DriveMode::Arcade { power, turn } => {
            power_val = power.y();
            turn_val = turn.x();
        }
        DriveMode::Tank { left, right } => {
            left_val = left.y();
            right_val = right.y();
        }
    }

    if !matches!(mappings.drive_mode, DriveMode::Tank { .. }) {
        left_val = get_acceleration(power_val + turn_val, 1);
        right_val = get_acceleration(power_val - turn_val, 1);
    }

    (
        left_val * Motor::V5_MAX_VOLTAGE,
        right_val * Motor::V5_MAX_VOLTAGE,
    )
}
