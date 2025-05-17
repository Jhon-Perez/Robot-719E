use vexide::prelude::{Float, Motor};

use crate::mappings::DriveMode;

/// Applies an acceleration function to the given power value.
/// Uses polynomial scaling based on the acceleration factor.
fn get_acceleration(power: f64, acceleration: i32) -> f64 {
    if acceleration == 1 {
        return power; // If acceleration is 1, return power as is (linear mapping)
    }

    // Polynomial acceleration adjustment
    power.powi(acceleration - 1)
        * if acceleration % 2 == 0 {
            power.abs() // Even acceleration preserves absolute magnitude
        } else {
            power // Odd acceleration preserves sign
        }
}

/// Computes the left and right motor power values based on the selected drive mode.
/// Supports both Arcade and Tank drive configurations.
pub fn differential_drive(drive_mode: &DriveMode) -> (f64, f64) {
    let mut power_val = 0.0;
    let mut turn_val = 0.0;
    let mut left_val = 0.0;
    let mut right_val = 0.0;

    // Extract joystick values based on the configured drive mode
    match drive_mode {
        DriveMode::Arcade { power, turn } => {
            power_val = power.y(); // Forward/backward movement
            turn_val = turn.x(); // Turning movement
        }
        DriveMode::Tank { left, right } => {
            left_val = left.y(); // Left side control
            right_val = right.y(); // Right side control
        }
    }

    // Apply acceleration function if using Arcade drive
    if !matches!(drive_mode, DriveMode::Tank { .. }) {
        left_val = get_acceleration(power_val + turn_val, 1);
        right_val = get_acceleration(power_val - turn_val, 1);
    }

    // Scale the final voltage values to the V5 motor's maximum voltage
    (
        left_val * Motor::V5_MAX_VOLTAGE,
        right_val * Motor::V5_MAX_VOLTAGE,
    )
}
