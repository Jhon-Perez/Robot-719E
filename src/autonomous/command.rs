use alloc::vec::Vec;

use evian::math::Vec2;

use super::parse;
use crate::{pose::Pose, subsystems::{intake::IntakeCommand, lady_brown::LadyBrownCommand}};


/// Represents different types of movement and action commands
/// the robot will execute during the autonomous period.
#[derive(Clone, Copy, Debug)]
pub enum Command {
    /// Move to a specific coordinate on the field
    Coordinate(Vec2<f64>),

    /// Follow a cubic Bezier curve with four control points
    CubicBezier(Vec2<f64>, Vec2<f64>, Vec2<f64>, Vec2<f64>),

    /// Move forward by a specified distance (in inches)
    DriveBy(f64),

    /// Set the initial robot position and orientation
    Pose(Vec2<f64>, f64),

    /// Rotate the robot by a relative angle
    TurnBy(f64),

    /// Rotate the robot to an absolute angle
    TurnTo(f64),

    /// Set a speed limit for the robot
    Speed(f64),

    /// Pause execution for a given duration (milliseconds)
    Sleep(u64),

    /// Toggle the intake system
    IntakeCommand(IntakeCommand),

    /// Move the "Lady Brown" system to the next stage
    NextLBStage,
    LadyBrownCommand(LadyBrownCommand),

    /// Toggle the mobile goal clamp
    ToggleClamp,
}

impl Command {
    pub fn from_str(command: &str, args: &[&str]) -> Result<Self, &'static str> {
        use parse::*;
        match command.trim() {
            "Coordinate" => single_vec2(args).map(Command::Coordinate),
            "Bezier" => {
                multiple_vec2(args, 4).map(|v| Command::CubicBezier(v[0], v[1], v[2], v[3]))
            }
            "Drive" => single_f64(args).map(Command::DriveBy),
            "Pose" => pose(args),
            "Turn" => single_f64(args).map(Command::TurnTo),
            "Sleep" => single_u64(args).map(Command::Sleep),
            "Speed" => single_f64(args).map(Command::Speed),
            "Intake" => intake_command(args).map(Command::IntakeCommand),
            "LadyBrown" => lady_brown_command(args).map(Command::LadyBrownCommand),
            "NextLBStage" => Ok(Command::NextLBStage),
            "ToggleClamp" => Ok(Command::ToggleClamp),
            _ => Err("Invalid Command"),
        }
    }
}

/// Converts a path string into a list of commands
pub fn path_to_commands(path: &str) -> Result<Vec<Command>, &'static str> {
    let mut commands = Vec::new();

    // Trim any whitespace and remove empty lines
    for line in path.lines().map(str::trim).filter(|l| !l.is_empty()) {
        if line.starts_with("//") {
            continue; // Ignore comments
        }

        let tokens: Vec<&str> = line.split_whitespace().collect();
        let (action, args) = tokens.split_first().ok_or("Empty command")?;

        // Try to convert the action + arguments into a Command enum variant
        commands.push(Command::from_str(action, args)?);
    }

    Ok(commands)
}

/// Converts movement-related commands into explicit coordinate-based commands.
/// This function ensures that movement commands (such as `DriveBy` and `Pose`)
/// are converted into explicit `Coordinate` commands for path processing.
pub fn command_to_coords(path: &[Command]) -> Vec<Command> {
    let mut new_path = Vec::new();

    // Initialize the starting pose based on the first command
    let (mut pose, path) = match path.first() {
        // Start at the given coordinate
        Some(Command::Coordinate(coord)) => (Pose::new(*coord, 0.0), &path[1..]),
        // Start at the first point of a cubic Bézier curve
        Some(Command::CubicBezier(p0, ..)) => (Pose::new(*p0, 0.0), path),
        // Explicitly defined start position and angle
        Some(Command::Pose(coord, angle)) => (Pose::new(*coord, *angle), &path[1..]),
        // Default to (0,0) if no starting pose is provided
        _ => (Pose::new(Vec2::new(0.0, 0.0), 0.0), path),
    };

    // Ensure the first command in the converted path is a coordinate
    new_path.push(Command::Coordinate(pose.position));

    // Process each command and update the pose accordingly
    for &command in path {
        match command {
            Command::Coordinate(coord) => {
                // If the command specifies a direct coordinate, update position
                pose.position = coord;
                new_path.push(command);
            }
            Command::DriveBy(distance) => {
                // Move forward by `distance` along the current heading
                pose.position += Vec2::from_polar(distance, pose.heading.to_radians());
                new_path.push(Command::Coordinate(pose.position)); // Store the new coordinate
            }
            Command::TurnBy(angle) => {
                // Turn relative to the current heading
                pose.heading = (pose.heading + angle) % 360.0;
            }
            Command::TurnTo(angle) => {
                // Set the absolute heading
                pose.heading = angle % 360.0;
            }
            Command::Pose(coord, angle) => {
                // Directly set the position and angle
                pose = Pose::new(coord, angle);
                new_path.push(Command::Coordinate(pose.position)); // Store the new coordinate
            }
            _ => new_path.push(command), // Keep other commands unchanged
        }
    }

    new_path
}
