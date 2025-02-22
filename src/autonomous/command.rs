use alloc::vec::Vec;

use evian::math::Vec2;
use vexide::core::println;

use crate::pose::Pose;

use super::parse;

#[derive(Clone, Copy, Debug)]
pub enum Command {
    Coordinate(Vec2<f64>),
    CubicBezier(Vec2<f64>, Vec2<f64>, Vec2<f64>, Vec2<f64>),
    DriveBy(f64),
    Pose(Vec2<f64>, f64), // mostly use to get starting pose of robot
    // UNSTABLE DO NOT USE `TurnBy`
    #[allow(unused)]
    TurnBy(f64),
    TurnTo(f64),
    Speed(f64),
    Sleep(u64),
    ToggleIntake,
    ToggleIntakeReverse,
    ToggleIntakePiston,
    NextLBStage,
    ToggleClamp,
}

impl Command {
    pub fn from_str(command: & str, args: &[& str]) -> Result<Self, &'static str> {
        match command {
            "Coordinate" => parse::single_vec2(args).map(Command::Coordinate),
            "CubicBezier" => {
                parse::multiple_vec2(args, 4).map(|v| Command::CubicBezier(v[0], v[1], v[2], v[3]))
            }
            "Drive" => parse::single_f64(args).map(Command::DriveBy),
            "Pose" => parse::pose(args),
            "Turn" => parse::single_f64(args).map(Command::TurnTo),
            "Sleep" => parse::single_u64(args).map(Command::Sleep),
            "ToggleIntake" => Ok(Command::ToggleIntake),
            "ToggleIntakeReverse" => Ok(Command::ToggleIntakeReverse),
            "ToggleIntakePiston" => Ok(Command::ToggleIntakePiston),
            "NextLBStage" => Ok(Command::NextLBStage),
            "ToggleClamp" => Ok(Command::ToggleClamp),
            _ => Err("Invalid Command"),
        }
    }
}

pub fn path_to_commands(path: &str) -> Vec<Command> {
    let path = path
        .split('\n')
        .map(|x| x.split_whitespace().collect::<Vec<&str>>());

    let mut commands: Vec<Command> = Vec::new();

    for p in path {
        if p.is_empty() {
            continue;
        }

        let action = p[0];
        let args = &p[1..];

        // indicates a comment
        if action.starts_with("//") {
            continue;
        }

        let command = match Command::from_str(action, args) {
            Ok(c) => c,
            Err(e) => {
                println!("{}", e);
                return Vec::new();
            }
        };

        commands.push(command);
    }

    println!("{:?}", commands);
    commands
}

pub fn command_to_coords(path: &[Command]) -> Vec<Command> {
    if path.is_empty() {
        return Vec::new();
    }

    let (mut pose, path) = match path[0] {
        Command::Coordinate(coord) => (Pose::new(coord, 0.0), &path[1..]),
        Command::CubicBezier(p0, ..) => (Pose::new(p0, 0.0), path),
        Command::Pose(coord, angle) => (Pose::new(coord, angle), &path[1..]),
        _ => (Pose::new(Vec2::new(0.0, 0.0), 0.0), path),
    };

    let mut new_path = Vec::new();

    new_path.push(Command::Coordinate(pose.position));

    for &command in path {
        match command {
            Command::Coordinate(coord) => {
                pose.position = coord;

                new_path.push(command);
            }
            Command::DriveBy(distance) => {
                let traveled = Vec2::from_polar(distance, pose.heading.to_radians());
                pose.position += traveled;

                new_path.push(Command::Coordinate(pose.position));
            }
            // The cubic bezier implementation does not take account of any turning, so do not use
            // this as it will not be able to find the relative angle after a cb is used
            Command::TurnBy(angle) => {
                pose.heading += angle;
                pose.heading %= 360.0;
            }
            Command::TurnTo(heading) => {
                pose.heading = heading;
                pose.heading %= 360.0;
            }
            Command::Pose(coord, angle) => {
                pose.position = coord;
                pose.heading = angle;

                new_path.push(Command::Coordinate(pose.position));
            }
            _ => new_path.push(command),
        };
    }

    new_path
}
