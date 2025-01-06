use alloc::vec::Vec;

use crate::pose::Pose;
use evian::math::Vec2;

#[derive(Clone, Copy)]
pub enum Command {
    Coordinate(Vec2<f64>),
    CubicBezier(Vec2<f64>, Vec2<f64>, Vec2<f64>, Vec2<f64>),
    DriveBy(f64),
    TurnBy(f64),
    TurnTo(f64),
    Speed(f64),
    Sleep(u64),
    ToggleIntake,
    ToggleIntakeReverse,
    ToggleIntakePiston,
    ToggleLadyBrown,
    ToggleClamp,
}

pub fn command_to_coords(path: &[Command]) -> Vec<Command> {
    let (coord, path) = match path[0] {
        Command::Coordinate(coord) => (coord, &path[1..]),
        Command::CubicBezier(p0, ..) => (p0, path),
        _ => (Vec2::new(0.0, 0.0), path),
    };

    let mut pose = Pose::new(coord, 0.0);
    let mut new_path = Vec::new();

    new_path.push(Command::Coordinate(pose.position));

    for &command in path {
        match command {
            Command::DriveBy(distance) => {
                let traveled = Vec2::from_polar(distance, pose.heading.to_radians());
                pose.position += traveled;

                new_path.push(Command::Coordinate(pose.position))
            }
            Command::TurnBy(angle) => {
                pose.heading += angle;
                pose.heading %= 360.0;
            }
            Command::TurnTo(heading) => {
                pose.heading = heading;
                pose.heading %= 360.0;
            }
            _ => new_path.push(command),
        };
    }

    new_path
}

