use alloc::vec::Vec;

use crate::{odometry::Pose, vector::Vec2};

#[derive(Clone, Copy, Debug)]
pub enum Command {
    Coordinate(Vec2),
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

pub fn command_to_coords(path: &[Command]) -> Vec<Vec2> {
    let coord = match path[0] {
        Command::Coordinate(coordinate) => coordinate,
        _ => panic!("COORDINATE IS THE ONLY SUPPORTED FEATURE AS OF NOW"),
    };

    let mut pose = Pose::new(coord.x, coord.y, 0.0);
    let mut coords = Vec::new();

    coords.push(pose.position);

    for &command in &path[1..] {
        let coord = match command {
            Command::Coordinate(coordinate) => Some(coordinate),
            Command::DriveBy(distance) => {
                let traveled = Vec2::from_polar(distance, pose.heading.to_radians());
                pose.position += traveled;

                Some(pose.position)
            }
            Command::TurnBy(angle) => {
                pose.heading += angle;
                pose.heading %= 360.0;

                None
            }
            Command::TurnTo(heading) => {
                pose.heading = heading;
                pose.heading %= 360.0;

                None
            }
            _ => None,
        };

        if let Some(coordinate) = coord {
            coords.push(coordinate);
        }
    }

    coords
}

