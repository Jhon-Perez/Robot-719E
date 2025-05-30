use alloc::vec::Vec;

use evian::prelude::Vec2;

use crate::autonomous::command::Command;

// Swap coordinates to match the blue side of the field
pub fn reverse_coord(coord: &Vec2<f64>) -> Vec2<f64> {
    Vec2::new(144.0 - coord.x(), coord.y())
}

fn reverse_angle(angle: f64) -> f64 {
    let mut flipped = angle + 180.0;
    if flipped > 180.0 {
        flipped -= 360.0;
    } else if flipped < -180.0 {
        flipped += 360.0;
    }
    flipped
}

// Go through the list of commands provided and swap coordinates and angles
pub fn invert_coords(commands: &[Command]) -> Vec<Command> {
    commands
        .iter()
        .map(|command| match command {
            Command::TurnTo(angle) => Command::TurnTo(reverse_angle(*angle)),
            Command::Coordinate(coord) => Command::Coordinate(reverse_coord(coord)),
            Command::CubicBezier(p0, p1, p2, p3) => Command::CubicBezier(
                reverse_coord(p0),
                reverse_coord(p1),
                reverse_coord(p2),
                reverse_coord(p3),
            ),
            Command::Pose(coord, angle) => Command::Pose(reverse_coord(coord), *angle),
            _ => *command,
        })
        .collect()
}
