use alloc::{vec, vec::Vec};

use evian::math::Vec2;

use crate::command::Command;

// change to choosing path rather than returning all
pub fn paths() -> [Vec<Command>; 6] {
    [
        vec![
            Command::Coordinate(Vec2 { x: 0.0, y: 0.0 }),
            Command::Coordinate(Vec2 { x: 0.0, y: 0.0 }),
        ],
        vec![
            // path 1
            Command::Coordinate(Vec2 { x: 12.0, y: 48.0 }), // START THE PATH WITH ITS STARTING POINT
            Command::Speed(0.8),
            Command::DriveBy(32.0), // move forward to clamp goal
            Command::Speed(1.0),
            Command::Sleep(750),
            Command::ToggleIntake,
            Command::ToggleClamp,
            Command::Sleep(500),
            Command::DriveBy(6.0),
            Command::TurnTo(90.0),   // turn to face rings
            Command::DriveBy(-26.0), // drive towards rings
            Command::TurnTo(180.0),  // turn 45 degrees right
            Command::DriveBy(-14.0),
            Command::TurnTo(-90.0),
            Command::Speed(0.8),
            Command::DriveBy(-22.5),
            Command::ToggleIntake,
        ],
        vec![
            // path 1 (mess up)
            Command::Coordinate(Vec2 { x: 12.0, y: 48.0 }), // START THE PATH WITH ITS STARTING POINT
            Command::DriveBy(32.0),                         // move forward to clamp goal
            Command::Sleep(750),
            Command::ToggleIntake,
            Command::ToggleClamp,
            Command::Sleep(500),
            Command::TurnTo(135.0), // turn to face rings
            Command::ToggleIntakePiston,
            Command::DriveBy(-28.0),      // drive towards rings
            Command::ToggleIntakeReverse, // INVERSE THIS
            Command::ToggleIntakePiston,
            Command::Sleep(1000),
            Command::ToggleIntake,
            Command::Sleep(1000),
            Command::TurnTo(0.0),
            Command::DriveBy(-20.0),
            Command::TurnTo(-67.5),
            Command::DriveBy(-48.0),
            Command::ToggleIntakePiston,
            Command::Sleep(250),
            Command::ToggleIntakePiston,
            Command::Sleep(250),
            Command::TurnTo(-45.0),
            Command::DriveBy(36.0),
        ],
        vec![
            // path 2
            Command::Coordinate(Vec2 { x: 24.0, y: 132.0 }),
            Command::DriveBy(32.0),
            Command::TurnTo(-50.0),
            Command::DriveBy(20.0),
            Command::Sleep(750),
            Command::ToggleClamp,
            Command::ToggleIntake,
            Command::Sleep(500),
            Command::TurnTo(0.0),
            Command::DriveBy(-18.0),
            Command::TurnTo(135.0),
            Command::DriveBy(-24.0),
        ],
        vec![
            // skills route
            Command::Coordinate(Vec2 {
                x: 10.0,
                y: 144.0 / 2.0,
            }),
            Command::ToggleIntake,
            Command::Sleep(1000),
            Command::ToggleIntake,
            Command::TurnTo(55.0),
            Command::DriveBy(-27.5),
            Command::ToggleClamp,
            Command::TurnTo(0.0),
            Command::ToggleIntake,
            Command::DriveBy(-23.0),
            Command::TurnTo(55.0),
            Command::DriveBy(-40.0),
            Command::TurnTo(-150.0),
            Command::DriveBy(-24.0),
            Command::TurnTo(180.0),
            Command::DriveBy(-24.0),
            Command::Sleep(500),
            Command::DriveBy(-12.0),
            Command::TurnTo(45.0),
            Command::DriveBy(-14.0),
            Command::TurnTo(-25.0),
            Command::DriveBy(20.0),
            Command::ToggleClamp,
        ],
        vec![
            Command::CubicBezier(
                Vec2::new(10.0, 72.0),
                Vec2::new(18.0, 8.0),
                Vec2::new(130.0, 24.0),
                Vec2::new(50.0, 72.0),
            ),
        ],
    ]
}
