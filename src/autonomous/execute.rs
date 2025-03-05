use core::time::Duration;

use evian::{
    differential::motion::{BasicMotion, Ramsete, Seeking},
    math::IntoAngle,
    prelude::{CubicBezier, Trajectory},
};
use vexide::prelude::{Motor, sleep};

use super::{ANGULAR_CONTROLLER, CONSTRAINTS, LINEAR_CONTROLLER, TOLERANCES, command::Command};
use crate::{GEARING, Robot, TRACK_WIDTH, WHEEL_DIAMETER};

pub async fn execute_command(robot: &mut Robot, command: Command) {
    let mut basic = BasicMotion {
        linear_controller: LINEAR_CONTROLLER,
        angular_controller: ANGULAR_CONTROLLER,
        linear_tolerances: TOLERANCES,
        angular_tolerances: TOLERANCES,
    };

    let mut seeking = Seeking {
        distance_controller: LINEAR_CONTROLLER,
        angle_controller: ANGULAR_CONTROLLER,
        tolerances: TOLERANCES,
    };

    let mut ramsete = Ramsete {
        b: 0.00129,
        zeta: 0.2,
        track_width: TRACK_WIDTH,
        wheel_diameter: WHEEL_DIAMETER,
        external_gearing: GEARING,
    };

    let dt = &mut robot.drivetrain;

    match command {
        Command::Coordinate(coord) => {
            _ = seeking.move_to_point(dt, coord).await;
        }
        Command::CubicBezier(p0, p1, p2, p3) => {
            let curve = CubicBezier::new(p0, p1, p2, p3);
            let trajectory = Trajectory::generate(curve, 0.1, CONSTRAINTS);

            ramsete.follow(dt, trajectory).await;
        }
        Command::DriveBy(distance) => {
            _ = basic.drive_distance(dt, distance).await;
        }
        Command::TurnTo(angle) => {
            _ = basic.turn_to_heading(dt, -angle.deg()).await;
        }
        // speed can be controlled using the `velocity_tolerance` constraint in the settler
        Command::Speed(max_speed) => {
            let output_limit = Some(Motor::V5_MAX_VOLTAGE * max_speed);

            basic.linear_controller.set_output_limit(output_limit);
        }
        Command::ToggleIntake => {
            robot.intake.toggle(1.0);
        }
        Command::ToggleIntakeReverse => robot.intake.toggle(-1.0),
        Command::ToggleIntakePiston => {
            _ = robot.intake_lift.toggle();
        }
        Command::ToggleClamp => {
            _ = robot.clamp.toggle();
        }
        Command::NextLBStage => {
            robot.lady_brown.next();
        }
        Command::Sleep(delay) => {
            sleep(Duration::from_millis(delay)).await;
        }
        _ => unreachable!(),
    };
}
