use core::time::Duration;

use evian::{
    control::loops::{AngularPid, Pid},
    math::IntoAngle,
    motion::{Basic, Seeking},
    prelude::*,
};
use vexide::prelude::{Motor, sleep};

use super::command::Command;
use crate::{Robot /*, DRIVE_RPM, GEARING, TRACK_WIDTH, WHEEL_DIAMETER*/};

pub const TOLERANCES: Tolerances = Tolerances::new()
    .duration(Duration::from_millis(250))
    .error(0.5)
    .velocity(50.0);

pub const LINEAR_CONTROLLER: Pid = Pid::new(1.25, 0.0, 0.0, None);
pub const ANGULAR_CONTROLLER: AngularPid =
    AngularPid::new(30.0, 1.75, 2.0, Some(Angle::from_degrees(25.0)));

pub async fn execute_command(robot: &mut Robot, command: Command, basic: &mut Basic<Pid, AngularPid>) {
    let mut seeking = Seeking {
        linear_controller: LINEAR_CONTROLLER,
        angular_controller: ANGULAR_CONTROLLER,
        tolerances: TOLERANCES,
        timeout: Some(Duration::from_millis(2000)),
    };

    let dt = &mut robot.drivetrain;

    match command {
        Command::Coordinate(coord) => {
            _ = seeking.move_to_point(dt, coord).await;
        }
        // replace this with pure pursuit
        Command::CubicBezier(_p0, _p1, _p2, _p3) => {
            //let curve = CubicBezier::new(p0, p1, p2, p3);
            //let trajectory = Trajectory::generate(curve, 0.1, CONSTRAINTS);
            //
            //ramsete.follow(dt, trajectory).await;
        }
        Command::DriveBy(distance) => {
            _ = basic.drive_distance(dt, distance).await;
            sleep(Duration::from_millis(250)).await;
        }
        Command::TurnTo(angle) => {
            _ = basic.turn_to_heading(dt, -angle.deg()).await;
            sleep(Duration::from_millis(250)).await;
        }
        // speed can be controlled using the `velocity_tolerance` constraint in the settler
        Command::Speed(max_speed) => {
            let output_limit = Some(Motor::V5_MAX_VOLTAGE * max_speed);

            basic.linear_controller.set_output_limit(output_limit);
        }
        Command::IntakeCommand(cmd) => {
            robot.intake.set_command(cmd);
        }
        Command::LadyBrownCommand(cmd) => {
            robot.lady_brown.set_command(cmd);
        }
        Command::ToggleClamp => {
            _ = robot.clamp.0.toggle();
            _ = robot.clamp.1.toggle();
        }
        Command::NextLBStage => {
            robot.lady_brown.next();
        }
        Command::Sleep(delay) => {
            sleep(Duration::from_millis(delay)).await;
        }
        _ => (),
    };
}
