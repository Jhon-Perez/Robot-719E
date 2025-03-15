use core::time::Duration;

use evian::{
    control::{AngularPid, Pid},
    differential::motion::{BasicMotion, Ramsete, Seeking},
    math::IntoAngle,
    prelude::*,
};
use vexide::prelude::{Motor, sleep};

use super::command::Command;
use crate::{Robot, DRIVE_RPM, GEARING, TRACK_WIDTH, WHEEL_DIAMETER};

const fn from_drive_rpm(rpm: f64, wheel_diameter: f64) -> f64 {
    (rpm / 60.0) * (core::f64::consts::PI * wheel_diameter)
}

pub const TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(0.5)
    .tolerance_duration(Duration::from_millis(250))
    .timeout(Duration::from_millis(1500))
    .velocity_tolerance(5.0)
;

pub const LINEAR_CONTROLLER: Pid = Pid::new(1.25, 0.0, 0.0, None);
pub const ANGULAR_CONTROLLER: AngularPid =
    AngularPid::new(30.0, 1.75, 2.0, Some(Angle::from_degrees(25.0)));
    //AngularPid::new(25.0, 1.75, 2.25, Some(Angle::from_degrees(20.0)));

const CONSTRAINTS: TrajectoryConstraints = TrajectoryConstraints {
    max_velocity: from_drive_rpm(DRIVE_RPM, WHEEL_DIAMETER),
    max_acceleration: 200.0,
    max_deceleration: 200.0,
    friction_coefficient: 1.0,
    track_width: TRACK_WIDTH,
};

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
