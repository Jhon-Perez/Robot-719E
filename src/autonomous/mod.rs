pub mod command;
pub mod execute;
mod parse;

use core::time::Duration;

use evian::{
    control::{AngularPid, Pid},
    prelude::*,
};

use crate::{DRIVE_RPM, TRACK_WIDTH, WHEEL_DIAMETER, from_drive_rpm};

/// A list of commands for the robot to execute
pub static PATHS: [&str; 6] = [
    "",
    include_str!("paths/Left.botpath"),
    include_str!("paths/Left (mess up).botpath"),
    include_str!("paths/Goal Rush.botpath"),
    include_str!("paths/Right.botpath"),
    include_str!("paths/Skills.botpath"),
];

const TOLERANCES: Tolerances = Tolerances::new()
    .error_tolerance(0.75)
    .tolerance_duration(Duration::from_millis(150))
    .timeout(Duration::from_millis(1250));

const LINEAR_CONTROLLER: Pid = Pid::new(1.25, 0.0, 0.0, None);
const ANGULAR_CONTROLLER: AngularPid =
    AngularPid::new(22.5, 1.75, 2.25, Some(Angle::from_degrees(20.0)));

const CONSTRAINTS: TrajectoryConstraints = TrajectoryConstraints {
    max_velocity: from_drive_rpm(DRIVE_RPM, WHEEL_DIAMETER),
    max_acceleration: 200.0,
    max_deceleration: 200.0,
    friction_coefficient: 1.0,
    track_width: TRACK_WIDTH,
};
