#![no_main]
#![no_std]

extern crate alloc;

mod autonomous;
mod backend;
mod mappings;
mod pose;
mod subsystems;

use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;

use autonomous::{command::Command, execute::execute_command};
use backend::{Color, initialize_slint_gui};
use evian::{drivetrain::Drivetrain, math::Vec2, prelude::*};
use mappings::{ControllerMappings, DriveMode};
use subsystems::{drivetrain::differential_drive, intake::Intake, lady_brown::LadyBrown};
use vexide::{
    core::time::Instant, devices::adi::digital::LogicLevel, prelude::*,
    startup::banner::themes::THEME_MURICA,
};

const TRACK_WIDTH: f64 = 12.75;
const DRIVE_RPM: f64 = 450.0;
const GEARING: f64 = 36.0 / 48.0;
const WHEEL_DIAMETER: f64 = 2.0;

pub struct RobotSettings {
    pub auton_path: Vec<Command>,
    pub test_auton: bool,
    pub curr_color: Color,
}

struct Robot {
    drivetrain: Drivetrain<Differential, PerpendicularWheelTracking>,

    intake: Intake,
    color_sort: OpticalSensor,
    intake_lift: AdiDigitalOut,
    lady_brown: LadyBrown,
    clamp: AdiDigitalOut,

    controller: Controller,

    settings: Rc<RefCell<RobotSettings>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous control started.");

        let auton_path: Vec<Command> = self.settings.borrow().auton_path.clone();

        // return early if there is no path
        if auton_path.is_empty() {
            return;
        }

        // Set starting position to the first line of autonomous paths
        if let Command::Pose(position, angle) = auton_path[0] {
            self.drivetrain.tracking.set_position(position);
            self.drivetrain.tracking.set_heading(angle.deg());
        }

        for &command in auton_path[1..].iter() {
            execute_command(self, command).await;
        }
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        _ = self.intake_lift.set_low();

        loop {
            let delay = Instant::now() + Controller::UPDATE_INTERVAL;

            let state = self.controller.state().unwrap_or_default();

            let mappings = ControllerMappings {
                drive_mode: DriveMode::Arcade {
                    power: state.left_stick,
                    turn: state.right_stick,
                },
                intake: state.button_l1,
                outake: state.button_l2,
                intake_lift: state.button_a,
                lady_brown: state.button_y,
                clamp: state.button_r1,
            };

            let power = differential_drive(&mappings);
            _ = self.drivetrain.motors.set_voltages(power);

            // check if there is the intake has the wrong ring color
            if mappings.intake.is_pressed() {
                // blue ring has a hue of 160 to 200 and red ring has a hue of 0 to 40
                // so the oppisite hue is what we are checking for
                let color_range = match self.settings.borrow().curr_color {
                    Color::Red => 160.0..200.0,
                    Color::Blue => 0.0..40.0,
                };

                let color_hue = self.color_sort.hue().unwrap_or_default();
                let proximity = self.color_sort.proximity().unwrap_or_default();

                // The oppisite color has obstructed the view, kick the ring out
                let multiplier = if color_range.contains(&color_hue) && proximity == 1.0 {
                    1.0
                } else {
                    0.8 // tune this value later
                };

                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE * multiplier);
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.brake(BrakeMode::Coast);
            }

            if mappings.lady_brown.is_now_pressed() {
                self.lady_brown.next();
            }

            // run autonomous when button is pressed to prevent the need of a competition switch
            if self.settings.borrow().test_auton {
                {
                    let mut settings = RefCell::borrow_mut(&self.settings);
                    settings.test_auton = false;
                }

                self.autonomous().await;
            }

            if mappings.clamp.is_now_pressed() {
                _ = self.clamp.toggle();
            }

            if mappings.intake_lift.is_now_pressed() {
                _ = self.intake_lift.toggle();
            }

            sleep_until(delay).await;
        }
    }
}

#[inline]
pub const fn from_drive_rpm(rpm: f64, wheel_diameter: f64) -> f64 {
    (rpm / 60.0) * (core::f64::consts::PI * wheel_diameter)
}

#[vexide::main(banner(theme = THEME_MURICA))]
async fn main(peripherals: Peripherals) {
    println!("Program started.");

    let settings = Rc::new(RefCell::new(RobotSettings {
        auton_path: Vec::new(),
        test_auton: false,
        curr_color: Color::Red,
    }));

    initialize_slint_gui(peripherals.display, settings.clone());

    let mut imu = InertialSensor::new(peripherals.port_4);

    match imu.calibrate().await {
        Ok(_) => println!("Calibration Successful"),
        Err(e) => println!("Error {:?}", e),
    }

    let robot = Robot {
        drivetrain: Drivetrain::new(
            Differential::new(
                shared_motors![
                    Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
                ],
                shared_motors![
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
                ],
            ),
            PerpendicularWheelTracking::new(
                Vec2::default(),
                Angle::default(),
                TrackingWheel::new(
                    RotationSensor::new(peripherals.port_5, Direction::Forward),
                    WHEEL_DIAMETER,
                    TRACK_WIDTH / 2.0, // to be determined
                    None,
                ),
                TrackingWheel::new(
                    RotationSensor::new(peripherals.port_6, Direction::Forward),
                    WHEEL_DIAMETER,
                    TRACK_WIDTH / 2.0, // to be determined
                    None,
                ),
                imu,
            ),
        ),
        intake: Intake::new(
            Motor::new(peripherals.port_19, Gearset::Green, Direction::Reverse),
            Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
        ),
        color_sort: OpticalSensor::new(peripherals.port_18),
        intake_lift: AdiDigitalOut::with_initial_level(peripherals.adi_h, LogicLevel::High),
        lady_brown: LadyBrown::new(
            [
                Motor::new_exp(peripherals.port_14, Direction::Reverse),
                Motor::new_exp(peripherals.port_13, Direction::Forward),
            ],
            None,
            Some(12.0 / 60.0),
        ),
        clamp: AdiDigitalOut::with_initial_level(peripherals.adi_e, LogicLevel::Low),
        controller: peripherals.primary_controller,
        settings: settings.clone(),
    };

    robot.compete().await;
}
