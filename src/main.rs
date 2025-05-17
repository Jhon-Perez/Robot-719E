#![no_main]
#![no_std]

extern crate alloc;

mod autonomous;
#[cfg(feature = "gui")]
mod backend;
mod mappings;
mod pose;
mod subsystems;

use alloc::{rc::Rc, vec::Vec};
use core::{cell::RefCell, time::Duration};

use autonomous::{
    command::Command,
    execute::{
        ANGULAR_CONTROLLER, LINEAR_CONTROLLER,
        TOLERANCES, /*, ANGULAR_CONTROLLER, LINEAR_CONTROLLER, TOLERANCES*/
        execute_command,
    },
};
use evian::{drivetrain::Drivetrain, math::Vec2, motion::Basic, prelude::*};
use mappings::{ControllerMappings, DriveMode};
use subsystems::{
    drivetrain::differential_drive,
    intake::{Intake, IntakeCommand},
    lady_brown::LadyBrown,
};
use vexide::{
    devices::adi::digital::LogicLevel, prelude::*, startup::banner::themes::THEME_MURICA,
    time::Instant,
};

const TRACK_WIDTH: f64 = 12.75;
// const DRIVE_RPM: f64 = 450.0;
const GEARING: f64 = 36.0 / 48.0;
const WHEEL_DIAMETER: f64 = 3.25;

#[derive(Clone, Copy)]
pub enum Color {
    Red,
    Blue,
}

pub struct RobotSettings {
    pub auton_path: Vec<Command>,
    pub test_auton: bool,
    pub curr_color: Color,
}

struct Robot {
    drivetrain: Drivetrain<Differential, WheeledTracking>,
    intake: Intake,
    doinker: AdiDigitalOut,
    lady_brown: LadyBrown,
    clamp: (AdiDigitalOut, AdiDigitalOut),

    controller: Controller,

    settings: Rc<RefCell<RobotSettings>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous control started.");

        let auton_path: Vec<Command> = self.settings.borrow().auton_path.clone();

        // Check if the path is empty
        let Some(pose) = auton_path.first() else {
            return;
        };

        // First element is starting position
        if let Command::Pose(position, angle) = pose {
            self.drivetrain.tracking.set_position(*position);
            self.drivetrain.tracking.set_heading(angle.deg());
        }

        let mut basic = Basic {
            linear_controller: LINEAR_CONTROLLER,
            angular_controller: ANGULAR_CONTROLLER,
            linear_tolerances: TOLERANCES,
            angular_tolerances: TOLERANCES,
            timeout: Some(Duration::from_millis(2000)),
        };

        for &command in auton_path.iter().skip(1) {
            execute_command(self, command, &mut basic).await;
        }
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        let mut basic = Basic {
            linear_controller: LINEAR_CONTROLLER,
            angular_controller: ANGULAR_CONTROLLER,
            linear_tolerances: TOLERANCES,
            angular_tolerances: TOLERANCES,
            timeout: Some(Duration::from_millis(2000)),
        };

        loop {
            let delay = Instant::now() + Controller::UPDATE_INTERVAL;

            let state = self.controller.state().unwrap_or_default();

            let mappings = ControllerMappings {
                drive_mode: DriveMode::Arcade {
                    power: state.left_stick,
                    turn: state.left_stick,
                },
                intake: state.button_l1,
                outake: state.button_l2,
                doinker: state.button_a,
                toggle_color_sort: state.button_a,
                lady_brown: state.button_x,
                manual_lady_brown: state.right_stick,
                clamp: state.button_r1,
                test_linear: state.button_up,
                test_angular: state.button_right,
            };

            let power = differential_drive(&mappings.drive_mode);
            // self.drivetrain.set_voltage(power);
            _ = self.drivetrain.motors.set_voltages(power);

            // neaten with refactor
            if mappings.intake.is_pressed() {
                self.intake.set_command(IntakeCommand::On);
            } else if mappings.outake.is_pressed() {
                self.intake
                    .set_command(IntakeCommand::Voltage(-Motor::V5_MAX_VOLTAGE));
            } else {
                self.intake.set_command(IntakeCommand::Off);
            }

            if mappings.toggle_color_sort.is_now_pressed() {
                self.intake.toggle_color_sort();
            }

            if mappings.lady_brown.is_now_pressed() {
                self.lady_brown.next();
            } else {
                // do lady brown manual control
            }

            // run autonomous when button is pressed to prevent the need of a competition switch
            if self.settings.borrow().test_auton {
                {
                    let mut settings = RefCell::borrow_mut(&self.settings);
                    settings.test_auton = false;
                }

                self.autonomous().await;
            }

            if mappings.test_linear.is_now_pressed() {
                execute_command(self, Command::DriveBy(12.0), &mut basic).await;
            } else if mappings.test_angular.is_now_pressed() {
                execute_command(self, Command::TurnTo(0.0), &mut basic).await;
            }

            if mappings.clamp.is_now_pressed() {
                _ = self.clamp.0.toggle();
                _ = self.clamp.1.toggle();
            }

            if mappings.doinker.is_now_pressed() {
                _ = self.doinker.toggle();
            }

            sleep_until(delay).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_MURICA))]
async fn main(peripherals: Peripherals) {
    println!("Program started.");

    let settings = Rc::new(RefCell::new(RobotSettings {
        auton_path: Vec::new(),
        test_auton: false,
        curr_color: Color::Red,
    }));

    #[cfg(feature = "gui")]
    backend::initialize_slint_gui(peripherals.display, settings.clone());

    let mut imu = InertialSensor::new(peripherals.port_8);

    match imu.calibrate().await {
        Ok(_) => println!("Calibration Successful"),
        Err(e) => println!("Error {:?}", e),
    }

    let left_motors = shared_motors![
        Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),
    ];
    let right_motors = shared_motors![
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
    ];

    let robot = Robot {
        drivetrain: Drivetrain::new(
            Differential::from_shared(left_motors.clone(), right_motors.clone()),
            WheeledTracking::new(
                Vec2::default(),
                Angle::default(),
                [
                    TrackingWheel::new(
                        left_motors.clone(),
                        WHEEL_DIAMETER,
                        TRACK_WIDTH,
                        Some(GEARING),
                    ),
                    TrackingWheel::new(
                        right_motors.clone(),
                        WHEEL_DIAMETER,
                        TRACK_WIDTH,
                        Some(GEARING),
                    ),
                ],
                [
                    // Fake tracking wheel because i can't figure out how to work without sideways
                    TrackingWheel::new(
                        RotationSensor::new(peripherals.port_20, Direction::Forward),
                        WHEEL_DIAMETER,
                        TRACK_WIDTH,
                        Some(GEARING),
                    ),
                ],
                Some(imu),
            ),
        ),
        intake: Intake::new(
            [Motor::new(
                peripherals.port_9,
                Gearset::Blue,
                Direction::Forward,
            )],
            OpticalSensor::new(peripherals.port_10),
            settings.clone(),
            6.0,
            14.0,
        ),
        doinker: AdiDigitalOut::with_initial_level(peripherals.adi_c, LogicLevel::High),
        lady_brown: LadyBrown::new(
            [Motor::new(
                peripherals.port_12,
                Gearset::Red,
                Direction::Forward,
            )],
            RotationSensor::new(peripherals.port_13, Direction::Reverse),
            None,
        ),
        clamp: (
            AdiDigitalOut::with_initial_level(peripherals.adi_g, LogicLevel::Low),
            AdiDigitalOut::with_initial_level(peripherals.adi_b, LogicLevel::High),
        ),
        controller: peripherals.primary_controller,
        settings: settings.clone(),
    };

    robot.compete().await;
}

