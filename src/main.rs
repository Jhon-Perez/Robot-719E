#![no_main]
#![no_std]

extern crate alloc;

mod autonomous;
mod backend;
mod mappings;
mod pose;
mod subsystems;

use alloc::{rc::Rc, vec::Vec};
use core::{cell::RefCell, time::Duration};

use autonomous::{command::Command, execute::{execute_command, ANGULAR_CONTROLLER, LINEAR_CONTROLLER, TOLERANCES}};
use backend::{Color, initialize_slint_gui};
use evian::{differential::motion::BasicMotion, drivetrain::Drivetrain, math::Vec2, prelude::*};
use mappings::{ControllerMappings, DriveMode};
use subsystems::{drivetrain::differential_drive, intake::{Intake, IntakeCommand}, lady_brown::LadyBrown};
use vexide::{
    core::time::Instant, devices::adi::digital::LogicLevel, prelude::*,
    startup::banner::themes::THEME_MURICA,
};

const TRACK_WIDTH: f64 = 12.75;
const DRIVE_RPM: f64 = 450.0;
const GEARING: f64 = 36.0 / 48.0;
const WHEEL_DIAMETER: f64 = 3.25;

pub struct RobotSettings {
    pub auton_path: Vec<Command>,
    pub test_auton: bool,
    pub curr_color: Color,
}

struct Robot {
    drivetrain: Drivetrain<Differential, ParallelWheelTracking>,

    intake: Intake,
    intake_lift: AdiDigitalOut,
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

        for &command in auton_path.iter().skip(1) {
            execute_command(self, command).await;
        }
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        // for testing and tuning PID
        let mut basic = BasicMotion {
            linear_controller: LINEAR_CONTROLLER,
            angular_controller: ANGULAR_CONTROLLER,
            linear_tolerances: TOLERANCES,
            angular_tolerances: TOLERANCES,
        };

        _ = self.intake_lift.set_low();
        _ = self.intake_lift.set_high();

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
                intake_lift: state.button_a,
                toggle_color_sort: state.button_a,
                lady_brown: state.button_x,
                clamp: state.button_r1,
                test_linear: state.button_up,
                test_angular: state.button_right,
                test_alliance: state.button_down,
            };

            let power = differential_drive(&mappings.drive_mode);
            _ = self.drivetrain.motors.set_voltages(power);

            // neaten with refactor
            if mappings.intake.is_pressed() {
                self.intake.set_command(IntakeCommand::On);
            } else if mappings.outake.is_pressed() {
                self.intake.set_command(IntakeCommand::Voltage(-Motor::V5_MAX_VOLTAGE));
            } else {
                self.intake.set_command(IntakeCommand::Off);
            }

            if mappings.toggle_color_sort.is_now_pressed() {
                self.intake.toggle_color_sort();
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

            if mappings.test_linear.is_now_pressed() {
                basic.drive_distance(&mut self.drivetrain, 24.0).await;
            }

            if mappings.test_angular.is_now_pressed() {
                basic.turn_to_heading(&mut self.drivetrain, 0.0.deg()).await;
            }

            if mappings.test_alliance.is_now_pressed() {
                self.intake.set_command(IntakeCommand::Voltage(4.0));
                sleep(Duration::from_millis(250)).await;
            }

            if mappings.clamp.is_now_pressed() {
                _ = self.clamp.0.toggle();
                _ = self.clamp.1.toggle();
            }

            if mappings.intake_lift.is_now_pressed() {
                _ = self.intake_lift.toggle();
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

    initialize_slint_gui(peripherals.display, settings.clone());

    let mut imu = InertialSensor::new(peripherals.port_17);

    match imu.calibrate().await {
        Ok(_) => println!("Calibration Successful"),
        Err(e) => println!("Error {:?}", e),
    }

    let left_motors = shared_motors![
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
    ];
    let right_motors = shared_motors![
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_16, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
    ];

    let robot = Robot {
        drivetrain: Drivetrain::new(
            Differential::new(
                left_motors.clone(),
                right_motors.clone(),           
            ),
            ParallelWheelTracking::new(
                Vec2::default(),
                Angle::default(),
                TrackingWheel::new(
                    left_motors.clone(),
                    WHEEL_DIAMETER,
                    TRACK_WIDTH,
                    Some(36.0 / 48.0),
                ),
                TrackingWheel::new(
                    right_motors.clone(),
                    WHEEL_DIAMETER,
                    TRACK_WIDTH,
                    Some(36.0 / 48.0),
                ),
                Some(imu),
            ),
        ),
        intake: Intake::new(
            [
                Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                Motor::new_exp(peripherals.port_1, Direction::Forward),
            ],
            OpticalSensor::new(peripherals.port_20),
            settings.clone(),
            6.0,
            18.0,
        ),
        intake_lift: AdiDigitalOut::with_initial_level(peripherals.adi_h, LogicLevel::High),
        lady_brown: LadyBrown::new(
            [
                Motor::new_exp(peripherals.port_6, Direction::Reverse),
            ],
            RotationSensor::new(peripherals.port_9, Direction::Reverse),
            Some(12.0 / 36.0),
        ),
        clamp: (
            AdiDigitalOut::with_initial_level(peripherals.adi_a, LogicLevel::Low),
            AdiDigitalOut::with_initial_level(peripherals.adi_b, LogicLevel::High),
        ),
        controller: peripherals.primary_controller,
        settings: settings.clone(),
    };

    robot.compete().await;
}
