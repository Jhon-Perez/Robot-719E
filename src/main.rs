#![no_main]
#![no_std]

extern crate alloc;

mod chassis;
mod mappings;
mod odometry;
mod pid;
mod vector;

use core::time::Duration;

use alloc::{boxed::Box, sync::Arc};

// use slint::{ComponentHandle, SharedString};

use chassis::{Chassis, TargetType};
use mappings::{ControllerMappings, DriveMode};
use odometry::{Odometry, Pose};
use pid::Pid;
use vexide::{
    core::{sync::Mutex, time::Instant},
    prelude::*,
    startup::banner::themes::THEME_MURICA,
};

// slint::include_modules!();

struct Robot {
    chassis: chassis::Chassis,

    intake: Motor,
    lady_brown: Motor,

    clamp: (AdiDigitalOut, AdiDigitalOut),

    controller: Controller,

    pub auton_selection: u8,
    pub pose: Arc<Mutex<Pose>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous control started.");

        const AUTON_SELECTOR: u8 = 1;
        self.clamp.0.set_high().ok();
        self.clamp.1.set_high().ok();


        // Later on do some error handling or something instead of throwing away the error
        match AUTON_SELECTOR {
            0 => {
                self.chassis.set_drive_target(TargetType::Distance(40.0));
                self.chassis.run(0.5).await.ok();

                sleep(Duration::from_millis(500)).await;

                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE).ok();

                self.clamp.0.set_low().ok();
                self.clamp.1.set_low().ok();

                sleep(Duration::from_millis(250)).await;

                self.chassis.set_drive_target(TargetType::Distance(6.0));
                self.chassis.run(1.0).await.ok();

                self.chassis.set_turn_target(TargetType::Distance(90.0));
                self.chassis.run(1.0).await.ok();

                self.chassis.set_drive_target(TargetType::Distance(-26.0));
                self.chassis.run(1.0).await.ok();

                self.chassis.set_turn_target(TargetType::Distance(180.0));
                self.chassis.run(1.0).await.ok();

                self.chassis.set_drive_target(TargetType::Distance(-16.0));
                self.chassis.run(1.0).await.ok();

                self.chassis.set_turn_target(TargetType::Distance(90.0));
                self.chassis.run(1.0).await.ok();

                self.chassis.set_drive_target(TargetType::Distance(20.0));
                self.chassis.run(1.0).await.ok();

                self.clamp.0.set_high().ok();
                self.clamp.1.set_high().ok();
            }
            1 => {}
            _ => (),
        }
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        const LADY_ANGLES: [f64; 4] = [
            100.0, // flick tolerance
            30.0,  // Intake
            110.0, // Align
            135.0, // Scoring
        ];

        let mut lady_stages = (0..LADY_ANGLES.len()).cycle();
        let mut stage = 0;

        self.clamp.0.set_high().ok();
        self.clamp.1.set_high().ok();

        loop {
            let delay = Instant::now() + Controller::UPDATE_INTERVAL;

            let state = self.controller.state().unwrap_or_default();

            let mappings = ControllerMappings {
                drive_mode: DriveMode::Arcade {
                    arcade: state.right_stick,
                },
                intake: state.button_r1,
                outake: state.button_r2,
                lady_brown: state.button_y,
                clamp: state.button_l1,
                drive_pid_test: state.button_x,
                turn_pid_test: state.button_a,
            };

            let power = self.chassis.differential_drive(&mappings);
            self.chassis.set_voltage(power);

            if mappings.intake.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE).ok();
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE).ok();
            } else {
                self.intake.brake(BrakeMode::Coast).ok();
            }

            if let Ok(angle) = self.lady_brown.position() {
                if mappings.lady_brown.is_now_pressed() {
                    stage = lady_stages.next().unwrap();
                }

                let angle = angle.as_degrees(); // if needed multiply with gear ratio
                match stage {
                    0 => {
                        if angle > LADY_ANGLES[stage] {
                            self.lady_brown.set_voltage(-Motor::V5_MAX_VOLTAGE).ok();
                        } else {
                            self.lady_brown.brake(BrakeMode::Coast).ok();
                        }
                    }
                    1..4 => {
                        if angle < LADY_ANGLES[stage] {
                            self.lady_brown.set_voltage(Motor::V5_MAX_VOLTAGE).ok();
                        } else {
                            self.lady_brown.brake(BrakeMode::Hold).ok();
                        }
                    }
                    _ => unreachable!(),
                }
            }

            if mappings.turn_pid_test.is_pressed() {
                self.chassis.set_turn_target(TargetType::Distance(180.0));
                self.chassis.run(1.0).await.ok();
            } else if mappings.drive_pid_test.is_pressed() {
                self.chassis.set_drive_target(TargetType::Distance(10.0));
                self.chassis.run(1.0).await.ok();
            }

            if mappings.clamp.is_now_pressed() {
                self.clamp.0.toggle().ok();
                self.clamp.1.toggle().ok();
            }

            sleep_until(delay).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_MURICA))]
async fn main(peripherals: Peripherals) {
    println!("Program started.");

    let left_motors = Box::new([
        Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
    ]);
    let right_motors = Box::new([
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_16, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
    ]);

    let mut imu = InertialSensor::new(peripherals.port_19);
    match imu.calibrate().await {
        Ok(_) => println!("Inertial Sensor Sucessfully Calibrated"),
        Err(_) => println!("Inertial Error"),
    }

    let chassis = Chassis::new(
        left_motors,
        right_motors,
        Pid::new(0.1, 0.0, 0.0, 0.0),  // linear
        Pid::new(0.1, 0.005, 1.25, 0.20), // angular
        imu,
    );

    let robot = Robot {
        chassis,
        intake: Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
        lady_brown: Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
        clamp: (
            AdiDigitalOut::new(peripherals.adi_h),
            AdiDigitalOut::new(peripherals.adi_g),
        ),
        controller: peripherals.primary_controller,
        auton_selection: 0,
        pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, 0.0))),
    };

    let odometry = Odometry::new(
        robot.pose.clone(),
        RotationSensor::new(peripherals.port_10, Direction::Forward),
        RotationSensor::new(peripherals.port_9, Direction::Forward),
        InertialSensor::new(peripherals.port_3),
    );

    // initialize_slint_platform(peripherals.screen);
    odometry::start(odometry);

    // let app = App::new().unwrap();

    // let selected_value_backend = Rc::new(RefCell::new(String::new()));

    // app.on_selected_index_changed(|index| {
    //     robot.auton_selection = index;
    //     println!("Selected item: {}", robot.auton_selection);
    // });

    // app.run().unwrap();

    robot.compete().await;
}
