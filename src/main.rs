#![no_main]
#![no_std]

extern crate alloc;

mod chassis;
mod mappings;
mod odometry;
mod pid;
mod vector;

use alloc::{boxed::Box, sync::Arc};
use chassis::{Chassis, TargetType};
use mappings::{ControllerMappings, DriveMode};
use odometry::{start, Odometry, Pose};
use pid::Pid;
use vexide::{
    core::{sync::Mutex, time::Instant}, prelude::*,
    prelude::InertialSensor,
    startup::banner::themes::THEME_MURICA,
};

// slint::include_modules!();

struct Robot {
    chassis: chassis::Chassis,

    intake: Motor,
    flick: Motor,
    lift: Motor,

    pub clamp: AdiSolenoid,
    pub doinker: AdiSolenoid,

    controller: Controller,

    pub pose: Arc<Mutex<Pose>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous control started.");

        let somewhere = vector::Vec2::new(10.0, 10.0);
        self.chassis
            .set_drive_target(TargetType::Coordinate(somewhere));
        self.chassis.set_turn_target(TargetType::Distance(100.0));

        self.chassis.run().await;
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        const FLICK_MAX_ANGLE: f64 = 135.0;
        const INTAKE_ANGLE: f64 = 30.0;
        const FLICK_TOLERANCE: f64 = 100.0;
        const FLICK_GEAR_RATIO: f64 = 36.0 / 60.0;
        let mut flick_stage = (0..4).cycle();
        // let mut flick_scored = false;
        // let mut turn = false;
        let mut stage = 0;

        let mappings = ControllerMappings {
            drive_mode: DriveMode::SplitArcade {
                power: &self.controller.left_stick,
                turn: &self.controller.right_stick,
            },
            intake: &self.controller.right_trigger_1,
            outake: &self.controller.right_trigger_2,
            lift_up: &self.controller.button_up,
            lift_down: &self.controller.button_down,
            flick: &mut self.controller.button_b,
            clamp: &mut self.controller.button_y,
            doinker: &mut self.controller.button_right,
        };

        loop {
            let start_time = Instant::now();

            let power = self.chassis.differential_drive(&mappings);
            self.chassis.set_voltage(power);

            if mappings.intake.is_pressed().unwrap_or_default() {
                self.intake.set_voltage(Motor::MAX_VOLTAGE).ok();
            } else if mappings.outake.is_pressed().unwrap_or_default() {
                self.intake.set_voltage(-Motor::MAX_VOLTAGE).ok();
            } else {
                self.intake.brake(BrakeMode::Coast).ok();
            }

            if mappings.lift_up.is_pressed().unwrap_or_default() {
                self.lift.set_voltage(Motor::MAX_VOLTAGE).ok();
            } else if mappings.lift_down.is_pressed().unwrap_or_default() {
                self.lift.set_voltage(-Motor::MAX_VOLTAGE).ok();
            } else {
                self.lift.brake(BrakeMode::Coast).ok();
            }

            // if self.controller.button_up.is_pressed().unwrap_or_default() {
            //     self.chassis.set_drive_target(TargetType::None);
            //     self.chassis.set_turn_target(TargetType::Distance(180.0));
            //     self.chassis.run().await;
            // } else if self.controller.button_down.is_pressed().unwrap_or_default() {
            //     self.chassis.set_drive_target(TargetType::None);
            //     self.chassis.set_turn_target(TargetType::None);
            //     self.chassis.run().await;
            // }

            // if mappings.flick.is_pressed().unwrap_or_default() {
            //     turn = true;
            // }

            // if turn {
            //     if let Ok(angle) = self.flick.position() {
            //         println!("t raw angle: {}", angle.as_degrees());
            //         let angle = angle.as_degrees();
            //         println!("angle: {}", angle);
            //         if !flick_scored {
            //             if angle < FLICK_MAX_ANGLE {
            //                 self.flick.set_voltage(10.0).ok();
            //             } else {
            //                 flick_scored = true;
            //             }
            //         } else if angle > FLICK_TOLERANCE {
            //             self.flick.set_voltage(-10.0).ok();
            //         } else {
            //             self.flick.brake(BrakeMode::Coast).ok();
            //             flick_scored = false;
            //             turn = false;
            //         }
            //     }
            // }

            if let Ok(angle) = self.flick.position() {
                if mappings.flick.was_pressed().unwrap_or_default() {
                    stage = flick_stage.next().unwrap();
                }
                let angle = angle.as_degrees() * FLICK_GEAR_RATIO;
                match stage {
                    1 => {
                        if angle < INTAKE_ANGLE {
                            self.flick.set_voltage(8.0).ok();
                        } else {
                            self.flick.brake(BrakeMode::Hold).ok();
                        }
                    }
                    2 => {
                        if angle < FLICK_MAX_ANGLE {
                            self.flick.set_voltage(8.0).ok();
                        } else {
                            self.flick.brake(BrakeMode::Hold).ok();
                        }
                    }
                    3 => {
                        if angle > FLICK_TOLERANCE {
                            self.flick.set_voltage(-8.0).ok();
                        } else {
                            self.flick.brake(BrakeMode::Coast).ok();
                            flick_stage.next();
                        }
                    }
                    _ => ()
                }
            }

            if mappings.clamp.was_pressed().unwrap_or_default() {
                self.clamp.toggle().ok();
            }

            if mappings.doinker.was_pressed().unwrap_or_default() {
                self.doinker.toggle().ok();
            }

            sleep_until(start_time + Controller::UPDATE_INTERVAL).await;
            // println!("{:?}", start_time);
            // sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_MURICA))]
async fn main(peripherals: Peripherals) {
    println!("Program started.");

    // initialize_slint_platform(peripherals.screen);

    // let ui = AppWindow::new().unwrap();

    // ui.on_request_increase_value({
    //     let ui_handler = ui.as_weak();
    //     move || {
    //         let ui = ui_handler.unwrap();
    //         ui.set_counter(ui.get_counter() + 1);
    //     }
    // });

    // ui.run().unwrap();

    let left_motors = Box::new([
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
    ]);
    let right_motors = Box::new([
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
    ]);

    let mut imu = InertialSensor::new(peripherals.port_6);
    match imu.calibrate().await {
        Ok(_) => (),
        Err(_) => println!("Inertial Error Bitch"),
    }

    let chassis = Chassis::new(
        left_motors,
        right_motors,
        Pid::new(0.0, 0.0, 0.0, 0.0), // linear
        Pid::new(0.001, 0.0, 0.0, 0.0), // angular
        imu,
    );

    let mut clamp = AdiSolenoid::new(peripherals.adi_a);
    let mut doinker = AdiSolenoid::new(peripherals.adi_b);

    clamp.open().ok();
    doinker.open().ok();

    let robot = Robot {
        chassis,
        intake: Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
        flick: Motor::new(peripherals.port_14, Gearset::Red, Direction::Forward),
        lift: Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward),
        clamp,
        doinker,
        controller: peripherals.primary_controller,
        pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, 0.0))),
    };

    let odometry = Odometry::new(
        robot.pose.clone(),
        RotationSensor::new(peripherals.port_13, Direction::Forward),
        RotationSensor::new(peripherals.port_12, Direction::Forward),
        InertialSensor::new(peripherals.port_11),
    );

    start(odometry);

    robot.compete().await;
}
