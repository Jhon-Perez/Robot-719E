#![no_main]
#![no_std]

extern crate alloc;

mod chassis;
mod mappings;
mod odometry;
mod pid;
mod subsystems;
mod vector;

use core::time::Duration;

use alloc::{boxed::Box, sync::Arc};

// use slint::{ComponentHandle, SharedString};

use chassis::{Chassis, TargetType};
use mappings::{ControllerMappings, DriveMode};
use odometry::{Odometry, Pose};
use pid::Pid;
use subsystems::{intake::Intake, lady_brown::LadyBrown};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::adi::digital::LogicLevel,
    prelude::*,
    startup::banner::themes::THEME_MURICA,
};

// slint::include_modules!();

struct Robot {
    chassis: chassis::Chassis,

    intake: Intake,
    intake_lift: AdiDigitalOut,
    lady_brown: Arc<Mutex<LadyBrown>>,

    clamp: AdiDigitalOut,

    controller: Controller,

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

        loop {
            let delay = Instant::now() + Controller::UPDATE_INTERVAL;

            let state = self.controller.state().unwrap_or_default();

            let mappings = ControllerMappings {
                drive_mode: DriveMode::Arcade {
                    arcade: state.right_stick,
                },
                intake: state.button_r1,
                outake: state.button_r2,
                intake_lift: state.button_b,
                lady_brown: state.button_y,
                clamp: state.button_a,
                drive_pid_test: state.button_left,
                turn_pid_test: state.button_right,
            };

            let power = self.chassis.differential_drive(&mappings);
            self.chassis.set_voltage(power);

            if mappings.intake.is_pressed() {
                self.intake.set_voltage(Motor::V5_MAX_VOLTAGE);
            } else if mappings.outake.is_pressed() {
                self.intake.set_voltage(-Motor::V5_MAX_VOLTAGE);
            } else {
                self.intake.brake(BrakeMode::Coast);
            }

            if mappings.lady_brown.is_now_pressed() {
                let mut next_data = self.lady_brown.lock().await;
                next_data.next();
            }

            {
                let mut next_data = self.lady_brown.lock().await;
                next_data.run();
            }

            if mappings.turn_pid_test.is_pressed() {
                self.chassis.set_turn_target(TargetType::Distance(0.0));
                self.chassis.run(1.0).await.ok();
            } else if mappings.drive_pid_test.is_pressed() {
                self.chassis.set_drive_target(TargetType::Distance(10.0));
                self.chassis.run(1.0).await.ok();
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

#[vexide::main(banner(theme = THEME_MURICA))]
async fn main(peripherals: Peripherals) {
    println!("Program started.");

    let left_motors = Box::new([
        Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
    ]);
    let right_motors = Box::new([
        Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
    ]);

    let mut imu = InertialSensor::new(peripherals.port_5);
    match imu.calibrate().await {
        Ok(_) => println!("Inertial Sensor Sucessfully Calibrated"),
        Err(_) => println!("Inertial Error"),
    }

    let chassis = Chassis::new(
        left_motors,
        right_motors,
        Pid::new(0.0, 0.0, 0.0, 0.0),   // linear
        Pid::new(0.125, 0.0, 0.25, 0.0), // angular
        imu,
    );

    let robot = Robot {
        chassis,
        intake: Intake::new(Motor::new(
            peripherals.port_2,
            Gearset::Blue,
            Direction::Forward,
        )),
        intake_lift: AdiDigitalOut::with_initial_level(peripherals.adi_g, LogicLevel::Low),
        lady_brown: Arc::new(Mutex::new(LadyBrown::new(
            (
                Motor::new_exp(peripherals.port_20, Direction::Reverse),
                Motor::new_exp(peripherals.port_17, Direction::Forward),
            ),
            12.0 / 60.0,
        ))),
        clamp: AdiDigitalOut::with_initial_level(peripherals.adi_h, LogicLevel::Low),
        controller: peripherals.primary_controller,
        pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, 0.0))),
    };

    //let mut imu_sensor = InertialSensor::new(peripherals.port_14);
    //match imu_sensor.calibrate().await {
    //    Ok(_) => println!("Inertial sensor successfully calibrated"),
    //    Err(e) => println!("Inertial Error {:?}", e),
    //}
    //
    //let odometry = Odometry::new(
    //    robot.pose.clone(),
    //    RotationSensor::new(peripherals.port_4, Direction::Forward),
    //    RotationSensor::new(peripherals.port_5, Direction::Forward),
    //    //InertialSensor::new(peripherals.port_3),
    //    imu_sensor,
    //);
    //
    //odometry::start(odometry);

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
