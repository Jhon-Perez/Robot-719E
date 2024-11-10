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
use odometry::{start, Odometry, Pose};
use pid::Pid;
use vexide::{
    core::{sync::Mutex, time::Instant},
    prelude::*,
    startup::banner::themes::THEME_MURICA,
};

// slint::include_modules!();

struct Robot {
    chassis: chassis::Chassis,

    intake: (Motor, Motor),

    clamp: (AdiSolenoid, AdiSolenoid),

    controller: Controller,

    pub auton_selection: u8,
    pub pose: Arc<Mutex<Pose>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous control started.");

        const AUTON_SELECTOR: u8 = 1;

        self.clamp.0.open().ok();
        self.clamp.1.open().ok();

        // Later on do some error handling or something instead of throwing away the error
        match AUTON_SELECTOR {
            0 => {
                self.chassis.set_drive_target(TargetType::Distance(40.0));
                self.chassis.run(0.5).await.ok();

                sleep(Duration::from_millis(500)).await;

                self.intake.0.set_voltage(Motor::MAX_VOLTAGE).ok();
                self.intake.0.set_voltage(Motor::MAX_VOLTAGE).ok();

                self.clamp.0.close().ok();
                self.clamp.1.close().ok();

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

                self.clamp.0.open().ok();
                self.clamp.1.open().ok();
            }
            1 => {
                self.clamp.0.close().ok();
                self.clamp.1.close().ok();
                self.chassis.set_drive_target(TargetType::Distance(-52.0));
                self.chassis.run(0.4).await.ok();

                self.clamp.0.open().ok();
                self.clamp.1.open().ok();

                self.intake.0.set_velocity(250).ok();
                self.intake.1.set_velocity(250).ok();

                sleep(Duration::from_millis(1000)).await;

                self.intake.0.set_velocity(0).ok();
                self.intake.1.set_velocity(0).ok();

                self.chassis.set_turn_target(TargetType::Distance(30.0));
                self.chassis.run(1.0).await.ok();
            }
            _ => (),
        }
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        let mappings = ControllerMappings {
            drive_mode: DriveMode::Arcade {
                arcade: &self.controller.right_stick,
            },
            intake: &self.controller.right_trigger_1,
            outake: &self.controller.right_trigger_2,
            clamp: &mut self.controller.left_trigger_1,
            drive_pid_test: &self.controller.button_x,
            turn_pid_test: &self.controller.button_a,
        };

        self.clamp.0.open().ok();
        self.clamp.1.open().ok();

        loop {
            let start_time = Instant::now();

            let power = self.chassis.differential_drive(&mappings);
            self.chassis.set_voltage(power);

            if mappings.intake.is_pressed().unwrap_or_default() {
                self.intake.0.set_voltage(Motor::MAX_VOLTAGE).ok();
                self.intake.1.set_voltage(Motor::MAX_VOLTAGE).ok();
            } else if mappings.outake.is_pressed().unwrap_or_default() {
                self.intake.0.set_voltage(-Motor::MAX_VOLTAGE).ok();
                self.intake.1.set_voltage(-Motor::MAX_VOLTAGE).ok();
            } else {
                self.intake.0.brake(BrakeMode::Coast).ok();
                self.intake.1.brake(BrakeMode::Coast).ok();
            }

            if mappings.turn_pid_test.is_pressed().unwrap_or_default() {
                self.chassis.set_turn_target(TargetType::Distance(180.0));
                self.chassis.run(1.0).await.ok();
            } else if mappings.drive_pid_test.is_pressed().unwrap_or_default() {
                self.chassis.set_drive_target(TargetType::Distance(10.0));
                self.chassis.run(1.0).await.ok();
            }

            if mappings.clamp.was_pressed().unwrap_or_default() {
                self.clamp.0.toggle().ok();
                self.clamp.1.toggle().ok();
            }

            sleep_until(start_time + Controller::UPDATE_INTERVAL).await;
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
        intake: (
            Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
        ),
        clamp: (AdiSolenoid::new(peripherals.adi_h), AdiSolenoid::new(peripherals.adi_g)),
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

    start(odometry);

    // initialize_slint_platform(peripherals.screen);

    // let app = App::new().unwrap();

    // let selected_value_backend = Rc::new(RefCell::new(String::new()));

    // app.on_selected_index_changed(|index| {
    //     robot.auton_selection = index;
    //     println!("Selected item: {}", robot.auton_selection);
    // });

    // app.run().unwrap();

    robot.compete().await;
}
