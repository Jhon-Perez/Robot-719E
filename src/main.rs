#![no_main]
#![no_std]

extern crate alloc;

mod chassis;
mod odometry;
mod pid;
mod vector;

use alloc::{boxed::Box, sync::Arc};
use chassis::{Chassis, TargetType};
use odometry::{start, Odometry, Pose};
use pid::Pid;
use vexide::{
    core::{sync::Mutex, time::Instant}, prelude::*,
    startup::banner::themes::THEME_MURICA,
};

// slint::include_modules!();

// 4x 25 by c-channels, flick 10, flick pt 2 8, bracing 25, 4 bar lift n 2x 20

struct Robot {
    pub chassis: chassis::Chassis,

    pub intake: Motor,
    pub flick: Motor,
    pub lift: Motor,

    pub color_sensor: OpticalSensor,
    pub color_guard: AdiSolenoid,
    pub clamp: AdiSolenoid,

    pub controller: Controller,

    pub pose: Arc<Mutex<Pose>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous control started.");

        let somewhere = vector::Vec2::new(10.0, 10.0);
        self.chassis
            .set_drive_target(TargetType::Coordinate(somewhere));
        self.chassis.set_turn_target(TargetType::Distance(100.0));

        self.chassis.run(self.pose.clone()).await;
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        const FLICK_MAX_ANGLE: f64 = 180.0;
        const FLICK_TOLERANCE: f64 = 10.0;
        let mut turn = false;
        let mut halfway_mark = false;

        loop {
            let start_time = Instant::now();

            let power = self.chassis.differential_drive(&self.controller);
            self.chassis.set_voltage(power);

            if self.controller.right_trigger_1.is_pressed().unwrap_or_default() {
                self.intake.set_voltage(Motor::MAX_VOLTAGE).ok();
            } else if self.controller.left_trigger_1.is_pressed().unwrap_or_default() {
                self.intake.set_voltage(-Motor::MAX_VOLTAGE).ok();
            } else {
                self.intake.brake(BrakeMode::Coast).ok();
            }

            if self.controller.button_up.is_pressed().unwrap_or_default() {
                self.lift.set_voltage(Motor::MAX_VOLTAGE).ok();
            } else if self.controller.button_down.is_pressed().unwrap_or_default() {
                self.lift.set_voltage(-Motor::MAX_VOLTAGE).ok();
            } else {
                self.lift.brake(BrakeMode::Coast).ok();
            }

            if self.controller.button_x.is_pressed().unwrap_or_default() {
                turn = true;
            }

            if turn {
                if let Ok(angle) = self.flick.position() {
                    if halfway_mark {
                        if angle.as_degrees() < FLICK_MAX_ANGLE {
                            self.flick.set_voltage(Motor::MAX_VOLTAGE).ok();
                        } else {
                            halfway_mark = false;
                        }
                    } else if angle.as_degrees() > FLICK_TOLERANCE {
                        self.flick.set_voltage(Motor::MAX_VOLTAGE).ok();
                    } else {
                        halfway_mark = true;
                        turn = false;
                    }
                }
            }

            if self.controller.button_a.was_pressed().unwrap_or_default() {
                self.clamp.toggle().ok();
            }

            if self.controller.button_b.was_pressed().unwrap_or_default() {
                self.color_guard.toggle().ok();
            }
            
            // let hue = self.color_sensor.hue().unwrap();
            // let rgb = self.color_sensor.rgb().unwrap();
            // let raw_rgb = self.color_sensor.raw().unwrap();
            // let proximity = self.color_sensor.proximity().unwrap();
            // let brightness = self.color_sensor.led_brightness().unwrap();
            // println!("{}", hue);
            // println!("{:?}", rgb);
            // println!("{:?}", raw_rgb);
            // println!("{}", proximity);
            // println!("{}", brightness);
            // println!();

            sleep_until(start_time + Controller::UPDATE_INTERVAL).await;
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
        Motor::new(peripherals.port_20, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_19, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward),
    ]);
    let right_motors = Box::new([
        Motor::new(peripherals.port_11, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
    ]);

    let chassis = Chassis::new(
        left_motors,
        right_motors,
        chassis::JoystickType::SplitArcade,
        Pid::new(0.0, 0.0, 0.0, 0.0),
        Pid::new(0.0, 0.0, 0.0, 0.0),
    );

    let robot = Robot {
        chassis,
        intake: Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
        flick: Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),
        lift: Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
        color_sensor: OpticalSensor::new(peripherals.port_9),
        color_guard: AdiSolenoid::new(peripherals.adi_a),
        clamp: AdiSolenoid::new(peripherals.adi_b),
        controller: peripherals.primary_controller,
        pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, 0.0))),
    };

    let odometry = Odometry::new(
        robot.pose.clone(),
        RotationSensor::new(peripherals.port_13, Direction::Forward),
        RotationSensor::new(peripherals.port_12, Direction::Forward),
        InertialSensor::new(peripherals.port_8),
    );

    start(odometry);

    robot.compete().await;
}
