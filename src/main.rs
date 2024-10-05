#![no_main]
#![no_std]

extern crate alloc;

mod chassis;
mod odometry;
mod pid;
mod vector;

use alloc::{boxed::Box, sync::Arc};

use chassis::{Chassis, TargetType};
use odometry::Pose;
use pid::Pid;
use vexide::{core::sync::Mutex, devices::adi::digital::LogicLevel, prelude::*, startup::banner::themes::THEME_MURICA};

struct Robot {
    pub chassis: chassis::Chassis,

    pub intake: Motor,
    pub color_sensor: OpticalSensor,
    pub color_guard: (AdiDigitalOut, LogicLevel),
    pub clamp: (AdiDigitalOut, LogicLevel),

    pub controller: Controller,

    pub pose: Arc<Mutex<Pose>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let somewhere = vector::Vec2::new(10.0, 10.0);
        self.chassis.set_drive_target(TargetType::Coordinate(somewhere));
        self.chassis.set_turn_target(TargetType::Distance(100.0));

        self.chassis.run(self.pose.clone()).await;
    }

    async fn driver(&mut self) {
        loop {
            let power = self.chassis.differential_drive(&self.controller);
            self.chassis.set_motors(power);

            if self.controller.right_trigger_1.is_pressed().unwrap_or_default() {
                self.intake.set_voltage(Motor::MAX_VOLTAGE).ok();
            } else if self.controller.left_trigger_1.is_pressed().unwrap_or_default() {
                self.intake.set_voltage(-Motor::MAX_VOLTAGE).ok();
            } else {
                self.intake.brake(BrakeMode::Coast).ok();
            }
            
            // if self.controller.button_a.was_pressed().unwrap_or_default() {
            //     self.clamp.1 = !self.clamp.1;
            //     self.clamp.0.set_level(self.clamp.1).unwrap();
            // } 
            
            // if self.controller.button_b.was_pressed().unwrap_or_default() {
            //     self.color_guard.1 = !self.color_guard.1;
            //     self.color_guard.0.set_level(self.color_guard.1).unwrap();
            // }

            // let hue = self.color_sensor.hue().unwrap_or_default();
            // println!("{}", hue);

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_MURICA))]
async fn main(peripherals: Peripherals) {
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
        color_sensor: OpticalSensor::new(peripherals.port_9),
        color_guard: (AdiDigitalOut::new(peripherals.adi_a), LogicLevel::Low),
        clamp: (AdiDigitalOut::new(peripherals.adi_b), LogicLevel::Low),
        controller: peripherals.primary_controller,
        pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, 0.0))),
    };

    let mut imu_sensor = InertialSensor::new(peripherals.port_8);
    let left_pod = RotationSensor::new(peripherals.port_13, Direction::Forward);
    let right_pod = RotationSensor::new(peripherals.port_12, Direction::Forward);

    imu_sensor.calibrate().await.unwrap();

    spawn(odometry::step_math(
        robot.pose.clone(),
        left_pod,
        right_pod,
        imu_sensor,
    ))
    .detach();

    robot.compete().await;
}
