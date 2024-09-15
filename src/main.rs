#![no_main]
#![no_std]

extern crate alloc;

mod chassis;
mod odometry;
mod pid;
mod vector;

use alloc::{sync::Arc, vec};
use core::time::Duration;

use chassis::Chassis;
use odometry::Pose;
use pid::{AngularPid, LinearPid, Pid};
use vexide::{core::sync::Mutex, devices::adi::digital::LogicLevel, prelude::*};

struct Robot {
    pub chassis: chassis::Chassis,

    pub intake_motor: Motor,
    pub color_sensor: OpticalSensor,
    pub color_guard: (AdiDigitalOut, LogicLevel),
    pub clamp: (AdiDigitalOut, LogicLevel),

    pub controller: Controller,

    pub pose: Arc<Mutex<Pose>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let mut linear = LinearPid::new(0.0, 0.0, 0.0, 0.0);
        let mut angular = AngularPid::new(0.0, 0.0, 0.0, 0.0);

        // What is this ugly ahh code 😭
        // Rewrite pid in a better way
        linear.run(&mut self.chassis, self.pose.clone(), 10.0, 5000).await;
        angular.run(&mut self.chassis, self.pose.clone(), 180.0, 5000).await;
    }

    async fn driver(&mut self) {
        loop {
            let power = self.chassis.differential_drive(&self.controller);
            self.chassis.set_motors(power);

            self.intake();

            if self.controller.button_a.was_pressed().unwrap_or_default() {
                self.clamp.1 = !self.clamp.1;
                self.clamp.0.set_level(self.clamp.1).unwrap();
            } else if self.controller.button_b.was_pressed().unwrap_or_default() {
                self.color_guard.1 = !self.color_guard.1;
                self.color_guard.0.set_level(self.color_guard.1).unwrap();
            }

            let hue = self.color_sensor.hue().unwrap_or_default();
            println!("{}", hue);

            sleep(Duration::from_millis(10)).await;
        }
    }
}

impl Robot {
    pub fn intake(&mut self) {
        if self
            .controller
            .right_trigger_1
            .is_pressed()
            .unwrap_or_default()
        {
            self.intake_motor.set_voltage(12.0).unwrap();
        } else if self
            .controller
            .left_trigger_1
            .is_pressed()
            .unwrap_or_default()
        {
            self.intake_motor.set_voltage(-12.0).unwrap();
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = vec![
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward),
    ];
    let right_motors = vec![
        Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_5, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_6, Gearset::Green, Direction::Reverse),
    ];

    let mut chassis = Chassis::new(
        left_motors,
        right_motors,
        chassis::JoystickType::SplitArcade,
    );
    chassis.set_joystick_type(chassis::JoystickType::SplitArcade);

    let robot = Robot {
        chassis,
        intake_motor: Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
        color_sensor: OpticalSensor::new(peripherals.port_9),
        color_guard: (AdiDigitalOut::new(peripherals.adi_a), LogicLevel::Low),
        clamp: (AdiDigitalOut::new(peripherals.adi_b), LogicLevel::Low),
        controller: peripherals.primary_controller,
        pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, 0.0))),
    };

    let imu_sensor = InertialSensor::new(peripherals.port_8);
    let left_pod = RotationSensor::new(peripherals.port_11, Direction::Forward);
    let right_pod = RotationSensor::new(peripherals.port_12, Direction::Forward);

    spawn(odometry::step_math(
        robot.pose.clone(),
        left_pod,
        right_pod,
        imu_sensor,
    ))
    .detach();

    robot.compete().await;
}
