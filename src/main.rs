#![no_main]
#![no_std]

extern crate alloc;

mod chassis;
mod command;
mod image_gen;
mod mappings;
mod odometry;
mod pid;
mod subsystems;
mod vector;

use alloc::{boxed::Box, rc::Rc, sync::Arc, vec, vec::Vec};
use core::{borrow::BorrowMut, cell::RefCell, time::Duration};

use chassis::{Chassis, TargetType};
use command::Command;
use mappings::{ControllerMappings, DriveMode};
use odometry::{Odometry, Pose};
use pid::Pid;
use slint::ComponentHandle;
use subsystems::{intake::Intake, lady_brown::LadyBrown};
use vector::Vec2;
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::adi::digital::LogicLevel,
    prelude::*,
    startup::banner::themes::THEME_MURICA,
};

slint::include_modules!();

struct Robot {
    chassis: chassis::Chassis,

    intake: Intake,
    intake_lift: AdiDigitalOut,
    lady_brown: Arc<Mutex<LadyBrown>>,

    clamp: AdiDigitalOut,

    controller: Controller,

    pub auton_path: Rc<RefCell<Vec<Command>>>,
    pub pose: Arc<Mutex<Pose>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous control started.");

        let lady_clone = self.lady_brown.clone();
        spawn(async move {
            let mut data = lady_clone.lock().await;
            data.run();
        })
        .detach();

        for command in self.auton_path.replace(vec![]) {
            match command {
                Command::Coordinate(coord) => {
                    let (radius, angle) = coord.to_polar();

                    self.chassis.set_turn_target(TargetType::Distance(angle));
                    _ = self.chassis.run(1.0).await;

                    self.chassis.set_drive_target(TargetType::Distance(radius));
                    _ = self.chassis.run(1.0).await;
                }
                Command::ToggleIntake => {
                    self.intake.toggle();
                }
                Command::ToggleClamp => {
                    _ = self.clamp.toggle();
                }
                Command::ToggleLadyBrown => {
                    let mut next_data = self.lady_brown.lock().await;
                    next_data.next();
                }
                Command::Sleep(delay) => {
                    sleep(Duration::from_millis(delay)).await;
                }
                _ => unreachable!(),
            };
        }

        // Later on do some error handling or something instead of throwing away the error
        //match self.auton_selected {
        //    0 => {
        //        //let coord = path_iteration.next().expect("path ended early");
        //        //self.chassis.run_coord(coord, 0.5).await.ok();
        //
        //        self.chassis.set_drive_target(TargetType::Distance(40.0));
        //        self.chassis.run(0.5).await.ok();
        //
        //        sleep(Duration::from_millis(500)).await;
        //
        //        self.intake.set_voltage(Motor::V5_MAX_VOLTAGE).ok();
        //
        //        self.clamp.0.set_low().ok();
        //        self.clamp.1.set_low().ok();
        //
        //        sleep(Duration::from_millis(250)).await;
        //
        //        self.chassis.set_drive_target(TargetType::Distance(6.0));
        //        self.chassis.run(1.0).await.ok();
        //
        //        self.chassis.set_turn_target(TargetType::Distance(90.0));
        //        self.chassis.run(1.0).await.ok();
        //
        //        self.chassis.set_drive_target(TargetType::Distance(-26.0));
        //        self.chassis.run(1.0).await.ok();
        //
        //        self.chassis.set_turn_target(TargetType::Distance(180.0));
        //        self.chassis.run(1.0).await.ok();
        //
        //        self.chassis.set_drive_target(TargetType::Distance(-16.0));
        //        self.chassis.run(1.0).await.ok();
        //
        //        self.chassis.set_turn_target(TargetType::Distance(90.0));
        //        self.chassis.run(1.0).await.ok();
        //
        //        self.chassis.set_drive_target(TargetType::Distance(20.0));
        //        self.chassis.run(1.0).await.ok();
        //
        //        self.clamp.0.set_high().ok();
        //        self.clamp.1.set_high().ok();
        //    }
        //    1 => {}
        //    _ => (),
        //}
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
        auton_path: Rc::new(RefCell::new(Vec::new())),
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

    initialize_slint_platform(peripherals.display);

    // Pseudo numbers for what a path could look like
    let auton_paths = [
        vec![
            Command::Coordinate(Vec2 { x: 0.0, y: 0.0 }),
            Command::Coordinate(Vec2 { x: 0.0, y: 0.0 }),
        ],
        vec![ // path 1
            Command::Coordinate(Vec2 {
                x: 0.0,
                y: 48.0,
            }), // START THE PATH WITH ITS STARTING POINT
            Command::DriveBy(46.0), // move forward to clamp goal
            Command::Sleep(500),
            Command::ToggleIntake,
            Command::ToggleClamp,
            Command::Sleep(250),
            Command::DriveBy(6.0),
            Command::TurnTo(90.0), // turn to face rings
            Command::DriveBy(-26.0), // drive towards rings
            Command::TurnTo(180.0), // turn 45 degrees right
            Command::DriveBy(-16.0),
            Command::TurnTo(90.0),
            Command::DriveBy(20.0),
            Command::ToggleIntake,
        ],
        vec![
            // path 2
            Command::Coordinate(Vec2 {
                x: 24.0, y: 132.0
            }),
            Command::DriveBy(32.0),
            Command::TurnTo(-50.0),
            Command::DriveBy(20.0),
            Command::TurnTo(0.0),
            Command::DriveBy(-18.0),
            Command::TurnTo(135.0),
            Command::DriveBy(-24.0),
        ],
    ];

    let mut path_coords = Vec::new();
    let mut updated_paths = Vec::new();

    for path in auton_paths.iter() {
        let (path_coord, updated_path) = command::command_to_coords(&path);
        path_coords.push(path_coord);
        updated_paths.push(updated_path);
    }

    let app = AppWindow::new().unwrap();

    app.on_autonomous({
        let ui_handler = app.as_weak();
        let mut auton_path = robot.auton_path.clone();

        move |autonomous| {
            println!("{:?}", autonomous);

            // Each robot starts across from each other, so the oppisite side can
            // be replicated by flipping the x-axis. Right now this implementation
            // is ugly with clone, will try to find a cleaner solution
            //let color;
            //let index = autonomous.index as usize;
            //let (coords, path) = if autonomous.color == "red" {
            //    color = [255u8, 0u8, 0u8, 255u8];
            //    (path_coords[index].clone(), updated_paths[index].clone())
            //} else {
            //    let mut coords = Vec::new();
            //    for coord in &path_coords[index] {
            //        coords.push(Vec2 {
            //            x: 144.0 - coord.x,
            //            y: coord.y,
            //        });
            //    }
            //
            //    let mut path = Vec::new();
            //    for coord in &updated_paths[index] {
            //        match coord {
            //            Command::Coordinate(coord) => {
            //                path.push(Command::Coordinate(Vec2 {
            //                    x: 144.0 - coord.x,
            //                    y: coord.y,
            //                }));
            //            }
            //            Command::TurnTo(..) | Command::TurnBy(..) | Command::DriveBy(..) => (),
            //            _ => path.push(*coord),
            //        }
            //    }
            //
            //    color = [0u8, 0u8, 255u8, 255u8];
            //    (coords, path)
            //};
            fn invert_coords(coords: &[Vec2]) -> Vec<Vec2> {
                coords.iter().map(|coord| Vec2 { x: 144.0 - coord.x, y: coord.y }).collect()
            }

            fn invert_commands(commands: &[Command]) -> Vec<Command> {
                commands
                    .iter()
                    .map(|command| match command {
                        Command::Coordinate(coord) => Command::Coordinate(Vec2 { x: 144.0 - coord.x, y: coord.y }),
                        Command::TurnTo(..) | Command::TurnBy(..) | Command::DriveBy(..) => command.clone(),
                        _ => *command,
                    })
                    .collect()
            }

            let index = autonomous.index as usize;

            let (coords, path, color) = if autonomous.color == "red" {
                (
                    path_coords[index].clone(),
                    updated_paths[index].clone(),
                    [255u8, 0u8, 0u8, 255u8],
                )
            } else {
                (
                    invert_coords(&path_coords[index]),
                    invert_commands(&updated_paths[index]),
                    [0u8, 0u8, 255u8, 255u8],
                )
            };
            let image = image_gen::coord_to_img(144, 144, color, &coords);

            {
                let curr_path = auton_path.borrow_mut();
                curr_path.replace(path);
            }

            let ui = ui_handler.unwrap();
            ui.set_path_image(image);
        }
    });

    spawn(async move {
        _ = app.show();
        _ = slint::run_event_loop();
    })
    .detach();

    robot.compete().await;
}
