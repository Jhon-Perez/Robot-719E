#![no_main]
#![no_std]

extern crate alloc;

mod command;
mod image_gen;
mod mappings;
mod odometry;
// mod path;
mod pid;
// mod pure_pursuit;
mod subsystems;
mod vector;

use alloc::{boxed::Box, rc::Rc, sync::Arc, vec, vec::Vec};
use core::{borrow::BorrowMut, cell::RefCell, time::Duration};

use command::Command;
use mappings::{ControllerMappings, DriveMode};
use odometry::{Odometry, Pose};
use pid::Pid;
use slint::ComponentHandle;
use subsystems::{
    drivetrain::{Drivetrain, TargetType},
    intake::Intake,
    lady_brown::LadyBrown,
};
use vector::Vec2;
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::adi::digital::LogicLevel,
    prelude::*,
    startup::banner::themes::THEME_MURICA,
};

slint::include_modules!();

struct Robot {
    drivetrain: Drivetrain,

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

        //_ = self.intake_lift.toggle();

        for command in self.auton_path.borrow().iter() {
            match command {
                // no odom so coordinate won't be used
                Command::Coordinate(_coord) => {}
                Command::DriveBy(distance) => {
                    self.drivetrain.set_drive_target(TargetType::Distance(*distance));
                    _ = self.drivetrain.run().await;
                }
                Command::TurnTo(angle) => {
                    self.drivetrain.set_turn_target(TargetType::Distance(*angle));
                    _ = self.drivetrain.run().await;
                }
                Command::Speed(speed) => {
                    self.drivetrain.set_speed(*speed);
                }
                Command::ToggleIntake => {
                    self.intake.toggle(1.0);
                }
                Command::ToggleIntakeReverse => {
                    self.intake.toggle(-1.0);
                }
                Command::ToggleIntakePiston => {
                    _ = self.intake_lift.toggle();
                }
                Command::ToggleClamp => {
                    _ = self.clamp.toggle();
                }
                Command::ToggleLadyBrown => {
                    let mut next_data = self.lady_brown.lock().await;
                    next_data.next();
                }
                Command::Sleep(delay) => {
                    sleep(Duration::from_millis(*delay)).await;
                }
                _ => (),
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
                    arcade: state.left_stick,
                },
                //drive_mode: DriveMode::SplitArcade {
                //    power: state.left_stick,
                //    turn: state.right_stick,
                //},
                intake: state.button_l1,
                outake: state.button_l2,
                intake_lift: state.button_a,
                lady_brown: state.button_y,
                clamp: state.button_r1,
                drive_pid_test: state.button_left,
                turn_pid_test: state.button_right,
            };

            let power = self.drivetrain.differential_drive(&mappings);
            self.drivetrain.set_voltage(power);

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
                self.drivetrain.set_turn_target(TargetType::Distance(0.0));
                _ = self.drivetrain.run().await;
            } else if mappings.drive_pid_test.is_pressed() {
                self.drivetrain.set_drive_target(TargetType::Distance(10.0));
                _ = self.drivetrain.run().await;
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

    let drivetrain = Drivetrain::new(
        Box::new([
            Motor::new(peripherals.port_15, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
        ]),
        Box::new([
            Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
        ]),
        Pid::new(0.125, 0.0, 0.0, 0.0),    // linear
        Pid::new(0.125, 0.01, 1.25, 0.25), // angular
        InertialSensor::new(peripherals.port_5),
    );

    let mut robot = Robot {
        drivetrain,
        intake: Intake::new((
            Motor::new(
                peripherals.port_2,
                Gearset::Blue,
                Direction::Forward,
            ),
            Motor::new(
                peripherals.port_17,
                Gearset::Blue,
                Direction::Forward,
            ),
        )),
        intake_lift: AdiDigitalOut::with_initial_level(peripherals.adi_g, LogicLevel::High),
        // not used for this compeition
        lady_brown: Arc::new(Mutex::new(LadyBrown::new(
            (
                Motor::new_exp(peripherals.port_20, Direction::Reverse),
                Motor::new_exp(peripherals.port_18, Direction::Forward),
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
                x: 12.0,
                y: 48.0,
            }), // START THE PATH WITH ITS STARTING POINT
            Command::Speed(0.8),
            Command::DriveBy(32.0), // move forward to clamp goal
            Command::Speed(1.0),
            Command::Sleep(750),
            Command::ToggleIntake,
            Command::ToggleClamp,
            Command::Sleep(500),
            Command::DriveBy(6.0),
            Command::TurnTo(90.0), // turn to face rings
            Command::DriveBy(-26.0), // drive towards rings
            Command::TurnTo(180.0), // turn 45 degrees right
            Command::DriveBy(-14.0),
            Command::TurnTo(-90.0),
            Command::Speed(0.8),
            Command::DriveBy(-22.5),
            Command::ToggleIntake,
        ],
        vec![ // path 1 (mess up)
            Command::Coordinate(Vec2 {
                x: 12.0,
                y: 48.0,
            }), // START THE PATH WITH ITS STARTING POINT
            Command::DriveBy(32.0), // move forward to clamp goal
            Command::Sleep(750),
            Command::ToggleIntake,
            Command::ToggleClamp,
            Command::Sleep(500),
            Command::TurnTo(135.0), // turn to face rings
            Command::ToggleIntakePiston,
            Command::DriveBy(-28.0), // drive towards rings
            Command::ToggleIntakeReverse, // INVERSE THIS
            Command::ToggleIntakePiston,
            Command::Sleep(1000),
            Command::ToggleIntake,
            Command::Sleep(1000),
            Command::TurnTo(0.0),
            Command::DriveBy(-20.0),
            Command::TurnTo(-67.5),
            Command::DriveBy(-48.0),
            Command::ToggleIntakePiston,
            Command::Sleep(250),
            Command::ToggleIntakePiston,
            Command::Sleep(250),
            Command::TurnTo(-45.0),
            Command::DriveBy(36.0),
        ],
        vec![
            // path 2
            Command::Coordinate(Vec2 {
                x: 24.0, y: 132.0
            }),
            Command::DriveBy(32.0),
            Command::TurnTo(-50.0),
            Command::DriveBy(20.0),
            Command::Sleep(750),
            Command::ToggleClamp,
            Command::ToggleIntake,
            Command::Sleep(500),
            Command::TurnTo(0.0),
            Command::DriveBy(-18.0),
            Command::TurnTo(135.0),
            Command::DriveBy(-24.0),
        ],
        vec![ // skills route
            Command::Coordinate(Vec2 {
                x: 10.0,
                y: 144.0 / 2.0,
            }),
            Command::ToggleIntake,
            Command::Sleep(1000),
            Command::ToggleIntake,
            Command::TurnTo(55.0),
            Command::DriveBy(-27.5),
            Command::ToggleClamp,
            Command::TurnTo(0.0),
            Command::ToggleIntake,
            Command::DriveBy(-23.0),
            Command::TurnTo(55.0),
            Command::DriveBy(-40.0),
            Command::TurnTo(-150.0),
            Command::DriveBy(-24.0),
            Command::TurnTo(180.0),
            Command::DriveBy(-24.0),
            Command::Sleep(500),
            Command::DriveBy(-12.0),
            Command::TurnTo(45.0),
            Command::DriveBy(-14.0),
            Command::TurnTo(-25.0),
            Command::DriveBy(20.0),
            Command::ToggleClamp,
        ]
    ];

    let mut path_coords = Vec::new();
    //let mut updated_paths = Vec::new();

    for path in auton_paths.iter() {
        let path_coord = command::command_to_coords(&path);
        path_coords.push(path_coord);
        //updated_paths.push(updated_path);
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
                //commands
                //    .iter()
                //    .map(|command| match command {
                //        Command::Coordinate(coord) => Command::Coordinate(Vec2 { x: 144.0 - coord.x, y: coord.y }),
                //        Command::TurnTo(..) | Command::TurnBy(..) | Command::DriveBy(..) => command.clone(),
                //        _ => *command,
                //    })
                //    .collect()
                let mut new_commands = Vec::new();

                for command in commands.iter() {
                    match command {
                        Command::Coordinate(_) => (),
                        Command::TurnTo(angle) => new_commands.push(Command::TurnTo(-angle)),
                        Command::TurnBy(_) => panic!("no"),
                        _ => new_commands.push(*command),
                    }
                }

                new_commands
            }

            let index = autonomous.index as usize;

            let (coords, path, color) = if autonomous.color == "red" {
                (
                    path_coords[index].clone(),
                    //updated_paths[index].clone(),
                    auton_paths[index].clone(),
                    [255u8, 0u8, 0u8, 255u8],
                )
            } else {
                (
                    invert_coords(&path_coords[index]),
                    invert_commands(&auton_paths[index]),
                    [0u8, 0u8, 255u8, 255u8],
                )
            };

            println!("{:?}", path);

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

    match robot.drivetrain.imu.calibrate().await {
        Ok(_) => println!("Inertial Sensor Sucessfully Calibrated"),
        Err(_) => println!("Inertial Error"),
    }
    robot.compete().await;
}
