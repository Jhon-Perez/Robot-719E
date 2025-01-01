#![no_main]
#![no_std]

extern crate alloc;

mod autonomous;
mod command;
mod image_gen;
mod mappings;
mod odometry;
mod pid;
mod pose;
mod subsystems;
mod vector;

use alloc::{rc::Rc, sync::Arc, vec::Vec};
use core::{borrow::BorrowMut, cell::RefCell, time::Duration};

use command::Command;
use evian::{
    control::{AngularPid, Pid},
    differential::motion::{Ramsete, Seeking},
    drivetrain::Drivetrain,
    math::Vec2,
    prelude::*,
};
use mappings::{ControllerMappings, DriveMode};
use slint::ComponentHandle;
use subsystems::{drivetrain::differential_drive, intake::Intake, lady_brown::LadyBrown};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::adi::digital::LogicLevel,
    prelude::*,
    startup::banner::themes::THEME_MURICA,
};

slint::include_modules!();

const TRACK_WIDTH: f64 = 11.5;
const DRIVE_RPM: f64 = 450.0;
const GEARING: f64 = 36.0 / 48.0;
const WHEEL_DIAMETER: f64 = 3.25;

struct Robot {
    drivetrain: Drivetrain<Differential, ParallelWheelTracking>,

    intake: Intake,
    intake_lift: AdiDigitalOut,
    lady_brown: Arc<Mutex<LadyBrown>>,

    clamp: AdiDigitalOut,

    controller: Controller,

    pub auton_path: Rc<RefCell<Vec<Command>>>,
    pub test_auton: Rc<RefCell<bool>>,
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

        let constraints = TrajectoryConstraints {
            max_velocity: from_drive_rpm(DRIVE_RPM, WHEEL_DIAMETER),
            max_acceleration: 200.0,
            max_deceleration: 200.0,
            friction_coefficient: 1.0,
            track_width: TRACK_WIDTH,
        };

        let mut ramsete = Ramsete {
            b: 0.00129,
            zeta: 0.2,
            track_width: TRACK_WIDTH,
            wheel_diameter: WHEEL_DIAMETER,
            external_gearing: GEARING,
        };

        let curve = CubicBezier::new((0.0, 0.0), (18.0, 33.0), (50.0, -29.0), (46.0, 8.0));
        let trajectory = Trajectory::generate(curve, 0.1, constraints);

        let dt = &mut self.drivetrain;

        ramsete.follow(dt, trajectory).await;

        let mut seeking = Seeking {
            distance_controller: Pid::new(0.125, 0.0, 0.0, None),
            angle_controller: AngularPid::new(0.125, 0.01, 1.25, Some(20.0.deg())),
            settler: Settler::new()
                .error_tolerance(0.5)
                .tolerance_duration(Duration::from_millis(100))
                .timeout(Duration::from_millis(5000)),
        };

        for command in self.auton_path.borrow().iter() {
            match command {
                Command::Coordinate(coord) => _ = seeking.move_to_point(dt, *coord),
                // figure out how to control speed
                Command::Speed(_speed) => {
                    //self.drivetrain.set_speed(*speed);
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
                _ => unreachable!(),
            };
        }
    }

    async fn driver(&mut self) {
        println!("Driver control started.");

        _ = self.intake_lift.set_low();

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

            let power = differential_drive(&mappings);
            _ = self.drivetrain.motors.set_voltages((power.0, power.1));

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

            if self.test_auton.replace(false) {
                self.autonomous().await;
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

#[inline]
pub const fn from_drive_rpm(rpm: f64, wheel_diameter: f64) -> f64 {
    (rpm / 60.0) * (core::f64::consts::PI * wheel_diameter)
}

#[vexide::main(banner(theme = THEME_MURICA))]
async fn main(peripherals: Peripherals) {
    println!("Program started.");

    let mut imu = InertialSensor::new(peripherals.port_3);
    _ = imu.calibrate();
    let left_motors = shared_motors![Motor::new(
        peripherals.port_1,
        Gearset::Blue,
        Direction::Forward
    )];
    let right_motors = shared_motors![Motor::new(
        peripherals.port_2,
        Gearset::Blue,
        Direction::Forward
    )];

    let drivetrain = Drivetrain::new(
        Differential::new(left_motors.clone(), right_motors.clone()),
        ParallelWheelTracking::new(
            Vec2::default(),
            90.0.deg(),
            TrackingWheel::new(
                left_motors.clone(),
                WHEEL_DIAMETER,
                TRACK_WIDTH / 2.0,
                Some(GEARING),
            ),
            TrackingWheel::new(
                right_motors.clone(),
                WHEEL_DIAMETER,
                TRACK_WIDTH / 2.0,
                Some(GEARING),
            ),
            Some(imu),
        ),
    );

    let robot = Robot {
        drivetrain,
        intake: Intake::new((
            Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
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
        test_auton: Rc::new(RefCell::new(false)),
    };

    initialize_slint_platform(peripherals.display);

    let mut auton_paths = autonomous::paths();

    for i in 0..auton_paths.len() {
        auton_paths[i] = command::command_to_coords(&auton_paths[i]);
    }

    let app = AppWindow::new().unwrap();

    app.on_autonomous({
        let ui_handler = app.as_weak();
        let mut auton_path = robot.auton_path.clone();

        move |autonomous| {
            println!("{:?}", autonomous);

            fn invert_coords(commands: &[Command]) -> Vec<Command> {
                commands
                    .iter()
                    .map(|command| {
                        if let Command::Coordinate(coord) = command {
                            Command::Coordinate(Vec2 {
                                x: 144.0 - coord.x(),
                                y: coord.y(),
                            })
                        } else {
                            *command
                        }
                    })
                    .collect()
            }

            let index = autonomous.index as usize;

            let (commands, color) = if autonomous.color == "red" {
                (
                    auton_paths[index].clone(),
                    [255u8, 0u8, 0u8, 255u8],
                )
            } else {
                (
                    invert_coords(&auton_paths[index]),
                    [0u8, 0u8, 255u8, 255u8],
                )
            };

            let image = image_gen::coord_to_img(144, 144, color, &commands);

            {
                let curr_path = auton_path.borrow_mut();
                curr_path.replace(commands);
            }

            let ui = ui_handler.unwrap();
            ui.set_path_image(image);
        }
    });

    app.on_test({
        let mut test_auton = robot.test_auton.clone();

        move || {
            let test = test_auton.borrow_mut();
            test.replace(true);
        }
    });

    spawn(async move {
        _ = app.show();
        _ = slint::run_event_loop();
    })
    .detach();

    robot.compete().await;
}
