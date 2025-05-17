use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use vexide::{io::println, prelude::{sleep, spawn, BrakeMode, Motor, OpticalSensor, Task}};

use crate::{RobotSettings, Color};

#[derive(Copy, Clone, Debug)]
pub enum IntakeCommand {
    Off,
    On,
    Voltage(f64),
}

impl core::ops::Not for IntakeCommand {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            Self::Off => Self::On,
            Self::On => Self::Off,
            Self::Voltage(_) => Self::Off,
        }
    }
}

pub struct Intake {
    color_sort: Rc<RefCell<bool>>,
    command: Rc<RefCell<IntakeCommand>>,
    _task: Task<()>,
}

impl Intake {
    // make first motor the primary for now until a solution is made
    pub fn new<const COUNT: usize>(
        mut motors: [Motor; COUNT],
        optical_sensor: OpticalSensor,
        settings: Rc<RefCell<RobotSettings>>,
        sprocket_teeth: f64,
        sorting_distance: f64,
    ) -> Self {
        let color_sort = Rc::new(RefCell::new(true));
        let command = Rc::new(RefCell::new(IntakeCommand::Off));

        Self {
            color_sort: color_sort.clone(),
            command: command.clone(),
            _task: spawn(async move {
                let sorting_revolutions = sorting_distance / sprocket_teeth;

                loop {
                    println!("{:?}", *command.borrow());
                    match *command.borrow() {
                        IntakeCommand::Voltage(voltage) => {
                            for motor in motors.iter_mut() {
                                _ = motor.set_voltage(voltage);
                            }
                        }
                        IntakeCommand::On => {
                            for motor in motors.iter_mut() {
                                _ = motor.set_voltage(motor.max_voltage());
                            }

                            if *color_sort.borrow() {
                                // Blue ring has a hue of 120 to 240 and red ring has a hue of 0 to 60
                                // so the oppisite hue is what we are checking for
                                let color_range = match settings.borrow().curr_color {
                                    Color::Red => 120.0..240.0,
                                    Color::Blue => 0.0..60.0,
                                };

                                let color_hue = optical_sensor.hue().unwrap_or_default();
                                let proximity = optical_sensor.proximity().unwrap_or_default();

                                // The oppisite color has obstructed the view, kick the ring out
                                if color_range.contains(&color_hue) && proximity == 1.0 {
                                    _ = motors[0].reset_position();

                                    while let Ok(position) = motors[0].position() {
                                        sleep(Duration::from_millis(20)).await;

                                        // Check if the motor has moved 8.0 inches since its found
                                        // the wrong ring
                                        if position.as_revolutions() > sorting_revolutions {
                                            break;
                                        }
                                    }

                                    for motor in motors.iter_mut() {
                                        _ = motor.brake(BrakeMode::Brake);
                                    }
                                    sleep(Duration::from_millis(250)).await;
                                }
                            }
                        }
                        IntakeCommand::Off => {
                            for motor in motors.iter_mut() {
                                _ = motor.brake(BrakeMode::Coast);
                            }
                        }
                    }

                    sleep(Duration::from_millis(20)).await;
                }
            }),
        }
    }

    pub fn set_command(&mut self, cmd: IntakeCommand) {
        if let Ok(mut command) = self.command.try_borrow_mut() {
            *command = cmd;
        }
    }

    pub fn toggle_color_sort(&mut self) {
        let mut color_sort = self.color_sort.borrow_mut();
        *color_sort = !*color_sort;
    }
}
