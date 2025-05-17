use core::{cell::RefCell, iter::Cycle, ops::Range, time::Duration};

use alloc::rc::Rc;
use vexide::prelude::{sleep, spawn, BrakeMode, Motor, RotationSensor, Task};

#[derive(Clone, Copy, Debug)]
pub enum LadyBrownCommand {
    Next,
    Angle(f64),
    Voltage(f64),
}

pub struct LadyBrown {
    command: Rc<RefCell<LadyBrownCommand>>,
    stage: Rc<RefCell<usize>>,
    stages: Cycle<Range<usize>>,
    _task: Task<()>,
}

impl LadyBrown {
    /// Predefined angles for different stages of the scoring mechanism
    const ANGLES: [f64; 4] = [
        80.0, // Flick tolerance (default resting position)
        8.0, // Intake (aligns with hooks to pick up rings)
        100.0, // Align (positioning before scoring)
        150.0, // Scoring (final scoring position on wall stake)
    ];

    // the starting angle the rotation sensor reports
    // const STARTING_ANGLE: f64 = 0.0;

    /// Initializes the scoring mechanism with motors and optional sensors
    pub fn new<const COUNT: usize>(
        mut motors: [Motor; COUNT],
        mut rotation_sensor: RotationSensor,
        gear_ratio: Option<f64>,
    ) -> Self {
        let command = Rc::new(RefCell::new(LadyBrownCommand::Angle(0.0)));
        let stage = Rc::new(RefCell::new(0));
        let mut stages = (0..Self::ANGLES.len()).cycle();
        _ = stages.next();
        _ = rotation_sensor.reset_position();

        Self {
            command: command.clone(),
            stage: stage.clone(),
            stages, // Infinite cycle through the stages
            _task: spawn(async move {
                loop {
                    // Run the loop every 20ms to prevent CPU overload
                    sleep(Duration::from_millis(20)).await;

                    let Some(angle) = get_angle(&rotation_sensor, &motors, gear_ratio) else {
                        continue;
                    };

                    // Convert the angle to degrees, applying the gear ratio if provided
                    // as of right now rotation sensor should not work because it keeps the
                    // abosulte amount it has rotated rather than reseting when the program resets
                    //let angle = angle.as_degrees() * gear_ratio.unwrap_or(1.0);

                    // Adjust motor behavior based on the current stage
                    //println!("{}", *stage.borrow());
                    // if matches!(*command.borrow(), LadyBrownCommand::Angle(_)) {
                    //     for motor in motors.iter_mut() {
                    //         // _ = motor.set_voltage
                    //         todo!()
                    //     }
                    // }
                    match *stage.borrow() {
                        0 => {
                            // Stage 0: Let the lady brown fall down to a resting position
                            if angle > Self::ANGLES[*stage.borrow()] {
                                for motor in motors.iter_mut() {
                                    _ = motor.set_voltage(-motor.max_voltage() * 0.8);
                                }
                            } else {
                                for motor in motors.iter_mut() {
                                    _ = motor.brake(BrakeMode::Coast);
                                }
                            }
                        }
                        1..4 => {
                            // Stages 1-3: Move up until motors have reached the desired angle
                            if angle < Self::ANGLES[*stage.borrow()] {
                                for motor in motors.iter_mut() {
                                    _ = motor.set_voltage(motor.max_voltage() * 0.6);
                                }
                            } else {
                                for motor in motors.iter_mut() {
                                    _ = motor.brake(BrakeMode::Hold);
                                }
                            }
                        }
                        // Cycle should never go past 3 because of the infinite cycle
                        _ => unreachable!(),
                    } 
                }
            }),
        }
    }

    pub fn set_command(&mut self, cmd: LadyBrownCommand) {
        if let Ok(mut command) = self.command.try_borrow_mut() {
            *command = cmd;
        }
    }

    /// Advances to the next stage in the cycle
    pub fn next(&mut self) {
        let mut stage = self.stage.borrow_mut();
        *stage = self.stages.next().unwrap(); // Move to the next stage in the sequence
    }
}

fn get_angle(rotation_sensor: &RotationSensor, motors: &[Motor], gear_ratio: Option<f64>) -> Option<f64> {
    // Try to get the angle from the rotation sensor first
    
    if let Ok(angle) = rotation_sensor.position() {
        return Some(angle.as_degrees() * gear_ratio.unwrap_or(1.0));
    }

    // If the sensor is unavailable or failed, try the first motor
    Some(motors.first()?.position().ok()?.as_degrees() * gear_ratio.unwrap_or(1.0))
}
