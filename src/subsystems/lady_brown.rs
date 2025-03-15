use core::{cell::RefCell, iter::Cycle, ops::Range, time::Duration};

use alloc::rc::Rc;
use vexide::prelude::{sleep, spawn, BrakeMode, Motor, RotationSensor, Task};

//pub enum LadyBrownCommand {
//    Next,
//    Voltage(f64),
//}

pub struct LadyBrown {
    //command: Rc<RefCell<LadyBrownCommand>>,
    stage: Rc<RefCell<usize>>,
    stages: Cycle<Range<usize>>,
    _task: Task<()>,
}

impl LadyBrown {
    /// Predefined angles for different stages of the scoring mechanism
    const ANGLES: [f64; 4] = [
        62.0, // Flick tolerance (default resting position)
        11.0, // Intake (aligns with hooks to pick up rings)
        110.0, // Align (positioning before scoring)
        150.0,
        //180.0, // Scoring (final scoring position on wall stake)
    ];

    /// Initializes the scoring mechanism with motors and optional sensors
    pub fn new<const COUNT: usize>(
        mut motors: [Motor; COUNT],
        mut rotation_sensor: RotationSensor,
        gear_ratio: Option<f64>,
    ) -> Self {
        //let command = Rc::new(RefCell::new(LadyBrownCommand::Voltage(0.0)));
        let stage = Rc::new(RefCell::new(0));
        let mut stages = (0..Self::ANGLES.len()).cycle();
        _ = stages.next();
        _ = rotation_sensor.reset_position();

        Self {
            //command: command.clone(),
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
                    match *stage.borrow() {
                        0 => {
                            // Stage 0: Let the lady brown fall down to a resting position
                            if angle > Self::ANGLES[*stage.borrow()] {
                                for motor in motors.iter_mut() {
                                    _ = motor.set_voltage(-motor.max_voltage());
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
                                    _ = motor.set_voltage(motor.max_voltage());
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

    /// Advances to the next stage in the cycle
    pub fn next(&mut self) {
        let mut stage = self.stage.borrow_mut();
        *stage = self.stages.next().unwrap(); // Move to the next stage in the sequence
    }
}

fn get_angle(rotation_sensor: &RotationSensor, motors: &[Motor], gear_ratio: Option<f64>) -> Option<f64> {
    // Try to get the angle from the rotation sensor first
    
    if let Ok(angle) = rotation_sensor.position() {
        return Some(angle.as_degrees());
    }

    // If the sensor is unavailable or failed, try the first motor
    Some(motors.first()?.position().ok()?.as_degrees() * gear_ratio?)
}
