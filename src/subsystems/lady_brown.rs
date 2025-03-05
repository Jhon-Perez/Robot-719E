use core::{cell::RefCell, iter::Cycle, ops::Range, time::Duration};

use alloc::rc::Rc;
use vexide::prelude::{sleep, spawn, BrakeMode, Motor, RotationSensor, Task};

pub struct LadyBrown {
    /// Tracks the current stage in the scoring process
    stage: Rc<RefCell<usize>>,
    /// Cycles through the different stages
    stages: Cycle<Range<usize>>,
    /// Background async task that continuously adjusts motor positions
    _task: Task<()>,
}

impl LadyBrown {
    /// Predefined angles for different stages of the scoring mechanism
    const ANGLES: [f64; 4] = [
        45.0, // Flick tolerance (default resting position)
        15.0, // Intake (aligns with hooks to pick up rings)
        55.0, // Align (positioning before scoring)
        75.0, // Scoring (final scoring position on wall stake)
    ];

    /// Initializes the scoring mechanism with motors and optional sensors
    pub fn new<const COUNT: usize>(
        mut motors: [Motor; COUNT], // Array of motors controlling the mechanism
        rotation_sensor: Option<RotationSensor>, // Optional rotation sensor for precise positioning
        gear_ratio: Option<f64>, // Optional gear ratio to adjust angle readings
    ) -> Self {
        let stage = Rc::new(RefCell::new(0)); // Start at stage 0

        Self {
            stage: stage.clone(),
            stages: (0..Self::ANGLES.len()).cycle(), // Infinite cycle through the stages
            _task: spawn(async move {
                loop {
                    // Determine the current angle from the rotation sensor or first motor
                    let angle = match &rotation_sensor {
                        Some(sensor) => sensor.position().unwrap(), // Use sensor reading if available
                        None => {
                            let Ok(angle) = motors[0].position() else {
                                continue; // If the motor position fails, skip this loop iteration
                            };
                            angle
                        }
                    };

                    // Convert the angle to degrees, applying the gear ratio if provided
                    let angle = angle.as_degrees() * gear_ratio.unwrap_or(1.0);

                    // Adjust motor behavior based on the current stage
                    match *stage.borrow() {
                        0 => {
                            // Stage 0: Let the lady brown fall down to a resting position
                            if angle > Self::ANGLES[*stage.borrow()] {
                                for motor in motors.iter_mut() {
                                    _ = motor.set_voltage(-Motor::V5_MAX_VOLTAGE);
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
                                    _ = motor.set_voltage(Motor::V5_MAX_VOLTAGE);
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

                    // Run the loop every 20ms to prevent CPU overload
                    sleep(Duration::from_millis(20)).await;
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

