use core::{cell::RefCell, iter::Cycle, ops::Range, time::Duration};

use alloc::rc::Rc;
use vexide::prelude::{sleep, spawn, BrakeMode, Motor, RotationSensor, Task};

pub struct LadyBrown {
    stage: Rc<RefCell<usize>>,
    stages: Cycle<Range<usize>>,
    _task: Task<()>,
}

impl LadyBrown {
    const ANGLES: [f64; 4] = [
        45.0, // flick tolerance
        15.0, // Intake
        55.0, // Align
        75.0, // Scoring
    ];

    pub fn new<const COUNT: usize>(
        mut motors: [Motor; COUNT],
        rotation_sensor: Option<RotationSensor>,
        gear_ratio: Option<f64>,
    ) -> Self {
        let stage = Rc::new(RefCell::new(0));
        Self {
            stage: stage.clone(),
            stages: (0..Self::ANGLES.len()).cycle(),
            _task: spawn(async move {
                loop {
                    let angle = match &rotation_sensor {
                        Some(sensor) => sensor.position().unwrap(),
                        None => {
                            let Ok(angle) = motors[0].position() else {
                                continue;
                            };
                            angle
                        }
                    };

                    let angle = angle.as_degrees() * gear_ratio.unwrap_or(1.0);

                    match *stage.borrow() {
                        0 => {
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
                        _ => unreachable!(),
                    }

                    sleep(Duration::from_millis(20)).await;
                }
            }),
        }
    }

    pub fn next(&mut self) {
        let mut stage = self.stage.borrow_mut();
        
        *stage = self.stages.next().unwrap();
    }
}
