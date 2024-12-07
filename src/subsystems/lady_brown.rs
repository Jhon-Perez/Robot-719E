use core::{iter::Cycle, ops::Range};

use vexide::{core::println, prelude::{BrakeMode, Motor}};

pub struct LadyBrown {
    motor: (Motor, Motor),
    gear_ratio: f64,
    stage: usize,
    stages: Cycle<Range<usize>>,
}

impl LadyBrown {
    const ANGLES: [f64; 4] = [
        45.0, // flick tolerance
        15.0,  // Intake
        55.0, // Align
        75.0, // Scoring
    ];

    pub fn new(motor: (Motor, Motor), gear_ratio: f64) -> Self {
        Self {
            motor,
            gear_ratio,
            stage: 0,
            stages: (0..Self::ANGLES.len()).cycle(),
        }
    }

    pub fn next(&mut self) {
        // should not panic because the iterator cycles infinitely
        self.stage = self.stages.next().unwrap();
    }

    pub fn run(&mut self) {
        if let Ok(angle) = self.motor.0.position() {
            let angle = angle.as_degrees() * self.gear_ratio;
            println!("angle {}", angle);
            match self.stage {
                0 => {
                    if angle > Self::ANGLES[self.stage] {
                        _ = self.motor.0.set_voltage(-Motor::V5_MAX_VOLTAGE);
                        _ = self.motor.1.set_voltage(-Motor::V5_MAX_VOLTAGE);
                    } else {
                        _ = self.motor.0.brake(BrakeMode::Coast);
                        _ = self.motor.1.brake(BrakeMode::Coast);
                    }
                }
                1..4 => {
                    if angle < Self::ANGLES[self.stage] {
                        _ = self.motor.0.set_voltage(Motor::V5_MAX_VOLTAGE);
                        _ = self.motor.1.set_voltage(Motor::V5_MAX_VOLTAGE);
                    } else {
                        _ = self.motor.0.brake(BrakeMode::Hold);
                        _ = self.motor.1.brake(BrakeMode::Hold);
                    }
                }
                _ => unreachable!(),
            }
        }
    }

    pub fn set_voltage(&mut self) {
        _ = self.motor.0.set_voltage(Motor::V5_MAX_VOLTAGE);
        _ = self.motor.1.set_voltage(Motor::V5_MAX_VOLTAGE);
    }
}
