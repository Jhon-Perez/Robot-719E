use vexide::prelude::{BrakeMode, Motor};

pub struct Intake {
    pub motor: Motor,
    toggled: bool,
}

impl Intake {
    pub fn new(motor: Motor) -> Self {
        Self {
            motor,
            toggled: false,
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        _ = self.motor.set_voltage(voltage);
    }

    pub fn brake(&mut self, brake: BrakeMode) {
        _ = self.motor.brake(brake);
    }
    
    pub fn toggle(&mut self) {
        self.toggled = !self.toggled;

        if self.toggled {
            _ = self.motor.set_voltage(Motor::V5_MAX_VOLTAGE);
        } else {
            _ = self.motor.brake(BrakeMode::Coast);
        }
    }

    //pub fn is_jammed(&self) -> Result<bool, MotorError> {
    //   Ok(self.motor.voltage()? == Motor::V5_MAX_VOLTAGE && self.motor.velocity()? > 10.0)
    //}
}
