use vexide::prelude::{BrakeMode, Motor};

pub struct Intake {
    pub motor: (Motor, Motor),
    toggled: bool,
}

impl Intake {
    pub fn new(motor: (Motor, Motor)) -> Self {
        Self {
            motor,
            toggled: false,
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        _ = self.motor.0.set_voltage(voltage);
        _ = self.motor.1.set_voltage(voltage);
    }

    pub fn brake(&mut self, brake: BrakeMode) {
        _ = self.motor.0.brake(brake);
        _ = self.motor.1.brake(brake);
    }
    
    pub fn toggle(&mut self, reverse: f64) {
        self.toggled = !self.toggled;

        if self.toggled {
            self.set_voltage(Motor::V5_MAX_VOLTAGE * reverse);
        } else {
            self.brake(BrakeMode::Coast);
        }
    }

    //pub fn is_jammed(&self) -> Result<bool, MotorError> {
    //   Ok(self.motor.voltage()? == Motor::V5_MAX_VOLTAGE && self.motor.velocity()? > 10.0)
    //}
}
