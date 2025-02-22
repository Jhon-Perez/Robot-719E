use vexide::prelude::{BrakeMode, Motor};

pub struct Intake {
    pub intake: Motor,
    pub hook: Motor,
    toggled: bool,
}

impl Intake {
    pub fn new(intake: Motor, hook: Motor) -> Self {
        Self {
            intake,
            hook,
            toggled: false,
        }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        _ = self.intake.set_voltage(voltage);
        _ = self.hook.set_voltage(voltage);
    }

    pub fn brake(&mut self, brake: BrakeMode) {
        _ = self.intake.brake(brake);
        _ = self.hook.brake(brake);
    }
   
    pub fn toggle(&mut self, speed: f64) {
        self.toggled = !self.toggled;

        if self.toggled {
            self.set_voltage(Motor::V5_MAX_VOLTAGE * speed);
        } else {
            self.brake(BrakeMode::Coast);
        }
    }

    //pub fn is_jammed(&self) -> Result<bool, MotorError> {
    //   Ok(self.motor.voltage()? == Motor::V5_MAX_VOLTAGE && self.motor.velocity()? > 10.0)
    //}
}
