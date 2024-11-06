use vexide::devices::controller::{Button, Joystick};

pub enum DriveMode<'a> {
    Arcade {
        arcade: &'a Joystick,
    },
    SplitArcade {
        power: &'a Joystick,
        turn: &'a Joystick,
    },
    Tank {
        left: &'a Joystick,
        right: &'a Joystick,
    },
}

// TODO: Create ui to allow user to change mappings
pub struct ControllerMappings<'a> {
    pub drive_mode: DriveMode<'a>,

    pub intake: &'a Button,
    pub outake: &'a Button,

    pub lift_up: &'a Button,
    pub lift_down: &'a Button,

    pub flick: &'a mut Button,
    
    pub clamp: &'a mut Button,
    pub doinker: &'a mut Button,
}
