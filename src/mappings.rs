use vexide::devices::controller::{Button, Joystick};

pub enum DriveMode<'a> {
    // SplitArcade and Tank won't be used currently,
    // this will change when a ui is used to easily
    // switch between these controllers.
    Arcade {
        arcade: &'a Joystick,
    },
    #[allow(dead_code)]
    SplitArcade {
        power: &'a Joystick,
        turn: &'a Joystick,
    },
    #[allow(dead_code)]
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

    pub clamp: &'a mut Button,

    pub drive_pid_test: &'a Button,
    pub turn_pid_test: &'a Button,
}
