use vexide::devices::controller::{ButtonState, JoystickState};

pub enum DriveMode {
    // SplitArcade and Tank won't be used currently,
    // this will change when a ui is used to easily
    // switch between these controllers.
    Arcade {
        arcade: JoystickState,
    },
    #[allow(dead_code)]
    SplitArcade {
        power: JoystickState,
        turn: JoystickState,
    },
    #[allow(dead_code)]
    Tank {
        left: JoystickState,
        right: JoystickState,
    },
}

// TODO: Create ui to allow user to change mappings
pub struct ControllerMappings {
    pub drive_mode: DriveMode,

    pub intake: ButtonState,
    pub outake: ButtonState,

    pub lady_brown: ButtonState,

    pub clamp: ButtonState,

    pub drive_pid_test: ButtonState,
    pub turn_pid_test: ButtonState,
}
