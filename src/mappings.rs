use vexide::devices::controller::{ButtonState, JoystickState};

pub enum DriveMode {
    Arcade {
        power: JoystickState,
        turn: JoystickState,
    },
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
    pub intake_lift: ButtonState,

    pub lady_brown: ButtonState,

    pub clamp: ButtonState,

    pub drive_pid_test: ButtonState,
    pub turn_pid_test: ButtonState,
}
