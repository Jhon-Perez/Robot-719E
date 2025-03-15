use vexide::devices::controller::{ButtonState, JoystickState};

// Different drive mods that the driver can switch to
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
// Map all the controller keybinds with their respective subsystem
pub struct ControllerMappings {
    pub drive_mode: DriveMode,

    pub intake: ButtonState,
    pub outake: ButtonState,
    pub intake_lift: ButtonState,
    pub toggle_color_sort: ButtonState,

    pub lady_brown: ButtonState,

    pub test_angular: ButtonState,
    pub test_linear: ButtonState,
    pub test_alliance: ButtonState,

    pub clamp: ButtonState,
}
