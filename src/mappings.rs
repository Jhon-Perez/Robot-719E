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
    pub doinker: ButtonState,
    pub toggle_color_sort: ButtonState,

    pub lady_brown: ButtonState,
    pub manual_lady_brown: JoystickState,

    pub test_angular: ButtonState,
    pub test_linear: ButtonState,

    pub clamp: ButtonState,
}
