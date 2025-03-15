pub mod command;
pub mod execute;
mod parse;

/// A list of commands for the robot to execute
pub static PATHS: [&str; 6] = [
    "",
    include_str!("paths/Left.botpath"),
    include_str!("paths/Left (mess up).botpath"),
    include_str!("paths/Goal Rush.botpath"),
    include_str!("paths/Right.botpath"),
    include_str!("paths/Skills.botpath"),
];

