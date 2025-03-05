mod canvas;
mod reverse;

use alloc::rc::Rc;
use core::cell::RefCell;

use canvas::Canvas;
use vexide::{
    core::println,
    graphics::slint::initialize_slint_platform,
    prelude::{Display, spawn},
};

use crate::{
    RobotSettings,
    autonomous::{PATHS, command},
};

slint::include_modules!();

/// Initializes the Slint GUI and sets up event handlers for autonomous selection and testing.
pub fn initialize_slint_gui(display: Display, settings: Rc<RefCell<RobotSettings>>) {
    // Set up the Slint rendering platform
    initialize_slint_platform(display);

    // Create a new application window instance
    let app = AppWindow::new().unwrap();

    // Event handler for when an autonomous routine is selected
    app.on_autonomous({
        // Create a weak reference to the UI to avoid ownership issues
        let ui_handler = app.as_weak();
        // Clone the settings reference for use in the closure
        let settings = settings.clone();

        move |autonomous| {
            let index = autonomous.index as usize;

            // Convert the selected autonomous path into commands
            let commands = match command::path_to_commands(PATHS[index]) {
                Ok(commands) => commands,
                Err(e) => {
                    println!("Error parsing command '{}'", e);
                    return; // Early return to avoid further execution on error
                }
            };

            // If the autonomous color is blue, invert the path coordinates
            let commands = if autonomous.color == Color::Blue {
                reverse::invert_coords(&commands)
            } else {
                commands
            };

            // Convert commands into coordinates for visualization
            let coords = command::command_to_coords(&commands);

            // Create a canvas and draw the path
            let mut canvas = Canvas::new(144, 144, autonomous.color);
            canvas.draw_commands(&coords);

            // Ensure the UI handler is still valid before updating
            if let Some(ui) = ui_handler.upgrade() {
                ui.set_path_image(canvas.to_image());
            }

            // Update the robot settings with the selected autonomous path and color
            let mut settings = settings.borrow_mut();
            settings.auton_path = commands;
            settings.curr_color = autonomous.color;
        }
    });

    // Event handler for testing autonomous mode
    app.on_test({
        let settings = settings.clone();

        move || {
            let mut settings = settings.borrow_mut();
            settings.test_auton = true;
        }
    });

    // Spawn an asynchronous task to show the GUI and run the event loop
    spawn(async move {
        _ = app.show();
        _ = slint::run_event_loop();
    })
    .detach();
}
