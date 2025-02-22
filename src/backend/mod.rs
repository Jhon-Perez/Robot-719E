mod canvas;
mod reverse;

use alloc::rc::Rc;
use core::cell::RefCell;

use canvas::Canvas;
use vexide::{
    graphics::slint::initialize_slint_platform,
    prelude::{spawn, Display},
};

use crate::{
    autonomous::{command, PATHS},
    RobotSettings,
};

slint::include_modules!();

pub fn initialize_slint_gui(display: Display, settings: Rc<RefCell<RobotSettings>>) {
    initialize_slint_platform(display);

    let app = AppWindow::new().unwrap();

    app.on_autonomous({
        let ui_handler = app.as_weak();
        let settings = settings.clone();

        move |autonomous| {
            let index = autonomous.index as usize;

            let mut commands = command::path_to_commands(PATHS[index]);

            if autonomous.color == Color::Blue {
                commands = reverse::invert_coords(&commands)
            };

            let coords = command::command_to_coords(&commands);

            let mut canvas = Canvas::new(144, 144, autonomous.color);
            canvas.draw_commands(&coords);

            let ui = ui_handler.unwrap();
            ui.set_path_image(canvas.to_image());

            let mut settings = RefCell::borrow_mut(&settings);
            settings.auton_path = commands;
            settings.curr_color = autonomous.color;
        }
    });

    app.on_test({
        let settings = settings.clone();

        move || {
            let mut settings = RefCell::borrow_mut(&settings);
            settings.test_auton = true;
        }
    });

    spawn(async move {
        _ = app.show();
        _ = slint::run_event_loop();
    })
    .detach();
}
