use alloc::vec::Vec;

use slint::{Image, Rgba8Pixel, SharedPixelBuffer};

use crate::{command::Command, vector::Vec2};

//pub fn path_to_svg(path: &[Vec2]) -> String {
//    // 144 is the size of the field and we are starting on the halfway across the field
//    let mut curr_path = String::new();
//
//    curr_path.push_str(&format!("M {} {}", path[0].x, path[0].y));
//
//    for command in &path[1..] {
//        let svg = format!(" L {} {}", command.x, command.y);
//        curr_path.push_str(&svg);
//    }
//
//    curr_path
//}

// pub fn svg_to_image(commands: &str, width: u32, height: u32) -> Image {
//     let svg_data = format!(
//         r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">
//             <path d="{commands}" stroke="red" fill="none" stroke-width="2"/>
//         </svg>"#,
//         commands = commands,
//         width = width,
//         height = height
//     );
//     let buffer = svg_data.as_bytes();

//     Image::load_from_svg_data(buffer).unwrap()
// }

fn draw_line(
    mut x0: i32,
    mut y0: i32,
    x1: i32,
    y1: i32,
    buffer: &mut [u8],
    width: u32,
    height: u32,
    color: [u8; 4],
) {
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    while x0 != x1 || y0 != y1 {
        set_pixel(x0 as u32, y0 as u32, buffer, width, height, color);
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn set_pixel(x: u32, y: u32, buffer: &mut [u8], width: u32, height: u32, color: [u8; 4]) {
    if x >= width || y >= height {
        return;
    }
    let index = ((y * width + x) * 4) as usize;
    buffer[index..index + 4].copy_from_slice(&color);
}

pub fn coord_to_img(width: u32, height: u32, color: [u8; 4], coords: &[Command]) -> Image {
    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);

    let buffer = pixel_buffer.make_mut_bytes();

    let coords: Vec<Command> = coords
        .iter()
        .filter(|&command| matches!(command, Command::Coordinate(_)))
        .cloned()
        .collect();
    
    for i in 1..coords.len() {
        let (x0, y0) = if let Command::Coordinate(coord) = coords[i - 1] {
            coord.get_i32()
        } else {
            continue;
        };
        let (x1, y1) = if let Command::Coordinate(coord) = coords[i] {
            coord.get_i32()
        } else {
            continue;
        };

        draw_line(x0, y0, x1, y1, buffer, width, height, color);
    }

    Image::from_rgba8(pixel_buffer)
}
