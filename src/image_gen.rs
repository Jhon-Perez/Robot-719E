use alloc::vec::Vec;

use evian::math::Vec2;
use slint::{Image, Rgba8Pixel, SharedPixelBuffer};
use vexide::prelude::Float;

use crate::command::Command;

fn draw_bezier(
    p0: Vec2<f64>,
    p1: Vec2<f64>,
    p2: Vec2<f64>,
    p3: Vec2<f64>,
    steps: usize,
) -> Vec<(i32, i32)> {
    let mut points = Vec::with_capacity(steps);

    for i in 0..=steps {
        let t = i as f64 / steps as f64;

        let one_minus_t = 1.0 - t;
        let b0 = one_minus_t.powi(3);
        let b1 = 3.0 * one_minus_t.powi(2) * t;
        let b2 = 3.0 * one_minus_t * t.powi(2);
        let b3 = t.powi(3);

        let x = b0 * p0.x() + b1 * p1.x() + b2 * p2.x() + b3 * p3.x();
        let y = b0 * p0.y() + b1 * p1.y() + b2 * p2.y() + b3 * p3.y();

        points.push((x as i32, y as i32));
    }

    points
}

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
        .filter(|command| matches!(command, Command::Coordinate(_) | Command::CubicBezier(..)))
        .cloned()
        .collect();

    for i in 1..coords.len() {
        let (x0, y0) = match coords[i - 1] {
            Command::Coordinate(coord) => (coord.x() as i32, coord.y() as i32),
            Command::CubicBezier(.., p3) => (p3.x() as i32, p3.y() as i32),
            _ => unreachable!(),
        };

        let (x1, y1) = match coords[i] {
            Command::Coordinate(coord) => (coord.x() as i32, coord.y() as i32),
            Command::CubicBezier(p0, p1, p2, p3) => {
                let points = draw_bezier(p0, p1, p2, p3, 100);

                for j in 1..points.len() {
                    let (x0, y0) = points[j - 1];
                    let (x1, y1) = points[j];
                    draw_line(x0, y0, x1, y1, buffer, width, height, color);
                }

                (p0.x() as i32, p0.y() as i32)
            }
            _ => unreachable!(),
        };

        draw_line(x0, y0, x1, y1, buffer, width, height, color);
    }

    Image::from_rgba8(pixel_buffer)
}
