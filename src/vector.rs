extern crate alloc;

use core::ops::{AddAssign, Sub};

use alloc::vec::Vec;

use vexide::prelude::Float;

#[derive(Clone, Copy, Debug)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn get(&self) -> (f64, f64) {
        (self.x, self.y)
    }

    pub fn from_polar(radius: f64, angle: f64) -> Self {
        Self {
            x: radius * angle.cos(),
            y: radius * angle.sin(),
        }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn normalize(&self) -> Self {
        let magnitude = self.magnitude();
        Self {
            x: self.x / magnitude,
            y: self.y / magnitude,
        }
    }
}

impl From <(f64, f64)> for Vec2 {
    fn from(tuple: (f64, f64)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
        }
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

pub fn inject_points(points: &[Vec2], spacing: f64) -> Vec<Vec2> {
    let mut new_points = Vec::new();

    for i in 0..points.len() - 1 {
        let start_point = points[i];
        let end_point = points[i + 1];
        let vector = end_point - start_point;
        let vector_length = vector.magnitude();
        let num_points_that_fit = (vector_length / spacing).ceil() as usize;

        if num_points_that_fit == 0 {
            continue;
        }

        let normalized_vector = vector.normalize();
        let scaled_vector = Vec2 {
            x: normalized_vector.x * spacing,
            y: normalized_vector.y * spacing,
        };

        for j in 0..num_points_that_fit {
            let new_point = Vec2 {
                x: start_point.x + scaled_vector.x * j as f64,
                y: start_point.y + scaled_vector.y * j as f64,
            };
            new_points.push(new_point);
        }
    }

    if let Some(point) = points.last() {
        new_points.push(*point);
    }

    new_points
}

pub fn smooth_path(
    path: Vec<Vec2>,
    weight_data: f64,
    weight_smooth: f64,
    tolerance: f64,
) -> Vec<Vec2> {
    let mut new_path = path.clone();
    let mut change = tolerance;

    while change >= tolerance {
        change = 0.0;

        for i in 1..path.len() - 1 {
            let (x1, y1) = path[i].get();
            let (x0, y0) = new_path[i - 1].get();
            let (x2, y2) = new_path[i + 1].get();

            let (mut new_x, mut new_y) = new_path[i].get();

            let old_x = new_x;
            new_x += weight_data * (x1 - new_x) + weight_smooth * (x0 + x2 - 2.0 * new_x);
            change += (old_x - new_x).abs();

            let old_y = new_y;
            new_y += weight_data * (y1 - new_y) + weight_smooth * (y0 + y2 - 2.0 * new_y);
            change += (old_y - new_y).abs();

            new_path[i] = Vec2::from((new_x, new_y));
        }
    }

    new_path
}
