use core::ops::{AddAssign, Sub};

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

impl From<(f64, f64)> for Vec2 {
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
