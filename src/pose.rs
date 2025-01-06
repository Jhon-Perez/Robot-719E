use evian::math::Vec2;

#[derive(Copy, Clone)]
pub struct Pose {
    pub position: Vec2<f64>,
    pub heading: f64,
}

impl Pose {
    pub fn new(position: Vec2<f64>, heading: f64) -> Self {
        Self { position, heading }
    }
}
