use crate::geometry::{Shape, Transform};
use macroquad::math::{Vec2, Vec3};

#[derive(Clone, Debug)]
pub struct Body {
    pub mass: f32,
    pub inertia: f32,
    pub pos: Vec2,
    pub rot: f32,
    pub vel: Vec2,
    pub omega: f32,
    pub force: Vec2,
    pub torque: f32,

    pub shape: Shape,
}

impl Body {
    pub(crate) fn update_vel(&mut self, dt: f32) {
        self.vel += self.force / self.mass * dt;
        self.omega += self.torque / self.inertia * dt;
    }

    pub(crate) fn update_pos(&mut self, dt: f32) {
        self.pos += self.vel * dt;
        self.rot = (self.rot + self.omega * dt).rem_euclid(std::f32::consts::TAU)
    }

    pub(crate) fn clear_forces(&mut self) {
        self.force = Vec2::ZERO;
        self.torque = 0.0;
    }

    pub(crate) fn velocity_vector(&self) -> Vec3 {
        [self.vel.x, self.vel.y, self.omega].into()
    }

    pub(crate) fn render(&self) {
        let t = Transform::new(self.pos, self.rot);
        self.shape.render(&t);
    }
}
