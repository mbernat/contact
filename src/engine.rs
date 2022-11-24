use crate::body::Body;
use crate::draw::draw_line_vec;
use crate::geometry::Transform;
use macroquad::color::RED;
use macroquad::math::{Mat3, Vec2, Vec3};
use macroquad::shapes::draw_circle;

pub struct Contact {
    pub pos: Vec2,
    pub normal: Vec2,
    pub depth: f32,
    pub this_body_index: usize,
    pub other_body_index: Option<usize>,
}

pub struct Engine {
    pub bodies: Vec<Body>,
}

impl Engine {
    pub fn step(&mut self, dt: f32) {
        for body in &mut self.bodies {
            body.update_vel(dt);
        }

        let contacts = self.find_contacts();
        self.solve_contacts(dt, &contacts);

        for body in &mut self.bodies {
            body.update_pos(dt);
            body.clear_forces();
        }
    }

    fn find_contacts(&self) -> Vec<Contact> {
        let mut result = vec![];
        let identi_t = Transform::new(Vec2::ZERO, 0.0);
        for (this_body_index, body) in self.bodies.iter().enumerate() {
            let t = Transform::new(body.pos, body.rot);
            // Contacts with bodies
            for (other_body_index, other_body) in self.bodies.iter().enumerate() {
                if this_body_index == other_body_index {
                    continue;
                }
                let other_t = Transform::new(other_body.pos, other_body.rot);
                let mut contacts = body.shape.find_contacts(
                    &t,
                    &other_body.shape,
                    &other_t,
                    this_body_index,
                    Some(other_body_index),
                );
                result.append(&mut contacts);
            }

            // TODO move ground() into the engine geometry
            // Contacts with geometry
            let mut contacts =
                body.shape
                    .find_contacts(&t, &crate::ground(), &identi_t, this_body_index, None);
            result.append(&mut contacts);
        }
        result
    }

    fn solve_contacts(&mut self, dt: f32, contacts: &Vec<Contact>) {
        for c in contacts {
            draw_circle(c.pos.x, c.pos.y, 3.0, RED);
            let d = c.depth.clamp(10.0, 50.0);
            draw_line_vec(c.pos, c.pos - c.normal * d, 1.0, RED);
        }

        let mut acc_impulses = vec![0.0; contacts.len()];
        let num_iters = 4;

        for _i in 0..num_iters {
            for (ci, c) in contacts.iter().enumerate() {
                let other_rel_vel = if let Some(other_body_index) = c.other_body_index {
                    let other_body = &self.bodies[other_body_index];
                    let cross = (c.pos - other_body.pos).perp_dot(c.normal);
                    let j: Vec3 = [c.normal.x, c.normal.y, cross].into();
                    j.dot(other_body.velocity_vector())
                } else {
                    0.0
                };

                let this_body = &mut self.bodies[c.this_body_index];
                let w = Mat3::from_diagonal(
                    [
                        1.0 / this_body.mass,
                        1.0 / this_body.mass,
                        1.0 / this_body.inertia,
                    ]
                    .into(),
                );
                let cross = (c.pos - this_body.pos).perp_dot(c.normal);
                let j: Vec3 = [c.normal.x, c.normal.y, cross].into();

                // Should be ~0.8 but it adds too much bounciness on impact
                let penetration_vel = -0.05 * c.depth / dt;
                let rel_vel = j.dot(this_body.velocity_vector()) - other_rel_vel + penetration_vel;
                let meff_inv = j.dot(w * j);
                let meff = 1.0 / meff_inv;
                let lambda = -meff * rel_vel;
                let lambda_prev = acc_impulses[ci];
                acc_impulses[ci] = (lambda_prev + lambda).max(0.0);
                let vel_change = w * j * (acc_impulses[ci] - lambda_prev);
                this_body.vel += Vec2::new(vel_change.x, vel_change.y);
                this_body.omega += vel_change.z;
            }
        }
    }
}
