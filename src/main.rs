use macroquad::prelude::*;
use macroquad::window::next_frame;

fn draw_line_vec(a: Vec2, b: Vec2, thickness: f32, color: Color) {
    draw_line(a.x, a.y, b.x, b.y, thickness, color)
}

fn draw_central_line(pos: Vec2, half_extent: Vec2) {
    draw_line_vec(pos - half_extent, pos + half_extent, 1.0, WHITE);
}

struct Contact {
    pos: Vec2,
    normal: Vec2,
    depth: f32,
}

struct Segment {
    start: Vec2,
    end: Vec2,
}

impl Segment {
    fn render(&self, thickness: f32, color: Color) {
        draw_line_vec(self.start, self.end, thickness, color)
    }
}

#[derive(Clone)]
struct Rect {
    half_extents: Vec2,
}

impl Rect {
    fn corners(&self, t: &Transform) -> [Vec2; 4] {
        let mat = Mat2::from_angle(t.rot);
        // bottom right
        let v1 = mat * self.half_extents;
        // top right
        let v2 = mat * Vec2::new(self.half_extents.x, -self.half_extents.y);
        [t.pos + v1, t.pos - v2, t.pos - v1, t.pos + v2]
    }

    fn edges(&self, t: &Transform) -> [Segment; 4] {
        let [a, b, c, d] = self.corners(t);
        [
            Segment { start: a, end: b },
            Segment { start: b, end: c },
            Segment { start: c, end: d },
            Segment { start: d, end: a },
        ]
    }
}

#[derive(Clone)]
enum Shape {
    Circle { radius: f32 },
    Rect(Rect),
}

impl Shape {
    fn render(&self, t: &Transform) {
        let rot = t.rot.to_degrees();
        let mat = Mat2::from_angle(rot);

        match self {
            Shape::Circle { radius } => {
                draw_circle_lines(t.pos.x, t.pos.y, *radius, 1.0, WHITE);
                draw_central_line(t.pos, mat.col(0) * *radius);
                draw_central_line(t.pos, mat.col(1) * *radius);
            }
            Shape::Rect(r) => {
                let t = Transform::new(t.pos, t.rot);
                for e in r.edges(&t).iter() {
                    e.render(1.0, WHITE);
                }
            }
        }
    }
}

struct Transform {
    pos: Vec2,
    rot: f32,
}

impl Transform {
    fn new(pos: Vec2, rot: f32) -> Transform {
        Transform { pos, rot }
    }
}

fn collide_segment_segment(a: &Segment, b: &Segment) -> Vec<Vec2> {
    let c = a.end - a.start;
    let d = -(b.end - b.start); 
    let e = b.start - a.start;
    let det = c.perp_dot(d);
    // TODO handle parallel lines
    if det.abs() < 1.0 {
        let bs_perp = (b.start - a.start).perp_dot(c.normalize());
        return if bs_perp.abs() < 1.0 {
            let bs_t = (b.start - a.start).dot(c) / c.length_squared();
            let be_t = (b.end - a.end).dot(c) / c.length_squared();
            if be_t >= 0.0 && bs_t <= 1.0 {
                let bs_c = bs_t.clamp(0.0, 1.0);
                let be_c = be_t.clamp(0.0, 1.0);
                vec![a.start + bs_c * c, a.start + be_c * c]
            } else {
                vec![]
            }
        } else {
            vec![]
        }
    }

    let inv: Mat2 = Mat2::from_cols([d.y, -c.y].into(), [-d.x, c.x].into()) * (1.0 / det);
    let sol = inv * e;
    if sol.x >= 0.0 && sol.x <= 1.0 && sol.y >= 0.0 && sol.y <= 1.0 {
        vec![a.start + sol.x * c]
    } else {
        vec![]
    }
}

fn collide_rect_rect(t1: &Transform, r1: &Rect, t2: &Transform, r2: &Rect) -> Vec<Contact> {
    let mut results = vec![];
    for e1 in r1.edges(t1).iter() {
        for e2 in r2.edges(t2).iter() {
            for pos in collide_segment_segment(e1, e2) {
                results.push(Contact {
                    pos,
                    normal: (e1.end - e1.start).perp().normalize(),
                    depth: 0.0
                })
            }
        }
    }
    results
}

fn ground() -> (Rect, Transform) {
    let r = Rect {half_extents: [300.0, 50.0].into()};
    let t = Transform{pos: [400.0, 450.0].into(), rot: 0.0};
    (r, t)
}

impl Shape {
    fn find_contacts(&self, t: &Transform) -> Vec<Contact> {
        match self {
            // TODO implement circle rect collision
            Shape::Circle { radius } => {
                let bottom = t.pos.y + radius;
                if bottom >= 400.0 {
                    let pos: Vec2 = [t.pos.x, bottom].into();
                    let normal = -Vec2::Y;
                    vec![Contact {
                        pos,
                        normal,
                        depth: bottom - 400.0,
                    }]
                } else {
                    vec![]
                }
            }
            Shape::Rect(r) => {
                let (r2, t2) = ground();
                collide_rect_rect(t, r, &t2, &r2)
            }
        }
    }
}

#[derive(Clone)]
struct Body {
    mass: f32,
    inertia: f32,
    pos: Vec2,
    rot: f32,
    vel: Vec2,
    omega: f32,
    force: Vec2,
    torque: f32,

    shape: Shape,
}

impl Body {
    fn velocity_at(&self, pos: Vec2) -> Vec2 {
        self.vel + (pos - self.pos).perp() * self.omega
    }

    fn predict(&self, dt: f32) -> Body {
        let mut next = self.clone();
        next.vel += self.force / self.mass * dt;
        next.pos += self.vel * dt;
        next.force = Vec2::ZERO;

        next.omega += self.torque / self.inertia * dt;
        next.rot += self.omega * dt;
        next.torque = 0.0;
        next
    }

    fn add_force_at(&mut self, force: Vec2, pos: Vec2) {
        self.force += force;
        self.torque += (pos - self.pos).perp_dot(force);
    }

    fn add_impulse_at(&mut self, impulse: Vec2, pos: Vec2) {
        self.vel += impulse / self.mass;
        self.omega += (pos - self.pos).perp_dot(impulse) / self.inertia;
    }

    fn step(&mut self, dt: f32) {
        let next = self.predict(dt);
        let t = Transform::new(next.pos, next.rot);

        for c in self.shape.find_contacts(&t) {
            draw_circle(c.pos.x, c.pos.y, 5.0, RED);

            // TODO compute optimal force magnitude
            let rel_vel = self.velocity_at(c.pos).dot(c.normal);
            if rel_vel < 0.0 {
                let j: Vec3 = [c.normal.x, c.normal.y, (c.pos - t.pos).perp_dot(c.normal)].into();
                let w = Mat3::from_diagonal([1.0 / self.mass, 1.0 / self.mass, 1.0 / self.inertia].into());
                let meff_inv = j.dot(w * j);
                let meff = 1.0 / meff_inv;
                let lambda = meff * -rel_vel;
                //self.add_force_at(c.normal * lambda / dt, c.pos);
                self.add_impulse_at(c.normal * lambda, c.pos)
            }

            // This bit resolves penetration
            // TODO compute optimal force magnitude
            self.add_impulse_at(c.normal * c.depth * 10.0, c.pos);
        }

        *self = self.predict(dt);
    }

    fn render(&self) {
        let t = Transform::new(self.pos, self.rot);
        self.shape.render(&t);

    }
}

#[macroquad::main("Hi")]
async fn main() {
    let mut body = Body {
        mass: 1.0,
        inertia: 1000.0,
        pos: [300.0, 350.0].into(),
        rot: 0.0,
        vel: [0.0, 0.0].into(),
        //omega: 0.01,
        omega: 0.0,
        force: Vec2::ZERO,
        torque: 0.0,
        //shape: Shape::Circle { radius: 100.0 },
        shape: Shape::Rect(Rect {
            half_extents: [70.0, 40.0].into(),
        }),
    };

    loop {
        let dt = get_frame_time();

        body.force = [0.0, 10.0].into();
        body.step(dt);
        body.render();
        let g = ground();
        Shape::Rect(g.0).render(&g.1);

        next_frame().await;
    }
}
