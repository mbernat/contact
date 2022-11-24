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

#[derive(Clone)]
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
}

#[derive(Clone, Debug)]
enum Shape {
    Circle { radius: f32 },
    Polygon(Polygon),
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
            Shape::Polygon(p) => p.transformed(&t).render(1.0, WHITE),
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
    let eps = 1e-2;
    let min = -eps;
    let max = 1.0 + eps;
    if det.abs() < 1e-12 {
        let bs_perp = (b.start - a.start).perp_dot(c.normalize());
        return if bs_perp.abs() < 1e-3 {
            let bs_t = (b.start - a.start).dot(c) / c.length_squared();
            let be_t = (b.end - a.end).dot(c) / c.length_squared();
            if be_t >= min && bs_t <= max {
                let bs_c = bs_t.clamp(min, max);
                let be_c = be_t.clamp(min, max);
                vec![a.start + bs_c * c, a.start + be_c * c]
            } else {
                vec![]
            }
        } else {
            vec![]
        };
    }

    let inv: Mat2 = Mat2::from_cols([d.y, -c.y].into(), [-d.x, c.x].into()) * (1.0 / det);
    let sol = inv * e;
    if sol.x >= min && sol.x <= max && sol.y >= min && sol.y <= max {
        vec![a.start + sol.x * c]
    } else {
        vec![]
    }
}

// convex polygon determined by clockwise-ordered vertices
#[derive(Clone, Debug)]
struct Polygon(Vec<Vec2>);

enum PolygonSegmentResult {
    Inside,
    Outside,
    Entering(Vec2),
    Leaving(Vec2),
    Crossing(Vec2, Vec2),
}

#[cfg(test)]
mod tests {
    use crate::collide_segment_segment;
    use crate::Polygon;
    use crate::Segment;

    #[test]
    fn test_triangle() {
        let p = Polygon(vec![
            [0.0, 100.0].into(),
            [0.0, 0.0].into(),
            [100.0, 100.0].into(),
        ]);

        assert!(p.contains_point([10.0, 10.0].into()));
    }

    #[test]
    fn test_rectangle() {
        let p = Polygon(vec![
            [700.0, 500.0].into(),
            [100.0, 500.0].into(),
            [100.0, 400.0].into(),
            [700.0, 400.0].into(),
        ]);

        assert!(p.contains_point([370.0, 414.0].into()));
    }

    #[test]
    fn test_lines_1() {
        let a = Segment {
            start: [100.0, 400.0].into(),
            end: [700.0, 400.0].into(),
        };
        let b = Segment {
            start: [218.121979, 342.987549].into(),
            end: [245.500473, 400.0].into(),
        };
        let result = collide_segment_segment(&a, &b);
        assert_ne!(result, vec![])
    }

    #[test]
    fn test_lines_2() {
        let a = Segment {
            start: [100.0, 400.0].into(),
            end: [700.0, 400.0].into(),
        };
        let b = Segment {
            start: [233.375381, 337.175507].into(),
            end: [241.056549, 399.952911].into(),
        };
        let result = collide_segment_segment(&a, &b);
        assert_ne!(result, vec![])
    }
}

impl Polygon {
    fn from_rect(r: &Rect, t: &Transform) -> Self {
        Polygon(r.corners(t).into())
    }

    fn transformed(&self, t: &Transform) -> Polygon {
        Polygon(
            self.0
                .iter()
                .map(|v| t.pos + Mat2::from_angle(t.rot) * *v)
                .collect(),
        )
    }

    fn contains_point(&self, p: Vec2) -> bool {
        self.edges()
            .iter()
            .all(|s| (s.end - s.start).perp_dot(p - s.end) >= 0.0)
    }

    fn test_segment(&self, s: &Segment) -> PolygonSegmentResult {
        let p1 = self.contains_point(s.start);
        let p2 = self.contains_point(s.end);
        match (p1, p2) {
            (true, true) => PolygonSegmentResult::Inside,
            (false, false) => {
                let mut cross = vec![];
                for e in &self.edges() {
                    if let Some(p) = collide_segment_segment(s, e).first() {
                        cross.push(*p)
                    }
                }
                // Sometimes one point on the boundary can be reported; we can ignore it
                if cross.len() < 2 {
                    PolygonSegmentResult::Outside
                } else {
                    // TODO This doesn't always hold, we can easily intersect more lines at vertices
                    // assert_eq!(cross.len(), 2);
                    if (s.start - cross[0]).length_squared()
                        <= (s.start - cross[1]).length_squared()
                    {
                        PolygonSegmentResult::Crossing(cross[0], cross[1])
                    } else {
                        PolygonSegmentResult::Crossing(cross[1], cross[0])
                    }
                }
            }
            (_, _) => {
                let p = self
                    .edges()
                    .iter()
                    .find_map(|e| collide_segment_segment(e, s).first().copied())
                    .expect("Intersection");
                if p1 == false {
                    PolygonSegmentResult::Entering(p)
                } else {
                    PolygonSegmentResult::Leaving(p)
                }
            }
        }
    }

    fn edges(&self) -> Vec<Segment> {
        let mut res = vec![];
        for i in 0..self.0.len() - 1 {
            res.push(Segment {
                start: self.0[i],
                end: self.0[i + 1],
            })
        }
        res.push(Segment {
            start: self.0[self.0.len() - 1],
            end: self.0[0],
        });
        res
    }

    fn chain(&self, other: &Polygon) -> Vec<Segment> {
        let mut inside1 = vec![];
        let mut inside2 = vec![];
        let mut was_outside = false;
        for e in self.edges().iter() {
            match other.test_segment(e) {
                PolygonSegmentResult::Inside => {
                    if was_outside {
                        inside1.push(e.clone())
                    } else {
                        inside2.push(e.clone())
                    }
                }
                PolygonSegmentResult::Entering(p) => {
                    was_outside = true;
                    inside1.push(Segment {
                        start: p,
                        end: e.end,
                    });
                }
                PolygonSegmentResult::Leaving(p) => {
                    let s = Segment {
                        start: e.start,
                        end: p,
                    };
                    if was_outside {
                        inside1.push(s)
                    } else {
                        inside2.push(s)
                    }
                }
                PolygonSegmentResult::Crossing(p1, p2) => {
                    was_outside = true;
                    inside1.push(Segment { start: p1, end: p2 });
                }
                PolygonSegmentResult::Outside => was_outside = true,
            }
        }
        let mut lines = inside1.clone();
        lines.append(&mut inside2);
        lines
    }

    // NOTE assumes simple intersections where the other polygon's intersection boundary is connected
    fn intersect(&self, other: &Polygon) -> Vec<Segment> {
        let chain = other.chain(self);
        for s in chain.iter() {
            s.render(5.0, GREEN)
        }
        chain
    }

    // chain is the part of the other polygon's boundary that intersects us
    fn contacts_from_intersection(&self, chain: &Vec<Segment>) -> Vec<Contact> {
        let mut result = vec![];
        for e in chain.iter() {
            let diff = e.end - e.start;
            // TODO handle very short segments properly
            if diff.length() < 1.0 {
                continue;
            }
            let normal = -diff.perp().normalize();
            let mut max_depth = 0.0;
            for v in &self.0 {
                let depth = -(*v - e.start).dot(normal);
                if depth > max_depth {
                    max_depth = depth
                }
            }
            result.push(Contact {
                pos: e.start,
                normal,
                depth: max_depth,
            });
            result.push(Contact {
                pos: e.end,
                normal,
                depth: max_depth,
            });
        }
        result
    }

    fn render(&self, thickness: f32, color: Color) {
        for e in self.edges() {
            e.render(thickness, color)
        }
    }
}

fn ground() -> (Rect, Transform) {
    let r = Rect {
        half_extents: [150.0, 50.0].into(),
    };
    let t = Transform {
        pos: [350.0, 450.0].into(),
        rot: 0.0,
    };
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
            Shape::Polygon(p) => {
                let (r2, t2) = ground();
                let p1 = p.transformed(t);
                let p2 = Polygon::from_rect(&r2, &t2);
                let chain = p1.intersect(&p2);
                p1.contacts_from_intersection(&chain)
            }
        }
    }
}

#[derive(Clone, Debug)]
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
    fn update_vel(&mut self, dt: f32) {
        self.vel += self.force / self.mass * dt;
        self.omega += self.torque / self.inertia * dt;
    }

    fn update_pos(&mut self, dt: f32) {
        self.pos += self.vel * dt;
        self.rot += self.omega * dt;
    }

    fn clear_forces(&mut self) {
        self.force = Vec2::ZERO;
        self.torque = 0.0;
    }

    fn velocity_vector(&self) -> Vec3 {
        [self.vel.x, self.vel.y, self.omega].into()
    }

    fn step(&mut self, dt: f32) {
        self.update_vel(dt);
        let t = Transform::new(self.pos, self.rot);

        let contacts = self.shape.find_contacts(&t);

        for c in self.shape.find_contacts(&t) {
            draw_circle(c.pos.x, c.pos.y, 3.0, RED);
            let d = c.depth.clamp(10.0, 50.0);
            draw_line_vec(c.pos, c.pos - c.normal * d, 1.0, RED);
        }

        let mut acc_impulses = vec![0.0; contacts.len()];

        let w = Mat3::from_diagonal([1.0 / self.mass, 1.0 / self.mass, 1.0 / self.inertia].into());

        let num_iters = 4;
        for _i in 0..num_iters {
            for (ci, c) in contacts.iter().enumerate() {
                let cross = (c.pos - self.pos).perp_dot(c.normal);
                let j: Vec3 = [c.normal.x, c.normal.y, cross].into();

                let penetration_vel = -0.1 * c.depth / dt;
                // This works very strangely
                let rel_vel = j.dot(self.velocity_vector()) + penetration_vel;
                let meff_inv = j.dot(w * j);
                let meff = 1.0 / meff_inv;
                let lambda = -meff * rel_vel;
                let lambda_prev = acc_impulses[ci];
                acc_impulses[ci] = (lambda_prev + lambda).max(0.0);
                let vel_change = w * j * (acc_impulses[ci] - lambda_prev);
                self.vel += Vec2::new(vel_change.x, vel_change.y);
                self.omega += vel_change.z;

                // This bit resolves penetration
                // TODO compute optimal force magnitude
                // self.add_impulse_at(c.normal * c.depth, c.pos);
            }
        }

        self.update_pos(dt);
        self.clear_forces();
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
        pos: [200.0, 300.0].into(),
        rot: 0.2,
        vel: [50.0, 0.0].into(),
        omega: -0.5,
        force: Vec2::ZERO,
        torque: 0.0,
        shape: Shape::Polygon(Polygon(vec![
            [30.0, -20.0].into(),
            [40.0, 0.0].into(),
            [50.0, 40.0].into(),
            [-20.0, 30.0].into(),
            [-20.0, -10.0].into(),
        ])),
    };

    loop {
        let dt = get_frame_time();
        //let dt = 0.0;

        body.force = [0.0, 100.0].into();
        body.step(dt);
        body.render();
        let g = ground();
        let p = Polygon(g.0.corners(&g.1).into());
        p.render(1.0, WHITE);

        next_frame().await;
    }
}
