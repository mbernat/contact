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

#[derive(Clone)]
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
    let eps = 1e-3;
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
#[derive(Clone)]
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

fn point_line_dist(p: Vec2, l: &Segment) -> f32 {
    let n = (l.end - l.start).normalize();
    let v = p - l.start;
    let v_proj = v.dot(n) * n;
    (v - v_proj).length()
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

    fn point_distance(&self, p: Vec2) -> f32 {
        let mut min = 1e6;
        for e in &self.edges() {
            let dist = point_line_dist(p, e);
            if dist < min {
                min = dist
            }
        }
        min
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

    fn chain(&self, other: &Polygon) -> Vec<Vec2> {
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
        let pts: Vec<Vec2> = lines.iter().map(|s| s.start).collect();
        /*
        if !lines.is_empty() {
            pts.push(lines.last().unwrap().end)
        }
        */
        pts
    }

    // NOTE assumes simple intersections where the boundary is composed of only 2 chains rather than 4
    fn intersect(&self, other: &Polygon) -> Option<Polygon> {
        let mut c1 = self.chain(other);
        let mut c2 = other.chain(self);
        c1.append(&mut c2);
        if c1.len() >= 3 {
            let p = Polygon(c1);
            p.render(2.0, GREEN);
            Some(p)
        } else {
            None
        }
    }

    fn render(&self, thickness: f32, color: Color) {
        for e in self.edges() {
            e.render(thickness, color)
        }
    }
}

fn ground() -> (Rect, Transform) {
    let r = Rect {
        half_extents: [300.0, 50.0].into(),
    };
    let t = Transform {
        pos: [400.0, 450.0].into(),
        rot: 0.0,
    };
    (r, t)
}

// p is the other polygon, i is the intersection polygon
fn contacts_from_intersection(p: &Polygon, i: &Polygon) -> Vec<Contact> {
    let mut result = vec![];
    for e in &i.edges() {
        // Ignore internal edges
        let a = p.point_distance(e.start);
        let b = p.point_distance(e.end);
        if a > 0.01 || b > 0.01 {
            continue;
        }
        let normal = -(e.end - e.start).perp().normalize();
        let mut max_depth = -1e6;
        for v in &i.0 {
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
                if let Some(ip) = p1.intersect(&p2) {
                    contacts_from_intersection(&p2, &ip)
                    //collide_rect_rect(t, r, &t2, &r2)
                } else {
                    vec![]
                }
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
        // TODO do not update positions, only velocities
        let next = self.predict(dt);
        let t = Transform::new(next.pos, next.rot);

        for c in self.shape.find_contacts(&t) {
            draw_circle(c.pos.x, c.pos.y, 2.0, RED);
            let d = c.depth.clamp(5.0, 50.0);
            draw_line_vec(c.pos, c.pos - c.normal * d, 1.0, RED);

            // TODO compute optimal force magnitude
            let rel_vel = self.velocity_at(c.pos).dot(c.normal);
            if rel_vel < 0.0 {
                let j: Vec3 = [c.normal.x, c.normal.y, (c.pos - t.pos).perp_dot(c.normal)].into();
                let w = Mat3::from_diagonal(
                    [1.0 / self.mass, 1.0 / self.mass, 1.0 / self.inertia].into(),
                );
                let meff_inv = j.dot(w * j);
                let meff = 1.0 / meff_inv;
                let lambda = meff * -rel_vel;
                //self.add_force_at(c.normal * lambda / dt, c.pos);
                self.add_impulse_at(c.normal * lambda, c.pos)
            }

            // This bit resolves penetration
            // TODO compute optimal force magnitude
            // self.add_impulse_at(c.normal * c.depth, c.pos);
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
        pos: [200.0, 250.0].into(),
        rot: 0.2,
        vel: [50.0, 0.0].into(),
        omega: -0.9,
        force: Vec2::ZERO,
        torque: 0.0,
        /*
        shape: Shape::Rect(Rect {
            half_extents: [70.0, 40.0].into(),
        }),
        */
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
