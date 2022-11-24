use crate::draw;
use crate::polygon::Polygon;
use macroquad::prelude as mq;
use macroquad::prelude::{Color, Mat2, Vec2, WHITE};

#[derive(Default)]
pub struct Transform {
    pub pos: Vec2,
    pub rot: f32,
}

impl Transform {
    pub fn new(pos: Vec2, rot: f32) -> Transform {
        Transform { pos, rot }
    }
}

#[derive(Clone)]
pub struct Segment {
    pub start: Vec2,
    pub end: Vec2,
}

impl Segment {
    pub fn render(&self, thickness: f32, color: Color) {
        draw::draw_line_vec(self.start, self.end, thickness, color)
    }
}

pub fn intersect_segments(a: &Segment, b: &Segment) -> Vec<Vec2> {
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

#[cfg(test)]
mod tests {
    use super::intersect_segments;
    use super::Segment;

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
        let result = intersect_segments(&a, &b);
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
        let result = intersect_segments(&a, &b);
        assert_ne!(result, vec![])
    }
}

#[derive(Clone)]
pub struct Rect {
    pub half_extents: Vec2,
}

impl Rect {
    pub fn corners(&self, t: &Transform) -> [Vec2; 4] {
        let mat = Mat2::from_angle(t.rot);
        // bottom right
        let v1 = mat * self.half_extents;
        // top right
        let v2 = mat * Vec2::new(self.half_extents.x, -self.half_extents.y);
        [t.pos + v1, t.pos - v2, t.pos - v1, t.pos + v2]
    }
}

#[derive(Clone, Debug)]
pub enum Shape {
    Circle { radius: f32 },
    Polygon(Polygon),
}

impl Shape {
    pub fn render(&self, t: &Transform) {
        let rot = t.rot.to_degrees();
        let mat = Mat2::from_angle(rot);

        match self {
            Shape::Circle { radius } => {
                mq::draw_circle_lines(t.pos.x, t.pos.y, *radius, 1.0, WHITE);
                draw::draw_central_line(t.pos, mat.col(0) * *radius);
                draw::draw_central_line(t.pos, mat.col(1) * *radius);
            }
            Shape::Polygon(p) => p.transformed(&t).render(1.0, WHITE),
        }
    }
}
