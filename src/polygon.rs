use crate::engine::Contact;
use crate::geometry::{collide_segment_segment, Rect, Segment, Transform};
use macroquad::color::{Color, GREEN};
use macroquad::math::{Mat2, Vec2};

// convex polygon determined by clockwise-ordered vertices
#[derive(Clone, Debug)]
pub struct Polygon(pub Vec<Vec2>);

pub enum PolygonSegmentResult {
    Inside,
    Outside,
    Entering(Vec2),
    Leaving(Vec2),
    Crossing(Vec2, Vec2),
}

impl Polygon {
    pub fn from_rect(r: &Rect, t: &Transform) -> Self {
        Polygon(r.corners(t).into())
    }

    pub fn transformed(&self, t: &Transform) -> Polygon {
        Polygon(
            self.0
                .iter()
                .map(|v| t.pos + Mat2::from_angle(t.rot) * *v)
                .collect(),
        )
    }

    pub fn contains_point(&self, p: Vec2) -> bool {
        self.edges()
            .iter()
            .all(|s| (s.end - s.start).perp_dot(p - s.end) >= 0.0)
    }

    pub fn test_segment(&self, s: &Segment) -> PolygonSegmentResult {
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

    pub fn edges(&self) -> Vec<Segment> {
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

    pub fn chain(&self, other: &Polygon) -> Vec<Segment> {
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
    pub fn intersect(&self, other: &Polygon) -> Vec<Segment> {
        let chain = other.chain(self);
        for s in chain.iter() {
            s.render(5.0, GREEN)
        }
        chain
    }

    // TODO move to engine::Contact
    // chain is the part of the other polygon's boundary that intersects us
    pub fn contacts_from_intersection(
        &self,
        chain: &Vec<Segment>,
        this_body_index: usize,
        other_body_index: Option<usize>,
    ) -> Vec<Contact> {
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
                this_body_index,
                other_body_index,
            });
            result.push(Contact {
                pos: e.end,
                normal,
                depth: max_depth,
                this_body_index,
                other_body_index,
            });
        }
        result
    }

    pub fn render(&self, thickness: f32, color: Color) {
        for e in self.edges() {
            e.render(thickness, color)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Polygon;

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
}
