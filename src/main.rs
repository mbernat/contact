use body::Body;
use engine::World;
use geometry::{Rect, Shape, Transform};
use macroquad::prelude::*;
use macroquad::window::next_frame;
use polygon::Polygon;

mod body;
mod draw;
mod engine;
mod geometry;
mod polygon;

fn falling_polygons() -> World {
    let body1 = Body {
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

    let body2 = Body {
        pos: [400.0, 300.0].into(),
        vel: [-50.0, 0.0].into(),
        ..body1.clone()
    };

    engine::World {
        bodies: vec![body1, body2],
    }
}

fn impact_squares() -> World {
    let r = Rect {
        half_extents: [50.0, 50.0].into(),
    };
    let s = Shape::Polygon(Polygon::from_rect(&r, &Transform::default()));
    let b1 = Body {
        mass: 1.0,
        inertia: 1000.0,
        pos: [200.0, 200.0].into(),
        rot: 0.0,
        vel: [100.0, 0.0].into(),
        omega: 0.0,
        force: Vec2::ZERO,
        torque: 0.0,
        shape: s,
    };
    let b2 = Body {
        pos: [400.0, 200.0].into(),
        vel: [-100.0, 0.0].into(),
        ..b1.clone()
    };
    World {
        bodies: vec![b1, b2],
    }
}

fn square_stack() -> World {
    let r = Rect {
        half_extents: [50.0, 50.0].into(),
    };
    let s = Shape::Polygon(Polygon::from_rect(&r, &Transform::default()));
    let b1 = Body {
        mass: 1.0,
        inertia: 1000.0,
        pos: [300.0, 100.0].into(),
        rot: 0.0,
        vel: [0.0, 0.0].into(),
        omega: 0.0,
        force: Vec2::ZERO,
        torque: 0.0,
        shape: s,
    };
    let b2 = Body {
        pos: [300.0, 250.0].into(),
        ..b1.clone()
    };
    World {
        bodies: vec![b1, b2],
    }
}

// TODO move into engine
fn ground() -> Shape {
    let r = Rect {
        half_extents: [250.0, 50.0].into(),
    };
    let t = Transform {
        pos: [350.0, 450.0].into(),
        rot: 0.0,
    };
    Shape::Polygon(Polygon::from_rect(&r, &t))
}

#[macroquad::main("2d physics engine")]
async fn main() {
    let mut engine = square_stack();
    let gravity = 100.0;

    loop {
        let dt = get_frame_time();
        //let dt = 0.0;

        for body in &mut engine.bodies {
            body.force = [0.0, gravity].into();
        }
        engine.step(dt);
        for body in &engine.bodies {
            body.render();
        }
        let g = ground();
        g.render(&Transform::default());

        next_frame().await;
    }
}
