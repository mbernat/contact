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

    World::new(&vec![body1, body2])
}

fn impact_squares() -> World {
    let r = Rect {
        half_extents: [50.0, 50.0].into(),
    };
    let s = Shape::Polygon(Polygon::from_rect(&r, &Transform::default()));
    impact_shapes(s)
}

fn impact_circles() -> World {
    let c = Shape::Circle { radius: 50.0 };
    impact_shapes(c)
}

fn impact_shapes(shape: Shape) -> World {
    let body1 = Body {
        mass: 1.0,
        inertia: 1000.0,
        pos: [200.0, 200.0].into(),
        rot: 0.0,
        vel: [100.0, 0.0].into(),
        omega: 0.0,
        force: Vec2::ZERO,
        torque: 0.0,
        shape,
    };
    let body2 = Body {
        pos: [400.0, 200.0].into(),
        vel: [-100.0, 0.0].into(),
        ..body1.clone()
    };
    World::new(&vec![body1, body2])
}

fn square_stack() -> World {
    let r = Rect {
        half_extents: [50.0, 50.0].into(),
    };
    let s = Shape::Polygon(Polygon::from_rect(&r, &Transform::default()));
    let body1 = Body {
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
    let body2 = Body {
        pos: [300.0, 250.0].into(),
        ..body1.clone()
    };
    World::new(&vec![body1, body2])
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

struct Control {
    simulate: bool,
    history: Vec<World>,
    index: usize,
}

impl Control {
    fn push(&mut self, world: &World) {
        self.history.push(world.clone());
        self.index += 1;
    }

    fn current(&self) -> &World {
        &self.history[self.index]
    }
}

#[macroquad::main("2d physics engine")]
async fn main() {
    let gravity = 100.0;

    let world = impact_circles();
    let history = vec![world.clone()];
    let mut control = Control {
        simulate: true,
        history,
        index: 0,
    };

    loop {
        let dt = get_frame_time();
        //let dt = 0.0;

        if is_key_pressed(KeyCode::Space) {
            control.simulate = !control.simulate;
            if control.simulate {
                control.index = control.history.len() - 1;
            }
        }

        if is_key_pressed(KeyCode::Enter) {
            let _: Vec<World> = control.history.drain(control.index + 1..).collect();
            control.simulate = true;
        }

        if is_key_down(KeyCode::Left) {
            control.simulate = false;
            if control.index > 0 {
                control.index -= 1;
            }
        }

        if is_key_down(KeyCode::Right) {
            control.simulate = false;
            if control.index < control.history.len() - 1{
                control.index += 1;
            }
        }

        let mut world = control.history.last().unwrap().clone();

        if control.simulate {
            for body in &mut world.bodies {
                body.force = [0.0, gravity].into();
            }
            world.step(dt);
            control.push(&world);
        }

        let snapshot = &control.current();
        snapshot.render();

        let g = ground();
        g.render(&Transform::default());

        next_frame().await;
    }
}
