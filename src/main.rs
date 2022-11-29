use body::Body;
use engine::World;
use geometry::{Geometry, Rect, Shape, Transform};
use macroquad::prelude::*;
use macroquad::window::next_frame;
use polygon::Polygon;

mod body;
mod draw;
mod engine;
mod geometry;
mod polygon;

fn _falling_polygons() -> World {
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

    World::new(&[body1, body2], &[ground()])
}

fn _impact_squares() -> World {
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
        rot: 0.1,
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
    World::new(&[body1, body2], &[ground()])
}

fn _square_stack() -> World {
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
    World::new(&[body1, body2], &[ground()])
}

// TODO move into engine
fn ground() -> Geometry {
    let r = Rect {
        half_extents: [250.0, 50.0].into(),
    };
    let t = Transform {
        pos: [350.0, 450.0].into(),
        rot: 0.0,
    };
    let p = Shape::Polygon(Polygon::from_rect(&r, &Transform::default()));
    Geometry { trans: t, shape: p }
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

    fn selected(&self) -> &World {
        &self.history[self.index]
    }

    fn process_input(&mut self) {
        if is_key_pressed(KeyCode::Space) {
            self.simulate = !self.simulate;
            if self.simulate {
                self.index = self.history.len() - 1;
            }
        }

        if is_key_pressed(KeyCode::Enter) {
            let _: Vec<World> = self.history.drain(self.index + 1..).collect();
            self.simulate = true;
        }

        if is_key_down(KeyCode::Left) {
            self.simulate = false;
            if self.index > 0 {
                self.index -= 1;
            }
        }

        if is_key_down(KeyCode::Right) {
            self.simulate = false;
            if self.index < self.history.len() - 1 {
                self.index += 1;
            }
        }
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

        control.process_input();

        if control.simulate {
            let mut world = control.history.last().unwrap().clone();
            for body in &mut world.bodies {
                body.force = [0.0, gravity].into();
            }
            world.step(dt);
            control.push(&world);
        }

        let snapshot = control.selected();
        snapshot.render();

        next_frame().await;
    }
}
