use macroquad::prelude::{draw_line, Color, Vec2, WHITE};

pub fn draw_line_vec(a: Vec2, b: Vec2, thickness: f32, color: Color) {
    draw_line(a.x, a.y, b.x, b.y, thickness, color)
}

pub fn draw_central_line(pos: Vec2, half_extent: Vec2) {
    draw_line_vec(pos - half_extent, pos + half_extent, 1.0, WHITE);
}
