use bvh::{BoundingBox, Bvh, Triangle};
use nannou::prelude::*;
use rand::{rngs::StdRng, Rng, SeedableRng};

struct Model {
    _rng: StdRng,
    bvh: Bvh<Vec2>,
}

fn main() {
    nannou::app(Box::new(model))
        .event(event)
        .simple_window(view)
        .run();
}

fn model(_app: &App) -> Model {
    let x_range = -500.0..500.0;
    let y_range = -500.0..500.0;
    let r_range = 20.0..60.0;

    let mut rng = rand::rngs::StdRng::from_seed([3; 32]);

    let mut triangles = vec![];

    for _ in 0..500 {
        let center_x = rng.gen_range(x_range.clone());
        let center_y = rng.gen_range(y_range.clone());
        let l_a = rng.gen_range(0.0..TAU);
        let l_r = rng.gen_range(r_range.clone());
        let m_a = rng.gen_range(0.0..TAU);
        let m_r = rng.gen_range(r_range.clone());
        let n_a = rng.gen_range(0.0..TAU);
        let n_r = rng.gen_range(r_range.clone());
        triangles.push(Triangle([
            vec2(
                (center_x + l_a.cos() * l_r).round(),
                (center_y + l_a.sin() * l_r).round(),
            ),
            vec2(
                (center_x + m_a.cos() * m_r).round(),
                (center_y + m_a.sin() * m_r).round(),
            ),
            vec2(
                (center_x + n_a.cos() * n_r).round(),
                (center_y + n_a.sin() * n_r).round(),
            ),
        ]));
    }

    let bvh = Bvh::new(triangles);

    Model { _rng: rng, bvh }
}

fn event(_app: &App, _model: &mut Model, _event: Event) {}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(BLACK);

    let bb_size = model.bvh.nodes[0].bb.size();
    let scale = vec2(
        app.window_rect().w() / bb_size.x,
        app.window_rect().h() / bb_size.y,
    );

    let mouse = vec2(app.mouse.x / scale.x, app.mouse.y / scale.y);

    draw_bvh(&draw, &model.bvh, 0, mouse, scale);

    draw.to_frame(app, &frame).unwrap();
}

fn draw_bvh(draw: &Draw, bvh: &Bvh<Vec2>, node: usize, mouse: Vec2, scale: Vec2) {
    let node = bvh.nodes[node];
    if node.bb.point_inside(mouse) {
        draw_bb(draw, node.bb, scale);
        if let Some(left) = node.children {
            draw_bvh(draw, bvh, left, mouse, scale);
            draw_bvh(draw, bvh, left + 1, mouse, scale);
        } else {
            for &triangle in &bvh.triangles[node.triangles.0..node.triangles.1] {
                draw_triangle(draw, triangle, mouse, scale)
            }
        }
    }
}

fn draw_triangle(draw: &Draw, triangle: Triangle<Vec2>, mouse: Vec2, scale: Vec2) {
    if triangle.point_inside(mouse) {
        let [a, b, c] = triangle.0;
        draw.tri()
            .points(a * scale, b * scale, c * scale)
            .color(RED)
            .finish();
    }
}

fn draw_bb(draw: &Draw, bb: BoundingBox<Vec2>, scale: Vec2) {
    let wh = bb.size() * scale;
    draw.rect()
        .no_fill()
        .stroke(BLUE)
        .stroke_weight(2.0)
        .x_y(
            bb.min.x * scale.x + wh.x * 0.5,
            bb.min.y * scale.y + wh.y * 0.5,
        )
        .w_h(wh.x, wh.y)
        .finish();
}
