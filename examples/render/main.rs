// use winit::{
//     event::{Event, WindowEvent},
//     event_loop::{ControlFlow, EventLoop},
//     window::WindowBuilder,
// };

// fn main() {
//     let event_loop = EventLoop::new().unwrap();
//     let window = WindowBuilder::new().build(&event_loop).unwrap();

//     // ControlFlow::Wait pauses the event loop if no events are available to process.
//     // This is ideal for non-game applications that only update in response to user
//     // input, and uses significantly less power/CPU time than ControlFlow::Poll.
//     event_loop.set_control_flow(ControlFlow::Wait);

//     event_loop
//         .run(move |event, elwt| {
//             match event {
//                 Event::WindowEvent {
//                     event: WindowEvent::CloseRequested,
//                     ..
//                 } => {
//                     elwt.exit();
//                 }
//                 Event::AboutToWait => {
//                     // Application update code.

//                     // Queue a RedrawRequested event.
//                     //
//                     // You only need to call this if you've determined that you need to redraw, in
//                     // applications which do not always need to. Applications that redraw continuously
//                     // can just render here instead.
//                     window.request_redraw();
//                 }
//                 Event::WindowEvent {
//                     event: WindowEvent::RedrawRequested,
//                     ..
//                 } => {
//                     // Redraw the application.
//                     //
//                     // It's preferable for applications that do not render continuously to render in
//                     // this event rather than in AboutToWait, since rendering in here allows
//                     // the program to gracefully handle redraws requested by the OS.
//                 }
//                 _ => (),
//             }
//         })
//         .unwrap();
// }

use bvh::{BVHNode, BoundingBox, Triangle};
use nannou::prelude::*;
use rand::{rngs::StdRng, Rng, SeedableRng};
use typed_arena::Arena;

struct Model<'a> {
    rng: StdRng,
    triangles: Vec<Triangle<Vec2>>,
    bvh: &'a bvh::BVHNode<'a, Vec2>,
}

fn main() {
    let arena = Box::leak(Box::new(Arena::new()));

    nannou::app(Box::new(|app| model(&*arena, app)))
        .event(event)
        .simple_window(view)
        .run();
}

fn model<'a>(arena: &'a Arena<BVHNode<'a, Vec2>>, app: &App) -> Model<'a> {
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

    let bvh = BVHNode::new(arena, &mut triangles, 0);

    Model {
        rng,
        triangles,
        bvh,
    }
}

fn event(_app: &App, model: &mut Model, event: Event) {
    // match event {
    //     Event::WindowEvent { id, simple } => todo!(),
    //     Event::DeviceEvent(_, _) => todo!(),
    //     Event::Update(_) => todo!(),
    //     Event::Suspended => todo!(),
    //     Event::Resumed => todo!(),
    // }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(BLACK);

    let bb_size = model.bvh.bb.size();
    let scale = vec2(
        app.window_rect().w() / bb_size.x,
        app.window_rect().h() / bb_size.y,
    );

    let mouse = vec2(app.mouse.x / scale.x, app.mouse.y / scale.y);

    draw_bvh(&draw, &model.triangles, model.bvh, mouse, scale);

    draw.to_frame(app, &frame).unwrap();
}

fn draw_bvh<'a>(
    draw: &Draw,
    triangles: &[Triangle<Vec2>],
    bvh: &'a BVHNode<'a, Vec2>,
    mouse: Vec2,
    scale: Vec2,
) {
    if bvh.bb.point_inside(mouse) {
        draw_bb(draw, bvh.bb, scale);
        if let Some([left, right]) = bvh.children {
            draw_bvh(draw, triangles, left, mouse, scale);
            draw_bvh(draw, triangles, right, mouse, scale);
        } else {
            for &triangle in &triangles[bvh.triangles.0..bvh.triangles.1] {
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