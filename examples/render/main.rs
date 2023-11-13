use std::{
    fs::OpenOptions,
    mem::swap,
    sync::atomic::{self, AtomicUsize},
    time::Instant,
};

use bvh::{Bvh, Ray, TotalF32, Triangle};
use glam::{vec3, Vec3};
use image::{codecs::gif::GifEncoder, Delay, Frame, Rgba, RgbaImage};
use indicatif::{ProgressBar, ProgressStyle};
use nannou::prelude::TAU;
use rayon::prelude::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};

fn main() {
    let start = Instant::now();

    // https://users.cs.utah.edu/~dejohnso/models/teapot.html
    let file = include_str!("utah.txt");
    let (triangles, mut file) = file.split_once('\n').unwrap();
    let n: usize = triangles.parse().unwrap();

    let mut triangles = vec![];
    for _ in 0..n {
        let r = file;
        let (a, r) = r.split_once('\n').unwrap();
        let (b, r) = r.split_once('\n').unwrap();
        let (c, r) = r.split_once('\n').unwrap();

        fn parse_coord(line: &str) -> Vec3 {
            let (x, r) = line.split_once(' ').expect(line);
            let (y, z) = r.split_once(' ').unwrap();
            vec3(x.parse().unwrap(), y.parse().unwrap(), z.parse().unwrap())
        }

        triangles.push(Triangle([parse_coord(a), parse_coord(b), parse_coord(c)]));

        let Some((_, r)) = r.split_once('\n') else {
            break;
        };
        file = r;
    }
    assert_eq!(triangles.len(), n);

    println!("Loaded geometry");

    let c = 2;
    let d = 10.0;
    for i in -c..=c {
        for j in -c..=c {
            for k in -c..=c {
                if i == 0 && j == 0 && k == 0 {
                    continue;
                }
                let v = vec3((i as f32) * d, (j as f32) * d, (k as f32) * d);

                for t in 0..n {
                    let mut t = triangles[t];
                    t.0[0] += v;
                    t.0[1] += v;
                    t.0[2] += v;
                    triangles.push(t);
                }
            }
        }
    }

    println!("Built scene (triangles={})", triangles.len());

    let bvh = Bvh::new(triangles);

    println!("Generated scene index (height={})", bvh.height);

    let size = bvh.nodes[0].bb.size().max_element();
    // let center = (bvh.bb.min + bvh.bb.max) / 2.0;

    let views = 36; //360 * 3;
    let progress = ProgressBar::new(views * (WIDTH * HEIGHT) as u64 * (AA * AA) as u64)
        .with_elapsed(start.elapsed());
    progress.set_style(
        ProgressStyle::with_template("{elapsed}/{duration} {wide_bar} {pos}/{len}").unwrap(),
    );
    let bb = AtomicUsize::new(0);
    let t = AtomicUsize::new(0);
    let mut frames = vec![];
    (0..views as usize)
        .into_par_iter()
        .map(|a| {
            let a = (a as f32) / (views as f32) * TAU;
            let x = 2.0 * size * a.sin();
            let z = 2.0 * size * a.cos();
            let origin = vec3(x, -1.0, z);

            let mut bb_count = 0;
            let mut t_count = 0;

            let frame = Frame::from_parts(
                render(&bvh, origin, &mut bb_count, &mut t_count, &progress),
                0,
                0,
                Delay::from_numer_denom_ms(10, 1),
            );

            bb.fetch_add(bb_count, atomic::Ordering::Relaxed);
            t.fetch_add(t_count, atomic::Ordering::Relaxed);

            frame
        })
        .collect_into_vec(&mut frames);

    println!(
        "performed {bb:?} bounding box intersection tests and {t:?} triangle intersection tests"
    );

    let mut gif = GifEncoder::new_with_speed(
        OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .open("utah/render.gif")
            .unwrap(),
        10,
    );

    gif.encode_frames(frames).unwrap();
}

const SCALE: f32 = -1.0 / 600.0;
const AA: i32 = 4;
const WIDTH: u32 = 500;
const HEIGHT: u32 = 500;

fn render(
    bvh: &Bvh<Vec3>,
    origin: Vec3,
    bb_count: &mut usize,
    t_count: &mut usize,
    progress: &ProgressBar,
) -> RgbaImage {
    let mut image = image::RgbaImage::new(WIDTH, HEIGHT);
    let bb = bvh.nodes[0].bb;
    let center = (bb.min + bb.max) / 2.0;
    let direction = (center - origin).normalize();

    let side = direction.cross(Vec3::Y);

    let scale = SCALE / (AA as f32);
    for xp in 0..WIDTH {
        for yp in 0..HEIGHT {
            let mut colour = 0;
            for xaa in 0..AA {
                for yaa in 0..AA {
                    let x = ((xp as i32 - WIDTH as i32 / 2) * AA + xaa) as f32 * scale;
                    let y = ((yp as i32 - HEIGHT as i32 / 2) * AA + yaa) as f32 * scale;
                    let direction = (y * Vec3::Y + x * side + direction).normalize();

                    if let Some((_, t)) =
                        test(bvh, 0, Ray::new(origin, direction), bb_count, t_count)
                    {
                        let n = t.normal().normalize();
                        // https://math.stackexchange.com/a/13263
                        let r = direction - 2.0 * (direction.dot(n)) * n;
                        let r = r.normalize();
                        let angle_with_sun = r.dot(vec3(0.0, 1.0, 0.0));

                        let a = (angle_with_sun.cos().abs() * 255.0) as i32;
                        colour += a;
                        // image.put_pixel(xp, yp, Rgba([a, a, a, 255]));
                        // image.put_pixel(xp, yp, Rgba([255, 255, 255, 255]));
                    } else {
                        // image.put_pixel(xp, yp, Rgba([0, 0, 0, 255]));
                    }
                }
            }
            progress.inc((AA * AA) as u64);
            let c = (colour / AA / AA) as u8;
            image.put_pixel(xp, yp, Rgba([c, c, c, 255]));
        }
    }

    image
}

fn test(
    bvh: &Bvh<Vec3>,
    node: usize,
    ray: Ray,
    bb_count: &mut usize,
    t_count: &mut usize,
) -> Option<(f32, Triangle<Vec3>)> {
    let node = bvh.nodes[node];
    if let Some(child) = node.children {
        let mut l = child;
        let mut r = child + 1;
        let [mut l1, mut r1] = ray.box_intersections([bvh.nodes[l].bb, bvh.nodes[r].bb]);
        *bb_count += 2;
        if l1.is_empty() || (!r1.is_empty() && l1.start > r1.start) {
            swap(&mut l, &mut r);
            swap(&mut l1, &mut r1);
        }

        // test the closest bounding box
        if let Some((dist, i)) = test(bvh, l, ray, bb_count, t_count) {
            // if the closest triangle is further than the second bb, test the second
            if !r1.is_empty() && dist >= r1.start {
                if let Some((dist1, i1)) = test(bvh, r, ray, bb_count, t_count) {
                    if dist1 < dist {
                        return Some((dist1, i1));
                    }
                }
            }
            Some((dist, i))
        } else if !r1.is_empty() {
            test(bvh, r, ray, bb_count, t_count)
        } else {
            None
        }
    } else {
        let tri = node.triangles;
        bvh.triangles[tri]
            .iter()
            .filter_map(|t| {
                *t_count += 1;
                let p = t.intersection(ray)?;
                let dist = p.distance(ray.origin);
                Some((dist, *t))
            })
            .min_by_key(|&(dist, _)| TotalF32(dist))
    }
}
