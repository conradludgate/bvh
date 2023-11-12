use std::{
    fs::OpenOptions,
    mem::swap,
    sync::atomic::{self, AtomicUsize},
};

use bvh::{BVHNode, Ray, TotalF32, Triangle};
use glam::{vec3, Vec3};
use image::{codecs::gif::GifEncoder, Frame, Rgba, RgbaImage};
use indicatif::ProgressBar;
use nannou::prelude::TAU;
use rayon::prelude::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use typed_arena::Arena;

fn main() {
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

    let c = 3;
    let d = 5.0;
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

    let arena = Arena::new();
    let bvh = BVHNode::new(&arena, &mut triangles, 0);

    println!("Generated scene index (height={})", bvh.height);

    let size = bvh.bb.size().max_element();
    // let center = (bvh.bb.min + bvh.bb.max) / 2.0;

    let views = 360;
    let progress = ProgressBar::new(views);
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

            let frame = Frame::new(render(bvh, &triangles, origin, &mut bb_count, &mut t_count));

            bb.fetch_add(bb_count, atomic::Ordering::Relaxed);
            t.fetch_add(t_count, atomic::Ordering::Relaxed);

            progress.inc(1);
            frame
        })
        .collect_into_vec(&mut frames);

    println!(
        "performed {bb:?} bounding box intersection tests and {t:?} triangle intersection tests"
    );

    let mut gif = GifEncoder::new(
        OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .open("utah/render.gif")
            .unwrap(),
    );

    gif.encode_frames(frames).unwrap();
}

fn render<'a>(
    bvh: &'a BVHNode<'a, Vec3>,
    triangles: &'a [Triangle<Vec3>],
    origin: Vec3,
    bb_count: &mut usize,
    t_count: &mut usize,
) -> RgbaImage {
    let mut image = image::RgbaImage::new(500, 500);
    let center = (bvh.bb.min + bvh.bb.max) / 2.0;
    let direction = (center - origin).normalize();

    let side = direction.cross(Vec3::Y);

    let scale = -1.0 / 600.0;
    for xp in 0..image.width() {
        for yp in 0..image.height() {
            let x = (xp as i32 - image.width() as i32 / 2) as f32 * scale;
            let y = (yp as i32 - image.height() as i32 / 2) as f32 * scale;
            let direction = (y * Vec3::Y + x * side + direction).normalize();
            // let direction = Quat::from_rotation_x(x) * Quat::from_rotation_y(y) * direction;
            // let direction = Quat::from_euler(EulerRot::XYZ, x, y, 0.0) * direction;

            if let Some((_, i)) = test(bvh, triangles, Ray { origin, direction }, bb_count, t_count)
            {
                let t = triangles[i];
                let n = t.normal().normalize();
                // https://math.stackexchange.com/a/13263
                let r = direction - 2.0 * (direction.dot(n)) * n;
                let r = r.normalize();
                let angle_with_sun = r.dot(vec3(0.0, 1.0, 0.0));

                let a = (angle_with_sun.cos().abs() * 255.0) as u8;
                image.put_pixel(xp, yp, Rgba([a, a, a, 255]));
                // image.put_pixel(xp, yp, Rgba([255, 255, 255, 255]));
            } else {
                image.put_pixel(xp, yp, Rgba([0, 0, 0, 255]));
            }
        }
    }

    image
}

fn test<'a>(
    bvh: &'a BVHNode<'a, Vec3>,
    triangles: &[Triangle<Vec3>],
    ray: Ray,
    bb_count: &mut usize,
    t_count: &mut usize,
) -> Option<(f32, usize)> {
    // if bvh.bb.intersection(ray) {
    if let Some([mut l, mut r]) = bvh.children {
        let mut l1 = l.bb.intersection(ray);
        let mut r1 = r.bb.intersection(ray);
        *bb_count += 2;
        if l1.is_empty() || (!r1.is_empty() && l1.start > r1.start) {
            swap(&mut l, &mut r);
            swap(&mut l1, &mut r1);
        }
        debug_assert!(!l1.is_empty());

        // test the closest bounding box
        if let Some((dist, i)) = test(l, triangles, ray, bb_count, t_count) {
            // if the closest triangle is further than the second bb, test the second
            if !r1.is_empty() && dist >= r1.start {
                if let Some((dist1, i1)) = test(r, triangles, ray, bb_count, t_count) {
                    if dist1 < dist {
                        return Some((dist1, i1));
                    }
                }
            }
            Some((dist, i))
        } else if !r1.is_empty() {
            test(r, triangles, ray, bb_count, t_count)
        } else {
            None
        }
    } else {
        triangles[bvh.triangles.0..bvh.triangles.1]
            .iter()
            .enumerate()
            .filter_map(|(i, t)| {
                *t_count += 1;
                let p = t.intersection(ray)?;
                let dist = p.distance(ray.origin);
                Some((dist, i + bvh.triangles.0))
            })
            .min_by_key(|&(dist, _)| TotalF32(dist))
    }
    // } else {
    //     None
    // }
}
