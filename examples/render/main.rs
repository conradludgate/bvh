use bvh::{BVHNode, Ray, TotalF32, Triangle};
use glam::{vec3, Vec3};
use image::Rgb;
use typed_arena::Arena;

fn main() {
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

    let arena = Arena::new();
    let bvh = BVHNode::new(&arena, &mut triangles, 0);

    let mut image = image::RgbImage::new(200, 200);
    let origin = vec3(0.0, -1.0, -5.0);
    let scale = -1.0 / 120.0;
    for xp in 0..image.width() {
        for yp in 0..image.height() {
            let x = (xp as i32 - image.width() as i32 / 2) as f32 * scale;
            let y = (yp as i32 - image.height() as i32 / 2) as f32 * scale;
            let direction = vec3(x, y, 1.0).normalize();

            if let Some((_, i)) = test(bvh, &triangles, Ray { origin, direction }) {
                let t = triangles[i];
                let n = t.normal().normalize();
                // https://math.stackexchange.com/a/13263
                let r = direction - 2.0 * (direction.dot(n)) * n;
                let r = r.normalize();
                let angle_with_sun = r.dot(vec3(0.0, 1.0, 0.0));

                let a = (angle_with_sun.cos().abs() * 255.0) as u8;
                image.put_pixel(xp, yp, Rgb([a, a, a]));
                // image.put_pixel(xp, yp, Rgb([255,255,255]));
            } else {
                image.put_pixel(xp, yp, Rgb([0, 0, 0]));
            }
        }
    }

    image.save_with_format("utag.png", image::ImageFormat::Png).unwrap();
}

fn test<'a>(
    bvh: &'a BVHNode<'a, Vec3>,
    triangles: &[Triangle<Vec3>],
    ray: Ray,
) -> Option<(f32, usize)> {
    if bvh.bb.intersection(ray).is_some() {
        if let Some([l, r]) = bvh.children {
            match (test(l, triangles, ray), test(r, triangles, ray)) {
                (None, None) => None,
                (None, Some(x)) => Some(x),
                (Some(x), None) => Some(x),
                (Some((x0, x1)), Some((y0, y1))) => {
                    if x0 < y0 {
                        Some((x0, x1))
                    } else {
                        Some((y0, y1))
                    }
                }
            }
        } else {
            triangles[bvh.triangles.0..bvh.triangles.1]
                .iter()
                .enumerate()
                .filter_map(|(i, t)| {
                    let p = t.intersection(ray)?;
                    let dist = p.distance(ray.origin);
                    Some((dist, i))
                })
                .min_by_key(|&(dist, _)| TotalF32(dist))
        }
    } else {
        None
    }
}
