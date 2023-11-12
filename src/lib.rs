use std::{
    f32::EPSILON,
    fmt::Debug,
    ops::{Add, Div, Sub},
};

use glam::{Vec2, Vec3};
use itertools::Itertools;
use typed_arena::Arena;

#[derive(Debug, Clone, Copy)]
pub struct Triangle<T>(pub [T; 3]);
#[derive(Debug, Clone, Copy)]
pub struct BoundingBox<T> {
    pub min: T,
    pub max: T,
}

pub trait VecT:
    Sized + Add<Self, Output = Self> + Sub<Self, Output = Self> + Div<f32, Output = Self> + Copy + Debug
{
    fn min(self, rhs: Self) -> Self;
    fn max(self, rhs: Self) -> Self;
    fn axes() -> usize;
    fn axis(&self, axis: usize) -> f32;
}

impl VecT for Vec2 {
    fn min(self, rhs: Self) -> Self {
        self.min(rhs)
    }

    fn max(self, rhs: Self) -> Self {
        self.max(rhs)
    }

    fn axes() -> usize {
        2
    }

    fn axis(&self, axis: usize) -> f32 {
        [self.x, self.y][axis]
    }
}

impl VecT for Vec3 {
    fn min(self, rhs: Self) -> Self {
        self.min(rhs)
    }

    fn max(self, rhs: Self) -> Self {
        self.max(rhs)
    }

    fn axes() -> usize {
        3
    }

    fn axis(&self, axis: usize) -> f32 {
        [self.x, self.y, self.z][axis]
    }
}

impl<V: VecT> Triangle<V> {
    pub fn centroid(self) -> V {
        let [l, m, n] = self.0;
        (l + m + n) / 3.0
    }

    pub fn bounding_box(self) -> BoundingBox<V> {
        let [l, m, n] = self.0;
        BoundingBox {
            min: l.min(m).min(n),
            max: l.max(m).max(n),
        }
    }
}

fn sign(p1: Vec2, p2: Vec2, p3: Vec2) -> f32 {
    (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)
}

impl Triangle<Vec2> {
    pub fn point_inside(self, pt: Vec2) -> bool {
        let [v1, v2, v3] = self.0;
        let d1 = sign(pt, v1, v2);
        let d2 = sign(pt, v2, v3);
        let d3 = sign(pt, v3, v1);

        let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
        let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

        !(has_neg && has_pos)
    }
}

impl BoundingBox<Vec2> {
    pub fn point_inside(self, pt: Vec2) -> bool {
        let x = self.min.x..=self.max.x;
        let y = self.min.y..=self.max.y;
        x.contains(&pt.x) && y.contains(&pt.y)
    }
}

#[derive(Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Triangle<Vec3> {
    pub fn intersection(self, ray: Ray) -> Option<Vec3> {
        // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
        let [v0, v1, v2] = self.0;
        let (e1, e2) = (v1 - v0, v2 - v0);
        let h = ray.direction.cross(e2);
        let a = e1.dot(h);

        if (-EPSILON..EPSILON).contains(&a) {
            return None;
        }

        let f = a.recip();
        let s = ray.origin - v0;
        let u = f * s.dot(h);

        if !(0.0..=1.0).contains(&u) {
            return None;
        }

        let q = s.cross(e1);
        let v = f * ray.direction.dot(q);

        if v < 0.0 || u + v > 1.0 {
            return None;
        }

        let t = f * e2.dot(q);

        if t > EPSILON {
            Some(ray.origin + ray.direction * t)
        } else {
            None
        }
    }

    pub fn normal(self) -> Vec3 {
        let [v0, v1, v2] = self.0;
        let (e1, e2) = (v1 - v0, v2 - v0);
        e1.cross(e2)
    }
}

impl BoundingBox<Vec3> {
    pub fn intersection(self, ray: Ray) -> bool {
        // https://tavianator.com/2022/ray_box_boundary.html
        let mut tmin = 0.0;
        let mut tmax = f32::INFINITY;

        let t1 = (self.min.x - ray.origin.x) / ray.direction.x;
        let t2 = (self.max.x - ray.origin.x) / ray.direction.x;

        tmin = f32::max(tmin, f32::min(t1, t2));
        tmax = f32::min(tmax, f32::max(t1, t2));

        let t1 = (self.min.y - ray.origin.y) / ray.direction.y;
        let t2 = (self.max.y - ray.origin.y) / ray.direction.y;

        tmin = f32::max(tmin, f32::min(t1, t2));
        tmax = f32::min(tmax, f32::max(t1, t2));

        let t1 = (self.min.z - ray.origin.z) / ray.direction.z;
        let t2 = (self.max.z - ray.origin.z) / ray.direction.z;

        tmin = f32::max(tmin, f32::min(t1, t2));
        tmax = f32::min(tmax, f32::max(t1, t2));

        tmin < tmax
    }
}

impl<V: VecT> BoundingBox<V> {
    pub fn merge(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn size(self) -> V {
        self.max - self.min
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BVHNode<'a, V> {
    // Range<usize>
    pub triangles: (usize, usize),
    pub children: Option<[&'a BVHNode<'a, V>; 2]>,
    pub bb: BoundingBox<V>,
    pub height: usize,
}

pub struct TotalF32(pub f32);
impl Eq for TotalF32 {}
impl PartialEq for TotalF32 {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}
impl PartialOrd for TotalF32 {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for TotalF32 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}

impl<'a, T: VecT> BVHNode<'a, T> {
    pub fn new(
        arena: &'a Arena<BVHNode<'a, T>>,
        triangles: &mut [Triangle<T>],
        offset: usize,
    ) -> &'a Self {
        let bounding_box = triangles
            .iter()
            .map(|t| t.bounding_box())
            .tree_fold1(BoundingBox::merge)
            .expect("triangles length should be > 0");

        if triangles.len() <= 4 {
            return arena.alloc(BVHNode {
                triangles: (offset, offset + triangles.len()),
                children: None,
                bb: bounding_box,
                height: 0,
            });
        }

        let axes = T::axes();
        let axis = (0..axes)
            .max_by_key(|&axis| TotalF32(bounding_box.size().axis(axis)))
            .expect("vector should have axes > 0");

        let mid = (triangles.len() + 1) / 2;
        triangles.select_nth_unstable_by_key(mid, |t| TotalF32(t.centroid().axis(axis)));
        let (left, right) = triangles.split_at_mut(mid);

        let left = Self::new(arena, left, offset);
        let right = Self::new(arena, right, offset + mid);

        arena.alloc(BVHNode {
            height: usize::max(left.height, right.height) + 1,
            triangles: (offset, offset + triangles.len()),
            children: Some([left, right]),
            bb: bounding_box,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::TAU;

    use glam::{vec2, Vec2};
    use rand::{Rng, SeedableRng};
    use svg::{
        node::element::{path::Data, Group, Path, Rectangle},
        Document, Node,
    };
    use typed_arena::Arena;

    use crate::{BVHNode, Triangle};

    #[test]
    fn test_bvh() {
        let arena = Arena::new();

        let min = 100.0;
        let max = 900.0;
        let min_r = 5.0;
        let max_r = 20.0;

        let mut rng = rand::rngs::StdRng::from_seed([3; 32]);

        let mut triangles = vec![];

        for _ in 0..500 {
            let center_x = rng.gen_range(min..max);
            let center_y = rng.gen_range(min..max);
            let l_a = rng.gen_range(0.0..TAU);
            let l_r = rng.gen_range(min_r..max_r);
            let m_a = rng.gen_range(0.0..TAU);
            let m_r = rng.gen_range(min_r..max_r);
            let n_a = rng.gen_range(0.0..TAU);
            let n_r = rng.gen_range(min_r..max_r);
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

        let mut doc = Document::new().set("viewBox", (0, 0, 1000, 1000));

        let bvh = BVHNode::new(&arena, &mut triangles, 0);

        doc = doc
            .set("fill", "none")
            .set("stroke-width", 2)
            .add(bvh_to_svg(&triangles, bvh));

        svg::save("test.svg", &doc).unwrap();
    }

    fn bvh_to_svg<'a>(triangles: &[Triangle<Vec2>], bvh: &'a BVHNode<'a, Vec2>) -> Box<dyn Node> {
        let mut group = Group::new();
        let size = bvh.bb.size();

        let hue_factor = TAU / (triangles.len() as f32);

        group = group.add(
            Rectangle::new()
                .set("x", bvh.bb.min.x as i32)
                .set("y", bvh.bb.min.y as i32)
                .set("width", size.x as i32)
                .set("height", size.y as i32)
                .set(
                    "stroke",
                    colour(
                        (bvh.triangles.0 as f32) * hue_factor,
                        0.5 + (bvh.height as f32 + 3.0).recip(),
                    ),
                ),
        );

        if let Some([left, right]) = bvh.children {
            group = group
                .add(bvh_to_svg(triangles, left))
                .add(bvh_to_svg(triangles, right));
        } else {
            for (i, t) in triangles[bvh.triangles.0..bvh.triangles.1]
                .iter()
                .enumerate()
            {
                let [l, m, n] = t.0;
                let data = Data::new()
                    .move_to((l.x as i32, l.y as i32))
                    .line_to((m.x as i32, m.y as i32))
                    .line_to((n.x as i32, n.y as i32))
                    .close();

                let path = Path::new()
                    .set(
                        "stroke",
                        colour(
                            (bvh.triangles.0 + i) as f32 * hue_factor,
                            0.5 + (bvh.height as f32 + 3.0).recip(),
                        ),
                    )
                    .set("d", data);

                group = group.add(path);
            }
        }

        Box::new(group)
    }

    fn colour(hue: f32, lightness: f32) -> String {
        let chroma = 0.5f32;
        let lab = Lab {
            a: chroma * hue.cos(),
            b: chroma * hue.sin(),
            l: lightness,
        };
        let [r, g, b] = oklab_to_linear_srgb(lab);
        let [r, g, b] = [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8];
        format!("rgb({r},{g},{b})")
    }

    struct Lab {
        a: f32,
        b: f32,
        l: f32,
    }

    fn oklab_to_linear_srgb(c: Lab) -> [f32; 3] {
        let l_ = c.l + 0.396_337_78 * c.a + 0.215_803_76 * c.b;
        let m_ = c.l - 0.105_561_346 * c.a - 0.063_854_17 * c.b;
        let s_ = c.l - 0.089_484_18 * c.a - 1.291_485_5 * c.b;

        let l = l_ * l_ * l_;
        let m = m_ * m_ * m_;
        let s = s_ * s_ * s_;

        [
            4.076_741_7 * l - 3.307_711_6 * m + 0.230_969_94 * s,
            -1.268_438 * l + 2.609_757_4 * m - 0.341_319_38 * s,
            -0.0041960863 * l - 0.703_418_6 * m + 1.707_614_7 * s,
        ]
    }
}
