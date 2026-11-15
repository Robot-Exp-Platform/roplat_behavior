//! Distances between the scene's shapes, never more than the true distance.
//!
//! The whole-motion check advances by the distances it measures, so a
//! distance measured too large lets a motion step into an obstacle by as
//! much (see [`motion`](crate::motion)), and
//! [`clearance`](crate::PlanningScene::clearance) would report room that is
//! not there. parry3d-f64 0.25 measures some pairs too large:
//!
//! - two cuboids by `distance_cuboid_cuboid`, a routine made for contacts,
//!   which can report several times the distance when faces are nearly
//!   parallel (2.3 cm for a box link 1.1 mm from a thin plate);
//! - other convex pairs by GJK, which returns the closest points of the
//!   simplex it has when it stops. It stops early on curved shapes: its
//!   simplex refuses a new support point when the area that point spans
//!   with the simplex's edge falls under an absolute threshold
//!   (`|ab × ac|² < 2.2e-15` m⁴), which a short chord of a narrow capsule or
//!   cylinder reaches while still bulging from the surface by up to 3e-5 m.
//!
//! [`distance`] therefore measures every pair of convex parts itself:
//!
//! - A ball is its centre and a capsule its segment, each dilated by its
//!   radius; the distance is that of the cores less both radii. Points and
//!   segments are polytopes, on which GJK converges exactly.
//! - The cores (or a cuboid, a cylinder, a convex polyhedron as they are)
//!   are measured by GJK, written here on parry's support maps, which keeps
//!   the support gap it finds on the way: for any unit direction `n`, no
//!   point of the second shape lies closer to the first than
//!   `min n·b − max n·a` (Cauchy–Schwarz), so the largest such gap is a
//!   lower bound of the distance however early GJK stops. That bound is the
//!   distance reported. The simplex keeps every support point that can
//!   still shorten the distance, and GJK stops when the bound is within
//!   [`GAP`] of the simplex's distance, or after [`ITERATIONS`].
//! - Compounds (sensed voxels, decomposed meshes) are searched part by part
//!   through parry's bounding-volume tree with these distances; its pruning
//!   by bounding boxes keeps the result a lower bound.
//!
//! Only rounding can make a distance larger than the truth, at the scale of
//! 1e-16 of the shapes' coordinates. A distance can come out smaller than
//! the truth, by at most [`GAP`] when GJK converges; the tests report how
//! much it does over every pair of shapes the scene builds.

use nalgebra as na;
use parry3d_f64::{
    query::{
        ClosestPoints, Contact, DefaultQueryDispatcher, NonlinearRigidMotion, QueryDispatcher,
        ShapeCastHit, ShapeCastOptions, Unsupported,
        details::{distance_composite_shape_shape, distance_shape_composite_shape},
        gjk::ConstantOrigin,
    },
    shape::{Shape, SupportMap},
};

/// GJK stops once its lower bound is this close (m) to the distance of its
/// simplex, which is never below the true distance.
const GAP: f64 = 1e-10;

/// GJK stops after this many support points; the bound it has is reported.
const ITERATIONS: usize = 100;

/// Distance (m) between `a` posed at `pa` and `b` at `pb`, zero when they
/// overlap; never larger than the true distance but by rounding, see the
/// module docs. Every shape the scene builds is convex or a compound of
/// convex parts; were a pair unsupported, it would count as touching.
pub(crate) fn distance(
    pa: &na::Isometry3<f64>,
    a: &dyn Shape,
    pb: &na::Isometry3<f64>,
    b: &dyn Shape,
) -> f64 {
    SceneQueries.distance(&pa.inv_mul(pb), a, b).unwrap_or(0.0)
}

/// parry's default queries, except the distance, which is the lower bound
/// of [`separation`] for convex pairs, at any depth of compound.
struct SceneQueries;

impl QueryDispatcher for SceneQueries {
    fn intersection_test(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<bool, Unsupported> {
        DefaultQueryDispatcher.intersection_test(pos12, g1, g2)
    }

    fn distance(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<f64, Unsupported> {
        if let Some(c1) = g1.as_composite_shape() {
            // The default dispatcher would measure the parts with itself.
            Ok(distance_composite_shape_shape(self, pos12, c1, g2))
        } else if let Some(c2) = g2.as_composite_shape() {
            Ok(distance_shape_composite_shape(self, pos12, g1, c2))
        } else {
            let (Some(a), Some(b)) = (Rounded::of(g1), Rounded::of(g2)) else {
                return Err(Unsupported);
            };
            Ok((separation(pos12, a.core, b.core) - a.radius - b.radius).max(0.0))
        }
    }

    fn contact(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: f64,
    ) -> Result<Option<Contact>, Unsupported> {
        DefaultQueryDispatcher.contact(pos12, g1, g2, prediction)
    }

    fn closest_points(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
        max_dist: f64,
    ) -> Result<ClosestPoints, Unsupported> {
        DefaultQueryDispatcher.closest_points(pos12, g1, g2, max_dist)
    }

    fn cast_shapes(
        &self,
        pos12: &na::Isometry3<f64>,
        local_vel12: &na::Vector3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        DefaultQueryDispatcher.cast_shapes(pos12, local_vel12, g1, g2, options)
    }

    fn cast_shapes_nonlinear(
        &self,
        motion1: &NonlinearRigidMotion,
        g1: &dyn Shape,
        motion2: &NonlinearRigidMotion,
        g2: &dyn Shape,
        start_time: f64,
        end_time: f64,
        stop_at_penetration: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        DefaultQueryDispatcher.cast_shapes_nonlinear(
            motion1,
            g1,
            motion2,
            g2,
            start_time,
            end_time,
            stop_at_penetration,
        )
    }
}

/// A convex shape as the points within `radius` of its core: a ball's
/// centre, a capsule's segment, or any other convex shape itself.
struct Rounded<'a> {
    // dyn(c): the cores are parry's shapes of different types.
    core: &'a dyn SupportMap,
    radius: f64,
}

impl<'a> Rounded<'a> {
    fn of(shape: &'a dyn Shape) -> Option<Self> {
        if let Some(ball) = shape.as_ball() {
            Some(Self { core: &ConstantOrigin, radius: ball.radius })
        } else if let Some(capsule) = shape.as_capsule() {
            Some(Self { core: &capsule.segment, radius: capsule.radius })
        } else {
            shape
                .as_support_map()
                .map(|core| Self { core, radius: 0.0 })
        }
    }
}

/// A lower bound (m) of the distance between convex `g1` and `g2` posed at
/// `pos12` in `g1`'s frame, zero or less when they overlap: the largest
/// support gap GJK meets.
///
/// GJK (Gilbert, Johnson and Keerthi, 1988) works on the Minkowski
/// difference `g1 − g2`, whose nearest point to the origin is the
/// separation. Its simplex of support points has a nearest point `p` no
/// nearer than the difference's; the support point `s` toward the origin,
/// along `n = −p / |p|`, bounds the difference by the plane `n·x = n·s`, at
/// `−n·s` from the origin. `s` joins the simplex, which keeps only the
/// face its new nearest point lies on.
fn separation(pos12: &na::Isometry3<f64>, g1: &dyn SupportMap, g2: &dyn SupportMap) -> f64 {
    let support = |n: &na::Vector3<f64>| -> na::Vector3<f64> {
        g1.local_support_point(n).coords - g2.support_point(pos12, &-n).coords
    };
    let start = if pos12.translation.vector.norm_squared() > 0.0 {
        -pos12.translation.vector
    } else {
        na::Vector3::x()
    };
    let mut simplex = Simplex::new(support(&start));
    let mut lower = f64::NEG_INFINITY;
    let mut upper = f64::INFINITY;
    for _ in 0..ITERATIONS {
        // The origin inside the simplex: the shapes overlap.
        let Some(nearest) = simplex.reduce() else {
            break;
        };
        let distance = nearest.norm();
        // No progress is left but rounding; or the shapes touch.
        if distance >= upper || distance <= TOUCHING {
            break;
        }
        upper = distance;
        let n = -nearest / distance;
        let s = support(&n);
        lower = lower.max(-n.dot(&s));
        if upper - lower <= GAP {
            break;
        }
        simplex.push(s);
    }
    lower
}

/// A simplex nearer than this (m) to the origin touches it.
const TOUCHING: f64 = 1e-14;

/// Up to four points of the Minkowski difference.
struct Simplex {
    points: [na::Vector3<f64>; 4],
    len: usize,
}

impl Simplex {
    fn new(point: na::Vector3<f64>) -> Self {
        Self { points: [point; 4], len: 1 }
    }

    /// Called only after [`reduce`](Self::reduce), which leaves at most
    /// three points.
    fn push(&mut self, point: na::Vector3<f64>) {
        self.points[self.len] = point;
        self.len += 1;
    }

    /// The point of the simplex's hull nearest the origin, keeping only the
    /// vertices of the face, edge or vertex it lies on; `None` when the hull
    /// holds the origin.
    fn reduce(&mut self) -> Option<na::Vector3<f64>> {
        let p = self.points;
        let (nearest, keep) = match self.len {
            1 => (p[0], Keep::of(&[0])),
            2 => segment(&p, 0, 1),
            3 => triangle(&p, [0, 1, 2]),
            _ => tetrahedron(&p)?,
        };
        for (k, &i) in keep.indices[..keep.len].iter().enumerate() {
            self.points[k] = p[i];
        }
        self.len = keep.len;
        Some(nearest)
    }
}

/// Indices of the simplex's points to keep.
#[derive(Clone, Copy)]
struct Keep {
    indices: [usize; 3],
    len: usize,
}

impl Keep {
    fn of(indices: &[usize]) -> Self {
        let mut keep = Self { indices: [0; 3], len: indices.len() };
        keep.indices[..indices.len()].copy_from_slice(indices);
        keep
    }
}

/// Nearest point to the origin on the segment `p[i] p[j]`.
fn segment(p: &[na::Vector3<f64>; 4], i: usize, j: usize) -> (na::Vector3<f64>, Keep) {
    let (a, b) = (p[i], p[j]);
    let ab = b - a;
    let length = ab.norm_squared();
    let t = if length > 0.0 {
        -a.dot(&ab) / length
    } else {
        0.0
    };
    if t <= 0.0 {
        (a, Keep::of(&[i]))
    } else if t >= 1.0 {
        (b, Keep::of(&[j]))
    } else {
        (a + ab * t, Keep::of(&[i, j]))
    }
}

/// Nearest point to the origin on the triangle `p[t[0]] p[t[1]] p[t[2]]`,
/// by its Voronoi regions (Ericson, Real-Time Collision Detection, 2005,
/// section 5.1.5). A triangle too flat to have a face region is measured
/// by its edges.
fn triangle(p: &[na::Vector3<f64>; 4], t: [usize; 3]) -> (na::Vector3<f64>, Keep) {
    let [i, j, k] = t;
    let (a, b, c) = (p[i], p[j], p[k]);
    let (ab, ac) = (b - a, c - a);
    let (d1, d2) = (-ab.dot(&a), -ac.dot(&a));
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, Keep::of(&[i]));
    }
    let (d3, d4) = (-ab.dot(&b), -ac.dot(&b));
    if d3 >= 0.0 && d4 <= d3 {
        return (b, Keep::of(&[j]));
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return (a + ab * fraction(d1, d1 - d3), Keep::of(&[i, j]));
    }
    let (d5, d6) = (-ab.dot(&c), -ac.dot(&c));
    if d6 >= 0.0 && d5 <= d6 {
        return (c, Keep::of(&[k]));
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return (a + ac * fraction(d2, d2 - d6), Keep::of(&[i, k]));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let w = fraction(d4 - d3, (d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, Keep::of(&[j, k]));
    }
    let sum = va + vb + vc;
    if sum > 0.0 {
        let (v, w) = (vb / sum, vc / sum);
        return (a + ab * v + ac * w, Keep::of(&[i, j, k]));
    }
    [segment(p, i, j), segment(p, i, k), segment(p, j, k)]
        .into_iter()
        .min_by(|x, y| x.0.norm_squared().total_cmp(&y.0.norm_squared()))
        .expect("three edges")
}

/// Nearest point to the origin in the tetrahedron of the four points, or
/// `None` when it holds the origin (Ericson, section 5.1.6): the nearest
/// over the faces the origin lies outside of. A tetrahedron too flat to
/// tell sides is measured by all its faces.
fn tetrahedron(p: &[na::Vector3<f64>; 4]) -> Option<(na::Vector3<f64>, Keep)> {
    const FACES: [([usize; 3], usize); 4] = [
        ([0, 1, 2], 3),
        ([0, 1, 3], 2),
        ([0, 2, 3], 1),
        ([1, 2, 3], 0),
    ];
    let mut nearest: Option<(na::Vector3<f64>, Keep)> = None;
    for (face, opposite) in FACES {
        let [i, j, k] = face;
        let normal = (p[j] - p[i]).cross(&(p[k] - p[i]));
        let towards = (p[opposite] - p[i]).dot(&normal);
        let origin = -p[i].dot(&normal);
        let flat = towards.abs() <= 1e-12 * normal.norm() * (p[opposite] - p[i]).norm();
        if flat || origin * towards < 0.0 {
            let candidate = triangle(p, face);
            if nearest
                .as_ref()
                .is_none_or(|n| candidate.0.norm_squared() < n.0.norm_squared())
            {
                nearest = Some(candidate);
            }
        }
    }
    nearest
}

/// `x / y` for the parameter of a point on an edge; 0 on an edge of no
/// length, where `y` vanishes.
fn fraction(x: f64, y: f64) -> f64 {
    if y > 0.0 { x / y } else { 0.0 }
}
