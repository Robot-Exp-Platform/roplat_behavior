//! The distances the scene reports agree with independent measurements.
//!
//! parry3d-f64 0.25's own routine for two cuboids overestimates their
//! distance (see `src/distance.rs`). Here every pair of shapes the scene
//! builds, a link's against an object's, is measured through
//! [`PlanningScene::clearance`] and compared with:
//!
//! - closed forms written in this file, sharing no code with parry, for
//!   boxes, meshes (the hull of a box), spheres, capsules and sensed voxels;
//! - for a cylinder, which has no closed form against most shapes, the
//!   prisms of [`SIDES`] sides inscribed in it and circumscribed about it,
//!   measured by parry: the cylinder's distance lies between theirs.
//!
//! The same pairs measured by parry's default `query::distance` and by
//! `query::contact` are printed beside them.

use std::{
    f64::consts::{FRAC_PI_2, PI, TAU},
    path::PathBuf,
    sync::OnceLock,
};

use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use parry3d_f64::{
    query::{self, details::distance_support_map_support_map},
    shape::{Cuboid, SharedShape},
};
use roplat_description::{Geometry, RobotModel};
use roplat_scene::PlanningScene;

/// Sides of the prisms that bound a cylinder: they differ from it by less
/// than 3.1e-4 of its radius.
const SIDES: usize = 128;

/// fix14b's planar arm with box links: two 0.6 m links 4 mm square,
/// turning about z.
const BOX_ARM: &str = r#"<robot name="planar">
  <link name="base"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="l1"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.004 0.004"/></geometry></collision></link>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0.6 0 0"/><axis xyz="0 0 1"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
  <link name="l2"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.004 0.004"/></geometry></collision></link>
</robot>"#;

/// The box arm and a plate 2 mm thick across the outer link's sweep: y
/// from 0.010 to 0.012, x from 1.05 to 1.15 m, z within 2.5 cm.
fn box_arm() -> PlanningScene<2> {
    let model = RobotModel::from_urdf_str(BOX_ARM).unwrap();
    let mut scene = PlanningScene::new(model, ["j1", "j2"], |name: &str| PathBuf::from(name)).unwrap();
    scene
        .add_object(
            "plate",
            &Geometry::Box { size: [0.1, 0.002, 0.05] },
            Isometry3::translation(1.1, 0.011, 0.0),
        )
        .unwrap();
    scene
}

/// The distances fix14b found wrong: 1.1 mm reported as 2.30 cm, 8.0 mm as
/// 5.06 cm.
#[test]
fn a_box_link_near_a_thin_plate_is_measured_as_it_is() {
    let scene = box_arm();
    let plate = Solid::cuboid(
        Isometry3::translation(1.1, 0.011, 0.0),
        [0.05, 0.001, 0.025],
    );
    for theta in [0.0_f64, 0.006] {
        // The outer link's upper face lies 2 mm above its axis; the plate's
        // lower edge at x = 1.15 m, y = 0.010 m is the nearest to it.
        let gap = 0.010 * theta.cos() - 1.15 * theta.sin() - 0.002;
        let pose = Isometry3::rotation(Vector3::z() * theta) * Isometry3::translation(0.9, 0.0, 0.0);
        let link = Solid::cuboid(pose, [0.3, 0.002, 0.002]);
        assert!((box_box(&link, &plate) - gap).abs() < 1e-12);
        let nearest = scene.clearance(&[theta, 0.0]).unwrap();
        assert_eq!(
            (nearest.first.as_str(), nearest.second.as_str()),
            ("l2", "plate")
        );
        let (by_parry, _) = parry_default(&link, &plate);
        println!(
            "SCENE14 box link at {theta} rad: {gap:.6} m from the plate, \
             the scene {:.6} m, parry's query::distance {by_parry:.6} m",
            nearest.distance
        );
        assert!(
            (nearest.distance - gap).abs() < 1e-9,
            "at {theta} rad: {nearest:?}, not {gap}"
        );
    }
}

/// Random box pairs, turned uniformly, nearly aligned or about a shared
/// axis, and from overlapping through nearly touching to far
/// apart, of sizes from 2 mm to 0.6 m, measured by the scene, by GJK on
/// the two cuboids, by GJK on their convex hulls and in closed form.
#[test]
fn box_pairs_agree_with_gjk_hulls_and_a_closed_form() {
    let mut random = Random(0x5ce1_e14b_0b0c_0001);
    let mut worst = [0.0_f64; 4];
    let (mut overlapping, mut near) = (0, 0);
    const PAIRS: usize = 2000;
    for k in 0..PAIRS {
        let a = Solid::random(Kind::Box, &mut random).posed(&mut random);
        let b = Solid::random(Kind::Box, &mut random);
        let b = place(&a, b, k % 2 == 1, Turn::ALL[k / 2 % 3], &mut random);
        let exact = box_box(&a, &b);
        let scene = scene_distance(&a, &b);
        let pos12 = a.pose.inv_mul(&b.pose);
        let gjk = distance_support_map_support_map(
            &pos12,
            &Cuboid::new(a.half),
            &Cuboid::new(b.half),
        );
        let hull = |s: &Solid| SharedShape::convex_hull(&corners(&s.half)).unwrap();
        let by_hull = query::distance(&a.pose, &*hull(&a), &b.pose, &*hull(&b)).unwrap();
        let (by_parry, _) = parry_default(&a, &b);
        for (worst, value) in worst.iter_mut().zip([gjk, by_hull, exact, by_parry]) {
            *worst = worst.max((scene - value).abs());
        }
        overlapping += usize::from(exact == 0.0);
        near += usize::from(exact > 0.0 && exact < 1e-3);
        assert!(
            (scene - exact).abs() <= tolerance(exact),
            "pair {k}: the scene {scene}, in closed form {exact}, GJK {gjk}, hulls {by_hull}"
        );
    }
    println!(
        "SCENE14 {PAIRS} box pairs ({overlapping} overlapping, {near} nearer than 1 mm): \
         the scene against GJK {:.1e} m at most, against hulls {:.1e}, against the closed form \
         {:.1e}; parry's query::distance off by up to {:.4} m",
        worst[0], worst[1], worst[2], worst[3]
    );
}

/// Every link shape against every object shape, as in the module docs.
#[test]
fn every_pair_of_shapes_agrees_with_an_independent_measure() {
    let mut random = Random(0x5ce1_e14b_5a1f_0002);
    const SAMPLES: usize = 160;
    let mut failures = Vec::new();
    for link in LINKS {
        for object in OBJECTS {
            let mut stats = Stats::default();
            for k in 0..SAMPLES {
                let a = Solid::random(link, &mut random).posed(&mut random);
                let b = Solid::random(object, &mut random);
                let b = place(&a, b, k % 2 == 1, Turn::ALL[k / 2 % 3], &mut random);
                let truth = truth(&a, &b);
                let scene = scene_distance(&a, &b);
                let (by_parry, by_contact) = parry_default(&a, &b);
                stats.add(truth, scene, by_parry, by_contact);
                if !truth.admits(scene, round(&a) || round(&b)) {
                    failures.push(format!("{link:?}-{object:?} {k}: {scene} for {truth:?}"));
                }
            }
            println!("SCENE14 survey {link:?}-{object:?}: {stats}");
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// What a distance should be, from closed forms or from bounds.
#[derive(Clone, Copy, Debug)]
enum Truth {
    Exact(f64),
    Between(f64, f64),
}

impl Truth {
    fn min(self, other: Self) -> Self {
        let (l1, u1) = self.bounds();
        let (l2, u2) = other.bounds();
        match (self, other) {
            (Self::Exact(_), Self::Exact(_)) => Self::Exact(l1.min(l2)),
            _ => Self::Between(l1.min(l2), u1.min(u2)),
        }
    }

    fn bounds(self) -> (f64, f64) {
        match self {
            Self::Exact(d) => (d, d),
            Self::Between(lower, upper) => (lower, upper),
        }
    }

    fn middle(self) -> f64 {
        let (lower, upper) = self.bounds();
        (lower + upper) / 2.0
    }

    /// Whether `d` is the distance, to within rounding, or, with a capsule
    /// or a cylinder (`round`), to within [`ROUND`].
    fn admits(self, d: f64, round: bool) -> bool {
        let (lower, upper) = self.bounds();
        let slack = |d: f64| if round { ROUND } else { tolerance(d) };
        d >= lower - slack(lower) && d <= upper + slack(upper)
    }
}

/// Rounding in the closed forms and in GJK between polyhedra.
fn tolerance(d: f64) -> f64 {
    1e-9 + 1e-7 * d
}

/// How far GJK may miss a distance to a capsule or a cylinder (m): its
/// support points on a curved surface converge slowly, and it stops early
/// with the points it has. Half of `MOTION_TOLERANCE`, which absorbs it.
const ROUND: f64 = 5e-5;

fn round(s: &Solid) -> bool {
    matches!(s.kind, Kind::Capsule | Kind::Cylinder)
}

/// Largest errors over and under the truth, of the scene and of parry's
/// default distance and contact, each with the truth's upper bound there.
#[derive(Default)]
struct Stats {
    samples: usize,
    overlapping: usize,
    near: usize,
    errors: [((f64, f64), (f64, f64)); 3],
}

impl Stats {
    fn add(&mut self, truth: Truth, scene: f64, by_parry: f64, by_contact: f64) {
        let (lower, upper) = truth.bounds();
        self.samples += 1;
        self.overlapping += usize::from(upper == 0.0);
        self.near += usize::from(lower > 0.0 && upper < 1e-3);
        for ((over, under), d) in self.errors.iter_mut().zip([scene, by_parry, by_contact]) {
            if d - upper > over.0 {
                *over = (d - upper, upper);
            }
            if lower - d > under.0 {
                *under = (lower - d, upper);
            }
        }
    }
}

impl std::fmt::Display for Stats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} pairs, {} overlapping, {} nearer than 1 mm; most over / under the truth (m, \
             at a distance of):",
            self.samples, self.overlapping, self.near,
        )?;
        for (name, (over, under)) in ["scene", "parry distance", "parry contact"]
            .iter()
            .zip(self.errors)
        {
            write!(
                f,
                " {name} {:.1e} (at {:.1e}) / {:.1e} (at {:.1e});",
                over.0, over.1, under.0, under.1
            )?;
        }
        Ok(())
    }
}

/// How `place` turns the second shape against the first.
#[derive(Clone, Copy, Debug)]
enum Turn {
    /// Uniformly.
    Any,
    /// Its axes along the first's, in any order and sense, then turned by
    /// up to 0.1 rad or not at all: faces nearly parallel, where parry's
    /// own cuboid routine goes wrong.
    Aligned,
    /// About one of the first's axes, by any angle, as the links of a
    /// planar arm turn against each other.
    SharedAxis,
}

impl Turn {
    const ALL: [Turn; 3] = [Turn::Any, Turn::Aligned, Turn::SharedAxis];

    fn rotation(self, a: &UnitQuaternion<f64>, random: &mut Random) -> UnitQuaternion<f64> {
        let quarter = |random: &mut Random| {
            let axis = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()]
                [(random.unit() * 3.0) as usize];
            UnitQuaternion::from_axis_angle(&axis, FRAC_PI_2 * (random.unit() * 4.0).floor())
        };
        match self {
            Turn::Any => random.rotation(),
            Turn::Aligned => {
                let order = quarter(&mut *random) * quarter(&mut *random) * quarter(&mut *random);
                let tilt = if random.unit() < 0.25 {
                    0.0
                } else {
                    10f64.powf(-9.0 + 8.0 * random.unit())
                };
                let axis = nalgebra::Unit::new_normalize(random.rotation() * Vector3::x());
                a * order * UnitQuaternion::from_axis_angle(&axis, tilt)
            }
            Turn::SharedAxis => {
                let axis = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()]
                    [(random.unit() * 3.0) as usize];
                let turn = UnitQuaternion::from_axis_angle(&axis, TAU * random.unit());
                a * turn * quarter(&mut *random)
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Box,
    /// The convex hull of a mesh: here of a box's corners.
    Mesh,
    Sphere,
    Capsule,
    Cylinder,
    /// Sensed points, as cubes.
    Voxels,
}

const LINKS: [Kind; 5] = [Kind::Box, Kind::Mesh, Kind::Sphere, Kind::Capsule, Kind::Cylinder];
const OBJECTS: [Kind; 6] = [
    Kind::Box,
    Kind::Mesh,
    Kind::Sphere,
    Kind::Capsule,
    Kind::Cylinder,
    Kind::Voxels,
];

/// A shape placed in the world.
#[derive(Clone)]
struct Solid {
    kind: Kind,
    pose: Isometry3<f64>,
    /// Box and mesh: half extents. Sphere: the radius, in x. Capsule and
    /// cylinder: the radius in x, half the length of the axis in z. Voxels:
    /// half the edge, in x.
    half: Vector3<f64>,
    /// Voxels: the occupied cells.
    cells: Vec<[i32; 3]>,
    /// Cylinder: the inscribed and the circumscribed prism, along z.
    prisms: Option<(SharedShape, SharedShape)>,
}

impl Solid {
    fn cuboid(pose: Isometry3<f64>, half: [f64; 3]) -> Self {
        Self { kind: Kind::Box, pose, half: half.into(), cells: Vec::new(), prisms: None }
    }

    /// Sizes log-uniform, so thin plates and long rods come as often as
    /// cubes; posed at the origin.
    fn random(kind: Kind, random: &mut Random) -> Self {
        let mut cells = Vec::new();
        let half = match kind {
            Kind::Box | Kind::Mesh => {
                Vector3::from_fn(|_, _| random.size(1e-3, 0.3))
            }
            Kind::Sphere => Vector3::new(random.size(2e-3, 0.2), 0.0, 0.0),
            Kind::Capsule | Kind::Cylinder => {
                Vector3::new(random.size(1e-3, 0.1), 0.0, random.size(2e-3, 0.3))
            }
            Kind::Voxels => {
                // The cell at the origin and up to five of its neighbours.
                cells.push([0, 0, 0]);
                for _ in 0..(random.unit() * 6.0) as usize {
                    let cell = [0; 3].map(|_| (random.unit() * 3.0) as i32 - 1);
                    if !cells.contains(&cell) {
                        cells.push(cell);
                    }
                }
                Vector3::new(random.size(2.5e-3, 0.025), 0.0, 0.0)
            }
        };
        let prisms = (kind == Kind::Cylinder).then(|| (prism(&half, false), prism(&half, true)));
        Self { kind, pose: Isometry3::identity(), half, cells, prisms }
    }

    /// Turned at random and moved within half a metre of the origin.
    fn posed(mut self, random: &mut Random) -> Self {
        let at = Vector3::from_fn(|_, _| random.unit() - 0.5);
        self.pose = Isometry3::from_parts(Translation3::from(at), random.rotation());
        self
    }

    /// Farthest a point of the solid lies from its origin.
    fn extent(&self) -> f64 {
        match self.kind {
            Kind::Box | Kind::Mesh => self.half.norm(),
            Kind::Sphere => self.half.x,
            Kind::Capsule => self.half.x + self.half.z,
            Kind::Cylinder => self.half.x.hypot(self.half.z),
            Kind::Voxels => self
                .cells
                .iter()
                .map(|&cell| self.cell_centre(cell).norm() + self.half.x * 3f64.sqrt())
                .fold(0.0, f64::max),
        }
    }

    fn edge(&self) -> f64 {
        2.0 * self.half.x
    }

    fn cell_centre(&self, cell: [i32; 3]) -> Vector3<f64> {
        Vector3::from_fn(|i, _| (cell[i] as f64 + 0.5) * self.edge())
    }

    /// Voxels: each cube, as a box in the world.
    fn cubes(&self) -> Vec<Solid> {
        self.cells
            .iter()
            .map(|&cell| {
                let pose = self.pose * Translation3::from(self.cell_centre(cell));
                Solid::cuboid(pose, [self.half.x; 3])
            })
            .collect()
    }

    fn centre(&self) -> Point3<f64> {
        self.pose.translation.vector.into()
    }

    /// Capsule and cylinder: the ends of the axis.
    fn axis(&self) -> (Point3<f64>, Point3<f64>) {
        let end = |z: f64| self.pose * Point3::new(0.0, 0.0, z * self.half.z);
        (end(-1.0), end(1.0))
    }

    /// The geometry as a URDF link writes it.
    fn urdf(&self) -> String {
        let h = self.half;
        match self.kind {
            Kind::Box => format!(r#"<box size="{} {} {}"/>"#, 2.0 * h.x, 2.0 * h.y, 2.0 * h.z),
            Kind::Mesh => format!(
                r#"<mesh filename="{}" scale="{} {} {}"/>"#,
                unit_cube().display(),
                2.0 * h.x,
                2.0 * h.y,
                2.0 * h.z
            ),
            Kind::Sphere => format!(r#"<sphere radius="{}"/>"#, h.x),
            Kind::Capsule => format!(r#"<capsule radius="{}" length="{}"/>"#, h.x, 2.0 * h.z),
            Kind::Cylinder => format!(r#"<cylinder radius="{}" length="{}"/>"#, h.x, 2.0 * h.z),
            Kind::Voxels => unreachable!("voxels are sensed objects, not links"),
        }
    }

    fn geometry(&self) -> Geometry {
        let h = self.half;
        match self.kind {
            Kind::Box => Geometry::Box { size: (2.0 * h).into() },
            Kind::Mesh => Geometry::Mesh {
                filename: unit_cube().to_string_lossy().into_owned(),
                scale: (2.0 * h).into(),
            },
            Kind::Sphere => Geometry::Sphere { radius: h.x },
            Kind::Capsule => Geometry::Capsule { radius: h.x, length: 2.0 * h.z },
            Kind::Cylinder => Geometry::Cylinder { radius: h.x, length: 2.0 * h.z },
            Kind::Voxels => unreachable!("voxels are added as points"),
        }
    }

    /// The shape as the scene builds it, and its pose.
    fn parry(&self) -> (Isometry3<f64>, SharedShape) {
        let h = self.half;
        match self.kind {
            Kind::Box => (self.pose, SharedShape::cuboid(h.x, h.y, h.z)),
            Kind::Mesh => (self.pose, SharedShape::convex_hull(&corners(&h)).unwrap()),
            Kind::Sphere => (self.pose, SharedShape::ball(h.x)),
            Kind::Capsule => (self.pose, SharedShape::capsule_z(h.z, h.x)),
            // parry's cylinder runs along y.
            Kind::Cylinder => (
                self.pose * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2),
                SharedShape::cylinder(h.z, h.x),
            ),
            Kind::Voxels => {
                let cubes = self
                    .cells
                    .iter()
                    .map(|&cell| {
                        let at = Isometry3::from_parts(
                            Translation3::from(self.cell_centre(cell)),
                            UnitQuaternion::identity(),
                        );
                        (at, SharedShape::cuboid(h.x, h.x, h.x))
                    })
                    .collect();
                (self.pose, SharedShape::compound(cubes))
            }
        }
    }
}

/// A box of edge 1 m about the origin, as an OBJ file of its corners; a
/// mesh's scale sizes it.
fn unit_cube() -> &'static PathBuf {
    static PATH: OnceLock<PathBuf> = OnceLock::new();
    PATH.get_or_init(|| {
        let text: String = corners(&Vector3::repeat(0.5))
            .iter()
            .map(|p| format!("v {} {} {}\n", p.x, p.y, p.z))
            .collect();
        let path = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("roplat_scene_unit_cube.obj");
        std::fs::write(&path, text).unwrap();
        path
    })
}

fn corners(half: &Vector3<f64>) -> Vec<Point3<f64>> {
    (0..8)
        .map(|i| {
            Point3::from(Vector3::from_fn(|k, _| {
                if (i >> k) & 1 == 1 { half[k] } else { -half[k] }
            }))
        })
        .collect()
}

/// The prism of [`SIDES`] sides inscribed in a cylinder along z, or
/// circumscribed about it when `outside`.
fn prism(half: &Vector3<f64>, outside: bool) -> SharedShape {
    let radius = if outside {
        half.x / (PI / SIDES as f64).cos()
    } else {
        half.x
    };
    let points: Vec<Point3<f64>> = (0..SIDES)
        .flat_map(|i| {
            let angle = TAU * i as f64 / SIDES as f64;
            [-half.z, half.z].map(|z| Point3::new(radius * angle.cos(), radius * angle.sin(), z))
        })
        .collect();
    SharedShape::convex_hull(&points).unwrap()
}

/// The scene's distance from a link shaped `a` to an object `b`, zero when
/// they overlap.
fn scene_distance(a: &Solid, b: &Solid) -> f64 {
    let urdf = format!(
        r#"<robot name="one"><link name="a"><collision><geometry>{}</geometry></collision></link></robot>"#,
        a.urdf()
    );
    let model = RobotModel::from_urdf_str(&urdf).unwrap();
    let mut scene = PlanningScene::new(model, [], |name: &str| PathBuf::from(name)).unwrap();
    scene.set_base(a.pose);
    if b.kind == Kind::Voxels {
        let points: Vec<[f64; 3]> = b.cells.iter().map(|&cell| b.cell_centre(cell).into()).collect();
        let voxels = scene.add_points("b", &points, b.edge(), b.pose).unwrap();
        assert_eq!(voxels, b.cells.len());
    } else {
        scene.add_object("b", &b.geometry(), b.pose).unwrap();
    }
    scene.clearance(&[]).unwrap().distance.max(0.0)
}

/// parry's default `query::distance`, and `query::contact`'s distance,
/// between the shapes the scene builds.
fn parry_default(a: &Solid, b: &Solid) -> (f64, f64) {
    let ((pa, sa), (pb, sb)) = (a.parry(), b.parry());
    let distance = query::distance(&pa, &*sa, &pb, &*sb).unwrap();
    let contact = query::contact(&pa, &*sa, &pb, &*sb, 10.0)
        .unwrap()
        .map_or(f64::INFINITY, |contact| contact.dist.max(0.0));
    (distance, contact)
}

/// `b` turned as `turn` says and moved from `a`'s centre along a random
/// direction: anywhere from overlapping to clear, or, when `tangent`,
/// within 1e-7 to 1e-2 of their extents of where they touch, either side.
fn place(a: &Solid, mut b: Solid, tangent: bool, turn: Turn, random: &mut Random) -> Solid {
    let rotation = turn.rotation(&a.pose.rotation, random);
    let direction = random.rotation() * Vector3::x();
    let span = a.extent() + b.extent();
    let at = |t: f64| {
        Isometry3::from_parts(
            Translation3::from(a.pose.translation.vector + direction * t),
            rotation,
        )
    };
    let t = if tangent {
        // The distance grows with t once they part, convex in t.
        let (mut inside, mut outside) = (0.0, 1.01 * span);
        for _ in 0..50 {
            let middle = (inside + outside) / 2.0;
            b.pose = at(middle);
            if truth(a, &b).middle() > 0.0 {
                outside = middle;
            } else {
                inside = middle;
            }
        }
        let offset = 10f64.powf(-7.0 + 5.0 * random.unit()) * span;
        if random.unit() < 0.5 {
            outside + offset
        } else {
            outside - offset
        }
    } else {
        1.2 * span * random.unit()
    };
    b.pose = at(t);
    b
}

/// The distance between `a` and `b`, zero when they overlap.
fn truth(a: &Solid, b: &Solid) -> Truth {
    use Kind::*;
    let d = match (a.kind, b.kind) {
        (_, Voxels) => {
            return b.cubes().iter().map(|cube| truth(a, cube)).reduce(Truth::min).unwrap();
        }
        (Voxels, _) => return truth(b, a),
        (Cylinder, Sphere) => point_cylinder(&b.centre(), a) - b.half.x,
        (Sphere, Cylinder) => return truth(b, a),
        (Cylinder, _) | (_, Cylinder) => return prism_bounds(a, b),
        (Box | Mesh, Box | Mesh) => box_box(a, b),
        (Box | Mesh, Sphere) => point_box(&b.centre(), a) - b.half.x,
        (Box | Mesh, Capsule) => {
            let (p, q) = b.axis();
            segment_box(&p, &q, a) - b.half.x
        }
        (Sphere | Capsule, Box | Mesh) => return truth(b, a),
        (Sphere, Sphere) => (a.centre() - b.centre()).norm() - a.half.x - b.half.x,
        (Sphere, Capsule) => {
            let (p, q) = b.axis();
            segment_segment(&a.centre(), &a.centre(), &p, &q) - a.half.x - b.half.x
        }
        (Capsule, Sphere) => return truth(b, a),
        (Capsule, Capsule) => {
            let ((p1, q1), (p2, q2)) = (a.axis(), b.axis());
            segment_segment(&p1, &q1, &p2, &q2) - a.half.x - b.half.x
        }
    };
    Truth::Exact(d.max(0.0))
}

/// Bounds on a distance with a cylinder: between the circumscribed prisms'
/// and the inscribed prisms' distances.
fn prism_bounds(a: &Solid, b: &Solid) -> Truth {
    let measure = |outside: bool| {
        let shape = |s: &Solid| match &s.prisms {
            Some((inscribed, circumscribed)) => {
                let prism = if outside { circumscribed } else { inscribed };
                (s.pose, prism.clone())
            }
            None => s.parry(),
        };
        let ((pa, sa), (pb, sb)) = (shape(a), shape(b));
        query::distance(&pa, &*sa, &pb, &*sb).unwrap()
    };
    Truth::Between(measure(true), measure(false))
}

fn axes(b: &Solid) -> [Vector3<f64>; 3] {
    [Vector3::x(), Vector3::y(), Vector3::z()].map(|axis| b.pose.rotation * axis)
}

fn box_corners(b: &Solid) -> Vec<Point3<f64>> {
    corners(&b.half).into_iter().map(|p| b.pose * p).collect()
}

/// The twelve edges: corners differing in one coordinate.
fn box_edges(b: &Solid) -> Vec<(Point3<f64>, Point3<f64>)> {
    let corners = box_corners(b);
    (0..8)
        .flat_map(|i| (0..3).map(move |k| (i, i ^ (1 << k))))
        .filter(|(i, j)| i < j)
        .map(|(i, j)| (corners[i], corners[j]))
        .collect()
}

fn point_box(p: &Point3<f64>, b: &Solid) -> f64 {
    let local = b.pose.inverse_transform_point(p);
    Vector3::from_fn(|k, _| (local[k].abs() - b.half[k]).max(0.0)).norm()
}

/// URDF's cylinder, along z.
fn point_cylinder(p: &Point3<f64>, c: &Solid) -> f64 {
    let local = c.pose.inverse_transform_point(p);
    let radial = (local.x.hypot(local.y) - c.half.x).max(0.0);
    let axial = (local.z.abs() - c.half.z).max(0.0);
    radial.hypot(axial)
}

/// Distance between segments `p1 q1` and `p2 q2` (Ericson, Real-Time
/// Collision Detection, 2005, section 5.1.9).
fn segment_segment(p1: &Point3<f64>, q1: &Point3<f64>, p2: &Point3<f64>, q2: &Point3<f64>) -> f64 {
    let (d1, d2, r) = (q1 - p1, q2 - p2, p1 - p2);
    let (a, e, f) = (d1.dot(&d1), d2.dot(&d2), d2.dot(&r));
    let (s, t) = if a <= 1e-30 && e <= 1e-30 {
        (0.0, 0.0)
    } else if a <= 1e-30 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(&r);
        if e <= 1e-30 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(&d2);
            let denominator = a * e - b * b;
            let s = if denominator > 0.0 {
                ((b * f - c * e) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t = (b * s + f) / e;
            if t < 0.0 {
                ((-c / a).clamp(0.0, 1.0), 0.0)
            } else if t > 1.0 {
                (((b - c) / a).clamp(0.0, 1.0), 1.0)
            } else {
                (s, t)
            }
        }
    };
    ((p1 + d1 * s) - (p2 + d2 * t)).norm()
}

/// Whether the segment meets the box: the slab test in the box's frame.
fn segment_meets_box(p: &Point3<f64>, q: &Point3<f64>, b: &Solid) -> bool {
    let (p, q) = (
        b.pose.inverse_transform_point(p),
        b.pose.inverse_transform_point(q),
    );
    let d = q - p;
    let (mut enter, mut leave) = (0.0_f64, 1.0_f64);
    (0..3).all(|k| {
        if d[k].abs() < 1e-300 {
            return p[k].abs() <= b.half[k];
        }
        let (t1, t2) = ((-b.half[k] - p[k]) / d[k], (b.half[k] - p[k]) / d[k]);
        enter = enter.max(t1.min(t2));
        leave = leave.min(t1.max(t2));
        enter <= leave
    })
}

/// A segment's distance to a box: zero where it meets the box, else the
/// least over its ends to the box and over the box's edges, among which
/// the nearest features always are.
fn segment_box(p: &Point3<f64>, q: &Point3<f64>, b: &Solid) -> f64 {
    if segment_meets_box(p, q, b) {
        return 0.0;
    }
    box_edges(b)
        .iter()
        .map(|(r, s)| segment_segment(p, q, r, s))
        .chain([point_box(p, b), point_box(q, b)])
        .fold(f64::INFINITY, f64::min)
}

/// Whether two boxes overlap: no separating axis among their faces' normals
/// and their edges' cross products (Gottschalk, Lin and Manocha, OBBTree,
/// 1996).
fn boxes_overlap(a: &Solid, b: &Solid) -> bool {
    let (on_a, on_b) = (axes(a), axes(b));
    let between = b.pose.translation.vector - a.pose.translation.vector;
    let crosses = on_a
        .iter()
        .flat_map(|u| on_b.iter().map(move |v| u.cross(v)))
        .filter(|axis| axis.norm() > 1e-9);
    on_a.into_iter().chain(on_b).chain(crosses).all(|axis| {
        let radius = |on: &[Vector3<f64>; 3], half: &Vector3<f64>| -> f64 {
            (0..3).map(|k| half[k] * on[k].dot(&axis).abs()).sum()
        };
        between.dot(&axis).abs() <= radius(&on_a, &a.half) + radius(&on_b, &b.half)
    })
}

/// The distance between two boxes: zero when they overlap, else the least
/// over each corner to the other box and over each pair of edges, among
/// which the nearest features of two convex polyhedra always are.
fn box_box(a: &Solid, b: &Solid) -> f64 {
    if boxes_overlap(a, b) {
        return 0.0;
    }
    let corners = box_corners(a)
        .iter()
        .map(|p| point_box(p, b))
        .chain(box_corners(b).iter().map(|p| point_box(p, a)))
        .collect::<Vec<_>>();
    let edges_b = box_edges(b);
    box_edges(a)
        .iter()
        .flat_map(|(p, q)| edges_b.iter().map(move |(r, s)| segment_segment(p, q, r, s)))
        .chain(corners)
        .fold(f64::INFINITY, f64::min)
}

/// A seeded generator (Knuth's MMIX constants).
struct Random(u64);

impl Random {
    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Log-uniform between `low` and `high`.
    fn size(&mut self, low: f64, high: f64) -> f64 {
        low * (high / low).powf(self.unit())
    }

    /// Uniform over rotations (Shoemake, Graphics Gems III, 1992).
    fn rotation(&mut self) -> UnitQuaternion<f64> {
        let (u1, u2, u3) = (self.unit(), self.unit(), self.unit());
        let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
        UnitQuaternion::new_normalize(Quaternion::new(
            b * (TAU * u3).cos(),
            a * (TAU * u2).sin(),
            a * (TAU * u2).cos(),
            b * (TAU * u3).sin(),
        ))
    }
}
