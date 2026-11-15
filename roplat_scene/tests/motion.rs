//! Whole-motion checks find what sampling steps over, and agree with dense
//! sampling everywhere else.

use std::path::PathBuf;

use nalgebra::Isometry3;
use roplat_description::{Geometry, RobotModel};
use roplat_scene::{CollisionCheck, MOTION_TOLERANCE, PlanningScene};
use roplat_trajectory::{
    PlannerOptions, plan_joint_motion, plan_joint_path, resolution_for_travel, segment_is_valid,
};

/// A planar two-link arm 0.25 m up: 0.5 m upper arm, 0.4 m forearm, and a
/// finger on a slide, 0.93 m from the shoulder when straight and retracted
/// (as in `scene.rs`).
const ARM: &str = r#"<robot name="toy">
  <link name="base"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0.25"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="upper"><collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision></link>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="0.5 0 0"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="fore"><collision><origin xyz="0.2 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.03" length="0.4"/></geometry></collision></link>
  <joint name="wrist" type="fixed"><parent link="fore"/><child link="flange"/><origin xyz="0.4 0 0"/></joint>
  <link name="flange"/>
  <joint name="slide" type="prismatic"><parent link="flange"/><child link="finger"/><origin xyz="0.03 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.05" effort="10" velocity="1"/></joint>
  <link name="finger"><collision><geometry><sphere radius="0.02"/></geometry></collision></link>
</robot>"#;

fn model() -> RobotModel {
    RobotModel::from_urdf_str(ARM).unwrap()
}

fn as_path(name: &str) -> PathBuf {
    PathBuf::from(name)
}

fn scene() -> PlanningScene<2> {
    PlanningScene::new(model(), ["shoulder", "elbow"], as_path).unwrap()
}

fn plate(size: [f64; 3]) -> Geometry {
    Geometry::Box { size }
}

fn at(x: f64, y: f64, z: f64) -> Isometry3<f64> {
    Isometry3::translation(x, y, z)
}

fn lerp<const N: usize>(a: &[f64; N], b: &[f64; N], t: f64) -> [f64; N] {
    std::array::from_fn(|i| a[i] + t * (b[i] - a[i]))
}

/// The first of `samples` evenly spaced configurations from `a` towards
/// `b` (`a` included, `b` not) that is in contact, as a fraction.
fn first_sampled_contact<const N: usize>(
    scene: &PlanningScene<N>,
    a: &[f64; N],
    b: &[f64; N],
    samples: usize,
) -> Option<f64> {
    (0..samples)
        .map(|k| k as f64 / samples as f64)
        .find(|&t| !scene.is_free(&lerp(a, b, t)))
}

/// A motion reported free has no sampled contact; a motion reported in
/// contact at `f` has none sampled before `f`, and touches at `f`.
fn agrees_with_sampling<const N: usize>(scene: &PlanningScene<N>, a: &[f64; N], b: &[f64; N]) {
    // About a tenth of a millimetre between samples at the finger.
    const SAMPLES: usize = 2000;
    match scene.motion_contact(a, b) {
        None => assert_eq!(
            first_sampled_contact(scene, a, b, SAMPLES),
            None,
            "{a:?} -> {b:?}"
        ),
        Some(hit) => {
            assert!((0.0..=1.0).contains(&hit.fraction));
            let touching = lerp(a, b, hit.fraction);
            let nearest = scene.clearance(&touching).unwrap();
            assert!(
                nearest.distance <= scene.padding() + MOTION_TOLERANCE + 1e-9,
                "{a:?} -> {b:?}: {hit:?}, nearest {nearest:?}"
            );
            if hit.fraction > 0.0
                && let Some(t) = first_sampled_contact(scene, a, &touching, SAMPLES)
            {
                panic!("{a:?} -> {b:?}: contact sampled at {t} of {}", hit.fraction);
            }
        }
    }
}

/// A plate 2 mm thick across the arm's sweep at shoulder angle zero, from
/// 0.6 m to 1 m out: the forearm sweeps it, and folds back to pass it.
fn plate_scene() -> PlanningScene<2> {
    let mut scene = scene();
    scene
        .add_object("plate", &plate([0.4, 0.002, 0.2]), at(0.8, 0.0, 0.25))
        .unwrap();
    scene
}

#[test]
fn a_thin_plate_between_samples_is_found() {
    let scene = plate_scene();
    let (from, to) = ([-0.3, 0.0], [0.3, 0.0]);
    assert!(scene.is_free(&from) && scene.is_free(&to));
    // Every 0.2 rad: the samples at ±0.1 rad pass either side of the plate.
    assert!(segment_is_valid(&from, &to, 0.2, |q| scene.is_free(q)));
    let hit = scene.motion_contact(&from, &to).unwrap();
    assert_eq!(
        (hit.contact.first.as_str(), hit.contact.second.as_str()),
        ("fore", "plate")
    );
    agrees_with_sampling(&scene, &from, &to);
    // Dense sampling first meets the plate just after the reported fraction.
    let sampled = first_sampled_contact(&scene, &from, &to, 6000).unwrap();
    assert!(
        sampled >= hit.fraction && sampled - hit.fraction < 0.002,
        "{hit:?} {sampled}"
    );
    assert!(!scene.motion_is_free(&from, &to));
    // Stopping short of the plate is free.
    assert!(scene.motion_is_free(&from, &[-0.1, 0.0]));
}

#[test]
fn the_padding_counts_along_the_motion() {
    let mut scene = plate_scene();
    let (from, to) = ([-0.3, 0.0], [-0.1, 0.0]);
    assert!(scene.motion_is_free(&from, &to));
    // At -0.1 rad the forearm passes about 3 cm from the plate.
    scene.set_padding(0.05).unwrap();
    let hit = scene.motion_contact(&from, &to).unwrap();
    assert!(hit.contact.distance > 0.0, "{hit:?}");
    agrees_with_sampling(&scene, &from, &to);
}

#[test]
fn motions_agree_with_dense_sampling() {
    let mut scene = plate_scene();
    scene
        .add_object(
            "post",
            &Geometry::Cylinder { radius: 0.04, length: 0.4 },
            at(-0.5, 0.5, 0.2),
        )
        .unwrap();
    scene
        .add_object(
            "ball",
            &Geometry::Sphere { radius: 0.1 },
            at(0.3, -0.6, 0.25),
        )
        .unwrap();
    // xorshift, so the motions are the same everywhere.
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    let mut unit = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let (mut free, mut hit) = (0, 0);
    for k in 0..80 {
        scene
            .set_padding(if k % 2 == 0 { 0.0 } else { 0.01 })
            .unwrap();
        let from = [unit() * 6.2 - 3.1, unit() * 6.2 - 3.1];
        let to: [f64; 2] = std::array::from_fn(|i| (from[i] + unit() - 0.5).clamp(-3.1, 3.1));
        if scene.motion_is_free(&from, &to) {
            free += 1;
        } else {
            hit += 1;
        }
        agrees_with_sampling(&scene, &from, &to);
    }
    // Both outcomes are exercised.
    assert!(free > 20 && hit > 20, "{free} free, {hit} in contact");
}

#[test]
fn an_attached_object_sweeps_with_its_link() {
    let mut scene = scene();
    // A rod 0.3 m long on the forearm's tip, along it, and a plate the arm
    // clears but the rod does not.
    scene
        .add_object(
            "rod",
            &Geometry::Cylinder { radius: 0.01, length: 0.3 },
            at(0.0, 0.0, 0.0),
        )
        .unwrap();
    let rod_on_tip = Isometry3::new(
        nalgebra::Vector3::new(1.05, 0.0, 0.25),
        nalgebra::Vector3::new(0.0, std::f64::consts::FRAC_PI_2, 0.0),
    );
    scene.move_object("rod", rod_on_tip).unwrap();
    scene
        .attach("rod", "fore", &["finger"], &[0.0, 0.0])
        .unwrap();
    scene
        .add_object("plate", &plate([0.1, 0.002, 0.2]), at(1.1, 0.0, 0.25))
        .unwrap();
    let (from, to) = ([-0.2, 0.0], [0.2, 0.0]);
    assert!(scene.is_free(&from) && scene.is_free(&to));
    let hit = scene.motion_contact(&from, &to).unwrap();
    assert_eq!(
        (hit.contact.first.as_str(), hit.contact.second.as_str()),
        ("rod", "plate")
    );
    agrees_with_sampling(&scene, &from, &to);
    // Detached and left behind, the rod no longer moves with the arm.
    scene.detach("rod", &[-0.2, 0.0]).unwrap();
    scene.remove_object("plate").unwrap();
    assert!(scene.motion_is_free(&[0.5, 0.0], &[0.7, 0.0]));
}

#[test]
fn a_prismatic_joint_moves_its_link_as_far_as_it_slides() {
    let mut scene: PlanningScene<3> =
        PlanningScene::new(model(), ["shoulder", "elbow", "slide"], as_path).unwrap();
    // The finger's surface is at x = 0.95 retracted and 1.0 extended.
    scene
        .add_object("stop", &plate([0.002, 0.2, 0.2]), at(0.98, 0.0, 0.25))
        .unwrap();
    let (from, to) = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.05]);
    let hit = scene.motion_contact(&from, &to).unwrap();
    assert_eq!(
        (hit.contact.first.as_str(), hit.contact.second.as_str()),
        ("finger", "stop")
    );
    // Touching at 0.979: (0.979 - 0.95) / 0.05.
    assert!((hit.fraction - 0.58).abs() < 0.01, "{hit:?}");
    agrees_with_sampling(&scene, &from, &to);
}

#[test]
fn the_arm_folding_onto_itself_is_found() {
    let scene = scene();
    let (from, to) = ([0.0, 2.0], [0.0, 3.1]);
    assert!(scene.is_free(&from) && !scene.is_free(&to));
    let hit = scene.motion_contact(&from, &to).unwrap();
    assert!(hit.fraction > 0.0 && hit.fraction < 1.0, "{hit:?}");
    agrees_with_sampling(&scene, &from, &to);
    // A configuration is checked as the motion to itself.
    assert!(
        scene
            .motion_contact(&to, &to)
            .is_some_and(|hit| hit.fraction == 0.0)
    );
    assert!(scene.motion_is_free(&from, &from));
}

#[test]
fn sensed_points_are_swept_against_too() {
    let mut scene = scene();
    // A sheet of points 2 cm apart across the sweep, one voxel thick.
    let points: Vec<[f64; 3]> = (0..20)
        .flat_map(|i| (0..10).map(move |k| [0.5 + 0.02 * i as f64, 0.0, 0.15 + 0.02 * k as f64]))
        .collect();
    scene
        .add_points("sheet", &points, 0.01, Isometry3::identity())
        .unwrap();
    let (from, to) = ([-0.3, 0.0], [0.3, 0.0]);
    assert!(scene.is_free(&from) && scene.is_free(&to));
    let hit = scene.motion_contact(&from, &to).unwrap();
    assert_eq!(hit.contact.second, "sheet");
    agrees_with_sampling(&scene, &from, &to);
}

#[test]
fn the_planner_goes_around_a_plate_it_would_sample_through() {
    let scene = plate_scene();
    let (start, goal) = ([-0.3, 0.0], [0.3, 0.0]);
    let (lower, upper) = ([-3.1; 2], [3.1; 2]);
    let options = PlannerOptions { resolution: 0.2, ..Default::default() };
    let sampled =
        plan_joint_path(start, goal, lower, upper, |q| scene.is_free(q), options).unwrap();
    assert_eq!(
        sampled,
        [start, goal],
        "sampling sees no plate on the direct line"
    );
    let path = plan_joint_motion(
        start,
        goal,
        lower,
        upper,
        |a, b| scene.motion_is_free(a, b),
        options,
    )
    .unwrap();
    assert!(path.len() > 2, "{path:?}");
    for pair in path.windows(2) {
        assert!(scene.motion_is_free(&pair[0], &pair[1]));
        assert_eq!(
            first_sampled_contact(&scene, &pair[0], &pair[1], 4000),
            None,
            "{pair:?}"
        );
    }
}

/// fix14b's planar arm with box links: two 0.6 m links 4 mm square,
/// turning about z (as in `distance.rs`).
const BOX_ARM: &str = r#"<robot name="planar">
  <link name="base"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="l1"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.004 0.004"/></geometry></collision></link>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0.6 0 0"/><axis xyz="0 0 1"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
  <link name="l2"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.004 0.004"/></geometry></collision></link>
</robot>"#;

/// The box arm and a box plate 2 mm thick across the outer link's sweep: y
/// from 0.010 to 0.012, x from 1.05 to 1.15 m.
fn box_arm_and_plate() -> PlanningScene<2> {
    let model = RobotModel::from_urdf_str(BOX_ARM).unwrap();
    let mut scene = PlanningScene::new(model, ["j1", "j2"], as_path).unwrap();
    scene
        .add_object("plate", &plate([0.1, 0.002, 0.05]), at(1.1, 0.011, 0.0))
        .unwrap();
    scene
}

/// Box against box, the pair parry 0.25 measures too far apart: straight
/// and at 0.02 rad the outer link clears the plate, between them it passes
/// through. With the overestimated distances the whole-motion check
/// advanced past the plate and called both motions free.
#[test]
fn a_box_link_sweeping_through_a_thin_plate_is_found() {
    let scene = box_arm_and_plate();
    let (from, to) = ([0.0, 0.0], [0.02, 0.0]);
    assert!(scene.is_free(&from) && scene.is_free(&to));
    assert!(!scene.is_free(&[0.01, 0.0]), "the sweep crosses the plate");
    let hit = scene
        .motion_contact(&from, &to)
        .expect("the motion through the plate is not free");
    assert_eq!(
        (hit.contact.first.as_str(), hit.contact.second.as_str()),
        ("l2", "plate")
    );
    // The link's upper face, 2 mm above its axis, first meets the plate's
    // lower edge at x = 1.15 m, y = 0.010 m, where
    // 0.010 cos θ - 1.15 sin θ = 0.002.
    let (a, b, c) = (0.010_f64, 1.15_f64, 0.002_f64);
    let touch = (c / a.hypot(b)).acos() - b.atan2(a);
    let reported = hit.fraction * to[0];
    assert!(
        reported <= touch && touch - reported < MOTION_TOLERANCE / 1.1,
        "touching at {touch} rad, reported at {reported} rad"
    );
    agrees_with_sampling(&scene, &from, &to);
    // The whole swing from -0.5 to 0.5 rad, whose checks 0.05 rad apart
    // all fall clear of the plate.
    assert!(!scene.motion_is_free(&[-0.5, 0.0], &[0.5, 0.0]));
    assert!(scene.motion_is_free(&[0.02, 0.0], &[0.5, 0.0]));
}

fn assert_reach<const N: usize>(reach: [f64; N], expected: [f64; N]) {
    assert!(
        reach
            .iter()
            .zip(&expected)
            .all(|(r, e)| (r - e).abs() < 1e-12),
        "reach {reach:?}, not {expected:?}"
    );
}

/// The toy arm's finger is the farthest from both joints: its 2 cm radius,
/// the slide's 3 cm and the wrist's 0.4 m from the elbow, 0.45 m, and the
/// elbow's 0.5 m more from the shoulder, 0.95 m.
#[test]
fn each_joint_reaches_as_far_as_what_it_moves() {
    let mut scene = scene();
    let (lower, upper) = ([-3.1; 2], [3.1; 2]);
    assert_reach(scene.reach(&lower, &upper), [0.95, 0.45]);
    // Slid out, the finger reaches 5 cm farther.
    scene.set_joint("slide", 0.05).unwrap();
    assert_reach(scene.reach(&lower, &upper), [1.0, 0.5]);
    scene.set_joint("slide", 0.0).unwrap();
    // A bar 0.2 m long held by the finger, from 0.93 to 1.13 m: its
    // bounding sphere is 0.1 m out from the finger and 0.101 m round.
    scene
        .add_object("bar", &plate([0.2, 0.02, 0.02]), at(1.03, 0.0, 0.25))
        .unwrap();
    scene.attach("bar", "finger", &[], &[0.0, 0.0]).unwrap();
    let bar = 0.1 + (0.1_f64.powi(2) + 2.0 * 0.01_f64.powi(2)).sqrt();
    assert_reach(
        scene.reach(&lower, &upper),
        [bar + 0.03 + 0.4 + 0.5, bar + 0.03 + 0.4],
    );
    scene.detach("bar", &[0.0, 0.0]).unwrap();
    assert_reach(scene.reach(&lower, &upper), [0.95, 0.45]);

    // In the group, the slide counts 1, and its range the others' reach.
    let slide: PlanningScene<3> =
        PlanningScene::new(model(), ["shoulder", "elbow", "slide"], as_path).unwrap();
    assert_reach(
        slide.reach(&[-3.1, -3.1, 0.0], &[3.1, 3.1, 0.05]),
        [1.0, 0.5, 1.0],
    );
}

/// Between two configurations checked at the resolution
/// `resolution_for_travel` gives for 1 cm and the scene's reach, no corner
/// of the upper arm, point on the forearm's rims or pole of the finger
/// moves 1 cm; turning both joints of the stretched arm one way, the
/// finger's tip moves nearly that.
#[test]
fn the_reach_bounds_how_far_a_point_moves_between_checks() {
    let scene = scene();
    let (lower, upper) = ([-3.1; 2], [3.1; 2]);
    let resolution = resolution_for_travel(0.01, &scene.reach(&lower, &upper));
    let mut points: Vec<(&str, nalgebra::Point3<f64>)> = Vec::new();
    for corner in 0..8 {
        let sign = |k: usize| if (corner >> k) & 1 == 1 { 1.0 } else { -1.0 };
        let p = nalgebra::Point3::new(0.25 + 0.25 * sign(0), 0.025 * sign(1), 0.025 * sign(2));
        points.push(("upper", p));
    }
    for k in 0..16 {
        let angle = std::f64::consts::TAU * k as f64 / 16.0;
        for x in [0.0, 0.4] {
            let p = nalgebra::Point3::new(x, 0.03 * angle.cos(), 0.03 * angle.sin());
            points.push(("fore", p));
        }
    }
    for axis in 0..3 {
        for sign in [-0.02, 0.02] {
            let mut p = nalgebra::Point3::origin();
            p[axis] = sign;
            points.push(("finger", p));
        }
    }
    let positions = |q: &[f64; 2]| -> Vec<nalgebra::Point3<f64>> {
        points
            .iter()
            .map(|(link, p)| scene.link_pose(link, q).unwrap() * p)
            .collect()
    };
    // xorshift, so the segments are the same everywhere.
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut unit = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut segments = vec![([0.0, 0.0], [0.5, 0.5])];
    for _ in 0..40 {
        let mut q = || [unit() * 6.2 - 3.1, unit() * 6.2 - 3.1];
        segments.push((q(), q()));
    }
    let mut farthest = 0.0_f64;
    for (a, b) in segments {
        let longest = (0..2).map(|i| (b[i] - a[i]).abs()).fold(0.0, f64::max);
        let steps = (longest / resolution).ceil().max(1.0) as usize;
        let mut before = positions(&a);
        for k in 1..=steps {
            let after = positions(&lerp(&a, &b, k as f64 / steps as f64));
            for (p, q) in before.iter().zip(&after) {
                farthest = farthest.max((q - p).norm());
            }
            before = after;
        }
    }
    assert!(farthest <= 0.01 + 1e-12, "a point moved {farthest} m");
    assert!(farthest > 0.0099, "the bound is loose: {farthest} m");
}

/// With the resolution from the scene's reach, the sampled planner no
/// longer steps over the 2 mm plate: no point moves more than 1 mm between
/// checks, and a padding of 2 mm keeps the samples 1.5 mm clear of the
/// plate between checks too, well above the whole-motion tolerance.
#[test]
fn the_reach_gives_the_sampled_planner_a_resolution_that_sees_the_plate() {
    let scene = box_arm_and_plate();
    let (start, goal) = ([-0.5, 0.0], [0.5, 0.0]);
    let (lower, upper) = ([-3.0, -2.5], [3.0, 2.5]);
    let reach = scene.reach(&lower, &upper);
    // 0.6 m from each joint to its link's end, and the link's half
    // diagonal past it.
    let tip = 0.3 + (0.3_f64.powi(2) + 2.0 * 0.002_f64.powi(2)).sqrt();
    assert_reach(reach, [tip + 0.6, tip]);
    let default = PlannerOptions::default();
    let stepped_over =
        plan_joint_path(start, goal, lower, upper, |q| scene.is_free(q), default).unwrap();
    assert_eq!(stepped_over, [start, goal]);

    let options = PlannerOptions { resolution: resolution_for_travel(0.001, &reach), ..default };
    let mut padded = scene.clone();
    padded.set_padding(0.002).unwrap();
    let path = plan_joint_path(start, goal, lower, upper, |q| padded.is_free(q), options).unwrap();
    assert_eq!((path[0], *path.last().unwrap()), (start, goal));
    assert!(path.len() > 2, "{path:?}");
    for pair in path.windows(2) {
        assert!(scene.motion_is_free(&pair[0], &pair[1]), "{pair:?}");
    }
}
