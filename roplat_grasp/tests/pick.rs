//! Picking a block off a table with a six-joint arm and a parallel gripper,
//! and putting it down elsewhere.

use std::path::PathBuf;

use nalgebra::{Isometry3, Vector3};
use roplat_description::{Geometry, RobotModel};
use roplat_grasp::{
    GraspOptions, PickFailure, PickOptions, PlaceFailure, PlaceOptions, grasps, plan_pick,
    plan_pick_motion, plan_place, plan_place_motion,
};
use roplat_kinematics::{IkSearch, chain_ik_solutions};
use roplat_msgs::geometry::Pose;
use roplat_scene::{CollisionCheck, EndEffector, NamedPoses, PlanningScene};

/// Base column 0.3 m, upper arm 0.4 m, forearm 0.35 m, a spherical wrist,
/// a 0.04 m palm on the flange and two fingers 0.06 m apart.
const ARM: &str = r#"<robot name="arm6">
  <link name="base"><collision><origin xyz="0 0 0.15"/><geometry><cylinder radius="0.06" length="0.3"/></geometry></collision></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="l1"/>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
  <link name="l2"><collision><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.36"/></geometry></collision></link>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="0 0 0.4"/><axis xyz="0 1 0"/><limit lower="-2.8" upper="2.8" effort="1" velocity="1"/></joint>
  <link name="l3"><collision><origin xyz="0 0 0.175"/><geometry><box size="0.04 0.04 0.3"/></geometry></collision></link>
  <joint name="j4" type="revolute"><parent link="l3"/><child link="l4"/><origin xyz="0 0 0.35"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="l4"/>
  <joint name="j5" type="revolute"><parent link="l4"/><child link="l5"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>
  <link name="l5"/>
  <joint name="j6" type="revolute"><parent link="l5"/><child link="flange"/><origin xyz="0 0 0.08"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="flange"/>
  <joint name="palm_joint" type="fixed"><parent link="flange"/><child link="palm"/></joint>
  <link name="palm"><collision><origin xyz="0 0 0.02"/><geometry><box size="0.04 0.1 0.04"/></geometry></collision></link>
  <joint name="left_joint" type="fixed"><parent link="palm"/><child link="left"/><origin xyz="0 0.03 0.07"/></joint>
  <link name="left"><collision><geometry><box size="0.02 0.01 0.06"/></geometry></collision></link>
  <joint name="right_joint" type="fixed"><parent link="palm"/><child link="right"/><origin xyz="0 -0.03 0.07"/></joint>
  <link name="right"><collision><geometry><box size="0.02 0.01 0.06"/></geometry></collision></link>
</robot>"#;

const JOINTS: [&str; 6] = ["j1", "j2", "j3", "j4", "j5", "j6"];
const LOWER: [f64; 6] = [-3.0, -2.5, -2.8, -3.0, -2.5, -3.0];
const UPPER: [f64; 6] = [3.0, 2.5, 2.8, 3.0, 2.5, 3.0];
/// Table top at z = 0.2; the block's centre 0.04 above it.
const BLOCK: [f64; 3] = [0.04, 0.04, 0.08];

fn model() -> RobotModel {
    RobotModel::from_urdf_str(ARM).unwrap()
}

/// The tool centre point between the fingertips, 0.09 m out of the flange.
fn hand() -> EndEffector {
    EndEffector::new("hand", "flange", Isometry3::translation(0.0, 0.0, 0.09))
        .with_links(["palm", "left", "right"])
}

fn scene(block_at: Vector3<f64>) -> PlanningScene<6> {
    let mut scene = PlanningScene::new(model(), JOINTS, |name: &str| PathBuf::from(name)).unwrap();
    scene
        .add_object(
            "table",
            &Geometry::Box { size: [0.5, 0.8, 0.04] },
            Isometry3::translation(0.55, 0.0, 0.18),
        )
        .unwrap();
    scene
        .add_object(
            "block",
            &Geometry::Box { size: BLOCK },
            Isometry3::translation(block_at.x, block_at.y, block_at.z),
        )
        .unwrap();
    scene
}

/// Inverse kinematics of the flange: several random starts, the solution
/// nearest the seed.
fn inverse(model: &RobotModel) -> impl FnMut(&Isometry3<f64>, &[f64; 6]) -> Option<[f64; 6]> {
    let chain = model.chain("base", "flange").unwrap();
    move |target, seed| {
        let search = IkSearch { restarts: 24, max_solutions: 1, ..IkSearch::default() };
        let solutions = chain_ik_solutions(
            &chain,
            &Pose::from_isometry(*target),
            seed,
            |_| true,
            search,
        )
        .ok()?;
        solutions.first().map(|q| std::array::from_fn(|i| q[i]))
    }
}

fn start() -> [f64; 6] {
    // Upright, the hand pointing up.
    [0.0; 6]
}

#[test]
fn picks_a_block_from_above_and_every_waypoint_is_free() {
    let at = Vector3::new(0.45, 0.1, 0.24);
    let scene = scene(at);
    let hand = hand();
    let candidates = grasps(
        &Geometry::Box { size: BLOCK },
        &Isometry3::translation(at.x, at.y, at.z),
        &GraspOptions::default(),
    );
    assert!(!candidates.is_empty());
    let model = model();
    let plan = plan_pick(
        &scene,
        &hand,
        "block",
        Some("table"),
        &candidates,
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_or_else(|e| panic!("{e}"));

    // A top grasp is preferred and reachable here.
    assert!((plan.grasp.approach() - (-Vector3::z())).norm() < 1e-9);

    // The paths join up.
    assert_eq!(plan.to_pregrasp.first(), Some(&start()));
    assert_eq!(plan.to_pregrasp.last(), plan.approach.first());
    assert_eq!(plan.approach.last(), plan.lift.first());

    // The tool ends at the grasp, then 0.1 m above it.
    let at_grasp = hand.pose(&scene, &plan.grasp_configuration()).unwrap();
    assert!((at_grasp.translation.vector - plan.grasp.pose.translation.vector).norm() < 1e-4);
    let lifted = hand.pose(&scene, plan.lift.last().unwrap()).unwrap();
    assert!(
        (lifted.translation.vector - at_grasp.translation.vector - Vector3::new(0.0, 0.0, 0.1))
            .norm()
            < 1e-4
    );

    // Before the grasp nothing touches; the approach itself is checked with
    // the fingers allowed to touch the block (they do not here: the block is
    // 0.04 wide and the fingers 0.05 apart inside).
    for q in plan.to_pregrasp.iter().chain(&plan.approach) {
        assert!(
            scene.is_free(q),
            "contact at {q:?}: {:?}",
            scene.first_contact(q)
        );
    }
    // Lifting with the block attached clears the table once it is up.
    let mut carrying = scene.clone();
    carrying
        .attach(
            "block",
            "flange",
            &["palm", "left", "right"],
            &plan.grasp_configuration(),
        )
        .unwrap();
    assert!(carrying.is_free(plan.lift.last().unwrap()));
}

#[test]
fn reports_why_each_candidate_failed() {
    // Far out of reach: every candidate's pre-grasp is unreachable.
    let at = Vector3::new(1.6, 0.0, 0.24);
    let scene = scene(at);
    let candidates = grasps(
        &Geometry::Box { size: BLOCK },
        &Isometry3::translation(at.x, at.y, at.z),
        &GraspOptions::default(),
    );
    let model = model();
    let err = plan_pick(
        &scene,
        &hand(),
        "block",
        Some("table"),
        &candidates,
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_err();
    assert_eq!(err.attempts.len(), candidates.len());
    assert!(
        err.attempts
            .iter()
            .all(|(_, why)| *why == PickFailure::PregraspUnreachable)
    );

    // No candidates at all.
    let err = plan_pick(
        &scene,
        &hand(),
        "block",
        None,
        &[],
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_err();
    assert!(err.attempts.is_empty());
    assert_eq!(err.to_string(), "no grasp candidates");
}

#[test]
fn named_poses_check_against_the_scene() {
    let scene = scene(Vector3::new(0.45, 0.1, 0.24));
    let mut poses = NamedPoses::<6>::new();
    poses.insert("up", start());
    // Tipped forward through the table top: in collision.
    poses.insert("slumped", [0.0, 2.0, 0.0, 0.0, 0.0, 0.0]);
    let free: Vec<_> = poses.free_in(&scene).collect();
    assert_eq!(free, ["up"]);
    assert_eq!(
        poses
            .nearest(&[0.0, 1.5, 0.0, 0.0, 0.0, 0.0])
            .map(|(n, _)| n),
        Some("slumped")
    );
}

#[test]
fn a_pick_checked_whole_has_every_segment_free() {
    let at = Vector3::new(0.45, 0.1, 0.24);
    let scene = scene(at);
    let hand = hand();
    let candidates = grasps(
        &Geometry::Box { size: BLOCK },
        &Isometry3::translation(at.x, at.y, at.z),
        &GraspOptions::default(),
    );
    let model = model();
    let plan = plan_pick_motion(
        &scene,
        &hand,
        "block",
        Some("table"),
        &candidates,
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_or_else(|e| panic!("{e}"));
    assert!((plan.grasp.approach() - (-Vector3::z())).norm() < 1e-9);
    assert_eq!(plan.to_pregrasp.first(), Some(&start()));
    assert_eq!(plan.to_pregrasp.last(), plan.approach.first());
    assert_eq!(plan.approach.last(), plan.lift.first());

    // Each path is free as whole motions, in the scene it was planned in.
    for pair in plan.to_pregrasp.windows(2) {
        assert!(scene.motion_is_free(&pair[0], &pair[1]), "{pair:?}");
    }
    let mut approaching = scene.clone();
    for link in hand.links() {
        approaching.allowed_mut().allow("block", link);
    }
    for pair in plan.approach.windows(2) {
        assert!(approaching.motion_is_free(&pair[0], &pair[1]), "{pair:?}");
    }
    let mut carrying = approaching;
    carrying
        .attach(
            "block",
            "flange",
            &hand.links(),
            &plan.grasp_configuration(),
        )
        .unwrap();
    carrying.allowed_mut().allow("block", "table");
    for pair in plan.lift.windows(2) {
        assert!(carrying.motion_is_free(&pair[0], &pair[1]), "{pair:?}");
    }
}

#[test]
fn places_the_picked_block_elsewhere_on_the_table() {
    let at = Vector3::new(0.45, 0.1, 0.24);
    let scene = scene(at);
    let hand = hand();
    let model = model();
    let candidates = grasps(
        &Geometry::Box { size: BLOCK },
        &Isometry3::translation(at.x, at.y, at.z),
        &GraspOptions::default(),
    );
    let pick = plan_pick(
        &scene,
        &hand,
        "block",
        Some("table"),
        &candidates,
        start(),
        LOWER,
        UPPER,
        inverse(&model),
        PickOptions::default(),
    )
    .unwrap_or_else(|e| panic!("{e}"));

    // The caller's scene after the pick: the block in the hand.
    let mut carrying = scene.clone();
    carrying
        .attach(
            "block",
            "flange",
            &hand.links(),
            &pick.grasp_configuration(),
        )
        .unwrap();
    let lifted = *pick.lift.last().unwrap();

    // Not held, nothing can be placed.
    let target = Isometry3::translation(0.5, -0.15, 0.24);
    let not_held = plan_place(
        &scene,
        &hand,
        "block",
        Some("table"),
        &[target],
        lifted,
        LOWER,
        UPPER,
        inverse(&model),
        PlaceOptions::default(),
    )
    .unwrap_err();
    assert_eq!(not_held.attempts, vec![(0, PlaceFailure::NotHeld)]);

    // A placement where a post stands fails; the next, free, works.
    carrying
        .add_object(
            "post",
            &Geometry::Box { size: [0.05, 0.05, 0.3] },
            Isometry3::translation(0.5, 0.15, 0.35),
        )
        .unwrap();
    let blocked = Isometry3::translation(0.5, 0.15, 0.24);
    for whole in [false, true] {
        let plan_with = if whole { plan_place_motion } else { plan_place };
        let plan = plan_with(
            &carrying,
            &hand,
            "block",
            Some("table"),
            &[blocked, target],
            lifted,
            LOWER,
            UPPER,
            inverse(&model),
            PlaceOptions::default(),
        )
        .unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(plan.candidate, 1, "{whole}");
        assert_eq!(plan.placement, target);
        assert_eq!(plan.to_preplace.first(), Some(&lifted));
        assert_eq!(plan.to_preplace.last(), plan.descent.first());
        assert_eq!(plan.descent.last(), plan.retreat.first());

        // Released where it was meant to go, with its orientation kept.
        let release = plan.release_configuration();
        let placed = carrying.object_pose("block", &release).unwrap();
        assert!((placed.translation.vector - target.translation.vector).norm() < 1e-4);
        assert!(placed.rotation.angle_to(&target.rotation) < 1e-3);

        // Carried clear of everything, then the hand backs off 0.1 m along
        // its axis from the released block.
        for q in &plan.to_preplace {
            assert!(
                carrying.is_free(q),
                "contact at {q:?}: {:?}",
                carrying.first_contact(q)
            );
        }
        let mut released = carrying.clone();
        released.detach("block", &release).unwrap();
        let backed = *plan.retreat.last().unwrap();
        assert!(released.is_free(&backed));
        let at_release = hand.pose(&released, &release).unwrap();
        let away = hand.pose(&released, &backed).unwrap();
        let back = at_release.inverse() * away;
        assert!((back.translation.vector - Vector3::new(0.0, 0.0, -0.1)).norm() < 1e-4);
    }
}

/// A coarse sampled check: a link certainly passes a plate between two of
/// its configurations. The defect does not depend on it; a check every
/// 0.02 rad steps over plates thinner than a link's sweep in 0.02 rad.
const COARSE: f64 = 0.1;

/// `scene` with a plate 1 mm thick across the straight motion from `from`
/// to `to`, midway between the two configurations a check every
/// [`COARSE`] looks at at which one of the fingers moves farthest,
/// perpendicular to that motion. It is the control: every such
/// configuration is clear of the plate, the straight motion is not, and
/// `plan_joint_path` at that resolution takes the straight line. Returns
/// the scene, the finger and how far it moves between the two
/// configurations.
fn plate_across(
    scene: &PlanningScene<6>,
    from: [f64; 6],
    to: [f64; 6],
) -> (PlanningScene<6>, &'static str, f64) {
    let (plated, link, moved) = ["left", "right"]
        .into_iter()
        .find_map(|link| plate_for(scene, from, to, link).map(|(s, m)| (s, link, m)))
        .expect("a plate between two checks in the way of a finger");
    let options = roplat_trajectory::PlannerOptions {
        resolution: COARSE,
        ..roplat_trajectory::PlannerOptions::default()
    };
    let sampled =
        roplat_trajectory::plan_joint_path(from, to, LOWER, UPPER, |q| plated.is_free(q), options)
            .unwrap();
    assert_eq!(sampled, vec![from, to], "the sampled check sees the plate");
    (plated, link, moved)
}

/// The plate across `link`'s farthest move between two checks, if every
/// checked configuration is clear of it and the straight motion is not.
fn plate_for(
    scene: &PlanningScene<6>,
    from: [f64; 6],
    to: [f64; 6],
    link: &str,
) -> Option<(PlanningScene<6>, f64)> {
    let distance = (0..6).map(|i| (to[i] - from[i]).abs()).fold(0.0, f64::max);
    let steps = (distance / COARSE).ceil() as usize;
    let at_sample = |k: usize| -> [f64; 6] {
        std::array::from_fn(|i| from[i] + (k as f64 / steps as f64) * (to[i] - from[i]))
    };
    let point = |k: usize| scene.link_pose(link, &at_sample(k)).unwrap().translation.vector;
    let k = (0..steps)
        .max_by(|&a, &b| {
            let d = |k: usize| (point(k + 1) - point(k)).norm();
            d(a).total_cmp(&d(b))
        })
        .unwrap();
    let (a, b) = (point(k), point(k + 1));
    let rotation = nalgebra::UnitQuaternion::rotation_between(&Vector3::z(), &(b - a))
        .unwrap_or_else(nalgebra::UnitQuaternion::identity);
    let mut plated = scene.clone();
    plated
        .add_object(
            "plate",
            &Geometry::Box { size: [0.015, 0.015, 0.001] },
            Isometry3::from_parts(((a + b) / 2.0).into(), rotation),
        )
        .unwrap();
    let clear = (1..=steps).all(|j| plated.is_free(&at_sample(j)));
    (clear && !plated.motion_is_free(&from, &to)).then(|| (plated, (b - a).norm()))
}

/// A plate 1 mm thick across the free path of a pick (in the way of a
/// finger) and of a place (in the way of the held block), between two of
/// the configurations a sampled check would look at, is not stepped over:
/// the free paths are checked as whole motions, whatever the resolution.
#[test]
fn a_thin_plate_across_the_free_path_is_not_stepped_over() {
    let at = Vector3::new(0.45, 0.1, 0.24);
    let model = model();
    let hand = hand();
    let candidates = grasps(
        &Geometry::Box { size: BLOCK },
        &Isometry3::translation(at.x, at.y, at.z),
        &GraspOptions::default(),
    );
    let planner = roplat_trajectory::PlannerOptions {
        resolution: COARSE,
        ..roplat_trajectory::PlannerOptions::default()
    };
    let pick = |scene: &PlanningScene<6>| {
        plan_pick(
            scene,
            &hand,
            "block",
            Some("table"),
            &candidates,
            start(),
            LOWER,
            UPPER,
            inverse(&model),
            PickOptions { planner, ..PickOptions::default() },
        )
        .unwrap_or_else(|e| panic!("{e}"))
    };
    // Where the free path goes without the plate, then with it.
    let clear = scene(at);
    let picked = pick(&clear);
    let q_pre = *picked.to_pregrasp.last().unwrap();
    let (plated, finger, moved) = plate_across(&clear, start(), q_pre);
    let plan = pick(&plated);
    assert_eq!(plan.to_pregrasp.last(), Some(&q_pre));
    assert!(plan.to_pregrasp.len() > 2);
    for pair in plan.to_pregrasp.windows(2) {
        assert!(plated.motion_is_free(&pair[0], &pair[1]), "{pair:?}");
        // An independent look, not resting on the scene's distances:
        // configurations 2 mrad apart, at which a finger moves about 2 mm,
        // less than its own thickness.
        assert!(
            roplat_trajectory::segment_is_valid(&pair[0], &pair[1], 0.002, |q| plated.is_free(q)),
            "{pair:?}"
        );
    }
    println!(
        "FIX14B grasp thin plate, pick: finger {finger} moves {moved:.3} m between two checks; the free path has {} waypoints",
        plan.to_pregrasp.len()
    );

    // The place, carrying the block from where the pick lifted it.
    let mut carrying = clear.clone();
    carrying
        .attach(
            "block",
            "flange",
            &hand.links(),
            &picked.grasp_configuration(),
        )
        .unwrap();
    let lifted = *picked.lift.last().unwrap();
    let target = Isometry3::translation(0.5, -0.15, 0.24);
    let place = |scene: &PlanningScene<6>| {
        plan_place(
            scene,
            &hand,
            "block",
            Some("table"),
            &[target],
            lifted,
            LOWER,
            UPPER,
            inverse(&model),
            PlaceOptions { planner, ..PlaceOptions::default() },
        )
        .unwrap_or_else(|e| panic!("{e}"))
    };
    let q_pre = *place(&carrying).to_preplace.last().unwrap();
    let (plated, finger, moved) = plate_across(&carrying, lifted, q_pre);
    let plan = place(&plated);
    assert_eq!(plan.to_preplace.last(), Some(&q_pre));
    assert!(plan.to_preplace.len() > 2);
    for pair in plan.to_preplace.windows(2) {
        assert!(plated.motion_is_free(&pair[0], &pair[1]), "{pair:?}");
        // An independent look, not resting on the scene's distances:
        // configurations 2 mrad apart, at which a finger moves about 2 mm,
        // less than its own thickness.
        assert!(
            roplat_trajectory::segment_is_valid(&pair[0], &pair[1], 0.002, |q| plated.is_free(q)),
            "{pair:?}"
        );
    }
    println!(
        "FIX14B grasp thin plate, place: finger {finger} moves {moved:.3} m between two checks; the free path has {} waypoints",
        plan.to_preplace.len()
    );
}
