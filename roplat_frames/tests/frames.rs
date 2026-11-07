//! Static, joint-driven and dynamic transforms looked up at an instant.

mod support;

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};
use std::time::Duration;

use roplat_description::{JointKind, RobotModel};
use roplat_frames::{
    FrameBuffer, FrameError, FrameTree, FrameTreeBuilder, History, JointGroup, TreeError, frame_id,
};
use roplat_msgs::{Pose, Quaternion, Stamp, Transform, Vector3, clock};
use support::*;

fn buffer(arm: History, mug: History) -> (FrameBuffer, JointGroup<2>) {
    let frames = FrameBuffer::new(cell(arm, mug), clock::MONOTONIC).unwrap();
    let group = frames.tree().group::<2>("arm").unwrap();
    (frames, group)
}

fn mug_seen(x: f64, angle: f64) -> Transform {
    Transform {
        parent: CAMERA,
        child: MUG,
        pose: Pose::from_axis_angle([x, 0., 0.5], [0., 0., 1.], angle),
    }
}

#[test]
fn every_frame_is_named_by_its_id() {
    let tree = cell(History::new(1), History::new(1));
    assert_eq!(tree.root(), WORLD);
    assert_eq!(tree.name(CAMERA), Some("camera"));
    assert_eq!(tree.parent(CAMERA), Some(HAND));
    assert_eq!(tree.parent(WORLD), None);
    assert!(!tree.contains(frame_id("table")));

    let names: Vec<&str> = tree.frames().map(|(_, name)| name).collect();
    assert_eq!(
        names,
        [
            "world", "base", "upper", "fore", "hand", "finger", "camera", "mug"
        ]
    );
    assert!(tree.frames().all(|(id, name)| id == frame_id(name)));
}

#[test]
fn static_lookups_use_fixed_transforms_only() {
    let tree = cell(History::new(1), History::new(1));
    assert_pose(tree.lookup_static(WORLD, BASE).unwrap(), BASE_IN_WORLD);
    assert_pose(
        tree.lookup_static(BASE, WORLD).unwrap(),
        BASE_IN_WORLD.inverse(),
    );
    assert_pose(tree.lookup_static(HAND, CAMERA).unwrap(), CAMERA_IN_HAND);
    assert_pose(tree.lookup_static(CAMERA, CAMERA).unwrap(), Pose::IDENTITY);
    assert_eq!(
        tree.lookup_static(WORLD, CAMERA),
        Err(FrameError::NotStatic(FORE))
    );
    let table = frame_id("table");
    assert_eq!(
        tree.lookup_static(WORLD, table),
        Err(FrameError::UnknownFrame(table))
    );
}

#[test]
fn joint_positions_place_every_link() {
    let (mut frames, arm) = buffer(History::new(8), History::new(1));
    let q = [0.3, -0.7];
    frames.set_joints(arm, ms(0.), &q).unwrap();
    assert_pose(frames.lookup(BASE, HAND, ms(0.)).unwrap(), hand_in_base(q));
    assert_pose(
        frames.lookup(WORLD, CAMERA, ms(0.)).unwrap(),
        BASE_IN_WORLD * hand_in_base(q) * CAMERA_IN_HAND,
    );
    // The finger's joint is in no group; the hand and camera do not depend on it.
    assert_eq!(
        frames.lookup(HAND, FINGER, ms(0.)),
        Err(FrameError::Unpositioned(FINGER))
    );
}

#[test]
fn interpolates_joint_positions_between_samples() {
    let (mut frames, arm) = buffer(History::new(8), History::new(1));
    frames.set_joints(arm, ms(0.), &[0., 0.]).unwrap();
    frames.set_joints(arm, ms(10.), &[0.2, -0.4]).unwrap();
    assert_pose(
        frames.lookup(BASE, HAND, ms(5.)).unwrap(),
        hand_in_base([0.1, -0.2]),
    );
    assert_pose(
        frames.lookup(BASE, HAND, ms(10.)).unwrap(),
        hand_in_base([0.2, -0.4]),
    );
}

#[test]
fn holds_the_end_samples_only_within_hold() {
    let (mut frames, arm) = buffer(
        History::new(8).hold(Duration::from_millis(1)),
        History::new(1),
    );
    assert_eq!(
        frames.lookup(BASE, HAND, ms(0.)),
        Err(FrameError::NoData(FORE))
    );

    frames.set_joints(arm, ms(10.), &[0.1, 0.2]).unwrap();
    frames.set_joints(arm, ms(20.), &[0.3, 0.4]).unwrap();
    assert_pose(
        frames.lookup(BASE, HAND, ms(20.9)).unwrap(),
        hand_in_base([0.3, 0.4]),
    );
    assert_pose(
        frames.lookup(BASE, HAND, ms(9.5)).unwrap(),
        hand_in_base([0.1, 0.2]),
    );
    assert_eq!(
        frames.lookup(BASE, HAND, ms(22.)),
        Err(FrameError::AfterLatest { frame: FORE, at: ms(22.).nanos, latest: ms(20.).nanos })
    );
    assert_eq!(
        frames.lookup(BASE, HAND, ms(8.)),
        Err(FrameError::BeforeHistory { frame: FORE, at: ms(8.).nanos, earliest: ms(10.).nanos })
    );
}

#[test]
fn does_not_interpolate_across_a_gap() {
    let (mut frames, arm) = buffer(
        History::new(8).max_gap(Duration::from_millis(20)),
        History::new(1),
    );
    frames.set_joints(arm, ms(0.), &[0., 0.]).unwrap();
    frames.set_joints(arm, ms(50.), &[0.5, 0.]).unwrap();
    frames.set_joints(arm, ms(60.), &[0.6, 0.]).unwrap();
    assert_eq!(
        frames.lookup(BASE, HAND, ms(25.)),
        Err(FrameError::Gap { frame: FORE, before: 0, after: ms(50.).nanos })
    );
    assert_pose(
        frames.lookup(BASE, HAND, ms(50.)).unwrap(),
        hand_in_base([0.5, 0.]),
    );
    assert_pose(
        frames.lookup(BASE, HAND, ms(55.)).unwrap(),
        hand_in_base([0.55, 0.]),
    );
}

#[test]
fn joint_and_dynamic_tracks_interpolate_across_the_signed_stamp_range() {
    let (mut frames, arm) = buffer(History::new(2), History::new(2));
    let stamp = |t| Stamp::new(clock::MONOTONIC, t);
    for (t, value) in [(i64::MIN, 0.), (i64::MAX, 1.)] {
        frames.set_joints(arm, stamp(t), &[value, -value]).unwrap();
        frames
            .set_transform(stamp(t), &mug_seen(value, value))
            .unwrap();
    }
    for (t, fraction) in [(-(1 << 62), 0.25), (0, 0.5), (1 << 62, 0.75)] {
        assert_pose(
            frames.lookup(BASE, HAND, stamp(t)).unwrap(),
            hand_in_base([fraction, -fraction]),
        );
        assert_pose(
            frames.lookup(CAMERA, MUG, stamp(t)).unwrap(),
            mug_seen(fraction, fraction).pose,
        );
    }
}

// Endpoint, configured limit, and whether the distance from i64::MIN is covered.
const WIDE_LIMITS: [(i64, Duration, bool); 6] = [
    (-1, Duration::from_nanos(i64::MAX as u64), true),
    (0, Duration::from_nanos(i64::MAX as u64), false),
    (0, Duration::from_nanos(1_u64 << 63), true),
    (i64::MAX, Duration::from_nanos(u64::MAX - 1), false),
    (i64::MAX, Duration::from_nanos(u64::MAX), true),
    (i64::MAX, Duration::MAX, true),
];

#[test]
fn wide_gap_limits_include_equality_and_leave_exact_samples_readable() {
    let stamp = |t| Stamp::new(clock::MONOTONIC, t);
    for (end, limit, covers_gap) in WIDE_LIMITS {
        let (mut frames, _) = buffer(History::new(1), History::new(2).max_gap(limit));
        for t in [i64::MIN, end] {
            frames.set_transform(stamp(t), &mug_seen(1., 0.)).unwrap();
        }
        let lookup = |t| frames.lookup(CAMERA, MUG, stamp(t));
        assert_pose(lookup(i64::MIN).unwrap(), mug_seen(1., 0.).pose);
        assert_pose(lookup(end).unwrap(), mug_seen(1., 0.).pose);
        let inside = lookup(-2);
        if covers_gap {
            assert_pose(inside.unwrap(), mug_seen(1., 0.).pose);
        } else {
            assert_eq!(
                inside,
                Err(FrameError::Gap { frame: MUG, before: i64::MIN, after: end })
            );
        }
    }
}

#[test]
fn wide_hold_limits_use_the_exact_distance_in_both_directions() {
    let stamp = |t| Stamp::new(clock::MONOTONIC, t);
    for (end, limit, held) in WIDE_LIMITS {
        let (mut frames, _) = buffer(History::new(1), History::new(1).hold(limit));
        let seen = mug_seen(1., 0.);
        frames.set_transform(stamp(i64::MIN), &seen).unwrap();
        let after = frames.lookup(CAMERA, MUG, stamp(end));
        frames.set_transform(stamp(end), &seen).unwrap();
        let before = frames.lookup(CAMERA, MUG, stamp(i64::MIN));
        if held {
            assert_pose(after.unwrap(), seen.pose);
            assert_pose(before.unwrap(), seen.pose);
        } else {
            assert_eq!(
                after,
                Err(FrameError::AfterLatest { frame: MUG, at: end, latest: i64::MIN })
            );
            assert_eq!(
                before,
                Err(FrameError::BeforeHistory { frame: MUG, at: i64::MIN, earliest: end })
            );
        }
    }
}

#[test]
fn a_full_history_drops_its_oldest_sample() {
    let (mut frames, arm) = buffer(History::new(3), History::new(1));
    for i in 0..5 {
        frames
            .set_joints(arm, ms(i as f64), &[0.1 * i as f64, 0.])
            .unwrap();
    }
    assert_eq!(
        frames.lookup(BASE, HAND, ms(1.)),
        Err(FrameError::BeforeHistory { frame: FORE, at: ms(1.).nanos, earliest: ms(2.).nanos })
    );
    assert_pose(
        frames.lookup(BASE, HAND, ms(3.5)).unwrap(),
        hand_in_base([0.35, 0.]),
    );
}

#[test]
fn samples_arrive_in_time_order() {
    let (mut frames, arm) = buffer(History::new(8), History::new(1));
    frames.set_joints(arm, ms(10.), &[0.1, 0.]).unwrap();
    assert_eq!(
        frames.set_joints(arm, ms(5.), &[0.2, 0.]),
        Err(FrameError::OutOfOrder { stamp: ms(5.).nanos, latest: ms(10.).nanos })
    );
    // The same instant again replaces it, as when a paused simulation repeats.
    frames.set_joints(arm, ms(10.), &[0.3, 0.]).unwrap();
    assert_pose(
        frames.lookup(BASE, HAND, ms(10.)).unwrap(),
        hand_in_base([0.3, 0.]),
    );
}

#[test]
fn one_buffer_takes_one_clock() {
    let tree = cell(History::new(1), History::new(1));
    assert_eq!(
        FrameBuffer::new(tree.clone(), clock::UNSPECIFIED).unwrap_err(),
        TreeError::UnspecifiedClock
    );

    let (mut frames, arm) = buffer(History::new(8), History::new(1));
    let simulated = Stamp::new(clock::SIMULATION, 0);
    let mismatch = FrameError::ClockMismatch { expected: clock::MONOTONIC, got: clock::SIMULATION };
    assert_eq!(frames.set_joints(arm, simulated, &[0., 0.]), Err(mismatch));
    assert_eq!(
        frames.set_transform(simulated, &mug_seen(0., 0.)),
        Err(mismatch)
    );
    frames.set_joints(arm, ms(0.), &[0., 0.]).unwrap();
    assert_eq!(frames.lookup(BASE, HAND, simulated), Err(mismatch));
    // Even a static path names an instant on the buffer's clock.
    assert_eq!(frames.lookup(HAND, CAMERA, simulated), Err(mismatch));
}

#[test]
fn rejects_values_that_are_not_poses() {
    let (mut frames, arm) = buffer(History::new(8), History::new(8));
    assert_eq!(
        frames.set_joints(arm, ms(0.), &[f64::NAN, 0.]),
        Err(FrameError::InvalidValue)
    );

    let mut seen = mug_seen(0., 0.);
    seen.pose.orientation = Quaternion { x: 0., y: 0., z: 0., w: 0. };
    assert_eq!(
        frames.set_transform(ms(0.), &seen),
        Err(FrameError::InvalidValue)
    );
    seen.pose = Pose::from_position([f64::INFINITY, 0., 0.]);
    assert_eq!(
        frames.set_transform(ms(0.), &seen),
        Err(FrameError::InvalidValue)
    );
}

#[test]
fn dynamic_transforms_interpolate_position_and_rotation() {
    let (mut frames, arm) = buffer(History::new(8), History::new(4));
    let q = [0.3, -0.7];
    frames.set_joints(arm, ms(0.), &q).unwrap();
    frames.set_joints(arm, ms(10.), &q).unwrap();
    frames.set_transform(ms(0.), &mug_seen(0., 0.)).unwrap();
    frames
        .set_transform(ms(10.), &mug_seen(0.1, FRAC_PI_2))
        .unwrap();

    let halfway = mug_seen(0.05, FRAC_PI_4).pose;
    assert_pose(frames.lookup(CAMERA, MUG, ms(5.)).unwrap(), halfway);
    assert_pose(
        frames.lookup(WORLD, MUG, ms(5.)).unwrap(),
        BASE_IN_WORLD * hand_in_base(q) * CAMERA_IN_HAND * halfway,
    );

    let identity = |parent, child| Transform { parent, child, pose: Pose::IDENTITY };
    assert_eq!(
        frames.set_transform(ms(20.), &identity(HAND, CAMERA)),
        Err(FrameError::NotDynamic(CAMERA))
    );
    assert_eq!(
        frames.set_transform(ms(20.), &identity(HAND, MUG)),
        Err(FrameError::WrongParent { child: MUG, parent: HAND })
    );
}

#[test]
fn a_latched_transform_holds_for_any_instant() {
    let latched = History::new(1).hold(Duration::MAX);
    let (mut frames, _) = buffer(History::new(1), latched);
    frames.set_transform(ms(50.), &mug_seen(0.2, 0.)).unwrap();
    assert_pose(
        frames.lookup(CAMERA, MUG, ms(0.)).unwrap(),
        mug_seen(0.2, 0.).pose,
    );
    assert_pose(
        frames.lookup(CAMERA, MUG, ms(1e9)).unwrap(),
        mug_seen(0.2, 0.).pose,
    );
}

#[test]
fn lookups_in_opposite_directions_are_inverse() {
    let (mut frames, arm) = buffer(History::new(8), History::new(4));
    frames.set_joints(arm, ms(0.), &[0.9, 0.4]).unwrap();
    frames.set_transform(ms(0.), &mug_seen(0.1, 0.3)).unwrap();
    let there = frames.lookup(BASE, MUG, ms(0.)).unwrap();
    let back = frames.lookup(MUG, BASE, ms(0.)).unwrap();
    assert_pose(there * back, Pose::IDENTITY);
}

#[test]
fn malformed_trees_are_rejected() {
    let pose = Pose::IDENTITY;
    let rejected = |builder: FrameTreeBuilder| builder.build().unwrap_err();
    let names = |names: &[&str]| names.iter().map(|&n| n.into()).collect::<Vec<String>>();

    assert_eq!(rejected(FrameTree::builder()), TreeError::Roots(vec![]));
    assert_eq!(
        rejected(
            FrameTree::builder()
                .fixed("a", "b", pose)
                .fixed("c", "b", pose)
        ),
        TreeError::MultipleParents("b".into())
    );
    assert_eq!(
        rejected(
            FrameTree::builder()
                .fixed("a", "b", pose)
                .fixed("c", "d", pose)
        ),
        TreeError::Roots(names(&["a", "c"]))
    );
    assert_eq!(
        rejected(
            FrameTree::builder()
                .fixed("root", "x", pose)
                .fixed("a", "b", pose)
                .fixed("b", "a", pose)
        ),
        TreeError::Cycle(names(&["a", "b"]))
    );
    // FNV-1a gives both names 0x62d4fc8d.
    assert_eq!(frame_id("m0oe1l"), frame_id("5aum35"));
    assert_eq!(
        rejected(FrameTree::builder().fixed("m0oe1l", "5aum35", pose)),
        TreeError::IdCollision("m0oe1l".into(), "5aum35".into())
    );
    let zero = Pose {
        position: Vector3::ZERO,
        orientation: Quaternion { x: 0., y: 0., z: 0., w: 0. },
    };
    assert_eq!(
        rejected(FrameTree::builder().fixed("a", "b", zero)),
        TreeError::InvalidPose("b".into())
    );
    assert_eq!(
        rejected(FrameTree::builder().dynamic("a", "b", History::new(0))),
        TreeError::ZeroCapacity("b".into())
    );
}

#[test]
fn groups_list_each_movable_joint_once() {
    let model = model();
    let arm = ["shoulder", "elbow"].as_slice();
    let build = |groups: &[(&str, &[&str])]| {
        groups
            .iter()
            .fold(FrameTree::builder().model(&model), |b, (group, joints)| {
                b.joints(group, joints, History::new(1))
            })
            .build()
    };

    assert_eq!(
        build(&[("arm", &["wrist"])]).unwrap_err(),
        TreeError::UnknownJoint("wrist".into())
    );
    assert_eq!(
        build(&[("arm", &["wrist_mount"])]).unwrap_err(),
        TreeError::NotMovable("wrist_mount".into())
    );
    assert_eq!(
        build(&[("arm", arm), ("forearm", &["elbow"])]).unwrap_err(),
        TreeError::GroupedTwice("elbow".into())
    );
    assert_eq!(
        build(&[("arm", arm), ("arm", &["finger"])]).unwrap_err(),
        TreeError::DuplicateGroup("arm".into())
    );
    assert_eq!(
        build(&[("arm", &[])]).unwrap_err(),
        TreeError::EmptyGroup("arm".into())
    );
    assert_eq!(
        FrameTree::builder()
            .model(&model)
            .joints("arm", arm, History::new(0))
            .build()
            .unwrap_err(),
        TreeError::ZeroCapacity("arm".into())
    );

    let tree = build(&[("arm", arm)]).unwrap();
    assert_eq!(
        tree.group::<3>("arm").unwrap_err(),
        TreeError::GroupSize { group: "arm".into(), len: 2, requested: 3 }
    );
    assert_eq!(
        tree.group::<2>("legs").unwrap_err(),
        TreeError::UnknownGroup("legs".into())
    );
}

#[test]
fn joints_without_one_position_are_rejected() {
    let floating = RobotModel::from_urdf_str(
        r#"<robot name="free">
          <link name="world"/><link name="box"/>
          <joint name="free" type="floating"><parent link="world"/><child link="box"/></joint>
        </robot>"#,
    )
    .unwrap();
    assert_eq!(
        FrameTree::builder().model(&floating).build().unwrap_err(),
        TreeError::UnsupportedJoint { joint: "free".into(), kind: JointKind::Floating }
    );

    let repeated = RobotModel::from_urdf_str(
        r#"<robot name="repeated">
          <link name="a"/><link name="b"/><link name="c"/>
          <joint name="j" type="continuous"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint>
          <joint name="j" type="continuous"><parent link="b"/><child link="c"/><axis xyz="0 0 1"/></joint>
        </robot>"#,
    )
    .unwrap();
    assert_eq!(
        FrameTree::builder()
            .model(&repeated)
            .joints("g", &["j"], History::new(1))
            .build()
            .unwrap_err(),
        TreeError::AmbiguousJoint("j".into())
    );
}

#[test]
fn group_handles_belong_to_their_tree() {
    let other = FrameTree::builder()
        .model(&model())
        .joints("fingers", &["finger"], History::new(1))
        .joints("arm", &["shoulder", "elbow"], History::new(1))
        .build()
        .unwrap();
    let foreign = other.group::<2>("arm").unwrap();
    let (mut frames, _) = buffer(History::new(1), History::new(1));
    assert_eq!(
        frames.set_joints(foreign, ms(0.), &[0., 0.]),
        Err(FrameError::ForeignGroup)
    );
}

#[test]
fn independently_built_groups_at_the_same_index_cannot_write_each_others_tracks() {
    let (mut frames, own) = buffer(History::new(2), History::new(1));
    let (_, foreign) = buffer(History::new(2), History::new(1));
    assert_ne!(own, foreign);
    frames.set_joints(own, ms(1.), &[0.1, 0.2]).unwrap();
    assert_eq!(
        frames.set_joints(foreign, ms(1.), &[0.3, 0.4]),
        Err(FrameError::ForeignGroup)
    );
    let wrong_clock = Stamp::new(clock::SIMULATION, 0);
    assert_eq!(
        frames.set_joints(foreign, wrong_clock, &[f64::NAN; 2]),
        Err(FrameError::ClockMismatch { expected: clock::MONOTONIC, got: clock::SIMULATION })
    );
    assert_eq!(
        frames.set_joints(foreign, ms(0.), &[f64::NAN; 2]),
        Err(FrameError::ForeignGroup)
    );
    assert_eq!(
        frames.set_joints(own, ms(0.), &[f64::NAN; 2]),
        Err(FrameError::InvalidValue)
    );
    assert_pose(
        frames.lookup(BASE, HAND, ms(1.)).unwrap(),
        hand_in_base([0.1, 0.2]),
    );
}

#[test]
fn clones_accept_the_same_handles_and_buffers_keep_their_own_history() {
    let tree = FrameTree::builder()
        .model(&model())
        .joints("finger", &["finger"], History::new(1))
        .joints("arm", &["shoulder", "elbow"], History::new(2))
        .build()
        .unwrap();
    let arm = tree.group::<2>("arm").unwrap();
    let mut frames = FrameBuffer::new(tree.clone(), clock::MONOTONIC).unwrap();
    assert_eq!(arm, frames.tree().group::<2>("arm").unwrap());
    frames.set_joints(arm, ms(0.), &[0.1, 0.2]).unwrap();
    let mut copy = frames.clone();
    copy.set_joints(arm, ms(0.), &[0.3, 0.4]).unwrap();
    assert_pose(
        frames.lookup(BASE, HAND, ms(0.)).unwrap(),
        hand_in_base([0.1, 0.2]),
    );
    assert_pose(
        copy.lookup(BASE, HAND, ms(0.)).unwrap(),
        hand_in_base([0.3, 0.4]),
    );
}
