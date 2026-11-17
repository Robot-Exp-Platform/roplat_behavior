//! A built tree lists its edges, and a fixed edge takes a new pose (a new
//! calibration) without rebuilding the tree or losing the buffer's history.

mod support;

use roplat_description::JointKind;
use roplat_frames::{EdgeKind, FrameBuffer, FrameEdge, FrameError, History, frame_id};
use roplat_msgs::{Pose, Transform, clock};
use support::*;

#[test]
fn every_edge_is_listed_with_its_kind() {
    let tree = cell(History::new(4), History::new(4));
    let edges: Vec<FrameEdge<'_>> = tree.edges().collect();
    let upper = frame_id("upper");
    assert_eq!(
        edges,
        [
            FrameEdge { parent: WORLD, child: BASE, kind: EdgeKind::Fixed },
            FrameEdge {
                parent: BASE,
                child: upper,
                kind: EdgeKind::Joint {
                    joint: "shoulder",
                    kind: JointKind::Revolute,
                    group: Some("arm"),
                },
            },
            FrameEdge {
                parent: upper,
                child: FORE,
                kind: EdgeKind::Joint {
                    joint: "elbow",
                    kind: JointKind::Revolute,
                    group: Some("arm"),
                },
            },
            // A model's fixed joint is a fixed edge.
            FrameEdge { parent: FORE, child: HAND, kind: EdgeKind::Fixed },
            // No group positions the finger.
            FrameEdge {
                parent: HAND,
                child: FINGER,
                kind: EdgeKind::Joint {
                    joint: "finger",
                    kind: JointKind::Prismatic,
                    group: None,
                },
            },
            FrameEdge { parent: HAND, child: CAMERA, kind: EdgeKind::Fixed },
            FrameEdge { parent: CAMERA, child: MUG, kind: EdgeKind::Dynamic },
        ]
    );
    // One edge per frame but the root, each child's parent as `parent`
    // reports it.
    assert_eq!(edges.len(), tree.frames().count() - 1);
    assert!(edges.iter().all(|e| tree.parent(e.child) == Some(e.parent)));
    assert_eq!(tree.edge(CAMERA), Some(edges[5]));
    assert_eq!(tree.edge(WORLD), None);
    assert_eq!(tree.edge(frame_id("table")), None);
}

#[test]
fn a_fixed_edge_takes_a_new_pose() {
    let mut tree = cell(History::new(4), History::new(4));
    let recalibrated = Pose::from_axis_angle([0.06, -0.01, 0.025], [0., 1., 0.], 0.1);
    tree.set_fixed(CAMERA, &recalibrated).unwrap();
    assert_pose(tree.lookup_static(HAND, CAMERA).unwrap(), recalibrated);
    // Other fixed edges keep theirs, a model's fixed joint can be
    // recalibrated too, and the edge stays fixed.
    assert_pose(tree.lookup_static(WORLD, BASE).unwrap(), BASE_IN_WORLD);
    tree.set_fixed(HAND, &Pose::IDENTITY).unwrap();
    assert_pose(tree.lookup_static(FORE, HAND).unwrap(), Pose::IDENTITY);
    assert_eq!(tree.edge(CAMERA).unwrap().kind, EdgeKind::Fixed);

    // Joints, dynamic transforms and the root are not fixed edges; a bad
    // pose leaves the old one.
    let upper = frame_id("upper");
    for frame in [upper, MUG, WORLD] {
        assert_eq!(
            tree.set_fixed(frame, &Pose::IDENTITY),
            Err(FrameError::NotStatic(frame))
        );
    }
    let table = frame_id("table");
    assert_eq!(
        tree.set_fixed(table, &Pose::IDENTITY),
        Err(FrameError::UnknownFrame(table))
    );
    let mut broken = recalibrated;
    broken.position.x = f64::NAN;
    assert_eq!(tree.set_fixed(CAMERA, &broken), Err(FrameError::InvalidValue));
    assert_pose(tree.lookup_static(HAND, CAMERA).unwrap(), recalibrated);
}

#[test]
fn a_buffer_keeps_its_history_across_a_new_calibration() {
    let mut frames = FrameBuffer::new(cell(History::new(8), History::new(8)), clock::MONOTONIC)
        .unwrap();
    let arm = frames.tree().group::<2>("arm").unwrap();
    frames.set_joints(arm, ms(0.0), &[0.3, -0.2]).unwrap();
    frames.set_joints(arm, ms(10.0), &[0.5, -0.4]).unwrap();
    let mug = Transform {
        parent: CAMERA,
        child: MUG,
        pose: Pose::from_position([0.0, 0.0, 0.4]),
    };
    frames.set_transform(ms(5.0), &mug).unwrap();

    let before = frames.lookup(WORLD, MUG, ms(5.0)).unwrap();
    let recalibrated = Pose::from_position([0.05, 0.01, 0.02]);
    frames.set_fixed(CAMERA, &recalibrated).unwrap();
    let after = frames.lookup(WORLD, MUG, ms(5.0)).unwrap();
    // The mug moves with the camera's new mount: the hand's pose at 5 ms,
    // interpolated from the kept joint samples, then the new mount.
    let hand = frames.lookup(WORLD, HAND, ms(5.0)).unwrap();
    assert_pose(
        after,
        Pose::from_isometry(hand.quat() * recalibrated.quat() * mug.pose.quat()),
    );
    let moved = (after.quat().translation.vector - before.quat().translation.vector).norm();
    assert!((moved - 0.01).abs() < 1e-9, "the mug moved {moved} m");
    assert_pose(frames.tree().lookup_static(HAND, CAMERA).unwrap(), recalibrated);
    assert_eq!(
        frames.set_fixed(MUG, &Pose::IDENTITY),
        Err(FrameError::NotStatic(MUG))
    );
}
