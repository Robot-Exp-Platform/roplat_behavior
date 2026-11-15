//! Joint-space planning around obstacles given only by a validity check.

use roplat_trajectory::{
    PlanError, PlannerOptions, plan_joint_motion, plan_joint_path, resolution_for_travel,
    segment_is_valid, simplify_motion, smooth_motion,
};

/// A wall at x ∈ [0.4, 0.6] across the unit square, with a gap at y ∈ [0.8, 0.9].
fn outside_wall(q: &[f64; 2]) -> bool {
    !((0.4..=0.6).contains(&q[0]) && !(0.8..=0.9).contains(&q[1]))
}

fn check(path: &[[f64; 2]], start: [f64; 2], goal: [f64; 2], valid: impl Fn(&[f64; 2]) -> bool) {
    assert_eq!(path.first(), Some(&start));
    assert_eq!(path.last(), Some(&goal));
    for pair in path.windows(2) {
        assert!(
            segment_is_valid(&pair[0], &pair[1], 0.005, &valid),
            "{pair:?}"
        );
    }
}

#[test]
fn finds_the_gap_and_repeats_for_a_seed() {
    let (start, goal) = ([0.1, 0.1], [0.9, 0.1]);
    let options = PlannerOptions { max_step: 0.1, resolution: 0.005, ..Default::default() };
    let path = plan_joint_path(start, goal, [0.0; 2], [1.0; 2], outside_wall, options).unwrap();
    check(&path, start, goal, outside_wall);
    assert!(path.iter().any(|q| q[1] > 0.75), "{path:?}");
    let again = plan_joint_path(start, goal, [0.0; 2], [1.0; 2], outside_wall, options).unwrap();
    assert_eq!(path, again);
}

#[test]
fn a_clear_line_is_one_segment() {
    let options = PlannerOptions::default();
    let path = plan_joint_path(
        [0.1, 0.1],
        [0.3, 0.9],
        [0.0; 2],
        [1.0; 2],
        outside_wall,
        options,
    );
    assert_eq!(path.unwrap(), [[0.1, 0.1], [0.3, 0.9]]);
}

#[test]
fn reports_invalid_ends_and_closed_walls() {
    let options = PlannerOptions { max_iterations: 300, ..Default::default() };
    let (lower, upper) = ([0.0; 2], [1.0; 2]);
    assert_eq!(
        plan_joint_path([0.5, 0.1], [0.9, 0.1], lower, upper, outside_wall, options),
        Err(PlanError::InvalidStart)
    );
    assert_eq!(
        plan_joint_path([0.1, 0.1], [1.5, 0.1], lower, upper, outside_wall, options),
        Err(PlanError::InvalidGoal)
    );
    let closed = |q: &[f64; 2]| !(0.4..=0.6).contains(&q[0]);
    assert_eq!(
        plan_joint_path([0.1, 0.1], [0.9, 0.1], lower, upper, closed, options),
        Err(PlanError::NotFound { iterations: 300 })
    );
}

/// A planar 2R arm (links 0.5 m) whose elbow, forearm midpoint and tip must
/// stay out of a disc in front of it; the tip crosses from one side to the other.
#[test]
fn plans_an_arm_around_a_disc() {
    let points = |q: &[f64; 2]| {
        let elbow = [0.5 * q[0].cos(), 0.5 * q[0].sin()];
        let tip = [
            elbow[0] + 0.5 * (q[0] + q[1]).cos(),
            elbow[1] + 0.5 * (q[0] + q[1]).sin(),
        ];
        [
            elbow,
            [(elbow[0] + tip[0]) / 2.0, (elbow[1] + tip[1]) / 2.0],
            tip,
        ]
    };
    let clear = |q: &[f64; 2]| points(q).iter().all(|p| (p[0] - 0.8).hypot(p[1]) > 0.15);
    let (start, goal) = ([0.6, -0.4], [-0.6, 0.4]);
    let (lower, upper) = ([-std::f64::consts::PI; 2], [std::f64::consts::PI; 2]);
    assert!(
        !segment_is_valid(&start, &goal, 0.005, clear),
        "the straight move must collide"
    );
    let path =
        plan_joint_path(start, goal, lower, upper, clear, PlannerOptions::default()).unwrap();
    check(&path, start, goal, clear);
}

/// A wall 1 cm thick at x ∈ [0.495, 0.505], open above y = 0.8.
fn crosses_thin_wall(a: &[f64; 2], b: &[f64; 2]) -> bool {
    let (x0, x1) = (0.495, 0.505);
    // The part of the segment with x in the wall, as fractions of it.
    let (lo, hi) = if a[0] == b[0] {
        if (x0..=x1).contains(&a[0]) {
            (0.0, 1.0)
        } else {
            return false;
        }
    } else {
        let (t0, t1) = ((x0 - a[0]) / (b[0] - a[0]), (x1 - a[0]) / (b[0] - a[0]));
        (t0.min(t1).max(0.0), t0.max(t1).min(1.0))
    };
    // y is linear in t, so its lowest value on that part is at an end.
    lo <= hi && [lo, hi].iter().any(|t| a[1] + t * (b[1] - a[1]) < 0.8)
}

fn motion_clear(a: &[f64; 2], b: &[f64; 2]) -> bool {
    !crosses_thin_wall(a, b)
}

#[test]
fn a_motion_check_does_not_step_over_a_thin_wall() {
    let (start, goal) = ([0.2, 0.2], [0.75, 0.2]);
    let (lower, upper) = ([0.0; 2], [1.0; 2]);
    let point_free = |q: &[f64; 2]| motion_clear(q, q);
    // Samples 0.4 apart land on both sides of the wall: one segment, through it.
    let coarse = PlannerOptions { resolution: 0.4, ..Default::default() };
    let sampled = plan_joint_path(start, goal, lower, upper, point_free, coarse).unwrap();
    assert_eq!(sampled, [start, goal]);
    assert!(crosses_thin_wall(&start, &goal));

    let path = plan_joint_motion(start, goal, lower, upper, motion_clear, coarse).unwrap();
    assert_eq!((path[0], *path.last().unwrap()), (start, goal));
    for pair in path.windows(2) {
        assert!(motion_clear(&pair[0], &pair[1]), "{pair:?}");
    }
    assert!(path.iter().any(|q| q[1] > 0.8), "{path:?}");
    let again = plan_joint_motion(start, goal, lower, upper, motion_clear, coarse).unwrap();
    assert_eq!(path, again);

    // An end in the wall is refused as invalid.
    assert_eq!(
        plan_joint_motion([0.5, 0.1], goal, lower, upper, motion_clear, coarse),
        Err(PlanError::InvalidStart)
    );
}

#[test]
fn simplifying_and_smoothing_by_motion_keep_segments_off_the_wall() {
    let path = [[0.2, 0.2], [0.3, 0.9], [0.7, 0.9], [0.75, 0.2]];
    let simple = simplify_motion(&path, motion_clear);
    // The wall blocks the direct line, so a waypoint above it stays.
    assert!(simple.len() >= 3, "{simple:?}");
    for pair in simple.windows(2) {
        assert!(motion_clear(&pair[0], &pair[1]), "{pair:?}");
    }
    let smooth = smooth_motion(&simple, 3, motion_clear);
    assert_eq!((smooth[0], *smooth.last().unwrap()), (path[0], path[3]));
    for pair in smooth.windows(2) {
        assert!(motion_clear(&pair[0], &pair[1]), "{pair:?}");
    }
}

/// A planar arm with two 0.6 m links from the origin: its elbow and tip.
fn arm(q: &[f64; 2]) -> [[f64; 2]; 2] {
    let elbow = [0.6 * q[0].cos(), 0.6 * q[0].sin()];
    let tip = [
        elbow[0] + 0.6 * (q[0] + q[1]).cos(),
        elbow[1] + 0.6 * (q[0] + q[1]).sin(),
    ];
    [elbow, tip]
}

/// How far the points each joint of `arm` moves lie from its axis at most.
const ARM_REACH: [f64; 2] = [1.2, 0.6];

/// The wall's direction from the origin (rad).
const WALL_ANGLE: f64 = 0.005;

/// Whether the segment from `a` to `b` meets a plate 2 cm thick, 1.10 m to
/// 1.19 m out along `WALL_ANGLE`: clipped to the plate's extent along and
/// across its direction (Liang and Barsky).
fn meets_wall(a: [f64; 2], b: [f64; 2]) -> bool {
    let (c, s) = (WALL_ANGLE.cos(), WALL_ANGLE.sin());
    let local = |p: [f64; 2]| [c * p[0] + s * p[1], -s * p[0] + c * p[1]];
    let (a, b) = (local(a), local(b));
    let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
    for (axis, min, max) in [(0, 1.10, 1.19), (1, -0.01, 0.01)] {
        let d = b[axis] - a[axis];
        if d == 0.0 {
            if !(min..=max).contains(&a[axis]) {
                return false;
            }
            continue;
        }
        let (t0, t1) = ((min - a[axis]) / d, (max - a[axis]) / d);
        lo = lo.max(t0.min(t1));
        hi = hi.min(t0.max(t1));
    }
    lo <= hi
}

fn arm_clear(q: &[f64; 2]) -> bool {
    let [elbow, tip] = arm(q);
    !meets_wall([0.0, 0.0], elbow) && !meets_wall(elbow, tip)
}

/// The sweep of the stretched arm from -0.2 to 0.21 rad passes through the
/// wall, but at the default 0.02 rad the checked configurations fall either
/// side of it, 2 cm being less than the 2.3 cm the tip moves between them.
#[test]
fn the_scene_resolution_does_not_step_over_a_thin_wall() {
    let (start, goal) = ([-0.2, 0.0], [0.21, 0.0]);
    let (lower, upper) = ([-1.0, -2.5], [1.0, 2.5]);
    assert!(arm_clear(&start) && arm_clear(&goal));
    assert!(!arm_clear(&[WALL_ANGLE, 0.0]), "the sweep crosses the wall");
    let default = PlannerOptions::default();
    let stepped_over = plan_joint_path(start, goal, lower, upper, arm_clear, default).unwrap();
    assert_eq!(stepped_over, [start, goal]);

    // No point moves more than 1 cm between two checks.
    let resolution = resolution_for_travel(0.01, &ARM_REACH);
    assert!((resolution - 0.01 / 1.8).abs() < 1e-15);
    let options = PlannerOptions { resolution, ..Default::default() };
    let path = plan_joint_path(start, goal, lower, upper, arm_clear, options).unwrap();
    assert_eq!((path[0], *path.last().unwrap()), (start, goal));
    // Checked again 20 times finer: no point moves 0.5 mm between checks.
    let fine = resolution_for_travel(0.0005, &ARM_REACH);
    for pair in path.windows(2) {
        assert!(
            segment_is_valid(&pair[0], &pair[1], fine, arm_clear),
            "{pair:?}"
        );
    }
    // The forearm passes the wall only folded, its tip nearer than 1.10 m:
    // the elbow bent by more than 0.82 rad.
    assert!(path.iter().any(|q| q[1].abs() > 0.8), "{path:?}");
    let again = plan_joint_path(start, goal, lower, upper, arm_clear, options).unwrap();
    assert_eq!(path, again);
}

/// Between two configurations `segment_is_valid` checks at the resolution
/// for 1 cm, neither the elbow, the middle of the forearm nor the tip of
/// `arm` moves 1 cm: on the stretched arm turning both joints one way, where
/// the tip moves fastest, and on seeded random segments.
#[test]
fn the_scene_resolution_bounds_how_far_a_point_moves() {
    let travel = 0.01;
    let resolution = resolution_for_travel(travel, &ARM_REACH);
    let mut seed = 0x2545_f491_4f6c_dd1d_u64;
    let mut unit = move || {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut segments = vec![([0.0, 0.0], [0.5, 0.5]), ([0.3, -0.2], [-0.4, 0.5])];
    for _ in 0..200 {
        let mut q = || [unit() * 6.0 - 3.0, unit() * 6.0 - 3.0];
        segments.push((q(), q()));
    }
    let mut longest = 0.0_f64;
    for (a, b) in segments {
        let mut checked = vec![a];
        assert!(segment_is_valid(&a, &b, resolution, |q| {
            checked.push(*q);
            true
        }));
        // The last configuration checked is the end, up to rounding.
        let last = checked.last().unwrap();
        assert!((0..2).all(|i| (last[i] - b[i]).abs() < 1e-12), "{last:?}");
        for pair in checked.windows(2) {
            let (p, q) = (arm(&pair[0]), arm(&pair[1]));
            let middle = |[e, t]: [[f64; 2]; 2]| [(e[0] + t[0]) / 2.0, (e[1] + t[1]) / 2.0];
            for (from, to) in [(p[0], q[0]), (p[1], q[1]), (middle(p), middle(q))] {
                let moved = (to[0] - from[0]).hypot(to[1] - from[1]);
                longest = longest.max(moved);
                assert!(moved <= travel + 1e-12, "{pair:?} moves {moved}");
            }
        }
    }
    // The bound is nearly reached, so it is not loose by much.
    assert!(longest > 0.95 * travel, "{longest}");
}
