//! `JointSCurve` keeps every joint within its velocity, acceleration and jerk
//! bounds, and takes the time the textbook double-S profile takes along the
//! joint line.
//!
//! The bounds are checked on finite differences of `apply`: the first, second
//! and third differences over a step `h`, divided by `h`, `h²` and `h³`, are
//! weighted means of the velocity, acceleration and jerk over the stencil, so
//! they never exceed the true peaks whatever the step. Only rounding is
//! allowed for.

use std::time::Duration;

use roplat_trajectory::path_generate::JointSCurve;

/// SplitMix64, so every run draws the same cases.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Log-uniform, so small and large bounds are drawn alike.
    fn log_uniform(&mut self, lo: f64, hi: f64) -> f64 {
        (self.uniform(lo.ln(), hi.ln())).exp()
    }
}

/// The duration of a rest-to-rest double-S move of length `h` within `v`,
/// `a` and `j`, by the case analysis of Biagiotti and Melchiorri,
/// "Trajectory Planning for Automatic Machines and Robots" (Springer 2008),
/// section 3.4, with zero initial and final velocity. The equations are
/// re-derived here, not copied from the book.
fn textbook_duration(h: f64, v: f64, a: f64, j: f64) -> f64 {
    // The acceleration phase that reaches v: a saturates only if the jerk
    // ramps gain less than v.
    let ta = if v * j < a * a {
        2.0 * (v / j).sqrt()
    } else {
        a / j + v / a
    };
    if h >= v * ta {
        // A cruise at v between two such phases.
        return h / v + ta;
    }
    if h >= 2.0 * a.powi(3) / (j * j) {
        // v is not reached, a is: Ta² − Tj Ta − h/a = 0 with Tj = a/j.
        let tj = a / j;
        return tj + (tj * tj + 4.0 * h / a).sqrt();
    }
    // Neither: four jerk phases of (h / 2j)^(1/3).
    4.0 * (h / (2.0 * j)).cbrt()
}

/// The largest speed, acceleration and jerk of each joint over the move,
/// from differences of `apply` at an exact nanosecond step.
fn peaks<const N: usize>(curve: &JointSCurve<N>, total: Duration, steps: u64) -> [[f64; 3]; N] {
    let step_ns = (total.as_nanos() as u64 / steps).max(1);
    let h = step_ns as f64 * 1e-9;
    let at = |k: u64| curve.apply(Duration::from_nanos(k * step_ns)).0;
    // A few steps before the start and after the end: the move must start
    // and end at rest.
    let samples: Vec<[f64; N]> = (0..steps + 8).map(at).collect();
    let mut peaks = [[0.0f64; 3]; N];
    for w in samples.windows(4) {
        for (i, joint) in peaks.iter_mut().enumerate() {
            let d1 = (w[1][i] - w[0][i]) / h;
            let d2 = (w[2][i] - 2.0 * w[1][i] + w[0][i]) / (h * h);
            let d3 = (w[3][i] - 3.0 * w[2][i] + 3.0 * w[1][i] - w[0][i]) / (h * h * h);
            for (peak, d) in joint.iter_mut().zip([d1, d2, d3]) {
                *peak = peak.max(d.abs());
            }
        }
    }
    peaks
}

/// What rounding adds to a difference of order `k` of positions up to
/// `scale` at step `h`.
fn rounding(scale: f64, h: f64, k: i32) -> f64 {
    16.0 * f64::EPSILON * scale.max(1.0) * 2f64.powi(k) / h.powi(k)
}

/// The case `motion11` reported: along the line the velocity bound is below
/// a²/j, so the bound is reached before the acceleration saturates.
///
/// Joint 0 sets all three line bounds: v = 0.2/0.3, a = 2/0.3, j = 5/0.3 per
/// unit of the line, and v = 0.667 < a²/j = 2.667. Each jerk phase lasts
/// sqrt(v/j) = 0.2 s, the acceleration phase 0.4 s, the cruise at v 1.1 s:
/// 1.9 s in all, joint 0 at its 0.2 m/s bound throughout the cruise. The jerk-
/// only profile the planner used to pick takes 1.243 s and peaks at 0.4827 m/s.
#[test]
fn the_velocity_bound_holds_when_reached_before_the_acceleration_saturates() {
    let mut curve = JointSCurve::new([0.2; 3], [2.0; 3], [5.0; 3]);
    let goal = [0.3, -0.2, 0.1];
    let total = curve.plan([0.0; 3], goal);
    assert!(
        (total.as_secs_f64() - 1.9).abs() < 1e-9,
        "planned {total:?}, the bounds give 1.9 s"
    );
    let peaks = peaks(&curve, total, 20_000);
    let h = total.as_secs_f64() / 20_000.0;
    for (i, [v, a, j]) in peaks.iter().enumerate() {
        assert!(*v <= 0.2 + rounding(0.3, h, 1), "joint {i}: {v} m/s");
        assert!(*a <= 2.0 + rounding(0.3, h, 2), "joint {i}: {a} m/s²");
        assert!(*j <= 5.0 + rounding(0.3, h, 3), "joint {i}: {j} m/s³");
    }
    // Joint 0 reaches its speed bound: the plan is not slower than needed.
    assert!(peaks[0][0] > 0.2 * (1.0 - 1e-6), "{}", peaks[0][0]);
    // At 0.2 s, the end of the first jerk phase, the line speed is
    // j t²/2 = 1/3 of v and the travelled fraction j t³/6 = 0.0222.
    let (q, _) = curve.apply(Duration::from_millis(200));
    assert!(
        (q[0] - 0.3 * (5.0 / 0.3) * 0.008 / 6.0).abs() < 1e-12,
        "{q:?}"
    );
    assert_eq!(curve.apply(total), (goal, true));
}

/// Random moves of three joints, a few joints standing still, bounds drawn
/// across four decades so every profile branch is taken: no bound is ever
/// exceeded, and the duration is the textbook one along the joint line.
#[test]
fn random_moves_stay_within_every_bound() {
    let mut rng = SplitMix64(0x5c0f_e12e);
    let mut below_a2_over_j = 0;
    for case in 0..1500 {
        let v: [f64; 3] = std::array::from_fn(|_| rng.log_uniform(0.05, 5.0));
        let a: [f64; 3] = std::array::from_fn(|_| rng.log_uniform(0.1, 20.0));
        let j: [f64; 3] = std::array::from_fn(|_| rng.log_uniform(0.5, 500.0));
        let start: [f64; 3] = std::array::from_fn(|_| rng.uniform(-2.0, 2.0));
        let goal: [f64; 3] = std::array::from_fn(|i| {
            if rng.uniform(0.0, 1.0) < 0.15 {
                start[i]
            } else {
                let d = rng.log_uniform(1e-3, 3.0);
                start[i] + if rng.next() & 1 == 0 { d } else { -d }
            }
        });
        let mut curve = JointSCurve::new(v, a, j);
        let total = curve.plan(start, goal);

        // The line's bounds: each joint's bound over its distance, the
        // smallest of them.
        let line = |bounds: &[f64; 3]| {
            (0..3)
                .filter(|&i| (goal[i] - start[i]).abs() > 1e-6)
                .map(|i| bounds[i] / (goal[i] - start[i]).abs())
                .fold(f64::INFINITY, f64::min)
        };
        let (lv, la, lj) = (line(&v), line(&a), line(&j));
        if !lv.is_finite() {
            assert_eq!(total, Duration::ZERO, "case {case}");
            continue;
        }
        if lv < la * la / lj {
            below_a2_over_j += 1;
        }
        let expected = textbook_duration(1.0, lv, la, lj);
        // Within the nanosecond of `Duration`.
        assert!(
            (total.as_secs_f64() - expected).abs() < 2e-9 * expected.max(1.0),
            "case {case}: planned {total:?}, the textbook gives {expected} s"
        );

        let steps = 2_000;
        let peaks = peaks(&curve, total, steps);
        let h = (total.as_nanos() as u64 / steps).max(1) as f64 * 1e-9;
        let scale = start
            .iter()
            .chain(&goal)
            .fold(0.0f64, |m, x| m.max(x.abs()));
        for (i, &[pv, pa, pj]) in peaks.iter().enumerate() {
            assert!(
                pv <= v[i] * (1.0 + 1e-9) + rounding(scale, h, 1),
                "case {case}, joint {i}: speed {pv} over {}",
                v[i]
            );
            assert!(
                pa <= a[i] * (1.0 + 1e-9) + rounding(scale, h, 2),
                "case {case}, joint {i}: acceleration {pa} over {}",
                a[i]
            );
            assert!(
                pj <= j[i] * (1.0 + 1e-9) + rounding(scale, h, 3),
                "case {case}, joint {i}: jerk {pj} over {}",
                j[i]
            );
        }
        assert_eq!(curve.apply(total), (goal, true), "case {case}");
    }
    // The region the defect was in is well covered.
    assert!(below_a2_over_j > 300, "{below_a2_over_j}");
}
