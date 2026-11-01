//! Point-to-point curves. Every generator is a struct: it is constructed with
//! the bounds, planned with a start and a target, and sampled by time with
//! `apply`, which returns the value and whether the move is complete. Where a
//! function is wanted, the caller writes `|t| curve.apply(t).0`.
use nalgebra as na;
use std::time::Duration;

#[cfg(test)]
use std::sync::Arc;

/// A synchronized constant-speed move of `N` joints: a [`JointSCurve`] with
/// unbounded acceleration and jerk, so every joint moves at the speed that
/// lets all of them arrive together.
#[derive(Clone, Debug)]
pub struct JointLinear<const N: usize>(JointSCurve<N>);

impl<const N: usize> JointLinear<N> {
    pub fn new(v_max: [f64; N]) -> Self {
        Self(JointSCurve::new(
            v_max,
            [f64::INFINITY; N],
            [f64::INFINITY; N],
        ))
    }

    /// Plans the move from `start` to `target` and returns its duration.
    pub fn plan(&mut self, start: [f64; N], target: [f64; N]) -> Duration {
        self.0.plan(start, target)
    }

    /// The joint positions at `time` and whether the move is complete.
    pub fn apply(&self, time: Duration) -> ([f64; N], bool) {
        self.0.apply(time)
    }
}

/// A synchronized trapezoidal-speed move of `N` joints: a [`JointSCurve`] with
/// unbounded jerk.
#[derive(Clone, Debug)]
pub struct JointTrapezoid<const N: usize>(JointSCurve<N>);

impl<const N: usize> JointTrapezoid<N> {
    pub fn new(v_max: [f64; N], a_max: [f64; N]) -> Self {
        Self(JointSCurve::new(v_max, a_max, [f64::INFINITY; N]))
    }

    /// Plans the move from `start` to `target` and returns its duration.
    pub fn plan(&mut self, start: [f64; N], target: [f64; N]) -> Duration {
        self.0.plan(start, target)
    }

    /// The joint positions at `time` and whether the move is complete.
    pub fn apply(&self, time: Duration) -> ([f64; N], bool) {
        self.0.apply(time)
    }
}

/// A straight-line move of a pose at a bounded linear speed; the orientation
/// turns along with it by spherical interpolation.
#[derive(Clone, Debug)]
pub struct CartesianLinear {
    v_max: f64,
    start: na::Isometry3<f64>,
    end: na::Isometry3<f64>,
    total: Duration,
    planned: bool,
}

impl CartesianLinear {
    pub fn new(v_max: f64) -> Self {
        Self {
            v_max,
            start: na::Isometry3::identity(),
            end: na::Isometry3::identity(),
            total: Duration::ZERO,
            planned: false,
        }
    }

    /// Plans the move from `start` to `end` and returns its duration.
    pub fn plan(&mut self, start: na::Isometry3<f64>, end: na::Isometry3<f64>) -> Duration {
        self.start = start;
        self.end = end;
        let distance = (end.translation.vector - start.translation.vector).norm();
        self.total = Duration::from_secs_f64(distance / self.v_max);
        self.planned = true;
        self.total
    }

    /// The pose at `time` and whether the move is complete.
    pub fn apply(&self, time: Duration) -> (na::Isometry3<f64>, bool) {
        if !self.planned || time >= self.total {
            return (self.end, true);
        }
        let progress = time.as_secs_f64() / self.total.as_secs_f64();
        (self.start.lerp_slerp(&self.end, progress), false)
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum ScalarSCurveProfile {
    Hold,
    /// Velocity and acceleration unbounded: the move takes no time.
    Instant {
        delta: f64,
    },
    /// Acceleration and jerk unbounded: constant speed from start to end.
    Constant {
        delta: f64,
        v_max: f64,
        t_total: f64,
    },
    Trapezoid {
        delta: f64,
        v_max: f64,
        a_max: f64,
        t_acc: f64,
        s_acc: f64,
        t_cruise: f64,
        t_total: f64,
    },
    Short {
        delta: f64,
        j_max: f64,
        t_min: f64,
        t1: f64,
    },
    Cruise {
        delta: f64,
        v_max: f64,
        a_max: f64,
        j_max: f64,
        t_min: f64,
        t1: f64,
        t2: f64,
        t34: f64,
    },
    NoCruise {
        delta: f64,
        a_max: f64,
        j_max: f64,
        t_min: f64,
        t1: f64,
        t2: f64,
    },
}

impl ScalarSCurveProfile {
    pub(crate) fn plan(delta: f64, v_max: f64, a_max: f64, j_max: f64) -> Self {
        if delta.abs() < 1e-6 {
            return Self::Hold;
        }

        let delta = delta.abs();
        if !a_max.is_finite() && !j_max.is_finite() {
            return if v_max.is_finite() {
                Self::Constant { delta, v_max, t_total: delta / v_max }
            } else {
                Self::Instant { delta }
            };
        }
        // An unbounded jerk plans a trapezoid. Compare with infinity, not
        // f64::MAX: a huge jerk bound divided by a short distance overflows.
        if !j_max.is_finite() {
            // A short move peaks at sqrt(delta * a_max): half the distance
            // accelerating, half braking.
            let v_max = v_max.min((delta * a_max).sqrt());
            let t_acc = v_max / a_max;
            let s_acc = 0.5 * a_max * t_acc * t_acc;
            let s_cruise = delta - 2. * s_acc;
            let t_cruise = if s_cruise > 0. { s_cruise / v_max } else { 0. };
            let t_total = 2. * t_acc + t_cruise;
            return Self::Trapezoid { delta, v_max, a_max, t_acc, s_acc, t_cruise, t_total };
        }

        let d2 = 2. * a_max.powi(3) / j_max.powi(2);
        let d1 = v_max * (a_max / j_max + v_max / a_max);

        if delta < d2 {
            let t_min = (32. * delta / j_max).powf(1. / 3.);
            Self::Short { delta, j_max, t_min, t1: t_min / 4. }
        } else if delta > d1 {
            Self::Cruise {
                delta,
                v_max,
                a_max,
                j_max,
                t_min: delta / v_max + v_max / a_max + a_max / j_max,
                t1: a_max / j_max,
                t2: v_max / a_max,
                t34: (delta - d1) / v_max,
            }
        } else {
            let t_min = (a_max.powi(2)
                + (a_max.powi(4) + 4. * a_max * delta * j_max.powi(2)).sqrt())
                / (a_max * j_max);
            Self::NoCruise {
                delta,
                a_max,
                j_max,
                t_min,
                t1: a_max / j_max,
                t2: t_min / 2. - a_max / j_max,
            }
        }
    }

    pub(crate) fn total_time(self) -> f64 {
        match self {
            Self::Hold | Self::Instant { .. } => 0.,
            Self::Constant { t_total, .. } => t_total,
            Self::Trapezoid { t_total, .. } => t_total,
            Self::Short { t_min, .. } => t_min,
            Self::Cruise { t_min, .. } => t_min,
            Self::NoCruise { t_min, .. } => t_min,
        }
    }

    pub(crate) fn sample(self, t: f64) -> f64 {
        match self {
            Self::Hold => 0.,
            Self::Instant { delta } => delta,
            Self::Constant { delta, v_max, t_total } => {
                if t < t_total {
                    v_max * t
                } else {
                    delta
                }
            }
            Self::Trapezoid { delta, v_max, a_max, t_acc, s_acc, t_cruise, t_total } => {
                let t = t.min(t_total);
                if t < t_acc {
                    0.5 * a_max * t * t
                } else if t < t_acc + t_cruise {
                    s_acc + v_max * (t - t_acc)
                } else {
                    delta - 0.5 * a_max * (t_total - t).powi(2)
                }
            }
            Self::Short { delta, j_max, t_min, t1 } => {
                let t = t.min(t_min);
                if t < t1 {
                    Self::path_1(j_max, t)
                } else if t < t1 * 3. {
                    Self::path_1(j_max, t1)
                        + Self::path_3(j_max, t - t1, j_max * t1.powi(2) / 2., j_max * t1)
                } else if t < t_min {
                    delta - Self::path_1(j_max, t_min - t)
                } else {
                    delta
                }
            }
            Self::Cruise { delta, v_max, a_max, j_max, t_min, t1, t2, t34 } => {
                let t = t.min(t_min);
                if t < t1 {
                    Self::path_1(j_max, t)
                } else if t < t2 {
                    Self::path_1(j_max, t1) + Self::path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t1 + t2 {
                    Self::path_1(j_max, t1)
                        + Self::path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + Self::path_3(j_max, t - t2, v_max - a_max * t1 / 2., a_max)
                } else if t < t1 + t2 + t34 {
                    delta / 2. + Self::path_4(t - t_min / 2., v_max)
                } else if t < t_min - t2 {
                    delta / 2.
                        + Self::path_4(t34 / 2., v_max)
                        + Self::path_5(j_max, t - (t1 + t2 + t34), v_max)
                } else if t < t_min - t1 {
                    delta
                        - Self::path_1(j_max, t1)
                        - Self::path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - Self::path_1(j_max, t_min - t)
                } else {
                    delta
                }
            }
            Self::NoCruise { delta, a_max, j_max, t_min, t1, t2 } => {
                let t = t.min(t_min);
                if t < t1 {
                    Self::path_1(j_max, t)
                } else if t < t2 {
                    Self::path_1(j_max, t1) + Self::path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min - t2 {
                    Self::path_1(j_max, t1)
                        + Self::path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + Self::path_3(j_max, t - t2, a_max * (t2 - t1 / 2.), a_max)
                } else if t < t_min - t1 {
                    delta
                        - Self::path_1(j_max, t1)
                        - Self::path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - Self::path_1(j_max, t_min - t)
                } else {
                    delta
                }
            }
        }
    }

    fn path_1(j_max: f64, t: f64) -> f64 {
        j_max * t.powi(3) / 6.
    }

    fn path_2(t: f64, v_s: f64, a: f64) -> f64 {
        a * t.powi(2) / 2. + v_s * t
    }

    fn path_3(j_max: f64, t: f64, v_s: f64, a_s: f64) -> f64 {
        -j_max * t.powi(3) / 6. + a_s * t.powi(2) / 2. + v_s * t
    }

    fn path_4(t: f64, v_s: f64) -> f64 {
        v_s * t
    }

    fn path_5(j_max: f64, t: f64, v_s: f64) -> f64 {
        v_s * t - j_max * t.powi(3) / 6.
    }
}

/// A synchronized jerk-limited move of `N` joints from a start to a target.
///
/// A bound may be `f64::INFINITY`. An unbounded jerk gives a trapezoid, an
/// unbounded acceleration and jerk a constant speed, and an unbounded
/// velocity and acceleration a move that takes no time. A caller that drives
/// hardware passes finite velocity and acceleration bounds.
#[derive(Clone, Debug)]
pub struct JointSCurve<const N: usize> {
    v_max: [f64; N],
    a_max: [f64; N],
    j_max: [f64; N],
    start: [f64; N],
    target: [f64; N],
    delta: [f64; N],
    profile: ScalarSCurveProfile,
    total: Duration,
    planned: bool,
}

impl<const N: usize> JointSCurve<N> {
    pub fn new(v_max: [f64; N], a_max: [f64; N], j_max: [f64; N]) -> Self {
        Self {
            v_max,
            a_max,
            j_max,
            start: [0.; N],
            target: [0.; N],
            delta: [0.; N],
            profile: ScalarSCurveProfile::Hold,
            total: Duration::ZERO,
            planned: false,
        }
    }

    pub fn plan(&mut self, start: [f64; N], target: [f64; N]) -> Duration {
        self.start = start;
        self.target = target;
        let mut v_path = f64::INFINITY;
        let mut a_path = f64::INFINITY;
        let mut j_path = f64::INFINITY;
        let mut has_motion = false;

        for i in 0..N {
            self.delta[i] = target[i] - start[i];
            let delta = self.delta[i].abs();
            if delta > 1e-6 {
                has_motion = true;
                v_path = v_path.min(self.v_max[i] / delta);
                a_path = a_path.min(self.a_max[i] / delta);
                j_path = j_path.min(self.j_max[i] / delta);
            }
        }

        if has_motion {
            self.profile = ScalarSCurveProfile::plan(1., v_path, a_path, j_path);
            self.total = Duration::from_secs_f64(self.profile.total_time());
        } else {
            self.profile = ScalarSCurveProfile::Hold;
            self.total = Duration::ZERO;
        }
        self.planned = true;
        self.total
    }

    pub fn apply(&self, time: Duration) -> ([f64; N], bool) {
        if !self.planned || self.total.is_zero() {
            return (self.target, true);
        }

        let progress = self.profile.sample(time.as_secs_f64()).min(1.);
        let mut result = self.start;
        for (value, delta) in result.iter_mut().zip(&self.delta) {
            *value += delta * progress;
        }

        if time >= self.total {
            (self.target, true)
        } else {
            (result, false)
        }
    }
}

#[cfg(all(test, any(target_os = "linux", target_os = "windows")))]
fn s_curve(
    delta: f64,
    v_max: f64,
    a_max: f64,
    j_max: f64,
) -> (f64, Arc<dyn Fn(Duration) -> f64 + Send + Sync>) {
    if !j_max.is_finite() {
        let profile = ScalarSCurveProfile::plan(delta, v_max, a_max, f64::INFINITY);
        return (
            profile.total_time(),
            Arc::new(move |t: Duration| profile.sample(t.as_secs_f64())),
        );
    }
    let d2 = 2. * a_max.powi(3) / j_max.powi(2);
    let d1 = v_max * (a_max / j_max + v_max / a_max);

    let path_1 = move |t: f64| j_max * t.powi(3) / 6.;
    let path_2 = move |t: f64, v_s: f64, a: f64| a * t.powi(2) / 2. + v_s * t;
    let path_3 =
        move |t: f64, v_s: f64, a_s: f64| -j_max * t.powi(3) / 6. + a_s * t.powi(2) / 2. + v_s * t;
    let path_4 = move |t: f64, v_s: f64| v_s * t;
    let path_5 = move |t: f64, v_s: f64| v_s * t - j_max * t.powi(3) / 6.;

    if delta < d2 {
        let t_min = (32. * delta / j_max).powf(1. / 3.);
        let t1 = t_min / 4.;
        (
            t_min,
            Arc::new(move |t: Duration| {
                let t = t.as_secs_f64();
                if t < t1 {
                    path_1(t)
                } else if t < t1 * 3. {
                    path_1(t1) + path_3(t - t1, j_max * t1.powi(2) / 2., j_max * t1)
                } else if t < t_min {
                    delta - path_1(t_min - t)
                } else {
                    delta
                }
            }),
        )
    } else if delta > d1 {
        let t_min = delta / v_max + v_max / a_max + a_max / j_max;
        let t1 = a_max / j_max;
        let t2 = v_max / a_max;
        let t34 = (delta - d1) / v_max;
        (
            t_min,
            Arc::new(move |t: Duration| {
                let t = t.as_secs_f64();
                if t < t1 {
                    path_1(t)
                } else if t < t2 {
                    path_1(t1) + path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t1 + t2 {
                    path_1(t1)
                        + path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + path_3(t - t2, v_max - a_max * t1 / 2., a_max)
                } else if t < t1 + t2 + t34 {
                    delta / 2. + path_4(t - t_min / 2., v_max)
                } else if t < t_min - t2 {
                    delta / 2. + path_4(t34 / 2., v_max) + path_5(t - (t1 + t2 + t34), v_max)
                } else if t < t_min - t1 {
                    delta - path_1(t1) - path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - path_1(t_min - t)
                } else {
                    delta
                }
            }),
        )
    } else {
        let t_min = (a_max.powi(2) + (a_max.powi(4) + 4. * a_max * delta * j_max.powi(2)).sqrt())
            / (a_max * j_max);
        let t1 = a_max / j_max;
        let t2 = t_min / 2. - a_max / j_max;
        (
            t_min,
            Arc::new(move |t: Duration| {
                let t = t.as_secs_f64();
                if t < t1 {
                    path_1(t)
                } else if t < t2 {
                    path_1(t1) + path_2(t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min - t2 {
                    path_1(t1)
                        + path_2(t2 - t1, a_max * t1 / 2., j_max * t1)
                        + path_3(t - t2, a_max * (t2 - t1 / 2.), a_max)
                } else if t < t_min - t1 {
                    delta - path_1(t1) - path_2(t_min - t - t1, a_max * t1 / 2., j_max * t1)
                } else if t < t_min {
                    delta - path_1(t_min - t)
                } else {
                    delta
                }
            }),
        )
    }
}

/// A synchronized fourth-order move of `N` joints. Each joint follows a smooth
/// polynomial speed profile within its own speed and acceleration bounds; the
/// profiles are stretched in time so that all joints arrive together.
#[derive(Clone, Debug)]
pub struct JointFourthOrder<const N: usize> {
    v_max: [f64; N],
    a_max: [f64; N],
    start: [f64; N],
    target: [f64; N],
    profiles: [ScalarFourthOrderProfile; N],
    times: [f64; N],
    total: Duration,
    planned: bool,
}

impl<const N: usize> JointFourthOrder<N> {
    pub fn new(v_max: [f64; N], a_max: [f64; N]) -> Self {
        Self {
            v_max,
            a_max,
            start: [0.; N],
            target: [0.; N],
            profiles: [ScalarFourthOrderProfile::Hold; N],
            times: [0.; N],
            total: Duration::ZERO,
            planned: false,
        }
    }

    /// Plans the move from `start` to `target` and returns its duration.
    pub fn plan(&mut self, start: [f64; N], target: [f64; N]) -> Duration {
        self.start = start;
        self.target = target;
        let mut longest = 0.0_f64;
        for i in 0..N {
            let distance = (target[i] - start[i]).abs();
            let profile = ScalarFourthOrderProfile::plan(distance, self.v_max[i], self.a_max[i]);
            self.times[i] = profile.total_time();
            self.profiles[i] = profile;
            longest = longest.max(self.times[i]);
        }
        self.total = Duration::from_secs_f64(longest);
        self.planned = true;
        self.total
    }

    /// The joint positions at `time` and whether the move is complete.
    pub fn apply(&self, time: Duration) -> ([f64; N], bool) {
        if !self.planned || self.total.is_zero() || time >= self.total {
            return (self.target, true);
        }
        let progress = time.as_secs_f64() / self.total.as_secs_f64();
        let mut result = self.start;
        for i in 0..N {
            let travelled = self.profiles[i].sample(progress * self.times[i]);
            result[i] += (self.target[i] - self.start[i]).signum() * travelled;
        }
        (result, false)
    }
}

/// A straight-line move of a pose with a fourth-order speed profile along the
/// line, within a linear speed and acceleration bound; the orientation turns
/// along with it by spherical interpolation.
#[derive(Clone, Debug)]
pub struct CartesianFourthOrder {
    v_max: f64,
    a_max: f64,
    start: na::Isometry3<f64>,
    end: na::Isometry3<f64>,
    profile: ScalarFourthOrderProfile,
    total: Duration,
    planned: bool,
}

impl CartesianFourthOrder {
    pub fn new(v_max: f64, a_max: f64) -> Self {
        Self {
            v_max,
            a_max,
            start: na::Isometry3::identity(),
            end: na::Isometry3::identity(),
            profile: ScalarFourthOrderProfile::Hold,
            total: Duration::ZERO,
            planned: false,
        }
    }

    /// Plans the move from `start` to `end` and returns its duration.
    pub fn plan(&mut self, start: na::Isometry3<f64>, end: na::Isometry3<f64>) -> Duration {
        self.start = start;
        self.end = end;
        let distance = (end.translation.vector - start.translation.vector).norm();
        // The profile runs over the unit path parameter; the bounds scale with
        // the distance. A zero distance makes them unbounded: an instant move.
        self.profile =
            ScalarFourthOrderProfile::plan(1.0, self.v_max / distance, self.a_max / distance);
        self.total = Duration::from_secs_f64(self.profile.total_time());
        self.planned = true;
        self.total
    }

    /// The pose at `time` and whether the move is complete.
    pub fn apply(&self, time: Duration) -> (na::Isometry3<f64>, bool) {
        if !self.planned || self.total.is_zero() || time >= self.total {
            return (self.end, true);
        }
        let progress = self.profile.sample(time.as_secs_f64());
        (self.start.lerp_slerp(&self.end, progress), false)
    }
}

/// A scalar fourth-order profile over a nonnegative distance: a cubic ramp up
/// to `v_max`, a cruise, and the mirrored ramp down.
#[derive(Clone, Copy, Debug)]
enum ScalarFourthOrderProfile {
    /// No distance to cover.
    Hold,
    /// Speed and acceleration unbounded: the move takes no time.
    Instant { delta: f64 },
    Curve {
        delta: f64,
        v_max: f64,
        t1: f64,
        t_min: f64,
    },
}

impl ScalarFourthOrderProfile {
    fn plan(delta: f64, v_max: f64, a_max: f64) -> Self {
        if delta < 1e-6 {
            return Self::Hold;
        }
        if !v_max.is_finite() && !a_max.is_finite() {
            return Self::Instant { delta };
        }
        // A short move never reaches v_max: the peak speed that still fits the
        // ramps into the distance.
        let mut v_max = v_max;
        if delta < 1.5 * v_max.powi(2) / a_max {
            v_max = (2. / 3. * delta * a_max).sqrt();
        }
        let t1 = 1.5 * v_max / a_max;
        Self::Curve { delta, v_max, t1, t_min: t1 + delta / v_max }
    }

    fn total_time(self) -> f64 {
        match self {
            Self::Hold | Self::Instant { .. } => 0.,
            Self::Curve { t_min, .. } => t_min,
        }
    }

    fn sample(self, t: f64) -> f64 {
        match self {
            Self::Hold => 0.,
            Self::Instant { delta } => delta,
            Self::Curve { delta, v_max, t1, t_min } => {
                if t < t1 {
                    (t / t1).powi(3) * (t1 - 0.5 * t) * v_max
                } else if t < t_min - t1 {
                    t1 * v_max / 2. + (t - t1) * v_max
                } else if t < t_min {
                    delta - ((t_min - t) / t1).powi(3) * (t1 - 0.5 * (t_min - t)) * v_max
                } else {
                    delta
                }
            }
        }
    }
}

#[cfg(any(target_os = "linux", target_os = "windows"))]
#[cfg(test)]
mod test {
    use std::f64::consts::PI;

    use super::*;

    /// Review finding F2: with an unbounded jerk and a move shorter than 1 rad
    /// the curve used to step to the target.
    #[test]
    fn unbounded_jerk_moves_smoothly_over_a_short_distance() {
        for jerk in [f64::INFINITY, f64::MAX] {
            let mut curve = JointSCurve::new([2.0], [10.0], [jerk]);
            let total = curve.plan([0.0], [0.5]);
            assert!(total > Duration::ZERO);
            let mut previous = 0.0;
            for k in 0..=(total.as_micros() / 1000) as u64 {
                let (q, _) = curve.apply(Duration::from_millis(k));
                assert!(q[0].is_finite());
                assert!((q[0] - previous).abs() <= 2.0 * 0.001 + 1e-9, "t = {k} ms");
                previous = q[0];
            }
            assert_eq!(curve.apply(total).0, [0.5]);
        }
    }

    #[test]
    fn unbounded_acceleration_moves_at_constant_speed_or_at_once() {
        let mut curve = JointSCurve::new([2.0], [f64::INFINITY], [f64::INFINITY]);
        assert_eq!(curve.plan([0.0], [0.5]), Duration::from_millis(250));
        assert!((curve.apply(Duration::from_millis(100)).0[0] - 0.2).abs() < 1e-12);
        let mut instant = JointSCurve::new([f64::INFINITY], [f64::INFINITY], [f64::INFINITY]);
        assert_eq!(instant.plan([0.0], [0.5]), Duration::ZERO);
        assert_eq!(instant.apply(Duration::ZERO), ([0.5], true));
        let profile = ScalarSCurveProfile::plan(0.5, 2.0, f64::INFINITY, f64::INFINITY);
        assert_eq!((profile.total_time(), profile.sample(1.0)), (0.25, 0.5));
    }

    /// A move too short to reach v_max peaks at sqrt(delta * a_max) and stays
    /// continuous where acceleration turns into braking.
    #[test]
    fn a_short_trapezoid_is_continuous() {
        let profile = ScalarSCurveProfile::plan(1.0, 3.5, 7.0, f64::INFINITY);
        let total = profile.total_time();
        assert!((total - 2.0 * (1.0f64 / 7.0).sqrt()).abs() < 1e-12);
        let mid = Duration::from_secs_f64(total / 2.0);
        // Within the nanosecond resolution of `Duration`.
        assert!((profile.sample(mid.as_secs_f64()) - 0.5).abs() < 1e-6);
        let mut curve = JointSCurve::new([3.5], [7.0], [f64::INFINITY]);
        let total = curve.plan([0.0], [1.0]);
        let mut previous = 0.0;
        for k in 0..=total.as_millis() as u64 {
            let q = curve.apply(Duration::from_millis(k)).0[0];
            assert!((q - previous).abs() < 0.01, "t = {k} ms");
            previous = q;
        }
    }

    #[test]
    fn test_joint_linear() {
        let start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let end = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let v_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let mut curve = JointLinear::new(v_max);
        curve.plan(start, end);

        let t = Duration::from_secs_f64(0.5);
        let (result, _) = curve.apply(t);
        assert_eq!(result, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn test_cartesian_quat_linear() {
        let start = na::Isometry3::identity();
        let end = na::Isometry3::from_parts(
            na::Translation3::new(1.0, 1.0, 1.0),
            na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        );
        let mut curve = CartesianLinear::new(1.0);
        curve.plan(start, end);

        for i in 0..210 {
            let t = Duration::from_secs_f64(i as f64 / 100.);
            let (result, _) = curve.apply(t);
            println!("time: {} | {:?}", i as f64 / 100., result);
        }
    }

    #[test]
    fn test_joint_trapezoid() {
        let start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let end = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let v_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let a_max = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let mut curve = JointTrapezoid::new(v_max, a_max);
        curve.plan(start, end);

        for i in 0..210 {
            let t = Duration::from_secs_f64(i as f64 / 100.);
            let (result, _) = curve.apply(t);
            println!("time: {} | {:?}", i as f64 / 100., result);
        }
    }

    #[test]
    fn test_joint_s_curve_struct_endpoints() {
        let start = [-0.2, -0.77, -0.36, -2.42, 0.00, 2.57, 0.77];
        let end = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        let v_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
        let a_max = [15., 7.5, 10., 12.5, 15., 20., 20.];
        let j_max = [100., 100., 100., 100., 100., 100., 100.];

        let mut curve = JointSCurve::new(v_max, a_max, j_max);
        let total = curve.plan(start, end);

        let (first, first_done) = curve.apply(Duration::ZERO);
        assert_eq!(first, start);
        assert!(!first_done);

        let (last, last_done) = curve.apply(total);
        assert_eq!(last, end);
        assert!(last_done);
    }

    #[test]
    fn test_joint_s_curve_struct_keeps_joint_line() {
        let start = [0.0, -0.3, 0.7];
        let end = [1.0, 0.1, -0.1];
        let v_max = [1.0, 0.8, 1.2];
        let a_max = [2.0, 1.5, 2.5];
        let j_max = [8.0, 7.0, 9.0];

        let mut curve = JointSCurve::new(v_max, a_max, j_max);
        let total = curve.plan(start, end);
        let (mid, done) = curve.apply(total / 2);

        assert!(!done);
        let progress = (mid[0] - start[0]) / (end[0] - start[0]);
        for i in 1..3 {
            let joint_progress = (mid[i] - start[i]) / (end[i] - start[i]);
            assert!((joint_progress - progress).abs() < 1e-12);
        }
    }

    use plotters::prelude::*;

    /// The plotting tests draw into the system temporary directory, not the
    /// source tree.
    fn plot_path() -> std::path::PathBuf {
        std::env::temp_dir().join("roplat_trajectory_plot.svg")
    }

    #[test]
    fn test_joint_s_curve() {
        let start = [-0.2, -0.77, -0.36, -2.42, 0.00, 2.57, 0.77];
        let end = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        let v_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
        let a_max = [15., 7.5, 10., 12.5, 15., 20., 20.];
        let j_max = [100., 100., 100., 100., 100., 100., 100.];
        let mut curve = JointSCurve::new(v_max, a_max, j_max);
        curve.plan(start, end);
        let f = |t: Duration| curve.apply(t).0;
        let f_dot = |t: Duration| {
            let dt: Duration = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            f(t_dot)
                .iter()
                .zip(f(t).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };
        let f_ddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            f_dot(t_ddot)
                .iter()
                .zip(f_dot(t_dot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };
        let f_dddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            let t_dddot = t_ddot + dt;
            f_ddot(t_dddot)
                .iter()
                .zip(f_ddot(t_ddot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };

        let path = plot_path();
        let root = plotters::prelude::SVGBackend::new(&path, (640, 480)).into_drawing_area();
        root.fill(&plotters::prelude::WHITE).unwrap();

        let mut chart = plotters::prelude::ChartBuilder::on(&root)
            .caption("S-Curve", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(0.0..1.0, -5.0..5.0)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        for j in 0..7 {
            let mut data = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f(Duration::from_secs_f64(t));
                data.push((t, y[j]));
            }

            let mut data_dot = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f_dot(Duration::from_secs_f64(t));
                data_dot.push((t, y[j]));
            }

            let mut data_ddot = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f_ddot(Duration::from_secs_f64(t));
                data_ddot.push((t, y[j]));
            }

            let mut data_dddot = Vec::new();
            for i in 0..1000 {
                let t = i as f64 / 1000.0;
                let y = f_dddot(Duration::from_secs_f64(t));
                data_dddot.push((t, y[j]));
            }

            for i in 0..1000 {
                println!(
                    "time: {} | {}, {}, {}",
                    i as f64 / 1000.,
                    data[i].1,
                    data_dot[i].1,
                    data_ddot[i].1
                );
            }

            chart
                .draw_series(plotters::prelude::LineSeries::new(
                    data,
                    &plotters::prelude::RED,
                ))
                .unwrap();

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data_dot,
            //         &plotters::prelude::GREEN,
            //     ))
            //     .unwrap();

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data_ddot,
            //         &plotters::prelude::BLUE,
            //     ))
            //     .unwrap();
        }
    }

    #[test]
    fn test_s_curve() {
        let delta = 2.57 - PI / 2.;
        let v_max = 2.6100;
        let a_max = 20.;
        let j_max = 10000.;
        let (t, f) = s_curve(delta, v_max, a_max, j_max);
        println!("t: {}", t);

        let path = plot_path();
        let root = SVGBackend::new(&path, (640, 480)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let mut chart = ChartBuilder::on(&root)
            .caption("S-Curve", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(0.0..1.0, 0.0..2.)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        let mut data = Vec::new();
        for i in 0..1000 {
            let t = i as f64 / 100.0;
            let y = f(Duration::from_secs_f64(t));
            data.push((t, y));
        }

        chart.draw_series(LineSeries::new(data, &RED)).unwrap();
    }

    #[test]
    fn test_joint_simple_4th_curve() {
        let start = [-0.2, -0.77, -0.36, -2.42, 0.00, 2.57, 0.77];
        let end = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        let v_max = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
        let a_max = [15., 7.5, 10., 12.5, 15., 20., 20.];
        let mut curve = JointFourthOrder::new(v_max, a_max);
        curve.plan(start, end);
        let f = |t: Duration| curve.apply(t).0;
        let f_dot = |t: Duration| {
            let dt: Duration = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            f(t_dot)
                .iter()
                .zip(f(t).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };
        let f_ddot = |t: Duration| {
            let dt = Duration::from_secs_f64(0.001);
            let t_dot = t + dt;
            let t_ddot = t_dot + dt;
            f_dot(t_ddot)
                .iter()
                .zip(f_dot(t_dot).iter())
                .map(|(a, b)| (a - b) / dt.as_secs_f64())
                .collect::<Vec<f64>>()
        };

        for i in 0..210 {
            let t = Duration::from_secs_f64(i as f64 / 100.);
            let result = f(t);
            println!("time: {} | {:?}", i as f64 / 100., result);
        }

        let path = plot_path();
        let root = plotters::prelude::SVGBackend::new(&path, (640, 480)).into_drawing_area();
        root.fill(&plotters::prelude::WHITE).unwrap();

        let mut chart = plotters::prelude::ChartBuilder::on(&root)
            .caption("Simple 4th Curve", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(0.0..1.0, -10.0..10.0)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        for j in 0..7 {
            let mut data = Vec::new();
            for i in 0..2000 {
                let t = i as f64 / 1000.0;
                let y = f(Duration::from_secs_f64(t));
                data.push((t, y[j]));
            }

            let mut data_dot = Vec::new();
            for i in 0..2000 {
                let t = i as f64 / 1000.0;
                let y = f_dot(Duration::from_secs_f64(t));
                data_dot.push((t, y[j]));
            }

            let mut data_ddot = Vec::new();
            for i in 0..2000 {
                let t = i as f64 / 1000.0;
                let y = f_ddot(Duration::from_secs_f64(t));
                data_ddot.push((t, y[j]));
            }

            // for i in 0..1000 {
            //     println!(
            //         "time: {} | {}, {}",
            //         i as f64 / 1000.,
            //         data[i].1,
            //         data_dot[i].1
            //     );
            // }

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data,
            //         &plotters::prelude::RED,
            //     ))
            //     .unwrap();

            // chart
            //     .draw_series(plotters::prelude::LineSeries::new(
            //         data_dot,
            //         &plotters::prelude::GREEN,
            //     ))
            //     .unwrap();

            chart
                .draw_series(plotters::prelude::LineSeries::new(
                    data_ddot,
                    &plotters::prelude::BLUE,
                ))
                .unwrap();
        }
    }
}
