//! Time scaling of geometric paths: time-optimal parameterization of joint
//! paths through COPP, and scalar time scaling of Cartesian paths through the
//! caller's inverse kinematics.
//!
//! Every planner takes the joint bounds as [`JointLimits`] and the control
//! period as a [`Duration`], and samples its result at that period. A speed
//! fraction is applied by the caller before planning (roplat_behavior's
//! `MotionLimits::scaled_joint` does). An unbounded entry is `f64::INFINITY`:
//! the planners need finite velocity and acceleration bounds and refuse
//! otherwise, while an infinite jerk bound leaves jerk unconstrained.

use std::{fmt::Display, time::Duration};

use nalgebra as na;
use roplat_msgs::{CartesianLimits, JointLimits, Pose};

use crate::path_generate::ScalarSCurveProfile;

/// Number of `s` samples used to discretise the geometric path for copp.
pub const COPP_PATH_SAMPLES: usize = 1001;

/// Default number of IK samples used to discretise a Cartesian geometric path.
pub const CARTESIAN_TIME_SCALE_PATH_SAMPLES: usize = 201;

/// Why time scaling failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum TimeScalingError {
    /// An argument is outside what the planner accepts.
    #[error("invalid time-scaling input: {0}")]
    Invalid(String),
    /// The path or its derivatives could not be evaluated.
    #[error("path evaluation failed: {0}")]
    Path(String),
    /// The inverse kinematics failed or returned a non-finite solution.
    #[error("inverse kinematics failed at s = {s}: {reason}")]
    InverseKinematics { s: f64, reason: String },
    /// COPP found no profile.
    #[error("copp planning failed: {0}")]
    Copp(String),
    /// The Cartesian time scaling kept violating a joint bound.
    #[error("joint limits still violated after {retries} retries: {last}")]
    LimitsViolated { retries: usize, last: String },
}

type TimeScalingResult<T> = Result<T, TimeScalingError>;

/// Bounds on the derivatives of the path parameter `s` in `[0, 1]` of a
/// geometric path: ds/dt, d²s/dt² and d³s/dt³. Unbounded unless set.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathBounds {
    pub velocity: f64,
    pub acceleration: f64,
    pub jerk: f64,
}

impl PathBounds {
    /// No bound on the path parameter.
    pub const UNBOUNDED: Self = Self {
        velocity: f64::INFINITY,
        acceleration: f64::INFINITY,
        jerk: f64::INFINITY,
    };

    /// The path bounds that keep a straight line with slerped orientation from
    /// `start` to `target` within `limits`: along a line the tip's speed,
    /// acceleration and jerk are the distance (or the rotation angle) times
    /// those of `s`.
    pub fn for_line(start: &Pose, target: &Pose, limits: &CartesianLimits) -> Self {
        let (a, b) = (start.quat(), target.quat());
        let distance = (b.translation.vector - a.translation.vector).norm();
        let angle = a.rotation.angle_to(&b.rotation);
        let bound = |linear: f64, angular: f64| {
            let by_distance = if distance > 0.0 {
                linear / distance
            } else {
                f64::INFINITY
            };
            let by_angle = if angle > 0.0 {
                angular / angle
            } else {
                f64::INFINITY
            };
            by_distance.min(by_angle)
        };
        Self {
            velocity: bound(limits.linear_velocity_max, limits.angular_velocity_max),
            acceleration: bound(
                limits.linear_acceleration_max,
                limits.angular_acceleration_max,
            ),
            jerk: bound(limits.linear_jerk_max, limits.angular_jerk_max),
        }
    }

    fn min(self, other: Self) -> Self {
        Self {
            velocity: self.velocity.min(other.velocity),
            acceleration: self.acceleration.min(other.acceleration),
            jerk: self.jerk.min(other.jerk),
        }
    }
}

impl Default for PathBounds {
    fn default() -> Self {
        Self::UNBOUNDED
    }
}

/// Options for Cartesian path time scaling.
///
/// The planner first samples IK along a geometric Cartesian path `x(s)`, then
/// generates a scalar S-curve `s(t)`. The final dense joint trajectory is
/// validated against joint velocity and acceleration limits. Jerk validation is
/// optional because the sampled IK path is interpolated and therefore is not
/// guaranteed to be globally C3-smooth.
#[derive(Debug, Clone, Copy)]
pub struct CartesianTimeScalingOptions {
    /// Number of IK samples over `s in [0, 1]`. Must be at least 5.
    pub path_samples: usize,
    /// Extra conservative factor applied to the initial scalar S-curve bounds.
    pub safety_scale: f64,
    /// Bound shrink factor used when validation fails.
    pub retry_scale: f64,
    /// Maximum validation / shrink retries.
    pub max_retries: usize,
    /// Whether to validate jerk with finite differences on the final trajectory.
    pub validate_jerk: bool,
    /// Bounds on the path parameter itself, on top of the joint bounds.
    pub path_bounds: PathBounds,
}

impl Default for CartesianTimeScalingOptions {
    fn default() -> Self {
        Self {
            path_samples: CARTESIAN_TIME_SCALE_PATH_SAMPLES,
            safety_scale: 0.8,
            retry_scale: 0.8,
            max_retries: 16,
            validate_jerk: false,
            path_bounds: PathBounds::UNBOUNDED,
        }
    }
}

/// Run copp's TOPP2-RA → TOPP3-SOCP (2-iteration SCP) pipeline on a discretised
/// joint-space path and return the `s(t)` grid sampled every `period`.
pub fn plan_s_t_via_copp<const N: usize>(
    q_grid: &na::DMatrix<f64>,
    dq_grid: &na::DMatrix<f64>,
    ddq_grid: &na::DMatrix<f64>,
    dddq_grid: Option<&na::DMatrix<f64>>,
    limits: &JointLimits<N>,
    period: Duration,
) -> TimeScalingResult<Vec<f64>> {
    use copp::InterpolationMode;
    use copp::robot::Robot;
    use copp::solver::topp2_ra::{ReachSet2OptionsBuilder, Topp2ProblemBuilder, topp2_ra};
    use copp::solver::topp3_socp::{
        ClarabelOptionsBuilder, Topp3ProblemBuilder, s_to_t_topp3, t_to_s_topp3, topp3_socp,
    };

    check_period(period)?;
    check_joint_bounds(limits, true)?;

    fn map_err<E: Display>(e: E) -> TimeScalingError {
        TimeScalingError::Copp(e.to_string())
    }

    let n = COPP_PATH_SAMPLES;
    let s: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();

    let mut robot = Robot::with_capacity(N, n);
    robot.with_s(s.as_slice()).map_err(map_err)?;
    robot
        .with_q(
            &q_grid.as_view(),
            &dq_grid.as_view(),
            &ddq_grid.as_view(),
            dddq_grid.map(|m| m.as_view()).as_ref(),
            0,
        )
        .map_err(map_err)?;
    robot
        .with_axial_velocity(
            (limits.dq_max.as_slice(), n),
            (limits.dq_max.map(|v| -v).as_slice(), n),
            0,
        )
        .map_err(map_err)?;
    robot
        .with_axial_acceleration(
            (limits.ddq_max.as_slice(), n),
            (limits.ddq_max.map(|v| -v).as_slice(), n),
            0,
        )
        .map_err(map_err)?;
    // copp leaves jerk unconstrained unless told otherwise.
    if limits.dddq_max.iter().any(|j| j.is_finite()) {
        robot
            .with_axial_jerk(
                (limits.dddq_max.as_slice(), n),
                (limits.dddq_max.map(|v| -v).as_slice(), n),
                0,
            )
            .map_err(map_err)?;
    }

    let idx_s_interval = (0, n - 1);
    let a_boundary = (0.0, 0.0);
    let a_ra0 = {
        let prob = Topp2ProblemBuilder::new(&robot, idx_s_interval, a_boundary)
            .build()
            .map_err(map_err)?;
        let opts = ReachSet2OptionsBuilder::new().build().map_err(map_err)?;
        topp2_ra(&prob, &opts).map_err(map_err)?
    };
    let a_ra0: Vec<f64> = a_ra0.into_iter().map(|a| a.max(0.0)).collect();
    robot
        .constraints
        .amax_substitute(&a_ra0, 0)
        .map_err(map_err)?;

    let opts_socp = ClarabelOptionsBuilder::new()
        .allow_almost_solved(true)
        .allow_insufficient_progress(true)
        .allow_max_iterations(true)
        .build()
        .map_err(map_err)?;

    let profile_qp1 = {
        let prob =
            Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_ra0, (0.0, 0.0), (0.0, 0.0))
                .build_with_linearization()
                .map_err(map_err)?;
        topp3_socp(&prob, &opts_socp).map_err(map_err)?
    };
    let a_qp1: Vec<f64> = profile_qp1.a.iter().map(|&a| a.max(0.0)).collect();
    let profile_qp2 = {
        let prob =
            Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_qp1, (0.0, 0.0), (0.0, 0.0))
                .build_with_linearization()
                .map_err(map_err)?;
        match topp3_socp(&prob, &opts_socp) {
            Ok(res) => res,
            Err(_) => {
                let prob = Topp3ProblemBuilder::new(
                    &mut robot,
                    idx_s_interval.0,
                    &a_ra0,
                    (0.0, 0.0),
                    (0.0, 0.0),
                )
                .build_with_linearization()
                .map_err(map_err)?;
                topp3_socp(&prob, &opts_socp).map_err(map_err)?
            }
        }
    };

    let (_t_final, t_s) = s_to_t_topp3(&s, profile_qp2.as_parts(), 0.0).map_err(map_err)?;
    let s_t = t_to_s_topp3(
        &s,
        profile_qp2.as_parts(),
        &t_s,
        InterpolationMode::UniformTimeGrid(0.0, period.as_secs_f64(), true),
    )
    .map_err(map_err)?;
    if s_t.is_empty() {
        return Err(TimeScalingError::Copp(
            "t_to_s_topp3 produced an empty time grid".to_string(),
        ));
    }
    Ok(s_t)
}

/// Plan a joint trajectory through discrete `waypoints` via copp, sampled
/// every `period`.
pub fn plan_waypoints_traj_via_copp<const N: usize>(
    waypoints: &[[f64; N]],
    limits: &JointLimits<N>,
    period: Duration,
) -> TimeScalingResult<Vec<[f64; N]>> {
    use copp::path::{Path, SplineConfig};

    if waypoints.len() < 2 {
        return Err(TimeScalingError::Invalid(
            "joint waypoint planning requires at least 2 waypoints".to_string(),
        ));
    }

    let n_pts = waypoints.len();
    let wp_mat = na::DMatrix::<f64>::from_fn(N, n_pts, |i, j| waypoints[j][i]);
    let path = Path::from_waypoints(&wp_mat, SplineConfig::default())
        .map_err(|e| TimeScalingError::Path(format!("copp Path::from_waypoints: {e}")))?;

    let n = COPP_PATH_SAMPLES;
    let s: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();
    let derivs = path
        .evaluate_up_to_3rd(&s)
        .map_err(|e| TimeScalingError::Path(format!("path.evaluate_up_to_3rd: {e}")))?;

    let s_t = plan_s_t_via_copp::<N>(
        &derivs.q,
        derivs.dq.as_ref().unwrap(),
        derivs.ddq.as_ref().unwrap(),
        derivs.dddq.as_ref(),
        limits,
        period,
    )?;

    let q_t = path
        .evaluate_q(&s_t)
        .map_err(|e| TimeScalingError::Path(format!("path.evaluate_q: {e}")))?;
    Ok(q_grid_to_traj::<N>(&q_t.q))
}

/// Plan a joint trajectory along a continuous path `q(s)`, `s` in `[0, 1]`,
/// via copp, sampled every `period`.
pub fn plan_path_traj_via_copp<F, const N: usize>(
    path_fn: &F,
    limits: &JointLimits<N>,
    period: Duration,
) -> TimeScalingResult<Vec<[f64; N]>>
where
    F: Fn(f64) -> Option<[f64; N]>,
{
    let n = COPP_PATH_SAMPLES;
    let s_grid: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();
    let (q_grid, dq_grid, ddq_grid, dddq_grid) =
        finite_difference_path_derivatives(path_fn, &s_grid)?;

    let s_t = plan_s_t_via_copp::<N>(
        &q_grid,
        &dq_grid,
        &ddq_grid,
        Some(&dddq_grid),
        limits,
        period,
    )?;

    let mut traj = Vec::with_capacity(s_t.len());
    for &s in &s_t {
        match path_fn(s) {
            Some(q) => traj.push(q),
            None => {
                return Err(TimeScalingError::Path(format!(
                    "the path returned None at s={s} during final resampling"
                )));
            }
        }
    }
    Ok(traj)
}

/// Plan a Cartesian geometric path without copp by scalar time scaling.
///
/// The path is a purely geometric curve `x(s), s in [0, 1]`. The caller supplies
/// an IK function that receives `(pose, seed)` and returns a joint solution. IK
/// is sampled once along the path with the previous solution as the next seed,
/// then a scalar S-curve `s(t)` is generated and shrunk until the dense joint
/// trajectory, sampled every `period`, satisfies the joint velocity and
/// acceleration bounds (and jerk, if [`validate_jerk`]).
///
/// The joint bounds and [`path_bounds`] together must bound the speed and
/// acceleration of `s` while the joints move.
///
/// This is intentionally not time-optimal. It is a compact, deterministic
/// planner for drivers that want a Cartesian straight-line style command while
/// still executing through joint-space constraints.
///
/// [`validate_jerk`]: CartesianTimeScalingOptions::validate_jerk
/// [`path_bounds`]: CartesianTimeScalingOptions::path_bounds
pub fn plan_cartesian_path_traj_via_time_scaling<F, IK, E, const N: usize>(
    path_fn: F,
    q_start: [f64; N],
    mut ik: IK,
    limits: &JointLimits<N>,
    period: Duration,
    options: CartesianTimeScalingOptions,
) -> TimeScalingResult<Vec<[f64; N]>>
where
    F: Fn(f64) -> Pose,
    IK: FnMut(Pose, [f64; N]) -> Result<[f64; N], E>,
    E: Display,
{
    check_period(period)?;
    check_options(&options)?;
    check_joint_bounds(limits, false)?;

    let s_grid = unit_grid(options.path_samples);
    let mut q_samples = Vec::with_capacity(s_grid.len());
    let mut seed = q_start;
    for &s in &s_grid {
        let q = ik(path_fn(s), seed)
            .map_err(|e| TimeScalingError::InverseKinematics { s, reason: e.to_string() })?;
        if !q.iter().all(|v| v.is_finite()) {
            return Err(TimeScalingError::InverseKinematics {
                s,
                reason: "non-finite joint values".to_string(),
            });
        }
        q_samples.push(q);
        seed = q;
    }

    if joint_path_is_stationary(&q_samples) {
        return Ok(vec![q_start]);
    }

    let (_q_grid, dq_ds, ddq_ds, dddq_ds) =
        finite_difference_sample_derivatives::<N>(&q_samples, &s_grid)?;
    let bounds = scalar_s_curve_bounds::<N>(&dq_ds, &ddq_ds, &dddq_ds, limits, &options)?;
    let (mut s_vel, mut s_acc, mut s_jerk) = (bounds.velocity, bounds.acceleration, bounds.jerk);
    let dt = period.as_secs_f64();
    let mut last = String::new();

    for _attempt in 0..=options.max_retries {
        let profile = ScalarSCurveProfile::plan(1.0, s_vel, s_acc, s_jerk);
        let total = Duration::from_secs_f64(profile.total_time());
        let s_curve = |time: Duration| [profile.sample(time.as_secs_f64())];
        let traj = sample_time_scaled_joint_path(&q_samples, &dq_ds, s_curve, total, dt);

        match joint_traj_limit_violation(&traj, dt, limits, options.validate_jerk) {
            None => return Ok(traj),
            Some(err) => last = err,
        }

        s_vel *= options.retry_scale;
        s_acc *= options.retry_scale * options.retry_scale;
        s_jerk *= options.retry_scale * options.retry_scale * options.retry_scale;
    }

    Err(TimeScalingError::LimitsViolated { retries: options.max_retries, last })
}

/// Plan a straight line with slerped orientation from `start` to `target`,
/// within the joint bounds and the end effector's `cartesian` bounds, by
/// [`plan_cartesian_path_traj_via_time_scaling`].
#[allow(
    clippy::too_many_arguments,
    reason = "start, target, seed, inverse kinematics, joint and Cartesian limits, period and options are the independent inputs of one planning call"
)]
pub fn plan_cartesian_line_traj_via_time_scaling<IK, E, const N: usize>(
    start: Pose,
    target: Pose,
    q_start: [f64; N],
    ik: IK,
    limits: &JointLimits<N>,
    cartesian: &CartesianLimits,
    period: Duration,
    options: CartesianTimeScalingOptions,
) -> TimeScalingResult<Vec<[f64; N]>>
where
    IK: FnMut(Pose, [f64; N]) -> Result<[f64; N], E>,
    E: Display,
{
    let options = CartesianTimeScalingOptions {
        path_bounds: options
            .path_bounds
            .min(PathBounds::for_line(&start, &target, cartesian)),
        ..options
    };
    let (start, target) = (start.quat(), target.quat());
    plan_cartesian_path_traj_via_time_scaling(
        move |s| Pose::from_isometry(start.lerp_slerp(&target, s.clamp(0.0, 1.0))),
        q_start,
        ik,
        limits,
        period,
        options,
    )
}

type PathDerivativeMatrices = (
    na::DMatrix<f64>,
    na::DMatrix<f64>,
    na::DMatrix<f64>,
    na::DMatrix<f64>,
);

fn finite_difference_path_derivatives<F, const N: usize>(
    path_fn: &F,
    s_grid: &[f64],
) -> TimeScalingResult<PathDerivativeMatrices>
where
    F: Fn(f64) -> Option<[f64; N]>,
{
    if s_grid.len() < 5 {
        return Err(TimeScalingError::Invalid(
            "finite-difference planning requires at least 5 path samples".to_string(),
        ));
    }

    let mut q_samples = Vec::with_capacity(s_grid.len());
    for &s in s_grid {
        match path_fn(s) {
            Some(q) if q.iter().all(|v| v.is_finite()) => q_samples.push(q),
            Some(_) => {
                return Err(TimeScalingError::Path(format!(
                    "the path returned non-finite joint values at s={s}"
                )));
            }
            None => {
                return Err(TimeScalingError::Path(format!(
                    "the path returned None at s={s}; it must be defined on [0, 1]"
                )));
            }
        }
    }

    finite_difference_sample_derivatives::<N>(&q_samples, s_grid)
}

fn check_period(period: Duration) -> TimeScalingResult<()> {
    if period.is_zero() {
        return Err(TimeScalingError::Invalid(
            "the control period is zero".to_string(),
        ));
    }
    Ok(())
}

/// Velocity, acceleration and jerk bounds positive (NaN refuses); velocity and
/// acceleration also finite when `finite_rates`.
fn check_joint_bounds<const N: usize>(
    limits: &JointLimits<N>,
    finite_rates: bool,
) -> TimeScalingResult<()> {
    for axis in 0..N {
        for (what, bound, must_be_finite) in [
            ("dq_max", limits.dq_max[axis], finite_rates),
            ("ddq_max", limits.ddq_max[axis], finite_rates),
            ("dddq_max", limits.dddq_max[axis], false),
        ] {
            if bound.is_nan() || bound <= 0.0 || (must_be_finite && !bound.is_finite()) {
                let need = if must_be_finite {
                    "positive and finite"
                } else {
                    "positive"
                };
                return Err(TimeScalingError::Invalid(format!(
                    "joint {axis}: {what} is {bound}; it must be {need}"
                )));
            }
        }
    }
    Ok(())
}

fn check_options(options: &CartesianTimeScalingOptions) -> TimeScalingResult<()> {
    if options.path_samples < 5 {
        return Err(TimeScalingError::Invalid(format!(
            "cartesian time scaling requires at least 5 path samples, got {}",
            options.path_samples
        )));
    }
    if !(options.safety_scale.is_finite() && options.safety_scale > 0.0) {
        return Err(TimeScalingError::Invalid(format!(
            "safety_scale must be positive finite, got {}",
            options.safety_scale
        )));
    }
    if !(options.retry_scale.is_finite() && options.retry_scale > 0.0 && options.retry_scale < 1.0)
    {
        return Err(TimeScalingError::Invalid(format!(
            "retry_scale must be in (0, 1), got {}",
            options.retry_scale
        )));
    }
    let p = options.path_bounds;
    if [p.velocity, p.acceleration, p.jerk]
        .iter()
        .any(|b| b.is_nan() || *b <= 0.0)
    {
        return Err(TimeScalingError::Invalid(format!(
            "path bounds must be positive, got {p:?}"
        )));
    }
    Ok(())
}

fn unit_grid(n: usize) -> Vec<f64> {
    (0..n).map(|i| i as f64 / (n - 1) as f64).collect()
}

fn joint_path_is_stationary<const N: usize>(q_samples: &[[f64; N]]) -> bool {
    let Some(first) = q_samples.first() else {
        return true;
    };
    q_samples
        .iter()
        .all(|q| q.iter().zip(first).all(|(a, b)| (*a - *b).abs() < 1e-10))
}

fn finite_difference_sample_derivatives<const N: usize>(
    q_samples: &[[f64; N]],
    s_grid: &[f64],
) -> TimeScalingResult<PathDerivativeMatrices> {
    if q_samples.len() != s_grid.len() || q_samples.len() < 5 {
        return Err(TimeScalingError::Invalid(
            "joint sample derivative estimation requires matching grids with at least 5 samples"
                .to_string(),
        ));
    }

    let n = s_grid.len();
    let q = na::DMatrix::<f64>::from_fn(N, n, |i, j| q_samples[j][i]);
    let mut dq = na::DMatrix::<f64>::zeros(N, n);
    let mut ddq = na::DMatrix::<f64>::zeros(N, n);
    let mut dddq = na::DMatrix::<f64>::zeros(N, n);

    for j in 0..n {
        let start = j.saturating_sub(2).min(n - 5);
        let indices = [start, start + 1, start + 2, start + 3, start + 4];
        let offsets = indices.map(|idx| s_grid[idx] - s_grid[j]);
        let weights_d1 = finite_difference_weights(&offsets, 1);
        let weights_d2 = finite_difference_weights(&offsets, 2);
        let weights_d3 = finite_difference_weights(&offsets, 3);

        for axis in 0..N {
            for (k, &idx) in indices.iter().enumerate() {
                let q_value = q_samples[idx][axis];
                dq[(axis, j)] += weights_d1[k] * q_value;
                ddq[(axis, j)] += weights_d2[k] * q_value;
                dddq[(axis, j)] += weights_d3[k] * q_value;
            }
        }
    }

    Ok((q, dq, ddq, dddq))
}

/// Initial bounds on `s`: the tightest of the joint bounds divided by the
/// path's derivatives and the caller's path bounds, times the safety scale.
fn scalar_s_curve_bounds<const N: usize>(
    dq_ds: &na::DMatrix<f64>,
    ddq_ds: &na::DMatrix<f64>,
    dddq_ds: &na::DMatrix<f64>,
    limits: &JointLimits<N>,
    options: &CartesianTimeScalingOptions,
) -> TimeScalingResult<PathBounds> {
    let eps = 1e-9;
    let mut bounds = PathBounds::UNBOUNDED;
    let mut moving = false;

    for axis in 0..N {
        let mut q1 = 0.0f64;
        let mut q2 = 0.0f64;
        let mut q3 = 0.0f64;
        for sample in 0..dq_ds.ncols() {
            q1 = q1.max(dq_ds[(axis, sample)].abs());
            q2 = q2.max(ddq_ds[(axis, sample)].abs());
            q3 = q3.max(dddq_ds[(axis, sample)].abs());
        }

        if q1 > eps {
            moving = true;
            bounds.velocity = bounds.velocity.min(limits.dq_max[axis] / q1);
            bounds.acceleration = bounds.acceleration.min(limits.ddq_max[axis] / q1);
            bounds.jerk = bounds.jerk.min(limits.dddq_max[axis] / q1);
        }
        if q2 > eps {
            bounds.velocity = bounds.velocity.min((limits.ddq_max[axis] / q2).sqrt());
        }
        if q3 > eps {
            bounds.velocity = bounds.velocity.min((limits.dddq_max[axis] / q3).cbrt());
        }
    }

    let mut bounds = bounds.min(options.path_bounds);
    if !moving {
        // The joints barely move: no joint bound applies, so any finite speed of
        // `s` keeps them within their limits.
        for bound in [
            &mut bounds.velocity,
            &mut bounds.acceleration,
            &mut bounds.jerk,
        ] {
            if !bound.is_finite() {
                *bound = 1.0;
            }
        }
    } else if !(bounds.velocity.is_finite() && bounds.acceleration.is_finite()) {
        return Err(TimeScalingError::Invalid(format!(
            "the speed and acceleration along the path are unbounded ({bounds:?}); \
             bound the joints' dq_max and ddq_max or set path_bounds"
        )));
    }

    Ok(PathBounds {
        velocity: bounds.velocity * options.safety_scale,
        acceleration: bounds.acceleration * options.safety_scale,
        jerk: bounds.jerk * options.safety_scale,
    })
}

fn sample_time_scaled_joint_path<const N: usize>(
    q_samples: &[[f64; N]],
    dq_ds: &na::DMatrix<f64>,
    s_curve: impl Fn(Duration) -> [f64; 1],
    total: Duration,
    dt: f64,
) -> Vec<[f64; N]> {
    let steps = (total.as_secs_f64() / dt).ceil() as usize;
    let mut traj = Vec::with_capacity(steps + 2);
    for step in 0..=steps {
        let t = (step as f64 * dt).min(total.as_secs_f64());
        let s = s_curve(Duration::from_secs_f64(t))[0].clamp(0.0, 1.0);
        traj.push(interpolate_joint_path(q_samples, dq_ds, s));
    }
    let end = *q_samples.last().unwrap();
    if traj.last().is_none_or(|q| {
        q.iter()
            .zip(end.iter())
            .any(|(a, b)| (*a - *b).abs() > 1e-10)
    }) {
        traj.push(end);
    }
    traj
}

fn interpolate_joint_path<const N: usize>(
    q_samples: &[[f64; N]],
    dq_ds: &na::DMatrix<f64>,
    s: f64,
) -> [f64; N] {
    if s <= 0.0 {
        return q_samples[0];
    }
    if s >= 1.0 {
        return *q_samples.last().unwrap();
    }

    let n = q_samples.len();
    let pos = s * (n - 1) as f64;
    let i0 = pos.floor() as usize;
    let i1 = (i0 + 1).min(n - 1);
    let u = pos - i0 as f64;
    let ds = 1.0 / (n - 1) as f64;

    let h00 = 2.0 * u.powi(3) - 3.0 * u.powi(2) + 1.0;
    let h10 = u.powi(3) - 2.0 * u.powi(2) + u;
    let h01 = -2.0 * u.powi(3) + 3.0 * u.powi(2);
    let h11 = u.powi(3) - u.powi(2);

    let mut q = [0.0; N];
    for axis in 0..N {
        q[axis] = h00 * q_samples[i0][axis]
            + h10 * ds * dq_ds[(axis, i0)]
            + h01 * q_samples[i1][axis]
            + h11 * ds * dq_ds[(axis, i1)];
    }
    q
}

/// The first finite-difference velocity, acceleration or (if `validate_jerk`)
/// jerk of `traj` beyond its bound. An infinite bound is never exceeded.
fn joint_traj_limit_violation<const N: usize>(
    traj: &[[f64; N]],
    dt: f64,
    limits: &JointLimits<N>,
    validate_jerk: bool,
) -> Option<String> {
    for step in 1..traj.len() {
        for axis in 0..N {
            let value = ((traj[step][axis] - traj[step - 1][axis]) / dt).abs();
            if value > limits.dq_max[axis] {
                return Some(format!(
                    "velocity axis {axis} at sample {step}: {value} > {}",
                    limits.dq_max[axis]
                ));
            }
        }
    }

    for step in 2..traj.len() {
        for axis in 0..N {
            let value = ((traj[step][axis] - 2.0 * traj[step - 1][axis] + traj[step - 2][axis])
                / (dt * dt))
                .abs();
            if value > limits.ddq_max[axis] {
                return Some(format!(
                    "acceleration axis {axis} at sample {step}: {value} > {}",
                    limits.ddq_max[axis]
                ));
            }
        }
    }

    if validate_jerk {
        for step in 3..traj.len() {
            for axis in 0..N {
                let value = ((traj[step][axis] - 3.0 * traj[step - 1][axis]
                    + 3.0 * traj[step - 2][axis]
                    - traj[step - 3][axis])
                    / (dt * dt * dt))
                    .abs();
                if value > limits.dddq_max[axis] {
                    return Some(format!(
                        "jerk axis {axis} at sample {step}: {value} > {}",
                        limits.dddq_max[axis]
                    ));
                }
            }
        }
    }

    None
}

fn finite_difference_weights(offsets: &[f64; 5], derivative_order: usize) -> [f64; 5] {
    let mut matrix = [[0.0f64; 6]; 5];
    for (row, values) in matrix.iter_mut().enumerate() {
        for (col, &offset) in offsets.iter().enumerate() {
            values[col] = offset.powi(row as i32);
        }
        values[5] = if row == derivative_order {
            factorial(derivative_order) as f64
        } else {
            0.0
        };
    }

    for pivot in 0..5 {
        let pivot_row = (pivot..5)
            .max_by(|&a, &b| matrix[a][pivot].abs().total_cmp(&matrix[b][pivot].abs()))
            .unwrap();
        matrix.swap(pivot, pivot_row);

        let pivot_value = matrix[pivot][pivot];
        for value in matrix[pivot].iter_mut().skip(pivot) {
            *value /= pivot_value;
        }

        for row in 0..5 {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            let pivot_values = matrix[pivot];
            for (value, pivot_value) in matrix[row].iter_mut().zip(pivot_values).skip(pivot) {
                *value -= factor * pivot_value;
            }
        }
    }

    matrix.map(|row| row[5])
}

fn factorial(n: usize) -> usize {
    (1..=n).product()
}

/// Convert a column-major `(N, M)` joint matrix to `Vec<[f64; N]>`.
pub fn q_grid_to_traj<const N: usize>(q: &na::DMatrix<f64>) -> Vec<[f64; N]> {
    let mut traj = Vec::with_capacity(q.ncols());
    for j in 0..q.ncols() {
        let mut joint = [0.0f64; N];
        for (i, slot) in joint.iter_mut().enumerate().take(N) {
            *slot = q[(i, j)];
        }
        traj.push(joint);
    }
    traj
}

#[cfg(test)]
mod tests {
    use super::*;

    const PERIOD: Duration = Duration::from_millis(1);

    fn one_joint(jerk: f64) -> JointLimits<1> {
        JointLimits {
            q_min: [-10.0],
            q_max: [10.0],
            dq_max: [1.0],
            ddq_max: [2.0],
            dddq_max: [jerk],
            ..JointLimits::UNBOUNDED
        }
    }

    fn options() -> CartesianTimeScalingOptions {
        CartesianTimeScalingOptions {
            path_samples: 21,
            safety_scale: 0.7,
            retry_scale: 0.8,
            max_retries: 8,
            ..Default::default()
        }
    }

    fn line(
        limits: &JointLimits<1>,
        cartesian: &CartesianLimits,
        options: CartesianTimeScalingOptions,
    ) -> TimeScalingResult<Vec<[f64; 1]>> {
        plan_cartesian_line_traj_via_time_scaling(
            Pose::from_position([0.0, 0.0, 0.0]),
            Pose::from_position([0.2, 0.0, 0.0]),
            [0.0],
            |pose: Pose, _seed| Ok::<_, String>([pose.position()[0]]),
            limits,
            cartesian,
            PERIOD,
            options,
        )
    }

    #[test]
    fn cartesian_line_time_scaling_generates_limited_joint_traj() {
        for jerk in [20.0, f64::INFINITY] {
            let limits = one_joint(jerk);
            let traj = line(&limits, &CartesianLimits::UNBOUNDED, options()).unwrap();

            assert!(traj.len() > 2);
            assert!((traj[0][0] - 0.0).abs() < 1e-9);
            assert!((traj.last().unwrap()[0] - 0.2).abs() < 1e-9);
            assert!(joint_traj_limit_violation(&traj, 0.001, &limits, false).is_none());
            // No step to the target: an infinite jerk bound plans a trapezoid.
            assert!(
                traj.windows(2)
                    .all(|w| (w[1][0] - w[0][0]).abs() <= 1.0 * 0.001 + 1e-12)
            );
        }
    }

    #[test]
    fn cartesian_bounds_slow_the_line() {
        let limits = one_joint(f64::INFINITY);
        let free = line(&limits, &CartesianLimits::UNBOUNDED, options()).unwrap();
        let cartesian = CartesianLimits { linear_velocity_max: 0.1, ..CartesianLimits::UNBOUNDED };
        let slow = line(&limits, &cartesian, options()).unwrap();
        assert!(slow.len() > free.len());
        let speed = slow
            .windows(2)
            .map(|w| (w[1][0] - w[0][0]).abs() / 0.001)
            .fold(0.0, f64::max);
        assert!(speed <= 0.1 + 1e-9, "{speed}");
    }

    #[test]
    fn unbounded_speed_along_a_moving_path_is_refused() {
        let limits = JointLimits::<1>::UNBOUNDED;
        let error = line(&limits, &CartesianLimits::UNBOUNDED, options()).unwrap_err();
        assert!(matches!(error, TimeScalingError::Invalid(_)), "{error}");
        // A Cartesian bound is enough.
        let cartesian = CartesianLimits {
            linear_velocity_max: 0.1,
            linear_acceleration_max: 0.5,
            ..CartesianLimits::UNBOUNDED
        };
        assert!(line(&limits, &cartesian, options()).is_ok());
    }

    #[test]
    fn copp_takes_an_unbounded_jerk_and_refuses_an_unbounded_speed() {
        let waypoints = [[0.0, 0.0], [0.5, -0.3], [1.0, 0.2]];
        let limits = JointLimits::<2> {
            dq_max: [1.0; 2],
            ddq_max: [2.0; 2],
            ..JointLimits::UNBOUNDED
        };
        let traj = plan_waypoints_traj_via_copp(&waypoints, &limits, PERIOD).unwrap();
        assert!(traj.iter().flatten().all(|v| v.is_finite()));
        let peak = traj
            .windows(2)
            .flat_map(|w| (0..2).map(move |i| (w[1][i] - w[0][i]).abs() / 0.001))
            .fold(0.0, f64::max);
        assert!(peak <= 1.0 + 1e-3, "{peak}");

        let unbounded = JointLimits::<2> { ddq_max: [2.0; 2], ..JointLimits::UNBOUNDED };
        let error = plan_waypoints_traj_via_copp(&waypoints, &unbounded, PERIOD).unwrap_err();
        assert!(matches!(error, TimeScalingError::Invalid(_)), "{error}");
    }

    #[test]
    fn line_bounds_follow_distance_and_angle() {
        let start = Pose::from_position([0.0, 0.0, 0.0]);
        let target = Pose::from_position([0.5, 0.0, 0.0]);
        let limits = CartesianLimits {
            linear_velocity_max: 0.25,
            angular_velocity_max: 1.0,
            ..CartesianLimits::UNBOUNDED
        };
        let bounds = PathBounds::for_line(&start, &target, &limits);
        assert_eq!(bounds.velocity, 0.5);
        assert_eq!(bounds.acceleration, f64::INFINITY);
    }
}
