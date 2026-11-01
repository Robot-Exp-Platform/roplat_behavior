//! Following a timed tool-pose trajectory on the Cartesian pose channel.
//!
//! [`CartesianTrajectoryFollower`] is the Cartesian counterpart of
//! `roplat_trajectory`'s `JointTrajectoryFollower`: every cycle it commands
//! the pose its trajectory has at the clock's time since the first cycle.
//! Between two waypoints the position moves on the straight line and the
//! orientation by spherical interpolation, at a constant rate; a smooth motion
//! comes from timing the waypoints well, for which [`cartesian_line`] gives a
//! straight move with a minimum-jerk profile sampled every control period, and
//! [`cartesian_arc`] a move along the circle through a via point;
//! [`cartesian_line_within`] and [`cartesian_arc_within`] take the shortest
//! duration that keeps the tool within [`CartesianLimits`], as Pilz's `LIN`
//! and `CIRC` take speed and acceleration limits.
//!
//! Tolerances refuse rather than correct, as the joint follower's do: a
//! measured flange pose farther from the trajectory than the path tolerance,
//! or a goal not reached within the goal time, is a
//! [`CartesianTrackingError`]. The command is a pose in whatever frame the
//! driver's `CartesianPoseControl` channel commands (usually the flange in the
//! base frame); the tolerances compare against `ArmState::flange`'s measured
//! pose, so give waypoints in that frame.
//!
//! ```
//! # use std::time::Duration;
//! # use nalgebra::{Isometry3, Translation3};
//! use roplat_behavior::msgs::ManualClock;
//! use roplat_control::{CartesianTrajectoryFollower, cartesian_line};
//!
//! let from = Isometry3::translation(0.4, 0.0, 0.3);
//! let to = Translation3::new(0.0, 0.0, -0.1) * from; // 10 cm straight down
//! let waypoints = cartesian_line(&from, &to, Duration::from_secs(1), Duration::from_millis(1));
//! let mut follower = CartesianTrajectoryFollower::<_, 7>::new(waypoints, ManualClock::simulation())
//!     .unwrap()
//!     .with_path_tolerance(0.01, 0.05);
//! // In a session, an error ends it:
//! // arm.control_with_flow::<CartesianPoseControl, _>(|state, dt| match follower.step_by(dt, &state) {
//! //     Ok(step) => ControlFlow::Continue(step),
//! //     Err(_) => ControlFlow::Break(()),
//! // })
//! # let _ = &mut follower;
//! ```
//!
//! With feature `roplat` the follower is also a roplat `Node` on the control
//! Yield `(ArmState, Duration)`, as `roplat_trajectory`'s joint follower is:
//! `follower?` in a `ControlRhythm` graph turns a refusal into `Break`, and
//! [`CartesianTrackingError::find`] recovers it from the drive's error.

use std::time::Duration;

use nalgebra as na;
use roplat_behavior::{
    ArmState,
    msgs::{Clock, ManualClock, Pose, Stamp},
};

/// A tool pose to be at `time_from_start` after the run starts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CartesianWaypoint {
    pub time_from_start: Duration,
    pub pose: na::Isometry3<f64>,
}

/// Why a Cartesian trajectory was rejected or its following stopped.
#[derive(Clone, Debug, PartialEq)]
pub enum CartesianTrackingError {
    /// A trajectory needs at least one waypoint.
    Empty,
    /// `time_from_start` must increase strictly.
    Timing { index: usize },
    /// The pose must be finite.
    NonFinite { index: usize },
    /// A tolerance is checked but the state has no measured flange pose.
    Unobserved,
    /// The clock went back behind the trajectory's start.
    ClockWentBack { nanos: i64 },
    /// The measured flange is farther from the trajectory than the path
    /// tolerance allows.
    PathTolerance {
        position: f64,
        rotation: f64,
        at: Duration,
    },
    /// The goal was not reached within the goal time.
    GoalTolerance {
        position: f64,
        rotation: f64,
        goal_time: Duration,
    },
}

impl std::fmt::Display for CartesianTrackingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => write!(f, "a trajectory needs at least one waypoint"),
            Self::Timing { index } => {
                write!(
                    f,
                    "waypoint {index}: time_from_start must increase strictly"
                )
            }
            Self::NonFinite { index } => write!(f, "waypoint {index}: the pose must be finite"),
            Self::Unobserved => {
                write!(
                    f,
                    "a tolerance is checked but the state has no measured flange pose"
                )
            }
            Self::ClockWentBack { nanos } => {
                write!(
                    f,
                    "the clock went back {nanos} ns behind the trajectory's start"
                )
            }
            Self::PathTolerance { position, rotation, at } => write!(
                f,
                "the flange is {position} m and {rotation} rad from the trajectory at {at:?}, beyond the path tolerance"
            ),
            Self::GoalTolerance { position, rotation, goal_time } => write!(
                f,
                "the flange is still {position} m and {rotation} rad from the goal {goal_time:?} after the trajectory ended"
            ),
        }
    }
}

impl std::error::Error for CartesianTrackingError {}

/// Commands a timed tool-pose trajectory; see the [module](self) docs.
///
/// A run starts at the first [`step`](Self::step) and ends when it reports
/// `done` or an error; the next step starts the trajectory again. A session
/// that ends otherwise leaves the run open: [`reset`](Self::reset) before
/// reusing the follower.
#[derive(Clone, Debug)]
pub struct CartesianTrajectoryFollower<C, const N: usize> {
    waypoints: Vec<CartesianWaypoint>,
    clock: C,
    start: Option<Stamp>,
    path_tolerance: (f64, f64),
    goal_tolerance: (f64, f64),
    goal_time: Duration,
}

impl<C: Clock, const N: usize> CartesianTrajectoryFollower<C, N> {
    pub fn new(
        waypoints: Vec<CartesianWaypoint>,
        clock: C,
    ) -> Result<Self, CartesianTrackingError> {
        check(&waypoints)?;
        Ok(Self {
            waypoints,
            clock,
            start: None,
            path_tolerance: (f64::INFINITY, f64::INFINITY),
            goal_tolerance: (f64::INFINITY, f64::INFINITY),
            goal_time: Duration::ZERO,
        })
    }

    /// Largest position (m) and rotation (rad) distance between the
    /// measured flange pose and the trajectory while it runs.
    pub fn with_path_tolerance(mut self, position: f64, rotation: f64) -> Self {
        self.path_tolerance = (position, rotation);
        self
    }

    /// Distance within which the final pose counts as reached, and how long
    /// after the trajectory ends the arm may take to get there; until then
    /// the final pose stays commanded and `done` is withheld.
    pub fn with_goal_tolerance(
        mut self,
        position: f64,
        rotation: f64,
        goal_time: Duration,
    ) -> Self {
        self.goal_tolerance = (position, rotation);
        self.goal_time = goal_time;
        self
    }

    pub fn waypoints(&self) -> &[CartesianWaypoint] {
        &self.waypoints
    }

    /// Time of the final waypoint.
    pub fn duration(&self) -> Duration {
        self.last().time_from_start
    }

    /// Ends the current run.
    pub fn reset(&mut self) {
        self.start = None;
    }

    /// The trajectory at `t`: the first pose before it starts, the final
    /// pose after it ends, and the interpolated pose in between.
    pub fn sample(&self, t: Duration) -> na::Isometry3<f64> {
        let after = self.waypoints.partition_point(|w| w.time_from_start <= t);
        match after {
            0 => self.waypoints[0].pose,
            n if n == self.waypoints.len() => self.last().pose,
            n => {
                let (a, b) = (&self.waypoints[n - 1], &self.waypoints[n]);
                let span = (b.time_from_start - a.time_from_start).as_secs_f64();
                let s = (t - a.time_from_start).as_secs_f64() / span;
                interpolate(&a.pose, &b.pose, s)
            }
        }
    }

    /// One control cycle: the pose to command and whether the trajectory
    /// is done. `state` is read only when a tolerance is checked.
    pub fn step(&mut self, state: &ArmState<N>) -> Result<(Pose, bool), CartesianTrackingError> {
        let step = self.advance(state);
        if !matches!(step, Ok((_, false))) {
            self.reset();
        }
        step
    }

    fn advance(&mut self, state: &ArmState<N>) -> Result<(Pose, bool), CartesianTrackingError> {
        let now = self.clock.now();
        let start = *self.start.get_or_insert(now);
        let nanos = now.nanos_since(&start).unwrap_or(i64::MIN);
        if nanos < 0 {
            return Err(CartesianTrackingError::ClockWentBack { nanos: nanos.saturating_neg() });
        }
        let at = Duration::from_nanos(nanos as u64);
        if at < self.duration() {
            let reference = self.sample(at);
            if let Some((position, rotation)) = beyond(self.path_tolerance, state, &reference)? {
                return Err(CartesianTrackingError::PathTolerance { position, rotation, at });
            }
            return Ok((Pose::from_isometry(reference), false));
        }
        let goal = self.last().pose;
        match beyond(self.goal_tolerance, state, &goal)? {
            None => Ok((Pose::from_isometry(goal), true)),
            Some(_) if at <= self.duration() + self.goal_time => {
                Ok((Pose::from_isometry(goal), false))
            }
            Some((position, rotation)) => Err(CartesianTrackingError::GoalTolerance {
                position,
                rotation,
                goal_time: self.goal_time,
            }),
        }
    }

    fn last(&self) -> &CartesianWaypoint {
        self.waypoints.last().expect("checked non-empty in new")
    }
}

impl<const N: usize> CartesianTrajectoryFollower<ManualClock, N> {
    /// A follower timed by the cycle durations the control callback reports,
    /// for a driver without a clock: call [`step_by`](Self::step_by) with the
    /// callback's `Duration` each cycle.
    pub fn cycle_timed(waypoints: Vec<CartesianWaypoint>) -> Result<Self, CartesianTrackingError> {
        Self::new(waypoints, ManualClock::simulation())
    }

    /// One control cycle, `dt` after the previous one. The first cycle of a
    /// run is at the trajectory's start whatever its `dt`.
    pub fn step_by(
        &mut self,
        dt: Duration,
        state: &ArmState<N>,
    ) -> Result<(Pose, bool), CartesianTrackingError> {
        self.clock.advance(dt);
        self.step(state)
    }
}

/// A straight move from `from` to `to` lasting `duration`, sampled every
/// `period`: the position along the line and the orientation by spherical
/// interpolation, both following the minimum-jerk profile
/// `s = 10τ³ − 15τ⁴ + 6τ⁵`, so it starts and ends at rest with zero
/// acceleration.
pub fn cartesian_line(
    from: &na::Isometry3<f64>,
    to: &na::Isometry3<f64>,
    duration: Duration,
    period: Duration,
) -> Vec<CartesianWaypoint> {
    minimum_jerk(duration, period, |s| interpolate(from, to, s))
}

/// A move along the circle through `from`'s position, `via` and `to`'s
/// position, lasting `duration` and sampled every `period`: from `from` past
/// `via` to `to` on the arc that contains `via`, with the orientation
/// interpolated spherically from `from`'s to `to`'s, both following the
/// minimum-jerk profile of [`cartesian_line`]. `via` constrains the position
/// only (as the interim point of MoveIt's Pilz `CIRC` does), and the move
/// passes it where its share of the arc length puts it on the profile.
///
/// `None` when the three positions do not define one circle: they are
/// collinear, or two of them are the same (so a full circle back to `from`
/// takes two arcs).
///
/// ```
/// # use std::time::Duration;
/// # use nalgebra::{Isometry3, Point3, Vector3};
/// use roplat_control::cartesian_arc;
///
/// // Half a circle of radius 10 cm over the top, in the x-z plane.
/// let from = Isometry3::translation(0.3, 0.0, 0.3);
/// let to = Isometry3::translation(0.5, 0.0, 0.3);
/// let over = Point3::new(0.4, 0.0, 0.4);
/// let period = Duration::from_millis(1);
/// let waypoints = cartesian_arc(&from, &over, &to, Duration::from_secs(2), period).unwrap();
/// let centre = Vector3::new(0.4, 0.0, 0.3);
/// assert!(waypoints.iter().all(|w| ((w.pose.translation.vector - centre).norm() - 0.1).abs() < 1e-12));
/// ```
pub fn cartesian_arc(
    from: &na::Isometry3<f64>,
    via: &na::Point3<f64>,
    to: &na::Isometry3<f64>,
    duration: Duration,
    period: Duration,
) -> Option<Vec<CartesianWaypoint>> {
    let Circle { centre, axis, radius, sweep } = circle(from, via, to)?;
    Some(minimum_jerk(duration, period, |s| {
        let position = if s >= 1.0 {
            to.translation.vector
        } else {
            centre + na::UnitQuaternion::from_axis_angle(&axis, s * sweep) * radius
        };
        na::Isometry3::from_parts(
            na::Translation3::from(position),
            turn(&from.rotation, &to.rotation, s),
        )
    }))
}

/// The tool's speed and acceleration limits for timing a move: linear in
/// m/s and m/s², angular in rad/s and rad/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CartesianLimits {
    pub linear_speed: f64,
    pub linear_acceleration: f64,
    pub angular_speed: f64,
    pub angular_acceleration: f64,
}

impl CartesianLimits {
    /// The shortest duration of a minimum-jerk move that travels `length`
    /// (m) while turning by `angle` (rad) within these limits. On the
    /// profile `s = 10τ³ − 15τ⁴ + 6τ⁵` over a duration `T`, the speed peaks
    /// at `1.875 L / T` and the acceleration at `(10 / √3) L / T²`, so `T`
    /// is the largest of the four bounds they give. `None` unless the
    /// limits are positive and the move finite.
    pub fn minimum_jerk_duration(&self, length: f64, angle: f64) -> Option<Duration> {
        let limits = [
            self.linear_speed,
            self.linear_acceleration,
            self.angular_speed,
            self.angular_acceleration,
        ];
        if !limits.iter().all(|l| *l > 0.0) || !(length.is_finite() && angle.is_finite()) {
            return None;
        }
        let peak_acceleration = 10.0 / 3f64.sqrt();
        let (length, angle) = (length.abs(), angle.abs());
        let seconds = [
            1.875 * length / self.linear_speed,
            (peak_acceleration * length / self.linear_acceleration).sqrt(),
            1.875 * angle / self.angular_speed,
            (peak_acceleration * angle / self.angular_acceleration).sqrt(),
        ]
        .into_iter()
        .fold(0.0, f64::max);
        Duration::try_from_secs_f64(seconds).ok()
    }
}

/// As [`cartesian_line`], lasting the shortest duration that keeps the
/// tool within `limits` (see [`CartesianLimits::minimum_jerk_duration`]).
/// `None` for limits that are not positive.
pub fn cartesian_line_within(
    from: &na::Isometry3<f64>,
    to: &na::Isometry3<f64>,
    limits: &CartesianLimits,
    period: Duration,
) -> Option<Vec<CartesianWaypoint>> {
    let length = (to.translation.vector - from.translation.vector).norm();
    let duration = limits.minimum_jerk_duration(length, from.rotation.angle_to(&to.rotation))?;
    Some(cartesian_line(from, to, duration, period))
}

/// As [`cartesian_arc`], lasting the shortest duration that keeps the tool
/// within `limits` along the arc. `None` for limits that are not positive,
/// or where [`cartesian_arc`] gives none.
pub fn cartesian_arc_within(
    from: &na::Isometry3<f64>,
    via: &na::Point3<f64>,
    to: &na::Isometry3<f64>,
    limits: &CartesianLimits,
    period: Duration,
) -> Option<Vec<CartesianWaypoint>> {
    let arc = circle(from, via, to)?;
    let length = arc.radius.norm() * arc.sweep;
    let duration = limits.minimum_jerk_duration(length, from.rotation.angle_to(&to.rotation))?;
    cartesian_arc(from, via, to, duration, period)
}

/// The circle through the three positions, swept from `from` past `via` to
/// `to`.
struct Circle {
    centre: na::Vector3<f64>,
    /// Counter-clockwise about it the arc goes.
    axis: na::Unit<na::Vector3<f64>>,
    /// From the centre to `from`.
    radius: na::Vector3<f64>,
    /// The arc's angle (rad).
    sweep: f64,
}

fn circle(
    from: &na::Isometry3<f64>,
    via: &na::Point3<f64>,
    to: &na::Isometry3<f64>,
) -> Option<Circle> {
    let start = from.translation.vector;
    let a = via.coords - start;
    let b = to.translation.vector - start;
    let normal = a.cross(&b);
    let area = normal.norm();
    if !area.is_finite() || area <= 1e-9 * a.norm() * b.norm() {
        return None;
    }
    // The circumcentre of the triangle (from, via, to).
    let centre = start
        + (b.cross(&normal) * a.norm_squared() + normal.cross(&a) * b.norm_squared())
            / (2.0 * area * area);
    let axis = na::Unit::new_unchecked(normal / area);
    let radius = start - centre;
    let end = to.translation.vector - centre;
    // Counter-clockwise about `axis`, the order the triangle's vertices go
    // round its circumcircle, `via` comes before `to`.
    let mut sweep = axis.dot(&radius.cross(&end)).atan2(radius.dot(&end));
    if sweep < 0.0 {
        sweep += std::f64::consts::TAU;
    }
    Some(Circle { centre, axis, radius, sweep })
}

/// Waypoints every `period` over `duration`, with the pose at the
/// minimum-jerk progress `s = 10τ³ − 15τ⁴ + 6τ⁵` of normalized time `τ`.
fn minimum_jerk(
    duration: Duration,
    period: Duration,
    pose: impl Fn(f64) -> na::Isometry3<f64>,
) -> Vec<CartesianWaypoint> {
    let steps = (duration.as_secs_f64() / period.as_secs_f64().max(f64::MIN_POSITIVE))
        .ceil()
        .max(1.0) as u32;
    (0..=steps)
        .map(|k| {
            let tau = f64::from(k) / f64::from(steps);
            let s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
            CartesianWaypoint { time_from_start: duration.mul_f64(tau), pose: pose(s) }
        })
        .collect()
}

fn interpolate(a: &na::Isometry3<f64>, b: &na::Isometry3<f64>, s: f64) -> na::Isometry3<f64> {
    let translation = a.translation.vector.lerp(&b.translation.vector, s);
    na::Isometry3::from_parts(
        na::Translation3::from(translation),
        turn(&a.rotation, &b.rotation, s),
    )
}

fn turn(
    a: &na::UnitQuaternion<f64>,
    b: &na::UnitQuaternion<f64>,
    s: f64,
) -> na::UnitQuaternion<f64> {
    a.try_slerp(b, s, 1e-9).unwrap_or(*b)
}

fn check(waypoints: &[CartesianWaypoint]) -> Result<(), CartesianTrackingError> {
    if waypoints.is_empty() {
        return Err(CartesianTrackingError::Empty);
    }
    for (index, w) in waypoints.iter().enumerate() {
        let finite = w
            .pose
            .translation
            .vector
            .iter()
            .chain(w.pose.rotation.coords.iter())
            .all(|c| c.is_finite());
        if !finite {
            return Err(CartesianTrackingError::NonFinite { index });
        }
        if index > 0 && w.time_from_start <= waypoints[index - 1].time_from_start {
            return Err(CartesianTrackingError::Timing { index });
        }
    }
    Ok(())
}

/// The distances, if either is beyond its `(position, rotation)` tolerance.
/// NaN counts as beyond.
fn beyond<const N: usize>(
    (position_tol, rotation_tol): (f64, f64),
    state: &ArmState<N>,
    reference: &na::Isometry3<f64>,
) -> Result<Option<(f64, f64)>, CartesianTrackingError> {
    if position_tol.is_infinite() && rotation_tol.is_infinite() {
        return Ok(None);
    }
    let measured: na::Isometry3<f64> = state
        .flange
        .meas
        .pose()
        .ok_or(CartesianTrackingError::Unobserved)?
        .into();
    let position = (measured.translation.vector - reference.translation.vector).norm();
    let rotation = measured.rotation.angle_to(&reference.rotation);
    let within = position <= position_tol && rotation <= rotation_tol;
    Ok((!within).then_some((position, rotation)))
}

#[cfg(feature = "roplat")]
mod node {
    use std::time::Duration;

    use roplat::{Lifecycle, Node, RoplatError};
    use roplat_behavior::{
        ArmState,
        msgs::{Clock, Pose},
    };

    use super::{CartesianTrackingError, CartesianTrajectoryFollower};

    /// Enters the core through `RoplatError::Node`, as the joint follower's
    /// `TrackingError` does.
    impl From<CartesianTrackingError> for RoplatError {
        fn from(error: CartesianTrackingError) -> Self {
            RoplatError::Node(Box::new(error))
        }
    }

    impl CartesianTrackingError {
        /// The tracking error a drive failed with, when it failed with one.
        pub fn find(error: &RoplatError) -> Option<&Self> {
            match error {
                RoplatError::Node(error) => error.downcast_ref(),
                _ => None,
            }
        }
    }

    impl<C, const N: usize> Lifecycle for CartesianTrajectoryFollower<C, N> {
        type Error = RoplatError;
    }

    /// Input is the control Yield `(state, dt)`; `dt` is not used, time comes
    /// from the follower's clock. Output is the Feed, refused as an error.
    impl<C: Clock, const N: usize> Node for CartesianTrajectoryFollower<C, N> {
        type Resources = ();
        type Input = (ArmState<N>, Duration);
        type Output = Result<(Pose, bool), CartesianTrackingError>;

        async fn process(&mut self, (state, _): Self::Input, _: ()) -> Self::Output {
            self.step(&state)
        }
    }
}

#[cfg(test)]
mod tests {
    use roplat_behavior::msgs::SpatialSample;

    use super::*;

    const MS: Duration = Duration::from_millis(1);

    fn at_flange(pose: &na::Isometry3<f64>) -> ArmState<7> {
        let mut state = ArmState::<7>::default();
        state.flange.meas = SpatialSample::default().with_pose(Pose::from_isometry(*pose));
        state
    }

    fn line() -> (
        na::Isometry3<f64>,
        na::Isometry3<f64>,
        Vec<CartesianWaypoint>,
    ) {
        let from = na::Isometry3::translation(0.4, 0.0, 0.3);
        let to = na::Isometry3::from_parts(
            na::Translation3::new(0.4, 0.2, 0.2),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.5),
        );
        (
            from,
            to,
            cartesian_line(&from, &to, Duration::from_millis(100), MS),
        )
    }

    #[test]
    fn the_line_starts_and_ends_at_rest_and_moves_straight() {
        let (from, to, waypoints) = line();
        assert_eq!(waypoints.len(), 101);
        let close = |a: &na::Isometry3<f64>, b: &na::Isometry3<f64>| {
            (a.translation.vector - b.translation.vector).norm() < 1e-12
                && a.rotation.angle_to(&b.rotation) < 1e-9
        };
        assert!(close(&waypoints[0].pose, &from) && close(&waypoints[100].pose, &to));
        assert_eq!(waypoints[100].time_from_start, Duration::from_millis(100));
        // Minimum jerk: the first and last steps are tiny, the middle fastest.
        let step = |k: usize| {
            (waypoints[k + 1].pose.translation.vector - waypoints[k].pose.translation.vector).norm()
        };
        assert!(step(0) < step(50) / 100.0 && step(99) < step(50) / 100.0);
        // Every point on the segment between the ends.
        let dir = (to.translation.vector - from.translation.vector).normalize();
        for w in &waypoints {
            let off = w.pose.translation.vector - from.translation.vector;
            assert!((off - dir * off.dot(&dir)).norm() < 1e-12);
        }
    }

    /// A point on the circle of radius 0.1 about (0.4, 0, 0.3) in a plane
    /// tilted off every axis, at an angle in degrees.
    fn on_circle(degrees: f64) -> na::Point3<f64> {
        let tilt = na::UnitQuaternion::from_euler_angles(0.4, -0.7, 1.1);
        let (sin, cos) = degrees.to_radians().sin_cos();
        na::Point3::new(0.4, 0.0, 0.3) + tilt * na::Vector3::new(0.1 * cos, 0.1 * sin, 0.0)
    }

    fn arc(via: f64, to: f64) -> (na::Isometry3<f64>, Vec<CartesianWaypoint>) {
        let from = na::Isometry3::from_parts(on_circle(0.0).into(), na::UnitQuaternion::identity());
        let to = na::Isometry3::from_parts(
            on_circle(to).into(),
            na::UnitQuaternion::from_euler_angles(0.0, 0.3, 0.5),
        );
        let waypoints = cartesian_arc(&from, &on_circle(via), &to, Duration::from_secs(1), MS)
            .expect("three points on one circle");
        assert_eq!(waypoints.len(), 1001);
        let (first, last) = (&waypoints[0].pose, &waypoints[1000].pose);
        assert!((first.translation.vector - from.translation.vector).norm() < 1e-15);
        assert_eq!(last.translation, to.translation);
        assert!(first.rotation.angle_to(&from.rotation) < 1e-9);
        assert!(last.rotation.angle_to(&to.rotation) < 1e-9);
        (to, waypoints)
    }

    fn nearest(waypoints: &[CartesianWaypoint], p: &na::Point3<f64>) -> f64 {
        waypoints
            .iter()
            .map(|w| (w.pose.translation.vector - p.coords).norm())
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn the_arc_stays_on_the_circle_and_passes_its_via_point() {
        // Via between the ends the short way round, then the long way.
        for (via, to, other_side) in [(100.0, 150.0, 250.0), (250.0, 150.0, 75.0)] {
            let (to, waypoints) = arc(via, to);
            let centre = na::Vector3::new(0.4, 0.0, 0.3);
            for w in &waypoints {
                assert!(((w.pose.translation.vector - centre).norm() - 0.1).abs() < 1e-12);
            }
            // Within half a step of the via point, and the other arc not taken.
            assert!(nearest(&waypoints, &on_circle(via)) < 5e-4);
            assert!(nearest(&waypoints, &on_circle(other_side)) > 0.05);
            // At rest at both ends, as on the line.
            let step = |k: usize| {
                (waypoints[k + 1].pose.translation.vector - waypoints[k].pose.translation.vector)
                    .norm()
            };
            assert!(step(0) < step(500) / 1e4 && step(999) < step(500) / 1e4);
            assert!(waypoints[500].pose.rotation.angle_to(&to.rotation) > 0.1);
        }
    }

    /// The largest speed and acceleration along `waypoints`, by finite
    /// differences of position (m) and of the angle turned (rad).
    fn peaks(waypoints: &[CartesianWaypoint], period: f64) -> [f64; 4] {
        let speed = |a: &CartesianWaypoint, b: &CartesianWaypoint| {
            (
                (b.pose.translation.vector - a.pose.translation.vector).norm() / period,
                a.pose.rotation.angle_to(&b.pose.rotation) / period,
            )
        };
        let speeds: Vec<(f64, f64)> = waypoints.windows(2).map(|w| speed(&w[0], &w[1])).collect();
        let max = |f: &dyn Fn(usize) -> f64, n: usize| (0..n).map(f).fold(0.0, f64::max);
        [
            max(&|k| speeds[k].0, speeds.len()),
            max(
                &|k| (speeds[k + 1].0 - speeds[k].0).abs() / period,
                speeds.len() - 1,
            ),
            max(&|k| speeds[k].1, speeds.len()),
            max(
                &|k| (speeds[k + 1].1 - speeds[k].1).abs() / period,
                speeds.len() - 1,
            ),
        ]
    }

    #[test]
    fn moves_within_limits_reach_the_binding_one() {
        let limits = CartesianLimits {
            linear_speed: 0.25,
            linear_acceleration: 1.0,
            angular_speed: 0.8,
            angular_acceleration: 4.0,
        };
        let from = na::Isometry3::translation(0.4, 0.0, 0.3);
        // A long straight move is held by the linear speed, a short one by
        // the linear acceleration, a turn in place by the angular limits.
        let cases = [
            (na::Isometry3::translation(0.4, 0.5, 0.3), 0),
            (na::Isometry3::translation(0.4, 0.02, 0.3), 1),
            (
                na::Isometry3::from_parts(
                    na::Translation3::new(0.4, 0.0, 0.3),
                    na::UnitQuaternion::from_euler_angles(0.0, 0.0, 1.5),
                ),
                2,
            ),
        ];
        for (to, binding) in cases {
            let waypoints = cartesian_line_within(&from, &to, &limits, MS).unwrap();
            let found = peaks(&waypoints, 1e-3);
            let bounds = [
                limits.linear_speed,
                limits.linear_acceleration,
                limits.angular_speed,
                limits.angular_acceleration,
            ];
            for k in 0..4 {
                assert!(found[k] <= bounds[k] * 1.01, "{k}: {found:?}");
            }
            assert!(
                found[binding] >= bounds[binding] * 0.98,
                "{binding}: {found:?}"
            );
        }
        // Along an arc the arc length counts, not the chord.
        let (to, _) = arc(250.0, 150.0);
        let from = na::Isometry3::from_parts(on_circle(0.0).into(), na::UnitQuaternion::identity());
        let slow = CartesianLimits { linear_speed: 0.05, ..limits };
        let waypoints = cartesian_arc_within(&from, &on_circle(250.0), &to, &slow, MS).unwrap();
        let found = peaks(&waypoints, 1e-3);
        assert!(
            found[0] <= 0.05 * 1.01 && found[0] >= 0.05 * 0.98,
            "{found:?}"
        );
        // No limits, no move.
        let none = CartesianLimits { linear_speed: 0.0, ..limits };
        assert!(cartesian_line_within(&from, &to, &none, MS).is_none());
        assert_eq!(limits.minimum_jerk_duration(0.0, 0.0), Some(Duration::ZERO));
    }

    #[test]
    fn three_points_off_one_circle_give_no_arc() {
        let from = na::Isometry3::translation(0.4, 0.0, 0.3);
        let to = na::Isometry3::translation(0.6, 0.2, 0.3);
        let arc = |via: na::Point3<f64>, to: &na::Isometry3<f64>| {
            cartesian_arc(&from, &via, to, Duration::from_secs(1), MS)
        };
        assert!(arc(na::Point3::new(0.5, 0.1, 0.3), &to).is_none()); // collinear
        assert!(arc(na::Point3::new(0.4, 0.0, 0.3), &to).is_none()); // via at the start
        assert!(arc(na::Point3::new(0.6, 0.2, 0.3), &to).is_none()); // via at the end
        assert!(arc(na::Point3::new(0.5, 0.0, 0.3), &from).is_none()); // back to the start
        assert!(arc(na::Point3::new(f64::NAN, 0.0, 0.3), &to).is_none());
        assert!(arc(na::Point3::new(0.5, 0.0, 0.3), &to).is_some());
    }

    #[test]
    fn follows_on_its_clock_and_reports_done_at_the_end() {
        let (_, to, waypoints) = line();
        let mut follower = CartesianTrajectoryFollower::<_, 7>::cycle_timed(waypoints).unwrap();
        let state = ArmState::<7>::default(); // no tolerance: state unread
        let mut cycles = 0;
        loop {
            let (pose, done) = follower.step_by(MS, &state).unwrap();
            cycles += 1;
            if done {
                let reached: na::Isometry3<f64> = pose.into();
                assert!((reached.translation.vector - to.translation.vector).norm() < 1e-12);
                break;
            }
        }
        // 100 ms at 1 ms: cycles at 0..=100 ms, done on the last.
        assert_eq!(cycles, 101);
    }

    #[test]
    fn tolerances_refuse_a_lagging_arm() {
        let (from, to, waypoints) = line();
        // The arm stays at the start: within 1 cm at first, then beyond.
        let mut follower = CartesianTrajectoryFollower::<_, 7>::cycle_timed(waypoints.clone())
            .unwrap()
            .with_path_tolerance(0.01, 0.1);
        let stuck = at_flange(&from);
        let error = loop {
            match follower.step_by(MS, &stuck) {
                Ok(_) => continue,
                Err(e) => break e,
            }
        };
        assert!(
            matches!(error, CartesianTrackingError::PathTolerance { position, .. } if position > 0.01)
        );
        // Goal: an arm short of the goal gets the goal time, then an error.
        let mut follower = CartesianTrajectoryFollower::<_, 7>::cycle_timed(waypoints)
            .unwrap()
            .with_goal_tolerance(0.001, 0.01, Duration::from_millis(5));
        let short = at_flange(&na::Isometry3::from_parts(
            na::Translation3::new(to.translation.x, to.translation.y - 0.01, to.translation.z),
            to.rotation,
        ));
        let mut last = Ok((Pose::IDENTITY, false));
        for _ in 0..120 {
            last = follower.step_by(MS, &short);
            if last.is_err() {
                break;
            }
        }
        assert!(
            matches!(last, Err(CartesianTrackingError::GoalTolerance { .. })),
            "{last:?}"
        );
        // At the goal it is done.
        let mut follower = CartesianTrajectoryFollower::<_, 7>::cycle_timed(line().2)
            .unwrap()
            .with_goal_tolerance(0.001, 0.01, Duration::from_millis(5));
        let at_goal = at_flange(&to);
        let done = (0..120).any(|_| matches!(follower.step_by(MS, &at_goal), Ok((_, true))));
        assert!(done);
    }

    #[test]
    fn invalid_trajectories_and_unobserved_states_are_refused() {
        assert!(matches!(
            CartesianTrajectoryFollower::<_, 7>::cycle_timed(Vec::new()),
            Err(CartesianTrackingError::Empty)
        ));
        let mut waypoints = line().2;
        waypoints[3].time_from_start = waypoints[2].time_from_start;
        assert!(matches!(
            CartesianTrajectoryFollower::<_, 7>::cycle_timed(waypoints),
            Err(CartesianTrackingError::Timing { index: 3 })
        ));
        let mut follower = CartesianTrajectoryFollower::<_, 7>::cycle_timed(line().2)
            .unwrap()
            .with_path_tolerance(0.01, 0.1);
        let mut unobserved = ArmState::<7>::default();
        unobserved.flange.meas = SpatialSample::EMPTY;
        assert_eq!(
            follower.step_by(MS, &unobserved),
            Err(CartesianTrackingError::Unobserved)
        );
        // A NaN measurement is beyond any tolerance.
        let nan = at_flange(&na::Isometry3::translation(f64::NAN, 0.0, 0.0));
        assert!(matches!(
            follower.step_by(MS, &nan),
            Err(CartesianTrackingError::PathTolerance { .. })
        ));
    }
}
