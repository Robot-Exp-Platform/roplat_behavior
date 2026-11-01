//! Following a timed joint trajectory on the joint-position channel.
//!
//! [`JointTrajectoryFollower`] plays [`JointTrajectoryPoint`]s against a
//! [`Clock`]: every cycle it commands the trajectory sampled at the clock's
//! time since its first cycle. Time comes from the clock rather than from the
//! cycle `Duration` a driver reports, so the same follower runs on wall,
//! simulation or replay time, and a late cycle commands where the trajectory
//! is by then instead of where it would have been. Between points it
//! interpolates quintic segments through each point's position, velocity and
//! acceleration; zero velocity and acceleration mean the motion rests there.
//! A new plan can take over mid-run through
//! [`replace`](JointTrajectoryFollower::replace), which continues from the
//! current sample instead of jumping to the new first point.
//!
//! Tolerances check and refuse, as `roplat_safety`'s gates do: a measured
//! position farther from the trajectory than the path tolerance, or a goal
//! not reached within the goal time, is a [`TrackingError`], and the follower
//! never substitutes another command. In a control domain `follower?` turns
//! the error into a failed cycle, so the device session receives `Break`.

use std::time::Duration;

use roplat_msgs::{Clock, JointState, JointTrajectoryPoint, ManualClock, Stamp};

/// Why a trajectory was rejected or its following stopped.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum TrackingError {
    #[error("a trajectory needs at least one point")]
    Empty,
    #[error("point {index}: time_from_start must be non-negative and increase strictly")]
    Timing { index: usize },
    #[error("point {index}: positions, velocities and accelerations must be finite")]
    NonFinite { index: usize },
    #[error("a tolerance is checked but the state has no measured position")]
    Unobserved,
    #[error("the clock went back {nanos} ns behind the trajectory's start")]
    ClockWentBack { nanos: i64 },
    #[error(
        "joint {joint} is {error} from the trajectory at {at:?}, beyond the path tolerance {tolerance}"
    )]
    PathTolerance {
        joint: usize,
        error: f64,
        tolerance: f64,
        at: Duration,
    },
    #[error(
        "joint {joint} is still {error} from the goal {goal_time:?} after the trajectory ended, beyond the goal tolerance {tolerance}"
    )]
    GoalTolerance {
        joint: usize,
        error: f64,
        tolerance: f64,
        goal_time: Duration,
    },
}

/// Commands a timed joint trajectory, see the [module](self) documentation.
///
/// The run starts at the first [`step`](Self::step) and lasts one control
/// session: reporting `done` or an error ends it, and the next step starts the
/// trajectory again. A session that ends otherwise (a stop, another node's
/// failure) leaves the run open, so a follower reused for a later session is
/// [`reset`](Self::reset) first.
#[derive(Clone, Debug)]
pub struct JointTrajectoryFollower<C, const N: usize> {
    points: Vec<JointTrajectoryPoint<N>>,
    clock: C,
    start: Option<Stamp>,
    path_tolerance: [f64; N],
    goal_tolerance: [f64; N],
    goal_time: Duration,
}

impl<C: Clock, const N: usize> JointTrajectoryFollower<C, N> {
    /// Checks that `points` are finite and strictly increasing in time.
    /// Without tolerances the follower only commands the trajectory.
    pub fn new(points: Vec<JointTrajectoryPoint<N>>, clock: C) -> Result<Self, TrackingError> {
        check(&points, -1)?;
        Ok(Self {
            points,
            clock,
            start: None,
            path_tolerance: [f64::INFINITY; N],
            goal_tolerance: [f64::INFINITY; N],
            goal_time: Duration::ZERO,
        })
    }

    /// Largest distance per joint between the measured position and the
    /// trajectory while it runs. `f64::INFINITY` leaves a joint unchecked.
    pub fn with_path_tolerance(mut self, tolerance: [f64; N]) -> Self {
        self.path_tolerance = tolerance;
        self
    }

    /// Distance per joint within which the final point counts as reached, and
    /// how long after the trajectory ends the arm may take to get there.
    /// Until then the final point stays commanded and `done` is withheld.
    pub fn with_goal_tolerance(mut self, tolerance: [f64; N], goal_time: Duration) -> Self {
        self.goal_tolerance = tolerance;
        self.goal_time = goal_time;
        self
    }

    pub fn points(&self) -> &[JointTrajectoryPoint<N>] {
        &self.points
    }

    /// Time of the final point.
    pub fn duration(&self) -> Duration {
        nanos_to_duration(self.last().time_from_start)
    }

    /// Ends the current run; the next step starts the trajectory again.
    pub fn reset(&mut self) {
        self.start = None;
    }

    /// Replaces the trajectory with `points`, timed from now.
    ///
    /// During a run the motion carries on from where the current trajectory
    /// is at this instant: that sample, with its velocity and acceleration,
    /// becomes the first point, so the command stays continuous, and `points`
    /// must start after it. Tolerances carry over and apply to the new goal.
    /// Before a run `points` simply take the old ones' place. A refused
    /// replacement leaves the current trajectory running.
    pub fn replace(&mut self, points: Vec<JointTrajectoryPoint<N>>) -> Result<(), TrackingError> {
        let Some(start) = self.start else {
            check(&points, -1)?;
            self.points = points;
            return Ok(());
        };
        check(&points, 0)?;
        let now = self.clock.now();
        let nanos = now.nanos_since(&start).unwrap_or(i64::MIN);
        if nanos < 0 {
            return Err(TrackingError::ClockWentBack { nanos: nanos.saturating_neg() });
        }
        let mut current = self.sample(nanos_to_duration(nanos));
        current.time_from_start = 0;
        self.points = std::iter::once(current).chain(points).collect();
        self.start = Some(now);
        Ok(())
    }

    /// The trajectory at `t`: the first point before it starts, the final
    /// point after it ends, and the quintic segment in between.
    pub fn sample(&self, t: Duration) -> JointTrajectoryPoint<N> {
        let t = i64::try_from(t.as_nanos()).unwrap_or(i64::MAX);
        let after = self.points.partition_point(|p| p.time_from_start <= t);
        let mut sample = match after {
            0 => self.points[0],
            n if n == self.points.len() => *self.last(),
            n => quintic(&self.points[n - 1], &self.points[n], t),
        };
        sample.time_from_start = t;
        sample
    }

    /// One control cycle: the position to command and whether the trajectory
    /// is done. `state` is only read when a tolerance is checked.
    pub fn step(&mut self, state: &JointState<N>) -> Result<([f64; N], bool), TrackingError> {
        let step = self.advance(state);
        if !matches!(step, Ok((_, false))) {
            self.reset();
        }
        step
    }

    fn advance(&mut self, state: &JointState<N>) -> Result<([f64; N], bool), TrackingError> {
        let now = self.clock.now();
        let start = *self.start.get_or_insert(now);
        let nanos = now.nanos_since(&start).unwrap_or(i64::MIN);
        if nanos < 0 {
            return Err(TrackingError::ClockWentBack { nanos: nanos.saturating_neg() });
        }
        let at = nanos_to_duration(nanos);
        let end = self.last().time_from_start;
        if nanos < end {
            let reference = self.sample(at).q;
            if let Some((joint, error)) = beyond(&self.path_tolerance, state, &reference)? {
                let tolerance = self.path_tolerance[joint];
                return Err(TrackingError::PathTolerance { joint, error, tolerance, at });
            }
            return Ok((reference, false));
        }
        let goal = self.last().q;
        match beyond(&self.goal_tolerance, state, &goal)? {
            None => Ok((goal, true)),
            Some(_) if at <= self.duration() + self.goal_time => Ok((goal, false)),
            Some((joint, error)) => Err(TrackingError::GoalTolerance {
                joint,
                error,
                tolerance: self.goal_tolerance[joint],
                goal_time: self.goal_time,
            }),
        }
    }

    fn last(&self) -> &JointTrajectoryPoint<N> {
        self.points.last().expect("checked non-empty in new")
    }
}

impl<const N: usize> JointTrajectoryFollower<ManualClock, N> {
    /// A follower timed by the cycle durations a control callback reports, for
    /// a driver without a clock of its own: each cycle calls
    /// [`step_by`](Self::step_by) with the callback's `Duration`.
    pub fn cycle_timed(points: Vec<JointTrajectoryPoint<N>>) -> Result<Self, TrackingError> {
        Self::new(points, ManualClock::simulation())
    }

    /// One control cycle, `dt` after the previous one. The first cycle of a
    /// run is at the trajectory's start whatever its `dt`, so a late cycle
    /// commands where the trajectory is by then.
    pub fn step_by(
        &mut self,
        dt: Duration,
        state: &JointState<N>,
    ) -> Result<([f64; N], bool), TrackingError> {
        self.clock.advance(dt);
        self.step(state)
    }
}

/// Timed points for a trajectory sampled every `period`, such as the COPP
/// plans in [`time_scaling`](crate::time_scaling), which sample at the
/// robot's control period. Velocities and accelerations are central
/// differences, one-sided at the two ends.
pub fn points_from_samples<const N: usize>(
    samples: &[[f64; N]],
    period: Duration,
) -> Vec<JointTrajectoryPoint<N>> {
    let dt = period.as_secs_f64();
    let nanos = i64::try_from(period.as_nanos()).unwrap_or(i64::MAX);
    let last = samples.len().saturating_sub(1);
    (0..samples.len())
        .map(|k| {
            // The three samples the differences use, kept inside the slice.
            let (a, b, c) = match k {
                _ if last < 2 => (0, 0, last),
                0 => (0, 1, 2),
                k if k == last => (k - 2, k - 1, k),
                k => (k - 1, k, k + 1),
            };
            let (pa, pb, pc) = (samples[a], samples[b], samples[c]);
            let span = (c - a) as f64 * dt;
            JointTrajectoryPoint {
                time_from_start: k as i64 * nanos,
                q: samples[k],
                dq: std::array::from_fn(|i| {
                    if span == 0. {
                        return 0.;
                    }
                    let central = (pc[i] - pa[i]) / span;
                    // One-sided at the ends: the slope at the end sample of a parabola.
                    match k {
                        0 if last >= 2 => (-3. * pa[i] + 4. * pb[i] - pc[i]) / (2. * dt),
                        k if k == last && last >= 2 => {
                            (pa[i] - 4. * pb[i] + 3. * pc[i]) / (2. * dt)
                        }
                        _ => central,
                    }
                }),
                ddq: std::array::from_fn(|i| {
                    if last < 2 {
                        return 0.;
                    }
                    (pa[i] - 2. * pb[i] + pc[i]) / (dt * dt)
                }),
            }
        })
        .collect()
}

/// Points are non-empty, finite, and strictly increasing in time from after
/// `after` nanoseconds.
fn check<const N: usize>(
    points: &[JointTrajectoryPoint<N>],
    after: i64,
) -> Result<(), TrackingError> {
    if points.is_empty() {
        return Err(TrackingError::Empty);
    }
    let mut previous = after;
    for (index, point) in points.iter().enumerate() {
        if point.time_from_start <= previous {
            return Err(TrackingError::Timing { index });
        }
        previous = point.time_from_start;
        let values = point.q.iter().chain(&point.dq).chain(&point.ddq);
        if !values.into_iter().all(|v| v.is_finite()) {
            return Err(TrackingError::NonFinite { index });
        }
    }
    Ok(())
}

/// The first joint whose measured position is not within `tolerance` of
/// `reference`. NaN anywhere counts as beyond.
fn beyond<const N: usize>(
    tolerance: &[f64; N],
    state: &JointState<N>,
    reference: &[f64; N],
) -> Result<Option<(usize, f64)>, TrackingError> {
    if tolerance.iter().all(|t| *t == f64::INFINITY) {
        return Ok(None);
    }
    let measured = state.meas.q().ok_or(TrackingError::Unobserved)?;
    Ok((0..N).find_map(|i| {
        let error = (measured[i] - reference[i]).abs();
        let beyond = error.is_nan() || tolerance[i].is_nan() || error > tolerance[i];
        beyond.then_some((i, error))
    }))
}

/// Quintic segment from `a` to `b` matching position, velocity and
/// acceleration at both ends, evaluated at `t` in `[a, b)`.
fn quintic<const N: usize>(
    a: &JointTrajectoryPoint<N>,
    b: &JointTrajectoryPoint<N>,
    t: i64,
) -> JointTrajectoryPoint<N> {
    let h = (b.time_from_start - a.time_from_start) as f64 * 1e-9;
    let s = (t - a.time_from_start) as f64 * 1e-9 / h;
    let mut sample =
        JointTrajectoryPoint { time_from_start: t, q: [0.; N], dq: [0.; N], ddq: [0.; N] };
    for i in 0..N {
        let (v0, v1) = (a.dq[i] * h, b.dq[i] * h);
        let (a0, a1) = (a.ddq[i] * h * h, b.ddq[i] * h * h);
        let d = b.q[i] - a.q[i];
        let c = [
            a.q[i],
            v0,
            a0 / 2.,
            10. * d - 6. * v0 - 4. * v1 - 1.5 * a0 + 0.5 * a1,
            -15. * d + 8. * v0 + 7. * v1 + 1.5 * a0 - a1,
            6. * d - 3. * v0 - 3. * v1 - 0.5 * a0 + 0.5 * a1,
        ];
        sample.q[i] = c[0] + s * (c[1] + s * (c[2] + s * (c[3] + s * (c[4] + s * c[5]))));
        let dq = c[1] + s * (2. * c[2] + s * (3. * c[3] + s * (4. * c[4] + s * 5. * c[5])));
        let ddq = 2. * c[2] + s * (6. * c[3] + s * (12. * c[4] + s * 20. * c[5]));
        sample.dq[i] = dq / h;
        sample.ddq[i] = ddq / (h * h);
    }
    sample
}

fn nanos_to_duration(nanos: i64) -> Duration {
    Duration::from_nanos(u64::try_from(nanos).unwrap_or(0))
}

#[cfg(feature = "roplat")]
mod node {
    use std::time::Duration;

    use roplat::{Lifecycle, Node, RoplatError};
    use roplat_msgs::{Clock, JointState};

    use super::{JointTrajectoryFollower, TrackingError};

    /// Enters the core through `RoplatError::Node`, as `ControlSessionError`
    /// and `roplat_safety`'s violations do.
    impl From<TrackingError> for RoplatError {
        fn from(error: TrackingError) -> Self {
            RoplatError::Node(Box::new(error))
        }
    }

    impl TrackingError {
        /// The tracking error a drive failed with, when it failed with one.
        pub fn find(error: &RoplatError) -> Option<&Self> {
            match error {
                RoplatError::Node(error) => error.downcast_ref(),
                _ => None,
            }
        }
    }

    impl<C, const N: usize> Lifecycle for JointTrajectoryFollower<C, N> {
        type Error = RoplatError;
    }

    /// Input is the control Yield `(state, dt)`; `dt` is not used, time comes
    /// from the follower's clock. Output is the Feed, refused as an error.
    impl<C: Clock, const N: usize> Node for JointTrajectoryFollower<C, N> {
        type Resources = ();
        type Input = (JointState<N>, Duration);
        type Output = Result<([f64; N], bool), TrackingError>;

        async fn process(&mut self, (state, _): Self::Input, _: ()) -> Self::Output {
            self.step(&state)
        }
    }
}
