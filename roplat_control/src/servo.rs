//! Cartesian servoing: a twist command to joint motion, in real time.
//!
//! [`CartesianServo`] turns `[vx, vy, vz, wx, wy, wz]` of the arm's tip into
//! joint velocities by damped least squares on the Jacobian, then slows the
//! whole motion by one factor, so the tip keeps the commanded direction:
//!
//! - near a singularity, but only when the motion approaches it (does not
//!   raise the smallest singular value of the Jacobian): from full speed at
//!   [`singularity_slowdown`](ServoConfig::singularity_slowdown) to a halt at
//!   [`singularity_stop`](ServoConfig::singularity_stop); moving away from
//!   the singularity is never slowed;
//! - within [`joint_limit_margin`](ServoConfig::joint_limit_margin) of a
//!   position limit, for motion toward it; no cycle steps past a limit;
//! - so no joint exceeds its velocity limit;
//! - by an optional caller-supplied factor, e.g. from the distance to the
//!   nearest obstacle ([`proximity_scale`]).
//!
//! [`velocity_control`](CartesianServo::velocity_control) and
//! [`position_control`](CartesianServo::position_control) wrap it as
//! controllers for `JointVelocityControl` and `JointPositionControl`, fed by
//! any twist source, such as
//! [`cartesian_teleop_control`](crate::cartesian_teleop_control). On a
//! velocity channel whatever the device does not track (e.g. sag under
//! gravity while commanded almost to rest) accumulates; the position
//! controller follows its own command and does not drift. A cycle uses no
//! heap allocation unless the model or the hook does.

use crate::exit::missing;
use std::{ops::ControlFlow, time::Duration};

use nalgebra as na;
use roplat_behavior::{
    ControlExit, ControlStep, ForwardKinematics, Jaco, JacobianModel, JointState, RobotException,
    RobotResult, msgs::JointLimits,
};

/// The frame a twist command is written in. Either way the linear part is
/// the velocity of the tip's origin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServoFrame {
    /// The model's base frame, the frame of its Jacobian.
    Base,
    /// The tip frame, as an operator holding the tool sees it.
    Tip,
}

/// How [`CartesianServo`] follows a twist.
#[derive(Clone, Copy, Debug)]
pub struct ServoConfig {
    pub frame: ServoFrame,
    /// Damping λ of the least-squares solution `Jᵀ (J Jᵀ + λ² I)⁻¹ v`, which
    /// keeps joint rates finite at a singularity.
    pub damping: f64,
    /// Jacobian condition number (largest over smallest singular value) at
    /// which motion toward a singularity starts to slow down.
    pub singularity_slowdown: f64,
    /// Condition number at which motion toward a singularity stops.
    pub singularity_stop: f64,
    /// Distance from a position limit (rad or m) within which motion toward
    /// it slows linearly to zero at the limit; zero only stops at the limit.
    pub joint_limit_margin: f64,
}

impl Default for ServoConfig {
    /// The singularity thresholds are MoveIt Servo's defaults.
    fn default() -> Self {
        Self {
            frame: ServoFrame::Base,
            damping: 1e-3,
            singularity_slowdown: 17.0,
            singularity_stop: 30.0,
            joint_limit_margin: 0.1,
        }
    }
}

/// What limited a [`ServoStep`]: the constraint with the smallest factor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServoStatus {
    /// Followed at the commanded speed.
    Free,
    /// Slowed approaching a singularity.
    NearSingularity,
    /// Halted at the singularity stop threshold.
    SingularityStop,
    /// Slowed or halted near a position limit of this joint.
    JointLimit(usize),
    /// Scaled so this joint stays within its velocity limit.
    VelocityLimit(usize),
    /// Slowed by the proximity hook.
    Proximity,
}

/// One cycle of [`CartesianServo::step`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoStep<const N: usize> {
    /// Joint velocities to command.
    pub dq: [f64; N],
    /// Factor in `[0, 1]` applied to the least-squares solution.
    pub scale: f64,
    /// Jacobian condition number at the given position.
    pub condition: f64,
    pub status: ServoStatus,
}

// dyn(d): a generic `F` would appear in the public `CartesianServo` type name;
// one call per control cycle is negligible against the servo step.
type Hook<const N: usize> = Box<dyn FnMut(&[f64; N]) -> f64 + Send>;

/// A Cartesian servo for an arm with `N` joints; see the
/// [module documentation](self).
///
/// The model gives the tip's Jacobian in the base frame, linear rows first,
/// as both `dh_kinematics!` and `roplat_kinematics::UrdfKinematics` do; the
/// forward kinematics is used only for [`ServoFrame::Tip`].
pub struct CartesianServo<M, const N: usize> {
    model: M,
    limits: JointLimits<N>,
    config: ServoConfig,
    period: Option<Duration>,
    proximity: Option<Hook<N>>,
}

fn invalid(message: String) -> RobotException {
    RobotException::InvalidInstruction(message)
}

/// Squares of the largest and smallest singular values of `J`, from the
/// eigenvalues of the 6×6 `J Jᵀ` (only the largest `min(N, 6)` of them can
/// be nonzero).
fn extreme_singular_values<const N: usize>(j: &Jaco<N>) -> (f64, f64) {
    let rank = N.min(6);
    let mut values = (j * j.transpose()).symmetric_eigenvalues();
    values
        .as_mut_slice()
        .sort_unstable_by(|a, b| b.total_cmp(a));
    (values[0], values[rank - 1].max(0.0))
}

/// Largest over smallest singular value; infinite at a singularity.
fn condition_number((largest, smallest): (f64, f64)) -> f64 {
    if smallest > 0.0 {
        (largest / smallest).sqrt()
    } else {
        f64::INFINITY
    }
}

/// A slowdown factor from a distance, e.g. to the nearest obstacle: 1 at
/// `slowdown` and beyond, falling linearly to 0 at `stop` and below.
pub fn proximity_scale(distance: f64, slowdown: f64, stop: f64) -> f64 {
    if distance >= slowdown {
        1.0
    } else if distance <= stop {
        0.0
    } else {
        (distance - stop) / (slowdown - stop)
    }
}

impl<M, const N: usize> CartesianServo<M, N>
where
    M: JacobianModel<N> + ForwardKinematics<N>,
{
    /// Rejects inverted position limits, velocity limits that are not
    /// positive, and thresholds out of order. Only the position and velocity
    /// limits are used.
    pub fn new(model: M, limits: JointLimits<N>, config: ServoConfig) -> RobotResult<Self> {
        if N == 0 {
            return Err(invalid("a servo needs at least one joint".into()));
        }
        for i in 0..N {
            let ordered = limits.q_min[i] <= limits.q_max[i];
            if !ordered {
                return Err(invalid(format!("joint {i}: position limits are inverted")));
            }
            let positive = limits.dq_max[i] > 0.0;
            if !positive {
                return Err(invalid(format!(
                    "joint {i}: velocity limit must be positive"
                )));
            }
        }
        let ServoConfig {
            damping,
            singularity_slowdown: slowdown,
            singularity_stop: stop,
            joint_limit_margin: margin,
            ..
        } = config;
        if !(damping.is_finite() && damping >= 0.0) {
            return Err(invalid("damping must be finite and non-negative".into()));
        }
        if !(1.0 <= slowdown && slowdown < stop) {
            return Err(invalid(
                "singularity thresholds must satisfy 1 <= slowdown < stop".into(),
            ));
        }
        if !(margin.is_finite() && margin >= 0.0) {
            return Err(invalid(
                "joint limit margin must be finite and non-negative".into(),
            ));
        }
        Ok(Self { model, limits, config, period: None, proximity: None })
    }

    /// Integrates and guards limits over the device's command period instead
    /// of the controller's `Duration`, which drivers do not yet report alike.
    pub fn with_period(mut self, period: Duration) -> Self {
        self.period = Some(period);
        self
    }

    /// Multiplies the motion by `hook(q)`, clamped to `[0, 1]`; a NaN halts.
    pub fn with_proximity(mut self, hook: impl FnMut(&[f64; N]) -> f64 + Send + 'static) -> Self {
        self.proximity = Some(Box::new(hook));
        self
    }

    pub fn model(&self) -> &M {
        &self.model
    }

    pub fn config(&self) -> &ServoConfig {
        &self.config
    }

    /// Joint velocities following `twist` from position `q`, for a cycle of
    /// `dt` seconds (used only so the cycle does not step past a limit;
    /// zero disables that check). A non-finite twist is an error.
    pub fn step(&mut self, q: &[f64; N], twist: [f64; 6], dt: f64) -> RobotResult<ServoStep<N>> {
        if twist.iter().any(|v| !v.is_finite()) {
            return Err(invalid("servo twist is not finite".into()));
        }
        let mut v = na::Vector6::from(twist);
        if self.config.frame == ServoFrame::Tip {
            let rotation = self.model.forward_kinematics(*q)?.quat().rotation;
            let linear = rotation * v.fixed_rows::<3>(0);
            let angular = rotation * v.fixed_rows::<3>(3);
            v.fixed_rows_mut::<3>(0).copy_from(&linear);
            v.fixed_rows_mut::<3>(3).copy_from(&angular);
        }

        let j = self.model.jacobian(*q)?;
        let lambda = self.config.damping;
        let damped = j * j.transpose() + na::Matrix6::identity() * (lambda * lambda);
        let solved = damped
            .cholesky()
            .map(|c| c.solve(&v))
            .ok_or_else(|| invalid("servo Jacobian is degenerate; set a damping".into()))?;
        let raw = j.transpose() * solved;
        let singular = extreme_singular_values(&j);
        let condition = condition_number(singular);

        let mut scale = 1.0;
        let mut status = ServoStatus::Free;
        let mut limit = |factor: f64, cause: ServoStatus| {
            let factor = if factor.is_nan() {
                0.0
            } else {
                factor.clamp(0.0, 1.0)
            };
            if factor < scale {
                scale = factor;
                status = cause;
            }
        };

        let fastest = raw.amax();
        let ServoConfig { singularity_slowdown: slowdown, singularity_stop: stop, .. } =
            self.config;
        if fastest > 0.0 && condition > slowdown {
            // The motion leaves the singularity only if it raises the
            // smallest singular value beyond numerical noise, looking 1e-3
            // rad ahead on the fastest joint; otherwise it approaches it.
            let ahead: [f64; N] = std::array::from_fn(|i| q[i] + raw[i] * 1e-3 / fastest);
            let (_, smallest_ahead) = extreme_singular_values(&self.model.jacobian(ahead)?);
            let leaves = smallest_ahead > singular.1 + 1e-12 * singular.0;
            if !leaves {
                if condition >= stop {
                    limit(0.0, ServoStatus::SingularityStop);
                } else {
                    limit(
                        1.0 - (condition - slowdown) / (stop - slowdown),
                        ServoStatus::NearSingularity,
                    );
                }
            }
        }

        let margin = self.config.joint_limit_margin;
        for i in 0..N {
            let rate = raw[i];
            if rate == 0.0 {
                continue;
            }
            let room = if rate > 0.0 {
                self.limits.q_max[i] - q[i]
            } else {
                q[i] - self.limits.q_min[i]
            }
            .max(0.0);
            let ramp = if margin > 0.0 {
                room / margin
            } else if room > 0.0 {
                1.0
            } else {
                0.0
            };
            let reach = if dt > 0.0 {
                room / (rate.abs() * dt)
            } else {
                f64::INFINITY
            };
            limit(ramp.min(reach), ServoStatus::JointLimit(i));
        }

        if let Some(hook) = self.proximity.as_mut() {
            limit(hook(q), ServoStatus::Proximity);
        }

        for i in 0..N {
            if raw[i] != 0.0 {
                limit(
                    self.limits.dq_max[i] / raw[i].abs(),
                    ServoStatus::VelocityLimit(i),
                );
            }
        }

        Ok(ServoStep {
            dq: std::array::from_fn(|i| raw[i] * scale),
            scale,
            condition,
            status,
        })
    }

    fn period_seconds(&self, period: Duration) -> f64 {
        self.period.unwrap_or(period).as_secs_f64()
    }

    /// A controller for `JointVelocityControl<N>`. Each cycle `source` gives
    /// the twist and whether it is done; the servo follows it from the
    /// measured position. A missing measured position, a non-finite twist
    /// or a model error ends the session with `Break` (call
    /// [`step`](Self::step) directly to handle them otherwise).
    pub fn velocity_control<F>(
        mut self,
        mut source: F,
    ) -> impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; 6], bool),
    {
        move |state, period| {
            let Some(q) = state.meas.q() else {
                return ControlFlow::Break(missing("joint positions"));
            };
            let (twist, done) = source(state, period);
            let dt = self.period_seconds(period);
            match self.step(&q, twist, dt) {
                Ok(step) => ControlFlow::Continue((step.dq, done)),
                Err(error) => ControlFlow::Break(ControlExit::Failed(error)),
            }
        }
    }

    /// A controller for `JointPositionControl<N>`: integrates the servo's
    /// velocities from the measured position of the first cycle, then from
    /// its own previous command, so measurement noise and tracking lag do
    /// not accumulate. Ends like [`velocity_control`](Self::velocity_control).
    pub fn position_control<F>(
        mut self,
        mut source: F,
    ) -> impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; 6], bool),
    {
        let mut command: Option<[f64; N]> = None;
        move |state, period| {
            let Some(q) = command.or_else(|| state.meas.q()) else {
                return ControlFlow::Break(missing("joint positions"));
            };
            let (twist, done) = source(state, period);
            let dt = self.period_seconds(period);
            match self.step(&q, twist, dt) {
                Ok(step) => {
                    let next = std::array::from_fn(|i| q[i] + step.dq[i] * dt);
                    command = Some(next);
                    ControlFlow::Continue((next, done))
                }
                Err(error) => ControlFlow::Break(ControlExit::Failed(error)),
            }
        }
    }
}
