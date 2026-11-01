//! Motion limits as data: what a robot declares and what its motions use.
//!
//! A driver declares its envelope at compile time as data:
//! [`Joints::JOINT_LIMITS`] and [`EndPoint::CARTESIAN_LIMITS`] are the
//! [`JointLimits`] and [`CartesianLimits`] algorithms and safety checks take.
//! An unbounded entry is `f64::INFINITY` (`f64::NEG_INFINITY` for a lower
//! position bound). [`MotionLimits`] is what an [`Arm`](crate::Arm) applies to
//! the motions it plans itself.

use crate::{CartesianLimits, EndPoint, JointLimits, Joints, RobotException, RobotResult};

/// Limits and speed fraction an arm's own planned motions
/// ([`MoveTo`](crate::MoveTo), [`MoveTraj`](crate::MoveTraj)) respect.
///
/// Read with [`Arm::motion_limits`](crate::Arm::motion_limits), replace with
/// [`Arm::set_motion_limits`](crate::Arm::set_motion_limits), or apply to one
/// motion with [`Motion::with_limits`](crate::Motion::with_limits).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionLimits<const N: usize> {
    pub joint: JointLimits<N>,
    pub cartesian: CartesianLimits,
    /// Fraction in (0, 1] of the velocity, acceleration and jerk bounds a
    /// motion uses.
    pub scale: f64,
}

impl<const N: usize> MotionLimits<N> {
    /// The envelope `R` declares, at full speed.
    pub fn envelope<R: Joints<N> + EndPoint + ?Sized>() -> Self {
        Self {
            joint: R::JOINT_LIMITS,
            cartesian: R::CARTESIAN_LIMITS,
            scale: 1.0,
        }
    }

    /// The same limits at another speed fraction.
    pub fn scaled(self, scale: f64) -> Self {
        Self { scale, ..self }
    }

    /// The joint bounds a planner uses: velocity, acceleration and jerk times
    /// [`scale`](Self::scale); position, effort and effort-rate bounds as they
    /// are. An infinite bound stays infinite.
    pub fn scaled_joint(&self) -> JointLimits<N> {
        let scale = |bounds: [f64; N]| bounds.map(|bound| bound * self.scale);
        JointLimits {
            dq_max: scale(self.joint.dq_max),
            ddq_max: scale(self.joint.ddq_max),
            dddq_max: scale(self.joint.dddq_max),
            ..self.joint
        }
    }

    /// The Cartesian bounds a planner uses, each times [`scale`](Self::scale).
    pub fn scaled_cartesian(&self) -> CartesianLimits {
        let c = &self.cartesian;
        CartesianLimits {
            linear_velocity_max: c.linear_velocity_max * self.scale,
            linear_acceleration_max: c.linear_acceleration_max * self.scale,
            linear_jerk_max: c.linear_jerk_max * self.scale,
            angular_velocity_max: c.angular_velocity_max * self.scale,
            angular_acceleration_max: c.angular_acceleration_max * self.scale,
            angular_jerk_max: c.angular_jerk_max * self.scale,
        }
    }

    /// Accepts limits that only tighten `envelope`: a scale in (0, 1], each
    /// position range non-empty and inside the envelope's, and each magnitude
    /// bound positive and no larger than the envelope's. NaN anywhere refuses.
    pub fn within(&self, envelope: &Self) -> RobotResult<()> {
        if !(self.scale > 0.0 && self.scale <= 1.0) {
            return Err(refused(format!("scale {} is not in (0, 1]", self.scale)));
        }
        let (j, e) = (&self.joint, &envelope.joint);
        for i in 0..N {
            if !(j.q_min[i] >= e.q_min[i] && j.q_max[i] <= e.q_max[i] && j.q_min[i] < j.q_max[i]) {
                return Err(refused(format!(
                    "joint {i}: position range [{}, {}] is not inside [{}, {}]",
                    j.q_min[i], j.q_max[i], e.q_min[i], e.q_max[i]
                )));
            }
            for (what, bound, declared) in [
                ("dq_max", j.dq_max[i], e.dq_max[i]),
                ("ddq_max", j.ddq_max[i], e.ddq_max[i]),
                ("dddq_max", j.dddq_max[i], e.dddq_max[i]),
                ("tau_max", j.tau_max[i], e.tau_max[i]),
                ("dtau_max", j.dtau_max[i], e.dtau_max[i]),
            ] {
                tightens(&format!("joint {i}: {what}"), bound, declared)?;
            }
        }
        let (c, e) = (&self.cartesian, &envelope.cartesian);
        for (what, bound, declared) in [
            (
                "linear_velocity_max",
                c.linear_velocity_max,
                e.linear_velocity_max,
            ),
            (
                "linear_acceleration_max",
                c.linear_acceleration_max,
                e.linear_acceleration_max,
            ),
            ("linear_jerk_max", c.linear_jerk_max, e.linear_jerk_max),
            (
                "angular_velocity_max",
                c.angular_velocity_max,
                e.angular_velocity_max,
            ),
            (
                "angular_acceleration_max",
                c.angular_acceleration_max,
                e.angular_acceleration_max,
            ),
            ("angular_jerk_max", c.angular_jerk_max, e.angular_jerk_max),
        ] {
            tightens(what, bound, declared)?;
        }
        Ok(())
    }
}

fn tightens(what: &str, bound: f64, declared: f64) -> RobotResult<()> {
    if bound > 0.0 && bound <= declared {
        Ok(())
    } else {
        Err(refused(format!("{what} {bound} is not in (0, {declared}]")))
    }
}

fn refused(reason: String) -> RobotException {
    RobotException::UnprocessableInstructionError(format!("motion limits: {reason}"))
}
