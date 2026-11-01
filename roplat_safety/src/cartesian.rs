//! Cartesian velocity channel.

use std::time::Duration;

use roplat_behavior::ArmState;
use roplat_msgs::CartesianLimits;

use crate::{Axis, CommandCheck, Quantity, SafetyViolation, bound};

/// Cartesian velocity commands `[vx, vy, vz, wx, wy, wz]`: linear and angular
/// speed within their limits, and their change from one command to the next
/// within the acceleration limits. The jerk bounds of [`CartesianLimits`] are
/// not checked.
///
/// The first command of a session is measured against the observed flange
/// twist, or against an explicit start for drivers that do not report one.
#[derive(Clone, Debug)]
pub struct CartesianVelocityCheck<const N: usize> {
    limits: CartesianLimits,
    start: Option<[f64; 6]>,
    previous: Option<[f64; 6]>,
}

fn split(twist: &[f64; 6]) -> (f64, f64) {
    let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
    (norm(&twist[..3]), norm(&twist[3..]))
}

impl<const N: usize> CartesianVelocityCheck<N> {
    pub fn new(limits: CartesianLimits) -> Self {
        Self { limits, start: None, previous: None }
    }

    /// The twist the first command of a session is measured against, instead
    /// of the observed flange twist.
    pub fn starting_at(mut self, twist: [f64; 6]) -> Self {
        self.start = Some(twist);
        self
    }

    fn evaluate(
        &self,
        arm: &ArmState<N>,
        dt: f64,
        twist: &[f64; 6],
    ) -> Result<[f64; 6], SafetyViolation> {
        let limits = &self.limits;
        if let Some(i) = twist.iter().position(|v| !v.is_finite()) {
            let axis = if i < 3 { Axis::Linear } else { Axis::Angular };
            return Err(SafetyViolation::NonFinite { quantity: Quantity::Velocity, axis });
        }
        let (linear, angular) = split(twist);
        bound::norm(
            Quantity::Velocity,
            Axis::Linear,
            linear,
            limits.linear_velocity_max,
        )?;
        bound::norm(
            Quantity::Velocity,
            Axis::Angular,
            angular,
            limits.angular_velocity_max,
        )?;
        let accelerating = limits.linear_acceleration_max != f64::INFINITY
            || limits.angular_acceleration_max != f64::INFINITY;
        let twist0 = match self.previous.or(self.start) {
            Some(twist0) => twist0,
            None => bound::measured(
                Quantity::Velocity,
                arm.flange.meas.twist().map(|t| t.to_array()),
                accelerating,
            )?,
        };
        let (linear, angular) = split(&std::array::from_fn(|i| twist[i] - twist0[i]));
        let (la, aa) = (
            limits.linear_acceleration_max,
            limits.angular_acceleration_max,
        );
        bound::norm_rate(Quantity::Acceleration, Axis::Linear, linear, la, dt)?;
        bound::norm_rate(Quantity::Acceleration, Axis::Angular, angular, aa, dt)?;
        Ok(*twist)
    }
}

impl<const N: usize> CommandCheck for CartesianVelocityCheck<N> {
    type Obs = ArmState<N>;
    type Command = [f64; 6];

    fn check(
        &mut self,
        arm: &ArmState<N>,
        dt: Duration,
        twist: &[f64; 6],
    ) -> Result<(), SafetyViolation> {
        let result = self.evaluate(arm, dt.as_secs_f64(), twist);
        self.previous = result.as_ref().ok().copied();
        result.map(drop)
    }

    fn reset(&mut self) {
        self.previous = None;
    }
}
