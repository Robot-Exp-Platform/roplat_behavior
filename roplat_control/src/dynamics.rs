use std::ops::ControlFlow::{Break, Continue};
use std::time::Duration;

use roplat_behavior::{ControlExit, ControlStep, DynamicsModel, JVec, JointState};

use crate::exit::missing;

/// Builds a gravity-compensation torque controller.
///
/// The returned closure observes [`JointState<N>`] and returns the model's
/// gravity torque for the measured joint position. It is ready to pass into
/// `control_with_flow::<TorqueControl<N>, _>(...)` or a whole-body torque
/// channel.
///
/// The session ends with `Break` when the state has no measured joint
/// positions or the model query fails; no torque is fabricated.
pub fn gravity_compensation_control<M, const N: usize>(
    model: M,
    gravity: [f64; 3],
) -> impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static
where
    M: DynamicsModel<N> + Send + 'static,
{
    move |state, _duration| {
        let Some(q) = state.meas.q() else {
            return Break(missing("joint positions"));
        };
        match model.gravity(q, gravity) {
            Ok(torque) => Continue((torque, false)),
            Err(error) => Break(ControlExit::Failed(error)),
        }
    }
}

/// Builds a computed-torque controller around a fixed joint target.
///
/// This is the fixed-reference convenience wrapper over
/// [`computed_torque_tracking_control`]. It is useful for simple model-based
/// regulation when the desired joint position, velocity and acceleration are
/// known constants.
///
/// Command law:
///
/// `tau = M(q) * (ddq_ref + kp * e + kd * de) + C(q, dq) + G(q)`
///
/// The closure is intentionally plain `FnMut`, matching the rest of the
/// controller module, so callers can pass it directly to `control_with_flow`.
pub fn computed_torque_control<M, const N: usize>(
    model: M,
    target: [f64; N],
    target_vel: [f64; N],
    target_acc: [f64; N],
    kp: [f64; N],
    kd: [f64; N],
    gravity: [f64; 3],
) -> impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static
where
    M: DynamicsModel<N> + Send + 'static,
{
    computed_torque_tracking_control(
        model,
        move |_state, _duration| (target, target_vel, target_acc, false),
        kp,
        kd,
        gravity,
    )
}

/// Builds a computed-torque controller around a dynamic reference source.
///
/// `target_fn` returns `(q_ref, dq_ref, ddq_ref, done)`. The controller augments
/// the desired acceleration with PD feedback, multiplies by the joint-space
/// mass matrix and adds Coriolis / centrifugal and gravity terms from the
/// supplied [`DynamicsModel`].
///
/// The session ends with `Break` when the state lacks measured joint positions
/// or velocities, or when the mass, Coriolis or gravity query fails. A partial
/// model is a failure, not a zero term.
pub fn computed_torque_tracking_control<M, F, const N: usize>(
    model: M,
    mut target_fn: F,
    kp: [f64; N],
    kd: [f64; N],
    gravity: [f64; 3],
) -> impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static
where
    M: DynamicsModel<N> + Send + 'static,
    F: FnMut(JointState<N>, Duration) -> ([f64; N], [f64; N], [f64; N], bool) + Send + 'static,
{
    move |state, duration| {
        let (target, target_vel, target_acc, done) = target_fn(state, duration);
        let Some(q) = state.meas.q() else {
            return Break(missing("joint positions"));
        };
        let Some(dq) = state.meas.dq() else {
            return Break(missing("joint velocities"));
        };

        let mut desired_acc = target_acc;
        for i in 0..N {
            desired_acc[i] += kp[i] * (target[i] - q[i]) + kd[i] * (target_vel[i] - dq[i]);
        }

        let (mass, coriolis, gravity_torque) = match (
            model.mass(q),
            model.coriolis(q, dq),
            model.gravity(q, gravity),
        ) {
            (Ok(mass), Ok(coriolis), Ok(gravity_torque)) => (mass, coriolis, gravity_torque),
            (Err(error), _, _) | (_, Err(error), _) | (_, _, Err(error)) => {
                return Break(ControlExit::Failed(error));
            }
        };

        let inertial_torque = mass * JVec::<N>::from_column_slice(&desired_acc);
        let mut torque: [f64; N] = inertial_torque
            .as_slice()
            .try_into()
            .expect("a joint-space vector has N entries");
        for i in 0..N {
            torque[i] += coriolis[i] + gravity_torque[i];
        }

        Continue((torque, done))
    }
}
