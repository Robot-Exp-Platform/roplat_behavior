use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

use nalgebra as na;

use std::ops::ControlFlow::{Break, Continue};

use crate::exit::missing;
use crate::target::{
    EmptyTrajectory, constant_joint_target_fn, constant_pose_target_fn, joint_traj_target_fn,
    pose_traj_target_fn,
};
use roplat_behavior::{
    ArmState, ControlExit, ControlStep, ForwardKinematics, JVec, JacobianModel, JointState, Pose,
};

#[derive(Debug, Clone)]
pub struct JointImpedanceSessionHandle<const N: usize> {
    target: Arc<Mutex<Option<[f64; N]>>>,
    stiffness: Arc<Mutex<[f64; N]>>,
    damping: Arc<Mutex<[f64; N]>>,
    is_finished: Arc<AtomicBool>,
}

impl<const N: usize> JointImpedanceSessionHandle<N> {
    /// Replaces the current joint target.
    ///
    /// Passing `None` makes the controller hold the measured joint position of
    /// the next cycle. This is useful for "engage and hold" interaction modes.
    pub fn set_target(&self, target: Option<[f64; N]>) {
        *self.target.lock().unwrap() = target;
    }

    /// Updates per-joint stiffness gains used by subsequent control cycles.
    pub fn set_stiffness(&self, stiffness: [f64; N]) {
        *self.stiffness.lock().unwrap() = stiffness;
    }

    /// Updates per-joint damping gains used by subsequent control cycles.
    pub fn set_damping(&self, damping: [f64; N]) {
        *self.damping.lock().unwrap() = damping;
    }

    /// Requests the session controller to report `done = true`.
    pub fn finish(&self) {
        self.is_finished.store(true, Ordering::SeqCst);
    }
}

#[derive(Debug, Clone)]
pub struct CartesianImpedanceSessionHandle {
    target: Arc<Mutex<Option<Pose>>>,
    stiffness: Arc<Mutex<[f64; 6]>>,
    damping: Arc<Mutex<[f64; 6]>>,
    is_finished: Arc<AtomicBool>,
}

impl CartesianImpedanceSessionHandle {
    /// Replaces the Cartesian equilibrium pose.
    ///
    /// Passing `None` makes the controller use the measured flange pose on the
    /// next cycle as its equilibrium.
    pub fn set_target(&self, target: Option<Pose>) {
        *self.target.lock().unwrap() = target;
    }

    /// Updates translational / rotational stiffness gains.
    pub fn set_stiffness(&self, stiffness: [f64; 6]) {
        *self.stiffness.lock().unwrap() = stiffness;
    }

    /// Updates translational / rotational damping gains.
    pub fn set_damping(&self, damping: [f64; 6]) {
        *self.damping.lock().unwrap() = damping;
    }

    /// Requests the session controller to report `done = true`.
    pub fn finish(&self) {
        self.is_finished.store(true, Ordering::SeqCst);
    }
}

/// Computes joint-space impedance torque for one measured joint position and
/// velocity.
///
/// The function is pure and does not own any controller state. It is useful for
/// tests, custom closures, or drivers that need to embed the torque calculation
/// into a larger control law.
///
/// Control law per joint:
///
/// `tau = stiffness * (q_ref - q) - damping * dq`
pub fn joint_space_impedance_torque<const N: usize>(
    target: [f64; N],
    stiffness: [f64; N],
    damping: [f64; N],
    q: &[f64; N],
    dq: &[f64; N],
) -> [f64; N] {
    let mut torque = [0.0; N];

    for i in 0..N {
        torque[i] = stiffness[i] * (target[i] - q[i]) - damping[i] * dq[i];
    }

    torque
}

/// Builds a joint-space impedance controller around one fixed equilibrium.
///
/// The returned closure observes [`JointState<N>`] and returns joint torque.
/// Use it with `TorqueControl<N>` or `TorqueControl<N>`.
pub fn joint_impedance_control<const N: usize>(
    target: [f64; N],
    stiffness: [f64; N],
    damping: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static {
    joint_impedance_tracking_control(constant_joint_target_fn(target), stiffness, damping)
}

/// Builds a joint-space impedance controller for a sampled joint trajectory.
///
/// This is the trajectory-following variant: the equilibrium target advances
/// through `traj` by delegating to [`joint_traj_target_fn`].
pub fn joint_traj_impedance_control<const N: usize>(
    traj: Vec<[f64; N]>,
    stiffness: [f64; N],
    damping: [f64; N],
) -> Result<
    impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static,
    EmptyTrajectory,
> {
    Ok(joint_impedance_tracking_control(
        joint_traj_target_fn(traj)?,
        stiffness,
        damping,
    ))
}

/// Builds a joint-space impedance controller around a dynamic target source.
///
/// `target_fn` receives the same [`JointState<N>`] and cycle duration as the
/// controller. It may hold a fixed target, follow a trajectory, query a handle,
/// or run an online generator.
pub fn joint_impedance_tracking_control<F, const N: usize>(
    mut target_fn: F,
    stiffness: [f64; N],
    damping: [f64; N],
) -> impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static
where
    F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
{
    move |state, duration| {
        let (target, done) = target_fn(state, duration);
        let Some(q) = state.meas.q() else {
            return Break(missing("joint positions"));
        };
        let Some(dq) = state.meas.dq() else {
            return Break(missing("joint velocities"));
        };
        Continue((
            joint_space_impedance_torque(target, stiffness, damping, &q, &dq),
            done,
        ))
    }
}

/// Builds a handle-driven joint impedance session.
///
/// The returned `(controller, handle)` pair is meant for interactive tasks:
/// spawn the controller in a realtime loop, then adjust target, stiffness or
/// damping from another task through the handle. Calling `finish` makes the
/// controller return `done = true`.
pub fn joint_impedance_session<const N: usize>(
    initial_target: Option<[f64; N]>,
    stiffness: [f64; N],
    damping: [f64; N],
) -> (
    impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static,
    JointImpedanceSessionHandle<N>,
) {
    let handle = JointImpedanceSessionHandle {
        target: Arc::new(Mutex::new(initial_target)),
        stiffness: Arc::new(Mutex::new(stiffness)),
        damping: Arc::new(Mutex::new(damping)),
        is_finished: Arc::new(AtomicBool::new(false)),
    };

    let target = handle.target.clone();
    let stiffness = handle.stiffness.clone();
    let damping = handle.damping.clone();
    let is_finished = handle.is_finished.clone();

    let controller = move |state: JointState<N>, _duration: Duration| {
        let Some(q) = state.meas.q() else {
            return Break(missing("joint positions"));
        };
        let Some(dq) = state.meas.dq() else {
            return Break(missing("joint velocities"));
        };
        // No target holds the arm where it is measured.
        let target = target.lock().unwrap().unwrap_or(q);
        let stiffness = *stiffness.lock().unwrap();
        let damping = *damping.lock().unwrap();
        let torque = joint_space_impedance_torque(target, stiffness, damping, &q, &dq);
        Continue((torque, is_finished.load(Ordering::SeqCst)))
    };

    (controller, handle)
}

/// Builds a Cartesian impedance torque controller around a dynamic pose source.
///
/// The closure observes the full [`ArmState<N>`] because Cartesian impedance
/// needs joint state, flange pose / twist and a model capable of Jacobian / FK
/// queries. Use the returned closure with `ArmTorqueControl<N>`.
pub fn cartesian_impedance_tracking_control<M, F, const N: usize>(
    model: M,
    mut target_fn: F,
    stiffness: [f64; 6],
    damping: [f64; 6],
) -> impl FnMut(ArmState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static
where
    M: ForwardKinematics<N> + JacobianModel<N> + Send + 'static,
    F: FnMut(ArmState<N>, Duration) -> (Pose, bool) + Send + 'static,
{
    move |state, duration| {
        let (target, done) = target_fn(state.clone(), duration);
        match cartesian_impedance_torque(&model, target, stiffness, damping, &state) {
            Ok(torque) => Continue((torque, done)),
            Err(exit) => Break(exit),
        }
    }
}

/// Computes a Cartesian impedance wrench in task space.
///
/// The pose error is computed in SE(3): translational error from the position
/// delta and rotational error from the relative quaternion scaled axis. The
/// returned wrench is `[fx, fy, fz, tx, ty, tz]`.
pub fn cartesian_space_impedance_wrench(
    target: Pose,
    stiffness: [f64; 6],
    damping: [f64; 6],
    current: Pose,
    current_velocity: [f64; 6],
) -> [f64; 6] {
    let target_iso = target.quat();
    let current_iso = current.quat();

    let position_error = target_iso.translation.vector - current_iso.translation.vector;
    let rotation_error = (target_iso.rotation * current_iso.rotation.inverse()).scaled_axis();

    let error = [
        position_error.x,
        position_error.y,
        position_error.z,
        rotation_error.x,
        rotation_error.y,
        rotation_error.z,
    ];

    let mut wrench = [0.0; 6];
    for i in 0..6 {
        wrench[i] = stiffness[i] * error[i] - damping[i] * current_velocity[i];
    }
    wrench
}

/// Builds a Cartesian impedance controller around one fixed equilibrium pose.
///
/// This is the fixed-target convenience wrapper over
/// [`cartesian_impedance_tracking_control`].
pub fn cartesian_impedance_control<M, const N: usize>(
    model: M,
    target: Pose,
    stiffness: [f64; 6],
    damping: [f64; 6],
) -> impl FnMut(ArmState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static
where
    M: ForwardKinematics<N> + JacobianModel<N> + Send + 'static,
{
    cartesian_impedance_tracking_control(
        model,
        constant_pose_target_fn::<N>(target),
        stiffness,
        damping,
    )
}

/// Builds a Cartesian impedance controller for a sampled pose trajectory.
pub fn cartesian_traj_impedance_control<M, const N: usize>(
    model: M,
    traj: Vec<Pose>,
    stiffness: [f64; 6],
    damping: [f64; 6],
) -> Result<
    impl FnMut(ArmState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static,
    EmptyTrajectory,
>
where
    M: ForwardKinematics<N> + JacobianModel<N> + Send + 'static,
{
    Ok(cartesian_impedance_tracking_control(
        model,
        pose_traj_target_fn(traj)?,
        stiffness,
        damping,
    ))
}

/// Builds a handle-driven Cartesian impedance session.
///
/// The handle can update the equilibrium pose and gains while the realtime loop
/// is running. Passing `None` as the initial target makes the first cycle hold
/// the measured flange pose.
pub fn cartesian_impedance_session<M, const N: usize>(
    model: M,
    initial_target: Option<Pose>,
    stiffness: [f64; 6],
    damping: [f64; 6],
) -> (
    impl FnMut(ArmState<N>, Duration) -> ControlStep<[f64; N]> + Send + 'static,
    CartesianImpedanceSessionHandle,
)
where
    M: ForwardKinematics<N> + JacobianModel<N> + Send + 'static,
{
    let handle = CartesianImpedanceSessionHandle {
        target: Arc::new(Mutex::new(initial_target)),
        stiffness: Arc::new(Mutex::new(stiffness)),
        damping: Arc::new(Mutex::new(damping)),
        is_finished: Arc::new(AtomicBool::new(false)),
    };

    let target = handle.target.clone();
    let stiffness = handle.stiffness.clone();
    let damping = handle.damping.clone();
    let is_finished = handle.is_finished.clone();

    let controller = move |state: ArmState<N>, _duration: Duration| {
        // No target holds the flange where it is measured.
        let target = match *target.lock().unwrap() {
            Some(target) => target,
            None => match state.flange.meas.pose() {
                Some(pose) => pose,
                None => return Break(missing("flange pose")),
            },
        };
        let stiffness = *stiffness.lock().unwrap();
        let damping = *damping.lock().unwrap();
        match cartesian_impedance_torque(&model, target, stiffness, damping, &state) {
            Ok(torque) => Continue((torque, is_finished.load(Ordering::SeqCst))),
            Err(exit) => Break(exit),
        }
    };

    (controller, handle)
}

/// The torque for one cycle, or why the session must end: a missing measured
/// joint position or velocity, or a failed model query. The flange pose and
/// twist come from the measurement when present and from the model otherwise.
fn cartesian_impedance_torque<M, const N: usize>(
    model: &M,
    target: Pose,
    stiffness: [f64; 6],
    damping: [f64; 6],
    state: &ArmState<N>,
) -> Result<[f64; N], ControlExit>
where
    M: ForwardKinematics<N> + JacobianModel<N>,
{
    let Some(q) = state.joint.meas.q() else {
        return Err(missing("joint positions"));
    };
    let Some(dq) = state.joint.meas.dq() else {
        return Err(missing("joint velocities"));
    };
    let current_pose = match state.flange.meas.pose() {
        Some(pose) => pose,
        None => model.forward_kinematics(q).map_err(ControlExit::Failed)?,
    };
    let current_velocity: [f64; 6] = match state.flange.meas.twist() {
        Some(twist) => twist.into(),
        None => model.twist(q, dq).map_err(ControlExit::Failed)?.into(),
    };

    let wrench = cartesian_space_impedance_wrench(
        target,
        stiffness,
        damping,
        current_pose,
        current_velocity,
    );
    let jacobian = model.jacobian(q).map_err(ControlExit::Failed)?;
    let torque = jacobian.transpose() * na::SVector::<f64, 6>::from_column_slice(&wrench);
    let torque: [f64; N] = JVec::<N>::from_column_slice(torque.as_slice())
        .as_slice()
        .try_into()
        .expect("a joint-space vector has N entries");
    Ok(torque)
}
