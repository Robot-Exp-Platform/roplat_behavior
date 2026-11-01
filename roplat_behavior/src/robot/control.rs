use std::{ops::ControlFlow, time::Duration};

use futures::executor;

use crate::{
    Robot, RobotException, RobotResult,
    robot::{
        arm::ArmState,
        state::{BaseState, JointState},
        types::Pose,
    },
};

/// Type-level mapping from a realtime *control channel* to its observation and
/// command types.
///
/// This is the control-side mirror of
/// [`Space`](crate::robot::motion::Space): it has **no methods** and
/// only fixes the [`Obs`](ControlSpace::Obs) / [`Command`](ControlSpace::Command)
/// pair. Because the channel is a *type-level marker*, two channels whose command
/// payloads share the same Rust type, e.g. joint torque and joint position are
/// both `[f64; N]`, they stay distinguishable, so a single device can implement one
/// [`ControlWith`] impl per channel without coherence conflicts.
pub trait ControlSpace {
    /// The observed-state type handed to the closure each control cycle.
    type Obs;
    /// The command type the closure returns each control cycle.
    type Command;
}

/// Realtime joint-**torque** channel. Observes [`JointState<N>`] and commands a
/// per-joint torque vector `[f64; N]`.
pub struct TorqueControl<const N: usize>;
/// Realtime arm torque channel for controllers that need full arm context.
pub struct ArmTorqueControl<const N: usize>;
/// Realtime joint-**position** channel. Observes [`JointState<N>`] and commands a
/// per-joint position vector `[f64; N]`.
pub struct JointPositionControl<const N: usize>;
/// Realtime joint-**velocity** channel. Observes [`JointState<N>`] and commands a
/// per-joint velocity vector `[f64; N]`.
pub struct JointVelocityControl<const N: usize>;
/// Realtime **cartesian-velocity** channel. Observes [`ArmState<N>`] and commands
/// a 6-vector spatial velocity `[vx, vy, vz, wx, wy, wz]`.
pub struct CartesianVelocityControl<const N: usize>;
/// Realtime **cartesian-pose** channel. Observes [`ArmState<N>`] and commands a
/// flange / end-effector [`Pose`].
pub struct CartesianPoseControl<const N: usize>;
/// Realtime mobile / floating base velocity channel. Observes [`BaseState`] and
/// commands `[vx, vy, vz, wx, wy, wz]`.
pub struct BaseVelocityControl;
/// Realtime balance channel. The command shape is deliberately small until a
/// concrete humanoid or legged consumer needs richer balance commands.
pub struct BalanceControl;

impl<const N: usize> ControlSpace for TorqueControl<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];
}

impl<const N: usize> ControlSpace for ArmTorqueControl<N> {
    type Obs = ArmState<N>;
    type Command = [f64; N];
}

impl<const N: usize> ControlSpace for JointPositionControl<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];
}

impl<const N: usize> ControlSpace for JointVelocityControl<N> {
    type Obs = JointState<N>;
    type Command = [f64; N];
}

impl<const N: usize> ControlSpace for CartesianVelocityControl<N> {
    type Obs = ArmState<N>;
    type Command = [f64; 6];
}

impl<const N: usize> ControlSpace for CartesianPoseControl<N> {
    type Obs = ArmState<N>;
    type Command = Pose;
}

impl ControlSpace for BaseVelocityControl {
    type Obs = BaseState;
    type Command = [f64; 6];
}

impl ControlSpace for BalanceControl {
    type Obs = BaseState;
    type Command = [f64; 6];
}

/// Why a control closure ends the session without another command.
#[derive(Debug)]
pub enum ControlExit {
    /// A cooperative stop: the session ends and `control_with_flow` returns
    /// `Ok(())`.
    Stopped,
    /// The controller failed: the session ends and `control_with_flow` returns
    /// this error. Nothing is fabricated in its place.
    Failed(RobotException),
}

impl From<RobotException> for ControlExit {
    fn from(error: RobotException) -> Self {
        Self::Failed(error)
    }
}

/// One control cycle's decision. `Continue((command, done))` sends the valid
/// command; `Break(exit)` sends no command and ends the session with `exit`.
/// The command and `done` flag exist only for successfully computed cycles.
pub type ControlStep<Command> = ControlFlow<ControlExit, (Command, bool)>;

/// Realtime, closure-driven control - the bridge into blocking control loops.
/// **Implemented by drivers**, one impl per control channel `S`.
///
/// Generic over the [`ControlSpace`] channel `S`, which fixes both the observed
/// state and the command type. The observation is decoupled from
/// [`Robot::State`](crate::Robot::State) on purpose: a driver's lifecycle
/// `State` may be a rich native struct, while the realtime loop observes a
/// task-shaped view. The closure is invoked once per control cycle with the
/// freshly observed state and the cycle's `Duration`, and returns the next
/// command together with a `done` flag.
///
/// The `Duration` is the robot's nominal [`CONTROL_PERIOD`] when no cycle was
/// missed, and `(k + 1)` times it when `k` cycles were missed before this one,
/// so summing it gives the time since the session began. The first cycle
/// reports the nominal period.
///
/// [`CONTROL_PERIOD`]: crate::Robot::CONTROL_PERIOD
///
/// `control_with` runs a control relation until the closure reports `done = true`
/// or the driver returns an error. The closure is scoped to the call, so drivers
/// should not store it after `control_with` returns.
pub trait ControlWith<S: ControlSpace>: Robot {
    /// Build a continuity-preserving fallback command from the latest observed
    /// state.
    ///
    /// This is deliberately not named "safe": physical safety belongs to the
    /// driver/controller stack, often using the previous command in `obs` to
    /// rate-limit the next command. `hold_command` only answers what this
    /// channel should emit when upstream has no fresh output for this cycle.
    fn hold_command(obs: &S::Obs) -> S::Command
    where
        Self: Sized;

    /// Canonical blocking control session, implemented by each driver.
    ///
    /// `Continue((command, done))` sends the valid command, then ends normally
    /// if `done`. `Break(exit)` sends no algorithm command for this cycle and
    /// enters the driver's session termination protocol; the session then
    /// returns `Ok(())` for [`ControlExit::Stopped`] and `Err(error)` for
    /// [`ControlExit::Failed`]. The driver must not invoke the callback again
    /// after either exit. Protocol-specific termination packets remain the
    /// driver's responsibility; `Break` is not an emergency stop.
    fn control_with_flow<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(S::Obs, Duration) -> ControlStep<S::Command>;

    /// Run a blocking control session with a command on every completed cycle.
    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(S::Obs, Duration) -> (S::Command, bool),
    {
        self.control_with_flow(move |obs, duration| ControlFlow::Continue(closure(obs, duration)))
    }

    /// Blocking control session with an async, exit-capable per-cycle callback.
    ///
    /// The default runs each callback future to completion inside the blocking
    /// loop. The method itself does not return a Future. Drivers with their own
    /// session runtime can override this primitive while keeping the same contract.
    fn control_with_flow_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(S::Obs, Duration) -> ControlStep<S::Command>,
    {
        self.control_with_flow(move |obs, duration| executor::block_on(closure(obs, duration)))
    }

    /// Blocking control session accepting an async per-cycle callback.
    ///
    /// This preserves the 0.6 session semantics: the call returns only after
    /// completion or failure. It does not make the outer session asynchronous.
    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(S::Obs, Duration) -> (S::Command, bool),
    {
        self.control_with_flow_async(async move |obs, duration| {
            ControlFlow::Continue(closure(obs, duration).await)
        })
    }
}

/// Ergonomic, channel-parameterised entry point. **Blanket-implemented for every
/// robot** - users never implement this trait, they only call it:
///
/// ```ignore
/// arm.control_with::<TorqueControl<7>, _>(|s, dt| (tau, false))?;
/// arm.control_with::<JointPositionControl<7>, _>(|s, dt| (q, false))?;
/// ```
///
/// The channel `S` is a method-level generic disambiguated by turbofish, so
/// channels that share a command type (torque vs. position, both `[f64; N]`)
/// never collide. Each call forwards to the [`ControlWith<S>`] impl the
/// driver provides for that channel.
pub trait Control: Robot + Sized {
    /// Blocking session with an explicit no-command exit path.
    fn control_with_flow<S, F>(&mut self, closure: F) -> RobotResult<()>
    where
        S: ControlSpace,
        Self: ControlWith<S>,
        F: FnMut(S::Obs, Duration) -> ControlStep<S::Command>,
    {
        <Self as ControlWith<S>>::control_with_flow(self, closure)
    }

    /// Async callback variant of [`Control::control_with_flow`]; still blocking.
    fn control_with_flow_async<S, F>(&mut self, closure: F) -> RobotResult<()>
    where
        S: ControlSpace,
        Self: ControlWith<S>,
        F: async FnMut(S::Obs, Duration) -> ControlStep<S::Command>,
    {
        <Self as ControlWith<S>>::control_with_flow_async(self, closure)
    }

    fn control_with<S, F>(&mut self, closure: F) -> RobotResult<()>
    where
        S: ControlSpace,
        Self: ControlWith<S>,
        F: FnMut(S::Obs, Duration) -> (S::Command, bool),
    {
        <Self as ControlWith<S>>::control_with(self, closure)
    }

    /// The continuity fallback command for channel `S`. See
    /// [`ControlWith::hold_command`].
    fn hold_command<S>(&self, obs: &S::Obs) -> S::Command
    where
        S: ControlSpace,
        Self: ControlWith<S> + Sized,
    {
        <Self as ControlWith<S>>::hold_command(obs)
    }

    /// Blocking async-closure facade for channel `S`.
    /// See [`ControlWith::control_with_async`].
    fn control_with_async<S, F>(&mut self, closure: F) -> RobotResult<()>
    where
        S: ControlSpace,
        Self: ControlWith<S>,
        F: async FnMut(S::Obs, Duration) -> (S::Command, bool),
    {
        <Self as ControlWith<S>>::control_with_async(self, closure)
    }
}

impl<R: Robot> Control for R {}
