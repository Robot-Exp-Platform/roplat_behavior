use crate::RobotResult;

use super::{
    Robot,
    control::{BaseVelocityControl, ControlWith, TorqueControl},
    joint::Joints,
    motion::MoveTo,
    spaces::{BasePoseSpace, CenterOfMassSpace, GaitSpace, WholeBodyJointSpace},
    state::{BaseState, HumanoidState, QuadrupedState},
};

/// A mobile or floating base capability bundle: it drives to poses and
/// reports its state.
///
/// This is the base-robot counterpart to [`Arm`](crate::Arm): it combines the
/// common capabilities expected from a base and leaves concrete kinematics to
/// the driver. A base that only takes goals, such as a MiR at mission level,
/// implements this alone; one that also takes a velocity every control cycle
/// implements [`ControlWith<BaseVelocityControl>`](ControlWith) too, which
/// makes it a [`VelocityControlledBase`].
pub trait MobileBase: Robot + MoveTo<BasePoseSpace> + Sized {
    /// Read a base-shaped state view.
    fn base_state(&mut self) -> RobotResult<BaseState>;
}

/// A [`MobileBase`] that also runs velocity control sessions, for code that
/// needs both. Every type with both capabilities has it; a driver implements
/// [`MobileBase`] and [`ControlWith<BaseVelocityControl>`](ControlWith), never
/// this trait.
///
/// A base that only drives to poses is a `MobileBase` but not this:
///
/// ```compile_fail
/// use std::time::Duration;
///
/// use roplat_behavior::{
///     BasePoseSpace, BaseState, MobileBase, MoveTo, Pose, Robot, RobotResult,
///     VelocityControlledBase,
/// };
///
/// struct Mission;
///
/// impl Robot for Mission {
///     type State = ();
///     const CONTROL_PERIOD: Duration = Duration::from_millis(100);
///     fn version() -> String {
///         String::new()
///     }
///     fn read_state(&mut self) -> RobotResult<()> {
///         Ok(())
///     }
/// }
///
/// impl MoveTo<BasePoseSpace> for Mission {
///     fn move_to(&mut self, _: Pose) -> RobotResult<()> {
///         Ok(())
///     }
/// }
///
/// impl MobileBase for Mission {
///     fn base_state(&mut self) -> RobotResult<BaseState> {
///         Ok(BaseState::default())
///     }
/// }
///
/// fn drive(_: &mut impl VelocityControlledBase) {}
///
/// drive(&mut Mission);
/// ```
pub trait VelocityControlledBase: MobileBase + ControlWith<BaseVelocityControl> {}

impl<T: MobileBase + ControlWith<BaseVelocityControl>> VelocityControlledBase for T {}

/// A quadruped capability bundle with `N` actuated joints.
///
/// The trait deliberately stays small: gait, whole-body joint motion, and
/// whole-body torque control are the first common surfaces expected across
/// quadruped backends.
pub trait Quadruped<const N: usize>:
    Robot
    + Joints<N>
    + MoveTo<GaitSpace>
    + MoveTo<WholeBodyJointSpace<N>>
    + ControlWith<TorqueControl<N>>
    + Sized
{
    /// Read the full quadruped state view.
    fn state(&mut self) -> RobotResult<QuadrupedState<N>>;
}

/// A humanoid capability bundle with `N` actuated joints.
///
/// This is only the first shared surface: whole-body joint motion, center of
/// mass motion, and whole-body torque control.
pub trait Humanoid<const N: usize>:
    Robot
    + Joints<N>
    + MoveTo<WholeBodyJointSpace<N>>
    + MoveTo<CenterOfMassSpace>
    + ControlWith<TorqueControl<N>>
    + Sized
{
    /// Read the full humanoid state view.
    fn state(&mut self) -> RobotResult<HumanoidState<N>>;
}
