use crate::RobotResult;

use super::{
    Robot,
    control::{BaseVelocityControl, ControlWith, TorqueControl},
    joint::Joints,
    motion::MoveTo,
    spaces::{BasePoseSpace, CenterOfMassSpace, GaitSpace, WholeBodyJointSpace},
    state::{BaseState, HumanoidState, QuadrupedState},
};

/// A mobile or floating base capability bundle.
///
/// This is the base-robot counterpart to [`Arm`](crate::Arm): it combines the
/// common capabilities expected from a base and leaves concrete kinematics to
/// the driver.
pub trait MobileBase:
    Robot + MoveTo<BasePoseSpace> + ControlWith<BaseVelocityControl> + Sized
{
    /// Read a base-shaped state view.
    fn base_state(&mut self) -> RobotResult<BaseState>;
}

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
