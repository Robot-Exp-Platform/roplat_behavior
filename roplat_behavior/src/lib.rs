//! Typed robot capability contracts.
//!
//! Drivers implement [`Robot`] plus the motion ([`MoveTo`], [`MoveTraj`]) and
//! control ([`ControlWith`], [`AsyncControlWith`]) traits for the spaces and
//! channels they support; applications program against those traits. Sample and
//! geometry types come from [`msgs`] (`roplat_msgs`). Kinematics, controllers
//! and trajectories live in `roplat_kinematics`, `roplat_control` and
//! `roplat_trajectory`; simulators, physics engines and renderers in
//! `roplat_world`. Feature `roplat` adds `Node` / `Rhythm` adapters.

#![feature(async_trait_bounds)]

#[cfg(feature = "conformance")]
pub mod conformance;
mod exception;

mod robot;
#[cfg(feature = "roplat")]
pub mod roplat;
mod simulation;
pub mod utils;

pub use exception::*;
pub use robot::*;
/// The shared message vocabulary, re-exported.
pub use roplat_msgs as msgs;
pub use roplat_msgs::{Accel, FrameId, Header, Quaternion, Stamp, Stamped, Twist, Vector3, Wrench};
pub use simulation::*;
pub use utils::*;
pub mod behavior {
    pub use crate::robot::{
        Arm, ArmState, ArmTorqueControl, AsyncControl, AsyncControlCallback, BalanceControl,
        BasePoseSpace, BaseState, BaseVelocityControl, BaseVelocitySpace, Camera, CartesianLimits,
        CartesianPoseControl, CartesianVelocityControl, CenterOfMassSpace, ContactState, Control,
        ControlExit, ControlObservation, ControlObserver, ControlSpace, ControlStep, CoriolisInput,
        CoriolisInputSpace, DynamicsModel, EndEffectorState, EndPoint, EndPointPose, EndSpace,
        FlangeSpace, FootSpace, ForceTorqueSensor, ForwardKinematics, GaitCommand, GaitSpace,
        Geodetic, GnssFix, GnssHeading, GnssReceiver, GnssStatus, GpsTime, GravityInput,
        GravityInputSpace, Gripper, HandSpace, Heading, Humanoid, HumanoidState, ImuSensor,
        InverseKinematics, JacobianModel, JacobianSpace, JointLimits, JointPositionControl,
        JointSample, JointSpace, JointState, JointTorqueSpace, JointVelocityControl, Joints, Lidar,
        LidarPoint, LoadState, MassMatrixSpace, MobileBase, MobileBaseState, Motion, MotionFile,
        MotionLimits, Pose, Quadruped, QuadrupedState, Relative, RelativeSpace, Robot,
        RobotDescription, Space, SpaceMap, SpatialSample, StateView, TcpSpace, TorqueControl,
        WholeBodyJointSpace, WholeBodyTorqueSpace, WholeBodyVelocitySpace,
    };

    pub use crate::simulation::{ModelFormat, SimulationModel};
}

pub mod driver {
    pub use crate::behavior::*;
    pub use crate::robot::{AsyncControlWith, ControlWith, MoveTo, MoveTraj};
}
