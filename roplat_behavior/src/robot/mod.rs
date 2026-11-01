pub mod arm;
pub mod async_control;
pub mod category;
pub mod control;
pub mod device;
pub mod endpoint;
pub mod joint;
pub mod limits;
pub mod math;
pub mod model;
pub mod motion;
pub mod observe;
pub mod spaces;
pub mod state;
pub mod types;

pub use arm::{Arm, ArmState};
pub use async_control::{AsyncControl, AsyncControlCallback, AsyncControlWith};
pub use category::{Humanoid, MobileBase, Quadruped};
pub use control::{
    ArmTorqueControl, BalanceControl, BaseVelocityControl, CartesianPoseControl,
    CartesianVelocityControl, Control, ControlExit, ControlSpace, ControlStep, ControlWith,
    JointPositionControl, JointVelocityControl, TorqueControl,
};
pub use device::{Camera, ForceTorqueSensor, Gripper, ImuSensor};
pub use endpoint::{EndPoint, EndPointPose};
pub use joint::Joints;
pub use limits::MotionLimits;
pub use math::{Iso3, JMat, JVec, Jaco};
pub use model::{
    CoriolisInput, DynamicsModel, ForwardKinematics, GravityInput, InverseKinematics,
    JacobianModel, SpaceMap,
};
pub use motion::{Motion, MotionFile, MoveTo, MoveTraj, Space};
pub use observe::{ControlObservation, ControlObserver};
pub use roplat_msgs::{CartesianLimits, JointLimits, LoadState};
pub use spaces::{
    BasePoseSpace, BaseVelocitySpace, CenterOfMassSpace, CoriolisInputSpace, EndSpace, FlangeSpace,
    FootSpace, GaitCommand, GaitSpace, GravityInputSpace, HandSpace, JacobianSpace, JointSpace,
    JointTorqueSpace, MassMatrixSpace, Relative, RelativeSpace, TcpSpace, WholeBodyJointSpace,
    WholeBodyTorqueSpace, WholeBodyVelocitySpace,
};
pub use state::{
    BaseState, ContactSample, ContactState, EndEffectorState, HumanoidState, JointSample,
    JointState, MobileBaseState, QuadrupedState, SpatialSample, SpatialState, StateView,
};
pub use types::Pose;

use std::time::Duration;

use crate::RobotResult;

/// # Robot: the single core behavior every device shares
///
/// `Robot` is the one root trait of the whole capability stack: it defines the
/// per-tick measured [`State`](Robot::State) plus a universally meaningful
/// lifecycle. Every capability trait has `Robot` as its super-trait,
/// so anything controllable can also be brought up, read and stopped.
///
/// Only [`State`](Robot::State), [`version`](Robot::version) and
/// [`read_state`](Robot::read_state) are required; the remaining lifecycle hooks
/// have safe no-op defaults so a minimal driver (or a simulator) overrides just
/// what it actually needs.
pub trait Robot {
    /// One frame of measured robot state.
    type State;

    /// The nominal period of the robot's control cycle. A control callback
    /// receives this period as its `Duration`, or `(k + 1)` times it when `k`
    /// cycles were missed before it.
    const CONTROL_PERIOD: Duration;

    /// Driver / firmware version string.
    fn version() -> String;

    /// Read the latest measured state.
    fn read_state(&mut self) -> RobotResult<Self::State>;

    /// One-time bring-up (connect, allocate handles, ...).
    fn init(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Tear down and release resources.
    fn shutdown(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Power-on / engage actuators.
    fn enable(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Disengage actuators.
    fn disable(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Reset the robot to a known state.
    fn reset(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Whether the robot is currently executing a motion.
    fn is_moving(&mut self) -> RobotResult<bool> {
        Ok(false)
    }

    /// Block until the current action finishes.
    fn waiting_for_finish(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Stop the current action (recoverable).
    fn stop(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Pause the current action.
    fn pause(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Resume a paused action.
    fn resume(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Emergency stop (latched). Defaults to a plain [`stop`](Robot::stop).
    fn emergency_stop(&mut self) -> RobotResult<()> {
        self.stop()
    }

    /// Clear a latched emergency stop.
    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

/// A robot description, used for dynamic dispatch and configuration.
pub trait RobotDescription {
    /// A robot with a URDF file.
    /// This path is relative to the shared asset directory
    /// (`roplat_world::assets_dir`).
    const URDF: Option<&'static str> = None;

    /// Models available for simulation, optionally adapted to particular backends.
    ///
    /// A simulator backend (`roplat_world::SimulationBackend`) selects only formats its robot loader
    /// accepts. The legacy [`Self::URDF`] remains a final URDF-only fallback.
    /// Declaring a model does not implement any robot control capability or
    /// guarantee that two backends simulate equivalent dynamics.
    const SIMULATION_MODELS: &'static [crate::SimulationModel] = &[];
}
