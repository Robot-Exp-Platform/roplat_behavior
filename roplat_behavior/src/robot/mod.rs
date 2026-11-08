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
pub use device::{
    Camera, ForceTorqueSensor, Geodetic, GnssFix, GnssHeading, GnssReceiver, GnssStatus, GpsTime,
    Gripper, Heading, ImuSensor, Lidar, LidarPoint, frd_to_flu, ned_frd_to_enu_flu, ned_to_enu,
};
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

use crate::{RobotException, RobotResult};

/// # Robot: the single core behavior every device shares
///
/// `Robot` is the one root trait of the whole capability stack: it defines the
/// per-tick measured [`State`](Robot::State) plus a universally meaningful
/// lifecycle. Every capability trait has `Robot` as its super-trait,
/// so anything controllable can also be brought up, read and stopped.
///
/// Only [`State`](Robot::State), [`version`](Robot::version) and
/// [`read_state`](Robot::read_state) are required; the remaining lifecycle hooks
/// have no-op defaults so a minimal driver (or a simulator) overrides just
/// what it actually needs. The exception is
/// [`emergency_stop`](Robot::emergency_stop), whose default stops and then
/// reports that it could not latch the stop.
pub trait Robot {
    /// One frame of measured robot state.
    type State;

    /// The nominal period of the robot's control cycle. A control callback
    /// receives this period as its `Duration`, or `(k + 1)` times it when `k`
    /// cycles were missed before it.
    ///
    /// For a driver with a control channel ([`ControlWith`]) it is that
    /// channel's cycle period. A driver without one, which moves only point to
    /// point (Yaskawa HSES, for example), gives the period at which it
    /// refreshes the robot's state. The driver documents which.
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

    /// Engages the actuators so that motion and control calls work, restoring
    /// the power [`disable`](Robot::disable) may have cut.
    fn enable(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Disengages the actuators. It may cut their power, with the brakes
    /// engaged where the device has them; [`enable`](Robot::enable) restores
    /// it.
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

    /// Performs the strongest stop the device allows.
    ///
    /// Returns `Ok` only when the controller latches the stop, so that the
    /// robot stays stopped until
    /// [`clear_emergency_stop`](Robot::clear_emergency_stop). A device that
    /// cannot latch one still stops as strongly as it can, then returns an
    /// error saying it did not latch (`roplat_rtde`'s `NoEmergencyStop` is
    /// one); it never does less without saying so. The default is such a
    /// device: it calls [`stop`](Robot::stop) and, when that succeeds,
    /// returns
    /// [`UnprocessableInstructionError`](RobotException::UnprocessableInstructionError).
    fn emergency_stop(&mut self) -> RobotResult<()> {
        self.stop()?;
        Err(RobotException::UnprocessableInstructionError(
            "this driver cannot latch an emergency stop; it called stop instead".to_string(),
        ))
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
