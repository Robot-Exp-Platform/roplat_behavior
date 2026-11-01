//! Every Python macro expands against a two-joint mock arm. A pyo3 extension
//! module does not link a test binary, so this is checked with
//! `cargo check --tests --features py`, not run.
#![cfg(feature = "py")]

use roplat_behavior::JointLimits;
use std::time::Duration;

use pyo3::{pyclass, pymethods};
use roplat_behavior::{
    Arm, ArmState, ArmTorqueControl, CartesianPoseControl, CartesianVelocityControl, ControlSpace,
    ControlStep, ControlWith, EndPoint, EndPointPose, FlangeSpace, JointPositionControl,
    JointSpace, JointVelocityControl, Joints, LoadState, MotionLimits, MoveTo, MoveTraj, Pose,
    Robot, RobotResult, TorqueControl,
};
use roplat_behavior_ffi::{
    py_arm, py_arm_torque_control, py_cartesian_pose_control, py_cartesian_velocity_control,
    py_flange_motion, py_joint_motion, py_joint_position_control, py_joint_torque_control,
    py_joint_velocity_control, py_robot,
};

pub struct Mock {
    limits: MotionLimits<2>,
}

impl Robot for Mock {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "mock".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl Joints<2> for Mock {
    const JOINT_LIMITS: JointLimits<2> = JointLimits::positions([-1.0; 2], [1.0; 2]);
}

impl EndPoint for Mock {}

impl EndPointPose for Mock {
    fn get_endpoint(&self) -> Pose {
        Pose::default()
    }
}

impl MoveTo<JointSpace<2>> for Mock {
    fn move_to(&mut self, _target: [f64; 2]) -> RobotResult<()> {
        Ok(())
    }
}

impl MoveTo<FlangeSpace> for Mock {
    fn move_to(&mut self, _target: Pose) -> RobotResult<()> {
        Ok(())
    }
}

impl MoveTraj<JointSpace<2>> for Mock {
    fn move_traj(&mut self, _traj: Vec<[f64; 2]>) -> RobotResult<()> {
        Ok(())
    }
    fn move_path<F: Fn(f64) -> Option<[f64; 2]>>(&mut self, _path: F) -> RobotResult<()> {
        Ok(())
    }
    fn move_waypoints(&mut self, _waypoints: Vec<[f64; 2]>) -> RobotResult<()> {
        Ok(())
    }
}

impl MoveTraj<FlangeSpace> for Mock {
    fn move_traj(&mut self, _traj: Vec<Pose>) -> RobotResult<()> {
        Ok(())
    }
    fn move_path<F: Fn(f64) -> Option<Pose>>(&mut self, _path: F) -> RobotResult<()> {
        Ok(())
    }
    fn move_waypoints(&mut self, _waypoints: Vec<Pose>) -> RobotResult<()> {
        Ok(())
    }
}

impl Arm<2> for Mock {
    fn state(&mut self) -> RobotResult<ArmState<2>> {
        Ok(ArmState::default())
    }
    fn set_load(&mut self, _load: LoadState) -> RobotResult<()> {
        Ok(())
    }
    fn get_joint(&self) -> [f64; 2] {
        [0.0; 2]
    }
    fn motion_limits(&self) -> MotionLimits<2> {
        self.limits
    }
    fn set_motion_limits(&mut self, limits: MotionLimits<2>) -> RobotResult<()> {
        self.limits = limits;
        Ok(())
    }
}

macro_rules! control_with {
    ($($channel:ty => $hold:expr;)*) => {$(
        impl ControlWith<$channel> for Mock {
            fn hold_command(_obs: &<$channel as ControlSpace>::Obs)
                -> <$channel as ControlSpace>::Command {
                $hold
            }
            fn control_with_flow<F>(&mut self, _closure: F) -> RobotResult<()>
            where
                F: FnMut(<$channel as ControlSpace>::Obs, Duration)
                    -> ControlStep<<$channel as ControlSpace>::Command>,
            {
                Ok(())
            }
        }
    )*};
}

control_with! {
    JointPositionControl<2> => [0.0; 2];
    JointVelocityControl<2> => [0.0; 2];
    TorqueControl<2> => [0.0; 2];
    ArmTorqueControl<2> => [0.0; 2];
    CartesianVelocityControl<2> => [0.0; 6];
    CartesianPoseControl<2> => Pose::default();
}

#[pyclass(name = "Mock")]
pub struct PyMock(Mock);

#[pymethods]
impl PyMock {
    #[new]
    fn new() -> Self {
        PyMock(Mock { limits: MotionLimits::envelope::<Mock>() })
    }
}

py_robot!(PyMock(Mock));
py_arm!(PyMock<{2}>(Mock));
py_joint_motion!(PyMock<{2}>(Mock));
py_flange_motion!(PyMock(Mock));
py_joint_position_control!(PyMock<{2}>(Mock));
py_joint_velocity_control!(PyMock<{2}>(Mock));
py_cartesian_velocity_control!(PyMock<{2}>(Mock));
py_cartesian_pose_control!(PyMock<{2}>(Mock));
py_joint_torque_control!(PyMock<{2}>(Mock));
py_arm_torque_control!(PyMock<{2}>(Mock));
