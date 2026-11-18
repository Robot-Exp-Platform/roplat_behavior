use std::time::Duration;

use pyo3::prelude::*;

use roplat_behavior::{
    ArmState, JointSample, JointState, LoadState, RobotException, SpatialSample, StateView,
};

use crate::PyPose;

/// Payload for Python: mass `m` (kg), center of mass `x` (m) and inertia `i`
/// (kg·m², column-major 3x3), the same fields as before 0.9.
#[derive(Debug, Clone)]
#[pyclass(name = "LoadState", from_py_object)]
pub struct PyLoadState {
    #[pyo3(get, set)]
    pub m: f64,
    #[pyo3(get, set)]
    pub x: [f64; 3],
    #[pyo3(get, set)]
    pub i: [f64; 9],
}

#[pymethods]
impl PyLoadState {
    #[new]
    pub fn new(m: f64, x: [f64; 3], i: [f64; 9]) -> Self {
        Self { m, x, i }
    }

    pub fn __repr__(&self) -> String {
        format!("LoadState(m={}, x={:?}, i={:?})", self.m, self.x, self.i)
    }
}

impl From<LoadState> for PyLoadState {
    fn from(value: LoadState) -> Self {
        Self { m: value.mass, x: value.center_of_mass, i: value.inertia }
    }
}

impl From<PyLoadState> for LoadState {
    fn from(value: PyLoadState) -> Self {
        Self { mass: value.m, center_of_mass: value.x, inertia: value.i }
    }
}

#[derive(Debug, Clone)]
#[pyclass(name = "JointSample", from_py_object)]
pub struct PyJointSample {
    #[pyo3(get, set)]
    pub q: Option<Vec<f64>>,
    #[pyo3(get, set)]
    pub dq: Option<Vec<f64>>,
    #[pyo3(get, set)]
    pub ddq: Option<Vec<f64>>,
    #[pyo3(get, set)]
    pub tau: Option<Vec<f64>>,
    #[pyo3(get, set)]
    pub dtau: Option<Vec<f64>>,
}

#[pymethods]
impl PyJointSample {
    #[new]
    fn new(
        q: Option<Vec<f64>>,
        dq: Option<Vec<f64>>,
        ddq: Option<Vec<f64>>,
        tau: Option<Vec<f64>>,
        dtau: Option<Vec<f64>>,
    ) -> Self {
        Self { q, dq, ddq, tau, dtau }
    }
}

impl<const N: usize> From<JointSample<N>> for PyJointSample {
    fn from(value: JointSample<N>) -> Self {
        Self {
            q: value.q().map(Vec::from),
            dq: value.dq().map(Vec::from),
            ddq: value.ddq().map(Vec::from),
            tau: value.tau().map(Vec::from),
            dtau: value.dtau().map(Vec::from),
        }
    }
}

#[derive(Debug, Clone)]
#[pyclass(name = "SpatialSample", from_py_object)]
pub struct PySpatialSample {
    #[pyo3(get, set)]
    pub pose: Option<PyPose>,
    #[pyo3(get, set)]
    pub vel: Option<[f64; 6]>,
    #[pyo3(get, set)]
    pub acc: Option<[f64; 6]>,
    #[pyo3(get, set)]
    pub wrench: Option<[f64; 6]>,
}

#[pymethods]
impl PySpatialSample {
    #[new]
    fn new(
        pose: Option<PyPose>,
        vel: Option<[f64; 6]>,
        acc: Option<[f64; 6]>,
        wrench: Option<[f64; 6]>,
    ) -> Self {
        Self { pose, vel, acc, wrench }
    }
}

impl From<SpatialSample> for PySpatialSample {
    fn from(value: SpatialSample) -> Self {
        Self {
            pose: value.pose().map(PyPose::from),
            vel: value.twist().map(<[f64; 6]>::from),
            acc: value.accel().map(<[f64; 6]>::from),
            wrench: value.wrench().map(<[f64; 6]>::from),
        }
    }
}

#[derive(Debug, Clone)]
#[pyclass(name = "JointState", from_py_object)]
pub struct PyJointState {
    #[pyo3(get, set)]
    pub meas: PyJointSample,
    #[pyo3(get, set)]
    pub cmd: PyJointSample,
    #[pyo3(get, set)]
    pub des: PyJointSample,
}

#[pymethods]
impl PyJointState {
    #[new]
    fn new(meas: PyJointSample, cmd: PyJointSample, des: PyJointSample) -> Self {
        Self { meas, cmd, des }
    }
}

impl<const N: usize> From<JointState<N>> for PyJointState {
    fn from(value: JointState<N>) -> Self {
        Self {
            meas: value.meas.into(),
            cmd: value.cmd.into(),
            des: value.des.into(),
        }
    }
}

#[derive(Debug, Clone)]
#[pyclass(name = "SpatialState", from_py_object)]
pub struct PySpatialState {
    #[pyo3(get, set)]
    pub meas: PySpatialSample,
    #[pyo3(get, set)]
    pub cmd: PySpatialSample,
    #[pyo3(get, set)]
    pub des: PySpatialSample,
}

#[pymethods]
impl PySpatialState {
    #[new]
    fn new(meas: PySpatialSample, cmd: PySpatialSample, des: PySpatialSample) -> Self {
        Self { meas, cmd, des }
    }
}

impl From<StateView<SpatialSample>> for PySpatialState {
    fn from(value: StateView<SpatialSample>) -> Self {
        Self {
            meas: value.meas.into(),
            cmd: value.cmd.into(),
            des: value.des.into(),
        }
    }
}

#[derive(Debug, Clone)]
#[pyclass(name = "ArmState", from_py_object)]
pub struct PyArmState {
    #[pyo3(get, set)]
    pub joint: PyJointState,
    #[pyo3(get, set)]
    pub flange: PySpatialState,
    #[pyo3(get, set)]
    pub tcp: Option<PySpatialState>,
    #[pyo3(get, set)]
    pub stiffness: Option<PySpatialState>,
    #[pyo3(get, set)]
    pub load: Option<PyLoadState>,
}

#[pymethods]
impl PyArmState {
    #[new]
    fn new(
        joint: PyJointState,
        flange: PySpatialState,
        tcp: Option<PySpatialState>,
        stiffness: Option<PySpatialState>,
        load: Option<PyLoadState>,
    ) -> Self {
        Self { joint, flange, tcp, stiffness, load }
    }
}

impl<const N: usize> From<ArmState<N>> for PyArmState {
    fn from(value: ArmState<N>) -> Self {
        Self {
            joint: value.joint.into(),
            flange: value.flange.into(),
            tcp: value.tcp.map(PySpatialState::from),
            stiffness: value.stiffness.map(PySpatialState::from),
            load: value.load.map(PyLoadState::from),
        }
    }
}

pub fn vec_to_array<const N: usize>(values: Vec<f64>) -> PyResult<[f64; N]> {
    values
        .try_into()
        .map_err(|values: Vec<f64>| {
            RobotException::InvalidFFIData(format!(
                "expected vector length {}, got {}",
                N,
                values.len()
            ))
        })
        .map_err(crate::py_err)
}

pub fn py_control_array<const N: usize>(
    callable: &Py<PyAny>,
    state: PyJointState,
    duration: Duration,
) -> ([f64; N], bool) {
    Python::attach(|py| {
        let result = callable
            .call1(py, (state, duration.as_secs_f64()))
            .and_then(|obj| obj.extract::<(Vec<f64>, bool)>(py))
            .and_then(|(cmd, done)| vec_to_array::<N>(cmd).map(|cmd| (cmd, done)));

        match result {
            Ok(result) => result,
            Err(err) => {
                err.print(py);
                ([0.0; N], true)
            }
        }
    })
}

pub fn py_arm_control_array<const N: usize>(
    callable: &Py<PyAny>,
    state: PyArmState,
    duration: Duration,
) -> ([f64; N], bool) {
    Python::attach(|py| {
        let result = callable
            .call1(py, (state, duration.as_secs_f64()))
            .and_then(|obj| obj.extract::<(Vec<f64>, bool)>(py))
            .and_then(|(cmd, done)| vec_to_array::<N>(cmd).map(|cmd| (cmd, done)));

        match result {
            Ok(result) => result,
            Err(err) => {
                err.print(py);
                ([0.0; N], true)
            }
        }
    })
}

pub fn py_arm_control_vec6(
    callable: &Py<PyAny>,
    state: PyArmState,
    duration: Duration,
) -> ([f64; 6], bool) {
    Python::attach(|py| {
        let result = callable
            .call1(py, (state, duration.as_secs_f64()))
            .and_then(|obj| obj.extract::<(Vec<f64>, bool)>(py))
            .and_then(|(cmd, done)| vec_to_array::<6>(cmd).map(|cmd| (cmd, done)));

        match result {
            Ok(result) => result,
            Err(err) => {
                err.print(py);
                ([0.0; 6], true)
            }
        }
    })
}

pub fn py_arm_control_pose(
    callable: &Py<PyAny>,
    state: PyArmState,
    duration: Duration,
) -> (roplat_behavior::Pose, bool) {
    Python::attach(|py| {
        let result = callable
            .call1(py, (state, duration.as_secs_f64()))
            .and_then(|obj| obj.extract::<(PyPose, bool)>(py))
            .map(|(pose, done)| (pose.into(), done));

        match result {
            Ok(result) => result,
            Err(err) => {
                err.print(py);
                (roplat_behavior::Pose::default(), true)
            }
        }
    })
}

#[macro_export]
macro_rules! py_robot_wrapper {
    ($py:ident($inner:ty)) => {
        #[pyo3::pyclass]
        pub struct $py(pub $inner);
    };
}

#[macro_export]
macro_rules! py_robot {
    ($py:ident($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            #[staticmethod]
            fn version() -> String {
                <$inner as $crate::__private::roplat_behavior::Robot>::version()
            }

            fn init(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::init(&mut self.0)
                    .map_err($crate::py_err)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn enable(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::enable(&mut self.0)
                    .map_err($crate::py_err)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn disable(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::disable(&mut self.0)
                    .map_err($crate::py_err)
            }

            fn shutdown(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::shutdown(&mut self.0)
                    .map_err($crate::py_err)
            }

            fn reset(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::reset(&mut self.0)
                    .map_err($crate::py_err)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn stop(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::stop(&mut self.0)
                    .map_err($crate::py_err)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn pause(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::pause(&mut self.0)
                    .map_err($crate::py_err)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn resume(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::resume(&mut self.0)
                    .map_err($crate::py_err)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn emergency_stop(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::emergency_stop(&mut self.0)
                    .map_err($crate::py_err)
            }

            fn waiting_for_finish(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::waiting_for_finish(
                    &mut self.0,
                )
                .map_err($crate::py_err)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn clear_emergency_stop(&mut self) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::clear_emergency_stop(
                    &mut self.0,
                )
                .map_err($crate::py_err)
            }

            fn is_moving(&mut self) -> pyo3::PyResult<bool> {
                <$inner as $crate::__private::roplat_behavior::Robot>::is_moving(&mut self.0)
                    .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn state(&mut self) -> pyo3::PyResult<$crate::PyArmState> {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::state(&mut self.0)
                    .map($crate::PyArmState::from)
                    .map_err($crate::py_err)
            }

            fn set_load(&mut self, load: $crate::PyLoadState) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::set_load(
                    &mut self.0,
                    load.into(),
                )
                .map_err($crate::py_err)
            }

            fn get_joint(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::get_joint(&self.0)
            }

            fn get_endpoint(&self) -> $crate::PyPose {
                <$inner as $crate::__private::roplat_behavior::EndPointPose>::get_endpoint(&self.0)
                    .into()
            }

            fn get_joint_min(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::motion_limits(&self.0)
                    .joint
                    .q_min
            }

            fn get_joint_max(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::motion_limits(&self.0)
                    .joint
                    .q_max
            }

            fn get_joint_vel_bound(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::motion_limits(&self.0)
                    .joint
                    .dq_max
            }

            fn get_joint_acc_bound(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::motion_limits(&self.0)
                    .joint
                    .ddq_max
            }

            fn get_joint_jerk_bound(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::motion_limits(&self.0)
                    .joint
                    .dddq_max
            }

            fn get_torque_bound(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::motion_limits(&self.0)
                    .joint
                    .tau_max
            }

            fn get_torque_dot_bound(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::motion_limits(&self.0)
                    .joint
                    .dtau_max
            }
        }
    };
}

#[macro_export]
macro_rules! py_joint_motion {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn move_joint(&mut self, target: [f64; $dof]) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::JointSpace<$dof>,
                >>::move_to(&mut self.0, target)
                .map_err($crate::py_err)
            }

            fn move_joint_sync(&mut self, target: [f64; $dof]) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::JointSpace<$dof>,
                >>::move_to(&mut self.0, target)
                .map_err($crate::py_err)
            }

            fn move_joint_traj(&mut self, traj: Vec<[f64; $dof]>) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::MoveTraj<
                    $crate::__private::roplat_behavior::JointSpace<$dof>,
                >>::move_traj(&mut self.0, traj)
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_flange_move {
    ($py:ident($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn move_flange(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::FlangeSpace,
                >>::move_to(&mut self.0, target.into())
                .map_err($crate::py_err)
            }

            fn move_flange_sync(&mut self, target: $crate::PyPose) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::FlangeSpace,
                >>::move_to(&mut self.0, target.into())
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_flange_traj {
    ($py:ident($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn move_flange_traj(&mut self, traj: Vec<$crate::PyPose>) -> pyo3::PyResult<()> {
                let traj = traj.into_iter().map(Into::into).collect();
                <$inner as $crate::__private::roplat_behavior::MoveTraj<
                    $crate::__private::roplat_behavior::FlangeSpace,
                >>::move_traj(&mut self.0, traj)
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_flange_motion {
    ($py:ident($inner:ty)) => {
        $crate::py_flange_move!($py($inner));
        $crate::py_flange_traj!($py($inner));
    };
}

#[macro_export]
macro_rules! py_joint_torque_control {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn control_joint_torque(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::ControlWith<
                    $crate::__private::roplat_behavior::TorqueControl<$dof>,
                >>::control_with(&mut self.0, move |state, duration| {
                    $crate::py::py_control_array::<$dof>(&closure, state.into(), duration)
                })
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_arm_torque_control {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn control_arm_torque(&mut self, closure: pyo3::Py<pyo3::PyAny>) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::ControlWith<
                    $crate::__private::roplat_behavior::ArmTorqueControl<$dof>,
                >>::control_with(&mut self.0, move |state, duration| {
                    $crate::py::py_arm_control_array::<$dof>(&closure, state.into(), duration)
                })
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_joint_position_control {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn control_joint_position(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::ControlWith<
                    $crate::__private::roplat_behavior::JointPositionControl<$dof>,
                >>::control_with(&mut self.0, move |state, duration| {
                    $crate::py::py_control_array::<$dof>(&closure, state.into(), duration)
                })
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_joint_velocity_control {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn control_joint_velocity(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::ControlWith<
                    $crate::__private::roplat_behavior::JointVelocityControl<$dof>,
                >>::control_with(&mut self.0, move |state, duration| {
                    $crate::py::py_control_array::<$dof>(&closure, state.into(), duration)
                })
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_cartesian_velocity_control {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn control_cartesian_velocity(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::ControlWith<
                    $crate::__private::roplat_behavior::CartesianVelocityControl<$dof>,
                >>::control_with(&mut self.0, move |state, duration| {
                    $crate::py::py_arm_control_vec6(&closure, state.into(), duration)
                })
                .map_err($crate::py_err)
            }
        }
    };
}

#[macro_export]
macro_rules! py_cartesian_pose_control {
    ($py:ident < {$dof:expr} > ($inner:ty)) => {
        #[pyo3::pymethods]
        impl $py {
            fn control_cartesian_pose(
                &mut self,
                closure: pyo3::Py<pyo3::PyAny>,
            ) -> pyo3::PyResult<()> {
                <$inner as $crate::__private::roplat_behavior::ControlWith<
                    $crate::__private::roplat_behavior::CartesianPoseControl<$dof>,
                >>::control_with(&mut self.0, move |state, duration| {
                    $crate::py::py_arm_control_pose(&closure, state.into(), duration)
                })
                .map_err($crate::py_err)
            }
        }
    };
}
