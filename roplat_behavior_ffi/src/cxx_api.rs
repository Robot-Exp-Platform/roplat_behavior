use roplat_behavior::{Pose, RobotException};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CxxPoseKind {
    Euler,
    Quat,
    Homo,
    AxisAngle,
    Position,
}

#[derive(Debug, Clone)]
pub struct CxxPoseData {
    pub kind: CxxPoseKind,
    pub values: Vec<f64>,
}

/// A pose has one representation since 0.9; C++ receives the `Quat` form
/// `[x, y, z, qx, qy, qz, qw]`.
impl From<Pose> for CxxPoseData {
    fn from(value: Pose) -> Self {
        let (tran, rot) = value.quat_array();
        let mut values = Vec::with_capacity(7);
        values.extend_from_slice(&tran);
        values.extend_from_slice(&rot);
        Self { kind: CxxPoseKind::Quat, values }
    }
}

impl TryFrom<CxxPoseData> for Pose {
    type Error = RobotException;

    fn try_from(value: CxxPoseData) -> Result<Self, Self::Error> {
        let v = value.values.as_slice();
        match value.kind {
            CxxPoseKind::Euler if v.len() == 6 => {
                Ok(Pose::from_euler([v[0], v[1], v[2]], [v[3], v[4], v[5]]))
            }
            CxxPoseKind::Quat if v.len() == 7 => Ok(Pose::from_quat_array(
                [v[0], v[1], v[2]],
                [v[3], v[4], v[5], v[6]],
            )),
            CxxPoseKind::Homo if v.len() == 16 => Ok(Pose::from_homo(v.try_into().unwrap())),
            CxxPoseKind::AxisAngle if v.len() == 7 => Ok(Pose::from_axis_angle(
                [v[0], v[1], v[2]],
                [v[3], v[4], v[5]],
                v[6],
            )),
            CxxPoseKind::Position if v.len() == 3 => Ok(Pose::from_position([v[0], v[1], v[2]])),
            _ => Err(RobotException::InvalidFFIData(
                "invalid CxxPoseData length for pose kind".into(),
            )),
        }
    }
}

#[macro_export]
macro_rules! cxx_robot_api {
    ($ty:ty) => {
        fn version() -> String {
            <$ty as $crate::__private::roplat_behavior::Robot>::version()
        }

        fn init(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::init(self)
        }

        #[allow(
            deprecated,
            reason = "binds the deprecated `Robot` method for one more version"
        )]
        fn enable(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::enable(self)
        }

        #[allow(
            deprecated,
            reason = "binds the deprecated `Robot` method for one more version"
        )]
        fn disable(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::disable(self)
        }

        fn shutdown(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::shutdown(self)
        }

        fn reset(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::reset(self)
        }

        #[allow(
            deprecated,
            reason = "binds the deprecated `Robot` method for one more version"
        )]
        fn stop(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::stop(self)
        }

        #[allow(
            deprecated,
            reason = "binds the deprecated `Robot` method for one more version"
        )]
        fn emergency_stop(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::emergency_stop(self)
        }

        fn waiting_for_finish(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::Robot>::waiting_for_finish(self)
        }

        fn is_moving(&mut self) -> $crate::__private::roplat_behavior::RobotResult<bool> {
            <$ty as $crate::__private::roplat_behavior::Robot>::is_moving(self)
        }
    };
}

#[macro_export]
macro_rules! cxx_arm_motion_api {
    ($ty:ty, $dof:expr) => {
        fn move_joint(
            &mut self,
            target: [f64; $dof],
        ) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::MoveTo<
                $crate::__private::roplat_behavior::JointSpace<$dof>,
            >>::move_to(self, target)
        }

        fn move_joint_sync(
            &mut self,
            target: [f64; $dof],
        ) -> $crate::__private::roplat_behavior::RobotResult<()> {
            <$ty as $crate::__private::roplat_behavior::MoveTo<
                $crate::__private::roplat_behavior::JointSpace<$dof>,
            >>::move_to(self, target)
        }

        fn move_flange(
            &mut self,
            target: $crate::cxx_api::CxxPoseData,
        ) -> $crate::__private::roplat_behavior::RobotResult<()> {
            let target = $crate::__private::roplat_behavior::Pose::try_from(target)?;
            <$ty as $crate::__private::roplat_behavior::MoveTo<
                $crate::__private::roplat_behavior::FlangeSpace,
            >>::move_to(self, target)
        }

        fn move_flange_sync(
            &mut self,
            target: $crate::cxx_api::CxxPoseData,
        ) -> $crate::__private::roplat_behavior::RobotResult<()> {
            let target = $crate::__private::roplat_behavior::Pose::try_from(target)?;
            <$ty as $crate::__private::roplat_behavior::MoveTo<
                $crate::__private::roplat_behavior::FlangeSpace,
            >>::move_to(self, target)
        }
    };
}

#[macro_export]
macro_rules! cxx_arm_bridge {
    ($mod_name:ident, $wrapper:ident($inner:ty), dof = $dof:expr) => {
        #[cxx::bridge]
        pub mod $mod_name {
            enum CxxPoseKind {
                Euler,
                Quat,
                Homo,
                AxisAngle,
                Position,
            }

            struct CxxPoseData {
                kind: CxxPoseKind,
                values: Vec<f64>,
            }

            extern "Rust" {
                type $wrapper;

                fn version() -> String;
                fn init(&mut self) -> Result<()>;
                fn enable(&mut self) -> Result<()>;
                fn disable(&mut self) -> Result<()>;
                fn shutdown(&mut self) -> Result<()>;
                fn reset(&mut self) -> Result<()>;
                fn stop(&mut self) -> Result<()>;
                fn emergency_stop(&mut self) -> Result<()>;
                fn waiting_for_finish(&mut self) -> Result<()>;
                fn is_moving(&mut self) -> Result<bool>;

                fn state(&mut self) -> Result<String>;
                fn set_load(&mut self, m: f64, x: [f64; 3], i: [f64; 9]) -> Result<()>;
                fn get_joint(&self) -> [f64; $dof];
                fn get_endpoint(&self) -> CxxPoseData;

                fn move_joint(&mut self, target: [f64; $dof]) -> Result<()>;
                fn move_joint_sync(&mut self, target: [f64; $dof]) -> Result<()>;
                fn move_flange(&mut self, target: CxxPoseData) -> Result<()>;
                fn move_flange_sync(&mut self, target: CxxPoseData) -> Result<()>;
            }
        }

        impl $wrapper {
            fn version() -> String {
                <$inner as $crate::__private::roplat_behavior::Robot>::version()
            }

            fn init(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::init(&mut self.0)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn enable(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::enable(&mut self.0)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn disable(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::disable(&mut self.0)
            }

            fn shutdown(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::shutdown(&mut self.0)
            }

            fn reset(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::reset(&mut self.0)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn stop(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::stop(&mut self.0)
            }

            #[allow(
                deprecated,
                reason = "binds the deprecated `Robot` method for one more version"
            )]
            fn emergency_stop(&mut self) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::emergency_stop(&mut self.0)
            }

            fn waiting_for_finish(
                &mut self,
            ) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Robot>::waiting_for_finish(
                    &mut self.0,
                )
            }

            fn is_moving(&mut self) -> $crate::__private::roplat_behavior::RobotResult<bool> {
                <$inner as $crate::__private::roplat_behavior::Robot>::is_moving(&mut self.0)
            }

            fn state(&mut self) -> $crate::__private::roplat_behavior::RobotResult<String> {
                let state =
                    <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::state(&mut self.0)?;
                Ok(format!("{state:?}"))
            }

            fn set_load(
                &mut self,
                m: f64,
                x: [f64; 3],
                i: [f64; 9],
            ) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::set_load(
                    &mut self.0,
                    $crate::__private::roplat_behavior::LoadState {
                        mass: m,
                        center_of_mass: x,
                        inertia: i,
                    },
                )
            }

            fn get_joint(&self) -> [f64; $dof] {
                <$inner as $crate::__private::roplat_behavior::Arm<$dof>>::get_joint(&self.0)
            }

            fn get_endpoint(&self) -> $mod_name::CxxPoseData {
                let (tran, rot) =
                    <$inner as $crate::__private::roplat_behavior::EndPointPose>::get_endpoint(
                        &self.0,
                    )
                    .quat_array();
                let mut values = Vec::with_capacity(7);
                values.extend_from_slice(&tran);
                values.extend_from_slice(&rot);
                $mod_name::CxxPoseData { kind: $mod_name::CxxPoseKind::Quat, values }
            }

            fn move_joint(
                &mut self,
                target: [f64; $dof],
            ) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::JointSpace<$dof>,
                >>::move_to(&mut self.0, target)
            }

            fn move_joint_sync(
                &mut self,
                target: [f64; $dof],
            ) -> $crate::__private::roplat_behavior::RobotResult<()> {
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::JointSpace<$dof>,
                >>::move_to(&mut self.0, target)
            }

            fn move_flange(
                &mut self,
                target: $mod_name::CxxPoseData,
            ) -> $crate::__private::roplat_behavior::RobotResult<()> {
                let target = Self::cxx_pose_to_pose(target)?;
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::FlangeSpace,
                >>::move_to(&mut self.0, target)
            }

            fn move_flange_sync(
                &mut self,
                target: $mod_name::CxxPoseData,
            ) -> $crate::__private::roplat_behavior::RobotResult<()> {
                let target = Self::cxx_pose_to_pose(target)?;
                <$inner as $crate::__private::roplat_behavior::MoveTo<
                    $crate::__private::roplat_behavior::FlangeSpace,
                >>::move_to(&mut self.0, target)
            }

            fn cxx_pose_to_pose(
                target: $mod_name::CxxPoseData,
            ) -> $crate::__private::roplat_behavior::RobotResult<
                $crate::__private::roplat_behavior::Pose,
            > {
                let v = target.values.as_slice();
                match target.kind {
                    $mod_name::CxxPoseKind::Euler if v.len() == 6 => {
                        Ok($crate::__private::roplat_behavior::Pose::from_euler(
                            [v[0], v[1], v[2]],
                            [v[3], v[4], v[5]],
                        ))
                    }
                    $mod_name::CxxPoseKind::Quat if v.len() == 7 => {
                        Ok($crate::__private::roplat_behavior::Pose::from_quat_array(
                            [v[0], v[1], v[2]],
                            [v[3], v[4], v[5], v[6]],
                        ))
                    }
                    $mod_name::CxxPoseKind::Homo if v.len() == 16 => Ok(
                        $crate::__private::roplat_behavior::Pose::from_homo(v.try_into().unwrap()),
                    ),
                    $mod_name::CxxPoseKind::AxisAngle if v.len() == 7 => {
                        Ok($crate::__private::roplat_behavior::Pose::from_axis_angle(
                            [v[0], v[1], v[2]],
                            [v[3], v[4], v[5]],
                            v[6],
                        ))
                    }
                    $mod_name::CxxPoseKind::Position if v.len() == 3 => {
                        Ok($crate::__private::roplat_behavior::Pose::from_position([
                            v[0], v[1], v[2],
                        ]))
                    }
                    _ => Err(
                        $crate::__private::roplat_behavior::RobotException::InvalidFFIData(
                            "invalid CxxPoseData length for pose kind".into(),
                        ),
                    ),
                }
            }
        }
    };
}
