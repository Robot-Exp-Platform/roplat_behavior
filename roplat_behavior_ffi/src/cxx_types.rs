#[allow(unused_imports)]
pub mod types {
    use roplat_behavior::Pose;

    #[cxx::bridge]
    mod to_cxx_bridge {
        extern "Rust" {
            type CxxPose;
            #[Self = "CxxPose"]
            fn Euler(tran: [f64; 3], rot: [f64; 3]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Quat(tran: [f64; 3], rot: [f64; 4]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Homo(pose: [f64; 16]) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn AxisAngle(tran: [f64; 3], axis: [f64; 3], angle: f64) -> Box<CxxPose>;
            #[Self = "CxxPose"]
            fn Position(tran: [f64; 3]) -> Box<CxxPose>;
            fn to_vec(&self) -> Vec<f64>;
            #[Self = "CxxPose"]
            fn from_vec(vec: Vec<f64>) -> Result<Box<CxxPose>>;
        }
    }

    pub use to_cxx_bridge::*;

    struct CxxPose(Pose);

    impl CxxPose {
        #[allow(non_snake_case)]
        fn Euler(tran: [f64; 3], rot: [f64; 3]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_euler(tran, rot)))
        }
        #[allow(non_snake_case)]
        fn Quat(tran: [f64; 3], rot: [f64; 4]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_quat_array(tran, rot)))
        }
        #[allow(non_snake_case)]
        fn Homo(pose: [f64; 16]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_homo(pose)))
        }
        #[allow(non_snake_case)]
        fn AxisAngle(tran: [f64; 3], axis: [f64; 3], angle: f64) -> Box<Self> {
            Box::new(CxxPose(Pose::from_axis_angle(tran, axis, angle)))
        }
        #[allow(non_snake_case)]
        fn Position(tran: [f64; 3]) -> Box<Self> {
            Box::new(CxxPose(Pose::from_position(tran)))
        }

        fn to_vec(&self) -> Vec<f64> {
            self.0.into()
        }
        /// Seven numbers, as `to_vec` writes them: translation, then the
        /// `x, y, z, w` quaternion. Any other length is an error.
        fn from_vec(vec: Vec<f64>) -> Result<Box<Self>, String> {
            let v: [f64; 7] = vec.as_slice().try_into().map_err(|_| {
                format!(
                    "a pose is seven numbers (translation and x, y, z, w quaternion), got {}",
                    vec.len()
                )
            })?;
            Ok(Box::new(CxxPose(Pose::from_quat_array(
                [v[0], v[1], v[2]],
                [v[3], v[4], v[5], v[6]],
            ))))
        }
    }
}
