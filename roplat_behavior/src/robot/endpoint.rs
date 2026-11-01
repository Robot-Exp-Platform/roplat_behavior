use crate::{CartesianLimits, Pose, RobotResult};

/// The Cartesian limits an arm's end effector declares.
///
/// Where [`Joints`](crate::Joints) describes the joint envelope, `EndPoint`
/// describes the task-space envelope of the flange / TCP: how fast it may
/// translate and rotate, as [`CartesianLimits`] (m/s, m/s^2, m/s^3 and rad/s,
/// rad/s^2, rad/s^3). Every bound defaults to unbounded, so a driver overrides
/// only the limits its firmware enforces.
///
/// Like `Joints`, it is a declaration, so a model description implements it
/// too. Where the flange is now is [`EndPointPose`], as [`Arm`](crate::Arm)
/// is to `Joints`. `EndPoint` is a super-trait of [`Arm<N>`](crate::Arm).
///
/// # Example
/// ```ignore
/// use roplat_behavior::{CartesianLimits, EndPoint};
///
/// impl EndPoint for MyArm {
///     const CARTESIAN_LIMITS: CartesianLimits = CartesianLimits {
///         linear_velocity_max: 2.0,                   // m/s
///         angular_velocity_max: std::f64::consts::PI, // rad/s
///         ..CartesianLimits::UNBOUNDED
///     };
/// }
/// ```
pub trait EndPoint {
    /// The Cartesian envelope of this model. Defaults to unbounded.
    const CARTESIAN_LIMITS: CartesianLimits = CartesianLimits::UNBOUNDED;
}

/// Where a robot's flange is: the current pose behind [`EndPoint`]'s
/// declared limits.
///
/// Every [`Arm<N>`](crate::Arm) implements it. Its pose makes
/// [`FlangeSpace`](crate::FlangeSpace) a [`RelativeSpace`](crate::RelativeSpace),
/// so a [`Relative<FlangeSpace>`](crate::Relative) motion starts from
/// [`read_endpoint`](Self::read_endpoint).
pub trait EndPointPose {
    /// Current flange pose in the base frame, as last known to the driver.
    fn get_endpoint(&self) -> Pose;

    /// Read the flange pose from the robot, for a
    /// [`Relative<FlangeSpace>`](crate::Relative) motion. Defaults to
    /// [`get_endpoint`](Self::get_endpoint); a driver whose `get_endpoint`
    /// cannot reach the robot overrides it.
    fn read_endpoint(&mut self) -> RobotResult<Pose> {
        Ok(self.get_endpoint())
    }
}
