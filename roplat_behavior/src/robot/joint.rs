use crate::JointLimits;

/// A robot with a fixed number `N` of actuated joints, and the envelope of
/// those joints declared at compile time.
///
/// [`JOINT_LIMITS`](Joints::JOINT_LIMITS) is the model's envelope as data, the
/// form algorithms and safety checks take: positions in radians, velocities in
/// rad/s, accelerations in rad/s^2, jerk in rad/s^3, efforts in N*m. A driver
/// declares the position limits and the bounds its firmware enforces; every
/// other bound stays unbounded (`f64::INFINITY`).
///
/// `Joints<N>` is a super-trait of [`Arm<N>`](crate::Arm).
///
/// # Example
/// ```ignore
/// use roplat_behavior::{JointLimits, Joints, to_radians_array};
///
/// impl Joints<6> for MyArm {
///     const JOINT_LIMITS: JointLimits<6> = JointLimits {
///         dq_max: [3.14; 6],
///         ..JointLimits::positions(
///             to_radians_array([-170., -120., -170., -120., -170., -360.]),
///             to_radians_array([170., 120., 170., 120., 170., 360.]),
///         )
///     };
/// }
/// ```
pub trait Joints<const N: usize> {
    /// The arm's home / reference posture (rad), the seed of inverse
    /// kinematics. Defaults to all-zeros.
    const JOINT_DEFAULT: [f64; N] = [0.; N];
    /// A compact "packed" posture for transport or storage (rad). Declared for
    /// the caller: an application moves the arm here before shipping or
    /// storing it; the library itself never reads it. Defaults to all-zeros.
    const JOINT_PACKED: [f64; N] = [0.; N];
    /// The joint envelope of this model. Required.
    const JOINT_LIMITS: JointLimits<N>;
}
