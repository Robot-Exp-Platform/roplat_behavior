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
/// # Joint convention
///
/// Joint positions, and the velocities, efforts and limits that go with them,
/// follow the robot's kinematic description, its URDF or serial chain: each
/// joint gives its child link relative to its parent link, with the zero and
/// sign the description defines, not a controller's internal convention. A
/// driver whose controller couples joints differently converts in both
/// directions, readings and commands, and documents the conversion. FANUC,
/// for example, reports J3 relative to the horizontal, so J2 and J3 are
/// coupled. For a linear coupling `q = A q_c` from the controller's
/// coordinates `q_c`, velocities convert with `A` and efforts with the
/// inverse transpose of `A`, so that power is the same in both.
///
/// [`JOINT_LIMITS`](Joints::JOINT_LIMITS),
/// [`JOINT_DEFAULT`](Joints::JOINT_DEFAULT) and
/// [`JOINT_PACKED`](Joints::JOINT_PACKED) are in the same serial convention.
/// A controller limit on a coupled coordinate is not a box in the serial
/// convention, so the driver documents how it chose the box it declares.
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
