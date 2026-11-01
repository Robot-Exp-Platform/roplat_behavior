//! Denavit–Hartenberg kinematics for `roplat_behavior` arms.
//!
//! An arm type declares its [`DhParam`] table through [`ArmForwardKinematics`]
//! and, with joint limits, gets the iterative solvers of
//! [`ArmInverseKinematics`]. [`dh_kinematics!`] connects these to the
//! `roplat_behavior` model maps (`SpaceMap` for flange pose, Jacobian and,
//! optionally, inverse kinematics), so generic code can use the type through
//! `ForwardKinematics`, `JacobianModel` and `InverseKinematics`.
//!
//! An arm read from URDF needs no DH table: [`ArmKineCache::from_chain`]
//! builds the same cache from a `roplat_description::Chain`, [`chain_ik`]
//! solves its inverse kinematics within the URDF limits
//! ([`chain_ik_solutions`] collects the distinct solutions that pass a
//! validity check), [`UrdfKinematics`] makes the chain a
//! `ForwardKinematics` and `JacobianModel` (as `dh_kinematics!` does for a
//! DH arm), and
//! [`UrdfDynamics`] makes a URDF tree a `DynamicsModel` for the
//! `roplat_control` torque controllers.
//!
//! The crate does not use `generic_const_exprs`: the next-generation trait
//! solver does not support it, and nightly 2026-10-20 crashes on downstream
//! crates that depend on it.

mod chain_ik;
mod dh;
mod kinematics;
mod urdf_dynamics;
mod urdf_kinematics;

pub use chain_ik::{ChainIkSolution, IkSearch, chain_ik, chain_ik_solutions, dh_chain_discrepancy};
pub use dh::DhParam;
pub use kinematics::*;
pub use urdf_dynamics::UrdfDynamics;
pub use urdf_kinematics::UrdfKinematics;

#[doc(hidden)]
pub mod __private {
    pub use roplat_behavior;
}

/// Concatenate two fixed-size arrays into a single `[T; L]`.
///
/// The output length `L` is usually inferred from the destination; a length
/// other than `N + M` is rejected at compile time.
///
/// # Example
/// ```
/// use roplat_kinematics::combine_array;
///
/// let joined: [i32; 5] = combine_array(&[1, 2, 3], &[4, 5]);
/// assert_eq!(joined, [1, 2, 3, 4, 5]);
/// ```
pub fn combine_array<T: Copy, const N: usize, const M: usize, const L: usize>(
    v1: &[T; N],
    v2: &[T; M],
) -> [T; L] {
    const { assert!(N + M == L, "combine_array: output length must equal N + M") };
    core::array::from_fn(|i| if i < N { v1[i] } else { v2[i - N] })
}

/// Implements the `roplat_behavior` model maps of a Denavit–Hartenberg arm.
///
/// `dh_kinematics!(Arm, N)` implements `SpaceMap<JointSpace<N>, FlangeSpace>`
/// (forward kinematics) and `SpaceMap<JointSpace<N>, JacobianSpace<N>>` for a
/// type implementing [`ArmForwardKinematics<N>`], both at the flange
/// ([`ArmForwardKinematics::FLANGE`]). Adding `inverse` also implements
/// `SpaceMap<FlangeSpace, JointSpace<N>>` through
/// [`ArmInverseKinematics::ik_solve_converged`], seeded with
/// `Joints::JOINT_DEFAULT`; a target it does not reach is an error.
///
/// Before 0.9 these maps were blanket impls in `robot_behavior`; a macro keeps
/// the contract crate independent of this one.
///
/// ```
/// use roplat_behavior::{ForwardKinematics, Joints};
/// use roplat_kinematics::{ArmForwardKinematics, DhParam, dh_kinematics};
///
/// struct Planar;
/// impl ArmForwardKinematics<2> for Planar {
///     const DH: [DhParam; 2] = [
///         DhParam::DH { theta: 0., d: 0., r: 1., alpha: 0. },
///         DhParam::DH { theta: 0., d: 0., r: 1., alpha: 0. },
///     ];
/// }
/// dh_kinematics!(Planar, 2);
///
/// let pose = Planar.forward_kinematics([0., 0.]).unwrap();
/// assert!((pose.position()[0] - 2.).abs() < 1e-12);
/// ```
#[macro_export]
macro_rules! dh_kinematics {
    ($ty:ty, $n:expr) => {
        impl
            $crate::__private::roplat_behavior::SpaceMap<
                $crate::__private::roplat_behavior::JointSpace<$n>,
                $crate::__private::roplat_behavior::FlangeSpace,
            > for $ty
        {
            fn map(
                &self,
                input: [f64; $n],
            ) -> $crate::__private::roplat_behavior::RobotResult<
                $crate::__private::roplat_behavior::Pose,
            > {
                Ok(<$ty as $crate::ArmForwardKinematics<$n>>::fk_end_pose(
                    &input,
                ))
            }
        }

        impl
            $crate::__private::roplat_behavior::SpaceMap<
                $crate::__private::roplat_behavior::JointSpace<$n>,
                $crate::__private::roplat_behavior::JacobianSpace<$n>,
            > for $ty
        {
            fn map(
                &self,
                input: [f64; $n],
            ) -> $crate::__private::roplat_behavior::RobotResult<
                $crate::__private::roplat_behavior::Jaco<$n>,
            > {
                Ok(<$ty as $crate::ArmForwardKinematics<$n>>::flange_jacobian(
                    &input,
                ))
            }
        }
    };
    ($ty:ty, $n:expr, inverse) => {
        $crate::dh_kinematics!($ty, $n);

        impl
            $crate::__private::roplat_behavior::SpaceMap<
                $crate::__private::roplat_behavior::FlangeSpace,
                $crate::__private::roplat_behavior::JointSpace<$n>,
            > for $ty
        {
            fn map(
                &self,
                input: $crate::__private::roplat_behavior::Pose,
            ) -> $crate::__private::roplat_behavior::RobotResult<[f64; $n]> {
                let q0 = $crate::__private::roplat_behavior::JVec::<$n>::from_row_slice(
                    &<$ty as $crate::__private::roplat_behavior::Joints<$n>>::JOINT_DEFAULT,
                );
                let q = <$ty as $crate::ArmInverseKinematics<$n>>::ik_solve_converged(
                    &q0,
                    &input,
                    Default::default(),
                )?;
                Ok(q.as_slice().try_into().unwrap())
            }
        }
    };
}
