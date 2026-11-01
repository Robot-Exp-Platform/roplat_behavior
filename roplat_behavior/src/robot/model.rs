use roplat_msgs::Twist;

use crate::{
    RobotResult,
    robot::{
        math::{JMat, JVec, Jaco},
        motion::Space,
        spaces::{
            CoriolisInputSpace, FlangeSpace, GravityInputSpace, JacobianSpace, JointSpace,
            JointTorqueSpace, MassMatrixSpace,
        },
        types::Pose,
    },
};

/// A mapping from one space to another: the model's description of how its
/// quantities relate. A model is a composition of maps, and each
/// `impl SpaceMap<A, B>` states that the type turns an `A` value into a `B`
/// value. The spaces fix the value types; an implementor cannot choose others.
pub trait SpaceMap<From: Space, To: Space> {
    fn map(&self, input: From::Value) -> RobotResult<To::Value>;
}

/// Joint position and velocity input for Coriolis / centrifugal terms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoriolisInput<const N: usize> {
    pub q: [f64; N],
    pub dq: [f64; N],
}

/// Joint position plus gravity vector input for gravity terms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravityInput<const N: usize> {
    pub q: [f64; N],
    pub gravity: [f64; 3],
}

/// Forward kinematics as a typed map from joint space to flange / end-effector
/// pose space.
pub trait ForwardKinematics<const N: usize>: SpaceMap<JointSpace<N>, FlangeSpace> {
    fn forward_kinematics(&self, q: [f64; N]) -> RobotResult<Pose> {
        self.map(q)
    }
}

impl<T, const N: usize> ForwardKinematics<N> for T where T: SpaceMap<JointSpace<N>, FlangeSpace> {}

/// Inverse kinematics as a typed map from flange / end-effector pose space back
/// to joint space.
///
/// `roplat_kinematics::dh_kinematics!(Type, N, inverse)` implements this map
/// for a Denavit–Hartenberg arm.
pub trait InverseKinematics<const N: usize>: SpaceMap<FlangeSpace, JointSpace<N>> {
    fn inverse_kinematics(&self, target: Pose) -> RobotResult<[f64; N]> {
        self.map(target)
    }
}

impl<T, const N: usize> InverseKinematics<N> for T where T: SpaceMap<FlangeSpace, JointSpace<N>> {}

/// Geometric Jacobian model as a typed map from joint space to Jacobian space.
pub trait JacobianModel<const N: usize>: SpaceMap<JointSpace<N>, JacobianSpace<N>> {
    fn jacobian(&self, q: [f64; N]) -> RobotResult<Jaco<N>> {
        self.map(q)
    }

    fn twist(&self, q: [f64; N], dq: [f64; N]) -> RobotResult<Twist> {
        Ok((self.jacobian(q)? * JVec::<N>::from_column_slice(&dq)).into())
    }
}

impl<T, const N: usize> JacobianModel<N> for T where T: SpaceMap<JointSpace<N>, JacobianSpace<N>> {}

/// Dynamics model as typed maps for mass, Coriolis / centrifugal and gravity
/// terms.
pub trait DynamicsModel<const N: usize>:
    SpaceMap<JointSpace<N>, MassMatrixSpace<N>>
    + SpaceMap<CoriolisInputSpace<N>, JointTorqueSpace<N>>
    + SpaceMap<GravityInputSpace<N>, JointTorqueSpace<N>>
{
    fn mass(&self, q: [f64; N]) -> RobotResult<JMat<N>> {
        <Self as SpaceMap<JointSpace<N>, MassMatrixSpace<N>>>::map(self, q)
    }

    fn coriolis(&self, q: [f64; N], dq: [f64; N]) -> RobotResult<[f64; N]> {
        <Self as SpaceMap<CoriolisInputSpace<N>, JointTorqueSpace<N>>>::map(
            self,
            CoriolisInput { q, dq },
        )
    }

    fn gravity(&self, q: [f64; N], gravity: [f64; 3]) -> RobotResult<[f64; N]> {
        <Self as SpaceMap<GravityInputSpace<N>, JointTorqueSpace<N>>>::map(
            self,
            GravityInput { q, gravity },
        )
    }
}

impl<T, const N: usize> DynamicsModel<N> for T where
    T: SpaceMap<JointSpace<N>, MassMatrixSpace<N>>
        + SpaceMap<CoriolisInputSpace<N>, JointTorqueSpace<N>>
        + SpaceMap<GravityInputSpace<N>, JointTorqueSpace<N>>
{
}
