//! A URDF chain as `roplat_behavior` kinematic model maps.

use nalgebra as na;
use roplat_behavior::{
    FlangeSpace, Jaco, JacobianSpace, JointSpace, Pose, RobotException, RobotResult, SpaceMap,
    msgs::JointLimits,
};
use roplat_description::{Chain, DescriptionError};

/// The kinematics of a URDF chain with `N` movable joints, as
/// [`ForwardKinematics<N>`](roplat_behavior::ForwardKinematics) and
/// [`JacobianModel<N>`](roplat_behavior::JacobianModel): what
/// [`dh_kinematics!`](crate::dh_kinematics) gives a DH arm, for an arm read
/// from URDF. Any mix of revolute, continuous and prismatic joints works.
///
/// Coordinates are the chain's movable joints from base to tip. The pose is
/// the tip link in the base frame; the Jacobian is the tip's, in the base
/// frame, linear rows first. Each evaluation allocates; inverse kinematics
/// needs a seed, so it is left to [`chain_ik`](crate::chain_ik).
#[derive(Clone, Debug)]
pub struct UrdfKinematics<const N: usize> {
    chain: Chain,
}

impl<const N: usize> UrdfKinematics<N> {
    pub fn new(chain: Chain) -> Result<Self, DescriptionError> {
        let dof = chain.dof();
        if dof != N {
            return Err(DescriptionError::DofMismatch { expected: dof, got: N });
        }
        Ok(Self { chain })
    }

    pub fn chain(&self) -> &Chain {
        &self.chain
    }

    /// The URDF limits of the movable joints; unbounded where the file
    /// gives none.
    pub fn joint_limits(&self) -> JointLimits<N> {
        self.chain.joint_limits().expect("dof checked in new")
    }
}

fn model_error(error: DescriptionError) -> RobotException {
    RobotException::UnprocessableInstructionError(error.to_string())
}

impl<const N: usize> SpaceMap<JointSpace<N>, FlangeSpace> for UrdfKinematics<N> {
    fn map(&self, q: [f64; N]) -> RobotResult<Pose> {
        self.chain.forward_kinematics(&q).map_err(model_error)
    }
}

impl<const N: usize> SpaceMap<JointSpace<N>, JacobianSpace<N>> for UrdfKinematics<N> {
    fn map(&self, q: [f64; N]) -> RobotResult<Jaco<N>> {
        let j = self.chain.jacobian(&q).map_err(model_error)?;
        Ok(na::SMatrix::<f64, 6, N>::from_fn(|r, c| j[(r, c)]))
    }
}
