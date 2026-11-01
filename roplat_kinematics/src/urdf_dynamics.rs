//! A URDF tree as a `roplat_behavior` dynamics model.

use nalgebra as na;
use roplat_behavior::{
    CoriolisInput, CoriolisInputSpace, GravityInput, GravityInputSpace, JMat, JointSpace,
    JointTorqueSpace, MassMatrixSpace, RobotException, RobotResult, SpaceMap,
};
use roplat_description::{DescriptionError, RobotModel};

/// The dynamics of a fixed-base URDF tree with `N` movable joints, as a
/// [`DynamicsModel<N>`](roplat_behavior::DynamicsModel), so the
/// `roplat_control` gravity-compensation and computed-torque controllers run
/// on any URDF arm without hand-written dynamics.
///
/// Coordinates are the model's [movable joints](RobotModel::movable_joints)
/// in file order. The gravity vector is the gravitational acceleration in the
/// root frame (e.g. `[0, 0, -9.81]`); the gravity term is the effort that
/// holds the tree still. Inertia conventions are those of
/// [`RobotModel::inverse_dynamics`].
#[derive(Clone, Debug)]
pub struct UrdfDynamics<const N: usize> {
    model: RobotModel,
}

impl<const N: usize> UrdfDynamics<N> {
    pub fn new(model: RobotModel) -> Result<Self, DescriptionError> {
        let dof = model.movable_joints().len();
        if dof != N {
            return Err(DescriptionError::DofMismatch { expected: dof, got: N });
        }
        Ok(Self { model })
    }

    pub fn model(&self) -> &RobotModel {
        &self.model
    }

    /// Efforts of the movable joints for `(q, qd, qdd)` under `gravity`.
    fn efforts(&self, q: &[f64; N], qd: &[f64; N], qdd: &[f64; N], gravity: [f64; 3]) -> [f64; N] {
        let movable = self.model.movable_joints();
        let all = self.model.inverse_dynamics(
            |joint| {
                let i = movable
                    .iter()
                    .position(|&j| self.model.joints[j].name == joint.name)
                    .expect("movable joint");
                (q[i], qd[i], qdd[i])
            },
            gravity,
        );
        std::array::from_fn(|i| all[movable[i]])
    }
}

fn model_error(error: DescriptionError) -> RobotException {
    RobotException::UnprocessableInstructionError(error.to_string())
}

impl<const N: usize> SpaceMap<JointSpace<N>, MassMatrixSpace<N>> for UrdfDynamics<N> {
    fn map(&self, q: [f64; N]) -> RobotResult<JMat<N>> {
        let m = self.model.mass_matrix(&q).map_err(model_error)?;
        Ok(na::SMatrix::from_fn(|r, c| m[(r, c)]))
    }
}

impl<const N: usize> SpaceMap<CoriolisInputSpace<N>, JointTorqueSpace<N>> for UrdfDynamics<N> {
    /// `C(q, q̇) q̇`: the efforts at zero acceleration without gravity.
    fn map(&self, input: CoriolisInput<N>) -> RobotResult<[f64; N]> {
        Ok(self.efforts(&input.q, &input.dq, &[0.0; N], [0.0; 3]))
    }
}

impl<const N: usize> SpaceMap<GravityInputSpace<N>, JointTorqueSpace<N>> for UrdfDynamics<N> {
    fn map(&self, input: GravityInput<N>) -> RobotResult<[f64; N]> {
        Ok(self.efforts(&input.q, &[0.0; N], &[0.0; N], input.gravity))
    }
}
