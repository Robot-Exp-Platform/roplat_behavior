//! Poses and points with covariances, carried through composition,
//! inversion and transformation to first order.
//!
//! A pose's uncertainty is a perturbation `τ = [ρ; θ]` on the right, in the
//! pose's own (child) frame: the true pose is `T̄ · Exp(τ)` with
//! `τ ~ N(0, Σ)`, the translational part `ρ` (m) first and the rotational
//! part `θ` (rad, a rotation vector) second, and `Σ` the 6 × 6
//! [`UncertainPose::covariance`]. `Exp` is the exponential map of SE(3).
//! This is the convention, and these are the Jacobians, of Solà, Deray and
//! Atchuthan, "A micro Lie theory for state estimation in robotics"
//! (arXiv:1812.01537), with `Ad(T) = [[R, [t]× R], [0, R]]`:
//!
//! | Operation | Result | Covariance |
//! |---|---|---|
//! | [`compose`](UncertainPose::compose) | `X · Y` | `Ad(Y⁻¹) Σx Ad(Y⁻¹)ᵀ + Σy` |
//! | [`inverse`](UncertainPose::inverse) | `X⁻¹` | `Ad(X) Σx Ad(X)ᵀ` |
//! | [`transform_point`](UncertainPose::transform_point) | `X · p` | `J Σx Jᵀ + R Σp Rᵀ`, `J = [R, −R [p]×]` |
//!
//! Composition assumes the two poses' errors independent, as when each comes
//! from a different sensor or calibration; correlated errors need their
//! cross-covariance, which this module does not take. The results are first
//! order: the mean pose is the composed one, not the mean of the composed
//! distribution, and the covariance is exact only in the limit of small
//! perturbations. Against Monte Carlo in the tests, with standard deviations
//! of 0.01–0.03 rad and 1–3 cm on poses about a metre apart, every
//! covariance entry is within 0.7 % of the product of the standard
//! deviations of its row and column; with 0.2 rad and 5 cm, within 2.3 %.
//! Larger errors are not characterised.
//!
//! Many tools give the covariance of a pose instead as an error of the
//! translation added in the parent frame and small rotations about the
//! parent's axes: `t = t̄ + δt`, `R = Exp(δφ) R̄`. To first order that is
//! `δt = R̄ ρ`, `δφ = R̄ θ`; [`UncertainPose::parent_covariance`] and
//! [`UncertainPose::from_parent_covariance`] convert.
//!
//! ```
//! use nalgebra as na;
//! use roplat_frames::uncertain::{UncertainPoint, UncertainPose};
//!
//! // A base with 0.01 rad² of yaw variance, and a sensor 2 m ahead of it.
//! let mut covariance = na::Matrix6::zeros();
//! covariance[(5, 5)] = 0.01;
//! let base = UncertainPose::new(na::Isometry3::identity(), covariance);
//! let mount = UncertainPose::exact(na::Isometry3::translation(2.0, 0.0, 0.0));
//! let sensor = base.compose(&mount);
//! // The yaw swings the sensor sideways by 2 m per radian: 0.04 m².
//! assert!((sensor.covariance[(1, 1)] - 0.04).abs() < 1e-12);
//! // A point the sensor sees 1 m further ahead: 3 m from the base.
//! let seen = sensor.transform_point(&UncertainPoint::exact(na::Point3::new(1.0, 0.0, 0.0)));
//! assert!((seen.covariance[(1, 1)] - 0.09).abs() < 1e-12);
//! ```

use nalgebra as na;

/// A pose with the covariance of its error, in the convention of the
/// [module](self).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UncertainPose {
    pub pose: na::Isometry3<f64>,
    /// Of `[ρ; θ]` on the right, in the pose's own frame: m², m·rad, rad².
    pub covariance: na::Matrix6<f64>,
}

/// A point with the covariance of its error, in the point's frame (m²).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UncertainPoint {
    pub point: na::Point3<f64>,
    pub covariance: na::Matrix3<f64>,
}

impl UncertainPoint {
    pub fn new(point: na::Point3<f64>, covariance: na::Matrix3<f64>) -> Self {
        Self { point, covariance }
    }

    /// A point known exactly.
    pub fn exact(point: na::Point3<f64>) -> Self {
        Self::new(point, na::Matrix3::zeros())
    }
}

impl UncertainPose {
    pub fn new(pose: na::Isometry3<f64>, covariance: na::Matrix6<f64>) -> Self {
        Self { pose, covariance }
    }

    /// A pose known exactly, such as a fixed mount from a drawing.
    pub fn exact(pose: na::Isometry3<f64>) -> Self {
        Self::new(pose, na::Matrix6::zeros())
    }

    /// `self · other`: `other` expressed in `self`'s frame, carried into
    /// `self`'s parent. The two errors are taken as independent.
    pub fn compose(&self, other: &UncertainPose) -> UncertainPose {
        let ad = adjoint(&other.pose.inverse());
        UncertainPose {
            pose: self.pose * other.pose,
            covariance: ad * self.covariance * ad.transpose() + other.covariance,
        }
    }

    /// The inverse pose: the parent seen from this frame.
    pub fn inverse(&self) -> UncertainPose {
        let ad = adjoint(&self.pose);
        UncertainPose {
            pose: self.pose.inverse(),
            covariance: ad * self.covariance * ad.transpose(),
        }
    }

    /// `point`, given in this pose's frame, in the parent frame. The two
    /// errors are taken as independent.
    pub fn transform_point(&self, point: &UncertainPoint) -> UncertainPoint {
        let r = self.pose.rotation.to_rotation_matrix().into_inner();
        let mut jacobian = na::Matrix3x6::zeros();
        jacobian.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
        jacobian
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(-r * point.point.coords.cross_matrix()));
        UncertainPoint {
            point: self.pose * point.point,
            covariance: jacobian * self.covariance * jacobian.transpose()
                + r * point.covariance * r.transpose(),
        }
    }

    /// The covariance of `[δt; δφ]`, the translation error added in the
    /// parent frame and the rotation error about the parent's axes.
    pub fn parent_covariance(&self) -> na::Matrix6<f64> {
        let rotate = block_rotation(&self.pose);
        rotate * self.covariance * rotate.transpose()
    }

    /// The pose with the covariance `parent` of `[δt; δφ]` in the parent
    /// frame, converted to this module's convention.
    pub fn from_parent_covariance(pose: na::Isometry3<f64>, parent: na::Matrix6<f64>) -> Self {
        let rotate = block_rotation(&pose);
        Self::new(pose, rotate.transpose() * parent * rotate)
    }
}

/// `Ad(T) = [[R, [t]× R], [0, R]]` for perturbations `[ρ; θ]`:
/// `T · Exp(τ) = Exp(Ad(T) τ) · T`.
fn adjoint(pose: &na::Isometry3<f64>) -> na::Matrix6<f64> {
    let r = pose.rotation.to_rotation_matrix().into_inner();
    let mut ad = na::Matrix6::zeros();
    ad.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    ad.fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(pose.translation.vector.cross_matrix() * r));
    ad.fixed_view_mut::<3, 3>(3, 3).copy_from(&r);
    ad
}

/// `diag(R, R)`.
fn block_rotation(pose: &na::Isometry3<f64>) -> na::Matrix6<f64> {
    let r = pose.rotation.to_rotation_matrix().into_inner();
    let mut rotate = na::Matrix6::zeros();
    rotate.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    rotate.fixed_view_mut::<3, 3>(3, 3).copy_from(&r);
    rotate
}
