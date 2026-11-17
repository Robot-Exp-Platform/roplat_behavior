//! Rotations as a Lie group: the exponential and logarithm between a
//! rotation and its tangent `phi` (the rotation vector: axis times angle,
//! radians), and the Jacobians of the exponential that estimators and
//! optimisers need to move a perturbation between the two sides of a
//! rotation.
//!
//! With `θ = |phi|` and `[phi]×` the matrix of `phi × ·` ([`hat`]):
//!
//! ```text
//! J_l(phi)    = I + (1 - cos θ)/θ² [phi]× + (θ - sin θ)/θ³ [phi]×²
//! J_l(phi)⁻¹ = I - ½ [phi]× + (1/θ² - (1 + cos θ)/(2θ sin θ)) [phi]×²
//! J_r(phi)    = J_l(-phi),   J_r(phi)⁻¹ = J_l(-phi)⁻¹
//! ```
//!
//! so that, to first order in a small `delta`,
//! `exp(phi + delta) = exp(J_l(phi) delta) exp(phi) = exp(phi) exp(J_r(phi) delta)`.
//! These are the forms of Solà, Deray and Atchuthan, "A micro Lie theory
//! for state estimation in robotics" (arXiv:1812.01537), and of Barfoot,
//! "State Estimation for Robotics", section 7.1. Near `θ = 0` each
//! coefficient is its Taylor series, which keeps full precision where the
//! closed forms cancel. `J_l⁻¹` is singular at `θ = 2π`; a logarithm never
//! gives such a tangent.
//!
//! ```
//! use nalgebra::Vector3;
//! use roplat_frames::so3;
//!
//! let phi = Vector3::new(0.3, -0.2, 0.9);
//! let back = so3::log(&so3::exp(&phi));
//! assert!((back - phi).norm() < 1e-12);
//! let identity = so3::left_jacobian(&phi) * so3::left_jacobian_inverse(&phi);
//! assert!((identity - nalgebra::Matrix3::identity()).norm() < 1e-12);
//! ```

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

/// Below this angle the coefficients are their Taylor series.
const SMALL: f64 = 1e-2;

/// The skew-symmetric matrix of `v`: `hat(v) w = v × w`.
pub fn hat(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// The vector of a skew-symmetric matrix, the inverse of [`hat`]; of a
/// matrix that is not skew-symmetric, the vector of its skew part.
pub fn vee(m: &Matrix3<f64>) -> Vector3<f64> {
    Vector3::new(m[(2, 1)] - m[(1, 2)], m[(0, 2)] - m[(2, 0)], m[(1, 0)] - m[(0, 1)]) / 2.0
}

/// The rotation of `phi`: `|phi|` radians about `phi`'s direction.
pub fn exp(phi: &Vector3<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_scaled_axis(*phi)
}

/// The tangent of `rotation`, with an angle in `[0, π]`.
pub fn log(rotation: &UnitQuaternion<f64>) -> Vector3<f64> {
    rotation.scaled_axis()
}

/// `(1 - cos θ)/θ²` and `(θ - sin θ)/θ³` for `θ = |phi|`.
fn coefficients(phi: &Vector3<f64>) -> (f64, f64) {
    let t2 = phi.norm_squared();
    if t2 < SMALL * SMALL {
        (
            0.5 - t2 / 24.0 + t2 * t2 / 720.0,
            1.0 / 6.0 - t2 / 120.0 + t2 * t2 / 5040.0,
        )
    } else {
        let t = t2.sqrt();
        ((1.0 - t.cos()) / t2, (t - t.sin()) / (t2 * t))
    }
}

/// `1/θ² - (1 + cos θ)/(2θ sin θ)` for `θ = |phi|`, written with
/// `(1 + cos θ)/sin θ = 1/tan(θ/2)` so that it holds at `θ = π`.
fn inverse_coefficient(phi: &Vector3<f64>) -> f64 {
    let t2 = phi.norm_squared();
    if t2 < SMALL * SMALL {
        1.0 / 12.0 + t2 / 720.0 + t2 * t2 / 30240.0
    } else {
        let t = t2.sqrt();
        1.0 / t2 - 1.0 / (2.0 * t * (t / 2.0).tan())
    }
}

/// The left Jacobian `J_l(phi)`: `exp(phi + delta) ≈ exp(J_l delta) exp(phi)`.
pub fn left_jacobian(phi: &Vector3<f64>) -> Matrix3<f64> {
    let (a, b) = coefficients(phi);
    let k = hat(phi);
    Matrix3::identity() + k * a + k * k * b
}

/// The inverse of [`left_jacobian`].
pub fn left_jacobian_inverse(phi: &Vector3<f64>) -> Matrix3<f64> {
    let c = inverse_coefficient(phi);
    let k = hat(phi);
    Matrix3::identity() - k * 0.5 + k * k * c
}

/// The right Jacobian `J_r(phi) = J_l(-phi)`:
/// `exp(phi + delta) ≈ exp(phi) exp(J_r delta)`.
pub fn right_jacobian(phi: &Vector3<f64>) -> Matrix3<f64> {
    left_jacobian(&-phi)
}

/// The inverse of [`right_jacobian`].
pub fn right_jacobian_inverse(phi: &Vector3<f64>) -> Matrix3<f64> {
    left_jacobian_inverse(&-phi)
}
