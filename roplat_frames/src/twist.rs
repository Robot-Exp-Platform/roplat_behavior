//! Twists and wrenches between frames: the adjoint of a pose.
//!
//! A twist is `[v, ω]`, linear before angular as in the contracts'
//! `[vx, vy, vz, wx, wy, wz]`: the angular velocity `ω` of a rigid body and
//! the velocity `v` of the body's point at the frame's origin, both along
//! the frame's axes. A wrench is `[f, τ]`: a force and the torque about the
//! frame's origin. With `pose` the pose of frame `b` in frame `a` (rotation
//! `R`, translation `p`), the same motion and the same load in frame `a`
//! are
//!
//! ```text
//! ω_a = R ω_b          v_a = R v_b + p × (R ω_b)
//! f_a = R f_b          τ_a = R τ_b + p × (R f_b)
//! ```
//!
//! that is `twist_a = Ad(pose) twist_b` with `Ad = [R, [p]× R; 0, R]`
//! ([`adjoint`]), and `wrench_a = Ad(pose⁻¹)ᵀ wrench_b`. A pose with no
//! rotation only moves the reference point: the velocity of another point
//! of the body, or the torque about another point. The power `v·f + ω·τ` is
//! the same in both frames.
//!
//! These are the adjoint of Murray, Li and Sastry, "A Mathematical
//! Introduction to Robotic Manipulation", section 2.4, with the linear part
//! first, and of Lynch and Park, "Modern Robotics", section 3.3.2, with the
//! linear part moved first.
//!
//! ```
//! use nalgebra::{Isometry3, Vector6};
//! use roplat_frames::twist;
//!
//! // A tool tip 0.1 m along the flange's z: the flange turning at 1 rad/s
//! // about its x moves the tip at 0.1 m/s along -y, ω × r.
//! let tip_in_flange = Isometry3::translation(0.0, 0.0, 0.1);
//! let at_flange = Vector6::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
//! let at_tip = twist::transform_twist(&tip_in_flange.inverse(), &at_flange);
//! assert!((at_tip - Vector6::new(0.0, -0.1, 0.0, 1.0, 0.0, 0.0)).norm() < 1e-12);
//! ```

use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};

use crate::so3::hat;

/// The adjoint of `pose`, the pose of frame `b` in frame `a`: the 6 × 6
/// matrix that takes a twist in `b` to the same twist in `a`.
pub fn adjoint(pose: &Isometry3<f64>) -> Matrix6<f64> {
    let r = pose.rotation.to_rotation_matrix().into_inner();
    let p = pose.translation.vector;
    let mut ad = Matrix6::zeros();
    ad.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    ad.fixed_view_mut::<3, 3>(0, 3).copy_from(&(hat(&p) * r));
    ad.fixed_view_mut::<3, 3>(3, 3).copy_from(&r);
    ad
}

/// `twist`, `[v, ω]` in frame `b`, as the same twist in frame `a`, given
/// `pose`, the pose of `b` in `a`.
pub fn transform_twist(pose: &Isometry3<f64>, twist: &Vector6<f64>) -> Vector6<f64> {
    let (v, w) = split(twist);
    let w_a = pose.rotation * w;
    let v_a = pose.rotation * v + pose.translation.vector.cross(&w_a);
    join(v_a, w_a)
}

/// `wrench`, `[f, τ]` in frame `b`, as the same wrench in frame `a`, given
/// `pose`, the pose of `b` in `a`.
pub fn transform_wrench(pose: &Isometry3<f64>, wrench: &Vector6<f64>) -> Vector6<f64> {
    let (f, t) = split(wrench);
    let f_a = pose.rotation * f;
    let t_a = pose.rotation * t + pose.translation.vector.cross(&f_a);
    join(f_a, t_a)
}

fn split(v: &Vector6<f64>) -> (Vector3<f64>, Vector3<f64>) {
    (
        v.fixed_rows::<3>(0).into_owned(),
        v.fixed_rows::<3>(3).into_owned(),
    )
}

fn join(a: Vector3<f64>, b: Vector3<f64>) -> Vector6<f64> {
    Vector6::new(a.x, a.y, a.z, b.x, b.y, b.z)
}
