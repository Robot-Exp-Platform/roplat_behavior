//! SO(3) and the adjoint against their definitions: the Jacobians as
//! derivatives of nalgebra's exponential and logarithm, the adjoint as the
//! velocity field of a rigid body and the power a load does on it.

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3, Vector6};
use roplat_frames::{so3, twist};

/// A reproducible pseudo-random sequence in `[-1, 1]` (a 64-bit LCG).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
    }

    fn vector(&mut self, scale: f64) -> Vector3<f64> {
        Vector3::new(self.next(), self.next(), self.next()) * scale
    }
}

/// Tangents from tiny to nearly a half turn, and some on the boundary of
/// the series.
fn tangents() -> Vec<Vector3<f64>> {
    let mut random = Lcg(7);
    let mut out = vec![
        Vector3::new(1e-9, 0.0, 0.0),
        Vector3::new(0.0, 5e-3, 5e-3),
        Vector3::new(0.0, 0.0, 1e-2),
        Vector3::new(0.0, 0.0, 1.0000001e-2),
        Vector3::new(0.0, 3.1, 0.0),
    ];
    for scale in [1e-4, 0.01, 0.3, 1.0, 1.7] {
        for _ in 0..6 {
            out.push(random.vector(scale));
        }
    }
    out
}

/// Column `i` of `d/dδ log(f(δ))` at δ = 0 by central differences.
fn derivative(f: impl Fn(Vector3<f64>) -> UnitQuaternion<f64>) -> Matrix3<f64> {
    let h = 1e-6;
    let mut m = Matrix3::zeros();
    for i in 0..3 {
        let mut d = Vector3::zeros();
        d[i] = h;
        let column = (f(d).scaled_axis() - f(-d).scaled_axis()) / (2.0 * h);
        m.set_column(i, &column);
    }
    m
}

#[test]
fn exp_and_log_are_inverse() {
    for phi in tangents() {
        let back = so3::log(&so3::exp(&phi));
        assert!((back - phi).norm() < 1e-12 * phi.norm().max(1.0), "{phi:?} -> {back:?}");
    }
    // A tangent beyond a half turn comes back as the shorter one.
    let long = Vector3::new(0.0, 0.0, 4.0);
    let back = so3::log(&so3::exp(&long));
    assert!((back - Vector3::new(0.0, 0.0, 4.0 - std::f64::consts::TAU)).norm() < 1e-12);
    let v = Vector3::new(0.3, -1.2, 2.0);
    assert_eq!(so3::vee(&so3::hat(&v)), v);
    let w = Vector3::new(-0.7, 0.1, 0.4);
    assert!((so3::hat(&v) * w - v.cross(&w)).norm() < 1e-15);
}

#[test]
fn the_jacobians_are_the_derivatives_of_the_exponential() {
    for phi in tangents().into_iter().filter(|p| p.norm() > 1e-6) {
        let r = so3::exp(&phi);
        // exp(phi + δ) exp(phi)⁻¹ ≈ exp(J_l δ): d/dδ of its log is J_l.
        let left = derivative(|d| so3::exp(&(phi + d)) * r.inverse());
        // exp(phi)⁻¹ exp(phi + δ) ≈ exp(J_r δ).
        let right = derivative(|d| r.inverse() * so3::exp(&(phi + d)));
        let (jl, jr) = (so3::left_jacobian(&phi), so3::right_jacobian(&phi));
        assert!((left - jl).norm() < 1e-7, "J_l at {phi:?}: {left} vs {jl}");
        assert!((right - jr).norm() < 1e-7, "J_r at {phi:?}: {right} vs {jr}");
    }
}

#[test]
fn the_inverses_invert() {
    for phi in tangents() {
        let i = Matrix3::identity();
        let l = so3::left_jacobian(&phi) * so3::left_jacobian_inverse(&phi);
        let r = so3::right_jacobian(&phi) * so3::right_jacobian_inverse(&phi);
        assert!((l - i).norm() < 1e-12, "J_l J_l⁻¹ at {phi:?}: {l}");
        assert!((r - i).norm() < 1e-12, "J_r J_r⁻¹ at {phi:?}: {r}");
        // J_l = R J_r: both describe the same perturbation.
        let rotation = so3::exp(&phi).to_rotation_matrix().into_inner();
        let moved = rotation * so3::right_jacobian(&phi);
        assert!((so3::left_jacobian(&phi) - moved).norm() < 1e-12, "{phi:?}");
    }
    // At a half turn the inverse is still finite: (1 + cos)/sin -> 0.
    let half = Vector3::new(0.0, std::f64::consts::PI, 0.0);
    let inverse = so3::left_jacobian_inverse(&half);
    assert!(inverse.iter().all(|v| v.is_finite()));
    assert!((so3::left_jacobian(&half) * inverse - Matrix3::identity()).norm() < 1e-12);
}

fn poses() -> Vec<Isometry3<f64>> {
    let mut random = Lcg(11);
    (0..20)
        .map(|_| {
            Isometry3::from_parts(
                Translation3::from(random.vector(2.0)),
                so3::exp(&random.vector(2.5)),
            )
        })
        .collect()
}

#[test]
fn a_transformed_twist_moves_every_point_as_before() {
    let mut random = Lcg(13);
    for pose in poses() {
        let (v_b, w_b) = (random.vector(1.0), random.vector(2.0));
        let twist_b = Vector6::new(v_b.x, v_b.y, v_b.z, w_b.x, w_b.y, w_b.z);
        let twist_a = twist::transform_twist(&pose, &twist_b);
        assert!((twist::adjoint(&pose) * twist_b - twist_a).norm() < 1e-12);
        let v_a: Vector3<f64> = twist_a.fixed_rows::<3>(0).into_owned();
        let w_a: Vector3<f64> = twist_a.fixed_rows::<3>(3).into_owned();
        // A body point at x in b moves at v_b + ω_b × x along b's axes; at
        // X = pose x in a, the a-twist gives v_a + ω_a × X along a's axes.
        for _ in 0..5 {
            let x = random.vector(1.5);
            let in_b = v_b + w_b.cross(&x);
            let point = pose * nalgebra::Point3::from(x);
            let in_a = v_a + w_a.cross(&point.coords);
            assert!((pose.rotation * in_b - in_a).norm() < 1e-12);
        }
        // There and back.
        let back = twist::transform_twist(&pose.inverse(), &twist_a);
        assert!((back - twist_b).norm() < 1e-12);
    }
}

#[test]
fn a_transformed_wrench_does_the_same_work() {
    let mut random = Lcg(17);
    for pose in poses() {
        let twist_b = Vector6::from_iterator((0..6).map(|_| random.next()));
        let wrench_b = Vector6::from_iterator((0..6).map(|_| 10.0 * random.next()));
        let twist_a = twist::transform_twist(&pose, &twist_b);
        let wrench_a = twist::transform_wrench(&pose, &wrench_b);
        assert!((twist_a.dot(&wrench_a) - twist_b.dot(&wrench_b)).abs() < 1e-10);
        // The wrench matrix is the transposed adjoint of the inverse.
        let dual = twist::adjoint(&pose.inverse()).transpose() * wrench_b;
        assert!((dual - wrench_a).norm() < 1e-10);
    }
    // A force along x at a lever of 0.5 m along y twists about -z.
    let lever = Isometry3::translation(0.0, 0.5, 0.0);
    let pull = Vector6::new(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    let about_origin = twist::transform_wrench(&lever, &pull);
    assert!((about_origin - Vector6::new(2.0, 0.0, 0.0, 0.0, 0.0, -1.0)).norm() < 1e-12);
}
