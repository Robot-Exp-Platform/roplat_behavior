//! First-order covariance propagation on SE(3) against Monte Carlo: poses
//! drawn as `T̄ · Exp(τ)`, `τ ~ N(0, Σ)`, pushed through the exact
//! operation, and the sample covariance of the result's perturbation
//! compared with the propagated one.
//!
//! The exponential and logarithm of SE(3) are written here from their
//! closed forms (rotation vector and the `V` matrix), not taken from the
//! library, which has none.

use nalgebra as na;
use roplat_frames::uncertain::{UncertainPoint, UncertainPose};

/// SplitMix64 and Box–Muller, so every run draws the same samples.
struct Normal(u64);

impl Normal {
    fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn standard(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }

    /// A draw of `N(0, covariance)` through its Cholesky factor.
    fn draw<const D: usize>(&mut self, factor: &na::SMatrix<f64, D, D>) -> na::SVector<f64, D> {
        factor * na::SVector::<f64, D>::from_fn(|_, _| self.standard())
    }
}

/// `V(θ) = I + (1 − cos a)/a² K + (a − sin a)/a³ K²`, `K = [θ]×`, `a = |θ|`.
fn v_matrix(theta: &na::Vector3<f64>) -> na::Matrix3<f64> {
    let a = theta.norm();
    let k = theta.cross_matrix();
    let (b, c) = if a < 1e-6 {
        (0.5 - a * a / 24.0, 1.0 / 6.0 - a * a / 120.0)
    } else {
        ((1.0 - a.cos()) / (a * a), (a - a.sin()) / (a * a * a))
    };
    na::Matrix3::identity() + k * b + k * k * c
}

fn exp(tau: &na::Vector6<f64>) -> na::Isometry3<f64> {
    let rho = tau.fixed_rows::<3>(0).into_owned();
    let theta = tau.fixed_rows::<3>(3).into_owned();
    na::Isometry3::from_parts(
        na::Translation3::from(v_matrix(&theta) * rho),
        na::UnitQuaternion::from_scaled_axis(theta),
    )
}

fn log(pose: &na::Isometry3<f64>) -> na::Vector6<f64> {
    let theta = pose.rotation.scaled_axis();
    let rho = v_matrix(&theta).try_inverse().unwrap() * pose.translation.vector;
    na::Vector6::new(rho.x, rho.y, rho.z, theta.x, theta.y, theta.z)
}

/// The sample covariance of `samples` about their mean.
fn sample_covariance<const D: usize>(samples: &[na::SVector<f64, D>]) -> na::SMatrix<f64, D, D> {
    let n = samples.len() as f64;
    let mean = samples.iter().sum::<na::SVector<f64, D>>() / n;
    samples
        .iter()
        .map(|s| (s - mean) * (s - mean).transpose())
        .sum::<na::SMatrix<f64, D, D>>()
        / (n - 1.0)
}

/// The largest difference between two covariances, each entry scaled by
/// the propagated standard deviations of its row and column: a correlation
/// error, comparable across units.
fn scaled_difference<const D: usize>(
    sampled: &na::SMatrix<f64, D, D>,
    propagated: &na::SMatrix<f64, D, D>,
) -> f64 {
    let mut worst = 0.0f64;
    for i in 0..D {
        for j in 0..D {
            let scale = (propagated[(i, i)] * propagated[(j, j)]).sqrt();
            worst = worst.max((sampled[(i, j)] - propagated[(i, j)]).abs() / scale);
        }
    }
    worst
}

/// A covariance with every entry correlated: `L Lᵀ` for a lower-triangular
/// `L` with diagonal `sigma` and off-diagonal terms `coupling` of the
/// diagonal.
fn correlated(sigma: [f64; 6], coupling: f64) -> (na::Matrix6<f64>, na::Matrix6<f64>) {
    let factor = na::Matrix6::from_fn(|i, j| {
        if i == j {
            sigma[i]
        } else if j < i {
            coupling * sigma[i] * if (i + j) % 2 == 0 { 1.0 } else { -1.0 }
        } else {
            0.0
        }
    });
    (factor * factor.transpose(), factor)
}

fn pose(t: [f64; 3], axis_angle: [f64; 3]) -> na::Isometry3<f64> {
    na::Isometry3::new(na::Vector3::from(t), na::Vector3::from(axis_angle))
}

const SAMPLES: usize = 200_000;

/// Two poses a metre or so apart, each with centimetre and
/// hundredth-of-a-radian errors, correlated within each pose.
#[test]
fn composition_matches_monte_carlo() {
    let x = pose([0.8, -0.3, 0.5], [0.4, -0.7, 1.1]);
    let y = pose([0.2, 1.1, -0.4], [-1.2, 0.3, 0.5]);
    let (sx, lx) = correlated([0.01, 0.02, 0.015, 0.01, 0.02, 0.015], 0.3);
    let (sy, ly) = correlated([0.02, 0.01, 0.01, 0.015, 0.01, 0.02], -0.2);
    let propagated = UncertainPose::new(x, sx).compose(&UncertainPose::new(y, sy));
    assert_eq!(propagated.pose, x * y);
    let mut rng = Normal(12);
    let mean_inverse = propagated.pose.inverse();
    let samples: Vec<na::Vector6<f64>> = (0..SAMPLES)
        .map(|_| {
            let xi = x * exp(&rng.draw(&lx));
            let yi = y * exp(&rng.draw(&ly));
            log(&(mean_inverse * xi * yi))
        })
        .collect();
    let difference = scaled_difference(&sample_covariance(&samples), &propagated.covariance);
    println!("compose: largest scaled difference from Monte Carlo {difference:.4}");
    assert!(difference < 0.02, "{difference}");
}

#[test]
fn inversion_matches_monte_carlo() {
    let x = pose([1.5, -0.6, 0.9], [0.9, 0.2, -1.4]);
    let (sx, lx) = correlated([0.02, 0.01, 0.03, 0.01, 0.02, 0.01], 0.4);
    let propagated = UncertainPose::new(x, sx).inverse();
    let mut rng = Normal(34);
    let samples: Vec<na::Vector6<f64>> = (0..SAMPLES)
        .map(|_| {
            let xi = x * exp(&rng.draw(&lx));
            // X⁻¹ = X̄⁻¹ · Exp(δ), so δ = Log(X̄ · X⁻¹).
            log(&(x * xi.inverse()))
        })
        .collect();
    let difference = scaled_difference(&sample_covariance(&samples), &propagated.covariance);
    println!("inverse: largest scaled difference from Monte Carlo {difference:.4}");
    assert!(difference < 0.02, "{difference}");
}

#[test]
fn transforming_a_point_matches_monte_carlo() {
    let x = pose([0.3, 0.2, 1.0], [-0.5, 0.8, 0.3]);
    let (sx, lx) = correlated([0.01, 0.01, 0.02, 0.02, 0.01, 0.015], 0.25);
    let point = na::Point3::new(2.0, -1.0, 0.5);
    let sp = na::Matrix3::new(4e-4, 1e-4, 0.0, 1e-4, 1e-4, -5e-5, 0.0, -5e-5, 9e-4);
    let lp = sp.cholesky().unwrap().l();
    let propagated = UncertainPose::new(x, sx).transform_point(&UncertainPoint::new(point, sp));
    assert_eq!(propagated.point, x * point);
    let mut rng = Normal(56);
    let samples: Vec<na::Vector3<f64>> = (0..SAMPLES)
        .map(|_| {
            let xi = x * exp(&rng.draw(&lx));
            (xi * (point + rng.draw(&lp))).coords
        })
        .collect();
    let difference = scaled_difference(&sample_covariance(&samples), &propagated.covariance);
    println!("transform_point: largest scaled difference from Monte Carlo {difference:.4}");
    assert!(difference < 0.02, "{difference}");
}

/// The parent-frame form `t = t̄ + δt`, `R = Exp(δφ) R̄`, sampled, and the
/// conversion back.
#[test]
fn parent_frame_covariance_matches_monte_carlo() {
    let x = pose([0.5, 1.5, -0.2], [1.0, -0.4, 0.6]);
    let (sx, lx) = correlated([0.02, 0.01, 0.01, 0.01, 0.02, 0.015], 0.35);
    let uncertain = UncertainPose::new(x, sx);
    let mut rng = Normal(78);
    let samples: Vec<na::Vector6<f64>> = (0..SAMPLES)
        .map(|_| {
            let xi = x * exp(&rng.draw(&lx));
            let dt = xi.translation.vector - x.translation.vector;
            let dphi = (xi.rotation * x.rotation.inverse()).scaled_axis();
            na::Vector6::new(dt.x, dt.y, dt.z, dphi.x, dphi.y, dphi.z)
        })
        .collect();
    let parent = uncertain.parent_covariance();
    let difference = scaled_difference(&sample_covariance(&samples), &parent);
    println!("parent frame: largest scaled difference from Monte Carlo {difference:.4}");
    assert!(difference < 0.02, "{difference}");
    let back = UncertainPose::from_parent_covariance(x, parent);
    assert!(
        (back.covariance - sx).abs().max() < 1e-15,
        "{}",
        back.covariance - sx
    );
}

/// Closed-form cases: a base's yaw error swings a point ahead of it
/// sideways in proportion to the distance; a pure translation error
/// inverted is the same error turned into the other frame.
#[test]
fn simple_cases_by_hand() {
    let sigma2 = 0.01;
    let mut yaw = na::Matrix6::zeros();
    yaw[(5, 5)] = sigma2;
    let base = UncertainPose::new(pose([0.0; 3], [0.0, 0.0, std::f64::consts::FRAC_PI_2]), yaw);
    let ahead = UncertainPose::exact(na::Isometry3::translation(1.0, 0.0, 0.0));
    let tip = base.compose(&ahead);
    // In the tip's frame: lateral (y) variance σ², fully correlated with
    // the yaw.
    let mut expected = na::Matrix6::zeros();
    for (i, j) in [(1, 1), (1, 5), (5, 1), (5, 5)] {
        expected[(i, j)] = sigma2;
    }
    assert!(
        (tip.covariance - expected).abs().max() < 1e-15,
        "{}",
        tip.covariance
    );
    // In the base's parent frame the lateral direction is -x.
    let parent = tip.parent_covariance();
    assert!((parent[(0, 0)] - sigma2).abs() < 1e-15 && (parent[(0, 5)] + sigma2).abs() < 1e-15);

    let mut along_x = na::Matrix6::zeros();
    along_x[(0, 0)] = 4e-4;
    let turned = UncertainPose::new(
        pose([0.0; 3], [0.0, 0.0, std::f64::consts::FRAC_PI_2]),
        along_x,
    );
    let inverse = turned.inverse();
    // The inverse's own frame is the parent; its error there is −R ρ, the
    // turned frame's x error along the parent's y.
    let mut expected = na::Matrix6::zeros();
    expected[(1, 1)] = 4e-4;
    assert!(
        (inverse.covariance - expected).abs().max() < 1e-15,
        "{}",
        inverse.covariance
    );
}

/// At tenths of a radian the first-order result drifts from Monte Carlo:
/// printed for the record, and bounded loosely.
#[test]
fn large_rotation_errors_are_only_approximate() {
    let x = pose([0.8, -0.3, 0.5], [0.4, -0.7, 1.1]);
    let y = pose([0.2, 1.1, -0.4], [-1.2, 0.3, 0.5]);
    let (sx, lx) = correlated([0.05, 0.05, 0.05, 0.2, 0.2, 0.2], 0.3);
    let (sy, ly) = correlated([0.05, 0.05, 0.05, 0.2, 0.2, 0.2], -0.2);
    let propagated = UncertainPose::new(x, sx).compose(&UncertainPose::new(y, sy));
    let mut rng = Normal(90);
    let mean_inverse = propagated.pose.inverse();
    let samples: Vec<na::Vector6<f64>> = (0..50_000)
        .map(|_| log(&(mean_inverse * x * exp(&rng.draw(&lx)) * y * exp(&rng.draw(&ly)))))
        .collect();
    let difference = scaled_difference(&sample_covariance(&samples), &propagated.covariance);
    println!("compose at 0.2 rad: largest scaled difference from Monte Carlo {difference:.4}");
    assert!(difference < 0.5, "{difference}");
}
