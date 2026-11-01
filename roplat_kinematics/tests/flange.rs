//! A flange beyond the last DH frame, and inverse kinematics that reports an
//! unreachable target.
use roplat_behavior::JointLimits;
use roplat_behavior::{ForwardKinematics, InverseKinematics, JacobianModel, Joints, Pose};
use roplat_kinematics::{ArmForwardKinematics, ArmInverseKinematics, DhParam, dh_kinematics};

/// Two 1 m links in the x–y plane with the flange 0.5 m further along the
/// last link, so the arm reaches 2.5 m.
struct Planar;

impl ArmForwardKinematics<2> for Planar {
    const DH: [DhParam; 2] = [
        DhParam::DH { theta: 0., d: 0., r: 1., alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 1., alpha: 0. },
    ];
    const FLANGE: Pose = Pose::from_position([0.5, 0., 0.]);
}

impl Joints<2> for Planar {
    const JOINT_LIMITS: JointLimits<2> = JointLimits::positions([-3.; 2], [3.; 2]);
}

impl ArmInverseKinematics<2> for Planar {}

dh_kinematics!(Planar, 2, inverse);

#[test]
fn forward_kinematics_reaches_the_flange() {
    let p = Planar
        .forward_kinematics([0., std::f64::consts::FRAC_PI_2])
        .unwrap()
        .position();
    assert!(
        (p[0] - 1.).abs() < 1e-12 && (p[1] - 1.5).abs() < 1e-12,
        "{p:?}"
    );
}

#[test]
fn the_jacobian_is_the_flange_velocity() {
    let q = [0.3, -0.7];
    let j = Planar.jacobian(q).unwrap();
    let h = 1e-7;
    for col in 0..2 {
        let mut plus = q;
        plus[col] += h;
        let (a, b) = (
            Planar::fk_end_pose(&q).position(),
            Planar::fk_end_pose(&plus).position(),
        );
        for row in 0..3 {
            let numeric = (b[row] - a[row]) / h;
            assert!(
                (j[(row, col)] - numeric).abs() < 1e-5,
                "row {row} col {col}"
            );
        }
    }
}

#[test]
fn a_reachable_target_is_reached_and_an_unreachable_one_refused() {
    let target = Planar::fk_end_pose(&[0.4, 0.9]);
    let q = Planar.inverse_kinematics(target).unwrap();
    let reached = Planar::fk_end_pose(&q).position();
    let wanted = target.position();
    assert!(
        (0..3).all(|i| (reached[i] - wanted[i]).abs() < 1e-4),
        "{reached:?}"
    );

    // Review finding F3: a target beyond reach used to come back as Ok.
    assert!(
        Planar
            .inverse_kinematics(Pose::from_position([5., 0., 0.]))
            .is_err()
    );
}
