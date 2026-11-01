//! A URDF tree as a `DynamicsModel`, and the `roplat_control` torque
//! controllers running on it.

use std::time::Duration;

use roplat_behavior::{DynamicsModel, JointState, msgs::JointSample};
use roplat_description::RobotModel;
use roplat_kinematics::UrdfDynamics;

const G: f64 = 9.81;

/// A planar 2R arm swinging in the vertical x–z plane (axes along y).
const ARM: &str = r#"<robot name="arm"><link name="base"/>
  <link name="upper"><inertial><origin xyz="0.2 0 0"/><mass value="3"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.01"/></inertial></link>
  <link name="fore"><inertial><origin xyz="0.15 0 0"/><mass value="2"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.01"/></inertial></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="1" effort="1"/></joint>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="0.5 0 0"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" velocity="1" effort="1"/></joint></robot>"#;

fn arm() -> UrdfDynamics<2> {
    UrdfDynamics::new(RobotModel::from_urdf_str(ARM).unwrap()).unwrap()
}

fn state(q: [f64; 2], dq: [f64; 2]) -> JointState<2> {
    JointState::from_meas(JointSample::default().with_q(q).with_dq(dq))
}

#[test]
fn terms_match_the_closed_form() {
    let model = arm();
    let (m1, m2, l1, c1, c2, i1, i2) = (3.0, 2.0, 0.5, 0.2, 0.15, 0.04, 0.02);
    let (q, dq): ([f64; 2], [f64; 2]) = ([0.3, 1.1], [0.8, -1.7]);
    let k2 = q[1].cos();
    let m = model.mass(q).unwrap();
    let m11 = i1 + i2 + m1 * c1 * c1 + m2 * (l1 * l1 + c2 * c2 + 2.0 * l1 * c2 * k2);
    let m12 = i2 + m2 * (c2 * c2 + l1 * c2 * k2);
    let m22 = i2 + m2 * c2 * c2;
    for (got, want) in [
        (m[(0, 0)], m11),
        (m[(0, 1)], m12),
        (m[(1, 0)], m12),
        (m[(1, 1)], m22),
    ] {
        assert!((got - want).abs() < 1e-12, "{m}");
    }

    // About +y, positive angles take +x towards -z.
    let h = m2 * l1 * c2 * q[1].sin();
    let c = model.coriolis(q, dq).unwrap();
    assert!(
        (c[0] + h * (2.0 * dq[0] * dq[1] + dq[1] * dq[1])).abs() < 1e-12,
        "{c:?}"
    );
    assert!((c[1] - h * dq[0] * dq[0]).abs() < 1e-12, "{c:?}");

    let g = model.gravity(q, [0.0, 0.0, -G]).unwrap();
    let (a1, a12) = (q[0].cos(), (q[0] + q[1]).cos());
    let g2 = -m2 * G * c2 * a12;
    let g1 = -(m1 * c1 + m2 * l1) * G * a1 + g2;
    assert!(
        (g[0] - g1).abs() < 1e-12 && (g[1] - g2).abs() < 1e-12,
        "{g:?} vs {g1} {g2}"
    );

    assert!(UrdfDynamics::<3>::new(RobotModel::from_urdf_str(ARM).unwrap()).is_err());
}

/// Integrates the arm under `controller`'s torques with semi-implicit Euler.
fn simulate(
    model: &UrdfDynamics<2>,
    mut controller: impl FnMut(JointState<2>, Duration) -> roplat_behavior::ControlStep<[f64; 2]>,
    q0: [f64; 2],
    seconds: f64,
) -> ([f64; 2], [f64; 2]) {
    let dt = 1e-3;
    let (mut q, mut dq) = (q0, [0.0; 2]);
    for _ in 0..(seconds / dt) as usize {
        let std::ops::ControlFlow::Continue((tau, _)) =
            controller(state(q, dq), Duration::from_secs_f64(dt))
        else {
            panic!("the controller ended the session");
        };
        let ddq = model
            .model()
            .forward_dynamics(&q, &dq, &tau, [0.0, 0.0, -G])
            .unwrap();
        for i in 0..2 {
            dq[i] += ddq[i] * dt;
            q[i] += dq[i] * dt;
        }
    }
    (q, dq)
}

#[test]
fn gravity_compensation_holds_the_arm() {
    let model = arm();
    let controller = roplat_control::gravity_compensation_control(arm(), [0.0, 0.0, -G]);
    let q0 = [0.4, -0.9];
    let (q, dq) = simulate(&model, controller, q0, 1.0);
    assert!(
        (q[0] - q0[0]).abs() < 1e-6 && (q[1] - q0[1]).abs() < 1e-6,
        "{q:?}"
    );
    assert!(dq.iter().all(|v| v.abs() < 1e-6));

    // Without it the arm falls.
    let (q, _) = simulate(
        &model,
        |_, _| std::ops::ControlFlow::Continue(([0.0; 2], false)),
        q0,
        0.3,
    );
    assert!((q[0] - q0[0]).abs() > 0.1, "{q:?}");
}

#[test]
fn computed_torque_reaches_its_target() {
    let model = arm();
    let target = [1.0, -0.5];
    let controller = roplat_control::computed_torque_control(
        arm(),
        target,
        [0.0; 2],
        [0.0; 2],
        [100.0; 2],
        [20.0; 2],
        [0.0, 0.0, -G],
    );
    let (q, dq) = simulate(&model, controller, [-0.3, 0.6], 2.0);
    assert!(
        (q[0] - target[0]).abs() < 1e-3 && (q[1] - target[1]).abs() < 1e-3,
        "{q:?}"
    );
    assert!(dq.iter().all(|v| v.abs() < 1e-2), "{dq:?}");
}
