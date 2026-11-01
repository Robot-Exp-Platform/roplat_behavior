//! The Cartesian servo on a UR5-like arm read from URDF.

use std::time::Duration;

use nalgebra as na;
use roplat_behavior::{
    ForwardKinematics, JacobianModel, JointState,
    msgs::{InputAxes, JointSample},
};
use roplat_control::{
    CartesianServo, ServoConfig, ServoFrame, ServoStatus, TeleopMapping, cartesian_teleop_control,
    proximity_scale,
};
use roplat_description::RobotModel;
use roplat_kinematics::UrdfKinematics;

/// UR5 standard DH rows `(d, a, alpha)`, all offsets zero: the elbow is
/// straight (singular) at `q[2] = 0`, the wrist at `q[4] = 0`.
const UR5: [(f64, f64, f64); 6] = [
    (0.089159, 0.0, std::f64::consts::FRAC_PI_2),
    (0.0, -0.425, 0.0),
    (0.0, -0.39225, 0.0),
    (0.10915, 0.0, std::f64::consts::FRAC_PI_2),
    (0.09465, 0.0, -std::f64::consts::FRAC_PI_2),
    (0.0823, 0.0, 0.0),
];

const HOME: [f64; 6] = [0.3, -1.0, 1.4, -1.97, -1.57, 0.2];

fn origin(t: na::Isometry3<f64>) -> String {
    let p = t.translation.vector;
    let (r, pi, y) = t.rotation.euler_angles();
    format!(
        r#"<origin xyz="{} {} {}" rpy="{r} {pi} {y}"/>"#,
        p.x, p.y, p.z
    )
}

fn urdf() -> String {
    let rx = |a: f64| na::Isometry3::rotation(na::Vector3::x() * a);
    let mut links = String::from(r#"<link name="l0"/>"#);
    let mut joints = String::new();
    let mut after = na::Isometry3::identity();
    for (i, (d, a, alpha)) in UR5.into_iter().enumerate() {
        let before = after * na::Isometry3::translation(0.0, 0.0, d);
        links += &format!(r#"<link name="l{}"/>"#, i + 1);
        joints += &format!(
            r#"<joint name="j{}" type="revolute"><parent link="l{i}"/><child link="l{}"/>{}<axis xyz="0 0 1"/>
               <limit lower="-6.28" upper="6.28" velocity="2.5" effort="150"/></joint>"#,
            i + 1,
            i + 1,
            origin(before)
        );
        after = na::Isometry3::translation(a, 0.0, 0.0) * rx(alpha);
    }
    links += r#"<link name="tool"/>"#;
    joints += &format!(
        r#"<joint name="tool_mount" type="fixed"><parent link="l6"/><child link="tool"/>{}</joint>"#,
        origin(after)
    );
    format!(r#"<robot name="ur5">{links}{joints}</robot>"#)
}

fn model() -> UrdfKinematics<6> {
    let model = RobotModel::from_urdf_str(&urdf()).unwrap();
    UrdfKinematics::new(model.chain("l0", "tool").unwrap()).unwrap()
}

fn servo(config: ServoConfig) -> CartesianServo<UrdfKinematics<6>, 6> {
    let model = model();
    let limits = model.joint_limits();
    CartesianServo::new(model, limits, config).unwrap()
}

fn twist_of(model: &UrdfKinematics<6>, q: [f64; 6], dq: [f64; 6]) -> na::Vector6<f64> {
    model.jacobian(q).unwrap() * na::Vector6::from(dq)
}

fn tip(model: &UrdfKinematics<6>, q: [f64; 6]) -> na::Isometry3<f64> {
    model.forward_kinematics(q).unwrap().quat()
}

fn condition(model: &UrdfKinematics<6>, q: [f64; 6]) -> f64 {
    let s = model.jacobian(q).unwrap().singular_values();
    s.max() / s.min()
}

fn state(q: [f64; 6]) -> JointState<6> {
    JointState::from_meas(JointSample::default().with_q(q))
}

/// From the shoulder (on the axis of joint 2) to the tip.
fn shoulder_to_tip(q: [f64; 6]) -> na::Vector3<f64> {
    let model = RobotModel::from_urdf_str(&urdf()).unwrap();
    let shoulder = UrdfKinematics::<1>::new(model.chain("l0", "l1").unwrap()).unwrap();
    let s = shoulder
        .forward_kinematics([q[0]])
        .unwrap()
        .quat()
        .translation
        .vector;
    tip(&self::model(), q).translation.vector - s
}

/// Moves the tip straight away from the shoulder at `speed`, orientation
/// fixed: this straightens the elbow.
fn extend(q: [f64; 6], speed: f64) -> [f64; 6] {
    let d = shoulder_to_tip(q).normalize() * speed;
    [d.x, d.y, d.z, 0.0, 0.0, 0.0]
}

/// The twist of joint motion `dq` at `q`.
fn twist_array(q: [f64; 6], dq: [f64; 6]) -> [f64; 6] {
    twist_of(&model(), q, dq).into()
}

/// Runs a position controller on an ideal device for `cycles` of `dt`.
fn run(
    control: &mut impl FnMut(JointState<6>, Duration) -> roplat_behavior::ControlStep<[f64; 6]>,
    mut q: [f64; 6],
    cycles: usize,
    dt: Duration,
) -> [f64; 6] {
    for _ in 0..cycles {
        let std::ops::ControlFlow::Continue((command, _)) = control(state(q), dt) else {
            panic!("servo ended the session");
        };
        q = command;
    }
    q
}

#[test]
fn follows_the_twist_away_from_limits() {
    let twist = [0.05, -0.03, 0.02, 0.1, 0.0, -0.1];
    let mut exact = servo(ServoConfig { damping: 0.0, ..Default::default() });
    let step = exact.step(&HOME, twist, 0.001).unwrap();
    assert_eq!(step.status, ServoStatus::Free);
    assert_eq!(step.scale, 1.0);
    assert!((step.condition - condition(&model(), HOME)).abs() < 1e-6);
    let achieved = twist_of(&model(), HOME, step.dq);
    assert!(
        (achieved - na::Vector6::from(twist)).amax() < 1e-12,
        "{achieved}"
    );
    // The default damping biases the result by about (λ / σ_min)².
    let damped = servo(ServoConfig::default())
        .step(&HOME, twist, 0.001)
        .unwrap();
    let error = (twist_of(&model(), HOME, damped.dq) - na::Vector6::from(twist)).norm();
    assert!(error < 1e-4 * na::Vector6::from(twist).norm(), "{error}");

    // In the tip frame the same numbers are rotated by the tip's orientation.
    let mut tool =
        servo(ServoConfig { frame: ServoFrame::Tip, damping: 0.0, ..Default::default() });
    let step = tool.step(&HOME, twist, 0.001).unwrap();
    let r = tip(&model(), HOME).rotation;
    let v = na::Vector3::new(twist[0], twist[1], twist[2]);
    let w = na::Vector3::new(twist[3], twist[4], twist[5]);
    let expected = na::Vector6::from_iterator((r * v).iter().chain((r * w).iter()).copied());
    assert!((twist_of(&model(), HOME, step.dq) - expected).amax() < 1e-12);
}

#[test]
fn slows_toward_a_singularity_but_not_away_from_it() {
    let mut q = HOME;
    q[2] = 0.6;
    let c = condition(&model(), q);
    let config = ServoConfig {
        singularity_slowdown: c / 2.0,
        singularity_stop: 2.0 * c,
        ..Default::default()
    };
    let mut servo = servo(config);
    let out = extend(q, 0.05);
    let retract = out.map(|v| -v);

    let slowed = servo.step(&q, out, 0.0).unwrap();
    assert_eq!(slowed.status, ServoStatus::NearSingularity);
    assert!((slowed.scale - 2.0 / 3.0).abs() < 1e-9, "{}", slowed.scale);
    let free = servo.step(&q, retract, 0.0).unwrap();
    assert_eq!(free.status, ServoStatus::Free);
    assert_eq!(free.scale, 1.0);

    // Extending lengthens the reach and worsens the conditioning.
    let ahead = std::array::from_fn(|i| q[i] + slowed.dq[i] * 0.01);
    assert!(shoulder_to_tip(ahead).norm() > shoulder_to_tip(q).norm());
    assert!(condition(&model(), ahead) > c);
}

#[test]
fn halts_at_the_singularity_stop_and_backs_out_at_full_speed() {
    let config = ServoConfig::default();
    let dt = Duration::from_millis(1);
    let mut q = HOME;
    let out = extend(q, 0.1);
    let mut control = servo(config).position_control(move |_, _| (out, false));
    q = run(&mut control, q, 6000, dt);
    let c = condition(&model(), q);
    assert!(
        c > config.singularity_slowdown && c <= config.singularity_stop * 1.001,
        "{c}"
    );
    assert!(q[2] > 0.3, "the elbow passed toward straight: {}", q[2]);
    let before = q;
    q = run(&mut control, q, 100, dt);
    let crawl = q
        .iter()
        .zip(before)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(crawl < 1e-3, "still moving {crawl} rad in 0.1 s");

    let retract = out.map(|v| -v);
    let mut back = servo(config);
    let step = back.step(&q, retract, 0.001).unwrap();
    assert_eq!(step.status, ServoStatus::Free);
    let error = (twist_of(&model(), q, step.dq) - na::Vector6::from(retract)).norm();
    assert!(error < 1e-3 * na::Vector6::from(retract).norm(), "{error}");
}

#[test]
fn never_steps_past_a_position_limit() {
    let dt = Duration::from_millis(2);
    // Pure motion of joint 0, as a twist.
    let push = |q: [f64; 6]| twist_array(q, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]);
    for margin in [0.1, 0.0] {
        let model = model();
        let mut limits = model.joint_limits();
        limits.q_max[0] = HOME[0] + 0.05;
        let config = ServoConfig { joint_limit_margin: margin, ..Default::default() };
        let mut servo = CartesianServo::new(model, limits, config).unwrap();
        let mut q = HOME;
        let mut last = None;
        for _ in 0..500 {
            let step = servo.step(&q, push(q), dt.as_secs_f64()).unwrap();
            q = std::array::from_fn(|i| q[i] + step.dq[i] * dt.as_secs_f64());
            assert!(q[0] <= limits.q_max[0] + 1e-12, "margin {margin}: {}", q[0]);
            last = Some(step);
        }
        let last = last.unwrap();
        assert_eq!(last.status, ServoStatus::JointLimit(0), "margin {margin}");
        assert!(
            limits.q_max[0] - q[0] < 1e-3,
            "margin {margin}: stopped short at {}",
            q[0]
        );
        // Away from the limit: full speed.
        let away = push(q).map(|v| -v);
        let step = servo.step(&q, away, dt.as_secs_f64()).unwrap();
        assert_eq!(step.status, ServoStatus::Free);
    }
}

#[test]
fn caps_joint_rates_and_keeps_the_direction() {
    let mut servo = servo(ServoConfig { damping: 0.0, ..Default::default() });
    let twist = [1.0, -1.0, 1.0, 2.0, -2.0, 2.0];
    let step = servo.step(&HOME, twist, 0.001).unwrap();
    let ServoStatus::VelocityLimit(i) = step.status else {
        panic!("{:?}", step.status);
    };
    assert!((step.dq[i].abs() - 2.5).abs() < 1e-9);
    assert!(step.dq.iter().all(|v| v.abs() <= 2.5 + 1e-9));
    let achieved = twist_of(&model(), HOME, step.dq);
    let commanded = na::Vector6::from(twist);
    assert!((achieved - commanded * step.scale).amax() < 1e-12);
}

#[test]
fn the_proximity_hook_slows_and_halts() {
    let twist = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut slowed =
        servo(ServoConfig::default()).with_proximity(|_| proximity_scale(0.06, 0.1, 0.02));
    let step = slowed.step(&HOME, twist, 0.001).unwrap();
    assert_eq!(step.status, ServoStatus::Proximity);
    assert!((step.scale - 0.5).abs() < 1e-12);
    let mut halted = servo(ServoConfig::default()).with_proximity(|_| f64::NAN);
    assert_eq!(halted.step(&HOME, twist, 0.001).unwrap().dq, [0.0; 6]);
    assert_eq!(proximity_scale(0.2, 0.1, 0.02), 1.0);
    assert_eq!(proximity_scale(0.01, 0.1, 0.02), 0.0);
}

#[test]
fn teleop_drives_the_servo_and_stops_it() {
    let mut axes = InputAxes::default();
    axes.axes[0] = 1.0;
    let stop = std::rc::Rc::new(std::cell::Cell::new(false));
    let flag = stop.clone();
    let mapping = TeleopMapping { max_linear_accel: f64::INFINITY, ..Default::default() };
    let teleop = cartesian_teleop_control(move || Some(axes), mapping, move || flag.get());
    let config = ServoConfig { damping: 0.0, ..Default::default() };
    let mut control = servo(config).velocity_control(teleop);
    let dt = Duration::from_millis(1);
    let std::ops::ControlFlow::Continue((dq, done)) = control(state(HOME), dt) else {
        panic!("ended");
    };
    assert!(!done);
    let t = twist_of(&model(), HOME, dq);
    assert!((t - na::Vector6::new(mapping.linear_speed, 0.0, 0.0, 0.0, 0.0, 0.0)).amax() < 1e-12);

    stop.set(true);
    let std::ops::ControlFlow::Continue((dq, done)) = control(state(HOME), dt) else {
        panic!("ended");
    };
    assert!(done && dq == [0.0; 6]);
}

#[test]
fn ends_the_session_without_a_position_or_with_a_bad_twist() {
    let mut control = servo(ServoConfig::default()).velocity_control(|_, _| ([0.0; 6], false));
    let blind = JointState::from_meas(JointSample::<6>::default());
    assert!(control(blind, Duration::from_millis(1)).is_break());
    let mut control = servo(ServoConfig::default()).position_control(|_, _| ([f64::NAN; 6], false));
    assert!(control(state(HOME), Duration::from_millis(1)).is_break());
}

#[test]
fn rejects_inconsistent_settings() {
    let model = model();
    let good = model.joint_limits();
    let mut slow = good;
    slow.dq_max[3] = 0.0;
    assert!(CartesianServo::new(model.clone(), slow, ServoConfig::default()).is_err());
    let mut inverted = good;
    inverted.q_min[1] = 1.0;
    inverted.q_max[1] = -1.0;
    assert!(CartesianServo::new(model.clone(), inverted, ServoConfig::default()).is_err());
    let swapped = ServoConfig {
        singularity_slowdown: 30.0,
        singularity_stop: 17.0,
        ..Default::default()
    };
    assert!(CartesianServo::new(model, good, swapped).is_err());
}

#[test]
fn at_an_exact_singularity_only_leaving_it_moves() {
    let mut q = HOME;
    q[4] = 0.0; // wrist axes 4 and 6 aligned
    let mut servo = servo(ServoConfig::default());
    let leave = twist_array(q, [0.0, 0.0, 0.0, 0.0, 0.3, 0.0]);
    let step = servo.step(&q, leave, 0.001).unwrap();
    assert!(step.condition > 1e6, "{}", step.condition);
    assert_eq!(step.status, ServoStatus::Free);
    assert!(step.dq[4] > 0.1, "{:?}", step.dq);
    // Spinning joint 6 alone keeps the wrist aligned: halted.
    let spin = twist_array(q, [0.0, 0.0, 0.0, 0.0, 0.0, 0.3]);
    let step = servo.step(&q, spin, 0.001).unwrap();
    assert_eq!(step.status, ServoStatus::SingularityStop);
    assert_eq!(step.dq, [0.0; 6]);
}
