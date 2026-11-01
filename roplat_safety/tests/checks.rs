use std::time::Duration;

use roplat_behavior::{
    ArmState, JointSample, JointState, Joints, SpatialSample, StateView, Twist,
    msgs::{CartesianLimits, JointLimits, Stamp, clock},
};
use roplat_safety::{
    AgeLimit, Axis, CartesianVelocityCheck, CommandCheck, JointPositionCheck, JointTorqueCheck,
    JointVelocityCheck, Quantity, SafetyLatch, SafetyViolation, Watchdog,
};

const MS: Duration = Duration::from_millis(1);

fn limits() -> JointLimits<2> {
    JointLimits {
        q_min: [-1.0; 2],
        q_max: [1.0; 2],
        dq_max: [2.0; 2],
        ddq_max: [100.0; 2],
        tau_max: [10.0; 2],
        ..JointLimits::UNBOUNDED
    }
}

fn unbounded() -> JointLimits<2> {
    JointLimits::UNBOUNDED
}

fn state(q: [f64; 2], dq: [f64; 2]) -> JointState<2> {
    StateView::from_meas(JointSample::EMPTY.with_q(q).with_dq(dq))
}

#[test]
fn non_finite_commands_and_limits_refuse() {
    let mut check = JointPositionCheck::new(limits());
    assert_eq!(
        check.check(&state([0.; 2], [0.; 2]), MS, &[0., f64::NAN]),
        Err(SafetyViolation::NonFinite { quantity: Quantity::Position, axis: Axis::Joint(1) })
    );
    let mut nan_limit = limits();
    nan_limit.q_max[0] = f64::NAN;
    let mut check = JointPositionCheck::new(nan_limit);
    assert!(matches!(
        check.check(&state([0.; 2], [0.; 2]), MS, &[0.; 2]),
        Err(SafetyViolation::OutOfRange { axis: Axis::Joint(0), .. })
    ));
}

#[test]
fn position_rates_follow_the_previous_command() {
    let mut check = JointPositionCheck::new(limits());
    let at_rest = state([0.; 2], [0.; 2]);
    // 1 mm per ms is 1 rad/s; from rest that is 1000 rad/s², above 100.
    assert!(matches!(
        check.check(&at_rest, MS, &[0.001, 0.]),
        Err(SafetyViolation::Exceeded { quantity: Quantity::Acceleration, .. })
    ));
    // 0.05 rad/s from rest is 50 rad/s²; then hold that speed.
    check.check(&at_rest, MS, &[0.00005, 0.]).unwrap();
    // The measurement no longer matters, only the previous command.
    let far = state([0.5, 0.], [0.; 2]);
    check.check(&far, MS, &[0.0001, 0.]).unwrap();
    check.check(&far, MS, &[0.00015, 0.]).unwrap();
}

#[test]
fn refusal_clears_the_reference() {
    let mut check = JointPositionCheck::new(unbounded_but_velocity());
    check
        .check(&state([0.; 2], [0.; 2]), MS, &[0.001, 0.])
        .unwrap();
    assert!(
        check
            .check(&state([0.; 2], [0.; 2]), MS, &[0.5, 0.])
            .is_err()
    );
    // After the refusal the measurement is the reference again.
    check
        .check(&state([0.5, 0.], [0.; 2]), MS, &[0.501, 0.])
        .unwrap();
}

fn unbounded_but_velocity() -> JointLimits<2> {
    JointLimits { dq_max: [2.0; 2], ..unbounded() }
}

#[test]
fn zero_dt_passes_only_an_unchanged_command() {
    let mut check = JointPositionCheck::new(unbounded_but_velocity());
    let s = state([0.2, 0.], [0.; 2]);
    check.check(&s, Duration::ZERO, &[0.2, 0.]).unwrap();
    assert!(matches!(
        check.check(&s, Duration::ZERO, &[0.2001, 0.]),
        Err(SafetyViolation::Exceeded { quantity: Quantity::Velocity, value, .. }) if value.is_infinite()
    ));
}

#[test]
fn missing_measurement_refuses_only_when_a_bound_needs_it() {
    let unobserved = StateView::from_meas(JointSample::<2>::EMPTY);
    let mut check = JointPositionCheck::new(limits());
    assert_eq!(
        check.check(&unobserved, MS, &[0.; 2]),
        Err(SafetyViolation::Unobserved { quantity: Quantity::Position })
    );
    let mut check =
        JointPositionCheck::new(JointLimits { q_min: [-1.; 2], q_max: [1.; 2], ..unbounded() });
    check.check(&unobserved, MS, &[0.5; 2]).unwrap();
    let non_finite = state([f64::NAN, 0.], [0.; 2]);
    let mut check = JointPositionCheck::new(limits());
    assert_eq!(
        check.check(&non_finite, MS, &[0.; 2]),
        Err(SafetyViolation::Unobserved { quantity: Quantity::Position })
    );
}

#[test]
fn commands_cannot_run_away_from_a_blocked_arm() {
    let mut check = JointPositionCheck::new(unbounded_but_velocity())
        .with_tracking_error([0.0025, f64::INFINITY]);
    // The arm stays at zero while each command steps another millimetre.
    let blocked = state([0.; 2], [0.; 2]);
    check.check(&blocked, MS, &[0.001, 0.]).unwrap();
    check.check(&blocked, MS, &[0.002, 0.]).unwrap();
    assert!(matches!(
        check.check(&blocked, MS, &[0.003, 0.]),
        Err(SafetyViolation::Exceeded {
            quantity: Quantity::TrackingError,
            axis: Axis::Joint(0),
            ..
        })
    ));
}

#[test]
fn velocity_may_return_toward_the_range_but_not_leave_it() {
    let mut check =
        JointVelocityCheck::new(JointLimits { ddq_max: [f64::INFINITY; 2], ..limits() });
    let outside = state([1.2, 0.], [0.; 2]);
    check.check(&outside, MS, &[-0.5, 0.]).unwrap();
    check.reset();
    assert!(matches!(
        check.check(&outside, MS, &[0.5, 0.]),
        Err(SafetyViolation::OutOfRange { quantity: Quantity::Position, axis: Axis::Joint(0), .. })
    ));
    assert!(matches!(
        check.check(&state([0.; 2], [0.; 2]), MS, &[0., 2.5]),
        Err(SafetyViolation::Exceeded { quantity: Quantity::Velocity, axis: Axis::Joint(1), .. })
    ));
}

#[test]
fn velocity_change_is_bounded_by_acceleration() {
    let mut check = JointVelocityCheck::new(limits());
    let s = state([0.; 2], [0.; 2]);
    check.check(&s, MS, &[0.1, 0.]).unwrap();
    check.check(&s, MS, &[0.2, 0.]).unwrap();
    assert!(matches!(
        check.check(&s, MS, &[0.35, 0.]),
        Err(SafetyViolation::Exceeded {
            quantity: Quantity::Acceleration,
            axis: Axis::Joint(0),
            ..
        })
    ));
}

#[test]
fn torque_starts_from_an_explicit_value() {
    let s = state([0.; 2], [0.; 2]);
    let mut check = JointTorqueCheck::new(limits()).with_effort_rate([1000.; 2]);
    assert!(check.check(&s, MS, &[3., 0.]).is_err());
    let mut check = JointTorqueCheck::new(limits())
        .with_effort_rate([1000.; 2])
        .starting_at([2.5, 0.]);
    check.check(&s, MS, &[3., 0.]).unwrap();
    check.check(&s, MS, &[3.9, 0.]).unwrap();
    assert!(matches!(
        check.check(&s, MS, &[-11., 0.]),
        Err(SafetyViolation::Exceeded { quantity: Quantity::Effort, .. })
    ));
}

fn arm(twist: Option<[f64; 6]>) -> ArmState<2> {
    let mut flange = SpatialSample::EMPTY;
    flange.set_twist(twist.map(Twist::from));
    ArmState { flange: StateView::from_meas(flange), ..ArmState::default() }
}

#[test]
fn cartesian_checks_both_parts_and_their_change() {
    let limits = CartesianLimits {
        linear_velocity_max: 0.25,
        angular_velocity_max: 1.0,
        linear_acceleration_max: 10.0,
        ..CartesianLimits::UNBOUNDED
    };
    let mut check = CartesianVelocityCheck::<2>::new(limits);
    assert_eq!(
        check.check(
            &arm(Some([0.; 6])),
            MS,
            &[0., 0., 0., 0., f64::INFINITY, 0.]
        ),
        Err(SafetyViolation::NonFinite { quantity: Quantity::Velocity, axis: Axis::Angular })
    );
    assert!(matches!(
        check.check(&arm(Some([0.; 6])), MS, &[0., 0., 0., 0.8, 0.8, 0.]),
        Err(SafetyViolation::Exceeded { quantity: Quantity::Velocity, axis: Axis::Angular, .. })
    ));
    // 0.02 m/s within 1 ms is 20 m/s².
    assert!(matches!(
        check.check(&arm(Some([0.; 6])), MS, &[0.012, 0.016, 0., 0., 0., 0.]),
        Err(SafetyViolation::Exceeded { quantity: Quantity::Acceleration, axis: Axis::Linear, .. })
    ));
    assert_eq!(
        check.check(&arm(None), MS, &[0.; 6]),
        Err(SafetyViolation::Unobserved { quantity: Quantity::Velocity })
    );
    let mut check = CartesianVelocityCheck::<2>::new(limits).starting_at([0.; 6]);
    check
        .check(&arm(None), MS, &[0.005, 0., 0., 0., 0., 0.])
        .unwrap();
}

#[test]
fn age_needs_one_named_clock_and_no_future() {
    let limit = AgeLimit::new(Duration::from_millis(5));
    let now = Stamp::new(clock::SIMULATION, 10_000_000);
    assert_eq!(
        limit.check(&Stamp::new(clock::SIMULATION, 7_000_000), &now),
        Ok(Duration::from_millis(3))
    );
    assert_eq!(
        limit.check(&Stamp::new(clock::SIMULATION, 11_000_000), &now),
        Err(SafetyViolation::FromFuture { ahead: Duration::from_millis(1) })
    );
    assert_eq!(
        limit.check(
            &Stamp::new(clock::UNSPECIFIED, 9_000_000),
            &Stamp::new(clock::UNSPECIFIED, 10_000_000)
        ),
        Err(SafetyViolation::ClockMismatch {
            sample: clock::UNSPECIFIED,
            gate: clock::UNSPECIFIED
        })
    );
    assert_eq!(
        limit.check(&Stamp::new(clock::REALTIME, 9_000_000), &now),
        Err(SafetyViolation::ClockMismatch { sample: clock::REALTIME, gate: clock::SIMULATION })
    );
}

#[test]
fn watchdog_arms_on_the_first_feed_and_disarms_when_it_trips() {
    let at = |ms: i64| Stamp::new(clock::MONOTONIC, ms * 1_000_000);
    let mut watchdog = Watchdog::new(Duration::from_millis(5));
    watchdog.feed(at(100)).unwrap();
    watchdog.feed(at(104)).unwrap();
    assert_eq!(
        watchdog.check(&at(110)),
        Err(SafetyViolation::Overrun {
            gap: Duration::from_millis(6),
            max: Duration::from_millis(5)
        })
    );
    assert!(watchdog.feed(at(110)).is_err());
    watchdog.feed(at(200)).unwrap();
    assert!(matches!(
        watchdog.check(&Stamp::new(clock::REALTIME, 0)),
        Err(SafetyViolation::ClockMismatch { .. })
    ));
}

#[test]
fn latch_keeps_the_first_reason_until_cleared() {
    let latch = SafetyLatch::new();
    let other = latch.clone();
    assert!(latch.check().is_ok());
    assert!(other.trip("operator"));
    assert!(!latch.trip("later"));
    assert_eq!(
        latch.check(),
        Err(SafetyViolation::Tripped { reason: "operator".into() })
    );
    assert_eq!(other.clear().as_deref(), Some("operator"));
    assert!(latch.check().is_ok());
}

struct TwoJoints;

impl Joints<2> for TwoJoints {
    const JOINT_LIMITS: JointLimits<2> =
        JointLimits { dq_max: [3.; 2], ..JointLimits::positions([-2.; 2], [2.; 2]) };
}

#[test]
fn joint_limits_read_the_driver_constants() {
    let limits = TwoJoints::JOINT_LIMITS;
    assert_eq!(
        (limits.q_min, limits.q_max, limits.dq_max),
        ([-2.; 2], [2.; 2], [3.; 2])
    );
    // Bounds left at their default are unbounded.
    assert_eq!(limits.tau_max, [f64::INFINITY; 2]);
}

#[cfg(feature = "roplat")]
#[test]
fn violations_are_found_inside_wrapping_errors() {
    use roplat::{LifecycleError, ParallelError, RoplatError};
    use roplat_behavior::{
        RobotException,
        roplat::{ControlDomainExit, ControlSessionError},
    };

    let violation = SafetyViolation::NoSample;
    let direct = RoplatError::from(violation.clone());
    assert_eq!(SafetyViolation::find(&direct), Some(&violation));
    let parallel = ParallelError::combine(
        RoplatError::Node("sibling".into()),
        violation.clone().into(),
    );
    assert_eq!(SafetyViolation::find(&parallel), Some(&violation));
    let lifecycle = LifecycleError::finish::<()>(
        Err(violation.clone().into()),
        vec![RoplatError::Node("shutdown".into())],
    )
    .unwrap_err();
    assert_eq!(SafetyViolation::find(&lifecycle), Some(&violation));
    let session = RoplatError::Node(Box::new(ControlSessionError {
        domain: Some(ControlDomainExit::Failed(violation.clone().into())),
        device: RobotException::NetworkError("cleanup".into()),
    }));
    assert_eq!(SafetyViolation::find(&session), Some(&violation));
    assert_eq!(SafetyViolation::find(&RoplatError::Node("x".into())), None);
}
