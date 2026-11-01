//! Refusal paths inside real `ControlRhythm` graphs: a refused cycle reaches
//! the device session as `Break`, without a command, and the drive returns
//! the violation.
#![cfg(feature = "roplat")]

mod support;

use std::{sync::Arc, time::Duration};

use roplat::{
    Completion, Execution, Lifecycle, Node, RoplatError, RoplatResult,
    comm::{Publisher, Subscriber, create_triple_buffer},
};
use roplat_behavior::{
    ArmState, CartesianVelocityControl, JointPositionControl, JointSample, JointState,
    JointVelocityControl, StateView, TorqueControl,
    msgs::{
        CartesianLimits, Header, JointLimits, ManualClock, MonotonicClock, Stamp, Stamped, clock,
    },
    roplat::ControlRhythm,
};
use roplat_safety::{
    Axis, CartesianVelocityCheck, CommandGuard, CycleWatchdog, FreshnessGate, JointPositionCheck,
    JointTorqueCheck, JointVelocityCheck, LatchGate, Quantity, SafetyLatch, SafetyViolation, Trip,
};
use support::{DOF, MockArm};

type Joints = [f64; DOF];
type PositionControl = ControlRhythm<MockArm, JointPositionControl<DOF>>;
type PositionGuard = CommandGuard<JointPositionCheck<DOF>>;

fn limits() -> JointLimits<DOF> {
    JointLimits {
        q_min: [-1.0; DOF],
        q_max: [1.0; DOF],
        dq_max: [2.0; DOF],
        ddq_max: [f64::INFINITY; DOF],
        tau_max: [10.0; DOF],
        ..JointLimits::UNBOUNDED
    }
}

fn violation(outcome: &Execution<()>) -> &SafetyViolation {
    let error = outcome.as_ref().expect_err("the drive should fail");
    SafetyViolation::find(error).unwrap_or_else(|| panic!("not a safety violation: {error:?}"))
}

/// Adds `step` to the measured position of joint 0 each cycle; done after
/// `cycles` commands.
struct Ramp {
    step: f64,
    cycles: usize,
    sent: usize,
}

impl Ramp {
    fn new(step: f64, cycles: usize) -> Self {
        Self { step, cycles, sent: 0 }
    }
}

impl Lifecycle for Ramp {
    type Error = RoplatError;
}

impl Node for Ramp {
    type Resources = ();
    type Input = (JointState<DOF>, Duration);
    type Output = (Joints, bool);

    async fn process(&mut self, (state, _): Self::Input, _: ()) -> Self::Output {
        let mut q = state.meas.q().unwrap();
        q[0] += self.step;
        self.sent += 1;
        (q, self.sent == self.cycles)
    }
}

#[roplat::system]
async fn guarded_position(
    mut control: PositionControl,
    mut ramp: Ramp,
    mut guard: PositionGuard,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm, PositionControl, PositionGuard)> {
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            sample >> ramp;
            sample >> guard.input.0;
            ramp >> guard.input.1;
            guard?
        };
    Ok((control.outcome, arm, control, guard))
}

fn position_guard() -> PositionGuard {
    CommandGuard::new(JointPositionCheck::new(limits()))
}

#[tokio::test]
async fn accepted_position_commands_all_reach_the_device() {
    let (outcome, arm, ..) = guarded_position(
        PositionControl::new(),
        Ramp::new(0.001, 5),
        position_guard(),
        MockArm::at([0.; DOF]),
    )
    .await
    .unwrap();
    assert!(matches!(outcome, Ok(Completion::Completed(()))));
    assert_eq!(arm.commands().len(), 5);
    assert_eq!(arm.sessions(), (1, 0, 1));
}

#[tokio::test]
async fn position_beyond_range_breaks_before_it_is_sent() {
    let start = [0.9955, 0.];
    let (outcome, arm, ..) = guarded_position(
        PositionControl::new(),
        Ramp::new(0.001, 100),
        position_guard(),
        MockArm::at(start),
    )
    .await
    .unwrap();
    let SafetyViolation::OutOfRange { quantity, axis, value, max, .. } = *violation(&outcome)
    else {
        panic!("{outcome:?}")
    };
    assert_eq!(
        (quantity, axis, max),
        (Quantity::Position, Axis::Joint(0), 1.0)
    );
    assert!(value > 1.0);
    let commands = arm.commands();
    assert_eq!(commands.len(), 4);
    assert!(commands.iter().all(|q| q[0] <= 1.0));
    assert_eq!(arm.sessions(), (1, 1, 1));
}

#[tokio::test]
async fn position_step_faster_than_the_velocity_limit_breaks() {
    // 0.01 rad in 1 ms is 10 rad/s against a 2 rad/s limit.
    let (outcome, arm, ..) = guarded_position(
        PositionControl::new(),
        Ramp::new(0.01, 5),
        position_guard(),
        MockArm::at([0.; DOF]),
    )
    .await
    .unwrap();
    let SafetyViolation::Exceeded { quantity, axis, value, .. } = *violation(&outcome) else {
        panic!("{outcome:?}")
    };
    assert_eq!((quantity, axis), (Quantity::Velocity, Axis::Joint(0)));
    assert!((value - 10.0).abs() < 1e-9);
    assert!(arm.commands().is_empty());
    assert_eq!(arm.sessions(), (1, 1, 1));
}

#[tokio::test]
async fn done_ends_the_reference_so_the_next_session_starts_from_measurement() {
    let (outcome, mut arm, control, guard) = guarded_position(
        PositionControl::new(),
        Ramp::new(0.001, 3),
        position_guard(),
        MockArm::at([0.; DOF]),
    )
    .await
    .unwrap();
    assert!(outcome.is_ok());
    // Something else moved the arm between the sessions.
    arm.q = [0.5, 0.];
    let (outcome, arm, ..) = guarded_position(control, Ramp::new(0.001, 3), guard, arm)
        .await
        .unwrap();
    assert!(matches!(outcome, Ok(Completion::Completed(()))));
    assert_eq!(arm.commands().len(), 6);
    assert!((arm.commands()[3][0] - 0.501).abs() < 1e-12);
}

type VelocityControl = ControlRhythm<MockArm, JointVelocityControl<DOF>>;
type VelocityGuard = CommandGuard<JointVelocityCheck<DOF>>;

/// Commands one constant joint velocity, never done.
struct Cruise(Joints);

impl Lifecycle for Cruise {
    type Error = RoplatError;
}

impl Node for Cruise {
    type Resources = ();
    type Input = (JointState<DOF>, Duration);
    type Output = (Joints, bool);

    async fn process(&mut self, _: Self::Input, _: ()) -> Self::Output {
        (self.0, false)
    }
}

#[roplat::system]
async fn guarded_velocity(
    mut cruise: Cruise,
    mut guard: VelocityGuard,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm)> {
    let mut control = VelocityControl::new();
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            sample >> cruise;
            sample >> guard.input.0;
            cruise >> guard.input.1;
            guard?
        };
    Ok((control.outcome, arm))
}

#[tokio::test]
async fn velocity_that_cannot_brake_before_the_range_end_breaks() {
    let limits = JointLimits { ddq_max: [100.0; DOF], ..limits() };
    let mut arm = MockArm::at([0.9005, 0.]);
    arm.dq = [1.0, 0.];
    let guard = CommandGuard::new(JointVelocityCheck::new(limits));
    let (outcome, arm) = guarded_velocity(Cruise([1.0, 0.]), guard, arm)
        .await
        .unwrap();
    // One 1 ms cycle at 1 rad/s plus 5 mm of braking: refused once the joint
    // is past 0.994.
    let SafetyViolation::OutOfRange { quantity, axis, value, .. } = *violation(&outcome) else {
        panic!("{outcome:?}")
    };
    assert_eq!((quantity, axis), (Quantity::Position, Axis::Joint(0)));
    assert!(value > 1.0);
    assert_eq!(arm.commands().len(), 94);
    assert!(arm.q[0] < 1.0);
    assert_eq!(arm.sessions(), (1, 1, 1));
}

#[tokio::test]
async fn a_fixed_period_replaces_a_zero_first_dt() {
    // Velocity noise against the zero `dt` Franka reports before its first cycle.
    let limits = JointLimits { ddq_max: [100.0; DOF], ..limits() };
    let state = StateView::from_meas(JointSample::EMPTY.with_q([0.; DOF]).with_dq([1e-4, 0.]));
    let input = ((state, Duration::ZERO), ([0.; DOF], false));
    let mut guard = CommandGuard::new(JointVelocityCheck::new(limits));
    assert!(guard.process(input, ()).await.is_err());
    let mut guard = CommandGuard::new(JointVelocityCheck::new(limits)).with_period(support::PERIOD);
    assert_eq!(guard.process(input, ()).await, Ok(([0.; DOF], false)));
}

type TorqueRhythm = ControlRhythm<MockArm, TorqueControl<DOF>>;
type TorqueGuard = CommandGuard<JointTorqueCheck<DOF>>;

/// Adds `step` N·m to joint 0 each cycle.
struct TorqueRamp {
    step: f64,
    tau: Joints,
}

impl Lifecycle for TorqueRamp {
    type Error = RoplatError;
}

impl Node for TorqueRamp {
    type Resources = ();
    type Input = (JointState<DOF>, Duration);
    type Output = (Joints, bool);

    async fn process(&mut self, _: Self::Input, _: ()) -> Self::Output {
        self.tau[0] += self.step;
        (self.tau, false)
    }
}

#[roplat::system]
async fn guarded_torque(
    mut ramp: TorqueRamp,
    mut guard: TorqueGuard,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm)> {
    let mut control = TorqueRhythm::new();
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            sample >> ramp;
            sample >> guard.input.0;
            ramp >> guard.input.1;
            guard?
        };
    Ok((control.outcome, arm))
}

fn torque_guard() -> TorqueGuard {
    CommandGuard::new(JointTorqueCheck::new(limits()).with_effort_rate([1000.0; DOF]))
}

#[tokio::test]
async fn torque_above_its_bound_breaks() {
    let ramp = TorqueRamp { step: 0.5, tau: [0.; DOF] };
    let (outcome, arm) = guarded_torque(ramp, torque_guard(), MockArm::at([0.; DOF]))
        .await
        .unwrap();
    let SafetyViolation::Exceeded { quantity, value, max, .. } = *violation(&outcome) else {
        panic!("{outcome:?}")
    };
    assert_eq!((quantity, value, max), (Quantity::Effort, 10.5, 10.0));
    assert_eq!(arm.commands().len(), 20);
    assert_eq!(arm.sessions(), (1, 1, 1));
}

#[tokio::test]
async fn torque_jump_from_the_start_breaks() {
    // 5 N·m in one millisecond against 1000 N·m/s.
    let ramp = TorqueRamp { step: 5.0, tau: [0.; DOF] };
    let (outcome, arm) = guarded_torque(ramp, torque_guard(), MockArm::at([0.; DOF]))
        .await
        .unwrap();
    assert!(matches!(
        violation(&outcome),
        SafetyViolation::Exceeded { quantity: Quantity::EffortRate, axis: Axis::Joint(0), .. }
    ));
    assert!(arm.commands().is_empty());
}

type CartesianRhythm = ControlRhythm<MockArm, CartesianVelocityControl<DOF>>;
type CartesianGuard = CommandGuard<CartesianVelocityCheck<DOF>>;

/// Speeds up along x by `step` m/s each cycle.
struct Accelerate {
    step: f64,
    vx: f64,
}

impl Lifecycle for Accelerate {
    type Error = RoplatError;
}

impl Node for Accelerate {
    type Resources = ();
    type Input = (ArmState<DOF>, Duration);
    type Output = ([f64; 6], bool);

    async fn process(&mut self, _: Self::Input, _: ()) -> Self::Output {
        self.vx += self.step;
        ([self.vx, 0., 0., 0., 0., 0.], false)
    }
}

/// `ArmState` is not `Copy`; the graph copies it explicitly to give both the
/// controller and the guard their own.
struct Tee;

impl Lifecycle for Tee {
    type Error = RoplatError;
}

impl Node for Tee {
    type Resources = ();
    type Input = (ArmState<DOF>, Duration);
    type Output = (Self::Input, Self::Input);

    async fn process(&mut self, sample: Self::Input, _: ()) -> Self::Output {
        (sample.clone(), sample)
    }
}

#[roplat::system]
async fn guarded_cartesian(
    mut accelerate: Accelerate,
    mut guard: CartesianGuard,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm)> {
    let mut control = CartesianRhythm::new();
    let mut tee = Tee;
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            sample >> tee;
            tee.output.0 >> accelerate;
            tee.output.1 >> guard.input.0;
            accelerate >> guard.input.1;
            guard?
        };
    Ok((control.outcome, arm))
}

#[tokio::test]
async fn cartesian_speed_above_its_bound_breaks() {
    let limits = CartesianLimits {
        linear_velocity_max: 0.0505,
        linear_acceleration_max: 20.0,
        ..CartesianLimits::UNBOUNDED
    };
    let guard = CommandGuard::new(CartesianVelocityCheck::new(limits));
    let accelerate = Accelerate { step: 0.01, vx: 0. };
    let (outcome, arm) = guarded_cartesian(accelerate, guard, MockArm::at([0.; DOF]))
        .await
        .unwrap();
    assert!(matches!(
        violation(&outcome),
        SafetyViolation::Exceeded { quantity: Quantity::Velocity, axis: Axis::Linear, .. }
    ));
    assert_eq!(arm.commands().len(), 5);
    assert_eq!(arm.sessions(), (1, 1, 1));
}

/// Reads the latest target another domain published.
struct Target(Subscriber<Stamped<Joints>>);

impl Lifecycle for Target {
    type Error = RoplatError;
}

impl Node for Target {
    type Resources = ();
    type Input = ();
    type Output = Option<Stamped<Joints>>;

    async fn process(&mut self, (): (), _: ()) -> Self::Output {
        self.0.latest().copied()
    }
}

/// Commands the target directly.
struct Follow;

impl Lifecycle for Follow {
    type Error = RoplatError;
}

impl Node for Follow {
    type Resources = ();
    type Input = ((JointState<DOF>, Duration), Joints);
    type Output = (Joints, bool);

    async fn process(&mut self, (_, target): Self::Input, _: ()) -> Self::Output {
        (target, false)
    }
}

type TargetGate = FreshnessGate<Option<Stamped<Joints>>, Arc<ManualClock>>;

#[roplat::system]
async fn fresh_target(
    mut target: Target,
    mut fresh: TargetGate,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm)> {
    let mut control = PositionControl::new();
    let mut follow = Follow;
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            target >> fresh;
            sample >> follow.input.0;
            fresh? >> follow.input.1;
            follow
        };
    Ok((control.outcome, arm))
}

fn target_channel() -> (Publisher<Stamped<Joints>>, Target) {
    let (publisher, mut subscribers) = create_triple_buffer(1);
    (publisher, Target(subscribers.remove(0)))
}

fn stamped(stamp: Stamp, q: Joints) -> Stamped<Joints> {
    Stamped::new(Header::new(stamp, Default::default(), 0), q)
}

#[tokio::test]
async fn target_older_than_its_age_limit_breaks() {
    let arm = MockArm::at([0.; DOF]);
    let (mut publisher, target) = target_channel();
    publisher.publish(stamped(arm.now(), [0.1, 0.]));
    let gate = FreshnessGate::new(arm.clock.clone(), Duration::from_millis(5));
    let (outcome, arm) = fresh_target(target, gate, arm).await.unwrap();
    // Cycle k observes the arm k + 1 ms after the target was stamped.
    assert_eq!(
        *violation(&outcome),
        SafetyViolation::Stale {
            age: Duration::from_millis(6),
            max_age: Duration::from_millis(5)
        }
    );
    assert_eq!(arm.commands().len(), 5);
    assert_eq!(arm.sessions(), (1, 1, 1));
}

#[tokio::test]
async fn target_on_another_clock_breaks_at_once() {
    let arm = MockArm::at([0.; DOF]);
    let (mut publisher, target) = target_channel();
    publisher.publish(stamped(
        roplat_behavior::msgs::Clock::now(&MonotonicClock),
        [0.1, 0.],
    ));
    let gate = FreshnessGate::new(arm.clock.clone(), Duration::from_secs(1));
    let (outcome, arm) = fresh_target(target, gate, arm).await.unwrap();
    assert_eq!(
        *violation(&outcome),
        SafetyViolation::ClockMismatch { sample: clock::MONOTONIC, gate: clock::SIMULATION }
    );
    assert!(arm.commands().is_empty());
    assert_eq!(arm.sessions(), (1, 1, 1));
}

#[tokio::test]
async fn no_target_yet_breaks_at_once() {
    let arm = MockArm::at([0.; DOF]);
    let (_publisher, target) = target_channel();
    let gate = FreshnessGate::new(arm.clock.clone(), Duration::from_secs(1));
    let (outcome, arm) = fresh_target(target, gate, arm).await.unwrap();
    assert_eq!(*violation(&outcome), SafetyViolation::NoSample);
    assert!(arm.commands().is_empty());
}

/// Holds the measured position.
struct Hold;

impl Lifecycle for Hold {
    type Error = RoplatError;
}

impl Node for Hold {
    type Resources = ();
    type Input = (JointState<DOF>, Duration);
    type Output = (Joints, bool);

    async fn process(&mut self, (state, _): Self::Input, _: ()) -> Self::Output {
        (state.meas.q().unwrap(), false)
    }
}

#[roplat::system]
async fn watched_hold(
    mut watchdog: CycleWatchdog<Arc<ManualClock>>,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm)> {
    let mut control = PositionControl::new();
    let mut hold = Hold;
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            sample.1 >> watchdog;
            watchdog?;
            sample >> hold;
            hold
        };
    Ok((control.outcome, arm))
}

#[tokio::test]
async fn late_cycle_breaks_although_its_command_was_computed() {
    let mut arm = MockArm::at([0.; DOF]);
    arm.stall = Some((3, Duration::from_millis(10)));
    let watchdog = CycleWatchdog::new(arm.clock.clone(), Duration::from_millis(5));
    let (outcome, arm) = watched_hold(watchdog, arm).await.unwrap();
    assert_eq!(
        *violation(&outcome),
        SafetyViolation::Overrun {
            gap: Duration::from_millis(11),
            max: Duration::from_millis(5)
        }
    );
    assert_eq!(arm.commands().len(), 3);
    assert_eq!(arm.sessions(), (1, 1, 1));
}

#[roplat::system]
async fn tripping_position(
    mut ramp: Ramp,
    mut guard: PositionGuard,
    mut trip: Trip<(Joints, bool)>,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm)> {
    let mut control = PositionControl::new();
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            sample >> ramp;
            sample >> guard.input.0;
            ramp >> guard.input.1;
            guard >> trip;
            trip?
        };
    Ok((control.outcome, arm))
}

#[roplat::system]
async fn latched_position(
    mut ramp: Ramp,
    mut latch: LatchGate,
    mut arm: MockArm,
) -> RoplatResult<(Execution<()>, MockArm)> {
    let mut control = PositionControl::new();
    &mut arm >> control.resources.0;
    control
        >> |sample| {
            latch?;
            sample >> ramp;
            ramp
        };
    Ok((control.outcome, arm))
}

#[tokio::test]
async fn a_refusal_in_one_domain_stops_another_until_cleared() {
    let latch = SafetyLatch::new();
    let (outcome, _) = tripping_position(
        Ramp::new(0.01, 5),
        position_guard(),
        Trip::new(latch.clone()),
        MockArm::at([0.; DOF]),
    )
    .await
    .unwrap();
    let reason = violation(&outcome).to_string();
    assert_eq!(latch.tripped().as_deref(), Some(reason.as_str()));

    let (outcome, other) = latched_position(
        Ramp::new(0.001, 3),
        LatchGate::new(latch.clone()),
        MockArm::at([0.; DOF]),
    )
    .await
    .unwrap();
    assert_eq!(
        *violation(&outcome),
        SafetyViolation::Tripped { reason: reason.clone() }
    );
    assert!(other.commands().is_empty());
    assert_eq!(other.sessions(), (1, 1, 1));

    assert_eq!(latch.clear(), Some(reason));
    let (outcome, other) = latched_position(Ramp::new(0.001, 3), LatchGate::new(latch), other)
        .await
        .unwrap();
    assert!(matches!(outcome, Ok(Completion::Completed(()))));
    assert_eq!(other.commands().len(), 3);
}
