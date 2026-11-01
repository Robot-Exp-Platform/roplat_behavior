//! The follower as a node in a real `ControlRhythm` graph: it drives a device
//! session to the goal, and a refusal reaches the session as `Break` and the
//! drive as the tracking error.
#![cfg(feature = "roplat")]

use std::{ops::ControlFlow, sync::Arc, time::Duration};

use roplat::{Completion, Execution, Lifecycle, Node, RoplatError, RoplatResult};
use roplat_behavior::{
    ControlStep, ControlWith, JointPositionControl, JointSample, JointState, Robot, RobotResult,
    StateView,
    msgs::{JointTrajectoryPoint, ManualClock},
    roplat::ControlRhythm,
};
use roplat_trajectory::{JointTrajectoryFollower, TrackingError};

const PERIOD: Duration = Duration::from_millis(1);
const GOAL: [f64; 2] = [0.5, -0.3];

/// A two-joint position servo that closes `gain` of the remaining distance
/// each cycle. Its clock steps one period before every observation, as a
/// simulator's does.
struct Servo {
    q: [f64; 2],
    gain: f64,
    clock: Arc<ManualClock>,
    sent: usize,
    breaks: usize,
}

impl Servo {
    fn new(gain: f64) -> Self {
        Self {
            q: [0.; 2],
            gain,
            clock: Arc::new(ManualClock::simulation()),
            sent: 0,
            breaks: 0,
        }
    }
}

impl Robot for Servo {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "servo".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl ControlWith<JointPositionControl<2>> for Servo {
    fn hold_command(state: &JointState<2>) -> [f64; 2] {
        state.meas.q().unwrap_or_default()
    }
    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<2>, Duration) -> ControlStep<[f64; 2]>,
    {
        for _ in 0..10_000 {
            self.clock.advance(PERIOD);
            let state = StateView::from_meas(JointSample::EMPTY.with_q(self.q));
            match closure(state, PERIOD) {
                ControlFlow::Continue((q, done)) => {
                    self.sent += 1;
                    for (now, target) in self.q.iter_mut().zip(q) {
                        *now += self.gain * (target - *now);
                    }
                    if done {
                        return Ok(());
                    }
                }
                ControlFlow::Break(_) => {
                    self.breaks += 1;
                    return Ok(());
                }
            }
        }
        panic!("the session exceeded its cycle budget")
    }
}

type Control = ControlRhythm<Servo, JointPositionControl<2>>;
type Follower = JointTrajectoryFollower<Arc<ManualClock>, 2>;

#[roplat::system]
async fn follow(
    mut control: Control,
    mut follower: Follower,
    mut servo: Servo,
) -> RoplatResult<(Execution<()>, Servo, Control, Follower)> {
    &mut servo >> control.resources.0;
    control
        >> |sample| {
            sample >> follower;
            follower?
        };
    Ok((control.outcome, servo, control, follower))
}

/// From rest at the origin to rest at `GOAL` in 200 ms, on the servo's clock.
fn follower(servo: &Servo) -> Follower {
    let point = |ms: i64, q| JointTrajectoryPoint {
        time_from_start: ms * 1_000_000,
        q,
        dq: [0.; 2],
        ddq: [0.; 2],
    };
    JointTrajectoryFollower::new(
        vec![point(0, [0.; 2]), point(200, GOAL)],
        servo.clock.clone(),
    )
    .unwrap()
}

#[tokio::test]
async fn the_graph_follows_the_trajectory_to_its_goal() {
    let servo = Servo::new(0.5);
    let follower = follower(&servo)
        .with_path_tolerance([0.02; 2])
        .with_goal_tolerance([1e-4; 2], Duration::from_millis(50));
    let (outcome, servo, ..) = follow(Control::new(), follower, servo).await.unwrap();
    assert!(
        matches!(outcome, Ok(Completion::Completed(()))),
        "{outcome:?}"
    );
    assert_eq!(servo.breaks, 0);
    assert!(
        servo
            .q
            .iter()
            .zip(GOAL)
            .all(|(q, goal)| (q - goal).abs() <= 1e-4)
    );
    // 200 cycles along the trajectory, then a few more to settle.
    assert!((201..=215).contains(&servo.sent), "{}", servo.sent);
}

#[tokio::test]
async fn a_stuck_arm_breaks_the_session_with_the_tracking_error() {
    let servo = Servo::new(0.);
    let follower = follower(&servo).with_path_tolerance([0.05; 2]);
    let (outcome, servo, ..) = follow(Control::new(), follower, servo).await.unwrap();
    let error = outcome.as_ref().expect_err("the drive should fail");
    let Some(TrackingError::PathTolerance { joint, error, tolerance, at }) =
        TrackingError::find(error)
    else {
        panic!("not a tracking error: {error:?}")
    };
    assert_eq!((*joint, *tolerance), (0, 0.05));
    assert!(
        *error > 0.05 && *at < Duration::from_millis(200),
        "{error} at {at:?}"
    );
    assert_eq!(servo.breaks, 1);
    assert!(servo.q == [0.; 2]);
}

/// Follows the first plan, then hands the follower a second one after `at`
/// cycles, as a planner reacting mid-motion would.
struct Replanning {
    follower: Follower,
    next: Option<Vec<JointTrajectoryPoint<2>>>,
    at: usize,
    cycles: usize,
}

impl Lifecycle for Replanning {
    type Error = RoplatError;
}

impl Node for Replanning {
    type Resources = ();
    type Input = (JointState<2>, Duration);
    type Output = Result<([f64; 2], bool), TrackingError>;

    async fn process(&mut self, (state, _): Self::Input, _: ()) -> Self::Output {
        self.cycles += 1;
        if self.cycles == self.at
            && let Some(plan) = self.next.take()
        {
            self.follower.replace(plan)?;
        }
        self.follower.step(&state)
    }
}

#[roplat::system]
async fn replan(
    mut control: Control,
    mut planner: Replanning,
    mut servo: Servo,
) -> RoplatResult<(Execution<()>, Servo, Control, Replanning)> {
    &mut servo >> control.resources.0;
    control
        >> |sample| {
            sample >> planner;
            planner?
        };
    Ok((control.outcome, servo, control, planner))
}

#[tokio::test]
async fn a_new_plan_mid_motion_takes_over_without_a_jump() {
    const OTHER: [f64; 2] = [-0.2, 0.4];
    let servo = Servo::new(0.5);
    // Halfway through the first plan the arm moves fastest; a jump to the new
    // plan's first point there would break the path tolerance at once.
    let planner = Replanning {
        follower: follower(&servo)
            .with_path_tolerance([0.03; 2])
            .with_goal_tolerance([1e-4; 2], Duration::from_millis(50)),
        next: Some(vec![JointTrajectoryPoint {
            time_from_start: 150_000_000,
            q: OTHER,
            dq: [0.; 2],
            ddq: [0.; 2],
        }]),
        at: 100,
        cycles: 0,
    };
    let (outcome, servo, ..) = replan(Control::new(), planner, servo).await.unwrap();
    assert!(
        matches!(outcome, Ok(Completion::Completed(()))),
        "{outcome:?}"
    );
    assert_eq!(servo.breaks, 0);
    assert!(
        servo
            .q
            .iter()
            .zip(OTHER)
            .all(|(q, goal)| (q - goal).abs() <= 1e-4),
        "{:?}",
        servo.q
    );
    // 99 cycles on the first plan, 150 on the second, then a few to settle.
    assert!((250..=265).contains(&servo.sent), "{}", servo.sent);
}
