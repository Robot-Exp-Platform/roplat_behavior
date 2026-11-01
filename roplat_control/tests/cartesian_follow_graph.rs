//! The Cartesian follower as a node in a real `ControlRhythm` graph: it drives
//! a tool to the goal, and a refusal reaches the session as `Break` and the
//! drive as the tracking error.
#![cfg(feature = "roplat")]

use std::{ops::ControlFlow, sync::Arc, time::Duration};

use nalgebra as na;
use roplat::{Completion, Execution, RoplatResult};
use roplat_behavior::{
    ArmState, CartesianPoseControl, ControlStep, ControlWith, Robot, RobotResult,
    msgs::{ManualClock, Pose, SpatialSample},
    roplat::ControlRhythm,
};
use roplat_control::{CartesianTrackingError, CartesianTrajectoryFollower, cartesian_line};

const PERIOD: Duration = Duration::from_millis(1);

/// A tool that closes `gain` of the remaining way to the commanded pose each
/// cycle. Its clock steps one period before every observation, as a
/// simulator's does.
struct Tool {
    pose: na::Isometry3<f64>,
    gain: f64,
    clock: Arc<ManualClock>,
    sent: usize,
    breaks: usize,
}

impl Tool {
    fn new(gain: f64) -> Self {
        Self {
            pose: na::Isometry3::translation(0.4, 0.0, 0.3),
            gain,
            clock: Arc::new(ManualClock::simulation()),
            sent: 0,
            breaks: 0,
        }
    }
}

impl Robot for Tool {
    type State = ();
    const CONTROL_PERIOD: Duration = PERIOD;
    fn version() -> String {
        "tool".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl ControlWith<CartesianPoseControl<1>> for Tool {
    fn hold_command(state: &ArmState<1>) -> Pose {
        state.flange.meas.pose().unwrap_or(Pose::IDENTITY)
    }
    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<1>, Duration) -> ControlStep<Pose>,
    {
        for _ in 0..10_000 {
            self.clock.advance(PERIOD);
            let mut state = ArmState::<1>::default();
            state.flange.meas = SpatialSample::default().with_pose(Pose::from_isometry(self.pose));
            match closure(state, PERIOD) {
                ControlFlow::Continue((pose, done)) => {
                    self.sent += 1;
                    let target: na::Isometry3<f64> = pose.into();
                    let position = self
                        .pose
                        .translation
                        .vector
                        .lerp(&target.translation.vector, self.gain);
                    let rotation = self.pose.rotation.slerp(&target.rotation, self.gain);
                    self.pose = na::Isometry3::from_parts(position.into(), rotation);
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

type Control = ControlRhythm<Tool, CartesianPoseControl<1>>;
type Follower = CartesianTrajectoryFollower<Arc<ManualClock>, 1>;

#[roplat::system]
async fn follow(
    mut control: Control,
    mut follower: Follower,
    mut tool: Tool,
) -> RoplatResult<(Execution<()>, Tool, Control, Follower)> {
    &mut tool >> control.resources.0;
    control
        >> |sample| {
            sample >> follower;
            follower?
        };
    Ok((control.outcome, tool, control, follower))
}

/// 10 cm down with a 0.3 rad turn in 200 ms, on the tool's clock.
fn goal(tool: &Tool) -> na::Isometry3<f64> {
    na::Translation3::new(0.0, 0.0, -0.1)
        * tool.pose
        * na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3)
}

fn follower(tool: &Tool) -> Follower {
    let waypoints = cartesian_line(&tool.pose, &goal(tool), Duration::from_millis(200), PERIOD);
    CartesianTrajectoryFollower::new(waypoints, tool.clock.clone()).unwrap()
}

#[tokio::test]
async fn the_graph_follows_the_line_to_its_goal() {
    let tool = Tool::new(0.5);
    let goal = goal(&tool);
    let follower = follower(&tool)
        .with_path_tolerance(0.01, 0.05)
        .with_goal_tolerance(1e-4, 1e-3, Duration::from_millis(50));
    let (outcome, tool, ..) = follow(Control::new(), follower, tool).await.unwrap();
    assert!(
        matches!(outcome, Ok(Completion::Completed(()))),
        "{outcome:?}"
    );
    assert_eq!(tool.breaks, 0);
    assert!((tool.pose.translation.vector - goal.translation.vector).norm() <= 1e-4);
    // 200 cycles along the line, then a few more to settle.
    assert!((201..=220).contains(&tool.sent), "{}", tool.sent);
}

#[tokio::test]
async fn a_stuck_tool_breaks_the_session_with_the_tracking_error() {
    let tool = Tool::new(0.0);
    let follower = follower(&tool).with_path_tolerance(0.01, 0.05);
    let (outcome, tool, ..) = follow(Control::new(), follower, tool).await.unwrap();
    let error = outcome.as_ref().expect_err("the drive should fail");
    let Some(CartesianTrackingError::PathTolerance { at, .. }) =
        CartesianTrackingError::find(error)
    else {
        panic!("not a Cartesian tracking error: {error:?}")
    };
    assert!(*at < Duration::from_millis(200), "{at:?}");
    assert_eq!(tool.breaks, 1);
}
