#![cfg(feature = "roplat")]
//! Resource ownership and business errors are independent in an actual System.
use roplat::{Node, RoplatResult};
use roplat_behavior::{MoveTo, Robot, RobotException, RobotResult, Space, roplat::MotionNode};

#[derive(Default)]
struct Arm {
    attempts: Vec<u32>,
}
struct Position;
impl Robot for Arm {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "motion-mock".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}
impl Space for Position {
    type Value = u32;
}
impl MoveTo<Position> for Arm {
    fn move_to(&mut self, target: u32) -> RobotResult<()> {
        self.attempts.push(target);
        if target == 0 {
            Err(RobotException::CommandException("rejected target".into()))
        } else {
            Ok(())
        }
    }
}

#[roplat::system]
async fn attempt_and_reuse(mut arm: Arm) -> RoplatResult<(Arm, RobotResult<()>, RobotResult<()>)> {
    let mut first = MotionNode::<Arm, Position>::new();
    let mut second = MotionNode::<Arm, Position>::new();
    let rejected = 0_u32;
    let accepted = 7_u32;
    &mut arm >> first.resources.0;
    &rejected >> first;
    &mut arm >> second.resources.0;
    &accepted >> second;
    // A node's business error is ordinary data; `second` still runs after it.
    first >> second.trigger;
    Ok((arm, first.output, second.output))
}

#[roplat::system]
async fn propagate_error(arm: &mut Arm) -> RoplatResult<()> {
    let mut motion = MotionNode::<Arm, Position>::new();
    let rejected = 0_u32;
    &mut arm >> motion.resources.0;
    &rejected >> motion;
    motion.output?;
    Ok(())
}

#[tokio::test]
async fn business_failure_preserves_resource_for_the_next_node() {
    let (arm, failed, succeeded) = attempt_and_reuse(Arm::default()).await.unwrap();
    assert!(
        matches!(failed, Err(RobotException::CommandException(message)) if message == "rejected target")
    );
    assert!(succeeded.is_ok());
    assert_eq!(arm.attempts, [0, 7]);
}

#[tokio::test]
async fn promoted_failure_releases_the_external_resource_borrow() {
    let mut arm = Arm::default();
    assert!(propagate_error(&mut arm).await.is_err());
    let mut motion = MotionNode::<Arm, Position>::new();
    motion.process(9, (&mut arm,)).await.unwrap();
    assert_eq!(arm.attempts, [0, 9]);
}
