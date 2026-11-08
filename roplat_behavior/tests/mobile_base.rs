//! A mobile base that only takes goals is a `MobileBase`; one that also runs
//! velocity sessions is a `VelocityControlledBase` without saying so.

use std::{ops::ControlFlow, time::Duration};

use roplat_behavior::{
    BasePoseSpace, BaseState, BaseVelocityControl, ControlExit, ControlStep, ControlWith,
    MobileBase, MoveTo, Pose, Robot, RobotResult, VelocityControlledBase,
};

/// A base driven only by missions to a pose, as a MiR over its REST API.
struct Mission {
    goal: Option<Pose>,
}

impl Robot for Mission {
    type State = ();
    const CONTROL_PERIOD: Duration = Duration::from_millis(100);

    fn version() -> String {
        String::new()
    }

    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl MoveTo<BasePoseSpace> for Mission {
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        self.goal = Some(target);
        Ok(())
    }
}

impl MobileBase for Mission {
    fn base_state(&mut self) -> RobotResult<BaseState> {
        Ok(BaseState::default())
    }
}

/// A base that also takes a velocity every cycle.
struct Wheeled {
    cycles: usize,
}

impl Robot for Wheeled {
    type State = ();
    const CONTROL_PERIOD: Duration = Duration::from_millis(20);

    fn version() -> String {
        String::new()
    }

    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl MoveTo<BasePoseSpace> for Wheeled {
    fn move_to(&mut self, _: Pose) -> RobotResult<()> {
        Ok(())
    }
}

impl MobileBase for Wheeled {
    fn base_state(&mut self) -> RobotResult<BaseState> {
        Ok(BaseState::default())
    }
}

impl ControlWith<BaseVelocityControl> for Wheeled {
    fn hold_command(_: &BaseState) -> [f64; 6] {
        [0.0; 6]
    }

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(BaseState, Duration) -> ControlStep<[f64; 6]>,
    {
        loop {
            self.cycles += 1;
            match closure(BaseState::default(), Self::CONTROL_PERIOD) {
                ControlFlow::Continue((_, false)) => {}
                ControlFlow::Continue((_, true)) | ControlFlow::Break(ControlExit::Stopped) => {
                    return Ok(());
                }
                ControlFlow::Break(ControlExit::Failed(e)) => return Err(e),
            }
        }
    }
}

fn go_to(base: &mut impl MobileBase, goal: Pose) -> RobotResult<BaseState> {
    base.move_to(goal)?;
    base.base_state()
}

fn creep(base: &mut impl VelocityControlledBase, cycles: usize) -> RobotResult<()> {
    let mut left = cycles;
    base.control_with(|_, _| {
        left -= 1;
        ([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], left == 0)
    })
}

#[test]
fn a_mission_base_needs_no_velocity_channel() {
    let mut mir = Mission { goal: None };
    let goal = Pose::from_position([2.0, 1.0, 0.0]);
    go_to(&mut mir, goal).unwrap();
    assert_eq!(mir.goal, Some(goal));
}

#[test]
fn a_wheeled_base_is_both_without_saying_so() {
    let mut base = Wheeled { cycles: 0 };
    go_to(&mut base, Pose::IDENTITY).unwrap();
    creep(&mut base, 3).unwrap();
    assert_eq!(base.cycles, 3);
}
