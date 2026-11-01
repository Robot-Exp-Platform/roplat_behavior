use roplat_behavior::{ControlSpace, ControlStep, ControlWith, Robot, RobotException, RobotResult};
use std::{
    ops::ControlFlow,
    sync::{Arc, Mutex},
    time::Duration,
};

#[derive(Default, Debug)]
pub struct Trace {
    pub commands: Vec<u32>,
    pub entries: usize,
    pub exits: usize,
    pub callbacks: usize,
}
pub struct TestControl;
#[derive(Debug)]
pub struct TestRobot {
    pub trace: Arc<Mutex<Trace>>,
    pub fail_entry: bool,
    pub fail_exit: bool,
}
impl TestRobot {
    pub fn new() -> Self {
        Self { trace: Arc::default(), fail_entry: false, fail_exit: false }
    }
}
impl Robot for TestRobot {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "mock".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}
impl ControlSpace for TestControl {
    type Obs = u32;
    type Command = u32;
}
impl ControlWith<TestControl> for TestRobot {
    fn hold_command(obs: &u32) -> u32 {
        *obs
    }
    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(u32, Duration) -> ControlStep<u32>,
    {
        self.trace.lock().unwrap().entries += 1;
        if self.fail_entry {
            return Err(RobotException::NetworkError("entry failed".into()));
        }
        for cycle in 0..100_u32 {
            self.trace.lock().unwrap().callbacks += 1;
            match closure(cycle, Duration::from_millis(1)) {
                ControlFlow::Continue((command, done)) => {
                    self.trace.lock().unwrap().commands.push(command);
                    if !done {
                        continue;
                    }
                }
                ControlFlow::Break(_) => {}
            }
            self.trace.lock().unwrap().exits += 1;
            return if self.fail_exit {
                Err(RobotException::CommandException("cleanup failed".into()))
            } else {
                Ok(())
            };
        }
        panic!("test session exceeded finite cycle budget")
    }
}
