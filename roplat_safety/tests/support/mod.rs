//! A two-joint arm whose device session records what it was sent.

use std::{
    ops::ControlFlow,
    sync::{Arc, Mutex},
    time::Duration,
};

use roplat_behavior::{
    ArmState, CartesianVelocityControl, ControlStep, ControlWith, JointPositionControl,
    JointSample, JointState, JointVelocityControl, Robot, RobotResult, SpatialSample, StateView,
    TorqueControl, Twist,
    msgs::{Clock, ManualClock},
};

pub const DOF: usize = 2;
pub const PERIOD: Duration = Duration::from_millis(1);
const CYCLE_BUDGET: usize = 1000;

#[derive(Default, Debug)]
pub struct Trace {
    pub commands: Vec<Vec<f64>>,
    pub entries: usize,
    pub breaks: usize,
    pub exits: usize,
}

#[derive(Debug)]
pub struct MockArm {
    pub q: [f64; DOF],
    pub dq: [f64; DOF],
    pub twist: [f64; 6],
    pub trace: Arc<Mutex<Trace>>,
    /// Advanced by one period before every observation, as a simulator steps.
    pub clock: Arc<ManualClock>,
    /// Extra time that passes before the given cycle.
    pub stall: Option<(usize, Duration)>,
}

impl MockArm {
    pub fn at(q: [f64; DOF]) -> Self {
        Self {
            q,
            dq: [0.; DOF],
            twist: [0.; 6],
            trace: Arc::default(),
            clock: Arc::new(ManualClock::simulation()),
            stall: None,
        }
    }

    pub fn commands(&self) -> Vec<Vec<f64>> {
        self.trace.lock().unwrap().commands.clone()
    }

    /// `(entries, breaks, exits)` of the device sessions so far.
    pub fn sessions(&self) -> (usize, usize, usize) {
        let trace = self.trace.lock().unwrap();
        (trace.entries, trace.breaks, trace.exits)
    }

    pub fn now(&self) -> roplat_behavior::Stamp {
        self.clock.now()
    }

    fn joint_state(&self) -> JointState<DOF> {
        StateView::from_meas(JointSample::EMPTY.with_q(self.q).with_dq(self.dq))
    }

    fn arm_state(&self) -> ArmState<DOF> {
        ArmState {
            joint: self.joint_state(),
            flange: StateView::from_meas(SpatialSample::EMPTY.with_twist(Twist::from(self.twist))),
            ..ArmState::default()
        }
    }

    fn session<O, C>(
        &mut self,
        observe: fn(&Self) -> O,
        apply: fn(&mut Self, &C) -> Vec<f64>,
        mut closure: impl FnMut(O, Duration) -> ControlStep<C>,
    ) -> RobotResult<()> {
        self.trace.lock().unwrap().entries += 1;
        for cycle in 0..CYCLE_BUDGET {
            if let Some((at, extra)) = self.stall
                && at == cycle
            {
                self.clock.advance(extra);
            }
            self.clock.advance(PERIOD);
            match closure(observe(self), PERIOD) {
                ControlFlow::Continue((command, done)) => {
                    let sent = apply(self, &command);
                    self.trace.lock().unwrap().commands.push(sent);
                    if !done {
                        continue;
                    }
                }
                ControlFlow::Break(_) => self.trace.lock().unwrap().breaks += 1,
            }
            self.trace.lock().unwrap().exits += 1;
            return Ok(());
        }
        panic!("mock session exceeded its cycle budget")
    }
}

impl Robot for MockArm {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "mock".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl ControlWith<JointPositionControl<DOF>> for MockArm {
    fn hold_command(state: &JointState<DOF>) -> [f64; DOF] {
        state.meas.q().unwrap_or_default()
    }
    fn control_with_flow<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<DOF>, Duration) -> ControlStep<[f64; DOF]>,
    {
        let apply = |arm: &mut Self, q: &[f64; DOF]| {
            arm.dq = std::array::from_fn(|i| (q[i] - arm.q[i]) / PERIOD.as_secs_f64());
            arm.q = *q;
            q.to_vec()
        };
        self.session(Self::joint_state, apply, closure)
    }
}

impl ControlWith<JointVelocityControl<DOF>> for MockArm {
    fn hold_command(_: &JointState<DOF>) -> [f64; DOF] {
        [0.; DOF]
    }
    fn control_with_flow<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<DOF>, Duration) -> ControlStep<[f64; DOF]>,
    {
        let apply = |arm: &mut Self, dq: &[f64; DOF]| {
            arm.q = std::array::from_fn(|i| arm.q[i] + dq[i] * PERIOD.as_secs_f64());
            arm.dq = *dq;
            dq.to_vec()
        };
        self.session(Self::joint_state, apply, closure)
    }
}

impl ControlWith<TorqueControl<DOF>> for MockArm {
    fn hold_command(_: &JointState<DOF>) -> [f64; DOF] {
        [0.; DOF]
    }
    fn control_with_flow<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<DOF>, Duration) -> ControlStep<[f64; DOF]>,
    {
        self.session(
            Self::joint_state,
            |_, tau: &[f64; DOF]| tau.to_vec(),
            closure,
        )
    }
}

impl ControlWith<CartesianVelocityControl<DOF>> for MockArm {
    fn hold_command(_: &ArmState<DOF>) -> [f64; 6] {
        [0.; 6]
    }
    fn control_with_flow<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<DOF>, Duration) -> ControlStep<[f64; 6]>,
    {
        let apply = |arm: &mut Self, twist: &[f64; 6]| {
            arm.twist = *twist;
            twist.to_vec()
        };
        self.session(Self::arm_state, apply, closure)
    }
}
