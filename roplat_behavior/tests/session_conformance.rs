//! `check_control_session` accepts compliant sessions, reports how a driver
//! fills the callback `Duration`, and names broken session protocols.
#![cfg(feature = "conformance")]

use std::{ops::ControlFlow, time::Duration};

use roplat_behavior::{
    ControlStep, ControlWith, JointPositionControl, JointState, Robot, RobotException, RobotResult,
    conformance::{DurationPattern, check_control_session},
    msgs::JointSample,
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Fault {
    None,
    IgnoresDone,
    IgnoresBreak,
    EndsEarly,
    DeadAfterBreak,
    ErrorOnBreak,
}

#[derive(Clone, Copy)]
enum Timing {
    Period,
    SincePrevious,
    SessionTime,
    Jitter,
}

struct MockArm {
    q: [f64; 2],
    fault: Fault,
    timing: Timing,
    broke: bool,
}

impl MockArm {
    fn new(fault: Fault, timing: Timing) -> Self {
        Self { q: [0.3, -0.7], fault, timing, broke: false }
    }
}

impl Robot for MockArm {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "session-mock".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl ControlWith<JointPositionControl<2>> for MockArm {
    fn hold_command(obs: &JointState<2>) -> [f64; 2] {
        obs.meas.q().unwrap_or([0.0; 2])
    }

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<2>, Duration) -> ControlStep<[f64; 2]>,
    {
        if self.broke && self.fault == Fault::DeadAfterBreak {
            return Err(RobotException::CommandException("still stopping".into()));
        }
        let period = Duration::from_millis(1);
        for cycle in 0..100u32 {
            if self.fault == Fault::EndsEarly && cycle == 2 {
                return Ok(());
            }
            let dt = match self.timing {
                Timing::Period => period,
                Timing::SincePrevious if cycle == 0 => Duration::ZERO,
                Timing::SincePrevious => period,
                Timing::SessionTime => period * (cycle + 1),
                Timing::Jitter => period * (1 + 2 * (cycle % 2)),
            };
            let obs = JointState::from_meas(JointSample::default().with_q(self.q));
            match closure(obs, dt) {
                ControlFlow::Continue((command, done)) => {
                    self.q = command;
                    if done && self.fault != Fault::IgnoresDone {
                        return Ok(());
                    }
                }
                ControlFlow::Break(_) => {
                    self.broke = true;
                    match self.fault {
                        Fault::IgnoresBreak => continue,
                        Fault::ErrorOnBreak => {
                            return Err(RobotException::CommandException("stopped".into()));
                        }
                        _ => return Ok(()),
                    }
                }
            }
        }
        Ok(())
    }
}

fn check(arm: &mut MockArm) -> Result<roplat_behavior::conformance::SessionReport, String> {
    check_control_session::<_, JointPositionControl<2>>(arm, 5).map_err(|v| v.to_string())
}

#[test]
fn compliant_sessions_pass_and_report_the_duration_pattern() {
    for (timing, pattern) in [
        (Timing::Period, DurationPattern::CyclePeriod),
        (Timing::SincePrevious, DurationPattern::SincePrevious),
        (Timing::SessionTime, DurationPattern::SessionTime),
        (Timing::Jitter, DurationPattern::Other),
    ] {
        let mut arm = MockArm::new(Fault::None, timing);
        let report = check(&mut arm).unwrap();
        assert_eq!(report.pattern, pattern);
        assert_eq!(report.durations.len(), 5);
        assert!(report.break_error.is_none());
        // Holding commands leave the arm where it was.
        assert_eq!(arm.q, [0.3, -0.7]);
    }
}

#[test]
fn the_result_of_break_is_reported_not_judged() {
    let report = check(&mut MockArm::new(Fault::ErrorOnBreak, Timing::Period)).unwrap();
    assert_eq!(
        report.break_error.as_deref(),
        Some("Command exception: stopped")
    );
}

#[test]
fn broken_session_protocols_are_named() {
    for (fault, expected) in [
        (
            Fault::IgnoresDone,
            "control_session: done: done on cycle 5, but the callback ran again",
        ),
        (
            Fault::EndsEarly,
            "control_session: done: the session ended after 2 of 5 cycles without done",
        ),
        (
            Fault::IgnoresBreak,
            "control_session: break: Break on cycle 5, but the callback ran again",
        ),
        (
            Fault::DeadAfterBreak,
            "control_session: after break: the device returned an error: Command exception: still stopping",
        ),
    ] {
        let error = check(&mut MockArm::new(fault, Timing::Period)).unwrap_err();
        assert_eq!(error, expected);
    }
}
