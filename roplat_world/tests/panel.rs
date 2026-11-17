//! The kinematic arm's panel against `check_panel`, broken panels that the
//! check must reject, and a logged panel used while the arm moves.

use std::{
    sync::Arc,
    thread,
    time::{Duration, Instant},
};

use roplat_behavior::{
    Actor, HasPanel, JointLimits, JointSpace, JointState, Joints, LoggedPanel, MoveTo, Outcome,
    Panel, PanelOp, PanelStatus, Release, Robot, RobotException, RobotResult,
    conformance::{PanelCheck, Violation, check_panel},
};
use roplat_world::{Hardware, KinematicArm};

/// Three joints at 1 rad/s within ±3 rad.
struct Model;

impl Joints<3> for Model {
    const JOINT_LIMITS: JointLimits<3> = JointLimits {
        dq_max: [1.0; 3],
        ..JointLimits::positions([-3.0; 3], [3.0; 3])
    };
}

type Arm = KinematicArm<Model, 3>;

/// A one-second motion, stopped after 0.2 s.
const CHECK: PanelCheck<3> = PanelCheck {
    target: [1.0, -1.0, 0.5],
    stop_after: Duration::from_millis(200),
    watch: Duration::from_millis(100),
    still: 1e-9,
};

fn joints(arm: &mut Arm) -> RobotResult<[f64; 3]> {
    Ok(arm.joints())
}

/// The button, as `check_panel` presses it.
fn press(hardware: &Hardware<3>) -> impl FnMut(bool) -> bool + '_ {
    |pressed| {
        hardware.estop_button(pressed);
        true
    }
}

#[test]
fn the_kinematic_arm_passes_check_panel() {
    let mut arm = Arm::new([0.0; 3]);
    let hardware = arm.hardware();
    let report = check_panel(&mut arm, &CHECK, joints, press(&hardware)).unwrap();
    assert!(report.latched, "{report:?}");
    assert!(report.on_site, "{report:?}");
    assert!(report.stop_latency <= report.bound, "{report:?}");
    println!("stop latency {:?} of {:?}", report.stop_latency, report.bound);
}

#[test]
fn without_a_button_the_on_site_part_is_skipped() {
    let mut arm = Arm::new([0.0; 3]);
    let report = check_panel(&mut arm, &CHECK, joints, |_| false).unwrap();
    assert!(!report.on_site, "{report:?}");
}

/// Moves `arm` on another thread and returns the handle; the arm comes back
/// with the result.
fn move_on_thread(mut arm: Arm, target: [f64; 3]) -> thread::JoinHandle<(Arm, RobotResult<()>)> {
    thread::spawn(move || {
        let result = arm.move_to(target);
        (arm, result)
    })
}

#[test]
fn pause_holds_the_motion_and_resume_finishes_it() {
    let arm = Arm::new([0.0; 3]);
    let panel = arm.panel();
    let by = Actor::Operator("console".into());
    let motion = move_on_thread(arm, [0.4, 0.0, 0.0]);
    thread::sleep(Duration::from_millis(100));
    panel.pause(&by).unwrap();
    assert!(panel.status().unwrap().paused);
    thread::sleep(Duration::from_millis(400));
    assert!(!motion.is_finished(), "a paused motion must not finish");
    panel.resume(&by).unwrap();
    let (arm, result) = motion.join().unwrap();
    result.unwrap();
    assert_eq!(arm.joints(), [0.4, 0.0, 0.0]);
}

#[test]
fn disable_ends_the_motion_and_enable_is_needed_again() {
    let arm = Arm::new([0.0; 3]);
    let panel = arm.panel();
    let by = Actor::App("test".into());
    let motion = move_on_thread(arm, [1.0, 0.0, 0.0]);
    thread::sleep(Duration::from_millis(100));
    panel.disable(&by).unwrap();
    let (mut arm, result) = motion.join().unwrap();
    assert!(matches!(result, Err(RobotException::Stopped)), "{result:?}");
    assert!(!panel.status().unwrap().enabled);
    let refused = arm.move_to([0.0; 3]);
    assert!(matches!(refused, Err(RobotException::Stopped)), "{refused:?}");
    panel.enable(&by).unwrap();
    arm.move_to([0.0; 3]).unwrap();
}

#[test]
fn a_protective_stop_ends_the_motion_and_clear_releases_it() {
    let arm = Arm::new([0.0; 3]);
    let panel = arm.panel();
    let hardware = arm.hardware();
    let motion = move_on_thread(arm, [1.0, 0.0, 0.0]);
    thread::sleep(Duration::from_millis(100));
    hardware.protective_stop();
    let (mut arm, result) = motion.join().unwrap();
    assert!(matches!(result, Err(RobotException::Stopped)), "{result:?}");
    assert_eq!(panel.status().unwrap().protective_stop, Some(Release::Clear));
    let cleared = panel.clear(&Actor::Agent("planner".into())).unwrap();
    assert!(cleared.ready(), "{cleared:?}");
    arm.move_to([0.0; 3]).unwrap();
}

// A service's view: the operator stops the agent's motion through a logged
// panel, and the agent reads who did it; the button shows up as the device.

#[test]
fn a_logged_panel_tells_the_agent_who_stopped_it() {
    let arm = Arm::new([0.0; 3]);
    let hardware = arm.hardware();
    let log = Arc::new(LoggedPanel::new(arm.panel()));
    let operator: Arc<dyn Panel> = log.clone();
    let agent: Arc<dyn Panel> = log.clone();
    let mut news = log.subscribe();

    let motion = move_on_thread(arm, [1.0, 0.0, 0.0]);
    thread::sleep(Duration::from_millis(100));
    operator.emergency_stop(&Actor::Operator("web".into())).unwrap();
    let (mut arm, result) = motion.join().unwrap();
    assert!(matches!(result, Err(RobotException::Stopped)), "{result:?}");

    let batch = news.next(Duration::from_secs(1));
    assert_eq!(batch.missed, 0);
    assert_eq!(batch.events.len(), 1, "{batch:?}");
    let event = &batch.events[0];
    assert_eq!(event.by, Actor::Operator("web".into()));
    assert_eq!(event.op, PanelOp::EmergencyStop);
    assert_eq!(event.outcome, Outcome::Done);
    assert!(event.after.is_some_and(|status| status.software_estop));

    // The agent clears the operator's software stop and goes on.
    let by = Actor::Agent("planner".into());
    agent.clear(&by).unwrap();
    arm.move_to([0.0; 3]).unwrap();

    // The button is pressed at the device: nobody called the panel.
    hardware.estop_button(true);
    let status = agent.status().unwrap();
    assert_eq!(status.device_estop, Some(Release::OnSite));
    let refused = agent.clear(&by);
    assert!(matches!(refused, Err(RobotException::LocalResetRequired)), "{refused:?}");
    let batch = news.next(Duration::from_secs(1));
    let ops: Vec<(Actor, PanelOp, Outcome)> =
        batch.events.iter().map(|e| (e.by.clone(), e.op, e.outcome.clone())).collect();
    let before = PanelStatus { enabled: true, ..PanelStatus::default() };
    assert_eq!(
        ops,
        [
            (by.clone(), PanelOp::Clear, Outcome::Done),
            (Actor::Device, PanelOp::Changed { before }, Outcome::Done),
            (by.clone(), PanelOp::Clear, Outcome::LocalResetRequired),
        ]
    );
}

#[test]
fn check_panel_passes_through_a_logged_panel() {
    /// The arm, issuing a logged panel; the driver is unchanged.
    struct Logged {
        arm: Arm,
        log: Arc<LoggedPanel>,
    }
    impl Robot for Logged {
        type State = JointState<3>;
        const CONTROL_PERIOD: Duration = Arm::CONTROL_PERIOD;
        fn version() -> String {
            Arm::version()
        }
        fn read_state(&mut self) -> RobotResult<JointState<3>> {
            self.arm.read_state()
        }
    }
    impl MoveTo<JointSpace<3>> for Logged {
        fn move_to(&mut self, target: [f64; 3]) -> RobotResult<()> {
            self.arm.move_to(target)
        }
    }
    impl HasPanel for Logged {
        fn panel(&self) -> Arc<dyn Panel> {
            self.log.clone()
        }
    }

    let arm = Arm::new([0.0; 3]);
    let hardware = arm.hardware();
    let log = Arc::new(LoggedPanel::new(arm.panel()));
    let mut logged = Logged { arm, log: Arc::clone(&log) };
    check_panel(&mut logged, &CHECK, |l| Ok(l.arm.joints()), press(&hardware)).unwrap();
    let ops: Vec<PanelOp> = log.recent(100).iter().map(|e| e.op).collect();
    assert!(ops.contains(&PanelOp::Stop), "{ops:?}");
    assert!(ops.contains(&PanelOp::EmergencyStop), "{ops:?}");
    assert!(log.recent(100).iter().all(|e| e.by != Actor::Device || matches!(e.op, PanelOp::Changed { .. })));
}

// Broken panels around the same arm: check_panel must name each defect.

#[derive(Clone, Copy, Debug, PartialEq)]
enum Defect {
    /// `stop` answers `Ok` and does nothing.
    StopIgnored,
    /// `stop` latches, as an emergency stop does.
    StopLatches,
    /// The declared bound is shorter than the arm needs.
    BoundTooShort,
    /// `enable` answers `Ok` whatever holds the arm.
    EnableAlways,
    /// `clear` answers `Ok` with the button pressed.
    ClearIgnoresButton,
    /// The status never shows the device's own emergency stop.
    ButtonUnreported,
    /// A stopped motion reports success.
    StoppedIsOk,
}

struct Faulty {
    arm: Arm,
    defect: Defect,
}

struct FaultyPanel {
    inner: Arc<dyn Panel>,
    defect: Defect,
}

impl Panel for FaultyPanel {
    fn stop(&self, by: &Actor) -> RobotResult<()> {
        match self.defect {
            Defect::StopIgnored => Ok(()),
            Defect::StopLatches => self.inner.emergency_stop(by),
            _ => self.inner.stop(by),
        }
    }
    fn emergency_stop(&self, by: &Actor) -> RobotResult<()> {
        self.inner.emergency_stop(by)
    }
    fn clear(&self, by: &Actor) -> RobotResult<PanelStatus> {
        match (self.defect, self.inner.clear(by)) {
            (Defect::ClearIgnoresButton, Err(RobotException::LocalResetRequired)) => {
                self.inner.status()
            }
            (_, result) => result,
        }
    }
    fn enable(&self, by: &Actor) -> RobotResult<()> {
        match self.defect {
            Defect::EnableAlways => Ok(()),
            _ => self.inner.enable(by),
        }
    }
    fn status(&self) -> RobotResult<PanelStatus> {
        let mut status = self.inner.status()?;
        if self.defect == Defect::ButtonUnreported {
            status.device_estop = None;
        }
        Ok(status)
    }
    fn stop_bound(&self) -> Duration {
        match self.defect {
            Defect::BoundTooShort => Duration::from_nanos(1),
            _ => self.inner.stop_bound(),
        }
    }
}

impl Robot for Faulty {
    type State = JointState<3>;
    const CONTROL_PERIOD: Duration = Arm::CONTROL_PERIOD;
    fn version() -> String {
        Arm::version()
    }
    fn read_state(&mut self) -> RobotResult<JointState<3>> {
        self.arm.read_state()
    }
}

impl MoveTo<JointSpace<3>> for Faulty {
    fn move_to(&mut self, target: [f64; 3]) -> RobotResult<()> {
        match (self.defect, self.arm.move_to(target)) {
            (Defect::StoppedIsOk, Err(RobotException::Stopped)) => Ok(()),
            (_, result) => result,
        }
    }
}

impl HasPanel for Faulty {
    fn panel(&self) -> Arc<dyn Panel> {
        Arc::new(FaultyPanel { inner: self.arm.panel(), defect: self.defect })
    }
}

fn check_faulty(defect: Defect) -> Violation {
    let arm = Arm::new([0.0; 3]);
    let hardware = arm.hardware();
    let mut faulty = Faulty { arm, defect };
    let started = Instant::now();
    let result =
        check_panel(&mut faulty, &CHECK, |f| Ok(f.arm.joints()), press(&hardware));
    assert!(started.elapsed() < Duration::from_secs(10));
    result.expect_err("check_panel passed a broken panel")
}

#[test]
fn check_panel_names_each_broken_promise() {
    for (defect, check) in [
        (Defect::StopIgnored, "panel: stopped"),
        (Defect::StopLatches, "panel: stop latched"),
        (Defect::BoundTooShort, "panel: stop bound"),
        (Defect::EnableAlways, "panel: enable while held"),
        (Defect::ClearIgnoresButton, "panel: local reset"),
        (Defect::ButtonUnreported, "panel: device estop"),
        (Defect::StoppedIsOk, "panel: stopped"),
    ] {
        let violation = check_faulty(defect);
        assert_eq!(violation.check, check, "{defect:?}: {violation}");
    }
}
