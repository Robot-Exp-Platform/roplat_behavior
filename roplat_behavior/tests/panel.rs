//! The panel contract's types and defaults, and what a `LoggedPanel` records.

use std::{
    sync::{
        Arc, Mutex,
        mpsc::{Receiver, channel},
    },
    thread,
    time::{Duration, Instant},
};

use roplat_behavior::{
    Actor, LoggedPanel, Outcome, Panel, PanelOp, PanelStatus, Release, RobotException,
    RobotResult,
    msgs::{ManualClock, clock},
};

/// A panel whose status the test sets directly, as the device would.
struct Fake {
    status: Mutex<PanelStatus>,
    /// Taken by the first `status` call, which reads the status, then waits
    /// for the test before answering with what it read.
    gate: Mutex<Option<Receiver<()>>>,
}

impl Fake {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            status: Mutex::new(PanelStatus { enabled: true, ..PanelStatus::default() }),
            gate: Mutex::new(None),
        })
    }

    fn set(&self, f: impl FnOnce(&mut PanelStatus)) {
        f(&mut self.status.lock().unwrap());
    }
}

impl Panel for Fake {
    fn stop(&self, _by: &Actor) -> RobotResult<()> {
        Ok(())
    }

    fn emergency_stop(&self, _by: &Actor) -> RobotResult<()> {
        self.set(|status| status.software_estop = true);
        Ok(())
    }

    fn clear(&self, _by: &Actor) -> RobotResult<PanelStatus> {
        let mut status = self.status.lock().unwrap();
        status.software_estop = false;
        if status.device_estop == Some(Release::OnSite) {
            return Err(RobotException::LocalResetRequired);
        }
        Ok(*status)
    }

    fn enable(&self, _by: &Actor) -> RobotResult<()> {
        let mut status = self.status.lock().unwrap();
        if status.held() {
            return Err(RobotException::Stopped);
        }
        status.enabled = true;
        Ok(())
    }

    fn status(&self) -> RobotResult<PanelStatus> {
        let status = *self.status.lock().unwrap();
        let gate = self.gate.lock().unwrap().take();
        if let Some(gate) = gate {
            gate.recv().unwrap();
        }
        Ok(status)
    }

    fn stop_bound(&self) -> Duration {
        Duration::from_millis(10)
    }
}

fn agent() -> Actor {
    Actor::Agent("planner".into())
}

fn operator() -> Actor {
    Actor::Operator("web".into())
}

#[test]
fn a_panel_is_a_trait_object() {
    let fake = Fake::new();
    let panels: Vec<Arc<dyn Panel>> = vec![fake.clone(), Arc::new(LoggedPanel::new(fake))];
    for panel in &panels {
        assert_eq!(panel.stop_bound(), Duration::from_millis(10));
        panel.stop(&agent()).unwrap();
    }
}

#[test]
fn optional_operations_are_not_supported_by_default() {
    let panel = Fake::new();
    let by = agent();
    for (operation, result) in [
        ("pause", panel.pause(&by)),
        ("resume", panel.resume(&by)),
        ("disable", panel.disable(&by)),
    ] {
        assert!(
            matches!(result, Err(RobotException::NotSupported { operation: o }) if o == operation),
            "{operation}: {result:?}"
        );
    }
}

#[test]
fn the_status_says_what_holds_the_device() {
    let ready = PanelStatus { enabled: true, ..PanelStatus::default() };
    assert!(ready.ready() && !ready.held());
    for held in [
        PanelStatus { software_estop: true, ..ready },
        PanelStatus { device_estop: Some(Release::OnSite), ..ready },
        PanelStatus { device_estop: Some(Release::Clear), ..ready },
        PanelStatus { protective_stop: Some(Release::Clear), ..ready },
    ] {
        assert!(held.held() && !held.ready(), "{held:?}");
    }
    let paused = PanelStatus { paused: true, ..ready };
    assert!(!paused.held() && !paused.ready());
    assert!(!PanelStatus::default().ready());
}

#[test]
fn every_operation_is_recorded_with_its_caller_and_answer() {
    let log = LoggedPanel::new(Fake::new());
    log.stop(&agent()).unwrap();
    log.emergency_stop(&operator()).unwrap();
    let refused = log.enable(&agent());
    assert!(matches!(refused, Err(RobotException::Stopped)), "{refused:?}");
    let refused = log.pause(&agent());
    assert!(
        matches!(refused, Err(RobotException::NotSupported { operation: "pause" })),
        "the caller gets the panel's own error: {refused:?}"
    );
    let cleared = log.clear(&agent()).unwrap();
    assert!(cleared.ready(), "{cleared:?}");

    let events = log.recent(10);
    let seen: Vec<(u64, Actor, PanelOp, Outcome)> =
        events.iter().map(|e| (e.seq, e.by.clone(), e.op, e.outcome.clone())).collect();
    assert_eq!(
        seen,
        [
            (0, agent(), PanelOp::Stop, Outcome::Done),
            (1, operator(), PanelOp::EmergencyStop, Outcome::Done),
            (2, agent(), PanelOp::Enable, Outcome::Stopped),
            (3, agent(), PanelOp::Pause, Outcome::NotSupported),
            (4, agent(), PanelOp::Clear, Outcome::Done),
        ]
    );
    assert!(events[1].after.is_some_and(|s| s.software_estop));
    assert!(events[4].after.is_some_and(|s| s.ready()));
    assert!(events.windows(2).all(|w| w[0].at.nanos <= w[1].at.nanos));
    assert!(events.iter().all(|e| e.at.clock == clock::REALTIME));
    assert_eq!(log.recent(2).iter().map(|e| e.seq).collect::<Vec<_>>(), [3, 4]);
}

#[test]
fn a_change_nobody_asked_for_is_the_devices() {
    let fake = Fake::new();
    let log = LoggedPanel::new(fake.clone());
    // The first read only sets what the log has seen.
    log.status().unwrap();
    assert!(log.recent(10).is_empty());

    fake.set(|status| {
        status.device_estop = Some(Release::OnSite);
        status.enabled = false;
    });
    let now = log.status().unwrap();
    log.status().unwrap();
    let events = log.recent(10);
    assert_eq!(events.len(), 1, "{events:?}");
    let before = PanelStatus { enabled: true, ..PanelStatus::default() };
    assert_eq!(events[0].by, Actor::Device);
    assert_eq!(events[0].op, PanelOp::Changed { before });
    assert_eq!(events[0].after, Some(now));

    // A change an operation through the log made is that operation's.
    let refused = log.clear(&agent());
    assert!(matches!(refused, Err(RobotException::LocalResetRequired)), "{refused:?}");
    fake.set(|status| status.device_estop = None);
    log.status().unwrap();
    log.enable(&agent()).unwrap();
    log.status().unwrap();
    let ops: Vec<(Actor, PanelOp)> = log.recent(10).iter().map(|e| (e.by.clone(), e.op)).collect();
    let after_button = PanelStatus { device_estop: Some(Release::OnSite), ..PanelStatus::default() };
    assert_eq!(
        ops,
        [
            (Actor::Device, PanelOp::Changed { before }),
            (agent(), PanelOp::Clear),
            (Actor::Device, PanelOp::Changed { before: after_button }),
            (agent(), PanelOp::Enable),
        ]
    );
}

#[test]
fn a_status_read_overlapping_an_operation_is_not_the_devices() {
    let fake = Fake::new();
    let log = Arc::new(LoggedPanel::new(fake.clone()));
    log.status().unwrap();
    let (open, gate) = channel();
    *fake.gate.lock().unwrap() = Some(gate);

    // A reader takes the status before the emergency stop and answers after
    // it, with what it read.
    let reader = {
        let log = Arc::clone(&log);
        thread::spawn(move || log.status().unwrap())
    };
    while fake.gate.lock().unwrap().is_some() {
        thread::yield_now();
    }
    log.emergency_stop(&operator()).unwrap();
    open.send(()).unwrap();
    let stale = reader.join().unwrap();
    assert!(!stale.software_estop, "the reader saw the status before the stop");

    let ops: Vec<PanelOp> = log.recent(10).iter().map(|e| e.op).collect();
    assert_eq!(ops, [PanelOp::EmergencyStop]);
    // The baseline is still the operation's.
    log.status().unwrap();
    assert_eq!(log.recent(10).len(), 1);
}

#[test]
fn a_subscriber_reads_what_comes_after_it() {
    let log = Arc::new(LoggedPanel::new(Fake::new()));
    log.stop(&agent()).unwrap();
    let mut news = log.subscribe();
    let started = Instant::now();
    assert!(news.next(Duration::from_millis(50)).events.is_empty());
    assert!(started.elapsed() >= Duration::from_millis(50));

    // A waiting subscriber is woken by an operation on another thread.
    let writer = {
        let log = Arc::clone(&log);
        thread::spawn(move || {
            thread::sleep(Duration::from_millis(50));
            log.emergency_stop(&operator()).unwrap();
        })
    };
    let started = Instant::now();
    let batch = news.next(Duration::from_secs(10));
    assert!(started.elapsed() < Duration::from_secs(5));
    writer.join().unwrap();
    assert_eq!(batch.missed, 0);
    assert_eq!(batch.events.iter().map(|e| e.seq).collect::<Vec<_>>(), [1]);
    assert_eq!(batch.events[0].by, operator());
    assert!(news.next(Duration::ZERO).events.is_empty());
}

#[test]
fn a_subscriber_that_falls_behind_is_told_what_it_missed() {
    let log = LoggedPanel::with_capacity(Fake::new(), 2);
    let mut news = log.subscribe();
    for _ in 0..5 {
        log.stop(&agent()).unwrap();
    }
    let batch = news.next(Duration::ZERO);
    assert_eq!(batch.missed, 3);
    assert_eq!(batch.events.iter().map(|e| e.seq).collect::<Vec<_>>(), [3, 4]);
    assert_eq!(log.recent(10).len(), 2);
    log.stop(&agent()).unwrap();
    let batch = news.next(Duration::ZERO);
    assert_eq!((batch.missed, batch.events.len()), (0, 1));
}

#[test]
fn events_take_the_clock_they_are_given() {
    let clock = ManualClock::simulation();
    assert!(clock.set(42));
    let log = LoggedPanel::new(Fake::new()).with_clock(clock);
    log.stop(&agent()).unwrap();
    let at = log.recent(1)[0].at;
    assert_eq!((at.clock, at.nanos), (clock::SIMULATION, 42));
}

#[test]
fn events_serialize_for_a_service() {
    let log = LoggedPanel::new(Fake::new());
    log.emergency_stop(&operator()).unwrap();
    let event = log.recent(1).remove(0);
    let json = serde_json::to_string(&event).unwrap();
    let back: roplat_behavior::PanelEvent = serde_json::from_str(&json).unwrap();
    assert_eq!(back, event);
}
