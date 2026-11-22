//! A record of what was done through a panel, by whom, and what the device
//! did by itself.

use std::{
    collections::VecDeque,
    sync::{Arc, Condvar, Mutex, MutexGuard},
    time::{Duration, Instant},
};

use roplat_msgs::{Clock, RealtimeClock, Stamp};
use serde::{Deserialize, Serialize};

use crate::{
    Actor, Panel, PanelStatus, RobotException, RobotResult,
    robot::panel::{clear_no_effect, had_no_effect},
};

/// What a [`PanelEvent`] records.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PanelOp {
    Stop,
    EmergencyStop,
    Pause,
    Disable,
    Clear,
    Enable,
    Resume,
    /// The status changed without an operation through this log: the device
    /// did it, or something that bypassed the log (another panel, the
    /// device's own pendant). Recorded with [`Actor::Device`].
    Changed {
        before: PanelStatus,
    },
}

/// What the panel answered, as the log keeps it: the answers the panel
/// contract names, and any other error by its message.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Outcome {
    Done,
    /// The panel answered `Ok` but said the call did nothing on the device
    /// ([`note_no_effect`](crate::note_no_effect)): the default `enable` and
    /// `disable`, for one.
    NoEffect,
    NotSupported,
    NoEmergencyStop,
    /// The emergency stop is latched, but the device is not at a full stop:
    /// [`RobotException::EmergencyStopLatchedOnly`], with what it does.
    EmergencyStopLatchedOnly {
        reason: String,
    },
    Stopped,
    LocalResetRequired,
    /// Another error, e.g. a lost connection.
    Failed {
        error: String,
    },
}

impl From<&RobotException> for Outcome {
    fn from(error: &RobotException) -> Self {
        match error {
            RobotException::NotSupported { .. } => Self::NotSupported,
            RobotException::NoEmergencyStop => Self::NoEmergencyStop,
            RobotException::EmergencyStopLatchedOnly(reason) => {
                Self::EmergencyStopLatchedOnly { reason: reason.to_string() }
            }
            RobotException::Stopped => Self::Stopped,
            RobotException::LocalResetRequired => Self::LocalResetRequired,
            other => Self::Failed { error: other.to_string() },
        }
    }
}

/// One entry of a [`LoggedPanel`]'s record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PanelEvent {
    /// Rises by one per event of this log, so a gap shows lost events.
    pub seq: u64,
    /// When the operation returned, or when the change was observed.
    pub at: Stamp,
    pub by: Actor,
    pub op: PanelOp,
    /// What the panel answered; [`Outcome::Done`] for [`PanelOp::Changed`].
    pub outcome: Outcome,
    /// The status read right after the operation, or the changed status;
    /// `None` when that read failed.
    pub after: Option<PanelStatus>,
}

/// Events a [`PanelSubscription`] received in one call.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct EventBatch {
    /// Events that left the log before this subscriber read them.
    pub missed: u64,
    pub events: Vec<PanelEvent>,
}

/// The log behind a [`LoggedPanel`] and its subscriptions.
///
/// A mutex and a condition variable: panel operations are rare and not on a
/// control path, and a waiting subscriber has to be woken.
struct Log {
    state: Mutex<LogState>,
    grew: Condvar,
}

struct LogState {
    events: VecDeque<PanelEvent>,
    capacity: usize,
    next_seq: u64,
    /// The last status this log saw; a change from it that no operation
    /// explains is the device's.
    last: Option<PanelStatus>,
    /// Operations started so far; a status read during which it changed
    /// overlapped an operation.
    started: u64,
    in_flight: usize,
}

impl Log {
    fn lock(&self) -> MutexGuard<'_, LogState> {
        // A panic elsewhere while holding this lock leaves a record behind,
        // nothing half-done; keep recording.
        self.state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

impl LogState {
    fn push(
        &mut self,
        at: Stamp,
        by: Actor,
        op: PanelOp,
        outcome: Outcome,
        after: Option<PanelStatus>,
    ) {
        if self.events.len() == self.capacity {
            self.events.pop_front();
        }
        let seq = self.next_seq;
        self.next_seq += 1;
        self.events
            .push_back(PanelEvent { seq, at, by, op, outcome, after });
    }
}

/// A [`Panel`] that records every operation through it, and every status
/// change it observes that no operation through it explains.
///
/// Wrap the panel a device issues and hand this one out:
///
/// ```ignore
/// let log = Arc::new(LoggedPanel::new(robot.panel()));
/// let for_agent: Arc<dyn Panel> = log.clone();
/// // ... the service reads what happened:
/// for event in log.recent(10) { /* event.by, event.op, event.outcome */ }
/// ```
///
/// - Each operation is recorded when it returns, with its caller, its answer
///   and the status read right after it; that read is part of the call, so an
///   operation through the log takes one status read longer. An `Ok` from a
///   call that marked itself with [`note_no_effect`](crate::note_no_effect),
///   such as the default `enable`, is recorded as [`Outcome::NoEffect`]; the
///   mark is cleared before each call and read on the calling thread.
/// - A status change is noticed when the status is read through this panel,
///   by [`status`](Panel::status) or after an operation. A service that wants
///   the device's own changes promptly reads the status periodically; nothing
///   here polls. A change that happens while an operation through this panel
///   is in progress is credited to that operation, not to the device.
/// - The log keeps the latest `capacity` events. A subscriber that falls
///   further behind is told how many it missed.
///
/// The log trusts the `by` its callers pass; the service that hands out the
/// panel names them. The driver is not involved: any panel can be logged.
pub struct LoggedPanel<C: Clock = RealtimeClock> {
    // dyn(c): wraps whichever device's panel the service holds.
    inner: Arc<dyn Panel>,
    log: Arc<Log>,
    clock: C,
}

/// Events kept by [`LoggedPanel::new`].
pub const PANEL_LOG_CAPACITY: usize = 1024;

impl LoggedPanel<RealtimeClock> {
    /// Logs `inner`, keeping the latest [`PANEL_LOG_CAPACITY`] events,
    /// stamped with the system real-time clock.
    pub fn new(inner: Arc<dyn Panel>) -> Self {
        Self::with_capacity(inner, PANEL_LOG_CAPACITY)
    }

    /// Logs `inner`, keeping the latest `capacity` events (at least one).
    pub fn with_capacity(inner: Arc<dyn Panel>, capacity: usize) -> Self {
        let capacity = capacity.max(1);
        let state = LogState {
            events: VecDeque::with_capacity(capacity),
            capacity,
            next_seq: 0,
            last: None,
            started: 0,
            in_flight: 0,
        };
        Self {
            inner,
            log: Arc::new(Log { state: Mutex::new(state), grew: Condvar::new() }),
            clock: RealtimeClock,
        }
    }
}

impl<C: Clock> LoggedPanel<C> {
    /// Stamps events with `clock` instead, e.g. a simulation clock.
    pub fn with_clock<C2: Clock>(self, clock: C2) -> LoggedPanel<C2> {
        LoggedPanel { inner: self.inner, log: self.log, clock }
    }

    /// The latest `count` events, oldest first.
    pub fn recent(&self, count: usize) -> Vec<PanelEvent> {
        let state = self.log.lock();
        let skip = state.events.len().saturating_sub(count);
        state.events.iter().skip(skip).cloned().collect()
    }

    /// A reader of the events recorded from now on.
    pub fn subscribe(&self) -> PanelSubscription {
        let next = self.log.lock().next_seq;
        PanelSubscription { log: Arc::clone(&self.log), next }
    }

    fn record<T>(
        &self,
        by: &Actor,
        op: PanelOp,
        call: impl FnOnce(&dyn Panel) -> RobotResult<T>,
    ) -> RobotResult<T> {
        {
            let mut state = self.log.lock();
            state.started += 1;
            state.in_flight += 1;
        }
        clear_no_effect();
        let result = call(&*self.inner);
        let no_effect = had_no_effect(op);
        let after = self.inner.status().ok();
        let at = self.clock.now();
        let outcome = match &result {
            Ok(_) if no_effect => Outcome::NoEffect,
            Ok(_) => Outcome::Done,
            Err(error) => Outcome::from(error),
        };
        let mut state = self.log.lock();
        state.in_flight -= 1;
        if after.is_some() {
            state.last = after;
        }
        state.push(at, by.clone(), op, outcome, after);
        drop(state);
        self.log.grew.notify_all();
        result
    }
}

impl<C: Clock + 'static> Panel for LoggedPanel<C> {
    fn stop(&self, by: &Actor) -> RobotResult<()> {
        self.record(by, PanelOp::Stop, |panel| panel.stop(by))
    }

    fn emergency_stop(&self, by: &Actor) -> RobotResult<()> {
        self.record(by, PanelOp::EmergencyStop, |panel| panel.emergency_stop(by))
    }

    fn pause(&self, by: &Actor) -> RobotResult<()> {
        self.record(by, PanelOp::Pause, |panel| panel.pause(by))
    }

    fn disable(&self, by: &Actor) -> RobotResult<()> {
        self.record(by, PanelOp::Disable, |panel| panel.disable(by))
    }

    fn clear(&self, by: &Actor) -> RobotResult<PanelStatus> {
        self.record(by, PanelOp::Clear, |panel| panel.clear(by))
    }

    fn enable(&self, by: &Actor) -> RobotResult<()> {
        self.record(by, PanelOp::Enable, |panel| panel.enable(by))
    }

    fn resume(&self, by: &Actor) -> RobotResult<()> {
        self.record(by, PanelOp::Resume, |panel| panel.resume(by))
    }

    fn status(&self) -> RobotResult<PanelStatus> {
        let (started, busy) = {
            let state = self.log.lock();
            (state.started, state.in_flight > 0)
        };
        let status = self.inner.status()?;
        let at = self.clock.now();
        let mut state = self.log.lock();
        // A read that overlapped an operation may show its effect half done;
        // the operation's own read after it sets the baseline.
        if busy || state.in_flight > 0 || state.started != started {
            return Ok(status);
        }
        if let Some(before) = state.last.replace(status)
            && before != status
        {
            state.push(
                at,
                Actor::Device,
                PanelOp::Changed { before },
                Outcome::Done,
                Some(status),
            );
            drop(state);
            self.log.grew.notify_all();
        }
        Ok(status)
    }

    fn stop_bound(&self) -> Duration {
        self.inner.stop_bound()
    }
}

/// A reader of a [`LoggedPanel`]'s events, from the moment it subscribed.
///
/// It is a position in the log, not a queue of its own: a subscriber that
/// does not read costs nothing, and one that falls more than the log's
/// capacity behind is told how many events it missed.
pub struct PanelSubscription {
    log: Arc<Log>,
    next: u64,
}

impl PanelSubscription {
    /// The events after the ones already returned, waiting up to `timeout`
    /// for the first when there are none yet. An empty batch means the time
    /// ran out.
    pub fn next(&mut self, timeout: Duration) -> EventBatch {
        let deadline = Instant::now() + timeout;
        let mut state = self.log.lock();
        while state.next_seq <= self.next {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return EventBatch::default();
            }
            state = match self.log.grew.wait_timeout(state, left) {
                Ok((state, _)) => state,
                Err(poisoned) => poisoned.into_inner().0,
            };
        }
        let oldest = state
            .events
            .front()
            .map_or(state.next_seq, |event| event.seq);
        let missed = oldest.saturating_sub(self.next);
        let events = state
            .events
            .iter()
            .filter(|event| event.seq >= self.next)
            .cloned()
            .collect();
        self.next = state.next_seq;
        EventBatch { missed, events }
    }
}
