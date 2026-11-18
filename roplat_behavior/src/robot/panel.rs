//! The panel: a device's stops, enabling and status, apart from its motion.

use std::{sync::Arc, time::Duration};

use serde::{Deserialize, Serialize};

use crate::{Robot, RobotException, RobotResult};

/// Who asked for a panel operation.
///
/// The service that hands out a panel names its callers; the panel itself
/// does not check the name (see [`Panel`], "Who may").
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Actor {
    /// The device itself: its own emergency-stop circuit, a protective stop,
    /// a fault, or anything else that changed the panel's status without an
    /// operation through it.
    Device,
    /// A person, e.g. at a web console or a teach pendant program.
    Operator(String),
    /// An AI agent.
    Agent(String),
    /// A program that is neither, e.g. a supervisor loop or a test.
    App(String),
}

/// How a stop that holds the device is released.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Release {
    /// [`Panel::clear`] releases it.
    Clear,
    /// Only a reset at the device releases it: a pressed emergency-stop
    /// button, an open safety circuit. `clear` answers
    /// [`LocalResetRequired`](RobotException::LocalResetRequired) while it
    /// holds.
    OnSite,
}

/// What holds a device, and whether it can move.
///
/// A device that cannot tell one of these leaves it at its default (`false`
/// or `None`) and documents so; the fields are what the device reports, not
/// what the driver assumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub struct PanelStatus {
    /// The actuators are engaged ([`Panel::enable`]); with nothing below
    /// holding the device, motion calls work.
    pub enabled: bool,
    /// A motion is held by [`Panel::pause`] until [`Panel::resume`].
    pub paused: bool,
    /// A software emergency stop is latched: one asked for through
    /// [`Panel::emergency_stop`], by anyone. `clear` releases it.
    pub software_estop: bool,
    /// The device's own emergency stop holds it: its button, its safety
    /// controller. Usually only an on-site reset releases it.
    pub device_estop: Option<Release>,
    /// The device stopped itself to protect something: a collision, a
    /// safeguard input, a limit.
    pub protective_stop: Option<Release>,
}

impl PanelStatus {
    /// Whether any stop holds the device.
    pub fn held(&self) -> bool {
        self.software_estop || self.device_estop.is_some() || self.protective_stop.is_some()
    }

    /// Whether a motion call would run: enabled, not paused, nothing holding.
    pub fn ready(&self) -> bool {
        self.enabled && !self.paused && !self.held()
    }
}

/// The stops, enabling and status of one device, callable from any thread
/// while the device's control handle is inside a motion.
///
/// A driver's value ([`Robot`]) is the control handle: it moves the device and
/// needs `&mut self`, so it has one owner. The panel is the other handle,
/// issued by [`HasPanel::panel`]: every method takes `&self`, so any number of
/// holders can stop the device while it moves, and services keep the panels of
/// different devices as `Arc<dyn Panel>`.
///
/// # Stopping
///
/// - **Safety direction** ([`stop`](Panel::stop),
///   [`emergency_stop`](Panel::emergency_stop), [`pause`](Panel::pause),
///   [`disable`](Panel::disable)): a motion in progress on the control handle
///   ([`MoveTo`](crate::MoveTo), [`MoveTraj`](crate::MoveTraj), a control
///   session) returns [`Stopped`](RobotException::Stopped) within
///   [`stop_bound`](Panel::stop_bound) of the call, with the device at rest,
///   except that `pause` holds the motion instead of ending it.
/// - **Recovery direction** ([`clear`](Panel::clear),
///   [`enable`](Panel::enable), [`resume`](Panel::resume)): `clear` releases
///   every stop software may release, whoever or whatever caused it; a stop
///   that needs a reset at the device stays and `clear` says so.
///
/// A call returns once the device has the request; it does not wait for the
/// device to come to rest. While a stop holds the device (see
/// [`PanelStatus::held`]), motion calls and `enable` fail without moving it,
/// with `Stopped` where the driver can tell.
///
/// # Who may
///
/// The driver answers whether the device *can* do an operation, never who
/// *may*. Anyone holding a panel can call every method; a service that hands
/// out panels decides who gets one and which recovery operations it forwards
/// (for example, an agent may clear a stop an operator latched in software,
/// but not one that needs an on-site reset, which no software can clear).
/// [`LoggedPanel`](crate::LoggedPanel) records who did what.
///
/// # Defaults
///
/// `stop`, `emergency_stop`, `clear`, `status` and `stop_bound` are required.
/// `pause`, `resume`, `enable` and `disable` default to
/// [`NotSupported`](RobotException::NotSupported): a default `Ok` would claim
/// an effect the device may not have (an `enable` that succeeds while an
/// emergency stop is latched, for one).
pub trait Panel: Send + Sync + 'static {
    /// Ends the motion in progress: it returns `Stopped` within
    /// [`stop_bound`](Panel::stop_bound) and the device comes to rest. Nothing
    /// is latched: the next motion call runs normally. With no motion in
    /// progress this does nothing and answers `Ok`.
    fn stop(&self, by: &Actor) -> RobotResult<()>;

    /// The strongest stop the device allows, latched: the motion in progress
    /// ends as for [`stop`](Panel::stop), and motion calls and `enable` fail
    /// until [`clear`](Panel::clear). [`PanelStatus::software_estop`] shows the
    /// latch.
    ///
    /// Returns `Ok` only when the stop is latched. A device that cannot latch
    /// one stops as strongly as it can and returns
    /// [`NoEmergencyStop`](RobotException::NoEmergencyStop); it never does less
    /// without saying so.
    fn emergency_stop(&self, by: &Actor) -> RobotResult<()>;

    /// Holds the motion in progress where it is; [`resume`](Panel::resume)
    /// continues it, a stop ends it.
    fn pause(&self, by: &Actor) -> RobotResult<()> {
        let _ = by;
        Err(RobotException::NotSupported { operation: "pause" })
    }

    /// Disengages the actuators, possibly cutting their power with the brakes
    /// engaged. The motion in progress ends as for [`stop`](Panel::stop);
    /// [`enable`](Panel::enable) engages them again.
    fn disable(&self, by: &Actor) -> RobotResult<()> {
        let _ = by;
        Err(RobotException::NotSupported { operation: "disable" })
    }

    /// Releases every stop software may release: the software emergency stop
    /// and the device's own stops whose release is [`Release::Clear`], whoever
    /// or whatever caused them. It does not enable the device.
    ///
    /// Returns the status after clearing, which says what is left (the device
    /// may still need [`enable`](Panel::enable)). While a stop that needs an
    /// on-site reset holds the device, the others are still cleared and this
    /// answers [`LocalResetRequired`](RobotException::LocalResetRequired),
    /// never `Ok`.
    fn clear(&self, by: &Actor) -> RobotResult<PanelStatus>;

    /// Engages the actuators so that motion calls work. Fails while a stop
    /// holds the device.
    fn enable(&self, by: &Actor) -> RobotResult<()> {
        let _ = by;
        Err(RobotException::NotSupported { operation: "enable" })
    }

    /// Continues a motion held by [`pause`](Panel::pause).
    fn resume(&self, by: &Actor) -> RobotResult<()> {
        let _ = by;
        Err(RobotException::NotSupported { operation: "resume" })
    }

    /// What holds the device now, read from the device.
    fn status(&self) -> RobotResult<PanelStatus>;

    /// The longest time from a safety-direction call to the motion in
    /// progress having returned with the device at rest. A driver declares
    /// what its device needs (deceleration, a polling period, a reply it waits
    /// for); `conformance::check_panel` (feature `conformance`) holds it to it.
    fn stop_bound(&self) -> Duration;
}

/// A device that issues a [`Panel`].
///
/// The panel acts on the same device as `self` and stays valid while `self`
/// moves; `panel` may be called any number of times, and every panel it
/// returns is one more handle on the same device.
pub trait HasPanel: Robot {
    /// A panel for this device.
    // dyn(c): services keep the panels of different devices in one collection.
    fn panel(&self) -> Arc<dyn Panel>;
}
