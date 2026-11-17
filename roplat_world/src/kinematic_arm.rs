//! A simulated arm without dynamics, with a panel: the reference for drivers
//! whose motion is a loop in their own process.

use std::{
    marker::PhantomData,
    sync::{Arc, Condvar, Mutex, MutexGuard},
    time::{Duration, Instant},
};

use roplat_behavior::{
    Actor, HasPanel, JointLimits, JointSample, JointSpace, JointState, Joints, MoveTo, Panel,
    PanelStatus, Release, Robot, RobotException, RobotResult, StateView,
};

/// An `N`-joint arm whose joints move straight toward their target at the
/// velocity limits model `M` declares, in real time. No dynamics and no
/// collisions: it stands in for a device where the [`Panel`] contract, not
/// the motion, is under test, such as a service's stop path.
///
/// It starts enabled, with nothing holding it. [`Hardware`] plays what the
/// device does by itself: its emergency-stop button and its protective stop.
///
/// # How its panel stops a motion
///
/// The way a driver whose motion is a loop in its own process does (servo
/// streaming, a fieldbus cycle): the motion steps once per
/// [`CONTROL_PERIOD`](Robot::CONTROL_PERIOD) and, before every step, compares
/// a stop counter with the value it started with. Every stop raises the
/// counter and wakes the loop, so a stop ends the motion at once, with the
/// joints where they are; a real arm would decelerate first and declare a
/// longer [`stop_bound`](Panel::stop_bound).
pub struct KinematicArm<M, const N: usize> {
    device: Arc<Device<N>>,
    model: PhantomData<fn() -> M>,
}

/// What the panel, the hardware and the motion share.
///
/// A mutex and a condition variable: the motion waits between steps where a
/// stop can wake it, and panel calls are rare.
struct Device<const N: usize> {
    state: Mutex<DeviceState<N>>,
    changed: Condvar,
}

struct DeviceState<const N: usize> {
    q: [f64; N],
    enabled: bool,
    paused: bool,
    /// Rises with every stop; a motion ends when it differs from its start.
    stops: u64,
    software_estop: bool,
    device_estop: Option<Release>,
    protective_stop: Option<Release>,
}

impl<const N: usize> DeviceState<N> {
    fn status(&self) -> PanelStatus {
        PanelStatus {
            enabled: self.enabled,
            paused: self.paused,
            software_estop: self.software_estop,
            device_estop: self.device_estop,
            protective_stop: self.protective_stop,
        }
    }
}

impl<const N: usize> Device<N> {
    fn lock(&self) -> MutexGuard<'_, DeviceState<N>> {
        // The state is a handful of plain values, valid after any panic.
        self.state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Changes the state and wakes a waiting motion.
    fn change<T>(&self, f: impl FnOnce(&mut DeviceState<N>) -> T) -> T {
        let value = f(&mut self.lock());
        self.changed.notify_all();
        value
    }
}

/// How long the arm's panel promises a stop takes. The loop ends within one
/// step of a stop; the rest is room for the scheduler of a loaded machine.
const STOP_BOUND: Duration = Duration::from_millis(100);

impl<M, const N: usize> KinematicArm<M, N> {
    /// An arm at joint positions `q`, enabled and ready. `q` is taken as
    /// given, not checked against the model's limits.
    pub fn new(q: [f64; N]) -> Self {
        let state = DeviceState {
            q,
            enabled: true,
            paused: false,
            stops: 0,
            software_estop: false,
            device_estop: None,
            protective_stop: None,
        };
        Self {
            device: Arc::new(Device { state: Mutex::new(state), changed: Condvar::new() }),
            model: PhantomData,
        }
    }

    /// The joint positions now (rad).
    pub fn joints(&self) -> [f64; N] {
        self.device.lock().q
    }

    /// The device's own side: its button and its protective stop.
    pub fn hardware(&self) -> Hardware<N> {
        Hardware { device: Arc::clone(&self.device) }
    }
}

impl<M, const N: usize> Robot for KinematicArm<M, N> {
    type State = JointState<N>;

    /// One motion step.
    const CONTROL_PERIOD: Duration = Duration::from_millis(2);

    fn version() -> String {
        "roplat_world kinematic arm".into()
    }

    /// Measured positions only; the arm has no velocity or effort sensing.
    fn read_state(&mut self) -> RobotResult<JointState<N>> {
        Ok(StateView::from_meas(
            JointSample::EMPTY.with_q(self.joints()),
        ))
    }
}

impl<M: Joints<N>, const N: usize> Joints<N> for KinematicArm<M, N> {
    const JOINT_DEFAULT: [f64; N] = M::JOINT_DEFAULT;
    const JOINT_PACKED: [f64; N] = M::JOINT_PACKED;
    const JOINT_LIMITS: JointLimits<N> = M::JOINT_LIMITS;
}

impl<M: Joints<N>, const N: usize> MoveTo<JointSpace<N>> for KinematicArm<M, N> {
    /// Moves every joint toward `target` at its `dq_max`, so the joints
    /// arrive one by one (an infinite `dq_max` arrives in the first step).
    /// A target outside the position limits is refused with
    /// `UnprocessableInstructionError`; a stop ends the motion with
    /// `Stopped`, and while one holds the arm or it is paused or disabled the
    /// motion does not start.
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        let limits = M::JOINT_LIMITS;
        for (i, &q) in target.iter().enumerate() {
            if !(limits.q_min[i]..=limits.q_max[i]).contains(&q) {
                return Err(RobotException::UnprocessableInstructionError(format!(
                    "joint {i}: target {q} outside [{}, {}]",
                    limits.q_min[i], limits.q_max[i]
                )));
            }
        }
        let mut state = self.device.lock();
        if !state.status().ready() {
            return Err(RobotException::Stopped);
        }
        let start = state.stops;
        let mut last = Instant::now();
        loop {
            if state.stops != start {
                return Err(RobotException::Stopped);
            }
            let now = Instant::now();
            let dt = (now - last).as_secs_f64();
            last = now;
            if !state.paused {
                let mut arrived = true;
                for (i, &goal) in target.iter().enumerate() {
                    let step = limits.dq_max[i] * dt;
                    let error = goal - state.q[i];
                    if error.abs() <= step {
                        state.q[i] = goal;
                    } else {
                        state.q[i] += step.copysign(error);
                        arrived = false;
                    }
                }
                if arrived {
                    return Ok(());
                }
            }
            state = match self
                .device
                .changed
                .wait_timeout(state, Self::CONTROL_PERIOD)
            {
                Ok((state, _)) => state,
                Err(poisoned) => poisoned.into_inner().0,
            };
        }
    }
}

impl<M: 'static, const N: usize> HasPanel for KinematicArm<M, N> {
    fn panel(&self) -> Arc<dyn Panel> {
        Arc::new(ArmPanel { device: Arc::clone(&self.device) })
    }
}

/// The arm's panel. Every operation is supported; `by` is not used, as the
/// contract leaves who may to the service.
struct ArmPanel<const N: usize> {
    device: Arc<Device<N>>,
}

impl<const N: usize> Panel for ArmPanel<N> {
    fn stop(&self, _by: &Actor) -> RobotResult<()> {
        self.device.change(|state| state.stops += 1);
        Ok(())
    }

    fn emergency_stop(&self, _by: &Actor) -> RobotResult<()> {
        self.device.change(|state| {
            state.software_estop = true;
            state.stops += 1;
        });
        Ok(())
    }

    fn pause(&self, _by: &Actor) -> RobotResult<()> {
        self.device.change(|state| state.paused = true);
        Ok(())
    }

    fn disable(&self, _by: &Actor) -> RobotResult<()> {
        self.device.change(|state| {
            state.enabled = false;
            state.stops += 1;
        });
        Ok(())
    }

    fn clear(&self, _by: &Actor) -> RobotResult<PanelStatus> {
        let status = self.device.change(|state| {
            state.software_estop = false;
            if state.device_estop == Some(Release::Clear) {
                state.device_estop = None;
            }
            if state.protective_stop == Some(Release::Clear) {
                state.protective_stop = None;
            }
            state.status()
        });
        if status.device_estop.is_some() || status.protective_stop.is_some() {
            return Err(RobotException::LocalResetRequired);
        }
        Ok(status)
    }

    fn enable(&self, _by: &Actor) -> RobotResult<()> {
        self.device.change(|state| {
            if state.status().held() {
                return Err(RobotException::Stopped);
            }
            state.enabled = true;
            Ok(())
        })
    }

    fn resume(&self, _by: &Actor) -> RobotResult<()> {
        self.device.change(|state| {
            if state.status().held() {
                return Err(RobotException::Stopped);
            }
            state.paused = false;
            Ok(())
        })
    }

    fn status(&self) -> RobotResult<PanelStatus> {
        Ok(self.device.lock().status())
    }

    fn stop_bound(&self) -> Duration {
        STOP_BOUND
    }
}

/// What a [`KinematicArm`] does by itself, played from outside: the
/// device's own stops, which software did not ask for.
#[derive(Clone)]
pub struct Hardware<const N: usize> {
    device: Arc<Device<N>>,
}

impl<const N: usize> Hardware<N> {
    /// Presses (`true`) or releases (`false`) the arm's emergency-stop
    /// button. Pressing ends a motion in progress and cuts the actuators;
    /// while it is pressed only an on-site reset releases the stop
    /// ([`Release::OnSite`]). Releasing the button is that reset, and leaves
    /// a stop that [`Panel::clear`] releases, as on arms whose controller is
    /// told in software that the button is out again.
    pub fn estop_button(&self, pressed: bool) {
        self.device.change(|state| {
            if pressed {
                state.device_estop = Some(Release::OnSite);
                state.enabled = false;
                state.stops += 1;
            } else if state.device_estop.is_some() {
                state.device_estop = Some(Release::Clear);
            }
        });
    }

    /// The arm stops itself, as after a collision: a motion in progress
    /// ends, and [`Panel::clear`] releases the stop. The actuators stay
    /// engaged.
    pub fn protective_stop(&self) {
        self.device.change(|state| {
            state.protective_stop = Some(Release::Clear);
            state.stops += 1;
        });
    }
}
