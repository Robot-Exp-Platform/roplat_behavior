//! Contract conformance checks for driver authors.
//!
//! Each check drives a device through its contract and reports the first
//! violation as a [`Violation`] naming what was expected. `advance` is how the
//! caller lets time pass between samples: step a simulator, let a mock
//! transport emit the next message, or wait for a real device.
//!
//! The checks cover what the contracts promise for every implementation:
//! readings of one sensor share a clock and a frame, sequence numbers rise
//! and stamps do not go back, values are finite, an IMU orientation is a unit
//! quaternion, a tared static sensor reads zero, a gripper reaches the
//! widths it is sent, an arm's declared joint envelope is coherent, and a
//! real-time control session stops calling back once it is told to.
//! Device-specific behavior (ranges, noise, timing) stays in the driver's
//! own tests.

use std::{fmt, ops::ControlFlow, time::Duration};

use roplat_msgs::{GripperCommand, Header, Imu};

use crate::ControlExit;
use crate::{
    ControlSpace, ControlWith, ForceTorqueSensor, Gripper, ImuSensor, Joints, RobotException,
};

/// A broken contract promise.
#[derive(Debug)]
pub struct Violation {
    /// The check that failed, e.g. `"force_torque: seq"`.
    pub check: &'static str,
    pub detail: String,
}

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.check, self.detail)
    }
}

impl std::error::Error for Violation {}

fn violation(check: &'static str, detail: impl Into<String>) -> Violation {
    Violation { check, detail: detail.into() }
}

fn device(check: &'static str, error: RobotException) -> Violation {
    violation(check, format!("the device returned an error: {error}"))
}

/// Headers of consecutive readings from one stream: one clock, one frame,
/// rising sequence numbers, stamps that do not go back.
pub fn check_header_stream(check: &'static str, headers: &[Header]) -> Result<(), Violation> {
    for pair in headers.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        if a.stamp.clock != b.stamp.clock {
            return Err(violation(
                check,
                format!("clock changed from {} to {}", a.stamp.clock, b.stamp.clock),
            ));
        }
        if a.frame != b.frame {
            return Err(violation(
                check,
                format!("frame changed from {:?} to {:?}", a.frame, b.frame),
            ));
        }
        if b.seq <= a.seq {
            return Err(violation(
                check,
                format!("seq did not rise: {} then {}", a.seq, b.seq),
            ));
        }
        if b.stamp.nanos < a.stamp.nanos {
            return Err(violation(
                check,
                format!(
                    "stamp went back: {} then {} ns",
                    a.stamp.nanos, b.stamp.nanos
                ),
            ));
        }
    }
    Ok(())
}

/// Reads `samples` wrenches with `advance` between them. With
/// `static_load`, the sensor's load must not change while the check runs:
/// if the device supports [`bias_wrench`](ForceTorqueSensor::bias_wrench),
/// the next reading must then be zero within `tolerance` on every axis.
pub fn check_force_torque_sensor<S: ForceTorqueSensor>(
    sensor: &mut S,
    mut advance: impl FnMut(),
    samples: usize,
    static_load: bool,
    tolerance: f64,
) -> Result<(), Violation> {
    const CHECK: &str = "force_torque";
    let mut headers = Vec::with_capacity(samples);
    for _ in 0..samples {
        advance();
        let reading = sensor.read_wrench().map_err(|e| device(CHECK, e))?;
        if reading.value.to_array().iter().any(|v| !v.is_finite()) {
            return Err(violation(
                CHECK,
                format!("non-finite wrench {:?}", reading.value),
            ));
        }
        headers.push(reading.header);
    }
    check_header_stream(CHECK, &headers)?;

    if static_load {
        match sensor.bias_wrench() {
            Err(RobotException::UnprocessableInstructionError(_)) => {}
            Err(e) => return Err(device(CHECK, e)),
            Ok(()) => {
                advance();
                let tared = sensor.read_wrench().map_err(|e| device(CHECK, e))?.value;
                if tared.to_array().iter().any(|v| v.abs() > tolerance) {
                    return Err(violation(
                        "force_torque: bias",
                        format!("a tared static load reads {:?}", tared.to_array()),
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Reads `samples` IMU messages with `advance` between them: header stream,
/// finite present fields, unit orientation, and a mask that never sets bits
/// the message does not define.
pub fn check_imu<S: ImuSensor>(
    sensor: &mut S,
    mut advance: impl FnMut(),
    samples: usize,
) -> Result<(), Violation> {
    const CHECK: &str = "imu";
    let known = Imu::ORIENTATION | Imu::ANGULAR_VELOCITY | Imu::LINEAR_ACCELERATION;
    let mut headers = Vec::with_capacity(samples);
    for _ in 0..samples {
        advance();
        let reading = sensor.read_imu().map_err(|e| device(CHECK, e))?;
        let imu = reading.value;
        if imu.valid_mask() & !known != 0 {
            return Err(violation(
                CHECK,
                format!("unknown mask bits {:#x}", imu.valid_mask()),
            ));
        }
        if let Some(q) = imu.orientation() {
            let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
            if !norm.is_finite() || (norm - 1.0).abs() > 1e-6 {
                return Err(violation(CHECK, format!("orientation norm {norm}")));
            }
        }
        for v in [imu.angular_velocity(), imu.linear_acceleration()]
            .into_iter()
            .flatten()
        {
            if ![v.x, v.y, v.z].iter().all(|c| c.is_finite()) {
                return Err(violation(CHECK, format!("non-finite vector {v:?}")));
            }
        }
        headers.push(reading.header);
    }
    check_header_stream(CHECK, &headers)
}

/// Opens the gripper to each of `widths` (scaled by the reported
/// `max_width`, so `1.0` is fully open) with zero force, and requires the
/// reported width within `tolerance` (m) after each move. `advance` runs
/// after each move, for devices whose state reaches the reader later.
pub fn check_gripper<G: Gripper>(
    gripper: &mut G,
    mut advance: impl FnMut(),
    widths: &[f64],
    speed: f64,
    tolerance: f64,
) -> Result<(), Violation> {
    const CHECK: &str = "gripper";
    let state = gripper.read_gripper().map_err(|e| device(CHECK, e))?;
    if !(state.max_width.is_finite() && state.max_width > 0.0) {
        return Err(violation(CHECK, format!("max_width {}", state.max_width)));
    }
    if !(0.0..=state.max_width + tolerance).contains(&state.width) {
        return Err(violation(
            CHECK,
            format!("width {} outside [0, {}]", state.width, state.max_width),
        ));
    }
    for &fraction in widths {
        let width = fraction.clamp(0.0, 1.0) * state.max_width;
        gripper
            .move_gripper(GripperCommand { width, speed, force: 0.0 })
            .map_err(|e| device(CHECK, e))?;
        advance();
        let reached = gripper.read_gripper().map_err(|e| device(CHECK, e))?.width;
        if (reached - width).abs() > tolerance {
            return Err(violation(
                "gripper: width",
                format!("sent {width} m, reads {reached} m"),
            ));
        }
    }
    Ok(())
}

/// The joint envelope a [`Joints`] implementation declares: each lower limit
/// below its upper limit, the default and packed postures within the limits,
/// and every rate and torque bound positive. Needs no device.
pub fn check_joints<T: Joints<N>, const N: usize>() -> Result<(), Violation> {
    const CHECK: &str = "joints";
    let limits = T::JOINT_LIMITS;
    for i in 0..N {
        let (min, max) = (limits.q_min[i], limits.q_max[i]);
        if min.is_nan() || max.is_nan() || min >= max {
            return Err(violation(
                CHECK,
                format!("joint {i}: min {min} is not below max {max}"),
            ));
        }
        for (what, q) in [
            ("default", T::JOINT_DEFAULT[i]),
            ("packed", T::JOINT_PACKED[i]),
        ] {
            if !(min..=max).contains(&q) {
                return Err(violation(
                    CHECK,
                    format!("joint {i}: {what} posture {q} is outside [{min}, {max}]"),
                ));
            }
        }
        let bounds = [
            ("dq_max", limits.dq_max[i]),
            ("ddq_max", limits.ddq_max[i]),
            ("dddq_max", limits.dddq_max[i]),
            ("tau_max", limits.tau_max[i]),
            ("dtau_max", limits.dtau_max[i]),
        ];
        for (what, bound) in bounds {
            if bound.is_nan() || bound <= 0.0 {
                return Err(violation(
                    CHECK,
                    format!("joint {i}: {what} {bound} is not positive"),
                ));
            }
        }
    }
    Ok(())
}

/// What the `Duration` handed to a control callback measured, as
/// [`check_control_session`] saw it. The contract says the nominal period, or
/// `(k + 1)` periods after `k` missed cycles ([`ControlWith`]): a driver that
/// keeps it shows [`CyclePeriod`](Self::CyclePeriod) on a loss-free session.
///
/// [`ControlWith`]: crate::ControlWith
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DurationPattern {
    /// The same nonzero duration every cycle: the cycle period.
    CyclePeriod,
    /// Zero on the first cycle, then the time since the previous one.
    SincePrevious,
    /// Growing by about one period per cycle: time since the session began.
    SessionTime,
    /// None of these.
    Other,
}

/// What [`check_control_session`] observed beyond its checks.
#[derive(Clone, Debug)]
pub struct SessionReport {
    /// The callback `Duration` of each cycle of the first session.
    pub durations: Vec<Duration>,
    pub pattern: DurationPattern,
    /// The error a session ended by `Break` returned, if any; the contract
    /// leaves the result of `Break` to the driver.
    pub break_error: Option<String>,
}

/// Values within 25 % of each other, all positive.
fn steady(values: &[f64]) -> bool {
    let low = values.iter().copied().fold(f64::INFINITY, f64::min);
    let high = values.iter().copied().fold(0.0, f64::max);
    low > 0.0 && high <= 1.25 * low
}

fn classify(durations: &[Duration]) -> DurationPattern {
    let seconds: Vec<f64> = durations.iter().map(Duration::as_secs_f64).collect();
    let steps: Vec<f64> = seconds.windows(2).map(|w| w[1] - w[0]).collect();
    if steady(&seconds) {
        DurationPattern::CyclePeriod
    } else if seconds[0] == 0.0 && steady(&seconds[1..]) {
        DurationPattern::SincePrevious
    } else if steady(&steps) {
        DurationPattern::SessionTime
    } else {
        DurationPattern::Other
    }
}

/// Runs three blocking control sessions on channel `S`, each answering
/// every cycle with the device's own [`hold_command`](ControlWith::hold_command),
/// so the device only holds where it is:
///
/// 1. `done` on cycle `cycles`: the callback must run exactly `cycles`
///    times and the session return `Ok`;
/// 2. `Break` on cycle `cycles`: the callback must run exactly `cycles`
///    times and the session return (its result is reported, not checked);
/// 3. `done` on the first cycle: a `Break` is not an emergency stop, so the
///    device must accept the next session.
///
/// A callback invoked after `done` or `Break` answers `Break`, so a driver
/// that keeps calling still ends. The report classifies the first session's
/// callback durations. `cycles` must be at least 3. Holding is the gentlest
/// command a channel has, but this is still a live session: on hardware,
/// run it where holding is safe.
pub fn check_control_session<R, S>(robot: &mut R, cycles: usize) -> Result<SessionReport, Violation>
where
    R: ControlWith<S>,
    S: ControlSpace,
{
    assert!(cycles >= 3, "check_control_session needs at least 3 cycles");
    let mut calls = 0;
    let mut durations = Vec::with_capacity(cycles);
    let result = robot.control_with_flow(|obs, dt| {
        calls += 1;
        if calls > cycles {
            return ControlFlow::Break(ControlExit::Stopped);
        }
        durations.push(dt);
        ControlFlow::Continue((R::hold_command(&obs), calls == cycles))
    });
    if calls > cycles {
        return Err(violation(
            "control_session: done",
            format!("done on cycle {cycles}, but the callback ran again"),
        ));
    }
    result.map_err(|e| device("control_session: done", e))?;
    if calls < cycles {
        return Err(violation(
            "control_session: done",
            format!("the session ended after {calls} of {cycles} cycles without done"),
        ));
    }

    let mut calls = 0;
    let result = robot.control_with_flow(|obs, _| {
        calls += 1;
        if calls >= cycles {
            ControlFlow::Break(ControlExit::Stopped)
        } else {
            ControlFlow::Continue((R::hold_command(&obs), false))
        }
    });
    if calls > cycles {
        return Err(violation(
            "control_session: break",
            format!("Break on cycle {cycles}, but the callback ran again"),
        ));
    }
    if calls < cycles {
        return Err(match result {
            Err(e) => device("control_session: break", e),
            Ok(()) => violation(
                "control_session: break",
                format!("the session ended after {calls} of {cycles} cycles"),
            ),
        });
    }
    let break_error = result.err().map(|e| e.to_string());

    let mut calls = 0;
    let result = robot.control_with_flow(|obs, _| {
        calls += 1;
        if calls > 1 {
            return ControlFlow::Break(ControlExit::Stopped);
        }
        ControlFlow::Continue((R::hold_command(&obs), true))
    });
    if calls > 1 {
        return Err(violation(
            "control_session: after break",
            "done on the first cycle, but the callback ran again",
        ));
    }
    result.map_err(|e| device("control_session: after break", e))?;
    if calls == 0 {
        return Err(violation(
            "control_session: after break",
            "the next session returned without a cycle",
        ));
    }

    let pattern = classify(&durations);
    Ok(SessionReport { durations, pattern, break_error })
}
