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
//! quaternion, an IMU at rest reads gravity up in its world frame, a lidar
//! leaves out rays without a return and stamps a scan
//! with its earliest return, a GNSS fix has a position exactly when it has a
//! solution, a motion-capture frame reports each body once, a multirotor's
//! device time does not go back, a tared static sensor reads zero, a gripper
//! reaches the widths it is sent, an arm's declared joint envelope is
//! coherent, and a real-time control session stops calling back once it is
//! told to.
//! Device-specific behavior (ranges, noise, timing) stays in the driver's
//! own tests.

use std::{fmt, ops::ControlFlow, time::Duration};

use nalgebra as na;
use roplat_msgs::{GripperCommand, Header, Imu};

use crate::ControlExit;
use crate::{
    ControlSpace, ControlWith, ForceTorqueSensor, GnssFix, GnssReceiver, GnssStatus, Gripper,
    ImuSensor, Joints, Lidar, MotionCapture, Multirotor, RobotException,
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
/// a mask that never sets bits the message does not define, finite present
/// fields, and a unit orientation. These hold in every scene; whether the
/// readings keep the axes of [`ImuSensor`] needs a known scene, so the
/// driver's own simulation tests check them.
pub fn check_imu<S: ImuSensor>(
    sensor: &mut S,
    mut advance: impl FnMut(),
    samples: usize,
) -> Result<(), Violation> {
    const CHECK: &str = "imu";
    let mut headers = Vec::with_capacity(samples);
    for _ in 0..samples {
        advance();
        let reading = sensor.read_imu().map_err(|e| device(CHECK, e))?;
        check_imu_reading(CHECK, &reading.value)?;
        headers.push(reading.header);
    }
    check_header_stream(CHECK, &headers)
}

/// What one IMU reading keeps in every scene: known mask bits, finite
/// present fields, a unit orientation.
fn check_imu_reading(check: &'static str, imu: &Imu) -> Result<(), Violation> {
    let known = Imu::ORIENTATION | Imu::ANGULAR_VELOCITY | Imu::LINEAR_ACCELERATION;
    if imu.valid_mask() & !known != 0 {
        return Err(violation(
            check,
            format!("unknown mask bits {:#x}", imu.valid_mask()),
        ));
    }
    if let Some(q) = imu.orientation() {
        if !q.to_xyzw().iter().all(|c| c.is_finite()) {
            return Err(violation(check, format!("non-finite orientation {q:?}")));
        }
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        if (norm - 1.0).abs() > 1e-6 {
            return Err(violation(check, format!("orientation norm {norm}")));
        }
    }
    for v in [imu.angular_velocity(), imu.linear_acceleration()]
        .into_iter()
        .flatten()
    {
        if ![v.x, v.y, v.z].iter().all(|c| c.is_finite()) {
            return Err(violation(check, format!("non-finite vector {v:?}")));
        }
    }
    Ok(())
}

/// The scene of [`check_imu_at_rest`]: a device held still, in a pose nobody
/// needs to know, under gravity of magnitude `gravity`, and how far its
/// readings may stray from what such a device reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AtRest {
    /// m/s²: about 9.78 to 9.83 on Earth, the world's setting in a simulator.
    pub gravity: f64,
    /// m/s²: the largest distance between a specific force and the one at
    /// rest, noise and bias included.
    pub acceleration_tolerance: f64,
    /// rad/s: the largest angular speed a reading may show, noise, bias and
    /// the Earth's rotation (7.3e-5 rad/s) included.
    pub angular_velocity_tolerance: f64,
}

/// Reads `samples` IMU messages from a device at rest in any pose, with
/// `advance` between them: everything [`check_imu`] checks, an angular speed
/// within `rest.angular_velocity_tolerance`, and the specific force of a
/// body at rest. With an orientation, the specific force turned into the
/// world frame must be `(0, 0, +gravity)` within
/// `rest.acceleration_tolerance`; without one, only its magnitude must be
/// `gravity`. Readings without a linear acceleration are skipped, but at
/// least one must carry it.
///
/// Since the pose is unknown, this checks the world frame, the sign of
/// gravity and the units: it catches gravity removed or reversed, an
/// acceleration left in g, an orientation relative to NED, and an
/// acceleration and an orientation converted differently. It does not check
/// the body axes: a body frame turned the same way in the acceleration and
/// the orientation still passes, as does any body frame without an
/// orientation, so the driver's known-scene tests check them. A level pose
/// hides an orientation left FRD relative to NED next to a converted
/// acceleration, so hold the device pitched and rolled.
pub fn check_imu_at_rest<S: ImuSensor>(
    sensor: &mut S,
    mut advance: impl FnMut(),
    samples: usize,
    rest: AtRest,
) -> Result<(), Violation> {
    const CHECK: &str = "imu";
    const GRAVITY: &str = "imu: gravity";
    let up = na::Vector3::new(0.0, 0.0, rest.gravity);
    let mut headers = Vec::with_capacity(samples);
    let mut accelerations = 0;
    for _ in 0..samples {
        advance();
        let reading = sensor.read_imu().map_err(|e| device(CHECK, e))?;
        let imu = reading.value;
        check_imu_reading(CHECK, &imu)?;
        if let Some(w) = imu.angular_velocity() {
            let speed = na::Vector3::from(w).norm();
            if speed > rest.angular_velocity_tolerance {
                return Err(violation(
                    "imu: angular velocity",
                    format!("a device at rest turns at {speed} rad/s"),
                ));
            }
        }
        if let Some(a) = imu.linear_acceleration() {
            accelerations += 1;
            let a = na::Vector3::from(a);
            if let Some(q) = imu.orientation() {
                let world = na::UnitQuaternion::from(q) * a;
                if (world - up).norm() > rest.acceleration_tolerance {
                    return Err(violation(
                        GRAVITY,
                        format!(
                            "at rest the specific force in the world frame is {:?} m/s², \
                             not about [0, 0, {}]",
                            [world.x, world.y, world.z],
                            rest.gravity
                        ),
                    ));
                }
            } else if (a.norm() - rest.gravity).abs() > rest.acceleration_tolerance {
                return Err(violation(
                    GRAVITY,
                    format!(
                        "at rest the specific force is {} m/s², not about {}",
                        a.norm(),
                        rest.gravity
                    ),
                ));
            }
        }
        headers.push(reading.header);
    }
    if accelerations == 0 {
        return Err(violation(
            GRAVITY,
            "no reading carried a linear acceleration",
        ));
    }
    check_header_stream(CHECK, &headers)
}

/// Reads `scans` scans into one reused buffer with `advance` between them:
/// header stream, finite positions and intensities, no point at the origin
/// (the marker some devices send for a ray without a return), and an earliest
/// return at offset 0. The lidar frame's axes need a known scene, so the
/// driver's own simulation tests check them.
pub fn check_lidar<L: Lidar>(
    lidar: &mut L,
    mut advance: impl FnMut(),
    scans: usize,
) -> Result<(), Violation> {
    const CHECK: &str = "lidar";
    let mut points = Vec::new();
    let mut headers = Vec::with_capacity(scans);
    for _ in 0..scans {
        advance();
        let header = lidar.read_scan(&mut points).map_err(|e| device(CHECK, e))?;
        for p in &points {
            if !p.position.iter().all(|c| c.is_finite())
                || p.intensity.is_some_and(|i| !i.is_finite())
            {
                return Err(violation(CHECK, format!("non-finite point {p:?}")));
            }
            if p.position == [0.0; 3] {
                return Err(violation(
                    "lidar: no return",
                    format!("a point at the origin, {} ns into the scan", p.offset_ns),
                ));
            }
        }
        if let Some(earliest) = points.iter().map(|p| p.offset_ns).min()
            && earliest != 0
        {
            return Err(violation(
                "lidar: stamp",
                format!("the earliest return is {earliest} ns after the stamp"),
            ));
        }
        headers.push(header);
    }
    check_header_stream(CHECK, &headers)
}

/// Reads `fixes` fixes with `advance` between them: header stream, finite
/// values, latitude in `[-90, 90]` and longitude in `[-180, 180]`, a position
/// exactly when the status is not `NoFix`, uncertainties and a correction age
/// that are not negative, and a time of week below one week. Where the
/// antenna really is needs a known scene, so the driver's own tests check it.
pub fn check_gnss<G: GnssReceiver>(
    receiver: &mut G,
    mut advance: impl FnMut(),
    fixes: usize,
) -> Result<(), Violation> {
    const CHECK: &str = "gnss";
    let mut headers = Vec::with_capacity(fixes);
    for _ in 0..fixes {
        advance();
        let reading = receiver.read_fix().map_err(|e| device(CHECK, e))?;
        check_gnss_fix(CHECK, &reading.value)?;
        headers.push(reading.header);
    }
    check_header_stream(CHECK, &headers)
}

fn check_gnss_fix(check: &'static str, fix: &GnssFix) -> Result<(), Violation> {
    const WEEK_NS: u64 = 604_800 * 1_000_000_000;
    match (fix.status, fix.position) {
        (GnssStatus::NoFix, Some(p)) => {
            return Err(violation(
                "gnss: no fix",
                format!("a NoFix epoch carries the position {p:?}"),
            ));
        }
        (status, None) if status != GnssStatus::NoFix => {
            return Err(violation(
                "gnss: no fix",
                format!("a {status:?} fix carries no position"),
            ));
        }
        _ => {}
    }
    if let Some(p) = fix.position
        && !(p.ellipsoidal_height.is_finite()
            && (-90.0..=90.0).contains(&p.latitude)
            && (-180.0..=180.0).contains(&p.longitude))
    {
        return Err(violation(
            check,
            format!("position {p:?} is not finite or out of range"),
        ));
    }
    if let Some(u) = fix.undulation
        && !u.is_finite()
    {
        return Err(violation(check, format!("non-finite undulation {u}")));
    }
    if let Some(v) = fix.velocity_enu
        && !v.iter().all(|c| c.is_finite())
    {
        return Err(violation(check, format!("non-finite velocity {v:?}")));
    }
    if let Some(s) = fix.sigma_enu
        && !s.iter().all(|c| c.is_finite() && *c >= 0.0)
    {
        return Err(violation(
            check,
            format!("sigma {s:?} is not finite and non-negative"),
        ));
    }
    if let Some(age) = fix.correction_age
        && !(age.is_finite() && age >= 0.0)
    {
        return Err(violation(check, format!("correction age {age} s")));
    }
    if let Some(time) = fix.gps_time
        && time.tow_ns >= WEEK_NS
    {
        return Err(violation(
            check,
            format!("time of week {} ns is not below one week", time.tow_ns),
        ));
    }
    Ok(())
}

/// Reads `frames` frames into one reused buffer with `advance` between them:
/// header stream, each body at most once per frame, a finite position and a
/// unit orientation for every tracked body, and marker errors that are finite
/// and not negative. The capture world's axes need a known scene, so the
/// driver's own simulation tests check them.
pub fn check_motion_capture<M: MotionCapture>(
    mocap: &mut M,
    mut advance: impl FnMut(),
    frames: usize,
) -> Result<(), Violation> {
    const CHECK: &str = "motion_capture";
    let mut bodies = Vec::new();
    let mut ids = Vec::new();
    let mut headers = Vec::with_capacity(frames);
    for _ in 0..frames {
        advance();
        let header = mocap
            .read_frame(&mut bodies)
            .map_err(|e| device(CHECK, e))?;
        ids.clear();
        for body in &bodies {
            if ids.contains(&body.id) {
                return Err(violation(
                    "motion_capture: id",
                    format!("body {} appears twice in one frame", body.id),
                ));
            }
            ids.push(body.id);
            if let Some(pose) = body.pose {
                let q = pose.orientation;
                let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
                if !pose.position().iter().all(|c| c.is_finite())
                    || !norm.is_finite()
                    || (norm - 1.0).abs() > 1e-6
                {
                    return Err(violation(
                        CHECK,
                        format!("body {}: pose {pose:?} is not finite or not unit", body.id),
                    ));
                }
            }
            if let Some(error) = body.error
                && !(error.is_finite() && error >= 0.0)
            {
                return Err(violation(
                    CHECK,
                    format!("body {}: marker error {error} m", body.id),
                ));
            }
        }
        headers.push(header);
    }
    check_header_stream(CHECK, &headers)
}

/// Reads `samples` flight states with `advance` between them, without
/// flying: header stream, device times that do not go back, finite present
/// values and a unit orientation. Flying and the world's axes need a scene,
/// so the driver's own simulation tests check them.
pub fn check_flight_state<M: Multirotor>(
    vehicle: &mut M,
    mut advance: impl FnMut(),
    samples: usize,
) -> Result<(), Violation> {
    const CHECK: &str = "flight_state";
    let mut headers = Vec::with_capacity(samples);
    let mut last_device_time = None;
    for _ in 0..samples {
        advance();
        let reading = vehicle.read_flight_state().map_err(|e| device(CHECK, e))?;
        let state = reading.value;
        if let Some(last) = last_device_time
            && state.device_time < last
        {
            return Err(violation(
                "flight_state: device time",
                format!(
                    "device time went back: {last:?} then {:?}",
                    state.device_time
                ),
            ));
        }
        last_device_time = Some(state.device_time);
        let vectors = [Some(state.position), state.velocity, state.angular_velocity];
        for v in vectors.into_iter().flatten() {
            if ![v.x, v.y, v.z].iter().all(|c| c.is_finite()) {
                return Err(violation(CHECK, format!("non-finite vector {v:?}")));
            }
        }
        if let Some(q) = state.orientation {
            let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
            if !norm.is_finite() || (norm - 1.0).abs() > 1e-6 {
                return Err(violation(CHECK, format!("orientation {q:?} is not unit")));
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
