//! Teleoperation: operator input to a velocity command.
//!
//! [`axes_to_velocity`] maps one [`InputAxes`] sample (a 3D mouse or a
//! gamepad, axes in `[-1, 1]`) to `[vx, vy, vz, wx, wy, wz]`.
//! [`cartesian_teleop_control`] wraps it as a controller for
//! `CartesianVelocityControl` or `BaseVelocityControl`, adding what an
//! operator device needs on a real robot: a dead-man button, zero command when
//! input stops arriving, and an acceleration limit.
//!
//! [`joint_jog_control`] is the joint-space counterpart for
//! `JointVelocityControl`: [`JogInput`] samples, one value per joint, become
//! joint velocities ([`JogMapping`]) that keep each joint's acceleration
//! limit and slow down before its position limits, so that a joint stops
//! short of them.

use std::time::Duration;

use roplat_behavior::{JointState, msgs::InputAxes};

/// How operator input becomes a velocity command.
#[derive(Clone, Copy, Debug)]
pub struct TeleopMapping {
    /// Axis magnitude read as zero (stick drift); the rest of the range is
    /// rescaled so the command starts from zero at the edge of the deadzone.
    pub deadzone: f64,
    /// Speed at full deflection of axes 0–2 (m/s).
    pub linear_speed: f64,
    /// Speed at full deflection of axes 3–5 (rad/s).
    pub angular_speed: f64,
    /// Per-axis sign, to match how the device is mounted.
    pub signs: [f64; 6],
    /// Button that must be held for any motion (dead-man switch); `None`
    /// moves whenever the axes are deflected.
    pub enable_button: Option<u32>,
    /// With no new sample for this long, the command decays to zero.
    pub input_timeout: Duration,
    /// Largest change of the linear (m/s²) and angular (rad/s²) command per
    /// second; `f64::INFINITY` disables the limit.
    pub max_linear_accel: f64,
    pub max_angular_accel: f64,
}

impl Default for TeleopMapping {
    fn default() -> Self {
        Self {
            deadzone: 0.1,
            linear_speed: 0.1,
            angular_speed: 0.5,
            signs: [1.0; 6],
            enable_button: None,
            input_timeout: Duration::from_millis(200),
            max_linear_accel: 0.5,
            max_angular_accel: 2.5,
        }
    }
}

/// The velocity one sample asks for. Out-of-range axes are clamped to
/// `[-1, 1]`, a non-finite axis reads as zero, and an unpressed enable button
/// gives zero.
pub fn axes_to_velocity(input: &InputAxes, mapping: &TeleopMapping) -> [f64; 6] {
    if let Some(button) = mapping.enable_button
        && (button >= 32 || input.buttons & (1 << button) == 0)
    {
        return [0.0; 6];
    }
    let span = (1.0 - mapping.deadzone).max(f64::EPSILON);
    std::array::from_fn(|i| {
        let a = input.axes[i];
        let a = if a.is_finite() {
            a.clamp(-1.0, 1.0)
        } else {
            0.0
        };
        let shaped = (a.abs() - mapping.deadzone).max(0.0) / span * a.signum();
        let speed = if i < 3 {
            mapping.linear_speed
        } else {
            mapping.angular_speed
        };
        mapping.signs[i] * shaped * speed
    })
}

/// A velocity controller following the operator. `input` is polled once per
/// cycle and returns the newest sample, or `None` if none arrived since the
/// last poll. The command follows [`axes_to_velocity`] within the
/// acceleration limits, and returns to zero (within the same limits) after
/// `input_timeout` without samples. `done` reports `true` once `stop`
/// returns `true` and the command has reached zero.
pub fn cartesian_teleop_control<S, F, D>(
    mut input: F,
    mapping: TeleopMapping,
    mut stop: D,
) -> impl FnMut(S, Duration) -> ([f64; 6], bool)
where
    F: FnMut() -> Option<InputAxes>,
    D: FnMut() -> bool,
{
    let mut command = [0.0; 6];
    let mut target = [0.0; 6];
    let mut since_input = Duration::MAX;
    move |_state, period| {
        match input() {
            Some(sample) => {
                target = axes_to_velocity(&sample, &mapping);
                since_input = Duration::ZERO;
            }
            None => since_input = since_input.saturating_add(period),
        }
        let stopping = stop();
        if stopping || since_input > mapping.input_timeout {
            target = [0.0; 6];
        }
        let dt = period.as_secs_f64();
        for i in 0..6 {
            let accel = if i < 3 {
                mapping.max_linear_accel
            } else {
                mapping.max_angular_accel
            };
            let step = accel * dt;
            command[i] += (target[i] - command[i]).clamp(-step, step);
        }
        (command, stopping && command.iter().all(|v| *v == 0.0))
    }
}

// Joint jogging, moved here from roplat_web's teleoperation (roplat_behavior
// 0.10), where it served a browser's jog channel.

/// One jog sample of `N` joints: each value in `[-1, 1]` of the joint's jog
/// speed, and the input device's buttons.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JogInput<const N: usize> {
    pub values: [f64; N],
    pub buttons: u32,
}

/// How jog input becomes joint velocities.
///
/// `JogMapping { q_min, q_max, ..JogMapping::new(speeds, accelerations,
/// enable_button) }` adds a position envelope, for example a model's
/// `JOINT_LIMITS` or a tighter one for the task.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JogMapping<const N: usize> {
    /// Input magnitude read as zero; the rest of the range is rescaled so
    /// the command starts from zero at the edge of the deadzone.
    pub deadzone: f64,
    /// Each joint's speed at full deflection (rad/s, or m/s for a prismatic
    /// joint).
    pub speeds: [f64; N],
    /// Largest change of each joint's command per second.
    pub accelerations: [f64; N],
    /// The jog stops `limit_margin` inside these positions: the speed
    /// toward a limit is capped so the joint can stop there at its
    /// acceleration (`v ≤ √(2 a d)`), and is zero beyond it; moving away
    /// is always allowed.
    pub q_min: [f64; N],
    pub q_max: [f64; N],
    pub limit_margin: f64,
    /// Button that must be held for any motion (dead-man switch); `None`
    /// moves whenever the input is deflected.
    pub enable_button: Option<u32>,
    /// With no new sample for this long, the command decays to zero.
    pub input_timeout: Duration,
}

impl<const N: usize> JogMapping<N> {
    /// Jogging at `speeds` with `accelerations`, gated by `enable_button`
    /// (the input device's dead-man switch; `None` for none), with no
    /// position envelope, a 0.1 deadzone, a 0.02 margin and a 200 ms input
    /// timeout. The button is not defaulted: which bit is the dead-man
    /// switch belongs to the input device.
    pub fn new(speeds: [f64; N], accelerations: [f64; N], enable_button: Option<u32>) -> Self {
        Self {
            deadzone: 0.1,
            speeds,
            accelerations,
            q_min: [f64::NEG_INFINITY; N],
            q_max: [f64::INFINITY; N],
            limit_margin: 0.02,
            enable_button,
            input_timeout: Duration::from_millis(200),
        }
    }

    /// The velocities one sample asks for, before the envelope: values
    /// clamped to `[-1, 1]`, a non-finite value reads as zero, and an
    /// unpressed enable button gives zero.
    pub fn velocities(&self, input: &JogInput<N>) -> [f64; N] {
        if let Some(button) = self.enable_button
            && (button >= 32 || input.buttons & (1 << button) == 0)
        {
            return [0.0; N];
        }
        let span = (1.0 - self.deadzone).max(f64::EPSILON);
        std::array::from_fn(|i| {
            let v = input.values[i];
            let v = if v.is_finite() {
                v.clamp(-1.0, 1.0)
            } else {
                0.0
            };
            (v.abs() - self.deadzone).max(0.0) / span * v.signum() * self.speeds[i]
        })
    }

    /// The fastest joint `i` may move toward each limit from `q` for one
    /// cycle of `dt` seconds: `(toward q_min, toward q_max)`, both
    /// non-negative; no farther than the limit within the cycle either.
    fn caps(&self, i: usize, q: f64, dt: f64) -> (f64, f64) {
        let cap = |distance: f64| {
            if distance > 0.0 {
                (2.0 * self.accelerations[i] * distance)
                    .sqrt()
                    .min(distance / dt)
            } else {
                0.0
            }
        };
        (
            cap(q - (self.q_min[i] + self.limit_margin)),
            cap((self.q_max[i] - self.limit_margin) - q),
        )
    }
}

/// A joint velocity controller following jog input, for
/// `JointVelocityControl<N>`: the joint-space counterpart of
/// [`cartesian_teleop_control`]. `input` is polled once per cycle and
/// returns the newest sample, or `None` if none arrived since the last
/// poll. The command follows [`JogMapping::velocities`] within the
/// acceleration limits and the position envelope, and returns to zero
/// (within the same limits) after `input_timeout` without samples. A joint
/// whose position the observation lacks is commanded zero: the envelope
/// cannot be kept without it. `done` reports `true` once `stop` returns
/// `true` and the command has reached zero.
pub fn joint_jog_control<const N: usize, F, D>(
    mut input: F,
    mapping: JogMapping<N>,
    mut stop: D,
) -> impl FnMut(JointState<N>, Duration) -> ([f64; N], bool)
where
    F: FnMut() -> Option<JogInput<N>>,
    D: FnMut() -> bool,
{
    let mut command = [0.0; N];
    let mut target = [0.0; N];
    let mut since_input = Duration::MAX;
    move |state, period| {
        match input() {
            Some(sample) => {
                target = mapping.velocities(&sample);
                since_input = Duration::ZERO;
            }
            None => since_input = since_input.saturating_add(period),
        }
        let stopping = stop();
        if stopping || since_input > mapping.input_timeout {
            target = [0.0; N];
        }
        let q = state.meas.q();
        let dt = period.as_secs_f64();
        for i in 0..N {
            let (down, up) = match q {
                Some(q) => mapping.caps(i, q[i], dt),
                None => (0.0, 0.0),
            };
            let want = target[i].clamp(-down, up);
            let step = mapping.accelerations[i] * dt;
            // The envelope holds even where the cap falls faster than the
            // acceleration allows (a jog started inside the margin).
            command[i] = (command[i] + (want - command[i]).clamp(-step, step)).clamp(-down, up);
        }
        (command, stopping && command.iter().all(|v| *v == 0.0))
    }
}
