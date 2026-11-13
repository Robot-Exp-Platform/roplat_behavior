//! Joint PD and PID and base-velocity PID on plants whose closed-loop
//! responses are known in closed form: steps under a constant load, a joint
//! held while the integral winds up, ramps and sine waves.
//!
//! The plants are integrated exactly for a command held over each period of
//! 0.1 ms, so the simulated responses differ from the continuous ones only by
//! the half-period lag of the hold. The first call of each run gets a zero
//! period, so after `k` calls the controllers have seen `k · DT` of time.

use std::ops::ControlFlow::{Break, Continue};
use std::time::Duration;

use roplat_behavior::{
    BaseState, ControlExit, ControlStep, JointSample, JointState, RobotException, SpatialSample,
    Twist,
};
use roplat_control::{
    base_velocity_pid_control, base_velocity_pid_tracking_control, joint_path_target_fn,
    joint_pd_control, joint_pd_tracking_control, joint_pid_control, joint_pid_tracking_control,
};

const DT: f64 = 1e-4;
const PERIOD: Duration = Duration::from_nanos(100_000);

fn period(k: usize) -> Duration {
    if k == 0 { Duration::ZERO } else { PERIOD }
}

/// Rigid joints of inertia `inertia` (kg m²) under a constant load torque
/// `load` (N m): `inertia · q̈ = τ + load`.
struct Joints<const N: usize> {
    inertia: [f64; N],
    load: [f64; N],
    q: [f64; N],
    dq: [f64; N],
}

impl<const N: usize> Joints<N> {
    fn at_rest(inertia: [f64; N], load: [f64; N]) -> Self {
        Self { inertia, load, q: [0.0; N], dq: [0.0; N] }
    }

    fn state(&self) -> JointState<N> {
        JointState::from_meas(JointSample::EMPTY.with_q(self.q).with_dq(self.dq))
    }

    /// One period with `tau` held.
    fn step(&mut self, tau: [f64; N]) {
        for i in 0..N {
            let acc = (tau[i] + self.load[i]) / self.inertia[i];
            self.q[i] += self.dq[i] * DT + 0.5 * acc * DT * DT;
            self.dq[i] += acc * DT;
        }
    }
}

/// Calls `control` `calls` times on `joints`, stepping them with each
/// command; `visit` sees the time, the joints and the command of each call.
fn run<const N: usize>(
    joints: &mut Joints<N>,
    control: &mut impl FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]>,
    calls: usize,
    mut visit: impl FnMut(f64, &Joints<N>, [f64; N]),
) {
    for k in 0..calls {
        let Continue((tau, done)) = control(joints.state(), period(k)) else {
            panic!("the controller stopped at call {k}");
        };
        assert!(!done, "call {k}");
        visit(k as f64 * DT, joints, tau);
        joints.step(tau);
    }
}

/// PD gains that place both closed-loop poles at `-p` on a joint of inertia
/// `j`: `kp = j p²`, `kd = 2 j p`.
fn critically_damped(j: f64, p: f64) -> (f64, f64) {
    (j * p * p, 2.0 * j * p)
}

/// PID gains that place all three closed-loop poles at `-p`, for the law
/// `kp e + ki ∫e − kd q̇` on a joint of inertia `j`: the characteristic
/// polynomial `j s³ + kd s² + kp s + ki` is `j (s + p)³`.
fn triple_pole(j: f64, p: f64) -> (f64, f64, f64) {
    (3.0 * j * p * p, j * p.powi(3), 3.0 * j * p)
}

/// The PID response to a step of `r` from rest, under a load `d`, with the
/// integral starting at `i0`:
/// `Q(s) = [(kp s + ki) r/s + ki i0 + d] / (j (s + p)³)`, which inverts to
/// `r (1 − e^{−pt} (1 + pt − p²t²)) + (p³ i0 + d/j) t² e^{−pt} / 2`.
fn pid_step(r: f64, d: f64, i0: f64, j: f64, p: f64, t: f64) -> f64 {
    let decay = (-p * t).exp();
    r * (1.0 - decay * (1.0 + p * t - p * p * t * t))
        + (p.powi(3) * i0 + d / j) * t * t * decay / 2.0
}

/// A step of 0.6 and -0.4 rad on two joints under constant loads, with the
/// poles at -8 and -5: `q = (r + d/kp)(1 − e^{−pt}(1 + pt))`. No overshoot,
/// and the load leaves the offset `d / kp` a PD law cannot remove.
#[test]
fn a_pd_step_is_critically_damped_and_offset_by_the_load() {
    let (inertia, p, load, target) = ([0.5, 2.0], [8.0, 5.0], [1.5, -3.0], [0.6, -0.4]);
    let gains = [
        critically_damped(inertia[0], p[0]),
        critically_damped(inertia[1], p[1]),
    ];
    let (kp, kd) = (gains.map(|g| g.0), gains.map(|g| g.1));
    let mut joints = Joints::at_rest(inertia, load);
    let mut control = joint_pd_control(target, kp, kd);
    let settled = [0, 1].map(|i| target[i] + load[i] / kp[i]);
    let mut worst = 0.0f64;
    run(&mut joints, &mut control, 40_000, |t, joints, tau| {
        for i in 0..2 {
            let expected = settled[i] * (1.0 - (-p[i] * t).exp() * (1.0 + p[i] * t));
            worst = worst.max((joints.q[i] - expected).abs());
            // The law itself, on the state the controller saw.
            let law = kp[i] * (target[i] - joints.q[i]) - kd[i] * joints.dq[i];
            assert!((tau[i] - law).abs() < 1e-9, "t = {t}: {tau:?}");
        }
    });
    println!("PD step: largest departure from the closed form {worst:.2e} rad");
    assert!(worst < 1e-3, "largest departure {worst} rad");
    for i in 0..2 {
        assert!((joints.q[i] - settled[i]).abs() < 1e-6, "{:?}", joints.q);
    }
}

/// The same steps with PID gains placing all poles at -6 and -4. The
/// integral takes up the load: the joints settle on the target.
#[test]
fn a_pid_step_settles_on_the_target_under_a_load() {
    let (inertia, p, load, target) = ([0.5, 2.0], [6.0, 4.0], [1.5, -3.0], [0.6, -0.4]);
    let gains = [triple_pole(inertia[0], p[0]), triple_pole(inertia[1], p[1])];
    let (kp, ki, kd) = (gains.map(|g| g.0), gains.map(|g| g.1), gains.map(|g| g.2));
    let mut joints = Joints::at_rest(inertia, load);
    let mut control = joint_pid_control(target, kp, ki, kd);
    let mut worst = 0.0f64;
    run(&mut joints, &mut control, 60_000, |t, joints, _| {
        for i in 0..2 {
            let expected = pid_step(target[i], load[i], 0.0, inertia[i], p[i], t);
            worst = worst.max((joints.q[i] - expected).abs());
        }
    });
    println!("PID step: largest departure from the closed form {worst:.2e} rad");
    assert!(worst < 1e-3, "largest departure {worst} rad");
    for i in 0..2 {
        assert!((joints.q[i] - target[i]).abs() < 1e-6, "{:?}", joints.q);
    }
}

/// The PID controller does not limit its integral, as documented. A joint
/// held still for 1 s against a step of 0.5 rad: the command grows as
/// `kp r + ki r t`, and after release the joint overshoots as the full
/// integral `r · 1 s` drives it, 179 % past the target here.
#[test]
fn the_integral_winds_up_while_the_joint_is_held() {
    let (j, p, r) = (0.5, 6.0, 0.5);
    let (kp, ki, kd) = triple_pole(j, p);
    let held = 10_000;
    let mut joints = Joints::at_rest([j], [0.0]);
    let mut control = joint_pid_control([r], [kp], [ki], [kd]);
    let mut peak = 0.0f64;
    let mut worst = 0.0f64;
    let i0 = r * held as f64 * DT;
    for k in 0..held + 30_000 {
        let Continue(([tau], _)) = control(joints.state(), period(k)) else {
            panic!("stopped at call {k}");
        };
        if k < held {
            let integral = r * k as f64 * DT;
            assert!(
                (tau - (kp * r + ki * integral)).abs() < 1e-9 * tau,
                "call {k}: {tau}"
            );
            continue;
        }
        let t = (k - held) as f64 * DT;
        worst = worst.max((joints.q[0] - pid_step(r, 0.0, i0, j, p, t)).abs());
        peak = peak.max(joints.q[0]);
        joints.step([tau]);
    }
    // The analytic peak: the step's own overshoot plus the integral's push.
    let expected_peak = (0..40_000)
        .map(|k| pid_step(r, 0.0, i0, j, p, k as f64 * DT))
        .fold(0.0f64, f64::max);
    println!(
        "windup: largest departure {worst:.2e} rad; peak {peak:.4} rad, closed form {expected_peak:.4} rad"
    );
    assert!(worst < 1e-3, "largest departure {worst} rad");
    assert!(
        (peak - expected_peak).abs() < 1e-3,
        "{peak} against {expected_peak}"
    );
    assert!(peak > 2.75 * r, "{peak}");
}

/// Conditional integration from outside: the caller passes a zero period
/// while the previous command was at the actuator limit, which holds the
/// integral. Held against the same step with a 40 N m actuator, the integral
/// stops at `(40 − kp r) / ki`; after release the response is the linear one
/// from that integral, the command stays within the limit, and the overshoot
/// is a fraction of the unlimited integral's.
#[test]
fn a_held_integral_bounds_the_overshoot_after_saturation() {
    let (j, p, r, limit) = (0.5, 6.0, 0.5, 40.0);
    let (kp, ki, kd) = triple_pole(j, p);
    let held = 10_000;
    let mut joints = Joints::at_rest([j], [0.0]);
    let mut pid = joint_pid_control([r], [kp], [ki], [kd]);
    let mut saturated = false;
    let mut control = move |state: JointState<1>, period: Duration| {
        let Continue(([tau], done)) = pid(state, if saturated { Duration::ZERO } else { period })
        else {
            panic!("stopped");
        };
        saturated = tau.abs() >= limit;
        (tau, done)
    };
    let mut last_held = 0.0;
    for k in 0..held {
        last_held = control(joints.state(), period(k)).0;
    }
    // The integral stopped within one period of reaching the limit.
    assert!(
        (limit..limit + ki * r * DT + 1e-9).contains(&last_held),
        "{last_held}"
    );
    let i0 = (last_held - kp * r) / ki;
    assert!((i0 - (limit - kp * r) / ki).abs() < r * DT + 1e-12, "{i0}");
    let mut worst = 0.0f64;
    let mut peak = 0.0f64;
    for k in 0..30_000 {
        let t = k as f64 * DT;
        let (tau, _) = control(joints.state(), PERIOD);
        if k > 10 {
            assert!(tau < limit, "t = {t}: {tau}");
        }
        worst = worst.max((joints.q[0] - pid_step(r, 0.0, i0, j, p, t)).abs());
        peak = peak.max(joints.q[0]);
        joints.step([tau.clamp(-limit, limit)]);
    }
    println!("held integral {i0:.5}: largest departure {worst:.2e} rad, peak {peak:.4} rad");
    assert!(worst < 1e-3, "largest departure {worst} rad");
    // The unlimited integral overshoots by 179 %, the held one by 60 %.
    assert!(peak > 1.55 * r && peak < 1.65 * r, "{peak}");
}

/// A ramp of 0.2 rad/s under a load: PD lags by `(kd v − d) / kp`, PID
/// catches up.
#[test]
fn a_ramp_lags_by_kd_v_over_kp_under_pd_and_not_under_pid() {
    let (j, v, d) = (0.5, 0.2, 1.0);
    let ramp = move |t: Duration| [v * t.as_secs_f64()];
    let total = Duration::from_secs(60);

    let (kp, kd) = critically_damped(j, 8.0);
    let mut joints = Joints::at_rest([j], [d]);
    let mut pd = joint_pd_tracking_control(joint_path_target_fn(ramp, total), [kp], [kd]);
    let mut lag = Vec::new();
    run(&mut joints, &mut pd, 50_000, |t, joints, _| {
        if t >= 4.0 {
            lag.push(v * t - joints.q[0]);
        }
    });
    let expected = (kd * v - d) / kp;
    assert!(
        lag.iter().all(|e| (e - expected).abs() < 1e-5),
        "{:?} against {expected}",
        lag.first()
    );

    let (kp, ki, kd) = triple_pole(j, 6.0);
    let mut joints = Joints::at_rest([j], [d]);
    let mut pid = joint_pid_tracking_control(joint_path_target_fn(ramp, total), [kp], [ki], [kd]);
    let mut worst = 0.0f64;
    run(&mut joints, &mut pid, 80_000, |t, joints, _| {
        if t >= 7.0 {
            worst = worst.max((v * t - joints.q[0]).abs());
        }
    });
    println!(
        "ramp: PD lag {:.6} rad, (kd v - d) / kp = {expected:.6}; PID largest error {worst:.2e} rad",
        lag[lag.len() - 1]
    );
    assert!(worst < 1e-5, "{worst}");
}

/// `|E(jω) / R(jω)|` for the error transfer `num(s) / den(s)` given by
/// polynomial coefficients, highest power first.
fn gain(num: &[f64], den: &[f64], w: f64) -> f64 {
    let eval = |c: &[f64]| {
        // Horner in complex arithmetic, s = jω.
        c.iter()
            .fold((0.0, 0.0), |(re, im), &a| (-im * w + a, re * w))
    };
    let ((nr, ni), (dr, di)) = (eval(num), eval(den));
    (nr.hypot(ni)) / (dr.hypot(di))
}

/// The largest error over the last two periods of a sine of `w` rad/s.
fn steady_peak(errors: &[(f64, f64)], w: f64) -> f64 {
    let end = errors.last().unwrap().0;
    let from = end - 2.0 * std::f64::consts::TAU / w;
    errors
        .iter()
        .filter(|(t, _)| *t >= from)
        .fold(0.0f64, |m, (_, e)| m.max(e.abs()))
}

/// A sine of 0.3 rad at 2 rad/s: the steady tracking error is the
/// amplitude times the closed loop's error gain at 2 rad/s,
/// `|j s² + kd s| / |j s² + kd s + kp|` under PD and
/// `|j s³ + kd s²| / |j s³ + kd s² + kp s + ki|` under PID.
#[test]
fn a_sine_is_tracked_within_the_error_gain_of_the_loop() {
    let (j, a, w) = (0.5, 0.3, 2.0);
    let sine = move |t: Duration| [a * (w * t.as_secs_f64()).sin()];
    let total = Duration::from_secs(60);

    let (kp, kd) = critically_damped(j, 8.0);
    let mut joints = Joints::at_rest([j], [0.0]);
    let mut pd = joint_pd_tracking_control(joint_path_target_fn(sine, total), [kp], [kd]);
    let mut errors = Vec::new();
    run(&mut joints, &mut pd, 120_000, |t, joints, _| {
        errors.push((t, a * (w * t).sin() - joints.q[0]));
    });
    let bound = a * gain(&[j, kd, 0.0], &[j, kd, kp], w);
    let peak = steady_peak(&errors, w);
    println!("sine under PD: peak error {peak:.6} rad, closed form {bound:.6}");
    assert!((peak / bound - 1.0).abs() < 0.01, "{peak} against {bound}");

    let (kp, ki, kd) = triple_pole(j, 6.0);
    let mut joints = Joints::at_rest([j], [0.0]);
    let mut pid = joint_pid_tracking_control(joint_path_target_fn(sine, total), [kp], [ki], [kd]);
    errors.clear();
    run(&mut joints, &mut pid, 120_000, |t, joints, _| {
        errors.push((t, a * (w * t).sin() - joints.q[0]));
    });
    let bound = a * gain(&[j, kd, 0.0, 0.0], &[j, kd, kp, ki], w);
    let peak = steady_peak(&errors, w);
    println!("sine under PID: peak error {peak:.6} rad, closed form {bound:.6}");
    assert!((peak / bound - 1.0).abs() < 0.01, "{peak} against {bound}");
}

/// A base whose velocity follows the command with time constant `tau` (s),
/// less a constant slip `slip` (m/s or rad/s) per axis:
/// `tau · v̇ = u − slip − v`.
struct Base {
    tau: f64,
    slip: [f64; 6],
    v: [f64; 6],
}

impl Base {
    fn state(&self) -> BaseState {
        BaseState::from_meas(SpatialSample::EMPTY.with_twist(Twist::from(self.v)))
    }

    fn step(&mut self, u: [f64; 6]) {
        let decay = (-DT / self.tau).exp();
        for i in 0..6 {
            let goal = u[i] - self.slip[i];
            self.v[i] = goal + (self.v[i] - goal) * decay;
        }
    }
}

/// Base PID gains placing both poles at `-p`. The derivative acts on the
/// error, `kd ė = −kd v̇` for a constant target, so it adds `kd` to the time
/// constant: `(tau + kd) s² + (1 + kp) s + ki = (tau + kd)(s + p)²`.
fn base_gains(tau: f64, kd: f64, p: f64) -> (f64, f64) {
    let lag = tau + kd;
    (2.0 * lag * p - 1.0, lag * p * p)
}

/// A step on all six axes against a slip: with `T = tau + kd`,
/// `v = r (1 − e^{−pt} + (kp/T − p) t e^{−pt}) − (slip / T) t e^{−pt}`,
/// settling on the target.
#[test]
fn a_base_velocity_step_settles_on_the_target_against_a_slip() {
    let (tau, kd, p) = (0.2, 0.05, 10.0);
    let (kp, ki) = base_gains(tau, kd, p);
    let target = [0.5, -0.2, 0.0, 0.1, -0.3, 0.8];
    let slip = [0.05, -0.02, 0.01, 0.0, 0.03, -0.1];
    let mut base = Base { tau, slip, v: [0.0; 6] };
    let mut control = base_velocity_pid_control(target, [kp; 6], [ki; 6], [kd; 6]);
    let lag = tau + kd;
    let mut worst = 0.0f64;
    for k in 0..30_000 {
        let Continue((u, done)) = control(base.state(), period(k)) else {
            panic!("stopped at call {k}");
        };
        assert!(!done);
        let t = k as f64 * DT;
        let decay = (-p * t).exp();
        for i in 0..6 {
            let expected =
                target[i] * (1.0 - decay + (kp / lag - p) * t * decay) - slip[i] / lag * t * decay;
            worst = worst.max((base.v[i] - expected).abs());
        }
        base.step(u);
    }
    println!("base step: largest departure from the closed form {worst:.2e}");
    assert!(worst < 1e-3, "largest departure {worst}");
    for i in 0..6 {
        assert!((base.v[i] - target[i]).abs() < 1e-6, "{:?}", base.v);
    }
}

/// A sine of 0.4 m/s at 3 rad/s on the base's x axis: with the derivative
/// on the error, `E/R = (tau s² + s) / ((tau + kd) s² + (1 + kp) s + ki)`.
#[test]
fn a_base_velocity_sine_is_tracked_within_the_error_gain() {
    let (tau, kd, p, a, w) = (0.2, 0.05, 10.0, 0.4, 3.0);
    let (kp, ki) = base_gains(tau, kd, p);
    let mut base = Base { tau, slip: [0.0; 6], v: [0.0; 6] };
    let mut elapsed = Duration::ZERO;
    let mut control = base_velocity_pid_tracking_control(
        move |_, period| {
            elapsed += period;
            (
                [
                    a * (w * elapsed.as_secs_f64()).sin(),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                false,
            )
        },
        [kp; 6],
        [ki; 6],
        [kd; 6],
    );
    let mut errors = Vec::new();
    for k in 0..80_000 {
        let Continue((u, _)) = control(base.state(), period(k)) else {
            panic!("stopped");
        };
        let t = k as f64 * DT;
        errors.push((t, a * (w * t).sin() - base.v[0]));
        base.step(u);
    }
    let bound = a * gain(&[tau, 1.0, 0.0], &[tau + kd, 1.0 + kp, ki], w);
    let peak = steady_peak(&errors, w);
    println!("base sine: peak error {peak:.6} m/s, closed form {bound:.6}");
    assert!((peak / bound - 1.0).abs() < 0.01, "{peak} against {bound}");
}

/// Without the measured quantity a law needs, each controller ends the
/// session with an error rather than a command.
#[test]
fn a_missing_measurement_ends_the_session() {
    let only_q = JointState::from_meas(JointSample::EMPTY.with_q([0.0]));
    let only_dq = JointState::from_meas(JointSample::EMPTY.with_dq([0.0]));
    let failed = |step: ControlStep<[f64; 1]>| {
        matches!(
            step,
            Break(ControlExit::Failed(
                RobotException::UnprocessableInstructionError(_)
            ))
        )
    };
    let mut pd = joint_pd_control([0.0], [1.0], [1.0]);
    assert!(failed(pd(only_q, PERIOD)) && failed(pd(only_dq, PERIOD)));
    let mut pid = joint_pid_control([0.0], [1.0], [1.0], [1.0]);
    assert!(failed(pid(only_q, PERIOD)) && failed(pid(only_dq, PERIOD)));
    let mut base = base_velocity_pid_control([0.0; 6], [1.0; 6], [1.0; 6], [1.0; 6]);
    assert!(matches!(
        base(BaseState::from_meas(SpatialSample::EMPTY), PERIOD),
        Break(ControlExit::Failed(
            RobotException::UnprocessableInstructionError(_)
        ))
    ));
}
