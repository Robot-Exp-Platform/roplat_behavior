//! Joint jogging: the dead-man switch, the acceleration limits and the
//! position envelope.

use std::{cell::Cell, time::Duration};

use roplat_behavior::{JointState, msgs::JointSample};
use roplat_control::{JogInput, JogMapping, joint_jog_control};

const DEADMAN: u32 = 0;

fn at<const N: usize>(q: [f64; N]) -> JointState<N> {
    JointState::from_meas(JointSample::EMPTY.with_q(q))
}

fn jog<const N: usize>(values: [f64; N], buttons: u32) -> Option<JogInput<N>> {
    Some(JogInput { values, buttons })
}

/// The dead-man bit gates the jog; the envelope brakes before a limit and
/// lets the joint back out; a joint without a position stays.
#[test]
fn jogs_need_the_deadman_and_stop_short_of_the_limits() {
    let mapping = JogMapping {
        deadzone: 0.0,
        q_min: [-1.0, -1.0],
        q_max: [1.0, 1.0],
        limit_margin: 0.1,
        ..JogMapping::new([0.5, 0.5], [f64::INFINITY, 2.0], Some(DEADMAN))
    };
    let period = Duration::from_millis(10);
    let next = Cell::new(jog([1.0, 0.0], 0));
    let mut control = joint_jog_control(|| next.take(), mapping, || false);
    assert_eq!(control(at([0.0, 0.0]), period).0, [0.0, 0.0], "no dead-man");
    next.set(jog([1.0, -0.5], 1 << DEADMAN));
    assert_eq!(control(at([0.0, 0.0]), period).0[0], 0.5);
    // Joint 0 inside the margin: nothing toward the limit, all of it away
    // from it.
    next.set(jog([1.0, 0.0], 1));
    assert_eq!(control(at([0.95, 0.0]), period).0[0], 0.0);
    next.set(jog([-1.0, 0.0], 1));
    assert_eq!(control(at([0.95, 0.0]), period).0[0], -0.5);

    // Joint 1 jogged up at 2 rad/s² with the joint following the command:
    // it slows down and stops by the margin, not past it.
    let next = Cell::new(None);
    let mut control = joint_jog_control(|| next.take(), mapping, || false);
    let mut q = [0.0, 0.0];
    let mut peak: f64 = 0.0;
    for k in 0..400 {
        if k % 5 == 0 {
            next.set(jog([0.0, 1.0], 1));
        }
        let (command, _) = control(at(q), period);
        peak = peak.max(command[1]);
        q[1] += command[1] * period.as_secs_f64();
    }
    assert!((peak - 0.5).abs() < 1e-12, "peak {peak}");
    assert!(q[1] <= 0.9 + 1e-9 && q[1] > 0.88, "joint 1 at {}", q[1]);

    let next = Cell::new(jog([1.0, 1.0], 1));
    let mut control = joint_jog_control(|| next.take(), mapping, || false);
    let without = JointState::from_meas(JointSample::<2>::EMPTY);
    assert_eq!(control(without, period).0, [0.0, 0.0]);
}

/// Input that stops arriving, and a stop request, bring the command down
/// to zero within the acceleration limit; the session reports done only
/// once it is there.
#[test]
fn silence_and_stop_ramp_the_command_down() {
    let mapping = JogMapping {
        deadzone: 0.0,
        input_timeout: Duration::from_millis(50),
        ..JogMapping::new([1.0], [5.0], None)
    };
    let period = Duration::from_millis(10);
    let next = Cell::new(jog([1.0], 0));
    let stop = Cell::new(false);
    let mut control = joint_jog_control(|| next.take(), mapping, || stop.get());
    let mut command = 0.0;
    for _ in 0..30 {
        next.set(jog([1.0], 0));
        command = control(at([0.0]), period).0[0];
    }
    assert!((command - 1.0).abs() < 1e-12, "{command}");
    // Silence: held for the timeout, then down by 5 rad/s² × 10 ms a cycle.
    let mut seen = Vec::new();
    for _ in 0..30 {
        seen.push(control(at([0.0]), period).0[0]);
    }
    assert!(
        seen[..5].iter().all(|&c| (c - 1.0).abs() < 1e-12),
        "{seen:?}"
    );
    assert!((seen[5] - 0.95).abs() < 1e-12, "{seen:?}");
    assert_eq!(*seen.last().unwrap(), 0.0);

    // A stop request while jogging: not done until the command is zero.
    let next = Cell::new(None);
    let mut control = joint_jog_control(|| next.take(), mapping, || stop.get());
    for _ in 0..30 {
        next.set(jog([-1.0], 0));
        control(at([0.0]), period);
    }
    stop.set(true);
    next.set(jog([-1.0], 0));
    let mut cycles = 0;
    loop {
        let (command, done) = control(at([0.0]), period);
        cycles += 1;
        if done {
            assert_eq!(command, [0.0]);
            break;
        }
        assert!(command[0] < 0.0 && cycles < 100);
    }
    // 1 rad/s down at 5 rad/s² in 10 ms cycles is 20 cycles, one more
    // where rounding leaves a remainder below a step.
    assert!((20..=21).contains(&cycles), "{cycles} cycles");
}

/// A reproducible pseudo-random sequence in `[-1, 1]` (a 64-bit LCG).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
    }
}

/// Random stick motion on three joints that follow their commands: no
/// joint ever passes its margin, and no command changes faster than its
/// joint's acceleration allows, except where the envelope cuts it.
#[test]
fn random_jogging_never_leaves_the_envelope() {
    let mapping = JogMapping {
        q_min: [-1.0, -0.5, 0.0],
        q_max: [1.0, 0.5, 2.0],
        limit_margin: 0.05,
        ..JogMapping::new([1.2, 0.8, 2.0], [3.0, 1.5, 10.0], None)
    };
    let period = Duration::from_millis(4);
    let dt = period.as_secs_f64();
    let mut random = Lcg(20_261_118);
    let next = Cell::new(None);
    let mut control = joint_jog_control(|| next.take(), mapping, || false);
    let mut q = [0.0, 0.0, 1.0];
    let mut previous = [0.0; 3];
    for k in 0..20_000 {
        if k % 7 == 0 {
            // Mostly hard against one end, to drive the joints to their
            // limits again and again.
            let values = std::array::from_fn(|_| (random.next() * 3.0).clamp(-1.0, 1.0));
            next.set(jog(values, 0));
        }
        let (command, _) = control(at(q), period);
        for i in 0..3 {
            let change = (command[i] - previous[i]).abs();
            let braking = command[i].abs() < previous[i].abs();
            assert!(
                change <= mapping.accelerations[i] * dt + 1e-12 || braking,
                "joint {i} cycle {k}: {} -> {}",
                previous[i],
                command[i]
            );
            q[i] += command[i] * dt;
            let (low, high) = (
                mapping.q_min[i] + mapping.limit_margin,
                mapping.q_max[i] - mapping.limit_margin,
            );
            assert!(
                q[i] >= low - 1e-9 && q[i] <= high + 1e-9,
                "joint {i} at {} after cycle {k}",
                q[i]
            );
        }
        previous = command;
    }
}
