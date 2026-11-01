//! Finite CPU-only comparison of blocking and native asynchronous control.
//! No device, network, scheduler fairness or deadline claim is made. Each
//! invocation measures a batch mean, not a per-cycle latency percentile.
use roplat::{Completion, ExecutionContext, rhythm::Rhythm};
use roplat_behavior::{
    AsyncControlCallback, AsyncControlWith, ControlSpace, ControlStep, ControlWith, Robot,
    RobotResult,
    roplat::{AsyncControlRhythm, ControlRhythm},
};
use std::{
    hint::black_box,
    ops::ControlFlow,
    time::{Duration, Instant},
};

struct Channel<const W: usize>;
struct CpuRobot<const W: usize> {
    cycles: usize,
    checksum: f64,
}
impl<const W: usize> CpuRobot<W> {
    fn new() -> Self {
        Self { cycles: 0, checksum: 0.0 }
    }
    fn commit(&mut self, command: [f64; W]) {
        self.checksum += black_box(command)[0];
        self.cycles += 1;
    }
}
impl<const W: usize> Robot for CpuRobot<W> {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "cpu-benchmark".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}
impl<const W: usize> ControlSpace for Channel<W> {
    type Obs = usize;
    type Command = [f64; W];
}
impl<const W: usize> ControlWith<Channel<W>> for CpuRobot<W> {
    fn hold_command(_: &usize) -> [f64; W] {
        [0.0; W]
    }
    fn control_with_flow<F>(&mut self, mut callback: F) -> RobotResult<()>
    where
        F: FnMut(usize, Duration) -> ControlStep<[f64; W]>,
    {
        loop {
            match callback(black_box(self.cycles), Duration::from_millis(1)) {
                ControlFlow::Continue((command, done)) => {
                    self.commit(command);
                    if !done {
                        continue;
                    }
                }
                ControlFlow::Break(_) => {}
            }
            return Ok(());
        }
    }
}
impl<const W: usize> AsyncControlWith<Channel<W>> for CpuRobot<W> {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<usize, [f64; W]>,
    {
        loop {
            match callback
                .call(black_box(self.cycles), Duration::from_millis(1))
                .await
            {
                ControlFlow::Continue((command, done)) => {
                    self.commit(command);
                    if !done {
                        continue;
                    }
                }
                ControlFlow::Break(_) => {}
            }
            return Ok(());
        }
    }
}
fn command<const W: usize>(tick: usize, cycles: usize) -> ([f64; W], bool) {
    ([black_box(tick as f64); W], tick + 1 == cycles)
}
fn rhythm_path<const W: usize>(native: bool, cycles: usize, runtime: &tokio::runtime::Runtime) {
    let domain = |n, (tick, _), _| async move {
        (
            Ok(Completion::Completed(command::<W>(tick, cycles))),
            black_box(n + 1),
        )
    };
    let mut robot = CpuRobot::<W>::new();
    let context = ExecutionContext::new();
    let (outcome, nodes) = if native {
        runtime.block_on(AsyncControlRhythm::<CpuRobot<W>, Channel<W>>::new().drive(
            0_u64,
            domain,
            (),
            (&mut robot,),
            context,
        ))
    } else {
        runtime.block_on(ControlRhythm::<CpuRobot<W>, Channel<W>>::new().drive(
            0_u64,
            domain,
            (),
            (&mut robot,),
            context,
        ))
    };
    let Completion::Completed(()) = outcome.unwrap() else {
        panic!("unexpected stop")
    };
    assert_eq!(nodes, cycles as u64);
    assert_eq!(robot.cycles, cycles);
    black_box((nodes, robot.checksum));
}
fn measure(mut run: impl FnMut(), cycles: usize) -> f64 {
    let start = Instant::now();
    run();
    start.elapsed().as_secs_f64() * 1e9 / cycles as f64
}
fn compare<const W: usize>(cycles: usize, runtime: &tokio::runtime::Runtime) {
    for _ in 0..4 {
        rhythm_path::<W>(false, cycles, runtime);
        rhythm_path::<W>(true, cycles, runtime);
    }
    let mut blocking = Vec::new();
    let mut native = Vec::new();
    for sample in 0..21 {
        for path in if sample % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        } {
            let elapsed = measure(|| rhythm_path::<W>(path, cycles, runtime), cycles);
            if path {
                native.push(elapsed);
            } else {
                blocking.push(elapsed);
            }
        }
    }
    blocking.sort_by(f64::total_cmp);
    native.sort_by(f64::total_cmp);
    println!(
        "{},{cycles},21,{:.3},{:.3},{:.3},{:.3},{:.2}",
        W * 8,
        blocking[10],
        native[10],
        blocking[20],
        native[20],
        (native[10] / blocking[10] - 1.0) * 100.0
    );
}
fn main() {
    let cycles = std::env::var("CONTROL_BENCH_CYCLES")
        .ok()
        .map(|value| value.parse().expect("positive cycle count"))
        .unwrap_or(200_000);
    assert!(cycles > 0);
    let runtime = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();
    println!(
        "command_bytes,cycles,samples,blocking_median_batch_ns_per_cycle,native_median_batch_ns_per_cycle,blocking_max_batch_ns_per_cycle,native_max_batch_ns_per_cycle,median_delta_percent"
    );
    compare::<6>(cycles, &runtime);
    compare::<7>(cycles, &runtime);
    compare::<128>(cycles, &runtime);
}
