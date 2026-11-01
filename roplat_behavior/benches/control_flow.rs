#![feature(async_trait_bounds)]
//! Finite CPU-only paired microbenchmark; no hardware, pacing or I/O.
//! The legacy rhythm reproduces the old successful path at be5974bf; the old
//! crate cannot compile against current roplat. This is a path comparison, not
//! a claim that an unchanged old workspace was built or that hardware latency
//! was measured. Run: cargo bench -p roplat_behavior --features roplat --bench control_flow
use roplat::{Completion, ExecutionContext, rhythm::Rhythm};
use roplat_behavior::{
    ControlSpace, ControlStep, ControlWith, Robot, RobotResult, roplat::ControlRhythm,
};
use std::{
    future::Future,
    hint::black_box,
    ops::ControlFlow,
    time::{Duration, Instant},
};

struct Channel<const W: usize>;
struct BenchRobot<const W: usize> {
    count: usize,
    checksum: f64,
}
impl<const W: usize> Robot for BenchRobot<W> {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "benchmark".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}
impl<const W: usize> ControlSpace for Channel<W> {
    type Obs = usize;
    type Command = [f64; W];
}
impl<const W: usize> BenchRobot<W> {
    fn new() -> Self {
        Self { count: 0, checksum: 0.0 }
    }
    // Snapshot of the pre-ControlFlow loop shape, sharing observation/command work.
    fn legacy<F: FnMut(usize, Duration) -> ([f64; W], bool)>(
        &mut self,
        mut callback: F,
    ) -> RobotResult<()> {
        loop {
            let (command, done) = callback(black_box(self.count), Duration::from_millis(1));
            self.checksum += black_box(command)[0];
            self.count += 1;
            if done {
                break;
            }
        }
        Ok(())
    }
    // Exact pre-change async callback adapter shape (rather than a directly
    // inlined async block, whose capture/layout can compile differently).
    fn legacy_async<F>(&mut self, mut callback: F) -> RobotResult<()>
    where
        F: async FnMut(usize, Duration) -> ([f64; W], bool),
    {
        self.legacy(move |obs, duration| futures::executor::block_on(callback(obs, duration)))
    }
}
impl<const W: usize> ControlWith<Channel<W>> for BenchRobot<W> {
    fn hold_command(_: &usize) -> [f64; W] {
        [0.0; W]
    }
    fn control_with_flow<F>(&mut self, mut callback: F) -> RobotResult<()>
    where
        F: FnMut(usize, Duration) -> ControlStep<[f64; W]>,
    {
        loop {
            match callback(black_box(self.count), Duration::from_millis(1)) {
                ControlFlow::Break(_) => break,
                ControlFlow::Continue((command, done)) => {
                    self.checksum += black_box(command)[0];
                    self.count += 1;
                    if done {
                        break;
                    }
                }
            }
        }
        Ok(())
    }
}
fn command<const W: usize>(tick: usize, cycles: usize) -> ([f64; W], bool) {
    ([black_box(tick as f64); W], tick + 1 == cycles)
}
fn sync_path<const W: usize>(flow: bool, cycles: usize) {
    let mut robot = BenchRobot::<W>::new();
    if flow {
        <BenchRobot<W> as ControlWith<Channel<W>>>::control_with(&mut robot, |t, _| {
            command(t, cycles)
        })
        .unwrap();
    } else {
        robot.legacy(|t, _| command(t, cycles)).unwrap();
    }
    black_box(robot.checksum);
}
fn async_path<const W: usize>(flow: bool, cycles: usize) {
    let mut robot = BenchRobot::<W>::new();
    if flow {
        <BenchRobot<W> as ControlWith<Channel<W>>>::control_with_async(&mut robot, async |t, _| {
            command(t, cycles)
        })
        .unwrap();
    } else {
        robot.legacy_async(async |t, _| command(t, cycles)).unwrap();
    }
    black_box(robot.checksum);
}
// Successful and ordinary-error code shape copied from the old drive contract.
// It deliberately has no Execution/Context: those are the change being measured.
async fn legacy_rhythm<const W: usize, N, F, Fut>(
    nodes: N,
    mut domain: F,
    input: RobotResult<BenchRobot<W>>,
) -> (RobotResult<BenchRobot<W>>, N)
where
    N: Send,
    F: FnMut(N, (usize, Duration)) -> Fut + Send,
    Fut: Future<Output = (([f64; W], bool), N)> + Send,
{
    let mut robot = match input {
        Ok(robot) => robot,
        Err(error) => return (Err(error), nodes),
    };
    let mut nodes = Some(nodes);
    let result = robot.legacy_async(async |obs, duration| {
        let current_nodes = nodes.take().expect("legacy nodes lost");
        let (feed, returned_nodes) = domain(current_nodes, (obs, duration)).await;
        nodes = Some(returned_nodes);
        feed
    });
    let nodes = nodes.expect("legacy domain did not return nodes");
    match result {
        Ok(()) => (Ok(robot), nodes),
        Err(error) => (Err(error), nodes),
    }
}
fn rhythm_path<const W: usize>(flow: bool, cycles: usize, runtime: &tokio::runtime::Runtime) {
    if flow {
        let mut robot = BenchRobot::<W>::new();
        let (out, n) = runtime.block_on(ControlRhythm::<BenchRobot<W>, Channel<W>>::new().drive(
            0_u64,
            |n, (t, _), _| async move {
                (
                    Ok(Completion::Completed(command::<W>(t, cycles))),
                    black_box(n + 1),
                )
            },
            (),
            (&mut robot,),
            ExecutionContext::new(),
        ));
        let Completion::Completed(()) = out.unwrap() else {
            panic!()
        };
        black_box((robot.checksum, n));
    } else {
        let (robot, n) = runtime.block_on(legacy_rhythm::<W, _, _, _>(
            0_u64,
            |n, (t, _)| async move { (command::<W>(t, cycles), black_box(n + 1)) },
            Ok(BenchRobot::new()),
        ));
        black_box((robot.unwrap().checksum, n));
    }
}
fn measure(mut f: impl FnMut(), cycles: usize) -> f64 {
    let start = Instant::now();
    f();
    start.elapsed().as_nanos() as f64 / cycles as f64
}
fn pair<const W: usize>(name: &str, cycles: usize, mut f: impl FnMut(bool)) {
    for i in 0..4 {
        f(i % 2 == 0);
    }
    let mut old = Vec::new();
    let mut new = Vec::new();
    for i in 0..21 {
        if i % 2 == 0 {
            old.push(measure(|| f(false), cycles));
            new.push(measure(|| f(true), cycles));
        } else {
            new.push(measure(|| f(true), cycles));
            old.push(measure(|| f(false), cycles));
        }
    }
    old.sort_by(f64::total_cmp);
    new.sort_by(f64::total_cmp);
    println!(
        "{name},{},{cycles},21,{:.3},{:.3},{:.3},{:.3},{:.2}",
        W * 8,
        old[10],
        new[10],
        old[20],
        new[20],
        (new[10] / old[10] - 1.0) * 100.0
    );
}
fn run<const W: usize>(cycles: usize, rt: &tokio::runtime::Runtime) {
    pair::<W>("sync_tuple_vs_flow", cycles, |flow| {
        sync_path::<W>(flow, cycles)
    });
    pair::<W>("async_tuple_vs_flow", cycles, |flow| {
        async_path::<W>(flow, cycles)
    });
    pair::<W>("legacy_rhythm_vs_execution", cycles, |flow| {
        rhythm_path::<W>(flow, cycles, rt)
    });
}
fn main() {
    let cycles = std::env::var("CONTROL_BENCH_CYCLES")
        .ok()
        .map(|s| s.parse().unwrap())
        .unwrap_or(200_000);
    assert!(cycles > 0, "CONTROL_BENCH_CYCLES must be positive");
    let runtime = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();
    println!(
        "path,command_bytes,cycles,samples,old_median_ns_per_cycle,new_median_ns_per_cycle,old_max_sample_ns_per_cycle,new_max_sample_ns_per_cycle,median_delta_percent"
    );
    run::<6>(cycles, &runtime);
    run::<7>(cycles, &runtime);
    run::<128>(cycles, &runtime);
}
