#![cfg(feature = "roplat")]

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use roplat::rhythm::Rhythm;
use roplat::{Completion, ExecutionContext, Lifecycle, Node, RoplatError};
use roplat_world::{PhysicsEngine, SimulationRhythm};

#[derive(Default)]
struct Counts {
    attempts: AtomicUsize,
    integrations: AtomicUsize,
    observations: AtomicUsize,
    init: AtomicUsize,
    shutdown: AtomicUsize,
    engine_drops: AtomicUsize,
}

struct Engine {
    counts: Arc<Counts>,
    fail_at: Option<usize>,
}

#[derive(Debug)]
struct InjectedError;
impl fmt::Display for InjectedError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("injected native step error")
    }
}
impl std::error::Error for InjectedError {}

impl PhysicsEngine for Engine {
    type Error = InjectedError;
    fn reset(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn step(&mut self) -> Result<(), Self::Error> {
        let attempt = self.counts.attempts.fetch_add(1, Ordering::SeqCst) + 1;
        if self.fail_at == Some(attempt) {
            return Err(InjectedError);
        }
        self.counts.integrations.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    fn shutdown(self) {}
    fn set_step_time(&mut self, _: Duration) -> Result<&mut Self, Self::Error> {
        Ok(self)
    }
    fn set_gravity(&mut self, _: impl Into<[f64; 3]>) -> Result<&mut Self, Self::Error> {
        Ok(self)
    }
}
impl Drop for Engine {
    fn drop(&mut self) {
        self.counts.engine_drops.fetch_add(1, Ordering::SeqCst);
    }
}

struct Observer(Arc<Counts>);
impl Lifecycle for Observer {
    type Error = RoplatError;
    async fn on_init(&mut self) -> Result<(), Self::Error> {
        self.0.init.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    async fn on_shutdown(&mut self) -> Result<(), Self::Error> {
        self.0.shutdown.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}
impl Node for Observer {
    type Resources = ();
    type Input = ();
    type Output = ();
    async fn process(&mut self, _: (), _: ()) {
        self.0.observations.fetch_add(1, Ordering::SeqCst);
    }
}

#[roplat::system]
async fn finite_system(
    engine: Engine,
    counts: Arc<Counts>,
    steps: u64,
) -> roplat::RoplatResult<()> {
    let mut rhythm = SimulationRhythm::logical(engine).with_max_steps(steps);
    let mut observer = Observer(counts);
    rhythm >> {
        observer;
    };
    Ok(rhythm.output)
}

#[tokio::test]
async fn system_runs_exact_budget_and_closes_owned_nodes() {
    let counts = Arc::new(Counts::default());
    finite_system(
        Engine { counts: counts.clone(), fail_at: None },
        counts.clone(),
        7,
    )
    .await
    .unwrap();
    assert_eq!(counts.attempts.load(Ordering::SeqCst), 7);
    assert_eq!(counts.integrations.load(Ordering::SeqCst), 7);
    assert_eq!(counts.observations.load(Ordering::SeqCst), 7);
    assert_eq!(counts.init.load(Ordering::SeqCst), 1);
    assert_eq!(counts.shutdown.load(Ordering::SeqCst), 1);
    assert_eq!(counts.engine_drops.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn engine_error_prevents_downstream_tick_and_retains_source() {
    let counts = Arc::new(Counts::default());
    let result = finite_system(
        Engine { counts: counts.clone(), fail_at: Some(3) },
        counts.clone(),
        7,
    )
    .await;
    let Err(RoplatError::Node(error)) = result else {
        panic!("expected preserved native error");
    };
    assert!(error.downcast_ref::<InjectedError>().is_some());
    assert_eq!(counts.attempts.load(Ordering::SeqCst), 3);
    assert_eq!(counts.integrations.load(Ordering::SeqCst), 2);
    assert_eq!(counts.observations.load(Ordering::SeqCst), 2);
    assert_eq!(counts.shutdown.load(Ordering::SeqCst), 1);
    assert_eq!(counts.engine_drops.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn finite_budget_resets_per_drive_without_resetting_engine() {
    let counts = Arc::new(Counts::default());
    let mut rhythm = SimulationRhythm::logical(Engine { counts: counts.clone(), fail_at: None })
        .with_max_steps(3);
    let mut owned = Vec::new();
    for _ in 0..2 {
        let (result, returned) = rhythm
            .drive(
                owned,
                |mut owned, (), _| async move {
                    owned.push(1);
                    (Ok(Completion::Completed(())), owned)
                },
                (),
                (),
                ExecutionContext::new(),
            )
            .await;
        assert!(matches!(result, Ok(Completion::Completed(()))));
        assert_eq!(rhythm.steps_completed(), 3);
        owned = returned;
    }
    assert_eq!(owned.len(), 6);
    assert_eq!(counts.integrations.load(Ordering::SeqCst), 6);
}

#[tokio::test]
async fn long_wall_clock_wait_is_stoppable_without_an_extra_integration() {
    let counts = Arc::new(Counts::default());
    let context = ExecutionContext::new();
    let mut rhythm = SimulationRhythm::new(
        Engine { counts: counts.clone(), fail_at: None },
        Duration::from_secs(86_400),
    );
    let first = Arc::new(tokio::sync::Notify::new());
    let signal = first.clone();
    let drive = rhythm.drive(
        0usize,
        move |n, (), _| {
            signal.notify_one();
            async move { (Ok(Completion::Completed(())), n + 1) }
        },
        (),
        (),
        context.clone(),
    );
    let stop = async {
        first.notified().await;
        context.request_stop();
    };
    let ((result, nodes), ()) =
        tokio::time::timeout(Duration::from_secs(2), async { tokio::join!(drive, stop) })
            .await
            .unwrap();
    assert!(matches!(result, Ok(Completion::Stopped)));
    assert_eq!(nodes, 1);
    assert_eq!(counts.integrations.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn stopping_keeps_inflight_owned_nodes_until_domain_returns() {
    let counts = Arc::new(Counts::default());
    let context = ExecutionContext::new();
    let entered = Arc::new(tokio::sync::Notify::new());
    let release = Arc::new(tokio::sync::Notify::new());
    let mut rhythm = SimulationRhythm::logical(Engine { counts: counts.clone(), fail_at: None });
    let entry_signal = entered.clone();
    let release_wait = release.clone();
    let drive = rhythm.drive(
        vec![7],
        move |mut owned, (), _| {
            let entered = entry_signal.clone();
            let release = release_wait.clone();
            async move {
                entered.notify_one();
                release.notified().await;
                owned.push(8);
                (Ok(Completion::Completed(())), owned)
            }
        },
        (),
        (),
        context.clone(),
    );
    let stop = async {
        entered.notified().await;
        context.request_stop();
        tokio::task::yield_now().await;
        release.notify_one();
    };
    let ((result, owned), ()) =
        tokio::time::timeout(Duration::from_secs(2), async { tokio::join!(drive, stop) })
            .await
            .unwrap();
    assert!(matches!(result, Ok(Completion::Stopped)));
    assert_eq!(owned, [7, 8]);
    assert_eq!(counts.integrations.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn zero_budget_completes_but_preexisting_stop_takes_precedence() {
    let counts = Arc::new(Counts::default());
    let mut rhythm = SimulationRhythm::logical(Engine { counts: counts.clone(), fail_at: None })
        .with_max_steps(0);
    for stopped in [false, true] {
        let context = ExecutionContext::new();
        if stopped {
            context.request_stop();
        }
        let (result, owned) = rhythm
            .drive(
                99,
                |_, (), _| async { panic!("zero budget must not call downstream") },
                (),
                (),
                context,
            )
            .await;
        assert_eq!(owned, 99);
        assert_eq!(matches!(result, Ok(Completion::Stopped)), stopped);
        assert_eq!(matches!(result, Ok(Completion::Completed(()))), !stopped);
    }
    assert_eq!(counts.attempts.load(Ordering::SeqCst), 0);
}

#[tokio::test]
async fn downstream_error_preserves_owned_state_and_stops_context() {
    let counts = Arc::new(Counts::default());
    let context = ExecutionContext::new();
    let mut rhythm = SimulationRhythm::logical(Engine { counts: counts.clone(), fail_at: None });
    let (result, owned) = rhythm
        .drive(
            vec![1],
            |mut owned, (), _| async move {
                owned.push(2);
                (Err(RoplatError::Node("observer failed".into())), owned)
            },
            (),
            (),
            context.clone(),
        )
        .await;
    assert!(matches!(result, Err(RoplatError::Node(_))));
    assert_eq!(owned, [1, 2]);
    assert_eq!(rhythm.steps_completed(), 1);
    assert!(context.is_stopping());
    assert_eq!(counts.attempts.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn logical_pacing_yields_to_external_stop_task() {
    let counts = Arc::new(Counts::default());
    let context = ExecutionContext::new();
    let mut rhythm = SimulationRhythm::logical(Engine { counts: counts.clone(), fail_at: None });
    let drive = rhythm.drive(
        (),
        |(), (), _| async { (Ok(Completion::Completed(())), ()) },
        (),
        (),
        context.clone(),
    );
    let stop = async {
        while counts.integrations.load(Ordering::SeqCst) == 0 {
            tokio::task::yield_now().await;
        }
        context.request_stop();
    };
    let ((result, ()), ()) =
        tokio::time::timeout(Duration::from_secs(2), async { tokio::join!(drive, stop) })
            .await
            .unwrap();
    assert!(matches!(result, Ok(Completion::Stopped)));
    assert!(counts.integrations.load(Ordering::SeqCst) < 10);
}

#[tokio::test]
async fn final_finite_tick_does_not_schedule_an_overflowing_unused_deadline() {
    let counts = Arc::new(Counts::default());
    let mut rhythm = SimulationRhythm::new(
        Engine { counts: counts.clone(), fail_at: None },
        Duration::MAX,
    )
    .with_max_steps(1);
    let (result, ()) = rhythm
        .drive(
            (),
            |(), (), _| async { (Ok(Completion::Completed(())), ()) },
            (),
            (),
            ExecutionContext::new(),
        )
        .await;
    assert!(matches!(result, Ok(Completion::Completed(()))));
    assert_eq!(counts.integrations.load(Ordering::SeqCst), 1);
}
