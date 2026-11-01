// Adapted from rsbullet's SimRhythm; upstream MIT notice is preserved in
// licenses/rsbullet-MIT.txt. New behavior and tests were added during migration.
use std::error::Error;
use std::future::Future;
use std::time::{Duration, Instant};

use crate::PhysicsEngine;
use futures::future::{Either, select};
use roplat::rhythm::Rhythm;
use roplat::{Completion, Execution, ExecutionContext, Lifecycle, RoplatError};

/// How a simulation waits between integrations. This does not change its physics dt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimPacing {
    /// Integrate as fast as the engine and downstream domain allow.
    LogicalTime,
    /// Pace tick starts against a monotonic clock, with the first tick immediate.
    /// Late ticks run immediately; no physics steps are skipped.
    WallClock(Duration),
}

/// The wall-clock pacing deadline overflowed `Instant`.
#[derive(Debug)]
pub struct PacingOverflow;

impl std::fmt::Display for PacingOverflow {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("simulation pacing deadline overflow")
    }
}

impl std::error::Error for PacingOverflow {}

/// Owns an engine and emits one unit tick after every successful physics step.
///
/// The physics timestep belongs to the engine (`PhysicsEngine::set_step_time`),
/// independently of wall-clock pacing. The backend defines its command and state
/// refresh ordering. This rhythm emits a downstream tick only after `step` succeeds.
///
/// A finite budget applies **per drive**. A second drive continues the engine's
/// world without resetting it, and gets a new budget and pacing origin.
/// Engine errors stop the domain and preserve the original error behind
/// `RoplatError::Node`; the downstream domain is not called for that failed step.
///
/// The engine stays owned by this rhythm; its external `Resources` channel is `()`.
/// Entry/exit do not reset the engine or call its consuming `shutdown` method.
/// The backend must provide RAII cleanup, or the caller can use `into_engine()`
/// to manage explicit teardown outside the graph.
pub struct SimulationRhythm<E> {
    engine: E,
    pacing: SimPacing,
    max_steps: Option<u64>,
    steps_completed: u64,
}

impl<E> SimulationRhythm<E> {
    /// Retains the original wall-clock paced construction API.
    pub fn new(engine: E, interval: Duration) -> Self {
        Self::with_pacing(engine, SimPacing::WallClock(interval))
    }

    /// Run without wall-clock delays. Set a budget or arrange cooperative stopping.
    pub fn logical(engine: E) -> Self {
        Self::with_pacing(engine, SimPacing::LogicalTime)
    }

    pub fn with_pacing(engine: E, pacing: SimPacing) -> Self {
        Self { engine, pacing, max_steps: None, steps_completed: 0 }
    }

    /// Complete normally after at most `steps` successful `PhysicsEngine::step` calls per drive.
    /// Zero is a valid empty drive. A stopped context takes precedence.
    pub fn with_max_steps(mut self, steps: u64) -> Self {
        self.max_steps = Some(steps);
        self
    }

    /// Successful engine steps in the current or most recent drive.
    /// Includes a step whose downstream domain subsequently failed or stopped.
    /// A backend may partially advance before reporting an error. Its own state
    /// and counters describe that partial progress; this counter does not.
    pub fn steps_completed(&self) -> u64 {
        self.steps_completed
    }

    pub fn engine(&self) -> &E {
        &self.engine
    }

    pub fn engine_mut(&mut self) -> &mut E {
        &mut self.engine
    }

    pub fn into_engine(self) -> E {
        self.engine
    }
}

impl<E: Send> Lifecycle for SimulationRhythm<E> {
    type Error = RoplatError;
}

impl<E> Rhythm for SimulationRhythm<E>
where
    E: PhysicsEngine + Send,
    E::Error: Error + Send + Sync + 'static,
{
    type Yield = ();
    type Feed = ();
    type Input = ();
    type Output = ();
    type Resources = ();

    async fn drive<N, F, Fut>(
        &mut self,
        mut nodes: N,
        mut op_domain: F,
        _input: Self::Input,
        _resources: (),
        context: ExecutionContext,
    ) -> (Execution<()>, N)
    where
        N: Send,
        F: FnMut(N, Self::Yield, ExecutionContext) -> Fut + Send,
        Fut: Future<Output = (Execution<Self::Feed>, N)> + Send,
    {
        self.steps_completed = 0;
        let mut next_target = Instant::now();

        loop {
            if context.is_stopping() {
                return (Ok(Completion::Stopped), nodes);
            }
            if self
                .max_steps
                .is_some_and(|limit| self.steps_completed >= limit)
            {
                return (Ok(Completion::Completed(())), nodes);
            }

            // Only cancel a device wait. Never drop the domain future that owns N.
            if matches!(self.pacing, SimPacing::WallClock(_))
                && stop_before_tick(&context, next_target).await
            {
                return (Ok(Completion::Stopped), nodes);
            }

            if context.is_stopping() {
                return (Ok(Completion::Stopped), nodes);
            }
            if let Err(error) = self.engine.step() {
                context.request_stop();
                return (Err(RoplatError::Node(error.into())), nodes);
            }
            self.steps_completed += 1;

            let (outcome, returned_nodes) = op_domain(nodes, (), context.clone()).await;
            nodes = returned_nodes;
            match outcome {
                Ok(Completion::Completed(())) => {}
                Ok(Completion::Stopped) => {
                    context.request_stop();
                    return (Ok(Completion::Stopped), nodes);
                }
                Err(error) => {
                    context.request_stop();
                    return (Err(error), nodes);
                }
            }

            if self
                .max_steps
                .is_some_and(|limit| self.steps_completed >= limit)
            {
                // Let the loop apply stop precedence without scheduling another tick.
                continue;
            }
            if let SimPacing::WallClock(interval) = self.pacing {
                match next_target.checked_add(interval) {
                    Some(target) => next_target = target,
                    None => {
                        context.request_stop();
                        return (Err(RoplatError::Node(Box::new(PacingOverflow))), nodes);
                    }
                }
            }
            // Immediate downstream futures must not starve an external stop task.
            tokio::task::yield_now().await;
        }
    }
}

// Pin both wait futures on the stack: pacing must not allocate two boxes per tick.
async fn stop_before_tick(context: &ExecutionContext, target: Instant) -> bool {
    let stopped = std::pin::pin!(context.stopped());
    let timer = std::pin::pin!(tokio::time::sleep_until(target.into()));
    matches!(select(stopped, timer).await, Either::Left(_))
}
