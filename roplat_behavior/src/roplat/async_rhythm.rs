//! Device-paced control using the driver's native asynchronous session.

use std::{future::Future, marker::PhantomData, ops::ControlFlow, time::Duration};

use roplat::{Completion, Execution, ExecutionContext, Lifecycle, RoplatError, rhythm::Rhythm};

use crate::{AsyncControlCallback, AsyncControlWith, ControlExit, ControlSpace, ControlStep};

use super::rhythm::{ControlDomainExit, device_error};

/// A native async device session, polled by the caller's executor.
///
/// The driver must implement [`AsyncControlWith`]; blocking-only `ControlWith`
/// implementations do not opt in automatically. The adapter does not create a
/// runtime, spawn a task, box a future, or clone the node tuple per cycle.
///
/// Like [`super::ControlRhythm`], successful Feed is committed for the current
/// cycle even if a concurrent stop was requested. Stop is observed before the
/// next callback. `done = true` sends its final command and completes normally.
/// A domain that has no command must return `Stopped` or `Err`.
///
/// An in-flight domain is always awaited to recover `N`. Creating scopes retain
/// lifecycle responsibility. The robot is borrowed through `Resources` for
/// the entire drive; its owner retains it on completion, failure and stop.
/// Dropping this future or panic does not promise
/// asynchronous shutdown or node return. Device I/O/runtime requirements and
/// wait timeouts remain part of the driver's native session contract.
pub struct AsyncControlRhythm<R, S> {
    _types: PhantomData<fn(R) -> S>,
}

impl<R, S> AsyncControlRhythm<R, S> {
    pub fn new() -> Self {
        Self { _types: PhantomData }
    }
}

impl<R, S> Default for AsyncControlRhythm<R, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R, S> Lifecycle for AsyncControlRhythm<R, S> {
    type Error = RoplatError;
}

/// The callback owns the state while a domain is pending, then restores it
/// before asking the driver to terminate. It is borrowed for the entire session.
struct DomainCallback<N, F> {
    nodes: Option<N>,
    op_domain: F,
    context: ExecutionContext,
    exit: Option<ControlDomainExit>,
}

impl<N, F, Fut, Obs, Command> AsyncControlCallback<Obs, Command> for DomainCallback<N, F>
where
    N: Send,
    Obs: Send,
    Command: Send,
    F: FnMut(N, (Obs, Duration), ExecutionContext) -> Fut + Send,
    Fut: Future<Output = (Execution<(Command, bool)>, N)> + Send,
{
    async fn call(&mut self, observation: Obs, duration: Duration) -> ControlStep<Command> {
        if self.context.is_stopping() {
            self.exit = Some(ControlDomainExit::Stopped);
            return ControlFlow::Break(ControlExit::Stopped);
        }
        let current_nodes = self
            .nodes
            .take()
            .expect("control driver invoked an overlapping domain callback");
        let (execution, returned_nodes) =
            (self.op_domain)(current_nodes, (observation, duration), self.context.clone()).await;
        self.nodes = Some(returned_nodes);
        match execution {
            Ok(Completion::Completed(feed)) => ControlFlow::Continue(feed),
            Ok(Completion::Stopped) => {
                self.context.request_stop();
                self.exit = Some(ControlDomainExit::Stopped);
                ControlFlow::Break(ControlExit::Stopped)
            }
            Err(error) => {
                self.context.request_stop();
                self.exit = Some(ControlDomainExit::Failed(error));
                ControlFlow::Break(ControlExit::Stopped)
            }
        }
    }
}

impl<R, S> Rhythm for AsyncControlRhythm<R, S>
where
    R: AsyncControlWith<S>,
    S: ControlSpace + Send,
    S::Obs: Send,
    S::Command: Send,
{
    type Yield = (S::Obs, Duration);
    type Feed = (S::Command, bool);
    type Input = ();
    type Output = ();
    type Resources = (R,);

    // Keep the explicit Send return bound: an async-fn implementation loses
    // the bound in nested System lifetime inference on the current toolchain.
    // The nested_system test includes assert_send to prevent this regression.
    #[allow(clippy::manual_async_fn)]
    fn drive<'call, N, F, Fut>(
        &'call mut self,
        nodes: N,
        op_domain: F,
        _input: (),
        (robot,): (&'call mut R,),
        context: ExecutionContext,
    ) -> impl Future<Output = (Execution<()>, N)> + Send
    where
        N: Send,
        F: FnMut(N, Self::Yield, ExecutionContext) -> Fut + Send,
        Fut: Future<Output = (Execution<Self::Feed>, N)> + Send,
    {
        async move {
            if context.is_stopping() {
                return (Ok(Completion::Stopped), nodes);
            }
            let mut callback = DomainCallback {
                nodes: Some(nodes),
                op_domain,
                context: context.clone(),
                exit: None,
            };
            let result =
                <R as AsyncControlWith<S>>::control_native_async(robot, &mut callback).await;
            let nodes = callback
                .nodes
                .expect("control domain did not return its node state");
            let execution = match result {
                Err(error) => {
                    context.request_stop();
                    Err(device_error(error, callback.exit))
                }
                Ok(()) => match callback.exit {
                    Some(ControlDomainExit::Failed(error)) => Err(error),
                    Some(ControlDomainExit::Stopped) => Ok(Completion::Stopped),
                    None => return (Ok(Completion::Completed(())), nodes),
                },
            };
            (execution, nodes)
        }
    }
}
