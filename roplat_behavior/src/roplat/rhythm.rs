//! Device-paced roplat control, enabled with the `roplat` feature.
//!
//! The rhythm borrows its robot for the complete drive through `Resources`.
//! A System connects `&mut robot >> control.resources.0`; Input and Output are
//! `()`. The creating layer owns the robot throughout success, failure and stop,
//! and remains responsible for its lifecycle.
//! Framework failures and cooperative stops live in `Execution`, not in Feed:
//! a successful Feed remains `(Command, done)`. The creating layer owns object
//! lifecycle; entering a drive never activates, resets or shuts down its nodes.
//!
//! The device session remains blocking, including when its per-cycle callback
//! is async. This preserves the 0.6 contract; it does not promise fair polling
//! with sibling futures on the outer executor. An in-flight domain is always
//! awaited to recover `N`. Dropped futures and panics are outside that guarantee.
//!
//! A successfully completed Feed is committed for this cycle. A concurrent stop
//! is observed before the next callback; if this Feed already has `done = true`,
//! its final command is sent and normal `Completed(())` wins. A domain that cannot
//! produce a command must return `Stopped` or `Err` instead of a successful Feed.

use std::{future::Future, marker::PhantomData, ops::ControlFlow, time::Duration};

use roplat::{Completion, Execution, ExecutionContext, Lifecycle, RoplatError, rhythm::Rhythm};

use crate::{ControlExit, ControlSpace, ControlWith, RobotException};

/// Why a domain asked the device session to end without another command.
#[derive(Debug)]
pub enum ControlDomainExit {
    Stopped,
    Failed(RoplatError),
}

/// Device failure, retaining the preceding domain exit if there was one.
///
/// Enters the core through `RoplatError::Node`; `downcast_ref` on that
/// variant's payload recovers this concrete type, including the original
/// domain error. No wrapper is allocated when only the domain fails or when
/// control succeeds.
#[derive(Debug, thiserror::Error)]
#[error("control device/session failed: {device}; preceding domain exit: {domain:?}")]
pub struct ControlSessionError {
    pub domain: Option<ControlDomainExit>,
    #[source]
    pub device: RobotException,
}

pub(super) fn device_error(
    device: RobotException,
    domain: Option<ControlDomainExit>,
) -> RoplatError {
    RoplatError::Node(Box::new(ControlSessionError { domain, device }))
}

/// A blocking device session driving one complete domain per control cycle.
pub struct ControlRhythm<R, S> {
    _types: PhantomData<fn(R) -> S>,
}

impl<R, S> ControlRhythm<R, S> {
    pub fn new() -> Self {
        Self { _types: PhantomData }
    }
}

impl<R, S> Default for ControlRhythm<R, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R, S> Lifecycle for ControlRhythm<R, S> {
    type Error = RoplatError;
}

impl<R, S> Rhythm for ControlRhythm<R, S>
where
    R: ControlWith<S> + Send,
    S: ControlSpace + Send,
    S::Obs: Send,
    S::Command: Send,
{
    type Yield = (S::Obs, Duration);
    type Feed = (S::Command, bool);
    type Input = ();
    type Output = ();
    type Resources = (R,);

    async fn drive<'call, N, F, Fut>(
        &'call mut self,
        nodes: N,
        mut op_domain: F,
        _input: (),
        (robot,): (&'call mut R,),
        context: ExecutionContext,
    ) -> (Execution<Self::Output>, N)
    where
        N: Send,
        F: FnMut(N, Self::Yield, ExecutionContext) -> Fut + Send,
        Fut: Future<Output = (Execution<Self::Feed>, N)> + Send,
    {
        if context.is_stopping() {
            return (Ok(Completion::Stopped), nodes);
        }

        let mut nodes = Some(nodes);
        let mut domain_exit = None;
        let result =
            <R as ControlWith<S>>::control_with_flow_async(robot, async |obs, duration| {
                if context.is_stopping() {
                    domain_exit = Some(ControlDomainExit::Stopped);
                    return ControlFlow::Break(ControlExit::Stopped);
                }
                let current_nodes = nodes
                    .take()
                    .expect("control driver invoked an overlapping domain callback");
                let (execution, returned_nodes) =
                    op_domain(current_nodes, (obs, duration), context.clone()).await;
                // Restore ownership before inspecting the exit or asking the driver to stop.
                nodes = Some(returned_nodes);
                match execution {
                    Ok(Completion::Completed(feed)) => ControlFlow::Continue(feed),
                    Ok(Completion::Stopped) => {
                        context.request_stop();
                        domain_exit = Some(ControlDomainExit::Stopped);
                        ControlFlow::Break(ControlExit::Stopped)
                    }
                    Err(error) => {
                        context.request_stop();
                        domain_exit = Some(ControlDomainExit::Failed(error));
                        ControlFlow::Break(ControlExit::Stopped)
                    }
                }
            });
        let nodes = nodes.expect("control domain did not return its node state");
        let execution = match result {
            Err(error) => {
                context.request_stop();
                Err(device_error(error, domain_exit))
            }
            Ok(()) => match domain_exit {
                Some(ControlDomainExit::Failed(error)) => Err(error),
                Some(ControlDomainExit::Stopped) => Ok(Completion::Stopped),
                None => return (Ok(Completion::Completed(())), nodes),
            },
        };
        (execution, nodes)
    }
}
