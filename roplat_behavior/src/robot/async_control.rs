//! Native asynchronous control sessions, separate from the blocking 0.6 API.

use std::{future::Future, time::Duration};

use crate::{ControlSpace, ControlStep, Robot, RobotResult};

/// A statically dispatched, reusable controller whose invocation can borrow
/// its own state across await and whose invocation future is always `Send`.
///
/// Drivers await each call before invoking it again. The callback is borrowed
/// for the complete session, so the caller can recover its state on any ordinary
/// return. No boxed future, task spawn, or per-cycle state copy is required.
///
/// The `Duration` is the cycle's, as for [`ControlWith`](crate::ControlWith):
/// the nominal period, or `(k + 1)` periods after `k` missed cycles.
///
/// Ordinary `FnMut(Obs, Duration) -> Fut` closures implement this trait when
/// `Fut: Send`. A closure cannot always lend mutable captures into that future;
/// for such controllers implement this trait on a small named state struct.
pub trait AsyncControlCallback<Obs, Command>: Send {
    fn call(
        &mut self,
        observation: Obs,
        duration: Duration,
    ) -> impl Future<Output = ControlStep<Command>> + Send;
}

impl<Obs, Command, F, Fut> AsyncControlCallback<Obs, Command> for F
where
    F: FnMut(Obs, Duration) -> Fut + Send,
    Fut: Future<Output = ControlStep<Command>> + Send,
{
    fn call(
        &mut self,
        observation: Obs,
        duration: Duration,
    ) -> impl Future<Output = ControlStep<Command>> + Send {
        self(observation, duration)
    }
}

/// A driver's native asynchronous control capability for channel `S`.
///
/// Unlike [`crate::ControlWith::control_with_async`], this method returns the
/// complete session future. Drivers must await native asynchronous device I/O,
/// including session entry and termination, without blocking the calling
/// executor or creating a nested runtime. The driver's documentation specifies
/// which I/O runtime the future needs; this trait itself has no Tokio dependency.
///
/// Each callback is awaited to completion, and callbacks never overlap.
/// `Continue((command, done))` commits the valid command; `Break(exit)` sends no
/// algorithm command for that cycle and enters device-specific termination.
/// The session returns only after its termination protocol finishes or fails.
/// This is cooperative completion, not a guarantee for dropped futures or panic.
/// Device termination is not object lifecycle shutdown or an emergency stop.
///
/// This capability has no blocking default and does not require `ControlWith`.
/// Drivers opt in only when they can provide the complete asynchronous session.
pub trait AsyncControlWith<S: ControlSpace>: Robot + Send {
    fn control_native_async<C>(
        &mut self,
        callback: &mut C,
    ) -> impl Future<Output = RobotResult<()>> + Send
    where
        C: AsyncControlCallback<S::Obs, S::Command>;
}

/// Channel-parameterised convenience entry point for native async sessions.
pub trait AsyncControl: Robot + Send + Sized {
    fn control_native_async<S, C>(
        &mut self,
        callback: &mut C,
    ) -> impl Future<Output = RobotResult<()>> + Send
    where
        S: ControlSpace,
        Self: AsyncControlWith<S>,
        C: AsyncControlCallback<S::Obs, S::Command>,
    {
        <Self as AsyncControlWith<S>>::control_native_async(self, callback)
    }
}

impl<R: Robot + Send> AsyncControl for R {}
