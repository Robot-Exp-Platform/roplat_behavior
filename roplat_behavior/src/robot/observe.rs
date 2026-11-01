//! Robot-side observation helpers for control sessions.
//!
//! This module does not change [`ControlWith`](crate::ControlWith). It only
//! defines how a robot can accept observers before `control_with` is called:
//!
//! ```ignore
//! robot
//!     .before(|state: &FrankaState, dt| {
//!         // inspect the full robot state before the controller closure runs
//!     })
//!     .after(|state: &FrankaState, dt| {
//!         // inspect the full robot state after the controller closure runs
//!     })
//!     .control_with::<TorqueControl<7>, _>(controller)?;
//! ```
//!
//! The robot stores these observers itself. `roplat_behavior` deliberately does
//! not prescribe whether that storage is a `Vec`, a ring buffer handle, an
//! optional slot, or a driver-specific realtime telemetry channel.

use std::time::Duration;

use crate::Robot;

/// Per-cycle observer for a robot's full state.
///
/// The observer receives an immutable state reference and cannot alter the
/// controller command. It can still block, allocate or panic, so realtime
/// drivers should document how these observers are executed.
// dyn(c): observers of different types, registered by the driving side, share
// one list.
pub type ControlObserver<State> = Box<dyn FnMut(&State, Duration) + Send + 'static>;

/// User-facing methods for registering robot-level control observers.
///
/// This trait is intentionally not parameterized by control space. A robot owns
/// the observer storage, and each observer sees the robot's complete
/// [`Robot::State`]. The selected control space remains a concern of
/// [`control_with`](crate::Control::control_with).
pub trait ControlObservation: Robot + Sized {
    /// Register an observer that runs before the next/current control closure.
    fn before<H>(&mut self, observer: H) -> &mut Self
    where
        H: FnMut(&Self::State, Duration) + Send + 'static;

    /// Register an observer that runs after the next/current control closure.
    fn after<H>(&mut self, observer: H) -> &mut Self
    where
        H: FnMut(&Self::State, Duration) + Send + 'static;
}

#[cfg(test)]
mod tests {
    use super::{ControlObservation, ControlObserver};
    use crate::{Robot, RobotResult};
    use std::time::Duration;

    struct DummyRobot {
        before: Vec<ControlObserver<DummyState>>,
        after: Vec<ControlObserver<DummyState>>,
    }

    #[derive(Clone, Debug, Default)]
    struct DummyState {
        value: u32,
    }

    impl Robot for DummyRobot {
        type State = DummyState;

        const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);

        fn version() -> String {
            "dummy".to_string()
        }

        fn read_state(&mut self) -> RobotResult<Self::State> {
            Ok(DummyState::default())
        }
    }

    impl ControlObservation for DummyRobot {
        fn before<H>(&mut self, observer: H) -> &mut Self
        where
            H: FnMut(&Self::State, Duration) + Send + 'static,
        {
            let observer: ControlObserver<Self::State> = Box::new(observer);
            self.before.push(observer);
            self
        }

        fn after<H>(&mut self, observer: H) -> &mut Self
        where
            H: FnMut(&Self::State, Duration) + Send + 'static,
        {
            let observer: ControlObserver<Self::State> = Box::new(observer);
            self.after.push(observer);
            self
        }
    }

    #[test]
    fn robot_observation_methods_return_mut_self() {
        let mut robot = DummyRobot { before: Vec::new(), after: Vec::new() };

        robot
            .before(|state, _| {
                let _ = state.value;
            })
            .after(|state, _| {
                let _ = state.value;
            });

        assert_eq!(robot.before.len(), 1);
        assert_eq!(robot.after.len(), 1);
    }
}
