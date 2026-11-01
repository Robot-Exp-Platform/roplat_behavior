use std::{fmt, time::Duration};

/// The physical quantity a check bounds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Quantity {
    Position,
    Velocity,
    Acceleration,
    /// Joint torque or force.
    Effort,
    EffortRate,
    /// Command minus observed position.
    TrackingError,
}

/// Where a quantity was checked: one joint, or the norm of a Cartesian part.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Axis {
    Joint(usize),
    Linear,
    Angular,
}

impl fmt::Display for Quantity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Position => "position",
            Self::Velocity => "velocity",
            Self::Acceleration => "acceleration",
            Self::Effort => "effort",
            Self::EffortRate => "effort rate",
            Self::TrackingError => "tracking error",
        })
    }
}

impl fmt::Display for Axis {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Joint(index) => write!(f, "joint {index}"),
            Self::Linear => f.write_str("linear part"),
            Self::Angular => f.write_str("angular part"),
        }
    }
}

/// Why a safety node refused its input.
///
/// A refusal is never repaired into another value. Raised through `?` in a
/// control domain, it fails the cycle, so the device session gets `Break` and
/// sends no command for it.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SafetyViolation {
    #[error("command {quantity} of {axis} is not finite")]
    NonFinite { quantity: Quantity, axis: Axis },
    #[error("{quantity} of {axis} is {value}, outside [{min}, {max}]")]
    OutOfRange {
        quantity: Quantity,
        axis: Axis,
        value: f64,
        min: f64,
        max: f64,
    },
    #[error("{quantity} of {axis} is {value}, limit {max}")]
    Exceeded {
        quantity: Quantity,
        axis: Axis,
        value: f64,
        max: f64,
    },
    /// A check needs this measured quantity as its reference, and the
    /// observation lacks it or holds a non-finite value.
    #[error("observation has no finite {quantity} to check against")]
    Unobserved { quantity: Quantity },
    #[error("no sample has arrived")]
    NoSample,
    #[error("sample is {age:?} old, limit {max_age:?}")]
    Stale { age: Duration, max_age: Duration },
    /// The sample's clock differs from the gate's, or is unspecified.
    #[error("sample is on clock {sample}, gate reads clock {gate}")]
    ClockMismatch { sample: u32, gate: u32 },
    #[error("sample is stamped {ahead:?} ahead of the gate's clock")]
    FromFuture { ahead: Duration },
    #[error("{gap:?} between control cycles, limit {max:?}")]
    Overrun { gap: Duration, max: Duration },
    #[error("safety latch tripped: {reason}")]
    Tripped { reason: String },
}

#[cfg(feature = "roplat")]
mod transport {
    use roplat::RoplatError;
    use roplat_behavior::roplat::{ControlDomainExit, ControlSessionError};

    use super::SafetyViolation;

    /// Enters the core through `RoplatError::Node`, as `ControlSessionError`
    /// does; the concrete value survives.
    impl From<SafetyViolation> for RoplatError {
        fn from(violation: SafetyViolation) -> Self {
            RoplatError::Node(Box::new(violation))
        }
    }

    impl SafetyViolation {
        /// The violation that caused `error`, also when a parallel join, a
        /// lifecycle shutdown or a failed device cleanup wraps it.
        pub fn find(error: &RoplatError) -> Option<&Self> {
            match error {
                RoplatError::Node(inner) => {
                    if let Some(violation) = inner.downcast_ref::<Self>() {
                        return Some(violation);
                    }
                    match &inner.downcast_ref::<ControlSessionError>()?.domain {
                        Some(ControlDomainExit::Failed(error)) => Self::find(error),
                        _ => None,
                    }
                }
                RoplatError::Parallel(parallel) => parallel.failures.iter().find_map(Self::find),
                RoplatError::Lifecycle(lifecycle) => {
                    lifecycle.primary.as_deref().and_then(Self::find)
                }
                _ => None,
            }
        }
    }
}
