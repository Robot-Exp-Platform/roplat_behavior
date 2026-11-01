//! roplat adapters for roplat_behavior capabilities.
//!
//! This module keeps roplat-specific execution glue out of the core behavior
//! traits. Drivers still implement `Robot`, `MoveTo`, `ControlWith`, `SpaceMap`
//! and related traits; these adapters make those capabilities usable as roplat
//! `Rhythm` domains and `Node`s. `AsyncControlRhythm` additionally requires the
//! explicit native asynchronous capability `AsyncControlWith`.

pub mod async_rhythm;
pub mod node;
pub mod rhythm;

pub use async_rhythm::*;
pub use node::*;
pub use rhythm::*;

/// Lets a graph raise a device result with `?`, e.g. `motion.1?;` after the
/// robot has been returned. The exception enters the core through
/// `RoplatError::Node`; `downcast_ref::<RobotException>()` on that variant's
/// payload recovers it.
impl From<crate::RobotException> for ::roplat::RoplatError {
    fn from(error: crate::RobotException) -> Self {
        ::roplat::RoplatError::Node(Box::new(error))
    }
}
