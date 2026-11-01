//! Controllers for `roplat_behavior` robots: joint PD/PID and base-velocity
//! PID ([`pid`]), joint and Cartesian impedance ([`impedance`]), gravity
//! compensation and computed torque ([`dynamics`]), operator teleoperation
//! ([`teleop`]), Cartesian servoing ([`servo`]), following a timed tool-pose
//! trajectory ([`cartesian_follow`]), the target sources the tracking
//! controllers take ([`target`]), the signal filters a control loop needs
//! ([`filter`]), and helpers to raise a control thread to real-time priority.
//!
//! Every controller is a closure that returns a `ControlStep`: a command for
//! the cycle, or `Break` when the observation lacks a measured quantity the
//! control law needs or a model query fails. No value is fabricated in their
//! place.

pub mod cartesian_follow;
pub mod dynamics;
mod exit;
pub mod filter;
pub mod impedance;
pub mod pid;
mod realtime;
pub mod servo;
pub mod target;
pub mod teleop;

pub use cartesian_follow::*;
pub use dynamics::*;
pub use impedance::*;
pub use pid::*;
pub use realtime::*;
pub use servo::*;
pub use target::*;
pub use teleop::*;
