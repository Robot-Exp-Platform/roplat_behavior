//! How the controllers end a session without a command.

use roplat_behavior::{ControlExit, RobotException};

/// The exit of a controller whose observation lacks a measured quantity.
/// Nothing is fabricated in its place; the session ends with this error.
pub(crate) fn missing(quantity: &'static str) -> ControlExit {
    ControlExit::Failed(RobotException::UnprocessableInstructionError(format!(
        "the controller needs the measured {quantity}"
    )))
}
