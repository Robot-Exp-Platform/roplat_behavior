use std::{any::type_name, io};
use thiserror::Error;

#[derive(Error, Debug)]
/// The unified error type for every fallible robot operation.
///
/// Most driver and behavior APIs return [`RobotResult<T>`]. `From` impls are
/// provided for [`io::Error`], [`anyhow::Error`] and [`serde_json::Error`] so
/// the `?` operator works across transport, command and (de)serialization
/// failures. With the `to_py` feature it also converts into a Python exception.
pub enum RobotException {
    /// A control operation failed and its device-specific cleanup also failed.
    /// Allocation occurs only on this double-failure path.
    #[error("{primary}; control session cleanup also failed: {cleanup}")]
    ControlSession {
        #[source]
        primary: Box<RobotException>,
        cleanup: Box<RobotException>,
    },

    /// Sentinel "no error" value (not an actual failure).
    #[error("none")]
    NoException,

    /// ModelException is thrown if an error occurs when loading the model library.
    #[error("Model exception: {0}")]
    ModelException(String),

    /// NetworkException is thrown if a connection to the robot cannot be established, or when a timeout occurs.
    #[error("Network exception: {0}")]
    NetworkError(String),

    /// IncompatibleVersionException is thrown if the robot does not support this version.
    #[error(
        "Incompatible version: server version {server_version}, client version {client_version}"
    )]
    IncompatibleVersionException {
        server_version: u64,
        client_version: u64,
    },

    /// RealtimeException is thrown if realtime priority cannot be set.
    #[error("Realtime exception: {0}")]
    RealtimeException(String),

    /// Unprocessable instruction error
    #[error("Unprocessable instruction error: {0}")]
    UnprocessableInstructionError(String),

    /// Conflicting instruction error
    #[error("Conflicting instruction error: {0}")]
    ConflictingInstruction(String),

    /// CommandException is thrown if an error occurs during command execution.
    #[error("Command exception: {0}")]
    CommandException(String),

    /// Invalid instruction error
    #[error("Invalid instruction error: {0}")]
    InvalidInstruction(String),

    /// Deserialize error
    #[error("Deserialize error: {0}")]
    DeserializeError(String),

    /// InvalidFFIData error
    #[error("Invalid FFI data error: {0}")]
    InvalidFFIData(String),

    /// unwarp error
    #[error("UnWarp error: {0}")]
    UnWarpError(String),

    /// The device cannot do `operation` at all: it has no such function, or
    /// the driver offers none. Retrying, or another argument, does not help.
    #[error("{operation} is not supported by this device")]
    NotSupported { operation: &'static str },

    /// The robot stopped, but the device cannot latch an emergency stop, so
    /// nothing keeps it stopped; see
    /// [`Robot::emergency_stop`](crate::Robot::emergency_stop).
    #[error("the robot stopped but cannot latch an emergency stop")]
    NoEmergencyStop,
}

/// A convenient alias for `Result<T, `[`RobotException`]`>`.
pub type RobotResult<T> = Result<T, RobotException>;

impl RobotException {
    /// An algorithm crate's error (trajectory planning, kinematics, ...) as an
    /// [`UnprocessableInstructionError`](Self::UnprocessableInstructionError)
    /// keeping its message, for `plan(...).map_err(RobotException::from_algorithm)?`.
    pub fn from_algorithm(error: impl std::fmt::Display) -> Self {
        Self::UnprocessableInstructionError(error.to_string())
    }
}

impl From<io::Error> for RobotException {
    fn from(e: io::Error) -> Self {
        RobotException::NetworkError(e.to_string())
    }
}

impl From<anyhow::Error> for RobotException {
    fn from(e: anyhow::Error) -> Self {
        RobotException::CommandException(e.to_string())
    }
}

impl From<serde_json::Error> for RobotException {
    fn from(e: serde_json::Error) -> Self {
        RobotException::DeserializeError(e.to_string())
    }
}

/// Build a closure that maps any error into a [`RobotException::DeserializeError`],
/// embedding the target type name and the offending `data` string. Handy as the
/// argument to [`Result::map_err`] when decoding `data`.
pub fn deserialize_error<T, E>(data: &str) -> impl FnOnce(E) -> RobotException {
    move |_| {
        RobotException::DeserializeError(format!("exception {}, find {}", type_name::<T>(), data))
    }
}

/// Construct a [`RobotException::InvalidFFIData`] describing the unexpected
/// `data` and its type name.
pub fn invalid_ffi<T: std::fmt::Debug>(data: T) -> RobotException {
    RobotException::InvalidFFIData(format!("exception {}, find {:?}", type_name::<T>(), data))
}
