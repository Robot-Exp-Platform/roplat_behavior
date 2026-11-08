//! Multirotors flown through position setpoints.

use std::time::Duration;

use roplat_msgs::{Quaternion, Stamped, Vector3};

use crate::{Robot, RobotResult};

/// What the flight controller reports about ground contact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LandedState {
    OnGround,
    TakingOff,
    InAir,
    Landing,
}

/// One estimate of a multirotor's motion, as its flight controller reports
/// it, in the frames [`Multirotor`] defines. A field the device did not
/// report is `None`, never zero.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MultirotorState {
    /// Time since the flight controller booted, of the position estimate.
    pub device_time: Duration,
    /// Metres, world frame.
    pub position: Vector3,
    /// m/s, world frame.
    pub velocity: Option<Vector3>,
    /// The FLU body in the world frame, taking body vectors to world vectors.
    pub orientation: Option<Quaternion>,
    /// rad/s about the FLU body axes.
    pub angular_velocity: Option<Vector3>,
    pub armed: Option<bool>,
    pub landed: Option<LandedState>,
}

impl MultirotorState {
    /// The heading of the body's forward axis, radians counter-clockwise
    /// about world +Z from world +X, when the attitude is known.
    pub fn yaw(&self) -> Option<f64> {
        self.orientation.map(|q| {
            let forward_y = 2.0 * (q.w * q.z + q.x * q.y);
            let forward_x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            forward_y.atan2(forward_x)
        })
    }
}

/// A position to fly to and hold, in the world frame of [`Multirotor`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionSetpoint {
    /// Metres.
    pub position: Vector3,
    /// Radians counter-clockwise about world +Z from world +X; `None` keeps
    /// the heading the vehicle has.
    pub yaw: Option<f64>,
}

impl PositionSetpoint {
    pub const fn new(position: [f64; 3], yaw: Option<f64>) -> Self {
        Self {
            position: Vector3::new(position[0], position[1], position[2]),
            yaw,
        }
    }
}

/// A multirotor flown through position setpoints.
///
/// # Frames
///
/// - The world is the flight controller's local frame with east-north-up
///   axes, +X east, +Y north and +Z up, and its origin wherever the
///   controller put it: the EKF origin for PX4 and ArduPilot, the
///   positioning system's origin on a Crazyflie, or the start point without
///   one. Each driver names it.
/// - The body frame is forward-left-up at the controller's reference point,
///   as for [`ImuSensor`](crate::ImuSensor).
/// - Yaw runs counter-clockwise about +Z from world +X.
///
/// A driver whose autopilot uses north-east-down and forward-right-down
/// rotates into these with [`ned_to_enu`](crate::ned_to_enu),
/// [`frd_to_flu`](crate::frd_to_flu) and
/// [`ned_frd_to_enu_flu`](crate::ned_frd_to_enu_flu); an autopilot's yaw
/// clockwise from north is `π/2` minus it, wrapped.
///
/// # States
///
/// [`read_flight_state`](Self::read_flight_state) returns one state per
/// position estimate, stamped on the driver's clock when it arrived; the
/// device's own time is [`MultirotorState::device_time`]. The attitude and
/// rates in it are those reported within one telemetry period of the
/// position, else `None`. Without a control channel,
/// [`CONTROL_PERIOD`](Robot::CONTROL_PERIOD) is the period of these
/// estimates.
///
/// # Lifecycle and flight
///
/// From [`Robot`]: `enable` arms; `disable` disarms on the ground, which a
/// flight controller refuses in the air; `emergency_stop` stops the motors
/// wherever the vehicle is.
///
/// The blocking calls ([`take_off`](Self::take_off),
/// [`go_to`](Self::go_to), [`land`](Self::land)) read states and keep
/// sending what the device needs until they are done, or until a timeout of
/// device time the driver documents has passed.
/// [`send_setpoint`](Self::send_setpoint) sends one setpoint. A caller
/// streaming its own must send at least as often as the device requires
/// (PX4 offboard faster than 2 Hz, a Crazyflie within 0.5 s), or the device
/// falls back to its failsafe.
pub trait Multirotor: Robot {
    /// Blocks for the next position estimate.
    fn read_flight_state(&mut self) -> RobotResult<Stamped<MultirotorState>>;

    /// Arms if needed and climbs `height` metres above where the vehicle
    /// is; returns once within the driver's tolerance of that point.
    fn take_off(&mut self, height: f64) -> RobotResult<()>;

    /// Sends one setpoint and returns.
    fn send_setpoint(&mut self, setpoint: PositionSetpoint) -> RobotResult<()>;

    /// Flies to `setpoint` and returns the state once the vehicle is within
    /// the driver's tolerance of it and slower than its settle speed, both
    /// documented; the vehicle holds it afterwards only while setpoints keep
    /// coming.
    fn go_to(&mut self, setpoint: PositionSetpoint) -> RobotResult<MultirotorState>;

    /// Lands where the vehicle is; returns once it is on the ground with its
    /// motors stopped.
    fn land(&mut self) -> RobotResult<()>;
}
