//! End-effector and sensor contracts.
//!
//! These are not [`Robot`](crate::Robot) subtraits: a gripper or sensor may be
//! part of an arm driver (Franka's hand) or a device of its own, so a driver
//! implements them on whichever type owns the connection. Readings carry a
//! [`Header`](crate::Header) with the producer's clock and frame.

use roplat_msgs::{CameraInfo, ForceTorque, GripperCommand, GripperState, Header, Imu, Stamped};

use crate::{RobotException, RobotResult};

/// A parallel or adaptive gripper driven by width, speed and force.
pub trait Gripper {
    fn read_gripper(&mut self) -> RobotResult<GripperState>;

    /// Moves to `command.width`; a positive `command.force` grasps. Returns
    /// when the motion ends; `GripperState::grasped` reports the outcome.
    fn move_gripper(&mut self, command: GripperCommand) -> RobotResult<()>;

    /// Calibrates the finger range, when the device needs it.
    fn home_gripper(&mut self) -> RobotResult<()> {
        Ok(())
    }

    /// Stops the current motion or releases a held grasp force.
    fn stop_gripper(&mut self) -> RobotResult<()> {
        Err(RobotException::UnprocessableInstructionError(
            "stopping the gripper is not implemented for this driver".to_string(),
        ))
    }
}

/// A six-axis force/torque sensor; the wrench is in the sensor frame named by
/// the header.
pub trait ForceTorqueSensor {
    fn read_wrench(&mut self) -> RobotResult<ForceTorque>;

    /// Zeroes the current reading (tare), when the device supports it.
    fn bias_wrench(&mut self) -> RobotResult<()> {
        Err(RobotException::UnprocessableInstructionError(
            "biasing the wrench is not implemented for this driver".to_string(),
        ))
    }
}

/// An inertial measurement unit; absent fields are cleared in `Imu`'s mask.
pub trait ImuSensor {
    fn read_imu(&mut self) -> RobotResult<Stamped<Imu>>;
}

/// A camera with fixed intrinsics.
///
/// The frame type is the driver's until the shared image types land with the
/// data-stream work; the header carries the capture time and optical frame.
pub trait Camera {
    type Frame;

    fn camera_info(&self) -> CameraInfo;

    /// Blocks for the next frame.
    fn capture(&mut self) -> RobotResult<Stamped<Self::Frame>>;
}

/// One return of a [`Lidar`] scan.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LidarPoint {
    /// Metres, in the lidar frame that [`Lidar`] defines.
    pub position: [f64; 3],
    /// Nanoseconds from the scan's stamp to this return, as the device
    /// measured them.
    pub offset_ns: u32,
    /// Reflectivity or intensity in the device's own units; `None` when the
    /// device reports none.
    pub intensity: Option<f32>,
    /// The beam, ranked by elevation from the lowest (0); `None` when the
    /// device's beams have no fixed elevation, as in non-repetitive patterns.
    /// A planar scanner's one beam is 0.
    pub ring: Option<u16>,
}

/// A scanning range sensor that delivers complete scans of returns.
///
/// # Lidar frame
///
/// Every implementation reports points in one right-handed frame fixed to the
/// sensor:
///
/// - +X points to the sensor's front, the direction its maker documents as
///   the front;
/// - +Z points up, out of the top of the housing and away from its mounting
///   face;
/// - +Y points left, completing the frame;
/// - azimuth runs counter-clockwise about +Z from +X, so +Y is at +90°; a
///   planar scanner's points lie in the plane z = 0.
///
/// The origin is that of the maker's own sensor frame, which each driver
/// names. A driver whose maker uses other axes rotates its points into this
/// frame and documents the rotation; it never mirrors them.
///
/// # Scans
///
/// A scan is one rotation of a spinning sensor, one sweep of a planar
/// scanner, or one integration period of a non-repetitive pattern. It holds
/// returns only: a ray without a return is left out, never reported as a
/// point at zero or infinite range. The points come in no promised order;
/// [`LidarPoint::offset_ns`] gives each one's time.
///
/// The header's stamp is the time of the scan's earliest return, on the
/// clock the driver stamps with, so that return's offset is 0. A scan with no
/// return in range still comes back, with no points, stamped when it ended.
/// Each driver documents how it places the device's time on that clock. `seq`
/// rises with every scan, and `frame` names the lidar frame.
pub trait Lidar {
    /// Blocks for the next complete scan, replaces the contents of `points`
    /// with its returns, and returns its header.
    ///
    /// The buffer belongs to the caller, so a loop that reuses it allocates
    /// only when a scan outgrows every scan before it.
    fn read_scan(&mut self, points: &mut Vec<LidarPoint>) -> RobotResult<Header>;
}
