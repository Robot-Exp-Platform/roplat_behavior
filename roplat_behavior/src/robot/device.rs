//! End-effector and sensor contracts.
//!
//! These are not [`Robot`](crate::Robot) subtraits: a gripper or sensor may be
//! part of an arm driver (Franka's hand) or a device of its own, so a driver
//! implements them on whichever type owns the connection. Readings carry a
//! [`Header`](crate::Header) with the producer's clock and frame.

use std::f64::consts::FRAC_1_SQRT_2;

use roplat_msgs::{
    CameraInfo, ForceTorque, GripperCommand, GripperState, Header, Imu, Quaternion, Stamped,
    Vector3,
};

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

/// An inertial measurement unit.
///
/// # Body frame
///
/// Every implementation reports in one right-handed frame fixed to the
/// device, with the axes of the [`Lidar`] frame, forward-left-up (FLU):
///
/// - +X points to the device's front, the direction its maker documents as
///   the front, or the maker's +X axis when the maker marks axes but no
///   front;
/// - +Z points up, out of the top of the housing and away from its mounting
///   face;
/// - +Y points left, completing the frame.
///
/// What the maker documents means the same as for the [`Lidar`] frame: its
/// mechanical drawing or datasheet. The origin is that of the maker's own
/// sensor frame, which each driver names, and the header's `frame` names this
/// body frame. A driver whose
/// device uses other axes rotates its readings into this frame and documents
/// the rotation; it never mirrors them. A device marked forward-right-down
/// (FRD) needs the half turn about +X that [`frd_to_flu`] applies.
///
/// # Readings
///
/// A field the device does not report is cleared in `Imu`'s mask, never
/// filled with a stand-in value.
///
/// - `linear_acceleration` is the specific force in the body frame, m/s²:
///   what an accelerometer reads. Gravity is not removed, so a device at rest
///   reads about 9.81 m/s² pointing up, `(0, 0, +9.81)` when it is level, and
///   a falling one reads zero. A driver whose device can also output
///   acceleration with gravity removed reports the specific force.
/// - `angular_velocity` is the body's rate of turn in the body frame, rad/s:
///   what a gyroscope reads. The frame is right-handed, so turning left
///   (counter-clockwise seen from above) reads positive about +Z.
/// - `orientation` is the rotation of the body frame relative to an
///   east-north-up (ENU) world frame: the unit quaternion `q` that takes body
///   vectors to world vectors, `v_world = q v_body q*`. World +Z is up. When
///   the device references north, magnetic or true as its driver documents,
///   world +X is east and +Y north: a level device facing east reads the
///   identity, and one facing north a quarter turn about +Z. For a device
///   without a north reference, world +Z is still up and the driver documents
///   where heading zero is. `q` and `-q` are the same rotation; neither sign
///   is promised. A device that reports its FRD body relative to
///   north-east-down (NED) converts with [`ned_frd_to_enu_flu`]: NED to ENU
///   is a rotation too, never a mirror.
///
/// Whether a driver keeps these axes needs a known scene, so its own
/// simulation tests check them; `conformance::check_imu` (feature
/// `conformance`) checks what holds in every scene.
pub trait ImuSensor {
    fn read_imu(&mut self) -> RobotResult<Stamped<Imu>>;
}

/// A body vector from forward-right-down (FRD) axes into the forward-left-up
/// (FLU) axes of the [`ImuSensor`] body frame: the half turn about +X, which
/// takes `(x, y, z)` to `(x, -y, -z)`.
///
/// It is a rotation, not a mirror, so angular velocity converts as specific
/// force does. A half turn is its own inverse: the same call takes FLU vectors
/// to FRD, as a simulator that encodes device messages needs.
///
/// ```
/// use roplat_behavior::{Vector3, frd_to_flu};
///
/// // A level FRD device at rest reads its specific force on -Z.
/// let at_rest = frd_to_flu(Vector3::new(0.0, 0.0, -9.81));
/// assert_eq!(at_rest, Vector3::new(0.0, 0.0, 9.81));
/// ```
pub const fn frd_to_flu(v: Vector3) -> Vector3 {
    Vector3::new(v.x, -v.y, -v.z)
}

/// A world vector from north-east-down (NED) axes into east-north-up (ENU):
/// the half turn about the horizontal axis halfway between north and east,
/// which takes `(n, e, d)` to `(e, n, -d)`.
///
/// Like [`frd_to_flu`], it is a rotation and its own inverse.
pub const fn ned_to_enu(v: Vector3) -> Vector3 {
    Vector3::new(v.y, v.x, -v.z)
}

/// An orientation from the frames many devices report in, an FRD body
/// relative to NED, into those of [`ImuSensor`], the FLU body relative to ENU.
///
/// With `q` taking FRD vectors to NED vectors, the result takes FLU vectors to
/// ENU vectors. It is `q` between the half turns of [`ned_to_enu`] and
/// [`frd_to_flu`], in Hamilton products of quaternions `(w, x, y, z)`:
///
/// ```text
/// q_ENU<-FLU = q_ENU<-NED * q * q_FRD<-FLU
/// q_ENU<-NED = (0, 1, 1, 0) / sqrt(2)    half turn about (1, 1, 0) / sqrt(2)
/// q_FRD<-FLU = (0, -1, 0, 0)             half turn about X
///
/// (w, x, y, z)  ->  (w + z, x + y, x - y, w - z) / sqrt(2)
/// ```
///
/// The map is linear and keeps the norm, so normalising before or after it
/// gives the same result. Each half turn squares to `-1`, so the map is its own
/// inverse: the same call takes an FLU body relative to ENU back to an FRD body
/// relative to NED.
///
/// ```
/// use roplat_behavior::{Quaternion, ned_frd_to_enu_flu};
///
/// // Level and facing north: the identity relative to NED, and a quarter
/// // turn about +Z relative to ENU.
/// let s = std::f64::consts::FRAC_1_SQRT_2;
/// let north = ned_frd_to_enu_flu(Quaternion::IDENTITY);
/// assert_eq!(north, Quaternion { x: 0.0, y: 0.0, z: s, w: s });
/// ```
pub const fn ned_frd_to_enu_flu(q: Quaternion) -> Quaternion {
    let s = FRAC_1_SQRT_2;
    Quaternion {
        x: s * (q.x + q.y),
        y: s * (q.x - q.y),
        z: s * (q.w - q.z),
        w: s * (q.w + q.z),
    }
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
/// What the maker documents is its mechanical drawing or datasheet. The
/// maker's own driver and its ROS frame are only references: where they
/// disagree with the drawing, the driver follows the drawing and says so in
/// its README.
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

/// How the receiver solved a [`GnssFix`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GnssStatus {
    /// No solution this epoch: the fix has no position.
    NoFix,
    /// A standalone solution from the satellites' signals alone.
    Single,
    /// Corrected by a satellite-based augmentation system (WAAS, EGNOS,
    /// MSAS, GAGAN, ...).
    Sbas,
    /// Code-differential corrections from a base station or a network.
    Dgnss,
    /// Real-time kinematic, carrier-phase ambiguities not yet fixed.
    RtkFloat,
    /// Real-time kinematic, carrier-phase ambiguities fixed.
    RtkFixed,
    /// Precise point positioning from precise orbit and clock corrections.
    Ppp,
    /// Propagated by the receiver's own inertial or odometry dead reckoning,
    /// without a GNSS solution this epoch.
    DeadReckoning,
    /// A position set in the receiver, such as a base station's surveyed or
    /// entered position, not measured.
    FixedPosition,
}

/// A position on the WGS84 ellipsoid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Geodetic {
    /// Degrees, north positive, in `[-90, 90]`.
    pub latitude: f64,
    /// Degrees, east positive, in `[-180, 180]`.
    pub longitude: f64,
    /// Metres above the ellipsoid, not above mean sea level; see
    /// [`GnssFix::undulation`].
    pub ellipsoidal_height: f64,
}

/// A time on the GPS time scale, which has no leap seconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GpsTime {
    /// Weeks since the GPS epoch, 1980-01-06 00:00 UTC, counted through the
    /// broadcast rollovers rather than modulo 1024.
    pub week: u16,
    /// Nanoseconds since the start of the week, below 604 800 s.
    pub tow_ns: u64,
}

/// One solution epoch of a [`GnssReceiver`]. A field the device does not
/// report this epoch is `None`, never filled with a stand-in value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssFix {
    /// How the receiver solved this epoch.
    pub status: GnssStatus,
    /// The antenna reference point; `None` exactly when `status` is
    /// [`GnssStatus::NoFix`].
    pub position: Option<Geodetic>,
    /// Metres from the ellipsoid up to the geoid at the position, by the
    /// device's geoid model, so the height above mean sea level is
    /// `ellipsoidal_height - undulation`.
    pub undulation: Option<f64>,
    /// Velocity of the antenna reference point, m/s, along east, north and up
    /// of the local frame at the position.
    pub velocity_enu: Option<[f64; 3]>,
    /// The device's 1σ estimate of the position error along east, north and
    /// up, metres. A device that reports one horizontal figure gives it for
    /// both east and north; each driver documents which figures it maps.
    pub sigma_enu: Option<[f64; 3]>,
    /// Satellites used in the solution, not merely tracked.
    pub satellites: Option<u16>,
    /// Age of the differential or RTK corrections in use, seconds.
    pub correction_age: Option<f64>,
    /// The epoch's time by the device. A driver whose device sends UTC
    /// converts it with a leap-second count it documents, or leaves this
    /// `None`.
    pub gps_time: Option<GpsTime>,
}

impl GnssFix {
    /// An epoch without a solution and nothing else reported, and the base a
    /// driver fills in: `GnssFix { status, position, ..GnssFix::NO_FIX }`.
    pub const NO_FIX: Self = Self {
        status: GnssStatus::NoFix,
        position: None,
        undulation: None,
        velocity_enu: None,
        sigma_enu: None,
        satellites: None,
        correction_age: None,
        gps_time: None,
    };
}

/// A GNSS receiver that delivers one fix per solution epoch.
///
/// # Position
///
/// The position is that of the antenna reference point, which the header's
/// `frame` names. Latitude and longitude are WGS84 degrees, and the height is
/// above the ellipsoid: a driver whose device sends the height above mean sea
/// level adds the device's undulation back, as NMEA GGA, which sends both,
/// requires. With RTK or PPP corrections the position is in the reference
/// frame of the corrections, such as the coordinates an RTK base was given;
/// the contract transforms no datum. Velocity is along the local east, north
/// and up axes at the position, the world axes of [`ImuSensor`] orientation.
///
/// # Epochs
///
/// [`read_fix`](Self::read_fix) returns one fix per solution epoch. An epoch
/// without a solution still comes back, with status
/// [`NoFix`](GnssStatus::NoFix) and no position: never an error, and never a
/// made-up position such as the last one, a zero or a configured default.
/// [`DeadReckoning`](GnssStatus::DeadReckoning) and
/// [`FixedPosition`](GnssStatus::FixedPosition) are positions the receiver
/// itself reports as such.
///
/// The header's stamp is the driver's clock when the epoch's message was
/// completely received, with no latency subtracted; a fix assembled from
/// several messages of one epoch is stamped when the last of them is
/// received. The device's own time of the epoch goes in
/// [`gps_time`](GnssFix::gps_time). `seq` rises with every fix.
pub trait GnssReceiver {
    /// Blocks for the next epoch's fix.
    fn read_fix(&mut self) -> RobotResult<Stamped<GnssFix>>;
}

/// The direction of a [`GnssHeading`] receiver's antenna baseline.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Heading {
    /// Radians in `(-π, π]`, counter-clockwise about up from east, as the yaw
    /// of an [`ImuSensor`] orientation: east is 0 and north `π/2`.
    pub yaw: f64,
    /// Radians, the baseline's elevation above the local horizontal, positive
    /// up.
    pub pitch: Option<f64>,
    /// 1σ of `yaw`, radians.
    pub yaw_sigma: Option<f64>,
    /// 1σ of `pitch`, radians.
    pub pitch_sigma: Option<f64>,
}

/// A GNSS receiver with two antennas, or a moving-base pair, that measures
/// the direction of the baseline between them.
///
/// The baseline runs from the primary antenna to the secondary as the device
/// names them, after any mounting offset the device itself applies; the
/// driver documents both, and the header's `frame` names the frame whose +X
/// the yaw gives.
///
/// Makers report heading clockwise from north, usually in degrees. The yaw is
/// `π/2 - heading` in radians, wrapped into `(-π, π]`: a heading of 0° (north)
/// is `π/2`, 90° (east) is 0, 180° (south) is `-π/2`, and 270° (west) is `π`.
///
/// An epoch without a heading solution has no heading:
/// [`read_heading`](Self::read_heading) returns the next epoch that has one,
/// waiting as long as the driver's read budget allows, and the driver
/// documents the error it returns when the budget runs out. Stamps and `seq`
/// follow [`GnssReceiver`].
pub trait GnssHeading {
    /// Blocks for the next epoch with a heading solution.
    fn read_heading(&mut self) -> RobotResult<Stamped<Heading>>;
}
