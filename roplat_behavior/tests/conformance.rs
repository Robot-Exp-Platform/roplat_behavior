//! The conformance checks accept compliant mock devices and name the
//! violation of broken ones.
#![cfg(feature = "conformance")]

use roplat_behavior::JointLimits;
use std::{cell::Cell, f64::consts::PI, rc::Rc, time::Duration};

use nalgebra as na;
use roplat_behavior::{
    Camera, ForceTorqueSensor, Geodetic, GnssFix, GnssReceiver, GnssStatus, GpsTime, Gripper,
    ImuSensor, Joints, LandedState, Lidar, LidarPoint, MotionCapture, Multirotor, MultirotorState,
    PositionSetpoint, Robot, RobotResult, TrackedBody,
    conformance::{
        AtRest, FacingWall, FrameView, check_camera, check_camera_facing_wall, check_flight_state,
        check_force_torque_sensor, check_gnss, check_gripper, check_imu, check_imu_at_rest,
        check_joints, check_lidar, check_motion_capture,
    },
    frd_to_flu,
    msgs::{
        CameraInfo, ForceTorque, FrameId, GripperCommand, GripperState, Header, Imu, Pose,
        Quaternion, Stamp, Stamped, Vector3, Wrench, clock, distortion_model,
    },
    ned_frd_to_enu_flu,
};

/// A sensor under a constant load; `ticks` is the mock transport's clock,
/// advanced by the check's `advance`.
struct MockForceTorque {
    ticks: Rc<Cell<u64>>,
    load: Wrench,
    bias: Option<Wrench>,
    /// Faults to inject.
    repeat_seq: bool,
    ignore_bias: bool,
    supports_bias: bool,
}

impl MockForceTorque {
    fn new() -> Self {
        Self {
            ticks: Rc::new(Cell::new(0)),
            load: Wrench::from([0.0, 0.0, 19.62, 0.0, -1.96, 0.0]),
            bias: None,
            repeat_seq: false,
            ignore_bias: false,
            supports_bias: true,
        }
    }
}

impl ForceTorqueSensor for MockForceTorque {
    fn read_wrench(&mut self) -> RobotResult<ForceTorque> {
        let ticks = self.ticks.get();
        let seq = if self.repeat_seq { 1 } else { ticks };
        let header = Header::new(
            Stamp::new(clock::MONOTONIC, ticks as i64 * 1_000_000),
            FrameId::new(2),
            seq,
        );
        let raw = self.load.to_array();
        let bias = self.bias.map_or([0.0; 6], |b| b.to_array());
        Ok(ForceTorque::new(
            header,
            Wrench::from(std::array::from_fn(|i| raw[i] - bias[i])),
        ))
    }

    fn bias_wrench(&mut self) -> RobotResult<()> {
        if !self.supports_bias {
            return Err(roplat_behavior::RobotException::NotSupported { operation: "bias_wrench" });
        }
        if !self.ignore_bias {
            self.bias = Some(self.load);
        }
        Ok(())
    }
}

#[test]
fn force_torque_checks() {
    let check = |mut s: MockForceTorque| {
        let ticks = s.ticks.clone();
        check_force_torque_sensor(&mut s, || ticks.set(ticks.get() + 1), 5, true, 1e-9)
    };
    check(MockForceTorque::new()).unwrap();
    check(MockForceTorque { supports_bias: false, ..MockForceTorque::new() }).unwrap();

    let e = check(MockForceTorque { repeat_seq: true, ..MockForceTorque::new() }).unwrap_err();
    assert_eq!(e.check, "force_torque");
    assert!(e.detail.contains("seq"), "{e}");

    let e = check(MockForceTorque { ignore_bias: true, ..MockForceTorque::new() }).unwrap_err();
    assert_eq!(e.check, "force_torque: bias");
}

/// An IMU that sends the same reading every time.
struct MockImu {
    seq: u64,
    reading: Imu,
}

impl ImuSensor for MockImu {
    fn read_imu(&mut self) -> RobotResult<Stamped<Imu>> {
        self.seq += 1;
        let header = Header::new(
            Stamp::new(clock::SIMULATION, self.seq as i64),
            FrameId::new(1),
            self.seq,
        );
        Ok(Stamped::new(header, self.reading))
    }
}

#[test]
fn imu_checks() {
    let check = |reading| check_imu(&mut MockImu { seq: 0, reading }, || {}, 4);
    let level = Imu::EMPTY.with_linear_acceleration(Vector3::new(0.0, 0.0, 9.81));
    check(level.with_orientation(Quaternion::IDENTITY)).unwrap();

    let skewed = Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 2.0 };
    let e = check(level.with_orientation(skewed)).unwrap_err();
    assert!(e.detail.contains("orientation norm 2"), "{e}");

    let broken = Quaternion { x: f64::NAN, ..Quaternion::IDENTITY };
    let e = check(level.with_orientation(broken)).unwrap_err();
    assert!(e.detail.contains("non-finite orientation"), "{e}");

    let e =
        check(level.with_linear_acceleration(Vector3::new(0.0, f64::INFINITY, 9.81))).unwrap_err();
    assert!(e.detail.contains("non-finite vector"), "{e}");
}

const AT_REST: AtRest = AtRest {
    gravity: 9.81,
    acceleration_tolerance: 0.05,
    angular_velocity_tolerance: 1e-3,
};

/// The specific force of a body at rest, in the body frame of `pose`.
fn at_rest_in(pose: na::UnitQuaternion<f64>, up: f64) -> Vector3 {
    (pose.inverse() * na::Vector3::new(0.0, 0.0, up)).into()
}

#[test]
fn imu_at_rest_checks() {
    let check = |reading| {
        let mut imu = MockImu { seq: 0, reading };
        check_imu_at_rest(&mut imu, || {}, 4, AT_REST).map_err(|e| e.check)
    };
    // An FLU body relative to ENU, rolled, pitched and turned.
    let pose = na::UnitQuaternion::from_euler_angles(0.4, -0.3, 2.0);
    let still = Imu::EMPTY
        .with_orientation(pose.into())
        .with_linear_acceleration(at_rest_in(pose, 9.81))
        .with_angular_velocity(Vector3::ZERO);
    check(still).unwrap();
    // Without an orientation only the magnitude counts.
    let unoriented = Imu::EMPTY.with_linear_acceleration(at_rest_in(pose, 9.81));
    check(unoriented).unwrap();

    // The device's own FRD body relative to NED, not converted: up reads down.
    let ned_frd = ned_frd_to_enu_flu(pose.into());
    let raw = Imu::EMPTY
        .with_orientation(ned_frd)
        .with_linear_acceleration(at_rest_in(ned_frd.into(), -9.81));
    assert_eq!(check(raw), Err("imu: gravity"));
    // The acceleration converted, the orientation not.
    let half = raw.with_linear_acceleration(frd_to_flu(at_rest_in(ned_frd.into(), -9.81)));
    assert_eq!(check(half), Err("imu: gravity"));
    // Gravity removed, or left in g.
    assert_eq!(
        check(still.with_linear_acceleration(Vector3::ZERO)),
        Err("imu: gravity")
    );
    let in_g = Imu::EMPTY.with_linear_acceleration(at_rest_in(pose, 1.0));
    assert_eq!(check(in_g), Err("imu: gravity"));
    // No reading to check gravity with.
    let attitude_only = Imu::EMPTY.with_orientation(pose.into());
    assert_eq!(check(attitude_only), Err("imu: gravity"));
    // Turning.
    let turning = still.with_angular_velocity(Vector3::new(0.0, 0.0, 0.1));
    assert_eq!(check(turning), Err("imu: angular velocity"));

    // Not promised: a body frame turned the same way in both fields passes,
    // so the body axes stay in the driver's known-scene tests. Here an FRD
    // body relative to ENU.
    let frd = pose * na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), PI);
    let turned_body = Imu::EMPTY
        .with_orientation(frd.into())
        .with_linear_acceleration(at_rest_in(frd, 9.81));
    check(turned_body).unwrap();
}

/// A receiver that sends the same fix every epoch, its `seq` rising by
/// `step`.
struct MockGnss {
    seq: u64,
    step: u64,
    fix: GnssFix,
}

impl GnssReceiver for MockGnss {
    fn read_fix(&mut self) -> RobotResult<Stamped<GnssFix>> {
        self.seq += self.step;
        let header = Header::new(
            Stamp::new(clock::SIMULATION, self.seq as i64),
            FrameId::new(2),
            self.seq,
        );
        Ok(Stamped::new(header, self.fix))
    }
}

#[test]
fn gnss_checks() {
    let check = |fix| {
        let mut receiver = MockGnss { seq: 0, step: 1, fix };
        check_gnss(&mut receiver, || {}, 3).map_err(|e| e.check)
    };
    let antenna = Geodetic { latitude: 40.0, longitude: 116.3, ellipsoidal_height: 52.0 };
    let rtk = GnssFix {
        status: GnssStatus::RtkFixed,
        position: Some(antenna),
        undulation: Some(-9.0),
        velocity_enu: Some([0.1, 0.0, 0.0]),
        sigma_enu: Some([0.01, 0.01, 0.02]),
        satellites: Some(24),
        correction_age: Some(1.0),
        gps_time: Some(GpsTime { week: 2444, tow_ns: 3_600_000_000_000 }),
    };
    check(rtk).unwrap();
    // An epoch without a solution comes back with no position.
    check(GnssFix::NO_FIX).unwrap();
    let dead_reckoning = GnssFix {
        status: GnssStatus::DeadReckoning,
        position: Some(antenna),
        ..GnssFix::NO_FIX
    };
    check(dead_reckoning).unwrap();

    // A position exactly when there is a solution.
    let lost = GnssFix { position: None, ..rtk };
    assert_eq!(check(lost), Err("gnss: no fix"));
    let made_up = GnssFix { status: GnssStatus::NoFix, ..rtk };
    assert_eq!(check(made_up), Err("gnss: no fix"));

    // Ranges and finite values.
    for position in [
        Geodetic { latitude: 90.5, ..antenna },
        Geodetic { longitude: -180.5, ..antenna },
        Geodetic { latitude: f64::NAN, ..antenna },
        Geodetic { ellipsoidal_height: f64::INFINITY, ..antenna },
    ] {
        let fix = GnssFix { position: Some(position), ..rtk };
        assert_eq!(check(fix), Err("gnss"), "{position:?}");
    }
    let broken = [
        GnssFix { undulation: Some(f64::NAN), ..rtk },
        GnssFix { velocity_enu: Some([0.0, f64::NAN, 0.0]), ..rtk },
        GnssFix { sigma_enu: Some([0.01, -0.01, 0.02]), ..rtk },
        GnssFix { correction_age: Some(-1.0), ..rtk },
        GnssFix {
            gps_time: Some(GpsTime { week: 2444, tow_ns: 604_800_000_000_000 }),
            ..rtk
        },
    ];
    for fix in broken {
        assert_eq!(check(fix), Err("gnss"), "{fix:?}");
    }

    // The header stream: seq must rise.
    let mut stuck = MockGnss { seq: 1, step: 0, fix: rtk };
    let e = check_gnss(&mut stuck, || {}, 2).unwrap_err();
    assert!(e.detail.contains("seq did not rise"), "{e}");
}

/// A motion-capture system that reports the same bodies every frame, its
/// stamps moving by `stamp_step` ns.
struct MockMocap {
    seq: u64,
    stamp_step: i64,
    bodies: Vec<TrackedBody>,
}

impl MotionCapture for MockMocap {
    fn read_frame(&mut self, bodies: &mut Vec<TrackedBody>) -> RobotResult<Header> {
        self.seq += 1;
        bodies.clone_from(&self.bodies);
        let stamp = Stamp::new(
            clock::SIMULATION,
            1_000_000 + self.seq as i64 * self.stamp_step,
        );
        Ok(Header::new(stamp, FrameId::new(3), self.seq))
    }
}

#[test]
fn motion_capture_checks() {
    let check = |bodies: Vec<TrackedBody>| {
        let mut mocap = MockMocap { seq: 0, stamp_step: 8_333_333, bodies };
        check_motion_capture(&mut mocap, || {}, 3).map_err(|e| e.check)
    };
    let tracked = TrackedBody {
        id: 1,
        pose: Some(Pose::from_quat_array(
            [0.5, -0.2, 1.1],
            [0.0, 0.0, 0.6, 0.8],
        )),
        error: Some(0.0004),
    };
    // A lost body is reported without a pose; an empty frame is a frame.
    let lost = TrackedBody { id: 2, pose: None, error: None };
    check(vec![tracked, lost]).unwrap();
    check(Vec::new()).unwrap();

    assert_eq!(check(vec![tracked, tracked]), Err("motion_capture: id"));
    let skewed = Pose {
        orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 2.0 },
        ..Pose::IDENTITY
    };
    let broken = [
        TrackedBody { pose: Some(skewed), ..tracked },
        TrackedBody {
            pose: Some(Pose::from_position([f64::NAN, 0.0, 0.0])),
            ..tracked
        },
        TrackedBody { error: Some(-0.001), ..tracked },
        TrackedBody { error: Some(f64::INFINITY), ..tracked },
    ];
    for body in broken {
        assert_eq!(check(vec![body]), Err("motion_capture"), "{body:?}");
    }

    // Stamps never go back.
    let mut rewinding = MockMocap { seq: 0, stamp_step: -1, bodies: vec![tracked] };
    let e = check_motion_capture(&mut rewinding, || {}, 2).unwrap_err();
    assert!(e.detail.contains("stamp went back"), "{e}");
}

/// A multirotor hovering in place; its device clock moves by `tick` per
/// estimate, backwards when `rewind` is set.
struct MockVehicle {
    seq: u64,
    tick: Duration,
    rewind: bool,
    state: MultirotorState,
}

impl Robot for MockVehicle {
    type State = MultirotorState;
    const CONTROL_PERIOD: Duration = Duration::from_millis(20);

    fn version() -> String {
        String::new()
    }

    fn read_state(&mut self) -> RobotResult<MultirotorState> {
        Ok(self.state)
    }
}

impl Multirotor for MockVehicle {
    fn read_flight_state(&mut self) -> RobotResult<Stamped<MultirotorState>> {
        self.seq += 1;
        let time = &mut self.state.device_time;
        *time = if self.rewind {
            time.saturating_sub(self.tick)
        } else {
            *time + self.tick
        };
        let header = Header::new(
            Stamp::new(clock::SIMULATION, self.seq as i64),
            FrameId::new(4),
            self.seq,
        );
        Ok(Stamped::new(header, self.state))
    }

    fn take_off(&mut self, _: f64) -> RobotResult<()> {
        Ok(())
    }

    fn send_setpoint(&mut self, _: PositionSetpoint) -> RobotResult<()> {
        Ok(())
    }

    fn go_to(&mut self, _: PositionSetpoint) -> RobotResult<Stamped<MultirotorState>> {
        self.read_flight_state()
    }

    fn land(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

#[test]
fn flight_state_checks() {
    let check = |state, rewind| {
        let tick = Duration::from_millis(20);
        let mut vehicle = MockVehicle { seq: 0, tick, rewind, state };
        check_flight_state(&mut vehicle, || {}, 3).map_err(|e| e.check)
    };
    let hover = MultirotorState {
        device_time: Duration::from_secs(5),
        position: Vector3::new(1.0, 2.0, 1.5),
        velocity: Some(Vector3::ZERO),
        orientation: Some(Quaternion::IDENTITY),
        angular_velocity: Some(Vector3::ZERO),
        armed: Some(true),
        landed: Some(LandedState::InAir),
    };
    check(hover, false).unwrap();
    // Only the position is required.
    let bare = MultirotorState {
        velocity: None,
        orientation: None,
        angular_velocity: None,
        armed: None,
        landed: None,
        ..hover
    };
    check(bare, false).unwrap();

    assert_eq!(check(hover, true), Err("flight_state: device time"));
    let broken = [
        MultirotorState { position: Vector3::new(f64::NAN, 0.0, 0.0), ..hover },
        MultirotorState {
            velocity: Some(Vector3::new(0.0, f64::INFINITY, 0.0)),
            ..hover
        },
        MultirotorState {
            orientation: Some(Quaternion { w: 2.0, ..Quaternion::IDENTITY }),
            ..hover
        },
    ];
    for state in broken {
        assert_eq!(check(state, false), Err("flight_state"), "{state:?}");
    }
}

/// A gripper that reaches each commanded width, or ignores commands.
struct MockGripper {
    width: f64,
    stuck: bool,
}

impl Gripper for MockGripper {
    fn read_gripper(&mut self) -> RobotResult<GripperState> {
        Ok(GripperState { width: self.width, max_width: 0.08, grasped: 0, reserved: 0 })
    }

    fn move_gripper(&mut self, command: GripperCommand) -> RobotResult<()> {
        if !self.stuck {
            self.width = command.width;
        }
        Ok(())
    }
}

#[test]
fn gripper_checks() {
    let widths = [1.0, 0.5, 0.0];
    check_gripper(
        &mut MockGripper { width: 0.04, stuck: false },
        || {},
        &widths,
        0.1,
        1e-4,
    )
    .unwrap();

    let e = check_gripper(
        &mut MockGripper { width: 0.04, stuck: true },
        || {},
        &widths,
        0.1,
        1e-4,
    )
    .unwrap_err();
    assert_eq!(e.check, "gripper: width");
    assert!(e.detail.contains("sent 0.08"), "{e}");
}

struct GoodArm;
impl Joints<2> for GoodArm {
    const JOINT_DEFAULT: [f64; 2] = [0.0, -1.0];
    const JOINT_PACKED: [f64; 2] = [0.0, -2.0];
    const JOINT_LIMITS: JointLimits<2> = JointLimits {
        dq_max: [2.0, 2.0],
        ..JointLimits::positions([-3.0, -3.0], [3.0, -0.1])
    };
}

/// Leaves the packed posture at the all-zero default, outside joint 1.
struct ZeroPackedArm;
impl Joints<2> for ZeroPackedArm {
    const JOINT_DEFAULT: [f64; 2] = [0.0, -1.0];
    const JOINT_LIMITS: JointLimits<2> = JointLimits::positions([-3.0, -3.0], [3.0, -0.1]);
}

struct NegativeVelocityArm;
impl Joints<1> for NegativeVelocityArm {
    const JOINT_LIMITS: JointLimits<1> =
        JointLimits { dq_max: [-2.0], ..JointLimits::positions([-1.0], [1.0]) };
}

#[test]
fn joints_checks() {
    check_joints::<GoodArm, 2>().unwrap();
    let e = check_joints::<ZeroPackedArm, 2>().unwrap_err();
    assert!(e.detail.contains("joint 1: packed posture 0"), "{e}");
    let e = check_joints::<NegativeVelocityArm, 1>().unwrap_err();
    assert!(e.detail.contains("dq_max -2"), "{e}");
}

/// One broken lidar promise to inject.
#[derive(Clone, Copy)]
enum LidarFault {
    /// A ray without a return reported as a point at the origin.
    OriginPoint,
    /// The stamp is earlier than the earliest return.
    EarlyStamp,
    NanIntensity,
}

/// A planar scanner sweeping a 2 m circle.
struct MockLidar {
    seq: u64,
    fault: Option<LidarFault>,
}

impl Lidar for MockLidar {
    fn read_scan(&mut self, points: &mut Vec<LidarPoint>) -> RobotResult<Header> {
        self.seq += 1;
        points.clear();
        points.extend((0..8u32).map(|k| {
            let (s, c) = (f64::from(k) * 0.1).sin_cos();
            LidarPoint {
                position: [2.0 * c, 2.0 * s, 0.0],
                offset_ns: k * 1000,
                intensity: Some(50.0),
                ring: Some(0),
            }
        }));
        match self.fault {
            None => {}
            Some(LidarFault::OriginPoint) => points[3].position = [0.0; 3],
            Some(LidarFault::EarlyStamp) => points.iter_mut().for_each(|p| p.offset_ns += 500),
            Some(LidarFault::NanIntensity) => points[5].intensity = Some(f32::NAN),
        }
        Ok(Header::new(
            Stamp::new(clock::SIMULATION, self.seq as i64 * 100_000_000),
            FrameId::new(3),
            self.seq,
        ))
    }
}

#[test]
fn lidar_checks() {
    check_lidar(&mut MockLidar { seq: 0, fault: None }, || {}, 3).unwrap();

    let check = |fault| check_lidar(&mut MockLidar { seq: 0, fault: Some(fault) }, || {}, 1);
    assert_eq!(
        check(LidarFault::OriginPoint).unwrap_err().check,
        "lidar: no return"
    );
    assert_eq!(
        check(LidarFault::EarlyStamp).unwrap_err().check,
        "lidar: stamp"
    );
    assert_eq!(check(LidarFault::NanIntensity).unwrap_err().check, "lidar");
}

/// Samples of a Z16-like depth stream: millimetres, 0 where the device has
/// no depth.
struct Z16 {
    width: u32,
    height: u32,
    samples: Vec<u16>,
}

/// One broken camera promise to inject.
#[derive(Clone, Copy, PartialEq)]
enum CameraFault {
    /// The range along each pixel's ray instead of the depth along the axis.
    Radial,
    /// Frames a row short of the intrinsics.
    ShortFrame,
    /// Intrinsics that change after the first frame.
    Refocus,
    /// A `seq` stuck at 1.
    StuckSeq,
}

/// The distance from the camera to the wall it faces, m.
const WALL: f64 = 1.5;

/// 8 x 6 pixels with a 90° horizontal field of view.
fn pinhole() -> CameraInfo {
    CameraInfo {
        width: 8,
        height: 6,
        k: [4.0, 0.0, 3.5, 0.0, 4.0, 2.5, 0.0, 0.0, 1.0],
        ..Default::default()
    }
}

/// A depth camera facing a wall `WALL` ahead, square to its axis; its
/// top-left pixel has no depth.
struct MockDepthCamera {
    seq: u64,
    info: CameraInfo,
    fault: Option<CameraFault>,
}

impl MockDepthCamera {
    fn new(fault: Option<CameraFault>) -> Self {
        Self { seq: 0, info: pinhole(), fault }
    }
}

impl Camera for MockDepthCamera {
    type Frame = Z16;

    fn camera_info(&self) -> CameraInfo {
        let mut info = self.info;
        if self.fault == Some(CameraFault::Refocus) && self.seq > 1 {
            info.k[0] = 5.0;
        }
        info
    }

    fn capture(&mut self) -> RobotResult<Stamped<Z16>> {
        self.seq += 1;
        let (k, width) = (self.info.k, self.info.width);
        let height = match self.fault {
            Some(CameraFault::ShortFrame) => self.info.height - 1,
            _ => self.info.height,
        };
        let mut samples = Vec::new();
        for v in 0..height {
            for u in 0..width {
                let x = (f64::from(u) - k[2]) / k[0];
                let y = (f64::from(v) - k[5]) / k[4];
                let ray = match self.fault {
                    Some(CameraFault::Radial) => (1.0 + x * x + y * y).sqrt(),
                    _ => 1.0,
                };
                samples.push((WALL * ray * 1000.0).round() as u16);
            }
        }
        if let Some(first) = samples.first_mut() {
            *first = 0;
        }
        let seq = match self.fault {
            Some(CameraFault::StuckSeq) => 1,
            _ => self.seq,
        };
        let header = Header::new(
            Stamp::new(clock::SIMULATION, self.seq as i64 * 33_000_000),
            FrameId::new(5),
            seq,
        );
        Ok(Stamped::new(header, Z16 { width, height, samples }))
    }
}

/// The driver's accessor: millimetres to metres, 0 missing.
fn metres(frame: &Z16) -> FrameView {
    FrameView {
        width: frame.width,
        height: frame.height,
        depth: frame
            .samples
            .iter()
            .map(|&s| (s != 0).then_some(f64::from(s) / 1000.0))
            .collect(),
    }
}

#[test]
fn camera_checks() {
    let check = |fault, view: fn(&Z16) -> FrameView| {
        check_camera(&mut MockDepthCamera::new(fault), || {}, 3, view).map_err(|e| e.check)
    };
    check(None, metres).unwrap();
    // A frame without depth has only its size checked.
    let colour: fn(&Z16) -> FrameView = |f| FrameView { depth: Vec::new(), ..metres(f) };
    check(None, colour).unwrap();

    assert_eq!(
        check(Some(CameraFault::ShortFrame), metres),
        Err("camera: size")
    );
    let cropped: fn(&Z16) -> FrameView = |f| {
        let mut view = metres(f);
        view.depth.pop();
        view
    };
    assert_eq!(check(None, cropped), Err("camera: size"));
    assert_eq!(
        check(Some(CameraFault::Refocus), metres),
        Err("camera: intrinsics")
    );
    let e = check_camera(
        &mut MockDepthCamera::new(Some(CameraFault::StuckSeq)),
        || {},
        2,
        metres,
    )
    .unwrap_err();
    assert!(e.detail.contains("seq did not rise"), "{e}");

    // A pixel without a depth read as 0 m, or as infinity: never a number.
    let zero: fn(&Z16) -> FrameView = |f| FrameView {
        depth: f
            .samples
            .iter()
            .map(|&s| Some(f64::from(s) / 1000.0))
            .collect(),
        ..metres(f)
    };
    assert_eq!(check(None, zero), Err("camera: depth"));
    let infinite: fn(&Z16) -> FrameView = |f| FrameView {
        depth: f
            .samples
            .iter()
            .map(|&s| {
                Some(if s == 0 {
                    f64::INFINITY
                } else {
                    f64::from(s) / 1000.0
                })
            })
            .collect(),
        ..metres(f)
    };
    assert_eq!(check(None, infinite), Err("camera: depth"));

    // Intrinsics of a pinhole camera, distortion as its model uses it.
    let k = pinhole().k;
    let with_k = |at: usize, value: f64| {
        let mut k = k;
        k[at] = value;
        CameraInfo { k, ..pinhole() }
    };
    let plumb_bob = CameraInfo {
        distortion_model: distortion_model::PLUMB_BOB,
        d: [0.1, -0.2, 0.001, 0.0, 0.05, 0.0, 0.0, 0.0],
        ..pinhole()
    };
    let broken = [
        CameraInfo { width: 0, ..pinhole() },
        with_k(0, 0.0),
        with_k(4, -4.0),
        with_k(2, f64::NAN),
        with_k(2, 9.0),
        with_k(5, -1.0),
        with_k(3, 0.5),
        with_k(8, 2.0),
        CameraInfo { distortion_model: 9, ..pinhole() },
        CameraInfo { d: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], ..pinhole() },
        CameraInfo { d: [0.1, -0.2, 0.001, 0.0, 0.05, 0.3, 0.0, 0.0], ..plumb_bob },
        CameraInfo {
            d: [f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ..plumb_bob
        },
        CameraInfo { reserved: 1, ..pinhole() },
    ];
    for info in broken {
        let mut camera = MockDepthCamera { info, ..MockDepthCamera::new(None) };
        let result = check_camera(&mut camera, || {}, 1, metres).map_err(|e| e.check);
        assert_eq!(result, Err("camera: intrinsics"), "{info:?}");
    }
    let mut distorted = MockDepthCamera { info: plumb_bob, ..MockDepthCamera::new(None) };
    check_camera(&mut distorted, || {}, 1, metres).unwrap();
}

const FACING: FacingWall = FacingWall { distance: WALL, tolerance: 0.002, coverage: 0.9 };

#[test]
fn camera_facing_wall_checks() {
    let check = |fault, view: fn(&Z16) -> FrameView, wall| {
        let mut camera = MockDepthCamera::new(fault);
        check_camera_facing_wall(&mut camera, || {}, 3, view, wall).map_err(|e| e.check)
    };
    check(None, metres, FACING).unwrap();

    // The range along each ray: 47 % longer at the corners of a 90° view.
    assert_eq!(
        check(Some(CameraFault::Radial), metres, FACING),
        Err("camera: wall")
    );
    // Millimetres read as metres.
    let millimetres: fn(&Z16) -> FrameView = |f| FrameView {
        depth: f
            .samples
            .iter()
            .map(|&s| (s != 0).then_some(f64::from(s)))
            .collect(),
        ..metres(f)
    };
    assert_eq!(check(None, millimetres, FACING), Err("camera: wall"));
    // 47 of 48 pixels read a depth; a frame without depth has none.
    let everywhere = FacingWall { coverage: 1.0, ..FACING };
    assert_eq!(check(None, metres, everywhere), Err("camera: wall"));
    let colour: fn(&Z16) -> FrameView = |f| FrameView { depth: Vec::new(), ..metres(f) };
    assert_eq!(check(None, colour, FACING), Err("camera: wall"));
    // Everything check_camera checks comes first.
    assert_eq!(
        check(Some(CameraFault::ShortFrame), metres, FACING),
        Err("camera: size")
    );
}
