//! The end-effector and sensor contracts compose without a `Robot` impl and
//! carry the producer's clock and frame.

use std::{f64::consts::FRAC_PI_2, time::Duration};

use nalgebra as na;
use roplat_behavior::behavior::{Camera, ForceTorqueSensor, Gripper, ImuSensor, Lidar, LidarPoint};
use roplat_behavior::msgs::{
    CameraInfo, Clock, ForceTorque, FrameId, GripperCommand, GripperState, Header, Imu,
    ManualClock, Quaternion, Stamped, Vector3, Wrench,
};
use roplat_behavior::{
    MultirotorState, RobotException, RobotResult, frd_to_flu, ned_frd_to_enu_flu, ned_to_enu,
};

struct Hand {
    state: GripperState,
}

impl Gripper for Hand {
    fn read_gripper(&mut self) -> RobotResult<GripperState> {
        Ok(self.state)
    }

    fn move_gripper(&mut self, command: GripperCommand) -> RobotResult<()> {
        self.state.width = command.width.min(self.state.max_width);
        self.state.grasped = u32::from(command.force > 0.0);
        Ok(())
    }
}

struct Sensors<'a> {
    clock: &'a ManualClock,
    seq: u64,
}

impl Sensors<'_> {
    fn header(&mut self, frame: u32) -> Header {
        self.seq += 1;
        Header::new(self.clock.now(), FrameId::new(frame), self.seq)
    }
}

impl ForceTorqueSensor for Sensors<'_> {
    fn read_wrench(&mut self) -> RobotResult<ForceTorque> {
        let wrench = Wrench { force: Vector3::new(0., 0., -9.81), torque: Vector3::ZERO };
        Ok(Stamped::new(self.header(7), wrench))
    }
}

impl ImuSensor for Sensors<'_> {
    fn read_imu(&mut self) -> RobotResult<Stamped<Imu>> {
        let imu = Imu::EMPTY.with_linear_acceleration(Vector3::new(0., 0., 9.81));
        Ok(Stamped::new(self.header(8), imu))
    }
}

impl Camera for Sensors<'_> {
    type Frame = [u8; 4];

    fn camera_info(&self) -> CameraInfo {
        CameraInfo { width: 2, height: 2, ..Default::default() }
    }

    fn capture(&mut self) -> RobotResult<Stamped<[u8; 4]>> {
        Ok(Stamped::new(self.header(9), [0, 64, 128, 255]))
    }
}

fn grasp(gripper: &mut impl Gripper) -> RobotResult<bool> {
    gripper.home_gripper()?;
    gripper.move_gripper(GripperCommand { width: 0.01, speed: 0.05, force: 20.0 })?;
    Ok(gripper.read_gripper()?.grasped != 0)
}

#[test]
fn gripper_contract_drives_any_hand() {
    let mut hand = Hand {
        state: GripperState { width: 0.08, max_width: 0.08, ..Default::default() },
    };
    assert!(grasp(&mut hand).unwrap());
    assert_eq!(hand.read_gripper().unwrap().width, 0.01);
    // Optional operations report that the driver lacks them.
    assert!(matches!(
        hand.stop_gripper(),
        Err(RobotException::NotSupported { operation: "stop_gripper" })
    ));
}

#[test]
fn sensor_readings_carry_clock_frame_and_order() {
    let sim = ManualClock::simulation();
    let mut sensors = Sensors { clock: &sim, seq: 0 };

    let wrench = sensors.read_wrench().unwrap();
    sim.advance(std::time::Duration::from_millis(1));
    let imu = sensors.read_imu().unwrap();
    let frame = sensors.capture().unwrap();

    assert_eq!(wrench.header.frame, FrameId::new(7));
    assert_eq!(wrench.value.force.z, -9.81);
    assert_eq!(
        imu.header.stamp.nanos_since(&wrench.header.stamp),
        Some(1_000_000)
    );
    assert_eq!(imu.value.linear_acceleration().map(|a| a.z), Some(9.81));
    assert_eq!(imu.value.orientation(), None);
    assert_eq!(
        [wrench.header.seq, imu.header.seq, frame.header.seq],
        [1, 2, 3]
    );
    assert_eq!(sensors.camera_info().width, 2);
    assert!(matches!(
        sensors.bias_wrench(),
        Err(RobotException::NotSupported { operation: "bias_wrench" })
    ));
}

impl Lidar for Sensors<'_> {
    /// A planar sweep from the right to the left: a wall 2 m ahead, then a
    /// post 1 m to the left.
    fn read_scan(&mut self, points: &mut Vec<LidarPoint>) -> RobotResult<Header> {
        points.clear();
        points.push(LidarPoint {
            position: [2.0, 0.0, 0.0],
            offset_ns: 0,
            intensity: Some(80.0),
            ring: Some(0),
        });
        points.push(LidarPoint {
            position: [0.0, 1.0, 0.0],
            offset_ns: 6_250_000,
            intensity: None,
            ring: Some(0),
        });
        Ok(self.header(10))
    }
}

/// The range and azimuth of the nearest return, from any lidar.
fn nearest(lidar: &mut impl Lidar, points: &mut Vec<LidarPoint>) -> RobotResult<(f64, f64)> {
    lidar.read_scan(points)?;
    let [x, y, _] = points
        .iter()
        .min_by(|a, b| range(a).total_cmp(&range(b)))
        .expect("the scan has returns")
        .position;
    Ok((x.hypot(y), y.atan2(x)))
}

fn range(point: &LidarPoint) -> f64 {
    point.position.iter().map(|c| c * c).sum::<f64>().sqrt()
}

#[test]
fn lidar_scans_share_one_frame_convention_and_buffer() {
    let sim = ManualClock::simulation();
    let mut sensors = Sensors { clock: &sim, seq: 0 };
    let mut points = Vec::new();

    // +Y is left: the post 1 m to the left is at azimuth +90°.
    let (distance, azimuth) = nearest(&mut sensors, &mut points).unwrap();
    assert_eq!(distance, 1.0);
    assert_eq!(azimuth, std::f64::consts::FRAC_PI_2);

    let capacity = points.capacity();
    let header = sensors.read_scan(&mut points).unwrap();
    assert_eq!(points.len(), 2);
    assert_eq!(points.capacity(), capacity);
    assert_eq!((header.frame, header.seq), (FrameId::new(10), 2));
}

const X: Vector3 = Vector3::new(1.0, 0.0, 0.0);
const Y: Vector3 = Vector3::new(0.0, 1.0, 0.0);
const Z: Vector3 = Vector3::new(0.0, 0.0, 1.0);
const G: f64 = 9.81;

/// `v` turned by the unit quaternion `q`.
fn rotate(q: Quaternion, v: Vector3) -> Vector3 {
    (na::UnitQuaternion::from(q) * na::Vector3::from(v)).into()
}

/// `v` turned back by the unit quaternion `q`.
fn unrotate(q: Quaternion, v: Vector3) -> Vector3 {
    (na::UnitQuaternion::from(q).inverse() * na::Vector3::from(v)).into()
}

fn cross(a: Vector3, b: Vector3) -> Vector3 {
    na::Vector3::from(a).cross(&na::Vector3::from(b)).into()
}

fn close(a: Vector3, b: Vector3) -> bool {
    (na::Vector3::from(a) - na::Vector3::from(b)).norm() < 1e-12
}

/// An FRD body relative to NED after roll, pitch and yaw, in that order.
fn ned_frd(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
    na::UnitQuaternion::from_euler_angles(roll, pitch, yaw).into()
}

#[test]
fn imu_vector_conversions_are_half_turns() {
    let (a, b) = (Vector3::new(1.0, 2.0, 3.0), Vector3::new(-0.5, 0.25, 4.0));
    for convert in [frd_to_flu as fn(Vector3) -> Vector3, ned_to_enu] {
        assert_eq!(convert(convert(a)), a);
        // A rotation keeps cross products, so angular velocity converts as
        // specific force does; a mirror would flip them.
        assert!(close(cross(convert(a), convert(b)), convert(cross(a, b))));
    }
    // North is world +Y, east +X, down -Z.
    assert_eq!(ned_to_enu(X), Y);
    assert_eq!(ned_to_enu(Y), X);
    assert_eq!(ned_to_enu(Z), Vector3::new(0.0, 0.0, -1.0));
    // A left turn reads negative about FRD +Z, which points down, and
    // positive about FLU +Z.
    assert_eq!(
        frd_to_flu(Vector3::new(0.0, 0.0, -0.5)),
        Vector3::new(0.0, 0.0, 0.5)
    );
}

#[test]
fn a_ned_frd_orientation_converts_as_its_vectors_do() {
    let poses = [
        ned_frd(0.0, 0.0, 0.0),
        ned_frd(0.3, -0.2, 1.1),
        ned_frd(-2.0, 1.2, -0.4),
        ned_frd(3.0, 0.1, 2.5),
    ];
    let flu = Vector3::new(0.3, -1.2, 2.0);
    for q in poses {
        let enu_flu = ned_frd_to_enu_flu(q);
        // FLU to FRD, FRD to NED by the device's orientation, NED to ENU.
        let through_device = ned_to_enu(rotate(q, frd_to_flu(flu)));
        assert!(close(rotate(enu_flu, flu), through_device), "{q:?}");

        // At rest in any pose, the converted specific force points up.
        let device_reads = unrotate(q, Vector3::new(0.0, 0.0, -G));
        let reads = frd_to_flu(device_reads);
        assert!(
            close(rotate(enu_flu, reads), Vector3::new(0.0, 0.0, G)),
            "{q:?}"
        );

        let norm = enu_flu.to_xyzw().iter().map(|c| c * c).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-12, "{enu_flu:?}");
        let back = ned_frd_to_enu_flu(enu_flu).to_xyzw();
        assert!(
            back.iter()
                .zip(q.to_xyzw())
                .all(|(a, b)| (a - b).abs() < 1e-12),
            "{q:?} came back as {back:?}"
        );
    }
}

#[test]
fn imu_orientation_heads_from_east_and_turns_right_handed() {
    // Facing east, a heading of 90° relative to NED, is the identity.
    let east = ned_frd_to_enu_flu(ned_frd(0.0, 0.0, FRAC_PI_2));
    for axis in [X, Y, Z] {
        assert!(close(rotate(east, axis), axis), "{east:?}");
    }
    // Facing north, the front points to world +Y and the left to west.
    let north = ned_frd_to_enu_flu(ned_frd(0.0, 0.0, 0.0));
    assert!(close(rotate(north, X), Y));
    assert!(close(rotate(north, Y), Vector3::new(-1.0, 0.0, 0.0)));
    // Rolled right side down while facing north: the left points up and the
    // top east.
    let rolled = ned_frd_to_enu_flu(ned_frd(FRAC_PI_2, 0.0, 0.0));
    assert!(close(rotate(rolled, Y), Z));
    assert!(close(rotate(rolled, Z), X));
    // Nose up while facing north: the front points up and the top south.
    let nose_up = ned_frd_to_enu_flu(ned_frd(0.0, FRAC_PI_2, 0.0));
    assert!(close(rotate(nose_up, X), Z));
    assert!(close(rotate(nose_up, Z), Vector3::new(0.0, -1.0, 0.0)));
}

#[test]
fn a_multirotor_yaw_runs_from_east_like_the_imu_orientation() {
    // An autopilot heading 30 degrees east of north, level, relative to NED.
    let heading = 30f64.to_radians();
    let orientation = ned_frd_to_enu_flu(ned_frd(0.0, 0.0, heading));
    let state = MultirotorState {
        device_time: Duration::ZERO,
        position: Vector3::ZERO,
        velocity: None,
        orientation: Some(orientation),
        angular_velocity: None,
        armed: None,
        landed: None,
    };
    // Counter-clockwise from east: 90 degrees minus the heading.
    let yaw = state.yaw().unwrap();
    assert!((yaw - (FRAC_PI_2 - heading)).abs() < 1e-12, "{yaw}");
    assert_eq!(MultirotorState { orientation: None, ..state }.yaw(), None);
}
