//! The end-effector and sensor contracts compose without a `Robot` impl and
//! carry the producer's clock and frame.

use roplat_behavior::RobotResult;
use roplat_behavior::behavior::{Camera, ForceTorqueSensor, Gripper, ImuSensor, Lidar, LidarPoint};
use roplat_behavior::msgs::{
    CameraInfo, Clock, ForceTorque, FrameId, GripperCommand, GripperState, Header, Imu,
    ManualClock, Stamped, Vector3, Wrench,
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
    assert!(hand.stop_gripper().is_err());
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
    assert!(sensors.bias_wrench().is_err());
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
