//! The conformance checks accept compliant mock devices and name the
//! violation of broken ones.
#![cfg(feature = "conformance")]

use roplat_behavior::JointLimits;
use std::{cell::Cell, rc::Rc};

use roplat_behavior::{
    ForceTorqueSensor, Gripper, ImuSensor, Joints, RobotResult,
    conformance::{check_force_torque_sensor, check_gripper, check_imu, check_joints},
    msgs::{
        ForceTorque, FrameId, GripperCommand, GripperState, Header, Imu, Quaternion, Stamp,
        Stamped, Wrench, clock,
    },
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
            return Err(
                roplat_behavior::RobotException::UnprocessableInstructionError("no tare".into()),
            );
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

struct MockImu {
    seq: u64,
    orientation: Quaternion,
}

impl ImuSensor for MockImu {
    fn read_imu(&mut self) -> RobotResult<Stamped<Imu>> {
        self.seq += 1;
        let header = Header::new(
            Stamp::new(clock::SIMULATION, self.seq as i64),
            FrameId::new(1),
            self.seq,
        );
        Ok(Stamped::new(
            header,
            Imu::EMPTY.with_orientation(self.orientation),
        ))
    }
}

#[test]
fn imu_checks() {
    let mut good = MockImu { seq: 0, orientation: Quaternion::IDENTITY };
    check_imu(&mut good, || {}, 4).unwrap();

    let mut skewed = MockImu {
        seq: 0,
        orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 2.0 },
    };
    let e = check_imu(&mut skewed, || {}, 1).unwrap_err();
    assert!(e.detail.contains("orientation norm 2"), "{e}");
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
