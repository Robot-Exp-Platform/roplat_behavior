//! A `Relative<S>` target is an offset from where the robot is; the contract
//! implements it once for every robot.
use roplat_behavior::JointLimits;
use std::time::Duration;

use roplat_behavior::{
    Arm, ArmState, EndPoint, EndPointPose, FlangeSpace, JointSample, JointSpace, Joints, LoadState,
    Motion, MotionLimits, Pose, Relative, Robot, RobotResult, StateView,
};

struct Recorder {
    joint: [f64; 2],
    flange: Pose,
    limits: MotionLimits<2>,
    /// Whether `state` reports the measured joint positions.
    measures_joints: bool,
}

impl Robot for Recorder {
    type State = ();
    const CONTROL_PERIOD: Duration = Duration::from_millis(1);
    fn version() -> String {
        "recorder".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl Joints<2> for Recorder {
    const JOINT_LIMITS: JointLimits<2> = JointLimits::positions([-3.0; 2], [3.0; 2]);
}

impl EndPoint for Recorder {}

impl EndPointPose for Recorder {
    fn get_endpoint(&self) -> Pose {
        self.flange
    }
}

impl roplat_behavior::MoveTo<JointSpace<2>> for Recorder {
    fn move_to(&mut self, target: [f64; 2]) -> RobotResult<()> {
        self.joint = target;
        Ok(())
    }
}

impl roplat_behavior::MoveTo<FlangeSpace> for Recorder {
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        self.flange = target;
        Ok(())
    }
}

impl Arm<2> for Recorder {
    fn state(&mut self) -> RobotResult<ArmState<2>> {
        let mut sample = JointSample::EMPTY;
        if self.measures_joints {
            sample.set_q(Some(self.joint));
        }
        Ok(ArmState { joint: StateView::from_meas(sample), ..ArmState::default() })
    }
    fn set_load(&mut self, _load: LoadState) -> RobotResult<()> {
        Ok(())
    }
    fn get_joint(&self) -> [f64; 2] {
        self.joint
    }
    fn motion_limits(&self) -> MotionLimits<2> {
        self.limits
    }
    fn set_motion_limits(&mut self, limits: MotionLimits<2>) -> RobotResult<()> {
        self.limits = limits;
        Ok(())
    }
}

fn recorder() -> Recorder {
    Recorder {
        joint: [0.5, -0.5],
        flange: Pose::from_euler([0.3, 0.0, 0.4], [0.0, 0.0, std::f64::consts::FRAC_PI_2]),
        limits: MotionLimits::envelope::<Recorder>(),
        measures_joints: true,
    }
}

#[test]
fn a_relative_joint_target_adds_to_the_current_positions() {
    let mut robot = recorder();
    robot
        .move_to::<Relative<JointSpace<2>>>([0.1, 0.2])
        .unwrap();
    assert!((robot.joint[0] - 0.6).abs() < 1e-12 && (robot.joint[1] + 0.3).abs() < 1e-12);
    robot.move_to::<JointSpace<2>>([1.0, 1.0]).unwrap();
    assert_eq!(robot.joint, [1.0, 1.0], "an absolute target stays absolute");
}

#[test]
fn a_relative_joint_target_needs_the_measured_positions() {
    let mut robot = Recorder { measures_joints: false, ..recorder() };
    assert!(
        robot
            .move_to::<Relative<JointSpace<2>>>([0.1, 0.2])
            .is_err()
    );
    assert_eq!(robot.joint, [0.5, -0.5], "nothing was commanded");
}

#[test]
fn a_relative_flange_target_is_expressed_in_the_flange_frame() {
    let mut robot = recorder();
    // The flange is turned 90° about z, so its own x axis is the base's y.
    robot
        .move_to::<Relative<FlangeSpace>>(Pose::from_position([0.1, 0.0, 0.0]))
        .unwrap();
    let [x, y, z] = robot.flange.position();
    assert!((x - 0.3).abs() < 1e-12 && (y - 0.1).abs() < 1e-12 && (z - 0.4).abs() < 1e-12);
}
