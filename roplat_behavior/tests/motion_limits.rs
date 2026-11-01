//! Motion limits as data: the declared envelope, tightening it, and limits
//! applied to one motion.
use roplat_behavior::{
    Arm, ArmState, CartesianLimits, EndPoint, EndPointPose, FlangeSpace, JointLimits, JointSpace,
    Joints, LoadState, Motion, MotionLimits, Pose, Robot, RobotException, RobotResult,
};

/// Records the joint velocity bound in force for every motion.
struct TwoJointArm {
    limits: MotionLimits<2>,
    seen_dq_max: Vec<[f64; 2]>,
}

impl TwoJointArm {
    fn new() -> Self {
        Self {
            limits: MotionLimits::envelope::<Self>(),
            seen_dq_max: Vec::new(),
        }
    }
}

impl Robot for TwoJointArm {
    type State = ();
    const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1);
    fn version() -> String {
        "two-joint mock".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

impl Joints<2> for TwoJointArm {
    const JOINT_LIMITS: JointLimits<2> = JointLimits {
        dq_max: [3.0; 2],
        ddq_max: [10.0; 2],
        ..JointLimits::positions([-2.0; 2], [2.0; 2])
    };
}

impl EndPoint for TwoJointArm {
    const CARTESIAN_LIMITS: CartesianLimits =
        CartesianLimits { linear_velocity_max: 1.0, ..CartesianLimits::UNBOUNDED };
}

impl EndPointPose for TwoJointArm {
    fn get_endpoint(&self) -> Pose {
        Pose::default()
    }
}

impl roplat_behavior::MoveTo<JointSpace<2>> for TwoJointArm {
    fn move_to(&mut self, target: [f64; 2]) -> RobotResult<()> {
        if target[0].is_nan() {
            return Err(RobotException::UnprocessableInstructionError(
                "NaN target".into(),
            ));
        }
        self.seen_dq_max.push(self.limits.scaled_joint().dq_max);
        Ok(())
    }
}

impl roplat_behavior::MoveTo<FlangeSpace> for TwoJointArm {
    fn move_to(&mut self, _target: Pose) -> RobotResult<()> {
        Ok(())
    }
}

impl Arm<2> for TwoJointArm {
    fn state(&mut self) -> RobotResult<ArmState<2>> {
        Ok(ArmState::default())
    }
    fn set_load(&mut self, _load: LoadState) -> RobotResult<()> {
        Ok(())
    }
    fn get_joint(&self) -> [f64; 2] {
        [0.0; 2]
    }
    fn motion_limits(&self) -> MotionLimits<2> {
        self.limits
    }
    fn set_motion_limits(&mut self, limits: MotionLimits<2>) -> RobotResult<()> {
        limits.within(&MotionLimits::envelope::<Self>())?;
        self.limits = limits;
        Ok(())
    }
}

#[test]
fn undeclared_bounds_read_as_unbounded() {
    let joint = TwoJointArm::JOINT_LIMITS;
    assert_eq!(
        (joint.q_min, joint.q_max, joint.dq_max),
        ([-2.0; 2], [2.0; 2], [3.0; 2])
    );
    assert_eq!(joint.dddq_max, [f64::INFINITY; 2]);
    assert_eq!(joint.tau_max, [f64::INFINITY; 2]);
    assert_eq!(joint.dtau_max, [f64::INFINITY; 2]);

    let cartesian = TwoJointArm::CARTESIAN_LIMITS;
    assert_eq!(cartesian.linear_velocity_max, 1.0);
    assert_eq!(cartesian.angular_jerk_max, f64::INFINITY);
}

#[test]
fn scaling_keeps_unbounded_bounds_unbounded() {
    let limits = MotionLimits::<2>::envelope::<TwoJointArm>().scaled(0.5);
    let joint = limits.scaled_joint();
    assert_eq!(joint.dq_max, [1.5; 2]);
    assert_eq!(joint.ddq_max, [5.0; 2]);
    assert_eq!(joint.dddq_max, [f64::INFINITY; 2]);
    // Position and effort bounds are not scaled.
    assert_eq!(joint.q_max, [2.0; 2]);
    assert_eq!(limits.scaled_cartesian().linear_velocity_max, 0.5);
}

#[test]
fn limits_may_only_tighten_the_envelope() {
    let envelope = MotionLimits::<2>::envelope::<TwoJointArm>();
    let tighter = |edit: fn(&mut MotionLimits<2>)| {
        let mut limits = envelope;
        edit(&mut limits);
        limits.within(&envelope)
    };
    assert!(envelope.within(&envelope).is_ok());
    assert!(tighter(|l| l.joint.dq_max = [1.0; 2]).is_ok());
    assert!(tighter(|l| l.joint.dddq_max = [100.0; 2]).is_ok());
    assert!(tighter(|l| l.scale = 0.1).is_ok());

    assert!(tighter(|l| l.joint.dq_max = [4.0; 2]).is_err());
    assert!(tighter(|l| l.joint.q_max = [3.0; 2]).is_err());
    assert!(tighter(|l| l.joint.q_min = [2.0; 2]).is_err());
    assert!(tighter(|l| l.joint.ddq_max[1] = f64::NAN).is_err());
    assert!(tighter(|l| l.joint.tau_max = [0.0; 2]).is_err());
    assert!(tighter(|l| l.cartesian.linear_velocity_max = 2.0).is_err());
    assert!(tighter(|l| l.scale = 0.0).is_err());
    assert!(tighter(|l| l.scale = 1.5).is_err());
}

#[test]
fn limits_for_one_motion_are_restored_after_it() {
    let mut arm = TwoJointArm::new();
    let mut slow = arm.motion_limits();
    slow.joint.dq_max = [1.0; 2];

    arm.with_limits(slow, |r| r.move_to::<JointSpace<2>>([0.1, 0.2]))
        .unwrap();
    arm.move_to::<JointSpace<2>>([0.0; 2]).unwrap();
    assert_eq!(arm.seen_dq_max, [[1.0; 2], [3.0; 2]]);

    // A failing motion restores the limits too.
    let failed = arm.with_limits(slow, |r| r.move_to::<JointSpace<2>>([f64::NAN, 0.0]));
    assert!(failed.is_err());
    assert_eq!(arm.motion_limits(), MotionLimits::envelope::<TwoJointArm>());
}

#[test]
fn looser_limits_for_one_motion_are_refused_before_it() {
    let mut arm = TwoJointArm::new();
    let mut fast = arm.motion_limits();
    fast.joint.dq_max = [5.0; 2];
    let result = arm.with_limits(fast, |r| r.move_to::<JointSpace<2>>([0.1, 0.2]));
    assert!(result.is_err());
    assert!(arm.seen_dq_max.is_empty());
    assert_eq!(
        JointLimits::<2>::UNBOUNDED.dq_max,
        [f64::INFINITY; 2],
        "unbounded is infinite"
    );
    let _ = CartesianLimits::UNBOUNDED;
}
