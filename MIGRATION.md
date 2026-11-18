# Migrating from roplat_behavior 0.9 to 0.10

[简体中文](MIGRATION_zh.md)

0.10 changes how limits, relative motion, periods and foreign bindings are expressed, and which crate holds what. It also changes some semantics: stopping, enabling and clearing a device move from `Robot` to a second handle, the `Panel`, whose operations default to `NotSupported` rather than `Ok` (provisional for `enable` and `disable`, pending the author's ruling); the `Robot` methods they replace still compile and are deprecated; and `RobotException` gains `Stopped` and `LocalResetRequired`. New contracts cover multirotor velocity setpoints, car-like vehicles, batteries and segmentation images, and `target_pose` and the lens functions change behaviour (see "New contracts, perception and frames"). Each crate now has its own version. Needs `roplat_msgs` with the new `JointLimits` fields, which roplat main has; until roplat is released, patch it to a roplat checkout.

## Crates

| Need | 0.9 | 0.10 |
|---|---|---|
| Python / C++ bindings | `roplat_behavior` features `to_py` / `to_cxx` | `roplat_behavior_ffi` features `py` / `cxx` |
| Simulator backends, physics engines, renderers, `SimulationRhythm` | `roplat_behavior` | `roplat_world` (`SimulationRhythm` with its feature `roplat`) |
| Target sources (`joint_traj_target_fn`, …) | `roplat_trajectory::path_generate` | `roplat_control::target` |

`roplat_trajectory` no longer depends on `roplat_behavior`, and `roplat_control` no longer on `roplat_trajectory`. The C bindings (`to_c`) are removed.

## Limits

- An unbounded limit is `f64::INFINITY` (`f64::NEG_INFINITY` for a lower position bound). `Joints` and `EndPoint` constants now default to it; a declared `f64::MAX` still reads as unbounded.
- `Joints::joint_limits()` and `EndPoint::cartesian_limits()` return the declared envelope as `roplat_msgs::JointLimits` / `CartesianLimits`. `JointLimits` gains `dddq_max` (jerk) and `dtau_max` (effort rate); `..JointLimits::UNBOUNDED` fills fields you do not set.
- `Arm<N>` drops `get_joint_min` … `get_rotation_jerk_bound` and every `with_*` builder. A driver keeps one `MotionLimits<N>` and implements:

```rust
fn motion_limits(&self) -> MotionLimits<N> { self.limits }
fn set_motion_limits(&mut self, limits: MotionLimits<N>) -> RobotResult<()> {
    limits.within(&MotionLimits::envelope::<Self>())?;
    self.limits = limits;
    Ok(())
}
```

  It plans with `self.limits.scaled_joint()` / `scaled_cartesian()`. Callers use `robot.set_motion_limits(robot.motion_limits().scaled(0.2))` for a lasting change and `robot.with_limits(limits, |r| r.move_to::<…>(target))` for one motion; `with_limits` restores the previous limits even when the motion fails. `OverrideOnce`, `ArmBound` and `RobotBound` are removed.
- `roplat_safety::joint_limits::<R, N>()` becomes `R::joint_limits()`; `roplat_safety::CartesianLimits` is `roplat_msgs::CartesianLimits`, and `JointTorqueCheck` reads `dtau_max` from the limits.

## Relative motion, periods and the flange

- `Coord` and `Inertial<S>` are removed. A relative move is `move_to::<Relative<JointSpace<N>>>(dq)` or `move_to::<Relative<FlangeSpace>>(delta)` (`delta` in the flange frame); the contract implements both for every robot.
- `get_endpoint` moves from `Arm<N>` to the new `EndPointPose`, which every `Arm<N>` implements. `EndPoint` keeps only the declared Cartesian limits, so a model description implements `EndPoint` alone, as it implements `Joints` but not `Arm`. A relative motion reads the robot: joint offsets start from the measured positions in `Arm::state()`, flange offsets from `EndPointPose::read_endpoint()`, which defaults to `get_endpoint()`. A driver whose `get_endpoint` cannot reach the robot overrides `read_endpoint`.
- `Robot::CONTROL_PERIOD` is a `Duration`: `const CONTROL_PERIOD: Duration = Duration::from_millis(1);`. A control callback's `Duration` is that period, or `(k + 1)` periods after `k` missed cycles.

## Trajectories

- Planners take `&JointLimits<N>` and the control period instead of a robot type and a scale, and return `TimeScalingError`; a driver turns it into its own error with `.map_err(RobotException::from_algorithm)?`.
- The Cartesian planners are `plan_cartesian_path_traj_via_time_scaling(path, q_start, ik, &limits, period, options)` and `plan_cartesian_line_traj_via_time_scaling(start, target, q_start, ik, &limits, &cartesian, period, options)`; path bounds are `CartesianTimeScalingOptions::path_bounds`. The `_with_options` / `_with_limits` variants and `CartesianTimeScalingLimits` are gone.
- COPP and the Cartesian planner need finite velocity and acceleration bounds; an infinite jerk bound leaves jerk free.
- `joint_traj_position_control` and `copp_*_joint_position_control` are removed. Play a plan with `JointTrajectoryFollower`; a driver without a clock uses `JointTrajectoryFollower::cycle_timed(points_from_samples(&plan, period))` and calls `step_by(dt, &state)` each cycle.

## Kinematics

- `ArmForwardKinematics::FLANGE` is the flange frame in the last DH frame (Franka: `Pose::from_position([0., 0., 0.107])`). Forward kinematics, `flange_jacobian` and the IK refer to the flange.
- `dh_kinematics!(…, inverse)` uses `ik_solve_converged`: an unreachable target is now an error.

## Foreign bindings

```toml
[dependencies]
roplat_behavior_ffi = { version = "0.9.0", optional = true }
pyo3 = { version = "0.28.3", optional = true, features = ["extension-module", "multiple-pymethods"] }

[features]
to_py = ["dep:pyo3", "dep:roplat_behavior_ffi", "roplat_behavior_ffi/py"]
```

Import the macros and classes from `roplat_behavior_ffi`. Code of your own that turned a `RobotResult` into a `PyResult` with `?` calls `.map_err(roplat_behavior_ffi::py_err)?` instead.

## World

`PhysicsEngine`, `AddSearchPath`, `Renderer`, `AttachFrom`, `Entity`, the entity builders, `SimulationBackend`, `select_simulation_model` and `SimulationRhythm` are imported from `roplat_world`; `roplat_data_dir()` is `roplat_world::assets_dir()`. `RobotDescription`, `ModelFormat` and `SimulationModel` stay in `roplat_behavior`. A URDF `Geometry` converts into an `Entity`.

## The panel: stopping and enabling apart from motion

A device's stops, enabling and status move from `Robot` to a second handle, the `Panel`, which the device issues with `HasPanel::panel()` as an `Arc<dyn Panel>`. Its methods take `&self`, so a service can stop a device from another thread while `move_to` holds the control handle. Every operation names its caller (`by: &Actor`: `Device`, `Operator(name)`, `Agent(name)`, `App(name)`).

| 0.9 (`&mut self` on `Robot`) | 0.10 (`&self` on `Panel`) |
|---|---|
| `robot.stop()` | `panel.stop(&by)`: a motion in progress returns `Err(Stopped)` within `panel.stop_bound()`; nothing latches |
| `robot.emergency_stop()` | `panel.emergency_stop(&by)`: `Ok` only when latched, else `NoEmergencyStop`, as before |
| `robot.clear_emergency_stop()` | `panel.clear(&by) -> PanelStatus`: releases every stop software may release, whoever caused it; `Err(LocalResetRequired)` while a stop that needs an on-site reset holds the device |
| `robot.enable()`, `robot.disable()`, `robot.pause()`, `robot.resume()` | `panel.enable(&by)` and so on; they default to `NotSupported`, not `Ok` (the `enable` and `disable` defaults are provisional, pending the author's ruling: engineering-style ruling 27 keeps `Robot`'s at `Ok`; driver panels write both out until then) |
| none | `panel.status() -> PanelStatus`: `enabled`, `paused`, `software_estop`, `device_estop`, `protective_stop` (the last two with `Release::Clear` or `Release::OnSite`) |

`Robot::stop`, `emergency_stop`, `clear_emergency_stop`, `enable`, `disable`, `pause` and `resume` still work in 0.10 and are deprecated; they go in the next minor version. A driver that overrides them compiles without warnings; code that calls them gets a deprecation warning, which `-D warnings` turns into an error. `init`, `shutdown`, `reset`, `is_moving` and `waiting_for_finish` stay on `Robot`.

**A driver** implements `Panel` on a small value sharing state with the driver, and `HasPanel` on the driver:

```rust
impl HasPanel for MyArm {
    fn panel(&self) -> Arc<dyn Panel> {
        Arc::new(MyPanel { shared: Arc::clone(&self.shared) })
    }
}
```

- A controller with a second channel (a dashboard port, a second fieldbus client) sends the stop there.
- A driver whose motion is a loop in its own process shares a stop counter that the loop checks every cycle and returns `Err(RobotException::Stopped)` when it changes; `roplat_world::KinematicArm` is the reference.
- A blocking vendor call that cannot be interrupted: `stop` answers `NotSupported` and the docs say so.
- `stop_bound()` is what the device needs to end a motion and come to rest, including deceleration and the longest wait inside one cycle.
- `status()` reads the device: a software emergency stop is one asked for through `emergency_stop`; the device's own button or safety circuit is `device_estop`, usually `Release::OnSite`; a stop the device made to protect itself is `protective_stop`.

Run `conformance::check_panel(&mut robot, &check, joints, press)` in the driver's tests: it stops a motion from another thread and holds the driver to its bound, checks that an emergency stop holds until `clear` and that `enable` fails before it, and, when `press` can engage the device's own emergency stop, that `clear` answers `LocalResetRequired`.

**A service** (MCP, web, console) takes `robot.panel()` before handing the robot to its motion thread, instead of a second `Robot` value for stopping. It decides who may use which recovery operation; the panel does not. To keep a record, wrap the panel once and hand out the wrapper:

```rust
let log = Arc::new(LoggedPanel::new(robot.panel()));
let for_agent: Arc<dyn Panel> = log.clone();
let mut news = log.subscribe();           // news.next(timeout) -> EventBatch
let last_ten = log.recent(10);            // PanelEvent { seq, at, by, op, outcome, after }
```

`LoggedPanel` records each operation with its caller and answer, and a change of status that no operation through it explains as `Actor::Device` / `PanelOp::Changed`, noticed when the status is read through it. Events serialize with serde.

New errors: `RobotException::Stopped` (a stop ended the motion or refused the call) and `RobotException::LocalResetRequired`. Code that matches `RobotException` exhaustively adds them.

## New contracts, perception and frames

These additions keep existing code compiling; where a signature changed, the old calls still compile unless noted.

- **Multirotor velocity setpoints.** `Multirotor::send_velocity(VelocitySetpoint { velocity, yaw, duration })`: `velocity` in m/s along the world east, north and up axes of position setpoints, `yaw` one of `YawCommand::Hold`, `Angle(rad)` or `Rate(rad/s)` (counter-clockwise about up), and `duration` how long the setpoint holds without another. It defaults to `NotSupported`, so a driver without it still builds; a driver that sends velocities overrides it and calls `setpoint.validate()?` first.
- **Car-like vehicles.** A vehicle steered by its front wheels implements `CarLike`: `geometry()` (`CarGeometry { wheelbase, max_steering_angle, max_steering_rate }`) and `drive(SteerCommand { speed, steering_angle })`, the speed of the rear axle's midpoint and the bicycle model's front-wheel angle, left positive. A command beyond `max_steering_angle` steers as far as the vehicle can (`CarGeometry::clamp`); a command that is not finite is refused (`SteerCommand::validate`). It does not replace `MobileBase`, which stays for bases that turn on the spot. `conformance::check_car_like` checks the geometry and the refusal.
- **Batteries.** A type that reads a pack implements `Battery::read_battery() -> Stamped<BatteryReading>`. Every quantity is an `Option`; the current is positive while the pack discharges, the state of charge a share in `[0, 1]`, the temperature in °C, `faults` a `BatteryFaults` set. A driver whose device reports discharge current as negative, or percent, converts. `conformance::check_battery` checks the units and the sign against the status.
- **Segmentation images.** A camera that labels pixels gives its frame type an `Option<LabelImage>` (`width`, `height`, `ids` row by row, `classes` naming each id, `LabelImage::UNLABELLED` for nothing labelled) beside the image it labels. `conformance::check_segmentation(camera, advance, frames, |frame| frame.labels.as_ref())` checks it.
- **Joint jogging.** `roplat_control::joint_jog_control`, `JogMapping` and `JogInput` are roplat_web's `jog_control` and its types. `JogMapping::new(speeds, accelerations, enable_button)` takes the dead-man button, which roplat_web's version fixed to its page's bit 0: pass `Some(0)` for the same behaviour.
- **Object observations.** `roplat_perception::ObjectObservation` (with `PixelBox` and `ObjectMask`) is the type a detector, segmenter, marker reader or fusion step reports an object in. Build one with `ObjectObservation { score, pixel_box, ..ObjectObservation::new(class, stamp, frame) }`.
- **Frames.** `FrameTree::edges()` and `edge(child)` list how each frame is placed; `FrameTree::set_fixed` and `FrameBuffer::set_fixed` replace a fixed edge's pose in place. `so3` (exponential, logarithm, Jacobians) and `twist` (adjoint, twist and wrench transforms) are new modules.
- **Lens and target pose.** Changed behaviour:
  - `target::target_pose` returns `Result<TargetPose, TargetPoseError>` instead of an `Option` (`.unwrap()` and `.ok()` still work) and models the lens: pass the camera's own `CameraInfo` with the raw corners, or `lens::rectified(info)` with corners of a rectified image as before.
  - The lens functions refuse a `PLUMB_BOB` camera with any of `d[5..8]` set (ROS and OpenCV define five coefficients; such a camera is `RATIONAL_POLYNOMIAL`), an `EQUIDISTANT` one with any of `d[4..8]` set, and coefficients that are not finite.
  - The camera matrix's skew `k[1]` is now modelled (`u = fx x + s y + cx`), as the `Camera` contract defines it; before, it was ignored.
  - New: `lens::project`, `lens::ray`, `Rectify::onto` (another camera matrix and size), `Rectify::info` and `Rectify::mask`; `target::Chessboard::render`.


# Migrating from robot_behavior 0.8 to roplat_behavior 0.9

[简体中文](MIGRATION_zh.md)

`robot_behavior` is renamed `roplat_behavior` and split into four crates with one version. The contracts and their semantics are unchanged: typed motion spaces and control channels, `StateView`'s measured / accepted-command / desired views, `ControlStep`'s termination, the split between blocking sessions and native async sessions, and `Resources = (R,)` borrowing in the roplat adapters. What changes is where things live, and the sample and pose types, which now come from `roplat_msgs`.

As of 2026-10-26 neither `roplat_behavior` 0.9 nor `roplat_msgs` is on crates.io; build from source until they are released.

## Dependencies

```toml
[dependencies]
# robot_behavior = "0.8"
roplat_behavior = "0.9.0"
roplat_kinematics = "0.9.0"   # DH kinematics, IK
roplat_control = "0.9.0"      # PID, impedance, computed torque
roplat_trajectory = "0.9.0"   # interpolation, COPP time scaling
```

Replace `robot_behavior::` with `roplat_behavior::` in paths. Features are unchanged (`roplat`, `ffi`, `to_c`, `to_cxx`, `to_py`), and `roplat` now asks for `roplat = "0.6.0-rc.1"`.

## Where items moved

| 0.8 path | 0.9 path |
|---|---|
| `robot_behavior::{Robot, MoveTo, ControlWith, JointSpace, Arm, SpaceMap, …}` | `roplat_behavior::…` (unchanged names) |
| `Pose`, `JointSample`, `JointState`, `SpatialSample`, `ContactSample`, `StateView`, `LoadState` | `roplat_msgs`, re-exported by `roplat_behavior` |
| `DhParam`, `dh_param!`, `mdh_param!`, `ArmKineCache`, `ArmForwardKinematics`, `ArmInverseKinematics`, `IKMethod`, `CommonStop`, `AnalyticFamily`, `Link`, `Joint`, `JointType`, `ArmDynamics`, `combine_array` | `roplat_kinematics` |
| `robot_behavior::controller::*` and the root re-exports (`joint_pd_control`, `joint_impedance_control`, `gravity_compensation_control`, …), `set_realtime_priority`, `is_hardware_realtime`, `RealtimeBehavior` | `roplat_control` |
| `robot_behavior::utils::path_generate::*` | `roplat_trajectory::path_generate::*` |
| `plan_waypoints_traj_via_copp`, `plan_path_traj_via_copp`, `plan_s_t_via_copp`, `plan_cartesian_path_traj_via_time_scaling`, `CartesianTimeScalingOptions` | `roplat_trajectory` |
| `Iso3`, `JVec`, `JMat`, `Jaco` | `roplat_behavior` (unchanged) |
| nalgebra aliases `Twist`, `Wrench` (6-vectors) | `roplat_kinematics::SpatialVec`; the names `Twist` / `Wrench` are now the `roplat_msgs` structs |
| `OverrideOnce`, `RobotBound`, `ArmBound`, `limit`, `homo_to_isometry`, `to_radians_array`, … | `roplat_behavior` (unchanged) |

Removed: the uncompiled `src/robot_old` and `src/utils/inverse.rs` sources.

## Pose

`Pose` is one struct, `{ position: Vector3, orientation: Quaternion }` with the quaternion stored `x, y, z, w`, instead of five enum variants. The accessors keep their names: `euler()`, `quat()`, `quat_array()`, `homo()`, `axis_angle()`, `position()`.

| 0.8 | 0.9 |
|---|---|
| `Pose::Euler(p, rpy)` | `Pose::from_euler(p, rpy)` |
| `Pose::Quat(isometry)` | `Pose::from_isometry(isometry)` or `isometry.into()` |
| `Pose::Homo(m)` | `Pose::from_homo(m)` |
| `Pose::AxisAngle(p, axis, angle)` | `Pose::from_axis_angle(p, axis, angle)` |
| `Pose::Position(p)` | `Pose::from_position(p)` |
| `match pose { Pose::Homo(m) => … }` | `pose.homo()` |
| `pose_a * pose_b` | unchanged (composition, now always through quaternions) |
| `pose_a / pose_b` | removed; it summed `2^(a - b)` over the position and had no callers |

The array `From` conversions keep their meaning (`[f64; 3]` position, `[f64; 6]` Euler, `[f64; 7]` with its zero / unit / axis-angle rule, `[f64; 16]` homogeneous, slices and `Vec`). `Vec<f64>::from(pose)` always yields `[x, y, z, qx, qy, qz, qw]`; in 0.8 the quaternion variant produced only six numbers.

Serialized poses: JSON written by 0.8 (`{"Euler": …}`, `{"Quat": …}`, `{"Homo": …}`, `{"AxisAngle": …}`, `{"Position": …}`) still deserializes. New output is `{"position": …, "orientation": …}`.

The C, C++ and Python bindings return poses in the quaternion form; their constructors still accept every form. A C/C++ `Quat` input is now read as a quaternion directly instead of through the seven-number heuristic, which could misread a quaternion whose norm was not exactly 1.

## Samples

Fields that may be absent are tracked by a validity mask behind accessors, so the samples have a fixed layout.

| 0.8 | 0.9 |
|---|---|
| `sample.q` (`Option<[f64; N]>`) | `sample.q()` |
| `JointSample { q: Some(q), dq: Some(dq), ..Default::default() }` | `JointSample::EMPTY.with_q(q).with_dq(dq)` |
| `sample.q = Some(q)` / `= None` | `sample.set_q(Some(q))` / `set_q(None)` |
| `spatial.pose`, `spatial.vel`, `spatial.acc`, `spatial.wrench` | `spatial.pose()`, `spatial.twist()`, `spatial.accel()`, `spatial.wrench()` returning `Pose`, `Twist`, `Accel`, `Wrench` |
| `vel: Some([f64; 6])` | `.with_twist(array.into())`; read back with `.twist().map(<[f64; 6]>::from)` |
| `contact.on`, `contact.f`, `contact.p`, `contact.n` | `contact.is_active()`, `force()`, `point()`, `normal()` (as `Vector3`) |
| `LoadState { m, x, i }` | `LoadState { mass, center_of_mass, inertia }` |

Defaults are unchanged: `JointSample::default()` has nothing present; `SpatialSample::default()` and `ContactSample::default()` have every field present with identity / zero values, as before. `SpatialSample::EMPTY` and `ContactSample::EMPTY` have nothing present.

`JacobianModel::twist` returns the `Twist` struct instead of a nalgebra 6-vector.

## Kinematics

A type implementing `ArmForwardKinematics<N>` no longer gets the `SpaceMap` flange-pose and Jacobian maps automatically; the blanket impls would have tied the contract crate to the DH implementation. Add one line next to the DH table:

```rust
roplat_kinematics::dh_kinematics!(MyArm, 6);           // FK + Jacobian maps
roplat_kinematics::dh_kinematics!(MyArm, 6, inverse);  // also the IK map
```

Calls such as `<MyArm as ArmInverseKinematics<6>>::ik_solve` work as before.

## Python and C++

The Python module is `roplat_behavior` (was `robot_behavior`); its classes and fields are unchanged, including `LoadState(m, x, i)`. The C++ header is `roplat_behavior.hpp`, namespace `roplat_behavior`.

## Nightly

No crate uses `generic_const_exprs` any more: `roplat_behavior` needs only `async_trait_bounds`, and `roplat_kinematics` none. Nightly 2026-10-20's next-generation trait solver does not support `generic_const_exprs` and crashes on crates that depend on it, so remove `#![feature(generic_const_exprs)]` from your crates too.

`ArmKineCache` no longer stores `N + 1` frames: the base (link 0) is the identity and the fields `t_prefix`, `origins`, `z_axes` became `link_frames`, `link_origins`, `link_z_axes` with `N` entries for links `1..=N`. `link_pose`, `relative_pose`, `end_effector_pose` and the Jacobians are unchanged. `combine_array` now takes the output length as a const parameter, usually inferred: `let joined: [T; 5] = combine_array(&a, &b);`.
