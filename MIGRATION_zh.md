# 从 roplat_behavior 0.9 迁移到 0.10

[English](MIGRATION.md)

0.10 不改变契约的语义，改的是限值、相对运动、控制周期和外语绑定的表达方式，以及各项所在的包。每个包各自定版本。需要带新 `JointLimits` 字段的 `roplat_msgs`；发布之前用 roplat 分支 `claude/msgs-limits`。

## 包

| 需要 | 0.9 | 0.10 |
|---|---|---|
| Python／C++ 绑定 | `roplat_behavior` 的特性 `to_py`／`to_cxx` | `roplat_behavior_ffi` 的特性 `py`／`cxx` |
| 仿真后端、物理引擎、渲染器、`SimulationRhythm` | `roplat_behavior` | `roplat_world`（`SimulationRhythm` 在其 `roplat` 特性下） |
| 目标源（`joint_traj_target_fn` 等） | `roplat_trajectory::path_generate` | `roplat_control::target` |

`roplat_trajectory` 不再依赖 `roplat_behavior`，`roplat_control` 不再依赖 `roplat_trajectory`。C 绑定（`to_c`）已删除。

## 限值

- 无界写 `f64::INFINITY`（位置下限为 `f64::NEG_INFINITY`）。`Joints`、`EndPoint` 的常量默认值改为它；声明为 `f64::MAX` 的仍视为无界。
- `Joints::joint_limits()`、`EndPoint::cartesian_limits()` 把声明的包络转为 `roplat_msgs::JointLimits`／`CartesianLimits`。`JointLimits` 新增 `dddq_max`（jerk）与 `dtau_max`（力矩变化率），不设的字段用 `..JointLimits::UNBOUNDED` 补齐。
- `Arm<N>` 去掉 `get_joint_min` 至 `get_rotation_jerk_bound` 的各个 getter 和全部 `with_*`。驱动保存一份 `MotionLimits<N>`，实现：

```rust
fn motion_limits(&self) -> MotionLimits<N> { self.limits }
fn set_motion_limits(&mut self, limits: MotionLimits<N>) -> RobotResult<()> {
    limits.within(&MotionLimits::envelope::<Self>())?;
    self.limits = limits;
    Ok(())
}
```

  规划时用 `self.limits.scaled_joint()`／`scaled_cartesian()`。调用方持久修改写 `robot.set_motion_limits(robot.motion_limits().scaled(0.2))`；只对一次运动生效写 `robot.with_limits(limits, |r| r.move_to::<…>(target))`，运动失败也会恢复原限值。`OverrideOnce`、`ArmBound`、`RobotBound` 已删除。
- `roplat_safety::joint_limits::<R, N>()` 改为 `R::joint_limits()`；`roplat_safety::CartesianLimits` 改用 `roplat_msgs::CartesianLimits`；`JointTorqueCheck` 从限值中读取 `dtau_max`。

## 相对运动、周期与法兰

- 删除 `Coord` 与 `Inertial<S>`。相对运动写 `move_to::<Relative<JointSpace<N>>>(dq)` 或 `move_to::<Relative<FlangeSpace>>(delta)`（`delta` 在法兰坐标系中表达），由契约为所有机器人统一实现。
- `get_endpoint` 从 `Arm<N>` 移到新的 `EndPointPose`，每个 `Arm<N>` 都要实现它。`EndPoint` 只保留声明的笛卡尔限值，因此只描述模型的类型只实现 `EndPoint`，正如它只实现 `Joints` 而不实现 `Arm`。相对运动从机器人读取当前值：关节增量以 `Arm::state()` 中的实测位置为起点，法兰增量以 `EndPointPose::read_endpoint()` 为起点，后者默认返回 `get_endpoint()`。`get_endpoint` 读不到机器人的驱动应覆写 `read_endpoint`。
- `Robot::CONTROL_PERIOD` 改为 `Duration`：`const CONTROL_PERIOD: Duration = Duration::from_millis(1);`。控制回调的 `Duration` 是这个周期，丢 `k` 个周期后为 `(k + 1)` 个周期。

## 轨迹

- 规划函数接收 `&JointLimits<N>` 与控制周期，不再接收机器人类型和 scale，返回 `TimeScalingError`；驱动用 `.map_err(RobotException::from_algorithm)?` 转成自己的错误。
- 笛卡尔规划为 `plan_cartesian_path_traj_via_time_scaling(path, q_start, ik, &limits, period, options)` 与 `plan_cartesian_line_traj_via_time_scaling(start, target, q_start, ik, &limits, &cartesian, period, options)`；路径约束在 `CartesianTimeScalingOptions::path_bounds`。`_with_options`、`_with_limits` 变体与 `CartesianTimeScalingLimits` 已删除。
- COPP 与笛卡尔规划要求速度、加速度有界；jerk 为无穷时不约束 jerk。
- 删除 `joint_traj_position_control` 与 `copp_*_joint_position_control`。播放轨迹用 `JointTrajectoryFollower`；没有时钟的驱动用 `JointTrajectoryFollower::cycle_timed(points_from_samples(&plan, period))`，每个周期调用 `step_by(dt, &state)`。

## 运动学

- `ArmForwardKinematics::FLANGE` 是法兰在最后一个 DH 坐标系中的位姿（Franka 为 `Pose::from_position([0., 0., 0.107])`）。正运动学、`flange_jacobian` 与逆运动学都以法兰为准。
- `dh_kinematics!(…, inverse)` 改用 `ik_solve_converged`：不可达目标现在返回错误。

## 外语绑定

```toml
[dependencies]
roplat_behavior_ffi = { version = "0.9.0", optional = true }
pyo3 = { version = "0.28.3", optional = true, features = ["extension-module", "multiple-pymethods"] }

[features]
to_py = ["dep:pyo3", "dep:roplat_behavior_ffi", "roplat_behavior_ffi/py"]
```

宏和数据类改从 `roplat_behavior_ffi` 引入。自己写的、用 `?` 把 `RobotResult` 转成 `PyResult` 的代码，改为 `.map_err(roplat_behavior_ffi::py_err)?`。

## 世界

`PhysicsEngine`、`AddSearchPath`、`Renderer`、`AttachFrom`、`Entity`、各实体构建 trait、`SimulationBackend`、`select_simulation_model` 与 `SimulationRhythm` 改从 `roplat_world` 引入；`roplat_data_dir()` 改为 `roplat_world::assets_dir()`。`RobotDescription`、`ModelFormat`、`SimulationModel` 仍在 `roplat_behavior`。URDF 的 `Geometry` 可以转换为 `Entity`。

## 面板：停止与使能从运动中分出

设备的停止、使能与状态从 `Robot` 移到第二个句柄 `Panel`。设备经 `HasPanel::panel()` 发出它，类型为 `Arc<dyn Panel>`。它的方法都取 `&self`，所以 `move_to` 占着控制句柄时，服务可以从别的线程停住设备。每个操作写明调用者（`by: &Actor`：`Device`、`Operator(name)`、`Agent(name)`、`App(name)`）。

| 0.9（`Robot` 上，`&mut self`） | 0.10（`Panel` 上，`&self`） |
|---|---|
| `robot.stop()` | `panel.stop(&by)`：进行中的运动在 `panel.stop_bound()` 内返回 `Err(Stopped)`；不锁存 |
| `robot.emergency_stop()` | `panel.emergency_stop(&by)`：只有锁存了才答 `Ok`，否则答 `NoEmergencyStop`，与原来相同 |
| `robot.clear_emergency_stop()` | `panel.clear(&by) -> PanelStatus`：清除软件可以清除的一切停止，不论是谁引起的；需要现场复位的停止还在时答 `Err(LocalResetRequired)` |
| `robot.enable()`、`robot.disable()`、`robot.pause()`、`robot.resume()` | `panel.enable(&by)` 等；缺省答 `NotSupported`，不是 `Ok` |
| 无 | `panel.status() -> PanelStatus`：`enabled`、`paused`、`software_estop`、`device_estop`、`protective_stop`（后两个取 `Release::Clear` 或 `Release::OnSite`） |

`Robot` 的 `stop`、`emergency_stop`、`clear_emergency_stop`、`enable`、`disable`、`pause`、`resume` 在 0.10 里照常可用，标为弃用，下一个小版本删除。覆写它们的驱动编译时没有警告；调用它们的代码会有弃用警告，`-D warnings` 下即为错误。`init`、`shutdown`、`reset`、`is_moving`、`waiting_for_finish` 仍留在 `Robot` 上。

**驱动**在一个与驱动共享状态的小值上实现 `Panel`，在驱动上实现 `HasPanel`：

```rust
impl HasPanel for MyArm {
    fn panel(&self) -> Arc<dyn Panel> {
        Arc::new(MyPanel { shared: Arc::clone(&self.shared) })
    }
}
```

- 控制器有第二个通道（dashboard 端口、第二个现场总线客户端）的，停止从那里发。
- 运动是本进程里循环的驱动，共享一个停止计数，循环每周期检查，变了就返回 `Err(RobotException::Stopped)`；参照是 `roplat_world::KinematicArm`。
- 厂商的阻塞调用中途无法打断的，`stop` 答 `NotSupported`，文档写明。
- `stop_bound()` 是设备结束运动并静止所需的时间，含减速，以及一个周期内最长的等待。
- `status()` 从设备读：软件急停指经 `emergency_stop` 要求的急停；设备自己的按钮或安全回路是 `device_estop`，通常为 `Release::OnSite`；设备为保护自己而停是 `protective_stop`。

在驱动的测试里跑 `conformance::check_panel(&mut robot, &check, joints, press)`：它从另一个线程停住运动并检查停止时限；检查急停在 `clear` 之前一直有效、`enable` 在此之前不成功；`press` 能按下设备自己的急停时，检查 `clear` 答 `LocalResetRequired`。

**服务**（MCP、Web、控制台）在把机器人交给运动线程之前取 `robot.panel()`，不再另用一个 `Robot` 值来停止。谁可以做哪个恢复操作由服务定，面板不管。要留记录，就把面板包一层，发出去的是包装：

```rust
let log = Arc::new(LoggedPanel::new(robot.panel()));
let for_agent: Arc<dyn Panel> = log.clone();
let mut news = log.subscribe();           // news.next(timeout) -> EventBatch
let last_ten = log.recent(10);            // PanelEvent { seq, at, by, op, outcome, after }
```

`LoggedPanel` 记下每个操作的调用者与回答；经它读状态时发现的、没有任何经它的操作能解释的状态变化，记为 `Actor::Device`／`PanelOp::Changed`。事件可用 serde 序列化。

新的错误：`RobotException::Stopped`（停止结束了运动，或拒绝了这次调用）与 `RobotException::LocalResetRequired`。穷举匹配 `RobotException` 的代码要补上这两项。


# 从 robot_behavior 0.8 迁移到 roplat_behavior 0.9

[English](MIGRATION.md)

`robot_behavior` 改名为 `roplat_behavior`，并拆成共用一个版本的四个包。契约及其语义不变：类型化的运动空间与控制通道，`StateView` 的测量 / 已接受指令 / 期望三种视图，`ControlStep` 的收尾，阻塞会话与原生异步会话的区分，以及 roplat 适配中 `Resources = (R,)` 的借用。变化在于各部分的位置，以及采样和位姿类型改由 `roplat_msgs` 提供。

截至 2026-10-26，`roplat_behavior` 0.9 与 `roplat_msgs` 都还没有发布到 crates.io；发布之前请从源码构建。

## 依赖

```toml
[dependencies]
# robot_behavior = "0.8"
roplat_behavior = "0.9.0"
roplat_kinematics = "0.9.0"   # DH 运动学、逆解
roplat_control = "0.9.0"      # PID、阻抗、计算力矩
roplat_trajectory = "0.9.0"   # 插值、COPP 时间缩放
```

路径中的 `robot_behavior::` 改为 `roplat_behavior::`。feature 不变（`roplat`、`ffi`、`to_c`、`to_cxx`、`to_py`），`roplat` 现在要求 `roplat = "0.6.0-rc.1"`。

## 各项的新位置

| 0.8 路径 | 0.9 路径 |
|---|---|
| `robot_behavior::{Robot, MoveTo, ControlWith, JointSpace, Arm, SpaceMap, …}` | `roplat_behavior::…`（名字不变） |
| `Pose`、`JointSample`、`JointState`、`SpatialSample`、`ContactSample`、`StateView`、`LoadState` | `roplat_msgs`，由 `roplat_behavior` 重新导出 |
| `DhParam`、`dh_param!`、`mdh_param!`、`ArmKineCache`、`ArmForwardKinematics`、`ArmInverseKinematics`、`IKMethod`、`CommonStop`、`AnalyticFamily`、`Link`、`Joint`、`JointType`、`ArmDynamics`、`combine_array` | `roplat_kinematics` |
| `robot_behavior::controller::*` 及根部的重新导出（`joint_pd_control`、`joint_impedance_control`、`gravity_compensation_control` 等）、`set_realtime_priority`、`is_hardware_realtime`、`RealtimeBehavior` | `roplat_control` |
| `robot_behavior::utils::path_generate::*` | `roplat_trajectory::path_generate::*` |
| `plan_waypoints_traj_via_copp`、`plan_path_traj_via_copp`、`plan_s_t_via_copp`、`plan_cartesian_path_traj_via_time_scaling`、`CartesianTimeScalingOptions` | `roplat_trajectory` |
| `Iso3`、`JVec`、`JMat`、`Jaco` | `roplat_behavior`（不变） |
| nalgebra 别名 `Twist`、`Wrench`（六维向量） | `roplat_kinematics::SpatialVec`；`Twist` / `Wrench` 这两个名字现在指 `roplat_msgs` 的结构体 |
| `OverrideOnce`、`RobotBound`、`ArmBound`、`limit`、`homo_to_isometry`、`to_radians_array` 等 | `roplat_behavior`（不变） |

删除了从未参与编译的 `src/robot_old` 与 `src/utils/inverse.rs`。

## 位姿

`Pose` 从五个 enum 变体改为一个结构体 `{ position: Vector3, orientation: Quaternion }`，四元数按 `x, y, z, w` 存储。访问方法名字不变：`euler()`、`quat()`、`quat_array()`、`homo()`、`axis_angle()`、`position()`。

| 0.8 | 0.9 |
|---|---|
| `Pose::Euler(p, rpy)` | `Pose::from_euler(p, rpy)` |
| `Pose::Quat(isometry)` | `Pose::from_isometry(isometry)` 或 `isometry.into()` |
| `Pose::Homo(m)` | `Pose::from_homo(m)` |
| `Pose::AxisAngle(p, axis, angle)` | `Pose::from_axis_angle(p, axis, angle)` |
| `Pose::Position(p)` | `Pose::from_position(p)` |
| `match pose { Pose::Homo(m) => … }` | `pose.homo()` |
| `pose_a * pose_b` | 不变（复合，现在一律经四元数计算） |
| `pose_a / pose_b` | 删除；它对位置求 `2^(a - b)` 之和，且没有调用方 |

数组形式的 `From` 转换含义不变（`[f64; 3]` 位置，`[f64; 6]` 欧拉角，`[f64; 7]` 仍按零向量 / 单位四元数 / 轴角规则区分，`[f64; 16]` 齐次矩阵，以及切片和 `Vec`）。`Vec<f64>::from(pose)` 一律得到 `[x, y, z, qx, qy, qz, qw]`；0.8 的四元数变体只给出 6 个数。

序列化：0.8 写出的 JSON（`{"Euler": …}`、`{"Quat": …}`、`{"Homo": …}`、`{"AxisAngle": …}`、`{"Position": …}`）仍可读取，新输出为 `{"position": …, "orientation": …}`。

C、C++、Python 绑定以四元数形式返回位姿，构造时仍接受所有形式。C/C++ 的 `Quat` 输入现在直接按四元数读取，不再经过 7 个数的判别规则；旧规则在范数不恰好为 1 时可能把四元数误读成轴角。

## 采样

可能缺失的字段改为位掩码加访问器，采样因此有固定布局。

| 0.8 | 0.9 |
|---|---|
| `sample.q`（`Option<[f64; N]>`） | `sample.q()` |
| `JointSample { q: Some(q), dq: Some(dq), ..Default::default() }` | `JointSample::EMPTY.with_q(q).with_dq(dq)` |
| `sample.q = Some(q)` / `= None` | `sample.set_q(Some(q))` / `set_q(None)` |
| `spatial.pose`、`spatial.vel`、`spatial.acc`、`spatial.wrench` | `spatial.pose()`、`spatial.twist()`、`spatial.accel()`、`spatial.wrench()`，返回 `Pose`、`Twist`、`Accel`、`Wrench` |
| `vel: Some([f64; 6])` | `.with_twist(array.into())`；读取用 `.twist().map(<[f64; 6]>::from)` |
| `contact.on`、`contact.f`、`contact.p`、`contact.n` | `contact.is_active()`、`force()`、`point()`、`normal()`（`Vector3`） |
| `LoadState { m, x, i }` | `LoadState { mass, center_of_mass, inertia }` |

默认值不变：`JointSample::default()` 不含任何字段；`SpatialSample::default()` 与 `ContactSample::default()` 与以前一样含全部字段，取单位位姿和零值。`SpatialSample::EMPTY` 与 `ContactSample::EMPTY` 不含任何字段。

`JacobianModel::twist` 返回 `Twist` 结构体，不再是 nalgebra 六维向量。

## 运动学

实现 `ArmForwardKinematics<N>` 的类型不再自动获得 `SpaceMap` 的法兰位姿与雅可比映射；这些 blanket impl 会把契约包绑在 DH 实现上。在 DH 参数表旁加一行：

```rust
roplat_kinematics::dh_kinematics!(MyArm, 6);           // 正运动学与雅可比映射
roplat_kinematics::dh_kinematics!(MyArm, 6, inverse);  // 同时实现逆解映射
```

`<MyArm as ArmInverseKinematics<6>>::ik_solve` 这类调用照旧可用。

## Python 与 C++

Python 模块名为 `roplat_behavior`（原为 `robot_behavior`）；类与字段不变，包括 `LoadState(m, x, i)`。C++ 头文件为 `roplat_behavior.hpp`，命名空间为 `roplat_behavior`。

## Nightly

所有包都不再使用 `generic_const_exprs`：`roplat_behavior` 只用 `async_trait_bounds`，`roplat_kinematics` 不需要不稳定特性。nightly 2026-10-20 的新一代 trait solver 不支持 `generic_const_exprs`，依赖它的 crate 会让编译器崩溃，因此请同样去掉自己 crate 中的 `#![feature(generic_const_exprs)]`。

`ArmKineCache` 不再存放 `N + 1` 个坐标系：基座（link 0）恒为单位变换，字段 `t_prefix`、`origins`、`z_axes` 改名为 `link_frames`、`link_origins`、`link_z_axes`，长度为 `N`，对应 link `1..=N`。`link_pose`、`relative_pose`、`end_effector_pose` 与各雅可比不变。`combine_array` 的输出长度改为常量参数，通常可推断：`let joined: [T; 5] = combine_array(&a, &b);`。
