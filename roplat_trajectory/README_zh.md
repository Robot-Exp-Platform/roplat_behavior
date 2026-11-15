# roplat_trajectory

[English](README.md) | [简体中文](README_zh.md)

基于 `roplat_msgs` 数据的轨迹生成：插值、时间缩放、无碰撞关节路径及其平滑、末端直线路径，以及跟随定时的关节轨迹。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- 关节与笛卡尔插值：直线、梯形、S 曲线、四次曲线（`path_generate`）。
- 以数据表达限值（`JointLimits`、`CartesianLimits`、`PathBounds`）的 COPP 时间缩放（`time_scaling`）。
- 关节空间的无碰撞路径（`path_planning`）：`plan_joint_path` 用 RRT-Connect 求路径点，碰撞检查由调用方提供。它每隔 `resolution` 检查一个位形；`resolution_for_travel` 按场景尺度取分辨率，使相邻两次检查之间机器人上任何一点的位移不超过给定值（不大于最薄的障碍物）。默认的 0.02 rad 不来自任何场景：两节 0.6 m 连杆的机械臂会因此跨过 2 cm 的墙。
- 路径简化与平滑（`path_smoothing`）：`simplify_path` 与 `smooth_path` 去掉不需要的路径点并圆滑拐角，每段新路径都经过检查，与 OMPL 的路径简化器相同。
- `plan_joint_motion`、`simplify_motion` 与 `smooth_motion` 做同样的事，但对每段直线运动整体检查，不再逐点采样。
- 末端直线路径（`cartesian_path`）：`plan_cartesian_path` 让末端沿直线依次经过给定位姿，每步用调用方带种子的逆解求关节位形，在不可达、关节跳变或关节段无效之前停下；`plan_cartesian_motion` 对每一步整体检查。
- 在位置通道上跟随定时的关节轨迹（`follow`）：`JointTrajectoryFollower`。

碰撞检查由 [`roplat_scene`](../roplat_scene/README_zh.md) 提供，逆运动学由 [`roplat_kinematics`](../roplat_kinematics/README_zh.md) 提供。

## 以数据表达限值

本包以 `roplat_msgs` 数据接受限值，以 `Duration` 接受控制周期，从不接受机器人类型，因此只依赖 `roplat_msgs` 与 `copp`。无界写 `f64::INFINITY`，规划要求速度、加速度有界。

## 跟随关节轨迹

`JointTrajectoryFollower` 在关节位置通道上播放定时的 `JointTrajectoryPoint`。它从 `roplat_msgs::Clock` 读取时间，而不是累加周期 `Duration`，因此同一个跟随器可以运行在墙钟或仿真时间上；`JointTrajectoryFollower::cycle_timed` 配合 `step_by(dt, …)` 则按驱动回调报告的周期时长计时。路径与终点容差超出时返回 `TrackingError`，不修改命令；开启 `roplat` feature 后跟随器是一个节点，控制域中的 `follower?` 以 `Break` 结束会话。

工具位姿在笛卡尔位姿通道上的跟随，见 [`roplat_control`](../roplat_control/README_zh.md) 的 `CartesianTrajectoryFollower`。

## Feature

| Feature | 用途 |
|---|---|
| `roplat` | 把跟随器作为 roplat 节点，其错误经由 drive 传出。 |

默认 feature 为空。插值测试用 `plotters` 绘制曲线，它只在 Linux 与 Windows 上作为开发依赖。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
