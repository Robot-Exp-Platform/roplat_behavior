# roplat_control

[English](README.md) | [简体中文](README_zh.md)

[`roplat_behavior`](../roplat_behavior/README_zh.md) 机器人的控制器，以普通闭包的形式交给 `control_with`。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- 关节 PD/PID 与底盘速度 PID（`pid`）。
- 关节与笛卡尔阻抗（`impedance`）。
- 重力补偿与计算力矩（`dynamics`）。
- 跟踪控制器所用的目标源（`target`）。
- 操作员遥操作（`teleop`）：`cartesian_teleop_control`，带死区、使能按键、输入超时与加速度限制；关节空间的对应物 `joint_jog_control`（`JogMapping`、`JogInput`）让每个关节不超过其加速度限制，并在位置限位前减速，停在限位内 `limit_margin` 处。
- 笛卡尔伺服（`servo`）：`CartesianServo` 把末端速度旋量转为关节速度或位置命令；只在趋近奇异时减速，到关节限位即停，不超关节速度上限，可加接近障碍的减速系数。
- 在笛卡尔位姿通道上跟随定时的工具位姿轨迹（`cartesian_follow`）：`CartesianTrajectoryFollower`、`cartesian_line`、`cartesian_arc`，以及按 `CartesianLimits` 定时的 `cartesian_line_within`、`cartesian_arc_within`。
- 实时线程优先级（`set_realtime_priority`、`is_hardware_realtime`）。

## 使用控制器

控制器返回可直接交给 `control_with` 的普通闭包。增益与模型数据需按后端选择；它们不提供时序保证。力矩控制器接受一个 `DynamicsModel`，例如 [`roplat_kinematics`](../roplat_kinematics/README_zh.md) 的 `UrdfDynamics`。

## 笛卡尔轨迹

`CartesianTrajectoryFollower` 在 `CartesianPoseControl` 上播放定时的 `CartesianWaypoint`：

- 相邻路点之间位置走直线、姿态做球面插值。
- 路径与终点容差以米和弧度计，与测得的法兰位姿比较，超出时返回错误而不修改命令。
- `cartesian_line` 生成按控制周期采样的直线最小加加速度运动，`cartesian_arc` 生成经过一个途经点的圆弧运动（相当于 Pilz `CIRC` 的 interim 点方式）。
- `cartesian_line_within` 与 `cartesian_arc_within` 在线速度、角速度及其加速度限制内取最短时长，与 Pilz 的 `LIN`、`CIRC` 相同。

开启 `roplat` feature 后，它也是一个节点，与 [`roplat_trajectory`](../roplat_trajectory/README_zh.md) 的关节轨迹跟随相同。

## Feature

| Feature | 用途 |
|---|---|
| `roplat` | 把笛卡尔轨迹跟随作为 roplat 节点。 |

默认 feature 为空。实时优先级在 Linux 上使用 `libc`，在 Windows 上使用 `winapi`。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
