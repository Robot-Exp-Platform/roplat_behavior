# roplat_behavior

[English](README.md) | [简体中文](README_zh.md) | [从 robot_behavior 迁移](MIGRATION_zh.md)

> [!NOTE]
> **0.9 尚未发布到 crates.io。** 这些包以新名字延续 `robot_behavior` 0.8，并依赖 roplat 核心中同样尚未发布的 `roplat_msgs`。两者发布之前请从源码构建（见[从源码开发](#从源码开发)）。`robot_behavior` 的最后一个版本 0.8.1 只用于指向这里。

`roplat_behavior` 是一个 Rust 机器人接口库，让应用围绕**能力**编写：到达目标、读取结构化状态，或在每个控制周期计算下一条指令。硬件驱动或仿真后端实现这些能力，本仓库提供共同的接口契约以及建立在契约之上的算法。每个包都有自己的 README。

**契约。** 由驱动与仿真后端实现。

| 包 | 内容 |
|---|---|
| [`roplat_behavior`](roplat_behavior/README_zh.md) | 能力契约（`Robot`、运动空间、控制通道、模型映射；夹爪、力/力矩、IMU 与相机）、机器人状态、以数据表达的运动限值、roplat 适配。 |
| [`roplat_behavior_ffi`](roplat_behavior_ffi/README_zh.md) | 契约的 Python 与 C++ 绑定，供驱动包使用。 |
| [`roplat_world`](roplat_world/README_zh.md) | 仿真后端与模型选择、物理引擎、渲染器、世界实体、`SimulationRhythm`；带面板的仿真臂 `KinematicArm`。 |

**运行机器人。**

| 包 | 内容 |
|---|---|
| [`roplat_description`](roplat_description/README_zh.md) | 从 URDF 读出的同一份机器人模型：运动学树、惯量、几何、限位、链式运动学、逆动力学。 |
| [`roplat_kinematics`](roplat_kinematics/README_zh.md) | 接到契约模型映射上的 DH 与 URDF 运动学与逆运动学；URDF 动力学。 |
| [`roplat_control`](roplat_control/README_zh.md) | PID、阻抗、计算力矩、遥操作、笛卡尔伺服与笛卡尔轨迹跟随。 |
| [`roplat_trajectory`](roplat_trajectory/README_zh.md) | 插值、COPP 时间缩放、无碰撞关节路径及其平滑、末端直线路径、关节轨迹跟随。 |
| [`roplat_safety`](roplat_safety/README_zh.md) | 拒绝命令的安全门：限位、变化率、新鲜度、看门狗与停止锁存。 |
| [`roplat_diagnostics`](roplat_diagnostics/README_zh.md) | 频率与延迟探针、健康检查与状态汇总。 |

**操作。** 这些包建立在 `roplat_description` 之上，不依赖契约。

| 包 | 内容 |
|---|---|
| [`roplat_frames`](roplat_frames/README_zh.md) | 带时间缓冲变换的坐标树、手眼标定、棋盘格检测、镜头畸变。 |
| [`roplat_scene`](roplat_scene/README_zh.md) | 规划用的碰撞检查：机器人、世界中与附着的物体、整段运动检查、传感点与占据栅格。 |
| [`roplat_grasp`](roplat_grasp/README_zh.md) | 长方体、圆柱、球体的抓取候选，取放规划，抓取确认。 |
| [`roplat_perception`](roplat_perception/README_zh.md) | 点云滤波、桌面分割、形状拟合、ICP、法向与区域生长。 |

采样与几何类型（`Pose`、`JointSample`、`SpatialSample`、`LoadState`、`Twist` 等）来自 roplat 核心的 [`roplat_msgs`](https://github.com/Robot-Exp-Platform/roplat)，由 `roplat_behavior` 重新导出。每个类型只有一种表示和固定的原生布局，同一份数据可以跨进程、跨语言传递。除非打开 `roplat` feature，这些包都不依赖 roplat 运行时。

如果你希望在兼容的后端之间复用应用与控制器，或为新驱动建立一致的接口，可以使用它们。连接真机还需要选择对应驱动；物理仿真需要 [RsBullet](https://github.com/Robot-Exp-Platform/rsbullet) 等后端。

## 从哪里开始

- **编写应用或驱动：**[`roplat_behavior` 的 README](roplat_behavior/README_zh.md) 有一个不连接机器人的完整程序，以及状态类型、控制会话与 roplat 集成。[示例清单](EXAMPLES_zh.md)列出驱动应提供的示例。
- **抓取相机看到的物体：**[`roplat_perception`](roplat_perception/README_zh.md) 找到它们，[`roplat_scene`](roplat_scene/README_zh.md) 把它们作为障碍，[`roplat_grasp`](roplat_grasp/README_zh.md) 规划抓取。
- **从 robot_behavior 0.8 迁移：**[变化说明](MIGRATION_zh.md)。

## 从源码开发

本仓库是上述各包组成的 Cargo 工作区，每个包各自声明版本。`roplat_msgs` 发布之前，根清单把 `roplat` 与 `roplat_msgs` 指向同级的 roplat 核心检出，因此需要并排克隆：

```text
Robot-Exp-Platform/
├── roplat/
└── roplat_behavior/
```

`cargo +nightly test --workspace` 运行默认测试；加 `--features roplat_behavior/roplat` 运行适配器测试。这些测试是契约的有限模拟示例，不代表硬件时序。驱动集成见 [drives](https://github.com/Robot-Exp-Platform/drives)。

检查一项改动时，`python scripts/affected-tests.py` 先运行 `cargo fmt --check`，再对与 `origin/main` 不同的包以及工作区中依赖它们的每个包运行测试与 clippy；`--list` 只打印选择结果，`--base` 改为与另一个版本比较。改动根清单、锁文件或工具链时选中整个工作区。发布之前仍要跑整个工作区，因为 feature 只在选中的包之间合并。

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
