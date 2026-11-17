# roplat_world

[English](README.md) | [简体中文](README_zh.md)

机器人所处世界的契约：加载机器人模型的仿真后端、可步进的物理引擎、渲染器，以及放进世界的实体。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- 仿真后端与模型选择：`SimulationBackend`、`select_simulation_model`。
- 物理引擎（`PhysicsEngine`）与渲染器（`Renderer`）。
- 世界实体：`Entity`、`AddRobot`、`AddCollision`、`AddVisual`。
- 共享资产目录（`assets_dir`）。
- `SimulationRhythm`，把引擎作为 roplat 节律步进（feature `roplat`）。
- `KinematicArm`：无动力学的仿真臂，它的 `Panel` 能从另一个线程停止运动，`Hardware` 扮演它的急停按钮与保护停止。它是面板契约的参照实现，通过 `roplat_behavior::conformance::check_panel`。

机器人类型通过 `roplat_behavior::RobotDescription` 声明自己的模型文件，本包为后端从中选择。

## 组合机器人模型与仿真后端

同一种设备可以在 `RobotDescription::SIMULATION_MODELS` 中提供多个模型。
加载器实现 `SimulationBackend`，声明后端标识与实际接受的格式，再通过
`select_robot_model::<R>()` 选择模型。只声明旧 `URDF` 常量的设备仍可由接受
URDF 的后端使用。

```rust
use roplat_behavior::{ModelFormat, RobotDescription, SimulationModel};

struct ArmModels;
impl RobotDescription for ArmModels {
    const SIMULATION_MODELS: &'static [SimulationModel] = &[
        SimulationModel::new(ModelFormat::Urdf, "arm/robot.urdf"),
        SimulationModel::new(ModelFormat::Mjcf, "arm/scene.xml").for_backend("mujoco"),
    ];
}
```

选择顺序是：指定后端的模型、通用模型、旧 `URDF`；每层先按后端的格式优先级，
再按设备的声明顺序。`SelectedSimulationModel` 记录选择来源；不匹配时返回
带有后端和接受格式的 `ModelSelectionError`。路径遵循后端的资源搜索规则。
选中的文件加载失败时应报告失败，不应静默换用另一份模型并改变物理条件。

这层接口只选择模型，不自动转换文件或实现仿真器。后端仍需验证模型，并提供
实际支持的关节、控制、传感器和场景能力。操作系统与构建条件由后端安装契约
说明；接受 MJCF 不表示当前电脑已安装或支持 MuJoCo。仿真步进继续使用独立
的 `PhysicsEngine`，不改变真机的运动与控制语义。
[选择测试](tests/simulation_models.rs) 覆盖设备与后端匹配及不匹配的组合，
不启动仿真器。

## 在 roplat 图中步进仿真

可选的 `SimulationRhythm<E>` 为任意兼容的
`PhysicsEngine` 提供有限步数与墙钟节律，并在真实 `#[roplat::system]` 图中运行。
`logical(engine).with_max_steps(n)` 按引擎速度推进；`new(engine, period)`
只限制墙钟节奏，不改变物理步长。每次驱动重新计算步数预算，但不会重置世界。
引擎报错保留错误来源、停止后续图执行；合作退出可打断等待，仍会等待正在执行的
子域归还其节点。状态刷新、命令顺序及物理语义由具体后端负责。

这层适配不依赖 Bullet 或 MuJoCo，Tokio 仅随 `roplat` feature 启用。
它不会代替后端调用消耗所有权的 `shutdown`；后端可使用 RAII 清理，或由应用
通过 `into_engine()` 取回引擎并显式关闭。见[节律测试](tests/simulation_rhythm.rs)。

## Feature

| Feature | 用途 |
|---|---|
| `roplat` | `SimulationRhythm`；引入 `roplat`、`futures` 与 `tokio`。 |

默认 feature 为空。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
