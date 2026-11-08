# roplat_behavior

[English](README.md) | [简体中文](README_zh.md) | [从 robot_behavior 迁移](../MIGRATION_zh.md)

带类型的机器人能力契约：应用围绕**能力**编写（到达目标、读取结构化状态，或在每个控制周期计算下一条指令），每个硬件驱动或仿真后端实现一次这些能力。

属于 [roplat_behavior 工作区](../README_zh.md)，工作区中的其他包是建立在这些契约之上的算法。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- 能力契约：`Robot`、运动空间、控制通道与模型映射；夹爪、力/力矩、IMU 与相机契约。
- 机械臂、底盘与腿足状态。
- 以数据表达的运动限值（`MotionLimits`、`with_limits`）与相对运动（`Relative<S>`）。
- 供驱动测试使用的契约检查（feature `conformance`）与 roplat 适配（feature `roplat`）。
- 从 [`roplat_msgs`](https://github.com/Robot-Exp-Platform/roplat) 重新导出的采样与几何类型（`Pose`、`JointSample`、`SpatialSample`、`LoadState`、`Twist` 等）。

## 设计理念：统一接口，把设备差异留在驱动中

关节位置与关节力矩都可能是 `[f64; 6]`，但含义不同。`JointSpace<6>` 表示运动目标空间，`JointPositionControl<6>` 与 `TorqueControl<6>` 表示周期控制通道。这些类型选择对应的观测和指令类型，让应用明确表达操作意图。

各项能力独立实现。支持关节运动的驱动，不会自动获得力矩控制、逆运动学或原生异步 I/O。泛型应用声明需要的 trait，不必假定所有机器人都提供相同功能。

驱动负责通信、周期组织、状态采集和会话收尾。控制闭包接收观测值与 `Duration`，计算下一条指令。控制器辅助函数返回普通闭包，不自行创建调度器，也不提供硬件实时性保证。

| 想完成的操作 | 应用入口 | 后端实现 |
|---|---|---|
| 到达一个目标 | `Motion::move_to::<S>` | `MoveTo<S>` |
| 发送采样轨迹 | `Motion::move_traj::<S>` | `MoveTraj<S>` |
| 每周期计算指令 | `Control::control_with::<S, _>` | `ControlWith<S>` |
| 等待完整异步控制会话 | `AsyncControl::control_native_async::<S, _>` | `AsyncControlWith<S>` |
| 读取后端原生状态 | `Robot::read_state` | `Robot` |
| 使用机械臂状态或模型 | `Arm<N>`、`SpaceMap` 等模型 trait | 后端实际支持的能力 |

## 第一个程序：不连接机器人

示例使用 `roplat_behavior` 和 [drives](https://github.com/Robot-Exp-Platform/drives) 工作区中的控制台参考机器人 `roplat_exrobot`。库使用 nightly Rust 特性，依赖构建需要 C++ 工具链（Windows 使用 MSVC Build Tools 和 Windows SDK）：

```toml
[dependencies]
roplat_behavior = "0.9.0"
roplat_exrobot = "0.5.0"
```

```rust
use roplat_behavior::{
    Control, JointPositionControl, JointSpace, Motion, Robot, RobotResult,
};
use roplat_exrobot::ExRobot;

fn main() -> RobotResult<()> {
    let mut arm = ExRobot::<6>::new();
    arm.init()?;
    arm.move_to::<JointSpace<6>>([0.1; 6])?;

    let mut cycles = 0;
    arm.control_with::<JointPositionControl<6>, _>(|_state, _dt| {
        cycles += 1;
        ([0.1; 6], cycles == 3)
    })?;

    println!("completed {cycles} control cycles");
    arm.shutdown()
}
```

用 `cargo +nightly run` 运行。程序打印参考机器人的生命周期和指令，输出 `completed 3 control cycles` 后关闭。最后一次回调返回 `true`，这条指令发送后会话结束。参考机器人不会运动，也不积分物理状态。

接下来连接设备时，把 `ExRobot` 换成对应驱动的构造方式，并按该驱动的说明配置连接和运行模式。相同的 Rust 类型让代码可以复用，但不会让限位、坐标系、周期或控制增益变得通用。

## 读取状态

`Robot::State` 由后端决定。编写通用机械臂代码时，使用驱动的 `Arm<N>` 实现和结构化状态类型：

- `JointState<N>` 包含 `meas`、`cmd`、`des` 三个视图：测量反馈、已接受指令和期望参考。
- `JointSample<N>` 带有 `q`、`dq`、`ddq`、`tau`、`dtau`，通过 `sample.q()` 这类访问器读取。`None` 表示该值没有提供，绝不是伪造的零值。生产者使用 `JointSample::EMPTY.with_q(q)` 或 `sample.set_q(Some(q))`。
- `ArmState<N>` 增加法兰状态，以及可选的 TCP、刚度坐标系和负载信息。原生状态与这个通用视图不一定来自同一次设备读取。

单位、坐标系、字段是否可用以及数据新鲜度，以后端的状态说明为准。字段有值并不证明刚收到了新的传感器样本。

## 运动与控制会话

需要后端执行一个目标或一段轨迹时，使用运动调用；需要由你的代码计算每个周期的指令时，使用控制会话。`control_with` 的回调返回 `(command, done)`，调用会阻塞到会话结束。

如果需要在没有算法指令的情况下退出，使用 `control_with_flow` 并返回 `ControlStep<Command>`：

| 回调结果 | 含义 |
|---|---|
| `ControlFlow::Continue((command, false))` | 发送指令，继续。 |
| `ControlFlow::Continue((command, true))` | 发送这条最终指令，正常结束。 |
| `ControlFlow::Break(())` | 本周期不发送算法指令，执行驱动的会话收尾。 |

收尾方式取决于设备协议。`Break` 不规定发送零指令，也不代表急停。会话返回 `Ok` 之后，机械臂静止保持、不漂移，下一次运动调用无需 `enable`；驱动写明会话结束后控制器所处的模式。

async 有两种不同用法：

- `control_with_async` / `control_with_flow_async` 在**阻塞会话中使用 async 回调**，不返回会话 future。
- `control_native_async(&mut callback)` 返回**整个会话的 future**，包括 I/O 与收尾。它要求后端实现 `AsyncControlWith<S>`，并使用驱动规定的运行时。丢弃这个 future 不等于合作式关闭会话。

## Feature 与 Roplat 集成

默认 feature 为空。Rust 的运动、控制、状态和模型接口不依赖 Roplat。

| Feature | 用途 |
|---|---|
| `roplat` | `ControlRhythm`、`AsyncControlRhythm`，以及运动/模型节点适配器。 |
| `conformance` | 供驱动测试使用的契约检查。 |

Python 与 C++ 绑定在 [`roplat_behavior_ffi`](../roplat_behavior_ffi/README_zh.md)（特性 `py`、`cxx`）；`SimulationRhythm` 在 [`roplat_world`](../roplat_world/README_zh.md)（特性 `roplat`）。

编写 Roplat 应用时，为依赖添加 `features = ["roplat"]`，并依赖匹配的 `roplat = "0.5"`。用 `#[roplat::system]` 组织应用图；阻塞式后端使用 `ControlRhythm`，原生异步后端使用 `AsyncControlRhythm`。适配器把相同的观测和指令接入图中，不会为驱动补出缺失的能力。

机器人始终归创建层所有。节点和节律通过独立的 `Resources` 通道获得可变引用，业务 Input 和 Output 不需要携带机器人。

```rust
#[roplat::system]
async fn session(mut robot: MyRobot, mut law: MyLaw) -> RoplatResult<MyRobot> {
    let mut control = ControlRhythm::<MyRobot, JointPositionControl<7>>::new();
    let mut reach = MotionNode::<MyRobot, JointSpace<7>>::new();
    let mut goal = Goal::new();

    &mut robot >> reach.resources.0;
    goal >> reach;
    reach.output?;
    &mut robot >> control.resources.0;
    control >> |state| state >> law;

    Ok(robot)
}
```

创建层管理节点和机器人的生命周期。`MotionNode` 声明 `Resources = (R,)`、`Input = S::Target`、`Output = RobotResult<()>`；两种控制节律都声明 `Resources = (R,)`，`Input = Output = ()`。借用覆盖一次 process 或完整的 drive，包括失败与停止路径。`reach.output?` 显式把业务错误提升为 System 失败。接入图时参见[适配器实现](src/roplat)和[完整 System 示例](tests/control_rhythm.rs)。

## 编写驱动

驱动作者可以导入 `roplat_behavior::driver::*`，应用可以导入 `roplat_behavior::behavior::*`。先在 `Robot` 中实现设备生命周期与状态，再分别实现支持的运动、控制和模型能力。默认生命周期钩子是空操作，部分未实现操作会返回错误；继承默认实现不等于实现了设备行为。

[示例清单](../EXAMPLES_zh.md)列出驱动应提供的示例；feature `conformance` 在驱动自己的测试中检查其契约实现。

## 继续阅读

- [运动与轨迹接口](src/robot/motion.rs)、[控制契约](src/robot/control.rs)、[原生异步契约](src/robot/async_control.rs)。
- [状态类型](src/robot/state.rs)与[模型映射](src/robot/model.rs)。
- [运动学](../roplat_kinematics/README_zh.md)、[控制器](../roplat_control/README_zh.md)、[轨迹](../roplat_trajectory/README_zh.md)。
- [契约示例](tests/control_flow.rs)、[异步示例](tests/async_control.rs)和[外部接口示例](../roplat_behavior_ffi/examples)。
- [相对 robot_behavior 0.8 的变化](../MIGRATION_zh.md)。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
