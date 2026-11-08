# roplat_behavior

[English](README.md) | [简体中文](README_zh.md) | [Migrating from robot_behavior](../MIGRATION.md)

Typed robot capability contracts: write robot applications against **capabilities** (move to a target, read a structured state, or calculate a command each control cycle), and implement those capabilities once per hardware driver or simulator.

Part of the [roplat_behavior workspace](../README.md), whose other crates hold the algorithms built on these contracts. 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- Capability contracts: `Robot`, motion spaces, control channels and model maps; gripper, force/torque, IMU and camera contracts.
- Arm, base and legged state.
- Motion limits as data (`MotionLimits`, `with_limits`) and relative motion (`Relative<S>`).
- Contract checks for driver tests (feature `conformance`) and roplat adapters (feature `roplat`).
- Sample and geometry types (`Pose`, `JointSample`, `SpatialSample`, `LoadState`, `Twist`, …) re-exported from [`roplat_msgs`](https://github.com/Robot-Exp-Platform/roplat).

## Design: share the contract, keep the device-specific work in the driver

A joint position and a joint torque can both be `[f64; 6]`, but they mean different things. `JointSpace<6>` identifies a motion target; `JointPositionControl<6>` and `TorqueControl<6>` identify cyclic command channels. These marker types select the observation and command types and keep application calls explicit.

Capabilities are implemented separately. A driver supporting joint motion does not automatically support torque control, inverse kinematics, or native asynchronous I/O. Generic application code requests the traits it needs, rather than assuming every robot offers the same features.

The driver owns transport, timing, state acquisition, and session termination. Your control callback receives an observation and a `Duration`, then produces the next command. Controller helpers are ordinary closures; they do not create a scheduler or make a hardware timing guarantee.

| You want to… | Application API | Backend implements |
|---|---|---|
| Reach a target | `Motion::move_to::<S>` | `MoveTo<S>` |
| Send a sampled trajectory | `Motion::move_traj::<S>` | `MoveTraj<S>` |
| Compute a command every cycle | `Control::control_with::<S, _>` | `ControlWith<S>` |
| Await a complete control session | `AsyncControl::control_native_async::<S, _>` | `AsyncControlWith<S>` |
| Read a backend's native state | `Robot::read_state` | `Robot` |
| Work with arm state or a model | `Arm<N>`, `SpaceMap` and model traits | Only the supported capabilities |

## First example: a complete program without a robot

The example uses `roplat_behavior` with the console reference robot `roplat_exrobot` from the [drives](https://github.com/Robot-Exp-Platform/drives) workspace. It uses nightly Rust features, and dependency builds need a C++ toolchain (on Windows, MSVC Build Tools and the Windows SDK):

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

Run it with `cargo +nightly run`. You will see the reference robot's lifecycle and command messages, followed by `completed 3 control cycles` and shutdown. The final callback returns `true`, so its command is sent and the session finishes. The reference robot does not move or integrate a physical state.

To use a device next, replace `ExRobot` with the appropriate driver's constructor and follow that driver's connection and operating-mode setup. Matching Rust types make code reusable; they do not make robot limits, frames, timing, or controller gains interchangeable.

## Reading state

`Robot::State` is backend-specific. For portable arm-oriented code, use the driver's `Arm<N>` implementation and the structured state types:

- `JointState<N>` contains `meas`, `cmd`, and `des` views: measured feedback, accepted commands, and desired references.
- A `JointSample<N>` carries `q`, `dq`, `ddq`, `tau` and `dtau`, each read through an accessor such as `sample.q()`. `None` means that value was not supplied; it is never a fabricated zero. Producers use `JointSample::EMPTY.with_q(q)` or `sample.set_q(Some(q))`.
- `ArmState<N>` adds flange state and optional TCP, stiffness-frame, and load information. A native state and this portable view are not necessarily acquired through the same device operation.

Read the backend's state documentation for units, frames, field availability, and freshness. The presence of a value is not proof that a new sensor sample has arrived.

## Motion and control sessions

Use a motion call when the backend should execute a target or trajectory. Use a control session when your code must produce each cycle's command. `control_with` receives a callback returning `(command, done)` and blocks until the session ends.

For an exit without an algorithm command, use `control_with_flow` and return a `ControlStep<Command>`:

| Callback result | Meaning |
|---|---|
| `ControlFlow::Continue((command, false))` | Send the command; continue. |
| `ControlFlow::Continue((command, true))` | Send this final command; complete normally. |
| `ControlFlow::Break(())` | Send no algorithm command for this cycle; perform the driver's session termination. |

Termination depends on the device protocol. `Break` does not prescribe a zero command or represent an emergency stop. After a session that returns `Ok`, an arm holds still without drifting and takes the next motion call without `enable`; the driver documents the controller mode it leaves.

There are two different uses of async:

- `control_with_async` / `control_with_flow_async` accept async **callbacks inside a blocking session**. They do not return a session future.
- `control_native_async(&mut callback)` returns the **whole session future**, including its I/O and termination. It requires an `AsyncControlWith<S>` implementation and the runtime specified by the driver. Dropping that future is not cooperative session shutdown.

## Features and Roplat integration

Default features are empty. The Rust motion, control, state, and model interfaces work without Roplat.

| Feature | Purpose |
|---|---|
| `roplat` | `ControlRhythm`, `AsyncControlRhythm`, and motion/model node adapters. |
| `conformance` | Contract checks for driver tests. |

Python and C++ bindings are [`roplat_behavior_ffi`](../roplat_behavior_ffi/README.md) (features `py`, `cxx`); `SimulationRhythm` is [`roplat_world`](../roplat_world/README.md) (feature `roplat`).

For a Roplat application, add `features = ["roplat"]` and depend on the matching `roplat = "0.5"`. Build the application graph with `#[roplat::system]`; choose `ControlRhythm` for a blocking backend or `AsyncControlRhythm` for a native async backend. The adapters connect the same observations and commands to a graph; they do not add capabilities missing from the device driver.

The robot stays owned by its creating scope. Nodes and rhythms receive a mutable reference through their separate `Resources` channel; business Input and Output never need to carry it.

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

The creating scope manages node and robot lifecycle. `MotionNode` declares `Resources = (R,)`, `Input = S::Target`, and `Output = RobotResult<()>`; both control rhythms declare `Resources = (R,)` with `Input = Output = ()`. The borrow lasts for one process or complete drive, including failure and stop paths. The `?` on `reach.output` explicitly promotes its business error to a System failure. See the [adapter implementation](src/roplat) and [complete System examples](tests/control_rhythm.rs).

## Writing a driver

Driver authors can import `roplat_behavior::driver::*`; applications can import `roplat_behavior::behavior::*`. Implement the device's lifecycle and state in `Robot`, then each supported motion/control/model capability. Default lifecycle hooks are no-ops and some unimplemented operations return errors; inheriting a default is not an implementation of device behavior.

The [example checklist](../EXAMPLES.md) lists the examples a driver should provide, and feature `conformance` checks a driver's contract implementations in its own tests.

## Going further

- [Motion and trajectory APIs](src/robot/motion.rs), [control contracts](src/robot/control.rs) and [native async contracts](src/robot/async_control.rs).
- [State types](src/robot/state.rs) and [model mappings](src/robot/model.rs).
- [Kinematics](../roplat_kinematics/README.md), [controllers](../roplat_control/README.md) and [trajectories](../roplat_trajectory/README.md).
- [Executable contract examples](tests/control_flow.rs), [async examples](tests/async_control.rs), and [foreign-interface examples](../roplat_behavior_ffi/examples).
- [What changed from robot_behavior 0.8](../MIGRATION.md).

## License

Apache-2.0; see [LICENSE](../LICENSE).
