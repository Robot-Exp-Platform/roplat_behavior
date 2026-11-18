# roplat_behavior

[English](README.md) | [简体中文](README_zh.md) | [Migrating from robot_behavior](MIGRATION.md)

> [!NOTE]
> **0.9 is not on crates.io yet.** These crates continue `robot_behavior` 0.8 under new names and depend on `roplat_msgs` from the roplat core, which is also unpublished. Until both are released, build from source (see [Working from source](#working-from-source)). The last `robot_behavior` release, 0.8.1, only points here.

`roplat_behavior` is a Rust interface library for writing robot applications against **capabilities**: move to a target, read a structured state, or calculate a command each control cycle. A hardware driver or simulator implements those capabilities; this repository supplies their shared contracts and the algorithms built on them. Each crate has its own README.

**Contracts.** Drivers and simulators implement these.

| Crate | Contents |
|---|---|
| [`roplat_behavior`](roplat_behavior/README.md) | Capability contracts (`Robot`, motion spaces, control channels, model maps; gripper, force/torque, IMU and camera), robot state, motion limits as data, roplat adapters. |
| [`roplat_behavior_ffi`](roplat_behavior_ffi/README.md) | Python and C++ bindings of the contracts for driver packages. |
| [`roplat_world`](roplat_world/README.md) | Simulator backends and model selection, physics engines, renderers, world entities, `SimulationRhythm`; `KinematicArm`, a simulated arm with a panel. |

**Running a robot.**

| Crate | Contents |
|---|---|
| [`roplat_description`](roplat_description/README.md) | One robot model from URDF: tree, inertia, geometry, limits, chain kinematics, inverse dynamics. |
| [`roplat_kinematics`](roplat_kinematics/README.md) | DH and URDF kinematics and inverse kinematics on the contract's model maps; URDF dynamics. |
| [`roplat_control`](roplat_control/README.md) | PID, impedance, computed torque, teleoperation, Cartesian servoing and Cartesian trajectory following. |
| [`roplat_trajectory`](roplat_trajectory/README.md) | Interpolation, COPP time scaling, collision-free joint paths and their smoothing, straight tip paths, joint trajectory following. |
| [`roplat_safety`](roplat_safety/README.md) | Gates that refuse commands: limits, rates, freshness, a watchdog and a stop latch. |
| [`roplat_diagnostics`](roplat_diagnostics/README.md) | Rate and latency probes, health checks and status aggregation. |

**Manipulation.** These build on `roplat_description` and do not depend on the contracts.

| Crate | Contents |
|---|---|
| [`roplat_frames`](roplat_frames/README.md) | Frame tree with time-buffered transforms, hand-eye calibration, chessboard detection, lens distortion. |
| [`roplat_scene`](roplat_scene/README.md) | Collision checking for planning: the robot, world and attached objects, whole-motion checks, sensed points and occupancy. |
| [`roplat_grasp`](roplat_grasp/README.md) | Grasp candidates for boxes, cylinders and spheres, pick and place planning, grasp checks. |
| [`roplat_perception`](roplat_perception/README.md) | Point-cloud filtering, tabletop segmentation, shape fitting, ICP, normals and region growing. |

Sample and geometry types (`Pose`, `JointSample`, `SpatialSample`, `LoadState`, `Twist`, …) come from [`roplat_msgs`](https://github.com/Robot-Exp-Platform/roplat) and are re-exported by `roplat_behavior`. They have one representation each and a fixed native layout, so the same values cross process and language boundaries. None of these crates depends on the roplat runtime unless you enable the `roplat` feature.

Use them when you want to reuse application or controller code across compatible backends, or implement a new driver in the same vocabulary. To connect to a robot, also choose its driver. To simulate physics, choose a physics backend such as [RsBullet](https://github.com/Robot-Exp-Platform/rsbullet).

## Where to start

- **An application or a driver:** the [`roplat_behavior` README](roplat_behavior/README.md) has a complete program without a robot, the state types, control sessions and the roplat integration. The [example checklist](EXAMPLES.md) lists the examples a driver should provide.
- **Picking objects a camera sees:** [`roplat_perception`](roplat_perception/README.md) finds them, [`roplat_scene`](roplat_scene/README.md) keeps them as obstacles, and [`roplat_grasp`](roplat_grasp/README.md) plans the pick.
- **Coming from robot_behavior 0.8:** [what changed](MIGRATION.md).

## Working from source

This is a Cargo workspace of the crates above; each crate declares its own version. Until `roplat_msgs` is published, the root manifest patches `roplat` and `roplat_msgs` to a sibling checkout of the roplat core, so clone both side by side:

```text
Robot-Exp-Platform/
├── roplat/
└── roplat_behavior/
```

`cargo +nightly test --workspace` runs the default tests; add `--features roplat_behavior/roplat` for the adapter tests. The tests provide finite mock examples of contracts, not evidence of hardware timing. For driver integration, see [drives](https://github.com/Robot-Exp-Platform/drives).

To check a change, `python scripts/affected-tests.py` runs `cargo fmt --check`, then tests and clippy for the crates that differ from `origin/main` and every workspace crate that depends on them; `--list` prints the selection only, and `--base` compares with another revision. A change to the root manifest, the lock file or the toolchain selects the whole workspace. Before a release, run the whole workspace anyway, since features are unified only across the selected crates.

## License

Apache-2.0; see [LICENSE](LICENSE).
