# roplat_control

[English](README.md) | [简体中文](README_zh.md)

Controllers for [`roplat_behavior`](../roplat_behavior/README.md) robots, returned as ordinary closures for `control_with`.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- Joint PD/PID and base-velocity PID (`pid`).
- Joint and Cartesian impedance (`impedance`).
- Gravity compensation and computed torque (`dynamics`).
- The target sources tracking controllers take (`target`).
- Operator teleoperation (`teleop`): `cartesian_teleop_control` with a deadzone, a dead-man button, an input timeout and an acceleration limit, and its joint-space counterpart `joint_jog_control` (`JogMapping`, `JogInput`), which keeps each joint within its acceleration limit and brakes before its position limits so that it stops `limit_margin` short of them.
- Cartesian servoing (`servo`): `CartesianServo` turns a tip twist into joint velocity or position commands. It slows only when approaching a singularity, stops at joint limits, stays within joint velocity limits, and takes an optional proximity factor.
- Following a timed tool-pose trajectory on the Cartesian pose channel (`cartesian_follow`): `CartesianTrajectoryFollower`, `cartesian_line`, `cartesian_arc`, and `cartesian_line_within` / `cartesian_arc_within` timed by `CartesianLimits`.
- Real-time thread priority (`set_realtime_priority`, `is_hardware_realtime`).

## Using a controller

The controllers return ordinary closures for `control_with`. Choose gains and model data for your backend; they do not make timing guarantees. The torque controllers take a `DynamicsModel`, such as `UrdfDynamics` from [`roplat_kinematics`](../roplat_kinematics/README.md).

## Cartesian trajectories

`CartesianTrajectoryFollower` plays timed `CartesianWaypoint`s on `CartesianPoseControl`:

- Straight-line position and spherical orientation interpolation between waypoints.
- Path and goal tolerances in metres and radians against the measured flange pose; beyond them it refuses with an error rather than altering the command.
- `cartesian_line` makes a straight minimum-jerk move sampled at the control period, and `cartesian_arc` one along the circle through a via point (Pilz `CIRC` with an interim point).
- `cartesian_line_within` and `cartesian_arc_within` take the shortest duration within linear and angular speed and acceleration limits, as Pilz's `LIN` and `CIRC` do.

With the `roplat` feature the follower is a node too, like the joint follower in [`roplat_trajectory`](../roplat_trajectory/README.md).

## Features

| Feature | Purpose |
|---|---|
| `roplat` | The Cartesian trajectory follower as a roplat node. |

Default features are empty. Real-time priority uses `libc` on Linux and `winapi` on Windows.

## License

Apache-2.0; see [LICENSE](../LICENSE).
