# roplat_trajectory

[English](README.md) | [简体中文](README_zh.md)

Trajectory generation on `roplat_msgs` data: interpolation, time scaling, collision-free joint paths and their smoothing, straight tip paths, and following a timed joint trajectory.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- Joint and Cartesian interpolation: linear, trapezoid, S-curve and fourth-order (`path_generate`).
- COPP time scaling on limits as data: `JointLimits`, `CartesianLimits`, `PathBounds` (`time_scaling`).
- Collision-free joint-space paths (`path_planning`): `plan_joint_path` finds waypoints by RRT-Connect, with a collision check the caller supplies. It checks configurations `resolution` apart; `resolution_for_travel` picks the resolution from the scene's scale, so that no point of the robot moves farther than a given travel (at most the thinnest obstacle) between two checks. The default 0.02 rad comes from no scene: it lets an arm with two 0.6 m links step over a 2 cm wall.
- Simplifying and smoothing them (`path_smoothing`): `simplify_path` and `smooth_path` drop the waypoints the path does not need and round its corners, checking every new segment, as OMPL's path simplifier does.
- `plan_joint_motion`, `simplify_motion` and `smooth_motion` do the same with a check of each whole straight segment in place of samples.
- Straight tip paths (`cartesian_path`): `plan_cartesian_path` moves the tip in straight lines through given poses, solving each step with the caller's seeded inverse kinematics. It stops before an unreachable pose, a joint jump or an invalid segment; `plan_cartesian_motion` checks each step whole.
- Following a timed joint trajectory on a position channel (`follow`): `JointTrajectoryFollower`.

[`roplat_scene`](../roplat_scene/README.md) supplies the collision checks, and [`roplat_kinematics`](../roplat_kinematics/README.md) the inverse kinematics.

## Limits as data

The crate takes limits as `roplat_msgs` data and the control period as a `Duration`, never a robot type, so it depends only on `roplat_msgs` and `copp`. An unbounded entry is `f64::INFINITY`, and the planners need finite velocity and acceleration bounds.

## Following a joint trajectory

`JointTrajectoryFollower` plays timed `JointTrajectoryPoint`s on the joint-position channel. It reads time from a `roplat_msgs::Clock`, not from the cycle `Duration`, so the same follower runs on wall or simulation time; `JointTrajectoryFollower::cycle_timed` with `step_by(dt, …)` times it by the cycle durations a driver's callback reports. Path and goal tolerances refuse with a `TrackingError` rather than altering the command; with the `roplat` feature the follower is a node, and `follower?` in a control domain ends the session with `Break`.

For tool poses on the Cartesian pose channel, see `CartesianTrajectoryFollower` in [`roplat_control`](../roplat_control/README.md).

## Features

| Feature | Purpose |
|---|---|
| `roplat` | The follower as a roplat node, and its error carried through a drive. |

Default features are empty. The interpolation tests plot their curves with `plotters`, a dev-dependency on Linux and Windows only.

## License

Apache-2.0; see [LICENSE](../LICENSE).
