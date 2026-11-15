# roplat_grasp

[English](README.md) | [简体中文](README_zh.md)

Grasping for a parallel gripper: where to grasp an object, how to get there and away with it, and where to put it down. It answers what MoveIt Grasps and MoveIt's pick and place stages answer.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- `grasps` lists `Grasp` candidates for a box, cylinder or sphere: every tool pose from which the fingers close across the object at a width the gripper opens to, best first (approach from above first).
- `plan_pick` tries the candidates in order against a [`PlanningScene`](../roplat_scene/README.md) and returns the first that works as three joint paths: a collision-free path to the pre-grasp pose, a straight approach to the grasp, and a straight lift with the object attached. For each candidate that fails it says why. The free path is checked as whole motions by `PlanningScene::motion_is_free`, so no obstacle is stepped over however thin (a check of configurations `PlannerOptions::resolution` apart would let a link pass a plate thinner than what it sweeps between two of them; the resolution is not used); the straight approach and lift are checked at their Cartesian steps (`CartesianPathOptions`). `plan_pick_motion` plans the same with every segment checked whole. The whole check is as sound as `roplat_scene`'s distances: between two boxes it takes parry 0.25's cuboid-cuboid distance, which can overestimate a gap (1.1 mm measured as 2.3 cm for a thin box and a thin plate), so a box link can still be certified through a thin box; pairs with another shape are measured by GJK and are not affected.
- `plan_place` (and `plan_place_motion`) puts the held object down: a path carrying it to above the first placement that works, a straight descent onto it, and a straight retreat once released.
- `GraspCheck` judges whether the closed fingers hold the object from the opening any gripper reports: held, empty, an unexpected width, or disputed by the driver's own flag.
- `carried_mass` confirms the load after the lift from a wrist force sensor.

Closing the fingers is not planned: the caller commands its `Gripper` between the approach and the lift. Times are not assigned either; the paths are waypoints for [`roplat_trajectory`](../roplat_trajectory/README.md)'s time scaling.

## Tool frame

A grasp is a pose of the tool centre point (TCP) in the world: `z` is the approach direction (into the object), `y` the closing direction of the fingers, `x = y × z`. `EndEffector` in `roplat_scene` places the TCP on the arm's link with the same convention.

## Limits

Candidates come from the shape alone: friction, mass and the object's neighbours are not considered, so a candidate can still be blocked or slip; the planner rejects the blocked ones. Meshes give no candidates. The shapes can come from [`roplat_perception`](../roplat_perception/README.md)'s fits.

## License

Apache-2.0; see [LICENSE](../LICENSE).
