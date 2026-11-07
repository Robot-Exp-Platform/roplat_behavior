# roplat_scene

[English](README.md) | [简体中文](README_zh.md)

Collision checking for motion planning: a robot from its URDF, the objects around it and the ones it holds, and the obstacles its sensors see.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

The scene:

- `PlanningScene` holds a robot read from its URDF (meshes as convex hulls), objects in the world, objects attached to links, allowed contacts (adjacent links from the start) and a padding.
- `add_object` requires a new name. `set_object` installs new geometry and pose for a world object, keeping its old geometry on error. `move_object` updates only the pose.
- Through `CollisionCheck` it answers the validity check `plan_joint_path` in [`roplat_trajectory`](../roplat_trajectory/README.md) asks of its caller (`scene.is_free(q)`); `contacts` lists the contacts and `clearance` gives the nearest pair.
- `set_pair_padding` lets a pair keep its own clearance in place of the scene's padding, in configuration and whole-motion checks alike.
- `motion_contact` and `motion_is_free` check a straight joint motion as a whole by conservative advancement, so a thin obstacle between two samples is not stepped over.
- Concave objects such as bins and shelves keep their hollows: `add_concave_object` (a mesh file) and `add_trimesh` (triangles built in code) split them into convex parts by V-HACD instead of one hull. `decompose_link` splits a concave link's meshes the same way.
- Named configurations (`NamedPoses`) and tool placement (`EndEffector`), the parts of an SRDF that planning code needs, built in code.

Sensed geometry:

- `points::depth_to_points` back-projects a depth image through `CameraInfo` intrinsics, and `points::register_depth` re-projects it into a colour camera beside it (as `depth_image_proc/register` does).
- `add_points` adds points as occupied voxels the planners avoid.
- `without_robot_points` drops the sensed points that fall on the robot or what it holds before they join the scene (MoveIt's self filter).
- `occupancy::OccupancyMap` keeps sensed obstacles current across scans the way MoveIt's OctoMap does: log-odds per voxel, hits mark, rays clear. `set_occupancy` puts its occupied voxels into the scene; voxels never observed are not added.

For a viewer, `meshes` gives every link and object as world-frame triangles (RViz's planning-scene display).

## Shapes and their limits

URDF boxes, spheres, cylinders and capsules keep their shape. A mesh (OBJ or STL) becomes the convex hull of its vertices, which contains the mesh: the hull may report a contact the mesh would not, never the reverse. A hull fills a concave object's hollows, so nothing can reach into a bin or a shelf unless the object is split into convex parts as above.

Collision geometry is computed by `parry3d-f64` (0.25, the last release on nalgebra); there is no native dependency.

## Related crates

- [`roplat_grasp`](../roplat_grasp/README.md) plans picks and places against a `PlanningScene`.
- [`roplat_perception`](../roplat_perception/README.md) turns point clouds into the objects `add_object` takes.
- [`roplat_frames`](../roplat_frames/README.md) (feature `scene`) moves objects to their tracked frames and places depth readings at the camera pose of their stamp.

## License

Apache-2.0; see [LICENSE](../LICENSE).
