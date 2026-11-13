# roplat_perception

[English](README.md) | [简体中文](README_zh.md)

Point clouds for manipulation: from sensed points to the objects a planner avoids and a grasp generator grasps. These are the steps of the tabletop pipeline that ROS builds from PCL, on plain `[f64; 3]` points in one frame, with no native dependency.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- `cloud`: thinning to an even density (`voxel_downsample`), cropping to a workspace box, and removing isolated returns by a radius (`remove_sparse`) or by the cloud's own statistics (`remove_statistical_outliers`, PCL's `StatisticalOutlierRemoval`).
- `segment`: the dominant plane by RANSAC (`fit_plane`, optionally level within a tilt of `up`), and Euclidean clusters (`euclidean_clusters`).
- `fit`: a box (smallest-area rectangle of the outline) or an upright cylinder (circle fit) resting on a plane, or a sphere anywhere from the side that is seen (`fit_sphere`). Each is a `roplat_description` `Geometry` and a pose, the inputs of `roplat_grasp::grasps` and of `PlanningScene::add_object`.
- `tabletop`: all of it in one call (`segment_tabletop`), returning the table and each object on it.
- `registration`: ICP, point-to-point (`icp`) or point-to-plane (`icp_point_to_plane`), for the pose of an object whose model is known.
- `normals`: surface normals and curvature (`estimate_normals`, PCL's `NormalEstimation`), and smooth regions grown across them (`smooth_regions`, PCL's `RegionGrowing`), which separate surfaces that touch at a crease.
- `search`: exact nearest-neighbour queries by a k-d tree (`KdTree`): the nearest point, the `k` nearest (`k_nearest`, PCL's `nearestKSearch`) and every point within a radius (`within`, PCL's `radiusSearch`) of any query point. Answers are ordered by distance and then by index, and match brute force exactly ([`search.rs`](tests/search.rs)).

Depth images become points in [`roplat_scene`](../roplat_scene/README.md) (`points::depth_to_points`), and the objects found go to [`roplat_grasp`](../roplat_grasp/README.md) and the planning scene.

## Example

```rust
use roplat_perception::{TabletopOptions, segment_tabletop};

// A table at 0.7 m and the top of a 4 cm cube on it, every 5 mm.
let mut points = Vec::new();
for i in 0..100 {
    for j in 0..100 {
        points.push([0.005 * i as f64, 0.005 * j as f64, 0.7]);
    }
}
for i in 0..=8 {
    for j in 0..=8 {
        points.push([0.2 + 0.005 * i as f64, 0.2 + 0.005 * j as f64, 0.74]);
    }
}
let found = segment_tabletop(&points, &TabletopOptions::default()).unwrap();
assert_eq!(found.objects.len(), 1);
let centre = found.objects[0].shape.pose.translation.vector;
assert!((centre.x - 0.22).abs() < 0.005 && (centre.z - 0.72).abs() < 0.005);
```

The integration tests go further:

- [`tabletop.rs`](tests/tabletop.rs): a depth camera looks at a table with two boxes and a cylinder; the objects found become grasp candidates and planning-scene obstacles.
- [`pick_from_depth.rs`](tests/pick_from_depth.rs): from a depth image of an arm over a table to a planned pick, with the arm's own points removed first.
- [`touching.rs`](tests/touching.rs): a can pressed against a box, separated by smooth regions where Euclidean clustering sees one object.

## License

Apache-2.0; see [LICENSE](../LICENSE).
