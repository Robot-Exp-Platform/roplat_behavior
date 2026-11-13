# roplat_perception

[English](README.md) | [简体中文](README_zh.md)

面向操作的点云处理：从传感点得到规划要避开、抓取要抓的物体。这些是 ROS 用 PCL 搭的桌面流程中的各步，输入是同一坐标系下的普通 `[f64; 3]` 点，无原生依赖。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- `cloud`：体素降采样到均匀密度（`voxel_downsample`）、按工作空间长方体裁剪、按半径（`remove_sparse`）或按点云自身统计（`remove_statistical_outliers`，即 PCL 的 `StatisticalOutlierRemoval`）去除孤立点。
- `segment`：RANSAC 求主平面（`fit_plane`，可要求在 `up` 的倾角内水平）与欧氏聚类（`euclidean_clusters`）。
- `fit`：在平面上拟合长方体（轮廓的最小面积矩形）或直立圆柱（圆拟合），或由看到的一侧拟合任意位置的球（`fit_sphere`）。结果都是 `roplat_description` 的 `Geometry` 与位姿，即 `roplat_grasp::grasps` 与 `PlanningScene::add_object` 的输入。
- `tabletop`：一次完成全部步骤（`segment_tabletop`），给出桌面和其上的每个物体。
- `registration`：点到点（`icp`）或点到面（`icp_point_to_plane`）ICP，求已知模型的物体位姿。
- `normals`：表面法向与曲率（`estimate_normals`，即 PCL 的 `NormalEstimation`），以及沿法向生长的光滑区域（`smooth_regions`，即 PCL 的 `RegionGrowing`），可把在棱处相接的表面分开。
- `search`：用 k-d 树（`KdTree`）做精确的近邻查询：任意查询点的最近点、`k` 个最近点（`k_nearest`，即 PCL 的 `nearestKSearch`）与半径内的全部点（`within`，即 PCL 的 `radiusSearch`）。结果按距离、再按下标排序，与暴力搜索逐条一致（[`search.rs`](tests/search.rs)）。

深度图在 [`roplat_scene`](../roplat_scene/README_zh.md) 中变成点（`points::depth_to_points`），找到的物体交给 [`roplat_grasp`](../roplat_grasp/README_zh.md) 与规划场景。

## 示例

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

集成测试走得更远：

- [`tabletop.rs`](tests/tabletop.rs)：深度相机看着桌上的两个长方体与一个圆柱；找到的物体成为抓取候选与规划场景中的障碍。
- [`pick_from_depth.rs`](tests/pick_from_depth.rs)：从拍到桌面上方机械臂的深度图，到规划好的抓取，先去掉机械臂自身的点。
- [`touching.rs`](tests/touching.rs)：罐子紧贴盒子，欧氏聚类只看到一个物体，光滑区域把它们分开。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
