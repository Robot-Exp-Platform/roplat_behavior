# roplat_scene

[English](README.md) | [简体中文](README_zh.md)

运动规划的碰撞检查：由 URDF 构造的机器人、周围的物体与它手中的物体，以及传感器看到的障碍。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

场景本身：

- `PlanningScene` 包含由 URDF 构造的机器人（网格取凸包）、世界中的物体与附着在连杆上的物体、允许接触对（相邻连杆默认允许）与安全距离。
- 它通过 `CollisionCheck` 回答 [`roplat_trajectory`](../roplat_trajectory/README_zh.md) 中 `plan_joint_path` 向调用方要的有效性检查（`scene.is_free(q)`）；`contacts` 列出接触，`clearance` 给出最近的一对。
- `set_pair_padding` 让某一对连杆或物体使用自己的安全间隙，取代场景的统一间隙，位形检查与整段运动检查都按它计算。
- `motion_contact` 与 `motion_is_free` 用保守推进整体检查一段关节直线运动，两个采样点之间的薄障碍不会被跳过。
- `reach` 给出规划组每个关节所带动的任何一点离该关节轴线的最远距离（移动关节为 1），在规划器搜索的上下限之内成立；`roplat_trajectory` 的 `resolution_for_travel` 据此选采样分辨率，使相邻两次检查之间任何一点的位移不超过给定值。
- 料箱、货架等凹形物体可保留内部空间：`add_concave_object`（网格文件）与 `add_trimesh`（代码中给出的三角形）用 V-HACD 把它们分成若干凸块，而不是取一个凸包。`decompose_link` 以同样方式分解凹形连杆的网格。
- 命名位姿（`NamedPoses`）与工具安装位置（`EndEffector`），即规划代码需要的 SRDF 内容，在代码中构造。

传感得到的几何：

- `points::depth_to_points` 按 `CameraInfo` 内参把深度图反投影为点，`points::register_depth` 把深度图重投影到旁边的彩色相机（即 `depth_image_proc/register`）。
- `add_points` 把点作为占据体素加入场景，供规划避开。
- `without_robot_points` 在传感点加入场景前，去掉落在机器人或其所持物体上的点（即 MoveIt 的自身滤波）。
- `occupancy::OccupancyMap` 像 MoveIt 的 OctoMap 一样随扫描更新传感障碍：每个体素保存对数几率，命中处标记占据，光线经过处清除。`set_occupancy` 把占据体素放进场景；从未观测到的体素不加入。

可视化时，`meshes` 把每个连杆与物体给成世界系中的三角网格（即 RViz 的规划场景显示）。

## 形状及其局限

URDF 的长方体、球、圆柱与胶囊保持原形。网格（OBJ 或 STL）取其顶点的凸包，凸包包含网格：凸包可能报告网格本不会有的接触，反之不会。凸包会填满凹形物体的内部空间，因此除非像上面那样分成凸块，什么都伸不进料箱或货架。

碰撞几何由 `parry3d-f64` 计算（0.25，最后一个基于 nalgebra 的版本），无原生依赖。它的距离不用，因为有的会偏大，而按偏大的距离推进的整段检查会越过接触：parry 0.25 对两个长方体用为接触设计的例程测距离，面接近平行时可能报出数倍于实际的距离（离薄板 1.1 mm 的盒子连杆报 2.3 cm）；它的 GJK 对细的胶囊或圆柱会提前停下，最多多出约 0.03 mm。场景自己测每一对凸块的距离，使距离除舍入外绝不大于真实值：球取球心、胶囊取线段，再减去半径；这些核心（或原样的长方体、圆柱、网格凸包）用 GJK 测，报告它遇到的最大支撑间隙，不论何时停下都是距离的下界。距离可能比真实值小，GJK 收敛时不超过 1e-10 m；测试对场景构造的每一种形状对用独立的闭式核对，两个方向的误差都报告。位形检查（`is_free`、`contacts`）仍用 parry 的接触查询，以 padding 为其余量。

debug 构建应当优化 parry，它的 GJK 与凸包不优化时很慢；工作区的 `Cargo.toml` 在 dev profile 里给 `parry3d-f64` 设了 `opt-level = 2`，依赖本 crate 的项目也可以照做。

## 相关的包

- [`roplat_grasp`](../roplat_grasp/README_zh.md) 在 `PlanningScene` 中规划抓取与放置。
- [`roplat_perception`](../roplat_perception/README_zh.md) 把点云变成 `add_object` 接受的物体。
- [`roplat_frames`](../roplat_frames/README_zh.md)（feature `scene`）把物体移到其跟踪坐标系，并按图像时刻的相机位姿放置深度读数。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
