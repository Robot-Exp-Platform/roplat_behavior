# roplat_grasp

[English](README.md) | [简体中文](README_zh.md)

平行夹爪的抓取：在哪里抓一个物体，怎样带着它过去、离开，以及把它放到哪里。它回答 MoveIt Grasps 与 MoveIt 的 pick、place 阶段所回答的问题。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- `grasps` 列出长方体、圆柱或球体的 `Grasp` 候选：手指能以夹爪可达的开口宽度横跨物体闭合的每个工具位姿，由优到劣（先从上方接近）。
- `plan_pick` 在 [`PlanningScene`](../roplat_scene/README_zh.md) 中按顺序尝试候选，返回第一个可行候选的三段关节路径：到预抓取位姿的无碰撞路径、直线接近抓取位姿、带着物体直线抬起。每个失败的候选都说明原因。无碰撞路径用 `PlanningScene::motion_is_free` 按整段运动检查，再薄的障碍物也不会被跨过（按 `PlannerOptions::resolution` 间隔检查位形时，比连杆在两次检查之间扫过的距离更薄的板会被跨过；该分辨率不再使用）；直线接近与抬起按笛卡尔步长检查（`CartesianPathOptions`）。`plan_pick_motion` 做同样的规划，但每一段都整体检查。
- `plan_place`（与 `plan_place_motion`）把手中的物体放下：带着物体到第一个可行放置位姿上方的路径、直线下放、松开后的直线后撤。
- `GraspCheck` 依据任何夹爪都会报告的开口宽度判断是否抓住：抓住、空抓、宽度不符，或与驱动自身的判断不一致。
- `carried_mass` 在抬起后由腕部力传感器确认负载。

手指闭合不在规划之内：调用方在接近与抬起之间自行命令其 `Gripper`。也不分配时间；路径是供 [`roplat_trajectory`](../roplat_trajectory/README_zh.md) 时间缩放的路径点。

## 工具坐标系

抓取是工具中心点（TCP）在世界中的位姿：`z` 为接近方向（指向物体），`y` 为手指闭合方向，`x = y × z`。`roplat_scene` 的 `EndEffector` 以同样约定把 TCP 放在机械臂连杆上。

## 局限

候选只由形状得出，不考虑摩擦、质量与相邻物体，因此候选仍可能被挡住或打滑；被挡住的会由规划拒绝。网格不产生候选。形状可以来自 [`roplat_perception`](../roplat_perception/README_zh.md) 的拟合。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
