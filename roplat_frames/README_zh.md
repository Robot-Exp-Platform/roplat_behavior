# roplat_frames

[English](README.md) | [简体中文](README_zh.md)

机器人工作单元的坐标系：一棵由静态、关节驱动与动态变换组成、可按时刻查询的树，以及确定其中传感器边的相机标定。

属于 [roplat_behavior 工作区](../README_zh.md)。0.9 尚未发布到 crates.io，请按[从源码开发](../README_zh.md#从源码开发)从源码构建。

## 内容

- `frame_id` 在编译期由帧名得到 `FrameId`，生产者填写 `Header::frame` 时不需要持有注册表。
- `FrameTree` 在构造时确定拓扑与静态变换：URDF 的连杆与关节经 `FrameTreeBuilder::model`，标定得到的外参经 `FrameTreeBuilder::fixed`。
- `FrameBuffer` 在同一时钟上保存关节位置与动态变换的时间历史，按 `Stamp` 回答 `FrameBuffer::lookup`。写入与查询不分配内存，也不阻塞。
- `FrameBuffer::latest_sample_time` 取路径上各采样边最新时刻的最小值，纯静态路径返回 `None`。以返回的时刻调用 `lookup` 可验证每条历史能否在该时取值。
- 相机安装位置的手眼标定（`calibration::calibrate_hand_eye`，眼在手上或眼在手外），结果成为树中的固定边。
- 从图像得到标定板位姿：`target::detect_chessboard` 以亚像素精度找出棋盘格角点，各视角下顺序一致；`target::target_pose` 求标定板位姿。
- 按 `CameraInfo` 描述的镜头畸变：`lens::distort`、`lens::undistort` 与 `lens::Rectify`，支持 plumb_bob、rational polynomial 与鱼眼模型，相当于 `image_proc` 的校正。
- feature `roplat`：`JointStateNode`，把关节状态记录到借来的缓冲中。
- feature `scene`：`scene::place_objects` 把规划场景中的物体移到其跟踪坐标系在某一时刻的位姿，`scene::add_sensed_points` 按图像时刻的相机位姿加入深度读数。

没有全局注册表或全局缓冲：`FrameBuffer` 是普通的值，归创建层所有，作为资源借给节点。[坐标树设计](docs/design_zh.md)说明了各项取舍和尚未解决的问题。

## Feature

| Feature | 用途 |
|---|---|
| `roplat` | `roplat::JointStateNode`。 |
| `scene` | `scene` 模块，基于 [`roplat_scene`](../roplat_scene/README_zh.md)。 |

默认 feature 为空。

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
