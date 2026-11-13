# roplat_frames

[English](README.md) | [简体中文](README_zh.md)

Coordinate frames of a robot cell: one tree of static, joint-driven and dynamic transforms, looked up at an instant, and the camera calibration that fixes its sensor edges.

Part of the [roplat_behavior workspace](../README.md). 0.9 is not on crates.io yet; build it from source as [Working from source](../README.md#working-from-source) describes.

## Contents

- `frame_id` derives a `FrameId` from a frame name at compile time, so producers can fill `Header::frame` without holding a registry.
- `FrameTree` fixes the topology and the static transforms when it is built: URDF links and joints through `FrameTreeBuilder::model`, calibrated extrinsics through `FrameTreeBuilder::fixed`.
- `FrameBuffer` keeps a time history of joint positions and dynamic transforms on one clock and answers `FrameBuffer::lookup` at a `Stamp`. Writes and lookups do not allocate and never block.
- Hand-eye calibration of a camera mount (`calibration::calibrate_hand_eye`, eye in hand or eye to hand), for the fixed edge it becomes.
- The target pose from images: `target::detect_chessboard` finds a chessboard's corners to a fraction of a pixel, in an order fixed across views, and `target::target_pose` solves the board's pose.
- Lens distortion as `CameraInfo` describes it: `lens::distort`, `lens::undistort` and `lens::Rectify` for the plumb_bob, rational polynomial and fisheye models, as `image_proc` rectifies.
- General image processing on 8-bit images (`image`): `Image::crop`, `Image::resize` by nearest neighbour, bilinear interpolation or pixel area, and conversions between RGB, grey (BT.601 luma), HSV (OpenCV's 8-bit convention), NV12 and YUYV (BT.601 or BT.709, limited or full range) and raw Bayer mosaics (bilinear demosaicing). The tests check each pixel against the published formula and against OpenCV 4.14 and GStreamer 1.20 output.
- Poses and points with covariances (`uncertain`): `UncertainPose::compose`, `inverse` and `transform_point` propagate an SE(3) covariance to first order, in the right-perturbation convention of Sola et al., "A micro Lie theory for state estimation in robotics"; `parent_covariance` converts to the translation-and-rotation error in the parent frame. The tests compare with Monte Carlo.
- Feature `roplat`: `JointStateNode`, which records joint states into a borrowed buffer.
- Feature `scene`: `scene::place_objects` moves planning-scene objects to their tracked frames at a stamp, and `scene::add_sensed_points` adds a depth reading at the camera pose of its stamp.

There is no global registry or buffer: a `FrameBuffer` is an ordinary value owned by its creating scope and lent to nodes as a resource. The [frame tree design (zh)](docs/design_zh.md) explains its choices and what is left open.

## Features

| Feature | Purpose |
|---|---|
| `roplat` | `roplat::JointStateNode`. |
| `scene` | The `scene` module, on [`roplat_scene`](../roplat_scene/README.md). |

Default features are empty.

## License

Apache-2.0; see [LICENSE](../LICENSE).
