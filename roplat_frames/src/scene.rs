//! Planning-scene geometry placed from the frame buffer (feature `scene`).
//!
//! A [`PlanningScene`] is plain data with no clock; the [`FrameBuffer`]
//! knows where every frame was at each instant. These connect the two:
//!
//! - [`place_objects`] moves scene objects that perception tracks as frames
//!   to where the buffer has them at a stamp;
//! - [`add_sensed_points`] adds a depth reading at the camera pose of the
//!   instant it was taken. For a camera on the arm that is where the joints
//!   put it then, interpolated from the recorded joint states, not where the
//!   arm is by the time the points are processed.
//!
//! `world` is the frame the scene's world is in: the tree's frame in which
//! [`PlanningScene::base`] places the robot.

use roplat_msgs::{FrameId, Stamp};
use roplat_scene::{PlanningScene, SceneError};

use crate::{FrameBuffer, FrameError};

/// A frame that could not be looked up, or a scene that refused the result.
#[derive(Debug, thiserror::Error)]
pub enum SceneSyncError {
    #[error("looking up the frame of `{object}`: {source}")]
    Frame {
        object: String,
        #[source]
        source: FrameError,
    },
    #[error("scene object `{object}`: {source}")]
    Scene {
        object: String,
        #[source]
        source: SceneError,
    },
}

/// Moves each `(object, frame)` of the scene to where `frame` is in `world`
/// at `at`. Stops at the first failure; the objects before it stay moved.
pub fn place_objects<const N: usize>(
    buffer: &FrameBuffer,
    scene: &mut PlanningScene<N>,
    world: FrameId,
    at: Stamp,
    objects: &[(&str, FrameId)],
) -> Result<(), SceneSyncError> {
    for &(object, frame) in objects {
        let pose = buffer
            .lookup(world, frame, at)
            .map_err(|source| SceneSyncError::Frame { object: object.into(), source })?;
        scene
            .move_object(object, pose.into())
            .map_err(|source| SceneSyncError::Scene { object: object.into(), source })?;
    }
    Ok(())
}

/// Adds `points`, measured in `camera`'s frame at `at`, as the object `name`
/// of voxels with edge `voxel` (see [`PlanningScene::add_points`]), placed
/// where `camera` was in `world` at `at`. Returns the number of voxels.
#[allow(
    clippy::too_many_arguments,
    reason = "frame buffer, scene, object name, points, voxel size, world and camera frames and the time are the independent inputs of one scene update"
)]
pub fn add_sensed_points<const N: usize>(
    buffer: &FrameBuffer,
    scene: &mut PlanningScene<N>,
    name: &str,
    points: &[[f64; 3]],
    voxel: f64,
    world: FrameId,
    camera: FrameId,
    at: Stamp,
) -> Result<usize, SceneSyncError> {
    let pose = buffer
        .lookup(world, camera, at)
        .map_err(|source| SceneSyncError::Frame { object: name.into(), source })?;
    scene
        .add_points(name, points, voxel, pose.into())
        .map_err(|source| SceneSyncError::Scene { object: name.into(), source })
}
