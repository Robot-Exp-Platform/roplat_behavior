//! Point clouds for manipulation: from sensed points to the objects a
//! planner avoids and a grasp generator grasps.
//!
//! The steps of the tabletop pipeline that ROS builds from PCL, on plain
//! `[f64; 3]` points in one frame:
//!
//! - [`cloud`]: thinning to an even density ([`voxel_downsample`]),
//!   cropping to a workspace box, removing isolated returns by a radius
//!   ([`remove_sparse`]) or by the cloud's own statistics
//!   ([`remove_statistical_outliers`]).
//! - [`segment`]: the dominant plane by RANSAC ([`fit_plane`]), and
//!   Euclidean clusters ([`euclidean_clusters`]).
//! - [`fit`]: a box or an upright cylinder resting on a plane, or a sphere
//!   anywhere ([`fit_sphere`]), as a
//!   `roplat_description` [`Geometry`](roplat_description::Geometry) and a
//!   pose: the inputs of `roplat_grasp::grasps` and of
//!   `PlanningScene::add_object` in `roplat_scene`.
//! - [`tabletop`]: all of it in one call ([`segment_tabletop`]): the table
//!   and each object on it.
//! - [`registration`]: ICP, point-to-point ([`icp`]) or point-to-plane
//!   ([`icp_point_to_plane`]), for the pose of an object whose model is
//!   known.
//! - [`normals`]: surface normals and curvature ([`estimate_normals`]), and
//!   smooth surfaces grown across them ([`smooth_regions`]), which separate
//!   surfaces that touch at a crease.
//! - [`search`]: the nearest, the `k` nearest and the points within a radius
//!   of any query point, exact, by a k-d tree ([`KdTree`]).
//!
//! ```
//! use roplat_perception::{TabletopOptions, segment_tabletop};
//!
//! // A table at 0.7 m and the top of a 4 cm cube on it, every 5 mm.
//! let mut points = Vec::new();
//! for i in 0..100 {
//!     for j in 0..100 {
//!         points.push([0.005 * i as f64, 0.005 * j as f64, 0.7]);
//!     }
//! }
//! for i in 0..=8 {
//!     for j in 0..=8 {
//!         points.push([0.2 + 0.005 * i as f64, 0.2 + 0.005 * j as f64, 0.74]);
//!     }
//! }
//! let found = segment_tabletop(&points, &TabletopOptions::default()).unwrap();
//! assert_eq!(found.objects.len(), 1);
//! let centre = found.objects[0].shape.pose.translation.vector;
//! assert!((centre.x - 0.22).abs() < 0.005 && (centre.z - 0.72).abs() < 0.005);
//! ```
//!
//! Points come from a depth image through
//! `roplat_scene::points::depth_to_points`, in the camera frame. Move them
//! into a frame whose up direction is known, such as the world, with the
//! camera's pose (from `roplat_frames`) before segmenting. A robot arm in
//! view is segmented like any object; remove its points first when it may
//! be over the table. Randomized steps take a seed, so results repeat.

pub mod cloud;
pub mod fit;
mod grid;
pub mod normals;
pub mod registration;
pub mod search;
pub mod segment;
pub mod tabletop;

pub use cloud::{crop_box, remove_sparse, remove_statistical_outliers, voxel_downsample};
pub use fit::{FittedShape, fit_box, fit_box_on, fit_cylinder_on, fit_on, fit_sphere};
pub use normals::{RegionOptions, SurfaceNormal, estimate_normals, smooth_regions};
pub use registration::{IcpOptions, Registration, icp, icp_point_to_plane};
pub use search::{KdTree, Neighbour};
pub use segment::{Plane, PlaneFit, PlaneOptions, euclidean_clusters, fit_plane};
pub use tabletop::{Tabletop, TabletopError, TabletopObject, TabletopOptions, segment_tabletop};
