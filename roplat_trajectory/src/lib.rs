//! Trajectory generation: joint and Cartesian interpolation (linear,
//! trapezoid, S-curve, fourth-order) in [`path_generate`], time-optimal path
//! parameterization through COPP in [`time_scaling`], collision-free
//! joint-space paths in [`path_planning`] and their simplification and
//! smoothing in [`path_smoothing`], straight tip paths checked in
//! joint space in [`cartesian_path`], and following a timed joint trajectory
//! on a position channel in [`follow`].
//!
//! The crate takes limits as `roplat_msgs` data ([`JointLimits`],
//! [`CartesianLimits`]) and the control period as a `Duration`, never a robot
//! type, so it depends only on `roplat_msgs` and `copp`. An unbounded entry is
//! `f64::INFINITY`.
//!
//! [`JointLimits`]: roplat_msgs::JointLimits
//! [`CartesianLimits`]: roplat_msgs::CartesianLimits

pub mod cartesian_path;
pub mod constraints;
pub mod follow;
pub mod path_generate;
pub mod path_planning;
pub mod path_smoothing;
pub mod time_scaling;

pub use cartesian_path::{
    CartesianPath, CartesianPathOptions, CartesianStop, plan_cartesian_motion, plan_cartesian_path,
};
pub use constraints::{AxisConstraint, OrientationConstraint, PoseConstraint, constrained};
pub use follow::{JointTrajectoryFollower, TrackingError, points_from_samples};
pub use path_planning::{
    PlanError, PlannerOptions, plan_joint_motion, plan_joint_path, resolution_for_travel,
    segment_is_valid,
};
pub use path_smoothing::{path_length, simplify_motion, simplify_path, smooth_motion, smooth_path};
pub use time_scaling::*;
