//! Grasping for a parallel gripper: where to grasp an object, and how to
//! get there and away with it.
//!
//! The two halves answer what MoveIt Grasps and MoveIt's pick stage answer:
//!
//! - [`grasps`] lists [`Grasp`] candidates for a box, cylinder or sphere:
//!   every tool pose from which the fingers close across the object at a
//!   width the gripper opens to, best first (approach from above first).
//! - [`plan_pick`] tries candidates in order against a
//!   [`PlanningScene`](roplat_scene::PlanningScene) and returns the first
//!   that works as three joint paths: a collision-free path to the
//!   pre-grasp pose, a straight approach to the grasp, and a straight lift
//!   with the object attached. For each candidate that fails it says why.
//!   The free path is checked as whole motions, so no thin obstacle is
//!   stepped over; the straight motions at their Cartesian steps.
//!   [`plan_pick_motion`] does the same with every segment checked whole.
//! - [`plan_place`] (and [`plan_place_motion`]) puts the held object down:
//!   a path carrying it to above the first placement that works, a
//!   straight descent onto it, and a straight retreat once released.
//!
//! Closing the fingers is not planned: the caller commands its `Gripper`
//! between the approach and the lift. Times are not assigned either; the
//! paths are waypoints for `roplat_trajectory`'s time scaling. Whether the
//! closed fingers hold the object, [`GraspCheck`] judges from the opening
//! any gripper reports, and [`carried_mass`] confirms after the lift from a
//! wrist force sensor.
//!
//! # Tool frame
//!
//! A grasp is a pose of the tool centre point (TCP) in the world: `z` is
//! the approach direction (into the object), `y` the closing direction of
//! the fingers, `x = y × z`. [`EndEffector`](roplat_scene::EndEffector)
//! places the TCP on the arm's link with the same convention.
//!
//! # Limits
//!
//! Candidates come from the shape alone: friction, mass and the object's
//! neighbours are not considered, so a candidate can still be blocked or
//! slip; the planner rejects the blocked ones. Meshes give no candidates.

mod candidates;
mod pick;
mod place;
mod verify;

pub use candidates::{Grasp, GraspOptions, box_grasps, cylinder_grasps, grasps, sphere_grasps};
pub use pick::{PickError, PickFailure, PickOptions, PickPlan, plan_pick, plan_pick_motion};
pub use place::{PlaceError, PlaceFailure, PlaceOptions, PlacePlan, plan_place, plan_place_motion};
pub use verify::{GraspCheck, GraspOutcome, carried_mass};
