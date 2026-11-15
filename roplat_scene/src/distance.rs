//! Distances between the scene's shapes.
//!
//! parry3d-f64 0.25 measures two cuboids with `distance_cuboid_cuboid`,
//! which takes its points from `closest_points_cuboid_cuboid`, a routine
//! made for contacts: it finds the axis of greatest separation and projects
//! one support vertex of a box onto the other box. That vertex need not be
//! the nearest point, so the distance can come out far too large: 2.3 cm
//! for a box link 1.1 mm from a thin plate. A distance too large breaks the
//! whole-motion check, which advances by the distance and may then step
//! past a contact, and makes [`clearance`](crate::PlanningScene::clearance)
//! report room that is not there. Its other pairs are measured correctly:
//! analytically for a ball, by GJK between other convex shapes.
//!
//! [`distance`] measures two cuboids by GJK too, also as parts of a
//! compound (sensed voxels), and leaves every other pair to parry.

use nalgebra as na;
use parry3d_f64::{
    query::{
        ClosestPoints, Contact, DefaultQueryDispatcher, NonlinearRigidMotion, QueryDispatcher,
        ShapeCastHit, ShapeCastOptions, Unsupported,
        details::{
            distance_composite_shape_shape, distance_shape_composite_shape,
            distance_support_map_support_map,
        },
    },
    shape::Shape,
};

/// Distance (m) between `a` posed at `pa` and `b` at `pb`, zero when they
/// overlap. Between polyhedra GJK finds it to rounding. Against a capsule
/// or a cylinder it may stop before it converges, with points on the two
/// shapes slightly farther apart than the nearest: up to 3.3e-5 m in the
/// tests (see [`motion`](crate::motion) for what that allows). Every shape
/// the scene builds is convex or a compound of convex parts; were a pair
/// unsupported, it would count as touching.
pub(crate) fn distance(
    pa: &na::Isometry3<f64>,
    a: &dyn Shape,
    pb: &na::Isometry3<f64>,
    b: &dyn Shape,
) -> f64 {
    SceneQueries.distance(&pa.inv_mul(pb), a, b).unwrap_or(0.0)
}

/// parry's default queries, except the distance between two cuboids, which
/// is found by GJK, at any depth of compound.
struct SceneQueries;

impl QueryDispatcher for SceneQueries {
    fn intersection_test(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<bool, Unsupported> {
        DefaultQueryDispatcher.intersection_test(pos12, g1, g2)
    }

    fn distance(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<f64, Unsupported> {
        if let (Some(c1), Some(c2)) = (g1.as_cuboid(), g2.as_cuboid()) {
            Ok(distance_support_map_support_map(pos12, c1, c2))
        } else if let Some(c1) = g1.as_composite_shape() {
            // The default dispatcher would measure the parts with itself.
            Ok(distance_composite_shape_shape(self, pos12, c1, g2))
        } else if let Some(c2) = g2.as_composite_shape() {
            Ok(distance_shape_composite_shape(self, pos12, g1, c2))
        } else {
            DefaultQueryDispatcher.distance(pos12, g1, g2)
        }
    }

    fn contact(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: f64,
    ) -> Result<Option<Contact>, Unsupported> {
        DefaultQueryDispatcher.contact(pos12, g1, g2, prediction)
    }

    fn closest_points(
        &self,
        pos12: &na::Isometry3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
        max_dist: f64,
    ) -> Result<ClosestPoints, Unsupported> {
        DefaultQueryDispatcher.closest_points(pos12, g1, g2, max_dist)
    }

    fn cast_shapes(
        &self,
        pos12: &na::Isometry3<f64>,
        local_vel12: &na::Vector3<f64>,
        g1: &dyn Shape,
        g2: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        DefaultQueryDispatcher.cast_shapes(pos12, local_vel12, g1, g2, options)
    }

    fn cast_shapes_nonlinear(
        &self,
        motion1: &NonlinearRigidMotion,
        g1: &dyn Shape,
        motion2: &NonlinearRigidMotion,
        g2: &dyn Shape,
        start_time: f64,
        end_time: f64,
        stop_at_penetration: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        DefaultQueryDispatcher.cast_shapes_nonlinear(
            motion1,
            g1,
            motion2,
            g2,
            start_time,
            end_time,
            stop_at_penetration,
        )
    }
}
