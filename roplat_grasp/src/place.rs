//! Planning a place: carry the held object over its target, lower it,
//! release, retreat.

use nalgebra as na;
use roplat_scene::{CollisionCheck, EndEffector, PlanningScene};
use roplat_trajectory::{
    CartesianPathOptions, CartesianStop, PlanError, PlannerOptions, plan_cartesian_motion,
    plan_cartesian_path, plan_joint_motion, plan_joint_path,
};

use crate::pick::Check;

/// Settings for [`plan_place`].
#[derive(Clone, Copy, Debug)]
pub struct PlaceOptions {
    /// The object's straight motion onto its placement, in the world (m);
    /// usually down. The descent starts this far before the placement.
    pub descent: na::Vector3<f64>,
    /// How far the tool backs off along its approach axis after the
    /// fingers open (m).
    pub retreat: f64,
    /// Search for the free path to above the placement.
    pub planner: PlannerOptions,
    /// Steps of the straight descent and retreat.
    pub cartesian: CartesianPathOptions,
}

impl Default for PlaceOptions {
    fn default() -> Self {
        Self {
            descent: na::Vector3::new(0.0, 0.0, -0.1),
            retreat: 0.1,
            planner: PlannerOptions::default(),
            cartesian: CartesianPathOptions::default(),
        }
    }
}

/// A place that works: the placement used and the three joint paths, each
/// starting where the previous one ended.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacePlan<const N: usize> {
    /// The object's pose in the world once placed.
    pub placement: na::Isometry3<f64>,
    /// Index of [`placement`](Self::placement) in the placements given.
    pub candidate: usize,
    /// From the start to above the placement, carrying the object clear of
    /// everything.
    pub to_preplace: Vec<[f64; N]>,
    /// Straight onto the placement; the object may touch its support.
    pub descent: Vec<[f64; N]>,
    /// Straight back along the tool's approach axis with the object left
    /// behind; the tool may touch it.
    pub retreat: Vec<[f64; N]>,
}

impl<const N: usize> PlacePlan<N> {
    /// The configuration in which to open the fingers.
    pub fn release_configuration(&self) -> [f64; N] {
        *self
            .descent
            .last()
            .expect("a descent has at least its start")
    }
}

/// Why one placement did not work.
#[derive(Clone, Debug, PartialEq)]
pub enum PlaceFailure {
    /// The object is not attached to the robot in the scene.
    NotHeld,
    /// The inverse kinematics found no configuration above the placement.
    PreplaceUnreachable,
    /// The configuration above the placement collides or is outside the
    /// bounds.
    PreplaceInvalid,
    /// No free path from the start to above the placement.
    Path(PlanError),
    /// The straight descent stopped after `fraction` of the way.
    Descent { stop: CartesianStop, fraction: f64 },
    /// The straight retreat stopped after `fraction` of the way.
    Retreat { stop: CartesianStop, fraction: f64 },
    /// The scene refused a name (object, link or support).
    Scene(String),
}

/// No placement worked; `attempts` holds each placement tried and why it
/// failed, in order. Empty when there were no placements.
#[derive(Clone, Debug, PartialEq)]
pub struct PlaceError {
    pub attempts: Vec<(usize, PlaceFailure)>,
}

impl std::fmt::Display for PlaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.attempts.is_empty() {
            return write!(f, "no placements");
        }
        write!(f, "none of {} placements works:", self.attempts.len())?;
        for (k, why) in &self.attempts {
            write!(f, " #{k} {why:?};")?;
        }
        Ok(())
    }
}

impl std::error::Error for PlaceError {}

/// Plans putting the held `object` down at the first of `placements` (its
/// pose in the world once placed) that works, from `start`: the place half
/// of MoveIt's pick and place.
///
/// - `scene` holds `object` attached to `hand`'s link, as after a pick
///   ([`PlanningScene::attach`] at the grasp configuration); the grasp it
///   was taken with fixes where the tool goes for each placement.
/// - `support`, if given, is the object or link the object is set on,
///   which it may touch during the descent; a placement sunk into it is
///   therefore not refused, so give placements resting on it.
/// - `inverse` and `lower..=upper` as for [`plan_pick`](crate::plan_pick).
///
/// The scene is not changed: releasing the object for the retreat happens
/// on a copy. After executing the plan, detach the object in the caller's
/// own scene with [`PlanningScene::detach`] at
/// [`release_configuration`](PlacePlan::release_configuration).
#[allow(
    clippy::too_many_arguments,
    reason = "the place planner's inputs are independent: scene, hand, object, support, placements, start, joint range, inverse kinematics and options; grouping them into a planning context is the later form (style ruling 10)"
)]
pub fn plan_place<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    placements: &[na::Isometry3<f64>],
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: PlaceOptions,
) -> Result<PlacePlan<N>, PlaceError> {
    place(
        scene,
        hand,
        object,
        support,
        placements,
        start,
        lower,
        upper,
        inverse,
        options,
        Check::Sampled,
    )
}

/// As [`plan_place`], with every segment checked whole by
/// [`PlanningScene::motion_is_free`], as
/// [`plan_pick_motion`](crate::plan_pick_motion) does for the pick.
#[allow(
    clippy::too_many_arguments,
    reason = "the place motion planner's inputs are independent: scene, hand, object, support, placements, start, joint range, inverse kinematics and options; grouping them into a planning context is the later form (style ruling 10)"
)]
pub fn plan_place_motion<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    placements: &[na::Isometry3<f64>],
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: PlaceOptions,
) -> Result<PlacePlan<N>, PlaceError> {
    place(
        scene,
        hand,
        object,
        support,
        placements,
        start,
        lower,
        upper,
        inverse,
        options,
        Check::Whole,
    )
}

#[allow(
    clippy::too_many_arguments,
    reason = "internal step taking the public planner's inputs unchanged"
)]
fn place<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    placements: &[na::Isometry3<f64>],
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    mut inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: PlaceOptions,
    check: Check,
) -> Result<PlacePlan<N>, PlaceError> {
    let inside = |q: &[f64; N]| (0..N).all(|i| (lower[i]..=upper[i]).contains(&q[i]));
    let mut attempts = Vec::new();
    for (candidate, placement) in placements.iter().enumerate() {
        match try_placement(
            scene,
            hand,
            object,
            support,
            placement,
            start,
            lower,
            upper,
            &inside,
            &mut inverse,
            &options,
            check,
        ) {
            Ok((to_preplace, descent, retreat)) => {
                return Ok(PlacePlan {
                    placement: *placement,
                    candidate,
                    to_preplace,
                    descent,
                    retreat,
                });
            }
            Err(why) => attempts.push((candidate, why)),
        }
    }
    Err(PlaceError { attempts })
}

type Paths<const N: usize> = (Vec<[f64; N]>, Vec<[f64; N]>, Vec<[f64; N]>);

#[allow(
    clippy::too_many_arguments,
    reason = "internal step taking the public planner's inputs unchanged"
)]
fn try_placement<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    placement: &na::Isometry3<f64>,
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    inside: &impl Fn(&[f64; N]) -> bool,
    inverse: &mut impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: &PlaceOptions,
    check: Check,
) -> Result<Paths<N>, PlaceFailure> {
    let scene_error = |e: roplat_scene::SceneError| PlaceFailure::Scene(e.to_string());

    // Where the tool holds the object, from the attachment.
    if scene.clone().detach(object, &start).is_err() {
        return Err(PlaceFailure::NotHeld);
    }
    let tool = hand.pose(scene, &start).map_err(scene_error)?;
    let held = scene.object_pose(object, &start).map_err(scene_error)?;
    let object_in_tool = tool.inverse() * held;
    let tool_at = |object_pose: &na::Isometry3<f64>| object_pose * object_in_tool.inverse();

    // 1. A free path, carrying the object, to above the placement.
    let place_tool = tool_at(placement);
    let preplace_tool = na::Translation3::from(-options.descent) * place_tool;
    let q_pre = inverse(&hand.link_target(&preplace_tool), &start)
        .ok_or(PlaceFailure::PreplaceUnreachable)?;
    if !inside(&q_pre) || !scene.is_free(&q_pre) {
        return Err(PlaceFailure::PreplaceInvalid);
    }
    let to_preplace = match check {
        Check::Sampled => plan_joint_path(
            start,
            q_pre,
            lower,
            upper,
            |q| scene.is_free(q),
            options.planner,
        ),
        Check::Whole => plan_joint_motion(
            start,
            q_pre,
            lower,
            upper,
            |a, b| scene.motion_is_free(a, b),
            options.planner,
        ),
    }
    .map_err(PlaceFailure::Path)?;

    let forward = |scene: &PlanningScene<N>, q: &[f64; N]| {
        hand.pose(scene, q)
            .expect("the hand's link is in the scene")
    };
    // A straight tool motion from `from` to `pose`, valid in `scene`.
    let mut straight = |scene: &PlanningScene<N>, from: [f64; N], pose: na::Isometry3<f64>| {
        let forward = |q: &[f64; N]| forward(scene, q);
        let inverse =
            |pose: &na::Isometry3<f64>, seed: &[f64; N]| inverse(&hand.link_target(pose), seed);
        match check {
            Check::Sampled => plan_cartesian_path(
                from,
                &[pose],
                forward,
                inverse,
                |q| inside(q) && scene.is_free(q),
                options.cartesian,
            ),
            Check::Whole => plan_cartesian_motion(
                from,
                &[pose],
                forward,
                inverse,
                |a, b| inside(a) && inside(b) && scene.motion_is_free(a, b),
                options.cartesian,
            ),
        }
    };

    // 2. Straight down; the object may touch its support.
    let mut descent_scene = scene.clone();
    if let Some(support) = support {
        descent_scene.allowed_mut().allow(object, support);
    }
    let descent = straight(&descent_scene, q_pre, place_tool);
    if let Some(stop) = descent.stop {
        return Err(PlaceFailure::Descent { stop, fraction: descent.fraction });
    }
    let q_place = *descent
        .waypoints
        .last()
        .expect("a complete path has its start");

    // 3. Released, straight back along the tool's axis; the tool may touch
    // the object it leaves.
    let mut retreat_scene = descent_scene;
    retreat_scene
        .detach(object, &q_place)
        .map_err(scene_error)?;
    for link in hand.links() {
        retreat_scene.allowed_mut().allow(object, link);
    }
    let backed = place_tool * na::Translation3::new(0.0, 0.0, -options.retreat);
    let retreat = straight(&retreat_scene, q_place, backed);
    if let Some(stop) = retreat.stop {
        return Err(PlaceFailure::Retreat { stop, fraction: retreat.fraction });
    }
    Ok((to_preplace, descent.waypoints, retreat.waypoints))
}
