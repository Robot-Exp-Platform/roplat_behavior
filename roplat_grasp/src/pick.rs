//! Planning a pick: pre-grasp, approach, lift.

use nalgebra as na;
use roplat_scene::{CollisionCheck, EndEffector, PlanningScene};
use roplat_trajectory::{
    CartesianPathOptions, CartesianStop, PlanError, PlannerOptions, plan_cartesian_motion,
    plan_cartesian_path, plan_joint_motion, plan_joint_path,
};

use crate::Grasp;

/// Settings for [`plan_pick`].
#[derive(Clone, Copy, Debug)]
pub struct PickOptions {
    /// How far back along the approach the straight approach starts (m).
    pub pregrasp_distance: f64,
    /// The tool's straight motion after the fingers close, in the world
    /// (m); usually up.
    pub lift: na::Vector3<f64>,
    /// Search for the free path to the pre-grasp pose.
    pub planner: PlannerOptions,
    /// Steps of the straight approach and lift.
    pub cartesian: CartesianPathOptions,
}

impl Default for PickOptions {
    fn default() -> Self {
        Self {
            pregrasp_distance: 0.1,
            lift: na::Vector3::new(0.0, 0.0, 0.1),
            planner: PlannerOptions::default(),
            cartesian: CartesianPathOptions::default(),
        }
    }
}

/// A pick that works: the candidate used and the three joint paths, each
/// starting where the previous one ended.
#[derive(Clone, Debug, PartialEq)]
pub struct PickPlan<const N: usize> {
    pub grasp: Grasp,
    /// Index of [`grasp`](Self::grasp) in the candidates given.
    pub candidate: usize,
    /// From the start to the pre-grasp pose, avoiding the object.
    pub to_pregrasp: Vec<[f64; N]>,
    /// Straight in to the grasp; the tool's links may touch the object.
    pub approach: Vec<[f64; N]>,
    /// Straight out with the object attached; the object may touch its
    /// support.
    pub lift: Vec<[f64; N]>,
}

impl<const N: usize> PickPlan<N> {
    /// The configuration in which to close the fingers.
    pub fn grasp_configuration(&self) -> [f64; N] {
        *self
            .approach
            .last()
            .expect("an approach has at least its start")
    }
}

/// Why one candidate did not work.
#[derive(Clone, Debug, PartialEq)]
pub enum PickFailure {
    /// The inverse kinematics found no configuration for the pre-grasp.
    PregraspUnreachable,
    /// The pre-grasp configuration collides or is outside the bounds.
    PregraspInvalid,
    /// No free path from the start to the pre-grasp configuration.
    Path(PlanError),
    /// The straight approach stopped after `fraction` of the way.
    Approach { stop: CartesianStop, fraction: f64 },
    /// The straight lift stopped after `fraction` of the way.
    Lift { stop: CartesianStop, fraction: f64 },
    /// The scene refused a name (object, link or support).
    Scene(String),
}

/// No candidate worked; `attempts` holds each candidate tried and why it
/// failed, in order. Empty when there were no candidates.
#[derive(Clone, Debug, PartialEq)]
pub struct PickError {
    pub attempts: Vec<(usize, PickFailure)>,
}

impl std::fmt::Display for PickError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.attempts.is_empty() {
            return write!(f, "no grasp candidates");
        }
        write!(f, "none of {} grasp candidates works:", self.attempts.len())?;
        for (k, why) in &self.attempts {
            write!(f, " #{k} {why:?};")?;
        }
        Ok(())
    }
}

impl std::error::Error for PickError {}

/// Plans picking `object` from `start` with the first of `grasps` that
/// works; see the [crate documentation](crate).
///
/// - `hand` places the tool on its link; its links may touch the object
///   during the approach, and carry it during the lift.
/// - `support`, if given, is the object or link the object rests on, which
///   it may touch during the lift.
/// - `inverse(link_pose, seed)` returns a configuration putting `hand`'s
///   link at `link_pose` in the world, near `seed`, or `None`; with
///   `roplat_kinematics` that is `chain_ik` or `chain_ik_solutions` on the
///   chain to the link.
/// - Every configuration stays within `lower..=upper`.
///
/// The scene is not changed: attaching the object for the lift happens on
/// a copy. After executing the plan, attach the object in the caller's own
/// scene with [`PlanningScene::attach`] at
/// [`grasp_configuration`](PickPlan::grasp_configuration).
#[allow(
    clippy::too_many_arguments,
    reason = "the pick planner's inputs are independent: scene, hand, object, support, grasps, start, joint range, inverse kinematics and options; grouping them into a planning context is the later form (style ruling 10)"
)]
pub fn plan_pick<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    grasps: &[Grasp],
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: PickOptions,
) -> Result<PickPlan<N>, PickError> {
    pick(
        scene,
        hand,
        object,
        support,
        grasps,
        start,
        lower,
        upper,
        inverse,
        options,
        Check::Sampled,
    )
}

/// As [`plan_pick`], with every segment checked whole by
/// [`PlanningScene::motion_is_free`] instead of sampled every
/// `resolution` radians: the free path, the approach and the lift (see
/// `roplat_scene::motion`). A thin obstacle between two samples, such as a
/// shelf edge, is not stepped over. The `resolution` of `options` is not
/// used.
#[allow(
    clippy::too_many_arguments,
    reason = "the pick motion planner's inputs are independent: scene, hand, object, support, grasps, start, joint range, inverse kinematics and options; grouping them into a planning context is the later form (style ruling 10)"
)]
pub fn plan_pick_motion<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    grasps: &[Grasp],
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: PickOptions,
) -> Result<PickPlan<N>, PickError> {
    pick(
        scene,
        hand,
        object,
        support,
        grasps,
        start,
        lower,
        upper,
        inverse,
        options,
        Check::Whole,
    )
}

/// How segments are checked.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Check {
    /// At samples `resolution` apart.
    Sampled,
    /// As whole motions.
    Whole,
}

#[allow(
    clippy::too_many_arguments,
    reason = "internal step taking the public planner's inputs unchanged"
)]
fn pick<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    grasps: &[Grasp],
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    mut inverse: impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: PickOptions,
    check: Check,
) -> Result<PickPlan<N>, PickError> {
    let inside = |q: &[f64; N]| (0..N).all(|i| (lower[i]..=upper[i]).contains(&q[i]));
    let mut attempts = Vec::new();
    for (candidate, grasp) in grasps.iter().enumerate() {
        match try_grasp(
            scene,
            hand,
            object,
            support,
            grasp,
            start,
            lower,
            upper,
            &inside,
            &mut inverse,
            &options,
            check,
        ) {
            Ok((to_pregrasp, approach, lift)) => {
                return Ok(PickPlan { grasp: *grasp, candidate, to_pregrasp, approach, lift });
            }
            Err(why) => attempts.push((candidate, why)),
        }
    }
    Err(PickError { attempts })
}

type Paths<const N: usize> = (Vec<[f64; N]>, Vec<[f64; N]>, Vec<[f64; N]>);

#[allow(
    clippy::too_many_arguments,
    reason = "internal step taking the public planner's inputs unchanged"
)]
fn try_grasp<const N: usize>(
    scene: &PlanningScene<N>,
    hand: &EndEffector,
    object: &str,
    support: Option<&str>,
    grasp: &Grasp,
    start: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    inside: &impl Fn(&[f64; N]) -> bool,
    inverse: &mut impl FnMut(&na::Isometry3<f64>, &[f64; N]) -> Option<[f64; N]>,
    options: &PickOptions,
    check: Check,
) -> Result<Paths<N>, PickFailure> {
    let scene_error = |e: roplat_scene::SceneError| PickFailure::Scene(e.to_string());

    // 1. A free path to the pre-grasp pose.
    let pregrasp = grasp.pregrasp(options.pregrasp_distance);
    let q_pre =
        inverse(&hand.link_target(&pregrasp), &start).ok_or(PickFailure::PregraspUnreachable)?;
    if !inside(&q_pre) || !scene.is_free(&q_pre) {
        return Err(PickFailure::PregraspInvalid);
    }
    let to_pregrasp = match check {
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
    .map_err(PickFailure::Path)?;

    // 2. Straight in; the tool may touch the object.
    let mut approach_scene = scene.clone();
    for link in hand.links() {
        approach_scene.allowed_mut().allow(object, link);
    }
    let forward = |scene: &PlanningScene<N>, q: &[f64; N]| {
        hand.pose(scene, q)
            .expect("the hand's link is in the scene")
    };
    hand.pose(scene, &start).map_err(scene_error)?;
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
    let approach = straight(&approach_scene, q_pre, grasp.pose);
    if let Some(stop) = approach.stop {
        return Err(PickFailure::Approach { stop, fraction: approach.fraction });
    }
    let q_grasp = *approach
        .waypoints
        .last()
        .expect("a complete path has its start");

    // 3. Straight out with the object attached; it may touch its support.
    let mut lift_scene = approach_scene;
    lift_scene
        .attach(object, hand.link(), &hand.links(), &q_grasp)
        .map_err(scene_error)?;
    if let Some(support) = support {
        lift_scene.allowed_mut().allow(object, support);
    }
    let lifted = na::Translation3::from(options.lift) * grasp.pose;
    let lift = straight(&lift_scene, q_grasp, lifted);
    if let Some(stop) = lift.stop {
        return Err(PickFailure::Lift { stop, fraction: lift.fraction });
    }
    Ok((to_pregrasp, approach.waypoints, lift.waypoints))
}
