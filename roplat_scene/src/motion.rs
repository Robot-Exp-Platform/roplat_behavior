//! Checking a straight joint motion as a whole, not at samples.
//!
//! The planners in `roplat_trajectory` check a segment between two
//! configurations by sampling it every `resolution` radians. Between two
//! samples a link sweeps a band as wide as the step times its distance from
//! the joints that move it, and a thin plate or a corner inside that band is
//! missed. [`PlanningScene::motion_contact`] certifies the whole motion
//! instead, by conservative advancement (Schwarzer, Saha and Latombe,
//! "Adaptive dynamic collision checking for single and multiple articulated
//! robots in complex environments", IEEE T-RO 21(3), 2005):
//!
//! - Along the motion `q(s) = from + s · (to − from)`, no point of a body
//!   moves faster than a bound found from the robot's geometry: for each
//!   moving joint, the step of that joint times the farthest the body can
//!   reach from the joint's axis. A pair's distance shrinks no faster than
//!   the sum of the two bounds, counting only joints below the pair's
//!   common ancestor, since joints above it move both bodies together.
//! - At `s` each pair that might still meet is measured; the motion then
//!   advances by the smallest clearance over its pair's bound, which no pair
//!   can use up. Pairs that cannot meet before the end, by distance or by
//!   their bounding boxes, are dropped.
//!
//! The steps are as long as the clearance allows: far from everything a
//! motion is certified in one or two measurements, near an obstacle the
//! steps shrink with the clearance. A pair whose clearance above the
//! padding falls to [`MOTION_TOLERANCE`] counts as touching, which ends the
//! advance where a motion grazes an obstacle.
//!
//! The certificate is only as good as the distances: an advance by a
//! distance measured too large by `e` can carry a pair up to `e` into its
//! padding between two measurements. Pairs with a ball are measured in
//! closed form, the rest by GJK, exact to rounding between polyhedra and up
//! to 3.3e-5 m too far against a capsule or a cylinder in the tests, a
//! third of [`MOTION_TOLERANCE`]: a motion may graze that deep, and cannot
//! pass through anything thicker. parry 0.25's own routine for two boxes,
//! which can overestimate by centimetres, is not used.

use nalgebra as na;
use parry3d_f64::{bounding_volume::Aabb, query, shape::SharedShape};
use roplat_description::JointKind;

use crate::{Body, CollisionCheck, Contact, Mount, PlanningScene, distance};

/// Clearance above the padding (m) at which a motion counts as touching.
pub const MOTION_TOLERANCE: f64 = 1e-4;

/// Largest joint step (rad or m) between the configurations
/// [`PlanningScene::motion_is_free`] checks before certifying a motion.
pub const REJECT_STEP: f64 = 0.05;

/// Where a straight joint motion first comes into contact.
#[derive(Clone, Debug, PartialEq)]
pub struct MotionContact {
    /// Fraction of the motion, in `[0, 1]`, of the first configuration
    /// found within [`MOTION_TOLERANCE`] of the padding. Every
    /// configuration before it is free.
    pub fraction: f64,
    /// The pair there, and their signed distance.
    pub contact: Contact,
}

impl<const N: usize> PlanningScene<N> {
    /// The first contact along the straight joint motion from `from` to
    /// `to`, or `None` when the whole motion is free; see the
    /// [module](crate::motion) docs. The configuration at
    /// [`fraction`](MotionContact::fraction) comes within
    /// [`MOTION_TOLERANCE`] of the padding, and all before it are free, so
    /// no contact is stepped over. A motion from a configuration to itself
    /// checks that configuration.
    pub fn motion_contact(&self, from: &[f64; N], to: &[f64; N]) -> Option<MotionContact> {
        let chains = self.chains(from, to);
        let mut bodies = self.bodies(from);
        let mut pairs = Vec::new();
        for i in 0..bodies.len() {
            for j in i + 1..bodies.len() {
                if self.checked(&bodies[i], &bodies[j]) {
                    pairs.push((i, j, pair_rate(&chains[i], &chains[j])));
                }
            }
        }
        let mut s = 0.0;
        loop {
            let remaining = 1.0 - s;
            let mut step = remaining;
            let mut near = Vec::new();
            for &(i, j, rate) in &pairs {
                let (a, b) = (&bodies[i], &bodies[j]);
                // How far the pair may close before the end.
                let reach = rate * remaining;
                let Some(distance) = pair_distance(a, b, reach) else {
                    continue;
                };
                let clearance = distance - self.pair_padding(a.name, b.name);
                if clearance <= MOTION_TOLERANCE {
                    let distance = if distance > 0.0 {
                        distance
                    } else {
                        penetration(a, b)
                    };
                    return Some(MotionContact {
                        fraction: s,
                        contact: Contact { first: a.name.into(), second: b.name.into(), distance },
                    });
                }
                if clearance > reach {
                    continue;
                }
                step = step.min(clearance / rate);
                near.push((i, j, rate));
            }
            if near.is_empty() {
                return None;
            }
            pairs = near;
            s = (s + step).min(1.0);
            bodies = self.bodies(&lerp(from, to, s));
        }
    }

    /// Whether the straight joint motion from `from` to `to` is free; see
    /// [`motion_contact`](Self::motion_contact). The check to give
    /// `plan_joint_motion` in `roplat_trajectory`.
    ///
    /// Most motions a planner tries end in an obstacle or pass through one,
    /// and advancing towards a contact takes many short steps. So the end,
    /// then configurations [`REJECT_STEP`] apart, are checked first, and
    /// only a motion they all pass is certified.
    pub fn motion_is_free(&self, from: &[f64; N], to: &[f64; N]) -> bool {
        let longest = (0..N).map(|k| (to[k] - from[k]).abs()).fold(0.0, f64::max);
        let samples = (longest / REJECT_STEP).ceil() as usize;
        (1..=samples)
            .rev()
            .all(|k| self.is_free(&lerp(from, to, k as f64 / samples as f64)))
            && self.motion_contact(from, to).is_none()
    }

    /// For each body, in the order of [`bodies`](Self::bodies), the joints
    /// from its link up to the root: each joint's child link, and how fast
    /// the joint moves the body's points along the motion (m per unit of
    /// `s`). Bodies in the world have none.
    fn chains(&self, from: &[f64; N], to: &[f64; N]) -> Vec<Vec<(usize, f64)>> {
        let mut into = vec![None; self.model.links.len()];
        for (j, joint) in self.model.joints.iter().enumerate() {
            into[joint.child] = Some(j);
        }
        let mut chains = Vec::new();
        for (link, parts) in self.links.iter().enumerate() {
            if !parts.is_empty() {
                let radius = parts
                    .iter()
                    .map(|part| radius(&part.at, &part.shape))
                    .fold(0.0, f64::max);
                chains.push(self.chain(&into, link, radius, from, to));
            }
        }
        for object in &self.objects {
            chains.push(match &object.mount {
                Mount::World(_) => Vec::new(),
                Mount::Link { link, at, .. } => {
                    let radius = radius(&(at * object.part.at), &object.part.shape);
                    self.chain(&into, *link, radius, from, to)
                }
            });
        }
        chains
    }

    /// The joints above `link` for a body reaching `radius` from the link's
    /// origin. A revolute joint's axis passes through its child's origin,
    /// so it moves the body at most its step times the body's reach from
    /// there; the reach then grows by the joint's offset from its parent.
    fn chain(
        &self,
        into: &[Option<usize>],
        mut link: usize,
        radius: f64,
        from: &[f64; N],
        to: &[f64; N],
    ) -> Vec<(usize, f64)> {
        let mut reach = radius;
        let mut chain = Vec::new();
        while let Some(j) = into[link] {
            let joint = &self.model.joints[j];
            let group = self.group.iter().position(|&child| child == joint.child);
            let step = group.map(|k| (to[k] - from[k]).abs());
            let (rate, travel) = match joint.kind {
                JointKind::Revolute | JointKind::Continuous => (step.unwrap_or(0.0) * reach, 0.0),
                // A unit axis: the body moves as far as the joint.
                JointKind::Prismatic => match group {
                    Some(k) => (step.unwrap_or(0.0), from[k].abs().max(to[k].abs())),
                    None => (0.0, self.positions[joint.child].abs()),
                },
                _ => (0.0, 0.0),
            };
            chain.push((link, rate));
            reach += joint.origin.quat().translation.vector.norm() + travel;
            link = joint.parent;
        }
        chain
    }
}

fn lerp<const N: usize>(from: &[f64; N], to: &[f64; N], s: f64) -> [f64; N] {
    std::array::from_fn(|k| from[k] + s * (to[k] - from[k]))
}

/// How fast the distance between two bodies can shrink: the joints that
/// move one body but not the other. Joints both chains share (their common
/// tail, up from the common ancestor) move the pair together.
fn pair_rate(a: &[(usize, f64)], b: &[(usize, f64)]) -> f64 {
    let shared = a
        .iter()
        .rev()
        .zip(b.iter().rev())
        .take_while(|(x, y)| x.0 == y.0)
        .count();
    let own = |chain: &[(usize, f64)]| -> f64 {
        chain[..chain.len() - shared]
            .iter()
            .map(|&(_, rate)| rate)
            .sum()
    };
    own(a) + own(b)
}

/// Farthest a shape posed by `at` reaches from the origin of that frame.
fn radius(at: &na::Isometry3<f64>, shape: &SharedShape) -> f64 {
    let sphere = shape.compute_local_bounding_sphere();
    (at * sphere.center()).coords.norm() + sphere.radius()
}

/// Smallest distance between the parts of `a` and `b` that could still come
/// within their padding after closing by `reach`; `None` when none could.
/// The bounds are grown by half the widest padding each, so a gap between
/// them beyond `reach` leaves the parts farther apart than their padding
/// plus reach.
fn pair_distance(a: &Body<'_>, b: &Body<'_>, reach: f64) -> Option<f64> {
    let mut nearest: Option<f64> = None;
    for ((pa, sa), ba) in a.parts.iter().zip(&a.bounds) {
        for ((pb, sb), bb) in b.parts.iter().zip(&b.bounds) {
            if gap(ba, bb) > reach {
                continue;
            }
            let distance = distance::distance(pa, sa.as_ref(), pb, sb.as_ref());
            nearest = Some(nearest.map_or(distance, |nearest| nearest.min(distance)));
        }
    }
    nearest
}

/// Signed distance of overlapping bodies: the deepest contact's depth.
fn penetration(a: &Body<'_>, b: &Body<'_>) -> f64 {
    let mut deepest: f64 = 0.0;
    for (pa, sa) in &a.parts {
        for (pb, sb) in &b.parts {
            if let Ok(Some(contact)) = query::contact(pa, sa.as_ref(), pb, sb.as_ref(), 0.0) {
                deepest = deepest.min(contact.dist);
            }
        }
    }
    deepest
}

/// Distance between two boxes, zero when they overlap.
fn gap(a: &Aabb, b: &Aabb) -> f64 {
    let axis = |k: usize| (b.mins[k] - a.maxs[k]).max(a.mins[k] - b.maxs[k]).max(0.0);
    na::Vector3::new(axis(0), axis(1), axis(2)).norm()
}
