//! Collision-free joint-space paths.
//!
//! [`plan_joint_path`] runs RRT-Connect between two configurations inside
//! joint bounds, then shortcuts the result. Collision checking is the
//! caller's: `is_valid` answers for one configuration (a simulator's
//! distance query, primitive geometry, a workspace limit), and edges are
//! checked by sampling it every `resolution` radians. The sampler is a seeded
//! xorshift, so a seed always gives the same path. The waypoints can be timed
//! with [`plan_waypoints_traj_via_copp`](crate::plan_waypoints_traj_via_copp).
//!
//! Sampling can step over an obstacle thinner than what a link sweeps
//! between two samples. A step of `resolution` in every joint moves a point
//! of the robot by up to `resolution` times the sum of the joints' reaches
//! (the farthest that point lies from each moving joint's axis): 0.02 rad
//! moves the tip of a planar arm with two 0.6 m links by up to 3.6 cm,
//! past a 2 cm wall. [`resolution_for_travel`] picks the resolution from the scene's
//! scale instead: the travel allowed between two checks, no more than the
//! thinnest obstacle, and the robot's reaches. [`plan_joint_motion`] runs
//! the same search with a check that answers for a whole straight motion,
//! such as `PlanningScene::motion_is_free` in `roplat_scene`, which leaves no
//! resolution to choose.

/// Search settings for [`plan_joint_path`].
#[derive(Clone, Copy, Debug)]
pub struct PlannerOptions {
    /// Longest tree edge, as the largest per-joint step (rad).
    pub max_step: f64,
    /// Largest per-joint gap between configurations checked along an edge
    /// (rad, or m for a prismatic joint). The default, 0.02, comes from no
    /// scene; choose it with [`resolution_for_travel`].
    pub resolution: f64,
    /// Tree extensions before giving up.
    pub max_iterations: usize,
    /// Random shortcut attempts on the found path.
    pub shortcut_attempts: usize,
    pub seed: u64,
}

impl Default for PlannerOptions {
    fn default() -> Self {
        Self {
            max_step: 0.2,
            resolution: 0.02,
            max_iterations: 5000,
            shortcut_attempts: 200,
            seed: 1,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlanError {
    /// The start is invalid or outside the bounds.
    InvalidStart,
    /// The goal is invalid or outside the bounds.
    InvalidGoal,
    /// No path within the iteration budget.
    NotFound { iterations: usize },
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidStart => write!(f, "the start configuration is invalid"),
            Self::InvalidGoal => write!(f, "the goal configuration is invalid"),
            Self::NotFound { iterations } => write!(f, "no path found in {iterations} iterations"),
        }
    }
}

impl std::error::Error for PlanError {}

/// Waypoints from `start` to `goal`, both included, with every straight
/// segment between consecutive waypoints valid at `options.resolution`.
///
/// Only the sampled configurations are checked. With the resolution from
/// [`resolution_for_travel`], no point of the robot moves farther than the
/// travel between two of them: an obstacle thicker than the travel cannot
/// be crossed, and a contact between two samples goes no deeper than half
/// the travel. Nothing is guaranteed between samples at any other
/// resolution.
pub fn plan_joint_path<const N: usize>(
    start: [f64; N],
    goal: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
    options: PlannerOptions,
) -> Result<Vec<[f64; N]>, PlanError> {
    check_ends(&start, &goal, &lower, &upper, &mut is_valid)?;
    connect(
        start,
        goal,
        lower,
        upper,
        |a, b| segment_is_valid(a, b, options.resolution, &mut is_valid),
        options,
    )
}

/// As [`plan_joint_path`], with each straight segment checked whole by
/// `motion_is_valid(from, to)` instead of sampled; a configuration is
/// checked as the motion from it to itself. `options.resolution` is not
/// used. Every segment between consecutive waypoints of the result passed
/// the check.
pub fn plan_joint_motion<const N: usize>(
    start: [f64; N],
    goal: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    mut motion_is_valid: impl FnMut(&[f64; N], &[f64; N]) -> bool,
    options: PlannerOptions,
) -> Result<Vec<[f64; N]>, PlanError> {
    check_ends(&start, &goal, &lower, &upper, |q| motion_is_valid(q, q))?;
    connect(start, goal, lower, upper, motion_is_valid, options)
}

/// Refuses a start or goal outside the bounds or invalid.
fn check_ends<const N: usize>(
    start: &[f64; N],
    goal: &[f64; N],
    lower: &[f64; N],
    upper: &[f64; N],
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
) -> Result<(), PlanError> {
    let inside = |q: &[f64; N]| (0..N).all(|i| (lower[i]..=upper[i]).contains(&q[i]));
    if !inside(start) || !is_valid(start) {
        return Err(PlanError::InvalidStart);
    }
    if !inside(goal) || !is_valid(goal) {
        return Err(PlanError::InvalidGoal);
    }
    Ok(())
}

/// RRT-Connect and shortcutting between valid ends, with `segment` deciding
/// whether a straight segment is valid.
fn connect<const N: usize>(
    start: [f64; N],
    goal: [f64; N],
    lower: [f64; N],
    upper: [f64; N],
    mut segment: impl FnMut(&[f64; N], &[f64; N]) -> bool,
    options: PlannerOptions,
) -> Result<Vec<[f64; N]>, PlanError> {
    if segment(&start, &goal) {
        return Ok(vec![start, goal]);
    }

    let mut rng = XorShift(options.seed.max(1));
    // Each tree is a list of (configuration, parent index).
    let mut trees = [vec![(start, usize::MAX)], vec![(goal, usize::MAX)]];
    for iteration in 0..options.max_iterations {
        let (a, b) = if iteration % 2 == 0 { (0, 1) } else { (1, 0) };
        let sample: [f64; N] =
            std::array::from_fn(|i| lower[i] + rng.unit() * (upper[i] - lower[i]));
        let Some(new) = extend(&mut trees[a], &sample, &options, &mut segment) else {
            continue;
        };
        // Greedily grow the other tree towards the new node.
        let target = trees[a][new].0;
        let mut reached = None;
        while let Some(node) = extend(&mut trees[b], &target, &options, &mut segment) {
            if distance(&trees[b][node].0, &target) == 0.0 {
                reached = Some(node);
                break;
            }
        }
        if let Some(meet) = reached {
            // Root of `a` to the new node, then on from the meeting node
            // (the same configuration) to the root of `b`.
            let mut path = branch(&trees[a], new);
            path.reverse();
            path.extend(branch(&trees[b], meet).into_iter().skip(1));
            if a == 1 {
                path.reverse();
            }
            shortcut(&mut path, &options, &mut rng, &mut segment);
            return Ok(path);
        }
    }
    Err(PlanError::NotFound { iterations: options.max_iterations })
}

/// Whether the straight segment from `a` to `b` stays valid, checked at
/// most `resolution` apart per joint.
pub fn segment_is_valid<const N: usize>(
    a: &[f64; N],
    b: &[f64; N],
    resolution: f64,
    mut is_valid: impl FnMut(&[f64; N]) -> bool,
) -> bool {
    let steps = (distance(a, b) / resolution).ceil().max(1.0) as usize;
    (1..=steps).all(|k| {
        let t = k as f64 / steps as f64;
        is_valid(&std::array::from_fn(|i| a[i] + t * (b[i] - a[i])))
    })
}

/// The [`resolution`](PlannerOptions::resolution) at which no point of a
/// robot moves farther than `travel` (m) between two configurations checked
/// along a segment, so that no obstacle thicker than `travel` is stepped
/// over.
///
/// `reach[i]` bounds how far any point that joint `i` moves can lie from
/// that joint's axis (m), or is 1 for a prismatic joint. Turning joint `i`
/// by `dq` moves such a point by at most `reach[i] · |dq|`, and the joints'
/// motions add, so a step of `resolution` in every joint moves no point
/// farther than `resolution · Σ reach`.
///
/// # Panics
///
/// If `travel` is not positive and finite, or a reach is negative or not
/// finite, or the reaches add to zero.
pub fn resolution_for_travel<const N: usize>(travel: f64, reach: &[f64; N]) -> f64 {
    assert!(
        travel.is_finite() && travel > 0.0,
        "travel must be positive and finite"
    );
    assert!(
        reach.iter().all(|r| r.is_finite() && *r >= 0.0),
        "reaches must be finite and nonnegative"
    );
    let total: f64 = reach.iter().sum();
    assert!(total > 0.0, "the reaches must not add to zero");
    travel / total
}

/// Largest per-joint difference.
fn distance<const N: usize>(a: &[f64; N], b: &[f64; N]) -> f64 {
    (0..N).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

/// Adds a node one step from the nearest node towards `target`, if that
/// edge is valid; returns its index.
fn extend<const N: usize>(
    tree: &mut Vec<([f64; N], usize)>,
    target: &[f64; N],
    options: &PlannerOptions,
    segment: &mut impl FnMut(&[f64; N], &[f64; N]) -> bool,
) -> Option<usize> {
    let (nearest, gap) = tree
        .iter()
        .enumerate()
        .map(|(i, (q, _))| (i, distance(q, target)))
        .min_by(|x, y| x.1.total_cmp(&y.1))?;
    if gap == 0.0 {
        return None;
    }
    let from = tree[nearest].0;
    let t = (options.max_step / gap).min(1.0);
    let to: [f64; N] = std::array::from_fn(|i| from[i] + t * (target[i] - from[i]));
    if !segment(&from, &to) {
        return None;
    }
    tree.push((to, nearest));
    Some(tree.len() - 1)
}

/// Configurations from `node` back to the tree's root.
fn branch<const N: usize>(tree: &[([f64; N], usize)], mut node: usize) -> Vec<[f64; N]> {
    let mut path = Vec::new();
    while node != usize::MAX {
        path.push(tree[node].0);
        node = tree[node].1;
    }
    path
}

fn shortcut<const N: usize>(
    path: &mut Vec<[f64; N]>,
    options: &PlannerOptions,
    rng: &mut XorShift,
    segment: &mut impl FnMut(&[f64; N], &[f64; N]) -> bool,
) {
    for _ in 0..options.shortcut_attempts {
        if path.len() < 3 {
            return;
        }
        let i = (rng.unit() * (path.len() - 2) as f64) as usize;
        let j = i + 2 + (rng.unit() * (path.len() - i - 2) as f64) as usize;
        let j = j.min(path.len() - 1);
        if segment(&path[i], &path[j]) {
            path.drain(i + 1..j);
        }
    }
}

/// xorshift64*: small, seedable, the same everywhere.
struct XorShift(u64);

impl XorShift {
    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }
}
