//! Topology, static transforms and the shape of each time history.

use std::time::Duration;

use nalgebra as na;
use roplat_description::{Joint, JointKind, RobotModel};
use roplat_msgs::{FrameId, Pose};

use crate::{FrameError, TreeError, frame_id};

/// How many samples a joint group or dynamic transform keeps, and how a
/// lookup treats instants they do not cover.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct History {
    pub(crate) capacity: usize,
    pub(crate) hold: Duration,
    pub(crate) max_gap: Option<Duration>,
}

impl History {
    /// The latest `capacity` samples; no hold and no gap bound.
    pub const fn new(capacity: usize) -> Self {
        Self { capacity, hold: Duration::ZERO, max_gap: None }
    }

    /// A lookup up to `hold` after the latest sample, or before the earliest,
    /// uses that sample. Lookups never extrapolate.
    pub const fn hold(mut self, hold: Duration) -> Self {
        self.hold = hold;
        self
    }

    /// A lookup between two samples further apart than `max_gap` fails
    /// instead of interpolating across the gap.
    pub const fn max_gap(mut self, max_gap: Duration) -> Self {
        self.max_gap = Some(max_gap);
        self
    }
}

/// A joint group of `N` joints, issued by [`FrameTree::group`] and valid for
/// buffers of that tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct JointGroup<const N: usize> {
    /// Index of the group, which is also its track.
    pub(crate) index: usize,
}

/// One edge of a [`FrameTree`]: `child` placed in `parent`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameEdge<'a> {
    pub parent: FrameId,
    pub child: FrameId,
    pub kind: EdgeKind<'a>,
}

/// How an edge places its child, as the tree was declared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeKind<'a> {
    /// A pose fixed when the tree was built, and replaced only by
    /// [`FrameTree::set_fixed`]: a [`fixed`](FrameTreeBuilder::fixed)
    /// declaration, or a model's fixed joint.
    Fixed,
    /// A movable joint of a model, positioned by the joint group listing
    /// it, `None` when no group does.
    Joint {
        joint: &'a str,
        kind: JointKind,
        group: Option<&'a str>,
    },
    /// A transform written at run time, see
    /// [`FrameTreeBuilder::dynamic`].
    Dynamic,
}

#[derive(Clone, Debug)]
pub(crate) enum Edge {
    Root,
    Fixed(na::Isometry3<f64>),
    /// `slot` is the joint's (track, position index), if a group lists it.
    Joint {
        joint: Joint,
        slot: Option<(usize, usize)>,
    },
    Dynamic(usize),
}

#[derive(Clone, Debug)]
pub(crate) struct Frame {
    pub(crate) name: String,
    pub(crate) id: FrameId,
    pub(crate) parent: usize,
    pub(crate) depth: usize,
    /// Transform of this frame in its parent.
    pub(crate) edge: Edge,
}

/// Shape of one time history: `stride` values per sample.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TrackSpec {
    pub(crate) stride: usize,
    pub(crate) history: History,
}

/// Every frame of a cell and how each is placed in its parent.
///
/// The topology and the static transforms are fixed when the tree is built.
/// There is exactly one root, so every two frames are connected. Lookups walk
/// only up to the frames' common ancestor and do not allocate.
#[derive(Clone, Debug)]
pub struct FrameTree {
    pub(crate) frames: Box<[Frame]>,
    by_id: Box<[(FrameId, usize)]>,
    root: usize,
    /// Group `i` has track `i`; dynamic transforms follow.
    groups: Box<[(String, usize)]>,
    pub(crate) tracks: Box<[TrackSpec]>,
}

impl FrameTree {
    pub fn builder() -> FrameTreeBuilder {
        FrameTreeBuilder::default()
    }

    pub fn root(&self) -> FrameId {
        self.frames[self.root].id
    }

    pub fn contains(&self, frame: FrameId) -> bool {
        self.index(frame).is_ok()
    }

    pub fn name(&self, frame: FrameId) -> Option<&str> {
        let index = self.index(frame).ok()?;
        Some(&self.frames[index].name)
    }

    /// `None` for the root and for frames not in the tree.
    pub fn parent(&self, frame: FrameId) -> Option<FrameId> {
        let index = self.index(frame).ok()?;
        (index != self.root).then(|| self.frames[self.frames[index].parent].id)
    }

    /// Ids and names, in declaration order.
    pub fn frames(&self) -> impl Iterator<Item = (FrameId, &str)> {
        self.frames.iter().map(|f| (f.id, f.name.as_str()))
    }

    pub fn group<const N: usize>(&self, name: &str) -> Result<JointGroup<N>, TreeError> {
        let index = self
            .groups
            .iter()
            .position(|(group, _)| group == name)
            .ok_or_else(|| TreeError::UnknownGroup(name.into()))?;
        let len = self.groups[index].1;
        if len != N {
            return Err(TreeError::GroupSize { group: name.into(), len, requested: N });
        }
        Ok(JointGroup { index })
    }

    /// Every edge of the tree, one per frame but the root, in declaration
    /// order of the child frames: which frame places which, and how.
    pub fn edges(&self) -> impl Iterator<Item = FrameEdge<'_>> {
        (0..self.frames.len()).filter_map(|index| self.edge_of(index))
    }

    /// The edge that places `child` in its parent; `None` for the root and
    /// for frames not in the tree.
    pub fn edge(&self, child: FrameId) -> Option<FrameEdge<'_>> {
        self.edge_of(self.index(child).ok()?)
    }

    fn edge_of(&self, index: usize) -> Option<FrameEdge<'_>> {
        let frame = &self.frames[index];
        let kind = match &frame.edge {
            Edge::Root => return None,
            Edge::Fixed(_) => EdgeKind::Fixed,
            Edge::Joint { joint, slot } => EdgeKind::Joint {
                joint: &joint.name,
                kind: joint.kind,
                group: slot.map(|(track, _)| self.groups[track].0.as_str()),
            },
            Edge::Dynamic(_) => EdgeKind::Dynamic,
        };
        Some(FrameEdge {
            parent: self.frames[frame.parent].id,
            child: frame.id,
            kind,
        })
    }

    /// Replaces the pose of `child` in its parent, which a fixed edge
    /// places: a [`fixed`](FrameTreeBuilder::fixed) declaration or a model's
    /// fixed joint. Every lookup from then on uses the new pose, at any
    /// stamp, since fixed edges keep no history; to change a transform over
    /// time, declare it [dynamic](FrameTreeBuilder::dynamic).
    ///
    /// [`NotStatic`](FrameError::NotStatic) for the root and for a frame a
    /// joint or a dynamic transform places, and
    /// [`InvalidValue`](FrameError::InvalidValue) for a pose with a
    /// non-finite component or a zero rotation; the tree is unchanged then.
    pub fn set_fixed(&mut self, child: FrameId, pose: &Pose) -> Result<(), FrameError> {
        let index = self.index(child)?;
        let frame = &mut self.frames[index];
        let Edge::Fixed(iso) = &mut frame.edge else {
            return Err(FrameError::NotStatic(child));
        };
        *iso = isometry(pose).ok_or(FrameError::InvalidValue)?;
        Ok(())
    }

    /// Pose of `child` in `parent` through fixed transforms only.
    pub fn lookup_static(&self, parent: FrameId, child: FrameId) -> Result<Pose, FrameError> {
        self.relative(parent, child, |frame| match frame.edge {
            Edge::Fixed(iso) => Ok(iso),
            _ => Err(FrameError::NotStatic(frame.id)),
        })
        .map(Pose::from_isometry)
    }

    pub(crate) fn index(&self, frame: FrameId) -> Result<usize, FrameError> {
        self.by_id
            .binary_search_by_key(&frame, |&(id, _)| id)
            .map(|at| self.by_id[at].1)
            .map_err(|_| FrameError::UnknownFrame(frame))
    }

    pub(crate) fn is_group(&self, index: usize, len: usize) -> bool {
        self.groups.get(index).is_some_and(|&(_, n)| n == len)
    }

    /// Pose of `child` in `parent`, given each frame's pose in its parent.
    pub(crate) fn relative(
        &self,
        parent: FrameId,
        child: FrameId,
        mut edge: impl FnMut(&Frame) -> Result<na::Isometry3<f64>, FrameError>,
    ) -> Result<na::Isometry3<f64>, FrameError> {
        let (mut a, mut b) = (self.index(parent)?, self.index(child)?);
        // Poses of `parent` and `child` in their common ancestor.
        let (mut in_a, mut in_b) = (na::Isometry3::identity(), na::Isometry3::identity());
        while a != b {
            let (fa, fb) = (&self.frames[a], &self.frames[b]);
            if fa.depth >= fb.depth {
                in_a = edge(fa)? * in_a;
                a = fa.parent;
            } else {
                in_b = edge(fb)? * in_b;
                b = fb.parent;
            }
        }
        Ok(in_a.inverse() * in_b)
    }
}

/// `None` for a pose with a non-finite component or a zero rotation;
/// otherwise the rotation is normalized.
pub(crate) fn isometry(pose: &Pose) -> Option<na::Isometry3<f64>> {
    let (p, q) = pose.quat_array();
    let finite = p.iter().chain(&q).all(|v| v.is_finite());
    let q = na::Quaternion::new(q[3], q[0], q[1], q[2]);
    (finite && q.norm_squared() > 0.).then(|| {
        na::Isometry3::from_parts(
            na::Translation3::new(p[0], p[1], p[2]),
            na::UnitQuaternion::from_quaternion(q),
        )
    })
}

#[derive(Clone, Debug)]
enum Decl {
    Fixed(Pose),
    Joint(Joint),
    Dynamic(History),
}

/// Declarations of a [`FrameTree`]. Frames are created by the first
/// declaration that names them; [`build`](Self::build) reports every error.
#[derive(Clone, Debug, Default)]
pub struct FrameTreeBuilder {
    names: Vec<String>,
    edges: Vec<(usize, usize, Decl)>,
    groups: Vec<(String, Vec<String>, History)>,
}

impl FrameTreeBuilder {
    fn frame(&mut self, name: &str) -> usize {
        self.names
            .iter()
            .position(|n| n == name)
            .unwrap_or_else(|| {
                self.names.push(name.into());
                self.names.len() - 1
            })
    }

    fn edge(mut self, parent: &str, child: &str, decl: Decl) -> Self {
        let (parent, child) = (self.frame(parent), self.frame(child));
        self.edges.push((parent, child, decl));
        self
    }

    /// `child` at `pose` in `parent` for the tree's lifetime, such as a
    /// calibrated camera mount.
    pub fn fixed(self, parent: &str, child: &str, pose: Pose) -> Self {
        self.edge(parent, child, Decl::Fixed(pose))
    }

    /// Every link of `model` as a frame and every joint as the transform of
    /// its child link. Movable joints are positioned by the group listing
    /// them.
    pub fn model(mut self, model: &RobotModel) -> Self {
        for link in &model.links {
            self.frame(&link.name);
        }
        for joint in &model.joints {
            let (parent, child) = (
                &model.links[joint.parent].name,
                &model.links[joint.child].name,
            );
            self = self.edge(parent, child, Decl::Joint(joint.clone()));
        }
        self
    }

    /// Movable joints positioned together, in the order of the position
    /// array given to [`FrameBuffer::set_joints`](crate::FrameBuffer::set_joints).
    pub fn joints(mut self, group: &str, joints: &[&str], history: History) -> Self {
        let joints = joints.iter().map(|&j| j.into()).collect();
        self.groups.push((group.into(), joints, history));
        self
    }

    /// `child` in `parent` as written at run time with
    /// [`FrameBuffer::set_transform`](crate::FrameBuffer::set_transform).
    pub fn dynamic(self, parent: &str, child: &str, history: History) -> Self {
        self.edge(parent, child, Decl::Dynamic(history))
    }

    pub fn build(self) -> Result<FrameTree, TreeError> {
        let names = self.names;
        let n = names.len();
        let mut parent = vec![None; n];
        let mut decls: Vec<Option<Decl>> = vec![None; n];
        for (p, c, decl) in self.edges {
            if parent[c].replace(p).is_some() {
                return Err(TreeError::MultipleParents(names[c].clone()));
            }
            decls[c] = Some(decl);
        }

        let roots: Vec<usize> = (0..n).filter(|&f| parent[f].is_none()).collect();
        let mut depth = vec![None; n];
        let mut pending = roots.clone();
        for &r in &roots {
            depth[r] = Some(0);
        }
        while let Some(f) = pending.pop() {
            for c in (0..n).filter(|&c| parent[c] == Some(f)) {
                depth[c] = depth[f].map(|d| d + 1);
                pending.push(c);
            }
        }
        let cycle: Vec<String> = (0..n)
            .filter(|&f| depth[f].is_none())
            .map(|f| names[f].clone())
            .collect();
        if !cycle.is_empty() {
            return Err(TreeError::Cycle(cycle));
        }
        let [root] = roots[..] else {
            return Err(TreeError::Roots(
                roots.iter().map(|&r| names[r].clone()).collect(),
            ));
        };

        for decl in decls.iter().flatten() {
            if let Decl::Joint(j) = decl
                && !(j.kind == JointKind::Fixed || j.is_movable())
            {
                return Err(TreeError::UnsupportedJoint { joint: j.name.clone(), kind: j.kind });
            }
        }

        let mut slots = vec![None; n];
        let mut tracks = Vec::new();
        let mut groups: Vec<(String, usize)> = Vec::new();
        for (group, joints, history) in self.groups {
            if groups.iter().any(|(g, _)| *g == group) {
                return Err(TreeError::DuplicateGroup(group));
            }
            if joints.is_empty() {
                return Err(TreeError::EmptyGroup(group));
            }
            if history.capacity == 0 {
                return Err(TreeError::ZeroCapacity(group));
            }
            for (k, name) in joints.iter().enumerate() {
                let mut found = (0..n)
                    .filter(|&f| matches!(&decls[f], Some(Decl::Joint(j)) if j.name == *name));
                let f = found
                    .next()
                    .ok_or_else(|| TreeError::UnknownJoint(name.clone()))?;
                if found.next().is_some() {
                    return Err(TreeError::AmbiguousJoint(name.clone()));
                }
                if let Some(Decl::Joint(j)) = &decls[f]
                    && !j.is_movable()
                {
                    return Err(TreeError::NotMovable(name.clone()));
                }
                if slots[f].replace((tracks.len(), k)).is_some() {
                    return Err(TreeError::GroupedTwice(name.clone()));
                }
            }
            tracks.push(TrackSpec { stride: joints.len(), history });
            groups.push((group, joints.len()));
        }

        let mut frames = Vec::with_capacity(n);
        for (f, name) in names.into_iter().enumerate() {
            let edge = match decls[f].take() {
                None => Edge::Root,
                Some(Decl::Fixed(pose)) => Edge::Fixed(
                    isometry(&pose).ok_or_else(|| TreeError::InvalidPose(name.clone()))?,
                ),
                Some(Decl::Joint(joint)) if joint.kind == JointKind::Fixed => {
                    Edge::Fixed(joint.transform(0.))
                }
                Some(Decl::Joint(joint)) => Edge::Joint { joint, slot: slots[f] },
                Some(Decl::Dynamic(history)) => {
                    if history.capacity == 0 {
                        return Err(TreeError::ZeroCapacity(name));
                    }
                    tracks.push(TrackSpec { stride: 7, history });
                    Edge::Dynamic(tracks.len() - 1)
                }
            };
            frames.push(Frame {
                id: frame_id(&name),
                name,
                parent: parent[f].unwrap_or(f),
                depth: depth[f].unwrap_or_default(),
                edge,
            });
        }

        let mut by_id: Vec<(FrameId, usize)> =
            frames.iter().enumerate().map(|(i, f)| (f.id, i)).collect();
        by_id.sort_unstable();
        if let Some(w) = by_id.windows(2).find(|w| w[0].0 == w[1].0) {
            let (a, b) = (&frames[w[0].1].name, &frames[w[1].1].name);
            return Err(TreeError::IdCollision(a.clone(), b.clone()));
        }

        Ok(FrameTree {
            frames: frames.into(),
            by_id: by_id.into(),
            root,
            groups: groups.into(),
            tracks: tracks.into(),
        })
    }
}
