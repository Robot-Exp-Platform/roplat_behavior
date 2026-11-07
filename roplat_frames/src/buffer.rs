//! Time history of joint positions and dynamic transforms.

use std::cmp::Ordering;
use std::time::Duration;

use nalgebra as na;
use roplat_msgs::{FrameId, Pose, Stamp, Transform, clock};

use crate::tree::{Edge, Frame, TrackSpec, isometry};
use crate::{FrameError, FrameTree, JointGroup, TreeError};

/// A [`FrameTree`] with the recent history of its joint groups and dynamic
/// transforms, all on one clock.
///
/// Storage is allocated by [`new`](Self::new). Writes and lookups do not
/// allocate and never wait for data: a lookup the history does not cover
/// fails, and the caller decides whether to try again later.
/// Cloning copies the samples into independent storage and preserves group handles.
#[derive(Clone, Debug)]
pub struct FrameBuffer {
    tree: FrameTree,
    clock: u32,
    tracks: Box<[Track]>,
}

impl FrameBuffer {
    /// Every stamp written or looked up must be on `clock`, see
    /// [`roplat_msgs::clock`].
    pub fn new(tree: FrameTree, clock: u32) -> Result<Self, TreeError> {
        if clock == clock::UNSPECIFIED {
            return Err(TreeError::UnspecifiedClock);
        }
        let tracks = tree.tracks.iter().map(Track::new).collect();
        Ok(Self { tree, clock, tracks })
    }

    pub fn tree(&self) -> &FrameTree {
        &self.tree
    }

    pub fn clock(&self) -> u32 {
        self.clock
    }

    /// Records the positions of `group` at `stamp`. A stamp equal to the
    /// latest one replaces that sample. Checks the clock, the group, the
    /// positions and the timestamp order, in that order.
    pub fn set_joints<const N: usize>(
        &mut self,
        group: JointGroup<N>,
        stamp: Stamp,
        q: &[f64; N],
    ) -> Result<(), FrameError> {
        self.check(stamp)?;
        let track = self.tree.group_track(group)?;
        self.tracks[track].push(stamp.nanos, q)
    }

    /// Records `transform.child` in `transform.parent` at `stamp`; the child
    /// must have been declared [dynamic](crate::FrameTreeBuilder::dynamic)
    /// under that parent.
    pub fn set_transform(&mut self, stamp: Stamp, transform: &Transform) -> Result<(), FrameError> {
        self.check(stamp)?;
        let frame = &self.tree.frames[self.tree.index(transform.child)?];
        let Edge::Dynamic(track) = frame.edge else {
            return Err(FrameError::NotDynamic(transform.child));
        };
        if self.tree.frames[frame.parent].id != transform.parent {
            return Err(FrameError::WrongParent {
                child: transform.child,
                parent: transform.parent,
            });
        }
        let iso = isometry(&transform.pose).ok_or(FrameError::InvalidValue)?;
        let (p, q) = (iso.translation.vector, iso.rotation);
        self.tracks[track].push(stamp.nanos, &[p.x, p.y, p.z, q.i, q.j, q.k, q.w])
    }

    /// Pose of `child` in `parent` at `at`, each joint and dynamic transform
    /// on the way taken at that instant.
    pub fn lookup(&self, parent: FrameId, child: FrameId, at: Stamp) -> Result<Pose, FrameError> {
        self.check(at)?;
        self.tree
            .relative(parent, child, |frame| self.edge(frame, at.nanos))
            .map(Pose::from_isometry)
    }

    fn check(&self, stamp: Stamp) -> Result<(), FrameError> {
        if stamp.clock == self.clock {
            Ok(())
        } else {
            Err(FrameError::ClockMismatch { expected: self.clock, got: stamp.clock })
        }
    }

    fn edge(&self, frame: &Frame, at: i64) -> Result<na::Isometry3<f64>, FrameError> {
        match &frame.edge {
            Edge::Root => Ok(na::Isometry3::identity()),
            Edge::Fixed(iso) => Ok(*iso),
            Edge::Joint { slot: None, .. } => Err(FrameError::Unpositioned(frame.id)),
            Edge::Joint { joint, slot: Some((track, k)) } => {
                let track = &self.tracks[*track];
                let q = match track.locate(at, frame.id)? {
                    Sample::One(i) => track.row(i)[*k],
                    Sample::Two(i, j, s) => {
                        let (a, b) = (track.row(i)[*k], track.row(j)[*k]);
                        a + (b - a) * s
                    }
                };
                Ok(joint.transform(q))
            }
            Edge::Dynamic(track) => {
                let track = &self.tracks[*track];
                Ok(match track.locate(at, frame.id)? {
                    Sample::One(i) => pose_row(track.row(i)),
                    Sample::Two(i, j, s) => {
                        interpolate(pose_row(track.row(i)), pose_row(track.row(j)), s)
                    }
                })
            }
        }
    }
}

/// `[x, y, z, qx, qy, qz, qw]` with a unit quaternion.
fn pose_row(row: &[f64]) -> na::Isometry3<f64> {
    na::Isometry3::from_parts(
        na::Translation3::new(row[0], row[1], row[2]),
        na::UnitQuaternion::new_unchecked(na::Quaternion::new(row[6], row[3], row[4], row[5])),
    )
}

fn interpolate(a: na::Isometry3<f64>, b: na::Isometry3<f64>, s: f64) -> na::Isometry3<f64> {
    let translation = a.translation.vector.lerp(&b.translation.vector, s);
    // Shorter arc; `try_slerp` declines rotations too close to tell apart.
    let rotation = a
        .rotation
        .try_slerp(&b.rotation, s, 1e-12)
        .unwrap_or(a.rotation);
    na::Isometry3::from_parts(translation.into(), rotation)
}

/// The greatest distance between two signed nanosecond stamps is u64::MAX.
fn nanos(d: Duration) -> u64 {
    u64::try_from(d.as_nanos()).unwrap_or(u64::MAX)
}

/// Where a lookup instant falls: on one sample, or between two at fraction `s`.
enum Sample {
    One(usize),
    Two(usize, usize, f64),
}

/// Samples in time order in a ring of fixed capacity, `stride` values each.
#[derive(Clone, Debug)]
struct Track {
    stride: usize,
    hold: u64,
    max_gap: u64,
    stamps: Box<[i64]>,
    values: Box<[f64]>,
    /// Slot of the oldest sample.
    head: usize,
    len: usize,
}

impl Track {
    fn new(spec: &TrackSpec) -> Self {
        let history = spec.history;
        Self {
            stride: spec.stride,
            hold: nanos(history.hold),
            max_gap: history.max_gap.map_or(u64::MAX, nanos),
            stamps: vec![0; history.capacity].into(),
            values: vec![0.; history.capacity * spec.stride].into(),
            head: 0,
            len: 0,
        }
    }

    /// Slot of the `i`-th oldest sample.
    fn slot(&self, i: usize) -> usize {
        (self.head + i) % self.stamps.len()
    }

    fn row(&self, slot: usize) -> &[f64] {
        &self.values[slot * self.stride..][..self.stride]
    }

    fn push(&mut self, stamp: i64, row: &[f64]) -> Result<(), FrameError> {
        if !row.iter().all(|v| v.is_finite()) {
            return Err(FrameError::InvalidValue);
        }
        let slot = if self.len == 0 {
            self.len = 1;
            self.head
        } else {
            let last = self.slot(self.len - 1);
            match stamp.cmp(&self.stamps[last]) {
                Ordering::Less => {
                    return Err(FrameError::OutOfOrder { stamp, latest: self.stamps[last] });
                }
                Ordering::Equal => last,
                Ordering::Greater if self.len < self.stamps.len() => {
                    self.len += 1;
                    self.slot(self.len - 1)
                }
                Ordering::Greater => {
                    let oldest = self.head;
                    self.head = self.slot(1);
                    oldest
                }
            }
        };
        self.stamps[slot] = stamp;
        self.values[slot * self.stride..][..self.stride].copy_from_slice(row);
        Ok(())
    }

    fn locate(&self, at: i64, frame: FrameId) -> Result<Sample, FrameError> {
        if self.len == 0 {
            return Err(FrameError::NoData(frame));
        }
        let (first, last) = (self.slot(0), self.slot(self.len - 1));
        let (earliest, latest) = (self.stamps[first], self.stamps[last]);
        if at >= latest {
            return if at.abs_diff(latest) <= self.hold {
                Ok(Sample::One(last))
            } else {
                Err(FrameError::AfterLatest { frame, at, latest })
            };
        }
        if at < earliest {
            return if earliest.abs_diff(at) <= self.hold {
                Ok(Sample::One(first))
            } else {
                Err(FrameError::BeforeHistory { frame, at, earliest })
            };
        }
        // stamps[lo] <= at < stamps[hi]
        let (mut lo, mut hi) = (0, self.len - 1);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if self.stamps[self.slot(mid)] <= at {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let (lo, hi) = (self.slot(lo), self.slot(hi));
        let (before, after) = (self.stamps[lo], self.stamps[hi]);
        if at == before {
            return Ok(Sample::One(lo));
        }
        let span = after.abs_diff(before);
        if span > self.max_gap {
            return Err(FrameError::Gap { frame, before, after });
        }
        Ok(Sample::Two(
            lo,
            hi,
            at.abs_diff(before) as f64 / span as f64,
        ))
    }
}
