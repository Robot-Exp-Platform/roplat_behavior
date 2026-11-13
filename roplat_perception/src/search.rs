//! Nearest-neighbour queries on a cloud: the nearest point, the `k` nearest
//! points, and every point within a radius of any query point, exact, by a
//! k-d tree. These are the queries of PCL's `KdTreeFLANN` (`nearestKSearch`,
//! `radiusSearch`) with exact search.
//!
//! The tree borrows the cloud and indexes its finite points; a non-finite
//! point (no return) is never found. Answers are in a total order, by
//! squared distance and then by index, so a query has exactly one answer
//! and equal distances do not depend on how the tree was split.
//!
//! ```
//! use roplat_perception::KdTree;
//!
//! let points = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [f64::NAN, 0.0, 0.0]];
//! let tree = KdTree::new(&points);
//! assert_eq!(tree.len(), 3);
//! let near = tree.k_nearest(&[0.9, 0.1, 0.0], 2);
//! assert_eq!(near.iter().map(|n| n.index).collect::<Vec<_>>(), [1, 0]);
//! assert!((near[0].squared_distance - 0.02).abs() < 1e-12);
//! assert_eq!(tree.within(&[0.0, 0.0, 0.0], 1.0).len(), 2);
//! assert_eq!(tree.nearest(&[0.0, 1.9, 0.0]).map(|n| n.index), Some(2));
//! ```
//!
//! Building takes `O(n log n)` time and `O(n)` memory; a query visits
//! `O(log n)` nodes on clouds of even density. Each query allocates only its
//! answer.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::grid::{is_finite, squared_distance};

/// A point a query found: its index in the cloud and its squared distance to
/// the query point (m²).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Neighbour {
    pub index: usize,
    pub squared_distance: f64,
}

impl Neighbour {
    /// The order of answers: nearer first, the lower index first at equal
    /// distance.
    fn cmp_key(&self, other: &Self) -> Ordering {
        self.squared_distance
            .total_cmp(&other.squared_distance)
            .then(self.index.cmp(&other.index))
    }
}

/// [`Neighbour`] in the answer order, for the heap of the `k` best.
struct Ranked(Neighbour);

impl PartialEq for Ranked {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Ranked {}

impl PartialOrd for Ranked {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Ranked {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.cmp_key(&other.0)
    }
}

/// Ranges of at most this many points are scanned rather than split.
const LEAF: usize = 8;

/// A k-d tree over the finite points of a borrowed cloud.
///
/// Each range of the index is split at its median along the axis of its
/// largest extent; the median point is the node, the halves its subtrees.
#[derive(Clone, Debug)]
pub struct KdTree<'a> {
    points: &'a [[f64; 3]],
    /// Indices of the finite points: every range that is split has its
    /// splitting point in the middle, the points not after it along the
    /// split axis before it and the points not before it after it.
    order: Vec<usize>,
    /// The split axis of a range, at the position of its middle entry.
    axes: Vec<u8>,
}

impl<'a> KdTree<'a> {
    /// The tree over the finite points of `points`; indices in answers are
    /// indices into `points`.
    pub fn new(points: &'a [[f64; 3]]) -> Self {
        let order: Vec<usize> = (0..points.len()).filter(|&i| is_finite(&points[i])).collect();
        let mut tree = Self { points, axes: vec![0; order.len()], order };
        tree.split(0, tree.order.len());
        tree
    }

    fn split(&mut self, lo: usize, hi: usize) {
        if hi - lo <= LEAF {
            return;
        }
        let points = self.points;
        let range = &mut self.order[lo..hi];
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for &i in range.iter() {
            for k in 0..3 {
                min[k] = min[k].min(points[i][k]);
                max[k] = max[k].max(points[i][k]);
            }
        }
        let axis = (0..3)
            .max_by(|&a, &b| (max[a] - min[a]).total_cmp(&(max[b] - min[b])))
            .unwrap_or(0);
        let mid = (hi - lo) / 2;
        range.select_nth_unstable_by(mid, |&a, &b| points[a][axis].total_cmp(&points[b][axis]));
        self.axes[lo + mid] = axis as u8;
        self.split(lo, lo + mid);
        self.split(lo + mid + 1, hi);
    }

    /// The number of points indexed: the finite points of the cloud.
    pub fn len(&self) -> usize {
        self.order.len()
    }

    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    /// The cloud the tree indexes.
    pub fn points(&self) -> &'a [[f64; 3]] {
        self.points
    }

    /// The point nearest to `query`; `None` for an empty tree or a query
    /// that is not finite.
    pub fn nearest(&self, query: &[f64; 3]) -> Option<Neighbour> {
        self.k_nearest(query, 1).pop()
    }

    /// The `k` points nearest to `query`, nearest first; fewer when the
    /// tree has fewer, none for a query that is not finite.
    pub fn k_nearest(&self, query: &[f64; 3], k: usize) -> Vec<Neighbour> {
        if k == 0 || !is_finite(query) {
            return Vec::new();
        }
        let mut best = Best { k, heap: BinaryHeap::with_capacity(k.min(self.len()) + 1) };
        self.search(0, self.order.len(), query, &mut best);
        best.heap.into_sorted_vec().into_iter().map(|r| r.0).collect()
    }

    /// The points within `radius` (m) of `query`, the boundary included,
    /// nearest first; none for a query that is not finite or a radius that
    /// is negative or NaN.
    pub fn within(&self, query: &[f64; 3], radius: f64) -> Vec<Neighbour> {
        if !is_finite(query) || radius.is_nan() || radius < 0.0 {
            return Vec::new();
        }
        let mut found = Within { squared_radius: radius * radius, found: Vec::new() };
        self.search(0, self.order.len(), query, &mut found);
        found.found.sort_unstable_by(Neighbour::cmp_key);
        found.found
    }

    /// Offers `collector` every point of the range `lo..hi` that may beat its
    /// bound, nearest side first.
    fn search(&self, lo: usize, hi: usize, query: &[f64; 3], collector: &mut impl Collector) {
        if hi - lo <= LEAF {
            for &index in &self.order[lo..hi] {
                collector.offer(Neighbour {
                    index,
                    squared_distance: squared_distance(&self.points[index], query),
                });
            }
            return;
        }
        let mid = lo + (hi - lo) / 2;
        let index = self.order[mid];
        let axis = self.axes[mid] as usize;
        let across = query[axis] - self.points[index][axis];
        let (near, far) = if across < 0.0 {
            ((lo, mid), (mid + 1, hi))
        } else {
            ((mid + 1, hi), (lo, mid))
        };
        self.search(near.0, near.1, query, collector);
        collector.offer(Neighbour {
            index,
            squared_distance: squared_distance(&self.points[index], query),
        });
        // Every point beyond the split is at least `across` away. At equal
        // distance the far side may hold a lower index, so it is searched.
        if across * across <= collector.bound() {
            self.search(far.0, far.1, query, collector);
        }
    }
}

/// What a search keeps of the points it is offered.
trait Collector {
    fn offer(&mut self, candidate: Neighbour);
    /// The squared distance beyond which no point is kept.
    fn bound(&self) -> f64;
}

/// The `k` best, the worst on top.
struct Best {
    k: usize,
    heap: BinaryHeap<Ranked>,
}

impl Collector for Best {
    fn offer(&mut self, candidate: Neighbour) {
        if self.heap.len() < self.k {
            self.heap.push(Ranked(candidate));
        } else if self
            .heap
            .peek()
            .is_some_and(|worst| candidate.cmp_key(&worst.0) == Ordering::Less)
        {
            self.heap.pop();
            self.heap.push(Ranked(candidate));
        }
    }

    fn bound(&self) -> f64 {
        match self.heap.peek() {
            Some(worst) if self.heap.len() == self.k => worst.0.squared_distance,
            _ => f64::INFINITY,
        }
    }
}

struct Within {
    squared_radius: f64,
    found: Vec<Neighbour>,
}

impl Collector for Within {
    fn offer(&mut self, candidate: Neighbour) {
        if candidate.squared_distance <= self.squared_radius {
            self.found.push(candidate);
        }
    }

    fn bound(&self) -> f64 {
        self.squared_radius
    }
}
