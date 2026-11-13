//! `KdTree` against brute force, answer by answer: the same indices in the
//! same order and the same squared distances, on random clouds, on a grid
//! where many distances tie, with duplicate and collinear points, and with
//! points that are not finite.

use roplat_perception::{KdTree, Neighbour};

/// SplitMix64, so every run draws the same clouds.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn point(&mut self, half: f64) -> [f64; 3] {
        std::array::from_fn(|_| self.uniform(-half, half))
    }
}

/// Every finite point with its squared distance to `query`, nearest first,
/// the lower index first at equal distance.
fn brute(points: &[[f64; 3]], query: &[f64; 3]) -> Vec<Neighbour> {
    let mut all: Vec<Neighbour> = points
        .iter()
        .enumerate()
        .filter(|(_, p)| p.iter().all(|c| c.is_finite()))
        .map(|(index, p)| Neighbour {
            index,
            squared_distance: (0..3).map(|k| (p[k] - query[k]).powi(2)).sum(),
        })
        .collect();
    all.sort_by(|a, b| {
        a.squared_distance
            .total_cmp(&b.squared_distance)
            .then(a.index.cmp(&b.index))
    });
    all
}

/// Checks every query kind at `query` against brute force.
fn check(tree: &KdTree, query: &[f64; 3], radii: &[f64]) -> usize {
    let all = brute(tree.points(), query);
    let mut compared = 0;
    for k in [1, 2, 7, 8, 9, 33, all.len(), all.len() + 5] {
        let expected = &all[..k.min(all.len())];
        assert_eq!(tree.k_nearest(query, k), expected, "k = {k} at {query:?}");
        compared += 1;
    }
    assert_eq!(tree.nearest(query), all.first().copied(), "{query:?}");
    for &radius in radii {
        let expected: Vec<Neighbour> = all
            .iter()
            .filter(|n| n.squared_distance <= radius * radius)
            .copied()
            .collect();
        assert_eq!(
            tree.within(query, radius),
            expected,
            "radius {radius} at {query:?}"
        );
        compared += 1;
    }
    compared
}

#[test]
fn random_clouds_match_brute_force() {
    let mut rng = SplitMix64(0xbe12);
    let mut compared = 0;
    for &(n, half) in &[(0, 1.0), (1, 1.0), (9, 1.0), (300, 0.5), (4000, 2.0)] {
        let points: Vec<[f64; 3]> = (0..n).map(|_| rng.point(half)).collect();
        let tree = KdTree::new(&points);
        assert_eq!(tree.len(), n);
        for _ in 0..60 {
            // Inside the cloud, and well outside it.
            let query = rng.point(half * 1.5);
            compared += check(&tree, &query, &[0.0, 0.05, 0.2, 0.6, f64::INFINITY]);
        }
    }
    assert!(compared > 3000, "{compared}");
}

/// A 10 x 10 x 10 grid 0.125 m apart, every coordinate and distance exact
/// in binary: queries at grid points, edge midpoints and cell centres meet
/// many points at exactly the same distance, so the order among ties
/// decides the answer; radii fall exactly on grid distances.
#[test]
fn ties_on_a_grid_are_broken_by_index() {
    let step = 0.125;
    let points: Vec<[f64; 3]> = (0..1000)
        .map(|i| {
            [
                (i % 10) as f64 * step,
                (i / 10 % 10) as f64 * step,
                (i / 100) as f64 * step,
            ]
        })
        .collect();
    let tree = KdTree::new(&points);
    let (mut queries, mut ties) = (0, 0);
    for x in 0..19 {
        for y in [0, 5, 9, 13, 18] {
            for z in [0, 7, 18] {
                let query = [x as f64, y as f64, z as f64].map(|c| c * step / 2.0);
                check(&tree, &query, &[0.0, 0.0625, 0.125, 0.1875, 0.25]);
                let all = brute(&points, &query);
                // A tie across one of the cut-offs `check` asks for.
                if [1, 2, 7, 8, 9, 33]
                    .iter()
                    .any(|&k| all[k - 1].squared_distance == all[k].squared_distance)
                {
                    ties += 1;
                }
                queries += 1;
            }
        }
    }
    assert!(ties * 2 > queries, "{ties} of {queries}");
}

/// Repeated points, a cloud on one line (no extent across it), a single
/// point repeated, and points that are not finite, which are never found.
#[test]
fn duplicates_lines_and_missing_points() {
    let mut rng = SplitMix64(7);
    let base: Vec<[f64; 3]> = (0..200).map(|_| rng.point(1.0)).collect();
    let mut repeated: Vec<[f64; 3]> = base.iter().flat_map(|p| [*p, *p, *p]).collect();
    repeated.insert(17, [f64::NAN, 0.0, 0.0]);
    repeated.insert(400, [0.0, f64::INFINITY, 0.0]);
    let line: Vec<[f64; 3]> = (0..500)
        .map(|i| [0.3, -0.2 + 0.001 * (i % 97) as f64, 1.0])
        .collect();
    let same = vec![[0.5, 0.5, 0.5]; 50];
    for cloud in [&repeated, &line, &same] {
        let tree = KdTree::new(cloud);
        assert_eq!(
            tree.len(),
            cloud
                .iter()
                .filter(|p| p.iter().all(|c| c.is_finite()))
                .count()
        );
        for _ in 0..40 {
            let query = rng.point(1.2);
            check(&tree, &query, &[0.0, 0.01, 0.3, f64::INFINITY]);
        }
        for p in cloud.iter().take(30) {
            if p.iter().all(|c| c.is_finite()) {
                check(&tree, p, &[0.0, 0.002]);
            }
        }
    }
}

#[test]
fn bad_queries_find_nothing() {
    let points = [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]];
    let tree = KdTree::new(&points);
    assert!(tree.k_nearest(&[f64::NAN, 0.0, 0.0], 1).is_empty());
    assert!(tree.k_nearest(&[0.0, 0.0, 0.0], 0).is_empty());
    assert_eq!(tree.nearest(&[0.0, f64::INFINITY, 0.0]), None);
    assert!(tree.within(&[0.0, 0.0, 0.0], -1.0).is_empty());
    assert!(tree.within(&[0.0, 0.0, 0.0], f64::NAN).is_empty());
    assert_eq!(tree.within(&[0.0, 0.0, 0.0], f64::INFINITY).len(), 2);
    let empty = KdTree::new(&[]);
    assert!(empty.is_empty());
    assert_eq!(empty.nearest(&[0.0; 3]), None);
}
