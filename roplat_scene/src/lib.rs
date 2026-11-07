//! Collision checking for motion planning.
//!
//! A [`PlanningScene`] holds a robot read from its [`RobotModel`], objects
//! placed in the world, objects attached to the robot's links, and the pairs
//! that may touch ([`AllowedCollisions`]). For a configuration of one
//! planning group it reports a contact that is not allowed
//! ([`CollisionCheck::first_contact`]), which is the check `plan_joint_path`
//! in `roplat_trajectory` asks of its caller:
//!
//! ```no_run
//! # use roplat_scene::{CollisionCheck, PlanningScene};
//! # use roplat_trajectory::{PlannerOptions, plan_joint_path};
//! # fn plan(scene: &PlanningScene<7>, start: [f64; 7], goal: [f64; 7], lower: [f64; 7],
//! #         upper: [f64; 7], options: PlannerOptions) {
//! let path = plan_joint_path(start, goal, lower, upper, |q| scene.is_free(q), options);
//! # }
//! ```
//!
//! URDF boxes, spheres, cylinders and capsules keep their shape. A mesh (OBJ
//! or STL) becomes the convex hull of its vertices, which contains the mesh:
//! the hull may report a contact the mesh would not, never the reverse. A
//! hull fills a concave object's hollows, so nothing can reach into a bin or
//! a shelf; [`PlanningScene::add_concave_object`] and
//! [`PlanningScene::add_trimesh`] split such an object into convex parts
//! instead, and [`PlanningScene::decompose_link`] does the same for a link.
//! Before sensed points join the scene,
//! [`PlanningScene::without_robot_points`] removes those on the robot.
//! [`occupancy::OccupancyMap`] keeps sensed obstacles current across scans,
//! clearing the voxels rays pass through, and
//! [`PlanningScene::set_occupancy`] puts it into the scene.
//! Distances come from [parry3d-f64](https://parry.rs).
//!
//! The planners sample a motion between two configurations every few
//! hundredths of a radian, which can step over a thin obstacle.
//! [`PlanningScene::motion_contact`] checks a straight joint motion as a
//! whole instead, with no step to choose; see [`motion`].
//!
//! The scene is plain data with no clock, registry or runtime. Its owner
//! moves, attaches and detaches objects between queries; a query poses the
//! robot without changing the scene.

pub mod drawing;
mod mesh;
pub mod motion;
pub mod occupancy;
pub mod points;
mod robot;
pub mod semantics;

pub use drawing::{BodyKind, BodyMesh};
pub use motion::{MOTION_TOLERANCE, MotionContact, REJECT_STEP};
pub use semantics::{EndEffector, NamedPoses};

use std::{
    collections::{BTreeMap, BTreeSet},
    ops::ControlFlow,
    path::{Path, PathBuf},
};

use nalgebra as na;
use parry3d_f64::{
    bounding_volume::{Aabb, BoundingVolume},
    query,
    shape::SharedShape,
    transformation::vhacd::{VHACD, VHACDParameters},
};
use roplat_description::{Geometry, RobotModel};

#[derive(Debug, thiserror::Error)]
pub enum SceneError {
    #[error("unknown joint `{0}`")]
    UnknownJoint(String),
    #[error("joint `{0}` is not movable")]
    FixedJoint(String),
    #[error("joint `{0}` appears twice in the planning group")]
    RepeatedJoint(String),
    #[error("joint `{0}` belongs to the planning group; each query gives its position")]
    GroupJoint(String),
    #[error("unknown link `{0}`")]
    UnknownLink(String),
    #[error("unknown object `{0}`")]
    UnknownObject(String),
    #[error("`{0}` already names a link or an object")]
    NameTaken(String),
    #[error("object `{0}` is attached")]
    Attached(String),
    #[error("object `{0}` is not attached")]
    NotAttached(String),
    #[error("mesh {path:?}: {reason}")]
    Mesh { path: PathBuf, reason: String },
    #[error("invalid geometry: {0}")]
    InvalidGeometry(String),
    #[error("padding {0} must be finite and not negative")]
    InvalidPadding(f64),
}

/// Two bodies, links or objects, that come within their padding: the
/// scene's, or the pair's own (see [`PlanningScene::set_pair_padding`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Contact {
    pub first: String,
    pub second: String,
    /// Signed distance (m) between their closest points, negative when they
    /// overlap.
    pub distance: f64,
}

/// Answers whether a configuration of `N` joints is free of contacts.
pub trait CollisionCheck<const N: usize> {
    /// A contact that is not allowed at `q`, or `None` when `q` is free.
    fn first_contact(&self, q: &[f64; N]) -> Option<Contact>;

    fn is_free(&self, q: &[f64; N]) -> bool {
        self.first_contact(q).is_none()
    }
}

/// Pairs of names, links or objects, that may touch, in either order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct AllowedCollisions {
    /// Both orders of each pair, so lookups borrow `&str`.
    pairs: BTreeMap<String, BTreeSet<String>>,
}

impl AllowedCollisions {
    pub fn new() -> Self {
        Self::default()
    }

    /// Each link with collision geometry and its nearest ancestor with any:
    /// links joined by a joint, or by links without geometry between them
    /// (Panda's `panda_link8` between `panda_link7` and `panda_hand`). Such
    /// links usually overlap where they meet.
    pub fn adjacent(model: &RobotModel) -> Self {
        let mut parent = vec![None; model.links.len()];
        for joint in &model.joints {
            parent[joint.child] = Some(joint.parent);
        }
        let solid = |link: usize| !model.links[link].collisions.is_empty();
        let mut allowed = Self::new();
        for link in (0..model.links.len()).filter(|&link| solid(link)) {
            let mut up = parent[link];
            while let Some(above) = up {
                if solid(above) {
                    allowed.allow(&model.links[link].name, &model.links[above].name);
                    break;
                }
                up = parent[above];
            }
        }
        allowed
    }

    pub fn allow(&mut self, a: &str, b: &str) {
        self.pairs.entry(a.into()).or_default().insert(b.into());
        self.pairs.entry(b.into()).or_default().insert(a.into());
    }

    /// Returns whether the pair was allowed.
    pub fn forbid(&mut self, a: &str, b: &str) -> bool {
        let mut remove = |from: &str, to: &str| {
            let Some(set) = self.pairs.get_mut(from) else {
                return false;
            };
            let removed = set.remove(to);
            if set.is_empty() {
                self.pairs.remove(from);
            }
            removed
        };
        remove(a, b) | remove(b, a)
    }

    pub fn is_allowed(&self, a: &str, b: &str) -> bool {
        self.pairs.get(a).is_some_and(|set| set.contains(b))
    }

    /// Each pair once, its names in order.
    pub fn pairs(&self) -> impl Iterator<Item = (&str, &str)> {
        self.pairs.iter().flat_map(|(a, set)| {
            set.iter()
                .filter(move |b| a.as_str() <= b.as_str())
                .map(move |b| (a.as_str(), b.as_str()))
        })
    }
}

/// A convex shape posed in the frame of whatever carries it.
#[derive(Clone)]
struct Part {
    at: na::Isometry3<f64>,
    shape: SharedShape,
}

#[derive(Clone)]
enum Mount {
    World(na::Isometry3<f64>),
    /// Posed `at` in link `link`'s frame, free to touch it and the `touch`
    /// links.
    Link {
        link: usize,
        at: na::Isometry3<f64>,
        touch: Vec<usize>,
    },
}

#[derive(Clone)]
struct Object {
    name: String,
    part: Part,
    mount: Mount,
}

/// A robot, the objects around and on it, and the contacts allowed between
/// them, checked for one planning group of `N` joints.
///
/// The robot's root link sits at [`base`](Self::base) in the world. Joints
/// outside the group keep the positions given by
/// [`set_joint`](Self::set_joint), zero until then. Objects are named like
/// links, and a contact is reported between:
///
/// - two links, unless the pair is allowed;
/// - a link and an object, unless allowed, or the object is attached to that
///   link or lists it as a link it may touch;
/// - two objects, unless allowed, or both are in the world.
///
/// [`AllowedCollisions::adjacent`] pairs are allowed from the start.
#[derive(Clone)]
pub struct PlanningScene<const N: usize> {
    model: RobotModel,
    /// Child link of each group joint: a joint is known by its child in
    /// [`RobotModel::link_frames`].
    group: [usize; N],
    /// Position of every movable joint, by child link; the group's entries
    /// are replaced in each query.
    positions: Vec<f64>,
    base: na::Isometry3<f64>,
    /// Collision parts of each link, in its frame.
    links: Vec<Vec<Part>>,
    objects: Vec<Object>,
    allowed: AllowedCollisions,
    padding: f64,
    /// Paddings of pairs that keep their own, under both orders of the
    /// pair, so lookups borrow `&str`.
    pair_padding: BTreeMap<String, BTreeMap<String, f64>>,
}

impl<const N: usize> PlanningScene<N> {
    /// The scene of `model` alone. `group` names the joints a query's `q`
    /// positions, in order. `mesh_path` turns a mesh file name as written in
    /// the description (`package://…`, relative) into a path to read.
    pub fn new(
        model: RobotModel,
        group: [&str; N],
        mut mesh_path: impl FnMut(&str) -> PathBuf,
    ) -> Result<Self, SceneError> {
        let mut children = [0; N];
        for (k, name) in group.iter().enumerate() {
            let joint = model
                .joint(name)
                .ok_or_else(|| SceneError::UnknownJoint(name.to_string()))?;
            if !joint.is_movable() {
                return Err(SceneError::FixedJoint(name.to_string()));
            }
            if children[..k].contains(&joint.child) {
                return Err(SceneError::RepeatedJoint(name.to_string()));
            }
            children[k] = joint.child;
        }
        let links = model
            .links
            .iter()
            .map(|link| {
                link.collisions
                    .iter()
                    .map(|shape| part(&shape.geometry, shape.origin.quat(), &mut mesh_path))
                    .collect::<Result<Vec<_>, _>>()
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            group: children,
            positions: vec![0.0; model.links.len()],
            base: na::Isometry3::identity(),
            links,
            objects: Vec::new(),
            allowed: AllowedCollisions::adjacent(&model),
            padding: 0.0,
            pair_padding: BTreeMap::new(),
            model,
        })
    }

    pub fn model(&self) -> &RobotModel {
        &self.model
    }

    pub fn base(&self) -> na::Isometry3<f64> {
        self.base
    }

    /// Places the robot's root link in the world.
    pub fn set_base(&mut self, base: na::Isometry3<f64>) {
        self.base = base;
    }

    /// Fixes a movable joint outside the planning group, such as a finger.
    pub fn set_joint(&mut self, name: &str, position: f64) -> Result<(), SceneError> {
        let joint = self
            .model
            .joint(name)
            .ok_or_else(|| SceneError::UnknownJoint(name.into()))?;
        if !joint.is_movable() {
            return Err(SceneError::FixedJoint(name.into()));
        }
        if self.group.contains(&joint.child) {
            return Err(SceneError::GroupJoint(name.into()));
        }
        self.positions[joint.child] = position;
        Ok(())
    }

    pub fn allowed(&self) -> &AllowedCollisions {
        &self.allowed
    }

    pub fn allowed_mut(&mut self) -> &mut AllowedCollisions {
        &mut self.allowed
    }

    pub fn padding(&self) -> f64 {
        self.padding
    }

    /// Reports bodies closer than `padding` (m) as in contact, keeping
    /// that clearance in a plan.
    pub fn set_padding(&mut self, padding: f64) -> Result<(), SceneError> {
        if !(padding.is_finite() && padding >= 0.0) {
            return Err(SceneError::InvalidPadding(padding));
        }
        self.padding = padding;
        Ok(())
    }

    /// Keeps `padding` (m) between `a` and `b`, links or objects in either
    /// order, in place of the scene's padding: more around a fragile
    /// fixture or a person's workspace, less where the hand must pass close
    /// to a shelf. Replaces the pair's earlier value; the names need not
    /// exist yet.
    pub fn set_pair_padding(&mut self, a: &str, b: &str, padding: f64) -> Result<(), SceneError> {
        if !(padding.is_finite() && padding >= 0.0) {
            return Err(SceneError::InvalidPadding(padding));
        }
        self.pair_padding
            .entry(a.into())
            .or_default()
            .insert(b.into(), padding);
        self.pair_padding
            .entry(b.into())
            .or_default()
            .insert(a.into(), padding);
        Ok(())
    }

    /// Returns the pair to the scene's padding; whether it had its own.
    pub fn clear_pair_padding(&mut self, a: &str, b: &str) -> bool {
        let mut remove = |from: &str, to: &str| {
            let Some(paddings) = self.pair_padding.get_mut(from) else {
                return false;
            };
            let removed = paddings.remove(to).is_some();
            if paddings.is_empty() {
                self.pair_padding.remove(from);
            }
            removed
        };
        remove(a, b) | remove(b, a)
    }

    /// The padding kept between `a` and `b`: the pair's own, else the
    /// scene's.
    pub fn pair_padding(&self, a: &str, b: &str) -> f64 {
        self.pair_padding
            .get(a)
            .and_then(|paddings| paddings.get(b))
            .copied()
            .unwrap_or(self.padding)
    }

    /// The largest padding any pair keeps, which bounding boxes grow by.
    fn widest_padding(&self) -> f64 {
        self.pair_padding
            .values()
            .flat_map(|paddings| paddings.values())
            .copied()
            .fold(self.padding, f64::max)
    }

    /// Places a new object in the world. A mesh file name is read as a path.
    pub fn add_object(
        &mut self,
        name: &str,
        geometry: &Geometry,
        pose: na::Isometry3<f64>,
    ) -> Result<(), SceneError> {
        self.write_world_object(name, geometry, pose, false)
    }

    /// Places a world object or replaces its geometry and pose in place.
    ///
    /// The caller supplies a finite rigid pose in the world frame.
    /// The name and position in [`object_names`](Self::object_names) survive a
    /// replacement. Link names return [`SceneError::NameTaken`], attached
    /// objects return [`SceneError::Attached`], and shape construction errors
    /// leave the previous object intact. Use [`move_object`](Self::move_object)
    /// when only the pose changes to avoid rebuilding the shape.
    pub fn set_object(
        &mut self,
        name: &str,
        geometry: &Geometry,
        pose: na::Isometry3<f64>,
    ) -> Result<(), SceneError> {
        self.write_world_object(name, geometry, pose, true)
    }

    fn write_world_object(
        &mut self,
        name: &str,
        geometry: &Geometry,
        pose: na::Isometry3<f64>,
        replace: bool,
    ) -> Result<(), SceneError> {
        if self.model.link(name).is_some() {
            return Err(SceneError::NameTaken(name.into()));
        }
        let existing = self.objects.iter_mut().find(|object| object.name == name);
        if let Some(object) = existing.as_deref() {
            if !replace {
                return Err(SceneError::NameTaken(name.into()));
            }
            if matches!(object.mount, Mount::Link { .. }) {
                return Err(SceneError::Attached(name.into()));
            }
        }
        let part = part(geometry, na::Isometry3::identity(), |name: &str| {
            PathBuf::from(name)
        })?;
        if let Some(object) = existing {
            object.part = part;
            object.mount = Mount::World(pose);
        } else {
            self.objects
                .push(Object { name: name.into(), part, mount: Mount::World(pose) });
        }
        Ok(())
    }

    /// Places a new object that may be concave, such as a bin or a shelf. A
    /// mesh is split into convex parts by approximate convex decomposition
    /// (V-HACD) rather than wrapped in one hull, so its hollows stay free;
    /// other geometry is placed as [`add_object`](Self::add_object) does.
    /// Returns the number of convex parts. A mesh file name is read as a path,
    /// and its faces are needed (OBJ `f` lines, or STL).
    ///
    /// The parts follow the mesh to within about 1/64 of its largest extent;
    /// walls thinner than that may close small openings. Decomposing takes
    /// time, a fraction of a second for a simple bin and seconds for a
    /// detailed mesh, so add a concave object once and move it with
    /// [`move_object`](Self::move_object). Unoptimised it is about 200 times
    /// slower; a debug build that adds concave objects should optimise parry:
    ///
    /// ```toml
    /// [profile.dev.package.parry3d-f64]
    /// opt-level = 3
    /// ```
    pub fn add_concave_object(
        &mut self,
        name: &str,
        geometry: &Geometry,
        pose: na::Isometry3<f64>,
    ) -> Result<usize, SceneError> {
        let Geometry::Mesh { filename, scale } = geometry else {
            self.add_object(name, geometry, pose)?;
            return Ok(1);
        };
        if self.model.link(name).is_some() || self.object(name).is_ok() {
            return Err(SceneError::NameTaken(name.into()));
        }
        let path = PathBuf::from(filename);
        let failed = |reason: String| SceneError::Mesh { path: path.clone(), reason };
        if !scale.iter().all(|v| v.is_finite() && *v > 0.0) {
            return Err(SceneError::InvalidGeometry(format!(
                "mesh scale {scale:?} must be positive"
            )));
        }
        let (vertices, triangles) = mesh::triangles(&path).map_err(failed)?;
        let vertices: Vec<[f64; 3]> = vertices
            .into_iter()
            .map(|v| [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]])
            .collect();
        let shape = decompose(&vertices, &triangles).map_err(failed)?;
        Ok(self.place(name, shape, pose))
    }

    /// Places a new concave object given as triangles over `vertices`, in
    /// the object's frame, as [`add_concave_object`](Self::add_concave_object)
    /// does for a mesh file. For objects built in code, such as a bin of given
    /// dimensions.
    pub fn add_trimesh(
        &mut self,
        name: &str,
        vertices: &[[f64; 3]],
        triangles: &[[u32; 3]],
        pose: na::Isometry3<f64>,
    ) -> Result<usize, SceneError> {
        if self.model.link(name).is_some() || self.object(name).is_ok() {
            return Err(SceneError::NameTaken(name.into()));
        }
        let shape = decompose(vertices, triangles).map_err(SceneError::InvalidGeometry)?;
        Ok(self.place(name, shape, pose))
    }

    /// Adds `shape` as a world object; returns its number of convex parts.
    fn place(&mut self, name: &str, shape: SharedShape, pose: na::Isometry3<f64>) -> usize {
        let parts = shape
            .as_compound()
            .map_or(1, |compound| compound.shapes().len());
        let part = Part { at: na::Isometry3::identity(), shape };
        self.objects
            .push(Object { name: name.into(), part, mount: Mount::World(pose) });
        parts
    }

    /// Places sensed `points` (in the frame `pose` gives in the world) as
    /// one object: the voxels of edge `voxel` (m) that contain a point, each
    /// a cube. Returns the number of voxels. The object moves, attaches and
    /// is removed like any other; replace it by removing it and adding the
    /// new reading. See [`points`] for turning a depth image into points.
    pub fn add_points(
        &mut self,
        name: &str,
        points: &[[f64; 3]],
        voxel: f64,
        pose: na::Isometry3<f64>,
    ) -> Result<usize, SceneError> {
        if !(voxel.is_finite() && voxel > 0.0) {
            return Err(SceneError::InvalidGeometry(format!(
                "voxel edge {voxel} must be positive"
            )));
        }
        if self.model.link(name).is_some() || self.object(name).is_ok() {
            return Err(SceneError::NameTaken(name.into()));
        }
        let cells = points::occupied_voxels(points, voxel);
        if cells.is_empty() {
            return Err(SceneError::InvalidGeometry(format!(
                "no finite points for {name}"
            )));
        }
        let half = voxel / 2.0;
        let cubes: Vec<_> = cells
            .iter()
            .map(|cell| {
                let centre = cell.map(|c| (c as f64 + 0.5) * voxel);
                (
                    na::Isometry3::translation(centre[0], centre[1], centre[2]),
                    SharedShape::cuboid(half, half, half),
                )
            })
            .collect();
        let part = Part {
            at: na::Isometry3::identity(),
            shape: SharedShape::compound(cubes),
        };
        self.objects
            .push(Object { name: name.into(), part, mount: Mount::World(pose) });
        Ok(cells.len())
    }

    /// Moves an object that is in the world.
    pub fn move_object(&mut self, name: &str, pose: na::Isometry3<f64>) -> Result<(), SceneError> {
        let k = self.object(name)?;
        match &mut self.objects[k].mount {
            Mount::World(at) => *at = pose,
            Mount::Link { .. } => return Err(SceneError::Attached(name.into())),
        }
        Ok(())
    }

    pub fn remove_object(&mut self, name: &str) -> Result<(), SceneError> {
        let k = self.object(name)?;
        self.objects.remove(k);
        Ok(())
    }

    pub fn object_names(&self) -> impl Iterator<Item = &str> {
        self.objects.iter().map(|object| object.name.as_str())
    }

    /// Fixes a world object to `link` where the robot at `q` finds it, as
    /// when a grasp closes. It then moves with the link and may touch it and
    /// the `touch` links (the fingers) until [detached](Self::detach).
    pub fn attach(
        &mut self,
        name: &str,
        link: &str,
        touch: &[&str],
        q: &[f64; N],
    ) -> Result<(), SceneError> {
        let k = self.object(name)?;
        let Mount::World(pose) = self.objects[k].mount else {
            return Err(SceneError::Attached(name.into()));
        };
        let link = self.link_index(link)?;
        let touch = touch
            .iter()
            .map(|name| self.link_index(name))
            .collect::<Result<Vec<_>, _>>()?;
        let at = self.link_frames(q)[link].inverse() * pose;
        self.objects[k].mount = Mount::Link { link, at, touch };
        Ok(())
    }

    /// Leaves an attached object in the world where the robot at `q` holds
    /// it.
    pub fn detach(&mut self, name: &str, q: &[f64; N]) -> Result<(), SceneError> {
        let k = self.object(name)?;
        let Mount::Link { link, at, .. } = self.objects[k].mount else {
            return Err(SceneError::NotAttached(name.into()));
        };
        self.objects[k].mount = Mount::World(self.link_frames(q)[link] * at);
        Ok(())
    }

    /// An object's pose in the world with the robot at `q`.
    pub fn object_pose(&self, name: &str, q: &[f64; N]) -> Result<na::Isometry3<f64>, SceneError> {
        Ok(match &self.objects[self.object(name)?].mount {
            Mount::World(pose) => *pose,
            Mount::Link { link, at, .. } => self.link_frames(q)[*link] * at,
        })
    }

    /// A link's pose in the world with the robot at `q`.
    pub fn link_pose(&self, link: &str, q: &[f64; N]) -> Result<na::Isometry3<f64>, SceneError> {
        Ok(self.link_frames(q)[self.link_index(link)?])
    }

    /// Every contact that is not allowed at `q`, one per pair of bodies,
    /// at their closest parts.
    pub fn contacts(&self, q: &[f64; N]) -> Vec<Contact> {
        let mut contacts = Vec::new();
        self.each_contact(q, |contact| {
            contacts.push(contact);
            ControlFlow::Continue(())
        });
        contacts
    }

    /// The closest pair of bodies at `q` among those that may not touch,
    /// ignoring the padding; `None` when no pair is checked.
    pub fn clearance(&self, q: &[f64; N]) -> Option<Contact> {
        let bodies = self.bodies(q);
        let mut nearest: Option<Contact> = None;
        for (i, a) in bodies.iter().enumerate() {
            for b in &bodies[i + 1..] {
                if !self.checked(a, b) {
                    continue;
                }
                for (pa, sa) in &a.parts {
                    for (pb, sb) in &b.parts {
                        let distance = match query::distance(pa, sa.as_ref(), pb, sb.as_ref()) {
                            Ok(distance) if distance > 0.0 => distance,
                            // Overlapping: the depth, from the contact.
                            Ok(_) => query::contact(pa, sa.as_ref(), pb, sb.as_ref(), 0.0)
                                .ok()
                                .flatten()
                                .map_or(0.0, |contact| contact.dist),
                            Err(_) => 0.0,
                        };
                        if nearest
                            .as_ref()
                            .is_none_or(|nearest| distance < nearest.distance)
                        {
                            nearest = Some(Contact {
                                first: a.name.into(),
                                second: b.name.into(),
                                distance,
                            });
                        }
                    }
                }
            }
        }
        nearest
    }

    fn object(&self, name: &str) -> Result<usize, SceneError> {
        self.objects
            .iter()
            .position(|object| object.name == name)
            .ok_or_else(|| SceneError::UnknownObject(name.into()))
    }

    fn link_index(&self, name: &str) -> Result<usize, SceneError> {
        self.model
            .links
            .iter()
            .position(|link| link.name == name)
            .ok_or_else(|| SceneError::UnknownLink(name.into()))
    }

    /// Every link frame in the world with the group at `q`.
    fn link_frames(&self, q: &[f64; N]) -> Vec<na::Isometry3<f64>> {
        let mut positions = self.positions.clone();
        for (&child, &position) in self.group.iter().zip(q) {
            positions[child] = position;
        }
        let mut frames = self.model.link_frames(|joint| positions[joint.child]);
        for frame in &mut frames {
            *frame = self.base * *frame;
        }
        frames
    }

    /// Links with geometry, then objects, posed in the world at `q`, with
    /// bounding boxes grown by half the widest padding each.
    fn bodies(&self, q: &[f64; N]) -> Vec<Body<'_>> {
        let frames = self.link_frames(q);
        let widest = self.widest_padding();
        let body = |name, role, parts| Body::new(name, role, parts, widest);
        let mut bodies = Vec::new();
        for (link, parts) in self.links.iter().enumerate() {
            if !parts.is_empty() {
                let parts = parts
                    .iter()
                    .map(|part| (frames[link] * part.at, &part.shape))
                    .collect();
                bodies.push(body(&self.model.links[link].name, Role::Link(link), parts));
            }
        }
        for object in &self.objects {
            let (role, frame) = match &object.mount {
                Mount::World(pose) => (Role::World, *pose),
                Mount::Link { link, at, touch } => {
                    (Role::Carried { link: *link, touch }, frames[*link] * at)
                }
            };
            let parts = vec![(frame * object.part.at, &object.part.shape)];
            bodies.push(body(&object.name, role, parts));
        }
        bodies
    }

    fn checked(&self, a: &Body<'_>, b: &Body<'_>) -> bool {
        let exempt = match (&a.role, &b.role) {
            (Role::World, Role::World) => true,
            (Role::Link(link), Role::Carried { link: on, touch })
            | (Role::Carried { link: on, touch }, Role::Link(link)) => {
                link == on || touch.contains(link)
            }
            _ => false,
        };
        !exempt && !self.allowed.is_allowed(a.name, b.name)
    }

    fn each_contact(&self, q: &[f64; N], mut visit: impl FnMut(Contact) -> ControlFlow<()>) {
        let bodies = self.bodies(q);
        for (i, a) in bodies.iter().enumerate() {
            for b in &bodies[i + 1..] {
                if !self.checked(a, b) {
                    continue;
                }
                let padding = self.pair_padding(a.name, b.name);
                let mut closest: Option<f64> = None;
                for ((pa, sa), ba) in a.parts.iter().zip(&a.bounds) {
                    for ((pb, sb), bb) in b.parts.iter().zip(&b.bounds) {
                        if !ba.intersects(bb) {
                            continue;
                        }
                        let distance =
                            match query::contact(pa, sa.as_ref(), pb, sb.as_ref(), padding) {
                                Ok(Some(contact)) => contact.dist,
                                Ok(None) => continue,
                                // Every shape built here is convex. Were a
                                // pair unsupported, it would count as touching
                                // rather than free.
                                Err(_) => 0.0,
                            };
                        closest = Some(closest.map_or(distance, |closest| closest.min(distance)));
                    }
                }
                if let Some(distance) = closest {
                    let contact = Contact { first: a.name.into(), second: b.name.into(), distance };
                    if visit(contact).is_break() {
                        return;
                    }
                }
            }
        }
    }
}

impl<const N: usize> CollisionCheck<N> for PlanningScene<N> {
    fn first_contact(&self, q: &[f64; N]) -> Option<Contact> {
        let mut first = None;
        self.each_contact(q, |contact| {
            first = Some(contact);
            ControlFlow::Break(())
        });
        first
    }
}

enum Role<'a> {
    Link(usize),
    World,
    Carried { link: usize, touch: &'a [usize] },
}

/// A link or object posed in the world.
struct Body<'a> {
    name: &'a str,
    role: Role<'a>,
    parts: Vec<(na::Isometry3<f64>, &'a SharedShape)>,
    bounds: Vec<Aabb>,
}

impl<'a> Body<'a> {
    fn new(
        name: &'a str,
        role: Role<'a>,
        parts: Vec<(na::Isometry3<f64>, &'a SharedShape)>,
        padding: f64,
    ) -> Self {
        let bounds = parts
            .iter()
            .map(|(pose, shape)| shape.compute_aabb(pose).loosened(padding / 2.0))
            .collect();
        Self { name, role, parts, bounds }
    }
}

/// The convex shape of `geometry` placed at `origin`.
fn part(
    geometry: &Geometry,
    origin: na::Isometry3<f64>,
    mesh_path: impl FnOnce(&str) -> PathBuf,
) -> Result<Part, SceneError> {
    let positive = |what: &str, values: &[f64]| {
        if values.iter().all(|v| v.is_finite() && *v > 0.0) {
            Ok(())
        } else {
            Err(SceneError::InvalidGeometry(format!(
                "{what} {values:?} must be positive"
            )))
        }
    };
    let (shape, at) = match geometry {
        Geometry::Box { size } => {
            positive("box size", size)?;
            (
                SharedShape::cuboid(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0),
                origin,
            )
        }
        Geometry::Sphere { radius } => {
            positive("sphere radius", &[*radius])?;
            (SharedShape::ball(*radius), origin)
        }
        // parry's cylinder runs along y, URDF's along z.
        Geometry::Cylinder { radius, length } => {
            positive("cylinder radius and length", &[*radius, *length])?;
            let along_z = na::UnitQuaternion::from_axis_angle(
                &na::Vector3::x_axis(),
                std::f64::consts::FRAC_PI_2,
            );
            (
                SharedShape::cylinder(length / 2.0, *radius),
                origin * na::Isometry3::from_parts(na::Translation3::identity(), along_z),
            )
        }
        Geometry::Capsule { radius, length } => {
            positive("capsule radius and length", &[*radius, *length])?;
            (SharedShape::capsule_z(length / 2.0, *radius), origin)
        }
        Geometry::Mesh { filename, scale } => {
            positive("mesh scale", scale)?;
            let path = mesh_path(filename);
            (hull(&path, scale)?, origin)
        }
    };
    Ok(Part { at, shape })
}

/// Convex parts of the solid the triangles bound (or of the shell, where
/// they bound none), each the hull of the mesh within one part.
fn decompose(vertices: &[[f64; 3]], triangles: &[[u32; 3]]) -> Result<SharedShape, String> {
    if triangles.is_empty() {
        return Err("no triangles".into());
    }
    if let Some(v) = vertices
        .iter()
        .position(|v| !v.iter().all(|c| c.is_finite()))
    {
        return Err(format!("vertex {v} is not finite"));
    }
    if let Some(t) = triangles
        .iter()
        .position(|t| t.iter().any(|&i| i as usize >= vertices.len()))
    {
        return Err(format!(
            "triangle {t} names a vertex beyond the {} given",
            vertices.len()
        ));
    }
    let points: Vec<na::Point3<f64>> = vertices.iter().map(|&v| v.into()).collect();
    let decomposition = VHACD::decompose(&VHACDParameters::default(), &points, triangles, true);
    let parts: Vec<_> = decomposition
        .compute_exact_convex_hulls(&points, triangles)
        .into_iter()
        .filter_map(|(vertices, indices)| SharedShape::convex_mesh(vertices, &indices))
        .map(|convex| (na::Isometry3::identity(), convex))
        .collect();
    if parts.is_empty() {
        return Err("the triangles enclose no volume".into());
    }
    Ok(SharedShape::compound(parts))
}

fn hull(path: &Path, scale: &[f64; 3]) -> Result<SharedShape, SceneError> {
    let failed = |reason: String| SceneError::Mesh { path: path.to_owned(), reason };
    let points: Vec<na::Point3<f64>> = mesh::vertices(path)
        .map_err(failed)?
        .into_iter()
        .map(|[x, y, z]| na::Point3::new(x * scale[0], y * scale[1], z * scale[2]))
        .collect();
    let flat = || failed(format!("the {} vertices span no volume", points.len()));
    let hull = SharedShape::convex_hull(&points).ok_or_else(flat)?;
    // Coplanar vertices still give a hull, a flat one. 1 mm³.
    let volume = hull.mass_properties(1.0).mass();
    if volume.is_nan() || volume <= 1e-9 {
        return Err(flat());
    }
    Ok(hull)
}
