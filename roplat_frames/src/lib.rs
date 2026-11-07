//! Coordinate frames of a robot cell: one tree of static, joint-driven and
//! dynamic transforms, looked up at an instant.
//!
//! - [`frame_id`] derives a [`FrameId`] from a frame name at compile time, so
//!   producers can fill `Header::frame` without holding a registry.
//! - [`FrameTree`] fixes the topology and the static transforms when it is
//!   built: URDF links and joints through [`FrameTreeBuilder::model`],
//!   calibrated extrinsics through [`FrameTreeBuilder::fixed`].
//! - [`FrameBuffer`] keeps a time history of joint positions and dynamic
//!   transforms on one clock and answers [`FrameBuffer::lookup`] at a
//!   [`Stamp`]. Writes and lookups do not allocate and never block.
//! - [`calibration`] solves a camera's mount from paired arm and target poses
//!   (hand-eye calibration), for the fixed edge it becomes.
//! - [`target`] finds a chessboard's corners in an image and the board's
//!   pose relative to the camera, the target pose each calibration sample
//!   needs.
//! - [`lens`] applies and removes the lens distortion a `CameraInfo`
//!   describes, and rectifies raw images for the pinhole code above.
//! - Feature `roplat` adds [`roplat::JointStateNode`], which records joint
//!   states into a borrowed buffer.
//! - Feature `scene` adds [`scene`]: planning-scene objects moved to their
//!   tracked frames, and depth readings placed at the camera pose of their
//!   stamp.
//!
//! There is no global registry or buffer: a `FrameBuffer` is an ordinary value
//! owned by its creating scope and lent to nodes as a resource.
//!
//! [`Stamp`]: roplat_msgs::Stamp

mod buffer;
pub mod calibration;
pub mod lens;
pub mod target;
mod tree;

#[cfg(feature = "roplat")]
pub mod roplat;
#[cfg(feature = "scene")]
pub mod scene;

pub use buffer::FrameBuffer;
pub use roplat_msgs::FrameId;
pub use tree::{FrameTree, FrameTreeBuilder, History, JointGroup};

/// The id of the frame named `name`: FNV-1a over its UTF-8 bytes, with `0`
/// (unspecified) mapped to `1`.
///
/// The same name has the same id in every process. A tree rejects two names
/// with the same id when it is built.
///
/// ```
/// use roplat_frames::{FrameId, frame_id};
///
/// const TOOL: FrameId = frame_id("tool");
/// assert_eq!(TOOL, frame_id("tool"));
/// assert_ne!(TOOL, FrameId::UNSPECIFIED);
/// ```
pub const fn frame_id(name: &str) -> FrameId {
    let bytes = name.as_bytes();
    let mut hash: u32 = 0x811c_9dc5;
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u32;
        hash = hash.wrapping_mul(0x0100_0193);
        i += 1;
    }
    FrameId::new(if hash == 0 { 1 } else { hash })
}

/// A tree that cannot be built, or a group handle that cannot be issued.
#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum TreeError {
    #[error("frame `{0}` has more than one parent")]
    MultipleParents(String),
    #[error("expected exactly one root frame, found {0:?}")]
    Roots(Vec<String>),
    #[error("frames {0:?} are on or below a cycle")]
    Cycle(Vec<String>),
    #[error("frames `{0}` and `{1}` have the same id")]
    IdCollision(String, String),
    #[error("joint `{joint}` is {kind:?}, which has no single-position transform")]
    UnsupportedJoint {
        joint: String,
        kind: roplat_description::JointKind,
    },
    #[error("unknown joint `{0}`")]
    UnknownJoint(String),
    #[error("more than one joint is named `{0}`")]
    AmbiguousJoint(String),
    #[error("joint `{0}` is fixed and cannot be in a group")]
    NotMovable(String),
    #[error("joint `{0}` is in more than one group")]
    GroupedTwice(String),
    #[error("more than one group is named `{0}`")]
    DuplicateGroup(String),
    #[error("group `{0}` has no joints")]
    EmptyGroup(String),
    #[error("history of `{0}` has zero capacity")]
    ZeroCapacity(String),
    #[error("pose of frame `{0}` is not finite or has a zero rotation")]
    InvalidPose(String),
    #[error("unknown group `{0}`")]
    UnknownGroup(String),
    #[error("group `{group}` has {len} joints, not {requested}")]
    GroupSize {
        group: String,
        len: usize,
        requested: usize,
    },
    #[error("no unique joint group ids remain for this tree")]
    GroupIdsExhausted,
    #[error("a frame buffer needs a named clock, not `clock::UNSPECIFIED`")]
    UnspecifiedClock,
}

/// A write or lookup that failed. Frames are named by id; resolve names with
/// [`FrameTree::name`]. Stamps are nanoseconds on the buffer's clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum FrameError {
    #[error("frame {:#010x} is not in the tree", .0.id)]
    UnknownFrame(FrameId),
    #[error("stamp is on clock {got}, the buffer on clock {expected}")]
    ClockMismatch { expected: u32, got: u32 },
    #[error("the transform of frame {:#010x} is not static", .0.id)]
    NotStatic(FrameId),
    #[error("no joint group positions the joint of frame {:#010x}", .0.id)]
    Unpositioned(FrameId),
    #[error("no samples yet for frame {:#010x}", .0.id)]
    NoData(FrameId),
    #[error("frame {:#010x}: {at} ns is after the latest sample at {latest} ns", .frame.id)]
    AfterLatest {
        frame: FrameId,
        at: i64,
        latest: i64,
    },
    #[error("frame {:#010x}: {at} ns is before the earliest sample at {earliest} ns", .frame.id)]
    BeforeHistory {
        frame: FrameId,
        at: i64,
        earliest: i64,
    },
    #[error("frame {:#010x}: samples at {before} ns and {after} ns are too far apart to interpolate", .frame.id)]
    Gap {
        frame: FrameId,
        before: i64,
        after: i64,
    },
    #[error("stamp {stamp} ns is earlier than the latest sample at {latest} ns")]
    OutOfOrder { stamp: i64, latest: i64 },
    #[error("value is not finite or has a zero rotation")]
    InvalidValue,
    #[error("frame {:#010x} does not have a dynamic transform", .0.id)]
    NotDynamic(FrameId),
    #[error("frame {:#010x} has a different parent than {:#010x}", .child.id, .parent.id)]
    WrongParent { child: FrameId, parent: FrameId },
    #[error("joint group handle belongs to another tree")]
    ForeignGroup,
    #[error("joint state has no positions")]
    MissingPositions,
}
