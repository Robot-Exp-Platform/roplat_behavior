//! Segmentation images: which object or class each pixel of a camera frame
//! sees.

/// One id of a [`LabelImage`] and the class it belongs to.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct LabelClass {
    pub id: u32,
    /// The class name, as the source labels it (`"box"`, `"person"`,
    /// `"BACKGROUND"`).
    pub name: String,
}

/// A label for every pixel of a camera frame: a semantic segmentation (one
/// id per class) or an instance segmentation (one id per object, several
/// ids of one class), and the class of each id.
///
/// The image has the frame's size and orientation: `ids` holds `width ×
/// height` ids, row by row from the top-left pixel, as the
/// [`Camera`](crate::Camera) contract lays out frames. A camera that
/// segments its frames carries one of these in its frame type, beside the
/// colour or depth image it labels, so that labels and pixels belong to one
/// capture; `conformance::check_segmentation` (feature `conformance`)
/// checks it.
///
/// A pixel that sees nothing labelled has
/// [`UNLABELLED`](Self::UNLABELLED), which `classes` does not list. A
/// source that names such pixels itself (Isaac Sim's `BACKGROUND` and
/// `UNLABELLED` classes) may list them as classes instead; the driver
/// documents which.
///
/// ```
/// use roplat_behavior::{LabelClass, LabelImage};
///
/// let labels = LabelImage {
///     width: 2,
///     height: 1,
///     ids: vec![7, LabelImage::UNLABELLED],
///     classes: vec![LabelClass { id: 7, name: "box".into() }],
/// };
/// assert_eq!(labels.class_at(0, 0), Some("box"));
/// assert_eq!(labels.class_at(1, 0), None);
/// ```
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LabelImage {
    /// Pixels per row.
    pub width: u32,
    /// Rows.
    pub height: u32,
    /// One id per pixel, row-major from the top-left pixel.
    pub ids: Vec<u32>,
    /// The class of every id the image holds, each id once.
    pub classes: Vec<LabelClass>,
}

impl LabelImage {
    /// The id of a pixel that sees nothing labelled.
    pub const UNLABELLED: u32 = u32::MAX;

    /// The id of pixel `(u, v)`: column `u` from the left, row `v` from the
    /// top. `None` outside the image.
    pub fn id_at(&self, u: u32, v: u32) -> Option<u32> {
        if u >= self.width || v >= self.height {
            return None;
        }
        self.ids
            .get(v as usize * self.width as usize + u as usize)
            .copied()
    }

    /// The class name of `id`; `None` for an id `classes` does not list,
    /// [`UNLABELLED`](Self::UNLABELLED) among them.
    pub fn class_of(&self, id: u32) -> Option<&str> {
        self.classes
            .iter()
            .find_map(|c| (c.id == id).then_some(c.name.as_str()))
    }

    /// The class name of pixel `(u, v)`; `None` outside the image and for
    /// a pixel without a listed class.
    pub fn class_at(&self, u: u32, v: u32) -> Option<&str> {
        self.class_of(self.id_at(u, v)?)
    }
}
