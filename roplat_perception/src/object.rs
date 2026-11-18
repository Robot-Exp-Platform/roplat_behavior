//! Objects as perception reports them: one type for what a detector, a
//! segmenter, a marker reader, a radar's object list or a fusion step says
//! it saw, so that a tracker, a planning scene or a language model's
//! grounding reads any of them.

use nalgebra::{Isometry3, Vector3};
use roplat_msgs::{FrameId, Stamp};

/// One object seen at one instant.
///
/// Only the class, the stamp and the frame are always known; every other
/// field is `None` when the source does not give it, never a stand-in value.
/// A camera detector gives a pixel box and a score; a segmenter adds a mask;
/// a depth or lidar step adds a pose and a size; a marker reader a pose and
/// an identity; a tracker a track number.
///
/// ```
/// use roplat_msgs::{FrameId, Stamp, clock};
/// use roplat_perception::{ObjectObservation, PixelBox};
///
/// let cup = ObjectObservation {
///     score: Some(0.92),
///     pixel_box: Some(PixelBox { min: [120.0, 80.0], max: [180.0, 160.0] }),
///     ..ObjectObservation::new("cup", Stamp::new(clock::MONOTONIC, 0), FrameId::new(3))
/// };
/// assert_eq!(cup.pixel_box.unwrap().area(), 4800.0);
/// assert!(cup.pose.is_none());
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct ObjectObservation {
    /// What the object is, as the source names it: a label of its class
    /// set (`"cup"`), an open-vocabulary query it answers, or a marker
    /// family (`"tag36h11"`).
    pub class: String,
    /// The source's index of the class, when it has one.
    pub class_id: Option<u32>,
    /// The source's confidence in `[0, 1]`; `None` from a source that gives
    /// none (a marker decoded without errors, a radar object list).
    pub score: Option<f32>,
    /// When the object was seen: the stamp of the image or scan it was
    /// found in, not when the result was computed.
    pub stamp: Stamp,
    /// The frame `pose` is in, and the image's optical frame for a source
    /// that gives only pixels.
    pub frame: FrameId,
    /// The object's own frame in `frame`: its centre, with its axes along
    /// the edges `size` measures.
    pub pose: Option<Isometry3<f64>>,
    /// Metres: the edges of the object's bounding box along its own axes.
    pub size: Option<Vector3<f64>>,
    /// Where the object appears in the image it was found in.
    pub pixel_box: Option<PixelBox>,
    /// Which pixels of that image are the object's.
    pub mask: Option<ObjectMask>,
    /// The object's colour, 8-bit sRGB, as the source measured it.
    pub color: Option<[u8; 3]>,
    /// The same number for the same object across observations, from a
    /// tracker or a source that tells objects apart (a marker's id, a
    /// radar's object id); unique among the objects of one instant.
    pub track: Option<u64>,
}

impl ObjectObservation {
    /// An observation of `class` with nothing else known, the base a source
    /// fills in: `ObjectObservation { score, pixel_box, ..ObjectObservation::new(…) }`.
    pub fn new(class: impl Into<String>, stamp: Stamp, frame: FrameId) -> Self {
        Self {
            class: class.into(),
            class_id: None,
            score: None,
            stamp,
            frame,
            pose: None,
            size: None,
            pixel_box: None,
            mask: None,
            color: None,
            track: None,
        }
    }
}

/// An axis-aligned box in image pixels: `min` is its top-left corner and
/// `max` its bottom-right, `[column, row]`, with pixel `(u, v)` covering
/// `[u, u + 1) × [v, v + 1)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PixelBox {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl PixelBox {
    /// Square pixels; zero for an empty or inverted box.
    pub fn area(&self) -> f64 {
        (self.max[0] - self.min[0]).max(0.0) * (self.max[1] - self.min[1]).max(0.0)
    }

    pub fn centre(&self) -> [f64; 2] {
        [
            (self.min[0] + self.max[0]) / 2.0,
            (self.min[1] + self.max[1]) / 2.0,
        ]
    }

    /// Intersection over union with `other`, in `[0, 1]`; zero when the
    /// boxes do not overlap or both are empty.
    pub fn iou(&self, other: &Self) -> f64 {
        let overlap = Self {
            min: [self.min[0].max(other.min[0]), self.min[1].max(other.min[1])],
            max: [self.max[0].min(other.max[0]), self.max[1].min(other.max[1])],
        }
        .area();
        let union = self.area() + other.area() - overlap;
        if union > 0.0 { overlap / union } else { 0.0 }
    }
}

/// Which pixels of an image belong to an object: one bit per pixel, row by
/// row from the top-left pixel, eight to a byte with the first pixel in the
/// most significant bit. That is `numpy.packbits` with its default bit
/// order over the flattened mask, so masks pass to and from Python models
/// unchanged.
#[derive(Clone, PartialEq, Eq)]
pub struct ObjectMask {
    width: u32,
    height: u32,
    bits: Vec<u8>,
}

impl std::fmt::Debug for ObjectMask {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ObjectMask")
            .field("width", &self.width)
            .field("height", &self.height)
            .field("area", &self.area())
            .finish()
    }
}

impl ObjectMask {
    /// A mask from its packed bits; `None` unless `bits` holds exactly
    /// `⌈width × height / 8⌉` bytes with the padding bits after the last
    /// pixel clear.
    pub fn from_bits(width: u32, height: u32, bits: Vec<u8>) -> Option<Self> {
        let pixels = width as usize * height as usize;
        if bits.len() != pixels.div_ceil(8) {
            return None;
        }
        let spare = bits.len() * 8 - pixels;
        if spare > 0 && bits[bits.len() - 1] & ((1u8 << spare) - 1) != 0 {
            return None;
        }
        Some(Self { width, height, bits })
    }

    /// A `width × height` mask with the pixels for which `inside(u, v)`
    /// holds.
    pub fn from_fn(width: u32, height: u32, mut inside: impl FnMut(u32, u32) -> bool) -> Self {
        let pixels = width as usize * height as usize;
        let mut bits = vec![0u8; pixels.div_ceil(8)];
        for v in 0..height {
            for u in 0..width {
                if inside(u, v) {
                    let i = v as usize * width as usize + u as usize;
                    bits[i / 8] |= 0x80 >> (i % 8);
                }
            }
        }
        Self { width, height, bits }
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    /// The packed bits.
    pub fn bits(&self) -> &[u8] {
        &self.bits
    }

    /// Whether pixel `(u, v)` is the object's; `false` outside the image.
    pub fn contains(&self, u: u32, v: u32) -> bool {
        if u >= self.width || v >= self.height {
            return false;
        }
        let i = v as usize * self.width as usize + u as usize;
        self.bits[i / 8] & (0x80 >> (i % 8)) != 0
    }

    /// The object's pixels.
    pub fn area(&self) -> usize {
        self.bits.iter().map(|b| b.count_ones() as usize).sum()
    }

    /// The smallest box holding every pixel of the object; `None` for an
    /// empty mask.
    pub fn bounding_box(&self) -> Option<PixelBox> {
        let mut found: Option<[u32; 4]> = None;
        for v in 0..self.height {
            for u in 0..self.width {
                if self.contains(u, v) {
                    let b = found.get_or_insert([u, v, u, v]);
                    *b = [b[0].min(u), b[1].min(v), b[2].max(u), b[3].max(v)];
                }
            }
        }
        found.map(|[u0, v0, u1, v1]| PixelBox {
            min: [f64::from(u0), f64::from(v0)],
            max: [f64::from(u1) + 1.0, f64::from(v1) + 1.0],
        })
    }
}
