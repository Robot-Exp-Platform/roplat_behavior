//! General processing of 8-bit images: cropping, resizing by nearest
//! neighbour, bilinear interpolation or pixel area, and conversion between
//! RGB, grey, HSV, the Y′CbCr layouts cameras deliver (NV12, YUYV) and raw
//! Bayer mosaics.
//!
//! An [`Image`] is `width × height` pixels of `channels` interleaved bytes,
//! row by row from the top-left pixel, without padding. What the channels
//! mean (RGB, grey, HSV) is the caller's to know, as with the images
//! [`lens::Rectify`](crate::lens::Rectify) and [`target`](crate::target)
//! take; each conversion states what it reads and writes.
//!
//! The definitions, each checked pixel by pixel against an independent
//! computation of the published formula in the tests:
//!
//! - **Grey**: BT.601 luma, `0.299 R + 0.587 G + 0.114 B`, rounded (OpenCV's
//!   `COLOR_RGB2GRAY`).
//! - **HSV**: OpenCV's 8-bit HSV (`COLOR_RGB2HSV`): `V = max(R, G, B)`,
//!   `S = 255 (V − min) / V`, the hue in degrees halved so that it fits a
//!   byte, `0..180`, a hue that rounds to 180 wrapping to 0.
//! - **Y′CbCr** ([`YCbCr`]): the matrix of BT.601 or BT.709 and the
//!   limited (video) or full range. Encoding averages the chroma of the
//!   pixels that share a sample (a pair for YUYV, a 2 × 2 block for NV12);
//!   decoding gives each pixel its pair's or block's chroma, without
//!   interpolation.
//! - **Bayer**: bilinear demosaicing with mirrored edges.
//! - **Resizing** ([`Filter`]): pixel centres at half-integer coordinates,
//!   as OpenCV's `resize` and Pillow place them.
//!
//! Results are rounded half away from zero and clamped to `0..=255`. Every
//! operation returns a new image: one allocation per frame, small next to
//! the per-pixel work.
//!
//! ```
//! use roplat_frames::image::{Filter, Image, Rect};
//!
//! // A 4 × 2 RGB image: red, green, blue and white over black.
//! let mut data = vec![255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255];
//! data.extend([0; 12]);
//! let image = Image::new(4, 2, 3, data).unwrap();
//! let grey = image.rgb_to_gray().unwrap();
//! assert_eq!(grey.data(), [76, 150, 29, 255, 0, 0, 0, 0]);
//! let top = image.crop(Rect { x: 1, y: 0, width: 2, height: 1 }).unwrap();
//! assert_eq!(top.data(), [0, 255, 0, 0, 0, 255]);
//! let half = grey.resize(2, 1, Filter::Area).unwrap();
//! assert_eq!(half.data(), [57, 71]);
//! ```

/// An 8-bit image of interleaved channels, rows without padding.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Image {
    width: usize,
    height: usize,
    channels: usize,
    data: Vec<u8>,
}

/// A rectangle of pixels: the top-left pixel `(x, y)` and the size.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Rect {
    pub x: usize,
    pub y: usize,
    pub width: usize,
    pub height: usize,
}

/// How [`Image::resize`] computes each pixel. Destination pixel `x` is
/// centred at `(x + 0.5) · s` in source pixels, `s` the ratio of the source
/// to the destination size.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Filter {
    /// The source pixel under the centre: `floor((x + 0.5) · s)` (OpenCV's
    /// `INTER_NEAREST_EXACT`, Pillow's `NEAREST`).
    Nearest,
    /// Linear in each direction between the four source pixels around
    /// `(x + 0.5) · s − 0.5`, the coordinate clamped to the image so that
    /// the border is repeated (OpenCV's `INTER_LINEAR`, in floating point).
    Bilinear,
    /// The mean of the source pixels under the destination pixel's
    /// footprint `[x · s, (x + 1) · s)`, each weighted by the area it
    /// covers: a box filter. When shrinking this is OpenCV's `INTER_AREA`;
    /// when enlarging, OpenCV switches to a bilinear rule and this does not.
    Area,
}

/// The colour filter over the first two pixels of the first two rows of a
/// Bayer mosaic.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BayerPattern {
    /// `R G` over `G B`.
    Rggb,
    /// `G R` over `B G`.
    Grbg,
    /// `G B` over `R G`.
    Gbrg,
    /// `B G` over `G R`.
    Bggr,
}

impl BayerPattern {
    /// The channel (0 red, 1 green, 2 blue) sampled at pixel `(x, y)`.
    pub const fn channel(self, x: usize, y: usize) -> usize {
        let rows = match self {
            Self::Rggb => [[0, 1], [1, 2]],
            Self::Grbg => [[1, 0], [2, 1]],
            Self::Gbrg => [[1, 2], [0, 1]],
            Self::Bggr => [[2, 1], [1, 0]],
        };
        rows[y & 1][x & 1]
    }
}

/// The matrix and quantisation of 8-bit Y′CbCr samples. With `Kr` and `Kb`
/// the matrix's red and blue luma weights, `Y′ = Kr R + (1 − Kr − Kb) G +
/// Kb B`, `Cb = (B − Y′) / (2 (1 − Kb))` and `Cr = (R − Y′) / (2 (1 − Kr))`
/// on `R, G, B` in `0..=1`; limited range stores `16 + 219 Y′` and
/// `128 + 224 C`, full range `255 Y′` and `128 + 255 C`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum YCbCr {
    /// BT.601 (`Kr = 0.299`, `Kb = 0.114`), limited range: V4L2's default
    /// for YUYV and NV12 in the sRGB and SMPTE 170M colorspaces, and
    /// standard-definition video.
    Bt601Limited,
    /// BT.601, full range: JPEG (JFIF).
    Bt601Full,
    /// BT.709 (`Kr = 0.2126`, `Kb = 0.0722`), limited range: high-definition
    /// video.
    Bt709Limited,
}

impl YCbCr {
    /// `(Kr, Kb, full range)`.
    const fn parameters(self) -> (f64, f64, bool) {
        match self {
            Self::Bt601Limited => (0.299, 0.114, false),
            Self::Bt601Full => (0.299, 0.114, true),
            Self::Bt709Limited => (0.2126, 0.0722, false),
        }
    }

    /// The unrounded `Y′, Cb, Cr` samples of an RGB pixel.
    fn encode(self, [r, g, b]: [u8; 3]) -> [f64; 3] {
        let (kr, kb, full) = self.parameters();
        let (r, g, b) = (f64::from(r) / 255.0, f64::from(g) / 255.0, f64::from(b) / 255.0);
        let y = kr * r + (1.0 - kr - kb) * g + kb * b;
        let cb = (b - y) / (2.0 * (1.0 - kb));
        let cr = (r - y) / (2.0 * (1.0 - kr));
        if full {
            [255.0 * y, 128.0 + 255.0 * cb, 128.0 + 255.0 * cr]
        } else {
            [16.0 + 219.0 * y, 128.0 + 224.0 * cb, 128.0 + 224.0 * cr]
        }
    }

    /// The RGB pixel of `Y′, Cb, Cr` samples.
    fn decode(self, y: u8, cb: u8, cr: u8) -> [u8; 3] {
        let (kr, kb, full) = self.parameters();
        let (y, cb, cr) = if full {
            (f64::from(y) / 255.0, (f64::from(cb) - 128.0) / 255.0, (f64::from(cr) - 128.0) / 255.0)
        } else {
            (
                (f64::from(y) - 16.0) / 219.0,
                (f64::from(cb) - 128.0) / 224.0,
                (f64::from(cr) - 128.0) / 224.0,
            )
        };
        let r = y + 2.0 * (1.0 - kr) * cr;
        let b = y + 2.0 * (1.0 - kb) * cb;
        let g = (y - kr * r - kb * b) / (1.0 - kr - kb);
        [r, g, b].map(|c| byte(255.0 * c))
    }
}

/// Why an image could not be made or processed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ImageError {
    #[error("an image needs a positive width, height and channel count, not {width} x {height} x {channels}")]
    Empty {
        width: usize,
        height: usize,
        channels: usize,
    },
    #[error("{got} bytes where the image needs {needed}")]
    Length { needed: usize, got: usize },
    #[error("the operation reads {expected} channels, the image has {got}")]
    Channels { expected: usize, got: usize },
    #[error("{region:?} is not inside the {width} x {height} image")]
    OutOfBounds {
        region: Rect,
        width: usize,
        height: usize,
    },
    #[error("the layout needs an even width{}, not {width} x {height}", if *.rows_in_pairs { " and height" } else { "" })]
    OddSize {
        width: usize,
        height: usize,
        /// Whether the height must be even too (NV12).
        rows_in_pairs: bool,
    },
}

/// Rounds half away from zero and clamps to a byte.
fn byte(v: f64) -> u8 {
    v.round().clamp(0.0, 255.0) as u8
}

impl Image {
    /// The image of `width × height` pixels of `channels` bytes in `data`.
    pub fn new(
        width: usize,
        height: usize,
        channels: usize,
        data: Vec<u8>,
    ) -> Result<Self, ImageError> {
        if width == 0 || height == 0 || channels == 0 {
            return Err(ImageError::Empty { width, height, channels });
        }
        let needed = width * height * channels;
        if data.len() != needed {
            return Err(ImageError::Length { needed, got: data.len() });
        }
        Ok(Self { width, height, channels, data })
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn channels(&self) -> usize {
        self.channels
    }

    /// The bytes, row by row.
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    pub fn into_data(self) -> Vec<u8> {
        self.data
    }

    /// The channels of pixel `(x, y)`; `None` outside the image.
    pub fn pixel(&self, x: usize, y: usize) -> Option<&[u8]> {
        (x < self.width && y < self.height).then(|| {
            let at = (y * self.width + x) * self.channels;
            &self.data[at..at + self.channels]
        })
    }

    fn expect_channels(&self, expected: usize) -> Result<(), ImageError> {
        if self.channels == expected {
            Ok(())
        } else {
            Err(ImageError::Channels { expected, got: self.channels })
        }
    }

    /// The pixels of `region`.
    pub fn crop(&self, region: Rect) -> Result<Image, ImageError> {
        let inside = region.width > 0
            && region.height > 0
            && region.x.checked_add(region.width).is_some_and(|r| r <= self.width)
            && region.y.checked_add(region.height).is_some_and(|b| b <= self.height);
        if !inside {
            return Err(ImageError::OutOfBounds {
                region,
                width: self.width,
                height: self.height,
            });
        }
        let row = region.width * self.channels;
        let mut data = Vec::with_capacity(row * region.height);
        for y in region.y..region.y + region.height {
            let at = (y * self.width + region.x) * self.channels;
            data.extend_from_slice(&self.data[at..at + row]);
        }
        Ok(Image { width: region.width, height: region.height, channels: self.channels, data })
    }

    /// The image resampled to `width × height` pixels by `filter`, every
    /// channel alike.
    pub fn resize(&self, width: usize, height: usize, filter: Filter) -> Result<Image, ImageError> {
        if width == 0 || height == 0 {
            return Err(ImageError::Empty { width, height, channels: self.channels });
        }
        let c = self.channels;
        let mut data = vec![0; width * height * c];
        match filter {
            Filter::Nearest => {
                // floor((x + 0.5) · W / w) in integers.
                let xs: Vec<usize> =
                    (0..width).map(|x| ((2 * x + 1) * self.width) / (2 * width)).collect();
                for y in 0..height {
                    let sy = ((2 * y + 1) * self.height) / (2 * height);
                    for (x, &sx) in xs.iter().enumerate() {
                        let from = (sy * self.width + sx) * c;
                        let to = (y * width + x) * c;
                        data[to..to + c].copy_from_slice(&self.data[from..from + c]);
                    }
                }
            }
            Filter::Bilinear => {
                let xs: Vec<(usize, usize, f64)> =
                    (0..width).map(|x| linear_taps(x, width, self.width)).collect();
                for y in 0..height {
                    let (y0, y1, fy) = linear_taps(y, height, self.height);
                    for (x, &(x0, x1, fx)) in xs.iter().enumerate() {
                        for k in 0..c {
                            let at = |sx: usize, sy: usize| f64::from(self.data[(sy * self.width + sx) * c + k]);
                            let top = at(x0, y0) + (at(x1, y0) - at(x0, y0)) * fx;
                            let bottom = at(x0, y1) + (at(x1, y1) - at(x0, y1)) * fx;
                            data[(y * width + x) * c + k] = byte(top + (bottom - top) * fy);
                        }
                    }
                }
            }
            Filter::Area => {
                let xs: Vec<Vec<(usize, u64)>> =
                    (0..width).map(|x| area_taps(x, width, self.width)).collect();
                // Weights are overlaps in units of 1/w and 1/h of a source
                // pixel; a destination pixel's add up to W · H.
                let total = (self.width * self.height) as u64;
                for y in 0..height {
                    let ys = area_taps(y, height, self.height);
                    for (x, xw) in xs.iter().enumerate() {
                        for k in 0..c {
                            let mut sum = 0u64;
                            for &(sy, wy) in &ys {
                                for &(sx, wx) in xw {
                                    sum += wx * wy * u64::from(self.data[(sy * self.width + sx) * c + k]);
                                }
                            }
                            // Exact, rounded half up.
                            data[(y * width + x) * c + k] = ((2 * sum + total) / (2 * total)) as u8;
                        }
                    }
                }
            }
        }
        Ok(Image { width, height, channels: c, data })
    }

    /// One grey channel from three RGB channels: BT.601 luma.
    pub fn rgb_to_gray(&self) -> Result<Image, ImageError> {
        self.expect_channels(3)?;
        // In thousandths, exactly, rounded half up.
        let luma = |p: &[u8]| {
            (299 * u32::from(p[0]) + 587 * u32::from(p[1]) + 114 * u32::from(p[2]) + 500) / 1000
        };
        let data = self.data.chunks_exact(3).map(|p| luma(p) as u8).collect();
        Ok(Image { width: self.width, height: self.height, channels: 1, data })
    }

    /// Three equal RGB channels from one grey channel.
    pub fn gray_to_rgb(&self) -> Result<Image, ImageError> {
        self.expect_channels(1)?;
        let data = self.data.iter().flat_map(|&g| [g, g, g]).collect();
        Ok(Image { width: self.width, height: self.height, channels: 3, data })
    }

    /// OpenCV's 8-bit HSV from RGB: hue in `0..180` (degrees halved),
    /// saturation and value in `0..=255`. A grey pixel has hue and
    /// saturation 0.
    pub fn rgb_to_hsv(&self) -> Result<Image, ImageError> {
        self.expect_channels(3)?;
        let mut data = Vec::with_capacity(self.data.len());
        for p in self.data.chunks_exact(3) {
            let (r, g, b) = (f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
            let v = r.max(g).max(b);
            let spread = v - r.min(g).min(b);
            let s = if v > 0.0 { 255.0 * spread / v } else { 0.0 };
            let degrees = if spread == 0.0 {
                0.0
            } else if v == r {
                60.0 * (g - b) / spread
            } else if v == g {
                120.0 + 60.0 * (b - r) / spread
            } else {
                240.0 + 60.0 * (r - g) / spread
            };
            let degrees = if degrees < 0.0 { degrees + 360.0 } else { degrees };
            let h = (degrees / 2.0).round() as u32 % 180;
            data.extend([h as u8, byte(s), p[0].max(p[1]).max(p[2])]);
        }
        Ok(Image { width: self.width, height: self.height, channels: 3, data })
    }

    /// RGB from OpenCV's 8-bit HSV (the inverse of [`Image::rgb_to_hsv`] up
    /// to its rounding). A hue past 179 is taken modulo 180.
    pub fn hsv_to_rgb(&self) -> Result<Image, ImageError> {
        self.expect_channels(3)?;
        let mut data = Vec::with_capacity(self.data.len());
        for p in self.data.chunks_exact(3) {
            let sector = f64::from(p[0] % 180) * 2.0 / 60.0;
            let (s, v) = (f64::from(p[1]) / 255.0, f64::from(p[2]));
            let chroma = v * s;
            let x = chroma * (1.0 - (sector % 2.0 - 1.0).abs());
            let (r, g, b) = match sector as u32 {
                0 => (chroma, x, 0.0),
                1 => (x, chroma, 0.0),
                2 => (0.0, chroma, x),
                3 => (0.0, x, chroma),
                4 => (x, 0.0, chroma),
                _ => (chroma, 0.0, x),
            };
            let m = v - chroma;
            data.extend([byte(r + m), byte(g + m), byte(b + m)]);
        }
        Ok(Image { width: self.width, height: self.height, channels: 3, data })
    }

    /// RGB from NV12: a plane of `width × height` luma samples, then
    /// `height / 2` rows of interleaved `Cb Cr` pairs, one per 2 × 2 block.
    /// Width and height must be even.
    pub fn from_nv12(
        data: &[u8],
        width: usize,
        height: usize,
        encoding: YCbCr,
    ) -> Result<Image, ImageError> {
        check_layout(width, height, true)?;
        let needed = width * height * 3 / 2;
        if data.len() != needed {
            return Err(ImageError::Length { needed, got: data.len() });
        }
        let (luma, chroma) = data.split_at(width * height);
        let mut rgb = Vec::with_capacity(width * height * 3);
        for y in 0..height {
            for x in 0..width {
                let at = (y / 2) * width + (x / 2) * 2;
                rgb.extend(encoding.decode(luma[y * width + x], chroma[at], chroma[at + 1]));
            }
        }
        Image::new(width, height, 3, rgb)
    }

    /// NV12 from RGB: full-resolution luma, then the mean chroma of each
    /// 2 × 2 block. Width and height must be even.
    pub fn rgb_to_nv12(&self, encoding: YCbCr) -> Result<Vec<u8>, ImageError> {
        self.expect_channels(3)?;
        check_layout(self.width, self.height, true)?;
        let (w, h) = (self.width, self.height);
        let samples: Vec<[f64; 3]> = self
            .data
            .chunks_exact(3)
            .map(|p| encoding.encode([p[0], p[1], p[2]]))
            .collect();
        let mut out: Vec<u8> = samples.iter().map(|s| byte(s[0])).collect();
        out.reserve(w * h / 2);
        for y in (0..h).step_by(2) {
            for x in (0..w).step_by(2) {
                let block = [y * w + x, y * w + x + 1, (y + 1) * w + x, (y + 1) * w + x + 1];
                for k in 1..3 {
                    out.push(byte(block.iter().map(|&i| samples[i][k]).sum::<f64>() / 4.0));
                }
            }
        }
        Ok(out)
    }

    /// RGB from YUYV (YUY2): `Y0 Cb Y1 Cr` for each pair of pixels in a row.
    /// The width must be even.
    pub fn from_yuyv(
        data: &[u8],
        width: usize,
        height: usize,
        encoding: YCbCr,
    ) -> Result<Image, ImageError> {
        check_layout(width, height, false)?;
        let needed = width * height * 2;
        if data.len() != needed {
            return Err(ImageError::Length { needed, got: data.len() });
        }
        let mut rgb = Vec::with_capacity(width * height * 3);
        for q in data.chunks_exact(4) {
            rgb.extend(encoding.decode(q[0], q[1], q[3]));
            rgb.extend(encoding.decode(q[2], q[1], q[3]));
        }
        Image::new(width, height, 3, rgb)
    }

    /// YUYV from RGB: both lumas of each pair of pixels and their mean
    /// chroma. The width must be even.
    pub fn rgb_to_yuyv(&self, encoding: YCbCr) -> Result<Vec<u8>, ImageError> {
        self.expect_channels(3)?;
        check_layout(self.width, self.height, false)?;
        let mut out = Vec::with_capacity(self.width * self.height * 2);
        for pair in self.data.chunks_exact(6) {
            let [y0, cb0, cr0] = encoding.encode([pair[0], pair[1], pair[2]]);
            let [y1, cb1, cr1] = encoding.encode([pair[3], pair[4], pair[5]]);
            out.extend([byte(y0), byte((cb0 + cb1) / 2.0), byte(y1), byte((cr0 + cr1) / 2.0)]);
        }
        Ok(out)
    }

    /// RGB from an 8-bit Bayer mosaic of `width × height` samples, one byte
    /// each, by bilinear interpolation: a pixel keeps its own colour; a
    /// missing green is the mean of the four side neighbours, a missing red
    /// or blue the mean of the two or four nearest samples of that colour.
    /// Edges are mirrored, which lands on the same colour of the pattern.
    /// Width and height must be at least 2.
    pub fn from_bayer(
        data: &[u8],
        width: usize,
        height: usize,
        pattern: BayerPattern,
    ) -> Result<Image, ImageError> {
        if width < 2 || height < 2 {
            return Err(ImageError::Empty { width, height, channels: 1 });
        }
        let needed = width * height;
        if data.len() != needed {
            return Err(ImageError::Length { needed, got: data.len() });
        }
        let mirror = |i: isize, len: usize| -> usize {
            let last = len as isize - 1;
            (if i < 0 { -i } else if i > last { 2 * last - i } else { i }) as usize
        };
        let at = |x: isize, y: isize| f64::from(data[mirror(y, height) * width + mirror(x, width)]);
        let mut rgb = Vec::with_capacity(needed * 3);
        for y in 0..height {
            for x in 0..width {
                let (xi, yi) = (x as isize, y as isize);
                let own = pattern.channel(x, y);
                let mut pixel = [0.0; 3];
                pixel[own] = at(xi, yi);
                if own == 1 {
                    // The row's other colour left and right, the column's
                    // above and below.
                    pixel[pattern.channel(x + 1, y)] = (at(xi - 1, yi) + at(xi + 1, yi)) / 2.0;
                    pixel[pattern.channel(x, y + 1)] = (at(xi, yi - 1) + at(xi, yi + 1)) / 2.0;
                } else {
                    pixel[1] = (at(xi - 1, yi) + at(xi + 1, yi) + at(xi, yi - 1) + at(xi, yi + 1)) / 4.0;
                    pixel[2 - own] = (at(xi - 1, yi - 1)
                        + at(xi + 1, yi - 1)
                        + at(xi - 1, yi + 1)
                        + at(xi + 1, yi + 1))
                        / 4.0;
                }
                rgb.extend(pixel.map(byte));
            }
        }
        Image::new(width, height, 3, rgb)
    }
}

fn check_layout(width: usize, height: usize, rows_in_pairs: bool) -> Result<(), ImageError> {
    if width == 0 || height == 0 {
        return Err(ImageError::Empty { width, height, channels: 1 });
    }
    if width % 2 != 0 || (rows_in_pairs && height % 2 != 0) {
        return Err(ImageError::OddSize { width, height, rows_in_pairs });
    }
    Ok(())
}

/// The two source pixels and the weight of the second for destination
/// pixel `x` of `size`, from a source of `source` pixels.
fn linear_taps(x: usize, size: usize, source: usize) -> (usize, usize, f64) {
    let u = ((x as f64 + 0.5) * source as f64 / size as f64 - 0.5).clamp(0.0, (source - 1) as f64);
    let x0 = u.floor() as usize;
    (x0, (x0 + 1).min(source - 1), u - x0 as f64)
}

/// The source pixels under destination pixel `x` of `size`, from a source
/// of `source` pixels, with their overlaps in units of `1/size` of a source
/// pixel: the footprint is `[x · source, (x + 1) · source)` and source pixel
/// `i` spans `[i · size, (i + 1) · size)`.
fn area_taps(x: usize, size: usize, source: usize) -> Vec<(usize, u64)> {
    let (start, end) = (x * source, (x + 1) * source);
    (start / size..end.div_ceil(size))
        .map(|i| {
            let overlap = end.min((i + 1) * size) - start.max(i * size);
            (i, overlap as u64)
        })
        .collect()
}
