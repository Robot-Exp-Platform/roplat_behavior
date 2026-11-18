//! Lens distortion as a [`CameraInfo`] describes it: where an ideal pinhole
//! pixel appears in the raw image ([`distort`]), where a raw pixel would
//! appear through the ideal pinhole ([`undistort`]), where a point of the
//! optical frame lands in the raw image ([`project`]) and which ray a raw
//! pixel sees ([`ray`]), and a [`Rectify`] map that resamples raw images
//! into the rectified ones that [`target`]'s detector, `roplat_scene`'s
//! `depth_to_points` and other pinhole code expect, as ROS's `image_proc`
//! rectifies.
//!
//! The models are ROS's [`distortion_model`]s, with the coefficients `d`
//! applied to normalised image coordinates `(x, y)`, where a pixel is
//! `u = fx x + s y + cx`, `v = fy y + cy` with the skew `s = k[1]`, as the
//! `Camera` contract of `roplat_behavior` defines the camera matrix:
//!
//! - `PLUMB_BOB` (Brown–Conrady, as OpenCV's `projectPoints` with five
//!   coefficients): radial `1 + k1 r² + k2 r⁴ + k3 r⁶` and tangential
//!   `p1, p2`, with `d = [k1, k2, p1, p2, k3]`. ROS and OpenCV define
//!   plumb_bob with these five only, so a camera with any of `d[5..8]` set
//!   is refused rather than read as a rational polynomial.
//! - `RATIONAL_POLYNOMIAL`: the same tangential terms and the radial
//!   factor `(1 + k1 r² + k2 r⁴ + k3 r⁶) / (1 + k4 r² + k5 r⁴ + k6 r⁶)`,
//!   with `d = [k1, k2, p1, p2, k3, k4, k5, k6]`.
//! - `EQUIDISTANT` (fisheye, as OpenCV's `fisheye` module): the angle `θ`
//!   off the axis maps to `θ (1 + k1 θ² + k2 θ⁴ + k3 θ⁶ + k4 θ⁸)`, with
//!   `d = [k1, k2, k3, k4]`; a camera with any of `d[4..8]` set is refused.
//! - `NONE`: the identity.
//!
//! A camera these functions refuse (an unknown model, a camera matrix
//! without positive focal lengths, a coefficient that is not finite or that
//! the model does not have) gives `None`.
//!
//! [`Rectify::new`] keeps the camera matrix `k` and the image size
//! (OpenCV's `initUndistortRectifyMap` with the same `k` as the new
//! matrix), so the pinhole model for its output is [`rectified`]`(info)`;
//! [`Rectify::onto`] takes another camera matrix and size, as OpenCV's
//! `newCameraMatrix` and `newImageSize` do.
//!
//! ```
//! use roplat_frames::lens::{distort, project, ray, undistort};
//! use roplat_msgs::{CameraInfo, distortion_model};
//!
//! let info = CameraInfo {
//!     width: 640,
//!     height: 480,
//!     k: [600.0, 0.0, 319.5, 0.0, 600.0, 239.5, 0.0, 0.0, 1.0],
//!     distortion_model: distortion_model::PLUMB_BOB,
//!     d: [-0.28, 0.07, 0.001, -0.0005, 0.0, 0.0, 0.0, 0.0],
//!     ..Default::default()
//! };
//! let raw = distort(&info, [600.0, 40.0]).unwrap(); // pulled in by barrel distortion
//! assert!(raw[0] < 600.0 && raw[1] > 40.0);
//! let ideal = undistort(&info, raw).unwrap();
//! assert!((ideal[0] - 600.0).abs() < 1e-6 && (ideal[1] - 40.0).abs() < 1e-6);
//! // A point of the optical frame and the ray back from its pixel.
//! let pixel = project(&info, [0.2, -0.1, 1.5]).unwrap();
//! let [x, y] = ray(&info, pixel).unwrap();
//! assert!((x - 0.2 / 1.5).abs() < 1e-9 && (y + 0.1 / 1.5).abs() < 1e-9);
//! ```
//!
//! [`target`]: crate::target

use roplat_msgs::{CameraInfo, distortion_model};

/// The raw-image pixel where the ideal pinhole pixel `pixel` appears.
/// `None` for a camera the module refuses.
pub fn distort(info: &CameraInfo, pixel: [f64; 2]) -> Option<[f64; 2]> {
    let lens = Lens::new(info)?;
    let xy = lens.normalise(pixel);
    Some(lens.pixel(lens.distort(xy)))
}

/// The ideal pinhole pixel where the raw-image pixel `pixel` would appear
/// without distortion. `None` for a camera the module refuses, or a pixel
/// no ray maps to (beyond where the model folds back on itself, or past
/// 90° from the axis for a fisheye).
pub fn undistort(info: &CameraInfo, pixel: [f64; 2]) -> Option<[f64; 2]> {
    let lens = Lens::new(info)?;
    let xy = lens.normalise(pixel);
    Some(lens.pixel(lens.undistort(xy)?))
}

/// The raw-image pixel where `point`, `[x, y, z]` in metres in the optical
/// frame, lands through the lens. `None` for a point that is not in front
/// of the camera (`z` not positive, or a coordinate not finite) and for a
/// camera the module refuses.
pub fn project(info: &CameraInfo, point: [f64; 3]) -> Option<[f64; 2]> {
    let lens = Lens::new(info)?;
    lens.project(point)
}

/// The ray the raw-image pixel `pixel` sees, as the undistorted normalised
/// coordinates `[x / z, y / z]` of its points. `None` where the lens gives
/// no ray (see [`undistort`]) and for a camera the module refuses.
pub fn ray(info: &CameraInfo, pixel: [f64; 2]) -> Option<[f64; 2]> {
    let lens = Lens::new(info)?;
    lens.undistort(lens.normalise(pixel))
}

/// `info` for the rectified image: the same size and camera matrix, no
/// distortion.
pub fn rectified(info: &CameraInfo) -> CameraInfo {
    CameraInfo {
        distortion_model: distortion_model::NONE,
        d: [0.0; 8],
        ..*info
    }
}

/// For each pixel of the rectified image, where to read the raw image;
/// built once per camera and applied to every frame.
#[derive(Clone, Debug)]
pub struct Rectify {
    /// The raw image's size.
    raw: (usize, usize),
    /// The rectified camera.
    info: CameraInfo,
    /// Raw-image coordinates per rectified pixel, row-major; `None` where
    /// they fall outside the raw image.
    source: Vec<Option<[f32; 2]>>,
}

impl Rectify {
    /// The map for `info`'s image size and lens, keeping its camera matrix
    /// and size. `None` for a camera the module refuses.
    pub fn new(info: &CameraInfo) -> Option<Self> {
        Self::onto(info, &rectified(info))
    }

    /// The map from the raw images of `raw` onto those of the distortion-free
    /// camera `rectified`: its camera matrix `k` and its `width × height`,
    /// such as a matrix with a smaller focal length that keeps the whole
    /// raw view. `None` for a camera the module refuses, and for a
    /// `rectified` camera with a distortion model other than `NONE`.
    pub fn onto(raw: &CameraInfo, rectified: &CameraInfo) -> Option<Self> {
        let lens = Lens::new(raw)?;
        if rectified.distortion_model != distortion_model::NONE {
            return None;
        }
        let pinhole = Lens::new(rectified)?;
        let (w, h) = ((raw.width as f64) - 1.0, (raw.height as f64) - 1.0);
        let (width, height) = (rectified.width as usize, rectified.height as usize);
        let source = (0..height)
            .flat_map(|v| (0..width).map(move |u| [u as f64, v as f64]))
            .map(|pixel| {
                let [x, y] = lens.pixel(lens.distort(pinhole.normalise(pixel)));
                ((0.0..=w).contains(&x) && (0.0..=h).contains(&y)).then_some([x as f32, y as f32])
            })
            .collect();
        Some(Self {
            raw: (raw.width as usize, raw.height as usize),
            info: *rectified,
            source,
        })
    }

    /// The rectified camera: what pinhole code reads the output with.
    pub fn info(&self) -> &CameraInfo {
        &self.info
    }

    /// Width of the rectified image.
    pub fn width(&self) -> usize {
        self.info.width as usize
    }

    /// Height of the rectified image.
    pub fn height(&self) -> usize {
        self.info.height as usize
    }

    /// For each pixel of the rectified image, row-major, whether it has a
    /// source in the raw image. A pixel without one is 0 in
    /// [`bilinear`](Self::bilinear)'s output and `outside` in
    /// [`nearest`](Self::nearest)'s, values that say nothing about the
    /// scene; this tells them apart from a dark pixel or a real reading.
    pub fn mask(&self) -> Vec<bool> {
        self.source.iter().map(Option::is_some).collect()
    }

    /// Rectifies an 8-bit raw image of `channels` interleaved channels (1
    /// for grey, 3 for RGB) by bilinear interpolation; pixels whose source
    /// lies outside the raw image are 0, which [`mask`](Self::mask)
    /// marks.
    ///
    /// # Panics
    ///
    /// If `raw` holds fewer than the raw image's `width × height × channels`
    /// values.
    pub fn bilinear(&self, raw: &[u8], channels: usize) -> Vec<u8> {
        let (raw_width, raw_height) = self.raw;
        assert!(
            raw.len() >= raw_width * raw_height * channels,
            "raw image too small"
        );
        let at = |x: usize, y: usize, c: usize| f32::from(raw[(y * raw_width + x) * channels + c]);
        let mut out = vec![0; self.source.len() * channels];
        for (k, source) in self.source.iter().enumerate() {
            let Some([x, y]) = *source else { continue };
            let (x0, y0) = (x.floor() as usize, y.floor() as usize);
            let (x1, y1) = ((x0 + 1).min(raw_width - 1), (y0 + 1).min(raw_height - 1));
            let (fx, fy) = (x - x0 as f32, y - y0 as f32);
            for c in 0..channels {
                let top = at(x0, y0, c) * (1.0 - fx) + at(x1, y0, c) * fx;
                let bottom = at(x0, y1, c) * (1.0 - fx) + at(x1, y1, c) * fx;
                out[k * channels + c] = (top * (1.0 - fy) + bottom * fy).round() as u8;
            }
        }
        out
    }

    /// Rectifies a one-channel raw image by taking the nearest raw pixel,
    /// for values that must not be blended, such as depth (an average across
    /// an object's edge is a point in mid-air) or labels; pixels whose
    /// source lies outside the raw image are `outside`.
    ///
    /// # Panics
    ///
    /// If `raw` holds fewer than the raw image's `width × height` values.
    pub fn nearest<T: Copy>(&self, raw: &[T], outside: T) -> Vec<T> {
        let (raw_width, raw_height) = self.raw;
        assert!(raw.len() >= raw_width * raw_height, "raw image too small");
        self.source
            .iter()
            .map(|source| match *source {
                Some([x, y]) => raw[y.round() as usize * raw_width + x.round() as usize],
                None => outside,
            })
            .collect()
    }
}

/// A camera's intrinsics and lens model.
pub(crate) struct Lens {
    fx: f64,
    fy: f64,
    skew: f64,
    cx: f64,
    cy: f64,
    model: Model,
}

enum Model {
    None,
    /// `[k1, k2, p1, p2, k3, k4, k5, k6]`.
    Rational([f64; 8]),
    /// `[k1, k2, k3, k4]`.
    Equidistant([f64; 4]),
}

impl Lens {
    /// `None` for a camera the module refuses.
    pub(crate) fn new(info: &CameraInfo) -> Option<Self> {
        let (fx, fy, skew, cx, cy) = (info.k[0], info.k[4], info.k[1], info.k[2], info.k[5]);
        if !(fx > 0.0
            && fy > 0.0
            && fx.is_finite()
            && fy.is_finite()
            && skew.is_finite()
            && cx.is_finite()
            && cy.is_finite())
        {
            return None;
        }
        let d = info.d;
        // Coefficients the model has, and those it must leave zero.
        let (model, used) = match info.distortion_model {
            distortion_model::NONE => (Model::None, 0),
            distortion_model::PLUMB_BOB => (Model::Rational(d), 5),
            distortion_model::RATIONAL_POLYNOMIAL => (Model::Rational(d), 8),
            distortion_model::EQUIDISTANT => (Model::Equidistant([d[0], d[1], d[2], d[3]]), 4),
            _ => return None,
        };
        let (kept, unused) = d.split_at(used);
        if !kept.iter().all(|c| c.is_finite()) || (used > 0 && unused.iter().any(|&c| c != 0.0)) {
            return None;
        }
        Some(Self { fx, fy, skew, cx, cy, model })
    }

    pub(crate) fn normalise(&self, [u, v]: [f64; 2]) -> [f64; 2] {
        let y = (v - self.cy) / self.fy;
        [(u - self.cx - self.skew * y) / self.fx, y]
    }

    pub(crate) fn pixel(&self, [x, y]: [f64; 2]) -> [f64; 2] {
        [self.fx * x + self.skew * y + self.cx, self.fy * y + self.cy]
    }

    /// The raw pixel of a point of the optical frame; see [`project`].
    pub(crate) fn project(&self, [x, y, z]: [f64; 3]) -> Option<[f64; 2]> {
        if !(z > 0.0 && x.is_finite() && y.is_finite() && z.is_finite()) {
            return None;
        }
        Some(self.pixel(self.distort([x / z, y / z])))
    }

    pub(crate) fn distort(&self, [x, y]: [f64; 2]) -> [f64; 2] {
        match self.model {
            Model::None => [x, y],
            Model::Rational([k1, k2, p1, p2, k3, k4, k5, k6]) => {
                let r2 = x * x + y * y;
                let radial = (1.0 + r2 * (k1 + r2 * (k2 + r2 * k3)))
                    / (1.0 + r2 * (k4 + r2 * (k5 + r2 * k6)));
                [
                    x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
                    y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
                ]
            }
            Model::Equidistant(k) => {
                let r = x.hypot(y);
                if r < 1e-12 {
                    return [x, y];
                }
                let scale = equidistant(k, r.atan()) / r;
                [x * scale, y * scale]
            }
        }
    }

    /// The ideal normalised coordinates that distort to `xy`.
    pub(crate) fn undistort(&self, xy: [f64; 2]) -> Option<[f64; 2]> {
        match self.model {
            Model::None => Some(xy),
            Model::Rational(_) => {
                // Newton's method on distort(p) = xy, from p = xy, with a
                // finite-difference Jacobian.
                let mut p = xy;
                for _ in 0..50 {
                    let f = self.distort(p);
                    let (ex, ey) = (f[0] - xy[0], f[1] - xy[1]);
                    if ex.hypot(ey) < 1e-14 {
                        break;
                    }
                    let h = 1e-7;
                    let dx = self.distort([p[0] + h, p[1]]);
                    let dy = self.distort([p[0], p[1] + h]);
                    let (a, c) = ((dx[0] - f[0]) / h, (dx[1] - f[1]) / h);
                    let (b, d) = ((dy[0] - f[0]) / h, (dy[1] - f[1]) / h);
                    let det = a * d - b * c;
                    if det.abs() < 1e-12 {
                        return None;
                    }
                    p = [
                        p[0] - (d * ex - b * ey) / det,
                        p[1] - (a * ey - c * ex) / det,
                    ];
                }
                let f = self.distort(p);
                ((f[0] - xy[0]).hypot(f[1] - xy[1]) < 1e-10).then_some(p)
            }
            Model::Equidistant(k) => {
                let rd = xy[0].hypot(xy[1]);
                if rd < 1e-12 {
                    return Some(xy);
                }
                // Newton's method on θ_d(θ) = rd, from θ = rd.
                let mut theta = rd.min(1.5);
                for _ in 0..50 {
                    let t2 = theta * theta;
                    let slope = 1.0
                        + t2 * (3.0 * k[0]
                            + t2 * (5.0 * k[1] + t2 * (7.0 * k[2] + t2 * 9.0 * k[3])));
                    if slope <= 0.0 {
                        return None;
                    }
                    let step = (equidistant(k, theta) - rd) / slope;
                    theta -= step;
                    if step.abs() < 1e-15 {
                        break;
                    }
                }
                let half_turn = std::f64::consts::FRAC_PI_2;
                if !(theta > 0.0 && theta < half_turn) || (equidistant(k, theta) - rd).abs() > 1e-10
                {
                    return None;
                }
                let scale = theta.tan() / rd;
                Some([xy[0] * scale, xy[1] * scale])
            }
        }
    }
}

/// The equidistant model's distorted angle for the angle `theta` off the
/// axis.
fn equidistant(k: [f64; 4], theta: f64) -> f64 {
    let t2 = theta * theta;
    theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera(model: u32, d: [f64; 8]) -> CameraInfo {
        CameraInfo {
            width: 640,
            height: 480,
            k: [600.0, 0.0, 319.5, 0.0, 610.0, 239.5, 0.0, 0.0, 1.0],
            distortion_model: model,
            d,
            ..Default::default()
        }
    }

    fn lenses() -> [CameraInfo; 4] {
        [
            camera(distortion_model::NONE, [0.0; 8]),
            // A webcam's barrel distortion.
            camera(
                distortion_model::PLUMB_BOB,
                [-0.28, 0.07, 0.001, -0.0005, 0.0, 0.0, 0.0, 0.0],
            ),
            camera(
                distortion_model::RATIONAL_POLYNOMIAL,
                [0.8, -0.1, 0.0005, 0.0008, 0.02, 1.1, 0.1, 0.01],
            ),
            camera(
                distortion_model::EQUIDISTANT,
                [-0.013, -0.0175, 0.0097, -0.0031, 0.0, 0.0, 0.0, 0.0],
            ),
        ]
    }

    #[test]
    fn undistort_inverts_distort_over_the_image() {
        for info in lenses() {
            for v in (0..=480).step_by(40) {
                for u in (0..=640).step_by(40) {
                    let ideal = [u as f64, v as f64];
                    let raw = distort(&info, ideal).unwrap();
                    let back = undistort(&info, raw).unwrap();
                    let error = (back[0] - ideal[0]).hypot(back[1] - ideal[1]);
                    assert!(
                        error < 1e-6,
                        "model {}: {ideal:?} -> {raw:?} -> {back:?}",
                        info.distortion_model
                    );
                }
            }
        }
    }

    #[test]
    fn the_models_match_their_formulas() {
        // Radial only: a point at r = 0.5 moves to 0.5 (1 + k1 r²).
        let k1 = camera(
            distortion_model::PLUMB_BOB,
            [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        let raw = distort(&k1, [319.5 + 300.0, 239.5]).unwrap();
        assert!((raw[0] - (319.5 + 300.0 * (1.0 - 0.2 * 0.25))).abs() < 1e-9);
        assert!((raw[1] - 239.5).abs() < 1e-12);
        // Tangential only: p1 shifts y by p1 (r² + 2y²) and x by 2 p1 x y.
        let p1 = camera(
            distortion_model::PLUMB_BOB,
            [0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        let raw = distort(&p1, [319.5 + 60.0, 239.5 + 61.0]).unwrap(); // x = y = 0.1
        assert!((raw[0] - (319.5 + 600.0 * (0.1 + 2.0 * 0.01 * 0.01))).abs() < 1e-9);
        assert!((raw[1] - (239.5 + 610.0 * (0.1 + 0.01 * (0.02 + 0.02)))).abs() < 1e-9);
        // Fisheye without coefficients: θ maps to θ, so r = tan θ becomes θ.
        let fisheye = camera(distortion_model::EQUIDISTANT, [0.0; 8]);
        let raw = distort(&fisheye, [319.5 + 600.0, 239.5]).unwrap(); // r = 1, θ = 45°
        assert!((raw[0] - (319.5 + 600.0 * std::f64::consts::FRAC_PI_4)).abs() < 1e-9);
        // The principal point stays put.
        for info in lenses() {
            assert_eq!(distort(&info, [319.5, 239.5]), Some([319.5, 239.5]));
        }
    }

    #[test]
    fn unknown_models_and_bad_cameras_are_refused() {
        let unknown = camera(7, [0.0; 8]);
        assert_eq!(distort(&unknown, [1.0, 1.0]), None);
        assert!(Rectify::new(&unknown).is_none());
        let flat = CameraInfo { k: [0.0; 9], ..camera(distortion_model::NONE, [0.0; 8]) };
        assert_eq!(undistort(&flat, [1.0, 1.0]), None);
        // Past 90° off the axis a fisheye has no ray.
        let fisheye = camera(distortion_model::EQUIDISTANT, [0.0; 8]);
        assert_eq!(undistort(&fisheye, [319.5 + 600.0 * 2.0, 239.5]), None);
    }

    #[test]
    fn coefficients_a_model_does_not_have_are_refused() {
        // Plumb bob has five; k4..k6 belong to the rational polynomial.
        let mut plumb_bob = lenses()[1];
        assert!(Rectify::new(&plumb_bob).is_some());
        for i in 5..8 {
            plumb_bob.d[i] = 0.1;
            assert_eq!(distort(&plumb_bob, [10.0, 10.0]), None, "d[{i}]");
            plumb_bob.d[i] = 0.0;
        }
        // The same coefficients make a rational polynomial lens.
        let rational = CameraInfo {
            distortion_model: distortion_model::RATIONAL_POLYNOMIAL,
            d: [-0.28, 0.07, 0.001, -0.0005, 0.0, 0.1, 0.0, 0.0],
            ..plumb_bob
        };
        assert!(distort(&rational, [10.0, 10.0]).is_some());
        let mut fisheye = lenses()[3];
        fisheye.d[4] = 0.01;
        assert_eq!(distort(&fisheye, [10.0, 10.0]), None);
        let mut nan = lenses()[1];
        nan.d[0] = f64::NAN;
        assert_eq!(distort(&nan, [10.0, 10.0]), None);
    }

    #[test]
    fn skew_is_part_of_the_camera_matrix() {
        // u = fx x + s y + cx: a skewed pinhole shifts a pixel by s y.
        let mut info = lenses()[0];
        info.k[1] = 3.0;
        let pixel = project(&info, [0.1, 0.2, 1.0]).unwrap();
        assert!((pixel[0] - (600.0 * 0.1 + 3.0 * 0.2 + 319.5)).abs() < 1e-12);
        assert!((pixel[1] - (610.0 * 0.2 + 239.5)).abs() < 1e-12);
        let back = ray(&info, pixel).unwrap();
        assert!((back[0] - 0.1).abs() < 1e-12 && (back[1] - 0.2).abs() < 1e-12);
        // With a lens too.
        let mut barrel = lenses()[1];
        barrel.k[1] = 3.0;
        let pixel = project(&barrel, [0.3, -0.2, 1.1]).unwrap();
        let back = ray(&barrel, pixel).unwrap();
        assert!((back[0] - 0.3 / 1.1).abs() < 1e-10 && (back[1] + 0.2 / 1.1).abs() < 1e-10);
    }

    #[test]
    fn projection_and_ray_round_trip() {
        let p = [0.12, -0.08, 0.7];
        for info in lenses() {
            let pixel = project(&info, p).unwrap();
            let pinhole = [600.0 * p[0] / p[2] + 319.5, 610.0 * p[1] / p[2] + 239.5];
            let raw = distort(&info, pinhole).unwrap();
            assert!((raw[0] - pixel[0]).hypot(raw[1] - pixel[1]) < 1e-9);
            let back = ray(&info, pixel).unwrap();
            assert!(
                (back[0] - p[0] / p[2]).abs() < 1e-10 && (back[1] - p[1] / p[2]).abs() < 1e-10,
                "model {}: {back:?}",
                info.distortion_model
            );
        }
        let info = lenses()[0];
        assert_eq!(project(&info, [0.0, 0.0, -1.0]), None);
        assert_eq!(project(&info, [0.0, 0.0, 0.0]), None);
        assert_eq!(project(&info, [f64::NAN, 0.0, 1.0]), None);
    }

    #[test]
    fn rectifying_moves_pixels_to_their_ideal_place() {
        let info = lenses()[1];
        let map = Rectify::new(&info).unwrap();
        // A raw image whose value encodes the column: rectified, each pixel
        // holds the column it was read from.
        let raw: Vec<u16> = (0..480).flat_map(|_| 0..640u16).collect();
        let columns = map.nearest(&raw, u16::MAX);
        let ideal = [100.0, 60.0];
        let source = distort(&info, ideal).unwrap();
        assert_eq!(columns[60 * 640 + 100], source[0].round() as u16);
        // Barrel distortion pulls the view inwards: every rectified pixel
        // has a source, and the raw image's rim is cropped away.
        assert!(columns.iter().all(|&c| c != u16::MAX));
        assert!(map.mask().iter().all(|&m| m));
        assert!(columns[240 * 640] > 0 && columns[240 * 640 + 639] < 639);
        // Pincushion pushes it outwards: the rectified corners have none,
        // and the mask says so.
        let pincushion = camera(
            distortion_model::PLUMB_BOB,
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        let map = Rectify::new(&pincushion).unwrap();
        let grey = map.bilinear(&vec![200; 640 * 480], 1);
        assert_eq!((grey[0], grey[240 * 640 + 320]), (0, 200));
        let mask = map.mask();
        assert_eq!((mask[0], mask[240 * 640 + 320]), (false, true));
        assert!(grey.iter().zip(&mask).all(|(&g, &m)| (g == 200) == m));
        // No distortion: the identity.
        let none = Rectify::new(&lenses()[0]).unwrap();
        assert_eq!(none.nearest(&raw, 0), raw);
        let rgb: Vec<u8> = (0..640 * 480 * 3).map(|k| (k % 251) as u8).collect();
        assert_eq!(none.bilinear(&rgb, 3), rgb);
        assert_eq!(rectified(&info).distortion_model, distortion_model::NONE);
        assert_eq!(rectified(&info).k, info.k);
        assert_eq!(*Rectify::new(&info).unwrap().info(), rectified(&info));
    }

    #[test]
    fn rectifying_onto_another_camera_matrix_and_size() {
        let info = lenses()[1];
        // A third of the focal length at half the size: the whole raw view,
        // and more, in a quarter of the pixels.
        let half = CameraInfo {
            width: 320,
            height: 240,
            k: [200.0, 0.0, 159.5, 0.0, 200.0, 119.5, 0.0, 0.0, 1.0],
            ..rectified(&info)
        };
        let map = Rectify::onto(&info, &half).unwrap();
        assert_eq!((map.width(), map.height()), (320, 240));
        assert_eq!(map.info(), &half);
        let raw: Vec<u16> = (0..480).flat_map(|_| 0..640u16).collect();
        let columns = map.nearest(&raw, u16::MAX);
        assert_eq!(columns.len(), 320 * 240);
        // Each output pixel reads the raw pixel nearest to where its ray
        // lands.
        for (u, v) in [(40, 30), (160, 120), (300, 200), (2, 3)] {
            let ray = [(u as f64 - 159.5) / 200.0, (v as f64 - 119.5) / 200.0];
            let source = project(&info, [ray[0], ray[1], 1.0]).unwrap();
            let inside = (0.0..=639.0).contains(&source[0]) && (0.0..=479.0).contains(&source[1]);
            let column = columns[v * 320 + u];
            if inside {
                assert!(
                    (f64::from(column) - source[0]).abs() <= 0.5 + 1e-3,
                    "({u}, {v})"
                );
            } else {
                assert_eq!(column, u16::MAX, "({u}, {v})");
            }
        }
        // Barrel distortion pulls the raw view in, so the wider camera's
        // corners see past it: they have no source, its centre has one.
        let mask = map.mask();
        assert_eq!((mask[0], mask[120 * 320 + 160]), (false, true));
        // A target that is not a pinhole is refused.
        assert!(Rectify::onto(&info, &info).is_none());
    }
}
