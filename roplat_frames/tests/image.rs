//! Image operations pixel by pixel against two independent references: the
//! published formulas, computed here in a different form from the library,
//! and the output of OpenCV 4.14.0 and GStreamer 1.20.3 for the same input.
//!
//! OpenCV computes in fixed point, so it may differ from the exact formula
//! by one step; those comparisons allow one step and count the exact
//! matches. The formula comparisons are exact, except that a value within
//! rounding error of a half may round either way.

#[path = "support/opencv_image.rs"]
mod opencv;

use nalgebra as na;
use opencv::*;
use roplat_frames::image::{BayerPattern, Filter, Image, ImageError, Rect, YCbCr};

/// Asserts `ours` is the formula's `reference` rounded: exactly, or either
/// neighbour when `reference` is within rounding error of a half. Clamped to
/// a byte first, as the library clamps.
fn rounds_to(ours: u8, reference: f64, what: &str) {
    let clamped = reference.clamp(0.0, 255.0);
    let near_half = (clamped.fract() - 0.5).abs() < 1e-6;
    let ok = if near_half {
        f64::from(ours) == clamped.floor() || f64::from(ours) == clamped.ceil()
    } else {
        f64::from(ours) == clamped.round()
    };
    assert!(ok, "{what}: {ours} where the formula gives {reference}");
}

/// Asserts each byte is within one step of OpenCV's; returns how many are
/// equal.
fn within_one_step(ours: &[u8], opencv: &[u8], what: &str) -> usize {
    assert_eq!(ours.len(), opencv.len(), "{what}");
    for (k, (a, b)) in ours.iter().zip(opencv).enumerate() {
        assert!(
            a.abs_diff(*b) <= 1,
            "{what}, byte {k}: {a} against OpenCV's {b}"
        );
    }
    ours.iter().zip(opencv).filter(|(a, b)| a == b).count()
}

fn rgb() -> Image {
    Image::new(8, 6, 3, RGB.to_vec()).unwrap()
}

#[test]
fn grey_is_bt601_luma() {
    let grey = rgb().rgb_to_gray().unwrap();
    for (p, &g) in RGB.as_chunks::<3>().0.iter().zip(grey.data()) {
        let (r, gr, b) = (f64::from(p[0]), f64::from(p[1]), f64::from(p[2]));
        rounds_to(g, 0.299 * r + 0.587 * gr + 0.114 * b, "grey");
    }
    let exact = within_one_step(grey.data(), &GRAY, "grey");
    println!("grey: {exact} of 48 equal to OpenCV, the rest within one step");
    let back = grey.gray_to_rgb().unwrap();
    assert!(
        back.data()
            .as_chunks::<3>()
            .0
            .iter()
            .zip(grey.data())
            .all(|(p, &g)| *p == [g, g, g])
    );
}

/// OpenCV's documented RGB to HSV, `R, G, B` scaled to `0..1`, then `H/2`,
/// `255 S`, `255 V`.
fn hsv_formula([r, g, b]: [f64; 3]) -> [f64; 3] {
    let v = r.max(g).max(b);
    let min = r.min(g).min(b);
    let s = if v != 0.0 { (v - min) / v } else { 0.0 };
    let mut h = if v == min {
        0.0
    } else if v == r {
        60.0 * (g - b) / (v - min)
    } else if v == g {
        120.0 + 60.0 * (b - r) / (v - min)
    } else {
        240.0 + 60.0 * (r - g) / (v - min)
    };
    if h < 0.0 {
        h += 360.0;
    }
    [h / 2.0, 255.0 * s, 255.0 * v]
}

#[test]
fn hsv_is_opencvs_eight_bit_hsv() {
    let hsv = rgb().rgb_to_hsv().unwrap();
    for (p, q) in RGB
        .as_chunks::<3>()
        .0
        .iter()
        .zip(hsv.data().as_chunks::<3>().0.iter())
    {
        let [h, s, v] = hsv_formula([p[0], p[1], p[2]].map(|c| f64::from(c) / 255.0));
        // A hue that rounds to 180 is 0.
        rounds_to(if q[0] == 0 && h > 179.0 { 180 } else { q[0] }, h, "hue");
        rounds_to(q[1], s, "saturation");
        rounds_to(q[2], v, "value");
    }
    // Hue wraps: one step from OpenCV's modulo 180.
    for (k, (a, b)) in hsv.data().iter().zip(&HSV).enumerate() {
        let step = if k % 3 == 0 {
            let d = a.abs_diff(*b);
            d.min(180 - d)
        } else {
            a.abs_diff(*b)
        };
        assert!(step <= 1, "byte {k}: {a} against OpenCV's {b}");
    }
    let exact = hsv.data().iter().zip(&HSV).filter(|(a, b)| a == b).count();
    println!("HSV: {exact} of 144 equal to OpenCV, the rest within one step");
    // Grey, the primaries and a hue just short of 360°.
    let special = Image::new(
        5,
        1,
        3,
        vec![0, 0, 0, 90, 90, 90, 255, 0, 0, 0, 0, 255, 255, 0, 1],
    )
    .unwrap()
    .rgb_to_hsv()
    .unwrap();
    assert_eq!(
        special.data(),
        [0, 0, 0, 0, 0, 90, 0, 255, 255, 120, 255, 255, 0, 255, 255]
    );
}

/// HSV to RGB by the closed form that needs no sectors: with
/// `k = (n + H / 60) mod 6`, `f(n) = V − V S max(0, min(k, 4 − k, 1))`, and
/// `R, G, B = f(5), f(3), f(1)`.
fn rgb_formula(h_degrees: f64, s: f64, v: f64) -> [f64; 3] {
    let f = |n: f64| {
        let k = (n + h_degrees / 60.0) % 6.0;
        v - v * s * k.min(4.0 - k).clamp(0.0, 1.0)
    };
    [f(5.0), f(3.0), f(1.0)]
}

#[test]
fn hsv_converts_back_to_rgb() {
    let hsv = Image::new(8, 6, 3, HSV_IN.to_vec()).unwrap();
    let rgb = hsv.hsv_to_rgb().unwrap();
    for (p, q) in HSV_IN
        .as_chunks::<3>()
        .0
        .iter()
        .zip(rgb.data().as_chunks::<3>().0.iter())
    {
        let reference = rgb_formula(
            2.0 * f64::from(p[0]),
            f64::from(p[1]) / 255.0,
            f64::from(p[2]),
        );
        for (ours, reference) in q.iter().zip(reference) {
            rounds_to(*ours, reference, "hsv to rgb");
        }
    }
    let exact = within_one_step(rgb.data(), &HSV_IN_RGB, "hsv to rgb");
    println!("HSV to RGB: {exact} of 144 equal to OpenCV, the rest within one step");
    // There and back: the hue is kept to 2°, the saturation to 1/255.
    let round_trip = self::rgb().rgb_to_hsv().unwrap().hsv_to_rgb().unwrap();
    let worst = round_trip
        .data()
        .iter()
        .zip(&RGB)
        .map(|(a, b)| a.abs_diff(*b))
        .max()
        .unwrap();
    assert!(worst <= 5, "{worst}");
}

/// The inverse of the `Y′CbCr` matrix built from `Kr` and `Kb`, by
/// numerical inversion rather than the closed form the library uses.
fn decode_formula(encoding: YCbCr, y: u8, cb: u8, cr: u8) -> [f64; 3] {
    let (kr, kb, full) = match encoding {
        YCbCr::Bt601Limited => (0.299, 0.114, false),
        YCbCr::Bt601Full => (0.299, 0.114, true),
        YCbCr::Bt709Limited => (0.2126, 0.0722, false),
    };
    let kg = 1.0 - kr - kb;
    let forward = na::Matrix3::new(
        kr,
        kg,
        kb,
        -kr / (2.0 * (1.0 - kb)),
        -kg / (2.0 * (1.0 - kb)),
        0.5,
        0.5,
        -kg / (2.0 * (1.0 - kr)),
        -kb / (2.0 * (1.0 - kr)),
    );
    let (y, cb, cr) = (f64::from(y), f64::from(cb) - 128.0, f64::from(cr) - 128.0);
    let normalised = if full {
        na::Vector3::new(y / 255.0, cb / 255.0, cr / 255.0)
    } else {
        na::Vector3::new((y - 16.0) / 219.0, cb / 224.0, cr / 224.0)
    };
    let rgb = forward.try_inverse().unwrap() * normalised * 255.0;
    [rgb.x, rgb.y, rgb.z]
}

#[test]
fn nv12_and_yuyv_decode_by_the_inverse_matrix() {
    for encoding in [YCbCr::Bt601Limited, YCbCr::Bt601Full, YCbCr::Bt709Limited] {
        let rgb = Image::from_nv12(&NV12, 8, 6, encoding).unwrap();
        let (luma, chroma) = NV12.split_at(48);
        for y in 0..6 {
            for x in 0..8 {
                let at = (y / 2) * 8 + (x / 2) * 2;
                let reference =
                    decode_formula(encoding, luma[y * 8 + x], chroma[at], chroma[at + 1]);
                for (ours, reference) in rgb.pixel(x, y).unwrap().iter().zip(reference) {
                    rounds_to(*ours, reference, "nv12");
                }
            }
        }
        let rgb = Image::from_yuyv(&YUYV, 8, 6, encoding).unwrap();
        for (k, q) in YUYV.as_chunks::<4>().0.iter().enumerate() {
            for (half, y) in [q[0], q[2]].into_iter().enumerate() {
                let reference = decode_formula(encoding, y, q[1], q[3]);
                let pixel = &rgb.data()[(2 * k + half) * 3..][..3];
                for (ours, reference) in pixel.iter().zip(reference) {
                    rounds_to(*ours, reference, "yuyv");
                }
            }
        }
    }
    // OpenCV decodes BT.601 limited range.
    let nv12 = Image::from_nv12(&NV12, 8, 6, YCbCr::Bt601Limited).unwrap();
    let yuyv = Image::from_yuyv(&YUYV, 8, 6, YCbCr::Bt601Limited).unwrap();
    let exact = within_one_step(nv12.data(), &NV12_RGB, "nv12")
        + within_one_step(yuyv.data(), &YUYV_RGB, "yuyv");
    println!("NV12 and YUYV: {exact} of 288 equal to OpenCV, the rest within one step");
}

/// The seven 100 % colour bars of GStreamer's `videotestsrc
/// pattern=smpte100` at 16 × 2: pixels 0-1, 2-3, 4-5, 6-8, 9-10, 11-12 and
/// 13-15 are white, yellow, cyan, green, magenta, red and blue.
fn colour_bars() -> Image {
    let bars: [[u8; 3]; 7] = [
        [255, 255, 255],
        [255, 255, 0],
        [0, 255, 255],
        [0, 255, 0],
        [255, 0, 255],
        [255, 0, 0],
        [0, 0, 255],
    ];
    let row: Vec<u8> = [0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 6]
        .iter()
        .flat_map(|&b| bars[b])
        .collect();
    Image::new(16, 2, 3, row.repeat(2)).unwrap()
}

/// `gst-launch-1.0 videotestsrc num-buffers=1 pattern=smpte100 !
/// video/x-raw,format=<YUY2|NV12>,width=16,height=2,colorimetry=<bt601|bt709>
/// ! filesink`, GStreamer 1.20.3: byte for byte. Pairs that straddle two
/// bars carry their mean chroma.
#[test]
fn encoding_matches_gstreamer_colour_bars() {
    let bars = colour_bars();
    let yuy2_601 = [
        235, 128, 235, 128, 210, 16, 210, 146, 170, 166, 170, 16, 145, 54, 145, 34, 145, 128, 106,
        128, 106, 146, 81, 231, 81, 165, 41, 175, 41, 240, 41, 110,
    ];
    let yuy2_709 = [
        235, 128, 235, 128, 219, 16, 219, 138, 188, 154, 188, 16, 173, 42, 173, 26, 173, 128, 78,
        128, 78, 158, 63, 235, 63, 171, 32, 179, 32, 240, 32, 118,
    ];
    let nv12_601_chroma = [
        128, 128, 16, 146, 166, 16, 54, 34, 128, 128, 146, 231, 165, 175, 240, 110,
    ];
    let nv12_709_chroma = [
        128, 128, 16, 138, 154, 16, 42, 26, 128, 128, 158, 235, 171, 179, 240, 118,
    ];
    for (encoding, yuy2, chroma) in [
        (YCbCr::Bt601Limited, yuy2_601, nv12_601_chroma),
        (YCbCr::Bt709Limited, yuy2_709, nv12_709_chroma),
    ] {
        assert_eq!(
            bars.rgb_to_yuyv(encoding).unwrap(),
            yuy2.repeat(2),
            "{encoding:?}"
        );
        let luma: Vec<u8> = yuy2.iter().step_by(2).copied().collect();
        let nv12 = [luma.clone(), luma, chroma.to_vec()].concat();
        assert_eq!(bars.rgb_to_nv12(encoding).unwrap(), nv12, "{encoding:?}");
    }
}

/// The encoders against the published coefficient tables: BT.601's
/// studio-range table (219 and 224 times the luma and colour-difference
/// weights, to three decimals) and the JFIF table of ITU-T T.871 for full
/// range; chroma averaged over the pixels that share it.
#[test]
fn encoding_matches_the_coefficient_tables() {
    let bt601 = |[r, g, b]: [f64; 3]| {
        [
            16.0 + (65.481 * r + 128.553 * g + 24.966 * b) / 255.0,
            128.0 + (-37.797 * r - 74.203 * g + 112.0 * b) / 255.0,
            128.0 + (112.0 * r - 93.786 * g - 18.214 * b) / 255.0,
        ]
    };
    let jfif = |[r, g, b]: [f64; 3]| {
        [
            0.299 * r + 0.587 * g + 0.114 * b,
            128.0 - 0.168736 * r - 0.331264 * g + 0.5 * b,
            128.0 + 0.5 * r - 0.418688 * g - 0.081312 * b,
        ]
    };
    let image = rgb();
    let at = |x: usize, y: usize| {
        let p = image.pixel(x, y).unwrap();
        [p[0], p[1], p[2]].map(f64::from)
    };
    // Coefficients to three decimals: allow the step either side of a value
    // within 0.002 of a half.
    let close = |ours: u8, reference: f64| {
        let tolerance = if (reference.fract() - 0.5).abs() < 0.002 {
            1
        } else {
            0
        };
        assert!(
            (f64::from(ours) - reference.round()).abs() <= f64::from(tolerance),
            "{ours} where the table gives {reference}"
        );
    };
    for (encoding, table) in [
        (YCbCr::Bt601Limited, &bt601 as &dyn Fn([f64; 3]) -> [f64; 3]),
        (YCbCr::Bt601Full, &jfif),
    ] {
        let yuyv = image.rgb_to_yuyv(encoding).unwrap();
        for y in 0..6 {
            for x in (0..8).step_by(2) {
                let (a, b) = (table(at(x, y)), table(at(x + 1, y)));
                let q = &yuyv[(y * 8 + x) * 2..][..4];
                close(q[0], a[0]);
                close(q[1], (a[1] + b[1]) / 2.0);
                close(q[2], b[0]);
                close(q[3], (a[2] + b[2]) / 2.0);
            }
        }
        let nv12 = image.rgb_to_nv12(encoding).unwrap();
        for y in 0..6 {
            for x in 0..8 {
                close(nv12[y * 8 + x], table(at(x, y))[0]);
            }
        }
        for y in (0..6).step_by(2) {
            for x in (0..8).step_by(2) {
                let block = [at(x, y), at(x + 1, y), at(x, y + 1), at(x + 1, y + 1)].map(table);
                let q = &nv12[48 + (y / 2) * 8 + x..][..2];
                close(q[0], block.iter().map(|s| s[1]).sum::<f64>() / 4.0);
                close(q[1], block.iter().map(|s| s[2]).sum::<f64>() / 4.0);
            }
        }
    }
    // A pair of equal pixels comes back within two steps.
    let flat = Image::new(2, 2, 3, [90, 140, 30].repeat(4)).unwrap();
    for encoding in [YCbCr::Bt601Limited, YCbCr::Bt601Full, YCbCr::Bt709Limited] {
        let back = Image::from_yuyv(&flat.rgb_to_yuyv(encoding).unwrap(), 2, 2, encoding).unwrap();
        assert!(
            back.data()
                .iter()
                .zip(flat.data())
                .all(|(a, b)| a.abs_diff(*b) <= 2)
        );
        let back = Image::from_nv12(&flat.rgb_to_nv12(encoding).unwrap(), 2, 2, encoding).unwrap();
        assert!(
            back.data()
                .iter()
                .zip(flat.data())
                .all(|(a, b)| a.abs_diff(*b) <= 2)
        );
    }
}

#[test]
fn bayer_mosaics_demosaic_bilinearly() {
    let patterns = [
        (BayerPattern::Rggb, &BAYER_RGGB),
        (BayerPattern::Grbg, &BAYER_GRBG),
        (BayerPattern::Gbrg, &BAYER_GBRG),
        (BayerPattern::Bggr, &BAYER_BGGR),
    ];
    let mut exact = 0;
    for (pattern, reference) in patterns {
        let rgb = Image::from_bayer(&RAW, 8, 6, pattern).unwrap();
        // OpenCV copies its outermost pixels from the next ones in; the
        // inside follows the same bilinear rule.
        for y in 1..5 {
            for x in 1..7 {
                let at = (y * 8 + x) * 3;
                let ours = rgb.pixel(x, y).unwrap();
                exact += within_one_step(ours, &reference[at..at + 3], "bayer");
            }
        }
    }
    println!(
        "Bayer: {exact} of {} inner samples equal to OpenCV",
        4 * 24 * 3
    );
    assert_eq!(exact, 4 * 24 * 3);

    // A flat colour comes back everywhere, the edges included; a ramp
    // along x comes back exactly inside.
    for pattern in patterns.map(|p| p.0) {
        let colour = [200u8, 120, 40];
        let raw: Vec<u8> = (0..6 * 8)
            .map(|i| colour[pattern.channel(i % 8, i / 8)])
            .collect();
        let rgb = Image::from_bayer(&raw, 8, 6, pattern).unwrap();
        assert_eq!(rgb.data(), colour.repeat(48), "{pattern:?}");
        let ramp: Vec<u8> = (0..6 * 8).map(|i| 20 * (i % 8) as u8 + 10).collect();
        let rgb = Image::from_bayer(&ramp, 8, 6, pattern).unwrap();
        for y in 0..6 {
            for x in 1..7 {
                let v = 20 * x as u8 + 10;
                assert_eq!(
                    rgb.pixel(x, y).unwrap(),
                    [v, v, v],
                    "{pattern:?} at {x}, {y}"
                );
            }
        }
    }
}

/// Bilinear resampling by its definition: the source coordinate of
/// destination pixel `x` is `(x + 0.5) W / w − 0.5`, clamped to the image.
fn bilinear_formula(
    src: &[u8],
    w: usize,
    h: usize,
    x: usize,
    y: usize,
    dw: usize,
    dh: usize,
) -> f64 {
    let coordinate = |x: usize, dw: usize, w: usize| {
        let u = ((x as f64 + 0.5) * w as f64 / dw as f64 - 0.5)
            .max(0.0)
            .min((w - 1) as f64);
        let i = u as usize;
        (i, (i + 1).min(w - 1), u - i as f64)
    };
    let (x0, x1, fx) = coordinate(x, dw, w);
    let (y0, y1, fy) = coordinate(y, dh, h);
    let v = |x: usize, y: usize| f64::from(src[y * w + x]);
    (1.0 - fy) * ((1.0 - fx) * v(x0, y0) + fx * v(x1, y0))
        + fy * ((1.0 - fx) * v(x0, y1) + fx * v(x1, y1))
}

/// The box filter by refinement: each source pixel repeated `dw` times
/// across and `dh` times down makes a grid in which each destination pixel
/// is a plain block of `w × h` cells; its mean, rounded half up.
fn area_formula(src: &[u8], w: usize, h: usize, dw: usize, dh: usize) -> Vec<u8> {
    let fine = |fx: usize, fy: usize| u64::from(src[(fy / dh) * w + fx / dw]);
    let n = (w * h) as u64;
    let mut out = Vec::new();
    for y in 0..dh {
        for x in 0..dw {
            let sum: u64 = (y * h..(y + 1) * h)
                .flat_map(|fy| (x * w..(x + 1) * w).map(move |fx| (fx, fy)))
                .map(|(fx, fy)| fine(fx, fy))
                .sum();
            out.push(((2 * sum + n) / (2 * n)) as u8);
        }
    }
    out
}

/// A destination size and OpenCV's nearest, bilinear and area results.
type Expected = (usize, usize, &'static [u8], &'static [u8], &'static [u8]);

#[test]
fn resizing_follows_the_definitions_and_opencv() {
    let grey = Image::new(8, 6, 1, GRAY.to_vec()).unwrap();
    let cases: [Expected; 4] = [
        (
            5,
            4,
            &RESIZE_NEAREST_5X4,
            &RESIZE_LINEAR_5X4,
            &RESIZE_AREA_5X4,
        ),
        (
            3,
            2,
            &RESIZE_NEAREST_3X2,
            &RESIZE_LINEAR_3X2,
            &RESIZE_AREA_3X2,
        ),
        (
            11,
            9,
            &RESIZE_NEAREST_11X9,
            &RESIZE_LINEAR_11X9,
            &RESIZE_AREA_11X9,
        ),
        (
            16,
            12,
            &RESIZE_NEAREST_16X12,
            &RESIZE_LINEAR_16X12,
            &RESIZE_AREA_16X12,
        ),
    ];
    let (mut exact, mut total) = (0, 0);
    for (w, h, nearest, linear, area) in cases {
        let ours = grey.resize(w, h, Filter::Nearest).unwrap();
        for y in 0..h {
            for x in 0..w {
                let sx = ((x as f64 + 0.5) * 8.0 / w as f64).floor() as usize;
                let sy = ((y as f64 + 0.5) * 6.0 / h as f64).floor() as usize;
                assert_eq!(ours.pixel(x, y).unwrap()[0], GRAY[sy * 8 + sx], "{w} x {h}");
            }
        }
        assert_eq!(ours.data(), nearest, "nearest {w} x {h}");

        let ours = grey.resize(w, h, Filter::Bilinear).unwrap();
        for y in 0..h {
            for x in 0..w {
                rounds_to(
                    ours.pixel(x, y).unwrap()[0],
                    bilinear_formula(&GRAY, 8, 6, x, y, w, h),
                    "bilinear",
                );
            }
        }
        exact += within_one_step(ours.data(), linear, "bilinear");
        total += w * h;

        let ours = grey.resize(w, h, Filter::Area).unwrap();
        assert_eq!(
            ours.data(),
            area_formula(&GRAY, 8, 6, w, h),
            "area {w} x {h}"
        );
        // OpenCV's INTER_AREA is the box filter when shrinking, and
        // replicates pixels at an integer enlargement; at 11 x 9 it
        // interpolates instead.
        if (w, h) != (11, 9) {
            exact += within_one_step(ours.data(), area, "area");
            total += w * h;
        }
    }
    println!("resizing: {exact} of {total} bilinear and area pixels equal to OpenCV");
    // Every channel alike.
    let colour = rgb().resize(5, 4, Filter::Area).unwrap();
    for c in 0..3 {
        let plane: Vec<u8> = RGB.iter().skip(c).step_by(3).copied().collect();
        let expected = area_formula(&plane, 8, 6, 5, 4);
        let got: Vec<u8> = colour.data().iter().skip(c).step_by(3).copied().collect();
        assert_eq!(got, expected, "channel {c}");
    }
    // A 2 × 1 ramp doubled: 0, 25, 75, 100.
    let ramp = Image::new(2, 1, 1, vec![0, 100]).unwrap();
    assert_eq!(
        ramp.resize(4, 1, Filter::Bilinear).unwrap().data(),
        [0, 25, 75, 100]
    );
}

#[test]
fn cropping_and_bad_input() {
    let image = rgb();
    let crop = image
        .crop(Rect { x: 2, y: 1, width: 3, height: 4 })
        .unwrap();
    for y in 0..4 {
        for x in 0..3 {
            assert_eq!(crop.pixel(x, y), image.pixel(x + 2, y + 1));
        }
    }
    assert_eq!(crop.pixel(3, 0), None);
    let outside = Rect { x: 6, y: 0, width: 3, height: 1 };
    assert_eq!(
        image.crop(outside),
        Err(ImageError::OutOfBounds { region: outside, width: 8, height: 6 })
    );
    assert!(
        image
            .crop(Rect { x: 0, y: 0, width: 0, height: 1 })
            .is_err()
    );
    assert!(
        image
            .crop(Rect { x: usize::MAX, y: 0, width: 2, height: 1 })
            .is_err()
    );

    assert_eq!(
        Image::new(2, 2, 3, vec![0; 11]),
        Err(ImageError::Length { needed: 12, got: 11 })
    );
    assert!(matches!(
        Image::new(0, 2, 3, vec![]),
        Err(ImageError::Empty { .. })
    ));
    assert_eq!(
        image.rgb_to_gray().unwrap().rgb_to_hsv(),
        Err(ImageError::Channels { expected: 3, got: 1 })
    );
    assert_eq!(
        image.gray_to_rgb(),
        Err(ImageError::Channels { expected: 1, got: 3 })
    );
    let odd = image
        .crop(Rect { x: 0, y: 0, width: 3, height: 2 })
        .unwrap();
    assert_eq!(
        odd.rgb_to_yuyv(YCbCr::Bt601Limited),
        Err(ImageError::OddSize { width: 3, height: 2, rows_in_pairs: false })
    );
    let odd_rows = image
        .crop(Rect { x: 0, y: 0, width: 2, height: 3 })
        .unwrap();
    assert!(odd_rows.rgb_to_yuyv(YCbCr::Bt601Limited).is_ok());
    assert_eq!(
        odd_rows.rgb_to_nv12(YCbCr::Bt601Limited),
        Err(ImageError::OddSize { width: 2, height: 3, rows_in_pairs: true })
    );
    assert_eq!(
        Image::from_nv12(&NV12[..71], 8, 6, YCbCr::Bt601Limited),
        Err(ImageError::Length { needed: 72, got: 71 })
    );
    assert!(Image::from_bayer(&[0; 3], 3, 1, BayerPattern::Rggb).is_err());
    assert!(image.resize(0, 3, Filter::Nearest).is_err());
}
