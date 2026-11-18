//! A chessboard seen through a lens with strong barrel distortion: the
//! rectified image gives the board's pose as an ideal pinhole would, and so
//! do the raw image's corners once undistorted; the raw corners taken as
//! pinhole pixels do not.

use std::sync::OnceLock;

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};
use roplat_frames::{
    lens::{Rectify, project, rectified, undistort},
    target::{
        Chessboard, DetectOptions, TargetPose, TargetPoseError, detect_chessboard, target_pose,
    },
};
use roplat_msgs::{CameraInfo, distortion_model};

const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const FOCAL: f64 = 600.0;
const BOARD: Chessboard = Chessboard { columns: 9, rows: 6, square: 0.025 };

/// About 15 px of barrel distortion at the image's left and right edges.
fn info() -> CameraInfo {
    CameraInfo {
        width: WIDTH as u32,
        height: HEIGHT as u32,
        k: [FOCAL, 0.0, 319.5, 0.0, FOCAL, 239.5, 0.0, 0.0, 1.0],
        distortion_model: distortion_model::PLUMB_BOB,
        d: [-0.3, 0.12, 0.0005, -0.0008, -0.02, 0.0, 0.0, 0.0],
        ..Default::default()
    }
}

/// The board as the distorted camera at `board_in_camera` records it: as
/// in `target.rs`, but each sample's ray is the one its raw pixel sees.
fn render(board_in_camera: &Isometry3<f64>, seed: u64) -> Vec<u8> {
    let info = info();
    let camera_in_board = board_in_camera.inverse();
    let eye = camera_in_board.translation.vector;
    let s = BOARD.square;
    let shade = |x: f64, y: f64| -> f64 {
        let (a, b) = ((x / s).floor() as i64 + 1, (y / s).floor() as i64 + 1);
        let inside = |v: i64, n: usize| (0..=n as i64).contains(&v);
        let margin = |v: f64, n: usize| v >= -2.0 * s && v <= (n as f64 + 1.0) * s;
        if inside(a, BOARD.columns) && inside(b, BOARD.rows) {
            if (a + b) % 2 == 0 { 25.0 } else { 225.0 }
        } else if margin(x, BOARD.columns) && margin(y, BOARD.rows) {
            225.0
        } else {
            120.0
        }
    };
    let mut state = seed | 1;
    let mut image = Vec::with_capacity(WIDTH * HEIGHT);
    for v in 0..HEIGHT {
        for u in 0..WIDTH {
            let mut sum = 0.0;
            for sy in 0..3 {
                for sx in 0..3 {
                    let raw = [
                        u as f64 + (sx as f64 - 1.0) / 3.0,
                        v as f64 + (sy as f64 - 1.0) / 3.0,
                    ];
                    let [px, py] = undistort(&info, raw).expect("every raw pixel has a ray");
                    let ray = Vector3::new((px - 319.5) / FOCAL, (py - 239.5) / FOCAL, 1.0);
                    let direction = camera_in_board.rotation * ray;
                    let t = -eye.z / direction.z;
                    sum += if t > 0.0 {
                        let hit = eye + direction * t;
                        shade(hit.x, hit.y)
                    } else {
                        120.0
                    };
                }
            }
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let noise = (state % 9) as f64 - 4.0;
            image.push((sum / 9.0 + noise).clamp(0.0, 255.0) as u8);
        }
    }
    image
}

/// A camera at `eye` (in the board frame) looking at the board's centre,
/// turned by `roll` about its view axis; the board in the camera frame.
fn view(eye: Vector3<f64>, roll: f64) -> Isometry3<f64> {
    let centre = Vector3::new(4.0 * BOARD.square, 2.5 * BOARD.square, 0.0);
    let z = (centre - eye).normalize();
    let x = z.cross(&Vector3::y()).normalize();
    let y = z.cross(&x);
    let look = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[x, y, z]));
    let rotation = UnitQuaternion::from_rotation_matrix(&look)
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), roll);
    Isometry3::from_parts(Translation3::from(eye), rotation).inverse()
}

/// Close views, so the board reaches into the strongly distorted rim.
fn views() -> Vec<Isometry3<f64>> {
    [
        (0.12, 0.08, -0.3, 0.3),
        (0.1, 0.05, -0.33, 2.9),
        (0.2, 0.0, -0.34, -0.25),
    ]
    .into_iter()
    .map(|(x, y, z, roll)| view(Vector3::new(x, y, z), roll))
    .collect()
}

fn images() -> &'static [Vec<u8>] {
    static IMAGES: OnceLock<Vec<Vec<u8>>> = OnceLock::new();
    IMAGES.get_or_init(|| {
        views()
            .iter()
            .enumerate()
            .map(|(k, view)| render(view, k as u64 + 3))
            .collect()
    })
}

fn error(pose: &TargetPose, truth: &Isometry3<f64>) -> (f64, f64) {
    (
        (pose.pose.translation.vector - truth.translation.vector).norm(),
        pose.pose.rotation.angle_to(&truth.rotation).to_degrees(),
    )
}

#[test]
fn the_rectified_image_gives_the_pinhole_pose() {
    let map = Rectify::new(&info()).unwrap();
    let pinhole = rectified(&info());
    for (k, truth) in views().iter().enumerate() {
        let image = map.bilinear(&images()[k], 1);
        let corners = detect_chessboard(&image, WIDTH, HEIGHT, &BOARD, &DetectOptions::default())
            .unwrap_or_else(|| panic!("view {k}: no board in the rectified image"));
        let pose = target_pose(&corners, &BOARD.corners(), &pinhole).unwrap();
        let (offset, turn) = error(&pose, truth);
        eprintln!(
            "rectified view {k}: {offset:.6} m, {turn:.4} deg, rms {:.3} px",
            pose.rms
        );
        assert!(offset < 0.001 && turn < 0.2 && pose.rms < 0.2, "view {k}");
    }
}

#[test]
fn raw_corners_need_undistorting() {
    let pinhole = rectified(&info());
    for (k, truth) in views().iter().enumerate() {
        let Some(raw) = detect_chessboard(
            &images()[k],
            WIDTH,
            HEIGHT,
            &BOARD,
            &DetectOptions::default(),
        ) else {
            panic!("view {k}: no board in the raw image");
        };
        let ideal: Vec<[f64; 2]> = raw
            .iter()
            .map(|&p| undistort(&info(), p).unwrap())
            .collect();
        let pose = target_pose(&ideal, &BOARD.corners(), &pinhole).unwrap();
        let (offset, turn) = error(&pose, truth);
        // Taken as pinhole pixels, the raw corners give a pose that is off.
        let naive = target_pose(&raw, &BOARD.corners(), &pinhole).unwrap();
        let (naive_offset, naive_turn) = error(&naive, truth);
        eprintln!(
            "raw view {k}: undistorted {offset:.6} m, {turn:.4} deg, rms {:.3} px; \
             as pinhole {naive_offset:.6} m, {naive_turn:.4} deg, rms {:.3} px",
            pose.rms, naive.rms
        );
        assert!(offset < 0.001 && turn < 0.2 && pose.rms < 0.2, "view {k}");
        assert!(
            naive.rms > 5.0 * pose.rms && naive_offset > 5.0 * offset,
            "view {k}"
        );
    }
}

/// `target_pose` with the distorted camera takes the raw corners as they
/// were detected: the same pose as undistorting them first, with the
/// residual in raw pixels.
#[test]
fn the_pose_through_the_lens_needs_no_undistorting() {
    let pinhole = rectified(&info());
    for (k, truth) in views().iter().enumerate() {
        let raw = detect_chessboard(
            &images()[k],
            WIDTH,
            HEIGHT,
            &BOARD,
            &DetectOptions::default(),
        )
        .unwrap_or_else(|| panic!("view {k}: no board in the raw image"));
        let through = target_pose(&raw, &BOARD.corners(), &info()).unwrap();
        let (offset, turn) = error(&through, truth);
        let ideal: Vec<[f64; 2]> = raw
            .iter()
            .map(|&p| undistort(&info(), p).unwrap())
            .collect();
        let undistorted = target_pose(&ideal, &BOARD.corners(), &pinhole).unwrap();
        let (u_offset, u_turn) = error(&undistorted, truth);
        eprintln!(
            "through the lens view {k}: {offset:.6} m, {turn:.4} deg, rms {:.3} px; \
             undistorted first {u_offset:.6} m, {u_turn:.4} deg",
            through.rms
        );
        assert!(
            offset < 0.001 && turn < 0.2 && through.rms < 0.2,
            "view {k}"
        );
        let apart = (through.pose.translation.vector - undistorted.pose.translation.vector).norm();
        assert!(apart < 2e-4, "view {k}: the two poses are {apart} m apart");
    }
}

/// Exact raw pixels of a distorted camera give the exact pose; a pixel no
/// ray of the lens reaches is named.
#[test]
fn exact_raw_pixels_give_the_exact_pose() {
    let truth = views()[0];
    let corners = BOARD.corners();
    let raw: Vec<[f64; 2]> = corners
        .iter()
        .map(|c| {
            let p = truth * nalgebra::Point3::from(*c);
            project(&info(), [p.x, p.y, p.z]).unwrap()
        })
        .collect();
    let pose = target_pose(&raw, &corners, &info()).unwrap();
    let (offset, turn) = error(&pose, &truth);
    assert!(
        offset < 1e-9 && turn < 1e-7 && pose.rms < 1e-6,
        "{offset} m, {turn} deg"
    );
    // A fisheye sees nothing beyond 90 degrees off its axis.
    let fisheye = CameraInfo {
        distortion_model: distortion_model::EQUIDISTANT,
        d: [0.0; 8],
        ..info()
    };
    let mut far = raw.clone();
    far[7] = [319.5 + FOCAL * 2.0, 239.5];
    assert_eq!(
        target_pose(&far, &corners, &fisheye),
        Err(TargetPoseError::NoRay(7))
    );
}

/// The map onto a smaller camera matrix keeps the whole raw view, so a
/// board in the rim is found there too, and its pose comes out with that
/// camera.
#[test]
fn a_wider_rectified_camera_keeps_the_rim() {
    let wide = CameraInfo {
        k: [
            0.8 * FOCAL,
            0.0,
            319.5,
            0.0,
            0.8 * FOCAL,
            239.5,
            0.0,
            0.0,
            1.0,
        ],
        ..rectified(&info())
    };
    let map = Rectify::onto(&info(), &wide).unwrap();
    for (k, truth) in views().iter().enumerate() {
        let image = map.bilinear(&images()[k], 1);
        let corners = detect_chessboard(&image, WIDTH, HEIGHT, &BOARD, &DetectOptions::default())
            .unwrap_or_else(|| panic!("view {k}: no board in the wide image"));
        let pose = target_pose(&corners, &BOARD.corners(), map.info()).unwrap();
        let (offset, turn) = error(&pose, truth);
        eprintln!(
            "wide view {k}: {offset:.6} m, {turn:.4} deg, rms {:.3} px",
            pose.rms
        );
        assert!(offset < 0.0015 && turn < 0.3 && pose.rms < 0.25, "view {k}");
    }
}
