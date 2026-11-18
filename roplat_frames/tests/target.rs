//! A chessboard rendered from many views: its corners are found to a
//! fraction of a pixel in the board's order, its pose follows, and the
//! poses calibrate a camera on the flange.

use std::sync::OnceLock;

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use roplat_frames::{
    calibration::{HandEye, HandEyeSample, calibrate_hand_eye},
    target::{Chessboard, DetectOptions, TargetPoseError, detect_chessboard, target_pose},
};
use roplat_msgs::CameraInfo;

const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const FOCAL: f64 = 600.0;
/// 9 x 6 inner corners, 10 x 7 squares: one dark and one light corner
/// square, so the order is the same from every side.
const BOARD: Chessboard = Chessboard { columns: 9, rows: 6, square: 0.025 };

fn info() -> CameraInfo {
    CameraInfo {
        width: WIDTH as u32,
        height: HEIGHT as u32,
        k: [FOCAL, 0.0, 319.5, 0.0, FOCAL, 239.5, 0.0, 0.0, 1.0],
        ..Default::default()
    }
}

/// The board seen by a camera at `board_in_camera`: dark and light squares
/// with a light margin one square wide, on a mid-grey background; each
/// pixel averages 3 x 3 samples, plus a little deterministic noise.
fn render(board_in_camera: &Isometry3<f64>, seed: u64) -> Vec<u8> {
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
                    let px = u as f64 + (sx as f64 - 1.0) / 3.0;
                    let py = v as f64 + (sy as f64 - 1.0) / 3.0;
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
    let camera_in_board = Isometry3::from_parts(Translation3::from(eye), rotation);
    camera_in_board.inverse()
}

fn project(board_in_camera: &Isometry3<f64>, p: [f64; 3]) -> [f64; 2] {
    let c = board_in_camera * Point3::from(p);
    [FOCAL * c.x / c.z + 319.5, FOCAL * c.y / c.z + 239.5]
}

/// The images of [`views`], rendered once.
fn images() -> &'static [Vec<u8>] {
    static IMAGES: OnceLock<Vec<Vec<u8>>> = OnceLock::new();
    IMAGES.get_or_init(|| {
        views()
            .iter()
            .enumerate()
            .map(|(k, view)| render(view, k as u64 + 7))
            .collect()
    })
}

/// Views from above the board's front, tilted up to about 35 degrees, at
/// every quarter turn of roll and in between.
fn views() -> Vec<Isometry3<f64>> {
    let mut views = Vec::new();
    for (k, (x, y, z)) in [
        (0.15, 0.1, -0.45),
        (0.3, 0.0, -0.5),
        (-0.15, 0.2, -0.5),
        (0.05, -0.2, -0.45),
        (0.25, 0.25, -0.55),
        (0.0, 0.1, -0.6),
        (-0.1, -0.05, -0.4),
        (0.35, -0.1, -0.5),
    ]
    .into_iter()
    .enumerate()
    {
        views.push(view(Vector3::new(x, y, z), 0.3 + 0.8 * k as f64));
    }
    views
}

#[test]
fn corners_are_found_in_order_to_a_fraction_of_a_pixel() {
    let corners = BOARD.corners();
    for (k, board_in_camera) in views().iter().enumerate() {
        let found = detect_chessboard(
            &images()[k],
            WIDTH,
            HEIGHT,
            &BOARD,
            &DetectOptions::default(),
        )
        .unwrap_or_else(|| panic!("view {k}: no board"));
        let errors: Vec<f64> = found
            .iter()
            .zip(&corners)
            .map(|(f, c)| {
                let expected = project(board_in_camera, *c);
                (f[0] - expected[0]).hypot(f[1] - expected[1])
            })
            .collect();
        let mean = errors.iter().sum::<f64>() / errors.len() as f64;
        let worst = errors.iter().copied().fold(0.0, f64::max);
        assert!(
            mean < 0.06 && worst < 0.2,
            "view {k}: corners {mean} px off on average, {worst} at worst"
        );
    }
}

#[test]
fn the_pose_follows_from_the_corners() {
    let corners = BOARD.corners();
    for (k, board_in_camera) in views().iter().enumerate() {
        let found = detect_chessboard(
            &images()[k],
            WIDTH,
            HEIGHT,
            &BOARD,
            &DetectOptions::default(),
        )
        .unwrap();
        let pose = target_pose(&found, &corners, &info()).unwrap();
        let offset = (pose.pose.translation.vector - board_in_camera.translation.vector).norm();
        let turn = pose.pose.rotation.angle_to(&board_in_camera.rotation);
        assert!(offset < 0.0005, "view {k}: {offset} m off");
        assert!(turn < 0.1f64.to_radians(), "view {k}: {turn} rad off");
        assert!(pose.rms < 0.1, "view {k}: rms {} px", pose.rms);
    }
}

#[test]
fn exact_points_give_the_exact_pose() {
    let board_in_camera = views()[3];
    let corners = BOARD.corners();
    let pixels: Vec<[f64; 2]> = corners
        .iter()
        .map(|c| project(&board_in_camera, *c))
        .collect();
    let pose = target_pose(&pixels, &corners, &info()).unwrap();
    assert!((pose.pose.translation.vector - board_in_camera.translation.vector).norm() < 1e-9);
    assert!(pose.pose.rotation.angle_to(&board_in_camera.rotation) < 1e-9);
    assert!(pose.rms < 1e-6);
    // Too few points, points off the plane, lists that do not pair up, a
    // camera the lens module refuses, points all in one place.
    assert_eq!(
        target_pose(&pixels[..3], &corners[..3], &info()),
        Err(TargetPoseError::TooFewPoints(3))
    );
    let mut lifted = corners.clone();
    lifted[5][2] = 0.01;
    assert_eq!(
        target_pose(&pixels, &lifted, &info()),
        Err(TargetPoseError::OffPlane(5))
    );
    assert_eq!(
        target_pose(&pixels[..10], &corners, &info()),
        Err(TargetPoseError::Mismatch { pixels: 10, points: 54 })
    );
    let flat = CameraInfo { k: [0.0; 9], ..info() };
    assert_eq!(
        target_pose(&pixels, &corners, &flat),
        Err(TargetPoseError::Camera)
    );
    // The board's first row: nine points on a line fit many poses.
    assert_eq!(
        target_pose(&pixels[..9], &corners[..9], &info()),
        Err(TargetPoseError::Degenerate)
    );
    let one_point = vec![[0.0, 0.0, 0.0]; 9];
    assert_eq!(
        target_pose(&pixels[..9], &one_point, &info()),
        Err(TargetPoseError::Degenerate)
    );
    // Two rows are enough.
    assert!(target_pose(&pixels[..18], &corners[..18], &info()).is_ok());
}

#[test]
fn a_board_cut_off_by_the_frame_or_absent_is_not_reported() {
    let blank = vec![128u8; WIDTH * HEIGHT];
    assert!(detect_chessboard(&blank, WIDTH, HEIGHT, &BOARD, &DetectOptions::default()).is_none());
    // So close that its outer corners fall outside the image.
    let near = view(Vector3::new(0.1, 0.06, -0.14), 0.0);
    let image = render(&near, 3);
    assert!(detect_chessboard(&image, WIDTH, HEIGHT, &BOARD, &DetectOptions::default()).is_none());
    // An image of the board read as a board of another size.
    let other = Chessboard { columns: 7, rows: 6, square: 0.025 };
    assert!(
        detect_chessboard(
            &images()[0],
            WIDTH,
            HEIGHT,
            &other,
            &DetectOptions::default()
        )
        .is_none()
    );
}

#[test]
fn images_calibrate_a_camera_on_the_flange() {
    // The camera's optical frame in the flange frame, and the board in the
    // robot's base frame.
    let mount = Isometry3::from_parts(
        Translation3::new(0.05, -0.02, 0.08),
        UnitQuaternion::from_euler_angles(0.1, -0.05, 1.5),
    );
    let board = Isometry3::from_parts(
        Translation3::new(0.6, 0.1, 0.02),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3),
    );
    let corners = BOARD.corners();
    let samples: Vec<HandEyeSample> = views()
        .iter()
        .enumerate()
        .map(|(k, board_in_camera)| {
            let camera = board * board_in_camera.inverse();
            let flange = camera * mount.inverse();
            let found = detect_chessboard(
                &images()[k],
                WIDTH,
                HEIGHT,
                &BOARD,
                &DetectOptions::default(),
            )
            .unwrap();
            let target = target_pose(&found, &corners, &info()).unwrap().pose;
            HandEyeSample { flange, target }
        })
        .collect();
    let calibration = calibrate_hand_eye(&samples, HandEye::EyeInHand).unwrap();
    let offset = (calibration.camera.translation.vector - mount.translation.vector).norm();
    let turn = calibration.camera.rotation.angle_to(&mount.rotation);
    assert!(offset < 0.0005, "mount {offset} m off: {calibration:?}");
    assert!(
        turn < 0.1f64.to_radians(),
        "mount {turn} rad off: {calibration:?}"
    );
    assert!(calibration.translation_rms < 0.0005, "{calibration:?}");
}

/// A printed board, rendered by `Chessboard::render`, is found in order:
/// inner corner (i, j) at 30 + 20 (i + 1) - 0.5 in both directions.
#[test]
fn a_rendered_board_is_found_in_its_order() {
    let image = BOARD.render(20, 30).unwrap();
    assert_eq!(
        (image.width(), image.height(), image.channels()),
        (260, 200, 1)
    );
    // The top-left square is black, the margin white.
    assert_eq!(image.pixel(35, 35), Some(&[0u8][..]));
    assert_eq!(image.pixel(55, 35), Some(&[255u8][..]));
    assert_eq!(image.pixel(5, 5), Some(&[255u8][..]));
    let corners = detect_chessboard(
        image.data(),
        image.width(),
        image.height(),
        &BOARD,
        &DetectOptions::default(),
    )
    .unwrap();
    assert_eq!(corners.len(), 54);
    for (id, c) in corners.iter().enumerate() {
        let (i, j) = ((id % 9) as f64, (id / 9) as f64);
        let expected = [30.0 + 20.0 * (i + 1.0) - 0.5, 30.0 + 20.0 * (j + 1.0) - 0.5];
        let off = (c[0] - expected[0]).hypot(c[1] - expected[1]);
        assert!(off < 0.1, "corner {id} at {c:?}, not {expected:?}");
    }
    assert!(
        Chessboard { columns: 1, rows: 6, square: 0.025 }
            .render(20, 30)
            .is_none()
    );
    assert!(BOARD.render(0, 30).is_none());
}
