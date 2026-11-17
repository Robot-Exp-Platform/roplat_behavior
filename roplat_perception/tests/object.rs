//! Object observations: boxes, masks and the bits they pack into.

use nalgebra::{Isometry3, Vector3};
use roplat_msgs::{FrameId, Stamp, clock};
use roplat_perception::{ObjectMask, ObjectObservation, PixelBox};

#[test]
fn box_overlap_is_intersection_over_union() {
    let a = PixelBox { min: [0.0, 0.0], max: [10.0, 10.0] };
    let b = PixelBox { min: [5.0, 0.0], max: [15.0, 10.0] };
    // 50 shared of 150 covered.
    assert!((a.iou(&b) - 1.0 / 3.0).abs() < 1e-12);
    assert_eq!(a.iou(&b), b.iou(&a));
    assert_eq!(a.iou(&a), 1.0);
    // Touching edges share no area; a box inside another shares its own.
    let beside = PixelBox { min: [10.0, 0.0], max: [20.0, 10.0] };
    assert_eq!(a.iou(&beside), 0.0);
    let inner = PixelBox { min: [2.0, 2.0], max: [4.0, 6.0] };
    assert!((a.iou(&inner) - 0.08).abs() < 1e-12);
    // Empty and inverted boxes have no area.
    let inverted = PixelBox { min: [5.0, 5.0], max: [1.0, 9.0] };
    assert_eq!(inverted.area(), 0.0);
    assert_eq!(inverted.iou(&inverted), 0.0);
    assert_eq!(a.centre(), [5.0, 5.0]);
}

/// The packing is `numpy.packbits` over the flattened mask, default
/// (`big`) bit order. The vectors are the examples of numpy's documentation
/// of `packbits` (numpy 2.x reference, BSD-3-Clause): `[1, 0, 1]` packs to
/// 160, `[0, 1, 0]` to 64, `[1, 1, 0]` to 192 and `[0, 0, 1]` to 32.
#[test]
fn masks_pack_as_numpy_packbits() {
    for (row, byte) in [
        ([true, false, true], 160u8),
        ([false, true, false], 64),
        ([true, true, false], 192),
        ([false, false, true], 32),
    ] {
        let mask = ObjectMask::from_fn(3, 1, |u, _| row[u as usize]);
        assert_eq!(mask.bits(), &[byte], "{row:?}");
        assert_eq!(ObjectMask::from_bits(3, 1, vec![byte]), Some(mask));
    }
    // Rows run on without padding: a 3 x 3 cross is
    // 010 111 010 -> 0101_1101 0(000_0000).
    let cross = ObjectMask::from_fn(3, 3, |u, v| u == 1 || v == 1);
    assert_eq!(cross.bits(), &[0b0101_1101, 0b0000_0000]);
    assert_eq!(cross.area(), 5);
    assert!(cross.contains(1, 0) && cross.contains(0, 1) && !cross.contains(0, 0));
    assert!(!cross.contains(3, 1) && !cross.contains(1, 3));
    assert_eq!(
        cross.bounding_box(),
        Some(PixelBox { min: [0.0, 0.0], max: [3.0, 3.0] })
    );
}

#[test]
fn malformed_masks_are_refused() {
    // 3 x 3 needs two bytes; the 7 padding bits after the last pixel stay
    // clear.
    assert!(ObjectMask::from_bits(3, 3, vec![0]).is_none());
    assert!(ObjectMask::from_bits(3, 3, vec![0, 0, 0]).is_none());
    assert!(ObjectMask::from_bits(3, 3, vec![0, 0x40]).is_none());
    assert!(ObjectMask::from_bits(3, 3, vec![0, 0x80]).is_some());
    assert!(ObjectMask::from_bits(4, 2, vec![0xff]).is_some());
    let empty = ObjectMask::from_fn(4, 2, |_, _| false);
    assert_eq!((empty.area(), empty.bounding_box()), (0, None));
}

#[test]
fn an_observation_starts_with_nothing_but_its_class() {
    let stamp = Stamp::new(clock::SIMULATION, 1_500_000_000);
    let frame = FrameId::new(7);
    let bare = ObjectObservation::new("cup", stamp, frame);
    assert_eq!(
        (bare.class.as_str(), bare.stamp, bare.frame),
        ("cup", stamp, frame)
    );
    assert!(
        bare.class_id.is_none()
            && bare.score.is_none()
            && bare.pose.is_none()
            && bare.size.is_none()
            && bare.pixel_box.is_none()
            && bare.mask.is_none()
            && bare.color.is_none()
            && bare.track.is_none()
    );
    // A fused object: a detector's box and score, a cluster's pose and size.
    let mask = ObjectMask::from_fn(8, 6, |u, v| (2..5).contains(&u) && (1..4).contains(&v));
    let fused = ObjectObservation {
        class_id: Some(41),
        score: Some(0.8),
        pixel_box: mask.bounding_box(),
        mask: Some(mask),
        pose: Some(Isometry3::translation(0.4, -0.1, 0.75)),
        size: Some(Vector3::new(0.08, 0.08, 0.1)),
        color: Some([200, 30, 30]),
        track: Some(3),
        ..bare.clone()
    };
    assert_eq!(
        fused.pixel_box,
        Some(PixelBox { min: [2.0, 1.0], max: [5.0, 4.0] })
    );
    assert_eq!(fused.mask.as_ref().map(ObjectMask::area), Some(9));
    assert_ne!(fused, bare);
}
