//! After a buffer is created, writes and lookups do not allocate, including
//! the ones that fail.

mod support;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;

use roplat_frames::{FrameBuffer, History};
use roplat_msgs::{Pose, Transform, clock};
use support::*;

/// Counts allocations of the current thread while counting is on.
struct Counting;

thread_local! {
    static COUNT: Cell<Option<usize>> = const { Cell::new(None) };
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = COUNT.try_with(|count| count.set(count.get().map(|n| n + 1)));
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn allocations(run: impl FnOnce()) -> usize {
    COUNT.with(|count| count.set(Some(0)));
    run();
    COUNT.with(|count| count.take()).unwrap()
}

#[test]
fn writes_and_lookups_do_not_allocate() {
    assert_eq!(
        allocations(|| drop(black_box(vec![0_u8; 8]))),
        1,
        "the counter counts"
    );
    let tree = cell(History::new(16), History::new(4));
    let mut frames = FrameBuffer::new(tree.clone(), clock::MONOTONIC).unwrap();
    let arm = frames.tree().group::<2>("arm").unwrap();
    let foreign = cell(History::new(1), History::new(1))
        .group::<2>("arm")
        .unwrap();
    let seen = Transform {
        parent: CAMERA,
        child: MUG,
        pose: Pose::from_position([0., 0., 0.5]),
    };

    let count = allocations(|| {
        for i in 0..200 {
            let t = ms(i as f64);
            frames
                .set_joints(arm, t, &[0.01 * i as f64, -0.02 * i as f64])
                .unwrap();
            frames.set_transform(t, &seen).unwrap();
            black_box(frames.lookup(WORLD, MUG, ms(i as f64 - 0.5)).ok());
            black_box(frames.lookup(MUG, FINGER, t).unwrap_err());
            black_box(frames.lookup(BASE, HAND, ms(i as f64 + 5.)).unwrap_err());
            black_box(frames.set_joints(arm, ms(0.), &[0., 0.]).ok());
            black_box(frames.set_joints(foreign, t, &[0., 0.]).unwrap_err());
            black_box(tree.lookup_static(HAND, CAMERA).unwrap());
        }
    });
    assert_eq!(count, 0);
}
