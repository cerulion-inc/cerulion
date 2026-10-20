// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation proof for every SE(3) operation and every tf buffer
//! operation, error paths included.
//!
//! Frame lookups run inside node ticks, so the math they call and the
//! buffer they read must never touch the heap. A counting
//! `#[global_allocator]` observes every allocation; the window counts only
//! the thread that opened it, so libtest machinery on other threads cannot
//! fail or mask the measurement. The last phase is the control: an
//! allocation inside a window is counted, so a zero above is a real zero.
//!
//! Each buffer is built outside any window: its constructor is the one
//! place it allocates. The first pass on a fresh buffer is measured, so frame
//! registration, ring growth and the eviction that admits a newer sample
//! are inside a window; the repeat pass on the same buffer, which meets a
//! full ring and a full frame table, is measured on its own. A static insert
//! that clears a dynamic edge's history has its own window too.
//!
//! ONE `#[test]` body on purpose: the window is process-global, and a second
//! test in this binary could reset it mid-measurement.

use cerulion_tf::{TfBuffer, TfError, Transform, UnitQuaternion};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::f64::consts::FRAC_1_SQRT_2;
use std::hint::black_box;
use std::num::NonZeroUsize;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

thread_local! {
    /// `true` only on the thread that opened the window. Const-init with a
    /// non-`Drop` payload, so touching it from inside `alloc` neither
    /// allocates nor registers a destructor.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

struct CountingAllocator {
    count: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    fn open(&self) {
        self.count.store(0, Ordering::SeqCst);
        MEASURED_THREAD.with(|m| m.set(true));
        self.enabled.store(true, Ordering::SeqCst);
    }

    fn close(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.with(|m| m.set(false));
        self.count.load(Ordering::SeqCst)
    }

    fn note(&self) {
        if self.enabled.load(Ordering::Relaxed) && MEASURED_THREAD.with(Cell::get) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.note();
        // SAFETY: forwards the caller's layout unchanged to the system allocator.
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` came from `System.alloc` with this `layout`.
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        self.note();
        // SAFETY: forwards the caller's layout unchanged to the system allocator.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        self.note();
        // SAFETY: `ptr` came from `System` with `layout`; the caller upholds
        // the `realloc` contract for `new_size`.
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOC: CountingAllocator = CountingAllocator {
    count: AtomicU64::new(0),
    enabled: AtomicBool::new(false),
};

/// Every public operation, on valid input and on each error arm.
fn every_operation() {
    let a = Transform::new(
        black_box([1.5, -2.25, 3.0]),
        UnitQuaternion::new(0.1, 0.2, 0.3, 0.9).unwrap(),
    )
    .unwrap();
    let b = Transform::new(
        black_box([-3.0, 0.25, 1.0]),
        UnitQuaternion::from_array(black_box([-0.4, 0.1, 0.0, 0.8])).unwrap(),
    )
    .unwrap();
    let c = black_box(a.compose(&b)).unwrap();
    let i = black_box(a.inverse()).unwrap();
    black_box(c.apply_point([0.7, -0.2, 1.9]));
    black_box(i.apply_vector([0.7, -0.2, 1.9]));
    for t in [0.0, 0.25, 0.5, 1.0] {
        black_box(a.interpolate(&b, black_box(t))).unwrap();
    }
    let r = a.rotation();
    black_box(r.compose(&b.rotation()));
    black_box(r.inverse().rotate([1.0, 2.0, 3.0]));
    black_box(r.slerp(&r, 0.5)).unwrap();
    black_box(r.slerp(&b.rotation().inverse(), 0.3)).unwrap();

    // Every error arm.
    assert_eq!(
        UnitQuaternion::new(black_box(f64::NAN), 0.0, 0.0, 1.0),
        Err(TfError::NonFiniteQuaternion)
    );
    assert!(matches!(
        UnitQuaternion::new(0.0, 0.0, 0.0, black_box(0.0)),
        Err(TfError::DegenerateQuaternion { .. })
    ));
    assert_eq!(
        Transform::new(black_box([f64::INFINITY, 0.0, 0.0]), r),
        Err(TfError::NonFiniteTranslation)
    );
    let far = Transform::new(black_box([1e308, 0.0, 0.0]), UnitQuaternion::IDENTITY).unwrap();
    assert_eq!(far.compose(&far), Err(TfError::NonFiniteTranslation));
    assert_eq!(
        a.interpolate(&b, black_box(f64::NAN)),
        Err(TfError::FractionNotANumber)
    );
    assert!(matches!(
        a.interpolate(&b, black_box(1.5)),
        Err(TfError::FractionOutOfRange { .. })
    ));
}

/// Thirty-seven bytes: one over the frame id limit.
const LONG_ID: &str = "a_frame_id_that_is_thirty_seven_bytes";

/// Every buffer operation, on a buffer of 8 frames with rings of 3: inserts
/// that add, replace, evict and drop, a static edge, lookups of every shape,
/// and every refusal. The first pass on a fresh buffer registers frames and
/// grows rings; run again on the same buffer, the repeat meets a full ring
/// and a full frame table, so both are measured.
fn every_buffer_operation(b: &mut TfBuffer) {
    let shift = |x: f64| Transform::new([x, 0.0, 0.0], UnitQuaternion::IDENTITY).unwrap();
    let turn = Transform::new(
        [2.0, 0.0, 0.0],
        UnitQuaternion::new(0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2).unwrap(),
    )
    .unwrap();
    // On a ring of 3, 400 evicts 100; when run again, 100 arrives older
    // than the full ring and is dropped, and the others replace in place.
    b.insert("odom", "base", black_box(100), shift(0.0))
        .unwrap();
    b.insert("odom", "base", 200, shift(1.0)).unwrap();
    b.insert("odom", "base", 300, turn).unwrap();
    b.insert("odom", "base", 400, shift(4.0)).unwrap();
    b.insert_static("base", "lidar", shift(0.17)).unwrap();
    // A second tree, and a chain whose translations overflow when composed.
    b.insert("x", "y", 0, shift(1.0)).unwrap();
    b.insert("p", "q", 0, shift(1e308)).unwrap();
    b.insert("q", "r", 0, shift(1e308)).unwrap();

    black_box(b.lookup("odom", "base", black_box(300))).unwrap();
    black_box(b.lookup("odom", "base", 250)).unwrap();
    black_box(b.lookup("odom", "lidar", 350)).unwrap();
    black_box(b.lookup("lidar", "odom", 350)).unwrap();
    black_box(b.lookup("base", "lidar", 7)).unwrap();
    black_box(b.lookup_latest("odom", "lidar")).unwrap();
    black_box(b.lookup("base", "base", 0)).unwrap();
    black_box(b.lookup("/odom", "/base", 300)).unwrap();
    black_box(b.edge("base"));
    black_box(b.contains_frame("odom"));
    black_box(b.evictions());

    // Every refusal.
    assert!(matches!(
        b.lookup("map", "base", 300),
        Err(TfError::UnknownFrame { .. })
    ));
    assert!(matches!(
        b.lookup("odom", "y", 0),
        Err(TfError::NoPath { .. })
    ));
    assert!(matches!(
        b.lookup("odom", "base", 401),
        Err(TfError::NotYetAvailable { .. })
    ));
    assert!(matches!(
        b.lookup("odom", "base", 199),
        Err(TfError::OutOfRange { .. })
    ));
    assert!(matches!(
        b.lookup("p", "r", 0),
        Err(TfError::ChainMath { .. })
    ));
    assert_eq!(
        b.insert("", "base", 0, shift(0.0)),
        Err(TfError::EmptyFrameId)
    );
    assert!(matches!(
        b.insert(black_box(LONG_ID), "base", 0, shift(0.0)),
        Err(TfError::FrameIdTooLong { .. })
    ));
    assert!(matches!(
        b.insert("base", "base", 0, shift(0.0)),
        Err(TfError::SelfEdge { .. })
    ));
    assert!(matches!(
        b.insert("map", "base", 0, shift(0.0)),
        Err(TfError::SecondParent { .. })
    ));
    assert!(matches!(
        b.insert("lidar", "odom", 0, shift(0.0)),
        Err(TfError::Cycle { .. })
    ));
    assert!(matches!(
        b.insert("base", "lidar", 0, shift(0.0)),
        Err(TfError::EdgeIsStatic { .. })
    ));
    assert!(matches!(
        b.insert("z", "w", 0, shift(0.0)),
        Err(TfError::FrameTableFull { .. })
    ));
    assert!(matches!(
        b.insert_static("z", "w", shift(0.0)),
        Err(TfError::FrameTableFull { .. })
    ));
}

#[test]
fn every_operation_allocates_nothing_on_the_calling_thread() {
    // Warm-up outside the window: first-touch effects (lazy statics in
    // the harness) do not belong to the math.
    every_operation();

    ALLOC.open();
    every_operation();
    let during_math = ALLOC.close();
    assert_eq!(
        during_math, 0,
        "the SE(3) operations allocated {during_math} time(s)"
    );

    // Warm-up outside the window on a throwaway buffer, for the same
    // first-touch reason as above.
    let nz = |n| NonZeroUsize::new(n).unwrap();
    every_buffer_operation(&mut TfBuffer::with_capacity(nz(8), nz(3)));

    // The buffer's constructor is its one allocation site, outside the
    // window. The first pass registers all 8 frames, grows the rings from
    // empty and evicts to admit a newer sample, inside the window.
    let mut buffer = TfBuffer::with_capacity(nz(8), nz(3));
    assert_eq!(buffer.frame_count(), 0);
    ALLOC.open();
    every_buffer_operation(&mut buffer);
    let during_first_pass = ALLOC.close();
    assert_eq!(
        during_first_pass, 0,
        "the first pass on a fresh tf buffer allocated {during_first_pass} time(s)"
    );
    assert_eq!(buffer.frame_count(), 8);
    assert_eq!(buffer.evictions(), 1, "400 evicts 100 on the ring of 3");

    // The repeat pass meets a full ring (100 is dropped on arrival, the
    // others replace in place) and a full frame table.
    ALLOC.open();
    every_buffer_operation(&mut buffer);
    let during_repeat_pass = ALLOC.close();
    assert_eq!(
        during_repeat_pass, 0,
        "the repeat pass on a full tf buffer allocated {during_repeat_pass} time(s)"
    );
    assert_eq!(buffer.frame_count(), 8);
    assert_eq!(
        buffer.evictions(),
        2,
        "the dropped 100 is the second eviction"
    );

    // A dynamic edge turned static: the static insert clears the ring.
    let mut converted = TfBuffer::with_capacity(nz(2), nz(3));
    let mount = Transform::new([0.17, 0.0, 0.11], UnitQuaternion::IDENTITY).unwrap();
    ALLOC.open();
    for stamp in [100u64, 200, 300] {
        converted
            .insert("base", "camera", black_box(stamp), mount)
            .unwrap();
    }
    converted.insert_static("base", "camera", mount).unwrap();
    black_box(converted.lookup("base", "camera", black_box(7))).unwrap();
    let during_conversion = ALLOC.close();
    assert_eq!(
        during_conversion, 0,
        "a dynamic edge turned static allocated {during_conversion} time(s)"
    );
    let edge = converted.edge("camera").expect("the converted edge");
    assert!(edge.is_static);
    assert_eq!(edge.samples, 0, "the static insert cleared the ring");
    assert_eq!(converted.evictions(), 3, "three cleared samples");

    // Control: a real allocation inside a window is counted.
    ALLOC.open();
    let v: Vec<u64> = black_box(Vec::with_capacity(4));
    let during_control = ALLOC.close();
    drop(v);
    assert!(during_control >= 1, "the counter missed a real allocation");
}
