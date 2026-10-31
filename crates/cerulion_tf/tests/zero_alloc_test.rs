// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation proof for every SE(3) operation, error paths included.
//!
//! Frame lookups run inside node ticks, so the math they call must never
//! touch the heap. A counting `#[global_allocator]` observes every
//! allocation; the window counts only the thread that opened it, so libtest
//! machinery on other threads cannot fail or mask the measurement. The last
//! phase is the control: an allocation inside a window is counted, so a
//! zero above is a real zero.
//!
//! ONE `#[test]` body on purpose: the window is process-global, and a second
//! test in this binary could reset it mid-measurement.

use cerulion_tf::{TfMathError, Transform, UnitQuaternion};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;
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
        Err(TfMathError::NonFiniteQuaternion)
    );
    assert!(matches!(
        UnitQuaternion::new(0.0, 0.0, 0.0, black_box(0.0)),
        Err(TfMathError::DegenerateQuaternion { .. })
    ));
    assert_eq!(
        Transform::new(black_box([f64::INFINITY, 0.0, 0.0]), r),
        Err(TfMathError::NonFiniteTranslation)
    );
    let far = Transform::new(black_box([1e308, 0.0, 0.0]), UnitQuaternion::IDENTITY).unwrap();
    assert_eq!(far.compose(&far), Err(TfMathError::NonFiniteTranslation));
    assert_eq!(
        a.interpolate(&b, black_box(f64::NAN)),
        Err(TfMathError::FractionNotANumber)
    );
    assert!(matches!(
        a.interpolate(&b, black_box(1.5)),
        Err(TfMathError::FractionOutOfRange { .. })
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

    // Control: a real allocation inside a window is counted.
    ALLOC.open();
    let v: Vec<u64> = black_box(Vec::with_capacity(4));
    let during_control = ALLOC.close();
    drop(v);
    assert!(during_control >= 1, "the counter missed a real allocation");
}
