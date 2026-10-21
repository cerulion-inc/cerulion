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
//! registration and name interning, ring growth, expiry and overflow are
//! inside a window; the repeat pass on the same buffer, which begins a new
//! epoch and meets a full frame table and counted repeat refusals, is
//! measured on its own. A static insert that clears a dynamic edge's
//! history has its own window too.
//!
//! ONE `#[test]` body on purpose: the window is process-global, and a second
//! test in this binary could reset it mid-measurement.

use cerulion_tf::{
    EdgePolicy, TfBuffer, TfBufferConfig, TfError, Transform, UnitQuaternion, WriterId,
};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::f64::consts::FRAC_1_SQRT_2;
use std::fmt::Write as _;
use std::hint::black_box;
use std::num::NonZeroUsize;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Duration;

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

const W: WriterId = WriterId::new(1);
const W2: WriterId = WriterId::new(2);

/// A frame id longer than any inline label: interned whole, with no limit.
const LONG_ID: &str = "robot_0/camera_aligned_depth_to_color_frame_with_a_long_suffix";

/// A buffer of 10 frames, a window of 1000 ns and rings of 3.
fn small_buffer() -> TfBuffer {
    TfBuffer::with_config(TfBufferConfig {
        frames: NonZeroUsize::new(10).unwrap(),
        retention: Duration::from_nanos(1_000),
        ring_capacity: NonZeroUsize::new(3).unwrap(),
        name_bytes: 128,
    })
}

/// Every buffer operation, on a buffer of 10 frames with rings of 3 and a
/// window of 1000 ns: inserts that add, replace, expire and overflow, a
/// static edge, edge policies, an epoch, lookups of every shape, and every
/// refusal. The first pass on a fresh buffer registers frames and grows
/// rings; run again on the same buffer, the repeat meets full rings, a full
/// frame table and counted repeat refusals, so all are measured.
fn every_buffer_operation(b: &mut TfBuffer) {
    let shift = |x: f64| Transform::new([x, 0.0, 0.0], UnitQuaternion::IDENTITY).unwrap();
    let turn = Transform::new(
        [2.0, 0.0, 0.0],
        UnitQuaternion::new(0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2).unwrap(),
    )
    .unwrap();
    b.begin_epoch();
    b.set_edge_policy("odom", "base", EdgePolicy::Extrapolate { max_ns: 500 })
        .unwrap();
    // On a ring of 3, 400 overflows 100; 1300 expires 200; 300 is then
    // replaced by a different value.
    b.insert(W, "odom", "base", black_box(100), shift(0.0))
        .unwrap();
    b.insert(W, "odom", "base", 200, shift(1.0)).unwrap();
    b.insert(W, "odom", "base", 300, turn).unwrap();
    b.insert(W, "odom", "base", 400, shift(4.0)).unwrap();
    b.insert(W, "odom", "base", 1_300, shift(5.0)).unwrap();
    b.insert(W, "odom", "base", 300, shift(3.0)).unwrap();
    b.insert(W, "odom", "base", 300, shift(3.0)).unwrap();
    b.insert_static(W, "base", "lidar", shift(0.17)).unwrap();
    b.insert_static(W, "base", LONG_ID, shift(0.05)).unwrap();
    b.set_edge_policy("map", "x", EdgePolicy::HoldLast { max_age_ns: 10 })
        .unwrap();
    // A second tree, and a chain whose translations overflow when composed.
    b.insert(W, "x", "y", 0, shift(1.0)).unwrap();
    b.insert(W, "p", "q", 0, shift(1e308)).unwrap();
    b.insert(W, "q", "r", 0, shift(1e308)).unwrap();

    black_box(b.lookup("odom", "base", black_box(400))).unwrap();
    black_box(b.lookup("odom", "base", 850)).unwrap();
    black_box(b.lookup("odom", "base", 1_700)).unwrap();
    black_box(b.lookup("odom", "lidar", 350)).unwrap();
    black_box(b.lookup("lidar", "odom", 350)).unwrap();
    black_box(b.lookup("base", "lidar", 7)).unwrap();
    black_box(b.lookup(LONG_ID, "lidar", 7)).unwrap();
    black_box(b.lookup_latest("odom", "lidar")).unwrap();
    black_box(b.lookup("base", "base", 0)).unwrap();
    black_box(b.lookup("/odom", "/base", 400)).unwrap();
    let (odom, lidar) = (b.frame_id("odom").unwrap(), b.frame_id("lidar").unwrap());
    black_box(b.lookup_ids(odom, lidar, 400)).unwrap();
    black_box(b.lookup_latest_ids(lidar, odom)).unwrap();
    black_box(b.frame_name(odom));
    black_box(b.edge("base"));
    black_box(b.contains_frame("odom"));
    black_box((b.expired(), b.replacements(), b.epoch()));

    // Every refusal, and its text through the buffer: the text is written
    // into a fixed sink, so formatting it allocates nothing either.
    let refusals = [
        b.lookup("nope", "base", 300),
        b.lookup("odom", "y", 0),
        b.lookup("odom", "base", 1_801),
        b.lookup("odom", "base", 299),
        b.lookup("map", "x", 0),
        b.lookup("p", "r", 0),
    ];
    let mut sink = Sink::default();
    for refusal in &refusals {
        let err = refusal.as_ref().unwrap_err();
        write!(sink, "{}", b.describe(err)).unwrap();
    }
    assert!(matches!(refusals[0], Err(TfError::UnknownFrame { .. })));
    assert!(matches!(refusals[1], Err(TfError::NoPath { .. })));
    assert!(matches!(refusals[2], Err(TfError::NotYetAvailable { .. })));
    assert!(matches!(refusals[3], Err(TfError::OutOfRange { .. })));
    assert!(matches!(refusals[4], Err(TfError::EdgeEmpty { .. })));
    assert!(matches!(refusals[5], Err(TfError::ChainMath { .. })));
    assert_eq!(
        b.insert(W, "", "base", 0, shift(0.0)),
        Err(TfError::EmptyFrameId)
    );
    assert!(matches!(
        b.insert(W, "base", "base", 0, shift(0.0)),
        Err(TfError::SelfEdge { .. })
    ));
    assert!(matches!(
        b.insert(W, "map", "base", 0, shift(0.0)),
        Err(TfError::SecondParent { .. })
    ));
    assert!(matches!(
        b.insert(W, "lidar", "odom", 0, shift(0.0)),
        Err(TfError::Cycle { .. })
    ));
    assert!(matches!(
        b.insert(W, "odom", "base", 0, shift(0.0)),
        Err(TfError::StampBeforeWindow { .. })
    ));
    // A first refusal, or a counted repeat when the operations run again.
    assert!(matches!(
        b.insert(W, "base", "lidar", 0, shift(0.0)),
        Err(TfError::EdgeIsStatic { .. }) | Ok(())
    ));
    assert!(matches!(
        b.insert(W2, "odom", "base", 1_300, shift(0.0)),
        Err(TfError::SecondWriter { .. }) | Ok(())
    ));
    assert!(matches!(
        b.insert(W, "z", "w", 0, shift(0.0)),
        Err(TfError::FrameTableFull { .. })
    ));
    assert!(matches!(
        b.insert_static(W, "z", "w", shift(0.0)),
        Err(TfError::FrameTableFull { .. })
    ));
}

/// A fixed-size text sink: formatting into it never allocates.
struct Sink {
    bytes: [u8; 4_096],
    len: usize,
}

impl Default for Sink {
    fn default() -> Self {
        Self {
            bytes: [0; 4_096],
            len: 0,
        }
    }
}

impl std::fmt::Write for Sink {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        let end = self.len + s.len();
        let room = self.bytes.get_mut(self.len..end).ok_or(std::fmt::Error)?;
        room.copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
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
    every_buffer_operation(&mut small_buffer());

    // The buffer's constructor is its one allocation site, outside the
    // window. The first pass registers all 10 frames and interns their
    // names, grows the rings from empty, overflows, expires and replaces,
    // inside the window.
    let mut buffer = small_buffer();
    assert_eq!(buffer.frame_count(), 0);
    ALLOC.open();
    every_buffer_operation(&mut buffer);
    let during_first_pass = ALLOC.close();
    assert_eq!(
        during_first_pass, 0,
        "the first pass on a fresh tf buffer allocated {during_first_pass} time(s)"
    );
    assert_eq!(buffer.frame_count(), 10);
    assert_eq!(
        buffer.edge("base").unwrap().overflows,
        1,
        "400 overflows 100"
    );
    assert_eq!(buffer.expired(), 1, "1300 expires 200");
    assert_eq!(buffer.replacements(), 1, "300 replaced once");

    // The repeat pass clears the rings with an epoch, refills them and
    // meets a full frame table and counted repeat refusals.
    ALLOC.open();
    every_buffer_operation(&mut buffer);
    let during_repeat_pass = ALLOC.close();
    assert_eq!(
        during_repeat_pass, 0,
        "the repeat pass on a full tf buffer allocated {during_repeat_pass} time(s)"
    );
    assert_eq!(buffer.frame_count(), 10);
    assert_eq!(buffer.epoch(), 2);
    let base = buffer.edge("base").unwrap();
    assert_eq!((base.overflows, base.second_writer_refusals), (2, 2));
    assert_eq!(buffer.edge("lidar").unwrap().static_refusals, 2);

    // A dynamic edge turned static: the static insert clears the ring.
    let mut converted = TfBuffer::with_config(TfBufferConfig {
        frames: NonZeroUsize::new(2).unwrap(),
        ..TfBufferConfig::default()
    });
    let mount = Transform::new([0.17, 0.0, 0.11], UnitQuaternion::IDENTITY).unwrap();
    ALLOC.open();
    for stamp in [100u64, 200, 300] {
        converted
            .insert(W, "base", "camera", black_box(stamp), mount)
            .unwrap();
    }
    converted.insert_static(W, "base", "camera", mount).unwrap();
    black_box(converted.lookup("base", "camera", black_box(7))).unwrap();
    let during_conversion = ALLOC.close();
    assert_eq!(
        during_conversion, 0,
        "a dynamic edge turned static allocated {during_conversion} time(s)"
    );
    let edge = converted.edge("camera").expect("the converted edge");
    assert!(edge.is_static);
    assert_eq!(edge.samples, 0, "the static insert cleared the ring");
    assert_eq!(converted.expired(), 3, "three cleared samples");

    // Control: a real allocation inside a window is counted.
    ALLOC.open();
    let v: Vec<u64> = black_box(Vec::with_capacity(4));
    let during_control = ALLOC.close();
    drop(v);
    assert!(during_control >= 1, "the counter missed a real allocation");
}
