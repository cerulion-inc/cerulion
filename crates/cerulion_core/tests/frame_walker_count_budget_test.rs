// SPDX-License-Identifier: AGPL-3.0-only
//! The `FrameWalker`'s hostile-COUNT budget (`count > remaining / 4`)
//! is pinned by an oracle that can tell GUARD-REFUSAL from CURSOR-EXHAUSTION.
//!
//! # Why the pre-existing test could not do it
//!
//! `cerulion_viz`'s `a_declared_billion_elements_is_refused_by_the_walker_not_the_render_cap`
//! asserts the OUTCOME (`NestedArrayOpaque`) of a frame declaring 1e9 elements
//! over a short body. That outcome is reached with the guard DELETED too: every
//! element costs at least its own 4-byte length prefix, so a count above
//! `remaining / 4` can never be satisfied and the element loop falls out at the
//! first unreadable prefix — returning the same opaque value. The guard is
//! therefore **provably output-equivalent** and no output-shaped assertion
//! anywhere in the repo can kill its removal.
//!
//! # What the guard actually buys, and how this file measures it
//!
//! It bounds WORK. Without it, a blob of `L` bytes whose count word claims 1e9
//! still drives `L / 4` loop iterations, each pushing a `FrameValueKind` into an
//! element `Vec` that grows by doubling. Two figures, both measured on this
//! file's own control blob (`L` = 400_004 B, 100_000 elements,
//! `size_of::<FrameValueKind>()` = 40):
//!
//! - the list it ends up holding is 4_000_000 B — **~10x** the blob;
//! - the bytes it REQUESTS getting there are 10_475_981 B — **~26x** the blob,
//!   because doubling from 256 re-requests every intermediate capacity.
//!
//! The second number is the one this file asserts on, and it is the real cost
//! of a guard-less loop. `ELEMENT_PREALLOC_CAP` does not substitute for the
//! guard: it caps only the UP-FRONT reservation at 256 elements and does
//! nothing about that growth.
//!
//! The notes crediting the guard live in `cerulion_viz` —
//! `archetype.rs`'s `MAX_ELEMENT_INSTANCES` doc and the twin in
//! `tests/element_cap_bench.rs`, both written in the same change — which say the
//! walker "refuses a frame DECLARING a billion elements without ever sizing a
//! `Vec`". That claim is TRUE, and was simply untested until this file; the
//! allocation assertion below is what now holds it.
//!
//! So the oracle is ALLOCATION, measured against a CONTROL: the same blob
//! LENGTH carrying a TRUE count decodes into a real `K`-element list and
//! pays for it. The guarded hostile walk must pay a rounding error of that.
//! Both verdicts are asserted too, so the pre-existing outcome contract is not
//! weakened — it is merely no longer the only claim.
//!
//! # Mutation verification
//!
//! Deleting `if count > cur.remaining() / 4 { return opaque; }` from
//! `FrameWalker::decode_counted_elements` fails
//! `the_count_budget_refuses_before_building_an_element_list` with the hostile
//! walk allocating ~the control's bytes instead of ~none — an attributable
//! number, not a shrug. The verdict assertions stay green under that deletion,
//! which is precisely the point.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test frame_walker_count_budget_test
//! ```

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker};
use cerulion_core::wire::WireHeader;
use serial_test::serial;

thread_local! {
    /// `true` only on the thread that opened the measurement window.
    ///
    /// MUST be `const`-init with a non-`Drop` payload: a lazy (allocating) TLS
    /// init or a registered destructor touched from inside `GlobalAlloc::alloc`
    /// would recurse into the allocator.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// Counts heap BYTES requested by the measured thread while a window is open.
///
/// Bytes, not allocation COUNT: the thing under test is a `Vec` that grows by
/// doubling, so the accurate cost signal is total requested size. `realloc` is
/// overridden for the same reason — `Vec` growth reaches the allocator through
/// it, and a count-only or alloc-only probe would miss almost all of it.
///
/// Thread-scoped for the reason documented in `shm_ring_zero_alloc_test`:
/// libtest machinery and platform lazy-init allocate on other threads at times
/// this test does not control.
struct CountingAllocator {
    inner: System,
    bytes: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            bytes: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.bytes.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }

    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.bytes.load(Ordering::SeqCst)
    }

    #[inline]
    fn note(&self, n: usize) {
        if self.enabled.load(Ordering::Relaxed) {
            // `try_with`, never `with`: during thread teardown TLS is
            // inaccessible and `with` would panic INSIDE the allocator.
            if MEASURED_THREAD.try_with(Cell::get).unwrap_or(false) {
                self.bytes.fetch_add(n as u64, Ordering::Relaxed);
            }
        }
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.note(layout.size());
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        self.note(new_size);
        unsafe { self.inner.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

/// Elements in the CONTROL array. Large enough that building the list dwarfs
/// every fixed cost of a walk, small enough to stay instant in debug.
const ELEMENTS: usize = 100_000;

/// `sensor_msgs/JointState` carries five variable fields and no fixed section,
/// so its offset table is `5 * 8` bytes and `name` (a `string[]`) is entry 1.
const JOINT_STATE_TABLE_BYTES: usize = 40;
const NAME_ENTRY: usize = 1;

/// A `FrameWalker` over the vendored `.msg` corpus.
fn builtin_walker() -> FrameWalker {
    let mut schemas = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    let (walker, _warnings) = FrameWalker::new(schemas);
    walker
}

/// `[u32 count][per element: u32 len = 0]` — `n` zero-length string elements.
///
/// Zero-length elements are the CHEAPEST valid encoding, which makes the
/// budget exactly `remaining / 4`: a true `count == n` sits precisely AT the
/// boundary and passes, so the control is not merely "well under" the guard.
fn zero_length_string_blob(declared_count: u32, elements: usize) -> Vec<u8> {
    let mut v = Vec::with_capacity(4 + 4 * elements);
    v.extend_from_slice(&declared_count.to_le_bytes());
    for _ in 0..elements {
        v.extend_from_slice(&0u32.to_le_bytes());
    }
    v
}

/// A `sensor_msgs/JointState` frame whose `name` entry points at `blob`. Every
/// other entry stays `(0, 0)` — the empty-field idiom the walker accepts.
fn joint_state_frame(blob: &[u8]) -> Vec<u8> {
    let mut payload = vec![0u8; JOINT_STATE_TABLE_BYTES];
    let entry = NAME_ENTRY * 8;
    payload[entry..entry + 4].copy_from_slice(&(JOINT_STATE_TABLE_BYTES as u32).to_le_bytes());
    payload[entry + 4..entry + 8].copy_from_slice(&(blob.len() as u32).to_le_bytes());
    payload.extend_from_slice(blob);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader::with_schema(0).write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// Walk `frame` as a `JointState` with the allocation window open; returns the
/// `name` field's decoded kind plus the bytes the walk requested.
///
/// The `FrameValue` is dropped INSIDE the caller's scope after the window
/// closes — deallocation is not counted, so drop order cannot skew the number.
fn measure_walk(walker: &FrameWalker, frame: &[u8]) -> (String, usize, u64) {
    ALLOCATOR.enable();
    let walked = walker.walk("sensor_msgs/JointState", frame);
    let bytes = ALLOCATOR.disable();

    let fv = walked.expect("a JointState frame must walk");
    let (kind, len) = match fv.field("name") {
        Some(FrameValueKind::NestedArray { elements, .. }) => {
            ("NestedArray".to_string(), elements.len())
        }
        Some(FrameValueKind::NestedArrayOpaque(_)) => ("NestedArrayOpaque".to_string(), 0),
        other => (format!("{other:?}"), 0),
    };
    (kind, len, bytes)
}

/// THE pin: a count over the byte budget is refused BEFORE the element list is
/// built, not merely refused eventually.
///
/// Both halves live in one `#[test]` body on purpose — the control's measured
/// number is the hostile half's oracle, and two `#[serial]` bodies would still
/// let the waiting test's thread allocate inside the running window (the
/// one-body fold pattern).
#[test]
#[serial]
fn the_count_budget_refuses_before_building_an_element_list() {
    let walker = builtin_walker();
    let elem = std::mem::size_of::<FrameValueKind>();

    // Both blobs are the SAME LENGTH and differ in exactly four bytes: the
    // declared count. Any allocation difference is attributable to the count.
    let control = zero_length_string_blob(ELEMENTS as u32, ELEMENTS);
    let hostile = zero_length_string_blob(1_000_000_000, ELEMENTS);
    assert_eq!(
        control.len(),
        hostile.len(),
        "the two blobs must differ ONLY in the declared count"
    );
    let control_frame = joint_state_frame(&control);
    let hostile_frame = joint_state_frame(&hostile);

    // CONTROL — a true count sitting exactly AT the budget boundary
    // (`count == remaining / 4`) decodes into a real element list and pays for
    // it. This is also the anti-tautology arm: it proves the allocation probe
    // observes element-list growth at all, so a near-zero hostile number is
    // evidence rather than an artifact.
    let (control_kind, control_len, control_bytes) = measure_walk(&walker, &control_frame);
    assert_eq!(
        (control_kind.as_str(), control_len),
        ("NestedArray", ELEMENTS),
        "the control blob must decode into exactly {ELEMENTS} elements"
    );
    let list_floor = (ELEMENTS * elem) as u64;
    assert!(
        control_bytes >= list_floor,
        "control is not measuring the element list: walking {ELEMENTS} elements \
         requested {control_bytes} B, below the {list_floor} B the list itself occupies \
         ({elem} B per FrameValueKind)"
    );

    // THE GUARD — one word changed, and the walk must not build anything.
    let (hostile_kind, hostile_len, hostile_bytes) = measure_walk(&walker, &hostile_frame);
    assert_eq!(
        (hostile_kind.as_str(), hostile_len),
        ("NestedArrayOpaque", 0),
        "a 1e9 count over a {}-byte body must degrade to opaque",
        hostile.len()
    );
    assert!(
        hostile_bytes.saturating_mul(100) < control_bytes,
        "the count budget did not short-circuit. The hostile walk requested \
         {hostile_bytes} B against the control's {control_bytes} B — i.e. it walked the \
         payload and grew an element list before giving up, which is what \
         `count > remaining / 4` exists to prevent. (The opaque VERDICT is reached \
         either way, so only this number can see the guard.)"
    );
}

/// The boundary itself, asserted on both sides at the SAME body length: a
/// true `count == remaining / 4` decodes, and `count + 1` — the smallest
/// possible over-declaration — is refused.
///
/// This is the arm that fixes the guard's THRESHOLD rather than its existence:
/// widening the divisor (`/ 8`, say) would still short-circuit 1e9 and pass the
/// allocation pin, but it would start refusing well-formed arrays and fails here.
#[test]
#[serial]
fn the_budget_boundary_admits_an_exactly_full_array_and_refuses_one_more() {
    let walker = builtin_walker();
    const N: usize = 16;

    let at_budget = zero_length_string_blob(N as u32, N);
    let over_budget = zero_length_string_blob(N as u32 + 1, N);

    let at_budget_frame = joint_state_frame(&at_budget);
    let fv = walker
        .walk("sensor_msgs/JointState", &at_budget_frame)
        .expect("walk");
    match fv.field("name") {
        Some(FrameValueKind::NestedArray { elements, .. }) => {
            assert_eq!(elements.len(), N, "an exactly-full array must decode whole");
            assert!(
                elements.iter().all(|e| *e == FrameValueKind::Str("")),
                "every zero-length element is the empty string"
            );
        }
        other => panic!("count == remaining/4 must decode, got {other:?}"),
    }

    let over_budget_frame = joint_state_frame(&over_budget);
    let fv = walker
        .walk("sensor_msgs/JointState", &over_budget_frame)
        .expect("walk");
    assert_eq!(
        fv.field("name"),
        Some(&FrameValueKind::NestedArrayOpaque(over_budget.as_slice())),
        "count == remaining/4 + 1 exceeds the budget and must degrade to opaque"
    );
}
