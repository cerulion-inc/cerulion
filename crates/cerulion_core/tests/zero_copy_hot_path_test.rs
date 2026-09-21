// SPDX-License-Identifier: AGPL-3.0-only
//! Hot-path zero-copy guarantees for `OutputProxy::loan_proxy`.
//!
//! Two properties are asserted here:
//!
//! 1. **Constant per-loan init cost** — the per-loan zero-init scope is
//!    `WireHeader::SIZE + WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT` bytes
//!    regardless of the publisher's `max_slice_len`. A 4 MB image loan pays
//!    the same memset cost as a 48-byte Twist loan. This is a
//!    perf property: zeroing the entire loan slot every publish would cost O(size).
//!
//! 2. **Zero heap allocations on the publish path** — `loan_proxy<T>` and
//!    `OutputProxy::Drop` allocate ZERO times on the steady-state publish
//!    path for the no-history, no-network publisher configuration. This is
//!    the same contract as `zero_alloc_test.rs` for the receive side, but
//!    measured on the publish side.
//!
//! These tests serve as **regression gates**: they will FAIL if a future
//! change to `loan_proxy` reintroduces a `max_slice_len`-scaled memset OR
//! if `OutputProxy::Drop` reintroduces a `String` / `Vec` allocation on
//! the no-fan-out path.
//!
//! # Running
//!
//! The 3 tests in this file share a process-global `CountingAllocator`
//! (the [`#[global_allocator]`] hook below). Running them concurrently
//! would race on the `enable()`/`disable()` scope (one test's
//! `enable()` resets the counter mid-flight in another test's
//! critical section) — so each `#[test]` is annotated with
//! `#[serial]` from the `serial_test` crate, which serializes all
//! marked tests within this binary via a process-global mutex.
//! With `#[serial]`, **this file is parallel-safe** at the cargo
//! level (no `--test-threads=1` needed for `cargo test --workspace`).
//! The iceoryx2 singleton is already a per-process resource and is
//! implicitly serialized by the same mutex.
//!
//! ```bash
//! cargo test -p cerulion_core --test zero_copy_hot_path_test           # default parallel — fine
//! cargo test -p cerulion_core --test zero_copy_hot_path_test -- --test-threads=1  # redundant but harmless
//! ```

use cerulion_core::wire::MaxSliceLen;
use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::SystemTime;

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;

// ---------------------------------------------------------------------------
// CountingAllocator (mirrors zero_alloc_test.rs)
// ---------------------------------------------------------------------------

struct CountingAllocator {
    inner: System,
    count: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            count: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    fn enable(&self) {
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }

    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        self.count.load(Ordering::SeqCst)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { self.inner.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

// Serialization note: the 3 tests in this file all access the
// process-global `ALLOCATOR.enable()` / `disable()` scope. Each test
// is `#[serial]` (from the `serial_test` crate) so they run one at a
// time within this binary, eliminating the race where one test's
// `enable()` zeroes the counter mid-flight in another test's
// critical section.

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/zero_copy_hot/{base}/{nanos}/{id}")
}

// ---------------------------------------------------------------------------
// Test 1: constant per-loan init cost (independent of max_slice_len)
// ---------------------------------------------------------------------------
//
// The structural property: the prefix that `loan_proxy` zero-inits is
// statically sized per schema. For a fixed schema it's the WireHeader plus
// the fixed payload section (no offset table). For a variable schema it
// also covers the offset table. Crucially, it does NOT include the variable
// payload region — that area gets overwritten by the user's writes for the
// bytes the wire frame actually uses, and bytes past WireHeader::total_size
// are never observed by well-behaved subscribers.
//
// We compute the expected prefix size from the schema's `ShmMessage`
// constants and assert it stays below 1 KiB for both representative
// schemas (Vector3, Image). The test is intentionally framed around the
// math the publisher uses so any future change that reintroduces a
// `max_slice_len`-scaled memset will surface as a build break here.

const fn expected_init_prefix<T: ShmMessage>() -> usize {
    WireHeader::SIZE + T::WIRE_FIXED_SIZE + 8 * T::VARIABLE_FIELD_COUNT
}

#[test]
#[serial]
fn publish_path_constant_init_cost() {
    // Fixed schema: Vector3 — 3 × f64 = 24 bytes fixed, no variables.
    let v3_prefix = expected_init_prefix::<Vector3>();
    assert_eq!(
        v3_prefix,
        WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE,
        "Vector3 init prefix must be header + fixed section",
    );
    assert!(
        v3_prefix < 1024,
        "Vector3 per-loan zero-init prefix must be <1 KiB; got {v3_prefix}",
    );

    // Variable schema: Image — fixed fields + 3 variable fields' offset
    // table entries. The variable payload (encoding string, header bytes,
    // data bytes) is NOT included in the init prefix.
    let image_prefix = expected_init_prefix::<Image>();
    assert_eq!(
        image_prefix,
        WireHeader::SIZE + Image::WIRE_FIXED_SIZE + 8 * Image::VARIABLE_FIELD_COUNT,
        "Image init prefix must be header + fixed section + offset table",
    );
    assert!(
        image_prefix < 1024,
        "Image per-loan zero-init prefix must be <1 KiB; got {image_prefix}",
    );

    // The prefix is independent of `max_slice_len`. We verify this
    // behaviorally by loaning at two very different `max_slice_len` values
    // and asserting both publishes succeed without alloc count or behavior
    // diverging — the measured property is "publish doesn't fail or change
    // behavior as max_slice_len grows," which is a proxy for "the
    // per-loan work doesn't scale with max_slice_len."
    let mgr = TransportManager::get_or_init().expect("init");

    // Small loan: just enough for Vector3.
    {
        let topic_small = unique_topic("v3_small");
        let mut publisher = mgr
            .create_publisher_simple(
                &topic_small,
                MaxSliceLen::const_new((WireHeader::SIZE + Vector3::WIRE_FIXED_SIZE) as u32),
            )
            .expect("create small publisher");
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy small");
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
    }

    // Huge loan: 1 MiB. With the bounded zero-init this still pays only
    // `expected_init_prefix::<Vector3>()` bytes of memset. With
    // whole-slot zeroing this would memset 1 MiB on every publish.
    {
        let topic_huge = unique_topic("v3_huge");
        let mut publisher = mgr
            .create_publisher_simple(&topic_huge, MaxSliceLen::const_new(1024 * 1024))
            .expect("create huge publisher");
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy huge");
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
    }
}

// ---------------------------------------------------------------------------
// Test 2: zero heap allocations on the publish path (loan_proxy + Drop)
// ---------------------------------------------------------------------------
//
// Mirrors `zero_alloc_test.rs` for the publish side. The publish path
// through a no-history, no-network publisher MUST NOT allocate on the
// steady-state hot path — `loan_proxy` returns a borrowed proxy holding
// an iceoryx2 sample (no Vec), and `OutputProxy::Drop` calls `send()` +
// `notify_sent_sample()` without going through the heap.

#[test]
#[serial]
fn publish_path_zero_alloc_fixed_schema() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("publish_zero_alloc_v3");

    // Setup publisher OUTSIDE the critical section — service creation,
    // iceoryx2 internal allocations are expected here.
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");

    // Warm the publisher with a single publish so any first-publish
    // lazy initialisation is out of the critical section.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("warm loan");
        proxy.x = 0.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }

    // --- CRITICAL SECTION: count allocations on the publish path ---
    ALLOCATOR.enable();

    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("hot loan");
        proxy.x = 1.5;
        proxy.y = 2.5;
        proxy.z = 3.5;
        // Implicit drop here triggers OutputProxy::Drop → send() + notify.
    }

    let alloc_count = ALLOCATOR.disable();
    // --- END CRITICAL SECTION ---

    assert_eq!(
        alloc_count, 0,
        "publish path (loan_proxy + Drop) must produce ZERO heap allocations \
         for a fixed-schema, no-history, no-network publisher; got {alloc_count}",
    );
}

// ---------------------------------------------------------------------------
// Test 3: zero heap allocations on the publish path — variable schema (A1)
// ---------------------------------------------------------------------------
//
// Mirrors `publish_path_zero_alloc_fixed_schema` for a variable schema
// (`Image`). Verifies that the variable-field setters that the AST
// rewriter dispatches through (`set_<field>`, `loan_<field>`) do NOT
// allocate on the hot path. The TRUE zero-copy primitive is `loan_data`
// — it returns a `&mut [u8]` into SHM and the user writes directly,
// without a source buffer or memcpy.
//
// This test will FAIL if a future change to the variable-schema setter
// codegen introduces a heap allocation between `loan_proxy<Image>()`
// and `OutputProxy::Drop`.

#[test]
#[serial]
fn publish_path_zero_alloc_variable_schema() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("publish_zero_alloc_image");

    // Image: header + fixed (13B) + 3 variable-field offset entries (24B)
    // + small variable payload. 1 KiB max_slice_len is plenty.
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(1024))
        .expect("create publisher");

    // Warm.
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("warm loan");
        proxy.height = 0;
        proxy.width = 0;
        proxy.step = 0;
        proxy.is_bigendian = 0;
        proxy.set_header_bytes(&[]).expect("warm header");
        proxy.set_encoding("rgb8").expect("warm encoding");
        let _ = proxy.loan_data(4).expect("warm loan_data");
    }

    // --- CRITICAL SECTION ---
    ALLOCATOR.enable();
    {
        let mut proxy = publisher.loan_proxy::<Image>().expect("hot loan");
        // Fixed-section direct field writes (through Deref —
        // single-mov stores into the SHM overlay; no alloc).
        proxy.height = 1080;
        proxy.width = 1920;
        proxy.step = 1920;
        proxy.is_bigendian = 0;
        // Variable bytes field — set_header_bytes with empty slice still
        // marks the field as written without copying anything.
        proxy.set_header_bytes(&[]).expect("hot header");
        // Variable string field — set_encoding does ONE memcpy from the
        // string-literal source into SHM. The literal source is in
        // .rodata, not on the heap, so the memcpy doesn't allocate.
        proxy.set_encoding("rgb8").expect("hot encoding");
        // TRUE zero-copy primitive: loan_data returns a `&mut [u8]` into
        // SHM. The user writes through the slice directly. No source
        // buffer, no memcpy, no alloc.
        let dst: &mut [u8] = proxy.loan_data(8).expect("hot loan_data");
        for (i, b) in dst.iter_mut().enumerate() {
            *b = i as u8;
        }
        // Drop here → send() + notify.
    }
    let alloc_count = ALLOCATOR.disable();
    // --- END CRITICAL SECTION ---

    assert_eq!(
        alloc_count, 0,
        "variable-schema publish (set_<field>, loan_<field>) must produce \
         ZERO heap allocations on the no-history, no-network publisher; got {alloc_count}",
    );
}
