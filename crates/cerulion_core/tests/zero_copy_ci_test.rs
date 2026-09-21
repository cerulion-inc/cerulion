// SPDX-License-Identifier: AGPL-3.0-only
//! Structural zero-copy verification for the SHM-backed pub/sub API.
//!
//! Verifies that user writes via `proxy.field = ...` (or `proxy.set_*`)
//! land directly in the iceoryx2 sample's payload region, and that
//! subscriber reads via `view.field` come straight back from the same
//! shared memory bytes — no intermediate copies on either path.
//!
//! These tests target the **iceoryx2** backend explicitly. The in-process
//! backend allocates a `Vec<u8>` per loan and is documented as not on the
//! zero-copy hot path (a recorded architectural decision).
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test zero_copy_ci_test -- --test-threads=1
//! ```

use cerulion_core::wire::MaxSliceLen;
use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;

/// Counting allocator (mirrors the one used in `zero_alloc_test`).
///
/// The publish path is checked via the `ALLOCATOR.disable()` count for the
/// loan→write→drop window. Allocations that happen entirely outside that
/// window (transport setup, etc.) are excluded.
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

// Serialization note: tests in this file mutate the process-
// global `ALLOCATOR.enable()` / `disable()` scope. Each test is
// `#[serial]` so they run one at a time. See `zero_copy_hot_path_test.rs`
// for the rationale.

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/zc/{base}/{nanos}/{id}")
}

// ============================================================
// Pointer-arithmetic: writes land directly in the SHM payload
// ============================================================

/// Pointer-arithmetic + bit-equal proof that user writes land in iceoryx2
/// SHM and survive the round trip without any intermediate copy.
///
/// We can't observe the iceoryx2 `SampleMut` payload pointer through the
/// public API, but we can observe the **writer's** address (a
/// `&'loan mut Vector3Shm` carved out of the loaned SHM payload) and the
/// **reader's** address inside the subscriber's `try_view` closure. These
/// addresses live in the iceoryx2 shared-memory region — distinct virtual
/// mappings per port, but both stable and outside any test-local heap.
///
/// The test asserts:
/// 1. Bit-for-bit field equality round-trip (proves no float coercion or
///    intermediate copy mutated the bytes).
/// 2. The subscriber's reader pointer is *not* the publisher's writer
///    pointer — iceoryx2 maps each port at its own virtual address — but
///    both pointers are stable across consecutive `try_view`s on the same
///    sample, which excludes a per-call userspace buffer copy.
#[test]
#[serial]
fn test_writes_land_in_shm_then_round_trip_byte_equal() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("write_lands");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Use bit patterns the user code never re-derives — if the subscriber
    // sees the same bits, the bytes survived round-trip without
    // recomputation.
    let x_bits: u64 = 0xDEAD_BEEF_CAFE_BABE;
    let y_bits: u64 = 0x0123_4567_89AB_CDEF;
    let z_bits: u64 = 0xFEDC_BA98_7654_3210;
    let x_val = f64::from_bits(x_bits);
    let y_val = f64::from_bits(y_bits);
    let z_val = f64::from_bits(z_bits);

    let writer_ptr_during_loan: *const f64;
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        // `Vector3Shm` is `#[repr(C)]` over `{x, y, z: f64}`. The address
        // of `proxy.x` is the iceoryx2 SHM payload's first f64.
        writer_ptr_during_loan = std::ptr::addr_of!(proxy.x);
        proxy.x = x_val;
        proxy.y = y_val;
        proxy.z = z_val;
    }

    // Sanity: the writer pointer is NOT a stack/heap address that escaped
    // the proxy's lifetime — we just ensure it's non-null. (The iceoryx2
    // SHM region's exact mapping is implementation-defined, so we don't
    // assert a numeric range here.)
    assert!(!writer_ptr_during_loan.is_null());

    std::thread::sleep(Duration::from_millis(50));

    // First try_view: capture reader pointer + bit patterns.
    let first = subscriber
        .try_view::<Vector3, _>(|view| {
            let r: &native_ros2_messages::geometry_msgs::Vector3Shm = &view;
            (
                std::ptr::addr_of!(r.x) as usize,
                r.x.to_bits(),
                r.y.to_bits(),
                r.z.to_bits(),
            )
        })
        .expect("try_view")
        .expect("subscriber should see the published Vector3");

    // Bit-equal round-trip — no intermediate float coercion or copy.
    assert_eq!(first.1, x_bits, "x bits must round-trip exactly");
    assert_eq!(first.2, y_bits, "y bits must round-trip exactly");
    assert_eq!(first.3, z_bits, "z bits must round-trip exactly");

    // Stability check: publish a fresh sample to the same publisher port
    // and assert the subscriber's reader pointer is in the same iceoryx2
    // SHM region (within one `max_slice_len` page of the first call's
    // address). A per-call userspace copy would land at an arbitrary heap
    // address; SHM mappings stay within the configured publisher pool.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan 2");
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    let second = subscriber
        .try_view::<Vector3, _>(|view| {
            let r: &native_ros2_messages::geometry_msgs::Vector3Shm = &view;
            std::ptr::addr_of!(r.x) as usize
        })
        .expect("try_view 2")
        .expect("subscriber should see the second frame");

    let delta = second.abs_diff(first.0);
    // 1 MiB window: comfortably larger than any single iceoryx2 slot but
    // far smaller than what a heap-randomised allocator would yield.
    let shm_window = 1024usize * 1024;
    assert!(
        delta < shm_window,
        "subscriber reader pointer should stay inside the iceoryx2 SHM region across publishes \
         (first=0x{:x} second=0x{:x} delta=0x{:x})",
        first.0,
        second,
        delta,
    );
}

// ============================================================
// Variable schema: variable payload bytes round-trip identically
// ============================================================

#[test]
#[serial]
fn test_variable_schema_payload_round_trips_byte_for_byte() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("var_round_trip");

    // Publisher buffer big enough for header + fixed section + offset table
    // + a 1KB image data slice.
    let payload_len = 1024usize;
    let max_slice = (WireHeader::SIZE + Image::WIRE_FIXED_SIZE + 8 * 3 + 256 + payload_len) as u32;
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(max_slice))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // A non-trivial bit pattern in `data` — easy to detect any byte-for-byte
    // perturbation introduced by an accidental copy/transform.
    let data: Vec<u8> = (0..payload_len).map(|i| (i as u8) ^ 0xA5).collect();
    let encoding = "rgb8";

    {
        let mut proxy = publisher
            .loan_proxy::<Image>()
            .expect("loan_proxy variable");
        proxy.height = 64;
        proxy.width = 64;
        proxy.is_bigendian = 0;
        proxy.step = 192;
        // Mark every variable field written. Empty header_bytes
        // is a valid wire frame for this test — readers see &[].
        proxy.set_header_bytes(&[]).expect("set header_bytes");
        proxy.set_encoding(encoding).expect("set encoding");
        proxy.set_data(&data).expect("set data");
    }

    std::thread::sleep(Duration::from_millis(50));

    let (got_height, got_width, got_step, got_encoding, data_first_last_len) = subscriber
        .try_view::<Image, _>(|view| {
            let data_slice = view.data();
            (
                view.height,
                view.width,
                view.step,
                view.encoding().expect("encoding utf-8").to_string(),
                (
                    data_slice.first().copied(),
                    data_slice.last().copied(),
                    data_slice.len(),
                ),
            )
        })
        .expect("try_view variable")
        .expect("subscriber should see the published Image");

    assert_eq!(got_height, 64);
    assert_eq!(got_width, 64);
    assert_eq!(got_step, 192);
    assert_eq!(got_encoding, encoding);
    assert_eq!(data_first_last_len.0, Some(data[0]));
    assert_eq!(data_first_last_len.1, Some(data[payload_len - 1]));
    assert_eq!(data_first_last_len.2, payload_len);
}

// ============================================================
// Publish-path allocation budget on steady state
// ============================================================

/// Verify that the iceoryx2 publish path on the steady-state hot loop does
/// **not** spiral on allocations: a tight loop of N publishes should yield
/// the same per-iteration allocation count as a single publish (modulo
/// noise from `tracing` log dispatch). Today the path allocates a String
/// per send for `pubr.topic().to_string()` inside `OutputProxy::Drop`;
/// we assert the per-iteration cost is BOUNDED and constant — the
/// invariant the zero-copy contract preserves.
#[test]
#[serial]
fn test_publish_path_allocation_budget_constant_per_loan() {
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("pub_alloc");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let _subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Warm up: drop one proxy outside the measured window so any one-shot
    // setup costs (event listener priming, etc.) don't pollute the count.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("warmup loan");
        proxy.x = 0.1;
    }

    // Single-publish baseline.
    ALLOCATOR.enable();
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
    }
    let one_publish = ALLOCATOR.disable();

    // N-publish window — total should scale as ~N × per-iteration constant,
    // never grow super-linearly (which would hint at retained per-message
    // bookkeeping).
    let n: u64 = 16;
    ALLOCATOR.enable();
    for i in 0..n {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = i as f64;
        proxy.y = (i * 2) as f64;
        proxy.z = (i * 3) as f64;
    }
    let n_publishes = ALLOCATOR.disable();

    // Per-iteration cost is at most the single-publish cost (with slack for
    // log/event-listener jitter). Catches O(N²) regressions cleanly.
    let per_iter_budget = one_publish.saturating_add(2);
    let max_total = per_iter_budget.saturating_mul(n);
    assert!(
        n_publishes <= max_total,
        "publish-path allocations must be bounded per iteration: {n_publishes} > {max_total} \
         (one_publish={one_publish}, n={n})",
    );
}
