// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation proof for the SHM-backed receive path.
//!
//! Verifies that draining an inbound iceoryx2 sample through `try_view::<T, _>`
//! and reading SHM-resident fields produces ZERO heap allocations on the
//! steady-state subscriber path. This is the core zero-copy guarantee of
//! Cerulion's transport layer.
//!
//! # Why a separate binary?
//!
//! `#[global_allocator]` is process-wide — it replaces the allocator for ALL
//! code including the test harness. We use an `AtomicBool` switch so only
//! the critical receive section is counted.
//!
//! # Publish-path note
//!
//! Chunk 7 closed the publish-path zero-alloc gap on the iceoryx2
//! backend: `OutputProxy::Drop` no longer eagerly formats the topic
//! into a `String` (it borrows `&str` for tracing).
//! Native history TIGHTENED it further: the heap `HistoryBuffer` is gone, so the
//! post-send `to_vec()` history copy is gone — history is now
//! iceoryx2-native (the SHM frame is retained by offset on `send()`, a
//! borrow-refcount bump, not a malloc). The gateway split removed the last post-send
//! fan-out copy too: publishers are network-free (the separate gateway taps
//! produced topics for egress), so the publish path is UNCONDITIONALLY
//! capture-free and touches the heap zero times. The receive path remains the
//! canonical zero-alloc contract this file verifies; the publish path is
//! exercised here only via the setup section before the critical region.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test zero_alloc_test -- --test-threads=1
//! ```

use cerulion_core::wire::MaxSliceLen;
use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, SystemTime};

use cerulion_core::transport::TransportManager;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;

/// Counting allocator that wraps the system allocator.
///
/// When `enabled` is true, every allocation is counted. This lets the test
/// ignore setup allocations and only measure the critical receive section.
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

/// Monotonic counter so re-runs don't collide on the iceoryx2 service name.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/zero_alloc/{base}/{nanos}/{id}")
}

#[test]
#[serial]
fn test_fixed_schema_try_view_zero_alloc() {
    // Setup: allocations expected here — transport init, publisher/subscriber
    // creation, iceoryx2 internals.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("fixed");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publish a fixed-size Vector3 (3 × f64) so the subscriber has a sample
    // available before the critical section starts.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy fixed");
        proxy.x = 1.5;
        proxy.y = 2.5;
        proxy.z = 3.5;
    }

    // Small delay so iceoryx2 surfaces the sample on the subscriber side.
    std::thread::sleep(Duration::from_millis(50));

    // --- CRITICAL SECTION: count allocations ---
    ALLOCATOR.enable();

    let observed = subscriber
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("try_view fixed");

    let alloc_count = ALLOCATOR.disable();
    // --- END CRITICAL SECTION ---

    let observed = observed.expect("subscriber should see the published Vector3");
    assert!((observed.0 - 1.5).abs() < f64::EPSILON);
    assert!((observed.1 - 2.5).abs() < f64::EPSILON);
    assert!((observed.2 - 3.5).abs() < f64::EPSILON);

    assert_eq!(
        alloc_count, 0,
        "fixed-schema try_view receive path must produce ZERO heap allocations, got {alloc_count}",
    );
}

#[test]
#[serial]
fn test_variable_schema_try_view_zero_alloc() {
    // Setup: allocations expected here.
    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("variable");

    // Image is variable: 3 fixed fields + 3 variable fields (header bytes,
    // encoding string, data bytes).  Allocate enough room for a tiny payload
    // plus the offset table.
    use cerulion_core::message::ShmMessage;
    let max_slice =
        (cerulion_core::wire::WireHeader::SIZE + Image::WIRE_FIXED_SIZE + 8 * 3 + 64) as u32;
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(max_slice))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publish a small Image. Every variable field must be written or the
    // proxy's Drop skips the publish. We use empty-but-written
    // header bytes so the variable_payload region stays small.
    {
        let mut proxy = publisher
            .loan_proxy::<Image>()
            .expect("loan_proxy variable");
        proxy.height = 1;
        proxy.width = 1;
        proxy.is_bigendian = 0;
        proxy.step = 1;
        // Mark the three variable fields written — empty
        // payloads keep the wire frame minimal.
        proxy.set_header_bytes(&[]).expect("set header_bytes");
        proxy.set_encoding("g").expect("set encoding");
        proxy.set_data(&[7u8]).expect("set data");
    }

    std::thread::sleep(Duration::from_millis(50));

    // --- CRITICAL SECTION: count allocations ---
    ALLOCATOR.enable();

    let observed = subscriber
        .try_view::<Image, _>(|view| {
            // Read every accessor type the variable schema exposes:
            // - fixed-section direct field reads (through Deref)
            // - borrowed `&str` for the string (no alloc)
            // - borrowed `&[u8]` for the byte slice (no alloc)
            (
                view.height,
                view.width,
                view.is_bigendian,
                view.step,
                view.encoding().map(str::len).unwrap_or(0),
                view.data().first().copied().unwrap_or(0),
            )
        })
        .expect("try_view variable");

    let alloc_count = ALLOCATOR.disable();
    // --- END CRITICAL SECTION ---

    let observed = observed.expect("subscriber should see the published Image");
    assert_eq!(observed.0, 1, "height should round-trip");
    assert_eq!(observed.1, 1, "width should round-trip");
    assert_eq!(observed.2, 0, "is_bigendian should round-trip");
    assert_eq!(observed.3, 1, "step should round-trip");
    assert_eq!(observed.4, 1, "encoding length should be 1");
    assert_eq!(observed.5, 7, "data[0] should be 7");

    assert_eq!(
        alloc_count, 0,
        "variable-schema try_view receive path must produce ZERO heap allocations, got {alloc_count}",
    );
}

#[test]
#[serial]
fn test_probed_drop_oldest_try_view_zero_alloc() {
    // `drop_oldest` is the DEFAULT input policy, so every
    // graph-wired input runs `try_view` with the eviction probe installed —
    // the per-stream capture (scratch take/find-or-insert/restore) and the
    // per-id baseline walk are on the production hot path and must stay
    // alloc-free at steady state. The warm-up drain below establishes the
    // baseline + brings the scratch to steady-state capacity BEFORE the
    // counted window; a capacity-losing refactor (e.g. dropping a scratch
    // restore) then allocates inside the window and fails loudly.
    use cerulion_core::scheduler::BackpressureCounters;
    use std::sync::Arc;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("probed");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    let counters = Arc::new(BackpressureCounters::new());
    subscriber.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("zero_alloc_node"),
        Arc::from("probed_in"),
        8,
    );

    // Warm-up: one published frame + one drain establishes the per-stream
    // baseline and warms the scratch (allocations here are NOT counted —
    // registration pre-reserves, but the warm-up makes the steady-state
    // claim accurate rather than measuring first-drain behavior).
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan warm-up");
        proxy.x = 1.0;
    }
    std::thread::sleep(Duration::from_millis(50));
    let warmed = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("warm-up try_view");
    assert!(warmed.is_some(), "warm-up frame must arrive");

    // Steady-state frame (contiguous — the Clean path, the hot common case).
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan steady");
        proxy.x = 2.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    // --- CRITICAL SECTION: count allocations ---
    ALLOCATOR.enable();

    let observed = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("probed try_view");

    let alloc_count = ALLOCATOR.disable();
    // --- END CRITICAL SECTION ---

    assert_eq!(observed, Some(2.0));
    assert_eq!(counters.drop_oldest_count.load(Ordering::Relaxed), 0);
    assert_eq!(
        alloc_count, 0,
        "the PROBED drop_oldest receive path (scratch take/find-or-insert/\
         restore + per-id baseline walk) must produce ZERO heap allocations \
         at steady state, got {alloc_count}",
    );
}

/// The `block`-probed receive path is alloc-free at steady state,
/// on a LOCAL word and on a MAPPED (cross-process SHM) one alike.
///
/// The drain's decrement moved from a bare `AtomicU64::fetch_update` to
/// `CreditWord::record_drained`, which additionally bumps a wake epoch and
/// reads a `parked` mask. That is three atomic operations where there was one,
/// and it sits on the CONSUMER hot path of every lossless edge — so the claim
/// "no allocation" has to be re-made rather than inherited, and it has to be
/// made on BOTH arms, because the mapped arm reaches its `CreditShared` through
/// an extra `Deref` into an mmap'd page.
///
/// The pair also pins the OTHER half of the design: the two arms run the same
/// code, so a divergence would have to show up as a different allocation count.
#[test]
#[serial]
fn test_block_probed_try_view_is_zero_alloc_on_both_word_arms() {
    use cerulion_core::credit::{credit_edge_id, CreditWord, MappedCredit};
    use cerulion_core::scheduler::BackpressureCounters;
    use std::sync::Arc;

    /// One warm-then-count cycle over a `block`-probed subscriber.
    fn count_allocs_for(word: CreditWord, topic: &str) -> u64 {
        let mgr = TransportManager::get_or_init().expect("init");
        let mut publisher = mgr
            .create_publisher_simple(topic, MaxSliceLen::const_new(256))
            .expect("create publisher");
        let mut subscriber = mgr.create_subscriber(topic).expect("create subscriber");
        subscriber.register_block_probe_for_test(
            word.clone(),
            4,
            Arc::new(BackpressureCounters::new()),
            Arc::from("blocked_in"),
            8,
        );

        // Warm-up drain: brings the drain scratch to steady-state capacity, so
        // the counted window measures the STEADY path and not a first drain.
        {
            let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan warm-up");
            proxy.x = 1.0;
        }
        word.record_published();
        std::thread::sleep(Duration::from_millis(50));
        assert!(
            subscriber
                .try_view::<Vector3, _>(|view| view.x)
                .expect("warm-up try_view")
                .is_some(),
            "warm-up frame must arrive"
        );

        {
            let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan steady");
            proxy.x = 2.0;
        }
        word.record_published();
        std::thread::sleep(Duration::from_millis(50));

        // --- CRITICAL SECTION: count allocations ---
        ALLOCATOR.enable();
        let observed = subscriber
            .try_view::<Vector3, _>(|view| view.x)
            .expect("block-probed try_view");
        let alloc_count = ALLOCATOR.disable();
        // --- END CRITICAL SECTION ---

        assert_eq!(observed, Some(2.0));
        alloc_count
    }

    let local_topic = unique_topic("blocklocal");
    let local = count_allocs_for(CreditWord::local(4), &local_topic);
    assert_eq!(
        local, 0,
        "the block-probed receive path must allocate NOTHING at steady state on a LOCAL \
         credit word (the drain decrement + wake-epoch bump + parked read are all atomics), \
         got {local}"
    );

    let ns = format!("zeroalloc_{}", std::process::id());
    let id = credit_edge_id("/za/data", "consumer", "inp");
    let owner = MappedCredit::create_owned(&ns, &id, 4).expect("credit word");
    let peer = Arc::new(MappedCredit::open_unowned(&ns, &id).expect("peer mapping"));
    let mapped_topic = unique_topic("blockmapped");
    let mapped = count_allocs_for(CreditWord::mapped(Arc::clone(&peer)), &mapped_topic);
    assert_eq!(
        mapped, 0,
        "and NOTHING on a MAPPED word either — the extra hop is a Deref into an mmap'd page, \
         not an allocation, got {mapped}"
    );
    drop(owner);
}

#[test]
#[serial]
fn test_sample_gated_try_view_zero_alloc() {
    // The sample(N) read-gate path must stay
    // alloc-free on BOTH of its outcomes — accept and decimate. The accept
    // below is the subscriber's FIRST drain on purpose (no warm-up): a
    // sample(N) input reserves no per-stream scratch at registration
    // (`probing` is false for Sample), so a regression that includes Sample
    // in `try_view`'s `probing` predicate pushes into a capacity-0 scratch
    // on this very drain and fails the accept assertion loudly.
    use cerulion_core::scheduler::BackpressureCounters;
    use std::sync::Arc;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("sample_gated");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    let counters = Arc::new(BackpressureCounters::new());
    // 10s interval: the second frame (published ~50ms after the first) is
    // guaranteed to fall inside the gate window and decimate.
    subscriber.register_sample_gate_for_test(
        10_000,
        Arc::clone(&counters),
        Arc::from("zero_alloc_node"),
        Arc::from("sampled_in"),
        8,
    );

    // Frame 1 → the gate's first accept.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan accept");
        proxy.x = 1.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    // --- CRITICAL SECTION 1: the ACCEPT path ---
    ALLOCATOR.enable();
    let observed = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("accept try_view");
    let accept_allocs = ALLOCATOR.disable();
    // --- END CRITICAL SECTION 1 ---
    assert_eq!(observed, Some(1.0), "the first read must be accepted");
    assert_eq!(
        accept_allocs, 0,
        "the sample-gated ACCEPT path must produce ZERO heap allocations \
         (got {accept_allocs}) — a nonzero count here also catches a \
         regression that pays the per-stream scratch capture on Sample inputs",
    );

    // Frame 2, well inside the 10s window → decimated (dropped, counter
    // bumped, edge-trigger event queued — all alloc-free).
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan decimate");
        proxy.x = 2.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    // --- CRITICAL SECTION 2: the DECIMATE path ---
    ALLOCATOR.enable();
    let observed = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("decimate try_view");
    let decimate_allocs = ALLOCATOR.disable();
    // --- END CRITICAL SECTION 2 ---
    assert_eq!(observed, None, "the too-soon read must be decimated");
    assert_eq!(counters.sampled_count.load(Ordering::Relaxed), 1);
    assert!(
        subscriber.try_take_backpressure_event().is_some(),
        "the first decimate of a regime queues the edge-triggered event"
    );
    assert_eq!(
        decimate_allocs, 0,
        "the sample-gated DECIMATE path (gate check + counter bump + event \
         queue) must produce ZERO heap allocations, got {decimate_allocs}",
    );
}
