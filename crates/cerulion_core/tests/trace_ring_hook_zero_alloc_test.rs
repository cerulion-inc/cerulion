// SPDX-License-Identifier: AGPL-3.0-only
//! Recording-ON zero-allocation proof.
//!
//! With the trace-ring hook INSTALLED, a warmed flat `Scheduler::step` (fire a
//! `Period` node → `RingTraceSink::push_entry` → `TraceRingHook::push_fire` →
//! `TraceRingProducer::push`) must do ZERO heap allocations per step at steady
//! state. `push_fire` is alloc-free by construction — a `HashMap<String,u32>`
//! lookup by `&str` (via `Borrow`, no owned key), a 40-byte STACK encode, and
//! the wait-free `ShmRingProducer::push` (proven 0-alloc by
//! `shm_ring_zero_alloc_test`). The graph-runtime arm's consumer
//! also runs with its READ-OUTCOME stage ARMED, so the measured window covers
//! the full armed-capture path too — `stage.record` under the uncontended
//! mutex, the level-end `merge_read_outcomes` drain, and `push_read_outcome`
//! into the ring. This is the regression guard that a future change (an owned
//! key clone, a `format!`, a per-fire `Vec` — on the fire path OR the capture
//! path) would trip.
//!
//! # Why a separate binary + one `#[serial]` body + a thread-scoped counter
//!
//! `#[global_allocator]` is process-wide; the same reasoning as
//! `shm_ring_zero_alloc_test` applies verbatim — see that file's header. The
//! counter is thread-scoped so background-thread allocs (libtest / TLS / lazy
//! init, which differ per platform) never pollute the measured window; a real
//! push-path alloc still fails because it lands on the measured thread.
//!
//! SHM-touching (mints a real `TraceRingOwner`) — run it inside the serialized
//! SHM test window, never in parallel with other SHM tests.

#![cfg(unix)]

use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TriggerPolicy};
use cerulion_core::trace_ring::{TraceRingConsumer, TraceRingOwner};
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Exclude the iceoryx2 liveliness sweep from the alloc window (the
/// `step_zero_alloc_test` `SWEEP_DISABLED_MS` discipline -- the sweep is a
/// background registry scan OUTSIDE the per-step executor hot path).
const SWEEP_DISABLED_MS: u64 = 3_600_000;

/// A `Period(1)` source publishing into `Vector3.x` -- one macro TYPE reused
/// across the 3 node instances of the measured level (the
/// `step_zero_alloc_test` tri pattern; a `Vector3` fixed-field write is
/// proven zero-alloc at steady state by that file's D1 gates).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct ZaSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ZaSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// A data-trigger CONSUMER in the measured graph —
/// with recording ON its read-outcome stage is ARMED, so every measured step
/// exercises the full capture path (unified trigger drain → `stage.record`
/// under the uncontended mutex → level-end `merge_read_outcomes` →
/// `push_read_outcome` into the ring) inside the zero-alloc window.
#[cerulion_node]
#[derive(Default)]
struct ZaSink {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl ZaSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

thread_local! {
    /// `true` only on the thread that called [`CountingAllocator::enable`].
    /// Const-init non-`Drop` `Cell<bool>` so no lazy TLS init recurses into the
    /// allocator (see `shm_ring_zero_alloc_test` for the full rationale).
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// Counts heap allocations made by the MEASURED THREAD while a window is open.
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
        MEASURED_THREAD.set(true);
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.count.load(Ordering::SeqCst)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            let measured = MEASURED_THREAD.try_with(Cell::get).unwrap_or(false);
            if measured {
                self.count.fetch_add(1, Ordering::Relaxed);
            }
        }
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

#[test]
#[serial]
fn recording_on_scheduler_step_is_zero_alloc_at_steady_state() {
    // --- Part 1: recording-ON step is zero-alloc at steady state ---
    let mut owner = TraceRingOwner::create(&format!("za_{}", std::process::id()), 1024, 0, &["n0"])
        .expect("create ring");
    let producer = owner.producer().expect("mint producer");

    let mut scheduler = Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()));
    // Cap the in-memory trace so its VecDeque reaches steady-state capacity
    // (pop_front + push_back reuses the buffer — no per-step realloc).
    scheduler.set_trace_limit(4);
    scheduler.set_trace_ring_producer(producer, &["n0".to_string()]);
    scheduler
        .add_node(NodeConfig {
            id: "n0".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(10),
                max_catchup: None,
            },
            callback: Box::new(|| {}),
        })
        .unwrap();

    // Warm up (excluded): fill the trace to its cap + page the ring mapping in.
    for _ in 0..128 {
        scheduler.step_ms(10);
    }

    ALLOCATOR.enable();
    for _ in 0..10_000 {
        scheduler.step_ms(10);
    }
    let allocs = ALLOCATOR.disable();
    assert_eq!(
        allocs, 0,
        "recording-ON scheduler step (fire + ring push) must not heap-allocate \
         at steady state (got {allocs})"
    );

    drop(owner);

    // --- Part 1b: the PARALLEL path — a 3-wide
    // same-Period level routed through `tick_decided_parallel` (3 fires ≥ 2,
    // < PARALLEL_FIRE_THRESHOLD(8) → the strictly-zero-alloc serial REST; the
    // repo's established strict-0 arm — the WIDE path carries rayon's ~1/63
    // injector residual and is deliberately not the strict gate, see
    // `step_zero_alloc_test`). The
    // fragment merge funnels every fire through `RingTraceSink` →
    // `TraceRingHook::push_fire`, all on the measured (step-calling) thread,
    // so a ring-arm alloc regression lands on this counter. Uses the GLOBAL
    // TransportManager singleton (the `step_zero_alloc_test` build pattern) —
    // this binary is `#[serial]` and gated to the serialized SHM window. ---
    // The graph also carries a data-trigger CONSUMER
    // ("zsink" on za0/out) so the measured window covers the ARMED
    // read-outcome capture path end to end — stage.record at the unified
    // trigger drain, the level-end merge, and the kind-6 ring push. The 3
    // sources keep their own 3-wide level (the parallel-path property this
    // part exists for); the trigger edge levelizes zsink below them.
    let node_ids: Vec<String> = ["za0", "za1", "za2", "zsink"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    let id_refs: Vec<&str> = node_ids.iter().map(String::as_str).collect();
    let mut owner = TraceRingOwner::create(
        &format!("za_par_{}", std::process::id()),
        1 << 16,
        0,
        &id_refs,
    )
    .expect("create parallel ring");
    let ring_name = owner.name().to_string();
    let producer = owner.producer().expect("mint parallel producer");

    let src_ids = &node_ids[..3];
    let mut nodes: Vec<NodeDef> = src_ids
        .iter()
        .map(|id| NodeDef {
            ros2: None,
            id: id.clone(),
            node_type: "za_src".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        })
        .collect();
    nodes.push(NodeDef {
        ros2: None,
        id: "zsink".to_string(),
        node_type: "za_sink".to_string(),
        inputs: vec![cerulion_core::graph::config::InputDef {
            name: "inp".to_string(),
            source: "za0/out".to_string(),
        }],
        outputs: vec![],
    });
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "trace_ring_par_za".to_string(),
        prefix: format!("trpza{}", std::process::id()),
        nodes,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in src_ids {
        factories.insert(id.clone(), Box::new(ZaSrcEntry::new()));
    }
    factories.insert("zsink".to_string(), Box::new(ZaSinkEntry::new()));
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build(config, factories, &mgr, clock)
        .expect("build 3-wide parallel ring graph");
    runtime.set_liveliness_sweep_period_for_test(SWEEP_DISABLED_MS);
    // Cap the in-memory trace so its VecDeque reaches steady-state capacity
    // (pop_front + push_back reuses the buffer — no growth in the window).
    runtime.set_trace_limit(16);
    runtime.set_trace_ring_producer(producer, &node_ids);
    // The recording really ARMED the consumer's read-outcome stage
    // (else the alloc window below would not cover the capture path at all).
    assert!(
        runtime.read_outcome_stages_armed_for_test() > 0,
        "recording-ON must arm the wired read-outcome stages"
    );

    // Warm to steady state (SHM pool, trace-deque capacity, one-time lazies).
    const WARM_STEPS: usize = 300;
    const MEASURE_STEPS: usize = 2000;
    for _ in 0..WARM_STEPS {
        runtime.step(Duration::from_millis(1));
    }

    ALLOCATOR.enable();
    for _ in 0..MEASURE_STEPS {
        runtime.step(Duration::from_millis(1));
    }
    let allocs = ALLOCATOR.disable();
    assert_eq!(
        allocs, 0,
        "recording-ON PARALLEL step (3-wide level through tick_decided_parallel's \
         fragment merge + armed read-outcome capture + ring push) must not \
         heap-allocate at steady state on the calling thread (got {allocs})"
    );

    // Anti-tautology: the hook really pushed through the parallel path — per
    // step the ring carries 4 fires (3 sources + zsink) + 1 StepBoundary
    // (contract addendum) + 1 kind-6 READ-OUTCOME record (zsink's
    // unified trigger drain pops za0's one frame per step) = 6 records.
    let mut consumer = TraceRingConsumer::open(&ring_name).expect("open parallel consumer");
    let mut records = Vec::new();
    consumer.drain(&mut records).expect("drain (no overrun)");
    assert_eq!(
        records.len(),
        (WARM_STEPS + MEASURE_STEPS) * 6,
        "every fire (4/step) + the step boundary (1/step) + zsink's read-outcome \
         record (1/step) must reach the ring"
    );
    let kind6 = records
        .iter()
        .filter(|r| r.record_type == cerulion_core::trace_ring::RECORD_TYPE_READ_OUTCOME)
        .count();
    assert_eq!(
        kind6,
        WARM_STEPS + MEASURE_STEPS,
        "the armed capture really recorded inside the measured window \
         (anti-vacuity for the zero-alloc claim above)"
    );
    drop(consumer);
    runtime.shutdown();
    drop(owner);

    // --- Part 2: sanity control — the thread-scoped counter still BITES ---
    ALLOCATOR.enable();
    let v: Vec<u8> = Vec::with_capacity(1);
    std::hint::black_box(v);
    let allocs = ALLOCATOR.disable();
    assert_eq!(
        allocs, 1,
        "a deliberate measured-thread allocation must count exactly 1 (got {allocs})"
    );
}
