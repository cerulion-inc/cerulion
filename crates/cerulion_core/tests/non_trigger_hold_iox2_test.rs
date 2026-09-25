// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-step HOLD of a non-trigger latest-value `#[input]`, over real
//! iceoryx2 (`GraphRuntime::build_for_test`, per-test SHM root).
//!
//! # The contract these tests pin
//!
//! A non-trigger latest-value `#[input]` of a MACRO node:
//!
//! - **before its FIRST delivery** → the snapshot drain stays `Empty` (no
//!   `held_sample` yet), the macro's nested `try_view` collapses the WHOLE tick
//!   to a no-op (`Ok(None) => Ok(Ok(false))`, where the `bool`
//!   discriminant marks this a COLLAPSE, distinct from genuine success, so
//!   any declared output's publish stays un-armed too), so the node WAITS —
//!   it publishes nothing and NEVER fabricates a default (Principle #13).
//! - **after first delivery** → `snapshot_latest` REPLAYS the last-delivered
//!   value (`FrozenSlot::Held`) on every step with no new arrival, until a newer
//!   sample arrives. The replay is a pure function of past deliveries →
//!   deterministic (Principle #7).
//! - the hold pins ONE iceoryx2 sample across the drain, so a burst drain peaks
//!   at held(1)+latest(1)+receive-transient(1) = 3 borrowed samples — budgeted
//!   by `subscriber_max_borrowed_samples = 3` on snapshot-source topics (the
//!   iceoryx2 default is 2).
//!
//! # Observability (mirrors `snapshot_wiring_iox2_test`)
//!
//! The consumer records what it reads (`self.inp.x as u64`) into a shared
//! `Arc<AtomicU64>` INSIDE its tick. The harness resets it to [`MISSING`] before
//! each step, so:
//!
//!   - a recorded value  == "the tick RAN" (input was `Sample` or `Held`)
//!   - `MISSING` left     == "the tick NO-OP'd" (input was `Empty` → WAIT)
//!
//! i.e. `MISSING` is exactly "no output published this step". This is the same
//! MISSING-sentinel mechanism `snapshot_wiring_iox2_test` uses; it observes the
//! held value directly without a second iceoryx2 hop (more robust than a probe
//! subscriber on a re-published output — no extra delivery timing).
//!
//! Every recorded value comes from a REAL iceoryx2 publish read inside a real
//! tick (no fake data, Principle #13). Every assertion is against a HAND-WRITTEN
//! oracle (literal expected values), never a self-compare of two runs of the
//! same closure (the determinism test compares two runs AND the hand oracle).
//!
//! # Running (iceoryx2 SHM singleton + a process-global `#[global_allocator]`
//! whose measurement windows are shared state → serial, one thread; the
//! COUNTING is thread-scoped, but two concurrent `enable()`s
//! would still share one count)
//!
//! ```bash
//! cargo test -p cerulion_core --test non_trigger_hold_iox2_test -- --test-threads=1
//! ```

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::CrossProcessWiring;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// A sentinel the producers never publish (they publish 1, 2, 3, ... or a fixed
/// 42). Recorded ONLY when the consumer's tick did NOT run — i.e. the snapshot
/// served `Empty` (no held value / pre-first-delivery WAIT, or an errored
/// drain). The macro collapses an `Empty` non-trigger input to a no-op tick, so
/// the recording store never executes and `MISSING` survives.
const MISSING: u64 = u64::MAX;

/// Step delta. The graph nodes (tests 1-4) are external — they fire on
/// `trigger_external` regardless of the clock — so the delta is immaterial; 1 ms
/// keeps the virtual clock well under any graph grace deadline.
const STEP: Duration = Duration::from_millis(1);

// ===========================================================================
// Counting allocator (THREAD-SCOPED — the shm_ring_zero_alloc_test.rs
// discipline, adopted here) — used by the two zero-alloc replay tests
// only; disabled for every other test.
//
// THREAD-SCOPED because the contract under test is "the held-replay path
// performs no heap allocation ON THE CALLING THREAD". A
// process-wide counter counts EVERY thread while a window is open, so a
// background-thread or lazy-init allocation the test does not control —
// harness machinery, platform TLS init, a tracing lazy static — landing
// inside the measured window fails the pin (the signature is a failure that SURVIVES
// the in-binary nextest retry, because the polluting lazy init stays warm in
// the same process, yet vanishes on a fresh-process rerun). A real allocation
// introduced into the measured path still fails: it happens on the measured
// thread. The phase-3 sanity control at the end of
// `hold_replay_through_executor_is_zero_alloc` proves the scoped counter
// still bites.
// ===========================================================================

thread_local! {
    /// `true` only on the thread that called [`CountingAllocator::enable`].
    ///
    /// MUST be `const`-init with a non-`Drop` payload (`Cell<bool>`): a lazy
    /// (allocating) TLS init or a registered destructor touched from inside
    /// `GlobalAlloc::alloc` would RECURSE into the allocator. Const-init TLS of
    /// a plain `Cell<bool>` performs no allocation and registers no destructor
    /// on first touch.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

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
    /// Open a measurement window: zero the count, mark THIS thread as the
    /// measured one, and enable counting.
    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    /// Close the window (clearing both the global flag and this thread's
    /// measured mark) and return the count.
    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.count.load(Ordering::SeqCst)
    }
    /// Whether the calling thread's allocation counts right now.
    ///
    /// `try_with`, never `with`: during thread teardown TLS is inaccessible and
    /// `with` would panic INSIDE the allocator. An inaccessible TLS means "not
    /// the measured thread" — don't count.
    fn measuring_here(&self) -> bool {
        self.enabled.load(Ordering::Relaxed) && MEASURED_THREAD.try_with(Cell::get).unwrap_or(false)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.measuring_here() {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
    // Counted EXPLICITLY: a `Vec` growing inside the measured window is a heap
    // allocation like any other. The default trait impl would also count (it
    // routes through `self.alloc`) but degrades every realloc to
    // alloc+copy+free; overriding keeps `System::realloc` while counting.
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if self.measuring_here() {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { self.inner.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

/// Monotonic topic-name counter for the (global-manager) zero-alloc test so
/// re-runs never collide on the iceoryx2 service name (mirrors
/// snapshot_view_iox2_test).
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

// ===========================================================================
// Node types
// ===========================================================================

/// External producer publishing a FIXED scalar into `out.x` on each fire.
/// `self.out.x = ...` writes straight into the loaned SHM slot (Vector3 is a
/// fixed schema; fields via Deref).
#[cerulion_node(external)]
#[derive(Default)]
struct HoldProducer {
    #[output]
    out: Vector3,
    val: f64,
}

#[cerulion_node_impl]
impl HoldProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.val;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// External producer publishing an INCREMENTING counter (1, 2, 3, ...). Used by
/// the burst test, where the drained samples must carry distinct values.
#[cerulion_node(external)]
#[derive(Default)]
struct IncProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl IncProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Increment FIRST so the first publish carries 1 (distinct from MISSING
        // and from a Vector3 default 0).
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// External consumer with ONE non-trigger latest-value `#[input]` (plain
/// `#[input]` → DropOldest → snapshotted). External so the harness controls
/// EXACTLY which steps it fires (and thus snapshots). Records the value it reads
/// into `last_read` every tick; a no-op'd (Empty) tick records nothing.
#[cerulion_node(external)]
#[derive(Default)]
struct HoldConsumer {
    #[input]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl HoldConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // `self.inp.x` serves the FROZEN slot (snapshot_latest): a delivered
        // Sample, the replayed Held value, or — if Empty — the macro collapses
        // this whole tick to a no-op and this store never runs.
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

// ===========================================================================
// Graph builders + drivers
// ===========================================================================

fn vec3_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

/// `producer/out` → `consumer.inp` (non-trigger). `producer` is supplied by the
/// caller so the SAME topology serves both the fixed-value and incrementing
/// producers. Returns the runtime + the consumer's shared read record.
fn build_graph(
    prefix: &str,
    producer_type: &str,
    producer: Box<dyn NodeEntry>,
) -> (GraphRuntime, Arc<AtomicU64>) {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "non_trigger_hold".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: producer_type.to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "hold_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), producer);
    let consumer = HoldConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(HoldConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build hold graph");
    (runtime, last_read)
}

/// Fixed-value (42) producer + external consumer.
fn build_hold_graph(prefix: &str, val: f64) -> (GraphRuntime, Arc<AtomicU64>) {
    let producer = HoldProducer {
        val,
        ..Default::default()
    };
    build_graph(
        prefix,
        "hold_producer",
        Box::new(HoldProducerEntry::with_state(producer)),
    )
}

/// Incrementing producer + external consumer (burst test).
fn build_inc_graph(prefix: &str) -> (GraphRuntime, Arc<AtomicU64>) {
    build_graph(prefix, "inc_producer", Box::new(IncProducerEntry::new()))
}

/// Fire the producer ONCE (publishing its value), then fire ONLY the consumer
/// (producer silent) until it reads `expected` — establishing the held value.
/// The producer + consumer fire on SEPARATE steps, so there is no same-level
/// same-step snapshot lag to reason about (the snapshot drains the prior step's
/// publish). The bounded loop tolerates iceoryx2 surfacing latency.
fn establish_held(rt: &mut GraphRuntime, last_read: &Arc<AtomicU64>, expected: u64) {
    rt.trigger_external("producer").expect("trigger producer");
    rt.step(STEP); // producer publishes; consumer NOT fired this step
    let mut tries = 0;
    loop {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        if last_read.load(Ordering::Relaxed) == expected {
            return;
        }
        tries += 1;
        assert!(
            tries < 200,
            "held value {expected} never established within 200 consumer fires"
        );
    }
}

/// Fire ONLY the consumer (producer silent) for `horizon` steps, returning the
/// recorded read on each step. With the producer silent, every step's snapshot
/// drains `Empty` → REPLAYS the held value.
fn replay_window(rt: &mut GraphRuntime, last_read: &Arc<AtomicU64>, horizon: usize) -> Vec<u64> {
    let mut out = Vec::with_capacity(horizon);
    for _ in 0..horizon {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        out.push(last_read.load(Ordering::Relaxed));
    }
    out
}

// ===========================================================================
// Test 1 — core proof: a non-trigger input replays its last-delivered value
// across MANY silent steps.
// ===========================================================================

#[test]
#[serial]
fn many_step_hold_replays_last_delivered_value() {
    const V: u64 = 42;
    const HORIZON: usize = 20;
    let (mut rt, last_read) = build_hold_graph("nthcore", V as f64);

    // (a) PRE-DELIVERY: consumer fires, producer NEVER → no delivery → the macro
    // no-ops the whole tick (Empty input) → publishes nothing. MISSING == "tick
    // did not run" == WAIT.
    for _ in 0..5 {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        assert_eq!(
            last_read.load(Ordering::Relaxed),
            MISSING,
            "pre-delivery: a non-trigger input with no delivery must leave the \
             tick a no-op (WAIT) — nothing recorded, NO fabricated default"
        );
    }

    // (b) producer fires once with V; establish the held value.
    establish_held(&mut rt, &last_read, V);

    // (c) MANY silent steps: producer silent, consumer fires every step → the
    // snapshot REPLAYS the held value on EVERY step. Without the hold,
    // each of these steps would drain Empty → no-op → MISSING.
    let replays = replay_window(&mut rt, &last_read, HORIZON);

    // HAND ORACLE (anti-tautology): exactly HORIZON values, all == V.
    assert_eq!(
        replays,
        vec![V; HORIZON],
        "the held value {V} must replay on every one of the {HORIZON} silent \
         steps (got {replays:?})"
    );
}

// ===========================================================================
// Test 2 — the WAIT gate: a never-delivered non-trigger input never fabricates
// a default.
// ===========================================================================

#[test]
#[serial]
fn pre_first_delivery_consumer_waits_no_default() {
    const STEPS: usize = 15;
    let (mut rt, last_read) = build_hold_graph("nthwait", 42.0);

    // Producer NEVER fires; consumer fires every step.
    let mut observed = Vec::with_capacity(STEPS);
    for _ in 0..STEPS {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        observed.push(last_read.load(Ordering::Relaxed));
    }

    // HAND ORACLE: every step is MISSING — the tick NEVER ran (no delivery → no
    // held → snapshot stays Empty → macro no-ops the whole tick).
    assert_eq!(
        observed,
        vec![MISSING; STEPS],
        "a never-delivered non-trigger input must keep the consumer WAITING on \
         every step — nothing ever recorded (got {observed:?})"
    );
    // Decisive no-default pin: a fabricated Vector3 default (.x == 0.0 → 0u64)
    // would record 0, not MISSING — assert it NEVER appears.
    assert!(
        !observed.contains(&0),
        "a fabricated default (Vector3.x == 0.0) appeared — the consumer must \
         WAIT, never synthesize a zero value (got {observed:?})"
    );
}

// ===========================================================================
// Test 3 — the held value survives a multi-sample burst drain (peak 3 borrowed).
// ===========================================================================

#[test]
#[serial]
fn held_value_survives_multi_sample_burst() {
    // IncProducer publishes 1, 2, 3, ... on each external fire. Establish held=1,
    // accumulate a burst (2,3,4,5) WITHOUT firing the consumer, then fire it
    // ONCE: the snapshot drains the burst WHILE held=1 is still borrowed, so the
    // per-connection peak is held(1)+latest(1)+receive-transient(1) = 3.
    //
    // Reverting subscriber_max_borrowed_samples=3 to the
    // iceoryx2 default 2 makes this FAIL — the 2nd receive in the burst drain
    // would ExceedsMaxBorrows → the drain errors → FrozenSlot::Err → the
    // consumer's try_view returns Err → the tick propagates it (no store) →
    // last_read stays MISSING. With the =3 provisioning the drain succeeds and
    // serves the newest frame.
    const BURST: usize = 4; // producer publishes 2, 3, 4, 5
    const LATEST: u64 = 5; // the newest of the burst
    let (mut rt, last_read) = build_inc_graph("nthburst");

    // Establish held = V1 = 1.
    establish_held(&mut rt, &last_read, 1);

    // Burst-accumulate: producer publishes 2..=5 WITHOUT the consumer draining.
    for _ in 0..BURST {
        rt.trigger_external("producer").expect("trigger producer");
        rt.step(STEP);
    }

    // Burst-fire: the consumer's snapshot drains the whole burst (held=1 alive).
    last_read.store(MISSING, Ordering::Relaxed);
    rt.trigger_external("consumer").expect("trigger consumer");
    rt.step(STEP);
    let read = last_read.load(Ordering::Relaxed);

    // HAND ORACLE: the burst drain did NOT error (read != MISSING) and serves the
    // NEWEST frame of the burst (5). With the borrow floor at the default 2, read == MISSING.
    assert_ne!(
        read, MISSING,
        "the burst drain must NOT error — with subscriber_max_borrowed_samples=3 \
         the held(1)+latest(1)+transient(1) peak fits; a MISSING here means the \
         drain hit ExceedsMaxBorrows (the default-2 regression)"
    );
    assert_eq!(
        read, LATEST,
        "the burst snapshot must serve the NEWEST frame of the burst ({LATEST}), \
         got {read}"
    );
}

// ===========================================================================
// Test 4 — the hold replay is deterministic AND matches the hand oracle.
// ===========================================================================

#[test]
#[serial]
fn hold_is_deterministic_matches_oracle() {
    const V: u64 = 42;
    const HORIZON: usize = 20;
    let oracle = vec![V; HORIZON];

    // Two independent runs, fresh prefixes (build_for_test → per-test SHM root).
    let (mut rt_a, lr_a) = build_hold_graph("nthdet1", V as f64);
    establish_held(&mut rt_a, &lr_a, V);
    let run_a = replay_window(&mut rt_a, &lr_a, HORIZON);

    let (mut rt_b, lr_b) = build_hold_graph("nthdet2", V as f64);
    establish_held(&mut rt_b, &lr_b, V);
    let run_b = replay_window(&mut rt_b, &lr_b, HORIZON);

    // The two runs are byte-identical (determinism, Principle #7) ...
    assert_eq!(
        run_a, run_b,
        "the held-replay sequence must be identical across runs (a={run_a:?} \
         b={run_b:?})"
    );
    // ... AND both equal the HAND ORACLE. Equality of two runs alone is a
    // tautology — it passes even if BOTH were wrong; the literal oracle catches
    // a both-wrong bug.
    assert_eq!(
        run_a, oracle,
        "run A must match the hand oracle {oracle:?}, got {run_a:?}"
    );
    assert_eq!(
        run_b, oracle,
        "run B must match the hand oracle {oracle:?}, got {run_b:?}"
    );
}

// ===========================================================================
// Test 5 — the held-replay path is zero-alloc (CountingAllocator probe),
// DIRECT-subscriber drive.
//
// This drives `snapshot_latest` + `try_view` DIRECTLY on a raw subscriber
// (mirroring `snapshot_view_iox2_test`), NOT through `GraphRuntime::step()`.
// The direct drive deliberately ISOLATES the subscriber-level hold change —
// `held_sample` retention + `snapshot_latest`'s Empty→Held arm + `try_view`'s
// Held serve (via `SampleHandle::InboundRef`) — independently of the level
// executor: it pins exactly the subscriber-level hold, with no executor in
// the measured window. The FULL `GraphRuntime::step()` path is covered by the
// companion `hold_replay_through_executor_is_zero_alloc` below — in a
// consumer-only-fire replay window only ONE node is decided to fire per step, so
// the level executor's single-fire fast path runs and step() stays zero-alloc
// too (`tick_decided_parallel`'s HashMap is only reached with ≥2
// firing nodes on a level, which a 1-fire replay window never has). The
// subscriber is given a production-fidelity `drop_oldest` probe (graph
// non-trigger inputs always get one), so this ALSO proves the probe's Empty-drain
// accounting path is zero-alloc.
// ===========================================================================

/// Drive ONE delivery (establishing the held value), then count heap
/// allocations over `replays` held-replay `snapshot_latest()` + `try_view()`
/// cycles with the publisher SILENT (each cycle drains Empty → replays Held).
/// Returns (allocs, replays_that_served_42).
fn measure_hold_replay_allocs(replays: usize) -> (u64, usize) {
    use cerulion_core::scheduler::BackpressureCounters;
    use cerulion_core::transport::TransportManager;
    use cerulion_core::wire::MaxSliceLen;

    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/non_trigger_hold/alloc/{nanos}/{id}");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    // Production fidelity: graph non-trigger inputs are wired `drop_oldest`.
    let counters = Arc::new(BackpressureCounters::new());
    subscriber.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("hold_node"),
        Arc::from("inp"),
        16,
    );

    // FIRST delivery (42) → establish the held value via the snapshot path.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 42.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    // Let iceoryx2 surface the published sample on the subscriber side.
    std::thread::sleep(Duration::from_millis(50));
    subscriber.snapshot_latest();
    let established = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("establish try_view")
        .expect("first delivery serves a sample");
    assert_eq!(established, 42.0, "first delivery must establish held=42");

    // Warm: a few held replays (publisher SILENT) so any lazy capacity is grown
    // before the measured window. Each cycle: Empty drain → Held serve.
    for _ in 0..50 {
        subscriber.snapshot_latest();
        let _ = subscriber
            .try_view::<Vector3, _>(|view| view.x)
            .expect("warm replay try_view");
    }

    // MEASURE: `replays` held-replay cycles, publisher silent (Empty→Held).
    ALLOCATOR.enable();
    let mut served_42 = 0usize;
    for _ in 0..replays {
        subscriber.snapshot_latest();
        let r = subscriber
            .try_view::<Vector3, _>(|view| view.x)
            .expect("replay try_view");
        if r == Some(42.0) {
            served_42 += 1;
        }
    }
    let allocs = ALLOCATOR.disable();
    (allocs, served_42)
}

#[test]
#[serial]
fn hold_replay_is_zero_alloc() {
    const REPLAYS: usize = 200;
    let (allocs, served_42) = measure_hold_replay_allocs(REPLAYS);
    let per_replay = allocs as f64 / REPLAYS as f64;
    println!(
        "hold_replay_zero_alloc: {allocs} heap allocations over {REPLAYS} held-replay \
         snapshot_latest+try_view cycles ({per_replay:.3} allocs/replay), \
         {served_42} served the held value"
    );

    // Non-vacuous: every replay actually served the held value (a vacuous
    // zero-alloc where nothing replayed would be a bug).
    assert_eq!(
        served_42, REPLAYS,
        "every held replay must serve the held value 42 (served {served_42} of \
         {REPLAYS}) — otherwise the measured window did not exercise the hold path"
    );

    // The held-replay path must allocate ZERO times — the Empty drain
    // early-outs (scratch empty) and `try_view`'s Held arm serves the retained
    // SHM handle via `build_inbound_view` (no heap). (Debug OR release: an alloc
    // is an alloc.)
    assert_eq!(
        allocs, 0,
        "the held-replay path allocated {allocs} times over {REPLAYS} cycles \
         ({per_replay:.3} allocs/replay) — it must be zero-alloc (Empty drain \
         early-out + held-handle serve, no heap)"
    );
}

// ===========================================================================
// Additional node types + builders for the
// scenarios below. Each test asserts a HAND-WRITTEN literal oracle.
// ===========================================================================

/// THE motivating cross-step-hold use case: a DATA-TRIGGER consumer with a
/// trigger input `trig` AND a non-trigger latest-value context input `ctx`
/// (think a data-triggered controller whose `ctx` is a 1 Hz `/map`). The
/// DataTrigger policy snapshots `ctx` (held) but NOT `trig` (trigger drained).
/// Records `ctx.x` into a shared Arc; reads the trigger only to drain it.
#[cerulion_node]
#[derive(Default)]
struct TrigCtxConsumer {
    #[input(trigger)]
    trig: Vector3,
    #[input]
    ctx: Vector3,
    last_ctx: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl TrigCtxConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the trigger (frozen by the unified data-trigger drain — always
        // Some when this node fires, since it fired BECAUSE `trig` arrived), then
        // record the HELD context. If `ctx` is Empty (pre-delivery, no held), the
        // macro collapses this WHOLE tick to a no-op and this store never runs.
        let _ = self.trig.x;
        self.last_ctx.store(self.ctx.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// External consumer whose single non-trigger latest-value input also
/// carries an `expect_within_ms` watchdog. The held replay serves stale data;
/// the watchdog must STILL trip across a silent window (the safety net).
#[cerulion_node(external)]
#[derive(Default)]
struct HoldWatchConsumer {
    #[input(expect_within_ms = 5)]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl HoldWatchConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// External consumer whose single non-trigger latest-value input is
/// `sample(N)`-gated. A decimated step drains Empty → the hold replays the
/// last ACCEPTED value (not a no-op).
#[cerulion_node(external)]
#[derive(Default)]
struct SampleHoldConsumer {
    #[input(backpressure = sample(60000))]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SampleHoldConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// External consumer with TWO independent non-trigger latest-value
/// inputs, each recorded into its own Arc — pins per-input hold with no
/// cross-talk.
#[cerulion_node(external)]
#[derive(Default)]
struct TwoInputConsumer {
    #[input]
    a: Vector3,
    #[input]
    b: Vector3,
    last_a: Arc<AtomicU64>,
    last_b: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl TwoInputConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_a.store(self.a.x as u64, Ordering::Relaxed);
        self.last_b.store(self.b.x as u64, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// External consumer whose single non-trigger input is `block`. A
/// `block` input is EXCLUDED from the step-boundary snapshot (producer-paced) →
/// reads LIVE on the tick, NOT held. The negative pin for the doc claim.
#[cerulion_node(external)]
#[derive(Default)]
struct BlockHoldConsumer {
    #[input(backpressure = block, depth = 4)]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl BlockHoldConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `producer/out` → `consumer.inp` (single non-trigger input), with the
/// CONSUMER node supplied by the caller so each scenario plugs in its own
/// consumer type. Node ids are "producer"/"consumer" + input "inp", so the
/// existing [`establish_held`] / [`replay_window`] drivers work unchanged.
fn build_one_input_graph(
    prefix: &str,
    producer_type: &str,
    producer: Box<dyn NodeEntry>,
    consumer_type: &str,
    consumer: Box<dyn NodeEntry>,
    buffer: usize,
) -> GraphRuntime {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "non_trigger_hold".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: producer_type.to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: consumer_type.to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), producer);
    factories.insert("consumer".to_string(), consumer);
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, buffer).expect("build one-input graph")
}

// ===========================================================================
// The motivating use case: a DATA-TRIGGER consumer reads a SILENT
// non-trigger context. Phase (a): trigger fires, ctx never delivered → whole
// tick no-ops (MISSING) despite the trigger firing. Phase (b): ctx delivered
// once, then trigger fires while ctx is silent → reads the HELD ctx every step.
// ===========================================================================

#[test]
#[serial]
fn data_trigger_consumer_holds_silent_context() {
    const C: u64 = 99; // a clean, distinct ctx value (not 0, not MISSING)
    const PRE: usize = 6;
    const POST: usize = 12;

    let last_ctx = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "non_trigger_hold_trigctx".to_string(),
        prefix: "nthtrigctx".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "trig_producer".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "ctx_producer".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "trig_ctx_consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "trig".to_string(),
                        source: "trig_producer/out".to_string(),
                    },
                    InputDef {
                        name: "ctx".to_string(),
                        source: "ctx_producer/out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    // The trigger value is irrelevant (the consumer records ctx, not trig).
    let trig_producer = HoldProducer {
        val: 7.0,
        ..Default::default()
    };
    let ctx_producer = HoldProducer {
        val: C as f64,
        ..Default::default()
    };
    factories.insert(
        "trig_producer".to_string(),
        Box::new(HoldProducerEntry::with_state(trig_producer)),
    );
    factories.insert(
        "ctx_producer".to_string(),
        Box::new(HoldProducerEntry::with_state(ctx_producer)),
    );
    let consumer = TrigCtxConsumer {
        last_ctx: Arc::clone(&last_ctx),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(TrigCtxConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build trig-ctx graph");

    // (a) PRE-CTX: trigger fires every step (the data-trigger consumer fires)
    // but ctx is NEVER delivered → ctx snapshot stays Empty (no held) → the
    // macro no-ops the WHOLE tick → MISSING every step DESPITE the trigger.
    let mut phase_a = Vec::with_capacity(PRE);
    for _ in 0..PRE {
        last_ctx.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("trig_producer")
            .expect("trigger trig_producer");
        rt.step(STEP);
        phase_a.push(last_ctx.load(Ordering::Relaxed));
    }
    // HAND ORACLE: all MISSING. (Phase (b) below flips ONLY ctx-delivery and the
    // result flips to C — proving the trigger WAS driving fires here, so MISSING
    // is the ctx-Empty no-op, not an un-fired consumer.)
    assert_eq!(
        phase_a,
        vec![MISSING; PRE],
        "pre-ctx-delivery: a data-trigger consumer whose non-trigger context \
         never arrived must no-op every fire (got {phase_a:?})"
    );

    // Establish ctx held = C: fire BOTH producers + let the data-trigger fire
    // until the consumer reads C (bounded — tolerates iceoryx2 surfacing).
    let mut tries = 0;
    loop {
        last_ctx.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("trig_producer").expect("trigger trig");
        rt.trigger_external("ctx_producer").expect("trigger ctx");
        rt.step(STEP);
        if last_ctx.load(Ordering::Relaxed) == C {
            break;
        }
        tries += 1;
        assert!(tries < 200, "ctx held value {C} never established");
    }

    // (b) POST: fire ONLY the trigger (ctx_producer SILENT) → the consumer fires
    // each step and reads the HELD ctx C.
    let mut phase_b = Vec::with_capacity(POST);
    for _ in 0..POST {
        last_ctx.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("trig_producer").expect("trigger trig");
        rt.step(STEP);
        phase_b.push(last_ctx.load(Ordering::Relaxed));
    }
    // HAND ORACLE: all == C — the data-trigger consumer reads the held context
    // every step while its producer is silent (the core cross-step-hold scenario).
    assert_eq!(
        phase_b,
        vec![C; POST],
        "post-establish: the data-trigger consumer must read the HELD context {C} \
         on every step while ctx_producer is silent (got {phase_b:?})"
    );
}

// ===========================================================================
// The watchdog still trips under held replay: a held non-trigger input
// serves stale data, but its `expect_within_ms` miss is STILL counted (the
// silent-failure safety net).
// ===========================================================================

#[test]
#[serial]
fn held_replay_still_trips_expect_within_watchdog() {
    const V: u64 = 42;
    const HORIZON: usize = 60; // 60 × 1 ms ≫ the 5 ms watchdog window

    let last_read = Arc::new(AtomicU64::new(MISSING));
    let producer = HoldProducer {
        val: V as f64,
        ..Default::default()
    };
    let consumer = HoldWatchConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    let mut rt = build_one_input_graph(
        "nthwatch",
        "hold_producer",
        Box::new(HoldProducerEntry::with_state(producer)),
        "hold_watch_consumer",
        Box::new(HoldWatchConsumerEntry::with_state(consumer)),
        8,
    );

    establish_held(&mut rt, &last_read, V);

    // Producer STOPS; the consumer keeps firing, replaying the held value across
    // a silent window far longer than the 5 ms watchdog window.
    let replays = replay_window(&mut rt, &last_read, HORIZON);
    let handle = rt.node_handle("consumer").unwrap();
    let missed = handle.expect_within_missed_count();
    let backlogged = handle.expect_within_backlogged_count();

    // HAND ORACLE part 1: the held value is STILL served every step (the node
    // reads stale data — it does not starve).
    assert_eq!(
        replays,
        vec![V; HORIZON],
        "the held value {V} must still be served on every silent step (got {replays:?})"
    );
    // HAND ORACLE part 2: the watchdog COUNTED the staleness — a held replay is
    // NOT fresh data, so the silent window trips `expect_within_missed`. The
    // 60-step × 1 ms silent window against the 5 ms watchdog gives ~60/5 ≈ 12
    // misses, so `>= 10` unambiguously attributes the misses to the silent HOLD
    // window (a stray establish-phase miss could add at most 1).
    assert!(
        missed >= 10,
        "the expect_within watchdog must trip across the silent window even \
         though the held value is replayed (missed={missed}, expected >= 10 from \
         the ~60 ms / 5 ms window)"
    );
    // Boundary asserted rather than assumed: this consumer is an
    // `external`-policy node with a NON-trigger held input, so it carries no
    // per-message FIFO backlog and its staleness can never be reported as
    // one. An INVARIANT pin (the guard has no reachable path here), not
    // one a deletion fails — the held-context staleness detector is exactly
    // what the guard's `fifo_trigger` scoping exists to protect.
    assert_eq!(
        backlogged, 0,
        "a held NON-trigger input's staleness is silence, never backlog \
         (backlogged={backlogged})"
    );
}

// ===========================================================================
// Held UPDATES then re-holds at the NEW value (not stuck on the old,
// not Empty).
// ===========================================================================

#[test]
#[serial]
fn held_updates_then_reholds_at_new_value() {
    const N: usize = 10;
    const M: usize = 10;
    // IncProducer publishes 1, 2, 3, ... on each external fire.
    let (mut rt, last_read) = build_inc_graph("nthupdate");

    // Establish held = 1 (IncProducer's first publish), replay it.
    establish_held(&mut rt, &last_read, 1);
    let w1 = replay_window(&mut rt, &last_read, N);
    assert_eq!(
        w1,
        vec![1; N],
        "the first window must replay held=1 (got {w1:?})"
    );

    // Deliver a SINGLE new value (2) and re-establish the hold at it. Using
    // establish_held (fire producer once + drain via the consumer) accounts for
    // the 1-step snapshot lag on the new delivery.
    establish_held(&mut rt, &last_read, 2);
    let w2 = replay_window(&mut rt, &last_read, M);

    // HAND ORACLE: the second window is all 2 — the hold UPDATED to the new
    // value (not stuck on 1, not Empty).
    assert_eq!(
        w2,
        vec![2; M],
        "after delivering a new value the hold must UPDATE and re-hold at 2 — \
         not stay on the old 1, not go Empty (got {w2:?})"
    );
}

// ===========================================================================
// A transient drain Err does NOT discard the held value. Direct
// subscriber drive (mirrors snapshot_view_iox2_test::frozen_err_* + this file's
// measure_* setup) with the fire-once receive fault.
// ===========================================================================

#[test]
#[serial]
fn transient_err_does_not_discard_held_value() {
    use cerulion_core::scheduler::BackpressureCounters;
    use cerulion_core::transport::TransportManager;
    use cerulion_core::wire::MaxSliceLen;

    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/non_trigger_hold/err_survive/{nanos}/{id}");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    // Production fidelity: graph non-trigger inputs are wired `drop_oldest`.
    let counters = Arc::new(BackpressureCounters::new());
    subscriber.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("hold_node"),
        Arc::from("inp"),
        16,
    );

    // Establish held = 42.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 42.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    std::thread::sleep(Duration::from_millis(50));
    subscriber.snapshot_latest();
    let established = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("establish try_view")
        .expect("first delivery serves a sample");
    assert_eq!(established, 42.0, "first delivery must establish held=42");

    // Inject a fire-once receive error: the next snapshot drain errors on its
    // first receive (popping nothing) → FrozenSlot::Err; held_sample untouched.
    subscriber.fault_inject_receive_after_for_test(0);
    subscriber.snapshot_latest();
    let replayed = subscriber.try_view::<Vector3, _>(|view| view.x);
    assert!(
        replayed.is_err(),
        "the transient drain Err must be replayed once by try_view (got {replayed:?})"
    );

    // Publisher SILENT (no new sample). The next snapshot drains Empty → the
    // held value SURVIVED the transient Err and is replayed.
    subscriber.snapshot_latest();
    let served = subscriber
        .try_view::<Vector3, _>(|view| view.x)
        .expect("post-Err snapshot try_view must not error")
        .expect("the held value is still served after a transient Err");

    // HAND ORACLE: the held 42 survived the Err — not lost.
    assert_eq!(
        served, 42.0,
        "a transient drain Err must NOT discard the held value — post-Err \
         try_view must still serve 42 (got {served})"
    );
}

// ===========================================================================
// A `sample(N)` non-trigger input replays its last-ACCEPTED value on a
// decimated step (not a no-op, not the decimated value).
// ===========================================================================

#[test]
#[serial]
fn sample_non_trigger_replays_last_accepted_on_decimated_step() {
    // IncProducer (1, 2, 3, ...) → a sample(60s) non-trigger consumer. The first
    // delivery is ACCEPTED (gate first-accept); a second publish well inside the
    // window is DECIMATED → drain Empty → the hold replays last-accepted=1.
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let consumer = SampleHoldConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    let mut rt = build_one_input_graph(
        "nthsample",
        "inc_producer",
        Box::new(IncProducerEntry::new()),
        "sample_hold_consumer",
        Box::new(SampleHoldConsumerEntry::with_state(consumer)),
        8,
    );

    // Establish the first ACCEPTED value = 1.
    establish_held(&mut rt, &last_read, 1);

    // Publish a SECOND value (2): well inside the 60 s sample window → decimated.
    rt.trigger_external("producer").expect("trigger producer");
    rt.step(STEP);

    // Fire the consumer until the decimation is observed (bounded — tolerates
    // surfacing latency). On EVERY fire the consumer must read the held
    // last-accepted value (1) — never the decimated 2, never MISSING.
    let mut tries = 0;
    loop {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        let r = last_read.load(Ordering::Relaxed);
        assert_eq!(
            r, 1,
            "a sample(N) non-trigger input must REPLAY the last-accepted value \
             (1) on a decimated/silent step — never the decimated 2, never \
             MISSING (got {r})"
        );
        if rt
            .node_handle("consumer")
            .unwrap()
            .backpressure_sampled_count("inp")
            >= 1
        {
            break;
        }
        tries += 1;
        assert!(tries < 200, "the second publish was never decimated");
    }

    // HAND ORACLE: at least one decimation occurred (the loop above pinned that
    // the held last-accepted value 1 was served throughout).
    let sampled = rt
        .node_handle("consumer")
        .unwrap()
        .backpressure_sampled_count("inp");
    assert!(
        sampled >= 1,
        "the second publish must have been decimated by the sample gate (got {sampled})"
    );
}

// ===========================================================================
// The held replay is zero-alloc THROUGH the level executor (the full
// GraphRuntime::step() path; complements the direct-subscriber Test 5).
// ===========================================================================

#[test]
#[serial]
fn hold_replay_through_executor_is_zero_alloc() {
    const V: u64 = 42;
    // Warm MORE fire-steps than MEASURE so the execution-trace VecDeque grows its
    // capacity past the measured window's push count BEFORE clear_trace (which
    // keeps capacity). Otherwise the measured window's TraceEntry pushes would
    // realloc at capacity doublings (e.g. 64→128→256) and masquerade as a
    // per-step executor allocation (mirrors step_zero_alloc_test, which warms
    // ~750 fires for the same reason).
    const WARM: usize = 400;
    const MEASURE: usize = 200;

    let (mut rt, last_read) = build_hold_graph("nthexecalloc", V as f64);
    // Isolate the measurement from the periodic liveliness sweep — an ORTHOGONAL
    // per-250ms background task whose dead-node reclamation
    // (`LivelinessCleaner::cleanup_dead_nodes` → iceoryx2 `try_cleanup_dead_nodes`)
    // allocates a large, fixed amount per sweep, entirely unrelated to the
    // held-replay path (step_zero_alloc_test dodges the same sweep only by
    // clock-phase luck — its 750-950 ms measure window misses every 250 ms
    // boundary). Pushing the period far beyond this test's ~600 ms clock horizon
    // (1 ms/step × establish + WARM=400 + MEASURE=200) makes the isolation
    // explicit (same principle as `clear_trace` isolating trace-buffer growth).
    rt.set_liveliness_sweep_period_for_test(10_000_000);
    establish_held(&mut rt, &last_read, V);

    // Warm: a consumer-only-fire replay window so the SHM pool, IndexMaps, and
    // the execution-trace VecDeque capacity are all grown before measuring.
    let warm = replay_window(&mut rt, &last_read, WARM);
    assert_eq!(
        warm,
        vec![V; WARM],
        "warm replay must serve the held value (got {warm:?})"
    );

    // Reset the trace LENGTH (keeps the grown capacity) so trace-buffer growth
    // does not masquerade as a per-step executor allocation (mirrors
    // step_zero_alloc_test).
    rt.clear_trace();

    // MEASURE: a consumer-only-fire replay window through GraphRuntime::step().
    // Only the consumer is decided to fire each step (the producer is silent),
    // so the level executor's single-fire fast path runs (no per-level HashMap)
    // and the snapshot replay (Empty drain early-out + held-handle
    // serve) adds no heap.
    let mut served = 0usize;
    ALLOCATOR.enable();
    for _ in 0..MEASURE {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        if last_read.load(Ordering::Relaxed) == V {
            served += 1;
        }
    }
    let allocs = ALLOCATOR.disable();
    let per_step = allocs as f64 / MEASURE as f64;
    println!(
        "hold_replay_through_executor_zero_alloc: {allocs} heap allocations over \
         {MEASURE} consumer-only-fire GraphRuntime::step() replays \
         ({per_step:.3} allocs/step), {served} served the held value"
    );

    // Non-vacuous: every measured step actually served the held value.
    assert_eq!(
        served, MEASURE,
        "every measured step must serve the held value {V} through step() \
         (served {served} of {MEASURE})"
    );
    // The full step() held-replay path must allocate ZERO times.
    assert_eq!(
        allocs, 0,
        "GraphRuntime::step() held-replay allocated {allocs} times over {MEASURE} \
         steps ({per_step:.3} allocs/step) — a consumer-only-fire replay window \
         is a single decided fire per step (executor fast path) + a zero-alloc \
         held snapshot replay, so it must be zero-alloc"
    );

    // --- Sanity control (the phase-3 pattern): the thread-scoped
    // counter still BITES. Kills the "thread-scoping accidentally made the
    // counter count nothing" false-green: one deliberate heap allocation on
    // the measured thread must read back as EXACTLY 1, and a fresh empty
    // window as 0.
    ALLOCATOR.enable();
    let v: Vec<u8> = Vec::with_capacity(1);
    std::hint::black_box(v);
    let bite = ALLOCATOR.disable();
    assert_eq!(
        bite, 1,
        "a deliberate measured-thread allocation must count exactly 1 (got {bite})"
    );
    ALLOCATOR.enable();
    let empty = ALLOCATOR.disable();
    assert_eq!(empty, 0, "an empty fresh window must count 0 (got {empty})");
}

// ===========================================================================
// Two non-trigger inputs hold INDEPENDENTLY (no cross-talk).
// ===========================================================================

#[test]
#[serial]
fn two_non_trigger_inputs_hold_independently() {
    const A: u64 = 11;
    const B: u64 = 22;
    const N: usize = 15;

    let last_a = Arc::new(AtomicU64::new(MISSING));
    let last_b = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "non_trigger_hold_two".to_string(),
        prefix: "nthtwo".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod_a".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod_b".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "two_input_consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "a".to_string(),
                        source: "prod_a/out".to_string(),
                    },
                    InputDef {
                        name: "b".to_string(),
                        source: "prod_b/out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod_a".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer {
            val: A as f64,
            ..Default::default()
        })),
    );
    factories.insert(
        "prod_b".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer {
            val: B as f64,
            ..Default::default()
        })),
    );
    let consumer = TwoInputConsumer {
        last_a: Arc::clone(&last_a),
        last_b: Arc::clone(&last_b),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(TwoInputConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build two-input graph");

    // Establish both held values: fire both producers + the consumer until the
    // tick runs (the no-op gate passes only when BOTH inputs are Some) and
    // records A and B.
    let mut tries = 0;
    loop {
        last_a.store(MISSING, Ordering::Relaxed);
        last_b.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("prod_a").expect("trigger a");
        rt.trigger_external("prod_b").expect("trigger b");
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        if last_a.load(Ordering::Relaxed) == A && last_b.load(Ordering::Relaxed) == B {
            break;
        }
        tries += 1;
        assert!(tries < 200, "both held values never established");
    }

    // Both producers SILENT; fire the consumer N steps → each input replays its
    // OWN held value independently.
    let mut wa = Vec::with_capacity(N);
    let mut wb = Vec::with_capacity(N);
    for _ in 0..N {
        last_a.store(MISSING, Ordering::Relaxed);
        last_b.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        wa.push(last_a.load(Ordering::Relaxed));
        wb.push(last_b.load(Ordering::Relaxed));
    }
    // HAND ORACLE: [A; N] and [B; N] — each input holds its own value, no
    // cross-talk (a would-be cross-talk bug would record A into b or vice versa).
    assert_eq!(
        wa,
        vec![A; N],
        "input `a` must replay its OWN held value {A} every step (got {wa:?})"
    );
    assert_eq!(
        wb,
        vec![B; N],
        "input `b` must replay its OWN held value {B} every step (got {wb:?})"
    );
}

// ===========================================================================
// Negative pin: a `block` non-trigger input reads LIVE (producer-paced),
// NOT held. On a silent step it no-ops (MISSING), as an input with no hold does.
// ===========================================================================

#[test]
#[serial]
fn block_non_trigger_reads_live_not_held() {
    const V: u64 = 33;

    let last_read = Arc::new(AtomicU64::new(MISSING));
    let consumer = BlockHoldConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    let mut rt = build_one_input_graph(
        "nthblock",
        "hold_producer",
        Box::new(HoldProducerEntry::with_state(HoldProducer {
            val: V as f64,
            ..Default::default()
        })),
        "block_hold_consumer",
        Box::new(BlockHoldConsumerEntry::with_state(consumer)),
        8,
    );

    // Establish: publish V once, then fire the consumer until it reads V LIVE
    // (bounded — tolerates surfacing latency).
    rt.trigger_external("producer").expect("trigger producer");
    rt.step(STEP);
    let mut tries = 0;
    loop {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        if last_read.load(Ordering::Relaxed) == V {
            break;
        }
        tries += 1;
        assert!(tries < 200, "block consumer never read the live value {V}");
    }

    // Silent steps: producer NOT fired. A HELD input would replay V; a `block`
    // input reads LIVE → drain Empty → no-op → MISSING.
    let mut observed = Vec::with_capacity(5);
    for _ in 0..5 {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        observed.push(last_read.load(Ordering::Relaxed));
    }
    // HAND ORACLE: all MISSING — the block input is EXCLUDED from the snapshot,
    // so it never replays a held value (the negative pin for the doc claim).
    assert_eq!(
        observed,
        vec![MISSING; 5],
        "a block non-trigger input must NOT replay a held value on a silent step \
         — it reads live and no-ops, as an input with no hold does (got {observed:?})"
    );
}

// ===========================================================================
// Drift-guard — iceoryx2's default subscriber_max_borrowed_samples is 2.
// SUBSCRIBER_MAX_BORROWED_HELD is budgeted as that default + 1; an upstream
// iceoryx2 default change would silently break the held(+1) budget.
// ===========================================================================

#[test]
#[serial]
fn iceoryx2_default_borrowed_samples_is_two() {
    use cerulion_core::transport::TransportManager;

    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    let topic = format!("test/non_trigger_hold/borrow_default/{nanos}/{id}");

    // Read the ACTUAL borrowed-sample default off a freshly-created default
    // service (no override). HAND ORACLE: 2 (= ICEORYX2_DEFAULT_BORROWED_SAMPLES).
    let actual = mgr.default_subscriber_max_borrowed_samples_for_test(&topic);
    assert_eq!(
        actual, 2,
        "iceoryx2's default subscriber_max_borrowed_samples must be 2 — \
         SUBSCRIBER_MAX_BORROWED_HELD is defined as this default + 1, so an \
         upstream change here silently breaks the held(+1)=3 borrow budget \
         (got {actual})"
    );
}

// ===========================================================================
// EXTERNAL/absolute snapshot source — the held-context provisioning + the loud
// build-fail arm over a PRODUCER-LESS absolute `/ext/...` source.
//
// A MACRO snapshot consumer (HoldConsumer is `external` + a plain non-trigger
// `#[input]` → performs_input_snapshot) reading an absolute `/ext/...` source
// with NO in-graph producer takes the `PublisherProvisioning::External` arm,
// yet still lands in `snapshot_source_topics`, so the runtime's
// post-pass raises that topic to `subscriber_max_borrowed_samples = 3`.
//
//  - Happy path: no foreign service exists, so the consumer's body subscriber
//    CREATES the external service at borrow=3 and the graph builds; an external
//    publisher attaches, data flows, and the held value replays across a silent
//    window (hand oracle).
//  - Loud-failure path: a foreign raw publisher pre-creates the service at the
//    iceoryx2 default (borrow=2, transport-default ceiling 16 == the graph's
//    requirement), so the ONLY unmet requirement is borrow=3 → the build FAILS
//    with `DoesNotSupportRequestedMinSubscriberBorrowedSamples`, mapped to the
//    actionable hint naming `subscriber_max_borrowed_samples >= 3`.
//
// These use the explicit `init_for_test` manager + `GraphRuntime::build` pattern
// (NOT `build_for_test`) — mirroring `absolute_source_external_iox2_test` — so
// the loud-fail arm can pre-create the foreign service BEFORE the graph builds.
// Each test gets an isolated per-test SHM root (`init_for_test`) + a unique
// absolute topic; `#[serial]` for belt-and-suspenders against the iceoryx2 SHM
// singleton.
// ===========================================================================

/// An isolated test transport with a 16-deep subscriber buffer. The graph's
/// External topic ceiling is `max(DEFAULT_CONSUMER_DEPTH = 10, 16) = 16`, which
/// equals a foreign default publisher's ceiling — so the loud-fail arm's only
/// unmet requirement is the borrow=3, never the ceiling.
fn ext_test_manager(node_name: &str) -> Arc<cerulion_core::transport::TransportManager> {
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let transport_config = cerulion_core::transport::TransportConfig {
        node_name: node_name.into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 16,
        network: None,
    };
    cerulion_core::transport::TransportManager::init_for_test(transport_config, ix_config)
        .expect("init external-source test transport")
}

/// A single `external` HoldConsumer whose ONLY input is a non-trigger
/// latest-value read of an ABSOLUTE `/ext/...` source (no in-graph producer).
fn build_external_snapshot_graph(
    prefix: &str,
    abs_topic: &str,
    last_read: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "non_trigger_hold_ext".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "consumer".to_string(),
            node_type: "hold_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: abs_topic.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let consumer = HoldConsumer {
        last_read,
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(HoldConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Build a unique absolute external topic name (per-test SHM root keeps these
/// isolated, but a unique name avoids any same-process re-run collision).
fn unique_ext_topic() -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("/ext/held/{nanos}/{id}")
}

#[test]
#[serial]
fn external_absolute_snapshot_source_provisions_borrow3_and_holds() {
    use cerulion_core::wire::MaxSliceLen;

    const V: u64 = 77;
    const HORIZON: usize = 12;

    let abs_topic = unique_ext_topic();
    let mgr = ext_test_manager("nth_ext_happy");
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) =
        build_external_snapshot_graph("nthexth", &abs_topic, Arc::clone(&last_read));
    let clock = Arc::new(VirtualClock::new());

    // The graph builds: the absolute external source has NO in-graph producer
    // (External provisioning), yet the macro snapshot consumer puts it in
    // `snapshot_source_topics`, so the consumer's body subscriber CREATES the
    // service at subscriber_max_borrowed_samples=3 (no foreign creator exists).
    let mut rt = GraphRuntime::build(config, factories, &mgr, clock).expect(
        "a graph reading an absolute external snapshot source must build — the \
         consumer's body subscriber creates the service at borrow=3",
    );

    // An external publisher attaches to the now-existing service and publishes V
    // ONCE; it then stays SILENT for the rest of the test.
    let mut pubr = mgr
        .create_publisher(&abs_topic, MaxSliceLen::const_new(256), 0)
        .expect("external publisher attaches to the graph-created service");
    {
        let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
        proxy.x = V as f64;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }

    // Establish the held value: fire ONLY the consumer (publisher silent now)
    // until it drains V (bounded — tolerates iceoryx2 surfacing latency).
    let mut tries = 0;
    loop {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        if last_read.load(Ordering::Relaxed) == V {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "external held value {V} never established within 200 consumer fires"
        );
    }

    // Silent window: publisher silent, consumer fires every step → REPLAYS the
    // held EXTERNAL value on every step.
    let mut observed = Vec::with_capacity(HORIZON);
    for _ in 0..HORIZON {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        observed.push(last_read.load(Ordering::Relaxed));
    }
    // HAND ORACLE: the held value from an ABSOLUTE external source replays on
    // every one of the HORIZON silent steps (the held-context provisioning works
    // identically for an external source, not just an in-graph producer).
    assert_eq!(
        observed,
        vec![V; HORIZON],
        "the held value {V} from an ABSOLUTE external source must replay on every \
         one of the {HORIZON} silent steps (got {observed:?})"
    );
}

#[test]
#[serial]
fn external_absolute_snapshot_source_at_borrow2_fails_loudly() {
    use cerulion_core::wire::MaxSliceLen;

    let abs_topic = unique_ext_topic();
    let mgr = ext_test_manager("nth_ext_loud");

    // A FOREIGN raw publisher creates the service FIRST at the iceoryx2 default
    // borrowed-samples (2) and the transport-default ceiling (16). The ceiling
    // matches the graph's requirement (max(depth 10, 16) = 16), so the ONLY
    // requirement the foreign service fails to satisfy is the borrow=3.
    let _foreign = mgr
        .create_publisher(&abs_topic, MaxSliceLen::const_new(256), 0)
        .expect("foreign publisher creates the service at the default borrow=2");

    let last_read = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) = build_external_snapshot_graph("nthexlf", &abs_topic, last_read);
    let clock = Arc::new(VirtualClock::new());

    // Building the snapshot-consumer graph (which requires borrow=3) must FAIL
    // LOUDLY at build against the borrow-2 foreign service.
    let msg = match GraphRuntime::build(config, factories, &mgr, clock) {
        Ok(_) => panic!(
            "a borrow-3 snapshot consumer must NOT attach to a foreign borrow-2 \
             service — the build must fail loudly"
        ),
        Err(e) => e.to_string(),
    };
    // HAND ORACLE: the failure names the actionable borrow requirement
    // (`subscriber_max_borrowed_samples >= 3` — the mod.rs open-error hint), not
    // an at-a-distance generic iceoryx2 variant name.
    assert!(
        msg.contains("subscriber_max_borrowed_samples >= 3"),
        "the build failure must name the actionable borrow-ceiling hint \
         ('subscriber_max_borrowed_samples >= 3'), got: {msg}"
    );
}

// ===========================================================================
// Multi-process provisioning UNION. A `process_groups:` split
// builds each group's subgraph in its own process, so the producer-owning worker
// only sees ITS group's consumers when it creates a topic's iceoryx2 service — a
// snapshot consumer in ANOTHER group is invisible and the service is
// under-provisioned (`subscriber_max_borrowed_samples` stays at the iceoryx2
// default 2), refusing that consumer at open. The rule: the supervisor HARVESTS
// the FULL-graph monolith's per-topic requirements
// (`GraphRuntime::topic_requirements`) and stamps them into every WorkerPlan; the
// producer-owning worker provisions `max(local view, stamped union)`
// (`build_live_deterministic_with_manager_and_barrier`'s `topic_requirements`
// arg → `TopicRequirements::union_into`).
//
// These use `init_for_test` managers + a controlled `VirtualClock` (the barrier
// build path enforces a clock contract) + a unique per-test SHM root; `#[serial]`
// for the iceoryx2 SHM singleton.
// ===========================================================================

/// An isolated test transport whose transport CLOCK is the caller's `clock` — the
/// producer build path (`build_live_deterministic_with_manager_and_barrier`)
/// enforces `Arc::ptr_eq(transport.clock, build clock)`, so the manager must be
/// built with the SAME clock instance we hand the build. 16-deep buffer (matches
/// `ext_test_manager`).
fn ext_test_manager_with_clock(
    node_name: &str,
    clock: Arc<VirtualClock>,
) -> Arc<cerulion_core::transport::TransportManager> {
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let transport_config = cerulion_core::transport::TransportConfig {
        node_name: node_name.into(),
        clock,
        subscriber_buffer_size: 16,
        network: None,
    };
    cerulion_core::transport::TransportManager::init_for_test(transport_config, ix_config)
        .expect("init harvest test transport")
}

/// A single-node PRODUCER graph owning one output topic `/{prefix}/producer/out`
/// (no consumers — the cross-group consumer lives in another "process"). Built
/// via the multi-process worker path so the `topic_requirements` override
/// is threaded in.
fn build_producer_via_worker_path(
    mgr: &cerulion_core::transport::TransportManager,
    clock: Arc<VirtualClock>,
    prefix: &str,
    barrier_ns: &str,
    overrides: Option<&std::collections::BTreeMap<String, cerulion_core::TopicRequirements>>,
) -> GraphRuntime {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_prod"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "producer".to_string(),
            node_type: "hold_producer".to_string(),
            inputs: vec![],
            outputs: vec![vec3_out("out")],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer::default())),
    );
    // expected = 1: a single participant (this is a provisioning test, not a
    // multi-context rendezvous test — the runtime is never stepped).
    let barrier =
        Arc::new(MappedBarrier::create_owned(barrier_ns, "g", 1).expect("barrier owner create"));
    GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        mgr,
        clock,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        vec![Some(0)],
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the law every generation pin here asserts.
        vec![false; 1], // one node → one global level
        0,
        Duration::from_millis(1),
        CrossProcessWiring::requirements_only(overrides),
    )
    .expect("build producer via the multi-process worker path")
}

/// HARVEST correctness (+ monolith parity). A FULL-graph monolith build
/// with an in-graph producer→snapshot-consumer edge must report, via
/// `GraphRuntime::topic_requirements()`, the SAME provisioning it (pre-)created
/// the service with: `min_borrowed_samples == 3` (the snapshot post-pass)
/// and the introspection-headroom'd subscriber count. This is the map the
/// supervisor harvests off its planning build; asserting it against a HAND oracle
/// (not a self-compare) pins that the harvested numbers cannot drift from what
/// the monolith applies.
#[test]
#[serial]
fn topic_requirements_harvest_reports_snapshot_borrow3() {
    let (rt, _last_read) = build_hold_graph("nthtrh", 42.0);
    let reqs = rt.topic_requirements();

    // Exactly one OWNED topic (the producer's output); the harvest excludes any
    // External (unowned) topic.
    let topic = "/nthtrh/producer/out";
    assert!(
        reqs.contains_key(topic),
        "harvest must contain the owned producer topic '{topic}', got keys {:?}",
        reqs.keys().collect::<Vec<_>>()
    );
    let req = &reqs[topic];
    // HEADLINE: the snapshot consumer (HoldConsumer, holds its non-trigger input)
    // makes this a snapshot-source topic → borrow 3. This IS the value the
    // monolith provisions (the existing borrow-3 tests above prove that), so this
    // is the monolith==harvest parity pin.
    assert_eq!(
        req.min_borrowed_samples, 3,
        "a snapshot-source topic must harvest min_borrowed_samples == 3"
    );
    // Subscriber count = 1 body subscriber (the lone consumer) + the
    // introspection headroom. HoldConsumer is `external` policy → no data-trigger
    // drain subscriber, so in_graph_subs == 1.
    assert_eq!(
        req.min_subscribers,
        1 + cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM,
        "min_subscribers must be the monolith's (in-graph subs + introspection headroom)"
    );
    // Buffer ceiling = max(consumer depth, transport default) >= the default.
    assert!(
        req.min_buffer >= 8,
        "min_buffer must be at least the default buffer, got {}",
        req.min_buffer
    );
    rt.shutdown();
}

/// CONSUME (the rule, end-to-end): the producer-owning worker provisions
/// the STAMPED cross-group requirement into the service it CREATES.
///
/// POSITIVE arm: stamping borrow 3 and asserting the consumer opens
/// would be TAUTOLOGICAL — the create-side
/// floor also creates every owned topic at 3 (the consumer would open with or
/// without the stamp; only the negative arm below would distinguish them). The pin
/// therefore stamps a borrow requirement STRICTLY ABOVE the floor
/// (5 = SUBSCRIBER_MAX_BORROWED_HELD + 2) and asserts the created service's
/// static config carries 5: the floor alone yields 3, so ==5 is attributable
/// ONLY to the stamp machinery (`union_into` — an unclamped field-wise `max`
/// over the hand-built `TopicRequirements`). The consumer-opens assertion is
/// kept as the end-to-end harvest story (its borrow-3 requirement opens the
/// borrow-5 service via iceoryx2's at-least semantics) — but the load-bearing
/// oracle is the above-floor 5.
#[test]
#[serial]
fn stamped_borrow_requirement_raises_created_service_above_the_borrow_floor() {
    let prefix = "pos";
    let topic = "/pos/producer/out";
    let clock = Arc::new(VirtualClock::new());
    let mgr = ext_test_manager_with_clock("pos", Arc::clone(&clock));

    // The supervisor's harvested stamp — borrow 5, STRICTLY above the create-side
    // floor (3 == SUBSCRIBER_MAX_BORROWED_HELD; pub(crate), literal here), so
    // the created value below cannot be explained by the floor.
    let mut overrides: std::collections::BTreeMap<String, cerulion_core::TopicRequirements> =
        std::collections::BTreeMap::new();
    overrides.insert(
        topic.to_string(),
        cerulion_core::TopicRequirements {
            min_borrowed_samples: 5,
            min_buffer: 16,
            min_subscribers: 5,
            min_event_listeners: 0,
        },
    );

    // The producer worker creates the owned service, provisioning the UNION.
    let producer_rt = build_producer_via_worker_path(
        &mgr,
        Arc::clone(&clock),
        prefix,
        "pos_bar",
        Some(&overrides),
    );

    // LOAD-BEARING oracle: the created service's borrow ceiling is 5 (read off
    // the iceoryx2 static config via the drift-guard's accessor). The create-side
    // floor alone creates at 3 — the negative-arm test below pins exactly that
    // — so 5 here proves the STAMP reached the created service.
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(topic),
        5,
        "the producer worker must create the owned service at the stamped borrow=5 \
         (above the create-side floor 3 — only the stamp can explain this value)"
    );

    // E2E signal (kept, not load-bearing): a snapshot-consumer graph in another
    // "process" reads the same absolute topic; its borrow-3 body subscriber
    // OPENS the borrow-5 service (iceoryx2 at-least semantics) and BUILDS —
    // the cross-group shape, end-to-end.
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let (cons_cfg, cons_fac) = build_external_snapshot_graph("posc", topic, last_read);
    let cons_clock = Arc::new(VirtualClock::new());
    let consumer_rt = GraphRuntime::build(cons_cfg, cons_fac, &mgr, cons_clock).expect(
        "a cross-group snapshot consumer MUST open the producer-provisioned borrow-5 service",
    );

    consumer_rt.shutdown();
    producer_rt.shutdown();
}

/// CONSUME, the no-stamp arm: the borrow axis of the cross-group stamp is
/// REDUNDANT (its buffer/subscriber axes remain load-bearing). WITHOUT any stamp,
/// the producer worker's owned topic is CREATED at the borrow FLOOR
/// (create-side generosity — never an open requirement), so the cross-group
/// snapshot consumer's borrow-3 body subscriber OPENS the service and the graph
/// BUILDS. Without the create-side floor this exact no-stamp ordering creates the service at the
/// iceoryx2 default borrow 2 and REFUSES the consumer (the failure the
/// stamp's borrow axis covered) — so this test FAILS without the floor.
///
/// Anti-tautology (the consumer building is not vacuous): (1) the borrow floor is
/// asserted directly on the created service — a floor revert to 2 fails the
/// consumer open below; (2) the HARVEST still reads borrow 2 — the floor is
/// CREATE-SIDE only and deliberately does NOT enter the harvest/stamp (a stamped
/// borrow becomes an armed OPEN requirement on workers, which the floor must
/// never be); (3) the BUFFER axis stays at the un-stamped local default — a
/// deeper-buffer cross-group consumer would still need the supervisor stamp.
#[test]
#[serial]
fn without_stamp_cross_group_snapshot_consumer_builds_on_the_borrow_floor() {
    let prefix = "neg";
    let topic = "/neg/producer/out";
    let clock = Arc::new(VirtualClock::new());
    let mgr = ext_test_manager_with_clock("neg", Arc::clone(&clock));

    // NO override — the producer worker sees only its own subgraph (no snapshot
    // consumer). Without the floor that creates the service at the iceoryx2 default
    // borrow 2; the create-side owned-topic floor creates it at 3.
    let producer_rt =
        build_producer_via_worker_path(&mgr, Arc::clone(&clock), prefix, "neg_bar", None);

    // (1) DIRECT borrow-floor pin: the created service reads borrow 3 (== the
    // SUBSCRIBER_MAX_BORROWED_HELD; pub(crate), literal here). Reverting
    // the floor drops this to the iceoryx2 default 2 and the consumer
    // open below fails.
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(topic),
        3,
        "without any stamp the owned service is created at the create-side borrow floor (3), \
         not the iceoryx2 default 2 (the no-floor behavior)"
    );

    // (2)+(3) HARVEST NEGATIVES (anti-tautology): the harvest carries GENUINE
    // requirements only — borrow stays the folded default 2 (the create-side
    // floor deliberately does NOT ride the harvest/stamp, else a stamped union
    // would arm it as an OPEN requirement on workers) and the buffer axis stays
    // at the LOCAL default (16 = transport default, no consumer depth) — a
    // deeper-buffer cross-group consumer would still require the supervisor's
    // stamp.
    {
        let harvest = producer_rt.topic_requirements();
        assert_eq!(
            harvest[topic].min_borrowed_samples, 2,
            "the harvest must carry the GENUINE borrow requirement (folded default 2) — \
             the create-side floor must NOT enter the harvest/stamp"
        );
        assert_eq!(
            harvest[topic].min_buffer, 16,
            "the create-side floor lifts the created borrow only — the buffer axis stays at \
             the un-stamped local default (16); it still needs the stamp to raise"
        );
    }

    // POSITIVE pin: the cross-group snapshot consumer's borrow-3 body subscriber
    // OPENS the producer-created borrow-3 service and the graph BUILDS (without the
    // floor this hard-aborts with DoesNotSupportRequestedMinSubscriberBorrowedSamples).
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let (cons_cfg, cons_fac) = build_external_snapshot_graph("negc", topic, last_read);
    let cons_clock = Arc::new(VirtualClock::new());
    let consumer_rt = GraphRuntime::build(cons_cfg, cons_fac, &mgr, cons_clock).expect(
        "without any stamp, the cross-group snapshot consumer MUST open the create-side \
         borrow-floor service and build — an inflating harvest would refuse it",
    );

    consumer_rt.shutdown();
    producer_rt.shutdown();
}

// ===========================================================================
// Mutation-style pins on the two load-bearing guards.
//
// 1. External-topic EXCLUSION (both guards): the union loop must SKIP External
//    topics (a worker OPENS, never creates, an unowned topic — inflating an
//    open requirement against a foreign creator's service would fail) and the
//    harvest must FILTER them out (no worker owns them, nothing to stamp).
//    Deleting either guard is what this arm catches.
// 2. Per-topic KEYING: the union is keyed by `overrides.get(topic)` — a
//    regression to apply-first-entry/apply-to-all would pass any graph whose
//    overrides map covers every topic, so this one leaves a sibling topic out.
// ===========================================================================

/// A NON-snapshot consumer for the External-exclusion pin: its ONE input is a
/// `trigger` (data-trigger policy inferred), so it is drained — never held —
/// and its source topic is NOT a snapshot-source (no hold borrow-3 raise).
/// That keeps the External topic's LOCAL config at pure defaults (borrow None,
/// buffer 16), so any provisioning inflation observed in the test can ONLY come
/// from a (mutated) override leak — never from this consumer.
#[cerulion_node]
#[derive(Default)]
struct TrigConsumer {
    #[input(trigger)]
    inp: Vector3,
    sum: f64,
}

#[cerulion_node_impl]
impl TrigConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        Ok(())
    }
}

/// External topics are EXCLUDED from BOTH the
/// override union and the harvest — pins on the two guards.
///
/// Graph: an owned producer topic + a data-trigger consumer of an ABSOLUTE
/// external source (`/ext/...`, no in-graph producer → `External`
/// provisioning). A FOREIGN raw publisher pre-creates the external service at
/// the stock caps (borrow 2, ceiling 16). The worker-path build is handed an
/// overrides map keyed to the EXTERNAL topic with INFLATED requirements
/// (borrow 3, buffer 64 — both above the foreign caps).
///
/// - Union-loop guard (`runtime.rs` `if External { continue }`): with the guard,
///   the External topic's config stays at its local defaults, so the open
///   against the borrow-2/ceiling-16 foreign service SUCCEEDS and the foreign
///   service is observably untouched (borrow still 2). Deleting the
///   guard applies borrow-3/buffer-64 to the External open requirement → the
///   open against the foreign service FAILS → this test's build `expect` panics.
/// - Harvest-filter guard (`.filter(!= External)`): `topic_requirements()` must
///   contain ONLY the owned producer topic, never the External key. Deleting
///   the filter puts the External key in the map → the negative
///   assertion fails.
#[test]
#[serial]
fn external_topic_excluded_from_union_and_harvest() {
    use cerulion_core::wire::MaxSliceLen;

    let prefix = "extown";
    let owned_topic = "/extown/producer/out";
    let ext_topic = "/ext/cam";
    let clock = Arc::new(VirtualClock::new());
    let mgr = ext_test_manager_with_clock("ext", Arc::clone(&clock));

    // A FOREIGN raw publisher creates the external service FIRST at the stock
    // caps (borrow 2, ceiling 16) — the graph does not own this topic; the
    // foreign creator's provisioning is authoritative.
    let _foreign = mgr
        .create_publisher(ext_topic, MaxSliceLen::const_new(256), 0)
        .expect("foreign publisher creates the external service at stock caps");

    // Overrides keyed to the EXTERNAL topic ONLY, with requirements the foreign
    // service can NOT satisfy (borrow 3 > 2, buffer 64 > 16). With the External
    // guard intact these are dead weight; without it they poison the open.
    let mut overrides: std::collections::BTreeMap<String, cerulion_core::TopicRequirements> =
        std::collections::BTreeMap::new();
    overrides.insert(
        ext_topic.to_string(),
        cerulion_core::TopicRequirements {
            min_borrowed_samples: 3,
            min_buffer: 64,
            min_subscribers: 9,
            min_event_listeners: 0,
        },
    );

    // Owned producer + External-source data-trigger consumer, built via the
    // multi-process worker path with the stamped overrides. Both nodes are
    // roots (the consumer's trigger source has no in-graph producer) → ONE
    // global level.
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_prod".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "extcons".to_string(),
                node_type: "trig_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: ext_topic.to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer::default())),
    );
    factories.insert(
        "extcons".to_string(),
        Box::new(TrigConsumerEntry::with_state(TrigConsumer::default())),
    );
    let barrier =
        Arc::new(MappedBarrier::create_owned("ext_bar", "g", 1).expect("barrier owner create"));
    // UNION-GUARD PIN (behavioral): with the External skip intact the external
    // topic's open requirement stays at its local defaults, which the foreign
    // borrow-2/ceiling-16 service satisfies → the build SUCCEEDS. Deleting the
    // skip inflates the requirement to borrow-3/buffer-64 → this build FAILS.
    let rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        vec![Some(0)],
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the law every generation pin here asserts.
        vec![false; 1],
        0,
        Duration::from_millis(1),
        CrossProcessWiring::requirements_only(Some(&overrides)),
    )
    .expect(
        "an override keyed to an EXTERNAL topic must be IGNORED — the worker opens \
         (never creates) unowned topics, so the foreign borrow-2/ceiling-16 service \
         must still satisfy the graph's un-inflated requirement (the harvest's External guard)",
    );

    // UNION-GUARD PIN (direct): the foreign service's provisioning is untouched —
    // still the stock borrow 2, NOT the overridden 3.
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(ext_topic),
        2,
        "the External topic's foreign service must keep its stock borrow=2 — the \
         override must never inflate an unowned topic's provisioning"
    );

    // HARVEST-FILTER PIN: the harvested map carries ONLY owned topics — the
    // External key must be ABSENT (no worker owns it; stamping it would tempt a
    // worker into imposing requirements on a foreign creator's service).
    let reqs = rt.topic_requirements();
    assert!(
        !reqs.contains_key(ext_topic),
        "topic_requirements() must EXCLUDE the External topic '{ext_topic}', got keys {:?}",
        reqs.keys().collect::<Vec<_>>()
    );
    assert!(
        reqs.contains_key(owned_topic),
        "topic_requirements() must still contain the owned topic '{owned_topic}', got keys {:?}",
        reqs.keys().collect::<Vec<_>>()
    );
    // Hand oracle for the owned topic (no consumers, no snapshot): harvested
    // borrow 2 (the GENUINE requirement — the create-side floor
    // deliberately does NOT enter the harvest; the created SERVICE still
    // carries 3), buffer 16 (transport default), 0 in-graph subs + 4 headroom.
    let owned = &reqs[owned_topic];
    assert_eq!(
        owned.min_borrowed_samples, 2,
        "the harvest carries GENUINE borrow requirements only — the \
         create-side floor must not appear here"
    );
    assert_eq!(owned.min_buffer, 16);
    assert_eq!(
        owned.min_subscribers,
        cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM
    );

    rt.shutdown();
}

/// The union is keyed PER TOPIC; an override for
/// topic A must lift ONLY A, leaving sibling owned topic B byte-identical to its
/// local view (and B thereby exercises the map-absent `else continue` path).
///
/// Two owned producer topics; the overrides map contains ONLY A with a raised
/// borrow (3) + subscriber count (9). BOTH owned SERVICES are
/// CREATED at the borrow floor (3 — create-side generosity), so the created-
/// service borrow does not distinguish A from B; the per-topic keying
/// anti-tautology rides the HARVEST: A's harvest carries the stamped borrow 3 +
/// subscribers 9, B's harvest stays at its hand-computed local view (borrow 2 —
/// the genuine default, the create floor never enters the harvest — buffer 16,
/// subscribers 0+4). Swapping `overrides.get(topic)` for
/// `overrides.values().next()` (apply-first-entry-to-all) lifts B's harvest
/// too → the B assertions fail.
#[test]
#[serial]
fn override_applies_only_to_its_topic_not_siblings() {
    let prefix = "sib";
    let topic_a = "/sib/prod_a/out";
    let topic_b = "/sib/prod_b/out";
    let clock = Arc::new(VirtualClock::new());
    let mgr = ext_test_manager_with_clock("sib", Arc::clone(&clock));

    // Overrides for A ONLY (raised borrow + subscriber count); B is absent.
    let mut overrides: std::collections::BTreeMap<String, cerulion_core::TopicRequirements> =
        std::collections::BTreeMap::new();
    overrides.insert(
        topic_a.to_string(),
        cerulion_core::TopicRequirements {
            min_borrowed_samples: 3,
            min_buffer: 16,
            min_subscribers: 9,
            min_event_listeners: 0,
        },
    );

    // Two independent owned producer topics (both roots → ONE global level).
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "sib_prod".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod_a".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod_b".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod_a".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer::default())),
    );
    factories.insert(
        "prod_b".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer::default())),
    );
    let barrier =
        Arc::new(MappedBarrier::create_owned("sib_bar", "g", 1).expect("barrier owner create"));
    let rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        vec![Some(0)],
        // No level takes the extra mid-level rendezvous in this
        // fixture, so the run keeps exactly one barrier generation per global
        // level — the law every generation pin here asserts.
        vec![false; 1],
        0,
        Duration::from_millis(1),
        CrossProcessWiring::requirements_only(Some(&overrides)),
    )
    .expect("build the two-producer worker with an A-only override");

    // Both CREATED services carry borrow 3: A via the stamped union (armed as
    // a genuine requirement AND the create value), B via the
    // create-side floor. The created-service borrow therefore does not
    // distinguish A from B — the per-topic keying pin lives in the HARVEST
    // below (stamped borrow + subscribers on A only).
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(topic_a),
        3,
        "topic A is created at the stamped borrow=3"
    );
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(topic_b),
        3,
        "sibling topic B (absent from the overrides map) is created at the \
         owned-topic borrow floor (3) — create-side generosity, not a stamp leak; \
         the per-topic keying pin lives in the harvest below"
    );

    // Post-union harvest: A lifted on the stamped axes (borrow 3, subscribers
    // 9); B byte-identical to its hand-computed LOCAL view (borrow 2 = the
    // genuine folded default — the create-side floor never enters the harvest —
    // buffer 16 = transport default, subscribers 0 in-graph + 4 introspection
    // headroom, 0 event listeners).
    let reqs = rt.topic_requirements();
    let a = &reqs[topic_a];
    assert_eq!(a.min_borrowed_samples, 3, "A's harvest reflects the union");
    assert_eq!(
        a.min_subscribers, 9,
        "A's subscriber count lifted to the stamp"
    );
    let b = &reqs[topic_b];
    assert_eq!(
        (
            b.min_borrowed_samples,
            b.min_buffer,
            b.min_subscribers,
            b.min_event_listeners
        ),
        (
            2,
            16,
            cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM,
            0
        ),
        "B must stay byte-identical to its local view on EVERY axis — an \
         override for A must never leak onto B (the per-topic keying pin)"
    );

    rt.shutdown();
}

// ===========================================================================
// The OWNED-topic borrow FLOOR — CREATE-SIDE generosity only. A
// graph-owned (SingleWriter/Multi) topic is CREATED at
// `subscriber_max_borrowed_samples = 3`, so a HOLD consumer never aborts on
// start order (the hold needs borrow >= 3, and iceoryx2 bakes the
// ceiling in at CREATION). The floor is NEVER an open requirement: an owner
// TOLERATES a pre-existing sub-floor service (attach + degraded warn — the
// tolerance test below). Externals impose nothing (see the External-arm pins
// above).
// ===========================================================================

/// Two managers over ONE shared SHM root (crib `cross_graph_collision_iox2_test`
/// `manager`) — models two separate "graph processes" sharing the transport.
/// 16-deep buffer so the producer creates and the consumer opens the owned topic
/// at the SAME ceiling (`max(consumer depth 10, 16) = 16` on both sides).
fn shared_root_manager(
    name: &str,
    ix: iceoryx2::config::Config,
) -> Arc<cerulion_core::transport::TransportManager> {
    cerulion_core::transport::TransportManager::init_for_test(
        cerulion_core::transport::TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix,
    )
    .expect("init shared-root test manager")
}

/// A single-node PRODUCER graph OWNING an absolute topic via a `topic:` override
/// (the graph publishes under `abs_topic`, which the consumer graph reads as an
/// absolute external source). `producer` is a HostDriven `external` node fired via
/// the polled `trigger_external` + `step` seam.
fn build_producer_override_graph(
    prefix: &str,
    abs_topic: &str,
    val: f64,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_prod"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "producer".to_string(),
            node_type: "hold_producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: Some(abs_topic.to_string()),
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let producer = HoldProducer {
        val,
        ..Default::default()
    };
    factories.insert(
        "producer".to_string(),
        Box::new(HoldProducerEntry::with_state(producer)),
    );
    (config, factories)
}

/// A unique OWNED absolute topic (a producer graph creates it via a `topic:`
/// override; the consumer graph reads it as an absolute external source).
fn unique_owned_abs_topic() -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("/shared/held510/{nanos}/{id}")
}

/// HEADLINE: PRODUCER-graph-FIRST, then a HOLD-consumer graph as
/// a SEPARATE "process", builds AND holds. Without the floor this exact ordering — the producer
/// graph creates the owned service FIRST — hard-aborts: the producer
/// creates it at the iceoryx2 default borrow 2, so the HOLD consumer's borrow-3
/// body subscriber is refused with
/// `DoesNotSupportRequestedMinSubscriberBorrowedSamples` (the transport/mod.rs
/// open-error arm). The create-side floor provisions every OWNED topic at the borrow-3 floor at
/// CREATION, so start order does not matter. Without the create-side floor this test FAILS
/// (the consumer build panics at the `.expect`).
#[test]
#[serial]
fn producer_graph_first_then_snapshot_consumer_builds_and_holds() {
    const V: u64 = 88;
    const HORIZON: usize = 10;

    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_prod = shared_root_manager("nth_floor_prod", ix.clone());
    let mgr_cons = shared_root_manager("nth_floor_cons", ix);
    let abs_topic = unique_owned_abs_topic();

    // PRODUCER graph builds FIRST on its own manager, OWNING `abs_topic` (a
    // `topic:` override so a graph producer publishes under the absolute name the
    // consumer reads). It creates the service at the borrow-3 floor.
    let (prod_cfg, prod_fac) = build_producer_override_graph("nthfloorp", &abs_topic, V as f64);
    let prod_clock = Arc::new(VirtualClock::new());
    let mut prod_rt = GraphRuntime::build(prod_cfg, prod_fac, &mgr_prod, prod_clock)
        .expect("producer graph owns the absolute topic (created at the owned-topic borrow floor)");

    // The producer-created service carries the held floor (3), NOT the iceoryx2
    // default 2 — the exact provisioning that makes creation order irrelevant.
    assert_eq!(
        mgr_prod.default_subscriber_max_borrowed_samples_for_test(&abs_topic),
        3,
        "the producer graph must create the owned topic at the borrow-3 floor"
    );

    // CONSUMER graph builds SECOND as a separate "process": its borrow-3 HOLD body
    // subscriber must OPEN the producer-created service. Without the floor this hard-
    // aborts (producer creates at borrow 2 → borrow-3 open refused).
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let (cons_cfg, cons_fac) =
        build_external_snapshot_graph("nthfloorc", &abs_topic, Arc::clone(&last_read));
    let cons_clock = Arc::new(VirtualClock::new());
    let mut cons_rt = GraphRuntime::build(cons_cfg, cons_fac, &mgr_cons, cons_clock).expect(
        "the HOLD consumer graph must open the producer-created borrow-3 \
             service and build (an open below the borrow floor aborts with \
             DoesNotSupportRequestedMinSubscriberBorrowedSamples)",
    );

    // Establish the held value: fire the PRODUCER graph once (publishes V), then
    // fire ONLY the consumer graph until it drains V (bounded — tolerates iceoryx2
    // surfacing latency). Producer + consumer are DISTINCT runtimes on the shared
    // root.
    prod_rt
        .trigger_external("producer")
        .expect("trigger producer");
    prod_rt.step(STEP);
    let mut tries = 0;
    loop {
        last_read.store(MISSING, Ordering::Relaxed);
        cons_rt
            .trigger_external("consumer")
            .expect("trigger consumer");
        cons_rt.step(STEP);
        if last_read.load(Ordering::Relaxed) == V {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "producer-first held value {V} never established within 200 consumer fires"
        );
    }

    // Silent window: the producer graph NEVER fires again; the consumer replays the
    // held value on every step (the hold, over a producer-graph-first
    // owned service).
    let mut observed = Vec::with_capacity(HORIZON);
    for _ in 0..HORIZON {
        last_read.store(MISSING, Ordering::Relaxed);
        cons_rt
            .trigger_external("consumer")
            .expect("trigger consumer");
        cons_rt.step(STEP);
        observed.push(last_read.load(Ordering::Relaxed));
    }
    // HAND ORACLE: the held value replays on every one of the HORIZON silent steps.
    assert_eq!(
        observed,
        vec![V; HORIZON],
        "the held value {V} must replay on every silent step over a producer-graph-first \
         owned topic (got {observed:?})"
    );

    cons_rt.shutdown();
    prod_rt.shutdown();
}

/// DRIFT-GUARD: a producer-only OWNED topic — no consumer, no
/// snapshot source, no cross-group stamp — is created at the borrow FLOOR
/// (3 == `SUBSCRIBER_MAX_BORROWED_HELD`; `pub(crate)`, literal here) purely from
/// the CREATE leg (`create_borrowed_samples` at the open site — the open
/// requirement stays None). Without the floor an owned topic with no snapshot consumer
/// is created at the iceoryx2 default 2; a floor revert makes this read 2.
/// Isolates the FLOOR from the post-pass and the stamp.
#[test]
#[serial]
fn producer_only_owned_topic_carries_the_borrow_floor() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = shared_root_manager("nth_floor_drift", ix);
    let abs_topic = unique_owned_abs_topic();

    let (cfg, fac) = build_producer_override_graph("nthfloord", &abs_topic, 1.0);
    let rt = GraphRuntime::build(cfg, fac, &mgr, Arc::new(VirtualClock::new()))
        .expect("producer-only owned-topic graph builds");

    // The created service carries the owned-topic borrow floor (3), with NO
    // snapshot consumer and NO cross-group stamp — purely the create-leg floor
    // (`create_borrowed_samples` on the SingleWriter provisioning).
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(&abs_topic),
        3,
        "a producer-only OWNED topic must be CREATED at the create-side borrow floor (3), \
         not the iceoryx2 default 2 (the no-floor behavior)"
    );

    rt.shutdown();
}

/// TOLERANCE pin (the create-vs-open asymmetry): a raw
/// pre-existing borrow-2 service + an OWNED-producer graph → the graph ATTACHES
/// DEGRADED (builds, publishes fine) instead of refusing, and the borrow-axis
/// degraded warn is observable. This is the regression guard for the
/// floor-as-open-requirement bug: a version that arms the floor on the OPEN leg
/// fails this build with `DoesNotSupportRequestedMinSubscriberBorrowedSamples`
/// (exactly the `cross_graph_collision` degraded arm + the barrier_park_wake
/// multi-context splits, which a floor armed on the OPEN leg breaks).
///
/// Setup cribs `cross_graph_collision_iox2_test::degraded_service_still_rejects_
/// second_graph_publisher`: a default opener (a raw subscriber, holding NO
/// publisher slot so the single-writer active-publisher pre-check stays quiet)
/// pre-creates the service at iceoryx2 defaults (borrow 2). The warn pin uses
/// the `#[tracing_test::traced_test]` + `logs_assert` pattern from
/// `topic_buffer_sizing_test` (tracing-test has `no-env-filter`).
#[test]
#[serial]
#[tracing_test::traced_test]
fn owned_producer_attaches_degraded_to_preexisting_borrow2_service() {
    const V: u64 = 91;

    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = shared_root_manager("nth_floor_degr", ix);
    let abs_topic = unique_owned_abs_topic();

    // A raw DEFAULT opener pre-creates the service at iceoryx2 defaults
    // (borrow 2, no port requirements) and holds it alive — no publisher
    // port, so the graph's single-writer pre-check is not in play.
    let raw_sub = mgr
        .create_subscriber(&abs_topic)
        .expect("default opener pre-creates the service at iceoryx2 defaults");
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(&abs_topic),
        2,
        "precondition: the pre-existing service carries the iceoryx2 default borrow (2)"
    );

    // The OWNED-producer graph must ATTACH to the sub-floor service (degraded,
    // warned) — the floor is create-side generosity, never an open
    // requirement. A floor-as-open-requirement regression fails HERE with
    // DoesNotSupportRequestedMinSubscriberBorrowedSamples.
    let (cfg, fac) = build_producer_override_graph("nthfloorg", &abs_topic, V as f64);
    let mut rt = GraphRuntime::build(cfg, fac, &mgr, Arc::new(VirtualClock::new())).expect(
        "an owned producer must attach DEGRADED to a pre-existing borrow-2 service \
         (tolerate + warn), never refuse it — the create-vs-open asymmetry",
    );

    // The service was OPENED, not re-created: it still reads the pre-existing
    // borrow 2 (iceoryx2 bakes the ceiling in at creation — nothing can raise it).
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(&abs_topic),
        2,
        "attaching must not (and cannot) change the pre-existing service's borrow ceiling"
    );

    // Degraded-attach is FUNCTIONAL: the producer publishes through the
    // sub-floor service and the raw subscriber receives the hand-oracle value.
    rt.trigger_external("producer").expect("trigger producer");
    rt.step(STEP);
    let mut seen = Vec::new();
    raw_sub
        .try_receive(|msg| {
            // Vector3.x is the first f64 of the payload (crib
            // drain_discipline_seam_test's raw read).
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().unwrap());
            seen.push(x as u64);
        })
        .expect("receive from the degraded-attached producer");
    assert_eq!(
        seen,
        vec![V],
        "the degraded-attached producer must publish fine (hand oracle {V})"
    );

    // The borrow-axis degraded warn fired EXACTLY once (dedup per (topic,
    // kind)) — the tolerance is loud, mirroring the publisher/subscriber
    // degrade warns.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("borrowed-samples provisioning degraded"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 borrowed-samples degraded warn, got {n}"
            ))
        }
    });

    rt.shutdown();
}

// ===========================================================================
// InputView::wire_timestamp_ns / wire_header.
//
// The freshness surface the teleop safety mux (and any staleness arbiter)
// reads: the wire-header publish `timestamp_ns` of the frame a view serves.
// For a HELD latest-value input this MUST be the HELD frame's ORIGINAL stamp,
// replayed unchanged across silent steps — a held frame does NOT look fresh,
// so `age = now - wire_timestamp_ns` grows while the producer is silent
// (that growing age is exactly what the mux arbitrates on).
//
// Tests 1-3 drive raw publisher/subscriber DIRECTLY on an `init_for_test`
// manager whose CLOCK we control (a `VirtualClock`), so the publisher stamps
// `WireHeader::timestamp_ns == clock.now_ns()` at loan time (publisher.rs
// captures the stamp at loan) — letting `clock.set(TS)` pin an EXACT wire
// timestamp for a crisp HAND oracle (no self-compare). Test 4 proves the
// accessor works UNCHANGED inside a real `#[cerulion_node]` tick body
// (`self.inp.wire_timestamp_ns()` — an inherent method beats Deref, zero
// macro changes) and that the held stamp stays constant while the graph clock
// advances (the freshness-age contract).
// ===========================================================================

/// An isolated `init_for_test` transport whose CLOCK is the caller's `clock`,
/// so a publisher created on it stamps `WireHeader::timestamp_ns ==
/// clock.now_ns()` at loan time. `clock.set(TS)` before a `loan_proxy` pins an
/// EXACT wire timestamp — the anchor for the `wire_timestamp_ns`
/// hand oracles. 16-deep buffer (matches the other direct-drive helpers).
fn ts_test_manager(
    node_name: &str,
    clock: Arc<VirtualClock>,
) -> Arc<cerulion_core::transport::TransportManager> {
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let transport_config = cerulion_core::transport::TransportConfig {
        node_name: node_name.into(),
        clock,
        subscriber_buffer_size: 16,
        network: None,
    };
    cerulion_core::transport::TransportManager::init_for_test(transport_config, ix_config)
        .expect("init wire-timestamp test transport")
}

/// A unique direct-drive topic name (each `init_for_test` manager already gets
/// an isolated SHM root; the counter avoids any same-process re-run collision).
fn unique_ts_topic(tag: &str) -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/{tag}/{nanos}/{id}")
}

// Test 1 — a LIVE input's `wire_timestamp_ns()` returns the publisher's stamp.
#[test]
#[serial]
fn wire_timestamp_ns_on_live_input_returns_publisher_stamp() {
    use cerulion_core::wire::MaxSliceLen;

    const TS_LIVE: u64 = 1_234_000_000; // 1.234 s — the exact stamp we pin
    const VAL: f64 = 7.0;

    let clock = Arc::new(VirtualClock::new());
    let mgr = ts_test_manager("live", Arc::clone(&clock));
    let topic = unique_ts_topic("live");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Stamp the frame at EXACTLY TS_LIVE: set the controlled clock first (the
    // publisher captures `clock.now_ns()` at loan time), then loan+write+drop.
    clock.set(TS_LIVE);
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = VAL;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    // Let iceoryx2 surface the sample on the subscriber side.
    std::thread::sleep(Duration::from_millis(50));

    subscriber.snapshot_latest();
    // Read BOTH accessors in one view: `wire_timestamp_ns()` and the full
    // `wire_header()`. They share ONE parse, so the timestamps must agree.
    let (via_ts, via_header_ts, total_size) = subscriber
        .try_view::<Vector3, _>(|view| {
            let header = view.wire_header();
            (
                view.wire_timestamp_ns(),
                header.timestamp_ns,
                header.total_size,
            )
        })
        .expect("try_view ok")
        .expect("a live sample is served");

    // HAND ORACLE: the accessor returns the publisher's stamp EXACTLY, and the
    // sibling `wire_header()` agrees (the two accessors are one parse).
    assert_eq!(
        via_ts, TS_LIVE,
        "wire_timestamp_ns() must return the publisher's stamped timestamp \
         ({TS_LIVE}), got {via_ts}"
    );
    assert_eq!(
        via_header_ts, TS_LIVE,
        "wire_header().timestamp_ns must equal wire_timestamp_ns() ({TS_LIVE}) — \
         both parse the same 32 bytes, got {via_header_ts}"
    );
    // Non-vacuous: a real frame was served (total_size includes the 32-byte
    // header + a Vector3 payload), so the parse read a genuine header.
    assert!(
        total_size as usize > cerulion_core::wire::WireHeader::SIZE,
        "the served frame must carry a real payload (total_size {total_size} > 32)"
    );
}

// Test 2 — a HELD non-trigger input REPLAYS its original stamp across silent
// steps; the accessor reads the FRAME, never the (advancing) clock, so the
// freshness age grows.
#[test]
#[serial]
fn wire_timestamp_ns_held_replays_original_stamp_and_ages() {
    use cerulion_core::scheduler::BackpressureCounters;
    use cerulion_core::wire::MaxSliceLen;

    const TS_HELD: u64 = 5_000_000_000; // 5.0 s — the ORIGINAL publish stamp
    const N: usize = 6;
    const VAL: f64 = 42.0;

    let clock = Arc::new(VirtualClock::new());
    let mgr = ts_test_manager("held", Arc::clone(&clock));
    let topic = unique_ts_topic("held");

    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");
    // Production fidelity: a graph non-trigger input is wired `drop_oldest`.
    let counters = Arc::new(BackpressureCounters::new());
    subscriber.register_drop_oldest_probe_for_test(
        Arc::clone(&counters),
        Arc::from("mux_node"),
        Arc::from("inp"),
        16,
    );

    // Establish the held value stamped at EXACTLY TS_HELD.
    clock.set(TS_HELD);
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = VAL;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    std::thread::sleep(Duration::from_millis(50));
    subscriber.snapshot_latest();
    let established = subscriber
        .try_view::<Vector3, _>(|view| view.wire_timestamp_ns())
        .expect("establish try_view ok")
        .expect("first delivery serves a sample");
    assert_eq!(
        established, TS_HELD,
        "the established held frame must carry the original stamp {TS_HELD}"
    );

    // Silent window: the publisher NEVER sends again. Before each replay we
    // shove the clock FAR forward to distinct, increasing values — so a bug
    // that read "now" instead of the frame would drift; the held frame's stamp
    // must stay TS_HELD.
    let mut replayed_ts = Vec::with_capacity(N);
    let mut ages = Vec::with_capacity(N);
    for i in 0..N {
        let now = TS_HELD + (i as u64 + 1) * 300_000_000; // +0.3s, +0.6s, ...
        clock.set(now);
        subscriber.snapshot_latest(); // Empty drain → replay the held frame
        let ts = subscriber
            .try_view::<Vector3, _>(|view| view.wire_timestamp_ns())
            .expect("replay try_view ok")
            .expect("the held frame is replayed on a silent step");
        replayed_ts.push(ts);
        ages.push(now.saturating_sub(ts));
    }

    // HAND ORACLE (primary): the ORIGINAL stamp is replayed unchanged on every
    // one of the N silent steps — a held frame keeps its own publish time.
    assert_eq!(
        replayed_ts,
        vec![TS_HELD; N],
        "the held frame's wire_timestamp_ns() must replay the ORIGINAL stamp \
         {TS_HELD} on every silent step, NOT the advancing clock (got {replayed_ts:?})"
    );
    // HAND ORACLE (freshness contract): age = now - wire_timestamp_ns grows
    // strictly across the window — the held frame gets progressively STALER
    // (it does NOT look fresh). Oracle: +0.3s per step (300_000_000, 600_000_000, ...).
    let expected_ages: Vec<u64> = (0..N).map(|i| (i as u64 + 1) * 300_000_000).collect();
    assert_eq!(
        ages, expected_ages,
        "the freshness age (now - held stamp) must grow by 0.3s per silent step \
         (got {ages:?}) — a held frame ages, it does not stay fresh"
    );
}

// Test 3 — two inputs with DISTINCT stamps are both readable in one tick (the
// mux shape: joystick + keyboard, each with its own freshness).
#[test]
#[serial]
fn two_inputs_distinct_wire_timestamps_both_readable() {
    use cerulion_core::wire::MaxSliceLen;

    const TS_A: u64 = 2_000_000_000; // "joystick" stamp
    const TS_B: u64 = 7_000_000_000; // "keyboard" stamp (distinct)

    let clock = Arc::new(VirtualClock::new());
    let mgr = ts_test_manager("two", Arc::clone(&clock));
    let topic_a = unique_ts_topic("two_a");
    let topic_b = unique_ts_topic("two_b");

    let mut pub_a = mgr
        .create_publisher_simple(&topic_a, MaxSliceLen::const_new(256))
        .expect("create publisher a");
    let mut pub_b = mgr
        .create_publisher_simple(&topic_b, MaxSliceLen::const_new(256))
        .expect("create publisher b");
    let mut sub_a = mgr
        .create_subscriber(&topic_a)
        .expect("create subscriber a");
    let mut sub_b = mgr
        .create_subscriber(&topic_b)
        .expect("create subscriber b");

    // Publish A stamped at TS_A, then B stamped at TS_B (distinct clock sets).
    clock.set(TS_A);
    {
        let mut proxy = pub_a.loan_proxy::<Vector3>().expect("loan a");
        proxy.x = 3.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    clock.set(TS_B);
    {
        let mut proxy = pub_b.loan_proxy::<Vector3>().expect("loan b");
        proxy.x = 4.0;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }
    std::thread::sleep(Duration::from_millis(50));

    // Read BOTH inputs in one logical tick (the mux reads joy + key stamps
    // together): snapshot then view each, forming the tuple.
    sub_a.snapshot_latest();
    sub_b.snapshot_latest();
    let a_ts = sub_a
        .try_view::<Vector3, _>(|view| view.wire_timestamp_ns())
        .expect("view a ok")
        .expect("a serves a sample");
    let b_ts = sub_b
        .try_view::<Vector3, _>(|view| view.wire_timestamp_ns())
        .expect("view b ok")
        .expect("b serves a sample");

    // HAND ORACLE: each input reports its OWN stamp — no cross-talk, both
    // readable in one tick.
    assert_eq!(
        (a_ts, b_ts),
        (TS_A, TS_B),
        "each input must report its own wire timestamp (a={TS_A}, b={TS_B}), got \
         (a={a_ts}, b={b_ts})"
    );
    assert_ne!(a_ts, b_ts, "the two inputs must carry DISTINCT stamps");
}

// ---------------------------------------------------------------------------
// Test 4 — the accessor works UNCHANGED in a real macro tick body, and the
// held stamp stays constant while the graph clock advances (the
// freshness-age contract, end-to-end through `#[cerulion_node]`).
// ---------------------------------------------------------------------------

/// A macro consumer that records the wire-header publish timestamp of its
/// single non-trigger latest-value input (via the inherent
/// `InputView::wire_timestamp_ns()` accessor — ZERO macro changes) AND the
/// node's current clock, so the test can watch the freshness age grow while
/// the producer is silent. `external` so the harness controls firing.
#[cerulion_node(external)]
#[derive(Default)]
struct TsHoldConsumer {
    #[input]
    inp: Vector3,
    last_wire_ts: Arc<AtomicU64>,
    last_now_ns: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl TsHoldConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // `self.inp` is a `&InputView<'_, Vector3>`; the inherent accessor
        // beats Deref, so this reads the served frame's wire timestamp with no
        // macro change. If `inp` is Empty (pre-delivery) the macro collapses the
        // whole tick to a no-op and NEITHER store runs (MISSING survives).
        self.last_wire_ts
            .store(self.inp.wire_timestamp_ns(), Ordering::Relaxed);
        self.last_now_ns.store(self.now_ns(), Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
#[serial]
fn macro_body_wire_timestamp_holds_original_while_clock_advances() {
    const N: usize = 12;

    let last_wire_ts = Arc::new(AtomicU64::new(MISSING));
    let last_now_ns = Arc::new(AtomicU64::new(MISSING));
    let consumer = TsHoldConsumer {
        last_wire_ts: Arc::clone(&last_wire_ts),
        last_now_ns: Arc::clone(&last_now_ns),
        ..Default::default()
    };
    let mut rt = build_one_input_graph(
        "macro",
        "hold_producer",
        Box::new(HoldProducerEntry::with_state(HoldProducer {
            val: 42.0,
            ..Default::default()
        })),
        "ts_hold_consumer",
        Box::new(TsHoldConsumerEntry::with_state(consumer)),
        8,
    );

    // Establish the held frame: fire the producer once, then fire ONLY the
    // consumer until it records a real wire timestamp. Capture it as the oracle
    // anchor (a genuine, non-zero stamp — never hand-derived).
    rt.trigger_external("producer").expect("trigger producer");
    rt.step(STEP);
    let mut tries = 0;
    let held_ts = loop {
        last_wire_ts.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        let ts = last_wire_ts.load(Ordering::Relaxed);
        if ts != MISSING {
            break ts;
        }
        tries += 1;
        assert!(
            tries < 200,
            "the held frame never surfaced within 200 fires"
        );
    };
    assert!(
        held_ts > 0,
        "the established wire timestamp must be a real, non-zero publish stamp \
         (got {held_ts}) — a fabricated/zeroed header would read 0"
    );

    // Silent window: the producer NEVER fires again; the consumer fires every
    // step (each `step` advances the graph clock). Record (wire_ts, now_ns).
    let mut wire_window = Vec::with_capacity(N);
    let mut now_window = Vec::with_capacity(N);
    for _ in 0..N {
        last_wire_ts.store(MISSING, Ordering::Relaxed);
        last_now_ns.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        wire_window.push(last_wire_ts.load(Ordering::Relaxed));
        now_window.push(last_now_ns.load(Ordering::Relaxed));
    }

    // HAND ORACLE (held-stamp constancy): the held frame's wire timestamp is
    // the ORIGINAL, replayed unchanged on every silent step — a bug reading the
    // (advancing) clock instead of the frame would drift off `held_ts`.
    assert_eq!(
        wire_window,
        vec![held_ts; N],
        "self.inp.wire_timestamp_ns() must replay the ORIGINAL held stamp \
         {held_ts} on every silent step (got {wire_window:?})"
    );
    // FRESHNESS AGE: the graph clock advanced across the window (last > first)
    // and every recorded `now` is strictly past the held stamp — so
    // `age = now - held_ts` is positive and growing. The held frame does NOT
    // look fresh; that growing age is what the mux arbitrates on.
    assert!(
        *now_window.last().unwrap() > now_window[0],
        "the graph clock must advance across the silent window (first={}, last={})",
        now_window[0],
        now_window.last().unwrap()
    );
    for (i, &now) in now_window.iter().enumerate() {
        assert!(
            now != MISSING,
            "step {i}: the consumer must have fired (held input is served, not a no-op)"
        );
        assert!(
            now > held_ts,
            "step {i}: now ({now}) must be strictly past the held stamp ({held_ts}) — \
             the held frame ages (age = now - held_ts > 0)"
        );
    }
}
