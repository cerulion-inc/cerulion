// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-allocation proof for the level-executor `GraphRuntime::step()` hot
//! path.
//!
//! `zero_alloc_test.rs` proves the *subscriber receive* path is zero-alloc.
//! This file proves the complement: that driving a steady-state multi-level
//! trigger graph through `GraphRuntime::step()` — the full user-POV executor
//! (per-level drain → decide → fire-gated snapshot → tick, across DAG levels)
//! — allocates ZERO times on the heap once the chain has reached steady
//! state. This is the Principle-1 (zero-copy / zero-alloc hot path) contract
//! at the *executor* level, and its regression guard: a level executor that
//! builds its scratch per step costs 15 heap allocs/step (per-level `Vec<FireDecision>` /
//! `fire_times` / `other_ids` / drain `timestamps` / `fire_node`-local Vecs),
//! where this test requires ZERO. NOTE: profiling shows those allocs do NOT
//! set the user-POV p50, which is TRANSPORT-bound, so this test guards the
//! ALLOC COUNT, not latency. An alloc-free executor is the
//! right Principle-1 property regardless; this is its permanent guard.
//!
//! # Why a separate binary?
//!
//! `#[global_allocator]` is process-wide; we flip an `AtomicBool` so only the
//! measured steady-state `step()` window is counted (setup — transport init,
//! graph build, SHM pool warm — is excluded).
//!
//! # Topology
//!
//! ```text
//!  PingNode ── ping_out ──▶ PongNode ── echo_out ──▶ LatencyNode
//!  (period_ms=1, L0)        (data, L1)               (data, L2)
//! ```
//!
//! Three single-node DAG levels, all firing every step at steady state — the
//! exact moat chain `graph_latency_test` measures, so the alloc count here is
//! the mechanistic counterpart of that test's wall-clock p50.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test step_zero_alloc_test -- --test-threads=1
//! ```
//!
//! Runs in debug OR release (the alloc COUNT is build-mode-invariant — a heap
//! allocation is a heap allocation regardless of optimisation), so it gates on
//! every PR (unlike the release-only wall-clock `graph_latency_test`).
//!
//! ## What is EXCLUDED
//!
//! The per-step EXECUTOR is what this gate guards. The iceoryx2 liveliness sweep
//! (`cleanup_dead_nodes`, a 4 Hz iceoryx2-internal node-registry scan that
//! allocates ~13.8k/sweep, scaling with registry size — see
//! [`disable_liveliness_sweep`]) is a background task OUTSIDE that hot path, so
//! every measurement disables it. Without that, the gate would dodge the scan only by
//! clock-phase luck; the exclusion is explicit + deterministic. (That
//! "clock-phase luck" refers to the *liveliness-sweep cadence*, a separate
//! concern made deterministic by [`disable_liveliness_sweep`] — it is
//! NOT about the per-step alloc count, which is made robust by thread
//! scoping, below.)
//!
//! ## Counting mode
//!
//! The narrow/serial gates below drive `step()` on the CALLING thread and fire
//! their levels serially THERE (single-node fast-path, or narrow REST < the
//! parallel-fire threshold — see `scheduler::tick_decided_parallel`, which fires
//! those "on the calling thread, serially"), so their contract is precisely
//! "step() allocates nothing ON THE CALLING THREAD". They measure with a
//! THREAD-SCOPED counter (the same fix `shm_ring_zero_alloc_test.rs` already
//! adopted): only the measured thread's allocations count, so background-thread
//! allocations (iceoryx2-internal / libtest harness / platform TLS / lazy
//! runtime init) landing inside a measured window on a busy CI runner can no
//! longer be falsely attributed to `step()`. That false attribution was the
//! CI flake: `test_serial_gated_narrow_level_is_zero_alloc` failed CI with
//! 96 allocs/200 steps while the SAME SHA measured 0 locally, and the offending
//! allocs were proven NOT to come from the executor. The WIDE rayon gate is the
//! ONE exception — its level's REST (>= the threshold) fires on rayon WORKER
//! threads, so a thread-scope would blind it; it measures PROCESS-WIDE and
//! asserts a `< 64` injector-residual bound + node-count independence (never
//! `== 0`), which background jitter cannot flip. See the `CountingAllocator`
//! doc for the mechanics.

use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::Image;

// ---------------------------------------------------------------------------
// Counting allocator — TWO measurement modes on one process-global allocator
// (thread-scoping ported from `shm_ring_zero_alloc_test.rs`).
//
// Mode 1 — THREAD-SCOPED (default, `enable`): count only the measured/calling
// thread. Used by the narrow/serial gates, whose `step()` fires levels serially
// ON THE CALLING THREAD (single-node fast-path or narrow REST). Their contract
// is "step() allocates nothing ON THE CALLING THREAD", so background-thread
// allocations (iceoryx2-internal / libtest harness / platform TLS / lazy runtime
// init) that land inside a measured window on a busy CI runner are OUT of
// contract and must not be counted — that false attribution was the CI
// flake (96 allocs/200 steps on CI, 0 locally, provably not from the executor).
// A real allocation introduced onto the executor's calling-thread path still
// fails: it happens on the measured thread.
//
// Mode 2 — PROCESS-WIDE (`enable_process_wide`): count EVERY thread. Used ONLY
// by the wide rayon gate, whose level REST (>= PARALLEL_FIRE_THRESHOLD) fires on
// rayon WORKER threads via `pool.install(par_values_mut().for_each(..))`; a
// thread-scope would blind it. That gate already tolerates the crossbeam-deque
// `Injector` residual (~1 alloc / 63 installs, node-count-INDEPENDENT) plus
// inherent background noise — it asserts a `< 64` bound + node-count
// independence, NOT `== 0`, so process-wide background jitter cannot flip it.
// ---------------------------------------------------------------------------

thread_local! {
    /// `true` only on the thread that opened a THREAD-SCOPED window via
    /// [`CountingAllocator::enable`].
    ///
    /// MUST be `const`-init with a non-`Drop` payload (`Cell<bool>`): a lazy
    /// (allocating) TLS init or a registered destructor touched from inside
    /// `GlobalAlloc::alloc` would RECURSE into the allocator. Const-init TLS of a
    /// plain `Cell<bool>` performs no allocation and registers no destructor on
    /// first touch.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

struct CountingAllocator {
    inner: System,
    count: AtomicU64,
    enabled: AtomicBool,
    /// `false` (default) => THREAD-SCOPED: count only the measured/calling
    /// thread (the narrow/serial gates, immune to background-thread noise).
    /// `true` => PROCESS-WIDE: count every thread (the wide rayon gate, whose
    /// fires land on rayon workers).
    process_wide: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            count: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
            process_wide: AtomicBool::new(false),
        }
    }
    /// Open a THREAD-SCOPED window: mark THIS thread as the measured one, zero
    /// the count, and enable counting. Only this thread's allocations count.
    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.process_wide.store(false, Ordering::SeqCst);
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    /// Open a PROCESS-WIDE window: zero the count and enable counting on EVERY
    /// thread. For the wide rayon gate, whose fires run on rayon worker threads.
    fn enable_process_wide(&self) {
        MEASURED_THREAD.set(false);
        self.process_wide.store(true, Ordering::SeqCst);
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    /// Close the window (clearing the global flag, this thread's measured mark,
    /// and the process-wide mode) and return the count.
    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        self.process_wide.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.count.load(Ordering::SeqCst)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            // Count if the window is process-wide, else only on the measured
            // thread. `try_with`, never `with`: during thread teardown TLS is
            // inaccessible and `with` would panic INSIDE the allocator. An
            // inaccessible TLS means "not the measured thread" — don't count.
            let count_it = self.process_wide.load(Ordering::Relaxed)
                || MEASURED_THREAD.try_with(Cell::get).unwrap_or(false);
            if count_it {
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

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("salloc/{base}/{nanos}/{id}")
}

/// Bytes loaned for the variable `data` field (fixed across the run).
static PAYLOAD_SIZE: AtomicUsize = AtomicUsize::new(64);
/// How many round-trips LatencyNode has observed (drives steady-state detect).
static LATENCY_FIRES: AtomicU64 = AtomicU64::new(0);

// ---------------------------------------------------------------------------
// Graph nodes (mirror graph_latency_test's moat chain)
// ---------------------------------------------------------------------------

#[cerulion_node(period_ms = 1)]
struct PingNode {
    #[output]
    ping_out: Image,
}

#[cerulion_node_impl]
impl PingNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
        self.ping_out.height = 1;
        self.ping_out.width = 2;
        self.ping_out.step = 0;
        self.ping_out.is_bigendian = 0;
        self.ping_out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.ping_out
            .set_encoding("rt")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .ping_out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        Ok(())
    }
}

#[cerulion_node]
struct PongNode {
    #[input(trigger)]
    ping_in: Image,
    #[output]
    echo_out: Image,
}

#[cerulion_node_impl]
impl PongNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let h = self.ping_in.height;
        let w = self.ping_in.width;
        let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
        self.echo_out.height = h;
        self.echo_out.width = w;
        self.echo_out.step = 0;
        self.echo_out.is_bigendian = 0;
        self.echo_out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.echo_out
            .set_encoding("rt")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .echo_out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        Ok(())
    }
}

#[cerulion_node]
struct LatencyNode {
    #[input(trigger)]
    echo_in: Image,
}

#[cerulion_node_impl]
impl LatencyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the input so the read path runs; no heap work.
        let _ = self.echo_in.height;
        LATENCY_FIRES.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// The virtual-time cadence the iceoryx2 liveliness sweep is pushed to
/// so its registry scan can never fire during a MEASURED window. One hour of the
/// 1ms-per-`step()` virtual clock — unreachable by any window here.
const SWEEP_DISABLED_MS: u64 = 3_600_000;

/// Exclude the iceoryx2 liveliness sweep from an alloc measurement.
///
/// The sweep's `cleanup_dead_nodes` is an iceoryx2-INTERNAL node-registry scan
/// that allocates a large, fixed amount per sweep (~13.8k observed, scaling with
/// registry size) — entirely OUTSIDE Cerulion's per-step executor hot path.
/// Cerulion's own `liveliness_sweep` body is pure `u64` accumulator math + atomic
/// publisher-count reads; ONLY the cadence-gated (4 Hz at the 250ms production
/// period) `cleanup_dead_nodes()` call allocates, and that allocation happens
/// inside iceoryx2. It is a background task, EXCLUDED from the steady-state
/// per-step zero-alloc moat claim these tests guard.
///
/// Without this the moat gate dodges the scan only by clock-phase luck: warmup
/// (~750 steps of 1ms virtual time) leaves the 250ms sweep accumulator at a phase
/// where the 200-step measured window happens not to cross a sweep boundary. A
/// different warmup count, a tightened window, or a registry-size change could
/// land a sweep inside the measured window and SPURIOUSLY fail the gate (or, if
/// the executor regressed at the same time, mask it). Pushing the cadence past
/// the whole virtual-time horizon makes the per-step measurement robust + exact:
/// the sweep then never reaches its `accum >= period` gate, so the measured steps
/// pay only the (alloc-free) accumulator add + compare.
fn disable_liveliness_sweep(runtime: &mut GraphRuntime) {
    runtime.set_liveliness_sweep_period_for_test(SWEEP_DISABLED_MS);
    // The dead-node reclaim walk runs on its own thread at its own cadence
    // (`LIVELINESS_CLEANUP_PERIOD_MS`); park it too, so the PROCESS-WIDE gate
    // (which counts every thread) cannot catch one of its iceoryx2-internal
    // allocation bursts inside a measured window.
    runtime.set_liveliness_cleanup_period_for_test(SWEEP_DISABLED_MS);
}

fn build_moat_runtime() -> GraphRuntime {
    let prefix = unique_prefix("moat");
    let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
    let max_slice_len = 32 + 64 + 8 * 3 + n + 256;
    let yaml = format!(
        r#"
name: step_alloc_gate
prefix: {prefix}
nodes:
  - id: ping
    type: ping_node
    outputs:
      - name: ping_out
        schema: sensor_msgs/Image
        max_slice_len: {max_slice_len}
  - id: pong
    type: pong_node
    inputs:
      - name: ping_in
        source: ping/ping_out
    outputs:
      - name: echo_out
        schema: sensor_msgs/Image
        max_slice_len: {max_slice_len}
  - id: latency
    type: latency_node
    inputs:
      - name: echo_in
        source: pong/echo_out
"#
    );

    let config = parse_graph(&yaml).expect("parse graph");
    validate_graph(&config).expect("validate graph");
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let clock = Arc::new(VirtualClock::new());
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("ping".to_string(), Box::new(PingNodeEntry::new()));
    nodes.insert("pong".to_string(), Box::new(PongNodeEntry::new()));
    nodes.insert("latency".to_string(), Box::new(LatencyNodeEntry::new()));
    GraphRuntime::build(config, nodes, &mgr, clock).expect("build graph")
}

/// Drive the runtime until the 3-node chain is at steady state (LatencyNode
/// firing every step), then count heap allocations over a measured window of
/// `step()` calls. Returns total allocations in the window.
fn measure_step_allocs(measure_steps: usize) -> u64 {
    let mut runtime = build_moat_runtime();
    disable_liveliness_sweep(&mut runtime);

    // Warm: run until LatencyNode has fired enough that the whole chain
    // collapsed to one tick per step (pong + latency both flowing) AND the
    // scheduler's unbounded execution-trace `VecDeque` has grown its capacity
    // well past the measurement window's needs. The trace defaults to
    // unbounded (`max_trace_entries: None`), so it reallocates at capacity
    // doublings as fires accumulate — that growth is a SEPARATE concern from
    // the per-step EXECUTOR allocation this test guards. We warm ~700
    // round-trips (~2100 trace entries → capacity ≥ 4096), then `clear_trace()`
    // below resets the trace LENGTH to 0 while PRESERVING that grown capacity
    // (`VecDeque::clear`), so the measurement window's ~600 entries (200 steps ×
    // 3 fires) push in without any reallocation — isolating the executor path.
    LATENCY_FIRES.store(0, Ordering::SeqCst);
    let mut warm = 0usize;
    while LATENCY_FIRES.load(Ordering::Relaxed) < 700 {
        runtime.step(Duration::from_millis(1));
        warm += 1;
        assert!(warm < 10_000, "moat chain did not reach steady state");
    }
    // A few more to be sure caches / SHM pool / IndexMap capacity are warm.
    for _ in 0..50 {
        runtime.step(Duration::from_millis(1));
    }
    // Reset trace length (keeps the grown capacity) so trace-buffer growth does
    // not masquerade as a per-step executor allocation.
    runtime.clear_trace();

    ALLOCATOR.enable();
    for _ in 0..measure_steps {
        runtime.step(Duration::from_millis(1));
    }
    let allocs = ALLOCATOR.disable();

    runtime.shutdown();
    allocs
}

#[test]
#[serial]
fn test_graph_step_is_zero_alloc_at_steady_state() {
    PAYLOAD_SIZE.store(64, Ordering::Relaxed);
    const MEASURE_STEPS: usize = 200;
    let allocs = measure_step_allocs(MEASURE_STEPS);
    let per_step = allocs as f64 / MEASURE_STEPS as f64;
    println!(
        "step_zero_alloc: {allocs} heap allocations over {MEASURE_STEPS} steady-state \
         GraphRuntime::step() calls ({per_step:.3} allocs/step)"
    );

    // The level executor must allocate ZERO times per step at steady
    // state (the moat chain fires ping[Period]→pong[Data]→latency[Data] every
    // step with no payload growth). A non-zero count is a Principle-1 hot-path
    // allocation regression. (NOTE per the header: these allocs do NOT set
    // the user-POV p50 — that is transport-bound — but an alloc-free executor
    // is the correct Principle-1 property and this is its guard.)
    assert_eq!(
        allocs, 0,
        "GraphRuntime::step() allocated {allocs} times over {MEASURE_STEPS} steady-state \
         steps ({per_step:.3} allocs/step) — the level executor must be zero-alloc on the \
         hot path; a per-level Vec / fire_times / id-clone allocation reappeared."
    );
}

// ---------------------------------------------------------------------------
// Block-graph zero-alloc proof
//
// The moat test above exercises the NO-BLOCK fast-path (no `block` topics →
// `LevelPlan::block_ids` empty for every level, the partition is trivially
// `level.nodes`). This second graph makes the partition NON-trivial: a `block`
// consumer makes both it AND its upstream producer block-involved, so every
// level carries a non-empty `LevelPlan::block_ids` and `step()` runs the
// `evaluate_nodes_fused` seam. A `step()` that rebuilds that
// partition into two scratch `Vec<String>` per level per step, cloning each
// node-id `String`, pays non-zero allocs on the block path. The
// partition is precomputed at build and merely BORROWED, so the block path is
// zero-alloc too.
//
// # Topology
//
// ```text
//  BlockPingNode ── bp_out ──▶ BlockConsumerNode
//  (period_ms=1, L0)           (data-trigger block input `inp`, L1)
// ```
//
// Both fire EVERY step at steady state: the producer publishes one Vector3 per
// step, and the data-trigger consumer drains exactly one per step via the
// macro's `try_view`. With a depth-2 `block` input that 1:1 produce:drain ratio
// keeps `outstanding` ≤ 1 (< depth) so the producer's `block` pre-fire check
// NEVER defers — a defer would emit a `tracing::warn!` (which allocates) and
// also break the byte-identity contract. The result is exactly-zero allocs over
// the measured window, identical to the moat path.
// ---------------------------------------------------------------------------

/// How many times the block consumer has fired (drives steady-state detect).
static BLOCK_CONSUMER_FIRES: AtomicU64 = AtomicU64::new(0);

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct BlockPingNode {
    #[output]
    bp_out: native_ros2_messages::geometry_msgs::Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl BlockPingNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.bp_out.x = self.n as f64;
        Ok(())
    }
}

#[cerulion_node]
#[derive(Default)]
struct BlockConsumerNode {
    // `block` + `trigger`: the consumer fires on each arrival (data-trigger) and
    // the macro auto-drains one frame per tick via `try_view`, decrementing the
    // shared block `outstanding` mirror. This makes BOTH this node and its
    // upstream producer block-involved → every level has a non-empty
    // `LevelPlan::block_ids`, so `step()` exercises the fused block path.
    #[input(backpressure = block, trigger, depth = 2)]
    inp: native_ros2_messages::geometry_msgs::Vector3,
}

#[cerulion_node_impl]
impl BlockConsumerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the input so the read path runs; no heap work.
        let _ = self.inp.x;
        BLOCK_CONSUMER_FIRES.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

fn build_block_runtime() -> GraphRuntime {
    let prefix = unique_prefix("block");
    let yaml = format!(
        r#"
name: step_block_alloc_gate
prefix: {prefix}
nodes:
  - id: bping
    type: block_ping_node
    outputs:
      - name: bp_out
        schema: geometry_msgs/Vector3
  - id: bconsumer
    type: block_consumer_node
    inputs:
      - name: inp
        source: bping/bp_out
"#
    );

    let config = parse_graph(&yaml).expect("parse block graph");
    validate_graph(&config).expect("validate block graph");
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let clock = Arc::new(VirtualClock::new());
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("bping".to_string(), Box::new(BlockPingNodeEntry::new()));
    nodes.insert(
        "bconsumer".to_string(),
        Box::new(BlockConsumerNodeEntry::new()),
    );
    GraphRuntime::build(config, nodes, &mgr, clock).expect("build block graph")
}

/// Drive the block runtime to steady state, then count heap allocations over a
/// measured window of `step()` calls. Same warm/clear_trace discipline as
/// `measure_step_allocs`.
fn measure_block_step_allocs(measure_steps: usize) -> (u64, u64) {
    let mut runtime = build_block_runtime();
    disable_liveliness_sweep(&mut runtime);

    BLOCK_CONSUMER_FIRES.store(0, Ordering::SeqCst);
    let mut warm = 0usize;
    while BLOCK_CONSUMER_FIRES.load(Ordering::Relaxed) < 700 {
        runtime.step(Duration::from_millis(1));
        warm += 1;
        assert!(warm < 10_000, "block chain did not reach steady state");
    }
    for _ in 0..50 {
        runtime.step(Duration::from_millis(1));
    }
    runtime.clear_trace();

    ALLOCATOR.enable();
    for _ in 0..measure_steps {
        runtime.step(Duration::from_millis(1));
    }
    let allocs = ALLOCATOR.disable();

    // The producer's `block` pre-fire must NEVER have deferred (a defer warns =>
    // allocates AND would invalidate the zero-alloc claim). Read it AFTER the
    // allocator window is disabled so the read itself is never counted.
    let deferred = runtime
        .node_handle("bconsumer")
        .expect("bconsumer handle")
        .backpressure_block_fires_deferred_count("inp");

    runtime.shutdown();
    (allocs, deferred)
}

#[test]
#[serial]
fn test_block_graph_step_is_zero_alloc_at_steady_state() {
    const MEASURE_STEPS: usize = 200;
    let (allocs, deferred) = measure_block_step_allocs(MEASURE_STEPS);
    let per_step = allocs as f64 / MEASURE_STEPS as f64;
    println!(
        "block_step_zero_alloc: {allocs} heap allocations over {MEASURE_STEPS} steady-state \
         GraphRuntime::step() calls ({per_step:.3} allocs/step), {deferred} block defers"
    );

    // Steady-state sanity: the 1:1 produce:drain ratio on a depth-2 block input
    // must keep `outstanding` below depth, so the producer never defers. A defer
    // here would mean the consumer fell behind (a test-harness bug, not the
    // thing under test) AND would itself allocate via the defer warn path.
    assert_eq!(
        deferred, 0,
        "block producer deferred {deferred} times at steady state — the consumer must drain \
         every step so the block pre-fire check never trips (a defer warns => allocates and \
         breaks the zero-alloc precondition)"
    );

    // The BLOCK partition path must allocate ZERO times
    // per step at steady state — same Principle-1 contract as the moat path. The
    // partition is precomputed at build and borrowed in `step()`; rebuilding
    // it into two scratch Vec per level, cloning each node-id String, is the
    // regression. A non-zero count means the per-level partition allocation
    // reappeared on the block path.
    assert_eq!(
        allocs, 0,
        "GraphRuntime::step() allocated {allocs} times over {MEASURE_STEPS} steady-state steps \
         ({per_step:.3} allocs/step) on a BLOCK graph — the level executor's block/non-block \
         partition must be precomputed at build and borrowed, not rebuilt+cloned per step."
    );
}

// ---------------------------------------------------------------------------
// MULTI-NODE (>= 2-fire) LEVEL zero-alloc proof.
//
// The moat + block graphs above have only SINGLE-NODE DAG levels (ping[L0] →
// pong[L1] → latency[L2]; bping[L0] → bconsumer[L1]) — exactly ONE fire per
// level — so every `step()` only ever hits `tick_decided_parallel`'s
// `decisions.len() <= 1` FAST-PATH (`tick_decided`). They NEVER exercise the
// >= 2-fire scratch / fragment / decision-order-merge machinery.
//
// This graph puts THREE independent `Period(1)` sources in ONE wide DAG level 0
// (no inputs → no DAG edges → one 3-wide L0), so every step `decide_fires`
// returns 3 decisions (> 1 → NOT the fast-path). 3 < `PARALLEL_FIRE_THRESHOLD`
// (8) → the non-serial-gated REST fires SERIALLY (no rayon injector), so the
// level is strictly zero-alloc at steady state.
//
// THIS TEST FAILS ON A MULTI-FIRE PATH THAT ALLOCATES PER STEP: one that
// builds a `decided_by_idx: HashMap` + `serial_tasks`/`par_tasks` Vecs + a
// fresh per-node `Vec<TraceEntry>` + a `results` Vec (sorted) PER STEP
// measures 7.0 allocs/step here, against the required 0.
// It is the zero-alloc gate that exercises a >= 2-fire level (the moat/block
// graphs stay on the fast-path and so cannot catch that regression).
// ---------------------------------------------------------------------------

/// Round-trips observed on TriPingA — drives steady-state detection. All three
/// sources are `Period(1)` stepped at 1 ms, so they fire in lockstep every step;
/// counting one is sufficient.
static TRI_A_FIRES: AtomicU64 = AtomicU64::new(0);

#[cerulion_node(period_ms = 1)]
struct TriPingA {
    #[output]
    out: Image,
}

#[cerulion_node_impl]
impl TriPingA {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Mirror PingNode's body verbatim — already proven zero-alloc at steady
        // state by the moat test above (loan_data over a warm SHM pool, fixed
        // fields via Deref, the variable encoding/header writes into SHM).
        let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
        self.out.height = 1;
        self.out.width = 2;
        self.out.step = 0;
        self.out.is_bigendian = 0;
        self.out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.out
            .set_encoding("rt")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        TRI_A_FIRES.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

#[cerulion_node(period_ms = 1)]
struct TriPingB {
    #[output]
    out: Image,
}

#[cerulion_node_impl]
impl TriPingB {
    fn tick(&mut self) -> Result<(), NodeError> {
        let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
        self.out.height = 1;
        self.out.width = 2;
        self.out.step = 0;
        self.out.is_bigendian = 0;
        self.out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.out
            .set_encoding("rt")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        Ok(())
    }
}

#[cerulion_node(period_ms = 1)]
struct TriPingC {
    #[output]
    out: Image,
}

#[cerulion_node_impl]
impl TriPingC {
    fn tick(&mut self) -> Result<(), NodeError> {
        let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
        self.out.height = 1;
        self.out.width = 2;
        self.out.step = 0;
        self.out.is_bigendian = 0;
        self.out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.out
            .set_encoding("rt")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        Ok(())
    }
}

fn build_tri_runtime() -> GraphRuntime {
    let prefix = unique_prefix("tri");
    let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
    let max_slice_len = 32 + 64 + 8 * 3 + n + 256;
    let yaml = format!(
        r#"
name: tri_level_alloc_gate
prefix: {prefix}
nodes:
  - id: tria
    type: tri_ping_a
    outputs:
      - name: out
        schema: sensor_msgs/Image
        max_slice_len: {max_slice_len}
  - id: trib
    type: tri_ping_b
    outputs:
      - name: out
        schema: sensor_msgs/Image
        max_slice_len: {max_slice_len}
  - id: tric
    type: tri_ping_c
    outputs:
      - name: out
        schema: sensor_msgs/Image
        max_slice_len: {max_slice_len}
"#
    );

    let config = parse_graph(&yaml).expect("parse tri graph");
    validate_graph(&config).expect("validate tri graph");
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let clock = Arc::new(VirtualClock::new());
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("tria".to_string(), Box::new(TriPingAEntry::new()));
    nodes.insert("trib".to_string(), Box::new(TriPingBEntry::new()));
    nodes.insert("tric".to_string(), Box::new(TriPingCEntry::new()));
    GraphRuntime::build(config, nodes, &mgr, clock).expect("build tri graph")
}

/// Same warm/clear_trace discipline as `measure_step_allocs`, on the 3-wide
/// single-level graph.
fn measure_tri_step_allocs(measure_steps: usize) -> u64 {
    let mut runtime = build_tri_runtime();
    disable_liveliness_sweep(&mut runtime);

    TRI_A_FIRES.store(0, Ordering::SeqCst);
    let mut warm = 0usize;
    while TRI_A_FIRES.load(Ordering::Relaxed) < 700 {
        runtime.step(Duration::from_millis(1));
        warm += 1;
        assert!(warm < 10_000, "tri level did not reach steady state");
    }
    for _ in 0..50 {
        runtime.step(Duration::from_millis(1));
    }
    runtime.clear_trace();

    ALLOCATOR.enable();
    for _ in 0..measure_steps {
        runtime.step(Duration::from_millis(1));
    }
    let allocs = ALLOCATOR.disable();

    runtime.shutdown();
    allocs
}

#[test]
#[serial]
fn test_multi_node_level_step_is_zero_alloc_at_steady_state() {
    PAYLOAD_SIZE.store(64, Ordering::Relaxed);
    const MEASURE_STEPS: usize = 200;
    let allocs = measure_tri_step_allocs(MEASURE_STEPS);
    let per_step = allocs as f64 / MEASURE_STEPS as f64;
    println!(
        "tri_level_zero_alloc: {allocs} heap allocations over {MEASURE_STEPS} steady-state \
         GraphRuntime::step() calls ({per_step:.3} allocs/step) on a 3-wide single-level graph"
    );

    // A >= 2-fire level (3 < THRESHOLD → narrow serial REST) must
    // allocate ZERO times per step at steady state. A non-zero count means
    // per-step multi-fire scratch (a decided_by_idx HashMap /
    // serial+par task Vecs / per-node Vec::new() fragment / results sort)
    // reappeared. This is the gate that exercises the >= 2-fire path — it
    // FAILS on any executor that allocates on every multi-fire level.
    assert_eq!(
        allocs, 0,
        "GraphRuntime::step() allocated {allocs} times over {MEASURE_STEPS} steady-state steps \
         ({per_step:.3} allocs/step) on a 3-wide single-level graph — the multi-fire level \
         executor must be zero-alloc on the hot path: the index-keyed scratch + \
         per-node trace_fragment must be reused, not reallocated, per step."
    );
}

// ---------------------------------------------------------------------------
// SERIAL-GATED NARROW level zero-alloc proof.
//
// The tri test above has an all-parallel-class (`any_serial == false`) narrow
// level — PASS 1 is skipped entirely. This graph adds the complementary
// coverage: a narrow level that INCLUDES a serial-gated node, so PASS 1 (the
// serial-gated, calling-thread fire) runs alongside the serial REST.
//
// A `ClosureNodeEntry` carrying a NON-trigger plain input has
// `performs_input_snapshot() == false` AND appears in `snapshot_input_names`
// (Period ⇒ all inputs latest-value) → the build routes it into
// `serial_fire_node_ids` (set A). The non-trigger input is NOT a DAG edge
// (topology.rs), so the gated closure stays a LEVEL-0 ROOT in the SAME level as
// its `feeder` producer → a 2-wide level 0. `decisions.len() == 2` (> 1, NOT
// the fast-path) and 2 < THRESHOLD(8) → narrow: the gated closure fires in
// PASS 1, the feeder in the serial PASS 2. Both must be zero-alloc at steady
// state (PASS 1 + serial REST path).
// ---------------------------------------------------------------------------

/// Ticks observed on the gated closure — drives steady-state detection.
static GATED_FIRES: AtomicU64 = AtomicU64::new(0);

/// A `block_ping_node` (`BlockPingNode`, defined above) source emitting a
/// `Vector3` every step — reused as the gated closure's upstream feeder.
fn vec_feeder_def(id: &str) -> NodeDef {
    NodeDef {
        ros2: None,
        id: id.to_string(),
        node_type: "block_ping_node".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "bp_out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }
}

/// A plain (DropOldest) NON-trigger input meta — the property that makes the
/// closure serial-gated (set A of `serial_fire_node_ids`).
fn gated_plain_input_meta(name: &str) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: false,
        depth: 8,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

fn build_gated_runtime() -> GraphRuntime {
    let prefix = unique_prefix("gated");

    let nodes: Vec<NodeDef> = vec![
        vec_feeder_def("feeder"),
        // The serial-gated closure consumer at level 0 (non-trigger input → no
        // DAG edge → stays a level-0 root sharing the level with `feeder`).
        NodeDef {
            ros2: None,
            id: "gated".to_string(),
            node_type: "gated_closure".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "feeder/bp_out".to_string(),
            }],
            outputs: vec![],
        },
    ];
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "gated_narrow_alloc_gate".to_string(),
        prefix,
        nodes,
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("feeder".to_string(), Box::new(BlockPingNodeEntry::new()));
    // Period(1) + a plain non-trigger input → no-op snapshot + present in
    // snapshot_input_names → routed to serial_fire_node_ids (set A). Fires
    // serially on the calling thread in PASS 1.
    let info = NodeInfo::with_meta(vec![gated_plain_input_meta("inp")], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let closure = ClosureNodeEntry::new(info, move |ctx| {
        // A real iceoryx2 read every tick (no fake data); the value is ignored.
        // `try_view` is zero-alloc (proven by zero_alloc_test) — this just
        // proves the gated node's tick actually runs on the measured steps.
        if let Some(sub) = ctx.subscriber_mut("inp") {
            let _ = sub.try_view::<Vector3, _>(|view| view.x);
        }
        GATED_FIRES.fetch_add(1, Ordering::Relaxed);
        Ok(())
    })
    .with_label("gated_closure");
    // DECISIVE for PASS 1 (not just "narrow REST"): pin that this node is
    // genuinely serial-gated. The runtime routes a node to `serial_fire_node_ids`
    // iff `!performs_input_snapshot()` AND it has a snapshotted (non-trigger)
    // input. A `ClosureNodeEntry` reports `false`; if a future change flipped
    // that, the node would be demoted to the (also-zero-alloc) REST path and this
    // test would silently stop exercising the serial-gated PASS 1 fire it claims
    // to cover. Assert the classifying property so that demotion fails loudly.
    assert!(
        !closure.performs_input_snapshot(),
        "the gated closure must be serial-gated (performs_input_snapshot()==false) so it routes to \
         PASS 1; otherwise this test exercises the narrow REST path, not the serial-gated path"
    );
    factories.insert("gated".to_string(), Box::new(closure));

    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build gated graph");

    // ROUTING PIN: assert the RUNTIME actually routed `gated` onto the serial
    // (PASS 1) path — not just the node-property proxy (`performs_input_snapshot`)
    // asserted above. This pins the end-to-end classification: if a future change
    // dropped `gated` from `serial_fire_node_ids` (e.g. the snapshot-input wiring
    // regressed), this test would SILENTLY measure the REST path instead of the
    // serial-gated PASS 1 fire it claims to cover.
    assert!(
        runtime.serial_fire_node_ids().contains("gated"),
        "the runtime must route `gated` to the serial PASS-1 set; serial_fire_node_ids = {:?}",
        runtime.serial_fire_node_ids()
    );
    runtime
}

/// Same warm/clear_trace discipline as `measure_step_allocs`, on the 2-wide
/// serial-gated level.
fn measure_gated_step_allocs(measure_steps: usize) -> u64 {
    let mut runtime = build_gated_runtime();
    disable_liveliness_sweep(&mut runtime);

    GATED_FIRES.store(0, Ordering::SeqCst);
    let mut warm = 0usize;
    while GATED_FIRES.load(Ordering::Relaxed) < 700 {
        runtime.step(Duration::from_millis(1));
        warm += 1;
        assert!(warm < 10_000, "gated level did not reach steady state");
    }
    for _ in 0..50 {
        runtime.step(Duration::from_millis(1));
    }
    runtime.clear_trace();

    ALLOCATOR.enable();
    for _ in 0..measure_steps {
        runtime.step(Duration::from_millis(1));
    }
    let allocs = ALLOCATOR.disable();

    runtime.shutdown();
    allocs
}

#[test]
#[serial]
fn test_serial_gated_narrow_level_is_zero_alloc() {
    const MEASURE_STEPS: usize = 200;
    let allocs = measure_gated_step_allocs(MEASURE_STEPS);
    let per_step = allocs as f64 / MEASURE_STEPS as f64;
    println!(
        "gated_narrow_zero_alloc: {allocs} heap allocations over {MEASURE_STEPS} steady-state \
         GraphRuntime::step() calls ({per_step:.3} allocs/step) on a 2-wide serial-gated level"
    );

    // Non-vacuous: the gated closure actually ran during the measured window
    // (it kept firing — Period(1), one tick/step). Read after disable so the
    // read is never counted.
    assert!(
        GATED_FIRES.load(Ordering::Relaxed) >= 700 + 50 + MEASURE_STEPS as u64,
        "the serial-gated closure must have fired every step (Period(1)) — the \
         PASS 1 serial-gated path must actually run on the measured steps"
    );

    // A serial-gated narrow level (PASS 1 serial-gated fire +
    // serial PASS 2 REST) must allocate ZERO times per step at steady state.
    // A non-zero count means the multi-fire scratch / per-node trace_fragment
    // reuse broke on the serial-gated path.
    assert_eq!(
        allocs, 0,
        "GraphRuntime::step() allocated {allocs} times over {MEASURE_STEPS} steady-state steps \
         ({per_step:.3} allocs/step) on a 2-wide SERIAL-GATED level — the PASS 1 serial-gated \
         fire + serial REST must be zero-alloc: the index-keyed scratch + per-node \
         trace_fragment reuse must hold through the serial-gated path too."
    );
}

// ---------------------------------------------------------------------------
// WIDE (>= PARALLEL_FIRE_THRESHOLD) level — Cerulion-side zero-alloc.
//
// The tri / serial-gated gates above cover the NARROW (< 8 non-serial REST)
// branch, which is strictly zero-alloc (serial REST, no rayon). This gate covers
// the complementary WIDE branch: a level whose non-serial REST is >= 8 fans out
// via `pool.install(par_values_mut().for_each(..))`. That branch CANNOT be
// strictly zero-alloc — dispatching to rayon from the (non-worker) caller thread
// amortizes ~1 crossbeam-deque `Injector` block alloc per ~63 installs, an
// intrinsic, NODE-COUNT-INDEPENDENT residual no safe design avoids.
//
// What IS guaranteed is ZERO Cerulion-side allocs:
// the wide body builds no per-step `HashMap` + serial/par task
// Vecs + `.collect()` + per-node `Vec<TraceEntry>`. The decisive proof is
// NODE-COUNT INDEPENDENCE: a per-node / per-fire Cerulion alloc would scale O(n)
// with level width, so a 16-wide level would allocate ~2x a 8-wide one. We assert
// the wide alloc count (a) stays far below the O(n) floor and (b) does NOT
// grow materially from 8-wide to 16-wide — only the fixed injector residual.
//
// `CERULION_FIRE_THREADS` is forced >= 2 so the wide rayon path engages even on a
// single-core runner (where the auto-size would give a 1-thread pool that the
// `current_num_threads() <= 1` clause routes to the serial REST).
// ---------------------------------------------------------------------------

/// RAII guard restoring `CERULION_FIRE_THREADS` on drop (even on panic). Read at
/// build time; paired with `#[serial]` since env is process-global. Mirrors
/// `rayon_fire_iox2_test`'s `FireThreadsGuard`.
struct FireThreadsGuard;
impl FireThreadsGuard {
    fn set(value: &str) -> Self {
        std::env::set_var("CERULION_FIRE_THREADS", value);
        Self
    }
}
impl Drop for FireThreadsGuard {
    fn drop(&mut self) {
        std::env::remove_var("CERULION_FIRE_THREADS");
    }
}

/// Build a single DAG level of `width` independent `tri_ping_a` `Period(1)`
/// sources (the SAME node type instantiated `width` times with distinct ids →
/// distinct output topics → no single-writer collision; no inputs → no DAG edges
/// → ONE `width`-wide level 0). Reuses `TriPingAEntry` so no extra node types.
fn build_wide_runtime(width: usize) -> GraphRuntime {
    let prefix = unique_prefix("wide");
    let n = PAYLOAD_SIZE.load(Ordering::Relaxed);
    let max_slice_len = 32 + 64 + 8 * 3 + n + 256;
    let mut yaml = format!("name: wide_level_alloc_gate\nprefix: {prefix}\nnodes:\n");
    for i in 0..width {
        yaml.push_str(&format!(
            "  - id: ws{i}\n    type: tri_ping_a\n    outputs:\n      - name: out\n        schema: sensor_msgs/Image\n        max_slice_len: {max_slice_len}\n"
        ));
    }
    let config = parse_graph(&yaml).expect("parse wide graph");
    validate_graph(&config).expect("validate wide graph");
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let clock = Arc::new(VirtualClock::new());
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for i in 0..width {
        nodes.insert(format!("ws{i}"), Box::new(TriPingAEntry::new()));
    }
    let runtime = GraphRuntime::build(config, nodes, &mgr, clock).expect("build wide graph");

    // ROUTING PIN (close the green-wash): this test only proves the WIDE rayon
    // `par_values_mut` path is zero-alloc if the level ACTUALLY routes wide. Two
    // conditions gate that in `tick_decided_parallel`: a multi-thread pool AND a
    // non-serial REST `>= PARALLEL_FIRE_THRESHOLD`. All `width` sources are
    // trigger-only Period nodes (no serial-gating) → the REST == width. If a
    // future threshold bump pushed the threshold above this `width`, the level
    // would SILENTLY route narrow (serial) and pass trivially without exercising
    // rayon. Assert both so that regression fails LOUDLY here.
    let threshold = GraphRuntime::parallel_fire_threshold();
    assert!(
        runtime.fire_pool_thread_count() > 1,
        "the wide test must route WIDE: fire pool has only {} thread(s) — a single-thread pool \
         forces the narrow serial path regardless of width (is CERULION_FIRE_THREADS forced >= 2?)",
        runtime.fire_pool_thread_count()
    );
    assert!(
        width >= threshold,
        "the wide test must route WIDE: level width {width} must be >= PARALLEL_FIRE_THRESHOLD \
         ({threshold}); if PARALLEL_FIRE_THRESHOLD was raised above {width}, widen this test"
    );
    runtime
}

/// Measure heap allocs over a steady-state window on a `width`-wide level with
/// the fire pool forced to 4 threads (so the wide rayon REST engages). Warms by
/// a fixed step count (pure Period(1) sources fire from step 1 — no dependency
/// chain to settle) so the trace `VecDeque` grows past the measured window before
/// `clear_trace`.
fn measure_wide_step_allocs(width: usize, measure_steps: usize) -> u64 {
    let _threads = FireThreadsGuard::set("4");
    let mut runtime = build_wide_runtime(width);
    disable_liveliness_sweep(&mut runtime);

    // Warm: each source fires every step; 850 steps grows the trace capacity well
    // past the measured 200*width pushes and warms the SHM pools.
    for _ in 0..850 {
        runtime.step(Duration::from_millis(1));
    }
    runtime.clear_trace();

    // PROCESS-WIDE (not thread-scoped): the wide level's REST fires on rayon
    // WORKER threads via `pool.install(par_values_mut(..))`, so a thread-scoped
    // window (which counts only the calling thread) would blind this gate. The
    // test tolerates the resulting background/injector noise via a `< 64` bound +
    // node-count independence rather than `== 0`.
    ALLOCATOR.enable_process_wide();
    for _ in 0..measure_steps {
        runtime.step(Duration::from_millis(1));
    }
    let allocs = ALLOCATOR.disable();

    runtime.shutdown();
    allocs
}

#[test]
#[serial]
fn test_wide_level_is_cerulion_side_zero_alloc() {
    PAYLOAD_SIZE.store(64, Ordering::Relaxed);
    const MEASURE_STEPS: usize = 200;
    // 8 == PARALLEL_FIRE_THRESHOLD: 8 non-serial fires routes WIDE (>= threshold).
    let allocs_8 = measure_wide_step_allocs(8, MEASURE_STEPS);
    // 16-wide: if a per-node / per-fire Cerulion alloc regressed, this ~doubles.
    let allocs_16 = measure_wide_step_allocs(16, MEASURE_STEPS);
    println!(
        "wide_zero_alloc: 8-wide={allocs_8}, 16-wide={allocs_16} heap allocs over {MEASURE_STEPS} \
         steady-state steps (only the node-count-INDEPENDENT crossbeam-injector residual is allowed)"
    );

    // (a) Far below the O(n) wide-body floor. A wide path that allocates
    // a HashMap + serial/par task Vecs + a `.collect()` + a per-node Vec EVERY
    // step costs ~(2*width + 4)/step → ~3600 (8-wide) / ~7200 (16-wide) over 200
    // steps. The injector residual is ~1 per ~63 installs → ~3-5 over 200. A
    // bound of 64 cleanly separates "our-side-zero + injector" from any regression.
    const OUR_SIDE_ZERO_BOUND: u64 = 64;
    assert!(
        allocs_8 < OUR_SIDE_ZERO_BOUND && allocs_16 < OUR_SIDE_ZERO_BOUND,
        "wide-path allocs (8-wide={allocs_8}, 16-wide={allocs_16}) must stay below {OUR_SIDE_ZERO_BOUND} \
         (only rayon's ~1/63 injector residual). A higher count means a per-step Cerulion alloc \
         (HashMap / task Vec / collect / per-node Vec) regressed onto the wide path."
    );

    // (b) THE decisive pin — NODE-COUNT INDEPENDENCE. A per-node/per-fire Cerulion
    // alloc scales O(width): 16-wide would allocate ~2x 8-wide. The injector
    // residual is ONE install/step regardless of width, so doubling width must add
    // at most injector/epoch jitter — NOT a width-proportional amount.
    const WIDTH_JITTER_TOLERANCE: u64 = 32;
    assert!(
        allocs_16 <= allocs_8 + WIDTH_JITTER_TOLERANCE,
        "doubling the level width (8 -> 16) raised wide-path allocs from {allocs_8} to {allocs_16} \
         (> {WIDTH_JITTER_TOLERANCE} jitter tolerance) — the wide path is allocating PER NODE, not just \
         the fixed injector residual; the par_values_mut/trace_fragment reuse regressed to an O(n) \
         per-step allocation (the wide-path 'our-side-zero' claim broken)."
    );
}

// ---------------------------------------------------------------------------
// Sanity control for the two-mode counter.
//
// The narrow/serial gates above changed from process-wide to THREAD-SCOPED
// counting to fix the CI flake (a background-thread alloc was falsely attributed
// to `step()`). Thread-scoping introduces its OWN false-green risk: if the
// scoping accidentally made the counter count NOTHING, every `== 0` gate above
// would pass vacuously. This control (mirroring `shm_ring_zero_alloc_test.rs`'s
// phase-3 control, extended for the two modes) pins the apparatus itself:
//
//  (1) BITES        — a deliberate allocation on the MEASURED (calling) thread
//                     counts exactly 1 in thread-scoped mode (the counter still
//                     bites; the `== 0` gates are non-vacuous).
//  (2) IMMUNE       — an allocation on a NON-measured background thread does NOT
//                     count in thread-scoped mode (the new anti-flake property —
//                     the exact immunity that fixes the flake).
//  (3) PW BITES     — that same background allocation DOES count when the window
//                     is PROCESS-WIDE (proves the two modes are genuinely
//                     distinct, so the wide test isn't silently thread-scoped).
//
// Inside every measured window the calling thread touches only atomics + a
// spin-loop (no heap alloc), so the ONLY calling-thread allocation counted is
// the one deliberate probe in (1). All setup that allocates (Arc / thread spawn
// / join) happens OUTSIDE the window.
// ---------------------------------------------------------------------------

/// Spawn a background thread that, once released via `go`, allocates `n` times
/// (on a NON-measured thread) and then signals `done`. All of `spawn`'s own
/// allocation happens on the CALLING thread here — OUTSIDE any measured window.
fn spawn_bg_allocator(
    go: &Arc<AtomicBool>,
    done: &Arc<AtomicBool>,
    n: u32,
) -> std::thread::JoinHandle<()> {
    let go = Arc::clone(go);
    let done = Arc::clone(done);
    std::thread::spawn(move || {
        while !go.load(Ordering::Acquire) {
            std::hint::spin_loop();
        }
        for _ in 0..n {
            let v: Vec<u8> = Vec::with_capacity(64);
            std::hint::black_box(v);
        }
        done.store(true, Ordering::Release);
    })
}

#[test]
#[serial]
fn test_thread_scoped_counter_bites_and_is_background_immune() {
    // --- (1) BITES: a measured-thread alloc counts exactly 1 (thread-scoped) ---
    ALLOCATOR.enable();
    let v: Vec<u8> = Vec::with_capacity(1);
    std::hint::black_box(v);
    let allocs = ALLOCATOR.disable();
    assert_eq!(
        allocs, 1,
        "a deliberate measured-thread allocation must count exactly 1 in thread-scoped mode \
         (got {allocs}) — the thread-scoped counter must still BITE, else every `== 0` gate \
         above passes vacuously"
    );

    // A fresh empty window counts 0 (belt-and-suspenders; matches shm_ring).
    ALLOCATOR.enable();
    let allocs = ALLOCATOR.disable();
    assert_eq!(
        allocs, 0,
        "an empty fresh window must count 0 (got {allocs})"
    );

    // --- (2) IMMUNE: a background-thread alloc does NOT count (thread-scoped) ---
    // The background thread allocates 1000 times DURING the open window; the
    // measured (main) thread touches only atomics + spin. All allocating setup
    // (Arc / spawn / join) is outside the window.
    {
        const BG_ALLOCS: u32 = 1000;
        let go = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let handle = spawn_bg_allocator(&go, &done, BG_ALLOCS);

        ALLOCATOR.enable(); // thread-scoped: arms the MAIN thread only
        go.store(true, Ordering::Release); // release the background thread
        while !done.load(Ordering::Acquire) {
            std::hint::spin_loop();
        }
        let bg_allocs = ALLOCATOR.disable();
        handle.join().expect("bg thread join"); // join AFTER disable → uncounted

        assert_eq!(
            bg_allocs, 0,
            "background-thread allocations must NOT count in thread-scoped mode (got {bg_allocs}) \
             — this is the anti-flake immunity: an allocation on a background thread landing inside \
             a measured window must not be attributed to step()"
        );
    }

    // --- (3) PROCESS-WIDE bites: background allocs DO count process-wide ---
    // Anti-tautology for the wide gate's mode: prove process-wide is genuinely
    // distinct from thread-scoped (else the wide gate would be silently blinded).
    {
        const BG_ALLOCS: u32 = 1000;
        let go = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let handle = spawn_bg_allocator(&go, &done, BG_ALLOCS);

        ALLOCATOR.enable_process_wide(); // count EVERY thread
        go.store(true, Ordering::Release);
        while !done.load(Ordering::Acquire) {
            std::hint::spin_loop();
        }
        let pw_allocs = ALLOCATOR.disable();
        handle.join().expect("bg thread join");

        assert!(
            pw_allocs >= u64::from(BG_ALLOCS),
            "process-wide mode must count background-thread allocations (got {pw_allocs}, expected \
             >= {BG_ALLOCS}) — the wide rayon gate relies on process-wide counting; a failure here \
             means the two modes are not actually distinct and the wide gate is silently \
             thread-scoped"
        );
    }
}
