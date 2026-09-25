// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity cluster 1: behavioral proof that the type-routed `#[on_event]`
//! handler DISPATCH fires INSIDE a `DylibNodeEntry`-loaded cdylib — the
//! resolution of inert-shipping instance #7.
//!
//! # The gap this closes
//!
//! The `#[on_event]` dispatch machinery is emitted into the cdylib's
//! `__cer_zero_copy_tick` (guarded by the tick-Ok outcome), and each handler
//! drains its event from the `NodeContext` that `DylibNodeEntry::init` moves
//! across the FFI. But nothing ever proved that dispatch FIRES over the FFI: no
//! cdylib fixture declared a single `#[on_event]` handler, and `cdylib_qos_ffi`
//! only round-trips the QoS *metadata*. Every in-process `on_event_*` suite
//! loads its consumer as an in-process macro node.
//!
//! This suite loads `test_node_macro_onevent_cdylib` (one handler per reactive
//! kind) through `DylibNodeEntry` into a real-iceoryx2 `build_for_test` graph
//! and drives each of the four event kinds. The cdylib's handler counters
//! cannot be read directly (opaque Rust state across the FFI), so the fixture
//! publishes them into a fixed `Quaternion` output (x=bp, y=expect, z=live,
//! w=promise) that an in-process sink reads back. A counter crossing the SHM
//! boundary IS the proof the handler ran inside the cdylib (Principle #13 — no
//! fake data; every value is a real publish read in a real tick).
//!
//! # Why each stimulus fires (mirrors the proven in-process suites)
//!
//! The fixture is DATA-TRIGGERED on `trig`, so its publish cadence == its fire
//! cadence == `trig`'s arrival rate. That lets ONE fixture drive both watchdogs
//! to fire AND quiet:
//!   - `on_event_test` shape: a `sample(3)`-gated `samp` fed faster than the
//!     3 ms gate DECIMATES → `BackpressureEvent`.
//!   - `on_event_watchdog_test` shape: a SLOW `trig` producer starves the 20 ms
//!     `expect_within` window → `ExpectWithinEvent`; a FAST one keeps it fresh
//!     (quiet). A slow `trig` also means publishes slower than the 25 ms output
//!     promise → `PromiseWithinEvent` (`elapsed_ns > within_ns`).
//!   - `liveliness_sweep_iox2_test` (graceful path) shape: `samp` is
//!     wired to an absolute external topic; a raw publisher attaches (Alive
//!     edge, filtered — the fixture counts ONLY `Lost`), publishes once (so the
//!     body runs and `samp` is HELD thereafter per the cross-step hold), then gracefully
//!     DROPS — its `Drop` decrements iceoryx2's dynamic publisher count, the
//!     runtime sweep observes the 1→0 edge and mints a `Lost`, and the still-
//!     ticking DUT (its `trig` producer is in-graph and never stops) dispatches
//!     the handler. Never-drop → quiet. NOTE the seam-inject stimulus
//!     (`push_liveliness_event_for_test`, which `on_event_liveliness_test` uses
//!     with Arc-read counters) is UNSOUND in THIS harness: counters cross via
//!     the OUTPUT with a one-tick publish lag (tick N publishes counts through
//!     tick N-1's dispatch), so an inject before the final step increments the
//!     counter AFTER the last publish and never crosses — both the cdylib AND
//!     the in-process twin read 0 (measured). The real sweep transition +
//!     post-drop settle steps avoid the lag entirely and prove the production
//!     mechanism besides.
//!
//! # Determinism (Principle #7)
//!
//! Every gate keys off the wire `timestamp_ns` / `sequence` + the shared
//! `VirtualClock`, never wall-clock. The `dispatch_is_deterministic` test pins
//! two runs to a byte-identical counter tuple; `cdylib_matches_in_process_twin`
//! pins the cdylib's tuple EQUAL to an in-process twin with identical
//! declarations under the identical stimulus (an oracle, never a self-compare).
//!
//! # Running (iceoryx2 SHM singleton + cdylib `NODES` singleton → serial)
//!
//! ```bash
//! cargo build -p test_node_macro_onevent_cdylib
//! cargo test -p cerulion_core --test cdylib_on_event_dispatch_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::{Quaternion, Vector3};
use serial_test::serial;

/// Monotonic prefix counter so re-builds within one process never collide on an
/// iceoryx2 service name (mirrors non_trigger_hold / snapshot_view).
static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    format!("{stem}{}", PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed))
}

// ===========================================================================
// In-process producers (Vector3, one per needed period) + observing sink.
// Source code is truth; no fake data — every value is a real publish.
// ===========================================================================

/// 1 ms producer — feeds `samp`/`trig` fast enough for `sample(3)` decimation.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Prod1 {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl Prod1 {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// 5 ms producer — "fast enough" to keep a 20 ms window / 25 ms promise fresh.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct Prod5 {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl Prod5 {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// 100 ms producer — starves the 20 ms window and breaks the 25 ms promise.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct Prod100 {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl Prod100 {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Observing sink: data-triggers on the DUT's `out` (Quaternion) and records the
/// MAX of each counter field. MAX is monotone-safe: the counters only ever
/// increase, and the fixture's rare pre-warmup collapsed publish (default
/// x=y=z=0) can never lower an established value. The counters asserted `== 0`
/// (expect/live quiet paths) map to y/z, whose default is 0 — no leak.
#[cerulion_node]
#[derive(Default)]
struct CounterSink {
    #[input(trigger)]
    inp: Quaternion,
    bp: Arc<AtomicU64>,
    expect: Arc<AtomicU64>,
    live: Arc<AtomicU64>,
    promise: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl CounterSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.bp.fetch_max(self.inp.x as u64, Ordering::Relaxed);
        self.expect.fetch_max(self.inp.y as u64, Ordering::Relaxed);
        self.live.fetch_max(self.inp.z as u64, Ordering::Relaxed);
        self.promise.fetch_max(self.inp.w as u64, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// In-process TWIN of the cdylib fixture — IDENTICAL declarations, for the
// parity oracle (`cdylib_matches_in_process_twin`). Same tick + handlers.
// ===========================================================================

#[cerulion_node]
#[derive(Default)]
struct OnEventTwin {
    #[input(trigger, expect_within_ms = 20)]
    trig: Vector3,
    #[input(backpressure = sample(3))]
    samp: Vector3,
    #[output(promise_within_ms = 25)]
    out: Quaternion,
    last: f64,
    bp_count: u32,
    expect_count: u32,
    live_count: u32,
    promise_count: u32,
}
#[cerulion_node_impl]
impl OnEventTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.trig.x + self.samp.x;
        self.out.x = self.bp_count as f64;
        self.out.y = self.expect_count as f64;
        self.out.z = self.live_count as f64;
        self.out.w = self.promise_count as f64;
        Ok(())
    }

    #[on_event(input = "samp")]
    fn on_backpressure(&mut self, _event: BackpressureEvent) {
        self.bp_count += 1;
    }

    #[on_event(input = "trig")]
    fn on_expect(&mut self, _event: ExpectWithinEvent) {
        self.expect_count += 1;
    }

    // Mirrors the fixture: Lost-only, on `samp` (the (port,kind) coexistence
    // with the BackpressureEvent handler above).
    #[on_event(input = "samp")]
    fn on_liveliness(&mut self, event: LivelinessEvent) {
        if event.state == LivelinessState::Lost {
            self.live_count += 1;
        }
    }

    #[on_event(output = "out")]
    fn on_promise(&mut self, _event: PromiseWithinEvent) {
        self.promise_count += 1;
    }
}

// ===========================================================================
// Cdylib fixture locator (verbatim pattern from cdylib_non_trigger_hold_test).
// ===========================================================================

fn find_onevent_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_onevent_cdylib")
}

// ===========================================================================
// Recorded counters + graph builder.
// ===========================================================================

/// The four handler counters the sink reads back off the DUT's output.
#[derive(Clone)]
struct Counters {
    bp: Arc<AtomicU64>,
    expect: Arc<AtomicU64>,
    live: Arc<AtomicU64>,
    promise: Arc<AtomicU64>,
}
impl Counters {
    fn new() -> Self {
        Self {
            bp: Arc::new(AtomicU64::new(0)),
            expect: Arc::new(AtomicU64::new(0)),
            live: Arc::new(AtomicU64::new(0)),
            promise: Arc::new(AtomicU64::new(0)),
        }
    }
    /// Snapshot `(bp, expect, live, promise)` for tuple comparisons.
    fn tuple(&self) -> (u64, u64, u64, u64) {
        (
            self.bp.load(Ordering::Relaxed),
            self.expect.load(Ordering::Relaxed),
            self.live.load(Ordering::Relaxed),
            self.promise.load(Ordering::Relaxed),
        )
    }
}

/// Which node sits in the DUT slot — the real cdylib, or the in-process twin.
#[derive(Clone, Copy)]
enum Dut {
    Cdylib,
    Twin,
}

fn vec3_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

fn make_producer(node_type: &str) -> Box<dyn NodeEntry> {
    match node_type {
        "prod1" => Box::new(Prod1Entry::new()),
        "prod5" => Box::new(Prod5Entry::new()),
        "prod100" => Box::new(Prod100Entry::new()),
        other => panic!("unknown producer type {other}"),
    }
}

/// The absolute external topic the liveliness arms wire `samp` to. Producer-
/// less in the graph → the `External` provisioning arm, so the test's raw
/// publisher can attach/drop to drive REAL liveliness transitions (mirrors
/// `/live/cam` in `liveliness_sweep_iox2_test`). Safe as a fixed name:
/// `build_for_test` gives every runtime its own isolated SHM namespace.
const EXT_TOPIC: &str = "/oe/live";

/// Build the graph:
/// `trig_prod/out -> dut.trig (trigger)`; `dut/out -> sink.inp (trigger)`; and
/// `samp` wired either to an in-graph producer (`samp_prod = Some(type)`) or —
/// for the liveliness arms — to the producer-less absolute [`EXT_TOPIC`]
/// (`samp_prod = None`), where the test attaches/drops a raw external
/// publisher. The DUT is either the real cdylib or the in-process twin.
/// Returns the runtime + the sink's shared counters.
fn build_graph(
    prefix: &str,
    dut: Dut,
    trig_prod: &str,
    samp_prod: Option<&str>,
) -> (GraphRuntime, Counters) {
    let counters = Counters::new();
    let samp_source = match samp_prod {
        Some(_) => "samp_prod/out".to_string(),
        None => EXT_TOPIC.to_string(),
    };
    let mut nodes = vec![NodeDef {
        fuse: None,
        ros2: None,
        id: "trig_prod".to_string(),
        node_type: trig_prod.to_string(),
        inputs: vec![],
        outputs: vec![vec3_out("out")],
    }];
    if let Some(samp_type) = samp_prod {
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: "samp_prod".to_string(),
            node_type: samp_type.to_string(),
            inputs: vec![],
            outputs: vec![vec3_out("out")],
        });
    }
    nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "dut".to_string(),
        node_type: "onevent".to_string(),
        inputs: vec![
            InputDef {
                name: "trig".to_string(),
                source: "trig_prod/out".to_string(),
            },
            InputDef {
                name: "samp".to_string(),
                source: samp_source,
            },
        ],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Quaternion".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    });
    nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "sink".to_string(),
        node_type: "counter_sink".to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: "dut/out".to_string(),
        }],
        outputs: vec![],
    });
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_on_event_dispatch".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("trig_prod".to_string(), make_producer(trig_prod));
    if let Some(samp_type) = samp_prod {
        factories.insert("samp_prod".to_string(), make_producer(samp_type));
    }
    let dut_entry: Box<dyn NodeEntry> = match dut {
        Dut::Cdylib => {
            Box::new(DylibNodeEntry::load(&find_onevent_cdylib()).expect("load on_event cdylib"))
        }
        Dut::Twin => Box::new(OnEventTwinEntry::new()),
    };
    factories.insert("dut".to_string(), dut_entry);
    factories.insert(
        "sink".to_string(),
        Box::new(CounterSinkEntry::with_state(CounterSink {
            bp: Arc::clone(&counters.bp),
            expect: Arc::clone(&counters.expect),
            live: Arc::clone(&counters.live),
            promise: Arc::clone(&counters.promise),
            ..Default::default()
        })),
    );

    let clock = Arc::new(VirtualClock::new());
    // 16-deep buffer covers the held cdylib's borrow-3 provisioning
    // (holds_input_snapshot == true) on the source topics.
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build on_event dispatch graph");
    (runtime, counters)
}

/// Step `steps` times by `step`.
fn drive(runtime: &mut GraphRuntime, steps: usize, step: Duration) {
    for _ in 0..steps {
        runtime.step(step);
    }
}

/// Attach a raw external publisher to [`EXT_TOPIC`] via the runtime's parked
/// test transport (same SHM namespace as the graph — the sweep observes its
/// port via `number_of_publishers()`). Its graceful `Drop` is the REAL `Lost`
/// stimulus (mirrors `liveliness_sweep_iox2_test::attach_publisher`).
fn attach_ext_publisher(runtime: &GraphRuntime) -> CerulionPublisher {
    runtime
        .test_transport()
        .expect("test transport parked")
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("external publisher attaches to /oe/live")
}

/// Publish one Vector3 frame on the external publisher (loan-and-write; the
/// proxy Drop publishes). One frame is enough: `samp` is a held non-trigger
/// input, so the DUT's body keeps running off the held value after
/// the publisher drops.
fn publish_ext(publisher: &mut CerulionPublisher, v: f64) {
    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan ext frame");
    proxy.x = v;
    // drop(proxy) publishes the frame.
}

/// Set the 5 ms liveliness sweep cadence (one sweep per 5 ms step) and run the
/// controlled attach→(publish)→settle→drop→settle cycle against an already-
/// built runtime. `drop_publisher = false` is the quiet arm (attach-and-hold).
///
/// `step()` order is fire-then-sweep, so a transition the sweep pushes on step
/// N is dispatched by the DUT's tick on step N+1, and — because the DUT
/// publishes counters accumulated through the PREVIOUS tick's dispatch (the
/// one-tick publish lag) — the bumped counter crosses to the sink on step N+2.
/// The settle windows below are sized well past that.
fn run_liveliness_cycle(runtime: &mut GraphRuntime, drop_publisher: bool) {
    runtime.set_liveliness_sweep_period_for_test(5);
    let mut pubr = attach_ext_publisher(runtime);
    publish_ext(&mut pubr, 1.0);
    // Settle: the sweep observes the 0→1 attach (an Alive edge — FILTERED by
    // the fixture's Lost-only handler) and `samp`'s frame delivers + is held.
    drive(runtime, 4, MS5);
    if drop_publisher {
        // Graceful disconnect: Drop decrements the dynamic publisher count;
        // the next sweep observes 1→0 and mints the REAL Lost.
        drop(pubr);
    }
    // Settle: sweep (push) → next tick (dispatch) → next tick (publish the
    // bumped counter) → sink records. 8 steps is 4x the required margin.
    drive(runtime, 8, MS5);
}

const MS1: Duration = Duration::from_millis(1);
const MS5: Duration = Duration::from_millis(5);

// ===========================================================================
// (a) BackpressureEvent — sample(3) decimation over the cdylib FFI.
// ===========================================================================

#[test]
#[serial]
fn backpressure_event_dispatches_in_cdylib() {
    // 1 ms producers vs a 3 ms sample gate: consecutive reads 1 ms apart, so
    // ~2 of every 3 decimate → the edge-triggered BackpressureEvent fires and
    // the cdylib's handler bumps `bp_count`, published into out.x.
    let (mut runtime, c) = build_graph(&unique_prefix("cbp"), Dut::Cdylib, "prod1", Some("prod1"));
    drive(&mut runtime, 40, MS1);
    assert!(
        c.bp.load(Ordering::Relaxed) >= 1,
        "#[on_event] BackpressureEvent handler must fire INSIDE the cdylib under \
         sample(3) decimation and cross out.x to the host sink (got {})",
        c.bp.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (b) ExpectWithinEvent — input watchdog over the cdylib FFI (fire + quiet).
// ===========================================================================

#[test]
#[serial]
fn expect_within_event_fires_when_trig_slow_in_cdylib() {
    // 100 ms `trig` vs a 20 ms expect window over 400 ms: the window is starved
    // between the sparse arrivals, and the cdylib's ExpectWithinEvent handler
    // bumps `expect_count`, published into out.y.
    let (mut runtime, c) =
        build_graph(&unique_prefix("cew"), Dut::Cdylib, "prod100", Some("prod5"));
    drive(&mut runtime, 80, MS5);
    assert!(
        c.expect.load(Ordering::Relaxed) >= 1,
        "#[on_event] ExpectWithinEvent handler must fire INSIDE the cdylib when a \
         100 ms producer starves the 20 ms window (got {})",
        c.expect.load(Ordering::Relaxed)
    );
}

#[test]
#[serial]
fn expect_within_event_quiet_when_trig_fast_in_cdylib() {
    // 5 ms `trig` keeps the 20 ms window fresh → no miss, no event. The anti-
    // tautology control for the fire test above (proves the counter is tied to
    // starvation, not merely to load). out.y default is 0, so a pre-warmup
    // collapsed publish cannot leak a spurious count here.
    let (mut runtime, c) = build_graph(&unique_prefix("ceq"), Dut::Cdylib, "prod5", Some("prod5"));
    drive(&mut runtime, 80, MS5);
    assert_eq!(
        c.expect.load(Ordering::Relaxed),
        0,
        "#[on_event] ExpectWithinEvent handler must stay quiet inside the cdylib \
         when a 5 ms producer keeps the 20 ms window fresh (got {})",
        c.expect.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (c) PromiseWithinEvent — output watchdog over the cdylib FFI.
// ===========================================================================

#[test]
#[serial]
fn promise_within_event_fires_when_node_slow_in_cdylib() {
    // A slow (100 ms) `trig` makes the data-triggered DUT publish every ~100 ms,
    // far outside its own 25 ms output promise → `elapsed_ns > within_ns` each
    // gap → the cdylib's PromiseWithinEvent handler bumps `promise_count`,
    // published into out.w.
    let (mut runtime, c) =
        build_graph(&unique_prefix("cpw"), Dut::Cdylib, "prod100", Some("prod5"));
    drive(&mut runtime, 80, MS5);
    assert!(
        c.promise.load(Ordering::Relaxed) >= 1,
        "#[on_event] PromiseWithinEvent handler must fire INSIDE the cdylib when \
         the node publishes slower than its 25 ms promise (got {})",
        c.promise.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (d) LivelinessEvent — a REAL graceful publisher disconnect over the cdylib
// FFI (fire + quiet), via the production runtime liveliness sweep. `samp` is
// wired to the producer-less absolute EXT_TOPIC (samp_prod = None); the raw
// external publisher's attach mints an Alive (filtered — the fixture counts
// ONLY Lost) and its graceful Drop mints the Lost the handler counts.
// Mechanism mirrors liveliness_sweep_iox2_test::lost_fires_on_graceful_drop.
// The DUT stays data-triggered on the in-graph prod5, so it keeps ticking
// (and dispatching) after the external publisher drops — dodging the
// documented data-trigger dispatch limitation
// (liveliness_sweep_iox2_test::data_trigger_lost_handler_silent_but_counter_fires).
// ===========================================================================

#[test]
#[serial]
fn liveliness_lost_fires_on_real_disconnect_in_cdylib() {
    // Controlled sequence (all sim-clock stepped, deterministic): attach an
    // external publisher on samp's topic → publish one held frame → settle
    // (Alive edge, filtered) → DROP it → the sweep observes 1→0 and mints the
    // REAL Lost → the cdylib's handler (Lost-only) bumps `live_count` exactly
    // once, published into out.z. Exactly one drop ⇒ exactly 1.
    let (mut runtime, c) = build_graph(&unique_prefix("clv"), Dut::Cdylib, "prod5", None);
    run_liveliness_cycle(&mut runtime, true);
    assert_eq!(
        c.live.load(Ordering::Relaxed),
        1,
        "#[on_event] LivelinessEvent handler must fire INSIDE the cdylib exactly \
         once for one real graceful disconnect and cross out.z to the host (got {})",
        c.live.load(Ordering::Relaxed)
    );
}

#[test]
#[serial]
fn liveliness_lost_fires_in_process_control() {
    // DISCRIMINATOR control: the IDENTICAL real-disconnect stimulus against the
    // in-process twin. If THIS fires while the cdylib arm above reads 0, the
    // failure isolates to the FFI (a real inert-shipping gap); if both read 1,
    // the dispatch is proven on both sides of the boundary. (This control
    // caught the previous injected-seam stimulus being unsound in this
    // harness — the twin also read 0 — which is why the arm now drives the
    // production sweep mechanism instead.)
    let (mut runtime, c) = build_graph(&unique_prefix("clvt"), Dut::Twin, "prod5", None);
    run_liveliness_cycle(&mut runtime, true);
    assert_eq!(
        c.live.load(Ordering::Relaxed),
        1,
        "the real graceful disconnect must reach the IN-PROCESS twin's \
         LivelinessEvent handler exactly once (got {})",
        c.live.load(Ordering::Relaxed)
    );
}

#[test]
#[serial]
fn liveliness_quiet_while_publisher_held_in_cdylib() {
    // Attach + publish but NEVER drop: the only transition is the attach's
    // Alive edge, which the fixture's Lost-only handler filters — so the
    // counter must stay 0. Anti-tautology control for the fire arm (proves the
    // count is tied to the DISCONNECT, not to attach/load/sweep traffic);
    // out.z default is 0, so a pre-warmup collapsed publish cannot leak.
    let (mut runtime, c) = build_graph(&unique_prefix("clq"), Dut::Cdylib, "prod5", None);
    run_liveliness_cycle(&mut runtime, false);
    assert_eq!(
        c.live.load(Ordering::Relaxed),
        0,
        "#[on_event] LivelinessEvent (Lost-only) handler must stay quiet inside \
         the cdylib while the external publisher stays attached (got {})",
        c.live.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (e) PARITY — the cdylib's counters equal an in-process twin's under the
// identical stimulus (the in-process/dylib parity pattern; an oracle, not a self-compare).
// ===========================================================================

#[test]
#[serial]
fn cdylib_matches_in_process_twin() {
    // Drive the bp-arm stimulus through BOTH the cdylib and an in-process
    // twin with byte-identical declarations. bp is the DETERMINISTICALLY
    // driven kind in this arm (the expect/promise/live gates stay quiet
    // under this fast stimulus — those three are each pinned by their
    // dedicated FIRE arms above); the twin is the oracle for the full tuple:
    // a dispatch that silently dropped the bp kind over the FFI diverges on
    // x, and the quiet fields pin 0==0 against spurious dispatch.
    let (mut rt_cdy, cdy) =
        build_graph(&unique_prefix("cpar"), Dut::Cdylib, "prod1", Some("prod1"));
    drive(&mut rt_cdy, 40, MS1);

    let (mut rt_twin, twin) =
        build_graph(&unique_prefix("tpar"), Dut::Twin, "prod1", Some("prod1"));
    drive(&mut rt_twin, 40, MS1);

    assert_eq!(
        cdy.tuple(),
        twin.tuple(),
        "the cdylib's #[on_event] counters must EQUAL the in-process twin's under \
         the identical stimulus (cdylib {:?} vs twin {:?})",
        cdy.tuple(),
        twin.tuple()
    );
    // Anti-vacuity: the shared stimulus must actually exercise dispatch, or a
    // (0,0,0,0) == (0,0,0,0) would pass trivially. bp is the deterministically
    // driven kind in this arm.
    assert!(
        cdy.bp.load(Ordering::Relaxed) >= 1,
        "the parity stimulus must actually fire dispatch (bp got {})",
        cdy.bp.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (f) DETERMINISM — two cdylib runs yield a byte-identical counter tuple
// (Principle #7; the gates key off the wire timestamp + VirtualClock).
// ===========================================================================

#[test]
#[serial]
fn dispatch_is_deterministic_across_runs() {
    let (mut rt_a, a) = build_graph(&unique_prefix("cda"), Dut::Cdylib, "prod1", Some("prod1"));
    drive(&mut rt_a, 40, MS1);
    let (mut rt_b, b) = build_graph(&unique_prefix("cdb"), Dut::Cdylib, "prod1", Some("prod1"));
    drive(&mut rt_b, 40, MS1);
    assert_eq!(
        a.tuple(),
        b.tuple(),
        "cdylib #[on_event] dispatch must be bit-identical across runs \
         (a={:?} b={:?})",
        a.tuple(),
        b.tuple()
    );
    assert!(a.bp.load(Ordering::Relaxed) >= 1, "dispatch actually fired");
}
