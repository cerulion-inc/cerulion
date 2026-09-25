// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-step HOLD of a non-trigger latest-value `#[input]` for a
//! **cdylib** node (`DylibNodeEntry`), over real iceoryx2
//! (`GraphRuntime::build_for_test`, per-test SHM root).
//!
//! # The bug this pins
//!
//! The runtime's per-level snapshot pass calls `NodeEntry::snapshot_inputs`, which a
//! macro node forwards to `NodeContext::snapshot_inputs` (freeze + replay). Were a
//! cdylib to inherit the default NO-OP (no FFI snapshot entry point), the
//! cross-step hold would work ONLY for in-process macro nodes:
//! a cdylib node's non-trigger latest-value input would never be frozen →
//! on a step with no fresh sample its tick's nested `try_view` returns `None` →
//! the macro collapses the WHOLE tick to a no-op → the node effectively fires
//! only at the held input's producer rate (control loops throttled 10-100×).
//!
//! The optional `cerulion_node_set_snapshot_inputs` /
//! `cerulion_node_snapshot_inputs` FFI exports close that: `DylibNodeEntry::snapshot_inputs`
//! forwards the per-step freeze across the FFI to the cdylib's `NodeContext`. So
//! a cdylib HOLDS its non-trigger `#[input]` exactly like a macro node.
//!
//! # Topology + observability
//!
//! ```text
//!   producer(macro, external) --prod/out--> [inp, non-trigger] cdy(cdylib, period)
//!                                            cdy --cdy/out--> [inp, TRIGGER] sink(macro)
//! ```
//!
//! The cdylib node is the existing `test_node_macro_period_input_cdylib` fixture
//! (`period_ms = 10`, `#[input] inp` non-trigger, `#[output] out`, body
//! `out.x = inp.x`). It fires every step (period). Its `inp` is non-trigger →
//! snapshotted, so it HOLDS. The cdylib's state is opaque across the
//! FFI, so observability is a downstream in-process macro `sink` that
//! data-triggers on `cdy/out` and records `out.x` into a shared `Arc<AtomicU64>`.
//!
//! **Observable values (a node WITH an output):** the macro loans the `out`
//! proxy (default-initialised to 0) at tick start. **Collapse-no-publish:** a
//! `None` input that collapses the body must NOT publish that proxy on Drop
//! (were `build_nested_try_view` to map `Ok(None) => Ok(Ok(()))`,
//! structurally identical to a genuine success — the arm gate could not tell
//! them apart, and the cdylib would fabricate a zero-init publish on every collapsed
//! tick). A `bool` discriminant threads through the chain
//! (`Ok(Ok(true))` = ran, `Ok(Ok(false))` = collapsed) so a collapsed tick
//! never arms publish at all. So the cdylib publishes ONLY on ticks
//! where the body genuinely ran, and the sink records:
//!   - the **held value** on those ticks (input was `Sample` or `Held`), vs
//!   - **[`MISSING`]** (no delivery at all — the sink's data trigger never
//!     fires) when the body collapsed (input `Empty` → no held value:
//!     pre-first-delivery, or a no-op snapshot).
//!
//! The hold's signature is therefore "held value V" (held-value replay) vs
//! "[`MISSING`] / no delivery" (collapse) on a silent step — publish-
//! vs-no-publish, not "real value vs fabricated default".
//! The pre-delivery arm (the `pre` vector) of
//! `cdylib_held_value_appears_only_after_real_delivery` is the direct
//! regression pin for this: it asserts `vec![MISSING; PRE_STEPS]`
//! (no publish at all pre-delivery), not `vec![0; PRE_STEPS]` (a fabricated
//! zero-init publish).
//!
//! Every recorded value comes from a REAL iceoryx2 publish chain (no fake data,
//! Principle #13). Every assertion is against a HAND-WRITTEN oracle, never a
//! self-compare (the determinism test compares two runs AND the hand oracle).
//!
//! Also pins the source-topic borrow provisioning: a held cdylib forces
//! `subscriber_max_borrowed_samples = 3` on `prod/out` (`holds_input_snapshot()
//! == true`); without it the held burst drain would exceed the iceoryx2 default
//! borrow ceiling of 2.
//!
//! # Running (iceoryx2 SHM singleton + cdylib `NODES` singleton → serial)
//!
//! ```bash
//! cargo build -p test_node_macro_period_input_cdylib
//! cargo test -p cerulion_core --test cdylib_non_trigger_hold_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Per-step reset sentinel. **Collapse-no-publish:** were the cdylib fixture's
/// `#[output]` proxy to publish on Drop EVERY step (even when the body
/// collapsed on a `None` input), the sink would fire every step and MISSING would
/// never be the observed value. A collapsed tick does NOT arm publish
/// at all, so the sink's data-trigger genuinely does not fire on a collapsed
/// step — MISSING IS a real, expected observation (see
/// `cdylib_held_value_appears_only_after_real_delivery`'s pre-delivery arm),
/// not just a safety net for a failed delivery.
const MISSING: u64 = u64::MAX;

/// The cdylib fixture is `period_ms = 10`; step the virtual clock by the period
/// so it fires on every step.
const STEP: Duration = Duration::from_millis(10);

/// Monotonic prefix counter so re-builds within one process never collide on an
/// iceoryx2 service name (mirrors non_trigger_hold_iox2_test / snapshot_view).
static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    format!("{stem}{}", PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed))
}

// ===========================================================================
// In-process macro nodes (producer + observing sink). Source code is truth; no
// fake data — every value is a real publish read in a real tick.
// ===========================================================================

/// External producer publishing a FIXED scalar into `out.x` on each fire. The
/// harness fires it (or not) to control exactly which steps deliver to the
/// cdylib's held input.
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

/// In-process observing sink: data-triggers on the cdylib's `out`, recording the
/// value it reads into a shared `Arc<AtomicU64>`. Because it is a TRIGGER input,
/// the sink fires ONLY when the cdylib actually publishes `out` — so a recorded
/// value is exactly "the cdylib's tick ran and published this step".
#[cerulion_node]
#[derive(Default)]
struct RecordingSink {
    #[input(trigger)]
    inp: Vector3,
    rec: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl RecordingSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.rec.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// Cdylib fixture locator (verbatim pattern from rayon_fire_cdylib_serial_test).
// ===========================================================================

fn find_period_input_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_input_cdylib")
}

/// Locate the raw-FFI `test_node_cdylib` fixture, a cdylib that
/// exports NEITHER snapshot symbol. Built by `cargo build -p test_node_cdylib`.
fn find_symbolless_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_cdylib")
}

/// The backward-compatibility safety claim.
/// A cdylib that exports NEITHER snapshot symbol still loads, reports
/// `holds_input_snapshot() == false` (so the host provisions it at the default
/// borrow-2, NOT borrow-3), and `snapshot_inputs(...)` is a SAFE no-op — the
/// `snapshot_ffi == None` guard returns before any FFI pointer is touched (no
/// panic, no null-fn-ptr deref). This is the one regression class nothing else
/// catches: if `load`'s best-effort `.ok()` ever became `?`, or the `let-else`
/// guard in `snapshot_inputs` became an `unwrap`, an old cdylib would fail to
/// load / SIGSEGV. Both calls return at the `snapshot_ffi == None` guard before
/// `inputs.join` — the empty-names call is a redundant no-op safety check, NOT
/// the `names_len == 0` join path (that lives on the symbol-having Active side).
#[test]
#[serial]
fn symbolless_cdylib_does_not_hold_and_snapshot_is_safe_noop() {
    let mut entry =
        DylibNodeEntry::load(&find_symbolless_cdylib()).expect("pre-hold-fix cdylib still loads");
    // No symbols → does not hold → host keeps the default borrow-2 provisioning.
    assert!(
        !entry.holds_input_snapshot(),
        "a symbol-less cdylib must NOT report holding (back-compat: reads live)"
    );
    // Safe no-op even pre-init (handle is None) AND for any input names — the
    // `snapshot_ffi == None` guard short-circuits before the handle / FFI calls.
    entry.snapshot_inputs(&["whatever".to_string(), "x".to_string()]);
    entry.snapshot_inputs(&[]); // empty names also safe on the no-op path
                                // Reaching here without a panic / segfault IS the assertion.
}

// ===========================================================================
// Graph builder + drivers
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

/// `producer/out` -> cdy.inp (non-trigger, HELD); cdy/out -> sink.inp (trigger,
/// observed). The cdylib consumer is the REAL `DylibNodeEntry` fixture. Returns
/// the runtime + the sink's shared read record.
fn build_cdylib_hold_graph(prefix: &str, val: f64) -> (GraphRuntime, Arc<AtomicU64>) {
    let rec = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_non_trigger_hold".to_string(),
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
                id: "cdy".to_string(),
                node_type: "period_input".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "recording_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "cdy/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer {
            val,
            ..Default::default()
        })),
    );
    factories.insert(
        "cdy".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_period_input_cdylib()).expect("load period+input cdylib"),
        ),
    );
    factories.insert(
        "sink".to_string(),
        Box::new(RecordingSinkEntry::with_state(RecordingSink {
            rec: Arc::clone(&rec),
            ..Default::default()
        })),
    );

    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build cdylib hold graph");
    (runtime, rec)
}

/// Fire the producer ONCE, then step (producer silent) until the sink records
/// `expected` — establishing the cdylib's held value AND warming the full
/// producer->cdy->sink pipeline. The bounded loop tolerates iceoryx2 connection
/// warmup. `producer` and `cdy` share level 0 (the `inp` edge is non-trigger),
/// so the cdy snapshot reads the producer's PRIOR-step publish — hence the
/// producer fires once up front and the cdy drains it on a later step.
fn establish_held(rt: &mut GraphRuntime, rec: &Arc<AtomicU64>, expected: u64) {
    rt.trigger_external("producer").expect("trigger producer");
    let mut tries = 0;
    loop {
        rec.store(MISSING, Ordering::Relaxed);
        rt.step(STEP); // producer NOT re-triggered — only the pending first fire publishes
        if rec.load(Ordering::Relaxed) == expected {
            return;
        }
        tries += 1;
        assert!(
            tries < 200,
            "cdylib held value {expected} never established within 200 steps"
        );
    }
}

/// Step `horizon` times with the producer SILENT, returning the sink's recorded
/// read each step. Every step the cdy snapshot drains `Empty` → REPLAYS the held
/// value → body runs (NOT a collapse — `try_view` returns `Some`) →
/// publishes the held value → the sink records it. This function is ONLY ever
/// called AFTER [`establish_held`], so it never exercises the collapse path —
/// its window is always `[V; horizon]` (a hand oracle, not a self-compare).
/// (Were the cdy snapshot a no-op — never freezing —
/// each silent step would read `Empty` live and the body would collapse;
/// a collapsed tick does not publish at all, so that hypothetical window would
/// be `[MISSING; horizon]`, not `[0; horizon]` — see
/// `cdylib_held_value_appears_only_after_real_delivery`'s pre-delivery arm for
/// the actual collapse-path test.)
fn replay_window(rt: &mut GraphRuntime, rec: &Arc<AtomicU64>, horizon: usize) -> Vec<u64> {
    let mut out = Vec::with_capacity(horizon);
    for _ in 0..horizon {
        rec.store(MISSING, Ordering::Relaxed);
        rt.step(STEP);
        out.push(rec.load(Ordering::Relaxed));
    }
    out
}

// ===========================================================================
// PIN 0 (DIRECT): the rebuilt fixture exports the snapshot symbols →
// holds_input_snapshot() == true, while performs_input_snapshot() stays false
// (cdylib fires serially). The decoupling is the load-bearing fact for the
// borrow-3 provisioning + serial routing.
// ===========================================================================
#[test]
#[serial]
fn cdylib_holds_but_is_not_rayon_eligible() {
    let entry =
        DylibNodeEntry::load(&find_period_input_cdylib()).expect("load period+input cdylib");
    assert!(
        entry.holds_input_snapshot(),
        "the period+input cdylib fixture must export the snapshot FFI \
         symbols → holds_input_snapshot() == true. If false, rebuild it: \
         `cargo build -p test_node_macro_period_input_cdylib`"
    );
    assert!(
        !entry.performs_input_snapshot(),
        "a cdylib must stay OFF the rayon path (performs_input_snapshot() == \
         false) even though it now holds — making FFI ticks thread-safe is out \
         of scope"
    );
}

// ===========================================================================
// PIN 1 (CORE): a cdylib's non-trigger input replays its last-delivered value
// across MANY silent steps. Without the hold these steps would each be MISSING.
// ===========================================================================
#[test]
#[serial]
fn cdylib_non_trigger_input_replays_last_delivered_value() {
    const V: u64 = 42;
    const HORIZON: usize = 20;
    let (mut rt, rec) = build_cdylib_hold_graph(&unique_prefix("cdyhold"), V as f64);

    establish_held(&mut rt, &rec, V);

    // MANY silent steps: producer silent, cdy fires every step (period) → its
    // held input REPLAYS V on EVERY step → it publishes → the sink records V.
    let replays = replay_window(&mut rt, &rec, HORIZON);

    // HAND ORACLE (anti-tautology): exactly HORIZON values, all == V.
    assert_eq!(
        replays,
        vec![V; HORIZON],
        "the cdylib's held value {V} must replay on every one of the {HORIZON} \
         silent steps (without the hold the cdylib's non-trigger input is never \
         frozen, so its body collapses and every one of these steps is MISSING). \
         got {replays:?}"
    );
}

// ===========================================================================
// PIN 2 (NO FABRICATION before delivery, collapse-no-publish): pre-first-delivery the
// held mechanism must NOT invent a held value AND the collapsed
// tick must NOT publish AT ALL — no held value, no fabricated zero-init
// default either (Principle #13: no fake data). After a real delivery the
// held value appears and replays.
//
// A node WITH an output expresses the macro consumer's
// "WAIT == no publish" only if its proxy does not publish on Drop after a
// collapse. A proxy that always published would show up here as a
// fabricated default-0 before delivery. The
// collapsed tick genuinely withholds publish, so the sink's
// data trigger never fires pre-delivery and [`MISSING`] survives every
// pre-delivery step — the DIRECT regression pin for collapse-no-publish.
// ===========================================================================
#[test]
#[serial]
fn cdylib_held_value_appears_only_after_real_delivery() {
    const V: u64 = 99;
    const PRE_STEPS: usize = 8;
    const HORIZON: usize = 10;
    let (mut rt, rec) = build_cdylib_hold_graph(&unique_prefix("cdypre"), V as f64);

    // (a) PRE-DELIVERY: producer NEVER fires. The cdy fires every step (period)
    // but its `inp` never delivers → no held value → the body collapses →
    // The collapsed tick does NOT arm publish at all, so the sink's
    // data trigger never fires and MISSING survives every step. It must NEVER
    // fabricate the held value V before a real delivery, NOR a zero-init
    // default in its place.
    let mut pre = Vec::with_capacity(PRE_STEPS);
    for _ in 0..PRE_STEPS {
        rec.store(MISSING, Ordering::Relaxed);
        rt.step(STEP);
        pre.push(rec.load(Ordering::Relaxed));
    }
    assert_eq!(
        pre,
        vec![MISSING; PRE_STEPS],
        "pre-delivery every tick collapses and must NOT publish at \
         all — the sink must record MISSING (no delivery) on every one of the \
         {PRE_STEPS} steps (hand oracle), not a fabricated zero-init default. \
         got {pre:?}"
    );
    assert!(
        !pre.contains(&V),
        "the held value {V} must NOT appear before any real delivery — the hold \
         must not fabricate a value from nowhere (got {pre:?})"
    );

    // (b) Now deliver once: the held value V appears and replays on silent steps.
    establish_held(&mut rt, &rec, V);
    let replays = replay_window(&mut rt, &rec, HORIZON);
    assert_eq!(
        replays,
        vec![V; HORIZON],
        "after a real delivery the held value {V} replays on every silent step \
         (got {replays:?})"
    );
}

// ===========================================================================
// PIN 3 (DETERMINISM): two independent runs are byte-identical AND equal the
// hand oracle (Principle #7). Not a self-compare — the oracle is the literal.
// ===========================================================================
#[test]
#[serial]
fn cdylib_hold_replay_is_deterministic() {
    const V: u64 = 7;
    const HORIZON: usize = 12;

    let run = || {
        let (mut rt, rec) = build_cdylib_hold_graph(&unique_prefix("cdydet"), V as f64);
        establish_held(&mut rt, &rec, V);
        replay_window(&mut rt, &rec, HORIZON)
    };

    let a = run();
    let b = run();

    assert_eq!(a, b, "two runs of the cdylib hold must be byte-identical");
    assert_eq!(
        a,
        vec![V; HORIZON],
        "AND both must equal the hand oracle (anti-tautology): {V} replayed on \
         every silent step. got {a:?}"
    );
}
