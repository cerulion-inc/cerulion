// SPDX-License-Identifier: AGPL-3.0-only
//! Unified trigger drain for CDYLIB nodes — the production
//! surface of the one-receive-per-hop win.
//!
//! A macro-generated cdylib now exports the OPTIONAL
//! `cerulion_node_drain_trigger_input` FFI symbol (additive, NO ABI bump —
//! same precedent as the snapshot pair). Symbol PRESENCE is the
//! capability (`DylibNodeEntry::unifies_trigger_drain`): present ⇒ the
//! runtime elides the separate trigger-drain subscriber and wires the
//! binding `DrainSource::Unified`, with `drain_trigger_input` forwarded
//! across the FFI at the level boundary; absent (raw-FFI / older
//! cdylib) ⇒ today's dual-subscriber path, byte-identical. Because only
//! macro-generated cdylibs export the symbol — and their reads are generated
//! `try_view`, the one path the unified drain's frozen slot serves — the
//! gate also structurally enforces the trait's READ-PATH CONTRACT for the
//! production surface.
//!
//! What this file pins (real iceoryx2 via `build_for_test`, per-test SHM
//! root; hand oracles, downstream-DELIVERY based — never fire-count, which
//! records even on tick Err):
//!
//! - **(a) Unified + delivers:** the `test_node_macro_data_trigger_cdylib`
//!   consumer (forwards `trigger_in.x` → `cmd.x`) reports the capability
//!   DIRECTLY (`unifies_trigger_drain() == true`), builds with EXACTLY one
//!   Unified binding (the closure sink is `from_names` ⇒ Separate by the
//!   stage-1a fail-safe, so the count isolates the cdylib), and the sink
//!   observes the full hand-oracle sequence THROUGH the cdylib — the
//!   delivery oracle proves the FFI drain feeds both the fire signal and the
//!   tick's `try_view`.
//! - **(b) back-compat:** the raw-FFI `test_node_cdylib` fixture (no symbol)
//!   still loads and reports `unifies_trigger_drain() == false` — the
//!   no-silent-breakage pin. (No raw-FFI fixture carries a data-trigger
//!   input, so the behavioral Separate-delivery arm for a symbol-less cdylib
//!   is covered by (c)'s forced-Separate leg instead — same wiring, same
//!   drain path.)
//! - **(c) the drain-discipline seam extends to cdylibs:** `CERULION_DRAIN_DISCIPLINE=
//!   separate` forces the SAME cdylib graph back to 0 Unified bindings with
//!   BYTE-IDENTICAL delivery (the changes-HOW-not-WHAT invariant on
//!   the FFI surface) — which also behaviorally exercises the Separate path
//!   for a cdylib data-trigger input.
//!
//! - **(d) the FFI-FAILURE arm:** the `test_node_drain_fail_cdylib`
//!   raw-FFI fixture EXPORTS the drain symbol (capability TRUE → Unified) but
//!   FAILS it (-1) under `CER_FAIL_MODE=drain_fail`. TEST A (fault): the host
//!   maps every failure to the safe `(0, None)` — the consumer NEVER fires
//!   (no fabricated fire signal; fire_count 0 IS the right assert here, since
//!   fires record even on tick Err — 0 means no fire SIGNAL existed), the
//!   flood latch logs exactly ONE `error!` across N failing steps
//!   (`logs_assert` count pins the latch), and the RUNTIME SURVIVES (the
//!   producer keeps publishing all N steps — no wedge). TEST B (control, no
//!   fail mode): the drain returns 0 with `popped = 0` → still no fires.
//!   Anti-tautology: the fixture's process-global drain-call counter (read
//!   via the test's OWN `libloading` handle — a second `dlopen` shares the
//!   data segment) proves the FFI was invoked in BOTH arms.
//!
//! # Required fixture builds
//!
//! ```bash
//! cargo build -p test_node_macro_data_trigger_cdylib -p test_node_cdylib \
//!             -p test_node_drain_fail_cdylib
//! cargo test -p cerulion_core --test cdylib_unified_drain_test -- --test-threads=1
//! ```
//!
//! All `#[serial]`: the cdylib `NODES` registry is process-global, the env
//! knob is process-global, and iceoryx2's SHM singleton wants serial runs.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, DylibNodeEntry, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// The drain-discipline measurement seam knob (process-global).
const KNOB: &str = "CERULION_DRAIN_DISCIPLINE";

/// Steps discarded before the measured window.
const WARMUP: u32 = 3;

/// Measured steps: one producer publish + one full-chain delivery each.
const MEASURED: u32 = 8;

/// Sentinel the producer never publishes (it publishes 1, 2, ...). A measured
/// step that leaves this means the sink tick did not run — loud, not silent.
const MISSING: u64 = u64::MAX;

/// RAII guard for the seam knob (panic-safe removal).
struct EnvVarGuard;
impl EnvVarGuard {
    fn set(value: &str) -> Self {
        std::env::set_var(KNOB, value);
        Self
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(KNOB);
    }
}

/// Hand oracle for the measured window: the producer publishes `k` on global
/// step `k` (1-indexed); the 2-hop data-trigger chain (cdylib forward + sink)
/// delivers it within the SAME step (earlier-level publishes flow down the
/// DAG within one step), so measured step `i` observes `WARMUP + i + 1`.
fn oracle() -> Vec<u64> {
    ((WARMUP as u64 + 1)..=(WARMUP as u64 + MEASURED as u64)).collect()
}

/// Locate a cdylib fixture in the workspace target dir (the
/// `rayon_fire_cdylib_serial_test` pattern, generalized by crate stem).
fn find_cdylib(stem: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(stem)
}

// ===========================================================================
// Macro producer: Period(10), publishes an incrementing counter into
// Vector3.x (straight into the loaned SHM slot).
// ===========================================================================

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DrainFeedProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl DrainFeedProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Build + drive the 3-node chain: macro producer → CDYLIB forwarder
/// (`test_node_macro_data_trigger_cdylib`: `trigger_in.x` → `cmd.x`) →
/// closure sink (records its `try_view` read). Returns
/// `(cdylib_capability, unified_binding_count, sink_sequence)`.
///
/// The sink closure is built via `NodeInfo::from_names` (EMPTY `input_meta`),
/// so by the stage-1a fail-safe it stays `DrainSource::Separate` — the
/// Unified count therefore isolates the CDYLIB binding: 1 when unified, 0
/// when the seam forces Separate.
fn run_cdylib_chain(prefix: &str) -> (bool, usize, Vec<u64>) {
    let cdylib_path = find_cdylib("test_node_macro_data_trigger_cdylib");
    let fwd = DylibNodeEntry::load(&cdylib_path).expect("load data-trigger cdylib");
    let capability = fwd.unifies_trigger_drain();

    let sink_read = Arc::new(AtomicU64::new(MISSING));
    let sink_read_c = Arc::clone(&sink_read);
    let sink = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |ctx| {
            let v = ctx
                .subscriber_mut("in")
                .and_then(|s| {
                    s.try_view::<Vector3, _>(|view| view.x as u64)
                        .ok()
                        .flatten()
                })
                .unwrap_or(MISSING);
            sink_read_c.store(v, Ordering::Relaxed);
            Ok(())
        },
    )
    .with_label("cdylib_drain_sink");

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_unified_drain".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "drain_feed_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "fwd".to_string(),
                node_type: "data_trigger_node".to_string(),
                inputs: vec![InputDef {
                    name: "trigger_in".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![OutputDef {
                    name: "cmd".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "cdylib_drain_sink".to_string(),
                inputs: vec![InputDef {
                    name: "in".to_string(),
                    source: "fwd/cmd".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(DrainFeedProducerEntry::new()),
    );
    factories.insert("fwd".to_string(), Box::new(fwd));
    factories.insert("sink".to_string(), Box::new(sink));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build cdylib unified-drain graph");

    let unified = runtime.unified_binding_count_for_test();

    for _ in 0..WARMUP {
        runtime.step(Duration::from_millis(10));
    }
    let mut seq = Vec::with_capacity(MEASURED as usize);
    for _ in 0..MEASURED {
        sink_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        seq.push(sink_read.load(Ordering::Relaxed));
    }
    (capability, unified, seq)
}

// ===========================================================================
// (a) the macro cdylib builds Unified and delivers through the FFI drain.
// ===========================================================================
#[test]
#[serial]
fn macro_cdylib_data_trigger_builds_unified_and_delivers() {
    std::env::remove_var(KNOB);
    let (capability, unified, seq) = run_cdylib_chain("cud_default");
    assert!(
        capability,
        "a macro-generated cdylib must export cerulion_node_drain_trigger_input \
         and report unifies_trigger_drain() == true"
    );
    assert_eq!(
        unified, 1,
        "exactly the CDYLIB binding must be Unified (the from_names closure \
         sink stays Separate by the stage-1a fail-safe); got {unified}"
    );
    assert_eq!(
        seq,
        oracle(),
        "the sink must observe the full hand-oracle sequence THROUGH the \
         cdylib — the FFI drain must feed both the fire signal and the tick's \
         generated try_view"
    );
}

// ===========================================================================
// (b) back-compat: a raw-FFI cdylib (no symbol) loads + capability false.
// ===========================================================================
#[test]
#[serial]
fn raw_ffi_cdylib_without_symbol_reports_capability_false() {
    let path = find_cdylib("test_node_cdylib");
    let entry = DylibNodeEntry::load(&path).expect("raw-FFI cdylib must still load (no ABI bump)");
    assert!(
        !entry.unifies_trigger_drain(),
        "a symbol-less (raw-FFI / pre-unified-drain) cdylib must report the drain \
         capability FALSE — its data-trigger inputs stay DrainSource::Separate \
         (today's behavior, byte-identical)"
    );
    // The info surface is untouched by the optional symbol probe.
    entry
        .info()
        .expect("raw-FFI cdylib info() must still parse");
}

// ===========================================================================
// (c) the drain-discipline seam extends to cdylibs: =separate forces the cdylib back
// to Separate with byte-identical delivery.
// ===========================================================================
#[test]
#[serial]
fn seam_forces_cdylib_back_to_separate_byte_identical() {
    let expected = oracle();

    // Leg A — default: the cdylib binding is Unified.
    std::env::remove_var(KNOB);
    let (cap_a, unified_a, seq_a) = run_cdylib_chain("cud_ab_uni");
    assert!(cap_a, "leg A capability must be true");
    assert_eq!(unified_a, 1, "leg A must wire the cdylib Unified");
    assert_eq!(seq_a, expected, "leg A delivers the hand oracle");

    // Leg B — forced Separate: the capability is still reported (the symbol
    // exists) but the seam overrides the WIRING — behaviorally exercising the
    // dual-subscriber path for a cdylib data-trigger input.
    let _guard = EnvVarGuard::set("separate");
    let (cap_b, unified_b, seq_b) = run_cdylib_chain("cud_ab_sep");
    assert!(
        cap_b,
        "leg B capability is unchanged (the seam overrides wiring)"
    );
    assert_eq!(
        unified_b, 0,
        "leg B (=separate) must force the cdylib binding to Separate"
    );
    assert_eq!(seq_b, expected, "leg B delivers the SAME hand oracle");
    assert_eq!(
        seq_a, seq_b,
        "cdylib Separate delivery must be byte-identical to Unified (the \
         unified-drain changes-HOW-not-WHAT invariant across the FFI)"
    );
}

// ===========================================================================
// (d) the FAILING-drain FFI pin. The
// `test_node_drain_fail_cdylib` raw-FFI fixture exports the drain symbol
// (capability TRUE → Unified) but fails it under CER_FAIL_MODE=drain_fail.
// The host must map every failure to the safe (0, None): no fabricated fire
// signal, one flood-latched error!, and a surviving runtime.
// ===========================================================================

/// RAII guard for `CER_FAIL_MODE` (panic-safe removal; the
/// `chunk_c_ffi_codes_3_4_test` pattern).
struct FailModeGuard;
impl FailModeGuard {
    fn set(value: &str) -> Self {
        std::env::set_var("CER_FAIL_MODE", value);
        Self
    }
}
impl Drop for FailModeGuard {
    fn drop(&mut self) {
        std::env::remove_var("CER_FAIL_MODE");
    }
}

/// Steps driven in each drain-fail arm.
const FAIL_STEPS: u32 = 6;

/// Load the drain-fail fixture through the test's OWN `libloading` handle so
/// its process-global drain-call counter is readable/resettable (a second
/// `dlopen` of the same object shares the data segment — the
/// `test_node_snapshot_fail_cdylib` pattern).
fn load_drain_fail_accessor() -> libloading::Library {
    unsafe { libloading::Library::new(find_cdylib("test_node_drain_fail_cdylib")) }
        .expect("accessor dlopen of test_node_drain_fail_cdylib")
}

fn reset_drain_calls(lib: &libloading::Library) {
    let f: libloading::Symbol<unsafe extern "C" fn()> = unsafe {
        lib.get(b"cerulion_test_reset_drain_calls\0")
            .expect("reset symbol")
    };
    unsafe { f() };
}

fn get_drain_calls(lib: &libloading::Library) -> u64 {
    let f: libloading::Symbol<unsafe extern "C" fn(u64) -> u64> = unsafe {
        lib.get(b"cerulion_test_get_drain_calls\0")
            .expect("get symbol")
    };
    unsafe { f(0) }
}

/// Build + drive the drain-fail graph: macro producer → drain-fail cdylib
/// consumer (`inp` ← producer/out, data-trigger). Returns
/// `(capability, unified_count, consumer_fires, producer_fires)`.
fn run_drain_fail_graph(prefix: &str) -> (bool, usize, u64, u64) {
    let entry = DylibNodeEntry::load(&find_cdylib("test_node_drain_fail_cdylib"))
        .expect("load drain-fail cdylib");
    let capability = entry.unifies_trigger_drain();

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_drain_fail".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "drain_feed_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "drain_fail_node".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(DrainFeedProducerEntry::new()),
    );
    factories.insert("consumer".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build drain-fail graph");

    let unified = runtime.unified_binding_count_for_test();
    for _ in 0..FAIL_STEPS {
        runtime.step(Duration::from_millis(10));
    }
    let consumer_fires = runtime
        .node_handle("consumer")
        .map(|h| h.fire_count())
        .unwrap_or(u64::MAX);
    let producer_fires = runtime
        .node_handle("producer")
        .map(|h| h.fire_count())
        .unwrap_or(0);
    (capability, unified, consumer_fires, producer_fires)
}

// TEST A (fault): the failing drain never fabricates a fire signal, logs
// exactly one flood-latched error!, and the runtime survives.
#[test]
#[serial]
#[traced_test]
fn failing_drain_never_fabricates_a_fire_and_runtime_survives() {
    std::env::remove_var(KNOB);
    let accessor = load_drain_fail_accessor();
    reset_drain_calls(&accessor);

    let _fail = FailModeGuard::set("drain_fail");
    let (capability, unified, consumer_fires, producer_fires) = run_drain_fail_graph("cud_fail");

    assert!(
        capability,
        "the drain-fail fixture exports the symbol — capability must be TRUE"
    );
    assert_eq!(unified, 1, "the failing consumer must be wired Unified");
    assert_eq!(
        consumer_fires, 0,
        "a FAILING drain maps to (0, None) — the consumer must NEVER fire \
         (a nonzero fire_count means a fire signal was fabricated)"
    );
    assert_eq!(
        producer_fires,
        u64::from(FAIL_STEPS),
        "the runtime must SURVIVE the failing drain — the producer keeps \
         publishing on every step (no wedge)"
    );
    let calls = get_drain_calls(&accessor);
    assert!(
        calls >= 1,
        "the drain FFI must actually have been invoked (got {calls} calls) — \
         otherwise this test is vacuous"
    );
    // The flood latch: N failing steps ⇒ EXACTLY ONE error! (the rest are
    // suppressed to debug until recovery).
    assert!(
        logs_contain("cdylib drain_trigger_input FFI call failed"),
        "the first drain failure must log loudly"
    );
    logs_assert(|lines: &[&str]| {
        let errors = lines
            .iter()
            .filter(|l| l.contains("cdylib drain_trigger_input FFI call failed"))
            .count();
        if errors == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 flood-latched error across {FAIL_STEPS} \
                 failing steps, got {errors}"
            ))
        }
    });
}

// TEST B (control): no fail mode — the drain succeeds with popped = 0, so
// still no fires; the counter proves the FFI ran in this arm too.
#[test]
#[serial]
fn control_drain_returns_zero_popped_and_counter_proves_invocation() {
    std::env::remove_var(KNOB);
    std::env::remove_var("CER_FAIL_MODE");
    let accessor = load_drain_fail_accessor();
    reset_drain_calls(&accessor);

    let (capability, unified, consumer_fires, producer_fires) = run_drain_fail_graph("cud_ctrl");

    assert!(capability, "control arm capability must be TRUE");
    assert_eq!(unified, 1, "control arm must be wired Unified");
    assert_eq!(
        consumer_fires, 0,
        "a successful drain reporting popped = 0 must produce no fires \
         (nothing arrived, per the fixture)"
    );
    assert_eq!(
        producer_fires,
        u64::from(FAIL_STEPS),
        "the producer publishes on every step in the control arm"
    );
    let calls = get_drain_calls(&accessor);
    assert!(
        calls >= 1,
        "anti-tautology: the drain FFI must have been invoked in the CONTROL \
         arm too (got {calls} calls) — the fault arm's 0-fires is meaningful \
         only if the FFI actually runs in both"
    );
}
