// SPDX-License-Identifier: AGPL-3.0-only
//! The node-side producer-reconciliation teardown log (the FFI-free
//! parity fix for the cdylib producer-harvest gap).
//!
//! A `graph run` node is a CDYLIB that takes ownership of its `NodeContext` at
//! `init()`, so the host-side harvest
//! (`GraphRuntime::log_publisher_reconciliation_stats`) cannot reach a cdylib's
//! publishers — an all-cdylib graph (the humanoid repro) yields NO producer-side
//! terms host-side. The fix logs each publisher's reconciliation snapshot from
//! INSIDE the process that owns it, at teardown: `impl Drop for NodeContext`
//! calls `NodeContext::log_reconciliation_stats_at_teardown`, so a cdylib node
//! emits it via the cdylib-local stderr subscriber (host-visible in the
//! run log). No FFI export, no ABI bump.
//!
//! This file pins the IN-PROCESS half over `GraphRuntime::build_for_test` (real
//! iceoryx2, per-test SHM root — no fake data, Principle #13). A cdylib-level
//! stderr assertion (the `cdylib_tracing_stopgap_test` subprocess pattern) is
//! NOT built here; the in-process Drop path shares
//! the SAME `NodeContext::log_reconciliation_stats_at_teardown` code the cdylib
//! runs, so removing the teardown log fails these pins.
//!
//! Pins:
//! - **(a) node-side teardown log fires** — a real in-process producer graph is
//!   built and stepped WITHOUT the host harvest; `runtime.shutdown()` (the
//!   production teardown that drops the contexts) emits exactly ONE
//!   `"producer reconciliation (per topic)"` line carrying the
//!   producer's `node_id`, `topic`, and the exact committed-frame count (a hand
//!   oracle = steps taken). The `logs_assert` count == 1 FAILS if the Drop log
//!   is removed (count 0) — the mutation guard.
//! - **(b) at-most-once coordination** — calling the host harvest FIRST logs the
//!   per-topic line and MARKS the context; the subsequent teardown Drop is a
//!   no-op, so the marker still appears exactly ONCE (not twice). This FAILS if
//!   the `recon_logged` guard is removed (count 2).
//! - **(c) producer-less / consumer node stays silent** — the consumer (no
//!   outputs) emits no marker line; folded into (a)'s exact count == 1.
//! - **(d) a planning-only build is silent, a never-stepped execution build is
//!   not**: the same graph built `BuildPurpose::PlanningOnly` (the multi-process
//!   supervisor's planning build) and dropped without a step emits
//!   NO data line, while the same graph built `Execution` and dropped without
//!   a step emits its line with `producer_committed_frames=0`: the
//!   discriminator is the declared purpose, never the counters. A gate on zero
//!   counters passes the first half and FAILS the second.
//!
//! `#[serial]` to keep the `#[traced_test]` global log capture deterministic.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test publisher_recon_teardown_iox2_test -- --test-threads=1
//! ```

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, NodeEntry, NodeInfo};
use cerulion_core::graph::{BuildPurpose, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;
use tracing_test::traced_test;

/// The grep marker shared by BOTH the host harvest and the node-side teardown
/// log (hand-pasted oracle — NOT auto-extracted from the source). One grep of
/// this collects producer terms from in-process AND cdylib graphs alike; a
/// drift on EITHER side fails the pin exercising that side.
const MARKER: &str = "producer reconciliation (per topic)";

/// A field present ONLY on the per-publisher DATA lines (host per-topic AND
/// node-side teardown), never on the empty-graph NOTE (whose "grep '<MARKER>'"
/// instruction embeds the marker text). Counting this isolates producer data
/// lines from the note.
const DATA_FIELD: &str = "producer_next_sequence=";

/// Steps to drive before teardown. The producer is `period_ms = 10` and each
/// `step(10ms)` fires it exactly once, so after `STEPS` steps the publisher's
/// next sequence — and its committed-frame count — is exactly `STEPS` (seqs
/// start at 0). A HAND ORACLE, not a self-compare.
const STEPS: u32 = 5;

// ---------------------------------------------------------------------------
// Nodes: a Period(10) producer publishing an incrementing counter into a fixed
// Vector3 field, and a trigger consumer (no outputs) wired to it so the topic
// has a reader. Both are in-process macro nodes; only the producer owns a
// publisher, so only the producer should emit a teardown reconciliation line.
// ---------------------------------------------------------------------------

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct ReconProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl ReconProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

#[cerulion_node]
#[derive(Default)]
struct ReconConsumer {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl ReconConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Read the trigger input (keeps the port live); value unused.
        let _ = self.inp.x;
        Ok(())
    }
}

/// The producer id (also the middle segment of its output topic name).
// A node id NOT contained in MARKER, so `logs_contain(PRODUCER_ID)`
// genuinely verifies the node_id FIELD reached the log line
// (an id such as "producer" is a substring of MARKER —
// the assert would be vacuous).
const PRODUCER_ID: &str = "recon_xz_prod";
const CONSUMER_ID: &str = "consumer";

/// The `producer -> consumer` topology, shared by [`build`] (its own isolated
/// transport) and [`build_seeded`] (a transport the caller owns so it can
/// declare a replay sequence seed on it).
fn recon_graph(prefix: &str) -> GraphConfig {
    GraphConfig {
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "publisher_recon_teardown".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: PRODUCER_ID.to_string(),
                node_type: "recon_producer".to_string(),
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
                ros2: None,
                id: CONSUMER_ID.to_string(),
                node_type: "recon_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "recon_xz_prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    }
}

/// Build the `producer -> consumer` graph over an isolated test transport with
/// the supplied producer entry (macro or closure).
fn build(prefix: &str, producer: Box<dyn NodeEntry>) -> GraphRuntime {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(PRODUCER_ID.to_string(), producer);
    factories.insert(CONSUMER_ID.to_string(), Box::new(ReconConsumerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(recon_graph(prefix), factories, clock, 8)
        .expect("build publisher-recon-teardown graph")
}

/// Count producer DATA lines (host per-topic OR node-side teardown) in a
/// captured `#[traced_test]` buffer — excludes the empty-graph note.
fn data_line_count(lines: &[&str]) -> usize {
    lines.iter().filter(|l| l.contains(DATA_FIELD)).count()
}

/// Build a host-harvestable `ClosureNodeEntry` producer (Period(10)) that loans
/// its `out` publisher and writes a constant each fire. A closure — NOT a macro
/// — because the host harvest only reaches `ClosureNodeEntry` (the in-process
/// entry that forwards `collect_publisher_recon_stats`); the coordination pin
/// (b) needs the host harvest to actually mark the context.
fn closure_producer() -> Box<dyn NodeEntry> {
    let info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let producer = ClosureNodeEntry::new(info, move |ctx| {
        let mut proxy = ctx
            .publisher_mut("out")
            .expect("out publisher wired")
            .loan_proxy::<Vector3>()?;
        proxy.x = 1.0;
        Ok(())
    })
    .with_label(PRODUCER_ID);
    Box::new(producer)
}

// ===========================================================================
// (a) The node-side teardown log fires for an in-process producer WITHOUT the
//     host harvest — the cdylib-equivalent path. Exactly one marker line,
//     carrying the producer's node_id, topic, and the hand-oracle sequence.
//     `logs_assert` count == 1 is the mutation guard (0 if the Drop log is gone).
// ===========================================================================
#[traced_test]
#[serial]
#[test]
fn node_side_teardown_log_fires_with_fields_for_in_process_producer() {
    // A MACRO producer — the exact cdylib-equivalent (a `graph run` node is a
    // macro node whose host-harvest forwarder is a no-op), so this pins the very
    // path the cdylib runs via `impl Drop for NodeContext`.
    let mut runtime = build("recon_td_a", Box::new(ReconProducerEntry::new()));
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }
    // Production teardown: drops the node contexts -> `impl Drop for
    // NodeContext` -> the reconciliation log. NO host harvest first, so the
    // producer context is unmarked and the node-side path is the one that fires.
    runtime.shutdown();

    // Marker present, and it names the producer (node_id) + its topic.
    assert!(
        logs_contain(MARKER),
        "the node-side teardown reconciliation log must fire"
    );
    assert!(
        logs_contain(PRODUCER_ID),
        "the teardown log must carry the producer node_id field"
    );
    // The producer's output topic is `<prefix>/recon_xz_prod/out`.
    assert!(
        logs_contain("recon_td_a/recon_xz_prod/out"),
        "the teardown log must carry the producer topic field"
    );
    // Hand oracle: STEPS publishes -> next_sequence == committed_frames == STEPS.
    assert!(
        logs_contain(&format!("producer_next_sequence={STEPS}")),
        "the teardown log must carry the exact producer next sequence (hand oracle = STEPS)"
    );
    assert!(
        logs_contain(&format!("producer_committed_frames={STEPS}")),
        "the teardown log must carry the exact committed-frame count (hand oracle = STEPS)"
    );

    // Exactly ONE producer data line: the single producer publisher fired once,
    // and the producer-less consumer stayed silent. Count 0 == the Drop log was
    // removed (mutation guard); count 2 == the consumer wrongly emitted one.
    logs_assert(|lines: &[&str]| match data_line_count(lines) {
        1 => Ok(()),
        n => Err(format!(
            "expected exactly 1 teardown data line (producer only), got {n}"
        )),
    });
}

// ===========================================================================
// (b) At-most-once coordination: the HOST harvest fires FIRST (logs the
//     per-topic line + marks the context), so the subsequent teardown Drop is
//     suppressed. The marker still appears exactly ONCE. Count 2 == the
//     `recon_logged` guard was removed (both the host harvest AND the Drop log
//     fired for the same publisher).
// ===========================================================================
#[traced_test]
#[serial]
#[test]
fn host_harvest_suppresses_node_side_teardown_log_at_most_once() {
    // A CLOSURE producer — the ONLY in-process entry the host harvest reaches
    // (macro entries don't forward `collect_publisher_recon_stats`), so the
    // harvest actually marks the context and the coordination is exercised.
    let mut runtime = build("recon_td_b", closure_producer());
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }
    // Host harvest FIRST: logs the per-topic line and marks the in-process
    // producer context as surfaced.
    runtime.log_publisher_reconciliation_stats();
    // Teardown: the Drop path must NOT re-log the already-surfaced producer.
    runtime.shutdown();

    assert!(
        logs_contain(MARKER),
        "the host harvest must log the per-topic reconciliation line"
    );
    logs_assert(|lines: &[&str]| match data_line_count(lines) {
        1 => Ok(()),
        n => Err(format!(
            "expected exactly 1 producer data line (host harvest; node-side \
             teardown suppressed by the recon_logged guard), got {n}"
        )),
    });
}

// ===========================================================================
// The producer term is frames THIS RUN committed, not the
// wire counter. A restored replay seeds the counter with the recorded stream's
// next sequence, after which `sequence()` and "frames committed" differ by
// exactly the seed — and reporting the counter mints the whole seed as phantom
// loss against bag-side terms that only ever saw this run's frames.
// ===========================================================================

/// The seed a restored replay would declare. Deliberately far from `STEPS` so a
/// conflation is unmistakable in the failure text rather than an off-by-a-few.
const RESTORE_SEED: u32 = 4242;

/// Build the same producer graph over a manager whose replay seed table names
/// the producer's topic, and RETURN the manager: `GraphRuntime::build` does not
/// take ownership of it the way `build_for_test` does, so the caller must keep
/// it alive for the graph's lifetime.
fn build_seeded(
    prefix: &str,
    producer: Box<dyn NodeEntry>,
    seed: u32,
) -> (Arc<TransportManager>, GraphRuntime) {
    let clock = Arc::new(VirtualClock::new());
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "recon".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated manager");

    // The seed table is keyed by the RESOLVED topic name, which carries the
    // leading slash the runtime derives (`/{prefix}/{node}/{output}`) — the
    // same string the log line's `topic=` field shows. A key without it silently
    // seeds nothing, which is why both arms assert the seed came THROUGH rather
    // than only that the committed count is 5 (it is 5 either way).
    let mut seeds = BTreeMap::new();
    seeds.insert(format!("/{prefix}/{PRODUCER_ID}/out"), seed);
    mgr.set_replay_sequence_seeds(seeds);

    let config = recon_graph(prefix);
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(PRODUCER_ID.to_string(), producer);
    factories.insert(CONSUMER_ID.to_string(), Box::new(ReconConsumerEntry::new()));

    let runtime = GraphRuntime::build(config, factories, &mgr, clock).expect("build seeded graph");
    (mgr, runtime)
}

/// Read `key=` off the one producer DATA line as a whole WHITESPACE TOKEN.
///
/// A bare substring match is the hazard: `producer_committed_frames=5`
/// is a prefix of `producer_committed_frames=50`, and this arm's whole point is
/// separating 5 from 4247.
fn data_line_field(lines: &[&str], key: &str) -> Result<String, String> {
    let data: Vec<&&str> = lines.iter().filter(|l| l.contains(DATA_FIELD)).collect();
    if data.len() != 1 {
        return Err(format!(
            "expected exactly 1 producer data line, got {}",
            data.len()
        ));
    }
    data[0]
        .split_whitespace()
        .find_map(|tok| tok.strip_prefix(key).map(|v| v.to_string()))
        .ok_or_else(|| format!("no `{key}` token on the data line: {}", data[0]))
}

#[traced_test]
#[serial]
#[test]
fn the_host_harvest_reports_frames_this_run_committed_not_the_seed_plus_them() {
    // A CLOSURE producer: the host harvest only reaches `ClosureNodeEntry`.
    let (_mgr, mut runtime) = build_seeded("recon_seed_h", closure_producer(), RESTORE_SEED);
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }
    runtime.log_publisher_reconciliation_stats();

    // All three quantities on ONE line, each read as a whole token. The counter
    // and the committed count MUST disagree here — that disagreement is the
    // whole condition, and asserting both is what makes the pin insensitive to
    // which of them a regression reports.
    logs_assert(|lines: &[&str]| {
        let next = data_line_field(lines, "producer_next_sequence=")?;
        let initial = data_line_field(lines, "producer_initial_sequence=")?;
        let committed = data_line_field(lines, "producer_committed_frames=")?;
        if next != (RESTORE_SEED + STEPS).to_string() {
            return Err(format!(
                "the raw wire counter must still be the seed plus this run's frames: \
                 producer_next_sequence={next}, expected {}",
                RESTORE_SEED + STEPS
            ));
        }
        if initial != RESTORE_SEED.to_string() {
            return Err(format!(
                "the seed must be reported as its own field (Principle #3): \
                 producer_initial_sequence={initial}, expected {RESTORE_SEED}"
            ));
        }
        if committed != STEPS.to_string() {
            return Err(format!(
                "producer_committed_frames must be frames THIS RUN committed \
                 (hand oracle = STEPS = {STEPS}), got {committed}. Reporting the raw \
                 counter here mints {RESTORE_SEED} frames of phantom loss against \
                 record_health.json"
            ));
        }
        Ok(())
    });
}

#[traced_test]
#[serial]
#[test]
fn the_node_side_teardown_log_reports_frames_this_run_committed() {
    // The SECOND reconciliation site: no host harvest, so the `NodeContext`
    // Drop path is the one that logs (the cdylib-equivalent). It reads the
    // publisher directly rather than a harvested stat, so it is a separate
    // `tracing` call site with its own copy of the field list — and therefore
    // needs its own pin.
    let (_mgr, mut runtime) = build_seeded("recon_seed_n", closure_producer(), RESTORE_SEED);
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }
    runtime.shutdown();

    logs_assert(|lines: &[&str]| {
        let next = data_line_field(lines, "producer_next_sequence=")?;
        let initial = data_line_field(lines, "producer_initial_sequence=")?;
        let committed = data_line_field(lines, "producer_committed_frames=")?;
        if next != (RESTORE_SEED + STEPS).to_string() {
            return Err(format!("producer_next_sequence={next}"));
        }
        if initial != RESTORE_SEED.to_string() {
            return Err(format!("producer_initial_sequence={initial}"));
        }
        if committed != STEPS.to_string() {
            return Err(format!(
                "producer_committed_frames must be frames THIS RUN committed \
                 (hand oracle = STEPS = {STEPS}), got {committed}"
            ));
        }
        Ok(())
    });
}

// ===========================================================================
// (d) A PLANNING-ONLY build is silent at teardown; an EXECUTION build that
//     never stepped is not. The multi-process supervisor builds the full graph
//     once at plan time (every node's `init()` runs, so a cdylib node takes its
//     context across the FFI exactly as for a run) and shuts it down before any
//     worker spawns; without the discriminator that prints two all-zero reconciliation lines at
//     the TOP of every recorded run, before the first worker exists. The
//     discriminator is the DECLARED purpose: a producer that never published
//     is a real reconciliation fact on a runtime that was meant to run, so the
//     second half of this body pins that such a build still reports (with a
//     zero count). Gating the line on zero counters passes the first
//     half and fails the second; deleting the planning-only mark fails
//     the first.
// ===========================================================================

/// Build the producer graph over a manager the caller owns, through the
/// fuller virtual-path entry that carries the build PURPOSE (the same entry
/// the supervisor's planning build calls). `GraphRuntime::build` does not take
/// ownership of the manager the way `build_for_test` does, so it is returned.
fn build_with_purpose(
    prefix: &str,
    producer: Box<dyn NodeEntry>,
    purpose: BuildPurpose,
) -> (Arc<TransportManager>, GraphRuntime) {
    let clock = Arc::new(VirtualClock::new());
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "recon_purpose".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated manager");
    let config = recon_graph(prefix);
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(PRODUCER_ID.to_string(), producer);
    factories.insert(CONSUMER_ID.to_string(), Box::new(ReconConsumerEntry::new()));
    let runtime = GraphRuntime::build_with_schema_hashes_provisioning_and_recorded(
        config, factories, &mgr, clock, None, None, None, purpose,
    )
    .expect("build graph with a declared purpose");
    (mgr, runtime)
}

#[traced_test]
#[serial]
#[test]
fn a_planning_only_build_is_silent_at_teardown_but_a_never_stepped_run_still_reports() {
    // HALF 1: the planning build. Built, never stepped, torn down through the
    // production teardown (the exact shape of the supervisor's planning
    // build). The context was marked at build, so the Drop path
    // prints NOTHING: zero data lines.
    let (mgr, runtime) = build_with_purpose(
        "recon_plan_only",
        Box::new(ReconProducerEntry::new()),
        BuildPurpose::PlanningOnly,
    );
    runtime.shutdown();
    drop(mgr);
    logs_assert(|lines: &[&str]| match data_line_count(lines) {
        0 => Ok(()),
        n => Err(format!(
            "a planning-only build must print no teardown reconciliation line, got {n}"
        )),
    });

    // HALF 2 (the anti-tautology twin): the SAME graph declared for execution,
    // also never stepped. Its producer really did commit zero frames and that
    // is reported: exactly one data line, carrying a ZERO count. This is the
    // half that fails if the report is skipped when the counters are zero.
    let (mgr, runtime) = build_with_purpose(
        "recon_exec_zero",
        Box::new(ReconProducerEntry::new()),
        BuildPurpose::Execution,
    );
    runtime.shutdown();
    drop(mgr);
    logs_assert(|lines: &[&str]| {
        let committed = data_line_field(lines, "producer_committed_frames=")?;
        if committed != "0" {
            return Err(format!(
                "an execution build that never stepped reports its zero count, got \
                 producer_committed_frames={committed}"
            ));
        }
        Ok(())
    });
    // Attribution, scoped to the DATA line. A whole-capture `logs_contain` is
    // the wrong probe here: the capture also holds the build's own `debug!`
    // lines (`publisher created topic=..`, `subscriber created topic=..`), and
    // those name the planning build's topic by construction, so an absence
    // check over every line fails on a correct implementation.
    logs_assert(|lines: &[&str]| {
        let data: Vec<&&str> = lines.iter().filter(|l| l.contains(DATA_FIELD)).collect();
        if let Some(line) = data
            .iter()
            .find(|l| l.contains("recon_plan_only/recon_xz_prod/out"))
        {
            return Err(format!(
                "no reconciliation line may name the planning-only build's topic: {line}"
            ));
        }
        match data
            .iter()
            .filter(|l| l.contains("recon_exec_zero/recon_xz_prod/out"))
            .count()
        {
            1 => Ok(()),
            n => Err(format!(
                "the one data line must belong to the EXECUTION build's topic, got {n} such line(s)"
            )),
        }
    });
}
