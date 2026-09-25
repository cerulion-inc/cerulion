// SPDX-License-Identifier: AGPL-3.0-only
//! The RESTORE seam driven through a REAL `GraphRuntime`
//! over REAL iceoryx2 (`build_for_test`, per-test SHM roots ⇒ parallel-safe).
//!
//! `state_restore_test.rs` pins the DECISIONS. This file pins that the seam
//! those decisions feed actually reaches a node: recorded bytes are applied,
//! the node resumes from them rather than from its constructor, and the value
//! it resumes with crosses SHM to a real consumer.
//!
//! # Why the node is hand-written rather than a `#[cerulion_node]`
//!
//! It stays hand-written even though every macro node's `NodeEntry`
//! forwards the derived `state_shape()`
//! impl, because the seam
//! under test is `NodeEntry::{state_shape, restore_state}` plus
//! `GraphRuntime::restore_node_states`, and a hand-written entry drives
//! exactly that surface without also depending on what the macro chose to fold
//! in. `CerulionState` is implemented by hand for the same reason, which is
//! what its `INLINE_SAFE`-defaults-false discipline exists to make safe. The
//! macro's own answers are pinned where they are generated —
//! `cdylib_state_ffi_test.rs` reads the shape off a LOADED macro cdylib rather
//! than restating it.
//!
//! Nothing here hardcodes a shape: every anchor is framed by
//! `capture_anchor_blob`, so the recorded side and the build side read the one
//! associated const and cannot drift apart.
//!
//! # The ordering pin is the reason for the marker log
//!
//! The restore order is fixed: `init()` → apply fields → `restored()` → first
//! `tick()`, because `init()` opens handles from config fields whose values
//! are still `Default` until the state lands. A test that only checked the
//! restored VALUE would pass just as happily against a restore that ran after
//! the first tick, so each phase appends its own marker and the sequence
//! itself is the oracle.

#![cfg(unix)]

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::{TransportError, TransportResult};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, MacroPolicy, NodeContext, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::state::{
    CerulionState, StateCursor, StateError, StateShape, StateSink, VecSink,
};
use cerulion_core::state_restore::capture_anchor_blob;
use cerulion_core::transport::{
    PublisherProvisioning, TopicServiceConfig, TransportConfig, TransportManager,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// ---------------------------------------------------------------------------
// A node with real, restorable state
// ---------------------------------------------------------------------------

/// The user struct: one counter, captured and restored by hand.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct CounterState {
    count: u64,
}

impl CerulionState for CounterState {
    const STATE_SHAPE: u64 = StateShape::of("CounterState")
        .field("count", <u64 as CerulionState>::STATE_SHAPE)
        .finish();

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        self.count.cer_capture(out)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Self {
            count: u64::cer_read(src)?,
        })
    }
}

/// A DIFFERENT type with the same encoded bytes and a different shape — the
/// only faithful way to build a drift fixture, since a shape is a function of
/// the declaration and nothing else.
#[derive(Debug, Default)]
struct RenamedCounterState {
    count: u64,
}

impl CerulionState for RenamedCounterState {
    const STATE_SHAPE: u64 = StateShape::of("RenamedCounterState")
        .field("ticks", <u64 as CerulionState>::STATE_SHAPE)
        .finish();

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        self.count.cer_capture(out)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Self {
            count: u64::cer_read(src)?,
        })
    }
}

/// The phases a node passes through, appended in the order they happen.
type Phases = Arc<Mutex<Vec<String>>>;

/// A publishing node whose state is a counter. Each tick publishes the current
/// count into `out.x` and then advances it, so a restored node's FIRST frame
/// carries the count it was restored with.
struct CounterEntry {
    inner: CounterState,
    /// When true this entry declares `RenamedCounterState`'s shape while
    /// carrying `CounterState`'s bytes — the drift fixture.
    drifted: bool,
    /// When true this entry declares NO state at all (the staged-rollout /
    /// stateless population).
    stateless: bool,
    context: Option<NodeContext>,
    phases: Phases,
}

impl CounterEntry {
    fn new(phases: Phases) -> Self {
        Self {
            inner: CounterState::default(),
            drifted: false,
            stateless: false,
            context: None,
            phases,
        }
    }

    fn drifted(mut self) -> Self {
        self.drifted = true;
        self
    }

    fn stateless(mut self) -> Self {
        self.stateless = true;
        self
    }

    fn note(&self, phase: &str) {
        self.phases.lock().expect("phases").push(phase.to_string());
    }
}

impl NodeEntry for CounterEntry {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.note("init");
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        self.note(&format!("tick:{}", self.inner.count));
        let ctx = self.context.as_mut().expect("init ran");
        let publisher = ctx.publisher_mut("out").expect("output 'out' is wired");
        let mut proxy = publisher.loan_proxy::<Vector3>()?;
        proxy.x = self.inner.count as f64;
        drop(proxy);
        self.inner.count += 1;
        Ok(())
    }

    fn state_shape(&self) -> Option<u64> {
        if self.stateless {
            None
        } else if self.drifted {
            Some(RenamedCounterState::STATE_SHAPE)
        } else {
            Some(CounterState::STATE_SHAPE)
        }
    }

    fn restore_state(&mut self, payload: &[u8]) -> TransportResult<()> {
        let mut cursor = StateCursor::new(payload);
        let restored =
            CounterState::cer_read(&mut cursor).map_err(|e| TransportError::GraphError {
                reason: format!("counter state decode failed: {e}"),
            })?;
        cursor.finish().map_err(|e| TransportError::GraphError {
            reason: format!("counter state has trailing bytes: {e}"),
        })?;
        self.inner = restored;
        // The `restored()` hook runs HERE — after the fields are back and
        // before the entry returns, so the runtime's next call is `tick()`.
        self.note("restored");
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// graph + harness
// ---------------------------------------------------------------------------

/// A node that DECLARES a state shape and then leaves
/// [`NodeEntry::restore_state`] at its trait default.
///
/// This is the dangerous half-implementation, and it is the only population
/// the default's refusal can protect: `restore_node_states` skips a node whose
/// `state_shape()` is `None` before it ever reaches `restore_state`, so the
/// default is reachable ONLY through a node that claims a shape it cannot
/// apply. If the default succeeded silently the runtime would report that node
/// `restored` while its bytes went nowhere — a replay then diverges against
/// state it quietly dropped, which is exactly what the refusal exists to stop.
struct HalfDeclaredEntry {
    context: Option<NodeContext>,
}

impl NodeEntry for HalfDeclaredEntry {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 10 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        Ok(())
    }

    fn state_shape(&self) -> Option<u64> {
        Some(CounterState::STATE_SHAPE)
    }

    // `restore_state` is DELIBERATELY not overridden.
}

fn counter_graph(prefix: &str) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "restore".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "counter".to_string(),
                node_type: "counter".to_string(),
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
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "counter/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    }
}

/// Build the graph with a fresh `CounterEntry` plus a closure sink that
/// records every delivered `x`.
fn build(
    prefix: &str,
    counter: CounterEntry,
    seen: Arc<Mutex<Vec<u64>>>,
) -> TransportResult<GraphRuntime> {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(counter));

    let info = NodeInfo::from_names(vec!["inp".to_string()], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let sink = ClosureNodeEntry::new(info, move |ctx| {
        let sub = ctx.subscriber("inp").expect("sink subscriber wired");
        let mut batch = Vec::new();
        let _ = sub.try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().expect("8 bytes"));
            batch.push(x as u64);
        })?;
        seen.lock().expect("seen").extend(batch);
        Ok(())
    })
    .with_label("sink")
    .with_unified_drain(false);
    factories.insert("sink".to_string(), Box::new(sink));

    GraphRuntime::build_for_test(
        counter_graph(prefix),
        factories,
        Arc::new(VirtualClock::new()),
        16,
    )
}

fn prefix_for(tag: &str) -> String {
    format!("r9{}{}", tag, std::process::id() % 1000)
}

/// A framed anchor blob for a counter at `count`, built through the PRODUCTION
/// capture helper rather than by hand-assembling bytes.
fn anchor_for(count: u64) -> Vec<u8> {
    let mut sink = VecSink::new();
    capture_anchor_blob(&CounterState { count }, &mut sink).expect("capture fits");
    sink.into_inner()
}

// ===========================================================================
// the seam
// ===========================================================================

#[test]
fn a_restored_node_resumes_from_its_recorded_count_not_from_its_constructor() {
    // The oracle is HAND-WRITTEN: a node restored at 100 publishes 100, 101,
    // 102 — a constructor-fresh node would publish 0, 1, 2, and a restore that
    // landed a step late would publish 0, 100, 101.
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("a");
    let mut runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)),
        Arc::clone(&seen),
    )
    .expect("graph builds");

    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), anchor_for(100));
    let report = runtime
        .restore_node_states(&anchors)
        .expect("the anchor applies");
    assert_eq!(report.restored, vec!["counter".to_string()]);
    assert!(report.declared_no_state.is_empty());
    assert!(report.anchors_unused.is_empty());

    // FOUR steps for THREE delivered values: the sink drains its queue at the
    // top of its own tick, so the frame a step publishes is observed on the
    // NEXT one. That lag is the level executor's ordinary shape, not a
    // property of the restore, and it is the same on both sides of the
    // anti-tautology control below.
    for _ in 0..4 {
        runtime.step(Duration::from_millis(10));
    }

    let delivered = seen.lock().expect("seen").clone();
    assert_eq!(
        delivered,
        vec![100, 101, 102],
        "the restored count crossed SHM to a real consumer"
    );
}

#[test]
fn a_graph_with_no_anchor_is_byte_unchanged_from_the_constructor_path() {
    // The ANTI-TAUTOLOGY control for the test above: without the restore the
    // same graph publishes from zero, so "resumes from 100" is a property of
    // the restore rather than of the fixture.
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("b");
    let mut runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)),
        Arc::clone(&seen),
    )
    .expect("graph builds");

    // An EMPTY anchor set is the from-start plan's shape, and it must be a
    // no-op rather than an error.
    let report = runtime
        .restore_node_states(&BTreeMap::new())
        .expect("no anchors is not a failure");
    assert!(report.restored.is_empty());

    for _ in 0..4 {
        runtime.step(Duration::from_millis(10));
    }
    assert_eq!(seen.lock().expect("seen").clone(), vec![0, 1, 2]);
}

#[test]
fn the_restore_lands_after_init_and_before_the_first_tick() {
    // The fixed restore order, asserted as a SEQUENCE rather than as a value: a
    // restore that ran before `init()` would leave a node whose handles were
    // opened from `Default` config, and one that ran after the first tick
    // would overwrite state the graph had already advanced.
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("c");
    let mut runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)),
        Arc::clone(&seen),
    )
    .expect("graph builds");

    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), anchor_for(7));
    runtime.restore_node_states(&anchors).expect("applies");
    runtime.step(Duration::from_millis(10));

    assert_eq!(
        phases.lock().expect("phases").clone(),
        vec![
            "init".to_string(),
            "restored".to_string(),
            "tick:7".to_string()
        ],
        "init -> apply fields -> restored() -> first tick"
    );
}

#[test]
fn a_shape_mismatch_refuses_the_build_and_names_both_hashes() {
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("d");
    let runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)).drifted(),
        Arc::clone(&seen),
    )
    .expect("graph builds");

    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), anchor_for(100));
    let err = runtime
        .restore_node_states(&anchors)
        .expect_err("the recorded shape is not this build's");
    let text = err.to_string();
    assert!(text.contains("counter"), "names the node: {text}");
    assert!(
        text.contains(&format!("{:#018x}", CounterState::STATE_SHAPE)),
        "prints the RECORDED shape: {text}"
    );
    assert!(
        text.contains(&format!("{:#018x}", RenamedCounterState::STATE_SHAPE)),
        "prints THIS BUILD's shape: {text}"
    );
    assert_eq!(
        phases.lock().expect("phases").clone(),
        vec!["init".to_string()],
        "the node was never handed bytes it could not interpret"
    );
}

#[test]
fn an_unframed_anchor_refuses_rather_than_reaching_the_node() {
    // The bytes are a VALID `CounterState` encoding with no anchor framing —
    // exactly what a capture that skipped `capture_anchor_blob` produces. A
    // decoder that sniffed would consume the first 16 bytes as a header and
    // hand the node garbage.
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("e");
    let runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)),
        Arc::clone(&seen),
    )
    .expect("graph builds");

    let mut bare = VecSink::new();
    CounterState { count: 100 }
        .cer_capture(&mut bare)
        .expect("capture");
    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), bare.into_inner());

    let err = runtime.restore_node_states(&anchors).expect_err("unframed");
    assert!(
        err.to_string().contains("anchor framing"),
        "names the framing: {err}"
    );
    assert_eq!(
        phases.lock().expect("phases").clone(),
        vec!["init".to_string()]
    );
}

#[test]
fn a_node_that_declares_no_state_is_reported_rather_than_silently_restored() {
    // The staged-rollout population: the anchor is offered, the node
    // cannot take it, and the runtime SAYS so instead of pretending. That
    // report is exactly what `--strict-state` judges.
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("f");
    let mut runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)).stateless(),
        Arc::clone(&seen),
    )
    .expect("graph builds");

    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), anchor_for(100));
    let report = runtime.restore_node_states(&anchors).expect("not an error");
    assert!(report.restored.is_empty());
    assert_eq!(report.declared_no_state, vec!["counter".to_string()]);

    // And the node really did NOT take the state — it runs from its
    // constructor, which is the default reading the report describes.
    runtime.step(Duration::from_millis(10));
    runtime.step(Duration::from_millis(10));
    assert_eq!(seen.lock().expect("seen").clone(), vec![0]);

    // The same fact, read the way `--strict-state` reads it. The `sink` is a
    // `ClosureNodeEntry`, which declares no state either — so this also pins
    // that the accessor reports EVERY stateless node in the set it is given,
    // not just the interesting one.
    let executed: BTreeSet<String> = ["counter", "sink"].iter().map(|s| s.to_string()).collect();
    let without = runtime.nodes_without_state(&executed).expect("readable");
    assert_eq!(without, vec!["counter".to_string(), "sink".to_string()]);

    // The accessor is SCOPED to
    // the nodes the caller says the replay executes. A node outside that set
    // never ticks, so its lack of state cannot change a replayed byte — and
    // reporting it made `--strict-state` refuse recordings that covered
    // everything the run actually read.
    let only_sink: BTreeSet<String> = ["sink"].iter().map(|s| s.to_string()).collect();
    assert_eq!(
        runtime.nodes_without_state(&only_sink).expect("readable"),
        vec!["sink".to_string()],
        "a node outside the executed set is not reported"
    );
    assert!(
        runtime
            .nodes_without_state(&BTreeSet::new())
            .expect("readable")
            .is_empty(),
        "an empty executed set reports nothing at all"
    );
}

#[test]
fn an_anchor_for_a_node_this_graph_does_not_contain_is_reported_not_ignored() {
    // A machine-wide recording legitimately carries other runs' anchors, and a
    // typo'd node name looks identical from inside the runtime — so the fact
    // is handed back rather than judged here.
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("g");
    let runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)),
        Arc::clone(&seen),
    )
    .expect("graph builds");

    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), anchor_for(5));
    anchors.insert("someone_elses_node".to_string(), anchor_for(9));
    let report = runtime.restore_node_states(&anchors).expect("applies");
    assert_eq!(report.restored, vec!["counter".to_string()]);
    assert_eq!(
        report.anchors_unused,
        vec!["someone_elses_node".to_string()]
    );
}

#[test]
fn a_node_that_declares_a_shape_but_cannot_apply_bytes_refuses_rather_than_reporting_success() {
    // The default `restore_state`'s refusal, driven at the ONLY place it is
    // reachable. A node whose `state_shape()` is `None` is skipped before
    // `restore_state` is called at all, so without this arm the default could
    // silently return `Ok(())` and the runtime would report a node `restored`
    // whose bytes were dropped on the floor — the exact silent-divergence
    // class the refusal is written to prevent.
    let seen = Arc::new(Mutex::new(Vec::new()));
    let prefix = prefix_for("h");

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "counter".to_string(),
        Box::new(HalfDeclaredEntry { context: None }),
    );
    let info = NodeInfo::from_names(vec!["inp".to_string()], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let seen_cb = Arc::clone(&seen);
    let sink = ClosureNodeEntry::new(info, move |ctx| {
        let sub = ctx.subscriber("inp").expect("sink subscriber wired");
        let mut batch = Vec::new();
        let _ = sub.try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().expect("8 bytes"));
            batch.push(x as u64);
        })?;
        seen_cb.lock().expect("seen").extend(batch);
        Ok(())
    })
    .with_label("sink")
    .with_unified_drain(false);
    factories.insert("sink".to_string(), Box::new(sink));

    let runtime = GraphRuntime::build_for_test(
        counter_graph(&prefix),
        factories,
        Arc::new(VirtualClock::new()),
        16,
    )
    .expect("graph builds");

    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), anchor_for(100));
    let err = runtime
        .restore_node_states(&anchors)
        .expect_err("a declared shape with no restore path must refuse");
    assert!(
        err.to_string().contains("declares no restorable state"),
        "names the condition: {err}"
    );
}

// ===========================================================================
// the publisher-sequence seed, applied at the runtime
// ===========================================================================

fn manager(name: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated manager")
}

/// The wire `sequence` of the first frame a freshly-created publisher commits
/// on `topic`, read off the WIRE through a data-only tap.
fn first_committed_sequence(mgr: &Arc<TransportManager>, topic: &str) -> u32 {
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(256), 0)
        .expect("publisher");
    let mut tap = mgr
        .create_data_only_subscriber(topic)
        .expect("tap attaches to the live topic");
    let proxy = publisher.loan_proxy::<Vector3>().expect("loan");
    drop(proxy);
    let mut frames = Vec::new();
    let drained = tap.drain_owned(8, &mut frames).expect("drain");
    assert_eq!(drained, 1, "exactly one frame was committed");
    WireHeader::read_from_buf(frames[0].payload())
        .expect("valid wire header")
        .sequence
}

#[test]
fn a_seeded_publisher_starts_at_the_declared_sequence_and_an_unseeded_one_at_zero() {
    // Both halves in ONE body: without the control, "starts at 4242" is
    // satisfied by any implementation that ignores the table and happens to be
    // fed a seeded topic, and without the seeded half the control is vacuous.
    let mgr = manager("seed");

    let mut seeds = BTreeMap::new();
    seeds.insert("/restored".to_string(), 4242u32);
    mgr.set_replay_sequence_seeds(seeds);

    assert_eq!(
        first_committed_sequence(&mgr, "/restored"),
        4242,
        "the restored publisher continues the recorded stream's numbering"
    );
    assert_eq!(
        first_committed_sequence(&mgr, "/not_in_the_table"),
        0,
        "every topic with no seed is byte-unchanged from the live path"
    );
}

#[test]
fn a_manager_with_no_seeds_at_all_is_the_live_path() {
    // The from-start replay and every live run take THIS path, so it is pinned
    // separately from the "seeded some topics" case above: a table that was
    // never set must not perturb anything.
    let mgr = manager("noseed");
    assert_eq!(first_committed_sequence(&mgr, "/plain"), 0);
}

#[test]
fn a_seed_declared_for_a_multi_publisher_topic_refuses_the_publisher_rather_than_applying_one_counter_to_both(
) {
    // The seed table is keyed by TOPIC; the wire `sequence` it seeds is a
    // PER-PUBLISHER commit counter. On a `multi_publisher_topics:`-listed topic
    // one seed would therefore be handed to every publisher and be wrong for at
    // least one of them, and every frame it emitted would read as a byte
    // mismatch that looks like a node change. The pure layer refuses to DERIVE
    // such a seed; this is the backstop at the one place a seed can reach a
    // counter, so a future adapter that forgets the rule fails at build.
    let mgr = manager("multiseed");
    let multi = TopicServiceConfig::for_topology(
        mgr.default_topic_config(),
        8,
        1,
        PublisherProvisioning::Multi,
        0,
        0,
    );

    // CONTROL first, and on the SAME provisioning: without it, "a Multi topic
    // refuses" is satisfied by an implementation that refuses every Multi
    // publisher, seeded or not — which would break `/tf` on every live run.
    let unseeded = mgr
        .create_publisher_with_topic_config("/tf_unseeded", MaxSliceLen::const_new(256), 0, multi)
        .expect("a multi-publisher topic with NO seed is an ordinary live publisher");
    assert_eq!(
        unseeded.sequence(),
        0,
        "an unseeded publisher starts where it always has"
    );

    let mut seeds = BTreeMap::new();
    seeds.insert("/tf".to_string(), 4242u32);
    mgr.set_replay_sequence_seeds(seeds);

    // `CerulionPublisher` is not `Debug`, so the Ok arm is named by hand rather
    // than through `expect_err`.
    let text = match mgr.create_publisher_with_topic_config(
        "/tf",
        MaxSliceLen::const_new(256),
        0,
        multi,
    ) {
        Ok(_) => panic!("one seed cannot serve two counters, so this must refuse"),
        Err(e) => e.to_string(),
    };
    assert!(text.contains("/tf"), "names the topic: {text}");
    assert!(
        text.contains("multi_publisher_topics"),
        "names WHY this topic is different: {text}"
    );
    assert!(
        text.contains("4242"),
        "names the seed that was declared, so the offending table entry is \
         identifiable: {text}"
    );

    // And the refusal is SCOPED to the listed topic: a single-writer sibling in
    // the same table is still seeded exactly.
    let mut seeds = BTreeMap::new();
    seeds.insert("/tf".to_string(), 4242u32);
    seeds.insert("/odom".to_string(), 77u32);
    mgr.set_replay_sequence_seeds(seeds);
    let single = TopicServiceConfig::for_topology(
        mgr.default_topic_config(),
        8,
        1,
        PublisherProvisioning::SingleWriter,
        0,
        0,
    );
    let seeded = mgr
        .create_publisher_with_topic_config("/odom", MaxSliceLen::const_new(256), 0, single)
        .expect("a single-writer topic is seedable");
    assert_eq!(seeded.sequence(), 77);
}

#[test]
fn a_restored_publisher_counts_the_frames_this_run_committed_not_the_seed_plus_them() {
    // `sequence()` is the NEXT wire number; it equalled the committed count
    // only because the counter always started at 0. A seeded replay breaks that
    // identity by exactly the seed, and the reconciliation reads the
    // committed count — so the two are now separate observables, and this pins
    // BOTH against a hand oracle plus their relationship.
    let mgr = manager("committed");
    let mut seeds = BTreeMap::new();
    seeds.insert("/restored".to_string(), 4242u32);
    mgr.set_replay_sequence_seeds(seeds);

    const FRAMES: u32 = 3;

    let mut seeded = mgr
        .create_publisher("/restored", MaxSliceLen::const_new(256), 0)
        .expect("publisher");
    for _ in 0..FRAMES {
        drop(seeded.loan_proxy::<Vector3>().expect("loan"));
    }
    assert_eq!(seeded.initial_sequence(), 4242, "the declared seed");
    assert_eq!(
        seeded.sequence(),
        4242 + FRAMES,
        "the WIRE counter continues the recording's numbering"
    );
    assert_eq!(
        seeded.committed_frames(),
        FRAMES,
        "this run committed exactly {FRAMES} frames; reporting 4245 would mint \
         4242 frames of phantom loss against the bag-side terms"
    );

    // The live-path control, over the SAME manager and the same frame count:
    // without it, "committed_frames == 3" is satisfied by an implementation
    // that ignores the seed entirely.
    let mut plain = mgr
        .create_publisher("/live", MaxSliceLen::const_new(256), 0)
        .expect("publisher");
    for _ in 0..FRAMES {
        drop(plain.loan_proxy::<Vector3>().expect("loan"));
    }
    assert_eq!(plain.initial_sequence(), 0, "the live path seeds nothing");
    assert_eq!(plain.sequence(), FRAMES);
    assert_eq!(
        plain.committed_frames(),
        FRAMES,
        "unseeded, the counter and the committed count agree — which is why the \
         conflation went unnoticed until a replay seeded one"
    );
}

#[test]
fn a_seeded_publisher_that_wraps_still_counts_this_runs_frames() {
    // The wire sequence is a u32 that really wraps (~49.7 days at 1 kHz). Past
    // the wrap neither raw value alone is usable, but their DIFFERENCE still
    // is — so the subtraction is `wrapping_sub`, mirroring the counter's own
    // `wrapping` advance.
    let mgr = manager("wrapseed");
    let mut seeds = BTreeMap::new();
    seeds.insert("/wrapped".to_string(), u32::MAX - 1);
    mgr.set_replay_sequence_seeds(seeds);

    let mut pubr = mgr
        .create_publisher("/wrapped", MaxSliceLen::const_new(256), 0)
        .expect("publisher");
    for _ in 0..3 {
        drop(pubr.loan_proxy::<Vector3>().expect("loan"));
    }
    assert_eq!(pubr.initial_sequence(), u32::MAX - 1);
    assert_eq!(
        pubr.sequence(),
        1,
        "u32::MAX - 1, MAX, 0 committed; next is 1"
    );
    assert_eq!(
        pubr.committed_frames(),
        3,
        "the difference survives the wrap even though neither raw value does"
    );
}

// ===========================================================================
// A restore onto a runtime that has already run is REFUSED
// ===========================================================================

/// `restore → run_live → shutdown → restore` is refused LOUDLY, and the
/// refusal names the reason and the fix.
///
/// # The bug this closes, stated as the sequence that produces it
///
/// `collect_external_sources` is IDEMPOTENT: its second call takes the resume
/// branch and reuses session 1's bindings without re-querying
/// `NodeEntry::external_source`. A `Blocking` source's closure captured
/// whatever the node held when it was collected — typically an `Arc` the
/// caller also holds a clone of — while a restore REPLACES the node's fields
/// with a freshly deserialized allocation. So after a second restore the
/// session-1 pump is still driving the PRE-restore handle, a key-op pushed
/// through the caller's clone reaches nothing, and NOTHING anywhere reports
/// it: the graph runs, the node ticks, and the input is simply never seen
/// again.
///
/// The oracle is the REFUSAL itself, and it is asserted on the seam a
/// production restore path uses, not on a test-only setter: a runtime that has
/// run a live session must not accept a restore. Its anti-tautology half is in
/// the same body — the SAME anchors applied to a fresh runtime succeed and the
/// node really resumes at the recorded count, so the refusal is about the
/// runtime's history and not about the anchors.
#[test]
fn a_restore_onto_a_runtime_that_has_already_run_live_is_refused_and_says_why() {
    let prefix = prefix_for("c6r");
    let seen = Arc::new(Mutex::new(Vec::new()));
    let phases: Phases = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = build(
        &prefix,
        CounterEntry::new(Arc::clone(&phases)),
        Arc::clone(&seen),
    )
    .expect("builds");

    let mut anchors = BTreeMap::new();
    anchors.insert("counter".to_string(), anchor_for(100));

    // A live session. `running` is already false, so the loop exits at once —
    // but `run_live` COLLECTS its external sources at entry, which is the state
    // that makes a later restore unsound. This is the production seam: a
    // graph that has run is exactly a graph whose bindings are built.
    let running = std::sync::atomic::AtomicBool::new(false);
    runtime
        .run_live(&running)
        .expect("a graph with no external nodes runs");

    let err = runtime
        .restore_node_states(&anchors)
        .expect_err("a runtime that has run a live session must refuse a restore");
    let msg = err.to_string();
    for needle in [
        // WHICH graph.
        "restore",
        // WHAT is wrong — the reuse, named, not merely "invalid state".
        "external sources",
        "REUSES the bindings",
        // and the FIX, which is the only thing an operator can act on.
        "FRESHLY BUILT",
    ] {
        assert!(
            msg.contains(needle),
            "the refusal must carry `{needle}` — an operator reading it has to learn \
             which graph, why, and what to do: {msg}"
        );
    }
    runtime.shutdown();

    // ANTI-TAUTOLOGY: the very same anchors, on a FRESH runtime, restore and
    // take effect. Without this the arm would pass on a `restore_node_states`
    // that refused everything.
    let seen2 = Arc::new(Mutex::new(Vec::new()));
    let phases2: Phases = Arc::new(Mutex::new(Vec::new()));
    let mut fresh = build(
        &prefix_for("c6f"),
        CounterEntry::new(Arc::clone(&phases2)),
        Arc::clone(&seen2),
    )
    .expect("builds");
    let report = fresh
        .restore_node_states(&anchors)
        .expect("a freshly built runtime restores");
    assert_eq!(report.restored, vec!["counter".to_string()]);
    // FOUR steps for THREE delivered values — the sink drains at the top of its
    // own tick, so a step's frame is observed on the next one (see the sibling
    // arm; it is the executor's shape, not the restore's).
    for _ in 0..4 {
        fresh.step(Duration::from_millis(10));
    }
    // The hand oracle: a counter restored at 100 publishes 100, 101, 102 —
    // never 0, 1, 2.
    assert_eq!(
        *seen2.lock().expect("seen"),
        vec![100, 101, 102],
        "the restored count must be the one the anchor carried"
    );
    fresh.shutdown();
}
