// SPDX-License-Identifier: AGPL-3.0-only
//! The CROSS-PROCESS `block` credit word, end to end over real
//! iceoryx2 and a real `MAP_SHARED` credit page.
//!
//! # What is under test
//!
//! A `#[input(backpressure = block)]` edge whose producer and consumer land in
//! DIFFERENT process groups. Before this chunk such an edge could not run at
//! all: `subgraph_for` drops the foreign node, so the producer's worker sees a
//! topic with no `block` consumer (its wiring branch is skipped and it never
//! defers) and the consumer's worker sees one with no producer (`GraphTopology::
//! validate` refuses it outright). The credit word is what makes the edge
//! representable, and the PLAN-CARRIED
//! [`CreditBinding`](cerulion_core::graph::CreditBinding)s are what inject each
//! half into a build that cannot derive it.
//!
//! # The two-context shape (cribbed from `barrier_level_gate_iox2_test`)
//!
//! Two `TransportManager`s — two "graph processes" — over ONE isolated
//! iceoryx2 SHM root, plus ONE credit page created by a third handle standing
//! in for the SUPERVISOR and OPENED separately by each side, exactly as the
//! shipping supervisor/worker split does:
//!
//! ```text
//!   P (producer ctx)  period_ms producer ──▶ /cb/data ──▶ block consumer   C (consumer ctx)
//!            │                                                    │
//!            └──────────── one MappedCredit page ─────────────────┘
//!                     (supervisor creates; each side opens)
//! ```
//!
//! P's graph declares NO consumer and C's declares NO producer, so every
//! assertion below is about wiring that the local topology alone could not have
//! produced.
//!
//! # Every ctor that can carry a binding is driven with a NON-EMPTY one
//!
//! Decision: the credit word is mode-independent. A lockstep deployment
//! splits a `block` edge exactly as a free-run one does. So all three
//! multi-process ctors are exercised here with real bindings — the free-run
//! live ctor throughout, the LOCKSTEP barrier ctor in
//! `a_lockstep_barrier_context_defers_on_the_same_mapped_word`, and the
//! free-run RECORD ctor in
//! `the_record_ctor_threads_a_credit_binding_into_a_real_build`. A ctor that
//! merely COMPILES against the new argument would be inert shipping.
//!
//! # Why the producer's observable is its FIRE COUNT, not a defer counter
//!
//! `backpressure_block_fires_deferred_count` is registered on the CONSUMER's
//! `NodeHandle`, and on a split edge the consumer node does not exist in the
//! producer's process — so there is no handle there to register it on, and the
//! consumer's own copy is never bumped (a known gap; pinned as a gap
//! by `a_split_edge_consumer_reads_a_zero_defer_count_and_the_producer_warns`).
//! What IS observable on the producer's side is exactly what matters: the fire
//! count PLATEAUS at `depth` against a hand oracle, the shared page reads
//! `depth`, and the once-per-regime `warn!` names the consumer edge.
//!
//! Per-test isolated SHM roots + pid/tag-scoped credit namespaces. `#[serial]`
//! on 18 of the 19 arms: one is `#[traced_test]`, which installs a
//! PROCESS-GLOBAL subscriber, and a sibling running beside it would both
//! pollute its capture and race the install. The 19th —
//! `the_depth_max_consumer_tracks_the_topology_ceiling` — is a pure constant
//! drift guard that touches no transport and needs no turn.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::credit::{credit_edge_id, CreditWord, MappedCredit};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::topology::MAX_CONSUMER_DEPTH;
use cerulion_core::graph::{CreditBinding, CreditRole, CrossProcessWiring, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::scheduler::BackpressureCounters;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// The shared topic the split edge runs on — ABSOLUTE, because the consumer's
/// subgraph has no in-graph producer to resolve a relative `node/output`
/// against (exactly the shape `subgraph_for` leaves behind).
const TOPIC: &str = "/cb/data";
/// A SECOND absolute topic, for the arm that credits one producer-less `block`
/// input and leaves its neighbour uncredited.
const OTHER_TOPIC: &str = "/cb/other";
/// The producer's declared period. One fire per driven step, so every fire
/// count below is a hand oracle rather than a race.
const PERIOD_MS: u64 = 10;
/// The default consumer depth used by most arms.
const DEPTH: u64 = 2;

// ===========================================================================
// Node types (test binaries are separate crates — declared inline).
// ===========================================================================

/// The split edge's PRODUCER. Publishes one `Vector3` per fire carrying its own
/// fire ordinal, so the consumer's received sequence is checkable for gaps.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SplitProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl SplitProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// The split edge's `block` CONSUMER, depth 2. `external` + host-driven, so the
/// test decides exactly when it drains: a step without `trigger_external` never
/// ticks, never reads, never decrements.
#[cerulion_node(external)]
#[derive(Default)]
struct SplitBlockConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
    seen: Arc<Mutex<Vec<u64>>>,
}

#[cerulion_node_impl]
impl SplitBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let v = self.inp.x;
        if let Ok(mut seen) = self.seen.lock() {
            seen.push(v as u64);
        }
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A depth-1 twin — the tightest legal ceiling, where a producer may hold
/// exactly one frame in flight.
#[cerulion_node(external)]
#[derive(Default)]
struct DepthOneBlockConsumer {
    #[input(backpressure = block, depth = 1)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl DepthOneBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A `MAX_CONSUMER_DEPTH` twin — the other end of the legal range, where the
/// word's `u32` depth stamp and the topic's iceoryx2 buffer ceiling both have
/// to carry the maximum. The literal is drift-guarded against the constant by
/// `the_depth_max_consumer_tracks_the_topology_ceiling`.
#[cerulion_node(external)]
#[derive(Default)]
struct DepthMaxBlockConsumer {
    #[input(backpressure = block, depth = 64)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl DepthMaxBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A SECOND `block` consumer on the SAME topic at a DIFFERENT depth, so a
/// multi-consumer split can carry two words and the answer names which one did
/// the deferring.
#[cerulion_node(external)]
#[derive(Default)]
struct SecondBlockConsumer {
    #[input(backpressure = block, depth = 3)]
    other: Vector3,
}

#[cerulion_node_impl]
impl SecondBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.other.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A NON-`block` consumer at the same depth — the policy-skew fixture, and the
/// starving sibling that makes a topic MIXED.
#[cerulion_node(external)]
#[derive(Default)]
struct PlainConsumer {
    #[input(depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl PlainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

// ===========================================================================
// Harness
// ===========================================================================

/// One "graph process" over the shared SHM root `ix`, on its own clock.
///
/// `subscriber_buffer_size = 72` because the producer's context has NO in-graph
/// consumer of `/cb/data` and therefore derives that topic's buffer ceiling
/// from this default alone — it has to cover the deepest consumer any OTHER
/// context opens, which here is the `MAX_CONSUMER_DEPTH` (64) arm.
fn manager(
    name: &str,
    ix: iceoryx2::config::Config,
    clock: Arc<VirtualClock>,
) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock,
            subscriber_buffer_size: 72,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

/// A pid+tag-scoped credit namespace, so concurrent test binaries and re-runs
/// never collide on a POSIX SHM object.
fn credit_ns(tag: &str) -> String {
    format!("credit_{}_{tag}", std::process::id())
}

/// One node's `NodeDef`.
fn node(id: &str, inputs: Vec<InputDef>, outputs: Vec<OutputDef>) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: id.to_string(),
        inputs,
        outputs,
    }
}

/// The producer's single output, published under the ABSOLUTE test topic.
fn producer_out() -> OutputDef {
    OutputDef {
        name: "out".to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: Some(TOPIC.to_string()),
    }
}

/// One input reading the absolute test topic.
fn topic_in(name: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: TOPIC.to_string(),
    }
}

fn config_of(prefix: &str, tag: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_{tag}"),
        prefix: prefix.to_string(),
        nodes,
    }
}

/// The PRODUCER context's graph: one `period_ms` node publishing the absolute
/// topic, and nothing else. Deliberately declares NO consumer — that is the
/// whole point (the consumer lives in another process).
fn producer_graph(prefix: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = config_of(
        prefix,
        "producer_ctx",
        vec![node("producer", vec![], vec![producer_out()])],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(SplitProducerEntry::new()));
    (config, factories)
}

/// A graph that touches NEITHER end of the credited topic: one producer of an
/// UNRELATED topic (its output takes the default `/{prefix}/producer/out`
/// name). The isolation fixture for the two "this process owns neither half"
/// refusals — a node-less graph would be refused by `validate_graph` first
/// ("graph must have at least one node") and the arm under test would never
/// run.
fn unrelated_graph(prefix: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = config_of(
        prefix,
        "unrelated_ctx",
        vec![node(
            "producer",
            vec![],
            vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        )],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(SplitProducerEntry::new()));
    (config, factories)
}

/// A CONSUMER context's graph: one node reading the absolute topic, with NO
/// in-graph producer for it.
fn consumer_graph(
    prefix: &str,
    id: &str,
    input: &str,
    entry: Box<dyn NodeEntry>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = config_of(
        prefix,
        "consumer_ctx",
        vec![node(id, vec![topic_in(input)], vec![])],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(id.to_string(), entry);
    (config, factories)
}

/// One credit binding over a MAPPED word.
fn binding(
    word: &Arc<MappedCredit>,
    node_id: &str,
    input: &str,
    depth: u64,
    role: CreditRole,
) -> CreditBinding {
    CreditBinding {
        topic: TOPIC.to_string(),
        consumer_node: node_id.to_string(),
        consumer_input: input.to_string(),
        depth,
        role,
        word: CreditWord::mapped(Arc::clone(word)),
        // Every producer binding carries the rank's
        // edge-local slot. These fixtures mint ONE producer per edge (the
        // one-producer fence), so that slot is 0 — and the build REFUSES a producer binding
        // without one, which is why this is not `None` here.
        producer_slot: role
            .is_producer()
            .then(|| cerulion_core::credit::ProducerSlot::new(0)),
    }
}

/// Create the supervisor's word for `(consumer, input)` and open one peer handle
/// per side — the real three-handle shape (owner + two mappings), not one handle
/// shared by clone.
struct Word {
    /// The OWNER. Held (never `_`) so its `Drop` unlink happens at the end of
    /// the test, not at the end of this constructor.
    _owner: MappedCredit,
    producer_side: Arc<MappedCredit>,
    consumer_side: Arc<MappedCredit>,
}

fn supervisor_word(tag: &str, node_id: &str, input: &str, depth: u32) -> Word {
    let ns = credit_ns(tag);
    let id = credit_edge_id(TOPIC, node_id, input);
    let owner = MappedCredit::create_owned(&ns, &id, depth).expect("supervisor creates the word");
    let producer_side =
        Arc::new(MappedCredit::open_unowned(&ns, &id).expect("producer side opens the word"));
    let consumer_side =
        Arc::new(MappedCredit::open_unowned(&ns, &id).expect("consumer side opens the word"));
    Word {
        _owner: owner,
        producer_side,
        consumer_side,
    }
}

/// Drive ONE step of a FREE-RUN (RealClock-shaped) runtime whose clock this
/// test owns.
///
/// `build_live_free_run` puts the scheduler on `ClockInner::Real`, which READS
/// the clock and never advances it, so the `set` is what moves LOGICAL time and
/// the `step` delta cannot move it. The delta is not inert in general — it is
/// also the WALL delta the liveliness sweep is paced by — so it is passed as
/// one period rather than zero. Setting the clock to `(i + 1) * PERIOD_MS` gives a
/// `period_ms = PERIOD_MS` node EXACTLY one due fire per call — no catch-up
/// burst, so a fire count is a hand oracle.
///
/// The sibling driver for the two CONTROLLED-clock ctors is
/// [`step_quantum`]: there the clock is advanced BY `step` itself, and calling
/// `set` as well would advance it twice.
fn step_free_run(rt: &mut GraphRuntime, clock: &VirtualClock, step_index: u64) {
    clock.set((step_index + 1) * PERIOD_MS * 1_000_000);
    rt.step(Duration::from_millis(PERIOD_MS));
}

/// Drive ONE step of a CONTROLLED-clock runtime (the barrier ctor and the
/// free-run RECORD ctor): `step` advances the gating `VirtualClock` by the
/// delta, so nothing else may touch it. One period per call, same oracle.
fn step_quantum(rt: &mut GraphRuntime) {
    rt.step(Duration::from_millis(PERIOD_MS));
}

/// Build the standard PRODUCER context on the free-run live ctor with one
/// Producer-role binding.
fn producer_ctx(
    prefix: &str,
    ix: &iceoryx2::config::Config,
    mgr_name: &str,
    clock: &Arc<VirtualClock>,
    bindings: &[CreditBinding],
) -> (Arc<TransportManager>, GraphRuntime) {
    let mgr = manager(mgr_name, ix.clone(), Arc::clone(clock));
    let (cfg, fac) = producer_graph(prefix);
    let rt = GraphRuntime::build_live_free_run(
        cfg,
        fac,
        &mgr,
        Arc::clone(clock) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, bindings),
    )
    .expect("build the producer context with its credit binding");
    (mgr, rt)
}

/// The credit record for `(node, input)` on a runtime, as
/// `(outstanding, depth, mapped)`.
fn credit_of(rt: &GraphRuntime, node_id: &str, input: &str) -> (u64, u64, bool) {
    let all: Vec<_> = rt.block_credits().collect();
    let hit = all
        .iter()
        .find(|c| c.consumer_node == node_id && c.consumer_input == input)
        .unwrap_or_else(|| panic!("no block credit record for {node_id}.{input}: {all:?}"));
    (hit.outstanding, hit.depth, hit.mapped)
}

/// `Err`-or-panic helper: `GraphRuntime` is `!Debug`, so `expect_err` cannot be
/// used on a build result.
fn build_err(
    result: cerulion_core::TransportResult<GraphRuntime>,
    why: &str,
) -> cerulion_core::TransportError {
    match result {
        Ok(_) => panic!("{why}"),
        Err(e) => e,
    }
}

/// How long a drainer thread waits before freeing credit,
/// so its drain lands while the producer is genuinely PARKED rather than being
/// absorbed into the park-entry baseline.
///
/// Generous on purpose. It is not a discriminator — the oracle is the
/// attribution counter, and a drain that landed too early simply produces a
/// timeout exit, which the arm reports as a failure of the WAKE rather than
/// passing quietly.
const PARK_ENTRY_SETTLE_MS: u64 = 25;

/// The park window each wake arm gives the live step. Long enough that the
/// recheck timeout cannot beat a drain scheduled at `PARK_ENTRY_SETTLE_MS`,
/// short enough that a wedged arm fails in seconds rather than hanging CI.
const PARK_WINDOW_MS: u64 = 400;

/// Drains the peer performs during the no-false-wake control — enough credit
/// motion that a predicate keyed on the epoch alone would certainly fire.
const PEER_CHURN_DRAINS: usize = 20;

/// The park FORCED on, with no doorbell.
///
/// The arms that assert on park attribution are about the park's own
/// predicates, so they must not be at the mercy of what the CLI resolver
/// decides for the machine the suite happens to run on. On a target with no CPU
/// monitor-wait primitive the park degrades to a sleep-recheck — which still
/// polls every predicate every recheck, so the wake attribution these arms
/// assert on holds on every tier.
fn forced_park_policy() -> cerulion_core::MonitorWaitPolicy {
    cerulion_core::MonitorWaitPolicy::new(true, false, "cbpark".to_string())
}

/// Every needle appears in the refusal.
fn assert_names(text: &str, needles: &[&str], what: &str) {
    for n in needles {
        assert!(
            text.contains(n),
            "the {what} refusal must name {n:?}: {text}"
        );
    }
}

// ===========================================================================
// HAPPY — the headline
// ===========================================================================

/// The whole contract in one body: a producer in ONE process defers at the
/// depth of a `block` consumer in ANOTHER, the consumer's drain frees credit,
/// and BOTH sides' `block_credits()` track the SHARED page before AND after.
///
/// Every number is a hand oracle: with `depth = 2` and a consumer that never
/// drains, the pre-fire sees 0 → fires, sees 1 → fires, sees 2 → defers
/// forever. So `STEPS` steps yield EXACTLY 2 fires whatever `STEPS` is.
#[test]
#[serial]
fn a_producer_defers_at_a_foreign_consumers_depth_and_resumes_when_it_drains() {
    const STEPS: u64 = 12;
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("happy", "consumer", "inp", DEPTH as u32);

    let clock_p = Arc::new(VirtualClock::new());
    let bind_p = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let (_mgr_p, mut rt_p) = producer_ctx("cbh", &ix, "cb_happy_p", &clock_p, &bind_p);

    let seen = Arc::new(Mutex::new(Vec::new()));
    let clock_c = Arc::new(VirtualClock::new());
    let mgr_c = manager("cb_happy_c", ix, Arc::clone(&clock_c));
    let (cfg_c, fac_c) = consumer_graph(
        "cbh",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::with_state(SplitBlockConsumer {
            seen: Arc::clone(&seen),
            ..Default::default()
        })),
    );
    let bind_c = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let mut rt_c = GraphRuntime::build_live_free_run(
        cfg_c,
        fac_c,
        &mgr_c,
        Arc::clone(&clock_c) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_c),
    )
    .expect(
        "build the consumer context with its credit binding — the credit token is what \
         exempts a producer-less `block` topic from GraphTopology::validate",
    );

    // PHASE 1 — the consumer never drains: the producer must plateau at DEPTH.
    for i in 0..STEPS {
        step_free_run(&mut rt_p, &clock_p, i);
    }
    assert_eq!(
        rt_p.node_handle("producer")
            .expect("the producer's handle")
            .fire_count(),
        DEPTH,
        "the producer must plateau at the FOREIGN consumer's declared depth ({DEPTH}) over \
         {STEPS} steps — a producer that never saw the credit word would have fired {STEPS} \
         times"
    );
    assert_eq!(
        w.producer_side.outstanding(),
        DEPTH,
        "the shared page must hold exactly the frames the producer published"
    );
    // Both ranks report the SHARED page, and both mark it MAPPED so a reader
    // aggregating across ranks knows the two rows are one word.
    assert_eq!(
        credit_of(&rt_p, "consumer", "inp"),
        (DEPTH, DEPTH, true),
        "the producer's credit record must read the shared page"
    );
    assert_eq!(
        credit_of(&rt_c, "consumer", "inp"),
        (DEPTH, DEPTH, true),
        "and so must the consumer's — one page, two readers, one answer"
    );

    // PHASE 2 — the consumer drains once. Credit is freed on the SHARED page,
    // so the producer (in the other process) may fire again.
    rt_c.trigger_external("consumer").expect("trigger");
    clock_c.set(PERIOD_MS * 1_000_000);
    rt_c.step(Duration::from_millis(PERIOD_MS));
    assert_eq!(
        w.consumer_side.outstanding(),
        0,
        "a drain-to-latest pops every queued frame, so both are returned to the word"
    );
    // The occupancy READ BACK on both runtimes, not just the raw word: a
    // `block_credits()` that reported `depth` where `outstanding` belongs
    // (or a frozen snapshot) is indistinguishable at the full-queue instant
    // and obvious here.
    assert_eq!(
        credit_of(&rt_p, "consumer", "inp"),
        (0, DEPTH, true),
        "after the drain the producer's record must read 0 outstanding against depth {DEPTH}"
    );
    assert_eq!(
        credit_of(&rt_c, "consumer", "inp"),
        (0, DEPTH, true),
        "and so must the consumer's"
    );

    for i in STEPS..(STEPS + 2) {
        step_free_run(&mut rt_p, &clock_p, i);
    }
    assert_eq!(
        rt_p.node_handle("producer")
            .expect("the producer's handle")
            .fire_count(),
        DEPTH + 2,
        "with the queue drained the producer must fire again — the decrement crossed the \
         process boundary"
    );
    assert_eq!(
        seen.lock().expect("seen").clone(),
        vec![2],
        "the drain serves the latest of the two queued frames (hand oracle: the producer \
         published 1 then 2)"
    );

    rt_p.shutdown();
    rt_c.shutdown();
}

/// NO DATA LOSS, as a real oracle: every frame the producer committed reaches
/// the foreign consumer, in order, with no gap.
///
/// Asserting `backpressure_drop_oldest_count == 0` would be
/// VACUOUS — a `block`-probed input installs no eviction detector at
/// all, so that counter is structurally zero whatever happens to the frames.
/// The real oracle is the consumer's RECEIVED SEQUENCE against the producer's
/// fire ordinal, and getting it needs per-frame lockstep: the macro's input read
/// is drain-to-latest, so two frames in the queue would serve only the newest
/// and a gap would be indistinguishable from a batch.
#[test]
#[serial]
fn every_frame_the_producer_committed_reaches_the_foreign_consumer() {
    const FRAMES: u64 = 8;
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("lossless", "consumer", "inp", DEPTH as u32);

    let clock_p = Arc::new(VirtualClock::new());
    let bind_p = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let (_mgr_p, mut rt_p) = producer_ctx("cbl", &ix, "cb_loss_p", &clock_p, &bind_p);

    let seen = Arc::new(Mutex::new(Vec::new()));
    let clock_c = Arc::new(VirtualClock::new());
    let mgr_c = manager("cb_loss_c", ix, Arc::clone(&clock_c));
    let (cfg_c, fac_c) = consumer_graph(
        "cbl",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::with_state(SplitBlockConsumer {
            seen: Arc::clone(&seen),
            ..Default::default()
        })),
    );
    let bind_c = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let mut rt_c = GraphRuntime::build_live_free_run(
        cfg_c,
        fac_c,
        &mgr_c,
        Arc::clone(&clock_c) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_c),
    )
    .expect("consumer ctx");

    for i in 0..FRAMES {
        step_free_run(&mut rt_p, &clock_p, i);
        rt_c.trigger_external("consumer").expect("trigger");
        clock_c.set((i + 1) * PERIOD_MS * 1_000_000);
        rt_c.step(Duration::from_millis(PERIOD_MS));
    }

    let oracle: Vec<u64> = (1..=FRAMES).collect();
    assert_eq!(
        seen.lock().expect("seen").clone(),
        oracle,
        "every committed frame must reach the consumer, in order and with no gap — a `block` \
         edge is lossless BY DECLARATION, and the credit word is what makes that true across \
         a process boundary"
    );
    assert_eq!(
        rt_p.node_handle("producer").expect("h").fire_count(),
        FRAMES,
        "the producer was never blocked for good: one drain per publish keeps it running"
    );
    assert_eq!(
        w.producer_side.outstanding(),
        0,
        "and the word returns to empty — every publish was matched by a drain"
    );
    rt_p.shutdown();
    rt_c.shutdown();
}

/// Two runs of the identical stimulus produce identical counts (Principle #7).
/// The credit word is a WHEN gate on tick entry; it changes nothing about WHAT
/// fires.
#[test]
#[serial]
fn the_split_block_edge_is_deterministic_across_runs() {
    fn once(tag: &str) -> (u64, u64) {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let w = supervisor_word(tag, "consumer", "inp", DEPTH as u32);
        let clock_p = Arc::new(VirtualClock::new());
        let bind_p = [binding(
            &w.producer_side,
            "consumer",
            "inp",
            DEPTH,
            CreditRole::Producer,
        )];
        let (_mgr_p, mut rt_p) = producer_ctx("cbd", &ix, "cb_det_p", &clock_p, &bind_p);
        let clock_c = Arc::new(VirtualClock::new());
        let mgr_c = manager("cb_det_c", ix, Arc::clone(&clock_c));
        let (cfg_c, fac_c) = consumer_graph(
            "cbd",
            "consumer",
            "inp",
            Box::new(SplitBlockConsumerEntry::new()),
        );
        let bind_c = [binding(
            &w.consumer_side,
            "consumer",
            "inp",
            DEPTH,
            CreditRole::Consumer,
        )];
        let mut rt_c = GraphRuntime::build_live_free_run(
            cfg_c,
            fac_c,
            &mgr_c,
            Arc::clone(&clock_c) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind_c),
        )
        .expect("consumer ctx");
        for i in 0..6 {
            step_free_run(&mut rt_p, &clock_p, i);
        }
        rt_c.trigger_external("consumer").expect("trigger");
        clock_c.set(PERIOD_MS * 1_000_000);
        rt_c.step(Duration::from_millis(PERIOD_MS));
        for i in 6..10 {
            step_free_run(&mut rt_p, &clock_p, i);
        }
        let fires = rt_p.node_handle("producer").expect("h").fire_count();
        let outstanding = w.producer_side.outstanding();
        rt_p.shutdown();
        rt_c.shutdown();
        (fires, outstanding)
    }
    let a = once("det_a");
    let b = once("det_b");
    assert_eq!(
        a, b,
        "two runs of one stimulus must be bit-identical (Principle #7)"
    );
    // Anti-tautology: the pair is a hand oracle, not merely equal to itself.
    // 2 fires fill the depth-2 queue; the drain frees both; 2 more fires refill
    // it, so the run ends at 4 fires with the word full again.
    assert_eq!(
        a,
        (4, 2),
        "hand oracle: fill (2) → drain → refill (2) leaves 4 fires and a full word"
    );
}

// ===========================================================================
// NO INERT CTOR — the lockstep and record build paths
// ===========================================================================

/// The LOCKSTEP (barrier) ctor carries a credit binding into a real build and
/// the producer defers on the shared word.
///
/// The credit word is mode-independent. The
/// word is what makes a split `block` edge representable, not how the ranks
/// coordinate their steps. So the barrier ctor's new argument must be driven
/// with a NON-EMPTY list, or the mode-independence claim is compiled-but-unrun.
///
/// The barrier is `expected = 1` — a real `MappedBarrier` this context really
/// participates in, whose every arrive opens immediately, which is how
/// `free_run_ctor_iox2_test` drives its own lockstep arm. What is under test is
/// the CREDIT threading, not the rendezvous (that is
/// `barrier_level_gate_iox2_test`'s).
#[test]
#[serial]
fn a_lockstep_barrier_context_defers_on_the_same_mapped_word() {
    const STEPS: usize = 10;
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("lockstep", "consumer", "inp", DEPTH as u32);

    // The producer rank, on the LOCKSTEP ctor.
    let clock_p = Arc::new(VirtualClock::new());
    let mgr_p = manager("cb_ls_p", ix.clone(), Arc::clone(&clock_p));
    let (cfg_p, fac_p) = producer_graph("cbls");
    // The barrier namespace is derived through `credit_ns` (which folds in
    // `std::process::id()`) rather than from `w.ns`: under nextest every test
    // is its own PROCESS, and `/cer_bar_{fnv(ns/id)}` is a pure function of the
    // namespace with no pid in it, so two tests sharing a namespace map the
    // same `/dev/shm` page however isolated their iceoryx2 roots are. Building
    // it from the struct field would be pid-scoped in FACT but unresolvable to
    // `serial_discipline_test`'s walk, which cannot follow a field — and a rule
    // a reader (or a gate) cannot check is not a rule.
    let barrier = Arc::new(
        MappedBarrier::create_owned(&credit_ns("lockstep_bar"), "levelgate", 1)
            .expect("barrier owner"),
    );
    let bind_p = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let mut rt_p = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        cfg_p,
        fac_p,
        &mgr_p,
        Arc::clone(&clock_p),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        Arc::clone(&barrier),
        vec![Some(0)],
        vec![false; 1],
        0,
        Duration::from_millis(PERIOD_MS),
        CrossProcessWiring::with_credit(None, &bind_p),
    )
    .expect("build the LOCKSTEP producer rank with its credit binding");

    // The consumer rank stays on the free-run ctor: a deployment's ranks share
    // a mode in production, but what this arm pins is that the lockstep BUILD
    // wired the producer half — the peer is just the other end of the word.
    let clock_c = Arc::new(VirtualClock::new());
    let mgr_c = manager("cb_ls_c", ix, Arc::clone(&clock_c));
    let (cfg_c, fac_c) = consumer_graph(
        "cbls",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind_c = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let rt_c = GraphRuntime::build_live_free_run(
        cfg_c,
        fac_c,
        &mgr_c,
        Arc::clone(&clock_c) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_c),
    )
    .expect("consumer ctx");

    for _ in 0..STEPS {
        step_quantum(&mut rt_p);
    }
    assert_eq!(
        rt_p.node_handle("producer").expect("h").fire_count(),
        DEPTH,
        "a LOCKSTEP rank defers at the foreign consumer's depth exactly as a free-run rank \
         does — the credit word is mode-independent"
    );
    assert_eq!(w.producer_side.outstanding(), DEPTH);
    assert_eq!(
        credit_of(&rt_p, "consumer", "inp"),
        (DEPTH, DEPTH, true),
        "and the lockstep rank reports the mapped page"
    );
    assert_eq!(
        credit_of(&rt_c, "consumer", "inp"),
        (DEPTH, DEPTH, true),
        "the two ranks agree because there is one page"
    );
    rt_p.shutdown();
    rt_c.shutdown();
}

/// The free-run RECORD ctor carries a credit binding into a real build too.
///
/// `graph run --record` on a free-run deployment routes every rank through
/// `build_live_deterministic_free_run`, so a binding that reached only the
/// non-record ctor would leave recording deployments silently un-deferred.
#[test]
#[serial]
fn the_record_ctor_threads_a_credit_binding_into_a_real_build() {
    const STEPS: usize = 10;
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("record", "consumer", "inp", DEPTH as u32);

    let clock_p = Arc::new(VirtualClock::new());
    let mgr_p = manager("cb_rec_p", ix.clone(), Arc::clone(&clock_p));
    let (cfg_p, fac_p) = producer_graph("cbrec");
    let bind_p = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let mut rt_p = GraphRuntime::build_live_deterministic_free_run(
        cfg_p,
        fac_p,
        &mgr_p,
        Arc::clone(&clock_p),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::with_credit(None, &bind_p),
    )
    .expect("build the free-run RECORD rank with its credit binding");

    let clock_c = Arc::new(VirtualClock::new());
    let mgr_c = manager("cb_rec_c", ix, Arc::clone(&clock_c));
    let (cfg_c, fac_c) = consumer_graph(
        "cbrec",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind_c = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let rt_c = GraphRuntime::build_live_free_run(
        cfg_c,
        fac_c,
        &mgr_c,
        Arc::clone(&clock_c) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_c),
    )
    .expect("consumer ctx");

    for _ in 0..STEPS {
        step_quantum(&mut rt_p);
    }
    assert_eq!(
        rt_p.node_handle("producer").expect("h").fire_count(),
        DEPTH,
        "a RECORDING free-run rank defers at the foreign consumer's depth"
    );
    assert_eq!(
        credit_of(&rt_p, "consumer", "inp"),
        (DEPTH, DEPTH, true),
        "the recording rank reports the mapped page"
    );
    rt_p.shutdown();
    rt_c.shutdown();
}

// ===========================================================================
// EDGE — the depth range, a multi-consumer split, the `Both` role
// ===========================================================================

/// `depth == 1` (the tightest legal ceiling) and `depth == MAX_CONSUMER_DEPTH`
/// (the widest) both gate exactly at their own number — on the producer's FIRE
/// COUNT and on the shared WORD, because a wiring that dropped the consumer
/// half would leave the fire count right and the word empty.
#[test]
#[serial]
fn the_gate_plateaus_at_both_ends_of_the_legal_depth_range() {
    fn plateau(tag: &str, entry: Box<dyn NodeEntry>, depth: u64, steps: u64) -> (u64, u64) {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let w = supervisor_word(tag, "consumer", "inp", depth as u32);
        let clock_p = Arc::new(VirtualClock::new());
        let bind_p = [binding(
            &w.producer_side,
            "consumer",
            "inp",
            depth,
            CreditRole::Producer,
        )];
        let (_mgr_p, mut rt_p) = producer_ctx("cbr", &ix, "cb_range_p", &clock_p, &bind_p);
        let clock_c = Arc::new(VirtualClock::new());
        let mgr_c = manager("cb_range_c", ix, Arc::clone(&clock_c));
        let (cfg_c, fac_c) = consumer_graph("cbr", "consumer", "inp", entry);
        let bind_c = [binding(
            &w.consumer_side,
            "consumer",
            "inp",
            depth,
            CreditRole::Consumer,
        )];
        let rt_c = GraphRuntime::build_live_free_run(
            cfg_c,
            fac_c,
            &mgr_c,
            Arc::clone(&clock_c) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind_c),
        )
        .expect("consumer ctx");
        for i in 0..steps {
            step_free_run(&mut rt_p, &clock_p, i);
        }
        let out = (
            rt_p.node_handle("producer").expect("h").fire_count(),
            w.producer_side.outstanding(),
        );
        rt_p.shutdown();
        rt_c.shutdown();
        out
    }
    assert_eq!(
        plateau("d1", Box::new(DepthOneBlockConsumerEntry::new()), 1, 8),
        (1, 1),
        "a depth-1 edge lets exactly ONE frame be in flight, and the word holds it"
    );
    assert_eq!(
        plateau(
            "dmax",
            Box::new(DepthMaxBlockConsumerEntry::new()),
            MAX_CONSUMER_DEPTH as u64,
            (MAX_CONSUMER_DEPTH as u64) + 16
        ),
        (MAX_CONSUMER_DEPTH as u64, MAX_CONSUMER_DEPTH as u64),
        "a MAX_CONSUMER_DEPTH edge plateaus at the maximum, not at some clamped intermediate"
    );
}

/// DRIFT GUARD: `DepthMaxBlockConsumer`'s `#[input(depth = 64)]` is the
/// topology's own ceiling. The attribute cannot be a `const` expression, so the
/// literal is pinned here — a raised `MAX_CONSUMER_DEPTH` would otherwise leave
/// the "widest legal depth" arm quietly testing a middling one.
#[test]
fn the_depth_max_consumer_tracks_the_topology_ceiling() {
    assert_eq!(
        MAX_CONSUMER_DEPTH, 64,
        "DepthMaxBlockConsumer declares `depth = 64` as the widest LEGAL depth; if \
         MAX_CONSUMER_DEPTH moved, update the fixture's attribute and this pin together"
    );
}

/// A multi-consumer `block` topic split across two groups carries TWO words, the
/// single producer defers as soon as EITHER fills, and draining the SHALLOWER
/// one moves the plateau to the DEEPER one's ceiling.
///
/// The second half is what makes this an oracle rather than a coincidence: an
/// implementation that consulted only the FIRST binding would pass the first
/// assertion and fail the second.
#[test]
#[serial]
fn a_multi_consumer_split_carries_two_words_and_defers_on_whichever_is_full() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let ns = credit_ns("multi");
    let id_a = credit_edge_id(TOPIC, "consumer", "inp");
    let id_b = credit_edge_id(TOPIC, "consumer2", "other");
    let _own_a = MappedCredit::create_owned(&ns, &id_a, 2).expect("word a");
    let _own_b = MappedCredit::create_owned(&ns, &id_b, 3).expect("word b");
    let word_a = Arc::new(MappedCredit::open_unowned(&ns, &id_a).expect("open a"));
    let word_b = Arc::new(MappedCredit::open_unowned(&ns, &id_b).expect("open b"));

    let clock_p = Arc::new(VirtualClock::new());
    let bind_p = [
        binding(&word_a, "consumer", "inp", 2, CreditRole::Producer),
        binding(&word_b, "consumer2", "other", 3, CreditRole::Producer),
    ];
    let (_mgr_p, mut rt_p) = producer_ctx("cbm", &ix, "cb_multi_p", &clock_p, &bind_p);

    // Consumer A (depth 2) in its own context.
    let clock_a = Arc::new(VirtualClock::new());
    let mgr_a = manager("cb_multi_ca", ix.clone(), Arc::clone(&clock_a));
    let (cfg_a, fac_a) = consumer_graph(
        "cbm",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let word_a_c = Arc::new(MappedCredit::open_unowned(&ns, &id_a).expect("open a (consumer)"));
    let bind_a = [binding(
        &word_a_c,
        "consumer",
        "inp",
        2,
        CreditRole::Consumer,
    )];
    let mut rt_a = GraphRuntime::build_live_free_run(
        cfg_a,
        fac_a,
        &mgr_a,
        Arc::clone(&clock_a) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_a),
    )
    .expect("consumer A ctx");

    // Consumer B (depth 3) in a THIRD context — a genuinely different group.
    let clock_b = Arc::new(VirtualClock::new());
    let mgr_b = manager("cb_multi_cb", ix, Arc::clone(&clock_b));
    let cfg_b = config_of(
        "cbm",
        "consumer2_ctx",
        vec![node("consumer2", vec![topic_in("other")], vec![])],
    );
    let mut fac_b: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fac_b.insert(
        "consumer2".to_string(),
        Box::new(SecondBlockConsumerEntry::new()) as Box<dyn NodeEntry>,
    );
    let word_b_c = Arc::new(MappedCredit::open_unowned(&ns, &id_b).expect("open b (consumer)"));
    let bind_b = [binding(
        &word_b_c,
        "consumer2",
        "other",
        3,
        CreditRole::Consumer,
    )];
    let rt_b = GraphRuntime::build_live_free_run(
        cfg_b,
        fac_b,
        &mgr_b,
        Arc::clone(&clock_b) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_b),
    )
    .expect("consumer B ctx");

    for i in 0..10 {
        step_free_run(&mut rt_p, &clock_p, i);
    }
    assert_eq!(
        rt_p.node_handle("producer").expect("h").fire_count(),
        2,
        "the producer must stop at the SHALLOWER of its two credit words (2, not 3)"
    );
    assert_eq!(
        (word_a.outstanding(), word_b.outstanding()),
        (2, 2),
        "both words track the same publishes; only the shallower one is AT its ceiling"
    );
    let credits: Vec<_> = rt_p.block_credits().collect();
    assert_eq!(
        credits.len(),
        2,
        "one credit record per BOUND EDGE, so a multi-consumer split reports both: {credits:?}"
    );

    // Drain A alone. Its word empties, B's does not — so the producer resumes
    // and must now stop at B's DEEPER ceiling.
    rt_a.trigger_external("consumer").expect("trigger A");
    clock_a.set(PERIOD_MS * 1_000_000);
    rt_a.step(Duration::from_millis(PERIOD_MS));
    assert_eq!(
        (word_a.outstanding(), word_b.outstanding()),
        (0, 2),
        "draining A frees A's word only — B never ticked"
    );
    for i in 10..20 {
        step_free_run(&mut rt_p, &clock_p, i);
    }
    assert_eq!(
        rt_p.node_handle("producer").expect("h").fire_count(),
        3,
        "with A drained the producer runs until B's DEEPER word fills: 2 fires + 1 more = 3 \
         (B held 2 of its 3 already). An implementation that consulted only the first binding \
         would have run on forever here"
    );
    assert_eq!(
        (word_a.outstanding(), word_b.outstanding()),
        (1, 3),
        "and B is the one at its ceiling now"
    );
    rt_p.shutdown();
    rt_a.shutdown();
    rt_b.shutdown();
}

/// The `Both` role: ONE process owning both ends of a credited edge wires both
/// halves from the SAME word, and reports exactly ONE credit record.
///
/// This is also what makes the wiring loop's bound-edge SKIP reachable: without
/// it the local loop would wire this all-block topic AND the injection would
/// wire it again, installing two probes on one input.
#[test]
#[serial]
fn a_both_role_binding_wires_one_word_for_a_co_located_edge() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("both", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_both", ix, Arc::clone(&clock));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let cfg = config_of(
        "cbb",
        "both_ctx",
        vec![
            node("producer", vec![], vec![producer_out()]),
            node("consumer", vec![topic_in("inp")], vec![]),
        ],
    );
    let mut fac: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fac.insert("producer".to_string(), Box::new(SplitProducerEntry::new()));
    fac.insert(
        "consumer".to_string(),
        Box::new(SplitBlockConsumerEntry::with_state(SplitBlockConsumer {
            seen: Arc::clone(&seen),
            ..Default::default()
        })),
    );
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Both,
    )];
    let mut rt = GraphRuntime::build_live_free_run(
        cfg,
        fac,
        &mgr,
        Arc::clone(&clock) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .expect("a Both-role binding wires one context's own edge");

    for i in 0..8 {
        step_free_run(&mut rt, &clock, i);
    }
    assert_eq!(
        rt.node_handle("producer").expect("h").fire_count(),
        DEPTH,
        "the producer defers at its co-located consumer's depth, through the bound word"
    );
    assert_eq!(
        w.consumer_side.outstanding(),
        DEPTH,
        "the SHARED page is what both halves moved — not a private local mirror"
    );
    let credits: Vec<_> = rt.block_credits().collect();
    assert_eq!(
        credits.len(),
        1,
        "exactly ONE credit record: the local loop SKIPPED the bound edge, so the injection \
         is its sole wirer: {credits:?}"
    );
    rt.trigger_external("consumer").expect("trigger");
    step_free_run(&mut rt, &clock, 8);
    assert_eq!(
        w.consumer_side.outstanding(),
        0,
        "and the local drain frees the same word"
    );
    rt.shutdown();
}

// ===========================================================================
// EDGE — the held-head slot debt, over a MAPPED word
// ===========================================================================

/// The slot-debt re-derivation moves the MAPPED page in BOTH
/// directions, and the RELEASE direction rings the wake word.
///
/// The credit-back direction (`held > debt`) goes through `record_published_n`
/// and the release direction (`held < debt`) through `record_drained`, which is
/// what makes the release ring `note_credit_freed`. Ringing is not decoration
/// on a split edge: a producer parked in another process on this exact word is
/// woken by that bump, and a raw decrement would leave it asleep on room that
/// already exists.
///
/// Every occupancy is read back through an INDEPENDENT mapping of the same
/// page, so each assertion is about the shared OBJECT rather than about the
/// handle under test. Cribs `sync_per_set_backpressure_iox2_test`'s slot walk.
#[test]
#[serial]
fn the_held_head_slot_debt_moves_the_mapped_page_both_ways_and_the_release_rings() {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "cb_slotdebt".into(),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let ns = credit_ns("slotdebt");
    let id = credit_edge_id(TOPIC, "consumer", "inp");
    let _owner = MappedCredit::create_owned(&ns, &id, 4).expect("word");
    let probe_word = Arc::new(MappedCredit::open_unowned(&ns, &id).expect("probe mapping"));
    // The INDEPENDENT reader — a second mapping of the same physical page,
    // standing in for the producer's process.
    let peer = MappedCredit::open_unowned(&ns, &id).expect("peer mapping");

    let topic = "slot_debt_mapped";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    let mut sub = mgr.create_subscriber(topic).expect("subscriber");
    sub.register_block_probe_for_test(
        CreditWord::mapped(Arc::clone(&probe_word)),
        4,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        8,
    );
    sub.mark_fifo_consume_for_test();

    // Three published frames — the producer's side of the mirror, through the
    // SAME word a real cross-process publisher would bump.
    for x in 1..=3u32 {
        let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
        p.x = f64::from(x);
        drop(p);
        probe_word.record_published();
    }
    assert_eq!(
        (peer.outstanding(), sub.block_slot_debt_for_test()),
        (3, 0),
        "three frames queued, none held — read through an independent mapping"
    );

    // CREDIT-BACK. The boundary fill pops a frame (a pop-time decrement) and
    // the re-derivation credits it back, because a frame the matcher HOLDS is
    // still unserved. Both halves have to land on the SHARED page or a
    // cross-process producer buys itself headroom the declared depth never
    // gave it.
    let seq_before_credit = peer.wake_seq_snapshot();
    assert_eq!(
        sub.snapshot_latest_for_trigger().0,
        1,
        "the boundary drain pops exactly one frame"
    );
    assert_eq!(
        (peer.outstanding(), sub.block_slot_debt_for_test()),
        (3, 1),
        "occupancy is UNCHANGED at 3 — the credit-back reached the mapped page"
    );
    assert_eq!(
        peer.wake_seq_snapshot(),
        seq_before_credit + 1,
        "EXACTLY one ring across this transition, and it is the POP's: the boundary drain \
         decrements (which rings) and the re-derivation credits the frame back through \
         `record_published_n` (which must NOT ring — nothing waits for the count to RISE). A \
         `>=` here would be unfalsifiable on a monotonic counter; a second ring would mean the \
         credit-back went through the drain path"
    );

    // A second frame leaves the queue for the staged slot — the deepest the
    // matcher ever holds.
    assert!(
        sub.sync_peek_next_stamp_for_test()
            .expect("the peek pops without error")
            .is_some(),
        "a queued frame is there to stage"
    );
    assert_eq!(
        (peer.outstanding(), sub.block_slot_debt_for_test()),
        (3, 2),
        "both slots occupied, occupancy still 3 on the shared page"
    );

    // RELEASE. Skipping the head really removes a frame from the world, so the
    // debt falls and the word is DEBITED — and that direction must RING.
    let seq_before_release = peer.wake_seq_snapshot();
    assert!(
        sub.sync_discard_head_for_test()
            .expect("the advance never errors")
            .is_some(),
        "the staged frame becomes the new head"
    );
    assert_eq!(
        (peer.outstanding(), sub.block_slot_debt_for_test()),
        (2, 1),
        "exactly ONE frame left the world — the skipped head"
    );
    assert!(
        peer.wake_seq_snapshot() > seq_before_release,
        "the RELEASE direction must ring the wake word ({} -> {}) — a raw decrement leaves a \
         producer parked in another process asleep on room that already exists",
        seq_before_release,
        peer.wake_seq_snapshot()
    );

    // ANTI-TAUTOLOGY: the mapped word really can reach 0, so the numbers above
    // are not an artefact of a counter that never moves.
    assert!(
        sub.try_view::<Vector3, f64>(|v| v.x)
            .expect("the frozen head serves")
            .is_some(),
        "the promoted head serves"
    );
    assert_eq!(
        sub.snapshot_latest_for_trigger().0,
        1,
        "the last queued frame fills the head"
    );
    sub.sync_void_head_for_test();
    assert_eq!(
        (peer.outstanding(), sub.block_slot_debt_for_test()),
        (0, 0),
        "the mapped word drains to zero and stays there"
    );
}

/// THE FIREWALL: the credit word changes WHEN a producer fires, never WHAT
/// fires or with what payload (Principle #7, the claim `credit.rs`'s module
/// docs make).
///
/// Two runs of the SAME producer graph, one CREDITED against a consumer that
/// drains every step (so the gate is consulted on every fire and never bites)
/// and one with `CrossProcessWiring::none()` and no consumer at all. Their fire
/// sequences and published values must both equal ONE hand oracle. Every other
/// arm in this file measures the gate BITING; this is the only one that pins
/// what it does when it does not, which is the half the firewall is about.
#[test]
#[serial]
fn a_credited_edge_changes_when_not_what() {
    const STEPS: u64 = 6;

    // Leg 1 — CREDITED, with a consumer draining after every producer step.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("firewall", "consumer", "inp", DEPTH as u32);
    let clock_p = Arc::new(VirtualClock::new());
    let bind_p = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let (_mgr_p, mut rt_p) = producer_ctx("cbf", &ix, "cb_fw_p", &clock_p, &bind_p);
    let clock_c = Arc::new(VirtualClock::new());
    let mgr_c = manager("cb_fw_c", ix, Arc::clone(&clock_c));
    let (cfg_c, fac_c) = consumer_graph(
        "cbf",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind_c = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let mut rt_c = GraphRuntime::build_live_free_run(
        cfg_c,
        fac_c,
        &mgr_c,
        Arc::clone(&clock_c) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_c),
    )
    .expect("consumer ctx");
    for i in 0..STEPS {
        step_free_run(&mut rt_p, &clock_p, i);
        rt_c.trigger_external("consumer").expect("trigger");
        clock_c.set((i + 1) * PERIOD_MS * 1_000_000);
        rt_c.step(Duration::from_millis(PERIOD_MS));
    }
    let credited_fires = rt_p.node_handle("producer").expect("h").fire_count();
    rt_p.shutdown();
    rt_c.shutdown();

    // Leg 2 — UNCREDITED: the same producer graph, no binding, no consumer.
    let ix2 = cerulion_core::testing::iceoryx_test_config();
    let clock_u = Arc::new(VirtualClock::new());
    let mgr_u = manager("cb_fw_u", ix2, Arc::clone(&clock_u));
    let (cfg_u, fac_u) = producer_graph("cbfu");
    let mut rt_u = GraphRuntime::build_live_free_run(
        cfg_u,
        fac_u,
        &mgr_u,
        Arc::clone(&clock_u) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::none(),
    )
    .expect("uncredited producer ctx");
    for i in 0..STEPS {
        step_free_run(&mut rt_u, &clock_u, i);
    }
    let uncredited_fires = rt_u.node_handle("producer").expect("h").fire_count();
    rt_u.shutdown();

    // ONE hand oracle for both: a `period_ms` node stepped once per period fires
    // once per step, credited or not.
    assert_eq!(
        (credited_fires, uncredited_fires),
        (STEPS, STEPS),
        "a credit word whose gate never bites must leave the fire schedule EXACTLY as it found \
         it — the gate is a WHEN, and this is the run where the when never arrives"
    );
}

/// The RECORD ctor's determinism leg: its whole purpose is a byte-exact bag, so
/// two runs of one stimulus through it must agree.
#[test]
#[serial]
fn the_record_ctor_is_deterministic_across_runs() {
    fn once(tag: &str) -> (u64, u64) {
        let ix = cerulion_core::testing::iceoryx_test_config();
        let w = supervisor_word(tag, "consumer", "inp", DEPTH as u32);
        let clock_p = Arc::new(VirtualClock::new());
        let mgr_p = manager("cb_recdet_p", ix.clone(), Arc::clone(&clock_p));
        let (cfg_p, fac_p) = producer_graph("cbrd");
        let bind_p = [binding(
            &w.producer_side,
            "consumer",
            "inp",
            DEPTH,
            CreditRole::Producer,
        )];
        let mut rt_p = GraphRuntime::build_live_deterministic_free_run(
            cfg_p,
            fac_p,
            &mgr_p,
            Arc::clone(&clock_p),
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            None,
            CrossProcessWiring::with_credit(None, &bind_p),
        )
        .expect("record ctx");
        let clock_c = Arc::new(VirtualClock::new());
        let mgr_c = manager("cb_recdet_c", ix, Arc::clone(&clock_c));
        let (cfg_c, fac_c) = consumer_graph(
            "cbrd",
            "consumer",
            "inp",
            Box::new(SplitBlockConsumerEntry::new()),
        );
        let bind_c = [binding(
            &w.consumer_side,
            "consumer",
            "inp",
            DEPTH,
            CreditRole::Consumer,
        )];
        let rt_c = GraphRuntime::build_live_free_run(
            cfg_c,
            fac_c,
            &mgr_c,
            Arc::clone(&clock_c) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind_c),
        )
        .expect("consumer ctx");
        for _ in 0..8 {
            step_quantum(&mut rt_p);
        }
        let out = (
            rt_p.node_handle("producer").expect("h").fire_count(),
            w.producer_side.outstanding(),
        );
        rt_p.shutdown();
        rt_c.shutdown();
        out
    }
    let a = once("recdet_a");
    let b = once("recdet_b");
    assert_eq!(
        a, b,
        "the RECORD ctor must be run-to-run identical (Principle #7)"
    );
    assert_eq!(
        a,
        (DEPTH, DEPTH),
        "hand oracle: the consumer never drains, so the producer plateaus at depth and the \
         word holds exactly that"
    );
}

/// The exemption is EDGE-SPECIFIC: crediting one producer-less `block` input
/// does not quietly exempt its neighbour.
///
/// `CreditedEdges` is a SET keyed on `(topic, node, input)`, and the arms above
/// would all pass an implementation whose `contains` returned `true` — every
/// one of them credits every producer-less edge it declares. This is the arm
/// that separates the two.
#[test]
#[serial]
fn crediting_one_producer_less_block_input_does_not_exempt_its_neighbour() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("edgespec", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_edgespec", ix, Arc::clone(&clock));
    // TWO producer-less `block` consumers on DIFFERENT topics; only the first
    // is credited.
    let cfg = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cbes_two_inputs".to_string(),
        prefix: "cbes".to_string(),
        nodes: vec![
            node("consumer", vec![topic_in("inp")], vec![]),
            node(
                "consumer2",
                vec![InputDef {
                    name: "other".to_string(),
                    source: OTHER_TOPIC.to_string(),
                }],
                vec![],
            ),
        ],
    };
    let mut fac: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fac.insert(
        "consumer".to_string(),
        Box::new(SplitBlockConsumerEntry::new()),
    );
    fac.insert(
        "consumer2".to_string(),
        Box::new(SecondBlockConsumerEntry::new()),
    );
    let bind = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "the UNCREDITED producer-less block input must still be refused",
    );
    let text = err.to_string();
    assert_names(
        &text,
        &[OTHER_TOPIC, "consumer2", "other", "no in-graph producer"],
        "uncredited-neighbour",
    );
    assert!(
        !text.contains("'consumer.inp'"),
        "the CREDITED edge must not be the one named: {text}"
    );
}

/// A binding over a process-LOCAL word credits NOTHING.
///
/// `CreditBinding` is public with public fields and `CreditWord::local` is
/// public, so a local-word binding is constructible by anyone — and it must not
/// buy the producer-less-`block` exemption, because nothing in another process
/// is holding the other end of a heap word. Without the `is_mapped` filter in
/// `CreditedEdges::from_bindings` this build would SUCCEED, and the consumer
/// would drain a word no producer can ever see.
#[test]
#[serial]
fn a_binding_over_a_local_word_does_not_buy_the_exemption() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_localword", ix, Arc::clone(&clock));
    let (cfg, fac) = consumer_graph(
        "cblw",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind = [CreditBinding {
        topic: TOPIC.to_string(),
        consumer_node: "consumer".to_string(),
        consumer_input: "inp".to_string(),
        depth: DEPTH,
        role: CreditRole::Consumer,
        word: CreditWord::local(DEPTH as u32),
        // A pure CONSUMER binding: no producer half, therefore no slot.
        producer_slot: None,
    }];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a LOCAL word must not exempt a producer-less `block` topic",
    );
    assert!(
        err.to_string().contains("no in-graph producer"),
        "the ordinary producer-less-block refusal is what a local-word binding meets: {err}"
    );
}

/// The unlink-first contract, OBSERVED rather than claimed: a second
/// `create_owned` under a live, ARMED name replaces the page rather than
/// adopting it.
///
/// Every create-failure message rests on this (it is why a concurrent
/// deployment is not what a failure means), and the earlier owner is
/// deliberately LEAKED with `mem::forget` so the old page is still mapped and
/// still holds its outstanding count when the second create runs — the shape a
/// crashed supervisor leaves behind.
#[test]
#[serial]
fn a_stale_armed_orphan_is_replaced_by_the_next_create() {
    let ns = credit_ns("orphan");
    let id = credit_edge_id(TOPIC, "consumer", "inp");
    let first = MappedCredit::create_owned(&ns, &id, 8).expect("first owner");
    first.record_published_n(5);
    assert_eq!((first.depth(), first.outstanding()), (8, 5));
    // Leak it: its `Drop` must NOT be what makes the next create clean.
    std::mem::forget(first);

    let second = MappedCredit::create_owned(&ns, &id, 2).expect("second owner");
    assert_eq!(
        (second.depth(), second.outstanding()),
        (2, 0),
        "the create UNLINKS the stale name and O_EXCL-creates a FRESH zero-filled page — it \
         never adopts the orphan's depth or its outstanding count"
    );
    let peer = MappedCredit::open_unowned(&ns, &id).expect("a peer opens the new page");
    assert_eq!(
        (peer.depth(), peer.outstanding()),
        (2, 0),
        "and a peer opening BY NAME reaches the new page, not the orphan"
    );
}

// ===========================================================================
// ADVERSARIAL — every skew the injection refuses
// ===========================================================================

/// A binding naming a CONSUMER edge this process does not declare is refused
/// loudly, naming the edge.
///
/// Wiring it anyway would install a probe on nothing while the peer's producer
/// defers against a word nobody drains — a silent wedge, which is exactly the
/// class the credit word exists to make impossible.
#[test]
#[serial]
fn a_binding_naming_an_edge_this_process_does_not_own_is_refused_loudly() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("skew_c", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_skew_c", ix, Arc::clone(&clock));
    // A graph that touches NEITHER end of the credited topic, so the role
    // NEGATIVE cannot fire first and the "declares no such input" arm is what
    // is under test.
    let (cfg, fac) = unrelated_graph("cbsc");
    let bind = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a consumer-role binding on a graph with no such input must be refused",
    );
    // NOT `"inp"`: it is a substring of the static word "input" in the very
    // message under test, so it would pass without any interpolation at all.
    assert_names(
        &err.to_string(),
        &["consumer", TOPIC, "declares no such input"],
        "unowned-consumer",
    );
}

/// The mirror-image skew: a PRODUCER-role binding on a process that has no
/// in-graph producer for the topic.
#[test]
#[serial]
fn a_producer_role_binding_without_a_local_producer_is_refused_loudly() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("skew_p", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_skew_p", ix, Arc::clone(&clock));
    // A graph with NEITHER end of THIS topic, so neither role negative fires
    // and the producer half's own "no in-graph producer" arm is isolated.
    let (cfg, fac) = unrelated_graph("cbsp");
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a producer-role binding without a local producer must be refused",
    );
    // NOT `"PRODUCER"`: it is static text in the message under test. The
    // interpolated halves are the topic and the edge's consumer identity.
    assert_names(
        &err.to_string(),
        &[TOPIC, "consumer", "inp", "no in-graph producer"],
        "no-local-producer",
    );
}

/// ROLE NEGATIVE 1: a `Consumer` binding on a rank that ALSO owns the producer.
///
/// The wiring loop SKIPS every bound edge, so consumer-only wiring here removes
/// the local defer and installs only the drain — the producer then publishes
/// UNTHROTTLED on an edge declared lossless, with nothing to report it. `Both`
/// is the correct role, and it is what
/// `a_both_role_binding_wires_one_word_for_a_co_located_edge` exercises.
#[test]
#[serial]
fn a_consumer_only_binding_on_a_rank_that_also_produces_is_refused_loudly() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("role_c", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_role_c", ix, Arc::clone(&clock));
    let cfg = config_of(
        "cbrc",
        "both_ends",
        vec![
            node("producer", vec![], vec![producer_out()]),
            node("consumer", vec![topic_in("inp")], vec![]),
        ],
    );
    let mut fac: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fac.insert("producer".to_string(), Box::new(SplitProducerEntry::new()));
    fac.insert(
        "consumer".to_string(),
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a CONSUMER-only role on a rank that also produces must be refused",
    );
    // NOT `"producer"`: the message says "producer(s):" in static text, and
    // the local node is called `producer` too, so the needle could not tell the
    // interpolation from the prose. The topic and the consumer edge can.
    assert_names(
        &err.to_string(),
        &["CONSUMER ONLY", TOPIC, "consumer", "unthrottled", "`Both`"],
        "consumer-only role",
    );
}

/// ROLE NEGATIVE 2: a `Producer` binding on a rank that ALSO declares the
/// consumer input — the mirror. Producer-only wiring installs the defer half
/// and nothing decrements on the local drain, so the producer defers forever.
#[test]
#[serial]
fn a_producer_only_binding_on_a_rank_that_also_consumes_is_refused_loudly() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("role_p", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_role_p", ix, Arc::clone(&clock));
    let cfg = config_of(
        "cbrp",
        "both_ends",
        vec![
            node("producer", vec![], vec![producer_out()]),
            node("consumer", vec![topic_in("inp")], vec![]),
        ],
    );
    let mut fac: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fac.insert("producer".to_string(), Box::new(SplitProducerEntry::new()));
    fac.insert(
        "consumer".to_string(),
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a PRODUCER-only role on a rank that also consumes must be refused",
    );
    assert_names(
        &err.to_string(),
        &[
            "PRODUCER ONLY",
            TOPIC,
            "consumer",
            "inp",
            "defer forever",
            "`Both`",
        ],
        "producer-only role",
    );
}

/// THE HIGH ONE: a plan depth that disagrees with the LOCAL declaration is
/// refused, naming BOTH numbers.
///
/// The probe's defer threshold is the PLAN's depth while the real iceoryx2
/// queue is sized from the LOCAL declaration, so a plan depth above the local
/// one evicts frames on an edge declared lossless — silently, because a
/// `block`-probed input installs no eviction detector. Reachable from a rebuild
/// between the supervisor's harvest and the worker's cdylib load.
#[test]
#[serial]
fn a_plan_depth_that_disagrees_with_the_local_declaration_is_refused_loudly() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    // The WORD is stamped at the plan depth (8) — the CLI's own check compares
    // plan-vs-word and would pass. Only the runtime holds the local
    // declaration (2), which is the point.
    let w = supervisor_word("depthskew", "consumer", "inp", 8);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_depthskew", ix, Arc::clone(&clock));
    let (cfg, fac) = consumer_graph(
        "cbds",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        8,
        CreditRole::Consumer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a plan depth above the LOCAL declaration must be refused, not silently evict",
    );
    assert_names(
        &err.to_string(),
        &["depth 8", "depth 2", "EVICTS", TOPIC, "consumer"],
        "depth-skew",
    );
}

/// A binding on an input the local graph declares NON-`block` is refused.
#[test]
#[serial]
fn a_binding_on_an_input_declared_non_block_is_refused_loudly() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("policyskew", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_policyskew", ix, Arc::clone(&clock));
    let (cfg, fac) = consumer_graph(
        "cbps",
        "consumer",
        "inp",
        Box::new(PlainConsumerEntry::new()),
    );
    let bind = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a credit binding on a non-`block` input must be refused",
    );
    assert_names(
        &err.to_string(),
        &["not `block`", "DropOldest", TOPIC],
        "policy-skew",
    );
}

/// A binding on a LOCALLY MIXED topic is refused.
///
/// A mixed topic degrades its `block` consumers to `drop_oldest` (deferring the
/// producer would starve the non-block sibling). A bound edge there would defer
/// the peer producer on behalf of a consumer that is meanwhile DROPPING — both
/// halves of the mixed-topic rule broken at once.
#[test]
#[serial]
fn a_binding_on_a_locally_mixed_topic_is_refused_loudly() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("mixed", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_mixed", ix, Arc::clone(&clock));
    let cfg = config_of(
        "cbmx",
        "mixed_ctx",
        vec![
            node("consumer", vec![topic_in("inp")], vec![]),
            node("sibling", vec![topic_in("inp")], vec![]),
        ],
    );
    let mut fac: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fac.insert(
        "consumer".to_string(),
        Box::new(SplitBlockConsumerEntry::new()),
    );
    fac.insert("sibling".to_string(), Box::new(PlainConsumerEntry::new()));
    let bind = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a credit binding on a locally MIXED topic must be refused",
    );
    assert_names(
        &err.to_string(),
        &["MIXED", "drop_oldest", TOPIC, "own topic"],
        "mixed-topic",
    );
}

/// ANTI-TAUTOLOGY for every refusal above, and the arm that proves the
/// exemption is load-bearing: WITHOUT a credit binding, the very same consumer
/// graph is refused by `GraphTopology::validate`'s producer-less-`block` arm.
#[test]
#[serial]
fn without_a_credit_binding_the_split_consumer_graph_is_refused_as_producer_less() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_notoken", ix, Arc::clone(&clock));
    let (cfg, fac) = consumer_graph(
        "cbnt",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::none(),
        ),
        "with no credit binding the producer-less block arm must still refuse",
    );
    let msg = err.to_string();
    assert!(
        msg.contains("no in-graph producer"),
        "the un-exempted refusal is the ordinary producer-less-block one: {msg}"
    );
    // WHOLE SENTENCES from the worker-side remedy.
    //
    // That literal is multi-line and continuation-joined, which is the shape
    // that can carry runs of embedded spaces baked into the
    // string (a botched line-join leaves the source indentation INSIDE the
    // literal, and `cargo fmt` never rewrites a string's contents). So
    // the rendered text is asserted here,
    // in the one arm that reaches it.
    for sentence in [
        "this scheduler can only defer producers in THIS graph",
        "the partition split the producer away from this consumer and the plan carried no \
         cross-process credit word for the edge — one is minted only for a topic with \
         exactly ONE in-graph producer and NO non-`block` consumers.",
        "never switch to drop_oldest for that, it silently loses data",
    ] {
        assert!(
            msg.contains(sentence),
            "the worker-side remedy renders mangled whitespace — expected `{sentence}`; \
             got: {msg}"
        );
    }
}

// ===========================================================================
// The split-edge defer-count gap, pinned as a GAP
// ===========================================================================

/// On a split edge the consumer's `backpressure_block_fires_deferred_count`
/// reads ZERO, and the producer's once-per-regime `warn!` is the surviving
/// observable.
///
/// This is a DOCUMENTED consequence, not a bug being hidden: the counters are
/// registered on the consumer's `NodeHandle` but bumped by the pre-fire gate,
/// which on a split edge runs in the producer's process against its own copy.
/// Pinned so that closing the gap later is a deliberate, visible change rather
/// than a silent one.
///
/// The consumer runtime is bound WITHOUT `mut`, so stepping it is a COMPILE
/// error: every captured line therefore comes from the producer's process, and
/// the warn assertion below cannot be satisfied by the consumer's own probe.
#[test]
#[serial]
#[traced_test]
fn a_split_edge_consumer_reads_a_zero_defer_count_and_the_producer_warns() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("gap", "consumer", "inp", DEPTH as u32);
    let clock_p = Arc::new(VirtualClock::new());
    let bind_p = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let (_mgr_p, mut rt_p) = producer_ctx("cbg", &ix, "cb_gap_p", &clock_p, &bind_p);
    let clock_c = Arc::new(VirtualClock::new());
    let mgr_c = manager("cb_gap_c", ix, Arc::clone(&clock_c));
    let (cfg_c, fac_c) = consumer_graph(
        "cbg",
        "consumer",
        "inp",
        Box::new(SplitBlockConsumerEntry::new()),
    );
    let bind_c = [binding(
        &w.consumer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Consumer,
    )];
    let rt_c = GraphRuntime::build_live_free_run(
        cfg_c,
        fac_c,
        &mgr_c,
        Arc::clone(&clock_c) as Arc<dyn Clock>,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind_c),
    )
    .expect("consumer ctx");
    for i in 0..8 {
        step_free_run(&mut rt_p, &clock_p, i);
    }
    assert!(
        rt_p.node_handle("producer").expect("h").fire_count() < 8,
        "precondition: the producer really was deferred (else the gap assertion is vacuous)"
    );
    assert_eq!(
        rt_c.node_handle("consumer")
            .expect("consumer handle")
            .backpressure_block_fires_deferred_count("inp"),
        0,
        "DOCUMENTED GAP: the defers happened in the producer's process, so the consumer's own \
         counter never moves. The accessor still EXISTS and answers — it is a zero, not an \
         error — and the producer's once-per-regime warn names this edge"
    );
    // The SURVIVING observable. Matched as WHOLE WHITESPACE TOKENS on ONE line
    // at WARN, plus the message text that belongs to the PRODUCER's pre-fire
    // arm only: `tracing_test` renders the span name (this test's own name,
    // which contains "consumer") into every line, so a bare
    // `logs_contain("consumer")` would be a substring test over text the test
    // itself controls, and the field tokens alone would not say which side
    // emitted it.
    logs_assert(|lines: &[&str]| {
        let hit = lines.iter().any(|l| {
            let toks: Vec<&str> = l.split_whitespace().collect();
            toks.contains(&"WARN")
                && toks.contains(&"node_id=consumer")
                && toks.contains(&"input=inp")
                && l.contains("producer's tick deferred")
        });
        if hit {
            Ok(())
        } else {
            Err(format!(
                "the PRODUCER's block-defer warn must name the FOREIGN consumer edge \
                 (WARN + node_id=consumer + input=inp + \"producer's tick deferred\") — it is \
                 the only observable the split leaves; captured: {lines:#?}"
            ))
        }
    });
    rt_p.shutdown();
    rt_c.shutdown();
}

// ==========================================================================
// The DIRECTION-B wake plane — a credit-blocked producer
// parks on the credit word and a peer consumer's drain wakes it.
//
// Every arm here is about WHEN the producer wakes, never about WHAT it fires:
// the park is record-only WAIT machinery (Principle #7), so the fire set and
// order are the pre-fire gate's business and are pinned by the arms above.
// ==========================================================================

/// A MAPPED producer edge JOINS the park watch list — so `has_credit_edges()`
/// is true and the live loop idles in `monitor_wait_block` rather than the
/// select-backed WaitSet (a credit word is an SHM page, and epoll has no line
/// into one).
///
/// Paired with the LOCAL arm below, which must report ZERO on the same
/// observable. The two are separate `#[test]`s for a mechanical reason, not a
/// stylistic one: `producer_out()` pins an ABSOLUTE topic, so two producer
/// contexts alive at once in this process collide on the single-writer rule and
/// the second build is refused before either assertion runs. Sequencing them as
/// tests keeps each context alone.
///
/// This half is the anti-vacuity partner: it proves the counter MOVES, without
/// which the local arm's zero would be satisfied by a watch list that is never
/// populated at all.
#[test]
#[serial]
fn prc_a_mapped_producer_edge_joins_the_park_watch_list() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("parkset", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let (_mgr, rt) = producer_ctx("cbps", &ix, "cb_parkset_m", &clock, &bind);
    assert_eq!(
        rt.credit_park_edge_count_for_test(),
        1,
        "a MAPPED producer edge joins the park watch list"
    );
}

/// A LOCAL producer edge joins NOTHING — and that is a correctness fence, not
/// an optimisation.
///
/// A local word's consumer is a node of the SAME runtime, fired by the same
/// `step()`, so a live loop that blocked waiting for that word would be waiting
/// for a drain only it can perform: deadlock. An empty watch list is what makes
/// that unreachable, and it is also what keeps every single-process run's idle
/// behaviour byte-identical to before this plane existed.
///
/// Identical in every respect to the mapped arm above except the WORD — same
/// graph, same role, same depth — so the zero is attributable to the local/
/// mapped split and to nothing else.
#[test]
#[serial]
fn prc_a_local_producer_edge_never_joins_the_park_watch_list() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let clock = Arc::new(VirtualClock::new());
    let bind = [CreditBinding {
        topic: TOPIC.to_string(),
        consumer_node: "consumer".to_string(),
        consumer_input: "inp".to_string(),
        depth: DEPTH,
        role: CreditRole::Producer,
        word: CreditWord::local(DEPTH as u32),
        producer_slot: Some(cerulion_core::credit::ProducerSlot::new(0)),
    }];
    let (_mgr, rt) = producer_ctx("cbpsl", &ix, "cb_parkset_l", &clock, &bind);
    assert_eq!(
        rt.credit_park_edge_count_for_test(),
        0,
        "a LOCAL word never joins the watch list — its consumer is a node of THIS runtime, \
         so parking on it would be waiting for a drain only this loop can perform"
    );
}

/// HEADLINE (direction B): a peer consumer's drain wakes a credit-blocked
/// producer's park, and the wake is ATTRIBUTED to the credit word.
///
/// # Why the attribution counter is the oracle, not a wall clock
///
/// The producer fires eventually either way — the park's recheck cadence would
/// get there on its own — so "it fired" cannot distinguish working wake
/// plumbing from absent wake plumbing. What distinguishes them is WHICH
/// predicate ended the park, and that is exactly what `park_wakes_credit`
/// records. A wall assertion tight enough to separate the two is also tight
/// enough for a loaded runner to invert (the class), so the wall here
/// is only a generous ceiling on the whole arm, never the discriminator.
///
/// The drain runs on a SECOND thread on purpose: the park baseline is taken at
/// park ENTRY, so a drain that landed before the call would be absorbed into
/// the baseline and (correctly) wake nothing. The condition under test is a
/// drain arriving WHILE the producer is parked, which is the shipping shape.
///
/// Dropping `credit_got` from the park predicate
/// leaves this arm's counter at 0 while the producer still eventually fires —
/// which is the whole point of asserting the counter.
#[test]
#[serial]
fn prc_a_peer_drain_wakes_a_credit_blocked_producers_park() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("parkwake", "consumer", "inp", DEPTH as u32);

    let clock = Arc::new(VirtualClock::new());
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let mgr = manager("cb_parkwake_p", ix, Arc::clone(&clock));
    let (cfg, fac) = producer_graph("cbpw");
    let mut rt = GraphRuntime::build_live_free_run(
        cfg,
        fac,
        &mgr,
        Arc::clone(&clock) as Arc<dyn Clock>,
        None,
        // The park is FORCED on: this arm is about the park's predicate, and
        // the resolver's own answer depends on the host CPU.
        forced_park_policy(),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .expect("build the producer context with its credit binding");

    // Pin the edge FULL by hand, so the producer is genuinely blocked when it
    // goes to park — `outstanding == depth` is exactly the defer condition.
    w.producer_side.record_published_n(DEPTH);
    assert!(
        w.producer_side.is_full(),
        "precondition: the edge is at its declared depth, so the producer defers"
    );

    let (_, _, _, timeout_before) = rt.park_wake_counts();
    let credit_before = rt.park_wakes_credit_count_for_test();

    // The peer frees credit WHILE the producer is parked.
    let peer = Arc::clone(&w.consumer_side);
    let drainer = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(PARK_ENTRY_SETTLE_MS));
        peer.record_drained(1);
    });
    rt.run_live_step_once_for_test(Duration::from_millis(PARK_WINDOW_MS));
    drainer.join().expect("the drainer thread");

    let credit_after = rt.park_wakes_credit_count_for_test();
    let (_, _, _, timeout_after) = rt.park_wake_counts();
    assert!(
        credit_after > credit_before,
        "the peer's drain must WAKE the parked producer through the credit word \
         (park_wakes_credit {credit_before} -> {credit_after})"
    );
    assert_eq!(
        timeout_after, timeout_before,
        "and it must be the CREDIT wake that ended the park, not the recheck \
         timeout — a timeout exit here would mean the wake plumbing is inert and \
         the producer merely waited out its slice"
    );
}

/// THE NO-FALSE-WAKE CONTROL — and the reason the predicate is gated on
/// was-full-at-park-entry rather than on the word's state alone.
///
/// A producer that is NOT blocked must be woken by NOTHING on this plane, even
/// while its peer drains busily. The two ungated predicates both fail here, in
/// different directions: an absolute "the edge has room" test reads true on
/// every recheck (an unbounded tight loop), and a bare
/// "the epoch advanced" test fires once per peer drain (the stale-wake
/// class — a 1 kHz consumer would wake this loop 1000x/s for nothing).
///
/// Deliberately NOT folded into the headline arm: this asserts an ABSENCE, and
/// an absence assertion sharing a body with a presence assertion can be
/// satisfied by the apparatus simply not running. Here the drain really
/// happens — `wake_seq` is asserted to have MOVED — so the zero is earned.
#[test]
#[serial]
fn prc_an_unblocked_producer_is_not_woken_by_a_peer_drain() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("nofalse", "consumer", "inp", DEPTH as u32);

    let clock = Arc::new(VirtualClock::new());
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let mgr = manager("cb_nofalse_p", ix, Arc::clone(&clock));
    let (cfg, fac) = producer_graph("cbnf");
    let mut rt = GraphRuntime::build_live_free_run(
        cfg,
        fac,
        &mgr,
        Arc::clone(&clock) as Arc<dyn Clock>,
        None,
        forced_park_policy(),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .expect("build the producer context with its credit binding");

    // The edge is EMPTY: this producer is not deferred on it at all.
    assert!(
        !w.producer_side.is_full(),
        "precondition: the producer is NOT blocked"
    );
    let credit_before = rt.park_wakes_credit_count_for_test();
    let entries_before = rt.park_entry_count_for_test();
    let seq_before = w.producer_side.wake_seq_snapshot();

    // The peer publishes and drains repeatedly during the park — real motion on
    // the word, which a bare epoch predicate would report as a wake.
    let peer = Arc::clone(&w.consumer_side);
    let drainer = std::thread::spawn(move || {
        for _ in 0..PEER_CHURN_DRAINS {
            peer.record_published();
            peer.record_drained(1);
            std::thread::sleep(Duration::from_millis(1));
        }
    });
    rt.run_live_step_once_for_test(Duration::from_millis(PARK_WINDOW_MS));
    drainer.join().expect("the drainer thread");

    assert!(
        w.producer_side.wake_seq_snapshot() != seq_before,
        "anti-vacuity: the peer really did move the credit epoch, so a zero below \
         is an EARNED zero and not an apparatus that never ran"
    );
    // APPARATUS assertion. The epoch check above proves the PEER moved; it says
    // nothing about whether this runtime ever parked. Without this, a routing
    // regression that stopped entering `monitor_wait_block` altogether would
    // produce these exact observations — epoch moved, credit wakes zero — and
    // pass. `park_entry_count` counts entries to that function and is
    // tier-INDEPENDENT (unlike the kernel-block count), so it holds on every
    // host.
    assert!(
        rt.park_entry_count_for_test() > entries_before,
        "APPARATUS: the runtime must actually have ENTERED the park ({} -> {}); \
         a zero credit-wake count from a loop that never parked proves nothing \
         about the predicate",
        entries_before,
        rt.park_entry_count_for_test()
    );
    assert_eq!(
        rt.park_wakes_credit_count_for_test(),
        credit_before,
        "an UNBLOCKED producer must take ZERO credit wakes however busy its peer is — \
         the predicate is gated on was-full-at-park-entry precisely so that credit \
         contributes no wake source at all when this producer is not waiting on one"
    );
}

/// A producer slot beyond the 32-wide `parked` mask DEGRADES to the slice
/// cadence — correct, bounded, and never a wedge.
///
/// The bit is what tells a freeing drain to pay the wake syscall, so a producer
/// with no bit is simply never wake-syscalled: it keeps its recheck pacing and
/// still fires. The pin is that the run COMPLETES and the producer never takes
/// a credit wake — not that the wake is merely slower, which a timing assert
/// could not tell apart from a loaded box.
///
/// Unreachable from the shipped mint today (the scope fence gives every edge
/// exactly one producer, so the only slot is 0), which is exactly why it is
/// worth a test: nothing else in the suite would notice if the guard inverted.
#[test]
#[serial]
fn prc_a_slot_beyond_the_parked_mask_degrades_to_the_slice_cadence() {
    // This arm lives entirely inside the `!performed` kernel-block path, so it is
    // only distinguishable on the tier that REACHES that path. Two hosts cannot:
    //
    //  - a HARDWARE-park machine (x86 WAITPKG, aarch64 WFE): the CPU
    //    monitor performs the park, `performed == true`, and the wake-word arm
    //    never runs — so NEITHER an in-mask nor a beyond-mask slot enters the
    //    kernel block, and the counter is 0 for both by design. This is arm A of
    //    `barrier_park_wake_iox2_test`'s tier matrix, one plane over.
    //  - a host with no wake-word primitive at all: nobody takes the block.
    //
    // Measured the hard way: this test passed on macOS and FAILED on
    // aarch64 WFE hardware at its own CONTROL (`0 -> 0`), which is the control doing its job.
    let hw_park = cerulion_core::monitor_wait::monitor_wait_available();
    if hw_park || !cerulion_core::credit::credit_wake_word_primitive_available() {
        eprintln!(
            "SKIP: this host parks in hardware (hw_park={hw_park}) or has no \
             wake-word primitive, so the kernel-block arm is not reached and the in-mask \
             and beyond-mask slots are indistinguishable here. The bit claim itself is \
             pinned tier-independently by \
             `credit_test::a_slot_beyond_the_mask_claims_no_parked_bit`."
        );
        return;
    }

    // CONTROL: an IN-MASK slot claims its bit, mints a `ParkedEdgeGuard`, and
    // therefore ENTERS the credit kernel block. Without this, the pin below
    // would be satisfied by a run that never reached the park at all.
    let in_mask = credit_kernel_blocks_for_slot("inmask", 0);
    assert!(
        in_mask > 0,
        "CONTROL: an in-mask slot must ENTER the credit kernel block (got {in_mask}) — \
         if it does not, the beyond-mask zero below proves nothing"
    );

    // THE PIN: one past the mask owns no bit, so `ParkedEdgeGuard::enter`
    // returns `None` and the kernel block is SKIPPED — the producer falls
    // through to the bounded sleep and keeps its slice cadence.
    //
    // Deliberately NOT asserted on `park_wakes_credit`: that counter tracks the
    // park's RECHECK PREDICATE observing `wake_seq` move, which happens with or
    // without a bit (measured: a beyond-mask slot still records 1). The bit only
    // decides whether a freeing drain pays the wake SYSCALL, and entering the
    // kernel block is the observable that actually separates the two.
    let beyond = credit_kernel_blocks_for_slot("bigslot", cerulion_core::credit::PARKED_MASK_BITS);
    assert_eq!(
        beyond, 0,
        "a beyond-mask slot must NOT enter the credit kernel block — claiming a bit \
         would steal an in-mask producer's wake, and blocking on a word no drain will \
         syscall it about is the wedge this degradation avoids"
    );
}

/// Run one producer through a live park window with `slot`, with the peer
/// draining once mid-park, and return how many times it ENTERED the credit
/// kernel block.
///
/// The test needs a postcondition
/// after the runtime call: without one, a run that skips the beyond-mask park path entirely
/// passes it. The kernel-block count is the observable that distinguishes a slot
/// which claimed a bit from one which did not, and it is structural rather than
/// timing-based.
fn credit_kernel_blocks_for_slot(tag: &str, slot: u32) -> u64 {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word(tag, "consumer", "inp", DEPTH as u32);

    let clock = Arc::new(VirtualClock::new());
    let mut bind = binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    );
    bind.producer_slot = Some(cerulion_core::credit::ProducerSlot::new(slot));
    let bind = [bind];
    let mgr = manager(&format!("cb_{tag}_p"), ix, Arc::clone(&clock));
    let (cfg, fac) = producer_graph(tag);
    let mut rt = GraphRuntime::build_live_free_run(
        cfg,
        fac,
        &mgr,
        Arc::clone(&clock) as Arc<dyn Clock>,
        None,
        forced_park_policy(),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .expect("a beyond-mask slot still BUILDS — it degrades, it does not refuse");

    w.producer_side.record_published_n(DEPTH);
    assert!(
        w.producer_side.is_full(),
        "precondition: the edge is at its declared depth, so the producer defers"
    );
    let before = rt.park_wake_word_block_count_for_test();
    let peer = Arc::clone(&w.consumer_side);
    let drainer = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(PARK_ENTRY_SETTLE_MS));
        peer.record_drained(1);
    });
    rt.run_live_step_once_for_test(Duration::from_millis(PARK_WINDOW_MS));
    drainer.join().expect("the drainer thread");
    let delta = rt.park_wake_word_block_count_for_test() - before;
    rt.shutdown();
    delta
}

/// A PRODUCER binding with no slot is REFUSED, loudly.
///
/// The failure it prevents is the quiet kind: a producer with no slot claims no
/// bit, so every freeing drain skips its wake syscall and it falls back to the
/// park-timeout cadence — correct output, collapsed latency, and nothing in the
/// log. That is the shape one plane over, which is why the build stops
/// rather than carrying on.
#[test]
#[serial]
fn prc_a_producer_binding_without_a_slot_is_refused() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("noslot", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let mut bind = binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    );
    bind.producer_slot = None;
    let bind = [bind];
    let mgr = manager("cb_noslot_p", ix, Arc::clone(&clock));
    let (cfg, fac) = producer_graph("cbns");
    let err = build_err(
        GraphRuntime::build_live_free_run(
            cfg,
            fac,
            &mgr,
            Arc::clone(&clock) as Arc<dyn Clock>,
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            CrossProcessWiring::with_credit(None, &bind),
        ),
        "a PRODUCER binding with no slot must be refused, not silently un-parked",
    );
    assert_names(
        &err.to_string(),
        &[TOPIC, "consumer", "inp", "producer slot", "parked mask"],
        "missing-producer-slot",
    );
}

/// A consumer that is ALIVE but has STOPPED DRAINING is
/// reported — and a merely BUSY one is not.
///
/// # Why occupancy cannot carry this and the epoch can
///
/// On a mapped edge the defer gate reads a word another process owns. A word
/// pinned at `outstanding >= threshold` is produced by two completely different
/// situations — a consumer that will drain shortly (ordinary `block`
/// backpressure, exactly as designed) and one that is alive and has stopped
/// (this producer is retired forever) — and the occupancy reading is IDENTICAL
/// in both. Section 7's wedge alarm is blind here by construction: a deferred
/// producer never entered a tick, so there is no dwell to observe.
///
/// What separates them is MOTION on the wake epoch, because a consumer draining
/// at ANY rate bumps it on every freeing drain. That is what this pins.
///
/// # Both halves in one body, driven with the SAME stimulus
///
/// The stalled arm and the busy CONTROL differ in exactly one thing: whether
/// the peer drains. Same graph, same depth, same step count, same word. Without
/// the control the "it warned" assertion is satisfied by a reporter that warns
/// on every deferred step — which is the flood this latch exists to prevent and
/// which would pass a presence-only test.
///
/// The drive count is DERIVED from the shipped threshold, not hardcoded, so
/// raising the constant cannot silently leave this arm below the boundary.
#[test]
#[serial]
#[traced_test]
fn prc_a_consumer_that_stopped_draining_warns_and_a_busy_one_does_not() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("stall", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let (_mgr, mut rt) = producer_ctx("cbst", &ix, "cb_stall_p", &clock, &bind);

    // Pin the edge FULL and NEVER drain it: the peer is alive (its half of the
    // word is mapped and open) but is freeing nothing.
    w.producer_side.record_published_n(DEPTH);
    let steps = u64::from(cerulion_core::graph::runtime::CREDIT_STALL_UNMOVED_OBSERVATIONS) + 8;
    for i in 0..steps {
        step_free_run(&mut rt, &clock, i);
    }
    assert!(
        logs_contain("FULL and its credit has not moved"),
        "a consumer that freed NOTHING across {} consecutive deferred evaluations \
         must be reported — it is the shape the wedge-and-warn policy leaves \
         permanently deferred, and nothing else in the system names it",
        cerulion_core::graph::runtime::CREDIT_STALL_UNMOVED_OBSERVATIONS
    );
    // ONE loud head, not one per deferred step: the latch's whole job. The
    // suppressed repeats ride `debug!` and the decade re-announcements are the
    // latch's own pinned behaviour, so what is asserted here is that the WARN
    // level did not flood.
    assert_eq!(
        logs_assert(|lines: &[&str]| {
            let n = lines
                .iter()
                .filter(|l| l.contains("WARN") && l.contains("FULL and its credit has not moved"))
                .count();
            if n == 1 {
                Ok(())
            } else {
                Err(format!("expected exactly 1 loud WARN head, got {n}"))
            }
        }),
        (),
        "the stall regime opens ONCE — a per-deferred-step warn is the disk-fill \
         class this repo's shared latch exists to prevent"
    );
}

/// The BUSY-CONSUMER CONTROL — the half that makes the arm above a statement
/// about stalling rather than about deferring.
///
/// Identical stimulus and an identical number of steps, with ONE difference:
/// the peer frees a credit periodically. It is still deferred almost all the
/// time (the edge refills immediately), so a reporter keyed on "deferred for a
/// long time" fires here and fails — which is exactly the wrong reporter, and
/// the reason the predicate is MOTION rather than duration.
///
/// Its own `#[test]`, because `#[traced_test]` captures per test: sharing a
/// body with the stalled arm would make "did not warn" unassertable once the
/// stalled half had written its line.
#[test]
#[serial]
#[traced_test]
fn prc_a_busy_consumer_is_never_reported_as_stalled() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("busy", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let (_mgr, mut rt) = producer_ctx("cbbz", &ix, "cb_busy_p", &clock, &bind);

    w.producer_side.record_published_n(DEPTH);
    let steps = u64::from(cerulion_core::graph::runtime::CREDIT_STALL_UNMOVED_OBSERVATIONS) + 8;
    for i in 0..steps {
        // The peer drains one and the producer immediately refills it, so the
        // edge is at threshold on almost every evaluation — a DEFERRED
        // producer throughout, with credit genuinely MOVING.
        w.consumer_side.record_drained(1);
        w.producer_side.record_published();
        step_free_run(&mut rt, &clock, i);
    }
    assert!(
        w.producer_side.is_full(),
        "precondition: the edge really is at threshold, so this producer was \
         deferred — otherwise the absence below is about an unblocked producer \
         and proves nothing"
    );
    assert!(
        !logs_contain("FULL and its credit has not moved"),
        "a BUSY consumer must never be reported as stalled — it is freeing \
         credit on every step, which is `block` working exactly as designed"
    );
}

/// The ROUTING disjunct is load-bearing: a credit-blocked producer is woken
/// through the credit word even with the PARK OFF.
///
/// This is the shape one plane over. With `CERULION_MONITOR_WAIT=0` —
/// or on a target with no CPU monitor-wait primitive, where the CLI resolver
/// falls back to the WaitSet wait — a process that idled in the blocking
/// WaitSet could not observe a credit word at all, because epoll has no line
/// into an SHM page. `has_credit_edges()` in the four routing disjuncts is what
/// sends it to `monitor_wait_block` anyway, where the park opt-out is honoured
/// internally (sleep-recheck pacing, no CPU primitive) while the predicates are
/// still polled.
///
/// The headline arm cannot pin this: it FORCES the park on, so `park_active()`
/// alone satisfies every disjunct and the credit term is dead weight there.
/// With the park off it is the only term that can be true.
#[test]
#[serial]
fn prc_the_credit_wake_survives_the_park_being_off() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("parkoff", "consumer", "inp", DEPTH as u32);
    let clock = Arc::new(VirtualClock::new());
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let mgr = manager("cb_parkoff_p", ix, Arc::clone(&clock));
    let (cfg, fac) = producer_graph("cbpo");
    let mut rt = GraphRuntime::build_live_free_run(
        cfg,
        fac,
        &mgr,
        Arc::clone(&clock) as Arc<dyn Clock>,
        None,
        // THE difference from the headline arm: the park is OFF.
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .expect("build the producer context with its credit binding");

    w.producer_side.record_published_n(DEPTH);
    let (_, _, _, timeout_before) = rt.park_wake_counts();
    let credit_before = rt.park_wakes_credit_count_for_test();

    let peer = Arc::clone(&w.consumer_side);
    let drainer = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(PARK_ENTRY_SETTLE_MS));
        peer.record_drained(1);
    });
    rt.run_live_step_once_for_test(Duration::from_millis(PARK_WINDOW_MS));
    drainer.join().expect("the drainer thread");

    assert!(
        rt.park_wakes_credit_count_for_test() > credit_before,
        "with the park OFF the credit wake must STILL work — the routing \
         disjunct is what sends a credit-holding process to `monitor_wait_block` \
         instead of the blocking WaitSet, which cannot see an SHM page"
    );
    let (_, _, _, timeout_after) = rt.park_wake_counts();
    assert_eq!(
        timeout_after, timeout_before,
        "and it is the credit wake that ended the wait, not the recheck timeout"
    );
}

/// A LOCKSTEP rank that holds a barrier participant AND is credit-blocked at
/// depth does NOT wedge in the live park.
///
/// The shape looks like a possible DEADLOCK: the credit kernel-block arm is
/// tried before the barrier arm, so a lockstep rank could block on the credit
/// word while its peer waits at the barrier for an arrival that never comes.
/// Measured, it does not deadlock, for two independent reasons — and this
/// test pins both:
///
/// 1. The barrier RENDEZVOUS is `barrier.wait(gen, BARRIER_BOUNDARY_TIMEOUT)`
///    inside the step/level-advance path (`runtime.rs`), not inside
///    `monitor_wait_block`. The credit arm is the INTER-STEP IDLE park, reached
///    after this rank has already arrived at and passed its barrier generations
///    for the step, so a peer is never waiting on an arrival this rank is
///    withholding while blocked on credit.
/// 2. The block is BOUNDED: `cap = pace_slice.min(park_deadline - now)`, passed
///    to `park_wait_credit`, which is a futex wait with a `timespec` on Linux
///    and an `os_sync` wait with a relative timeout on macOS (a zero cap returns
///    immediately). A consumer that never drains delays this rank by at most one
///    pace slice, after which it re-derives at the loop top.
///
/// The existing lockstep+credit arm
/// (`a_lockstep_barrier_context_defers_on_the_same_mapped_word`) drives the
/// POLLED `step()`, which never enters `monitor_wait_block` at all, and the
/// live-seam arms carry no barrier participant — so only this arm covers
/// the lockstep + credit + live-step configuration.
///
/// The wedge condition is real here: the word is driven to depth and **nothing
/// ever drains it**. A true deadlock would HANG, so the live step runs on its
/// own thread and is joined with a timeout — a wedge fails loudly instead of
/// stalling the suite.
#[test]
#[serial]
fn prc_a_lockstep_barrier_rank_credit_blocked_at_depth_does_not_wedge() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let w = supervisor_word("lswedge", "consumer", "inp", DEPTH as u32);

    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("cb_lswedge_p", ix, Arc::clone(&clock));
    let (cfg, fac) = producer_graph("cblsw");
    // expected = 1: this rank's own arrival opens it. The barrier's own stall
    // path is a DIFFERENT mechanism with its own bounded escape
    // (`BARRIER_BOUNDARY_TIMEOUT` + terminal poison) and is pinned elsewhere;
    // what is under test here is the CREDIT arm's behaviour in a rank that has
    // a barrier participant at all.
    let barrier = Arc::new(
        MappedBarrier::create_owned(&credit_ns("lswedge_bar"), "levelgate", 1)
            .expect("barrier owner"),
    );
    let bind = [binding(
        &w.producer_side,
        "consumer",
        "inp",
        DEPTH,
        CreditRole::Producer,
    )];
    let mut rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        cfg,
        fac,
        &mgr,
        Arc::clone(&clock),
        None,
        // The park is FORCED ON: with it off the kernel-block arm is not the
        // thing being exercised, and the claim is about that arm.
        forced_park_policy(),
        Arc::clone(&barrier),
        vec![Some(0)],
        vec![false; 1],
        0,
        Duration::from_millis(PERIOD_MS),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .expect("build a LOCKSTEP rank that also holds a credit binding");

    assert_eq!(
        rt.credit_park_edge_count_for_test(),
        1,
        "PRECONDITION: a lockstep rank DOES watch its mapped credit edge — if this \
         were 0 the arm under test would be unreachable and the rest of this test \
         would prove nothing"
    );

    // The wedge condition: full, and nothing will ever drain it.
    w.producer_side.record_published_n(DEPTH);
    assert_eq!(
        w.producer_side.outstanding(),
        DEPTH,
        "PRECONDITION: the edge is AT depth, so the producer is genuinely blocked"
    );

    let blocks_before = rt.park_wake_word_block_count_for_test();
    let entries_before = rt.park_entry_count_for_test();

    // A real deadlock HANGS. Run the live step on its own thread so a wedge is
    // reported as a failure rather than stalling the whole binary.
    let (tx, rx) = std::sync::mpsc::channel();
    let h = std::thread::spawn(move || {
        let t0 = std::time::Instant::now();
        rt.run_live_step_once_for_test(Duration::from_millis(PARK_WINDOW_MS));
        let _ = tx.send(t0.elapsed());
        rt
    });
    let budget = Duration::from_secs(20);
    let elapsed = match rx.recv_timeout(budget) {
        Ok(d) => d,
        Err(_) => panic!(
            "WEDGE: a lockstep rank with a barrier participant, credit-blocked at depth \
             with no drain ever arriving, did not return from the live park within {budget:?}. \
             The credit kernel block is supposed to be capped by the park deadline."
        ),
    };
    let rt = h.join().expect("the live-step thread");

    assert!(
        elapsed < budget,
        "the live park returned, but took {elapsed:?} — the block must be capped by the \
         park deadline, not by the consumer draining"
    );

    // POSITIVE CONTROL. Without this, a run that never reached the credit arm
    // at all would satisfy the no-wedge assertion above — the vacuous-pass
    // class. Only meaningful where the kernel primitive exists; elsewhere the
    // arm degrades to the sleep, which is the same bounded cadence.
    // The no-wedge property above is tier-INDEPENDENT. The control is not: which
    // wait this rank performs depends on the host, so assert the strongest thing
    // each tier can actually show. Mirrors `barrier_park_wake_iox2_test`'s arm
    // matrix, and is the reason this test first passed on macOS and failed on
    // aarch64 WFE hardware — the control was written for one tier only.
    let hw_park = cerulion_core::monitor_wait::monitor_wait_available();
    let primitive = cerulion_core::credit::credit_wake_word_primitive_available();
    if !hw_park && primitive {
        // The tier that REACHES the kernel-block arm: it must have been entered,
        // or "it did not wedge" says nothing about the arm under test.
        assert!(
            rt.park_wake_word_block_count_for_test() > blocks_before,
            "CONTROL: the credit kernel-block arm must actually have been ENTERED \
             ({} -> {})",
            blocks_before,
            rt.park_wake_word_block_count_for_test()
        );
    } else {
        // A hardware-park box performs the park before that arm, and a host with
        // no primitive never takes it. Either way the rank must still have PARKED
        // — which is what makes "it returned" a statement about a park at all
        // rather than about a loop that never idled.
        assert!(
            rt.park_entry_count_for_test() > entries_before,
            "CONTROL: the rank must have PARKED (hw_park={hw_park}, primitive={primitive}); \
             entries {} -> {}",
            entries_before,
            rt.park_entry_count_for_test()
        );
    }
    assert_eq!(
        w.producer_side.outstanding(),
        DEPTH,
        "and the edge is still at depth — nothing drained it, so the return really \
         was the cap and not credit becoming available"
    );
    rt.shutdown();
}
