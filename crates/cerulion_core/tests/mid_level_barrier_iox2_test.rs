// SPDX-License-Identifier: AGPL-3.0-only
//! The MID-LEVEL barrier — two `period_ms` nodes at ONE global level,
//! joined by a plain NON-TRIGGER `#[input]`, split across two process groups.
//!
//! # The shape, and why the trigger-chain tests cannot see it
//!
//! `barrier_level_gate_iox2_test.rs` and `mp_record_replay_e2e_test.rs` are
//! all-`#[input(trigger)]` chains: a trigger edge levelizes its consumer strictly
//! BELOW its producer, so every pair there is separated by a level boundary and
//! ordered by the end-of-level rendezvous **by construction**. Those tests are
//! structurally blind to the one edge the DAG does not model — a latest-value
//! `#[input]`, whose consumer is fired by its own `period_ms` timer, so producer
//! and consumer share ONE global level and NOTHING between them orders the
//! producer's tick+publish against the consumer's `snapshot_inputs`.
//!
//! The consequence, measured on the `obstacle_avoidance` example without the barrier:
//! 7 of 12 process-per-node runs fail their own re-execution, and the
//! recordings differ **from each other** — the live run is nondeterministic,
//! so replay is structurally unable to reproduce it.
//!
//! # The fixture
//!
//! ONE global level, two contexts over one shared SHM root:
//!
//! * **A** — `scanner`, `period_ms = 1`, publishes an incrementing counter onto
//!   the graph-owned `/mlb/scan`.
//! * **B** — `controller`, `period_ms = 1`, reads `/mlb/scan` through a PLAIN
//!   (non-trigger) `#[input]` and records what it saw.
//!
//! Both own global level 0, so `MAP_A == MAP_B == [Some(0)]` and the deployment
//! crosses one end-of-level generation per step — plus, when the level is
//! FLAGGED, one mid-level generation before it.
//!
//! # Why the oracles are deterministic rather than raced
//!
//! Left to OS scheduling the unordered arm is a coin flip, which is exactly the
//! defect and exactly what a test must not assert on. So the interleave is FIXED
//! by the BARRIER'S OWN SIGNAL: context B does not begin a step until the
//! barrier reports its peer is already parked (`peers_waiting`). Where A parks
//! is decided by the flag and by nothing else —
//!
//! * UNFLAGGED, A's first rendezvous is the level's END, so "A is parked" means
//!   A has already TICKED AND PUBLISHED, and B's snapshot then reads A's
//!   SAME-step frame — the defect, on demand;
//! * FLAGGED, A's first rendezvous is the MID-LEVEL one, so "A is parked" means
//!   A has only DECIDED, and B snapshots the PRIOR frame before either ticks.
//!
//! Both arms run the IDENTICAL harness and the IDENTICAL gate, so the pair is a
//! genuine A/B: `mid_level_barrier: [true]` vs `[false]`, one flag apart,
//! nothing else changed. B never reads the flag — only its effect.
//!
//! The value oracle is HAND-WRITTEN (`prior_step_values`), never a second run of
//! the same code compared against itself.
//!
//! `#[serial]` + per-test SHM roots: the barrier registry and the iceoryx2
//! transport are process-global on macOS.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Measured steps. Small on purpose: every step is a full two-thread barrier
/// rendezvous, and the property is per-step, so more steps buy repetition, not
/// coverage.
const N: u64 = 6;

/// Priming steps, discarded. The first steps carry iceoryx2 connection warm-up
/// and the pre-first-delivery WAIT (the consumer has no held value yet),
/// neither of which the ordering property is about.
const WARMUP: u64 = 4;

/// One global level — both nodes own it. This is the whole point of the shape.
const GLOBAL_LEVELS: u64 = 1;

const STEP: Duration = Duration::from_millis(1);

/// The cross-context topic: graph-OWNED by A (single-writer), read by B as an
/// absolute external source.
const SCAN_TOPIC: &str = "/mlb/scan";

/// Both contexts own global level 0.
const MAP: [Option<usize>; 1] = [Some(0)];

// ===========================================================================
// Nodes.
// ===========================================================================

/// The PRODUCER: a `period_ms` node with no inputs at all, publishing an
/// incrementing counter. Fires on its own timer, so nothing levelizes it
/// against the consumer.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Scanner {
    #[output]
    scan: Vector3,
    count: f64,
}

#[cerulion_node_impl]
impl Scanner {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1.0;
        self.scan.x = self.count;
        Ok(())
    }
}

/// The CONSUMER: a `period_ms` node reading `scan` through a PLAIN
/// (non-trigger) `#[input]` — a latest-value read, NOT a DAG edge. It records
/// what each tick observed; that vector is the whole oracle.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Controller {
    #[input]
    scan: Vector3,
    observed: Arc<Mutex<Vec<f64>>>,
}

#[cerulion_node_impl]
impl Controller {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.observed.lock().unwrap().push(self.scan.x);
        Ok(())
    }
}

// ===========================================================================
// Config builders.
// ===========================================================================

fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        // ABSOLUTE topic override so the two contexts meet on ONE name (the
        // derived name would carry each context's own prefix).
        topic: Some(SCAN_TOPIC.to_string()),
        history_size: 0,
    }
}

fn base_config(name: &str, prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: name.to_string(),
        prefix: prefix.to_string(),
        nodes,
    }
}

/// CONTEXT A: the scanner alone (one local level).
fn context_a_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = base_config(
        "mlb_a",
        "mlba",
        vec![NodeDef {
            ros2: None,
            id: "scanner".to_string(),
            node_type: "scanner".to_string(),
            inputs: vec![],
            outputs: vec![out_def("scan")],
        }],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("scanner".to_string(), Box::new(ScannerEntry::new()));
    (config, factories)
}

/// CONTEXT B: the controller alone (one local level), reading the ABSOLUTE
/// `/mlb/scan` as an external source — the cross-group edge.
fn context_b_graph(
    observed: Arc<Mutex<Vec<f64>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = base_config(
        "mlb_b",
        "mlbb",
        vec![NodeDef {
            ros2: None,
            id: "controller".to_string(),
            node_type: "controller".to_string(),
            inputs: vec![InputDef {
                name: "scan".to_string(),
                source: SCAN_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "controller".to_string(),
        Box::new(ControllerEntry::with_state(Controller {
            observed,
            ..Default::default()
        })),
    );
    (config, factories)
}

fn manager(name: &str, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            // A snapshot-source topic needs `subscriber_max_borrowed_samples
            // >= 3`, which this buffer size carries — the consumer's plain
            // `#[input]` IS a snapshot source, so the default would refuse the build.
            subscriber_buffer_size: 16,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

fn barrier_ns(tag: &str) -> String {
    format!("{}_{}", tag, std::process::id())
}

/// HAND-WRITTEN oracle: with the snapshot-before-any-tick guarantee restored,
/// the consumer at measured step `k` reads the value the producer committed at
/// step `k - 1`.
///
/// Both nodes have been running through `WARMUP` steps, so the producer's
/// counter is already at `WARMUP` when the measured window opens: the first
/// measured read is `WARMUP` and the last is `WARMUP + n - 1`. Nothing here is
/// derived from a run — a self-compare could not tell the two arms apart, which
/// is the entire question.
fn prior_step_values(n: u64) -> Vec<f64> {
    (0..n).map(|k| (WARMUP + k) as f64).collect()
}

/// What the consumer reads when NOTHING orders the pair and the producer wins
/// the race: its OWN step's publish, one ahead of the oracle throughout.
fn same_step_values(n: u64) -> Vec<f64> {
    prior_step_values(n).into_iter().map(|v| v + 1.0).collect()
}

/// Block until the barrier reports the PEER is parked at the current
/// generation — see `run_split` for why this is the interleave gate.
///
/// Bounded: the peer cannot be overtaken (this thread has not arrived, so the
/// generation cannot open and the signal is stable once true), so expiry means
/// a genuinely mis-wired barrier and must fail LOUDLY rather than wedge the
/// suite. The ceiling is a liveness bound in seconds against microseconds of
/// real work — never a wall the property is measured in.
fn await_peer_parked(barrier: &MappedBarrier) {
    let deadline = std::time::Instant::now() + Duration::from_secs(20);
    while !barrier.peers_waiting() {
        assert!(
            std::time::Instant::now() < deadline,
            "peer never parked at the barrier — the interleave gate cannot proceed, so the \
             deployment is mis-wired (generation {})",
            barrier.current_generation()
        );
        std::hint::spin_loop();
    }
}

struct Outcome {
    observed: Vec<f64>,
    final_generation: u64,
    a_fires: usize,
    b_fires: usize,
}

/// Run the two contexts concurrently under ONE shared barrier, with
/// `mid_level_barrier = [mid]`.
///
/// # The FIXED INTERLEAVE, and why it is the barrier's OWN signal
///
/// Left to the scheduler the unordered arm is a coin flip — which is the defect
/// itself, and unassertable. So context B does not start a step until the
/// barrier reports that its PEER IS ALREADY WAITING (`peers_waiting`), i.e. A
/// has run everything it does before its next rendezvous and is parked. That
/// single gate makes BOTH arms deterministic, and it is the same gate in both,
/// so the arms really are ONE FLAG APART:
///
/// * UNFLAGGED — A's only rendezvous is at the level's END, so "A is waiting"
///   means A has already TICKED AND PUBLISHED. B then snapshots and reads A's
///   SAME-step frame: the defect, reproduced on demand.
/// * FLAGGED — A's first rendezvous is the MID-LEVEL one, so "A is waiting"
///   means A has only DECIDED; its tick has not run. B snapshots (reading the
///   PRIOR frame), arrives, and both tick after the generation opens.
///
/// The flag is observable to the gate only through its effect on WHERE A parks,
/// which is exactly the behaviour under test. B never reads the flag itself.
///
/// The wait is BOUNDED and panics on expiry: a mis-wired barrier must fail as a
/// loud test failure, never as a hung suite.
fn run_split(tag: &str, mid: bool, n: u64) -> Outcome {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("mlb_a", ix.clone());
    let mgr_b = manager("mlb_b", ix);

    // Build A first so it CREATES the graph-owned single-writer `/mlb/scan`;
    // B then opens it as an external source.
    let (cfg_a, fac_a) = context_a_graph();
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("build context A");

    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (cfg_b, fac_b) = context_b_graph(Arc::clone(&observed));
    let mut rt_b = GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new()))
        .expect("build context B");

    let ns = barrier_ns(tag);
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner create"));
    let peer = Arc::new(MappedBarrier::open_unowned(&ns, "g").expect("barrier peer open"));
    let probe = Arc::clone(&owner);

    rt_a.set_barrier_participant_with_mid_levels_for_test(owner, MAP.to_vec(), vec![mid], 0);
    rt_b.set_barrier_participant_with_mid_levels_for_test(peer, MAP.to_vec(), vec![mid], 1);

    // The run's generation-per-step law, read off the PLAN rather than restated
    // here — the whole reason `generations_per_step()` is an observable.
    let per_step = rt_a
        .generations_per_step()
        .expect("a barrier participant is installed");
    assert_eq!(
        per_step,
        rt_b.generations_per_step().expect("B participant"),
        "both contexts must agree on the generation law or the barrier desynchronises"
    );

    let gate = Arc::clone(&probe);
    let observed_for_b = Arc::clone(&observed);

    let a_handle = thread::spawn(move || {
        for _ in 0..WARMUP {
            rt_a.step(STEP);
        }
        rt_a.clear_trace();
        for _ in 0..n {
            rt_a.step(STEP);
        }
        let fires = rt_a.trace().len();
        rt_a.shutdown();
        drop(mgr_a);
        fires
    });

    let b_handle = thread::spawn(move || {
        for _ in 0..WARMUP {
            await_peer_parked(&gate);
            rt_b.step(STEP);
        }
        rt_b.clear_trace();
        // The warm-up reads are not part of the property: they carry iceoryx2
        // connection warm-up and the pre-first-delivery WAIT. Drop them
        // so the oracle describes the measured window only.
        observed_for_b.lock().unwrap().clear();
        for _ in 0..n {
            await_peer_parked(&gate);
            rt_b.step(STEP);
        }
        let fires = rt_b.trace().len();
        rt_b.shutdown();
        drop(mgr_b);
        fires
    });

    let a_fires = a_handle.join().expect("context A thread panicked");
    let b_fires = b_handle.join().expect("context B thread panicked");
    let observed = observed.lock().unwrap().clone();

    Outcome {
        observed,
        final_generation: probe.current_generation(),
        a_fires,
        b_fires,
    }
}

// ===========================================================================
// The pins.
// ===========================================================================

/// THE HEADLINE. With the level flagged, the consumer reads the PRIOR step's
/// value on every measured step — the monolith's snapshot-before-any-tick
/// guarantee, restored across two processes.
///
/// Asserted against the HAND oracle, so a run that merely agrees with itself
/// cannot pass.
#[test]
#[serial]
fn a_flagged_level_makes_the_consumer_read_the_prior_step_across_the_split() {
    let out = run_split("headline", true, N);

    assert_eq!(
        out.observed,
        prior_step_values(N),
        "a flagged level must order every group's snapshots before any group ticks, so the \
         consumer reads the PRIOR step's frame — got {:?}",
        out.observed
    );
    // Anti-vacuity: both nodes really fired every measured step (an oracle over
    // an empty or short vector proves nothing).
    assert_eq!(out.a_fires as u64, N, "producer must fire once per step");
    assert_eq!(out.b_fires as u64, N, "consumer must fire once per step");
}

/// THE ANTI-TAUTOLOGY CONTROL, one flag apart. The IDENTICAL fixture and the
/// IDENTICAL interleave with the level UNFLAGGED reproduces the defect:
/// the consumer reads its own step's publish.
///
/// Without this arm the headline is satisfied by a fixture whose interleave
/// never exposes the race in the first place — it would pass without the
/// mid-level barrier and pin nothing.
#[test]
#[serial]
fn an_unflagged_level_reproduces_the_unordered_read_the_flag_exists_to_fix() {
    let out = run_split("control", false, N);

    assert_eq!(
        out.observed,
        same_step_values(N),
        "the control must show the DEFECT: with no mid-level rendezvous the producer's \
         same-step publish lands before the consumer's snapshot — got {:?}",
        out.observed
    );
    assert_ne!(
        out.observed,
        prior_step_values(N),
        "if the control agrees with the flagged oracle the fixture never exposed the race \
         and the headline pins nothing"
    );
}

/// The GENERATION LAW, derived from the plan rather than restated. A flagged
/// level crosses TWO generations per step; the same fixture unflagged crosses
/// one — and BOTH are the pre/post halves of the cost this feature charges.
#[test]
#[serial]
fn a_flagged_level_costs_exactly_one_extra_generation_per_step() {
    let flagged = run_split("gen_on", true, N);
    let plain = run_split("gen_off", false, N);

    assert_eq!(
        flagged.final_generation,
        (WARMUP + N) * (GLOBAL_LEVELS + 1),
        "a flagged level crosses its mid-level rendezvous AND its end-of-level one"
    );
    assert_eq!(
        plain.final_generation,
        (WARMUP + N) * GLOBAL_LEVELS,
        "an unflagged level keeps exactly the pre-existing law"
    );
    assert_eq!(
        flagged.final_generation - plain.final_generation,
        WARMUP + N,
        "the whole cost is ONE extra rendezvous per flagged level per step"
    );
}

/// The FIREWALL: the barrier changes WHEN, never WHAT. Flagging a level must
/// not change how many times either node fires — only which frame the consumer
/// pairs with.
#[test]
#[serial]
fn flagging_a_level_changes_the_pairing_not_the_fire_set() {
    let flagged = run_split("fire_on", true, N);
    let plain = run_split("fire_off", false, N);

    assert_eq!(
        (flagged.a_fires, flagged.b_fires),
        (plain.a_fires, plain.b_fires),
        "the mid-level rendezvous is a WHEN-gate: the fire counts must be identical"
    );
    assert_eq!(flagged.a_fires as u64, N);
    assert_eq!(flagged.b_fires as u64, N);
}

/// DETERMINISM: two runs of the flagged split agree with each other AND with
/// the hand oracle. Two runs agreeing is not enough on its own — a split
/// without the mid-level barrier cannot do it, but a broken oracle could still agree with
/// itself — so both halves are asserted.
#[test]
#[serial]
fn the_flagged_split_is_deterministic_across_runs() {
    let first = run_split("det_1", true, N);
    let second = run_split("det_2", true, N);

    assert_eq!(first.observed, second.observed, "two runs must agree");
    assert_eq!(
        first.observed,
        prior_step_values(N),
        "and both must equal the hand oracle"
    );
    assert_eq!(first.final_generation, second.final_generation);
}

/// A flag vector whose length disagrees with the map is REFUSED.
///
/// This is the invariant that keeps two participants from crossing DIFFERENT
/// generation counts — the failure that does not announce itself, because a
/// desynchronised counter surfaces as every LATER rendezvous timing out on a
/// deployment whose actual bug is one short vector. Refusing at install time is
/// what turns it into a start-up error.
///
/// Driven through the test setter, which `.expect`s the same `Err` the
/// production `build_live_deterministic_with_manager_and_barrier` returns.
#[test]
#[serial]
#[should_panic(expected = "mid_level_barrier has 2 entries but global_level_map has 1")]
fn a_flag_vector_that_disagrees_with_the_map_is_refused() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = manager("mlb_len", ix);
    let (cfg, fac) = context_a_graph();
    let mut rt =
        GraphRuntime::build(cfg, fac, &mgr, Arc::new(VirtualClock::new())).expect("build A");

    let ns = barrier_ns("len_guard");
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier owner"));
    // ONE global level, TWO flags — the shape a supervisor sizing the vector
    // from the wrong graph would produce.
    rt.set_barrier_participant_with_mid_levels_for_test(owner, MAP.to_vec(), vec![false, true], 0);
}
