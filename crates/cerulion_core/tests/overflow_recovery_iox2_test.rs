// SPDX-License-Identifier: AGPL-3.0-only
//! Overflow RECOVERY-latency for the `drop_oldest` backpressure
//! policy, end-to-end over real iceoryx2 (`build_for_test`, per-test SHM
//! root — parallel-safe; `#[serial]` for the iceoryx2 SHM singleton).
//!
//! THE METRIC (age-of-information recovery transient). After a bounded
//! `drop_oldest` consumer (depth `D`) overflows, how fast does it get back to
//! FRESH data once the overload clears? Cerulion's data-trigger / body read
//! drains the bounded iceoryx2 queue TO LATEST
//! (`drain_to_latest_with_accounting`), so the FIRST read after the overload
//! clears already sees the newest sample — recovery is ~1 read FLAT regardless
//! of the overflow magnitude `K`, while the drop count scales as `K - D`. That
//! O(1)-recovery result is the moat; the drop count quantifies the
//! completeness traded for freshness (zero-copy, no Cerulion-side buffer).
//!
//! HARNESS (fully deterministic — `VirtualClock` + wire timestamps, NOT
//! wall-clock, Principle #7):
//!   - A `period_ms = 1` producer fires every `step()` and writes its publish
//!     time `self.now_ns()` into `out.x`. The publisher ALSO stamps the wire
//!     header `timestamp_ns` with the SAME `clock.now_ns()` at publish
//!     (`publisher.rs`), so the payload value IS the sample's wire timestamp by
//!     construction — "data-age = consumer logical-now − sample wire
//!     timestamp_ns" is computed in-node as `self.now_ns() − inp.x`.
//!   - An `external` consumer with a `drop_oldest` input of declared `depth =
//!     D` fires ONLY on `trigger_external` + `step` — so stepping WITHOUT a
//!     trigger advances the producer while the consumer is NOT stepped,
//!     overflowing the queue deterministically (the induced overflow).
//!     On a fire it records its data-age (in ms) into a shared trace.
//!
//! Sweep `K ∈ {D+1, 2D, 4D, 8D}` (and the `K == D` / never-stall edges).
//! ORACLE (hand-written, NOT a self-compare): recovery latency == 1 read FLAT across all `K`;
//! drop count == `K − D` exactly per `K`. The determinism test runs the whole
//! sweep TWICE and asserts byte-identity of (recovery, drops, data-ages) AND
//! against the hand oracle — two-run identity alone would be tautological.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// One `step()` advances the `VirtualClock` by this delta, so the producer
/// (period 1 ms) fires every step and the consumer's data-age is an exact
/// integer number of these ticks.
const TICK: Duration = Duration::from_millis(1);
const TICK_NS: u64 = 1_000_000;

/// The consumer's bounded `drop_oldest` queue depth `D`. Small + explicit so
/// the `K − D` arithmetic is hand-checkable.
const DEPTH: usize = 4;

/// Steady-state data-age, in `TICK`s, that a never-stalled consumer reads.
/// It is ONE inter-arrival (not 0): the data-trigger / body read drains the
/// bounded queue at the START of the consumer's step, so it serves the
/// producer's PRIOR-step publish (the same-step publish lands at the producer
/// level which the consumer level reads after — a single fixed inter-arrival
/// lag, NOT an overflow effect). The `never_stalled_consumer_*` edge test pins
/// this baseline empirically; the recovery criterion is "age back within one
/// inter-arrival of this baseline".
const BASELINE_AGE_MS: u64 = 1;

// ===========================================================================
// Nodes.
// ===========================================================================

/// Publishes one monotonically-timestamped `Vector3` per 1 ms tick. `out.x`
/// carries the publish time (`now_ns()`), which equals the wire `timestamp_ns`
/// the publisher stamps at the same instant — so the consumer can recover the
/// sample's age from the payload alone (the node body sees the deserialized
/// message, not the wire header).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct StampProducer {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl StampProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Publish time = wire timestamp by construction (publisher stamps
        // `clock.now_ns()` into the header at this same publish).
        self.out.x = self.now_ns() as f64;
        Ok(())
    }
}

/// Externally-triggered `drop_oldest` consumer (depth `D`). It fires ONLY when
/// the host calls `trigger_external` — so the harness controls exactly when it
/// drains, letting the queue overflow deterministically while it is "stalled".
/// On each fire it drains TO LATEST and records the observed data-age (ms).
#[cerulion_node(external)]
#[derive(Default)]
struct AgingConsumer {
    #[input(backpressure = drop_oldest, depth = 4)]
    inp: Vector3,
    /// Shared with the harness: every fire pushes the observed data-age in ms
    /// (consumer logical-now − sample publish time), so the harness can find
    /// the recovery transient.
    ages_ms: Arc<Mutex<Vec<u64>>>,
    /// Shared with the harness: number of fires (reads) so far.
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl AgingConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // `inp.x` is the surviving (latest) sample's publish time. Age in ns is
        // `now − publish`; convert to whole ms ticks (always exact: both clocks
        // advance by whole-ms `TICK`s).
        let publish_ns = self.inp.x as u64;
        let now = self.now_ns();
        let age_ns = now.saturating_sub(publish_ns);
        self.ages_ms.lock().unwrap().push(age_ns / TICK_NS);
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

// ===========================================================================
// Harness.
// ===========================================================================

/// Shared observability handles for one run.
struct Probe {
    ages_ms: Arc<Mutex<Vec<u64>>>,
    fires: Arc<AtomicU64>,
}

impl Probe {
    fn new() -> Self {
        Self {
            ages_ms: Arc::new(Mutex::new(Vec::new())),
            fires: Arc::new(AtomicU64::new(0)),
        }
    }
}

fn recovery_graph(probe: &Probe) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "overflow_recovery".to_string(),
        prefix: "ovr".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "stamp_producer".to_string(),
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
                node_type: "aging_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(StampProducerEntry::new()));
    let consumer = AgingConsumer {
        ages_ms: Arc::clone(&probe.ages_ms),
        fires: Arc::clone(&probe.fires),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(AgingConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Step the runtime once WITHOUT firing the consumer (a "stall" tick): the
/// producer publishes, the external consumer does not drain.
fn stall_tick(rt: &mut GraphRuntime) {
    rt.step(TICK);
}

/// Trigger + step: the external consumer fires this step (drains to latest).
fn fire_tick(rt: &mut GraphRuntime) {
    rt.trigger_external("consumer").expect("trigger consumer");
    rt.step(TICK);
}

/// Outcome of one overflow-recovery experiment for a single `K`.
#[derive(Debug, Clone, PartialEq, Eq)]
struct OverflowResult {
    /// The overflow magnitude requested.
    k: u64,
    /// Reads until the data-age returned to within one inter-arrival
    /// (`BASELINE_AGE_MS`) of the steady-state baseline. `drain_to_latest`
    /// makes this 1 — the resume read already serves the freshest sample.
    recovery_reads: u64,
    /// Evictions counted over the overflow window (`NodeHandle`
    /// `backpressure_drop_oldest_count`).
    drops: u64,
    /// The data-age (ms) of the FIRST post-overflow read — the freshness the
    /// recovery transient lands on. Under drain-to-latest it lands straight on
    /// the `BASELINE_AGE_MS` baseline (no overflow-sized age spike).
    first_post_overflow_age_ms: u64,
}

/// Run ONE overflow-recovery experiment for overflow magnitude `K`:
///   1. Baseline: fire the consumer a few times so it reads fresh
///      (`BASELINE_AGE_MS`) and the drop_oldest probe establishes its
///      per-stream baseline.
///   2. Overflow: step `K` ticks; the producer publishes `K` frames into the
///      `D`-deep queue while the consumer stays stalled. On the `K`-th tick we
///      ALSO fire the consumer (resume): between the last baseline drain and
///      this resume drain the producer published exactly `K` frames → `K − D`
///      evicted; drain-to-latest serves the newest → age == baseline →
///      recovery 1 read.
///
/// Returns the per-`K` outcome. The producer and consumer share ONE runtime so
/// the wire sequence + per-stream baseline carry across (a fresh graph per `K`
/// would re-baseline; one runtime per sweep is the realistic transient).
fn run_overflow_window(rt: &mut GraphRuntime, probe: &Probe, k: u64) -> OverflowResult {
    let drops_before = drop_count(rt);

    // (1) Baseline reads — fire-tick a few times so the consumer reads fresh
    // and the drop_oldest probe establishes its per-stream baseline.
    for _ in 0..3 {
        fire_tick(rt);
    }
    // Mark the read index AFTER baseline: the resume read is the NEXT recorded
    // age (indexed by length, robust to the one-step iceoryx2 connection lag on
    // the very first fire — never assume a fixed fire-count offset).
    let resume_idx = probe.ages_ms.lock().unwrap().len();

    // (2) Overflow: K stall ticks, firing the consumer on the LAST one so the
    // window between the prior baseline drain and this drain is exactly K
    // producer publishes (K − D evicted).
    debug_assert!(k >= 1, "overflow window needs at least one producer tick");
    for _ in 0..(k - 1) {
        stall_tick(rt);
    }
    fire_tick(rt); // resume: drain-to-latest on the K-th producer tick
                   // A couple more fresh reads so the recovery scan has room past the resume
                   // read (and so a HYPOTHETICAL multi-read recovery would be observable).
    for _ in 0..2 {
        fire_tick(rt);
    }

    let drops = drop_count(rt) - drops_before;
    let ages = probe.ages_ms.lock().unwrap();
    let first_post_overflow_age_ms = ages[resume_idx];

    // Recovery latency = reads until the age is back within one inter-arrival
    // (`BASELINE_AGE_MS`) of baseline. Count from the resume read forward.
    let mut recovery_reads = 0u64;
    for &age in &ages[resume_idx..] {
        recovery_reads += 1;
        if age <= BASELINE_AGE_MS {
            break;
        }
    }

    OverflowResult {
        k,
        recovery_reads,
        drops,
        first_post_overflow_age_ms,
    }
}

fn drop_count(rt: &GraphRuntime) -> u64 {
    rt.node_handle("consumer")
        .expect("consumer handle")
        .backpressure_drop_oldest_count("inp")
}

/// Run the full `K`-sweep on one fresh runtime; return the per-`K` results in
/// sweep order. (Each window's baseline phase re-freshens the consumer, so the
/// windows are independent transients on a shared, monotonically-advancing wire
/// stream.)
fn run_sweep(ks: &[u64]) -> Vec<OverflowResult> {
    let probe = Probe::new();
    let (config, factories) = recovery_graph(&probe);
    let clock = Arc::new(VirtualClock::new());
    let mut rt = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build overflow-recovery graph");
    // Warm-up: the very first `trigger_external` + `step` does not fire the
    // consumer (a one-step iceoryx2 connection-establishment lag on the body
    // input). Burn two fire-ticks so the connection is live and the consumer
    // is reading fresh BEFORE the first measured window's baseline phase.
    for _ in 0..2 {
        fire_tick(&mut rt);
    }
    ks.iter()
        .map(|&k| run_overflow_window(&mut rt, &probe, k))
        .collect()
}

/// The swept overflow magnitudes: minimal overflow, then 2×/4×/8× the depth.
const SWEEP_KS: [u64; 4] = [
    (DEPTH + 1) as u64,
    (2 * DEPTH) as u64,
    (4 * DEPTH) as u64,
    (8 * DEPTH) as u64,
];

/// HAND-WRITTEN ORACLE (NOT computed by re-running the system under test). For
/// each `K`: recovery is exactly 1 read (drain-to-latest), the first
/// post-overflow age lands straight on the `BASELINE_AGE_MS` baseline (no
/// overflow-sized age spike), and the drop count is exactly `K − D`. This is
/// the moat claim made concrete.
fn oracle(ks: &[u64]) -> Vec<OverflowResult> {
    ks.iter()
        .map(|&k| OverflowResult {
            k,
            recovery_reads: 1,
            drops: k - DEPTH as u64,
            first_post_overflow_age_ms: BASELINE_AGE_MS,
        })
        .collect()
}

/// Pretty-print the recovery/drop table under `--nocapture`.
fn print_table(title: &str, results: &[OverflowResult]) {
    println!("\n=== {title} (depth D = {DEPTH}) ===");
    println!(
        "{:>6} {:>10} {:>14} {:>16} {:>18}",
        "K", "K-D", "recovery_reads", "first_age_ms", "drops_counted"
    );
    for r in results {
        println!(
            "{:>6} {:>10} {:>14} {:>16} {:>18}",
            r.k,
            r.k - DEPTH as u64,
            r.recovery_reads,
            r.first_post_overflow_age_ms,
            r.drops
        );
    }
}

// ===========================================================================
// Tests.
// ===========================================================================

/// HAPPY PATH (the moat): across the K-sweep, recovery is FLAT 1 read and the
/// drop count is exactly `K − D`. Asserted against a HAND-WRITTEN oracle (not a
/// self-compare).
#[test]
#[serial]
fn drop_oldest_overflow_recovery_is_flat_one_read_across_k() {
    let results = run_sweep(&SWEEP_KS);
    print_table("overflow recovery K-sweep", &results);
    let expected = oracle(&SWEEP_KS);
    assert_eq!(
        results, expected,
        "recovery must be FLAT 1 read and drops must be EXACTLY K-D for every K \
         (drain-to-latest is O(1) in overflow magnitude — the moat). \
         got {results:?}, oracle {expected:?}"
    );

    // Redundant readability floors (subsumed by the oracle equality above):
    // recovery is independent of K (flat), and the drop count strictly grows
    // with K (so the harness really did induce larger overflows, not a no-op).
    let recoveries: Vec<u64> = results.iter().map(|r| r.recovery_reads).collect();
    assert!(
        recoveries.iter().all(|&r| r == 1),
        "recovery latency must be FLAT 1 across all K (got {recoveries:?})"
    );
    let drops: Vec<u64> = results.iter().map(|r| r.drops).collect();
    assert!(
        drops.windows(2).all(|w| w[1] > w[0]),
        "drop counts must strictly increase with K — proves the overflow scaled \
         (got {drops:?})"
    );
}

/// EDGE: minimal overflow `K = D + 1` drops EXACTLY 1, still recovers in 1
/// read.
#[test]
#[serial]
fn drop_oldest_minimal_overflow_drops_exactly_one() {
    let k = (DEPTH + 1) as u64;
    let results = run_sweep(&[k]);
    print_table("minimal overflow (K = D+1)", &results);
    assert_eq!(
        results[0],
        OverflowResult {
            k,
            recovery_reads: 1,
            drops: 1,
            first_post_overflow_age_ms: BASELINE_AGE_MS,
        },
        "K = D+1 must evict exactly one frame and recover in one read"
    );
}

/// EDGE: `K == D` exactly is NO overflow — the queue holds all D frames, zero
/// drops, and the consumer reads fresh throughout.
#[test]
#[serial]
fn no_overflow_when_k_equals_depth_zero_drops() {
    let k = DEPTH as u64;
    let results = run_sweep(&[k]);
    print_table("no overflow (K = D)", &results);
    assert_eq!(
        results[0],
        OverflowResult {
            k,
            recovery_reads: 1,
            drops: 0,
            first_post_overflow_age_ms: BASELINE_AGE_MS,
        },
        "K = D fills the queue exactly without eviction — zero drops, fresh read"
    );
}

/// EDGE: a consumer that NEVER stalls (fires every producer tick) drops
/// nothing and stays at baseline age throughout — the no-overload control.
#[test]
#[serial]
fn never_stalled_consumer_drops_nothing_and_stays_fresh() {
    let probe = Probe::new();
    let (config, factories) = recovery_graph(&probe);
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build no-stall graph");

    // Warm-up: absorb the one-step iceoryx2 connection lag on the first fire so
    // the measured fires all register (see `run_sweep`).
    for _ in 0..2 {
        fire_tick(&mut rt);
    }
    let fires_before = probe.fires.load(Ordering::Relaxed);
    let ages_before = probe.ages_ms.lock().unwrap().len();

    // Fire EVERY step — the consumer always drains to the latest sample.
    for _ in 0..20 {
        fire_tick(&mut rt);
    }

    let drops = drop_count(&rt);
    assert_eq!(
        drops, 0,
        "a never-stalled consumer evicts nothing (it drains every producer tick)"
    );
    let fires = probe.fires.load(Ordering::Relaxed) - fires_before;
    assert_eq!(
        fires, 20,
        "the warmed-up consumer fired on every one of the 20 triggered steps"
    );
    let ages = probe.ages_ms.lock().unwrap();
    let measured = &ages[ages_before..];
    assert!(
        measured.iter().all(|&a| a == BASELINE_AGE_MS),
        "a never-stalled consumer reads at the fixed baseline age \
         (BASELINE_AGE_MS = {BASELINE_AGE_MS}) every tick — no age drift, no \
         overflow spike (got {measured:?})"
    );
}

/// DETERMINISM: the whole sweep run TWICE is byte-identical (recovery counts +
/// drop counts + data-ages) AND matches the hand oracle. Byte-identity alone
/// would be tautological (two runs of the same closure); the oracle check is
/// what makes it non-vacuous (the self-compare anti-pattern). Determinism
/// holds because both the producer's publish time and the consumer's logical
/// now come from the SAME `VirtualClock`, and the eviction detector keys off
/// the wire `sequence`, never wall-clock (Principle #7).
#[test]
#[serial]
fn overflow_recovery_is_deterministic_and_matches_oracle() {
    let a = run_sweep(&SWEEP_KS);
    let b = run_sweep(&SWEEP_KS);
    print_table("determinism run A", &a);
    print_table("determinism run B", &b);
    assert_eq!(
        a, b,
        "two runs of the overflow-recovery sweep must be byte-identical \
         (VirtualClock + wire-sequence keying, no wall-clock — Principle #7). \
         a={a:?} b={b:?}"
    );
    // NON-tautological anchor: both runs must also equal the hand oracle.
    let expected = oracle(&SWEEP_KS);
    assert_eq!(
        a, expected,
        "the deterministic result must match the HAND-WRITTEN oracle, not just \
         itself — recovery 1, drops K-D, age == baseline. got {a:?}, oracle \
         {expected:?}"
    );
}
