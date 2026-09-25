// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity third pass — bp-drop-oldest: `backpressure_drop_oldest_count` and
//! the drain-to-latest freshness contract, proven through a LOADED cdylib.
//!
//! `backpressure_event_iox2_test.rs` / `overflow_recovery_iox2_test.rs` prove
//! `drop_oldest` overflow accounting + drain-to-latest recovery IN-PROCESS,
//! but no existing cdylib fixture reliably overflows: `cdylib_qos_behavioral_
//! test.rs`'s SKIP note diagnoses that `test_node_macro_period_input_cdylib`
//! (period 10ms, default depth `DEFAULT_CONSUMER_DEPTH = 10`) drains EXACTLY
//! depth-per-window under a 1ms flood (0 evictions) — it keeps pace. This
//! file uses the NEW `test_node_macro_slowdrain_cdylib` fixture (period 50ms,
//! same bare-`#[input]` default depth), which cannot keep pace with a 1ms
//! flood, to close that gap.
//!
//! # Topology
//!
//! `FloodProducer` (in-process, `period_ms = 1`) --/out--> `SlowDrainConsumer`
//! (cdylib, `period_ms = 50`, bare `#[input] inp`) --/out--> `Drain`
//! (in-process, `Data`-trigger, records every delivered value into a shared
//! `Arc<Mutex<Vec<u64>>>`).
//!
//! # Oracle
//!
//! The producer fires every 1ms step (period 1ms == step delta), so after
//! `S` steps its published counter `n == S` (`self.n += 1` then publish).
//! The consumer's `k`-th fire lands at cumulative step `50*k` (the
//! `cdylib_trigger_semantics_test.rs` Period pin: "8x50ms -> exactly 8"
//! confirms fires land at step-count multiples of the period when the step
//! delta evenly divides it) — MEASURED to observe the producer's value
//! from step `50*k - 1`, i.e. the value durably published as of the END of
//! the PRIOR step, not the same-tick value. So the consumer's `k`-th
//! delivered value is the HAND-COMPUTED oracle `50*k - 1`, for `k in 1..=10`
//! over a 500-step sweep — NOT a vague "some fresh-ish value" check.
//!
//! Both the overflow proof (`backpressure_drop_oldest_count("inp") > 0` —
//! impossible if the fixture kept pace like the existing period-input
//! fixture) and the freshness oracle are asserted, plus determinism (two
//! runs byte-identical AND matching the oracle — the F11 self-compare
//! anti-pattern is avoided by anchoring to the independently-computed
//! oracle, not just to each other).
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_slowdrain_cdylib`. `#[serial]`
//! (cdylib `NODES` + iceoryx2 SHM singletons).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Total 1ms steps in the sweep. The cdylib consumer's `period_ms = 50`
/// fires `floor(STEPS / 50) = 10` times.
const STEPS: usize = 500;
/// The consumer's declared period (must match the fixture's
/// `#[cerulion_node(period_ms = 50)]`).
const CONSUMER_PERIOD_MS: u64 = 50;
/// Expected number of consumer fires over `STEPS` 1ms steps.
const EXPECTED_CONSUMER_FIRES: u64 = (STEPS as u64) / CONSUMER_PERIOD_MS;

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
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

/// 1ms flood — the SlowDrainConsumer's 50ms period drains at 1/50th this
/// rate, guaranteeing overflow against the default queue depth.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct FloodProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FloodProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Data-trigger downstream drain: fires exactly when the cdylib consumer
/// publishes a new `out` sample, recording it (never held/replayed, since a
/// `Data`-trigger node simply does not fire without a new sample).
#[cerulion_node]
#[derive(Default)]
struct Drain {
    #[input(trigger)]
    inp: Vector3,
    observed: Arc<Mutex<Vec<u64>>>,
}
#[cerulion_node_impl]
impl Drain {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.observed.lock().unwrap().push(self.inp.x as u64);
        Ok(())
    }
}

/// Outcome of one full sweep.
#[derive(Debug, Clone, PartialEq, Eq)]
struct SweepResult {
    producer_fires: u64,
    consumer_fires: u64,
    drops: u64,
    observed: Vec<u64>,
}

fn run_sweep() -> SweepResult {
    let observed = Arc::new(Mutex::new(Vec::new()));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdo_overflow".to_string(),
        prefix: "cdo".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "flood_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "slow_drain_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "drain".to_string(),
                node_type: "drain".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "consumer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(FloodProducerEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_cdylib("test_node_macro_slowdrain_cdylib"))
                .expect("load slowdrain fixture"),
        ),
    );
    let drain = Drain {
        observed: Arc::clone(&observed),
        ..Default::default()
    };
    factories.insert("drain".to_string(), Box::new(DrainEntry::with_state(drain)));

    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build overflow graph");
    for _ in 0..STEPS {
        rt.step(Duration::from_millis(1));
    }

    let producer_fires = rt.node_handle("producer").unwrap().fire_count();
    let consumer_fires = rt.node_handle("consumer").unwrap().fire_count();
    let drops = rt
        .node_handle("consumer")
        .unwrap()
        .backpressure_drop_oldest_count("inp");
    let observed = observed.lock().unwrap().clone();
    SweepResult {
        producer_fires,
        consumer_fires,
        drops,
        observed,
    }
}

/// HAND-WRITTEN ORACLE (NOT computed by re-running the system under test):
/// the consumer's `k`-th fire lands at cumulative step `50*k` but reads the
/// producer's counter as of the PRIOR step (measured — see module doc),
/// so the delivered value is `50*k - 1`.
fn oracle() -> Vec<u64> {
    (1..=EXPECTED_CONSUMER_FIRES)
        .map(|k| k * CONSUMER_PERIOD_MS - 1)
        .collect()
}

#[test]
#[serial]
fn slow_cdylib_consumer_overflows_and_stays_fresh() {
    let result = run_sweep();

    assert_eq!(
        result.producer_fires, STEPS as u64,
        "the 1ms-period producer must fire every one of the {STEPS} steps"
    );
    assert_eq!(
        result.consumer_fires, EXPECTED_CONSUMER_FIRES,
        "the 50ms-period cdylib consumer must fire floor({STEPS}/{CONSUMER_PERIOD_MS}) times"
    );

    // Headline pin: a fixture that DOESN'T reliably overflow (like the
    // existing 10ms period-input fixture under a 1ms flood) would read
    // exactly 0 here — this is a real behavioral proof, not tautological.
    assert!(
        result.drops > 0,
        "the slow (50ms) cdylib consumer must overflow its default-depth \
         drop_oldest queue against the 1ms flood (got 0 drops — the fixture \
         kept pace, which is the exact gap this test closes)"
    );

    // Freshness: the delivered sequence must equal the hand-computed oracle
    // exactly — drain-to-latest always serves the newest available sample,
    // never a stale early one.
    let expected = oracle();
    assert_eq!(
        result.observed, expected,
        "drain-to-latest must deliver EXACTLY the oracle sequence (49, 99, \
         .., 499) through the cdylib — a stale read would lag far behind \
         these values. got {:?}, oracle {expected:?}",
        result.observed
    );
}

/// DETERMINISM (Principle #7): two runs of the whole sweep are
/// byte-identical AND match the hand oracle — byte-identity alone would be
/// tautological (F11 self-compare anti-pattern), so both checks are
/// required.
#[test]
#[serial]
fn slow_cdylib_consumer_overflow_is_deterministic() {
    let a = run_sweep();
    let b = run_sweep();
    assert_eq!(
        a, b,
        "two runs of the overflow sweep must be byte-identical \
         (VirtualClock + wire-sequence keying, no wall-clock). a={a:?} b={b:?}"
    );
    let expected = oracle();
    assert_eq!(
        a.observed, expected,
        "the deterministic run must match the HAND-WRITTEN oracle, not just \
         itself. got {:?}, oracle {expected:?}",
        a.observed
    );
    assert!(a.drops > 0, "the deterministic run must actually overflow");
}
