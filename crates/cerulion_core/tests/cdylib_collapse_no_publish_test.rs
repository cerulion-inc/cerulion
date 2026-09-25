// SPDX-License-Identifier: AGPL-3.0-only
//! Collapse-no-publish cdylib parity: the collapse-must-not-publish fix
//! (`cerulion_macros/src/impl_macro.rs`'s `build_nested_try_view` /
//! `rewrite_tick_method` `bool` discriminant) holds identically through
//! the `DylibNodeEntry` FFI boundary. The fix lives entirely inside
//! `__cer_zero_copy_tick`, which serves BOTH the in-process
//! `<Name>Entry::tick` AND the FFI `cerulion_node_tick` entry point (that
//! entry point calls `node.tick()` on the boxed entry) — so a bug fixed
//! only in-process (`collapse_no_publish_test.rs`) but not exercised
//! through the C ABI would be an inert-shipping gap (the cdylib-parity discipline).
//!
//! Reuses the existing `test_node_macro_period_input_cdylib` fixture
//! (`period_ms = 10`, plain non-trigger `#[input] inp`, fixed-schema
//! `#[output] out: Vector3`, body `out.x = inp.x`) — the exact shape
//! needed: a self-triggering (Period) node with a non-trigger input that
//! can legitimately be empty and a fixed-schema output with no
//! "was every field written" gate.
//!
//! # Observability
//!
//! Same technique as `collapse_no_publish_test.rs`: a downstream
//! in-process `#[input(trigger)]` sink data-triggers on the cdylib's
//! `out`, so a recorded increment is unambiguous proof of a real publish
//! (never a self-compare — the oracle is a hand-computed delivery count).
//!
//! # Running (iceoryx2 SHM singleton + cdylib `NODES` singleton → serial)
//!
//! ```bash
//! cargo build -p test_node_macro_period_input_cdylib
//! cargo test -p cerulion_core --test cdylib_collapse_no_publish_test -- --test-threads=1
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

/// The cdylib fixture is `period_ms = 10`; step the virtual clock by the
/// period so it fires on every step.
const STEP: Duration = Duration::from_millis(10);

/// Monotonic prefix counter so re-builds within one process never collide
/// on an iceoryx2 service name (mirrors cdylib_non_trigger_hold_test /
/// rayon_fire_cdylib_serial_test).
static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    format!("{stem}{}", PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed))
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

// ===========================================================================
// In-process producer + observing sink. Source code is truth; no fake
// data — every value is a real publish read in a real tick.
// ===========================================================================

/// External producer publishing a FIXED scalar into `out.x` on each fire.
/// The harness fires it (or not) to control exactly which steps deliver
/// to the cdylib's non-trigger input.
#[cerulion_node(external)]
#[derive(Default)]
struct FixedProducer {
    #[output]
    out: Vector3,
    val: f64,
}

#[cerulion_node_impl]
impl FixedProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.val;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// In-process observing sink: data-triggers on the cdylib's `out`.
/// `delivered` counts every fire (unambiguous "did a publish reach here"
/// oracle — the cdylib is opaque across the FFI, so this is the ONLY way
/// to observe whether it published); `last_value` records what it read.
#[cerulion_node]
#[derive(Default)]
struct DeliverySink {
    #[input(trigger)]
    inp: Vector3,
    delivered: Arc<AtomicU64>,
    last_value: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl DeliverySink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.delivered.fetch_add(1, Ordering::Relaxed);
        self.last_value.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// Cdylib fixture locator (verbatim pattern from cdylib_non_trigger_hold_test).
// ===========================================================================

fn find_period_input_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_input_cdylib")
}

// ===========================================================================
// Graph builder
// ===========================================================================

/// `producer/out` (external, HostDriven) -> `dut.inp` (the REAL cdylib
/// fixture, Period(10ms), plain non-trigger) -> `sink.inp` (DataTrigger,
/// observes `dut/out`).
fn build_cdylib_collapse_graph(
    prefix: &str,
    val: f64,
) -> (GraphRuntime, Arc<AtomicU64>, Arc<AtomicU64>) {
    let delivered = Arc::new(AtomicU64::new(0));
    let last_value = Arc::new(AtomicU64::new(u64::MAX));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_collapse_no_publish".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "fixed_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "dut".to_string(),
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
                node_type: "delivery_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "dut/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(FixedProducerEntry::with_state(FixedProducer {
            val,
            ..Default::default()
        })),
    );
    factories.insert(
        "dut".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_period_input_cdylib()).expect("load period+input cdylib"),
        ),
    );
    factories.insert(
        "sink".to_string(),
        Box::new(DeliverySinkEntry::with_state(DeliverySink {
            delivered: Arc::clone(&delivered),
            last_value: Arc::clone(&last_value),
            ..Default::default()
        })),
    );

    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build cdylib collapse graph");
    (runtime, delivered, last_value)
}

// ===========================================================================
// PIN (CDYLIB PARITY, HEADLINE): the loaded cdylib's non-trigger input is
// NEVER delivered — every Period(10ms) tick must collapse and therefore
// must NEVER publish, identically to the in-process case.
// ===========================================================================

#[test]
#[serial]
fn cdylib_pre_first_delivery_never_publishes() {
    const N: u32 = 30;
    let (mut rt, delivered, _last_value) =
        build_cdylib_collapse_graph(&unique_prefix("cdycollapse_never"), 123.0);

    // producer NEVER triggered — the cdylib's `inp` is NEVER delivered.
    for _ in 0..N {
        rt.step(STEP);
    }

    assert_eq!(
        delivered.load(Ordering::Relaxed),
        0,
        "collapse-no-publish cdylib parity: a loaded cdylib whose non-trigger input is \
         NEVER delivered must collapse EVERY tick and therefore must NEVER \
         arm its fixed-schema output's publish — the downstream \
         data-trigger sink must record ZERO deliveries across {N} steps \
         (hand oracle: 0). Before the collapse fix the collapsed tick still armed \
         publish through the FFI exactly like in-process, so this would \
         have been {N}."
    );
}

// ===========================================================================
// PIN (DETERMINISM): two independent runs of the headline cdylib scenario
// both pin to the hand oracle (0), not just cross-run equality.
// ===========================================================================

#[test]
#[serial]
fn cdylib_pre_first_delivery_never_publishes_is_deterministic() {
    const N: u32 = 20;

    let run = || {
        let (mut rt, delivered, _last_value) =
            build_cdylib_collapse_graph(&unique_prefix("cdycollapse_det"), 7.0);
        for _ in 0..N {
            rt.step(STEP);
        }
        delivered.load(Ordering::Relaxed)
    };

    let a = run();
    let b = run();

    assert_eq!(
        a, b,
        "two runs of the never-fed cdylib collapse must be byte-identical"
    );
    assert_eq!(
        a, 0,
        "AND both must equal the hand oracle (anti-tautology): a collapsed \
         cdylib tick never publishes, so delivered count is 0 on every run. \
         got {a}"
    );
}

// ===========================================================================
// PIN (REGRESSION GUARD, CDYLIB PARITY): once the cdylib's input is
// established and HELD (held-value replay — NOT a collapse, try_view returns
// Some every step), it must keep publishing on EVERY tick, completely
// unaffected by the fix.
// ===========================================================================

#[test]
#[serial]
fn cdylib_genuine_success_still_publishes_every_tick() {
    const V: u64 = 55;
    const HORIZON: u32 = 20;

    let (mut rt, delivered, last_value) =
        build_cdylib_collapse_graph(&unique_prefix("cdycollapse_held"), V as f64);

    // Establish: fire the producer ONCE, then step (bounded) until the
    // sink first observes V — producer and the cdylib share level 0 (the
    // `inp` edge is non-trigger), so the cdylib's snapshot reads the
    // producer's PRIOR-step publish (mirrors `establish_held` in
    // cdylib_non_trigger_hold_test.rs).
    rt.trigger_external("producer").expect("trigger producer");
    let mut tries = 0;
    loop {
        rt.step(STEP);
        if last_value.load(Ordering::Relaxed) == V {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "cdylib DUT never established the held value {V} within 200 steps"
        );
    }
    let established_delivered = delivered.load(Ordering::Relaxed);
    assert!(
        established_delivered >= 1,
        "at least one delivery once established"
    );

    // HORIZON more silent steps: producer never fires again, but the
    // cdylib is Period(10) and its input is now HELD (the hold replays the
    // last-delivered value every step) — genuinely runs EVERY tick, so it
    // must publish EVERY tick.
    for _ in 0..HORIZON {
        rt.step(STEP);
    }
    let total_delivered = delivered.load(Ordering::Relaxed);
    assert_eq!(
        total_delivered - established_delivered,
        u64::from(HORIZON),
        "the collapse fix must not affect the genuine-success path through the FFI: \
         a Period cdylib whose input is HELD runs its body EVERY \
         tick and must publish EVERY tick — expected exactly {HORIZON} \
         additional deliveries after establish (hand oracle), got {}",
        total_delivered - established_delivered
    );
    assert_eq!(
        last_value.load(Ordering::Relaxed),
        V,
        "every held-replay delivery must carry the established value {V}"
    );
}
