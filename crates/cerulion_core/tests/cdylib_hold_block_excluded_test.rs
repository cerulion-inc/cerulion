// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity third pass — hold-block-excluded: the `block` non-trigger input
//! is EXCLUDED from the cross-step snapshot/hold, proven through the
//! LOADED `test_node_macro_holdblock_cdylib` fixture.
//!
//! `non_trigger_hold_iox2_test.rs`'s `block_non_trigger_reads_live_not_held`
//! proves IN-PROCESS that a `block` non-trigger input reads LIVE
//! (drains `Empty` on a silent step, i.e. does NOT replay the last delivered
//! value like `drop_oldest`/`sample` inputs do) — but that test observes the
//! held value by writing straight into a host-injected `Arc<AtomicU64>`
//! *inside* the node's `tick`, a channel a cdylib-loaded node cannot receive
//! (a `DylibNodeEntry` constructs its own state via the FFI; the only
//! channel across that boundary is the declared ports). `test_node_macro_
//! depth_cdylib` (the existing block-input cdylib fixture) has no output, so
//! it cannot carry this observation either — hence the new
//! `test_node_macro_holdblock_cdylib` fixture, which ports `BlockHoldConsumer`
//! 1:1 but adds an `#[output] out` that mirrors the block input's read.
//!
//! # The observable (measured; reflects the collapse-no-publish behaviour)
//!
//! Because the cdylib's state is opaque, "what did the block input read" can
//! only be observed HOST-SIDE by adding a downstream in-process `Data`-
//! trigger drain node wired to the cdylib's `out`.
//!
//! The `Empty`-collapse (the same
//! `try_view` no-op documented in `non_trigger_hold_iox2_test.rs`'s header)
//! suppresses `out`'s publish entirely on a silent step, so "no new
//! downstream delivery" is the pin. That rests on the collapse-no-publish rule:
//! without it the cdylib publishes a zero-init default frame on every collapsed step
//! (the output is already loaned before the `try_view` chain runs, and a
//! collapsed tick's `Ok(Ok(()))` is structurally identical to a genuine
//! success at the arm gate, so it arms Drop-publish). This fixture is what
//! makes that defect observable. (The in-process
//! `BlockHoldConsumer` has NO output port at all — nothing to loan or
//! publish — so it cannot surface the defect; adding an output for
//! host-observability is exactly what exposes it.)
//!
//! With the rule in place (a `bool` discriminant marks ran-vs-collapsed and only
//! `Ok(Ok(true))` arms publish), a collapsed
//! silent step publishes NOTHING, the drain's data trigger never fires, and
//! [`MISSING`] survives the per-step reset.
//!
//! The pin therefore is two-fold: the silent-step observations are NEVER the
//! held value `V` (which excludes replay — a wrongly-held `block` input would
//! keep delivering `V` forever) and are exactly [`MISSING`] (no delivery at
//! all — the no-fabrication contract; a collapsed tick that still
//! published would show the fabricated zero-init default `0` here).
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_holdblock_cdylib`. `#[serial]`
//! (cdylib `NODES` + iceoryx2 SHM singletons).

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

/// A sentinel the producer never publishes (it publishes a fixed `V`).
/// Recorded ONLY when the downstream drain's tick did NOT run — i.e. no new
/// sample reached it (mirrors `non_trigger_hold_iox2_test.rs`'s `MISSING`).
const MISSING: u64 = u64::MAX;

/// Step delta. Both `producer` and `consumer` are `external` — they fire on
/// `trigger_external` regardless of the clock — so the delta is immaterial.
const STEP: Duration = Duration::from_millis(1);

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

/// External producer that publishes a FIXED value on demand — never
/// republished automatically (crib: `non_trigger_hold_iox2_test.rs::
/// HoldProducer`).
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

/// Downstream `Data`-trigger drain: fires ONLY when the cdylib publishes a
/// genuinely new `out` sample (a trigger input cannot "replay" — there is no
/// held value for it), recording it into a shared sentinel.
#[cerulion_node]
#[derive(Default)]
struct Drain {
    #[input(trigger)]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl Drain {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

fn build_graph(val: f64) -> (GraphRuntime, Arc<AtomicU64>) {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_hold_block".to_string(),
        prefix: "chb".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "hold_block_probe".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
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
    factories.insert(
        "producer".to_string(),
        Box::new(HoldProducerEntry::with_state(HoldProducer {
            val,
            ..Default::default()
        })),
    );
    factories.insert(
        "consumer".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_cdylib("test_node_macro_holdblock_cdylib"))
                .expect("load holdblock fixture"),
        ),
    );
    let drain = Drain {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    factories.insert("drain".to_string(), Box::new(DrainEntry::with_state(drain)));

    let clock = Arc::new(VirtualClock::new());
    let rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build hold-block graph");
    (rt, last_read)
}

#[test]
#[serial]
fn dylib_block_non_trigger_reads_live_not_held() {
    const V: u64 = 33;
    let (mut rt, last_read) = build_graph(V as f64);

    // Establish: publish V ONCE, then trigger the cdylib consumer until the
    // downstream drain observes it (bounded — tolerates the two-hop
    // connection-establishment lag: producer->cdylib AND cdylib->drain).
    rt.trigger_external("producer").expect("trigger producer");
    rt.step(STEP);
    let mut tries = 0;
    loop {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        if last_read.load(Ordering::Relaxed) == V {
            break;
        }
        tries += 1;
        assert!(
            tries < 200,
            "downstream drain never observed the live value {V} through the cdylib"
        );
    }

    // Silent steps: the producer is NEVER triggered again (no new upstream
    // data). A HELD `inp` would let the cdylib keep republishing V=33
    // forever. Instead (see the module
    // doc "The observable" section) the try_view Empty-collapse skips the
    // user body AND withholds publish (only a genuinely-ran tick's
    // `Ok(Ok(true))` arms the output), so the downstream drain sees NO
    // delivery at all on a silent step — MISSING survives the per-step
    // reset. Both halves are pinned below: live-read (never the held V)
    // and no-fabrication (never a zero-init default frame either).
    let mut observed = Vec::with_capacity(5);
    for _ in 0..5 {
        last_read.store(MISSING, Ordering::Relaxed);
        rt.trigger_external("consumer").expect("trigger consumer");
        rt.step(STEP);
        observed.push(last_read.load(Ordering::Relaxed));
    }
    assert!(
        observed.iter().all(|&v| v != V),
        "a block non-trigger input on the LOADED cdylib must NOT replay the \
         held value {V} on a silent step (got {observed:?})"
    );
    assert_eq!(
        observed,
        vec![MISSING; 5],
        "a collapsed silent step must NOT publish at all — the \
         drain must record MISSING (no delivery) on every silent step (hand \
         oracle), never a fabricated zero-init default frame nor the held \
         value — got {observed:?}"
    );
}
