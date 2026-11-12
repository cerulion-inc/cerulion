// SPDX-License-Identifier: AGPL-3.0-only
//! Host-owned non-trigger snapshots preserve actual downstream held values.
//! The node reads the real producer frame and holds 42 through silent steps.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use std::sync::Arc;

/// The fixture is `period_ms = 10`; step the virtual clock by the period so the
/// node fires (and the snapshot pass runs) on every step.
const STEP: Duration = Duration::from_millis(10);

/// Eight silent steps verify the node keeps the delivered latest value.
const STEPS: usize = 8;

/// Monotonic prefix counter so re-builds within one process never collide on an
/// iceoryx2 service name.
static PREFIX_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(stem: &str) -> String {
    format!("{stem}{}", PREFIX_COUNTER.fetch_add(1, Ordering::Relaxed))
}

/// RAII guard that removes a process-global env var on Drop — panic-safe cleanup
/// even if an assertion fires mid-test (`#[serial]` serializes the bodies but
/// does NOT reset env between them). Mirrors `chunk_c_ffi_codes_3_4_test.rs`.
struct EnvVarGuard {
    name: &'static str,
}

impl EnvVarGuard {
    fn set(name: &'static str, value: &str) -> Self {
        std::env::set_var(name, value);
        Self { name }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.name);
    }
}

// ===========================================================================
// In-process producer (source code is truth; no fake data). External so the
// harness controls exactly when it publishes.
// ===========================================================================

/// External producer publishing a fixed scalar into `out.x` on each fire. Wired
/// into the cdylib's non-trigger "inp" so the input has a real graph producer;
/// the consumer reads that real frame on its period, then retains it through
/// "data flowed, then went silent".
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

// ===========================================================================
// Fixture locator and real payload observations through the same library.
// ===========================================================================

fn find_snapshot_fail_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_snapshot_fail_cdylib")
}

/// Load the fixture .so through our OWN `libloading` handle so we can call its
/// test accessors. A second `dlopen` of the same path shares the .so's data
/// segment (mapped once, reference-counted), so this handle's view of
/// `SNAPSHOT_CALLS` IS the same static the runtime's `DylibNodeEntry`
/// increments.
fn load_accessor_handle() -> libloading::Library {
    unsafe { libloading::Library::new(find_snapshot_fail_cdylib()) }
        .expect("load snapshot-fail cdylib for test accessors")
}

// ===========================================================================
// Graph builder + driver
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

/// `producer/out` -> cdy.inp (non-trigger, period cdylib). The cdylib is the
/// REAL `test_node_snapshot_fail_cdylib` `DylibNodeEntry`. No downstream sink —
/// observability is the x value actually read inside the cdylib tick.
fn build_graph(prefix: &str) -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_snapshot_set_failure".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "producer".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "cdy".to_string(),
                node_type: "snapshot_fail".to_string(),
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
        Box::new(HoldProducerEntry::with_state(HoldProducer {
            val: 42.0,
            ..Default::default()
        })),
    );
    factories.insert(
        "cdy".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_snapshot_fail_cdylib()).expect("load snapshot-fail cdylib"),
        ),
    );

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build snapshot-fail graph")
}

/// Fire the producer once (data flows into the held input), then run `STEPS`
/// SILENT steps (producer not re-triggered). The cdylib fires every step
/// (period), so its host snapshot pass keeps the original latest value.
fn run_once_then_silent(rt: &mut GraphRuntime) {
    rt.trigger_external("producer").expect("trigger producer");
    for _ in 0..STEPS {
        rt.step(STEP);
    }
}

fn observed_input(lib: &libloading::Library) -> u64 {
    // SAFETY: built fixture exports this exact scalar signature; library stays live.
    unsafe {
        lib.get::<unsafe extern "C" fn() -> u64>(b"cerulion_test_observed_input\0")
            .expect("observation")()
    }
}
#[test]
#[serial]
fn host_snapshots_deliver_and_hold_input_with_obsolete_fault_environment() {
    let lib = load_accessor_handle();
    let _env = EnvVarGuard::set("CER_FAIL_MODE", "snapshot_set");
    let mut rt = build_graph(&unique_prefix("snapfail"));
    run_once_then_silent(&mut rt);
    assert_eq!(
        observed_input(&lib),
        42,
        "published input is still held and read after silent steps"
    );
}
#[test]
#[serial]
fn host_snapshot_delivery_is_identical_with_and_without_legacy_fault_environment() {
    let lib = load_accessor_handle();
    for _ in 0..2 {
        std::env::remove_var("CER_FAIL_MODE");
        let mut rt = build_graph(&unique_prefix("snapok"));
        run_once_then_silent(&mut rt);
        assert_eq!(observed_input(&lib), 42);
    }
}
