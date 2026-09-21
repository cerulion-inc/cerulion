// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-step-hold regression: a cdylib whose one-time
//! `cerulion_node_set_snapshot_inputs` FFI call FAILS must NOT keep calling the
//! per-step `cerulion_node_snapshot_inputs` FFI.
//!
//! # The bug this pins
//!
//! `DylibNodeEntry::snapshot_inputs` (`cerulion_core/src/graph/node.rs`) marshals
//! the build-fixed non-trigger input names to the cdylib exactly ONCE via
//! `set_snapshot_inputs`, then forwards the per-step freeze via `snapshot_inputs`
//! on every firing step. The fix folds the one-time `set` result into a
//! `SnapshotState` machine:
//!
//! ```text
//!   Pending --set ok (ret==0)--> Active   (snapshot called every step)
//!   Pending --set fail (ret!=0)-> Failed  (snapshot NEVER called — terminal)
//! ```
//!
//! `should_invoke_snapshot()` is true ONLY in `Active`. A host that latched
//! as if `set` had succeeded (effectively a hardcoded `Active`) would call
//! `snapshot` every step even after `set` failed — freezing nothing yet
//! returning 0 (success), SILENTLY masking the lost hold. The load-bearing
//! production line is
//! `self.snapshot_state = self.snapshot_state.after_set(ret);` (node.rs ~2537);
//! reverting it to a hardcoded `SnapshotState::Active` makes TEST A below fail.
//!
//! # How this test observes it (counter-based — no zero-copy I/O)
//!
//! The `test_node_snapshot_fail_cdylib` fixture is a raw-FFI cdylib declaring one
//! non-trigger input "inp" + a `period_ms` policy (so the runtime's per-fire
//! snapshot pass calls `snapshot_inputs(["inp"])` every step). It keeps a
//! PROCESS-GLOBAL `SNAPSHOT_CALLS` counter incremented on every
//! `cerulion_node_snapshot_inputs` invocation, and injects a `set` failure when
//! `CER_FAIL_MODE == "snapshot_set"`. This test loads the same .so through its
//! own `libloading` handle (a second `dlopen` of the same object shares the
//! static) to read/reset that counter.
//!
//! - **TEST A (fault):** `CER_FAIL_MODE=snapshot_set` → `set` fails → host goes
//!   `Failed` → `snapshot` is NEVER called → counter == 0. If the state were
//!   reverted to a hardcoded `Active`, `snapshot` WOULD be called every step →
//!   counter > 0 → this test fails. THE regression guard.
//! - **TEST B (control):** no fault → `set` succeeds → host calls `snapshot`
//!   every firing step → counter > 0. Proves the apparatus would DETECT a
//!   snapshot that stopped being called (anti-tautology for TEST A's `== 0`).
//!
//! Both assertions are against a HAND-WRITTEN oracle (0 vs > 0), not a
//! self-compare. Every snapshot call is a real FFI invocation driven by a real
//! `GraphRuntime::step()` over real iceoryx2 (`build_for_test`) — no fake data
//! (Principle #13).
//!
//! # Running (iceoryx2 SHM singleton + cdylib `NODES` / process-global env →
//! serial; build the fixture first so the .so exists)
//!
//! ```bash
//! cargo build -p test_node_snapshot_fail_cdylib
//! cargo test -p cerulion_core --test cdylib_snapshot_set_failure_test -- --test-threads=1
//! ```

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

/// Number of steps to run. >= 5 so a snapshot-every-step bug produces
/// a clearly-non-zero counter, and the control clearly exceeds 0.
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
/// the counter does not depend on delivery (the cdylib fires on its period
/// regardless), but firing it once makes the scenario a realistic
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
// Fixture locator + raw accessor reads (the counter lives in the .so, read via
// a second dlopen of the same path — the process-global static is shared).
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

fn reset_snapshot_calls(lib: &libloading::Library) {
    let f: libloading::Symbol<unsafe extern "C" fn()> = unsafe {
        lib.get(b"cerulion_test_reset_snapshot_calls\0")
            .expect("missing cerulion_test_reset_snapshot_calls symbol")
    };
    unsafe { f() };
}

fn get_snapshot_calls(lib: &libloading::Library) -> u64 {
    let f: libloading::Symbol<unsafe extern "C" fn(u64) -> u64> = unsafe {
        lib.get(b"cerulion_test_get_snapshot_calls\0")
            .expect("missing cerulion_test_get_snapshot_calls symbol")
    };
    // handle ignored by the fixture (counter is process-global).
    unsafe { f(0) }
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
/// observability is the cdylib's process-global snapshot-call counter.
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
                ros2: None,
                id: "producer".to_string(),
                node_type: "hold_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
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
/// (period), so its per-fire snapshot pass runs every step — that is what
/// drives the snapshot FFI (and the counter) regardless of producer activity.
fn run_once_then_silent(rt: &mut GraphRuntime) {
    rt.trigger_external("producer").expect("trigger producer");
    for _ in 0..STEPS {
        rt.step(STEP);
    }
}

// ===========================================================================
// TEST A (FAULT — the regression guard): set fails → snapshot NEVER called.
// ===========================================================================
#[test]
#[serial]
fn snapshot_not_called_after_set_failure() {
    let lib = load_accessor_handle();
    reset_snapshot_calls(&lib);

    // Inject the `set_snapshot_inputs` failure BEFORE any step (the first
    // firing step is when the host marshals the names via `set`).
    let _env = EnvVarGuard::set("CER_FAIL_MODE", "snapshot_set");

    let mut rt = build_graph(&unique_prefix("snapfail"));
    run_once_then_silent(&mut rt);

    let calls = get_snapshot_calls(&lib);
    assert_eq!(
        calls, 0,
        "set_snapshot_inputs failed → host must go terminal Failed and NEVER \
         call cerulion_node_snapshot_inputs, so the counter must be 0 after \
         {STEPS} firing steps. got {calls}. A non-zero count means the host kept \
         calling snapshot after a failed set (the silent-mask bug — \
         `snapshot_state = after_set(ret)` reverted to a hardcoded Active)."
    );
}

// ===========================================================================
// TEST B (CONTROL — anti-tautology): set succeeds → snapshot called each step.
// Proves the counter actually moves, so TEST A's `== 0` is meaningful (the
// apparatus WOULD detect a snapshot that stopped being called).
// ===========================================================================
#[test]
#[serial]
fn snapshot_is_called_each_step_when_set_succeeds() {
    let lib = load_accessor_handle();
    reset_snapshot_calls(&lib);

    // No fault. Belt-and-suspenders: ensure the env is clear even if a prior
    // test in this binary leaked it (EnvVarGuard already removes it on Drop,
    // and `#[serial]` serializes — but make the control hermetic).
    std::env::remove_var("CER_FAIL_MODE");

    let mut rt = build_graph(&unique_prefix("snapok"));
    run_once_then_silent(&mut rt);

    let calls = get_snapshot_calls(&lib);
    assert!(
        calls > 0,
        "set_snapshot_inputs succeeded → host must call \
         cerulion_node_snapshot_inputs on every firing step, so the counter must \
         be > 0 after {STEPS} firing steps (expected ~{STEPS}). got {calls}. A \
         zero count here means the snapshot pass never reached the cdylib — the \
         measurement apparatus is broken and TEST A's `== 0` would be vacuous."
    );
}
