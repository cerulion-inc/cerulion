// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity det-clock-accessors: `self.now_ns()` / `self.virt_ns()` /
//! `self.ext_ns()` are macro-generated shims (`gen_shim_methods` in
//! `cerulion_macros/src/codegen.rs`) that dispatch through an
//! `Arc<dyn Clock>` cloned from the host `NodeContext` — a TRAIT-OBJECT
//! VTABLE POINTER crossing the cdylib FFI boundary, exactly the hazard
//! class the maintainers recorded as a confirmed architectural decision
//! (`AnyPublisher`/`AnySubscriber` are kept as enums rather than
//! `Arc<dyn Trait>` for this reason) and in their
//! FFI struct-layout notes. `real_ns()` (a free
//! function, no clock-trait dispatch) and `request_shutdown()` already have
//! cdylib coverage (`cdylib_qos_ffi_test.rs` et al.); `now_ns()`/`virt_ns()`/
//! `ext_ns()` had ZERO cdylib coverage before this file.
//!
//! Crib: `cerulion_core/tests/macro_shim_clock_sources_test.rs` proves the
//! SAME contract for an IN-PROCESS (`ClosureNodeEntry`) macro node — under a
//! `VirtualClock` `build_for_test` runtime `self.now_ns()` tracks the active
//! clock and `self.virt_ns()`/`self.ext_ns()` are `Some`/`None` accordingly;
//! under a `build_live` runtime backed by an `ExternalClock`, `self.ext_ns()`
//! latches the external master time and equals `self.now_ns()`.
//!
//! Three tests, all driving the REAL `test_node_macro_clockprobe_cdylib`
//! fixture (period_ms=5, stamps `now_ns`/`virt_ns`/`ext_ns` into a `Twist`
//! output's `linear`/`angular` fields) through `DylibNodeEntry`:
//!
//! - **A (VirtualClock / `build_for_test`)**: `virt_ns()` is `Some` and
//!   equals `now_ns()` (both read the SAME active clock); `ext_ns()` is
//!   `None`; `now_ns()` advances in exact `period_ms`-sized steps — a hand
//!   oracle (exact expected ns values), not a self-compare.
//! - **B (ExternalClock / `build_live`)**: `ext_ns()` is `Some` and equals
//!   `now_ns()`; `virt_ns()` is `None`.
//! - **C (parity)**: an in-process twin with byte-identical tick logic, run
//!   through the SAME VirtualClock scenario as A, produces a byte-identical
//!   observed sequence — the direct proof that the FFI `Arc<dyn Clock>`
//!   vtable dispatch resolves exactly like the in-process call (a stale or
//!   mismatched vtable pointer would diverge or crash here).
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_clockprobe_cdylib`. All tests
//! `#[serial]` (cdylib `NODES` singleton + iceoryx2 SHM singleton; B also
//! builds a live WaitSet).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::{Clock, ExternalClock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Twist;
use serial_test::serial;

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

fn clock_probe_cdylib() -> Box<dyn NodeEntry> {
    Box::new(
        DylibNodeEntry::load(&find_cdylib("test_node_macro_clockprobe_cdylib"))
            .expect("load clockprobe fixture"),
    )
}

/// One observed row: `(now_ns, virt_present, virt_ns, ext_present, ext_ns)`
/// — all as `f64`, mirroring the fixture's `Twist.linear`/`angular`
/// stamping (`virt_present`/`ext_present` are 1.0/0.0 flags).
type ClockRow = (f64, f64, f64, f64, f64);

/// Data-triggered drain: records every delivered `Twist` into a shared Vec.
#[cerulion_node]
#[derive(Default)]
struct ClockDrain {
    #[input(trigger)]
    inp: Twist,
    captured: Arc<Mutex<Vec<ClockRow>>>,
}

#[cerulion_node_impl]
impl ClockDrain {
    fn tick(&mut self) -> Result<(), NodeError> {
        let row = (
            self.inp.linear.x,
            self.inp.linear.y,
            self.inp.linear.z,
            self.inp.angular.x,
            self.inp.angular.y,
        );
        self.captured.lock().unwrap().push(row);
        Ok(())
    }
}

/// In-process twin of the cdylib fixture's tick — byte-identical logic,
/// used ONLY by the parity test (C) to prove the FFI path matches the
/// in-process path.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct ClockProbeTwin {
    #[output]
    out: Twist,
}

#[cerulion_node_impl]
impl ClockProbeTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.now_ns();
        self.out.linear.x = now as f64;

        let virt = self.virt_ns();
        self.out.linear.y = if virt.is_some() { 1.0 } else { 0.0 };
        self.out.linear.z = virt.unwrap_or(0) as f64;

        let ext = self.ext_ns();
        self.out.angular.x = if ext.is_some() { 1.0 } else { 0.0 };
        self.out.angular.y = ext.unwrap_or(0) as f64;
        Ok(())
    }
}

fn clockprobe_graph(
    prefix: &str,
    probe: Box<dyn NodeEntry>,
    captured: Arc<Mutex<Vec<ClockRow>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("clockprobe_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "probe".to_string(),
                node_type: "clock_probe".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Twist".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "drain".to_string(),
                node_type: "clock_drain".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "probe/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("probe".to_string(), probe);
    let drain = ClockDrain {
        captured,
        ..Default::default()
    };
    factories.insert(
        "drain".to_string(),
        Box::new(ClockDrainEntry::with_state(drain)),
    );
    (config, factories)
}

/// Build + step a `period_ms=5` probe through a `VirtualClock`
/// `build_for_test` runtime for `steps` steps of 5ms each, returning every
/// row the drain observed.
fn run_virtualclock_scenario(
    probe: Box<dyn NodeEntry>,
    prefix: &str,
    steps: usize,
) -> Vec<ClockRow> {
    let captured: Arc<Mutex<Vec<ClockRow>>> = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = clockprobe_graph(prefix, probe, Arc::clone(&captured));
    let clock = Arc::new(VirtualClock::new());
    let mut rt =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build clockprobe graph");
    for _ in 0..steps {
        rt.step(Duration::from_millis(5));
    }
    let result = captured.lock().unwrap().clone();
    result
}

// ===========================================================================
// Test A — VirtualClock arm
// ===========================================================================

#[test]
#[serial]
fn virtualclock_arm_virt_present_ext_absent_and_now_advances_exactly() {
    let rows = run_virtualclock_scenario(clock_probe_cdylib(), "cpva", 20);
    assert_eq!(
        rows.len(),
        20,
        "the period_ms=5 cdylib probe must fire exactly 20 times over 20 steps"
    );
    for (i, &(now, virt_present, virt_val, ext_present, _ext_val)) in rows.iter().enumerate() {
        let expected_now = ((i as u64 + 1) * 5_000_000) as f64;
        assert_eq!(
            now, expected_now,
            "tick {i}: now_ns() must advance in exact 5ms steps under VirtualClock"
        );
        assert_eq!(
            virt_present, 1.0,
            "tick {i}: virt_ns() must be Some under a VirtualClock runtime"
        );
        assert_eq!(
            virt_val, now,
            "tick {i}: virt_ns() must equal now_ns() under VirtualClock (same active clock)"
        );
        assert_eq!(
            ext_present, 0.0,
            "tick {i}: ext_ns() must be None under a VirtualClock runtime (not external)"
        );
    }
}

// ===========================================================================
// Test B — ExternalClock arm (build_live)
// ===========================================================================

#[test]
#[serial]
fn externalclock_arm_ext_present_virt_absent_and_ext_equals_now() {
    let captured: Arc<Mutex<Vec<ClockRow>>> = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = clockprobe_graph("cpea", clock_probe_cdylib(), Arc::clone(&captured));

    let ext = Arc::new(ExternalClock::new());
    let clock_dyn: Arc<dyn Clock> = ext.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "clockprobe_ext_test".into(),
            clock: clock_dyn.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let mut runtime =
        GraphRuntime::build_live(config, factories, &mgr, clock_dyn).expect("build_live");

    let steps = 5usize;
    for i in 0..steps {
        let t = (i as u64 + 1) * 5_000_000;
        ext.set_external(t);
        runtime.step(Duration::from_millis(5));
    }

    let rows = captured.lock().unwrap().clone();
    assert_eq!(
        rows.len(),
        steps,
        "the period_ms=5 cdylib probe must fire once per step under ExternalClock"
    );
    for (i, &(now, virt_present, _virt_val, ext_present, ext_val)) in rows.iter().enumerate() {
        let expected = ((i as u64 + 1) * 5_000_000) as f64;
        assert_eq!(
            now, expected,
            "tick {i}: now_ns() must equal the latched external master time"
        );
        assert_eq!(
            ext_present, 1.0,
            "tick {i}: ext_ns() must be Some under an ExternalClock runtime"
        );
        assert_eq!(
            ext_val, now,
            "tick {i}: ext_ns() must equal now_ns() (ExternalClock IS the active source)"
        );
        assert_eq!(
            virt_present, 0.0,
            "tick {i}: virt_ns() must be None under an ExternalClock runtime (not virtual)"
        );
    }
}

// ===========================================================================
// Test C — parity: cdylib FFI dispatch vs in-process twin
// ===========================================================================

#[test]
#[serial]
fn cdylib_and_in_process_twin_produce_identical_clock_reads() {
    let cdylib_rows = run_virtualclock_scenario(clock_probe_cdylib(), "cppa", 20);
    let twin_rows = run_virtualclock_scenario(Box::new(ClockProbeTwinEntry::new()), "cppb", 20);
    assert_eq!(cdylib_rows.len(), 20, "the cdylib probe must fire 20 times");
    assert_eq!(
        twin_rows.len(),
        20,
        "the in-process twin must fire 20 times"
    );
    assert_eq!(
        cdylib_rows, twin_rows,
        "the FFI-dispatched Arc<dyn Clock> reads (now_ns/virt_ns/ext_ns) over the cdylib \
         boundary must be byte-identical to the in-process macro shim calls — a diverging \
         or crashing result here would indicate the trait-object vtable pointer does not \
         resolve correctly across the FFI boundary"
    );
}
