// SPDX-License-Identifier: AGPL-3.0-only
//! Host-owned Sync selects the real aligned pair while obsolete foreign head-op
//! traps stay inert. Native failure-position policy remains pinned by sync matcher tests.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const TOPIC_A: &str = "/sof/a";
const TOPIC_B: &str = "/sof/b";
const MS: u64 = 1_000_000;
const FAIL_KNOB: &str = "CER_FAIL_MODE";

/// RAII so a panicking arm cannot leave the knob set for its sibling.
struct EnvVarGuard;
impl EnvVarGuard {
    fn set(value: &str) -> Self {
        std::env::set_var(FAIL_KNOB, value);
        Self
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(FAIL_KNOB);
    }
}

/// What the fixture's process-global statics saw.
struct Probe {
    fires: u64,
    a: f64,
    b: f64,
}

/// Read (and reset) the fixture's counters through a SECOND `dlopen` of the
/// same path. `dlopen` refcounts, so the host's handle and this one share the
/// statics — the same trick `cdylib_snapshot_set_failure_test` uses.
fn probe(reset: bool) -> Probe {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_sync_op_fail_cdylib");
    // SAFETY: the fixture is built by this workspace; the two symbols are its
    // own `#[no_mangle] extern "C"` exports with exactly these signatures.
    unsafe {
        let lib = libloading::Library::new(&path).expect("open the fixture for probing");
        if reset {
            let f: libloading::Symbol<unsafe extern "C" fn()> =
                lib.get(b"sync_probe_reset\0").expect("reset symbol");
            f();
            return Probe {
                fires: 0,
                a: 0.0,
                b: 0.0,
            };
        }
        let f: libloading::Symbol<unsafe extern "C" fn(*mut u64, *mut u64, *mut u64)> =
            lib.get(b"sync_probe\0").expect("probe symbol");
        let (mut fires, mut a_bits, mut b_bits) = (0u64, 0, 0);
        f(&mut fires, &mut a_bits, &mut b_bits);
        Probe {
            fires,
            a: f64::from_bits(a_bits),
            b: f64::from_bits(b_bits),
        }
    }
}

fn graph(prefix: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_sync_op_fail_cdylib");
    let entry = DylibNodeEntry::load(&path).expect("load the sync-op-fail fixture");
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "sof".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "fuse".to_string(),
            node_type: "sync_op_fail".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: TOPIC_A.to_string(),
                    ..Default::default()
                },
                InputDef {
                    name: "b".to_string(),
                    source: TOPIC_B.to_string(),
                    ..Default::default()
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(entry));
    (config, factories)
}

/// A@0, A@20, B@20 — the descent shape. `b` holds no second frame, so on real
/// evidence the gate PASSES and the walk advances `a` from 0 to 20.
fn drive(prefix: &str) -> GraphRuntime {
    let (config, factories) = graph(prefix);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build the cdylib sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");
    for (p, ts, x) in [(0u8, 0u64, 0.0f64), (0, 20 * MS, 20.0), (1, 20 * MS, 20.0)] {
        clock.set(ts);
        let publisher = if p == 0 { &mut pub_a } else { &mut pub_b };
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
        proxy.x = x;
    }
    clock.set(0);
    for _ in 0..6 {
        runtime.step(Duration::from_millis(1));
    }
    runtime
}

#[test]
#[serial]
fn host_sync_selects_and_delivers_the_aligned_pair_without_foreign_head_ops() {
    std::env::remove_var(FAIL_KNOB);
    probe(true);
    let runtime = drive("sofok");
    let p = probe(false);
    assert_eq!(p.fires, 1);
    assert_eq!(
        (p.a, p.b),
        (20.0, 20.0),
        "both selected frames are read by the DLL body"
    );
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("handle")
            .sync_closer_skip_count(TOPIC_A),
        1
    );
}
#[test]
#[serial]
fn obsolete_foreign_sync_failure_cannot_change_native_selection_or_delivery() {
    let _guard = EnvVarGuard::set("sync_op_fail");
    for prefix in ["soffail_a", "soffail_b"] {
        probe(true);
        let runtime = drive(prefix);
        let p = probe(false);
        assert_eq!(p.fires, 1);
        assert_eq!((p.a, p.b), (20.0, 20.0));
        assert_eq!(
            runtime
                .node_handle("fuse")
                .expect("handle")
                .sync_closer_skip_count(TOPIC_A),
            1
        );
    }
}
