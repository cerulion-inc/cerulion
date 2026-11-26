// SPDX-License-Identifier: AGPL-3.0-only
//! Integration tests for build-time sync window period warnings and the
//! reworded runtime UNMATCHABLE discard warning.
//!
//! Uses `GraphRuntime::build_for_test` (per-test isolated SHM root; does not
//! touch the process-global `TransportManager` singleton), so every test here
//! is parallel-safe.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeContext, NodeEntry, NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use tracing_test::traced_test;

const MS: u64 = 1_000_000;

fn meta_input(name: &str, trigger: bool) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: 0xC425,
        trigger,
        depth: 4,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
        accumulate: false,
        consume: Default::default(),
    }
}

fn make_entry(info: NodeInfo) -> Box<dyn NodeEntry> {
    Box::new(ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_label("swp"))
}

fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

fn wired(name: &str, source: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: source.to_string(),
        ..Default::default()
    }
}

fn two_producer_sync_graph(prefix: &str) -> GraphConfig {
    GraphConfig {
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: format!("{prefix}_graph"),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "cam".to_string(),
                node_type: "cam".to_string(),
                inputs: vec![],
                outputs: vec![out_def("image")],
            },
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "imu".to_string(),
                node_type: "imu".to_string(),
                inputs: vec![],
                outputs: vec![out_def("sample")],
            },
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "fuse".to_string(),
                node_type: "fuse".to_string(),
                inputs: vec![wired("image", "cam/image"), wired("imu", "imu/sample")],
                outputs: vec![],
            },
        ],
    }
}

fn build_pair(
    prefix: &str,
    cam_period_ms: u64,
    imu_period_ms: u64,
    fuse_policy: MacroPolicy,
) -> cerulion_core::TransportResult<GraphRuntime> {
    let config = two_producer_sync_graph(prefix);
    let cam_info =
        NodeInfo::from_names(vec![], vec!["image".to_string()]).with_policy(MacroPolicy::Period {
            period_ms: cam_period_ms,
        });
    let imu_info =
        NodeInfo::from_names(vec![], vec!["sample".to_string()]).with_policy(MacroPolicy::Period {
            period_ms: imu_period_ms,
        });
    let fuse_info = NodeInfo::with_meta(
        vec![meta_input("image", true), meta_input("imu", true)],
        Vec::new(),
    )
    .with_policy(fuse_policy);

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("cam".to_string(), make_entry(cam_info));
    factories.insert("imu".to_string(), make_entry(imu_info));
    factories.insert("fuse".to_string(), make_entry(fuse_info));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8)
}

#[test]
#[traced_test]
fn build_warns_once_when_sync_window_reaches_declared_input_period() {
    let runtime = build_pair("swp_hit", 33, 5, MacroPolicy::Sync { window_ms: 33 })
        .expect("window >= period is a WARN, never a build refusal");
    drop(runtime);

    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| {
                l.contains("WARN")
                    && l.contains(
                        "sync_window_ms is at or above the slowest resolved trigger-input period",
                    )
            })
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "expected exactly one build-time sync window WARN, got {}: {hits:?}",
                hits.len()
            ));
        }
        let line = hits[0];
        for needle in [
            "node_id=fuse",
            "input=image",
            "period_ms=33",
            "sync_window_ms=33",
            "input_period - skew",
        ] {
            if !line.contains(needle) {
                return Err(format!("missing {needle:?} in warn line: {line}"));
            }
        }
        Ok(())
    });
}

#[test]
#[traced_test]
fn build_silent_when_sync_window_is_strictly_below_slowest_input_period() {
    let runtime = build_pair("swp_ok", 33, 5, MacroPolicy::Sync { window_ms: 15 })
        .expect("safe window must build cleanly");
    drop(runtime);

    logs_assert(|lines: &[&str]| {
        let hits = lines
            .iter()
            .filter(|l| {
                l.contains("WARN")
                    && (l.contains("sync_window_ms is at or above")
                        || l.contains("unbounded_sync aligns periodic"))
            })
            .count();
        if hits != 0 {
            return Err(format!(
                "expected zero sync-window period warnings for window 15 < period 33, got {hits}"
            ));
        }
        Ok(())
    });
}

#[test]
#[traced_test]
fn build_warns_once_for_unbounded_sync_on_periodic_inputs() {
    let runtime = build_pair("swp_unb", 33, 33, MacroPolicy::UnboundedSync)
        .expect("unbounded_sync on periodic inputs is a WARN, never a build refusal");
    drop(runtime);

    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| {
                l.contains("WARN") && l.contains("unbounded_sync aligns periodic trigger inputs")
            })
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "expected exactly one unbounded_sync period WARN, got {}: {hits:?}",
                hits.len()
            ));
        }
        let line = hits[0];
        for needle in [
            "node_id=fuse",
            "input=image",
            "period_ms=33",
            "policy=\"unbounded_sync\"",
        ] {
            if !line.contains(needle) {
                return Err(format!("missing {needle:?} in warn line: {line}"));
            }
        }
        Ok(())
    });
}

type Pairs = Arc<Mutex<Vec<(f64, f64)>>>;

#[cerulion_node(sync_window_ms = 20)]
#[derive(Default)]
struct BoundedPairFuse {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    pairs: Option<Pairs>,
}

#[cerulion_node_impl]
impl BoundedPairFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        if let Some(sink) = &self.pairs {
            sink.lock().expect("pairs lock").push((self.a.x, self.b.x));
        }
        Ok(())
    }
}

#[test]
#[traced_test]
fn unmatched_discard_warn_states_period_bound_not_unconditional_widen() {
    let pairs: Pairs = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(BoundedPairFuseEntry::with_state(BoundedPairFuse {
        pairs: Some(Arc::clone(&pairs)),
        ..Default::default()
    }));
    let config = GraphConfig {
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "swp_unmatched".to_string(),
        prefix: "swpu".to_string(),
        nodes: vec![NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "fuse".to_string(),
            node_type: "fuse".to_string(),
            inputs: vec![wired("a", "/swpu/a"), wired("b", "/swpu/b")],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build bounded pair fuse");

    let mgr = runtime.test_transport().expect("test transport");
    let mut pub_a = mgr
        .create_publisher("/swpu/a", MaxSliceLen::const_new(64), 0)
        .expect("pub a");
    let mut pub_b = mgr
        .create_publisher("/swpu/b", MaxSliceLen::const_new(64), 0)
        .expect("pub b");

    // Publish orphan a@0 and partner b@100 (span 100 ms > 20 ms window) so the
    // scheduler discards a@0 as UNMATCHABLE and emits the loud regime WARN.
    clock.set(0);
    {
        let mut loan = pub_a.loan_proxy::<Vector3>().expect("loan a");
        loan.x = 0.0;
    }
    clock.set(100 * MS);
    {
        let mut loan = pub_b.loan_proxy::<Vector3>().expect("loan b");
        loan.x = 100.0;
    }
    runtime.step(Duration::from_millis(1));

    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_unmatched_discard_count("/swpu/a"),
        1,
        "a@0 must be discarded as UNMATCHABLE"
    );
    drop(runtime);

    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("WARN") && l.contains("sync discarded an UNMATCHABLE frame"))
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "expected exactly one loud UNMATCHABLE WARN, got {}: {hits:?}",
                hits.len()
            ));
        }
        let line = hits[0];
        if line.contains("Widen `sync_window_ms`") {
            return Err(format!(
                "UNMATCHABLE warn must no longer lead with unconditional widen: {line}"
            ));
        }
        for needle in [
            "node_id=fuse",
            "topic=/swpu/a",
            "input_period - skew",
            "previous period",
        ] {
            if !line.contains(needle) {
                return Err(format!("missing {needle:?} in UNMATCHABLE warn: {line}"));
            }
        }
        Ok(())
    });
}

#[test]
#[traced_test]
fn split_two_worker_graph_warns_once_for_cross_worker_sync() {
    // Full graph has producers (cam 33 ms, imu 5 ms) in worker 0 and Sync node
    // `fuse` (window 33 ms) in worker 1. Neither the PlanningOnly build nor the
    // producer worker should warn, and the Sync node's worker must warn ONCE.
    let full_config = two_producer_sync_graph("swp_split");
    let cam_info = NodeInfo::from_names(vec![], vec!["image".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 33 });
    let imu_info = NodeInfo::from_names(vec![], vec!["sample".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 5 });
    let fuse_info = NodeInfo::with_meta(
        vec![meta_input("image", true), meta_input("imu", true)],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Sync { window_ms: 33 });

    // 1. PlanningOnly build of the full graph must emit zero warnings.
    let planning_transport = cerulion_core::TransportManager::init_for_test(
        cerulion_core::TransportConfig {
            node_name: format!("swp_split_plan_{}", std::process::id()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("planning transport");
    let mut planning_factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    planning_factories.insert("cam".to_string(), make_entry(cam_info.clone()));
    planning_factories.insert("imu".to_string(), make_entry(imu_info.clone()));
    planning_factories.insert("fuse".to_string(), make_entry(fuse_info.clone()));
    let planning_rt = GraphRuntime::build_with_schema_hashes_provisioning_and_recorded(
        full_config.clone(),
        planning_factories,
        &planning_transport,
        Arc::new(VirtualClock::new()),
        None,
        None,
        None,
        cerulion_core::graph::BuildPurpose::PlanningOnly,
    )
    .expect("planning build");
    planning_rt.shutdown();

    // Compute full-graph advisories once (as the multi-process supervisor does)
    // and partition them to the worker that owns each flagged Sync node.
    let mut full_infos: IndexMap<String, NodeInfo> = IndexMap::new();
    full_infos.insert("cam".to_string(), cam_info.clone());
    full_infos.insert("imu".to_string(), imu_info.clone());
    full_infos.insert("fuse".to_string(), fuse_info.clone());
    let full_topo =
        cerulion_core::graph::GraphTopology::build(&full_config, &full_infos).expect("full topo");
    let full_warnings =
        cerulion_core::graph::check_sync_window_periods(&full_config, &full_topo, &full_infos);
    assert_eq!(
        full_warnings,
        vec![cerulion_core::graph::SyncWindowPeriodWarning {
            node_id: "fuse".to_string(),
            kind: cerulion_core::graph::SyncWindowKind::Bounded { window_ms: 33 },
            slowest_input: "image".to_string(),
            slowest_period_ms: 33,
        }]
    );

    // 2. Two worker subgraphs: worker 0 owns producers (stamped with none),
    // worker 1 owns `fuse` reading cross-worker absolute topics (stamped with
    // the full-graph warning for `fuse`).
    let producers_cfg = GraphConfig {
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "swp_split_graph".to_string(),
        prefix: "swp_split".to_string(),
        nodes: vec![full_config.nodes[0].clone(), full_config.nodes[1].clone()],
    };
    let mut prod_factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    prod_factories.insert("cam".to_string(), make_entry(cam_info));
    prod_factories.insert("imu".to_string(), make_entry(imu_info));
    let rt_prod = GraphRuntime::build_for_test_with_policy_and_wiring(
        producers_cfg,
        prod_factories,
        Arc::new(VirtualClock::new()),
        8,
        cerulion_core::MonitorWaitPolicy::off(),
        cerulion_core::graph::CrossProcessWiring::none().with_sync_window_period_warnings(&[]),
    )
    .expect("producer worker build");
    drop(rt_prod);

    let fusion_cfg = GraphConfig {
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "swp_split_graph".to_string(),
        prefix: "swp_split".to_string(),
        nodes: vec![NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "fuse".to_string(),
            node_type: "fuse".to_string(),
            inputs: vec![
                wired("image", "/swp_split/cam/image"),
                wired("imu", "/swp_split/imu/sample"),
            ],
            outputs: vec![],
        }],
    };
    let mut fuse_factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fuse_factories.insert("fuse".to_string(), make_entry(fuse_info));
    let rt_fuse = GraphRuntime::build_for_test_with_policy_and_wiring(
        fusion_cfg,
        fuse_factories,
        Arc::new(VirtualClock::new()),
        8,
        cerulion_core::MonitorWaitPolicy::off(),
        cerulion_core::graph::CrossProcessWiring::none()
            .with_sync_window_period_warnings(&full_warnings),
    )
    .expect("fusion worker build");
    drop(rt_fuse);

    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| {
                l.contains("WARN")
                    && l.contains(
                        "sync_window_ms is at or above the slowest resolved trigger-input period",
                    )
            })
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "expected exactly one sync-window WARN across the two-worker split graph, got {}: {hits:?}",
                hits.len()
            ));
        }
        let line = hits[0];
        for needle in [
            "node_id=fuse",
            "input=image",
            "period_ms=33",
            "sync_window_ms=33",
        ] {
            if !line.contains(needle) {
                return Err(format!("missing {needle:?} in warn line: {line}"));
            }
        }
        Ok(())
    });
}

#[test]
#[traced_test]
fn worker_with_empty_stamped_sync_warnings_logs_none() {
    let config = two_producer_sync_graph("swp_empty_stamp");
    let cam_info = NodeInfo::from_names(vec![], vec!["image".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 33 });
    let imu_info = NodeInfo::from_names(vec![], vec!["sample".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 5 });
    let fuse_info = NodeInfo::with_meta(
        vec![meta_input("image", true), meta_input("imu", true)],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Sync { window_ms: 33 });
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("cam".to_string(), make_entry(cam_info));
    factories.insert("imu".to_string(), make_entry(imu_info));
    factories.insert("fuse".to_string(), make_entry(fuse_info));

    let rt = GraphRuntime::build_for_test_with_policy_and_wiring(
        config,
        factories,
        Arc::new(VirtualClock::new()),
        8,
        cerulion_core::MonitorWaitPolicy::off(),
        cerulion_core::graph::CrossProcessWiring::none().with_sync_window_period_warnings(&[]),
    )
    .expect("build with empty stamped warnings");
    drop(rt);

    logs_assert(|lines: &[&str]| {
        let hits = lines
            .iter()
            .filter(|l| {
                l.contains("WARN")
                    && (l.contains("sync_window_ms is at or above")
                        || l.contains("unbounded_sync aligns periodic"))
            })
            .count();
        if hits != 0 {
            return Err(format!(
                "expected zero warnings when stamped list is empty, got {hits}"
            ));
        }
        Ok(())
    });
}
