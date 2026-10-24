// SPDX-License-Identifier: AGPL-3.0-only
//! An in-process MCP client reads `robot://graph` against a fixture graph.
//!
//! Transport isolation is per-test (`init_for_test` SHM roots, unique node
//! names), so the tests run in parallel. Time is a `VirtualClock` driven by
//! the test, so every tick lands exactly where the test puts it.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::transport::liveness::LIVENESS_SWEEP_INTERVAL_NS;
use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_rcpd::graph::{NodeView, RobotGraph, RunStateView, RunsCompleteness};
use cerulion_rcpd::manifest::RcpManifest;
use cerulion_rcpd::observer::{GraphSource, GRAPH_REFRESH_INTERVAL_NS};
use cerulion_rcpd::server::RcpServer;
use cerulion_rcpd::{GRAPH_URI, SERVER_NAME};
use rmcp::model::{ReadResourceRequestParams, ResourceContents};
use rmcp::service::RunningService;
use rmcp::{RoleClient, ServiceExt};

const T0: u64 = 1_000_000_000_000;
const RUN_ID: u128 = 0x1862_0000_0000_0000_0000_0000_0000_0001;
const FRAMES_PER_SWEEP: u32 = 4;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique(tag: &str) -> String {
    format!(
        "rcpd_{tag}_{}_{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

fn manager(tag: &str) -> (Arc<TransportManager>, Arc<VirtualClock>) {
    let clock = Arc::new(VirtualClock::new());
    clock.set(T0);
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: unique(tag),
            clock: clock_dyn,
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport manager");
    (mgr, clock)
}

fn frame(seq: u32, ts: u64) -> Vec<u8> {
    let total = WireHeader::SIZE + 8;
    let mut h = WireHeader::new(0x889, seq, ts);
    h.total_size = total as u32;
    let mut f = vec![0u8; total];
    h.write_to_buf(&mut f[..WireHeader::SIZE]);
    f
}

fn fixture_manifest(dir: &std::path::Path) -> std::path::PathBuf {
    std::fs::write(
        dir.join("graph.yaml"),
        "name: rcp_fixture\nnodes:\n  - id: camera\n    type: camera_node\n  - id: planner\n    type: planner_node\n",
    )
    .expect("write graph");
    let manifest = dir.join("rcp.yaml");
    std::fs::write(&manifest, "robot: fixture_bot\ngraph: graph.yaml\n").expect("write manifest");
    manifest
}

async fn connect(server: RcpServer) -> RunningService<RoleClient, ()> {
    let (client_io, server_io) = tokio::io::duplex(64 * 1024);
    let (server, client) = tokio::join!(server.serve(server_io), ().serve(client_io));
    let server = server.expect("server session");
    tokio::spawn(async move {
        let _ = server.waiting().await;
    });
    client.expect("client session")
}

async fn read_graph(client: &RunningService<RoleClient, ()>) -> RobotGraph {
    let result = client
        .read_resource(ReadResourceRequestParams::new(GRAPH_URI))
        .await
        .expect("read robot://graph");
    assert_eq!(result.contents.len(), 1, "one content block");
    let ResourceContents::TextResourceContents {
        uri,
        mime_type,
        text,
        ..
    } = &result.contents[0]
    else {
        panic!("robot://graph is text");
    };
    assert_eq!(uri, GRAPH_URI);
    assert_eq!(mime_type.as_deref(), Some("application/json"));
    serde_json::from_str(text).expect("robot://graph is the documented JSON")
}

#[tokio::test(flavor = "multi_thread")]
async fn a_client_reads_the_fixture_graphs_nodes_topics_rates_and_run() {
    let dir = tempfile::tempdir().expect("tempdir");
    let identity = RcpManifest::load(&fixture_manifest(dir.path())).expect("load manifest");
    let (mgr, clock) = manager("graph");
    let topic = format!("/{}/camera", unique("t"));
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("publisher");
    let _run = RunHandle::publish_on_config(
        &mgr.iox_config(),
        RunRecord {
            run_id: RUN_ID,
            graph_name: "rcp_fixture".to_string(),
            run_dir: dir.path().to_string_lossy().into_owned(),
            supervisor_pid: std::process::id(),
            run_started_at_ns: 7,
            state: RunState::Live,
        },
    )
    .expect("publish run");

    let mut source = GraphSource::new(Arc::clone(&mgr), identity);
    let feed = source.feed();
    let first = feed.current();
    assert_eq!(first.graph.refreshed_at_ns, T0, "composed at open");

    // 20 Hz: four frames per 200 ms sweep, for 7.2 s of virtual time: past the
    // monitor's 5 s settle window, so the 7 s refresh carries judged samples.
    let step = LIVENESS_SWEEP_INTERVAL_NS / u64::from(FRAMES_PER_SWEEP);
    let mut seq = 0u32;
    for k in 1..=36u64 {
        let prev = T0 + (k - 1) * LIVENESS_SWEEP_INTERVAL_NS;
        for i in 0..u64::from(FRAMES_PER_SWEEP) {
            let ts = prev + (i + 1) * step;
            clock.set(ts);
            publisher.publish_raw(&frame(seq, ts)).expect("publish");
            seq += 1;
        }
        source.tick();
    }
    let refreshed_at = T0 + 7 * GRAPH_REFRESH_INTERVAL_NS;

    let client = connect(RcpServer::new(feed)).await;
    let graph = read_graph(&client).await;

    assert_eq!(graph.version, 1);
    assert_eq!(graph.robot, "fixture_bot");
    assert_eq!(
        graph.refreshed_at_ns, refreshed_at,
        "the 1 Hz refresh, not every tick"
    );
    assert_eq!(
        graph.nodes,
        vec![
            NodeView {
                node_id: "camera".into(),
                node_type: "camera_node".into(),
                counters: None,
            },
            NodeView {
                node_id: "planner".into(),
                node_type: "planner_node".into(),
                counters: None,
            },
        ]
    );

    assert_eq!(graph.runs.completeness, RunsCompleteness::Settled);
    assert_eq!(graph.runs.records.len(), 1, "{:?}", graph.runs);
    let run = &graph.runs.records[0];
    assert_eq!(run.run_id, "0x18620000000000000000000000000001");
    assert_eq!(run.graph_name, "rcp_fixture");
    assert_eq!(run.state, RunStateView::Live);
    assert_eq!(run.run_started_at_ns, 7);

    assert_eq!(graph.catalog.robot, "fixture_bot");
    assert_eq!(graph.catalog.error, None);
    let entry = graph
        .catalog
        .entries
        .iter()
        .find(|e| e.topic == topic)
        .expect("the fixture topic is in the catalog");
    assert_eq!(entry.producer_count, Some(1));
    let liveness = entry.liveness.expect("the fixture topic is observed");
    assert_eq!(
        liveness.last_frame_age_ms,
        Some(0),
        "newest frame stamped at the read"
    );
    let rate = liveness.rate_estimate.expect("a rate after a full window");
    assert_eq!(rate.millihertz, 20_000, "four frames per 200 ms");
    assert!(!rate.is_floor);

    let row = graph
        .monitors
        .iter()
        .find(|r| r.topic == topic)
        .expect("the monitor watches the fixture topic");
    assert!(
        row.conditions.is_empty(),
        "a steady topic raises nothing: {row:?}"
    );
    // The monitor first saw the topic at 0.2 s and samples every 0.4 s; the
    // settle window withholds everything before 5.2 s, so the samples kept by
    // the 7.0 s refresh are 5.4, 5.8, 6.2, 6.6 and 7.0 s.
    assert_eq!(row.samples, 5, "{row:?}");
    assert_eq!(row.observed_mhz, Some(20_000));

    client.cancel().await.expect("close client");
}

#[tokio::test(flavor = "multi_thread")]
async fn the_server_lists_one_resource_no_tools_and_names_itself() {
    let (mgr, _clock) = manager("list");
    let identity = RcpManifest::parse("robot: lister\n")
        .expect("manifest")
        .robot
        .map(|robot| cerulion_rcpd::manifest::RobotIdentity {
            robot,
            nodes: Vec::new(),
        })
        .expect("robot");
    let source = GraphSource::new(mgr, identity);
    let client = connect(RcpServer::new(source.feed())).await;

    let info = client.peer_info().expect("server info");
    assert_eq!(
        info.server_info.as_ref().map(|s| s.name.as_str()),
        Some(SERVER_NAME)
    );
    assert_eq!(
        info.capabilities
            .resources
            .as_ref()
            .and_then(|r| r.subscribe),
        Some(true)
    );

    let resources = client.list_resources(None).await.expect("list resources");
    let uris: Vec<&str> = resources.resources.iter().map(|r| r.uri.as_str()).collect();
    assert_eq!(uris, vec![GRAPH_URI]);

    let tools = client.list_tools(None).await.expect("list tools");
    assert!(tools.tools.is_empty(), "no tools in the read surface");

    let graph = read_graph(&client).await;
    assert_eq!(graph.robot, "lister");
    assert!(graph.nodes.is_empty());

    let missing = client
        .read_resource(ReadResourceRequestParams::new("robot://state"))
        .await;
    assert!(missing.is_err(), "an unknown resource is an error");

    client.cancel().await.expect("close client");
}
