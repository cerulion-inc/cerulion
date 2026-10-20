// SPDX-License-Identifier: AGPL-3.0-only
//! LIVE loopback tests for the remote half of `topic list -t`, `topic find`
//! and `topic type`: a robot-side zenoh session serves its catalog (and a
//! custom type's `.msg`), and the desk types the robot's topics from it.
//!
//! Real zenoh sessions with scouting off and explicit loopback locators, as
//! in `topic_network_live_test.rs`. No iceoryx2 and no shared state, so no
//! `--test-threads=1` requirement.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_cli_engine::topic_cmd::introspect::{
    remote_topic_types, render_remote_typed_topics_section, TopicType,
};
use cerulion_cli_engine::topic_cmd::{
    local_walker_from_workspace, query_remote_topics_and_catalogs_with_candidates,
    resolve_remote_walker_for_topic_with_opts, RemoteTopicsOptions, STD_MSGS_STRING_SCHEMA_HASH,
};
use cerulion_core::codegen::parse_rosmsg;
use cerulion_core::transport::bridge::TopicBridgeManager;
use cerulion_core::transport::discovery::TopicToken;
use cerulion_core::transport::network::{
    CatalogSource, GetDemandState, NetworkConfig, NetworkManager, RunSource, SchemaSource,
};
use cerulion_core::{SchemaDoc, SchemaEncoding};

const WIDGET: &str = "probe_msgs/Widget";
const WIDGET_TEXT: &str = "float64 x\nfloat64 y\n";

/// A `runs` source on a fresh isolated namespace (these arms never GET runs).
fn isolated_runs_source() -> RunSource {
    RunSource {
        iox_config: Arc::new(cerulion_core::testing::iceoryx_test_config()),
        gather_window: cerulion_core::transport::run_registry::RUN_GATHER_WINDOW,
    }
}

/// A listening robot-side session on a pid-derived localhost port.
fn bind_listening_robot(robot: &str, base: u16) -> (NetworkManager, u16) {
    let pid = std::process::id();
    for slot in 0..3u16 {
        let port = base + slot * 400 + (pid % 300) as u16;
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            return (mgr, port);
        }
    }
    panic!("no candidate localhost port could be bound for robot '{robot}'");
}

/// A robot that announces `/r/widget` (catalogued by NAME as the custom
/// Widget, whose `.msg` it serves) and `/r/plain` (catalogued by the
/// `std_msgs/String` HASH alone). The returned values keep it serving.
fn serve_robot(robot: &str, base: u16) -> (NetworkManager, u16, Vec<TopicToken>) {
    let (mgr, port) = bind_listening_robot(robot, base);
    let session = mgr.session().expect("robot session");
    let tokens = vec![
        TopicToken::announce(session, robot, "/r/widget").expect("announce widget"),
        TopicToken::announce(session, robot, "/r/plain").expect("announce plain"),
    ];
    let bridge = Arc::new(TopicBridgeManager::new());
    bridge.register_topic("/r/widget").expect("register widget");
    bridge.register_topic("/r/plain").expect("register plain");
    mgr.start_query_surface(
        Arc::clone(&bridge),
        Arc::new(GetDemandState::default()),
        CatalogSource {
            hashes: Arc::new(Mutex::new(HashMap::from([(
                "/r/plain".to_string(),
                STD_MSGS_STRING_SCHEMA_HASH,
            )]))),
            boot_topics: Arc::new(HashSet::new()),
            topic_schemas: Arc::new(Mutex::new(HashMap::from([(
                "/r/widget".to_string(),
                WIDGET.to_string(),
            )]))),
            hash_names: Arc::new(Mutex::new(HashMap::new())),
            producer_probe: None,
            liveness: None,
            liveness_clock: None,
        },
        SchemaSource {
            docs: Arc::new(Mutex::new(BTreeMap::from([(
                WIDGET.to_string(),
                SchemaDoc {
                    qualified: WIDGET.to_string(),
                    encoding: SchemaEncoding::Msg,
                    text: WIDGET_TEXT.to_string(),
                    deps: vec![],
                },
            )]))),
        },
        isolated_runs_source(),
    )
    .expect("declare query surface");
    (mgr, port, tokens)
}

#[test]
fn a_typed_listing_types_remote_rows_from_the_robot_catalog() {
    let (_robot, port, _tokens) = serve_robot("typed", 31200);
    let opts = RemoteTopicsOptions {
        connect: vec![format!("tcp/127.0.0.1:{port}")],
        listen: vec![],
        scouting: false,
    };
    let walker = local_walker_from_workspace(None);
    let expected: BTreeMap<String, TopicType> = [
        ("/r/plain", TopicType::Named("std_msgs/String".to_string())),
        ("/r/widget", TopicType::Named(WIDGET.to_string())),
    ]
    .into_iter()
    .map(|(t, ty)| (t.to_string(), ty))
    .collect();
    let mut last = BTreeMap::new();
    for _ in 0..30 {
        let (disc, catalogs) =
            query_remote_topics_and_catalogs_with_candidates(&opts, vec![]).expect("discovery");
        last = remote_topic_types(&catalogs.replies, &[], &walker);
        if last == expected {
            let rows: Vec<String> = render_remote_typed_topics_section(&disc, true, &[], &last)
                .lines()
                .filter(|l| l.starts_with('/'))
                .map(str::to_string)
                .collect();
            assert_eq!(
                rows,
                vec![
                    "/r/plain  [std_msgs/String]".to_string(),
                    "/r/widget  [probe_msgs/Widget]".to_string(),
                ]
            );
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!("the catalog never typed the robot's topics: expected {expected:?}, last {last:?}");
}

/// The `topic type` ladder for a frame whose schema HASH only the robot can
/// name: the desk's walker leaves the hash unnamed, and the walker seeded
/// from the robot (found through the robot's catalog entry for the topic,
/// as `topic type` finds it) names that hash. A catalog entry that carries a
/// hash alone is the `/r/plain` row of the listing test above.
#[test]
fn a_hash_only_a_robot_serves_is_named_by_the_remote_walker() {
    let (_robot, port, _tokens) = serve_robot("ladder", 31400);
    let connect = vec![format!("tcp/127.0.0.1:{port}")];
    let widget_hash = parse_rosmsg(WIDGET_TEXT, "Widget", Some("probe_msgs"))
        .unwrap()
        .schema_hash();
    let local = local_walker_from_workspace(None);
    // The desk alone cannot name the custom type.
    assert_eq!(
        TopicType::of_hash(widget_hash, &[&local]),
        TopicType::Unnamed(widget_hash)
    );
    for _ in 0..40 {
        if let Some((robot, remote)) =
            resolve_remote_walker_for_topic_with_opts("/r/widget", &connect, &[], false)
        {
            assert_eq!(robot, "ladder");
            // The `topic type` ladder: local first, then the robot's walker.
            assert_eq!(
                TopicType::of_hash(widget_hash, &[&local, &remote]),
                TopicType::Named(WIDGET.to_string())
            );
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!("the robot's walker never resolved /r/widget");
}
