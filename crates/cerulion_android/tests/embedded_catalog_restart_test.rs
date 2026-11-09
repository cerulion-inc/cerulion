// SPDX-License-Identifier: AGPL-3.0-only
//! A catalog read after a stop and a start in one process describes the
//! live run: its topics, its schemas and the frames its gateway observed,
//! and for a runtime topic the schema hash registered last. Own binary: the
//! transport manager and the runtime's latched query surface are
//! process-global.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use cerulion_android::EmbeddedRuntime;
use cerulion_core::transport::cerulion_q::{CatalogEntry, CatalogProvenance, CatalogReply};
use cerulion_core::transport::TransportManager;
use serde_json::json;
use tempfile::tempdir;

const ROBOT: &str = "android_catalog_restart_test";

/// How long the gateway may take to tap a topic and observe a frame: its
/// liveness sweep runs every 200 ms, so this leaves wide margin.
const OBSERVE_DEADLINE: Duration = Duration::from_secs(10);

fn topic(name: &str) -> String {
    format!("/{ROBOT}/{name}")
}

fn config(publishes: &[(&str, &str)]) -> String {
    let publish: Vec<_> = publishes
        .iter()
        .map(|(name, schema)| json!({"topic": topic(name), "schema": schema}))
        .collect();
    json!({
        "listen": ["tcp/127.0.0.1:0"],
        "lan_access": "open",
        "mdns": false,
        "publish": publish
    })
    .to_string()
}

fn query_catalog() -> CatalogReply {
    let manager = TransportManager::get().expect("transport manager");
    let network = manager.network().expect("network transport");
    cerulion_core::transport::discovery::query_robot_catalog(
        network.session().expect("network session"),
        ROBOT,
        Duration::from_secs(1),
    )
    .expect("runtime must answer its catalog query")
}

fn entry<'a>(catalog: &'a CatalogReply, name: &str) -> Option<&'a CatalogEntry> {
    let topic = topic(name);
    catalog.entries.iter().find(|entry| entry.topic == topic)
}

fn frames_observed(catalog: &CatalogReply, name: &str) -> u64 {
    entry(catalog, name)
        .and_then(|entry| entry.liveness)
        .map_or(0, |liveness| liveness.frames_observed)
}

/// Publish `json` on `name` until the catalog reports a frame observed on
/// it, and return that catalog. The gateway taps a topic on a sweep, and a
/// frame sent before the tap is not seen, so one publish is not enough.
fn publish_until_observed(runtime: &mut EmbeddedRuntime, name: &str, json: &str) -> CatalogReply {
    let deadline = Instant::now() + OBSERVE_DEADLINE;
    loop {
        runtime.publish(&topic(name), json).expect("publish");
        let catalog = query_catalog();
        if frames_observed(&catalog, name) > 0 {
            return catalog;
        }
        assert!(
            Instant::now() < deadline,
            "the catalog never reported a frame on {name}: {:?}",
            entry(&catalog, name)
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The first run's and the second run's schema hash for the runtime topic
/// `d`. Neither is zero, and they differ.
const FIRST_HASH: u64 = 0x1111_2222_3333_4444;
const SECOND_HASH: u64 = 0x5555_6666_7777_8888;

/// Register `d` as a runtime egress topic with `hash`, the way a producer
/// in the app process advertises a topic and its schema hash, and return
/// the first catalog that lists `d` with that hash. The registration
/// reaches the gateway over its control channel, so the catalog can lag.
fn register_until_listed(hash: u64) -> CatalogReply {
    TransportManager::get()
        .expect("transport manager")
        .register_dynamic_egress_topic(&topic("d"), hash)
        .expect("register the runtime topic d");
    let deadline = Instant::now() + OBSERVE_DEADLINE;
    loop {
        let catalog = query_catalog();
        if entry(&catalog, "d").and_then(|entry| entry.schema_hash) == Some(hash) {
            return catalog;
        }
        assert!(
            Instant::now() < deadline,
            "the catalog never listed d with schema hash {hash:#x}: {:?}",
            entry(&catalog, "d")
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn listed(catalog: &CatalogReply) -> Vec<String> {
    let mut topics: Vec<String> = catalog
        .entries
        .iter()
        .map(|entry| entry.topic.clone())
        .collect();
    topics.sort();
    topics
}

#[test]
fn catalog_after_a_restart_reports_the_live_run() {
    let files = tempdir().expect("app files directory");
    let mut runtime =
        EmbeddedRuntime::new(PathBuf::from(files.path()), ROBOT.to_string()).expect("runtime");

    // Run 1 serves A and B, and B carries frames.
    runtime
        .start(&config(&[
            ("a", "std_msgs/String"),
            ("b", "std_msgs/String"),
        ]))
        .expect("start the first run");
    let first = publish_until_observed(&mut runtime, "b", r#"{"data":"first run"}"#);
    assert_eq!(listed(&first), vec![topic("a"), topic("b")]);
    // A runtime topic carries the schema hash its producer registered.
    let first = register_until_listed(FIRST_HASH);
    let d = entry(&first, "d").expect("the catalog lists d");
    assert_eq!(d.provenance, CatalogProvenance::Runtime, "{d:?}");
    runtime.stop().expect("stop the first run");

    // Run 2 serves B with a new schema and C, and only C carries frames.
    runtime
        .start(&config(&[
            ("b", "geometry_msgs/Vector3"),
            ("c", "std_msgs/String"),
        ]))
        .expect("start the second run");
    publish_until_observed(&mut runtime, "c", r#"{"data":"second run"}"#);
    // Its producer registers d again under a new schema. Until then the
    // catalog can show the first run's hash (that registration stays on the
    // process-wide manager); after it, the catalog must show the new hash.
    let second = register_until_listed(SECOND_HASH);
    let d = entry(&second, "d").expect("the catalog lists d");
    assert_eq!(d.provenance, CatalogProvenance::Runtime, "{d:?}");

    assert_eq!(listed(&second), vec![topic("b"), topic("c"), topic("d")]);
    let b = entry(&second, "b").expect("the catalog lists b");
    assert_eq!(b.schema_name.as_deref(), Some("geometry_msgs/Vector3"));
    assert_eq!(
        frames_observed(&second, "b"),
        0,
        "b carried no frame in the second run, so the first run's frames must not show: {:?}",
        b.liveness
    );
    let c = entry(&second, "c").expect("the catalog lists c");
    assert_eq!(c.schema_name.as_deref(), Some("std_msgs/String"));
    // App publish topics are boot-plan topics, which catalog without a
    // `schema_hash`; no hash from the first run may appear on either.
    assert_eq!((b.schema_hash, c.schema_hash), (None, None));

    runtime.stop().expect("stop the second run");
}
