// SPDX-License-Identifier: AGPL-3.0-only
//! The BEHAVIORAL pin that the mirror
//! provenance registry `/__cerulion/mirrors` never surfaces in `topic list`.
//!
//! The structural pins (`data_topic_of_service(MIRROR_REGISTRY_SERVICE_NAME)` ==
//! `None`; the name has no `/data` suffix) live inline. THIS is the end-to-end
//! truth: a REAL registered mirror creates the `/__cerulion/mirrors` control
//! service AND a real `{topic}/data` mirror publisher over a per-test SHM root,
//! then the ACTUAL enumeration `topic list` uses (`list_topic_infos`) is run
//! against that root — the registry must be ABSENT from the output while the
//! mirror topic is PRESENT. Parallel-safe (per-test SHM root; NO `#[serial]`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

use cerulion_core::message::ShmMessage;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

#[test]
fn mirror_registry_service_never_surfaces_in_topic_list() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("mirror_invis_{id}"),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init_for_test");

    let topic = format!("/utlidar/robot_odom_{id}");
    // Create a REAL local mirror publisher (a genuine `{topic}/data` service) and
    // register its provenance (which lazily opens the `/__cerulion/mirrors`
    // control service).
    let _injector = manager
        .create_ingress_injector(&topic, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("create the local mirror publisher");
    manager
        .register_mirror_provenance(&topic, "ubuntu")
        .expect("register provenance");

    // Enumerate exactly as `topic list` does, on THIS per-test root.
    let names: Vec<String> = cerulion_cli_engine::topic_cmd::list_topic_infos(&root)
        .expect("enumerate")
        .into_iter()
        .map(|t| t.name)
        .collect();

    // The mirror's `{topic}/data` service IS a listable topic.
    assert!(
        names.iter().any(|n| n == &topic),
        "the mirror's data topic must be enumerable: {names:?}"
    );
    // The `/__cerulion/mirrors` control service (no `/data` suffix) must NEVER
    // appear — the invisibility is by design, and this proves it behaviorally.
    assert!(
        !names.iter().any(|n| n.contains("__cerulion/mirrors")),
        "the mirror provenance registry must never surface in `topic list`: {names:?}"
    );
    assert!(
        !names.iter().any(|n| n.contains("__cerulion")),
        "no reserved control-plane service may surface in `topic list`: {names:?}"
    );
}
