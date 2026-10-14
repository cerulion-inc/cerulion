// SPDX-License-Identifier: AGPL-3.0-only
//! The app gateway's LAN access posture, read by a real zenoh peer over TCP.

use std::time::{Duration, Instant};

use cerulion_android::EmbeddedRuntime;
use cerulion_core::transport::discovery::{query_robot_catalog, TopicToken};
use cerulion_core::transport::TransportManager;
use cerulion_core::CatalogReply;
use serde_json::json;
use tempfile::tempdir;
use zenoh::Wait;

const ROBOT: &str = "lan_access_test";

fn free_port() -> u16 {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local_addr").port()
}

fn client(port: u16) -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", r#""client""#).unwrap();
    config
        .insert_json5("connect/endpoints", &format!(r#"["tcp/127.0.0.1:{port}"]"#))
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().expect("open client session")
}

fn catalog_until(session: &zenoh::Session, done: impl Fn(&CatalogReply) -> bool) -> CatalogReply {
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let reply = query_robot_catalog(session, ROBOT, Duration::from_secs(1));
        match reply {
            Some(reply) if done(&reply) => return reply,
            other if Instant::now() >= deadline => panic!("no matching catalog reply: {other:?}"),
            _ => std::thread::sleep(Duration::from_millis(200)),
        }
    }
}

#[test]
fn stopping_an_open_runtime_revokes_lan_grants() {
    let temp = tempdir().unwrap();
    let port = free_port();
    let topic = format!("/{ROBOT}/heartbeat");
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into()).unwrap();
    let config = |extra: serde_json::Value| {
        let mut config = json!({
            "listen": [format!("tcp/127.0.0.1:{port}")],
            "mdns": false,
            "publish": [{"topic": topic, "schema": "std_msgs/String"}]
        });
        config
            .as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        config.to_string()
    };

    runtime.start(&config(json!({}))).unwrap();
    {
        let status = runtime.status_handle();
        let status = status.lock().unwrap();
        assert_eq!(status.lan_access.as_deref(), Some("paired"));
        assert!(!status.mdns_advertised);
    }
    let session = client(port);
    let refused = catalog_until(&session, |reply| reply.error.is_some());
    assert!(refused.entries.is_empty(), "a refusal serves no topic");
    let reason = refused.error.unwrap();
    assert!(
        reason.contains("LAN access is paired-only")
            && reason.contains("set the LAN access posture to open for local development"),
        "{reason}"
    );
    runtime.stop().unwrap();
    assert_eq!(
        runtime
            .status_handle()
            .lock()
            .unwrap()
            .lan_access
            .as_deref(),
        Some("paired"),
        "a stopped runtime keeps reporting the posture it ran with"
    );

    runtime
        .start(&config(json!({"lan_access": "open"})))
        .unwrap();
    assert_eq!(
        runtime
            .status_handle()
            .lock()
            .unwrap()
            .lan_access
            .as_deref(),
        Some("open")
    );
    let served = catalog_until(&session, |reply| {
        reply.error.is_none() && reply.entries.iter().any(|entry| entry.topic == topic)
    });
    assert!(served.error.is_none());
    let manager = TransportManager::get().expect("initialized app transport manager");
    let bridges = manager.bridge_manager();
    let _subscriber = session
        .declare_subscriber(topic.trim_start_matches('/'))
        .wait()
        .expect("subscribe to published topic");
    let _demand = TopicToken::declare(&session, &topic).expect("demand published topic");
    let enable_deadline = Instant::now() + Duration::from_secs(10);
    while !bridges.is_enabled(&topic).unwrap() {
        assert!(
            Instant::now() < enable_deadline,
            "the live peer demand must enable the published topic"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    assert!(bridges.is_enabled(&topic).unwrap());
    runtime.stop().unwrap();
    assert!(!bridges.is_enabled(&topic).unwrap());
    assert!(bridges.is_lan_closed().unwrap());
    let revoke_deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < revoke_deadline {
        assert!(
            !bridges.is_enabled(&topic).unwrap(),
            "live demand workers must not re-enable egress after stop"
        );
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(bridges.is_lan_closed().unwrap());
    drop(_demand);
    drop(_subscriber);
    drop(session);

    runtime.start(&config(json!({}))).unwrap();
    let session = client(port);
    std::thread::sleep(Duration::from_millis(1500));
    assert!(
        !bridges.is_enabled(&topic).unwrap(),
        "paired restart must not retain a bridge grant from the open run"
    );
    let refused_again = catalog_until(&session, |reply| reply.error.is_some());
    assert!(
        refused_again.entries.is_empty(),
        "restarting paired closes the LAN again"
    );
    runtime.stop().unwrap();
}
