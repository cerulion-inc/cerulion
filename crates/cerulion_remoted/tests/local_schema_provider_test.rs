// SPDX-License-Identifier: AGPL-3.0-only
//! Real local gateway metadata reaches a paired wire observer without a network query.

mod common;
#[path = "local_schema_provider/fixture.rs"]
mod fixture;

use std::io::{BufRead, BufReader, Read, Write};
use std::os::unix::net::UnixListener;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::transport::cerulion_q::{SchemaDoc, SchemaEncoding, SchemaReply};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{GatewayEgressPolicy, GatewayPlan, SchemaHashName, SchemaServing, TopicSchema};
use cerulion_netd::egress::EgressPlane;
use cerulion_pairing::format::PublicKey;
use cerulion_remoted::{WireRequest, WireResponse};
use common::{bounded, disabled_endpoint};
use fixture::{manager, revoke, Desk, LocalNetd, Robot, ROBOT_NAME};

const TOPIC: &str = "/schemas/reading";
const RAW_TOPIC: &str = "/schemas/raw_string";
const ROOT: &str = "example_msgs/Reading";
const NESTED: &str = "example_msgs/Detail";
const ROOT_TEXT: &str = "example_msgs/Detail detail\nuint64 count\n";
const NESTED_TEXT: &str = "float64 value\n";
// Hand-chosen hash/name bindings pin provider transport; no claim that these are
// generated recipe hashes. The wire carries exactly these values.
const CUSTOM_HASH: u64 = 0x0102_0304_0506_0708;
const BUILTIN_HASH: u64 = 0x1112_1314_1516_1718;

fn serving() -> SchemaServing {
    SchemaServing {
        topic_schemas: vec![TopicSchema {
            topic: TOPIC.into(),
            schema_name: ROOT.into(),
        }],
        schema_hashes: vec![
            SchemaHashName {
                schema_hash: CUSTOM_HASH,
                qualified: ROOT.into(),
            },
            SchemaHashName {
                schema_hash: BUILTIN_HASH,
                qualified: "std_msgs/String".into(),
            },
        ],
        // Deliberately nested-first: closure output must still be root-first.
        schema_docs: vec![
            SchemaDoc {
                qualified: NESTED.into(),
                encoding: SchemaEncoding::Msg,
                text: NESTED_TEXT.into(),
                deps: vec![],
            },
            SchemaDoc {
                qualified: ROOT.into(),
                encoding: SchemaEncoding::Msg,
                text: ROOT_TEXT.into(),
                deps: vec![NESTED.into()],
            },
        ],
    }
}
fn schema_request(topic: &str) -> WireRequest {
    WireRequest::Schema {
        topic: topic.into(),
    }
}
fn assert_closure(reply: WireResponse) -> SchemaReply {
    let WireResponse::Schema(reply) = reply else {
        panic!("expected a custom schema: {reply:?}")
    };
    assert_eq!(reply.robot, ROBOT_NAME);
    assert_eq!(reply.requested, TOPIC);
    assert!(reply.error.is_none());
    assert_eq!(
        reply.docs,
        vec![
            SchemaDoc {
                qualified: ROOT.into(),
                encoding: SchemaEncoding::Msg,
                text: ROOT_TEXT.into(),
                deps: vec![NESTED.into()]
            },
            SchemaDoc {
                qualified: NESTED.into(),
                encoding: SchemaEncoding::Msg,
                text: NESTED_TEXT.into(),
                deps: vec![]
            },
        ]
    );
    reply
}
fn assert_source_error(reply: WireResponse, fragment: &str) {
    let WireResponse::Error { message, .. } = reply else {
        panic!("schema source must fail closed: {reply:?}")
    };
    assert!(message.contains(fragment), "{message}");
}
fn frame(hash: u64) -> [u8; WireHeader::SIZE] {
    let mut bytes = [0; WireHeader::SIZE];
    WireHeader {
        schema_hash: hash,
        total_size: WireHeader::SIZE as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: 0,
        sequence: 7,
        timestamp_ns: 19,
    }
    .write_to_buf(&mut bytes);
    bytes
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn late_registration_serves_exact_nested_closure_and_names_builtin_wire_hash() {
    let netd = LocalNetd::start();
    let endpoint = disabled_endpoint([72; 32]).await;
    let robot = Robot::start(
        netd.manager.clone(),
        netd.path.clone(),
        &[PublicKey(*endpoint.id().as_bytes())],
    )
    .await;
    let mut desk = Desk::connect(&endpoint, &robot).await;
    let WireResponse::Schema(before) = desk.request(&schema_request(TOPIC)).await else {
        panic!("expected not-found before registration")
    };
    assert!(before.error.is_some());
    assert!(before.docs.is_empty());
    let mut registration = netd.client();
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![TOPIC.into(), RAW_TOPIC.into()],
        ingress: vec![],
    };
    let namespace = serde_json::to_string(&netd.manager.iox_config()).unwrap();
    registration
        .register_egress(&plan, &serving(), Some(&namespace))
        .unwrap();
    let first = assert_closure(desk.request(&schema_request(TOPIC)).await);
    let second = assert_closure(desk.request(&schema_request(TOPIC)).await);
    assert_eq!(
        serde_json::to_vec(&first).unwrap(),
        serde_json::to_vec(&second).unwrap()
    );

    let mut custom = netd
        .manager
        .create_publisher_simple(TOPIC, MaxSliceLen::const_new(256))
        .unwrap();
    let mut builtin = netd
        .manager
        .create_publisher_simple(RAW_TOPIC, MaxSliceLen::const_new(256))
        .unwrap();
    desk.send(&WireRequest::Catalog).await;
    // Wait for the actual catalog's introspection subscriptions, not a sleep
    // chosen to approximate gateway scheduling. Both frames are fixed oracles.
    bounded("catalog subscribers", async {
        let mut custom_sent = false;
        let mut builtin_sent = false;
        while !custom_sent || !builtin_sent {
            if !custom_sent && netd.manager.topic_subscriber_count(TOPIC) > 0 {
                custom.publish_raw(&frame(CUSTOM_HASH)).unwrap();
                custom_sent = true;
            }
            if !builtin_sent && netd.manager.topic_subscriber_count(RAW_TOPIC) > 0 {
                builtin.publish_raw(&frame(BUILTIN_HASH)).unwrap();
                builtin_sent = true;
            }
            if !custom_sent || !builtin_sent {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        }
    })
    .await;
    let WireResponse::Catalog(catalog) = desk.receive().await else {
        panic!("expected named catalog")
    };
    let custom_row = catalog
        .entries
        .iter()
        .find(|row| row.topic == TOPIC)
        .unwrap();
    assert_eq!(custom_row.schema_name.as_deref(), Some(ROOT));
    assert_eq!(custom_row.schema_hash, Some(CUSTOM_HASH));
    let builtin_row = catalog
        .entries
        .iter()
        .find(|row| row.topic == RAW_TOPIC)
        .unwrap();
    assert_eq!(builtin_row.schema_name.as_deref(), Some("std_msgs/String"));
    assert_eq!(builtin_row.schema_hash, Some(BUILTIN_HASH));
    let WireResponse::Schema(builtin_reply) = desk.request(&schema_request(RAW_TOPIC)).await else {
        panic!("expected structured builtin closure absence")
    };
    assert!(builtin_reply.docs.is_empty());
    assert!(builtin_reply
        .error
        .unwrap()
        .contains("no custom schema closure"));
    assert!(
        !netd
            .plane
            .boot_standing_gateway(&Default::default())
            .unwrap(),
        "provider must retain the existing gateway"
    );
    drop((custom, builtin, registration));
    desk.close();
    robot.shutdown().await;
    bounded("desk close", endpoint.close()).await;
    netd.shutdown();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn different_shared_memory_namespace_refuses_catalog_and_schema() {
    let netd = LocalNetd::start();
    let other_namespace = manager(false);
    assert_ne!(
        netd.manager.iox_shm_identity(),
        other_namespace.iox_shm_identity()
    );
    let endpoint = disabled_endpoint([73; 32]).await;
    let robot = Robot::start(
        other_namespace,
        netd.path.clone(),
        &[PublicKey(*endpoint.id().as_bytes())],
    )
    .await;
    let mut desk = Desk::connect(&endpoint, &robot).await;
    for request in [WireRequest::Catalog, schema_request(TOPIC)] {
        assert_source_error(
            desk.request(&request).await,
            "different shared-memory namespace",
        );
    }
    desk.close();
    robot.shutdown().await;
    bounded("desk close", endpoint.close()).await;
    netd.shutdown();
}

fn scripted_listener(path: &Path) -> UnixListener {
    UnixListener::bind(path).unwrap()
}
fn receive_snapshot_request(listener: UnixListener) -> BufReader<std::os::unix::net::UnixStream> {
    let (mut stream, _) = listener.accept().unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(10)))
        .unwrap();
    stream
        .set_write_timeout(Some(Duration::from_secs(10)))
        .unwrap();
    stream
        .write_all(b"{\"hello\":\"cerulion-netd\",\"protocol\":8}\n")
        .unwrap();
    let mut reader = BufReader::new(stream);
    let mut request = String::new();
    reader.read_line(&mut request).unwrap();
    let request: serde_json::Value = serde_json::from_str(&request).unwrap();
    assert_eq!(request["method"], "serving_schema_snapshot");
    assert_eq!(request["id"], 1);
    reader
}
fn assert_closed(reader: &mut BufReader<std::os::unix::net::UnixStream>) {
    let mut byte = [0];
    match reader.read(&mut byte) {
        Ok(0) => {}
        Err(error)
            if matches!(
                error.kind(),
                std::io::ErrorKind::ConnectionReset | std::io::ErrorKind::NotConnected
            ) => {}
        other => panic!("failed provider query must close its socket: {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn malformed_oversized_and_silent_local_sources_fail_closed_and_close_their_sockets() {
    for (index, kind) in ["malformed", "oversized", "silent"].into_iter().enumerate() {
        let directory = tempfile::tempdir_in("/tmp").unwrap();
        let path = directory.path().join("netd.sock");
        let listener = scripted_listener(&path);
        let peer = std::thread::spawn(move || {
            let mut reader = receive_snapshot_request(listener);
            match kind {
                "malformed" => reader.get_mut().write_all(b"{bad-json}\n").unwrap(),
                "oversized" => {
                    let chunk = [b'x'; 8192];
                    for _ in 0..1025 {
                        if reader.get_mut().write_all(&chunk).is_err() {
                            break;
                        }
                    }
                }
                "silent" => {}
                _ => unreachable!(),
            }
            assert_closed(&mut reader);
        });
        let endpoint = disabled_endpoint([80 + index as u8; 32]).await;
        let robot = Robot::start(
            manager(false),
            path,
            &[PublicKey(*endpoint.id().as_bytes())],
        )
        .await;
        let mut desk = Desk::connect(&endpoint, &robot).await;
        assert_source_error(desk.request(&schema_request(TOPIC)).await, "local schema");
        bounded(
            "scripted peer exit",
            tokio::task::spawn_blocking(move || peer.join().unwrap()),
        )
        .await
        .unwrap();
        desk.close();
        robot.shutdown().await;
        bounded("desk close", endpoint.close()).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn revocation_while_local_provider_is_awaited_refuses_the_pending_schema() {
    let netd = LocalNetd::start();
    let mut registration = netd.client();
    registration
        .register_egress(
            &GatewayPlan {
                egress_policy: GatewayEgressPolicy::AllowAll,
                announce: vec![TOPIC.into()],
                ingress: vec![],
            },
            &serving(),
            None,
        )
        .unwrap();
    // Delay an actual gateway snapshot at the local transport boundary. The
    // signed revocation arrives only after the provider's real request is read.
    let snapshot = netd.plane.serving_schema_snapshot().unwrap();
    let directory = tempfile::tempdir_in("/tmp").unwrap();
    let path = directory.path().join("netd.sock");
    let listener = scripted_listener(&path);
    let (requested_tx, requested_rx) = tokio::sync::oneshot::channel();
    let (release_tx, release_rx) = std::sync::mpsc::channel();
    let peer = std::thread::spawn(move || {
        let mut reader = receive_snapshot_request(listener);
        requested_tx.send(()).unwrap();
        release_rx.recv_timeout(Duration::from_secs(10)).unwrap();
        let reply = serde_json::json!({"id":1,"serving_schema":snapshot});
        writeln!(reader.get_mut(), "{reply}").unwrap();
        assert_closed(&mut reader);
    });
    let a_endpoint = disabled_endpoint([74; 32]).await;
    let b_endpoint = disabled_endpoint([75; 32]).await;
    let a_key = PublicKey(*a_endpoint.id().as_bytes());
    let robot = Robot::start(
        Arc::clone(&netd.manager),
        path,
        &[a_key, PublicKey(*b_endpoint.id().as_bytes())],
    )
    .await;
    let mut a = Desk::connect(&a_endpoint, &robot).await;
    let mut b = Desk::connect(&b_endpoint, &robot).await;
    a.send(&schema_request(TOPIC)).await;
    bounded("provider request observed", requested_rx)
        .await
        .unwrap();
    assert_eq!(
        b.request(&revoke(a_key)).await,
        WireResponse::EpochSynced {
            epoch: 1,
            applied: true
        }
    );
    release_tx.send(()).unwrap();
    assert_source_error(a.receive().await, "device key is revoked");
    // Re-authorization rejects the next metadata query before contacting the
    // source again, even though this connection still has no demand to sweep.
    assert_source_error(
        a.request(&schema_request(TOPIC)).await,
        "device key is revoked",
    );
    bounded(
        "scripted peer exit",
        tokio::task::spawn_blocking(move || peer.join().unwrap()),
    )
    .await
    .unwrap();
    drop(registration);
    a.close();
    b.close();
    robot.shutdown().await;
    bounded("desk A close", a_endpoint.close()).await;
    bounded("desk B close", b_endpoint.close()).await;
    netd.shutdown();
}
