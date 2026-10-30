// SPDX-License-Identifier: AGPL-3.0-only
//! The remote plane hosted in an embedded runtime, read by a paired desk.
//!
//! The runtime registers [`cerulion_android_remote::Host`] through
//! `embed_nodes!`, as an app runtime library does, and the desk is a real
//! iroh endpoint on loopback with the relay disabled. Keys come from fixed
//! seeds; every expected value is a hand-written literal. The host registry
//! and the transport are process state, so this file is its own binary and
//! holds ONE test that runs its steps in order. Run it with
//! `--features local-transport`, the transport an app runs on.

use std::future::Future;
use std::net::{Ipv4Addr, Ipv6Addr, SocketAddr};
use std::path::Path;
use std::time::Duration;

use cerulion_android::{codes, EmbeddedRuntime, RuntimeState};
use cerulion_core::wire::WireHeader;
use cerulion_link::{
    accept_uni_frame_stream, alpn, build_endpoint, dial, direct_addr, open_frame_stream,
    read_frame, write_frame, Connection, Endpoint, EndpointAddr, EndpointConfig, EndpointId,
    RecvStream, RelayConfig, SendStream, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::{DeviceAccountIndex, StreamPreamble, WireResponse};
use ed25519_dalek::SigningKey;
use serde_json::json;
use tempfile::tempdir;

cerulion_android::embed_nodes! {
    remote: cerulion_android_remote::Host,
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const ROBOT: &str = "hosted_plane_test";
const SCHEMA: &str = "std_msgs/Int32";
/// The app's device key seed; its public half is the endpoint id.
const DEVICE_SEED: u8 = 81;
/// A desk the owner paired.
const DESK_SEED: u8 = 82;
/// A key nobody paired.
const STRANGER_SEED: u8 = 83;
const OWNER: AccountId = AccountId([10; 32]);
const ROBOT_ID: RobotId = RobotId([5; 32]);
const CHASSIS: &[u8] = b"hosted-plane-test-chassis-secret";
const MAC_KEY: &[u8] = b"hosted-plane-test-trust-store-mac-key";
/// The clock the owner claim is made at, in nanoseconds.
const CLAIMED_AT: u64 = 1_000_000_000_000;
const STEP: Duration = Duration::from_secs(20);

fn public(seed: u8) -> PublicKey {
    PublicKey(
        SigningKey::from_bytes(&[seed; 32])
            .verifying_key()
            .to_bytes(),
    )
}

/// What a provisioning step leaves under `<state_root>/remoted/`: the device
/// key, the MAC key, a claimed trust store bound to the device key and a side
/// map that pairs the desk with the owner.
fn provision(state_root: &Path) {
    let remoted = state_root.join("remoted");
    std::fs::create_dir_all(&remoted).unwrap();
    std::fs::write(remoted.join("device_key"), [DEVICE_SEED; 32]).unwrap();
    std::fs::write(remoted.join("trust_store.mac_key"), MAC_KEY).unwrap();
    let mut store = TrustStore::provision(
        ROBOT_ID,
        public(DEVICE_SEED),
        RootSet::new(vec![public(1)], 1).unwrap(),
        CHASSIS,
        0,
    )
    .unwrap()
    .with_path(remoted.join("trust_store"));
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, CLAIMED_AT)
        .unwrap();
    store.save(MAC_KEY).unwrap();
    let mut index = DeviceAccountIndex::new().with_path(remoted.join("device_index.json"));
    index.bind(public(DESK_SEED), OWNER);
    index.save(MAC_KEY).unwrap();
}

fn config(topic: &str, relay: serde_json::Value) -> String {
    json!({
        "publish": [{"topic": topic, "schema": SCHEMA}],
        "remote": {"enabled": true, "relay": relay, "state_dir": "plane"},
    })
    .to_string()
}

fn status(runtime: &EmbeddedRuntime) -> cerulion_android::Status {
    runtime.status_handle().lock().unwrap().clone()
}

/// Every file under `root`, relative and sorted.
fn files_under(root: &Path) -> Vec<String> {
    let mut found = Vec::new();
    let mut pending = vec![root.to_path_buf()];
    while let Some(dir) = pending.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries {
            let path = entry.unwrap().path();
            if path.is_dir() {
                pending.push(path);
            } else {
                found.push(path.strip_prefix(root).unwrap().display().to_string());
            }
        }
    }
    found.sort();
    found
}

async fn bounded<F: Future>(what: &str, future: F) -> F::Output {
    tokio::time::timeout(STEP, future)
        .await
        .unwrap_or_else(|_| panic!("{what} took longer than {STEP:?}"))
}

async fn desk(seed: u8) -> Endpoint {
    bounded(
        "desk endpoint",
        build_endpoint(EndpointConfig::new([seed; 32]).with_relay(RelayConfig::Disabled)),
    )
    .await
    .unwrap()
}

/// The plane's address with every unspecified socket made loopback.
fn plane_address(endpoint_id: &str, sockets: &[String]) -> EndpointAddr {
    let id: EndpointId = endpoint_id.parse().expect("an endpoint id");
    let sockets = sockets.iter().map(|socket| {
        match socket.parse::<SocketAddr>().expect("a socket address") {
            SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
                SocketAddr::from((Ipv4Addr::LOCALHOST, v4.port()))
            }
            SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
                SocketAddr::from((Ipv6Addr::LOCALHOST, v6.port()))
            }
            other => other,
        }
    });
    direct_addr(id, sockets)
}

struct Control {
    connection: Connection,
    send: SendStream,
    recv: RecvStream,
}

impl Control {
    async fn open(desk: &Endpoint, address: EndpointAddr) -> Self {
        let connection = bounded("dial", dial(desk, address, alpn::WIRE))
            .await
            .expect("the paired desk dials");
        let (send, recv) = bounded("control stream", open_frame_stream(&connection))
            .await
            .expect("control stream");
        Control {
            connection,
            send,
            recv,
        }
    }

    async fn request(&mut self, request: serde_json::Value) -> Vec<u8> {
        let bytes = serde_json::to_vec(&request).unwrap();
        bounded("send request", write_frame(&mut self.send, &bytes))
            .await
            .unwrap();
        bounded(
            "read reply",
            read_frame(&mut self.recv, DEFAULT_MAX_FRAME_LEN),
        )
        .await
        .unwrap()
    }
}

fn reply(bytes: &[u8]) -> WireResponse {
    serde_json::from_slice(bytes)
        .unwrap_or_else(|error| panic!("{error}: {}", String::from_utf8_lossy(bytes)))
}

#[test]
fn a_paired_desk_reads_the_hosted_catalog_and_a_topic_and_a_fresh_install_serves_nothing() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let files = tempdir().unwrap();
    let state_root = files.path().join("plane");
    let topic = format!("/hosted_{}/reading", std::process::id());
    let mut runtime = EmbeddedRuntime::new(files.path().to_path_buf(), ROBOT.into()).unwrap();

    // A fresh install has no keys: the start fails as not provisioned, names
    // the device key, binds nothing and writes nothing.
    let error = runtime
        .start(&config(&topic, json!("disabled")))
        .expect_err("a fresh install fails closed");
    assert_eq!(error.code(), codes::REMOTE_NOT_PROVISIONED, "{error}");
    assert!(error.to_string().contains("device key"), "{error}");
    let failed = status(&runtime);
    assert_eq!(failed.state, RuntimeState::Failed);
    assert_eq!(failed.remote, None);
    assert!(!state_root.exists(), "nothing is written for the plane");

    // A relay URL that does not parse fails before any key is read.
    let error = runtime
        .start(&config(&topic, json!(["not a relay url"])))
        .expect_err("a bad relay URL fails the start");
    assert_eq!(error.code(), codes::REMOTE, "{error}");
    assert!(error.to_string().contains("'not a relay url'"), "{error}");
    assert!(!state_root.exists(), "nothing is written for the plane");

    provision(&state_root);
    let provisioned = files_under(&state_root);
    runtime
        .start(&config(&topic, json!("disabled")))
        .expect("a provisioned plane starts");
    let running = status(&runtime);
    assert_eq!(running.state, RuntimeState::Running);
    let remote = running.remote.expect("the plane is reported");
    let device = public(DEVICE_SEED).0;
    let device_hex: String = device.iter().map(|byte| format!("{byte:02x}")).collect();
    assert_eq!(remote.endpoint_id, device_hex);
    assert!(!remote.bound_sockets.is_empty(), "{remote:?}");
    let address = plane_address(&remote.endpoint_id, &remote.bound_sockets);

    let desk_runtime = tokio::runtime::Runtime::new().unwrap();
    let paired = desk_runtime.block_on(desk(DESK_SEED));
    let mut control = desk_runtime.block_on(Control::open(&paired, address.clone()));

    // The catalog names the app topic with its schema.
    let WireResponse::Catalog(catalog) =
        reply(&desk_runtime.block_on(control.request(json!({"verb": "catalog"}))))
    else {
        panic!("expected a catalog");
    };
    assert_eq!(catalog.robot, ROBOT);
    assert_eq!(catalog.error, None);
    let rows: Vec<_> = catalog
        .entries
        .iter()
        .filter(|row| row.topic == topic)
        .map(|row| row.schema_name.clone())
        .collect();
    assert_eq!(rows, [Some(SCHEMA.to_string())], "{:?}", catalog.entries);

    // A demanded topic streams the frame the app publishes.
    assert_eq!(
        reply(&desk_runtime.block_on(control.request(json!({"verb": "demand", "topic": topic})))),
        WireResponse::DemandAccepted {
            topic: topic.clone()
        }
    );
    let mut stream = desk_runtime
        .block_on(bounded(
            "data stream",
            accept_uni_frame_stream(&control.connection),
        ))
        .expect("the data stream opens");
    let preamble: StreamPreamble = serde_json::from_slice(
        &desk_runtime
            .block_on(bounded(
                "preamble",
                read_frame(&mut stream, DEFAULT_MAX_FRAME_LEN),
            ))
            .unwrap(),
    )
    .unwrap();
    assert_eq!(preamble.topic, topic);
    let sequence = runtime.publish(&topic, r#"{"data": 7}"#).expect("publish");
    let frame = desk_runtime
        .block_on(bounded(
            "frame",
            read_frame(&mut stream, DEFAULT_MAX_FRAME_LEN),
        ))
        .unwrap();
    assert_eq!(sequence, 0);
    assert_eq!(frame.len(), WireHeader::SIZE + 4, "{frame:?}");
    assert_eq!(&frame[WireHeader::SIZE..], 7i32.to_le_bytes());

    // A desk cannot write into the app: the plane has no publish verb.
    match reply(
        &desk_runtime
            .block_on(control.request(json!({"verb": "publish", "topic": topic, "frame": "00"}))),
    ) {
        WireResponse::Error { message, .. } => assert!(
            message.contains("malformed wire control request"),
            "{message}"
        ),
        other => panic!("a publish request must be refused, got {other:?}"),
    }
    control.connection.close(0u32.into(), b"done");

    // An unpaired key is refused before any request is served.
    let stranger = desk_runtime.block_on(desk(STRANGER_SEED));
    let refusal = desk_runtime
        .block_on(async {
            let connection = bounded(
                "stranger dial",
                dial(&stranger, address.clone(), alpn::WIRE),
            )
            .await
            .ok()?;
            let (mut send, mut recv) = bounded("stranger stream", open_frame_stream(&connection))
                .await
                .ok()?;
            let bytes = serde_json::to_vec(&json!({"verb": "catalog"})).unwrap();
            write_frame(&mut send, &bytes).await.ok()?;
            let answer = bounded(
                "stranger reply",
                read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN),
            )
            .await
            .ok()?;
            serde_json::from_slice::<serde_json::Value>(&answer).ok()
        })
        .expect("the plane answers an unpaired key with a refusal");
    assert_eq!(refusal["decision"], "refuse", "{refusal}");
    assert_eq!(refusal["admission_code"], "unpaired", "{refusal}");

    // Stop closes the plane: status withdraws it and a desk cannot dial it.
    runtime.stop().expect("stop");
    let stopped = status(&runtime);
    assert_eq!(stopped.state, RuntimeState::Stopped);
    assert_eq!(stopped.remote, None);
    assert_eq!(stopped.last_error, None, "a stop is not a plane failure");
    let after = desk_runtime.block_on(async {
        tokio::time::timeout(
            Duration::from_secs(5),
            dial(&paired, address.clone(), alpn::WIRE),
        )
        .await
    });
    assert!(
        !matches!(after, Ok(Ok(_))),
        "a stopped plane accepted a dial"
    );
    // The plane writes its own files beside the provisioned ones (the
    // public endpoint facts and the receipt log) and never a key.
    let written: Vec<String> = files_under(&state_root)
        .into_iter()
        .filter(|file| !provisioned.contains(file))
        .collect();
    assert!(
        written.iter().all(|file| !file.contains("key")),
        "{written:?}"
    );

    // A second start binds a fresh plane on the same keys, and the paired
    // desk reads it again.
    runtime
        .start(&config(&topic, json!("disabled")))
        .expect("the plane starts again");
    let again = status(&runtime)
        .remote
        .expect("the plane is reported again");
    assert_eq!(again.endpoint_id, device_hex);
    let address = plane_address(&again.endpoint_id, &again.bound_sockets);
    let mut control = desk_runtime.block_on(Control::open(&paired, address));
    let WireResponse::Catalog(catalog) =
        reply(&desk_runtime.block_on(control.request(json!({"verb": "catalog"}))))
    else {
        panic!("expected a catalog from the second plane");
    };
    assert!(catalog.entries.iter().any(|row| row.topic == topic));
    control.connection.close(0u32.into(), b"done");
    runtime.stop().expect("the second stop");
    assert_eq!(status(&runtime).last_error, None);

    // A plane that cannot open its receipt log binds, then stops serving on
    // its own: the run fails with the remote code and withdraws the plane.
    let receipts = state_root.join("remoted").join("receipts.log");
    std::fs::remove_file(&receipts).expect("the plane wrote its receipt log");
    std::fs::create_dir(&receipts).unwrap();
    runtime
        .start(&config(&topic, json!("disabled")))
        .expect("the plane binds before it opens the receipt log");
    let deadline = std::time::Instant::now() + STEP;
    while status(&runtime).state != RuntimeState::Failed {
        assert!(
            std::time::Instant::now() < deadline,
            "the plane ends on its own: {:?}",
            status(&runtime)
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    let ended = status(&runtime);
    assert_eq!(ended.remote, None);
    let last = ended.last_error.expect("the end is recorded");
    assert_eq!(last.code, codes::REMOTE);
    assert!(
        last.message
            .starts_with("the remote plane stopped serving: ops plane: "),
        "{last:?}"
    );
    runtime.stop().expect("a stop after the plane ended");
    std::fs::remove_dir(&receipts).unwrap();
}
