// SPDX-License-Identifier: AGPL-3.0-only
//! Real vizd control requests must preserve account refusals without LAN fallback.
//! Each daemon has isolated shared memory, no network configuration, a memory
//! render sink, and an injected refusing query plane. Run this binary serially.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::{CatalogReply, SchemaReply};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;
use cerulion_vizd::{start_with_demand_plane, DemandPlane, DEFAULT_POLL_INTERVAL};
use serde_json::{json, Value};

const CATALOG_REFUSAL: &str = "account catalog authorization revoked";
const SCHEMA_REFUSAL: &str = "account schema authorization revoked";
const ROUTE_REFUSAL: &str = "account robot route requires 64 lowercase hexadecimal digits";

#[derive(Default)]
struct RefusingPlane {
    catalogs: AtomicUsize,
    schemas: AtomicUsize,
    demands: AtomicUsize,
    releases: AtomicUsize,
}

impl DemandPlane for RefusingPlane {
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        self.demands.fetch_add(1, Ordering::SeqCst);
        Err("test forbids creating a demand".into())
    }

    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        self.releases.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn query_catalog(&self, _robot: &str) -> Result<Vec<CatalogReply>, String> {
        self.catalogs.fetch_add(1, Ordering::SeqCst);
        Err(CATALOG_REFUSAL.into())
    }

    fn query_catalog_all(&self) -> Result<Vec<CatalogReply>, String> {
        panic!("an explicit robot must not fall back to a robot-less catalog query")
    }

    fn query_schema(&self, _robot: &str, _requested: &str) -> Result<Vec<SchemaReply>, String> {
        self.schemas.fetch_add(1, Ordering::SeqCst);
        Err(SCHEMA_REFUSAL.into())
    }
}

impl RefusingPlane {
    fn assert_calls(&self, catalogs: usize, schemas: usize) {
        assert_eq!(self.catalogs.load(Ordering::SeqCst), catalogs);
        assert_eq!(self.schemas.load(Ordering::SeqCst), schemas);
        assert_eq!(self.demands.load(Ordering::SeqCst), 0);
        assert_eq!(self.releases.load(Ordering::SeqCst), 0);
    }
}

struct Client {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
}

impl Client {
    fn connect(socket: &std::path::Path) -> Self {
        let writer = UnixStream::connect(socket).expect("connect test daemon");
        writer
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        writer
            .set_write_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        let mut reader = BufReader::new(writer.try_clone().unwrap());
        let mut hello = String::new();
        reader.read_line(&mut hello).expect("read hello");
        let _: Value = serde_json::from_str(&hello).expect("hello JSON");
        Self { writer, reader }
    }

    fn request(&mut self, request: Value) -> Value {
        writeln!(self.writer, "{request}").expect("write request");
        let mut reply = String::new();
        assert_ne!(self.reader.read_line(&mut reply).unwrap(), 0);
        serde_json::from_str(&reply).expect("reply JSON")
    }

    fn attach(&mut self, robot: &str, schema: Option<&str>) -> Value {
        self.request(json!({
            "id": 17,
            "method": "attach",
            "topic": "/account_test/vector",
            "robot": robot,
            "schema": schema,
        }))
    }
}

struct TestDirectory(PathBuf);

impl Drop for TestDirectory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn with_daemon(test: impl FnOnce(&mut Client, &RefusingPlane)) {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let suffix = format!(
        "{}_{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    );
    let directory = TestDirectory(PathBuf::from(format!("/tmp/cer-viz-account-{suffix}")));
    std::fs::create_dir(&directory.0).expect("create unique test directory");
    let socket = directory.0.join("vizd.sock");
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("account_viz_{suffix}"),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated transport");
    let (recording, _storage) = rerun::RecordingStreamBuilder::new("account_route_test")
        .recording_id(suffix)
        .memory()
        .expect("memory sink");
    let worker = VizLogWorker::spawn(recording, builtin_walker(), SinkState::new()).unwrap();
    let plane = Arc::new(RefusingPlane::default());
    let mut daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        manager,
        worker,
        builtin_walker(),
        None,
        plane.clone(),
    )
    .expect("start isolated daemon");
    let mut client = Client::connect(&socket);
    test(&mut client, &plane);
    drop(client);
    daemon.shutdown();
}

fn assert_refusal(reply: &Value, error: &str) {
    assert_eq!(reply["id"], 17);
    assert_eq!(reply["ok"], false);
    assert_eq!(reply["error"], error);
    assert_eq!(reply["topic"], "/account_test/vector");
    assert!(
        reply.get("retry").is_none(),
        "refusal must not invite retry: {reply}"
    );
}

#[test]
fn account_catalog_and_schema_refusals_survive_the_real_control_socket() {
    with_daemon(|client, plane| {
        let robot = cerulion_netd::account_access::robot_route(&[0xab; 32]);
        for _ in 0..2 {
            assert_refusal(&client.attach(&robot, None), CATALOG_REFUSAL);
            assert_refusal(
                &client.attach(&robot, Some("account_test/RemoteOnly")),
                SCHEMA_REFUSAL,
            );
        }
        plane.assert_calls(2, 2);
        let listed = client.request(json!({ "id": 18, "method": "list" }));
        assert_eq!(listed["ok"], true);
        assert_eq!(listed["attached"], json!([]));
    });
}

#[test]
fn malformed_reserved_routes_are_refused_even_with_a_known_pinned_schema() {
    with_daemon(|client, plane| {
        let valid = cerulion_netd::account_access::robot_route(&[0xab; 32]);
        let malformed = [
            "account:".to_string(),
            "account:bad".to_string(),
            format!(" {valid}"),
            format!("{valid}\t"),
            format!("account:{}", "AB".repeat(32)),
            format!("account:{}g0", "ab".repeat(31)),
        ];
        for robot in malformed {
            for schema in [
                None,
                Some("geometry_msgs/Vector3"),
                Some("account_test/RemoteOnly"),
            ] {
                assert_refusal(&client.attach(&robot, schema), ROUTE_REFUSAL);
            }
        }
        plane.assert_calls(0, 0);
    });
}

#[test]
fn ordinary_lan_names_keep_the_existing_transient_fallback() {
    with_daemon(|client, plane| {
        for schema in [None, Some("account_test/RemoteOnly")] {
            let reply = client.attach("ordinary-robot", schema);
            assert_eq!(reply["ok"], false);
            assert!(reply["error"]
                .as_str()
                .unwrap()
                .contains("not network-configured"));
        }
        plane.assert_calls(1, 1);
    });
}
