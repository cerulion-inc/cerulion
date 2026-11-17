// SPDX-License-Identifier: AGPL-3.0-only
//! An app that dials a desk with `connect`, read by the desk's zenoh peer,
//! which listens on loopback TCP and never dials the app.
//!
//! The desk role runs in a re-executed child process: one zenoh session per
//! process. For each run the child catalog-queries the app, sends one demand
//! GET per topic, keeps a demand token and a data subscriber on each topic
//! and counts the frames that arrive. The parent drives it through step files
//! in a shared directory, publishes, and asserts on the child's reports.

use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_android::EmbeddedRuntime;
use cerulion_core::transport::discovery::{query_robot_catalog, TopicToken};
use cerulion_core::transport::network::NetworkManager;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WIRE_HEADER_VERSION;
use cerulion_core::CatalogReply;
use serde_json::json;
use tempfile::tempdir;
use zenoh::Wait;

const ROBOT: &str = "lan_connect_test";
/// On the app's `egress` list.
const TOPIC: &str = "/lan_connect_test/state";
/// Served by the app, but not on its `egress` list.
const PRIVATE: &str = "/lan_connect_test/notes";

const DESK_DIR_ENV: &str = "CERULION_LAN_CONNECT_DESK_DIR";
const DESK_PORT_ENV: &str = "CERULION_LAN_CONNECT_DESK_PORT";

/// The runs the parent starts, in order, each with its own step files.
const RUNS: [&str; 2] = ["paired", "open"];

fn free_port() -> u16 {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local_addr").port()
}

/// The desk: a peer that listens on loopback, connects nowhere and does not
/// scout, so the only link it has is the one the app dials.
fn desk_listener(port: u16) -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", r#""peer""#).unwrap();
    config
        .insert_json5("listen/endpoints", &format!(r#"["tcp/127.0.0.1:{port}"]"#))
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().expect("open desk session")
}

/// Catalog-query the app until `done` holds. The deadline covers the app's
/// redial backoff: the app started before the desk listened.
fn catalog_until(session: &zenoh::Session, done: impl Fn(&CatalogReply) -> bool) -> CatalogReply {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let reply = query_robot_catalog(session, ROBOT, Duration::from_secs(1));
        match reply {
            Some(reply) if done(&reply) => return reply,
            other if Instant::now() >= deadline => panic!("no matching catalog reply: {other:?}"),
            _ => std::thread::sleep(Duration::from_millis(200)),
        }
    }
}

/// One desk-side demand GET for `topic`, as the demand verb's wire states
/// it: the key `cerulion_q/{robot}/demand{topic}` and a payload of the wire
/// header version, two bytes little-endian. Returns the ack the app replied.
fn demand_ack(session: &zenoh::Session, topic: &str) -> String {
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let replies = session
            .get(format!("cerulion_q/{ROBOT}/demand{topic}"))
            .payload(WIRE_HEADER_VERSION.to_le_bytes().to_vec())
            .timeout(Duration::from_secs(1))
            .wait()
            .expect("send demand GET");
        let mut acks = Vec::new();
        while let Ok(reply) = replies.recv() {
            if let Ok(sample) = reply.into_result() {
                acks.push(String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned());
            }
        }
        match acks.as_slice() {
            [ack] => return ack.clone(),
            [] if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(200)),
            other => panic!("expected one demand ack for '{topic}', got {other:?}"),
        }
    }
}

/// Count the frames that arrive on the data key of `topic`.
fn count_frames(
    session: &zenoh::Session,
    topic: &str,
) -> (zenoh::pubsub::Subscriber<()>, Arc<AtomicU64>) {
    let count = Arc::new(AtomicU64::new(0));
    let seen = Arc::clone(&count);
    let subscriber = session
        .declare_subscriber(NetworkManager::data_key(topic))
        .callback(move |_sample| {
            seen.fetch_add(1, Ordering::Relaxed);
        })
        .wait()
        .expect("subscribe to a data key");
    (subscriber, count)
}

fn signal_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(name)
}

fn touch(dir: &Path, name: &str) {
    fs::write(signal_path(dir, name), b"1").expect("write step file");
}

fn wait_for_step(dir: &Path, name: &str) {
    let deadline = Instant::now() + Duration::from_secs(90);
    while !signal_path(dir, name).exists() {
        assert!(
            Instant::now() < deadline,
            "desk step '{name}' never appeared"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// DESK ROLE: the listening peer. For each run in [`RUNS`]:
///
/// 1. Wait for `{run}.go`. Under `paired`, catalog-query until the refusal
///    returns; under `open`, until both topics are listed.
/// 2. Send one demand GET per topic, write the acks to `{run}.acks`, reset
///    the frame counts and write `{run}.probed`.
/// 3. Wait for `{run}.published`, then write the frame counts to
///    `{run}.frames` and write `{run}.done`.
///
/// The demand tokens and the data subscribers stay declared across runs, as
/// a desk that stays up while the app restarts.
#[test]
#[ignore = "desk role; spawned by the parent test"]
fn desk_role() {
    let dir = PathBuf::from(std::env::var_os(DESK_DIR_ENV).expect(DESK_DIR_ENV));
    let port: u16 = std::env::var(DESK_PORT_ENV)
        .expect(DESK_PORT_ENV)
        .parse()
        .expect("port");
    let session = desk_listener(port);
    let (_listed_sub, listed_frames) = count_frames(&session, TOPIC);
    let (_private_sub, private_frames) = count_frames(&session, PRIVATE);
    let _listed_demand = TopicToken::declare(&session, TOPIC).expect("demand the listed topic");
    let _private_demand = TopicToken::declare(&session, PRIVATE).expect("demand the private topic");

    for run in RUNS {
        wait_for_step(&dir, &format!("{run}.go"));
        if run == "paired" {
            let refusal = catalog_until(&session, |reply| reply.error.is_some());
            assert!(refusal.entries.is_empty(), "a paired app lists no topic");
        } else {
            // The catalog names every topic the app serves, listed or not.
            let served = catalog_until(&session, |reply| {
                reply.error.is_none()
                    && [TOPIC, PRIVATE]
                        .iter()
                        .all(|topic| reply.entries.iter().any(|entry| entry.topic == *topic))
            });
            assert!(served.error.is_none());
        }
        let acks = format!(
            "{TOPIC}={}\n{PRIVATE}={}\n",
            demand_ack(&session, TOPIC),
            demand_ack(&session, PRIVATE)
        );
        fs::write(signal_path(&dir, &format!("{run}.acks")), acks).expect("write acks");
        listed_frames.store(0, Ordering::Relaxed);
        private_frames.store(0, Ordering::Relaxed);
        touch(&dir, &format!("{run}.probed"));
        wait_for_step(&dir, &format!("{run}.published"));
        let frames = format!(
            "{TOPIC}={}\n{PRIVATE}={}\n",
            listed_frames.load(Ordering::Relaxed),
            private_frames.load(Ordering::Relaxed)
        );
        fs::write(signal_path(&dir, &format!("{run}.frames")), frames).expect("write frames");
        touch(&dir, &format!("{run}.done"));
    }
}

fn desk_command(dir: &Path, port: u16, log_path: &Path) -> Command {
    let stdout = File::create(log_path).expect("create desk log");
    let stderr = stdout.try_clone().expect("clone desk log");
    let mut command = Command::new(std::env::current_exe().expect("current test executable"));
    command
        .args([
            "--exact",
            "desk_role",
            "--include-ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(DESK_DIR_ENV, dir)
        .env(DESK_PORT_ENV, port.to_string())
        .stdout(Stdio::from(stdout))
        .stderr(Stdio::from(stderr));
    command
}

/// Stops and reaps the desk process if it is still running when the parent
/// returns or panics.
struct DeskProcess(Child);

impl Drop for DeskProcess {
    fn drop(&mut self) {
        if matches!(self.0.try_wait(), Ok(None)) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}

/// Wait for a desk step file, reporting the child's log when it exits early
/// instead.
fn await_step(child: &mut Child, dir: &Path, name: &str, log_path: &Path) {
    let deadline = Instant::now() + Duration::from_secs(90);
    loop {
        if signal_path(dir, name).exists() {
            return;
        }
        if let Some(status) = child.try_wait().expect("desk try_wait") {
            let output = fs::read_to_string(log_path).unwrap_or_default();
            panic!("desk exited before step '{name}' with {status}:\n{output}");
        }
        assert!(
            Instant::now() < deadline,
            "desk step '{name}' did not appear in time:\n{}",
            fs::read_to_string(log_path).unwrap_or_default()
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn read_report(dir: &Path, name: &str) -> String {
    fs::read_to_string(signal_path(dir, name)).expect("read desk report")
}

/// Wait until the bridge flag of the topic reads `expected`.
fn await_enabled(expected: bool) {
    let manager = TransportManager::get().expect("initialized app transport manager");
    let bridges = manager.bridge_manager();
    let deadline = Instant::now() + Duration::from_secs(10);
    while bridges.is_enabled(TOPIC).unwrap() != expected {
        assert!(
            Instant::now() < deadline,
            "the bridge flag never read {expected}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
fn an_app_that_dials_a_desk_serves_it_under_open_and_refuses_it_under_paired() {
    let temp = tempdir().unwrap();
    let desk_dir = tempdir().unwrap();
    let dir = desk_dir.path();
    let log_path = dir.join("desk.log");
    let port = free_port();
    let connect = format!("tcp/127.0.0.1:{port}");
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into()).unwrap();
    // No `listen` and the default `mdns`: the app opens no listener, so the
    // only link is the one it dials. Only `TOPIC` is on the `egress` list.
    let config = |lan_access: &str| {
        json!({
            "connect": [connect],
            "lan_access": lan_access,
            "egress": [TOPIC],
            "publish": [
                {"topic": TOPIC, "schema": "std_msgs/String"},
                {"topic": PRIVATE, "schema": "std_msgs/String"}
            ]
        })
        .to_string()
    };

    // The desk is not up yet: the start still succeeds, and the app dials
    // again until the desk listens.
    runtime.start(&config("paired")).unwrap();
    {
        let status = runtime.status_handle();
        let status = status.lock().unwrap();
        assert_eq!(status.state, cerulion_android::RuntimeState::Running);
        assert!(status.listen.is_empty());
        assert_eq!(status.lan_access.as_deref(), Some("paired"));
        assert!(
            !status.mdns_advertised,
            "a connect-only app raises no beacon"
        );
    }
    let mut desk = DeskProcess(
        desk_command(dir, port, &log_path)
            .spawn()
            .expect("spawn desk process"),
    );

    for run in RUNS {
        if run == "open" {
            runtime.stop().unwrap();
            runtime.start(&config("open")).unwrap();
        }
        touch(dir, &format!("{run}.go"));
        await_step(&mut desk.0, dir, &format!("{run}.probed"), &log_path);
        if run == "open" {
            await_enabled(true);
        }
        let deadline = Instant::now() + Duration::from_secs(1);
        while Instant::now() < deadline {
            for topic in [TOPIC, PRIVATE] {
                runtime
                    .publish(topic, r#"{"data":"frame"}"#)
                    .expect("publish a frame");
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        touch(dir, &format!("{run}.published"));
        await_step(&mut desk.0, dir, &format!("{run}.done"), &log_path);
    }

    // Paired: the catalog refused (checked by the desk), every demand is
    // refused and no frame left the app.
    assert_eq!(
        read_report(dir, "paired.acks"),
        "/lan_connect_test/state=refused\n/lan_connect_test/notes=refused\n"
    );
    assert_eq!(
        read_report(dir, "paired.frames"),
        "/lan_connect_test/state=0\n/lan_connect_test/notes=0\n"
    );
    // Open: the catalog lists both topics; the listed one is granted and its
    // frames reach the desk, the other is refused and sends nothing.
    assert_eq!(
        read_report(dir, "open.acks"),
        "/lan_connect_test/state=enabled/w2\n/lan_connect_test/notes=refused\n"
    );
    let frames = read_report(dir, "open.frames");
    let count = |topic: &str| -> u64 {
        frames
            .lines()
            .find_map(|line| line.strip_prefix(&format!("{topic}=")))
            .expect("a frame count per topic")
            .parse()
            .expect("a numeric frame count")
    };
    assert!(
        count(TOPIC) >= 1,
        "the listed topic must reach the desk:\n{frames}"
    );
    assert_eq!(
        count(PRIVATE),
        0,
        "an unlisted topic must not leave:\n{frames}"
    );
    runtime.stop().unwrap();
    let status = desk.0.wait().expect("reap desk");
    assert!(
        status.success(),
        "desk failed with {status}:\n{}",
        fs::read_to_string(&log_path).unwrap_or_default()
    );

    // Connect locators are fixed for the process lifetime, as listen
    // endpoints are.
    for other in [
        json!({"connect": [format!("tcp/127.0.0.2:{port}")]}),
        json!({"connect": [connect.clone(), "tcp/127.0.0.1:9"]}),
        json!({}),
    ] {
        let error = runtime
            .start(&other.to_string())
            .expect_err("a start with other connect locators");
        assert_eq!(error.code(), cerulion_android::codes::BAD_STATE);
        assert_eq!(
            error.to_string(),
            "network connect locators are fixed for the process lifetime by the first start; restart the app process to change them"
        );
    }
}
