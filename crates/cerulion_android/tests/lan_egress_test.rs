// SPDX-License-Identifier: AGPL-3.0-only
//! The app gateway's `egress` list, read by a real zenoh peer over TCP.
//!
//! The peer role runs in a re-executed child process: one zenoh session per
//! process. For each run the child catalog-queries the app, sends one demand
//! GET per topic, keeps a demand token and a data subscriber on each topic
//! and counts the frames that arrive. The parent drives it through step
//! files in a shared directory, publishes, and asserts on the child's
//! reports and on the bridge manager, which is in-process state rather than
//! a second session.

use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_android::EmbeddedRuntime;
use cerulion_core::transport::discovery::{query_robot_catalog, TopicToken};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WIRE_HEADER_VERSION;
use cerulion_core::CatalogReply;
use serde_json::json;
use tempfile::tempdir;
use zenoh::Wait;

const ROBOT: &str = "lan_egress_test";
const SHARED: &str = "/lan_egress_test/shared";
const PRIVATE: &str = "/lan_egress_test/private";

const OBS_DIR_ENV: &str = "CERULION_LAN_EGRESS_OBS_DIR";
const OBS_PORT_ENV: &str = "CERULION_LAN_EGRESS_OBS_PORT";

/// The runs the parent starts, in order, each with its own step files.
const RUNS: [&str; 4] = ["paired", "allow_list", "deny_all", "absent"];

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

fn signal_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(name)
}

fn touch(dir: &Path, name: &str) {
    fs::write(signal_path(dir, name), b"1").expect("write step file");
}

fn wait_for_step(dir: &Path, name: &str) {
    let deadline = Instant::now() + Duration::from_secs(60);
    while !signal_path(dir, name).exists() {
        assert!(
            Instant::now() < deadline,
            "observer step '{name}' never appeared"
        );
        std::thread::sleep(Duration::from_millis(20));
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
        .declare_subscriber(cerulion_core::transport::network::NetworkManager::data_key(
            topic,
        ))
        .callback(move |_sample| {
            seen.fetch_add(1, Ordering::Relaxed);
        })
        .wait()
        .expect("subscribe to a data key");
    (subscriber, count)
}

/// Send one demand GET per topic and write the acks to `{run}.acks`.
fn write_acks(dir: &Path, run: &str, session: &zenoh::Session) {
    let acks = format!(
        "{SHARED}={}\n{PRIVATE}={}\n",
        demand_ack(session, SHARED),
        demand_ack(session, PRIVATE)
    );
    fs::write(signal_path(dir, &format!("{run}.acks")), acks).expect("write acks");
}

/// OBSERVER ROLE: the zenoh peer. For each run in [`RUNS`]:
///
/// 1. Wait for `{run}.go`. Under `paired`, check the catalog refusal, send
///    one demand GET per topic, write the acks to `{run}.acks` and write
///    `{run}.done`.
/// 2. Otherwise catalog-query until both topics are listed, send one demand
///    GET per topic and write the acks to `{run}.acks`, then write
///    `{run}.probed`.
/// 3. Reset the frame counts, wait for `{run}.published`, and write the
///    counts to `{run}.frames` and `{run}.done`.
///
/// The demand tokens and data subscribers stay declared across runs, as a
/// desk that stays connected while the app restarts.
#[test]
#[ignore = "observer role; spawned by the parent test"]
fn observer_role() {
    let dir = PathBuf::from(std::env::var_os(OBS_DIR_ENV).expect(OBS_DIR_ENV));
    let port: u16 = std::env::var(OBS_PORT_ENV)
        .expect(OBS_PORT_ENV)
        .parse()
        .expect("port");
    let session = client(port);
    let (_shared_sub, shared_frames) = count_frames(&session, SHARED);
    let (_private_sub, private_frames) = count_frames(&session, PRIVATE);
    let _shared_demand = TopicToken::declare(&session, SHARED).expect("demand the shared topic");
    let _private_demand = TopicToken::declare(&session, PRIVATE).expect("demand the private topic");

    for run in RUNS {
        wait_for_step(&dir, &format!("{run}.go"));
        if run == "paired" {
            let refusal = catalog_until(&session, |reply| reply.error.is_some());
            assert!(refusal.entries.is_empty(), "a paired app lists no topic");
            write_acks(&dir, run, &session);
            touch(&dir, &format!("{run}.done"));
            continue;
        }
        let served = catalog_until(&session, |reply| {
            reply.error.is_none()
                && [SHARED, PRIVATE]
                    .iter()
                    .all(|topic| reply.entries.iter().any(|entry| entry.topic == *topic))
        });
        assert!(served.error.is_none());
        write_acks(&dir, run, &session);
        shared_frames.store(0, Ordering::Relaxed);
        private_frames.store(0, Ordering::Relaxed);
        touch(&dir, &format!("{run}.probed"));
        wait_for_step(&dir, &format!("{run}.published"));
        let frames = format!(
            "{SHARED}={}\n{PRIVATE}={}\n",
            shared_frames.load(Ordering::Relaxed),
            private_frames.load(Ordering::Relaxed)
        );
        fs::write(signal_path(&dir, &format!("{run}.frames")), frames).expect("write frames");
        touch(&dir, &format!("{run}.done"));
    }
}

fn observer_command(dir: &Path, port: u16, log_path: &Path) -> Command {
    let stdout = File::create(log_path).expect("create observer log");
    let stderr = stdout.try_clone().expect("clone observer log");
    let mut command = Command::new(std::env::current_exe().expect("current test executable"));
    command
        .args([
            "--exact",
            "observer_role",
            "--include-ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(OBS_DIR_ENV, dir)
        .env(OBS_PORT_ENV, port.to_string())
        .stdout(Stdio::from(stdout))
        .stderr(Stdio::from(stderr));
    command
}

/// Stops and reaps the observer process if it is still running when the
/// parent returns or panics.
struct ObserverProcess(Child);

impl Drop for ObserverProcess {
    fn drop(&mut self) {
        if matches!(self.0.try_wait(), Ok(None)) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}

/// Wait for an observer step file, reporting the child's log when it exits
/// early instead.
fn await_step(child: &mut Child, dir: &Path, name: &str, log_path: &Path) {
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if signal_path(dir, name).exists() {
            return;
        }
        if let Some(status) = child.try_wait().expect("observer try_wait") {
            let output = fs::read_to_string(log_path).unwrap_or_default();
            panic!("observer exited before step '{name}' with {status}:\n{output}");
        }
        assert!(
            Instant::now() < deadline,
            "observer step '{name}' did not appear in time:\n{}",
            fs::read_to_string(log_path).unwrap_or_default()
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn read_report(dir: &Path, name: &str) -> String {
    fs::read_to_string(signal_path(dir, name)).expect("read observer report")
}

/// Publish one frame on each topic every 50 ms for `window`.
fn publish_both(runtime: &mut EmbeddedRuntime, window: Duration) {
    let deadline = Instant::now() + window;
    while Instant::now() < deadline {
        for topic in [SHARED, PRIVATE] {
            runtime
                .publish(topic, r#"{"data":"frame"}"#)
                .expect("publish a frame");
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Wait until the bridge flag of `topic` reads `expected`.
fn await_enabled(topic: &str, expected: bool) {
    let manager = TransportManager::get().expect("initialized app transport manager");
    let bridges = manager.bridge_manager();
    let deadline = Instant::now() + Duration::from_secs(10);
    while bridges.is_enabled(topic).unwrap() != expected {
        assert!(
            Instant::now() < deadline,
            "the bridge flag of '{topic}' never read {expected}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
fn an_egress_list_serves_only_its_topics_and_refuses_the_rest() {
    let temp = tempdir().unwrap();
    let obs_dir = tempdir().unwrap();
    let dir = obs_dir.path();
    let log_path = dir.join("observer.log");
    let port = free_port();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into()).unwrap();
    let config = |extra: serde_json::Value| {
        let mut config = json!({
            "listen": [format!("tcp/127.0.0.1:{port}")],
            "mdns": false,
            "publish": [
                {"topic": SHARED, "schema": "std_msgs/String"},
                {"topic": PRIVATE, "schema": "std_msgs/String"}
            ]
        });
        config
            .as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        config.to_string()
    };

    // A listed topic the app never sends is a start error: a misspelled
    // name would otherwise leave the topic it meant unsent with no sign.
    let error = runtime
        .start(&config(json!({
            "lan_access": "open",
            "egress": [SHARED, "/lan_egress_test/shraed"]
        })))
        .expect_err("an egress topic must be a graph output or a publish topic");
    assert_eq!(error.code(), cerulion_android::codes::INVALID_ARGUMENT);
    assert_eq!(
        error.to_string(),
        "egress topic '/lan_egress_test/shraed' is neither a graph output nor a publish topic"
    );

    // Paired with a list: the list changes nothing, the catalog refuses.
    runtime.start(&config(json!({"egress": [SHARED]}))).unwrap();
    // Spawned only after the runtime binds its listen endpoint; the
    // child's client session connects once and stays open across restarts.
    let mut observer = ObserverProcess(
        observer_command(dir, port, &log_path)
            .spawn()
            .expect("spawn observer process"),
    );
    touch(dir, "paired.go");
    await_step(&mut observer.0, dir, "paired.done", &log_path);
    assert_eq!(
        read_report(dir, "paired.acks"),
        "/lan_egress_test/shared=refused\n/lan_egress_test/private=refused\n"
    );
    let manager = TransportManager::get().expect("initialized app transport manager");
    let bridges = manager.bridge_manager();
    // The observer's demand tokens are live: a paired app grants neither
    // topic, and the closed LAN refuses before the list is ever read.
    for topic in [SHARED, PRIVATE] {
        assert!(!bridges.is_enabled(topic).unwrap(), "paired: '{topic}'");
        assert!(
            !bridges.was_egress_refused(topic).unwrap(),
            "paired: '{topic}'"
        );
    }
    runtime.stop().unwrap();

    for (run, extra, acks, enabled) in [
        (
            "allow_list",
            json!({"lan_access": "open", "egress": [SHARED]}),
            "/lan_egress_test/shared=enabled/w2\n/lan_egress_test/private=refused\n",
            (true, false),
        ),
        (
            "deny_all",
            json!({"lan_access": "open", "egress": []}),
            "/lan_egress_test/shared=refused\n/lan_egress_test/private=refused\n",
            (false, false),
        ),
        (
            "absent",
            json!({"lan_access": "open"}),
            "/lan_egress_test/shared=enabled/w2\n/lan_egress_test/private=enabled/w2\n",
            (true, true),
        ),
    ] {
        runtime.start(&config(extra)).unwrap();
        touch(dir, &format!("{run}.go"));
        await_step(&mut observer.0, dir, &format!("{run}.probed"), &log_path);
        assert_eq!(read_report(dir, &format!("{run}.acks")), acks, "{run}");
        await_enabled(SHARED, enabled.0);
        await_enabled(PRIVATE, enabled.1);
        // A refused demand is recorded against the topic it named.
        assert_eq!(
            bridges.was_egress_refused(SHARED).unwrap(),
            !enabled.0,
            "{run}"
        );
        assert_eq!(
            bridges.was_egress_refused(PRIVATE).unwrap(),
            !enabled.1,
            "{run}"
        );
        publish_both(&mut runtime, Duration::from_secs(2));
        touch(dir, &format!("{run}.published"));
        await_step(&mut observer.0, dir, &format!("{run}.done"), &log_path);
        let frames = read_report(dir, &format!("{run}.frames"));
        for (topic, served) in [(SHARED, enabled.0), (PRIVATE, enabled.1)] {
            let count: u64 = frames
                .lines()
                .find_map(|line| line.strip_prefix(&format!("{topic}=")))
                .expect("a frame count per topic")
                .parse()
                .expect("a numeric frame count");
            if served {
                assert!(count > 0, "{run}: '{topic}' must deliver frames:\n{frames}");
            } else {
                assert_eq!(count, 0, "{run}: '{topic}' must deliver nothing:\n{frames}");
            }
        }
        runtime.stop().unwrap();
    }

    let status = observer.0.wait().expect("reap observer");
    assert!(
        status.success(),
        "observer failed with {status}:\n{}",
        fs::read_to_string(&log_path).unwrap_or_default()
    );
}
