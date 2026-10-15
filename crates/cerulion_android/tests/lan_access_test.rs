// SPDX-License-Identifier: AGPL-3.0-only
//! The app gateway's LAN access posture, read by a real zenoh peer over TCP.
//!
//! The peer role runs in a re-executed child process: one zenoh session per
//! process. The child opens the client session, performs every catalog
//! query and declares the demand token; the parent drives it through step
//! files in a shared directory and asserts on the bridge manager, which is
//! in-process state rather than a second session.

use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_android::EmbeddedRuntime;
use cerulion_core::transport::discovery::{query_robot_catalog, TopicToken};
use cerulion_core::transport::TransportManager;
use cerulion_core::{CatalogReply, LanAccess};
use serde_json::json;
use tempfile::tempdir;
use zenoh::Wait;

const ROBOT: &str = "lan_access_test";

const OBS_DIR_ENV: &str = "CERULION_LAN_OBS_DIR";
const OBS_PORT_ENV: &str = "CERULION_LAN_OBS_PORT";
const OBS_TOPIC_ENV: &str = "CERULION_LAN_OBS_TOPIC";

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

fn signal_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(name)
}

fn touch(dir: &Path, name: &str) {
    fs::write(signal_path(dir, name), b"1").expect("write step file");
}

fn wait_for_step(dir: &Path, name: &str) -> Result<(), String> {
    let deadline = Instant::now() + Duration::from_secs(30);
    while !signal_path(dir, name).exists() {
        assert!(
            Instant::now() < deadline,
            "observer step '{name}' never appeared"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(())
}

/// Assert the paired-posture catalog refusal: no topic entries and a reason
/// naming both halves of the guidance.
fn assert_refusal(reply: &CatalogReply) {
    assert!(reply.entries.is_empty(), "a refusal serves no topic");
    let reason = reply.error.as_deref().expect("a refusal carries a reason");
    assert!(
        reason.contains("LAN access is paired-only")
            && reason.contains("set the LAN access posture to open for local development"),
        "{reason}"
    );
}

/// OBSERVER ROLE — the zenoh peer. Driven by step files in `OBS_DIR_ENV`:
///
/// 1. Wait for `open.signal`, then catalog-query until the topic is served.
/// 2. Declare a subscriber and a demand token, then write `demanded`.
/// 3. Wait for `closed.signal`, then query until the refusal returns and
///    write `refused` after checking it.
/// 4. Exit so the parent can assert on the child's exit status.
///
/// The initial paired refusal is checked BEFORE this role signals `refused`
/// the first time: the parent spawns the child, the child queries until the
/// refusal arrives, asserts its shape, and writes `refused` itself.
#[test]
#[ignore = "observer role; spawned by the parent test"]
fn observer_role() {
    let dir = PathBuf::from(std::env::var_os(OBS_DIR_ENV).expect(OBS_DIR_ENV));
    let port: u16 = std::env::var(OBS_PORT_ENV)
        .expect(OBS_PORT_ENV)
        .parse()
        .expect("port");
    let topic = std::env::var(OBS_TOPIC_ENV).expect(OBS_TOPIC_ENV);
    let session = client(port);

    // Paired runtime: the catalog answers with the refusal only.
    assert_refusal(&catalog_until(&session, |reply| reply.error.is_some()));
    touch(&dir, "refused");

    // Open runtime: the catalog serves the topic; demand it.
    wait_for_step(&dir, "open.signal").expect("open signal");
    let served = catalog_until(&session, |reply| {
        reply.error.is_none() && reply.entries.iter().any(|entry| entry.topic == topic)
    });
    assert!(served.error.is_none());
    let _subscriber = session
        .declare_subscriber(topic.trim_start_matches('/'))
        .wait()
        .expect("subscribe to published topic");
    let _demand = TopicToken::declare(&session, &topic).expect("demand published topic");
    touch(&dir, "demanded");

    // Paired restart: the same session sees the refusal again.
    wait_for_step(&dir, "closed.signal").expect("closed signal");
    assert_refusal(&catalog_until(&session, |reply| reply.error.is_some()));
    touch(&dir, "refused2");

    wait_for_step(&dir, "done.signal").expect("done signal");
}

fn observer_command(dir: &Path, port: u16, topic: &str, log_path: &Path) -> Command {
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
        .env(OBS_TOPIC_ENV, topic)
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

#[test]
fn stopping_an_open_runtime_revokes_lan_grants() {
    let temp = tempdir().unwrap();
    let obs_dir = tempdir().unwrap();
    let log_path = obs_dir.path().join("observer.log");
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
    // Spawned only after the runtime binds its listen endpoint; the
    // child's client session connects once and stays open across restarts.
    let mut observer = ObserverProcess(
        observer_command(obs_dir.path(), port, &topic, &log_path)
            .spawn()
            .expect("spawn observer process"),
    );
    await_step(&mut observer.0, obs_dir.path(), "refused", &log_path);
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
    touch(obs_dir.path(), "open.signal");
    await_step(&mut observer.0, obs_dir.path(), "demanded", &log_path);
    let manager = TransportManager::get().expect("initialized app transport manager");
    let bridges = manager.bridge_manager();
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

    runtime.start(&config(json!({}))).unwrap();
    touch(obs_dir.path(), "closed.signal");
    await_step(&mut observer.0, obs_dir.path(), "refused2", &log_path);
    std::thread::sleep(Duration::from_millis(1500));
    assert!(
        !bridges.is_enabled(&topic).unwrap(),
        "paired restart must not retain a bridge grant from the open run"
    );
    runtime.stop().unwrap();

    touch(obs_dir.path(), "done.signal");
    let status = observer.0.wait().expect("reap observer");
    assert!(
        status.success(),
        "observer failed with {status}:\n{}",
        fs::read_to_string(&log_path).unwrap_or_default()
    );

    runtime
        .start(&config(json!({"lan_access": "open"})))
        .unwrap();
    let network = manager.network().expect("app gateway network");
    assert_eq!(network.lan_access(), LanAccess::Open);
    // A failed component flips the state while the rest of the run keeps going.
    runtime.status_handle().lock().unwrap().state = cerulion_android::RuntimeState::Failed;
    assert!(
        runtime
            .start(&config(json!({"lan_access": "bogus"})))
            .is_err(),
        "an unknown posture is rejected"
    );
    assert_eq!(
        network.lan_access(),
        LanAccess::Paired,
        "a rejected restart must still close the failed open run"
    );
    assert!(bridges.is_lan_closed().unwrap());
}
