// SPDX-License-Identifier: AGPL-3.0-only
//! The app gateway's `ingress` list, written into by a real zenoh peer over
//! TCP.
//!
//! The desk role runs in a re-executed child process: one zenoh session per
//! process. It puts hand-written wire frames on the data key of each ingress
//! topic and answers the app's demand GETs. The parent starts the app with a
//! graph of relay nodes that read the ingress topics, drives the desk through
//! step files in a shared directory, and asserts on the relays' echoes and on
//! the ingress counters of the app's transport.
//!
//! The node registry and the transport are process state, so this file is
//! its own binary and holds ONE test that runs its steps in order.

use std::collections::BTreeSet;
use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_android::{codes, EmbeddedRuntime, RuntimeState};
use cerulion_core::graph::{NodeContext, NodeEntry, NodeInfo};
use cerulion_core::transport::network::{IngressStats, NetworkManager};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{StepTag, WireHeader};
use cerulion_core::{CerulionSubscriber, TransportResult};
use serde_json::json;
use tempfile::tempdir;
use test_node_macro_relay_rlib::RelayNodeEntry;
use zenoh::Wait;

cerulion_android::embed_nodes! {
    "relay" => RelayNodeEntry,
    "hashless_relay" => HashlessRelayEntry,
}

/// The relay, reporting no schema hash for its input, as a node entry
/// written by hand can. The node macro always reports one.
struct HashlessRelayEntry(RelayNodeEntry);

impl HashlessRelayEntry {
    fn new() -> Self {
        Self(RelayNodeEntry::new())
    }
}

impl NodeEntry for HashlessRelayEntry {
    fn info(&self) -> TransportResult<NodeInfo> {
        let info = self.0.info()?;
        let inputs = info
            .input_meta()
            .iter()
            .cloned()
            .map(|mut meta| {
                meta.schema_hash = 0;
                meta
            })
            .collect();
        Ok(NodeInfo::with_meta(inputs, info.output_meta().to_vec()))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.0.init(context)
    }

    fn tick(&mut self) -> TransportResult<()> {
        self.0.tick()
    }
}

const JNI_VERSION_1_6: i32 = 0x0001_0006;
const ROBOT: &str = "lan_ingress_test";
const PREFIX: &str = "lan_ingress_graph";
const CMD: &str = "/lan_ingress_test/cmd";
const GOAL: &str = "/lan_ingress_test/goal";
const VECTOR3: &str = "geometry_msgs/Vector3";

const DESK_DIR_ENV: &str = "CERULION_LAN_INGRESS_DESK_DIR";
const DESK_PORT_ENV: &str = "CERULION_LAN_INGRESS_DESK_PORT";

/// The `x` of every frame the desk sends with the right schema hash.
const RIGHT_X: f64 = 42.5;
/// The `x` of every frame the desk sends with the wrong schema hash.
const WRONG_X: f64 = -1.0;
/// A schema hash no input of the graph declares.
const WRONG_HASH: u64 = 0x0123_4567_89ab_cdef;

/// The runs the parent starts, in order: `(run, ingress topic, other topic)`.
/// Under `paired` both topics are listed; each `open` run lists one.
const RUNS: [(&str, &str, &str); 3] = [
    ("paired", CMD, GOAL),
    ("open", CMD, GOAL),
    ("restart", GOAL, CMD),
];

/// Two relays: `cmd_relay` reads `CMD` and `goal_relay` reads `GOAL`. Each
/// echoes the `x` of every frame it gets on `/{PREFIX}/<relay>/echo`.
fn graph_yaml(cmd_type: &str) -> String {
    format!(
        "prefix: {PREFIX}
nodes:
  - id: cmd_relay
    type: {cmd_type}
    inputs:
      - name: cmd
        source: {CMD}
    outputs:
      - name: echo
        schema: {VECTOR3}
  - id: goal_relay
    type: relay
    inputs:
      - name: cmd
        source: {GOAL}
    outputs:
      - name: echo
        schema: {VECTOR3}
"
    )
}

fn echo_topic(relay: &str) -> String {
    format!("/{PREFIX}/{relay}/echo")
}

/// An `ingress` entry must name a topic a graph input reads, and one no
/// other writer of the run sends: the start fails with the topic named and
/// the runtime reports `failed`.
fn a_misdeclared_ingress_topic_fails_the_start(runtime: &mut EmbeddedRuntime, port: u16) {
    let listen = json!([format!("tcp/127.0.0.1:{port}")]);
    let cases = [
        (
            json!({
                "graph_yaml": graph_yaml("relay"),
                "ingress": [CMD, "/lan_ingress_test/unread"],
            }),
            codes::INVALID_ARGUMENT,
            "ingress topic '/lan_ingress_test/unread' is not read by any graph input".to_string(),
        ),
        (
            json!({"ingress": [CMD]}),
            codes::INVALID_ARGUMENT,
            format!("ingress topic '{CMD}' is not read by any graph input"),
        ),
        (
            json!({
                "graph_yaml": graph_yaml("relay"),
                "ingress": [echo_topic("cmd_relay")],
            }),
            codes::INVALID_ARGUMENT,
            format!(
                "ingress topic '{}' is a graph output; graph topics are single-writer",
                echo_topic("cmd_relay")
            ),
        ),
        (
            json!({
                "graph_yaml": graph_yaml("relay"),
                "publish": [{"topic": CMD, "schema": VECTOR3}],
                "ingress": [CMD],
            }),
            codes::INVALID_ARGUMENT,
            format!("ingress topic '{CMD}' is a publish topic; a topic has one writer"),
        ),
        (
            json!({
                "graph_yaml": graph_yaml("hashless_relay"),
                "ingress": [CMD],
            }),
            codes::GRAPH,
            format!(
                "Graph validation error: network ingress topic '{CMD}' has no consumer with a declared schema — \
                 ingress frames are validated against the consuming input's schema hash \
                 (macro `#[input]` metadata); consume the topic from a `#[cerulion_node]` \
                 input or remove the ingress entry"
            ),
        ),
    ];
    for (mut config, code, message) in cases {
        config["listen"] = listen.clone();
        let error = runtime
            .start(&config.to_string())
            .expect_err("a misdeclared ingress topic cannot start");
        assert_eq!(error.code(), code, "{config}");
        assert_eq!(error.to_string(), message, "{config}");
        let status = runtime.status_handle().lock().unwrap().clone();
        assert_eq!(status.state, RuntimeState::Failed, "{status:?}");
    }
}

fn free_port() -> u16 {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local_addr").port()
}

/// The schema hash the relay declares for its `cmd` input: the hash every
/// ingress frame must carry.
fn relay_input_hash() -> u64 {
    let info = RelayNodeEntry::new().info().expect("relay node info");
    let hash = info.input_meta()[0].schema_hash;
    assert_ne!(hash, 0, "the relay declares its input schema");
    assert_ne!(
        hash, WRONG_HASH,
        "the wrong hash differs from the right one"
    );
    hash
}

/// A Vector3 wire frame written by hand: the 40-byte header, then `x`, `y`
/// and `z` as little-endian f64.
fn vector3_frame(schema_hash: u64, sequence: u32, x: f64) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    for value in [x, 0.0, 0.0] {
        payload.extend_from_slice(&f64::to_le_bytes(value));
    }
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence,
        timestamp_ns: u64::from(sequence) * 1_000_000,
        step_tag: StepTag::UNTAGGED,
    };
    let mut frame = vec![0u8; WireHeader::SIZE];
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
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
        assert!(Instant::now() < deadline, "step '{name}' never appeared");
        std::thread::sleep(Duration::from_millis(20));
    }
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

/// DESK ROLE: the zenoh peer. It records the topic of every demand GET it
/// gets. For each run in [`RUNS`]:
///
/// 1. Wait for `{run}.go` and clear the GET record.
/// 2. Under `paired`, put 20 right-hash frames on each topic, 50 ms apart.
///    Otherwise, every 50 ms put a wrong-hash and a right-hash frame on the
///    ingress topic and a right-hash frame on the other topic, until
///    `{run}.stop` appears.
/// 3. Write the GET topics, one per line, to `{run}.gets`, then `{run}.done`.
#[test]
#[ignore = "desk role; spawned by the parent test"]
fn desk_role() {
    let dir = PathBuf::from(std::env::var_os(DESK_DIR_ENV).expect(DESK_DIR_ENV));
    let port: u16 = std::env::var(DESK_PORT_ENV)
        .expect(DESK_PORT_ENV)
        .parse()
        .expect("port");
    let right_hash = relay_input_hash();
    let session = client(port);
    let gets = Arc::new(Mutex::new(BTreeSet::<String>::new()));
    let seen = Arc::clone(&gets);
    let _queryable = session
        .declare_queryable("cerulion_q/lan_ingress_desk/demand/**")
        .callback(move |query| {
            // `cerulion_q/<robot or *>/demand<topic>`: the app names the
            // desk, or every robot when it knows none.
            let key = query.key_expr().as_str();
            if let Some((_, topic)) = key
                .strip_prefix("cerulion_q/")
                .and_then(|rest| rest.split_once("/demand"))
            {
                seen.lock().unwrap().insert(topic.to_string());
            }
        })
        .wait()
        .expect("declare the demand queryable");
    let put = |topic: &str, frame: Vec<u8>| {
        session
            .put(NetworkManager::data_key(topic), frame)
            .wait()
            .expect("put a frame");
    };
    let mut sequence = 0u32;
    for (run, ingress, other) in RUNS {
        wait_for_step(&dir, &format!("{run}.go"));
        gets.lock().unwrap().clear();
        if run == "paired" {
            for _ in 0..20 {
                for topic in [ingress, other] {
                    sequence += 1;
                    put(topic, vector3_frame(right_hash, sequence, RIGHT_X));
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        } else {
            while !signal_path(&dir, &format!("{run}.stop")).exists() {
                sequence += 1;
                put(ingress, vector3_frame(WRONG_HASH, sequence, WRONG_X));
                sequence += 1;
                put(ingress, vector3_frame(right_hash, sequence, RIGHT_X));
                sequence += 1;
                put(other, vector3_frame(right_hash, sequence, RIGHT_X));
                std::thread::sleep(Duration::from_millis(50));
            }
        }
        let topics: String = gets
            .lock()
            .unwrap()
            .iter()
            .map(|topic| format!("{topic}\n"))
            .collect();
        fs::write(signal_path(&dir, &format!("{run}.gets")), topics).expect("write gets");
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

/// Wait for a desk step file, reporting the child's log when it exits early.
fn await_step(child: &mut Child, dir: &Path, name: &str, log_path: &Path) {
    let deadline = Instant::now() + Duration::from_secs(60);
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

/// The `x` of every echo waiting on `subscriber`.
fn drain_echoes(subscriber: &CerulionSubscriber, echoes: &mut Vec<f64>) {
    subscriber
        .wait_for_message(Duration::from_millis(50), |message| {
            let payload = message.payload();
            echoes.push(f64::from_le_bytes(
                payload[0..8].try_into().expect("x bytes"),
            ));
        })
        .expect("wait for relay output");
}

fn ingress_stats(topic: &str) -> Option<IngressStats> {
    TransportManager::get()
        .expect("initialized app transport manager")
        .network()
        .expect("the app transport has a network")
        .ingress_stats(topic)
}

fn is_ingress_registered(topic: &str) -> bool {
    TransportManager::get()
        .expect("initialized app transport manager")
        .network()
        .expect("the app transport has a network")
        .is_ingress_registered(topic)
}

fn relay_for(topic: &str) -> &'static str {
    if topic == CMD {
        "cmd_relay"
    } else {
        "goal_relay"
    }
}

/// One run of [`RUNS`]. Under `paired` no frame reaches either relay and no
/// ingress bridge exists. Under `open` the right-hash frames on the ingress
/// topic reach its relay, the wrong-hash ones are counted drops that never
/// reach it, and nothing reaches the relay of the other topic. The desk is
/// demanded the ingress topic only.
fn lan_run(
    runtime: &mut EmbeddedRuntime,
    desk: &mut DeskProcess,
    dir: &Path,
    log_path: &Path,
    port: u16,
    (run, ingress, other): (&str, &str, &str),
) {
    let paired = run == "paired";
    let config = if paired {
        json!({
            "graph_yaml": graph_yaml("relay"),
            "listen": [format!("tcp/127.0.0.1:{port}")],
            "ingress": [ingress, other],
        })
    } else {
        json!({
            "graph_yaml": graph_yaml("relay"),
            "listen": [format!("tcp/127.0.0.1:{port}")],
            "lan_access": "open",
            "ingress": [ingress],
        })
    };
    runtime.start(&config.to_string()).unwrap();
    let manager = TransportManager::get().expect("initialized app transport manager");
    let ingress_echo = manager
        .create_subscriber(&echo_topic(relay_for(ingress)))
        .expect("subscribe to the ingress relay output");
    let other_echo = manager
        .create_subscriber(&echo_topic(relay_for(other)))
        .expect("subscribe to the other relay output");
    let mut ingress_echoes = Vec::new();
    let mut other_echoes = Vec::new();
    touch(dir, &format!("{run}.go"));
    if !paired {
        // Until the first right-hash frame is echoed and a wrong-hash one
        // was counted.
        let deadline = Instant::now() + Duration::from_secs(20);
        while ingress_echoes.is_empty()
            || ingress_stats(ingress).is_none_or(|stats| stats.schema_mismatch_drops == 0)
        {
            assert!(
                Instant::now() < deadline,
                "{run}: no echo and counted drop on '{ingress}': {:?}",
                ingress_stats(ingress)
            );
            drain_echoes(&ingress_echo, &mut ingress_echoes);
            drain_echoes(&other_echo, &mut other_echoes);
        }
        // Long enough for at least two demand GET passes.
        let until = Instant::now() + Duration::from_secs(5);
        while Instant::now() < until {
            drain_echoes(&ingress_echo, &mut ingress_echoes);
            drain_echoes(&other_echo, &mut other_echoes);
        }
        touch(dir, &format!("{run}.stop"));
    }
    await_step(&mut desk.0, dir, &format!("{run}.done"), log_path);
    let quiet_until = Instant::now() + Duration::from_millis(300);
    while Instant::now() < quiet_until {
        drain_echoes(&ingress_echo, &mut ingress_echoes);
        drain_echoes(&other_echo, &mut other_echoes);
    }
    assert_eq!(
        other_echoes,
        Vec::<f64>::new(),
        "{run}: '{other}' is not ingress"
    );
    assert!(!is_ingress_registered(other), "{run}: '{other}'");
    assert!(
        ingress_stats(other).is_none(),
        "{run}: '{other}' has no bridge"
    );
    let gets = fs::read_to_string(signal_path(dir, &format!("{run}.gets"))).unwrap();
    if paired {
        assert_eq!(ingress_echoes, Vec::<f64>::new(), "{run}: '{ingress}'");
        assert!(!is_ingress_registered(ingress), "{run}: '{ingress}'");
        assert!(ingress_stats(ingress).is_none(), "{run}: '{ingress}'");
        assert_eq!(gets, "", "{run}: a paired app sends no demand GET");
    } else {
        assert!(
            ingress_echoes.iter().all(|&x| x == RIGHT_X),
            "{run}: only right-hash frames reach the node: {ingress_echoes:?}"
        );
        let stats = ingress_stats(ingress).expect("an open run bridges its ingress topic");
        assert_eq!(
            stats.frames,
            ingress_echoes.len() as u64,
            "{run}: one echo per re-injected frame: {stats:?}"
        );
        assert!(stats.schema_mismatch_drops > 0, "{run}: {stats:?}");
        assert_eq!(stats.decode_errors, 0, "{run}: {stats:?}");
        assert_eq!(stats.reinject_failure_drops, 0, "{run}: {stats:?}");
        assert_eq!(gets, format!("{ingress}\n"), "{run}: demand GETs");
    }
    runtime.stop().unwrap();
    assert!(
        !is_ingress_registered(ingress),
        "{run}: stop releases '{ingress}'"
    );
}

#[test]
fn ingress_topics_take_hash_checked_frames_from_a_lan_peer_only_when_open() {
    assert_eq!(
        JNI_OnLoad(std::ptr::null_mut(), std::ptr::null_mut()),
        JNI_VERSION_1_6
    );
    let temp = tempdir().unwrap();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into()).unwrap();
    let port = free_port();
    a_misdeclared_ingress_topic_fails_the_start(&mut runtime, port);

    let desk_dir = tempdir().unwrap();
    let dir = desk_dir.path();
    let log_path = dir.join("desk.log");
    let mut desk = None;
    for run in RUNS {
        if desk.is_none() {
            // Spawned once the runtime binds its listen endpoint, inside the
            // first run; the desk's client session stays open across restarts.
            let config = json!({
                "graph_yaml": graph_yaml("relay"),
                "listen": [format!("tcp/127.0.0.1:{port}")],
            });
            runtime.start(&config.to_string()).unwrap();
            desk = Some(DeskProcess(
                desk_command(dir, port, &log_path)
                    .spawn()
                    .expect("spawn desk process"),
            ));
            std::thread::sleep(Duration::from_secs(1));
            runtime.stop().unwrap();
        }
        let desk = desk.as_mut().expect("desk spawned");
        lan_run(&mut runtime, desk, dir, &log_path, port, run);
    }
    let mut desk = desk.expect("desk spawned");
    let status = desk.0.wait().expect("reap desk");
    assert!(
        status.success(),
        "desk failed with {status}:\n{}",
        fs::read_to_string(&log_path).unwrap_or_default()
    );
}
