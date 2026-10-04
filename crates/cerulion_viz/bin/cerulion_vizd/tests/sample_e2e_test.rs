// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END acceptance for the `sample` verb of `cerulion-vizd`, over a real
//! iceoryx2 transport and a real UDS control connection (`#[cfg(unix)]`).
//!
//! The daemon runs IN-PROCESS over an isolated per-test SHM root, with an explicit
//! demand-plane double so no test reaches a live netd. Every frame is a HAND-BUILT
//! `geometry_msgs/Vector3` (or a deliberately undecodable / oversize frame) published
//! by a REAL producer, so each reply is checked against a hand oracle: the producer
//! stamps `seq = s`, `ts = STEP * (s + 1)` and `x = s`, so a row's every field is
//! known without reading the daemon.
//!
//! What this pins, end to end:
//!
//! * the reply shape `{rows:[{seq, ts_ns, size, fields, summary}]}` and its decode;
//! * the ring exists only while sampled: the FIRST sample of a topic returns no rows
//!   however long the producer has been publishing, and a ring that is not sampled
//!   for five seconds is gone;
//! * `n` is bounded (clamped to 20, `0` refused);
//! * `sample` opens nothing: an unattached topic is refused and stays unattached;
//! * an oversize frame keeps its header facts and no body, and an undecodable frame
//!   keeps them with the reason;
//! * at most eight topics are sampled at once;
//! * the verb is additive: the banner still says protocol 1 and every other verb
//!   answers as before.
//!
//! The file takes one lock for every test: an attach touches the process-global
//! blueprint statics, and `cargo test -p cerulion_vizd` is documented serial anyway.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::codegen::FrameWalker;
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::VizLogWorker;
use cerulion_vizd::{start_with_demand_plane, DemandPlane, RunningDaemon, DEFAULT_POLL_INTERVAL};
use native_ros2_messages::geometry_msgs::Vector3;
use serde_json::Value;

/// One lock for the file (see the module docs). Poison-tolerant.
static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn serial() -> std::sync::MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|p| p.into_inner())
}

/// The publisher's timestamp step: frame `s` is stamped `STEP * (s + 1)`.
const STEP_NS: u64 = 10_000_000;

/// A demand plane that refuses everything: this file never touches netd.
struct NoNetdPlane;

impl DemandPlane for NoNetdPlane {
    fn demand(&self, _robot: &str, _topic: &str, _schema_hash: u64) -> Result<(), String> {
        Err("hermetic test: no netd".to_string())
    }
    fn release(&self, _robot: &str, _topic: &str) -> Result<(), String> {
        Ok(())
    }
    fn query_catalog(&self, _robot: &str) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_catalog_all(&self) -> Result<Vec<cerulion_core::CatalogReply>, String> {
        Ok(Vec::new())
    }
    fn query_schema(
        &self,
        _robot: &str,
        _requested: &str,
    ) -> Result<Vec<cerulion_core::SchemaReply>, String> {
        Ok(Vec::new())
    }
}

fn transport(node: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node.to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport")
}

fn temp_socket(tag: &str) -> (PathBuf, PathBuf) {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("cer_sample_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    let socket = dir.join("vizd.sock");
    assert!(
        socket.as_os_str().len() < 104,
        "temp_socket tag `{tag}` is too long for sockaddr_un"
    );
    (socket, dir)
}

/// A daemon over a memory sink, with the walker the production daemon starts with.
fn start(tag: &str, mgr: &Arc<TransportManager>) -> (RunningDaemon, PathBuf, PathBuf) {
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("vizd")
        .recording_id(tag)
        .memory()
        .expect("memory sink");
    let worker =
        VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("spawn worker");
    let (socket, dir) = temp_socket(tag);
    let walker: FrameWalker = builtin_walker();
    let daemon = start_with_demand_plane(
        socket.clone(),
        DEFAULT_POLL_INTERVAL,
        Arc::clone(mgr),
        worker,
        walker,
        None,
        Arc::new(NoNetdPlane),
    )
    .expect("daemon starts");
    (daemon, socket, dir)
}

/// A hand-built wire frame for a schema hash and payload.
fn frame(schema_hash: u64, seq: u32, ts: u64, payload: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + payload.len();
    let header = WireHeader {
        schema_hash,
        total_size: total as u32,
        offset_table_offset: total as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut buf = vec![0u8; total];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(payload);
    buf
}

/// `Vector3 { x: seq, y: 2.0, z: 3.0 }`, stamped `STEP * (seq + 1)`.
fn vector3(seq: u32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    for v in [f64::from(seq), 2.0, 3.0] {
        payload.extend_from_slice(&v.to_le_bytes());
    }
    frame(
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        seq,
        STEP_NS * (u64::from(seq) + 1),
        &payload,
    )
}

/// A real producer publishing `make(seq)` every 10 ms until dropped. The service
/// exists when `spawn` returns, so the topic is discoverable at once.
struct Producer {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Producer {
    fn spawn(mgr: &Arc<TransportManager>, topic: &str, make: fn(u32) -> Vec<u8>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_c = Arc::clone(&stop);
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
            .unwrap_or_else(|e| panic!("producer attaches on '{topic}': {e}"));
        let seq = AtomicU32::new(0);
        let handle = std::thread::spawn(move || {
            while !stop_c.load(Ordering::Relaxed) {
                let s = seq.fetch_add(1, Ordering::Relaxed);
                let _ = publisher.publish_raw(&make(s));
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        Producer {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Producer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

struct Client {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
    banner: Value,
}

impl Client {
    fn connect(socket: &std::path::Path) -> Self {
        let deadline = Instant::now() + Duration::from_secs(3);
        let stream = loop {
            match UnixStream::connect(socket) {
                Ok(s) => break s,
                Err(_) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(10))
                }
                Err(e) => panic!("could not connect to {}: {e}", socket.display()),
            }
        };
        let writer = stream.try_clone().expect("clone stream");
        let mut reader = BufReader::new(stream);
        let mut line = String::new();
        reader.read_line(&mut line).expect("read banner");
        Client {
            writer,
            reader,
            banner: serde_json::from_str(&line).expect("banner json"),
        }
    }

    fn request(&mut self, line: &str) -> Value {
        writeln!(self.writer, "{line}").expect("write request");
        let mut resp = String::new();
        self.reader.read_line(&mut resp).expect("read response");
        serde_json::from_str(&resp).expect("response json")
    }

    fn attach(&mut self, topic: &str) {
        let r = self.request(&format!(
            r#"{{"id":1,"method":"attach","topic":"{topic}"}}"#
        ));
        assert_eq!(r["ok"].as_bool(), Some(true), "attach {topic}: {r}");
    }

    fn sample(&mut self, topic: &str, n: u64) -> Value {
        self.request(&format!(
            r#"{{"id":7,"method":"sample","topic":"{topic}","n":{n}}}"#
        ))
    }

    /// Poll `sample` until it returns at least `want` rows (the ring is armed by the
    /// first call and fed by the poll thread), within a bound. Returns that reply.
    fn sample_until(&mut self, topic: &str, n: u64, want: usize) -> Value {
        let deadline = Instant::now() + Duration::from_secs(8);
        loop {
            let r = self.sample(topic, n);
            let got = r["rows"].as_array().map_or(0, Vec::len);
            if got >= want {
                return r;
            }
            assert!(
                Instant::now() < deadline,
                "sample never reached {want} rows for {topic}: {r}"
            );
            std::thread::sleep(Duration::from_millis(50));
        }
    }
}

fn rows(reply: &Value) -> &Vec<Value> {
    reply["rows"].as_array().expect("rows is an array")
}

// ── The headline ────────────────────────────────────────────────────────────

#[test]
fn sample_returns_the_latest_messages_decoded_end_to_end() {
    let _g = serial();
    let mgr = transport("sample_headline");
    let topic = "/sample/vel";
    let _producer = Producer::spawn(&mgr, topic, vector3);
    let (mut daemon, socket, dir) = start("headline", &mgr);
    let mut client = Client::connect(&socket);
    client.attach(topic);

    let reply = client.sample_until(topic, 3, 3);
    assert_eq!(reply["ok"].as_bool(), Some(true));
    assert_eq!(reply["id"].as_u64(), Some(7));
    assert_eq!(reply["topic"].as_str(), Some(topic));
    let got = rows(&reply);
    assert_eq!(got.len(), 3, "n=3 returns exactly three rows: {reply}");

    let mut last_seq: Option<u64> = None;
    for row in got {
        let seq = row["seq"].as_u64().expect("seq");
        if let Some(prev) = last_seq {
            assert!(seq > prev, "rows run oldest first: {prev} then {seq}");
        }
        last_seq = Some(seq);
        // The producer's own stamps are the oracle.
        assert_eq!(row["ts_ns"].as_u64(), Some(STEP_NS * (seq + 1)));
        assert_eq!(row["size"].as_u64(), Some((WireHeader::SIZE + 24) as u64));
        assert_eq!(
            row["fields"],
            serde_json::json!({"x": seq as f64, "y": 2.0, "z": 3.0}),
            "decoded fields of seq {seq}"
        );
        assert_eq!(
            row["summary"].as_str(),
            Some(format!("geometry_msgs/Vector3: x={seq}, y=2, z=3").as_str())
        );
    }

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── The ring exists only while sampled ──────────────────────────────────────

#[test]
fn the_first_sample_arms_the_ring_and_returns_nothing_from_before() {
    let _g = serial();
    let mgr = transport("sample_arm");
    let topic = "/sample/arm";
    let _producer = Producer::spawn(&mgr, topic, vector3);
    let (mut daemon, socket, dir) = start("arm", &mgr);
    let mut client = Client::connect(&socket);
    client.attach(topic);

    // The producer has been publishing for a while by now; none of it was kept.
    std::thread::sleep(Duration::from_millis(400));
    let first = client.sample(topic, 20);
    assert_eq!(first["ok"].as_bool(), Some(true), "{first}");
    assert!(
        rows(&first).is_empty(),
        "frames from before the first sample are never kept: {first}"
    );
    // ...and the ring then fills.
    let later = client.sample_until(topic, 20, 1);
    let seq = rows(&later)[0]["seq"].as_u64().unwrap();
    assert!(
        seq > 0,
        "the first retained frame came after the arm, not from the start of the stream"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn a_ring_nobody_samples_for_five_seconds_is_gone() {
    let _g = serial();
    let mgr = transport("sample_ttl");
    let topic = "/sample/ttl";
    let _producer = Producer::spawn(&mgr, topic, vector3);
    let (mut daemon, socket, dir) = start("ttl", &mgr);
    let mut client = Client::connect(&socket);
    client.attach(topic);

    let filled = client.sample_until(topic, 20, 2);
    assert!(rows(&filled).len() >= 2);

    // Stop asking. The ring has a five second life; wait it out with margin.
    std::thread::sleep(Duration::from_millis(5600));

    // The next sample finds no ring and arms a fresh, EMPTY one: nothing the
    // producer published during the silence survives.
    let after = client.sample(topic, 20);
    assert_eq!(after["ok"].as_bool(), Some(true), "{after}");
    assert!(
        rows(&after).is_empty(),
        "an unsampled ring must not outlive its deadline: {after}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── Bounds on the request ───────────────────────────────────────────────────

#[test]
fn n_is_clamped_to_twenty_and_zero_is_refused() {
    let _g = serial();
    let mgr = transport("sample_n");
    let topic = "/sample/n";
    let _producer = Producer::spawn(&mgr, topic, vector3);
    let (mut daemon, socket, dir) = start("n", &mgr);
    let mut client = Client::connect(&socket);
    client.attach(topic);

    let zero = client.sample(topic, 0);
    assert_eq!(zero["ok"].as_bool(), Some(false), "{zero}");
    assert_eq!(zero["id"].as_u64(), Some(7));
    assert!(zero["error"].as_str().unwrap().contains("between 1 and 20"));

    // A request far above the cap is answered, never with more than 20 rows, and
    // the ring (which only ever holds 20) does reach 20 on a 100 Hz producer.
    let full = client.sample_until(topic, 1000, 20);
    assert_eq!(rows(&full).len(), 20, "{full}");
    for _ in 0..5 {
        let r = client.sample(topic, 1000);
        assert!(rows(&r).len() <= 20, "never more than 20 rows: {r}");
    }

    // A count beyond u32 is still a request, clamped like any other large one.
    let huge = client.sample(topic, 4_294_967_296);
    assert_eq!(huge["ok"].as_bool(), Some(true), "{huge}");
    assert_eq!(rows(&huge).len(), 20, "{huge}");

    // Omitting `n` gives the default of five.
    let default = client.request(&format!(
        r#"{{"id":8,"method":"sample","topic":"{topic}"}}"#
    ));
    assert_eq!(rows(&default).len(), 5, "{default}");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── It opens nothing ────────────────────────────────────────────────────────

#[test]
fn sampling_an_unattached_topic_is_refused_and_attaches_nothing() {
    let _g = serial();
    let mgr = transport("sample_unattached");
    let topic = "/sample/unattached";
    let _producer = Producer::spawn(&mgr, topic, vector3);
    let (mut daemon, socket, dir) = start("unatt", &mgr);
    let mut client = Client::connect(&socket);

    let reply = client.sample(topic, 5);
    assert_eq!(reply["ok"].as_bool(), Some(false), "{reply}");
    assert_eq!(reply["topic"].as_str(), Some(topic));
    let error = reply["error"].as_str().unwrap();
    assert!(error.contains("not attached"), "{error}");
    assert!(error.contains("opens no subscription"), "{error}");
    assert!(
        !error.contains("  "),
        "no run of spaces in a message: {error}"
    );
    assert_eq!(
        error,
        format!(
            "sample: '{}' is not attached. sample reads the frames an attach already \
             drains and opens no subscription of its own; attach the topic first",
            reply["topic"].as_str().unwrap_or_default()
        ),
        "{reply}"
    );

    // Nothing was attached as a side effect, however often it is asked.
    for _ in 0..3 {
        let _ = client.sample(topic, 5);
    }
    let list = client.request(r#"{"id":2,"method":"list"}"#);
    assert_eq!(list["ok"].as_bool(), Some(true));
    assert!(
        list["attached"].as_array().unwrap().is_empty(),
        "sample must never attach: {list}"
    );

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn detach_drops_the_ring_and_a_reattach_starts_empty() {
    let _g = serial();
    let mgr = transport("sample_detach");
    let topic = "/sample/detach";
    let _producer = Producer::spawn(&mgr, topic, vector3);
    let (mut daemon, socket, dir) = start("detach", &mgr);
    let mut client = Client::connect(&socket);
    client.attach(topic);
    let _ = client.sample_until(topic, 5, 2);

    let detached = client.request(&format!(
        r#"{{"id":3,"method":"detach","topic":"{topic}"}}"#
    ));
    assert_eq!(detached["detached"].as_bool(), Some(true), "{detached}");
    let refused = client.sample(topic, 5);
    assert_eq!(refused["ok"].as_bool(), Some(false), "{refused}");

    // Re-attached within the five seconds, the old frames are not served again.
    client.attach(topic);
    let fresh = client.sample(topic, 20);
    assert_eq!(fresh["ok"].as_bool(), Some(true), "{fresh}");
    assert!(rows(&fresh).is_empty(), "{fresh}");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── Frames that cannot be fully shown ───────────────────────────────────────

/// A `Vector3`-hash frame padded past the byte limit: bulk data in the shape an
/// image or a cloud takes.
fn oversize(seq: u32) -> Vec<u8> {
    let mut payload = vec![0u8; 24 + 20_000];
    payload[..8].copy_from_slice(&f64::from(seq).to_le_bytes());
    frame(
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        seq,
        STEP_NS * (u64::from(seq) + 1),
        &payload,
    )
}

/// A frame whose schema hash no walker holds.
fn unknown_schema(seq: u32) -> Vec<u8> {
    frame(0x1234_5678_9ABC_DEF0, seq, 5, &[0u8; 16])
}

#[test]
fn an_oversize_frame_keeps_its_header_facts_and_no_body() {
    let _g = serial();
    let mgr = transport("sample_big");
    let topic = "/sample/big";
    let _producer = Producer::spawn(&mgr, topic, oversize);
    let (mut daemon, socket, dir) = start("big", &mgr);
    let mut client = Client::connect(&socket);
    client.attach(topic);

    let reply = client.sample_until(topic, 2, 2);
    for row in rows(&reply) {
        let seq = row["seq"].as_u64().unwrap();
        assert_eq!(row["ts_ns"].as_u64(), Some(STEP_NS * (seq + 1)));
        assert_eq!(
            row["size"].as_u64(),
            Some((WireHeader::SIZE + 24 + 20_000) as u64)
        );
        assert!(row["fields"].is_null(), "no body was kept: {row}");
        assert!(
            row["summary"].as_str().unwrap().contains("larger than"),
            "{row}"
        );
    }

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn an_undecodable_frame_has_null_fields_and_says_why() {
    let _g = serial();
    let mgr = transport("sample_unknown");
    let topic = "/sample/unknown";
    let _producer = Producer::spawn(&mgr, topic, unknown_schema);
    let (mut daemon, socket, dir) = start("unk", &mgr);
    let mut client = Client::connect(&socket);
    client.attach(topic);

    let reply = client.sample_until(topic, 2, 1);
    let row = &rows(&reply)[0];
    assert!(row["fields"].is_null(), "{row}");
    assert_eq!(row["size"].as_u64(), Some((WireHeader::SIZE + 16) as u64));
    assert_eq!(row["ts_ns"].as_u64(), Some(5));
    assert!(
        row["summary"].as_str().unwrap().contains("not decoded"),
        "{row}"
    );
    // The key is present-but-null, a stable shape.
    assert!(row.as_object().unwrap().contains_key("fields"));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── Bounded memory: at most eight topics ────────────────────────────────────

#[test]
fn at_most_eight_topics_are_sampled_at_once() {
    let _g = serial();
    let mgr = transport("sample_cap");
    let topics: Vec<String> = (0..9).map(|i| format!("/sample/cap{i}")).collect();
    let _producers: Vec<Producer> = topics
        .iter()
        .map(|t| Producer::spawn(&mgr, t, vector3))
        .collect();
    let (mut daemon, socket, dir) = start("cap", &mgr);
    let mut client = Client::connect(&socket);
    for t in &topics {
        client.attach(t);
    }
    for t in &topics[..8] {
        let r = client.sample(t, 1);
        assert_eq!(r["ok"].as_bool(), Some(true), "{t}: {r}");
    }
    let ninth = client.sample(&topics[8], 1);
    assert_eq!(ninth["ok"].as_bool(), Some(false), "{ninth}");
    assert!(
        ninth["error"]
            .as_str()
            .unwrap()
            .contains("already being sampled"),
        "{ninth}"
    );
    assert_eq!(
        ninth["error"].as_str().unwrap(),
        "sample: 8 topics are already being sampled; a topic stops counting \
         five seconds after its last sample",
        "{ninth}"
    );
    // A topic already sampled can still be asked again at the cap.
    let again = client.sample(&topics[0], 1);
    assert_eq!(again["ok"].as_bool(), Some(true), "{again}");

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}

// ── Compatibility ───────────────────────────────────────────────────────────

#[test]
fn the_verb_is_additive_and_old_traffic_is_unchanged() {
    let _g = serial();
    let mgr = transport("sample_compat");
    let topic = "/sample/compat";
    let _producer = Producer::spawn(&mgr, topic, vector3);
    let (mut daemon, socket, dir) = start("compat", &mgr);
    let mut client = Client::connect(&socket);

    // The banner a controller compares for exact equality is untouched.
    assert_eq!(client.banner["vizd"].as_str(), Some("cerulion-vizd"));
    assert_eq!(client.banner["protocol"].as_u64(), Some(1));
    let mut keys: Vec<&str> = client
        .banner
        .as_object()
        .unwrap()
        .keys()
        .map(String::as_str)
        .collect();
    keys.sort_unstable();
    assert_eq!(keys, ["protocol", "rerun_url", "vizd"]);

    // The pre-existing verbs answer exactly as before, before and after a sample.
    client.attach(topic);
    let before = client.request(r#"{"id":2,"method":"list"}"#);
    let _ = client.sample(topic, 5);
    let after = client.request(r#"{"id":2,"method":"list"}"#);
    assert_eq!(before["ok"], after["ok"]);
    // The liveness block ticks between two calls by design; the identity of each
    // attached row does not.
    let identity = |reply: &Value| -> Vec<(Value, Value, Value, Value)> {
        reply["attached"]
            .as_array()
            .unwrap()
            .iter()
            .map(|e| {
                (
                    e["topic"].clone(),
                    e["schema"].clone(),
                    e["entity"].clone(),
                    e["route"].clone(),
                )
            })
            .collect()
    };
    assert_eq!(identity(&before).len(), 1);
    assert_eq!(identity(&before), identity(&after));
    let status = client.request(r#"{"id":3,"method":"status"}"#);
    assert_eq!(status["ok"].as_bool(), Some(true));

    // A field this daemon does not know is ignored, as the contract says.
    let tolerant = client.request(&format!(
        r#"{{"id":4,"method":"sample","topic":"{topic}","n":1,"since_seq":3}}"#
    ));
    assert_eq!(tolerant["ok"].as_bool(), Some(true), "{tolerant}");

    // An unknown verb still gets the structured error with its id, as an old daemon
    // would answer `sample` itself: that is the capability handshake.
    let unknown = client.request(r#"{"id":5,"method":"sampler","topic":"/x"}"#);
    assert_eq!(unknown["ok"].as_bool(), Some(false));
    assert_eq!(unknown["id"].as_u64(), Some(5));

    daemon.shutdown();
    let _ = std::fs::remove_dir_all(dir);
}
