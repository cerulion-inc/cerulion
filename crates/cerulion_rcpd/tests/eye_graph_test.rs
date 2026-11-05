// SPDX-License-Identifier: AGPL-3.0-only
//! The eye on the graph: a stand-in gate publishes wrist frames and asks, the
//! `rcp-eye` thread answers them through a local stand-in route, and a tap
//! records the three topics into a bag. Then: the bag, the receipt log, the
//! status file and every log line hold no key bytes; a resim from the bag
//! with no route reproduces the gate's decisions byte for byte; and
//! `--verify-bag` joins every answer to its receipt.
//!
//! One test per binary: it installs the process's log subscriber.

mod eye_common;

use std::io::Write as _;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager, WireHeader};
use cerulion_rcpd::eye::host::{EyeHost, EyeSettings};
use cerulion_rcpd::eye::limits::jpeg_header_for_test;
use cerulion_rcpd::eye::replay::{string_frames, RecordedAnswers};
use cerulion_rcpd::eye::switch::{read_status, write_switch};
use cerulion_rcpd::eye::verdict::{BoxEdge, EyeAnswer, Look, Outcome};
use cerulion_rcpd::eye::verify::{verify_bag, EyeTopics};
use cerulion_rcpd::manifest::RcpManifest;
use cerulion_rcpd::tools::ReceiptSink;
use eye_common::{good_body, holds_key, Reply, Stub, ANSWERED_MODEL};
use native_ros2_messages::sensor_msgs::CompressedImage;
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::{json, Value};

/// Every log line of the process.
#[derive(Clone, Default)]
struct Logs(Arc<Mutex<Vec<u8>>>);

impl std::io::Write for Logs {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Logs {
    fn bytes(&self) -> Vec<u8> {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }
}

fn install_logs() -> Logs {
    use tracing_subscriber::util::SubscriberInitExt as _;
    let logs = Logs::default();
    let writer = logs.clone();
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .with_ansi(false)
        .with_writer(move || writer.clone())
        .finish()
        .try_init()
        .expect("the one log subscriber");
    logs
}

/// The stand-in route's reply per `case`.
fn scripted(body: &Value) -> Reply {
    match body["state"]["numbers"]["case"].as_u64().unwrap_or(0) {
        0 | 6 => Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.68)),
        1 => Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.80)),
        2 => Reply::ok(good_body(ANSWERED_MODEL, 0.2, 0.68)),
        3 => Reply::ok(good_body("typesafe/jev-1.15-20261201", 0.93, 0.68)),
        4 => Reply::status(500, json!({})),
        5 => Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.68)).after(Duration::from_millis(500)),
        _ => Reply::status(400, json!({})),
    }
}

/// One frame as a bag stores it: wire header, then the message.
#[derive(Clone)]
struct Recorded {
    topic: String,
    seq: u32,
    ts: u64,
    frame: Vec<u8>,
}

/// Records every frame on `topics` until `stop` is set.
fn spawn_tap(
    mgr: Arc<TransportManager>,
    topics: Vec<String>,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<Vec<Recorded>> {
    let (ready_tx, ready_rx) = std::sync::mpsc::channel();
    let handle = std::thread::spawn(move || {
        let subs: Vec<_> = topics
            .iter()
            .map(|t| (t.clone(), mgr.create_subscriber(t).expect("tap")))
            .collect();
        ready_tx.send(()).expect("ready");
        let mut out = Vec::new();
        loop {
            let stopping = stop.load(Ordering::SeqCst);
            for (topic, sub) in &subs {
                sub.try_receive(|m| {
                    let mut frame = vec![0u8; WireHeader::SIZE];
                    m.header().write_to_buf(&mut frame);
                    frame.extend_from_slice(m.payload());
                    out.push(Recorded {
                        topic: topic.clone(),
                        seq: m.header().sequence,
                        ts: m.header().timestamp_ns,
                        frame,
                    });
                })
                .expect("tap receive");
            }
            if stopping {
                return out;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    });
    ready_rx.recv().expect("tap ready");
    handle
}

fn write_bag(path: &Path, frames: &[Recorded], topics: &[(String, &str, u64, usize)]) {
    let schemas: Vec<TopicSchema> = topics
        .iter()
        .map(|(topic, name, hash, fixed)| TopicSchema {
            topic: topic.clone(),
            schema_name: (*name).to_owned(),
            schema_hash: *hash,
            wire_fixed_size: u32::try_from(*fixed).expect("size"),
        })
        .collect();
    let mut writer = BagWriter::create(path, BagWriterConfig::default(), &schemas).expect("bag");
    for r in frames {
        writer
            .write_message(r.topic.as_str(), r.seq, r.ts, r.ts, &[&r.frame])
            .expect("write");
    }
    writer.finalize().expect("finalize");
}

/// The gate's decision for one answer frame: Jev's rim row against the
/// gate's own 326 px, within 8 px.
fn decide(answer: &str) -> String {
    let parsed = EyeAnswer::parse(answer).expect("answer frame");
    let look = Look::compare(
        &parsed,
        |a| a.box_edge_px("rim", BoxEdge::Bottom),
        326.0,
        8.0,
    );
    format!("{} {look:?}", parsed.ask_id)
}

struct Gate {
    frames: cerulion_core::CerulionPublisher,
    asks: cerulion_core::CerulionPublisher,
    answers: cerulion_core::CerulionSubscriber,
    answers_topic: String,
    frame_topic: String,
}

impl Gate {
    fn publish_frame(&mut self, jpeg: &[u8]) -> u32 {
        let seq = self.frames.sequence();
        let mut proxy = self
            .frames
            .loan_proxy::<CompressedImage>()
            .expect("loan frame");
        proxy
            .with_header(|h| {
                h.set_frame_id("wrist")?;
                Ok(())
            })
            .expect("header");
        proxy.set_format("jpeg").expect("format");
        proxy.set_data(jpeg).expect("data");
        drop(proxy);
        seq
    }

    fn ask(&mut self, ask: &Value) {
        let mut proxy = self.asks.loan_proxy::<RosString>().expect("loan ask");
        proxy.set_data(&ask.to_string()).expect("set");
    }

    /// The answer to `ask_id`, waiting at most `within`.
    fn answer(&self, ask_id: &Value, within: Duration) -> String {
        let deadline = Instant::now() + within;
        let mut found = None;
        while found.is_none() && Instant::now() < deadline {
            self.answers
                .try_receive(|m| {
                    let text = RosString::build_reader(m.payload())
                        .data()
                        .expect("utf-8")
                        .to_owned();
                    let id = serde_json::from_str::<Value>(&text).expect("json")["ask_id"].clone();
                    if &id == ask_id {
                        found = Some(text);
                    }
                })
                .expect("receive");
            std::thread::sleep(Duration::from_millis(1));
        }
        found.unwrap_or_else(|| panic!("no answer to ask {ask_id} on {}", self.answers_topic))
    }

    fn ask_for(&mut self, id: u64, case: u64, seq: u32, deadline_ms: u64) -> Value {
        let ask = json!({
            "ask_id": id,
            "question_set": "press",
            "frame": { "topic": self.frame_topic, "seq": seq },
            "deadline_ms": deadline_ms,
            "numbers": { "case": case },
        });
        self.ask(&ask);
        ask
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn the_eye_records_answers_that_resim_and_verify_byte_for_byte() {
    let logs = install_logs();
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let prefix = format!("rcpd_eye_graph_{}", std::process::id());
    let key = eye_common::key_file(dir.path());
    let manifest = eye_common::write_manifest(
        dir.path(),
        &eye_common::eyes_yaml(&prefix, &stub.base_url(), &key),
    );
    let loaded = RcpManifest::load_full(&manifest).expect("manifest");
    let jev = loaded.eyes.jev.clone().expect("eyes.jev");
    let state_dir = dir.path().join("state");
    std::fs::create_dir_all(&state_dir).expect("state dir");
    let receipts = state_dir.join("receipts.jsonl");
    let settings =
        EyeSettings::resolve(&loaded.eyes, dir.path(), &state_dir, None).expect("settings");
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: prefix.clone(),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("transport");

    let stop = Arc::new(AtomicBool::new(false));
    let topics = vec![
        jev.frame_topic.clone(),
        jev.ask_topic.clone(),
        jev.answer_topic.clone(),
    ];
    let tap = spawn_tap(mgr.clone(), topics, stop.clone());
    let eye = EyeHost::spawn(
        mgr.clone(),
        settings,
        ReceiptSink::open(&receipts).expect("receipts"),
        tokio::runtime::Handle::current(),
    )
    .expect("eye");
    // The gate runs on its own thread: its ports stay there.
    let gate_mgr = mgr.clone();
    let gate_jev = jev.clone();
    let gate_state = state_dir.clone();
    let gate_thread = std::thread::spawn(move || {
        let mut gate = Gate {
            frames: gate_mgr
                .create_publisher_simple(&gate_jev.frame_topic, MaxSliceLen::const_new(512 * 1024))
                .expect("frame publisher"),
            asks: gate_mgr
                .create_publisher_simple(&gate_jev.ask_topic, MaxSliceLen::const_new(16 * 1024))
                .expect("ask publisher"),
            answers: gate_mgr
                .create_subscriber(&gate_jev.answer_topic)
                .expect("answer subscriber"),
            answers_topic: gate_jev.answer_topic.clone(),
            frame_topic: gate_jev.frame_topic.clone(),
        };
        let good = jpeg_header_for_test(640, 480, 30_000);
        let mut out = Vec::new();
        // One ask at a time.
        for (id, case) in [(0u64, 0u64), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6)] {
            let seq = gate.publish_frame(&good);
            let deadline = if case == 5 { 200 } else { 1_500 };
            let ask = gate.ask_for(id, case, seq, deadline);
            out.push((
                ask["ask_id"].clone(),
                gate.answer(&ask["ask_id"], Duration::from_secs(5)),
            ));
        }
        // A frame never published, then one larger than 640 x 480.
        let ask = gate.ask_for(7, 0, 9_999, 1_500);
        out.push((
            ask["ask_id"].clone(),
            gate.answer(&ask["ask_id"], Duration::from_secs(5)),
        ));
        let seq = gate.publish_frame(&jpeg_header_for_test(800, 600, 1_000));
        let ask = gate.ask_for(8, 0, seq, 1_500);
        out.push((
            ask["ask_id"].clone(),
            gate.answer(&ask["ask_id"], Duration::from_secs(5)),
        ));
        // The owner turns Jev off: the next ask is disabled at once.
        write_switch(&gate_state, false).expect("off");
        let seq = gate.publish_frame(&good);
        let ask = gate.ask_for(9, 0, seq, 1_500);
        out.push((
            ask["ask_id"].clone(),
            gate.answer(&ask["ask_id"], Duration::from_secs(5)),
        ));
        write_switch(&gate_state, true).expect("on");
        out
    });
    let live: Vec<(Value, String)> = tokio::task::spawn_blocking(move || gate_thread.join())
        .await
        .expect("join")
        .expect("gate");

    let outcomes: Vec<String> = live
        .iter()
        .map(|(_, a)| {
            EyeAnswer::parse(a)
                .expect("parse")
                .outcome
                .as_str()
                .to_owned()
        })
        .collect();
    assert_eq!(
        outcomes,
        [
            "ok",
            "ok",
            "not_visible",
            "wrong_model",
            "unavailable",
            "timeout",
            "ok",
            "refused",
            "refused",
            "disabled"
        ]
    );
    let decisions: Vec<String> = live.iter().map(|(_, a)| decide(a)).collect();
    assert_eq!(decisions[0], "0 Agree");
    assert_eq!(decisions[1], "1 Disagree");
    for d in &decisions[2..6] {
        assert!(d.contains("Absent"), "{d}");
    }

    // The late reply to ask 5 is receipted, not published.
    let late_deadline = Instant::now() + Duration::from_secs(8);
    loop {
        let text = std::fs::read_to_string(&receipts).unwrap_or_default();
        if text.contains("\"late\"") {
            break;
        }
        assert!(Instant::now() < late_deadline, "no late receipt");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    tokio::time::sleep(Duration::from_millis(100)).await;
    stop.store(true, Ordering::SeqCst);
    let recorded = tap.join().expect("tap");
    drop(eye);
    let answers_recorded = recorded
        .iter()
        .filter(|r| r.topic == jev.answer_topic)
        .count();
    assert_eq!(
        answers_recorded,
        live.len(),
        "every answer once, the late one never"
    );

    // The status file counts today's asks.
    let status = read_status(&state_dir).expect("status file");
    assert_eq!(status.asks_today, live.len() as u64);
    assert_eq!(status.looks_today, 7);
    assert_eq!(status.last_outcome.as_deref(), Some("disabled"));

    let bag = dir.path().join("run.mcap");
    write_bag(
        &bag,
        &recorded,
        &[
            (
                jev.frame_topic.clone(),
                "sensor_msgs/CompressedImage",
                CompressedImage::SCHEMA_HASH,
                CompressedImage::WIRE_FIXED_SIZE,
            ),
            (
                jev.ask_topic.clone(),
                "std_msgs/String",
                RosString::SCHEMA_HASH,
                RosString::WIRE_FIXED_SIZE,
            ),
            (
                jev.answer_topic.clone(),
                "std_msgs/String",
                RosString::SCHEMA_HASH,
                RosString::WIRE_FIXED_SIZE,
            ),
        ],
    );

    // No key bytes anywhere a run leaves behind.
    let _ = std::io::stderr().flush();
    let everything = [
        ("bag", std::fs::read(&bag).expect("bag")),
        ("receipts", std::fs::read(&receipts).expect("receipts")),
        (
            "status",
            std::fs::read(cerulion_rcpd::eye::switch::status_path(&state_dir)).expect("status"),
        ),
        ("logs", logs.bytes()),
    ];
    assert!(!everything[3].1.is_empty(), "the log capture ran");
    assert!(
        String::from_utf8_lossy(&everything[3].1).contains("so those frames leave this robot"),
        "the eye logged the data notice"
    );
    for (what, bytes) in &everything {
        assert!(!holds_key(bytes), "{what} holds key bytes");
    }
    // The stand-in route did see the key, so the scan can find it.
    assert!(stub
        .seen()
        .iter()
        .all(|s| holds_key(s.header("authorization").unwrap_or("").as_bytes())));

    // Resim: the recorded answers stand in for Jev. The stand-in route stays
    // up, so a request from the resim would be counted.
    let requests = stub.count();
    assert_eq!(requests, 7, "the live run sent seven requests");
    let reader = cerulion_bag::BagReader::open(&bag).expect("read bag");
    let asks = string_frames(&reader, &jev.ask_topic).expect("asks");
    assert_eq!(asks.len(), live.len());
    let mut recorded_answers = RecordedAnswers::from_bag(&bag, &jev.answer_topic).expect("answers");
    let mut replayed = Vec::new();
    for (_, ask) in &asks {
        replayed.push(recorded_answers.answer(ask).expect("an answer"));
    }
    let live_answers: Vec<&String> = live.iter().map(|(_, a)| a).collect();
    assert_eq!(
        replayed.iter().collect::<Vec<_>>(),
        live_answers,
        "the recorded answers are the live ones, byte for byte"
    );
    let replayed_decisions: Vec<String> = replayed.iter().map(|a| decide(a)).collect();
    assert_eq!(
        serde_json::to_vec(&replayed_decisions).expect("json"),
        serde_json::to_vec(&decisions).expect("json"),
        "resim reproduces the gate's decisions byte for byte"
    );
    let unknown = r#"{"ask_id":42,"question_set":"press"}"#;
    let missing = recorded_answers.answer(unknown).expect("answer");
    assert_eq!(
        decide(&missing),
        format!("42 {:?}", Look::Absent(Outcome::NotRecorded))
    );
    assert_eq!(recorded_answers.divergences(), 0);
    // A resim that asks about another frame under a recorded id is not
    // given the recorded answer.
    let mut again = RecordedAnswers::from_bag(&bag, &jev.answer_topic).expect("answers");
    let (_, first_ask) = &asks[0];
    let mut moved_ask: Value = serde_json::from_str(first_ask).expect("ask");
    moved_ask["frame"]["seq"] = json!(moved_ask["frame"]["seq"].as_u64().expect("seq") + 1);
    let diverged = again.answer(&moved_ask.to_string()).expect("answer");
    assert_eq!(
        decide(&diverged),
        format!("0 {:?}", Look::Absent(Outcome::NotRecorded))
    );
    assert_eq!(again.divergences(), 1);
    assert_eq!(
        again.answer(first_ask).as_deref(),
        Some(live[0].1.as_str()),
        "the recorded ask still gets its answer"
    );
    assert_eq!(stub.count(), requests, "the resim made no request");
    drop(stub);

    // `--verify-bag` joins every answer to its receipt and recomputes the
    // requests from the recorded frames.
    let topics = EyeTopics {
        ask: jev.ask_topic.clone(),
        answer: jev.answer_topic.clone(),
        frame: Some(jev.frame_topic.clone()),
    };
    let report = verify_bag(&bag, &receipts, &topics).expect("verify");
    assert!(report.problems.is_empty(), "{:?}", report.problems);
    assert_eq!(report.answers, live.len());
    assert_eq!(report.joined, live.len());
    assert_eq!(report.late, 1);
    assert_eq!(report.requests_checked, 7);
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_cerulion-rcpd"))
        .arg("--verify-bag")
        .arg(&bag)
        .arg("--manifest")
        .arg(&manifest)
        .arg("--receipts")
        .arg(&receipts)
        .output()
        .expect("run cerulion-rcpd");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "{stderr}");
    assert!(
        stderr.contains("10 of 10 answers have their receipt"),
        "{stderr}"
    );

    // A recording without its asks names answers to no ask.
    let no_asks = dir.path().join("no-asks.mcap");
    let answers_only: Vec<Recorded> = recorded
        .iter()
        .filter(|r| r.topic != jev.ask_topic)
        .cloned()
        .collect();
    write_bag(
        &no_asks,
        &answers_only,
        &[
            (
                jev.frame_topic.clone(),
                "sensor_msgs/CompressedImage",
                CompressedImage::SCHEMA_HASH,
                CompressedImage::WIRE_FIXED_SIZE,
            ),
            (
                jev.answer_topic.clone(),
                "std_msgs/String",
                RosString::SCHEMA_HASH,
                RosString::WIRE_FIXED_SIZE,
            ),
        ],
    );
    let report = verify_bag(&no_asks, &receipts, &topics).expect("verify");
    let orphans = report
        .problems
        .iter()
        .filter(|p| p.contains("has no ask in the recording"))
        .count();
    assert_eq!(orphans, live.len(), "{:?}", report.problems);

    // A late receipt whose request no timeout answer names follows nothing.
    let text = std::fs::read_to_string(&receipts).expect("receipts");
    let moved: Vec<String> = text
        .lines()
        .map(|l| {
            let mut entry: Value = serde_json::from_str(l).expect("receipt");
            if entry["outcome"]["detail"] == "late" {
                entry["args_digest"] = json!("0".repeat(64));
            }
            entry.to_string()
        })
        .collect();
    let cut = dir.path().join("moved-late.jsonl");
    std::fs::write(&cut, moved.join("\n") + "\n").expect("write");
    let report = verify_bag(&bag, &cut, &topics).expect("verify");
    assert!(
        report
            .problems
            .iter()
            .any(|p| p.contains("follows no timeout answer")),
        "{:?}",
        report.problems
    );

    // A receipt log that lost the newest eye receipt no longer joins.
    let text = std::fs::read_to_string(&receipts).expect("receipts");
    let mut lines: Vec<&str> = text.lines().collect();
    let last_answer = lines
        .iter()
        .rposition(|l| l.contains("\"disabled\""))
        .expect("the disabled receipt");
    lines.remove(last_answer);
    let cut = dir.path().join("cut.jsonl");
    std::fs::write(&cut, lines.join("\n") + "\n").expect("write");
    let report = verify_bag(&bag, &cut, &topics).expect("verify");
    assert!(
        report.problems.iter().any(|p| p.contains("has no receipt")),
        "{:?}",
        report.problems
    );

    // A frame edited after the looks no longer recomputes its image digest.
    let schemas = [
        (
            jev.frame_topic.clone(),
            "sensor_msgs/CompressedImage",
            CompressedImage::SCHEMA_HASH,
            CompressedImage::WIRE_FIXED_SIZE,
        ),
        (
            jev.ask_topic.clone(),
            "std_msgs/String",
            RosString::SCHEMA_HASH,
            RosString::WIRE_FIXED_SIZE,
        ),
        (
            jev.answer_topic.clone(),
            "std_msgs/String",
            RosString::SCHEMA_HASH,
            RosString::WIRE_FIXED_SIZE,
        ),
    ];
    let edited_frames: Vec<Recorded> = recorded
        .iter()
        .cloned()
        .map(|mut r| {
            if r.topic == jev.frame_topic {
                let run = r
                    .frame
                    .windows(64)
                    .position(|w| w.iter().all(|&b| b == 0x20))
                    .expect("the JPEG's padding");
                r.frame[run + 32] = 0x21;
            }
            r
        })
        .collect();
    let edited = dir.path().join("edited-frames.mcap");
    write_bag(&edited, &edited_frames, &schemas);
    let report = verify_bag(&edited, &receipts, &topics).expect("verify");
    assert!(
        report
            .problems
            .iter()
            .any(|p| p.contains("is not the image it sent")),
        "{:?}",
        report.problems
    );
    assert_eq!(report.requests_checked, 7);

    // An answer whose recorded state was edited, or dropped, fails the
    // request recompute; its receipt still joins.
    let edit_first_answer = |from: &[u8], to: &[u8]| -> Vec<Recorded> {
        let mut done = false;
        recorded
            .iter()
            .cloned()
            .map(|mut r| {
                if !done && r.topic == jev.answer_topic {
                    if let Some(at) = r.frame.windows(from.len()).position(|w| w == from) {
                        r.frame[at..at + to.len()].copy_from_slice(to);
                        done = true;
                    }
                }
                r
            })
            .collect()
    };
    let edited = dir.path().join("edited-state.mcap");
    write_bag(
        &edited,
        &edit_first_answer(b"Wrist camera", b"wrist camera"),
        &schemas,
    );
    let report = verify_bag(&edited, &receipts, &topics).expect("verify");
    assert_eq!(report.joined, live.len());
    assert!(
        report
            .problems
            .iter()
            .any(|p| p.contains("request_sha256 does not recompute")),
        "{:?}",
        report.problems
    );
    let edited = dir.path().join("dropped-state.mcap");
    write_bag(
        &edited,
        &edit_first_answer(b"\"state\":{", b"\"stat_\":{"),
        &schemas,
    );
    let report = verify_bag(&edited, &receipts, &topics).expect("verify");
    assert!(
        report
            .problems
            .iter()
            .any(|p| p.contains("does not record its model, state and questions")),
        "{:?}",
        report.problems
    );
    assert_eq!(
        report.requests_checked, 6,
        "the request without a state was not checked"
    );

    // An answer edited onto another ask's text no longer matches the
    // recorded ask with its id; its receipt still joins.
    let mut done = false;
    let moved_ask: Vec<Recorded> = recorded
        .iter()
        .cloned()
        .map(|mut r| {
            let field = b"\"ask_sha256\":\"";
            if !done && r.topic == jev.answer_topic {
                if let Some(at) = r.frame.windows(field.len()).position(|w| w == field) {
                    let hex = &mut r.frame[at + field.len()];
                    *hex = if *hex == b'0' { b'1' } else { b'0' };
                    done = true;
                }
            }
            r
        })
        .collect();
    assert!(done, "the first answer carries ask_sha256");
    let edited = dir.path().join("moved-ask.mcap");
    write_bag(&edited, &moved_ask, &schemas);
    let report = verify_bag(&edited, &receipts, &topics).expect("verify");
    assert_eq!(report.joined, live.len());
    let mismatched = report
        .problems
        .iter()
        .filter(|p| p.contains("is the digest of no recorded ask"))
        .count();
    assert_eq!(mismatched, 1, "{:?}", report.problems);
}
