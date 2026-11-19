// SPDX-License-Identifier: AGPL-3.0-only
//! The `rcp-eye` thread's own guarantees, on a private shared-memory root:
//!
//! - An ask flood is answered up to the ask bound and no further: the asks
//!   over it get no answer and no receipt, and each is counted as dropped.
//! - A key placed after the eye started still gets the data notice, logged
//!   before the first request leaves.
//! - An answer whose receipt cannot be written is not published.
//! - An eye with no client answers every look `unavailable`.
//! - An eye started before the graph creates neither the ask topic nor the
//!   frame topic: the graph creates them at its own depth, and the eye opens
//!   them then and answers.
//!
//! The first test installs the process's log subscriber and reads every
//! line; the second runs with no key on file, so it logs no data notice. The
//! third starts no eye: it pins the answer wait the other two use, which
//! reads the answer topic once more when its budget has run out.

mod eye_common;

use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_rcpd::eye::config::JevConfig;
use cerulion_rcpd::eye::host::{EyeHost, EyeSettings};
use cerulion_rcpd::eye::limits::{jpeg_header_for_test, ASKS_PER_SECOND, ASK_BURST};
use cerulion_rcpd::eye::switch::{read_status, write_switch};
use cerulion_rcpd::eye::verdict::Outcome;
use cerulion_rcpd::eye::{EYE_CALLER, JEV_VERB};
use cerulion_rcpd::manifest::RcpManifest;
use cerulion_rcpd::tools::ReceiptSink;
use eye_common::{good_body, Reply, Stub, ANSWERED_MODEL, EYE_WAIT};
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
    fn text(&self) -> String {
        String::from_utf8_lossy(&self.0.lock().unwrap_or_else(PoisonError::into_inner)).into_owned()
    }
}

fn install_logs() -> Logs {
    use tracing_subscriber::util::SubscriberInitExt as _;
    let logs = Logs::default();
    let writer = logs.clone();
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .with_ansi(false)
        .with_writer(move || writer.clone())
        .finish()
        .try_init()
        .expect("the one log subscriber");
    logs
}

/// The words every data notice line holds.
const NOTICE: &str = "so those frames leave this robot";

/// The asks in the flood.
const FLOOD: u64 = 1_000;

/// The gate's side of the eye's topics; it lives on the thread that made it.
struct Gate {
    frames: cerulion_core::CerulionPublisher,
    asks: cerulion_core::CerulionPublisher,
    answers: cerulion_core::CerulionSubscriber,
    frame_topic: String,
}

impl Gate {
    /// The gate's ports, once the eye subscribes to the frame and ask
    /// topics.
    fn open(mgr: &TransportManager, jev: &JevConfig) -> Self {
        let gate = Self {
            frames: mgr
                .create_publisher_simple(&jev.frame_topic, MaxSliceLen::const_new(512 * 1024))
                .expect("frame publisher"),
            asks: mgr
                .create_publisher_simple(&jev.ask_topic, MaxSliceLen::const_new(16 * 1024))
                .expect("ask publisher"),
            answers: mgr
                .create_subscriber(&jev.answer_topic)
                .expect("answer subscriber"),
            frame_topic: jev.frame_topic.clone(),
        };
        eye_common::wait_for_subscribers(mgr, &jev.frame_topic, 1);
        eye_common::wait_for_subscribers(mgr, &jev.ask_topic, 1);
        gate
    }

    fn publish_frame(&mut self) -> u32 {
        let seq = self.frames.sequence();
        let mut proxy = self
            .frames
            .loan_proxy::<CompressedImage>()
            .expect("loan frame");
        // Every variable field is written, or the frame is not published.
        proxy
            .with_header(|h| {
                h.set_frame_id("wrist")?;
                Ok(())
            })
            .expect("header");
        proxy.set_format("jpeg").expect("format");
        proxy
            .set_data(&jpeg_header_for_test(640, 480, 30_000))
            .expect("data");
        drop(proxy);
        seq
    }

    fn ask(&mut self, id: u64, seq: u32) {
        let ask = json!({
            "ask_id": id,
            "question_set": "press",
            "frame": { "topic": self.frame_topic, "seq": seq },
            "deadline_ms": 1_500,
            "numbers": { "case": 0 },
        });
        let mut proxy = self.asks.loan_proxy::<RosString>().expect("loan ask");
        proxy.set_data(&ask.to_string()).expect("set");
    }

    /// Every answer that arrives until `within` after the first one, by ask
    /// id, waiting at most [`EYE_WAIT`] for the first. The topic is read once
    /// more after the deadline has passed, as in [`answer_on`].
    fn answers(&self, within: Duration) -> Vec<(u64, String)> {
        let mut deadline = Instant::now() + EYE_WAIT;
        let mut opened = false;
        let mut out = Vec::new();
        loop {
            let last = Instant::now() >= deadline;
            self.answers
                .try_receive(|m| {
                    let text = RosString::build_reader(m.payload())
                        .data()
                        .expect("utf-8")
                        .to_owned();
                    let id = serde_json::from_str::<Value>(&text).expect("json")["ask_id"]
                        .as_u64()
                        .expect("id");
                    out.push((id, text));
                })
                .expect("receive");
            if !opened && !out.is_empty() {
                opened = true;
                deadline = Instant::now() + within;
            } else if last {
                return out;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// The answer to `id`, waiting at most `within`.
    fn answer(&self, id: u64, within: Duration) -> Option<String> {
        answer_on(&self.answers, id, within)
    }
}

/// The answer to `id` on `answers`, waiting at most `within`; answers to
/// other asks are read and dropped. The topic is read once more after the
/// deadline has passed, so a thread that was not scheduled until after it
/// still returns an answer queued meanwhile.
fn answer_on(
    answers: &cerulion_core::CerulionSubscriber,
    id: u64,
    within: Duration,
) -> Option<String> {
    let deadline = Instant::now() + within;
    loop {
        let last = Instant::now() >= deadline;
        let mut found = None;
        answers
            .try_receive(|m| {
                let text = RosString::build_reader(m.payload())
                    .data()
                    .expect("utf-8")
                    .to_owned();
                if serde_json::from_str::<Value>(&text).expect("json")["ask_id"] == id {
                    found = Some(text);
                }
            })
            .expect("receive");
        if found.is_some() || last {
            return found;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// The outcome and detail the eye published in an answer frame.
fn outcome(answer: &str) -> (Outcome, Option<String>) {
    let value = serde_json::from_str::<Value>(answer).expect("json");
    let outcome = value["outcome"]
        .as_str()
        .and_then(Outcome::parse)
        .expect("a known outcome");
    (outcome, value["detail"].as_str().map(str::to_owned))
}

/// The eye receipts in the log at `path`.
fn eye_receipts(path: &Path) -> usize {
    cerud::receipt::read_all(path)
        .expect("receipts")
        .iter()
        .filter(|r| r.caller == EYE_CALLER && r.verb == JEV_VERB)
        .count()
}

fn manager(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("rcpd_eye_host_{tag}_{}", std::process::id()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("transport")
}

#[tokio::test(flavor = "multi_thread")]
async fn the_eye_bounds_asks_gives_the_notice_first_and_publishes_only_receipted_answers() {
    let logs = install_logs();
    // Whether the notice was in the logs when the FIRST request reached the
    // route; later requests leave it as the first one set it.
    let notice_at_first_request: Arc<Mutex<Option<bool>>> = Arc::new(Mutex::new(None));
    let seen_notice = Arc::clone(&notice_at_first_request);
    let requests = Arc::new(AtomicUsize::new(0));
    let script_requests = Arc::clone(&requests);
    let script_logs = logs.clone();
    let stub = Stub::start(move |_| {
        script_requests.fetch_add(1, Ordering::SeqCst);
        seen_notice
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get_or_insert_with(|| script_logs.text().contains(NOTICE));
        Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.68))
    })
    .await;
    let dir = tempfile::tempdir().expect("tempdir");
    let prefix = format!("rcpd_eye_host_{}", std::process::id());
    // The key file the manifest names does not exist yet.
    let key = dir.path().join("robot-jev.key");
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
    let mgr = manager("main");
    let sink = ReceiptSink::open(&receipts).expect("receipts");
    let eye = EyeHost::spawn(
        mgr.clone(),
        settings.clone(),
        sink.clone(),
        tokio::runtime::Handle::current(),
    )
    .expect("eye");
    assert!(
        !logs.text().contains(NOTICE),
        "no notice while no key is on file"
    );

    let gate_mgr = mgr.clone();
    let gate_jev = jev.clone();
    let gate_state = state_dir.clone();
    let gate_receipts = receipts.clone();
    let gate_key = key.clone();
    let gate_sink = sink.clone();
    let gate_requests = Arc::clone(&requests);
    let gate = std::thread::spawn(move || {
        // The graph creates the ask topic deep enough to hold the whole
        // flood, so the eye reads every ask and the ask bound alone decides
        // which are answered. At the transport's default depth, below the
        // bound's burst, the queue drops the oldest asks while the eye writes
        // its synced receipts, and slow receipts let the refill cover every
        // ask the eye does read.
        let mut deep = gate_mgr.default_topic_config();
        deep.subscriber_max_buffer_size = FLOOD as usize;
        let _ask_topic = gate_mgr
            .precreate_topic_services(&gate_jev.ask_topic, deep)
            .expect("the graph creates its ask topic");
        let mut gate = Gate::open(&gate_mgr, &gate_jev);
        let seq = gate.publish_frame();

        // A flood of 1,000 asks with Jev off: each answered ask is
        // `disabled` at once, and only the ask bound's worth are answered
        // and receipted. One ask is answered first, so the eye reads the ask
        // topic and the gate the answer topic before the flood is counted:
        // `Gate::open` saw the eye's ask subscriber before this ask is
        // published, and the eye's answer publisher connects to the gate's
        // subscriber when it sends. The eye writes the answer's receipt
        // before it publishes the answer, so `before` counts it.
        write_switch(&gate_state, false).expect("off");
        gate.ask(9_000, seq);
        // A receipt with no answer means the answer was published late or
        // not at all; a dropped or replay ask count, that the eye refused the
        // ask; neither, that the eye had not reached it.
        let first = gate.answer(9_000, EYE_WAIT).unwrap_or_else(|| {
            panic!(
                "no answer to ask 9000: {} eye receipts, status {:?}",
                eye_receipts(&gate_receipts),
                read_status(&gate_state)
            )
        });
        assert_eq!(outcome(&first).0, Outcome::Disabled);
        let before = eye_receipts(&gate_receipts);
        let flood = Instant::now();
        for id in 0..FLOOD {
            gate.ask(10_000 + id, seq);
            if id % 10 == 9 {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        let published = flood.elapsed();
        let answered = gate.answers(Duration::from_millis(600));
        let flood_receipts = eye_receipts(&gate_receipts) - before;
        let elapsed = flood.elapsed();
        // The burst, and at most the refill over the time elapsed.
        let bound = ASK_BURST as usize
            + (f64::from(ASKS_PER_SECOND) * elapsed.as_secs_f64()).ceil() as usize;
        assert!(
            bound < FLOOD as usize / 2,
            "the flood outran the ask bound ({published:?} to publish 1,000 asks)"
        );
        assert!(
            flood_receipts <= bound,
            "{flood_receipts} receipts for a flood, bound {bound}"
        );
        assert!(
            flood_receipts >= 1,
            "the eye answered the flood's first asks"
        );
        assert!(!answered.is_empty(), "the flood's first asks were answered");
        assert!(
            answered.len() <= flood_receipts,
            "no answer without a receipt"
        );
        assert!(answered
            .iter()
            .all(|(_, a)| outcome(a).0 == Outcome::Disabled));
        // The eye reads every queued ask in one pass and answers the ones it
        // took (here `disabled`, at once) before it reads again, so a flood
        // ask still queued when ask 1 arrives is read with it and can spend
        // the token ask 1 needs. Wait until the receipts and the status
        // file's drop count cover every flood ask.
        let deadline = Instant::now() + EYE_WAIT;
        loop {
            let last = Instant::now() >= deadline;
            let dropped = read_status(&gate_state).map_or(0, |s| s.dropped_asks_today);
            let settled = dropped + (eye_receipts(&gate_receipts) - before) as u64;
            if settled >= FLOOD {
                break;
            }
            assert!(
                !last,
                "{settled} of {FLOOD} flood asks receipted or counted as dropped"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
        write_switch(&gate_state, true).expect("on");
        // A full ask bucket again.
        std::thread::sleep(Duration::from_millis(2_100));

        // No key: answered at once, nothing sent, no notice.
        gate.ask(1, seq);
        let first = gate.answer(1, EYE_WAIT).expect("answer 1");
        assert_eq!(outcome(&first).0, Outcome::NoCredential);
        // Ask 1 is read after every ask of the flood, so each flood ask the
        // eye answered is receipted by now, before ask 1's own receipt.
        let flood_answered = eye_receipts(&gate_receipts) - before - 1;
        // The owner places the key: the next look sends, after the notice.
        cerulion_rcpd::eye::credential::write_credential(&gate_key, eye_common::TEST_KEY)
            .expect("key");
        assert_eq!(
            gate_requests.load(Ordering::SeqCst),
            0,
            "ask 2 makes the run's first request"
        );
        let seq = gate.publish_frame();
        gate.ask(2, seq);
        let second = gate.answer(2, EYE_WAIT).expect("answer 2");
        assert_eq!(outcome(&second).0, Outcome::Ok, "{second}");

        // A receipt that cannot be written keeps its answer unpublished.
        gate_sink.fail_next_sync_for_test();
        let seq = gate.publish_frame();
        gate.ask(3, seq);
        assert!(
            gate.answer(3, Duration::from_millis(2_000)).is_none(),
            "an unreceipted answer was published"
        );
        // The next answer is receipted and published.
        let seq = gate.publish_frame();
        gate.ask(4, seq);
        let fourth = gate.answer(4, EYE_WAIT).expect("answer 4");
        assert_eq!(outcome(&fourth).0, Outcome::Ok, "{fourth}");
        flood_answered
    });
    let flood_answered = tokio::task::spawn_blocking(move || gate.join())
        .await
        .expect("join")
        .expect("gate");
    drop(eye);

    // Every ask of the flood was answered with a receipt or dropped and
    // counted; the stopped eye wrote its counts.
    let status = read_status(&state_dir).expect("status file");
    assert_eq!(
        status.dropped_asks_today + flood_answered as u64,
        FLOOD,
        "{flood_answered} flood asks answered: {status:?}"
    );
    let text = logs.text();
    assert!(
        text.contains("dropped an ask over the ask bound"),
        "the flood was logged"
    );
    assert_eq!(
        *notice_at_first_request
            .lock()
            .unwrap_or_else(PoisonError::into_inner),
        Some(true),
        "the notice was logged before the first request reached the route"
    );
    assert_eq!(text.matches(NOTICE).count(), 1, "one notice line: {text}");
    assert!(
        text.contains("answer not published: its receipt could not be written"),
        "the receipt failure was logged"
    );
    assert_eq!(
        stub.count(),
        3,
        "asks 2, 3 and 4 were sent; the flood and ask 1 were not"
    );

    // An eye whose client could not be built answers `unavailable`.
    let mut no_client = settings;
    no_client.endpoint = None;
    no_client.ask_topic = format!("/{prefix}/b/eye/ask");
    no_client.answer_topic = format!("/{prefix}/b/eye/answer");
    if let Some(jev) = no_client.jev.as_mut() {
        jev.frame_topic = format!("/{prefix}/b/wrist/image");
        jev.ask_topic.clone_from(&no_client.ask_topic);
        jev.answer_topic.clone_from(&no_client.answer_topic);
    }
    let b_jev = no_client.jev.clone().expect("jev");
    let mgr_b = manager("b");
    let eye_b = EyeHost::spawn(
        mgr_b.clone(),
        no_client,
        sink.clone(),
        tokio::runtime::Handle::current(),
    )
    .expect("eye without a client");
    let gate = std::thread::spawn(move || {
        let mut gate = Gate::open(&mgr_b, &b_jev);
        let seq = gate.publish_frame();
        gate.ask(5, seq);
        gate.answer(5, EYE_WAIT).expect("answer 5")
    });
    let answer = tokio::task::spawn_blocking(move || gate.join())
        .await
        .expect("join")
        .expect("gate");
    drop(eye_b);
    assert_eq!(
        outcome(&answer),
        (Outcome::Unavailable, Some(String::from("no_client")))
    );
    assert_eq!(stub.count(), 3, "nothing sent without a client");
    // The receipt log still takes the tools' receipts.
    assert!(sink
        .record(
            "tester",
            "arm.peek",
            "0",
            cerud::receipt::ReceiptOutcome::Ok
        )
        .is_some());
}

#[tokio::test(flavor = "multi_thread")]
async fn an_eye_started_before_the_graph_leaves_the_graphs_topics_to_the_graph() {
    let dir = tempfile::tempdir().expect("tempdir");
    let prefix = format!("rcpd_eye_order_{}", std::process::id());
    // No key on file: the ask is answered `no_credential`, with no request.
    let key = dir.path().join("robot-jev.key");
    let manifest = eye_common::write_manifest(
        dir.path(),
        &eye_common::eyes_yaml(&prefix, "http://127.0.0.1:9/api/v1", &key),
    );
    let loaded = RcpManifest::load_full(&manifest).expect("manifest");
    let jev = loaded.eyes.jev.clone().expect("eyes.jev");
    let state_dir = dir.path().join("state");
    std::fs::create_dir_all(&state_dir).expect("state dir");
    let settings =
        EyeSettings::resolve(&loaded.eyes, dir.path(), &state_dir, None).expect("settings");
    let mgr = manager("order");
    let sink = ReceiptSink::open(&state_dir.join("receipts.jsonl")).expect("receipts");
    let eye = EyeHost::spawn(
        mgr.clone(),
        settings,
        sink,
        tokio::runtime::Handle::current(),
    )
    .expect("eye");
    let gate_mgr = mgr.clone();
    let gate_jev = jev.clone();
    let gate = std::thread::spawn(move || {
        // The graph creates the topics its nodes publish, deeper than the
        // transport's default.
        let mut deep = gate_mgr.default_topic_config();
        deep.subscriber_max_buffer_size *= 4;
        let _frame_topic = gate_mgr
            .precreate_topic_services(&gate_jev.frame_topic, deep)
            .expect("the graph creates its frame topic");
        let _ask_topic = gate_mgr
            .precreate_topic_services(&gate_jev.ask_topic, deep)
            .expect("the graph creates its ask topic");
        let mut gate = Gate::open(&gate_mgr, &gate_jev);
        let seq = gate.publish_frame();
        gate.ask(1, seq);
        gate.answer(1, EYE_WAIT).expect("answer 1")
    });
    let answer = tokio::task::spawn_blocking(move || gate.join())
        .await
        .expect("join")
        .expect("gate");
    drop(eye);
    assert_eq!(
        outcome(&answer),
        (
            Outcome::NoCredential,
            Some(String::from("credential_missing"))
        )
    );
}

#[test]
fn a_wait_whose_budget_has_run_out_still_returns_the_answer_already_queued() {
    let mgr = manager("queued");
    let topic = format!("/rcpd_eye_queued_{}/eye/answer", std::process::id());
    let answers = mgr.create_subscriber(&topic).expect("answer subscriber");
    let mut stand_in = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(16 * 1024))
        .expect("answer publisher");
    let other = r#"{"ask_id":8,"outcome":"disabled","detail":"switched_off"}"#;
    let seven = r#"{"ask_id":7,"outcome":"disabled","detail":"switched_off"}"#;
    for text in [other, seven] {
        let mut proxy = stand_in.loan_proxy::<RosString>().expect("loan answer");
        proxy.set_data(text).expect("set");
    }
    // A thread first scheduled after its deadline: the budget is spent
    // before the first read, and the answer to 7 is already queued.
    assert_eq!(
        answer_on(&answers, 7, Duration::ZERO).as_deref(),
        Some(seven)
    );
}
