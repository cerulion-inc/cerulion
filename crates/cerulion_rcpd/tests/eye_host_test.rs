// SPDX-License-Identifier: AGPL-3.0-only
//! The `rcp-eye` thread's own guarantees, on a private shared-memory root:
//!
//! - An ask flood is answered up to the ask bound and no further: the asks
//!   over it get no answer and no receipt.
//! - A key placed after the eye started still gets the data notice, logged
//!   before the first request leaves.
//! - An answer whose receipt cannot be written is not published.
//! - An eye with no client answers every look `unavailable`.
//!
//! One test per binary: it installs the process's log subscriber.

mod eye_common;

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_rcpd::eye::config::JevConfig;
use cerulion_rcpd::eye::host::{EyeHost, EyeSettings};
use cerulion_rcpd::eye::limits::{jpeg_header_for_test, ASKS_PER_SECOND, ASK_BURST};
use cerulion_rcpd::eye::switch::write_switch;
use cerulion_rcpd::eye::verdict::{EyeAnswer, Outcome};
use cerulion_rcpd::eye::{EYE_CALLER, JEV_VERB};
use cerulion_rcpd::manifest::RcpManifest;
use cerulion_rcpd::tools::ReceiptSink;
use eye_common::{good_body, Reply, Stub, ANSWERED_MODEL};
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

/// The gate's side of the eye's topics; it lives on the thread that made it.
struct Gate {
    frames: cerulion_core::CerulionPublisher,
    asks: cerulion_core::CerulionPublisher,
    answers: cerulion_core::CerulionSubscriber,
    frame_topic: String,
}

impl Gate {
    fn open(mgr: &TransportManager, jev: &JevConfig) -> Self {
        Self {
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
        }
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

    /// Every answer that arrives within `within`, by ask id.
    fn answers(&self, within: Duration) -> Vec<(u64, String)> {
        let deadline = Instant::now() + within;
        let mut out = Vec::new();
        while Instant::now() < deadline {
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
            std::thread::sleep(Duration::from_millis(1));
        }
        out
    }

    /// The answer to `id`, waiting at most `within`.
    fn answer(&self, id: u64, within: Duration) -> Option<String> {
        let deadline = Instant::now() + within;
        while Instant::now() < deadline {
            let mut found = None;
            self.answers
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
            if found.is_some() {
                return found;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        None
    }
}

fn outcome(answer: &str) -> (Outcome, Option<String>) {
    let parsed = EyeAnswer::parse(answer).expect("answer frame");
    let detail = serde_json::from_str::<Value>(answer).expect("json")["detail"]
        .as_str()
        .map(str::to_owned);
    (parsed.outcome, detail)
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
    let notice_before_request = Arc::new(AtomicBool::new(false));
    let seen_notice = Arc::clone(&notice_before_request);
    let script_logs = logs.clone();
    let stub = Stub::start(move |_| {
        if script_logs.text().contains(NOTICE) {
            seen_notice.store(true, Ordering::SeqCst);
        }
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
    let gate = std::thread::spawn(move || {
        let mut gate = Gate::open(&gate_mgr, &gate_jev);
        let seq = gate.publish_frame();

        // A flood of 1,000 asks with Jev off: each answered ask is
        // `disabled` at once, and only the ask bound's worth are answered
        // and receipted.
        write_switch(&gate_state, false).expect("off");
        let before = eye_receipts(&gate_receipts);
        let flood = Instant::now();
        for id in 0..1_000u64 {
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
            bound < 1_000 / 2,
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
        write_switch(&gate_state, true).expect("on");
        // A full ask bucket again.
        std::thread::sleep(Duration::from_millis(2_100));

        // No key: answered at once, nothing sent, no notice.
        gate.ask(1, seq);
        let first = gate.answer(1, Duration::from_secs(5)).expect("answer 1");
        assert_eq!(outcome(&first).0, Outcome::NoCredential);
        // The owner places the key: the next look sends, after the notice.
        cerulion_rcpd::eye::credential::write_credential(&gate_key, eye_common::TEST_KEY)
            .expect("key");
        let seq = gate.publish_frame();
        gate.ask(2, seq);
        let second = gate.answer(2, Duration::from_secs(5)).expect("answer 2");
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
        let fourth = gate.answer(4, Duration::from_secs(5)).expect("answer 4");
        assert_eq!(outcome(&fourth).0, Outcome::Ok, "{fourth}");
    });
    tokio::task::spawn_blocking(move || gate.join())
        .await
        .expect("join")
        .expect("gate");
    drop(eye);

    let text = logs.text();
    assert!(
        text.contains("dropped an ask over the ask bound"),
        "the flood was logged"
    );
    assert!(
        notice_before_request.load(Ordering::SeqCst),
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
        gate.answer(5, Duration::from_secs(5)).expect("answer 5")
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
