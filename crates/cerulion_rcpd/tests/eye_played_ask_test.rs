// SPDX-License-Identifier: AGPL-3.0-only
//! A running eye beside a played recording. `bag play --resim` publishes the
//! graph's asks stamped by its replay clock at the recorded times, and
//! `bag play` re-publishes recorded frames byte for byte, wire timestamp
//! included, on the topics the eye reads. Each such ask is stale against the
//! eye's clock: with Jev on or off it gets no request, no answer frame and
//! no receipt, and one log line. A played wrist frame is not held, so it is
//! never sent for a live ask. A live ask beside them is still answered.
//!
//! One test per binary: it installs the process's log subscriber.

mod eye_common;

use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, TransportManager, VirtualClock, WireHeader};
use cerulion_rcpd::eye::config::JevConfig;
use cerulion_rcpd::eye::host::{EyeHost, EyeSettings};
use cerulion_rcpd::eye::limits::jpeg_header_for_test;
use cerulion_rcpd::eye::switch::write_switch;
use cerulion_rcpd::eye::verdict::Outcome;
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

/// What the eye logs for each stale ask.
const STALE: &str = "dropped a stale ask";

/// Long enough for the eye's loop to read everything published before it.
const SETTLE: Duration = Duration::from_millis(300);

/// A publisher of wrist frames and asks; it lives on the thread that made it.
struct Asker {
    frames: cerulion_core::CerulionPublisher,
    asks: cerulion_core::CerulionPublisher,
    frame_topic: String,
}

impl Asker {
    fn open(mgr: &TransportManager, jev: &JevConfig) -> Self {
        Self {
            frames: mgr
                .create_publisher_simple(&jev.frame_topic, MaxSliceLen::const_new(512 * 1024))
                .expect("frame publisher"),
            asks: mgr
                .create_publisher_simple(&jev.ask_topic, MaxSliceLen::const_new(16 * 1024))
                .expect("ask publisher"),
            frame_topic: jev.frame_topic.clone(),
        }
    }

    fn publish_frame(&mut self) -> u32 {
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

    /// Re-publish recorded wire frames byte for byte, as `bag play` does.
    fn play(&mut self, frames: &[Vec<u8>], asks: &[Vec<u8>]) {
        for frame in frames {
            let _ = self.frames.publish_raw(frame).expect("play frame");
        }
        for ask in asks {
            let _ = self.asks.publish_raw(ask).expect("play ask");
        }
    }
}

/// A tap that keeps every frame on one topic as a bag stores it: wire
/// header, then the message.
struct Tap(cerulion_core::CerulionSubscriber);

impl Tap {
    fn take(&self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        self.0
            .try_receive(|m| {
                let mut frame = vec![0u8; WireHeader::SIZE];
                m.header().write_to_buf(&mut frame);
                frame.extend_from_slice(m.payload());
                out.push(frame);
            })
            .expect("tap");
        out
    }
}

/// The answer topic, read by ask id.
struct Answers {
    sub: cerulion_core::CerulionSubscriber,
    seen: Vec<(u64, String)>,
}

impl Answers {
    fn read(&mut self) {
        let seen = &mut self.seen;
        self.sub
            .try_receive(|m| {
                let text = RosString::build_reader(m.payload())
                    .data()
                    .expect("utf-8")
                    .to_owned();
                let id = serde_json::from_str::<Value>(&text).expect("json")["ask_id"]
                    .as_u64()
                    .expect("id");
                seen.push((id, text));
            })
            .expect("receive");
    }

    /// The outcome of the answer to `id`, waiting at most `within`.
    fn outcome(&mut self, id: u64, within: Duration) -> Option<Outcome> {
        let deadline = Instant::now() + within;
        while Instant::now() < deadline {
            self.read();
            if let Some((_, text)) = self.seen.iter().find(|(seen, _)| *seen == id) {
                let value = serde_json::from_str::<Value>(text).expect("json");
                return value["outcome"].as_str().and_then(Outcome::parse);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        None
    }

    /// How many answers to `id` arrived.
    fn count(&mut self, id: u64) -> usize {
        self.read();
        self.seen.iter().filter(|(seen, _)| *seen == id).count()
    }
}

/// The eye receipts in the log at `path`.
fn eye_receipts(path: &Path) -> usize {
    cerud::receipt::read_all(path)
        .expect("receipts")
        .iter()
        .filter(|r| r.caller == EYE_CALLER && r.verb == JEV_VERB)
        .count()
}

#[tokio::test(flavor = "multi_thread")]
async fn a_played_ask_gets_no_request_no_answer_and_no_receipt() {
    let logs = install_logs();
    let requests = Arc::new(AtomicUsize::new(0));
    let route_requests = Arc::clone(&requests);
    let stub = Stub::start(move |_| {
        route_requests.fetch_add(1, Ordering::SeqCst);
        Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.68))
    })
    .await;
    let dir = tempfile::tempdir().expect("tempdir");
    let prefix = format!("rcpd_eye_played_{}", std::process::id());
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
    // One shared-memory root for the eye, the live gate, the resim and the
    // player, each a transport of its own stamping by its own clock.
    let root = cerulion_core::testing::iceoryx_test_config();
    let manager = |tag: &str, clock: Arc<dyn cerulion_core::Clock>| {
        TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("rcpd_eye_played_{tag}_{}", std::process::id()),
                clock,
                ..Default::default()
            },
            root.clone(),
        )
        .expect("transport")
    };
    let live_clock: Arc<dyn cerulion_core::Clock> = Arc::new(cerulion_core::RealClock);
    let replay_clock = Arc::new(VirtualClock::new());
    let eye_mgr = manager("eye", Arc::clone(&live_clock));
    let live_mgr = manager("live", Arc::clone(&live_clock));
    let resim_mgr = manager(
        "resim",
        Arc::clone(&replay_clock) as Arc<dyn cerulion_core::Clock>,
    );
    let player_mgr = manager("play", live_clock);
    let sink = ReceiptSink::open(&receipts).expect("receipts");
    let eye =
        EyeHost::spawn(eye_mgr, settings, sink, tokio::runtime::Handle::current()).expect("eye");

    let gate = std::thread::spawn(move || {
        let hour = 3_600_000_000_000_u64;
        let sent = || requests.load(Ordering::SeqCst);
        let mut live = Asker::open(&live_mgr, &jev);
        let mut answers = Answers {
            sub: live_mgr
                .create_subscriber(&jev.answer_topic)
                .expect("answer subscriber"),
            seen: Vec::new(),
        };
        let frame_tap = Tap(live_mgr
            .create_subscriber(&jev.frame_topic)
            .expect("frame tap"));
        let ask_tap = Tap(live_mgr.create_subscriber(&jev.ask_topic).expect("ask tap"));

        // Two live looks, recorded by the taps.
        let held = live.publish_frame();
        live.ask(1, held);
        live.ask(6, held);
        assert_eq!(
            answers.outcome(1, Duration::from_secs(5)),
            Some(Outcome::Ok)
        );
        assert_eq!(
            answers.outcome(6, Duration::from_secs(5)),
            Some(Outcome::Ok)
        );
        let recorded_at = Instant::now();
        let recorded_frames = frame_tap.take();
        let recorded_asks = ask_tap.take();
        assert_eq!((recorded_frames.len(), recorded_asks.len()), (1, 2));
        assert_eq!((sent(), eye_receipts(&receipts)), (2, 2));

        // A resim: the graph's asks stamped by the replay clock at recorded
        // times, an hour before this host's clock and an hour after it (a
        // recording from another boot), with re-injected wrist frames. The
        // played asks name the live frame the eye holds, so only their wire
        // time keeps them off the route.
        let mut resim = Asker::open(&resim_mgr, &jev);
        let mut played = Vec::new();
        replay_clock.set(cerulion_core::real_ns().saturating_sub(hour));
        for _ in 0..3 {
            played.push(resim.publish_frame());
        }
        resim.ask(101, held);
        resim.ask(102, held);
        replay_clock.set(cerulion_core::real_ns() + hour);
        played.push(resim.publish_frame());
        resim.ask(103, held);
        resim.ask(104, held);
        // The eye reads the played asks before the live one.
        std::thread::sleep(SETTLE);
        live.ask(2, held);
        assert_eq!(
            answers.outcome(2, Duration::from_secs(5)),
            Some(Outcome::Ok)
        );
        // The played frames were not held: a live ask for a sequence number
        // only the resim published waits for it and is refused.
        let only_played = played
            .iter()
            .copied()
            .find(|seq| *seq != held)
            .expect("a sequence number the live gate never published");
        live.ask(5, only_played);
        assert_eq!(
            answers.outcome(5, Duration::from_secs(5)),
            Some(Outcome::Refused)
        );
        std::thread::sleep(SETTLE);
        for id in 101..=104 {
            assert_eq!(answers.count(id), 0, "played ask {id} was answered");
        }
        assert_eq!(sent(), 3, "only the live asks reached the route");
        assert_eq!(eye_receipts(&receipts), 4, "no receipt for a played ask");
        drop(resim);

        // `bag play`: the recorded frame and asks byte for byte, once the
        // recorded asks are older than their deadline.
        let past_deadline = Duration::from_millis(1_600);
        if let Some(wait) = past_deadline.checked_sub(recorded_at.elapsed()) {
            std::thread::sleep(wait);
        }
        let mut player = Asker::open(&player_mgr, &jev);
        player.play(&recorded_frames, &recorded_asks);
        std::thread::sleep(SETTLE);
        let held = live.publish_frame();
        live.ask(3, held);
        assert_eq!(
            answers.outcome(3, Duration::from_secs(5)),
            Some(Outcome::Ok)
        );
        std::thread::sleep(SETTLE);
        for id in [1, 6] {
            assert_eq!(answers.count(id), 1, "recorded ask {id} was answered again");
        }
        assert_eq!(sent(), 4, "only the live asks reached the route");
        assert_eq!(eye_receipts(&receipts), 5, "no receipt for a played ask");
        drop(player);

        // Jev off: a live ask is answered `disabled` at once, a played one
        // not at all.
        write_switch(&state_dir, false).expect("off");
        let mut resim = Asker::open(&resim_mgr, &jev);
        replay_clock.set(cerulion_core::real_ns().saturating_sub(hour));
        resim.ask(201, held);
        resim.ask(202, held);
        std::thread::sleep(SETTLE);
        live.ask(4, held);
        assert_eq!(
            answers.outcome(4, Duration::from_secs(5)),
            Some(Outcome::Disabled)
        );
        std::thread::sleep(SETTLE);
        for id in [201, 202] {
            assert_eq!(answers.count(id), 0, "played ask {id} was answered");
        }
        assert_eq!(sent(), 4, "nothing reached the route while off");
        assert_eq!(eye_receipts(&receipts), 6, "no receipt for a played ask");
    });
    tokio::task::spawn_blocking(move || gate.join())
        .await
        .expect("join")
        .expect("gate");
    drop(eye);
    assert_eq!(stub.count(), 4, "the route saw only the live asks");

    let text = logs.text();
    let lines: Vec<&str> = text.lines().filter(|l| l.contains(STALE)).collect();
    assert_eq!(lines.len(), 8, "one line per played ask: {text}");
    assert_eq!(
        lines.iter().filter(|l| l.contains("WARN")).count(),
        3,
        "loud once per run of played asks, then counted: {text}"
    );
    assert!(
        text.contains("dropped a wrist frame stamped outside this host's clock"),
        "the played frames were logged: {text}"
    );
}
