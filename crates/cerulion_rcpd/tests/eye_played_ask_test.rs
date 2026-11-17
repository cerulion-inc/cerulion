// SPDX-License-Identifier: AGPL-3.0-only
//! A running eye beside a played recording. `bag play` re-publishes recorded
//! frames byte for byte from the node `cerulion_bag_play`, on the topics the
//! eye reads. A publisher there whose node name begins `cerulion_replay_`,
//! the name a resim's transport takes, reads as a replay too; the test drives
//! one stamped by a replay clock at recorded times (a real resim runs on a
//! private shared-memory namespace and never reaches the eye). The eye
//! tells them by their publisher's node, so however their stamps fall
//! against the host clock (an hour back, an hour ahead, or a replay clock
//! sweeping through now), a played ask gets no request, no answer
//! frame and no receipt with Jev on or off, and a played frame is never
//! held. A live ask beside them is answered, and so is a live gate whose
//! clock runs 2 s behind the host's, as a run resumed after a pause stamps.
//!
//! One test per binary: it installs the process's log subscriber.

mod eye_common;

use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{Clock, TransportConfig, TransportManager, VirtualClock, WireHeader};
use cerulion_rcpd::eye::config::JevConfig;
use cerulion_rcpd::eye::host::{EyeHost, EyeSettings};
use cerulion_rcpd::eye::limits::jpeg_header_for_test;
use cerulion_rcpd::eye::source::{BAG_PLAY_NODE, RESIM_NODE_PREFIX};
use cerulion_rcpd::eye::switch::{read_status, write_switch};
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

/// What the eye logs for each played ask it drops.
const PLAYED: &str = "dropped an ask from a replay or an unnamed publisher";

/// What the eye logs once for each replay publisher it names.
const REPLAY_PUBLISHER: &str = "a replay publishes on an eye topic";

/// Long enough for the eye's loop to read everything published before it.
const SETTLE: Duration = Duration::from_millis(300);

/// The depth the ask topic is created with. The eye's subscriber takes the
/// depth of the topic it opens, and this one holds every ask the test
/// publishes, so an eye that stalls during the sweep reads the asks queued
/// behind the stall late instead of losing the oldest to newer ones.
const ASK_DEPTH: usize = 256;

/// The host's clock less a fixed lag: a run's clock after a pause of that
/// length.
struct Behind(u64);

impl Clock for Behind {
    fn now_ns(&self) -> u64 {
        cerulion_core::real_ns().saturating_sub(self.0)
    }
}

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

/// Wait until the status file in `state_dir` counts `n` asks dropped from
/// a replay, so a replay leaves only after the eye has read and named it.
/// A count other than `n` after 10 s fails with the whole status file.
fn wait_replayed(state_dir: &Path, n: usize) {
    let deadline = Instant::now() + Duration::from_secs(10);
    let counted = || read_status(state_dir).map_or(0, |s| s.replay_asks_today);
    while counted() < n as u64 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
    }
    let status = read_status(state_dir);
    let count = status.as_ref().map_or(0, |s| s.replay_asks_today);
    assert_eq!(
        count, n as u64,
        "the asks dropped from a replay so far; status: {status:?}"
    );
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
    // One shared-memory root for the eye, the live gates, the resim and the
    // player, each a transport of its own under its own node name, stamping
    // by its own clock.
    let root = cerulion_core::testing::iceoryx_test_config();
    let manager = |name: String, clock: Arc<dyn Clock>| {
        TransportManager::init_for_test(
            TransportConfig {
                node_name: name,
                clock,
                ..Default::default()
            },
            root.clone(),
        )
        .expect("transport")
    };
    let pid = std::process::id();
    let live_clock: Arc<dyn Clock> = Arc::new(cerulion_core::RealClock);
    let replay_clock = Arc::new(VirtualClock::new());
    let eye_mgr = manager(
        format!("rcpd_eye_played_eye_{pid}"),
        Arc::clone(&live_clock),
    );
    let live_mgr = manager(
        format!("rcpd_eye_played_live_{pid}"),
        Arc::clone(&live_clock),
    );
    let paused_mgr = manager(
        format!("rcpd_eye_played_paused_{pid}"),
        Arc::new(Behind(2_000_000_000)),
    );
    let resim_mgr = manager(
        format!("{RESIM_NODE_PREFIX}played_{pid}"),
        Arc::clone(&replay_clock) as Arc<dyn Clock>,
    );
    let player_mgr = manager(BAG_PLAY_NODE.to_owned(), live_clock);
    let sink = ReceiptSink::open(&receipts).expect("receipts");
    let eye =
        EyeHost::spawn(eye_mgr, settings, sink, tokio::runtime::Handle::current()).expect("eye");

    let gate_state = state_dir.clone();
    let gate = std::thread::spawn(move || {
        let state_dir = gate_state;
        let hour = 3_600_000_000_000_u64;
        let sent = || requests.load(Ordering::SeqCst);
        let mut played = 0_usize;
        // The ask topic is created deeper than the transport's default, as a
        // graph creates the topics its nodes publish, before any asker opens
        // it at the default.
        let mut deep = live_mgr.default_topic_config();
        deep.subscriber_max_buffer_size = ASK_DEPTH;
        let _ask_topic = live_mgr
            .precreate_topic_services(&jev.ask_topic, deep)
            .expect("the ask topic");
        let mut live = Asker::open(&live_mgr, &jev);
        eye_common::wait_for_subscribers(&live_mgr, &jev.frame_topic, 1);
        eye_common::wait_for_subscribers(&live_mgr, &jev.ask_topic, 1);
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
        let recorded_frames = frame_tap.take();
        let recorded_asks = ask_tap.take();
        assert_eq!((recorded_frames.len(), recorded_asks.len()), (1, 2));
        assert_eq!((sent(), eye_receipts(&receipts)), (2, 2));

        // An unpaced resim: its replay clock sweeps the recording's span at
        // 10x, from 10 s behind this host's clock to 5 s ahead, so it passes
        // through now. Every played ask names the live frame the eye holds,
        // so only its publisher keeps it off the route. A live ask is made
        // as the sweep crosses now.
        let mut resim = Asker::open(&resim_mgr, &jev);
        let mut last_played_seq = 0;
        let start = cerulion_core::real_ns() - 10_000_000_000;
        let t0 = Instant::now();
        let mut nearest_ms = i128::MAX;
        let mut live_asked = false;
        for id in 1_000_u64.. {
            let elapsed = u64::try_from(t0.elapsed().as_nanos()).expect("ns");
            let stamp = start + 10 * elapsed;
            let from_now_ms =
                (i128::from(stamp) - i128::from(cerulion_core::real_ns())) / 1_000_000;
            if from_now_ms > 5_000 {
                break;
            }
            replay_clock.set(stamp);
            last_played_seq = resim.publish_frame();
            resim.ask(id, held);
            played += 1;
            nearest_ms = nearest_ms.min(from_now_ms.abs());
            if from_now_ms >= 0 && !live_asked {
                live.ask(9, held);
                live_asked = true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let swept = played;
        assert!(swept >= 10, "the sweep played {swept} asks");
        // At least one played ask carries a stamp within 1 s of the host
        // clock, as a live ask's would.
        assert!(
            nearest_ms <= 1_000,
            "the played ask nearest the host clock was stamped {nearest_ms} ms from it"
        );
        assert_eq!(
            answers.outcome(9, Duration::from_secs(5)),
            Some(Outcome::Ok),
            "the live ask made during the sweep"
        );
        // The same resim stamping an hour back and an hour ahead.
        replay_clock.set(cerulion_core::real_ns().saturating_sub(hour));
        resim.ask(101, held);
        resim.ask(102, held);
        replay_clock.set(cerulion_core::real_ns() + hour);
        resim.ask(103, held);
        resim.ask(104, held);
        played += 4;
        std::thread::sleep(SETTLE);
        live.ask(2, held);
        assert_eq!(
            answers.outcome(2, Duration::from_secs(5)),
            Some(Outcome::Ok)
        );
        // The played frames were not held: a live ask for a sequence number
        // only the resim published waits for it and is refused.
        assert!(last_played_seq > held);
        live.ask(5, last_played_seq);
        assert_eq!(
            answers.outcome(5, Duration::from_secs(5)),
            Some(Outcome::Refused)
        );
        std::thread::sleep(SETTLE);
        for id in (1_000..1_000 + swept as u64).chain(101..=104) {
            assert_eq!(answers.count(id), 0, "played ask {id} was answered");
        }
        assert_eq!(sent(), 4, "only the live asks reached the route");
        assert_eq!(eye_receipts(&receipts), 5, "no receipt for a played ask");
        wait_replayed(&state_dir, played);
        drop(resim);

        // `bag play`: the recorded frame and asks byte for byte, at once.
        let mut player = Asker::open(&player_mgr, &jev);
        player.play(&recorded_frames, &recorded_asks);
        played += recorded_asks.len();
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
        assert_eq!(sent(), 5, "only the live asks reached the route");
        assert_eq!(eye_receipts(&receipts), 6, "no receipt for a played ask");
        wait_replayed(&state_dir, played);
        drop(player);

        // A live gate whose clock runs 2 s behind the host's, as a run's clock
        // does after a 2 s pause: its frame is held and its ask answered with
        // its whole deadline. Its third frame has a sequence number no other
        // live gate has used.
        let mut paused = Asker::open(&paused_mgr, &jev);
        let mut resumed_seq = 0;
        for _ in 0..3 {
            resumed_seq = paused.publish_frame();
        }
        assert!(resumed_seq > held);
        paused.ask(7, resumed_seq);
        assert_eq!(
            answers.outcome(7, Duration::from_secs(5)),
            Some(Outcome::Ok),
            "a resumed run's ask"
        );
        assert_eq!(sent(), 6);
        assert_eq!(eye_receipts(&receipts), 7);
        drop(paused);

        // Jev off: a live ask is answered `disabled` at once, a played one
        // not at all. A topic takes two publishers at once here, so each
        // replay leaves before the next one starts, once the eye has read it.
        write_switch(&state_dir, false).expect("off");
        let mut resim = Asker::open(&resim_mgr, &jev);
        replay_clock.set(cerulion_core::real_ns().saturating_sub(hour));
        resim.ask(201, held);
        resim.ask(202, held);
        played += 2;
        wait_replayed(&state_dir, played);
        drop(resim);
        let mut player = Asker::open(&player_mgr, &jev);
        player.play(&[], &recorded_asks);
        played += recorded_asks.len();
        wait_replayed(&state_dir, played);
        drop(player);
        live.ask(4, held);
        assert_eq!(
            answers.outcome(4, Duration::from_secs(5)),
            Some(Outcome::Disabled)
        );
        std::thread::sleep(SETTLE);
        for id in [201, 202] {
            assert_eq!(answers.count(id), 0, "played ask {id} was answered");
        }
        for id in [1, 6] {
            assert_eq!(answers.count(id), 1, "recorded ask {id} was answered again");
        }
        assert_eq!(sent(), 6, "nothing reached the route while off");
        assert_eq!(eye_receipts(&receipts), 8, "no receipt for a played ask");
        played
    });
    let played = tokio::task::spawn_blocking(move || gate.join())
        .await
        .expect("join")
        .expect("gate");
    drop(eye);
    assert_eq!(stub.count(), 6, "the route saw only the live asks");

    // The status file counts the played asks it dropped.
    let status = read_status(&state_dir).expect("status");
    assert_eq!(status.replay_asks_today, played as u64, "{status:?}");
    assert_eq!(status.dropped_asks_today, 0, "{status:?}");
    assert_eq!(status.asks_today, 8, "{status:?}");
    assert_eq!(status.looks_today, 6, "{status:?}");

    // One line per played ask, loud on the first and each decade only: a
    // live ask between played ones does not make the next one loud again.
    let text = logs.text();
    let lines: Vec<&str> = text.lines().filter(|l| l.contains(PLAYED)).collect();
    assert_eq!(lines.len(), played, "one line per played ask");
    let decades = 1 + usize::from(played >= 10) + usize::from(played >= 100);
    assert_eq!(
        lines.iter().filter(|l| l.contains("WARN")).count(),
        decades,
        "loud on the first and each decade: {played} played"
    );
    let named: Vec<&str> = text
        .lines()
        .filter(|l| l.contains(REPLAY_PUBLISHER))
        .collect();
    assert!(
        named.len() >= 2
            && named.iter().all(|l| l.contains("WARN")
                && (l.contains(RESIM_NODE_PREFIX) || l.contains(BAG_PLAY_NODE))),
        "each replay publisher named once at warn: {named:?}"
    );
    assert!(
        text.contains("dropped a wrist frame from a replay or an unnamed publisher"),
        "the played frames were dropped"
    );
}
