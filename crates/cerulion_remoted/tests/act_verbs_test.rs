// SPDX-License-Identifier: AGPL-3.0-only
//! The act verbs over this machine's iceoryx2 shared memory, called the way
//! the ops server calls them (`VerbHandler::execute` with the request's JSON
//! args): `topic-publish` reaches a live subscriber, `topic-echo-once` and
//! `topic-snapshot` read a live publisher's frame, and `service-call` gets an
//! answer from `cerulion_core`'s `ServiceServer` (the server `rmw_cerulion`
//! wraps) built from a `.srv` in the `CERULION_REMOTED_SCHEMAS_DIR` store.
//! On a topic a raw iceoryx2 node created with safe overflow disabled, a
//! publish whose frames meet a full queue returns and reports the skipped
//! deliveries, and an e-stop engaged during a publish or a call there
//! completes within a bound.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::Duration;

use cerud::verbs::VerbHandler;
use cerud::CerudError;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_msgio::text::{body_to_json, message_frame};
use cerulion_remoted::act_verbs::{
    ServiceCallVerb, TopicEchoOnceVerb, TopicPublishVerb, TopicSnapshotVerb, SCHEMAS_DIR_ENV,
};
use iceoryx2::service::port_factory::PortFactory as _;
use serde_json::json;
use serial_test::serial;

/// One store for the whole binary: the verbs read the directory from the
/// environment, which every test here sets to this same path.
fn store() -> &'static std::path::Path {
    static DIR: std::sync::OnceLock<tempfile::TempDir> = std::sync::OnceLock::new();
    let dir = DIR.get_or_init(|| {
        let dir = tempfile::tempdir().expect("tempdir");
        let srv = dir.path().join("demo_srvs/srv");
        std::fs::create_dir_all(&srv).unwrap();
        std::fs::write(
            srv.join("Toggle.srv"),
            "bool on\n---\nbool ok\nstring note\n",
        )
        .unwrap();
        std::fs::write(srv.join("Ping.srv"), "---\nbool ok\n").unwrap();
        std::env::set_var(SCHEMAS_DIR_ENV, dir.path());
        dir
    });
    dir.path()
}

fn lease() -> Arc<std::sync::Mutex<cerud::lease::ControlLease>> {
    Arc::new(std::sync::Mutex::new(cerud::lease::ControlLease::new(
        1_000_000_000,
    )))
}

fn walker() -> cerulion_core::codegen::FrameWalker {
    cerulion_msgio::checked_walker(cerulion_msgio::store_schemas(Some(store()))).unwrap()
}

/// Publish `text` as `ty` on `topic` every 20 ms until `stop`.
fn publish_until(
    topic: &'static str,
    ty: &'static str,
    text: String,
    stop: Arc<AtomicBool>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let w = walker();
        let frame = message_frame(&w, ty, &text, 77).unwrap();
        let header = WireHeader::read_from_buf(&frame).unwrap();
        let transport = cerulion_core::TransportManager::get_or_init().unwrap();
        let injector = transport
            .create_ingress_injector(
                topic,
                header.schema_hash,
                MaxSliceLen::try_new(frame.len() as u32).unwrap(),
            )
            .unwrap();
        while !stop.load(Ordering::SeqCst) {
            let _ = injector.reinject_raw(&frame);
            std::thread::sleep(Duration::from_millis(20));
        }
    })
}

#[test]
#[serial]
fn topic_publish_reaches_a_live_subscriber() {
    store();
    let topic = "/c1_act/publish/twist";
    let transport = cerulion_core::TransportManager::get_or_init().unwrap();
    let subscriber = transport.create_subscriber(topic).unwrap();
    let reader = std::thread::spawn(move || {
        let mut got = Vec::new();
        for _ in 0..100 {
            subscriber
                .wait_for_message(Duration::from_millis(100), |m| {
                    got.push(m.payload().to_vec())
                })
                .unwrap();
            if !got.is_empty() {
                break;
            }
        }
        got
    });
    let result = TopicPublishVerb::new(lease())
        .execute(&json!({"topic": topic, "type": "geometry_msgs/msg/Twist",
                         "message": {"linear": {"x": 0.5}}, "wait_subscribers_s": 5}))
        .unwrap();
    assert_eq!(result["frames"], 1);
    assert_eq!(result["subscribers"], 1);
    assert_eq!(
        result["line"],
        format!("published: topic={topic} type=geometry_msgs/Twist frames=1 subscribers=1")
    );
    let got = reader.join().unwrap();
    let mut want = 0.5f64.to_le_bytes().to_vec();
    want.extend_from_slice(&[0u8; 40]);
    assert_eq!(got, vec![want]);
}

#[test]
#[serial]
fn topic_echo_once_decodes_a_live_publishers_frame() {
    store();
    let stop = Arc::new(AtomicBool::new(false));
    let publisher = publish_until(
        "/c1_act/echo/status",
        "std_msgs/String",
        r#"{"data": "ready"}"#.to_string(),
        stop.clone(),
    );
    std::thread::sleep(Duration::from_millis(200));
    let result = TopicEchoOnceVerb
        .execute(&json!({"topic": "/c1_act/echo/status", "timeout_s": 5}))
        .unwrap();
    stop.store(true, Ordering::SeqCst);
    publisher.join().unwrap();
    assert_eq!(result["type"], "std_msgs/String");
    assert_eq!(result["stamp_ns"], 77);
    assert_eq!(result["message"], json!({"data": "ready"}));
}

#[test]
#[serial]
fn topic_snapshot_returns_a_jpeg_of_a_live_camera_frame() {
    store();
    let stop = Arc::new(AtomicBool::new(false));
    let data: Vec<u8> = (0..4 * 2 * 3).map(|i| (i * 9) as u8).collect();
    let text = format!(
        "{{\"header\": {{\"stamp\": {{\"sec\": 1, \"nanosec\": 2}}}}, \"height\": 2, \"width\": 4, \
         \"encoding\": \"rgb8\", \"step\": 12, \"data\": {data:?}}}"
    );
    let publisher = publish_until(
        "/c1_act/snapshot/cam",
        "sensor_msgs/Image",
        text,
        stop.clone(),
    );
    std::thread::sleep(Duration::from_millis(200));
    let result = TopicSnapshotVerb
        .execute(&json!({"topic": "/c1_act/snapshot/cam", "timeout_s": 5}))
        .unwrap();
    stop.store(true, Ordering::SeqCst);
    publisher.join().unwrap();
    assert_eq!(
        (
            &result["width"],
            &result["height"],
            &result["encoding"],
            &result["stamp_ns"]
        ),
        (&json!(4), &json!(2), &json!("rgb8"), &json!(1_000_000_002))
    );
    let b64 = result["jpeg_base64"].as_str().unwrap();
    assert!(
        b64.starts_with("/9j/"),
        "a JPEG starts FF D8 FF: {}",
        &b64[..8]
    );
    assert_eq!(
        result["bytes"].as_u64().unwrap() as usize,
        b64.len() / 4 * 3 - b64.matches('=').count()
    );
}

#[test]
#[serial]
fn topic_snapshot_of_a_missing_topic_is_unavailable() {
    store();
    let err = TopicSnapshotVerb
        .execute(&json!({"topic": "/c1_act/snapshot/none", "timeout_s": 0.3}))
        .unwrap_err();
    match err {
        CerudError::Verb(m) => assert!(m.starts_with("unavailable: "), "{m}"),
        other => panic!("{other:?}"),
    }
}

#[test]
#[serial]
fn service_call_gets_the_answer_of_a_core_service_server() {
    let dir = store().to_path_buf();
    let stop = Arc::new(AtomicBool::new(false));
    let ready = Arc::new(AtomicBool::new(false));
    let (s, r) = (stop.clone(), ready.clone());
    let server = std::thread::spawn(move || {
        let types =
            cerulion_msgio::service::resolve_store_service_types("demo_srvs/Toggle", Some(&dir))
                .unwrap();
        let rq = types.walker.schema_hash_for(&types.request).unwrap();
        let rs = types.walker.schema_hash_for(&types.response).unwrap();
        let transport = cerulion_core::TransportManager::get_or_init().unwrap();
        let mut server = transport
            .create_service_server(
                "/c1_act/toggle",
                MaxSliceLen::const_new(4 * 1024 * 1024),
                rq,
                rs,
            )
            .unwrap();
        let reply = message_frame(
            &types.walker,
            &types.response,
            r#"{"ok": true, "note": "on"}"#,
            0,
        )
        .unwrap();
        r.store(true, Ordering::SeqCst);
        let mut seen = Vec::new();
        while !s.load(Ordering::SeqCst) {
            let mut got = None;
            server
                .try_take_one_request(|env, body| {
                    got = Some((env, body_to_json(&types.walker, rq, body).unwrap()))
                })
                .unwrap();
            match got {
                Some((env, json)) => {
                    seen.push(json);
                    server
                        .send_response(&env, &reply[WireHeader::SIZE..])
                        .unwrap();
                }
                None => std::thread::sleep(Duration::from_millis(2)),
            }
        }
        seen
    });
    while !ready.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(2));
    }
    let result = ServiceCallVerb::new(lease())
        .execute(
            &json!({"service": "/c1_act/toggle", "type": "demo_srvs/srv/Toggle",
                         "request": "on: true", "timeout_s": 10}),
        )
        .unwrap();
    stop.store(true, Ordering::SeqCst);
    assert_eq!(server.join().unwrap(), vec![r#"{"on":true}"#.to_string()]);
    assert_eq!(
        result,
        json!({"response": {"ok": true, "note": "on"},
               "response_text": r#"{"ok":true,"note":"on"}"#})
    );
}

#[test]
#[serial]
fn service_call_with_no_server_is_unavailable() {
    store();
    let err = ServiceCallVerb::new(lease())
        .execute(
            &json!({"service": "/c1_act/nobody", "type": "demo_srvs/srv/Ping",
                         "timeout_s": 0.3}),
        )
        .unwrap_err();
    match err {
        CerudError::Verb(m) => assert!(m.starts_with("unavailable: "), "{m}"),
        other => panic!("{other:?}"),
    }
}

#[test]
#[serial]
fn an_estop_engaged_during_the_server_wait_stops_the_call_as_unavailable() {
    store();
    let l = lease();
    let engager = {
        let l = l.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(300));
            l.lock().unwrap().engage_estop("test");
        })
    };
    let started = std::time::Instant::now();
    let err = ServiceCallVerb::new(l)
        .execute(
            &json!({"service": "/act_verbs_test/call/estop", "type": "demo_srvs/srv/Ping",
                         "timeout_s": 10}),
        )
        .unwrap_err();
    engager.join().unwrap();
    match err {
        CerudError::Verb(m) => assert!(
            m.starts_with("unavailable: ") && m.contains("e-stop engaged"),
            "{m}"
        ),
        other => panic!("{other:?}"),
    }
    assert!(
        started.elapsed() < Duration::from_secs(5),
        "the wait ended at the e-stop"
    );
}

#[test]
#[serial]
fn an_estop_engaged_after_the_latch_checks_and_before_the_send_sends_no_request() {
    let dir = store().to_path_buf();
    let service = "/act_verbs_test/call/estop_at_send";
    let l = lease();
    let caller = {
        let l = l.clone();
        std::thread::spawn(move || {
            ServiceCallVerb::new(l).execute(
                &json!({"service": service, "type": "demo_srvs/srv/Ping", "timeout_s": 10}),
            )
        })
    };
    // The call opens its request publisher after the handler's latch check
    // and the watcher's start, then waits for a server.
    let transport = cerulion_core::TransportManager::get_or_init().unwrap();
    let request = cerulion_core::transport::service::request_topic(service);
    let mut polls = 0;
    while transport.topic_publisher_count(&request) == 0 {
        polls += 1;
        assert!(polls < 5000, "the call never opened its client");
        std::thread::sleep(Duration::from_millis(2));
    }
    // Holding the lease keeps the watcher from reading the latch, so the
    // flag it clears stays set while the server comes up.
    let mut held = l.lock().unwrap();
    held.engage_estop("test");
    let types =
        cerulion_msgio::service::resolve_store_service_types("demo_srvs/Ping", Some(&dir)).unwrap();
    let server = transport
        .create_service_server(
            service,
            MaxSliceLen::const_new(4 * 1024 * 1024),
            types.walker.schema_hash_for(&types.request).unwrap(),
            types.walker.schema_hash_for(&types.response).unwrap(),
        )
        .unwrap();
    // A call that reads only that flag sends within milliseconds of the
    // server appearing; one that reads the latch under the lease's lock at
    // the send waits for this test to release it.
    let window = std::time::Instant::now() + Duration::from_secs(1);
    while !server.has_pending_request() && std::time::Instant::now() < window {
        std::thread::sleep(Duration::from_millis(2));
    }
    drop(held);
    let err = caller.join().unwrap().unwrap_err();
    // The client's publish lands in the server's queue before it returns.
    assert!(
        !server.has_pending_request(),
        "a request reached the server after the e-stop engaged"
    );
    match err {
        CerudError::Verb(m) => assert!(
            m.starts_with("unavailable: ") && m.contains("e-stop engaged"),
            "{m}"
        ),
        other => panic!("{other:?}"),
    }
}

#[test]
#[serial]
fn an_estop_engaged_after_the_latch_checks_and_before_a_frame_publishes_no_frame() {
    store();
    let topic = "/act_verbs_test/publish/estop_at_send";
    let l = lease();
    let caller = {
        let l = l.clone();
        std::thread::spawn(move || {
            TopicPublishVerb::new(l).execute(&json!({"topic": topic, "type": "std_msgs/String",
                         "message": {"data": "x"}, "wait_subscribers_s": 10}))
        })
    };
    // The publish opens its publisher after the handler's latch check and
    // the watcher's start, then waits for a subscriber.
    let transport = cerulion_core::TransportManager::get_or_init().unwrap();
    let mut polls = 0;
    while transport.topic_publisher_count(topic) == 0 {
        polls += 1;
        assert!(polls < 5000, "the publish never opened its publisher");
        std::thread::sleep(Duration::from_millis(2));
    }
    // Holding the lease keeps the watcher from reading the latch, so the
    // flag it clears stays set while the subscriber comes up.
    let mut held = l.lock().unwrap();
    held.engage_estop("test");
    let subscriber = transport.create_subscriber(topic).unwrap();
    // A publish that reads only that flag sends its frame within one
    // subscriber poll (20 ms) of the subscriber appearing; one that reads the
    // latch under the lease's lock at each frame waits for this test to
    // release it.
    let mut got = Vec::new();
    let window = std::time::Instant::now() + Duration::from_secs(1);
    while got.is_empty() && std::time::Instant::now() < window {
        subscriber
            .wait_for_message(Duration::from_millis(20), |m| {
                got.push(m.payload().to_vec())
            })
            .unwrap();
    }
    drop(held);
    let err = caller.join().unwrap().unwrap_err();
    // A frame lands in the subscriber's queue before its publish returns.
    subscriber
        .try_receive(|m| got.push(m.payload().to_vec()))
        .unwrap();
    assert!(
        got.is_empty(),
        "{} frame(s) reached the subscriber after the e-stop engaged",
        got.len()
    );
    match err {
        CerudError::Verb(m) => assert!(
            m.starts_with("unavailable: ") && m.contains("e-stop engaged after 0 of 1"),
            "{m}"
        ),
        other => panic!("{other:?}"),
    }
}

#[test]
#[serial]
fn an_estop_engaged_during_the_subscriber_wait_stops_the_publish_as_unavailable() {
    store();
    let l = lease();
    let engager = {
        let l = l.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(300));
            l.lock().unwrap().engage_estop("test");
        })
    };
    let started = std::time::Instant::now();
    let err = TopicPublishVerb::new(l)
        .execute(
            &json!({"topic": "/c1_act/publish/estop", "type": "std_msgs/String",
                         "message": {"data": "x"}, "wait_subscribers_s": 10}),
        )
        .unwrap_err();
    engager.join().unwrap();
    match err {
        CerudError::Verb(m) => assert!(
            m.starts_with("unavailable: ") && m.contains("e-stop engaged after 0 of 1"),
            "{m}"
        ),
        other => panic!("{other:?}"),
    }
    assert!(
        started.elapsed() < Duration::from_secs(5),
        "the wait ended at the e-stop"
    );
}

type Raw = iceoryx2::service::ipc::Service;
type RawSubscriber = iceoryx2::port::subscriber::Subscriber<Raw, [u8], ()>;

/// How long a verb, or an e-stop engaged during one, may take in the
/// full-queue tests below before the test ends the wait and fails.
const BOUND: Duration = Duration::from_secs(20);

/// A topic's data service a raw iceoryx2 node creates with safe overflow
/// disabled, so a publisher's send meets a full queue instead of replacing
/// the oldest queued sample. Its port caps and buffer ceiling are the ones
/// Cerulion's openers require, so they accept it. `blocker` is a 1-deep
/// subscriber that never takes: iceoryx2 gives each publisher and
/// subscriber pair its own queue, so one frame from a publisher fills that
/// publisher's queue to it. A send waits on a full queue only while both
/// ends of it are attached; a subscriber attaches its end of a publisher's
/// queue when it is created after that publisher, or when it updates its
/// connections. `observer`, when present, is a subscriber the test drains.
struct NoOverflowTopic {
    blocker: Option<RawSubscriber>,
    observer: Option<RawSubscriber>,
    observed: usize,
    service: iceoryx2::service::port_factory::publish_subscribe::PortFactory<Raw, [u8], ()>,
    _node: iceoryx2::node::Node<Raw>,
}

impl NoOverflowTopic {
    /// The data service on `topic`, with no subscriber yet.
    fn create(topic: &str) -> Self {
        let transport = cerulion_core::TransportManager::get_or_init().unwrap();
        let caps = cerulion_core::transport::TopicServiceConfig::for_service_request(
            transport.subscriber_buffer_size(),
        );
        let node = iceoryx2::node::NodeBuilder::new()
            .config(&transport.iox_config())
            .create::<Raw>()
            .unwrap();
        let name: iceoryx2::service::service_name::ServiceName =
            format!("{topic}/data").as_str().try_into().unwrap();
        let service = node
            .service_builder(&name)
            .publish_subscribe::<[u8]>()
            .enable_safe_overflow(false)
            .subscriber_max_buffer_size(transport.subscriber_buffer_size())
            .max_publishers(caps.max_publishers.unwrap())
            .max_subscribers(caps.max_subscribers.unwrap())
            .create_with_attributes(
                &cerulion_core::transport::wire_header_attribute::data_service_specifier().unwrap(),
            )
            .unwrap();
        assert!(!service.static_config().has_safe_overflow());
        Self {
            blocker: None,
            observer: None,
            observed: 0,
            service,
            _node: node,
        }
    }

    /// Create the blocker and, when `observed`, the observer after it.
    /// Each is attached to the queue of every publisher on the service now.
    fn subscribe(&mut self, observed: bool) {
        self.blocker = Some(
            self.service
                .subscriber_builder()
                .buffer_size(1)
                .create()
                .unwrap(),
        );
        if observed {
            self.observer = Some(self.service.subscriber_builder().create().unwrap());
        }
    }

    /// Wait, within [`BOUND`], until a publisher is on the service.
    fn wait_for_publisher(&self, what: &str) {
        let deadline = std::time::Instant::now() + BOUND;
        while self.service.dynamic_config().number_of_publishers() == 0 {
            assert!(
                std::time::Instant::now() < deadline,
                "{what} opened no publisher within {BOUND:?}"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    /// What `rx` delivers within [`BOUND`]. Past it the blocker is dropped,
    /// which ends a send waiting on its queue, and the test fails.
    fn bounded<T>(&mut self, rx: &mpsc::Receiver<T>, what: &str) -> T {
        match rx.recv_timeout(BOUND) {
            Ok(value) => value,
            Err(_) => {
                drop(self.blocker.take());
                panic!("{what} did not return within {BOUND:?}: a send waited on the full queue");
            }
        }
    }

    /// Drain the observer until it has received `n` samples in all, within
    /// [`BOUND`]. Past it the blocker is dropped and the test fails.
    fn wait_observed(&mut self, n: usize, what: &str) {
        let deadline = std::time::Instant::now() + BOUND;
        while self.observed < n {
            let observer = self.observer.as_ref().unwrap();
            while self.observed < n && observer.receive().unwrap().is_some() {
                self.observed += 1;
            }
            if self.observed < n && std::time::Instant::now() >= deadline {
                drop(self.blocker.take());
                panic!(
                    "{what} delivered {} of {n} samples to the observer within {BOUND:?}: a \
                     send waited on the blocker's full queue",
                    self.observed
                );
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

/// `f`'s value over a channel, from a new thread.
fn in_thread<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> mpsc::Receiver<T> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx
}

#[test]
#[serial]
fn a_publish_into_a_full_queue_returns_and_reports_the_skipped_deliveries() {
    store();
    let topic = format!("/act_verbs_test/full_queue/publish_{}", std::process::id());
    let mut full = NoOverflowTopic::create(&topic);
    let publish = in_thread({
        let topic = topic.clone();
        move || {
            TopicPublishVerb::new(lease()).execute(&json!({"topic": topic,
                "type": "std_msgs/String", "message": {"data": "x"}, "times": 3,
                "rate_hz": 100, "wait_subscribers_s": 5}))
        }
    });
    // The publish waits for a subscriber; the blocker, created after its
    // publisher, is attached to that publisher's queue.
    full.wait_for_publisher("the publish");
    full.subscribe(false);
    let result = full.bounded(&publish, "the publish").unwrap();
    // Frame 1 fills the blocker's queue; frames 2 and 3 skip it.
    assert_eq!(
        (
            &result["frames"],
            &result["subscribers"],
            &result["skipped"]
        ),
        (&json!(3), &json!(0), &json!(2))
    );
    assert_eq!(
        result["line"],
        format!(
            "published: topic={topic} type=std_msgs/String frames=3 subscribers=0; 2 \
             deliveries were skipped because a subscriber's queue was full"
        )
    );
}

#[test]
#[serial]
fn an_estop_engaged_while_a_publish_meets_a_full_queue_completes_within_the_bound() {
    store();
    let topic = format!("/act_verbs_test/full_queue/estop_{}", std::process::id());
    let mut full = NoOverflowTopic::create(&topic);
    let l = lease();
    let publish = in_thread({
        let (l, topic) = (l.clone(), topic.clone());
        move || {
            TopicPublishVerb::new(l).execute(&json!({"topic": topic,
                "type": "std_msgs/String", "message": {"data": "x"}, "times": 500,
                "rate_hz": 50, "wait_subscribers_s": 5}))
        }
    });
    full.wait_for_publisher("the publish");
    full.subscribe(true);
    // The blocker's queue is full after the first frame it takes, so at
    // least two of the three frames the observer receives met that full
    // queue.
    full.wait_observed(3, "the publish");
    let engaged = in_thread({
        let l = l.clone();
        move || {
            l.lock().unwrap().engage_estop("test");
        }
    });
    full.bounded(&engaged, "engaging the e-stop");
    match full.bounded(&publish, "the publish").unwrap_err() {
        CerudError::Verb(m) => assert!(
            m.starts_with("unavailable: the robot's e-stop engaged after ")
                && m.contains("of 500 frames; ")
                && m.ends_with(" skipped because a subscriber's queue was full"),
            "{m}"
        ),
        other => panic!("{other:?}"),
    }
}

/// The call sends its one request through a request publisher it creates,
/// and iceoryx2 gives each publisher and subscriber pair its own queue, so
/// the request meets an empty queue at both servers even with safe overflow
/// disabled. The call then waits for a response neither sends, and an e-stop
/// engaged during that wait completes at once and ends the call.
#[test]
#[serial]
fn a_call_on_a_no_overflow_request_topic_is_bounded_and_an_estop_during_it_completes() {
    store();
    let service = format!("/act_verbs_test/full_queue/call_{}", std::process::id());
    let request = cerulion_core::transport::service::request_topic(&service);
    let mut full = NoOverflowTopic::create(&request);
    full.subscribe(true);
    let l = lease();
    let call = in_thread({
        let (l, service) = (l.clone(), service.clone());
        move || {
            ServiceCallVerb::new(l).execute(
                &json!({"service": service, "type": "demo_srvs/srv/Ping", "timeout_s": 10}),
            )
        }
    });
    full.wait_observed(1, "the call");
    let engaged = in_thread({
        let l = l.clone();
        move || {
            l.lock().unwrap().engage_estop("test");
        }
    });
    full.bounded(&engaged, "engaging the e-stop");
    match full.bounded(&call, "the call").unwrap_err() {
        CerudError::Verb(m) => assert!(
            m.starts_with("unavailable: the robot's e-stop engaged")
                && m.contains("stopped before its response arrived"),
            "{m}"
        ),
        other => panic!("{other:?}"),
    }
}
