// SPDX-License-Identifier: AGPL-3.0-only
//! The eye's admission rules and its HTTP client against a local stand-in
//! route: every failure in the design's table gives Jev absent with its code,
//! a late answer is dropped, a timed-out request keeps its in-flight slot
//! until its late reply, another model is `wrong_model`, and the robot's own
//! limits refuse before any request is sent.

mod eye_common;

use std::path::Path;
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use base64::Engine as _;
use cerulion_rcpd::eye::answer::Answer;
use cerulion_rcpd::eye::client::{sha256_hex, JevClient};
use cerulion_rcpd::eye::config::EyesConfig;
use cerulion_rcpd::eye::host::{
    ask_sent, request, Admit, Completed, EyeCore, EyeSettings, FrameStore,
};
use cerulion_rcpd::eye::limits::jpeg_header_for_test;
use cerulion_rcpd::eye::switch::write_switch;
use cerulion_rcpd::eye::verdict::{BoxEdge, EyeAnswer, Look, Outcome};
use cerulion_rcpd::manifest::RcpManifest;
use eye_common::{eyes_yaml, good_body, key_file, Reply, Stub, ANSWERED_MODEL, TEST_KEY};
use serde_json::{json, Value};

/// The eye's rules for a manifest routed to `base_url`, with a key on file.
fn core(dir: &Path, base_url: &str) -> (EyeCore, Arc<JevClient>) {
    let key = key_file(dir);
    let manifest = eye_common::write_manifest(dir, &eyes_yaml("stub", base_url, &key));
    let loaded = RcpManifest::load_full(&manifest).expect("manifest");
    let settings = EyeSettings::resolve(&loaded.eyes, dir, dir, None).expect("settings");
    let client =
        Arc::new(JevClient::new(settings.endpoint.clone().expect("endpoint")).expect("client"));
    (EyeCore::new(Arc::new(settings), Instant::now()), client)
}

fn frames() -> FrameStore {
    let mut store = FrameStore::default();
    store.push(1, &jpeg_header_for_test(640, 480, 2_000));
    store.push(2, &jpeg_header_for_test(1280, 720, 2_000));
    store
}

fn ask(id: u64, case: u64, seq: u32, deadline_ms: u64) -> String {
    json!({
        "ask_id": id,
        "question_set": "press",
        "frame": { "topic": "/stub/wrist/image", "seq": seq },
        "deadline_ms": deadline_ms,
        "numbers": { "case": case },
    })
    .to_string()
}

/// Admit `text` now; when it yields a request, send it and return the
/// answer the gate gets and, later, a late answer if one came. Each report
/// is taken as the eye thread takes it ([`Completed::taken_at`]).
async fn look(core: &mut EyeCore, client: &Arc<JevClient>, text: &str) -> (Answer, Option<Answer>) {
    let now = Instant::now();
    match core.admit(text, now, now, now, &frames()) {
        Admit::Answer(answer) => (*answer, None),
        Admit::Send(dispatch) => {
            let (tx, rx) = mpsc::channel();
            request(Arc::clone(client), *dispatch, tx).await;
            let mut answer = None;
            let mut late = None;
            let taken: Vec<Completed> = rx
                .try_iter()
                .flat_map(|done| done.taken_at(Instant::now()))
                .collect();
            for done in taken {
                match done {
                    Completed::Answer {
                        answer: a,
                        retry_after,
                        ..
                    } => {
                        let _ =
                            core.note(a.outcome, retry_after, Instant::now(), SystemTime::now());
                        answer = Some(*a);
                    }
                    Completed::Late { answer: a, .. } => late = Some(*a),
                    Completed::Abandoned { .. } => {}
                }
            }
            (answer.expect("an answer"), late)
        }
        other => panic!("unexpected admission: {other:?}"),
    }
}

/// The stub's reply per `case` number in the request's state.
fn scripted(body: &Value) -> Reply {
    let case = body["state"]["numbers"]["case"].as_u64().unwrap_or(0);
    match case {
        0 => Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.68)),
        1 => Reply::ok(good_body("typesafe/jev-1.15-20261201", 0.93, 0.68)),
        2 => Reply::ok(json!({ "model": ANSWERED_MODEL, "usage": { "input_tokens": 10 } })),
        3 => Reply::ok(good_body(ANSWERED_MODEL, 0.2, 0.68)),
        4 => Reply::status(
            401,
            json!({ "error": { "code": 401, "message": "No auth credentials found" } }),
        ),
        5 => Reply::status(
            403,
            json!({ "error": { "code": 403, "message": "Key is disabled" } }),
        ),
        6 => Reply::status(
            403,
            json!({ "error": { "code": 403, "message": "Key limit exceeded" } }),
        ),
        7 => Reply::status(
            402,
            json!({ "error": { "code": 402, "message": "Insufficient credits" } }),
        ),
        8 => Reply::status(400, json!({ "detail": "bad request" })),
        9 => Reply::status(422, json!({ "detail": [] })),
        10 => Reply::status(500, json!({})),
        11 => Reply::status(529, json!({ "error": "overloaded" })),
        12 => {
            let mut r = Reply::status(429, json!({}));
            r.retry_after = Some(30);
            r
        }
        13 => Reply::ok(good_body(ANSWERED_MODEL, 0.93, 0.68)).after(Duration::from_millis(600)),
        14 => Reply::status(200, json!({ "not": "an answer" })),
        _ => Reply::status(418, json!({})),
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn every_failure_in_the_table_is_absent_with_its_code() {
    let stub = Stub::start(scripted).await;
    let cases = [
        (2, Outcome::Malformed),
        (1, Outcome::WrongModel),
        (3, Outcome::NotVisible),
        (4, Outcome::NoCredential),
        (5, Outcome::NoCredential),
        (6, Outcome::Quota),
        (7, Outcome::Quota),
        (8, Outcome::Refused),
        (9, Outcome::Refused),
        (10, Outcome::Unavailable),
        (11, Outcome::Unavailable),
        (14, Outcome::Malformed),
        (15, Outcome::Refused),
    ];
    let rim = |a: &EyeAnswer| a.box_edge_px("rim", BoxEdge::Bottom);
    for (i, (case, want)) in cases.into_iter().enumerate() {
        // A fresh robot per case: a quota answer holds the next ones back.
        let dir = tempfile::tempdir().expect("tempdir");
        let (mut core, client) = core(dir.path(), &stub.base_url());
        let asked = ask(i as u64, case, 1, 1_500);
        let (answer, late) = look(&mut core, &client, &asked).await;
        assert_eq!(answer.outcome, want, "case {case}: {answer:?}");
        assert!(late.is_none());
        let parsed = EyeAnswer::parse(&answer.to_json(), &asked).expect("parse");
        assert_eq!(
            Look::compare(&parsed, rim, 326.4, 1_000.0),
            Look::Absent(want),
            "case {case} must not count as agreement"
        );
    }
    assert_eq!(stub.count(), cases.len(), "one request per case");
    // A refused connection is `unavailable`.
    let closed = tokio::net::TcpListener::bind("127.0.0.1:0")
        .await
        .expect("bind");
    let gone = format!("http://{}/api", closed.local_addr().expect("addr"));
    drop(closed);
    let other = tempfile::tempdir().expect("tempdir");
    let (mut offline, offline_client) = core(other.path(), &gone);
    let (answer, _) = look(&mut offline, &offline_client, &ask(99, 0, 1, 1_500)).await;
    assert_eq!(answer.outcome, Outcome::Unavailable);
    assert_eq!(answer.detail.as_deref(), Some("connect"));
}

#[tokio::test(flavor = "multi_thread")]
async fn an_answer_sends_the_pinned_model_one_image_and_the_images_header() {
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, client) = core(dir.path(), &stub.base_url());
    let asked = ask(1, 0, 1, 1_500);
    let (answer, _) = look(&mut core, &client, &asked).await;
    assert_eq!(answer.outcome, Outcome::Ok, "{answer:?}");
    assert_eq!(answer.model_answered.as_deref(), Some(ANSWERED_MODEL));
    let parsed = EyeAnswer::parse(&answer.to_json(), &asked).expect("parse");
    let rim = |a: &EyeAnswer| a.box_edge_px("rim", BoxEdge::Bottom);
    assert_eq!(Look::compare(&parsed, rim, 326.4, 8.0), Look::Agree);
    assert_eq!(Look::compare(&parsed, rim, 360.0, 8.0), Look::Disagree);
    let seen = &stub.seen()[0];
    assert_eq!(seen.path, "/api/v1/systemone");
    assert_eq!(seen.header("typesafe-danger-zone-images"), Some("true"));
    assert_eq!(
        seen.header("authorization"),
        Some(format!("Bearer {TEST_KEY}").as_str())
    );
    let body = seen.json();
    assert_eq!(body["model"], "typesafe/jev-1.14");
    let assets = body["assets"].as_object().expect("assets");
    assert_eq!(assets.len(), 1, "one image");
    let (placeholder, asset) = assets.iter().next().expect("asset");
    assert_eq!(body["state"]["frame"], placeholder.as_str());
    let image = base64::engine::general_purpose::STANDARD
        .decode(asset["data_base64"].as_str().expect("b64"))
        .expect("decode");
    assert_eq!(
        image,
        jpeg_header_for_test(640, 480, 2_000),
        "the frame's bytes, unchanged"
    );
    assert_eq!(
        answer.image_sha256.as_deref(),
        Some(sha256_hex(&image).as_str())
    );
    assert_eq!(
        answer.request_sha256.as_deref(),
        Some(sha256_hex(&seen.body).as_str())
    );
    // The frame records what was sent, less the image bytes and the key.
    let frame = answer.to_json();
    assert!(!frame.contains(asset["data_base64"].as_str().expect("b64")));
    assert!(!eye_common::holds_key(frame.as_bytes()));
    assert_eq!(answer.state.as_ref(), Some(&body["state"]));
    assert_eq!(answer.questions.as_ref(), Some(&body["questions"]));
}

#[tokio::test(flavor = "multi_thread")]
async fn a_late_answer_is_dropped_and_the_gate_sees_a_timeout() {
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, client) = core(dir.path(), &stub.base_url());
    let started = Instant::now();
    let (answer, late) = look(&mut core, &client, &ask(1, 13, 1, 200)).await;
    assert_eq!(answer.outcome, Outcome::Timeout);
    assert!(answer.latency_ms.expect("latency") < 600, "{answer:?}");
    let late = late.expect("the late reply is reported");
    assert_eq!(late.outcome, Outcome::Ok, "it would have been an answer");
    assert!(started.elapsed() >= Duration::from_millis(600));
    assert_eq!(answer.request_sha256, late.request_sha256);
}

/// The next report on `rx`, waiting at most 10 s.
async fn next(rx: &mpsc::Receiver<Completed>) -> Completed {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if let Ok(done) = rx.try_recv() {
            return done;
        }
        assert!(Instant::now() < deadline, "no report within 10 s");
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn a_timed_out_request_keeps_its_in_flight_slot_until_its_late_reply() {
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, client) = core(dir.path(), &stub.base_url());
    let store = frames();
    let now = Instant::now();
    let (tx, rx) = mpsc::channel();
    let mut tasks = Vec::new();
    // Two requests whose replies take 600 ms, against a 200 ms deadline.
    for id in 1..=2 {
        let Admit::Send(dispatch) = core.admit(&ask(id, 13, 1, 200), now, now, now, &store) else {
            panic!("request {id} is sent");
        };
        tasks.push(tokio::spawn(request(
            Arc::clone(&client),
            *dispatch,
            tx.clone(),
        )));
    }
    drop(tx);
    for _ in 0..2 {
        let Completed::Answer { answer, .. } = next(&rx).await else {
            panic!("each ask is answered first");
        };
        assert_eq!(answer.outcome, Outcome::Timeout, "{answer:?}");
    }
    // Both have timed out and are still on the network: a third is
    // throttled for the in-flight limit.
    assert_eq!(core.in_flight().count(), 2);
    let later = Instant::now();
    let Admit::Answer(third) = core.admit(&ask(3, 0, 1, 1_500), later, later, later, &store) else {
        panic!("the third request must be refused");
    };
    assert_eq!(
        (third.outcome, third.detail.as_deref()),
        (Outcome::Throttled, Some("in_flight"))
    );
    for _ in 0..2 {
        let Completed::Late { answer, .. } = next(&rx).await else {
            panic!("the late reply follows its timeout");
        };
        assert_eq!(answer.outcome, Outcome::Ok, "{answer:?}");
    }
    for task in tasks {
        task.await.expect("request task");
    }
    // The late replies freed both slots.
    assert_eq!(core.in_flight().count(), 0);
    let after = Instant::now();
    assert!(matches!(
        core.admit(&ask(4, 0, 1, 1_500), after, after, after, &store),
        Admit::Send(_)
    ));
    assert_eq!(stub.count(), 2, "only the two slow requests were sent");
}

#[tokio::test(flavor = "multi_thread")]
async fn a_reply_taken_at_its_deadline_is_a_timeout_and_the_reply_is_late() {
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, _client) = core(dir.path(), "http://127.0.0.1:9/api/v1");
    let now = Instant::now();
    let Admit::Send(dispatch) = core.admit(&ask(1, 0, 1, 200), now, now, now, &frames()) else {
        panic!("the request is sent");
    };
    let mut reply = dispatch.prepared.answer(Outcome::Ok, None, Some(5));
    reply.model_answered = Some(String::from(ANSWERED_MODEL));
    reply.answers = Some(json!({ "edge_visible": { "type": "noul", "noul": 0.93 } }));
    let report = |deadline| Completed::Answer {
        answer: Box::new(reply.clone()),
        retry_after: None,
        deadline: Some(deadline),
    };
    let deadline = dispatch.deadline;
    // Taken before the deadline: published as is.
    let before = report(deadline).taken_at(deadline - Duration::from_millis(1));
    let [Completed::Answer { answer, .. }] = before.as_slice() else {
        panic!("{before:?}");
    };
    assert_eq!(**answer, reply);
    // Taken at the deadline: the ask's answer is `timeout`, the reply late.
    let at = report(deadline).taken_at(deadline);
    let [Completed::Answer {
        answer: timeout,
        deadline: None,
        ..
    }, Completed::Late { answer: late, .. }] = at.as_slice()
    else {
        panic!("{at:?}");
    };
    assert_eq!(
        (timeout.outcome, timeout.detail.as_deref()),
        (Outcome::Timeout, Some("deadline"))
    );
    assert!(timeout.answers.is_none() && timeout.model_answered.is_none());
    assert_eq!(timeout.request_sha256, reply.request_sha256);
    assert_eq!(**late, reply);
    // The `timeout` answer given at the deadline is never turned again.
    let given = Completed::Answer {
        answer: Box::new((**timeout).clone()),
        retry_after: None,
        deadline: None,
    };
    assert_eq!(given.taken_at(deadline + Duration::from_secs(1)).len(), 1);
}

#[tokio::test(flavor = "multi_thread")]
async fn a_429_backs_off_without_a_retry() {
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, client) = core(dir.path(), &stub.base_url());
    let (answer, _) = look(&mut core, &client, &ask(1, 12, 1, 1_500)).await;
    assert_eq!(answer.outcome, Outcome::RateLimited);
    let (next, _) = look(&mut core, &client, &ask(2, 0, 1, 1_500)).await;
    assert_eq!(next.outcome, Outcome::RateLimited);
    assert_eq!(next.detail.as_deref(), Some("backing_off"));
    assert_eq!(stub.count(), 1, "no request while backing off");
}

#[tokio::test(flavor = "multi_thread")]
async fn a_spend_limit_holds_every_look_until_midnight_utc() {
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, client) = core(dir.path(), &stub.base_url());
    let (answer, _) = look(&mut core, &client, &ask(1, 7, 1, 1_500)).await;
    assert_eq!(answer.outcome, Outcome::Quota);
    let (next, _) = look(&mut core, &client, &ask(2, 0, 1, 1_500)).await;
    assert_eq!(
        (next.outcome, next.detail.as_deref()),
        (Outcome::Quota, Some("daily_limit"))
    );
    assert_eq!(stub.count(), 1, "no request until the limit resets");
    // At 23:59:59 UTC the hold is one second.
    let late_evening = UNIX_EPOCH + Duration::from_secs(1_793_750_400 - 1);
    assert_eq!(
        cerulion_rcpd::eye::host::until_utc_midnight(late_evening),
        Duration::from_secs(1)
    );
    let now = Instant::now();
    let mut fresh = core;
    assert!(
        !fresh.note(Outcome::Quota, None, now, late_evening),
        "the hold was already on"
    );
    let store = frames();
    let after = now + Duration::from_secs(2);
    assert!(matches!(
        fresh.admit(&ask(3, 0, 1, 1_500), after, after, after, &store),
        Admit::Send(_)
    ));
}

#[tokio::test(flavor = "multi_thread")]
async fn the_rate_and_in_flight_limits_refuse_before_any_request() {
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, _client) = core(dir.path(), &stub.base_url());
    let now = Instant::now();
    let store = frames();
    // Two requests held in flight: the third is throttled.
    let first = core.admit(&ask(1, 0, 1, 1_500), now, now, now, &store);
    let second = core.admit(&ask(2, 0, 1, 1_500), now, now, now, &store);
    assert!(matches!(first, Admit::Send(_)) && matches!(second, Admit::Send(_)));
    let Admit::Answer(third) = core.admit(&ask(3, 0, 1, 1_500), now, now, now, &store) else {
        panic!("the third request must be refused");
    };
    assert_eq!(
        (third.outcome, third.detail.as_deref()),
        (Outcome::Throttled, Some("in_flight"))
    );
    drop((first, second));
    // The burst is ten; two were spent above.
    let mut sent = 2;
    let mut throttled = None;
    for id in 10..30 {
        match core.admit(&ask(id, 0, 1, 1_500), now, now, now, &store) {
            Admit::Send(dispatch) => {
                sent += 1;
                drop(dispatch);
            }
            Admit::Answer(answer) => {
                throttled = Some(answer);
                break;
            }
            other => panic!("{other:?}"),
        }
    }
    assert_eq!(sent, 10);
    let throttled = throttled.expect("the eleventh look is throttled");
    assert_eq!(
        (throttled.outcome, throttled.detail.as_deref()),
        (Outcome::Throttled, Some("rate"))
    );
    // A second later five more go.
    let later = now + Duration::from_secs(1);
    let more = (40..60)
        .take_while(|id| {
            matches!(
                core.admit(&ask(*id, 0, 1, 1_500), later, later, later, &store),
                Admit::Send(_)
            )
        })
        .count();
    assert_eq!(more, 5);
    assert_eq!(stub.count(), 0, "nothing reached the network");
}

#[tokio::test(flavor = "multi_thread")]
async fn off_no_key_a_bad_ask_and_a_bad_frame_never_reach_the_network() {
    let stub = Stub::start(scripted).await;
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, client) = core(dir.path(), &stub.base_url());
    let now = Instant::now();
    let store = frames();
    let answer = |admit: Admit| match admit {
        Admit::Answer(a) => (a.outcome, a.detail.clone().unwrap_or_default()),
        other => panic!("{other:?}"),
    };
    // A frame larger than 640 x 480.
    assert_eq!(
        answer(core.admit(&ask(1, 0, 2, 1_500), now, now, now, &store)),
        (Outcome::Refused, String::from("image_too_large"))
    );
    // A frame not held: wait, then refuse.
    assert!(matches!(
        core.admit(&ask(2, 0, 7, 1_500), now, now, now, &store),
        Admit::Wait
    ));
    let later = now + Duration::from_millis(150);
    assert_eq!(
        answer(core.admit(&ask(2, 0, 7, 1_500), now, now, later, &store)),
        (Outcome::Refused, String::from("frame_not_held"))
    );
    // A free-form ask.
    let prompt = json!({ "ask_id": 3, "question_set": "press", "frame": { "topic": "/stub/wrist/image", "seq": 1 }, "prompt": "describe the room" });
    assert_eq!(
        answer(core.admit(&prompt.to_string(), now, now, now, &store)),
        (Outcome::Malformed, String::from("unknown_key"))
    );
    // An ask past its deadline.
    let stale = now - Duration::from_secs(2);
    assert_eq!(
        answer(core.admit(&ask(4, 0, 1, 1_500), stale, stale, now, &store)).0,
        Outcome::Timeout
    );
    // The key file readable by others.
    let key = dir.path().join("robot-jev.key");
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&key, std::fs::Permissions::from_mode(0o644)).expect("chmod");
        assert_eq!(
            answer(core.admit(&ask(5, 0, 1, 1_500), now, now, now, &store)),
            (Outcome::NoCredential, String::from("credential_mode"))
        );
        std::fs::set_permissions(&key, std::fs::Permissions::from_mode(0o600)).expect("chmod");
    }
    // No key at all.
    std::fs::remove_file(&key).expect("remove key");
    assert_eq!(
        answer(core.admit(&ask(6, 0, 1, 1_500), now, now, now, &store)),
        (Outcome::NoCredential, String::from("credential_missing"))
    );
    eye_common::key_file(dir.path());
    // The owner's switch off.
    write_switch(dir.path(), false).expect("off");
    assert_eq!(
        answer(core.admit(&ask(7, 0, 1, 1_500), now, now, now, &store)),
        (Outcome::Disabled, String::from("switched_off"))
    );
    write_switch(dir.path(), true).expect("on");
    assert_eq!(stub.count(), 0, "nothing reached the network");
    let (answer, _) = look(&mut core, &client, &ask(8, 0, 1, 1_500)).await;
    assert_eq!(answer.outcome, Outcome::Ok, "back on: {answer:?}");
}

#[tokio::test(flavor = "multi_thread")]
async fn the_deadline_is_the_budget_from_the_wire_time_inside_the_window_else_from_the_read() {
    let dir = tempfile::tempdir().expect("tempdir");
    let (mut core, _client) = core(dir.path(), "http://127.0.0.1:9/api/v1");
    let now = Instant::now();
    let now_ns = 9_000_000_000_000_u64;
    let budget = Duration::from_millis(1_500);
    let ms = |n: u64| Duration::from_millis(n);
    let behind = |n: u64| now_ns - n * 1_000_000;
    // Stamped 200 ms before the eye read it: due 1.5 s after the stamp.
    let sent = ask_sent(behind(200), now_ns, now);
    let Admit::Send(dispatch) = core.admit(&ask(1, 0, 1, 1_500), sent, now, now, &frames()) else {
        panic!("the request is sent");
    };
    assert_eq!(dispatch.deadline, now - ms(200) + budget);
    assert_eq!(dispatch.received, now);
    // Each dispatch holds an in-flight slot until it is dropped.
    drop(dispatch);
    // Stamped 2 s behind, as a run resumed after a pause or a virtual clock
    // stamps: the whole budget from the read.
    let sent = ask_sent(behind(2_000), now_ns, now);
    let Admit::Send(dispatch) = core.admit(&ask(2, 0, 1, 1_500), sent, now, now, &frames()) else {
        panic!("the request is sent");
    };
    assert_eq!(dispatch.deadline, now + budget);
    drop(dispatch);
    // Stamped ahead: from the read.
    let sent = ask_sent(now_ns + 5_000_000_000, now_ns, now);
    let Admit::Send(dispatch) = core.admit(&ask(3, 0, 1, 1_500), sent, now, now, &frames()) else {
        panic!("the request is sent");
    };
    assert_eq!(dispatch.deadline, now + budget);
    drop(dispatch);
    // A live ask read after its own short deadline is answered `timeout`,
    // with no request.
    let sent = ask_sent(behind(200), now_ns, now);
    let Admit::Answer(late) = core.admit(&ask(4, 0, 1, 150), sent, now, now, &frames()) else {
        panic!("answered at once");
    };
    assert_eq!(
        (late.outcome, late.detail.as_deref()),
        (Outcome::Timeout, Some("deadline_before_send"))
    );
}

#[test]
fn without_an_eyes_section_every_ask_is_disabled() {
    let dir = tempfile::tempdir().expect("tempdir");
    let settings = EyeSettings::resolve(&EyesConfig::default(), dir.path(), dir.path(), None)
        .expect("settings");
    let mut core = EyeCore::new(Arc::new(settings), Instant::now());
    let now = Instant::now();
    match core.admit(r#"{"ask_id":"a1"}"#, now, now, now, &FrameStore::default()) {
        Admit::Answer(answer) => assert_eq!(answer.outcome, Outcome::Disabled),
        other => panic!("{other:?}"),
    }
    assert!(matches!(
        core.admit("{}", now, now, now, &FrameStore::default()),
        Admit::Drop(_)
    ));
}
