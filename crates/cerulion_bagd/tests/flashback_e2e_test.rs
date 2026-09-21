// SPDX-License-Identifier: AGPL-3.0-only
//! The FLASHBACK end to end — a rolling window, a request that
//! crosses the real trigger channel, and one bag on disk.
//!
//! Isolated per-test SHM roots (the `common` harness), hand-built wire frames,
//! hand-written oracles. Parallel-safe: no `#[serial]`, no shared namespace.
//!
//! # What this proves that the unit arms cannot
//!
//! `window.rs`, `flashback_plane.rs` and `flashback_channel.rs` are each
//! oracle-tested in isolation, and all of them stay green if nothing WIRES them
//! together: the drive loop could stop harvesting, `service_flashback` could stop
//! being called, the capture could be written to a path nobody reports. Those are
//! the inert-shipping shapes, and only a run that publishes a real request over a
//! real iceoryx2 channel and then READS THE BAG can see them.
//!
//! # The post window is INJECTED short, and nothing else is
//!
//! The shipped post window is 15 s. Waiting it out would make this file a
//! minute-long suite for a property that has nothing to do with the number, so
//! `TriggerPolicy::post_window_ns` is injected at ~300 ms. Every other constant
//! — the window span, the byte ceiling, the retention caps — is the shipped one
//! unless the arm is ABOUT that constant, and every deadline is a generous
//! liveness ceiling in seconds rather than a wall stated in units of the thing
//! under test.
//!
//! # Every arm RENDEZVOUS ON THE HANDSHAKE before it publishes anything
//!
//! Five distinct arms of this file failed across four unrelated CI runs in
//! one day, one arm at a time, never reproducing locally. The cause
//! is the same in every case and it is not a margin: `run_bagd` attaches its
//! data-only TAPS and opens its flashback RESPONDER inside `Recorder::setup`, so
//! anything published before setup returns is delivered to NOBODY — a tap
//! requests no late-joiner history, and iceoryx2 keeps none for a control
//! subscriber that attaches later. A loaded runner widens that window; nothing
//! narrows it.
//!
//! The symptoms were exactly what that predicts. `a_recorder_with_no_flashback_
//! settings_…` failed on "it still records exactly as it did before flashback"
//! (every frame published into no queue ⇒ `messages == 0`), and
//! `a_new_capture_sweeps_…` failed on "the capture must finish before the sweep
//! can run" after burning its whole 20 s ceiling (the REQUEST reached no
//! responder, so no verdict could ever come — a longer deadline could not have
//! helped).
//!
//! So every arm now passes `BagdConfig::ready_file` and blocks on
//! [`await_bagd_ready`] before its first publish. That sentinel is bagd's own
//! taps-ready launch handshake, not a test hook, and it is a CONDITION load can
//! delay but not invert. Every cross-thread join goes through [`join_bagd`] for
//! the same reason a wedge must fail rather than hang.
//!
//! # The `sleep`s that REMAIN are producer PACING, and they are sound
//!
//! Every surviving `thread::sleep` in this file is inside a publish loop,
//! spacing frames so the rolling window has a temporal span to hold. None is a
//! rendezvous: the tap provably exists by then (the handshake above), the
//! topic's queue is deeper than any burst here, so no frame is lost by pacing
//! them slower. Load can only spread a window WIDER, which strengthens the
//! span- and truncation-shaped oracles rather than inverting them.

#![cfg(unix)]

mod common;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bagd::{run_bagd, BagdConfig, FlashbackSettings, TapSpec};
use cerulion_core::flashback::channel::{FlashbackOutcome, FlashbackRequester};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::{CaptureRequest, TriggerPolicy};
use cerulion_core::trace_ring::{TraceRingOwner, TraceRingRecord};

use common::{
    await_bagd_ready, await_condition, build_frame, join_bagd, make_manager, publisher, unique_out,
    unique_ready_file, unique_ring_tag, unique_topic, wait_for_file,
};

/// Shipped everywhere except the post window — see the module docs.
const POST_WINDOW_MS: u64 = 300;
/// Generous liveness ceiling for every capture-side condition wait. Load can
/// only delay these, never invert them.
///
/// Raised from 20 s with the rest of this file's backstops: a condition wait
/// costs a healthy run only as long as the condition takes (MEASURED: the whole
/// eight-arm suite finishes in ~7 s), so a looser ceiling is free on the runs
/// that pass and buys margin on a starved runner. It is spent only when
/// something is genuinely wrong, and it is bounded.
const CAPTURE_DEADLINE: Duration = Duration::from_secs(60);
/// How long the no-flashback control watches a channel that must stay silent.
///
/// The one WALL in the file, and it survives only in the shape the discipline
/// allows: a generous CEILING on an ABSENCE, spent after two conditions have
/// already proved the recorder is up and looping. Load can only push a verdict
/// LATER, so widening this can make the arm stricter and never inverts it.
const ABSENCE_WINDOW: Duration = Duration::from_secs(3);

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

fn settings(dir: &Path, window_only: bool) -> FlashbackSettings {
    FlashbackSettings {
        window_span: Duration::from_secs(30),
        window_max_bytes: 64 * 1024 * 1024,
        // This file declares no state ring, so nothing is
        // ever retained here — the shipped ceiling keeps the arms about the
        // FRAME window, which is what they are for. The anchor half is driven by
        // `flashback_anchor_e2e_test.rs`.
        anchor_max_bytes: 64 * 1024 * 1024,
        // A stated ceiling, so the anchor-first reserve
        // may not grow past it. This file retains no anchors at all, so the
        // basis changes nothing here beyond keeping the number meant.
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        // Likewise, this file declares no TRACE ring either,
        // so the retention stays empty and the shipped ceiling keeps these arms
        // about the frame window. The trace half is driven by
        // `flashback_trace_e2e_test.rs`.
        trace_max_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TRACE_MAX_MB * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "demo".into(),
        caps: RetentionCaps::default(),
        policy: TriggerPolicy {
            post_window_ns: POST_WINDOW_MS * 1_000_000,
            ..Default::default()
        },
        posture: TriggerPosture::default(), // These arms are not about the exclude lever.
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),

        window_only,
        // The shipped per-topic tap budget. These arms are
        // about the WINDOW, not the tap depth, so they take the default the
        // resolver would give them.
        tap_budget_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024,
    }
}

/// Every `.mcap` in `dir`, sorted.
fn captures(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .map(|r| {
            r.flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("mcap"))
                .collect()
        })
        .unwrap_or_default();
    out.sort();
    out
}

/// THE headline: a window-only recorder holds the last N seconds, a manual
/// request crosses the real channel, and a readable bag lands on disk carrying
/// frames the request never sent.
///
/// The frames are published BEFORE the request, which is the whole point of a
/// black box and the one thing no ordinary recording can do: a tap has no
/// back-fill, so every frame in this bag got there by having been RETAINED.
#[test]
fn a_manual_request_captures_the_window_that_was_already_held() {
    let mgr = make_manager(64);
    let topic = unique_topic("/fb/head");
    let dir = temp_dir("head");
    let mut cfg = BagdConfig::new(unique_out("fb_head"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.flashback = Some(settings(&dir, true));
    // The run CONTEXT the always-on spawn now hands over
    // (`graph_cmd::flashback_argv`). Set here as bytes rather than as argv
    // because this harness drives `run_bagd` in-process — what it proves is the
    // half the argv pins cannot: that the config really reaches the capture, and
    // that the manifest's `handoff` block describes it rather than being a
    // property of the pure renderer.
    cfg.attachments = vec![
        (
            cerulion_bagd::GRAPH_YAML_ATTACHMENT.to_string(),
            b"prefix: /fb\nnodes: []\n".to_vec(),
        ),
        (
            cerulion_bagd::ENV_JSON_ATTACHMENT.to_string(),
            b"{}".to_vec(),
        ),
        (
            cerulion_bagd::RECORDER_JSON_ATTACHMENT.to_string(),
            br#"{"arch":"aarch64"}"#.to_vec(),
        ),
    ];
    // A plane nothing published under: the recorder's sweep finds no ring and
    // reports it, which is precisely the state a capture must describe accurately.
    // What is under test here is that the HANDOFF crossed, not that anchors did.
    cfg.state_ring_discovery_tag = Some("cer_run_ffffffffffffffffffffffffffffffff".to_string());
    let ready = unique_ready_file("head");
    cfg.ready_file = Some(ready.clone());

    // The tap is OPEN-ONLY (it never creates a topic's services), so the
    // producer must exist before `Recorder::setup` runs.
    let mut pub_ = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_cfg = cfg.clone();
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, rec_cfg, rec_shutdown));
    // …and the tap must be ATTACHED before the first frame, or the pre-window
    // this whole arm is about lands in no queue at all.
    await_bagd_ready(&ready, "the headline manual-capture arm");

    // THE PRE-WINDOW: frames published before anybody asks for anything.
    const PRE: u32 = 12;
    for seq in 0..PRE {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    // …then the request, over the REAL channel.
    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("an operator saw the wobble"))
        .expect("request");

    let mut accepted = None;
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if let FlashbackOutcome::Accepted { path, .. } = &frame.outcome {
                    accepted = Some(path.clone());
                }
            }
            accepted.is_some()
        }),
        "the recorder must ACCEPT a manual request over the trigger channel"
    );

    assert!(
        await_condition(CAPTURE_DEADLINE, || !captures(&dir).is_empty()),
        "a capture bag must land in {}",
        dir.display()
    );
    // The bag has to be FINALIZED before it is read, which is what the
    // `Finished` verdict says — and asserting on the verdict rather than on the
    // file's mere existence is also what pins that the verdict is published at
    // all.
    let mut finished_bytes = None;
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if let FlashbackOutcome::Finished { bytes, .. } = &frame.outcome {
                    finished_bytes = Some(*bytes);
                }
            }
            finished_bytes.is_some()
        }),
        "the recorder must report the capture FINISHED"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary =
        join_bagd(handle, &shutdown, "the headline manual-capture arm").expect("recorder");

    // WINDOW-ONLY: no continuous bag, and the summary says so rather than
    // reporting an empty one.
    assert!(
        summary.bag_paths.is_empty(),
        "a window-only recorder writes NO continuous bag, got {:?}",
        summary.bag_paths
    );
    assert_eq!(summary.messages, 0);

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    assert_eq!(
        accepted.as_deref(),
        bags[0].to_str(),
        "the path the operator was told must be the path the bag landed at"
    );

    // READ IT BACK. The frames are the PRE-window ones — published before the
    // request existed, which no ordinary tap could recover.
    let reader = cerulion_bag::BagReader::open(&bags[0]).expect("open the capture");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "a capture must be FINALIZED — an un-finalized bag is refused by every reader: \
         {completeness:?}"
    );
    let recovered: Vec<u32> = msgs
        .iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.sequence)
        .collect();
    assert!(
        !recovered.is_empty(),
        "the capture must contain frames the request never sent — that is the whole feature"
    );
    assert!(
        recovered.iter().all(|s| *s < PRE),
        "every recovered frame is from the PRE-window: {recovered:?}"
    );
    // Contiguous and ascending: a window that dropped frames out of the middle
    // would still be non-empty, so the shape is asserted, not just the count.
    assert!(
        recovered.windows(2).all(|w| w[1] == w[0] + 1),
        "the window must be a contiguous run: {recovered:?}"
    );

    // …and the manifest names what it was about.
    let manifest = reader
        .attachment("__cerulion/flashback.json")
        .expect("attachment lookup")
        .expect("the capture manifest");
    let text = String::from_utf8(manifest.data).expect("utf-8");
    assert!(text.contains("an operator saw the wobble"), "{text}");
    assert!(text.contains("\"manual\""), "{text}");
    assert!(
        text.contains("\"causes_dropped_exact\":true"),
        "an ordinary capture's dropped count is EXACT: {text}"
    );

    // …and what it was HANDED. This is the no-inert-shipping half
    // of the manifest change — the pure renderer's arms build a `CaptureHandoff`
    // by hand and are structurally blind to a `close_capture` that assembled it
    // from the wrong place (or not at all).
    let doc: serde_json::Value = serde_json::from_str(&text)
        .unwrap_or_else(|e| panic!("the capture manifest must be valid JSON: {e}\n{text}"));
    let h = &doc["handoff"];
    assert_eq!(h["graph"], serde_json::json!("embedded"), "{text}");
    assert_eq!(h["env"], serde_json::json!("embedded"), "{text}");
    assert_eq!(h["recorder"], serde_json::json!("embedded"), "{text}");
    assert!(
        h["state_plane"]
            .as_str()
            .is_some_and(|s| s.starts_with("handed:")),
        "the capture-plane tag this recorder was given must be reported as HANDED: {text}"
    );
    assert!(
        h["trace"].as_str().is_some_and(|s| s.starts_with("none:")),
        "…and a recorder handed no trace ring must say so BY CAUSE: {text}"
    );

    // The attachments really are IN the bag, not merely described. A `handoff`
    // block that said `embedded` over a bag carrying nothing would be the exact
    // false claim it exists to prevent.
    for name in [
        cerulion_bagd::GRAPH_YAML_ATTACHMENT,
        cerulion_bagd::ENV_JSON_ATTACHMENT,
        cerulion_bagd::RECORDER_JSON_ATTACHMENT,
        // The RECORDER's health document. Before it, a
        // capture carried no drain-gap histogram, no per-topic loss counts and
        // no absorbance verdict — so the always-on window-only plane, the very
        // recorder whose loss boundary D11(a) narrowed, was the one whose
        // evidence never left the process.
        cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT,
    ] {
        assert!(
            reader
                .attachment(name)
                .expect("attachment lookup")
                .is_some(),
            "the capture must CARRY `{name}`, or `handoff` is describing a bag that does not \
             exist — a capture with no graph.yaml cannot be re-executed"
        );
    }
    // ...and it DECODES, with a per-topic verdict and the counting basis in it.
    // The bytes being present is not the claim; being readable as what the
    // surfaces read is.
    let health_bytes = reader
        .attachment(cerulion_bagd::CAPTURE_RECORDER_HEALTH_ATTACHMENT)
        .expect("attachment lookup")
        .expect("present")
        .data;
    let health: cerulion_bagd::RecordHealth =
        serde_json::from_slice(&health_bytes).expect("the capture's health document must decode");
    assert!(
        health.loss_counting_basis.is_some(),
        "a capture must say what its loss numbers can SEE, or a zero in them is \
         an absence nobody established"
    );
    // THE topic under test, by name — not `any()`. This recorder taps exactly
    // one topic, so an `any()` over the map is satisfied by a row for something
    // else if the tap set ever grows, and it cannot fail for the reason it
    // exists to catch (this topic carrying none).
    let row = health
        .topics
        .get(&topic)
        .unwrap_or_else(|| panic!("no health row for the tapped topic '{topic}': {health:?}"));
    assert!(
        row.absorbance.is_some(),
        "the tapped topic must carry a verdict: {health:?}"
    );
    // The name is the SCOPE marker: a capture must NOT carry the recording's own
    // attachment name, or `bag info`'s producer-label reader picks it up and
    // advertises attribution on a `__cerulion/frame_producers` channel the
    // capture writer leaves empty.
    assert!(
        reader
            .attachment(cerulion_bagd::RECORD_HEALTH_ATTACHMENT)
            .expect("attachment lookup")
            .is_none(),
        "a capture must not carry the RECORDING's health attachment name"
    );
    assert!(finished_bytes.is_some_and(|b| b > 0));

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// A capture whose window was bounded by its BYTE CEILING reports
/// what it CARRIES, not what its floor claims — over the real recorder, the real
/// channel, and the bag's own manifest.
///
/// # What this proves that the unit arms cannot
///
/// `close_capture` computes the achieved reach and hands it to the renderer, and
/// the renderer's own arms are handed that number by the test. So a wiring that
/// passed `floor_ns` there — the exact defect this arm excludes — leaves every pure
/// arm green while every capture on every robot goes back to claiming a window it
/// does not hold. Only a run through the real close site can see it, and under
/// that defect the two assertions below read `achieved_from_ns == floor_ns` and
/// `coverage_shortfall_ms == 0`.
///
/// # The fixture is the model's shape
///
/// The ceiling is injected TINY and the inflow CONTINUES through the post window,
/// which is the shape that matters: the lead-up is evicted by post-trigger
/// traffic while the capture is still recording. Everything else — the span, the
/// caps — is shipped.
///
/// LOAD-SAFE, and on exact grounds. A span ratio such as
/// `achieved * 2 < claimed` is NOT safe, even though it looks as if "load can only
/// lengthen the run (more claimed) while the ceiling holds the reach where it
/// is". Load lengthens BOTH — a stall between the last publishes inflates
/// `achieved` too — and that ratio was MEASURED failing under 2x CPU
/// oversubscription at macOS background QoS. So the magnitude claim is
/// stated in FRAMES, which the byte ceiling fixes by arithmetic rather than
/// by timing. What remains is exact on both sides: the reach is LATER than the
/// floor, the shortfall is nonzero, and it is the difference of the two spans it
/// sits beside. The precondition that the ceiling really bit is asserted
/// separately, so a starved runner fails as a starved runner rather than
/// inverting the verdict.
#[test]
fn a_capture_bounded_by_its_byte_ceiling_reports_what_it_carries() {
    let mgr = make_manager(64);
    let topic = unique_topic("/fb/short");
    let dir = temp_dir("short");
    let mut cfg = BagdConfig::new(unique_out("fb_short"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    let mut s = settings(&dir, true);
    // The ONE injected constant: a ceiling a few frames wide, so the window is
    // provably bounded by bytes rather than by its (shipped, 30 s) span.
    const CAP_BYTES: u64 = 8 * 1024;
    const PAYLOAD: usize = 2 * 1024;
    s.window_max_bytes = CAP_BYTES;
    cfg.flashback = Some(s);
    let ready = unique_ready_file("short");
    cfg.ready_file = Some(ready.clone());

    // The slot must hold a frame (payload + the 32-byte wire header) — these
    // payloads are deliberately fat, so the ceiling is reached in a few frames
    // rather than in thousands.
    let mut pub_ = publisher(&mgr, &topic, PAYLOAD as u32 + 64);
    let payload = vec![0xCDu8; PAYLOAD];

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the byte-ceiling arm");

    // A pre-window far larger than the ceiling, so most of it is already gone.
    let mut seq = 0u32;
    for _ in 0..30 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), &payload))
            .expect("publish");
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("the window is bounded by bytes"))
        .expect("request");

    // …and the inflow CONTINUES through the whole post window, which is what
    // evicts the lead-up out from under the capture.
    let mut finished = false;
    for _ in 0..60 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), &payload))
            .expect("publish");
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
        if requester
            .drain_outcomes(request_id)
            .iter()
            .any(|f| matches!(f.outcome, FlashbackOutcome::Finished { .. }))
        {
            finished = true;
            break;
        }
    }
    if !finished {
        assert!(
            await_condition(CAPTURE_DEADLINE, || {
                requester
                    .drain_outcomes(request_id)
                    .iter()
                    .any(|f| matches!(f.outcome, FlashbackOutcome::Finished { .. }))
            }),
            "the recorder must report the capture FINISHED"
        );
    }

    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the byte-ceiling arm").expect("recorder");

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    let reader = cerulion_bag::BagReader::open(&bags[0]).expect("open the capture");
    let manifest = reader
        .attachment("__cerulion/flashback.json")
        .expect("attachment lookup")
        .expect("the capture manifest");
    let text = String::from_utf8(manifest.data).expect("utf-8");
    let doc: serde_json::Value = serde_json::from_str(&text)
        .unwrap_or_else(|e| panic!("the capture manifest must be valid JSON: {e}\n{text}"));

    // PRECONDITION: the ceiling really was the binding limit. Without this the
    // arm below could pass on a starved runner that published too little to
    // overflow anything, which is a different (and uninteresting) reason.
    assert_eq!(
        doc["window_cap_bytes"],
        serde_json::json!(CAP_BYTES),
        "the manifest must report the ceiling in force: {text}"
    );
    let carried = doc["frames"].as_u64().expect("a frame count");
    assert!(
        carried < u64::from(seq),
        "the fixture must really have overflowed its ceiling — it published {seq} frames and the \
         capture carried {carried}: {text}"
    );

    let floor = doc["floor_ns"].as_u64().expect("a floor");
    let achieved_from = doc["achieved_from_ns"].as_u64();
    let claimed = doc["span_ms"].as_u64().expect("a claimed span");
    let achieved = doc["achieved_span_ms"].as_u64().expect("an achieved span");
    let shortfall = doc["coverage_shortfall_ms"].as_u64().expect("a shortfall");

    // THE assertion, and the one that a wrong `floor_ns` wiring fails: the bag does not reach
    // the floor it claims.
    match achieved_from {
        Some(from) => assert!(
            from > floor,
            "the byte ceiling evicted past the floor, so the reach must be LATER than it \
             ({from} vs {floor}): {text}"
        ),
        // A capture that overflowed so hard it carries nothing at all is the
        // other valid answer, and it is `null` rather than the floor.
        None => assert_eq!(carried, 0, "a null reach means no frames: {text}"),
    }
    assert!(
        shortfall > 0,
        "the bag must report a shortfall — it does not reach the floor it claims (achieved \
         {achieved} ms of a claimed {claimed} ms): {text}"
    );
    // …and the shortfall is LARGE, stated in FRAMES because that is the term the
    // byte ceiling actually fixes.
    //
    // This replaces an `achieved * 2 < claimed` span ratio whose doc called it
    // load-safe. It is not, and the algebra says why: with `floor_ns` saturated
    // to 0 on a run shorter than the window span, `shortfall == achieved_from`,
    // so that ratio is exactly "the oldest RETAINED frame arrived after the
    // midpoint of the plane's life" — which one long stall among the last few
    // publishes inverts, by inflating `achieved` without moving `claimed`. It
    // FAILED that way under 2x CPU oversubscription at background QoS, and by
    // elimination it can only have been that conjunct (a `shortfall > 0` failure
    // implies `achieved_from == floor_ns`, which the assertion above catches
    // first).
    //
    // The frame form cannot be faked by timing: `carried` is bounded by
    // CAP_BYTES / frame size (~3 here) no matter how the publishes are spaced,
    // while `seq` is a loop count with a floor of 30. Nothing about the SPAN
    // claim is lost that the two assertions around it did not already carry —
    // a wrong `floor_ns` wiring reads `achieved_from_ns == floor_ns` and
    // `coverage_shortfall_ms == 0`, and both of those still fail it.
    assert!(
        carried * 4 <= u64::from(seq),
        "the byte ceiling must have taken the great majority of the run: it published {seq} \
         frames and the capture carried {carried}: {text}"
    );
    assert_eq!(
        shortfall,
        claimed - achieved,
        "the shortfall is the difference of the two spans it sits beside: {text}"
    );

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// A capture reports the frames this capture lost, never
/// the plane's lifetime total.
///
/// # Why this needs TWO captures
///
/// On a single-capture run the delta and the lifetime total are equal by
/// construction, so every other arm in the suite is structurally blind to the
/// swap — `truncated_frames: plane.truncated_frames()` at the publish site passes
/// all of them. What it costs on a real robot is the CAUSAL claim both render
/// sites now gate on: once ANY capture loses a frame to the byte ceiling, every
/// LATER capture would report itself truncated and be told, with a remedy naming
/// a cap that did not bind for it, to raise `CERULION_FLASHBACK_WINDOW_MAX_MB`.
///
/// So: capture 1 runs with the inflow overflowing a tiny ceiling (its own arm
/// pins that shape); the inflow then STOPS, so the window sits inside its cap
/// with nothing to evict; capture 2 takes the same window and must report ZERO.
///
/// The DISCRIMINATOR PRECONDITION is capture 1's own nonzero count — that is what
/// makes the plane's lifetime total nonzero, and without it capture 2's zero is
/// satisfied by a recorder that never truncated anything at all.
///
/// Manual requests are exempt from the latch and the refractory floor (decision
/// 90's design: a deliberate act must never be silently swallowed), which is what
/// lets one run take two captures seconds apart.
#[test]
fn a_later_capture_reports_its_own_truncation_not_the_planes_lifetime_total() {
    let mgr = make_manager(64);
    let topic = unique_topic("/fb/twocap");
    let dir = temp_dir("twocap");
    let mut cfg = BagdConfig::new(unique_out("fb_twocap"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    let mut s = settings(&dir, true);
    const CAP_BYTES: u64 = 8 * 1024;
    const PAYLOAD: usize = 2 * 1024;
    s.window_max_bytes = CAP_BYTES;
    cfg.flashback = Some(s);
    let ready = unique_ready_file("twocap");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, PAYLOAD as u32 + 64);
    let payload = vec![0xCDu8; PAYLOAD];

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the two-capture truncation arm");

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let mut seq = 0u32;

    // CAPTURE 1 — the inflow overflows the ceiling throughout, so the byte pass
    // takes frames this capture wanted.
    for _ in 0..20 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), &payload))
            .expect("publish");
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    }
    let first_id = requester
        .request(&CaptureRequest::manual("the ceiling is biting"))
        .expect("request");
    let mut first_span = None;
    for _ in 0..60 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), &payload))
            .expect("publish");
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
        for f in requester.drain_outcomes(first_id) {
            if let FlashbackOutcome::Finished { span, .. } = f.outcome {
                first_span = Some(span);
            }
        }
        if first_span.is_some() {
            break;
        }
    }
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            for f in requester.drain_outcomes(first_id) {
                if let FlashbackOutcome::Finished { span, .. } = f.outcome {
                    first_span = Some(span);
                }
            }
            first_span.is_some()
        }),
        "the recorder must report the FIRST capture finished"
    );

    // CAPTURE 2 — nothing further is published, so the window sits inside its
    // ceiling and the byte pass has nothing to take.
    let second_id = requester
        .request(&CaptureRequest::manual("the ceiling is quiet"))
        .expect("request");
    let mut second_span = None;
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            for f in requester.drain_outcomes(second_id) {
                if let FlashbackOutcome::Finished { span, .. } = f.outcome {
                    second_span = Some(span);
                }
            }
            second_span.is_some()
        }),
        "the recorder must report the SECOND capture finished"
    );

    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the two-capture truncation arm").expect("recorder");

    let bags = captures(&dir);
    assert_eq!(bags.len(), 2, "one bag per capture, got {bags:?}");
    let manifest_of = |bag: &std::path::Path| -> serde_json::Value {
        let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
        let att = reader
            .attachment("__cerulion/flashback.json")
            .expect("attachment lookup")
            .expect("the capture manifest");
        let text = String::from_utf8(att.data).expect("utf-8");
        serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("the capture manifest must be valid JSON: {e}\n{text}"))
    };
    // `captures()` sorts by name and the name carries a wall stamp, so the older
    // capture sorts first. Asserted through the SEQUENCE rather than trusted.
    let (first, second) = (manifest_of(&bags[0]), manifest_of(&bags[1]));
    assert_eq!(first["seq"], serde_json::json!(0), "{first}");
    assert_eq!(second["seq"], serde_json::json!(1), "{second}");

    // THE PRECONDITION: capture 1 really did lose frames, so the plane's lifetime
    // total is nonzero from here on. Without this, capture 2's zero says nothing.
    let first_truncated = first["truncated_frames"].as_u64().expect("a count");
    assert!(
        first_truncated > 0,
        "the fixture must overflow the ceiling during the FIRST capture, or the second \
         capture's zero is not discriminating: {first}"
    );

    // THE ASSERTION, in both places the number is served.
    assert_eq!(
        second["truncated_frames"],
        serde_json::json!(0),
        "the SECOND capture lost nothing — reporting the plane's lifetime total here would \
         label an unaffected bag truncated (first capture lost {first_truncated}): {second}"
    );
    let second_span = second_span
        .expect("the second capture's span")
        .expect("this recorder always claims a span — an absent trailer means an older recorder");
    assert_eq!(
        second_span.truncated_frames, 0,
        "…and the verdict an operator reads during the incident must agree with the bag \
         (first capture lost {first_truncated})"
    );
    assert!(
        !second_span.evicted_during_capture(),
        "…so the second capture must not blame the byte ceiling for its shortfall"
    );

    // The FIRST capture's trailer carries its own count, so the two are not both
    // zero for a reason unrelated to the split.
    let first_span = first_span
        .expect("the first capture's span")
        .expect("this recorder always claims a span");
    assert_eq!(
        first_span.truncated_frames, first_truncated,
        "the first capture's verdict and its bag must report the SAME loss"
    );
    assert!(first_span.evicted_during_capture());

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// A manual request at the rate cap is refused, loudly, over
/// the channel, with both numbers — and nothing is captured.
///
/// The cap is injected at ZERO, which is the cheapest way to reach the arm and
/// is exactly what the pure gate says it means ("capture nothing"). The arm this
/// pins is the CARRIAGE: that the verdict crosses the wire to the asker rather
/// than only reaching a daemon log the operator cannot see.
#[test]
fn a_rate_capped_manual_request_is_refused_over_the_channel_with_its_numbers() {
    let mgr = make_manager(16);
    let topic = unique_topic("/fb/capped");
    let dir = temp_dir("capped");
    let mut cfg = BagdConfig::new(unique_out("fb_capped"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    let mut s = settings(&dir, true);
    s.policy.max_per_hour = 0;
    cfg.flashback = Some(s);
    let ready = unique_ready_file("capped");
    cfg.ready_file = Some(ready.clone());

    // The tap is OPEN-ONLY (it never creates a topic's services), so the
    // producer must exist before `Recorder::setup` runs.
    // Held for the recorder's lifetime: the tap is OPEN-ONLY, and this arm's
    // whole point is that the request is refused rather than captured, so the
    // topic only has to EXIST.
    let _pub = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the rate-capped refusal arm");

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request = CaptureRequest::manual("ask at the cap");
    let request_id = requester.request(&request).expect("request");

    // RE-PUBLISHED while nothing has answered — the same loop `cerulion
    // flashback` runs, and for the same reason: iceoryx2 pub/sub keeps no
    // history for a subscriber that attaches later, so a request published
    // before this recorder's control subscriber existed is not delivered.
    //
    // The ready-file rendezvous above now closes that window for every arm in
    // this file (see the module docs), so this loop should re-publish nothing.
    // It is KEPT because it costs nothing and this arm is the one that can
    // afford it: every request here is REFUSED, so a duplicate cannot mint a
    // second capture. The arms asserting an exact bag count must not carry it,
    // and the handshake is what makes that safe rather than lucky.
    let mut refusal = None;
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if let FlashbackOutcome::Suppressed(reason) = &frame.outcome {
                    refusal = Some(reason.clone());
                }
            }
            if refusal.is_none() {
                let _ = requester.request_with_id(&request, request_id);
            }
            refusal.is_some()
        }),
        "the refusal must reach the ASKER, not only the recorder's log (by design)"
    );
    match refusal.expect("a refusal") {
        cerulion_core::flashback::trigger::SuppressReason::RateCapped {
            captures_in_window,
            cap,
            reserved_for_manual,
        } => {
            assert_eq!(cap, 0, "the cap in force is reported");
            assert_eq!(captures_in_window, 0);
            // A cap of zero reserves nothing: holding a
            // slot back out of a budget that admits no captures would invent one.
            assert_eq!(reserved_for_manual, 0);
        }
        other => panic!("expected a rate-cap refusal, got {other:?}"),
    }

    shutdown.store(true, Ordering::Relaxed);
    let _ = join_bagd(handle, &shutdown, "the rate-capped refusal arm");
    assert!(
        captures(&dir).is_empty(),
        "a refused request must capture NOTHING"
    );
    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// The pin: a `--record`-shaped recorder (window_only = false) holds the
/// window TOO — `cerulion flashback` means one thing whether or not somebody
/// asked for a recording.
///
/// The discriminator is that BOTH bags exist: the continuous one the flag asked
/// for, and the capture the window produced.
///
/// **It is ALSO the arm that proves a capture never claims a scheduler trace
/// it does not carry.** That claim is
/// unchanged; what it EVALUATES to on a `--record` run is not.
///
/// This arm was first written against a capture path that had no trace channel in it at
/// all, so the two bags one run produces disagreed about trace BY DESIGN and the
/// pin was "the capture says NONE". Since then
/// `capture::write_capture` has its trace channel, so the capture
/// CARRIES the run's trace and the manifest says how much. Leaving the
/// old oracle in place would have pinned the absence of that
/// feature.
///
/// The rule it was really written for survives intact, and is what this arm now
/// asserts: the manifest's number is MEASURED from what the bag HOLDS, never
/// read off `cfg.rings.len()`. That was the original defect — `attached: 1 trace
/// ring(s)` rendered over a capture holding zero records, sending a reader
/// straight into `bag play --resim`'s `BagNoSchedulerTrace` refusal — and the
/// cross-check below (manifest count == scheduler_trace messages in this very
/// bag) is a strictly stronger form of it than "says none" ever was.
///
/// The CONTINUOUS bag's trace records remain the ANTI-TAUTOLOGY half: without
/// them the ring was never live and the arm would pass for a reason unrelated to
/// what it is named for.
#[test]
fn a_recording_run_holds_the_window_as_well_as_writing_its_bag() {
    let mgr = make_manager(64);
    let topic = unique_topic("/fb/uniform");
    let dir = temp_dir("uniform");
    let out = unique_out("fb_uniform");
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    // window_only = FALSE — the `--record` shape.
    cfg.flashback = Some(settings(&dir, false));

    // …and the `--record` shape's OTHER half: this recorder is handed the run's
    // trace ring. `graph run --record` passes it as `--ring <name>`.
    let ring_tag = unique_ring_tag("fb_uniform");
    let mut ring_owner = TraceRingOwner::create(&ring_tag, 16, 0, &["n0"]).expect("ring create");
    let mut ring_producer = ring_owner.producer().expect("producer");
    cfg.rings = vec![ring_owner.name().to_string()];
    let ready = unique_ready_file("uniform");
    cfg.ready_file = Some(ready.clone());

    // The tap is OPEN-ONLY (it never creates a topic's services), so the
    // producer must exist before `Recorder::setup` runs.
    let mut pub_ = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_cfg = cfg.clone();
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, rec_cfg, rec_shutdown));
    // The CONTINUOUS bag's `messages > 0` and the capture's own frames both
    // depend on the tap existing before the first publish.
    await_bagd_ready(&ready, "the --record-shaped window arm");

    // Real trace records, so the ring is genuinely live and drained.
    const TRACE_RECORDS: u64 = 4;
    for step in 0..TRACE_RECORDS {
        ring_producer.push(&TraceRingRecord {
            step,
            fire_time_ns: 5_000 + step,
            duration_ns: 10,
            node_idx: 0,
            global_level: 0,
            record_type: 1,
            reserved: 0,
        });
    }
    for seq in 0..10u32 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), b"u"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("during a recording"))
        .expect("request");
    assert!(
        await_condition(CAPTURE_DEADLINE, || requester
            .drain_outcomes(request_id)
            .iter()
            .any(|f| matches!(f.outcome, FlashbackOutcome::Finished { .. }))),
        "a --record recorder must capture too (by design)"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the --record-shaped window arm").expect("recorder");

    assert!(
        !summary.bag_paths.is_empty(),
        "the CONTINUOUS bag is still written — the flag was not taken away"
    );
    assert!(summary.messages > 0, "and it still recorded frames");
    assert_eq!(
        summary.ring_records, TRACE_RECORDS,
        "ANTI-TAUTOLOGY: the ring must really have been live and drained into the CONTINUOUS \
         bag, or 'the capture carries no trace' is satisfied by a ring nothing ever wrote"
    );
    let bags = captures(&dir);
    assert_eq!(
        bags.len(),
        1,
        "…AND the window produced a capture beside it"
    );

    // THE FINDING, in its Stage-A form. The capture is a separate bag written by
    // a separate path — and since Stage A that path carries the trace, so the
    // question is no longer "is it absent?" but "does the manifest state what is
    // really there?".
    let reader = cerulion_bag::BagReader::open(&bags[0]).expect("open the capture");
    let (msgs, _completeness) = reader.recover_messages().expect("recover_messages");
    let carried_in_bag = msgs
        .iter()
        // EXACT, not `contains`: a substring match would also count a channel
        // merely NAMED like the canonical one, so the cross-check below could be
        // satisfied by the wrong channel entirely. The sibling file's
        // `trace_payloads` already compares against this constant.
        .filter(|m| m.topic == cerulion_bag::SCHEDULER_TRACE_TOPIC)
        .count();
    assert!(
        carried_in_bag > 0,
        "a `--record` run's capture CARRIES its trace since Stage A — that is the \
         feature: {:?}",
        msgs.iter().map(|m| &m.topic).collect::<Vec<_>>()
    );
    let manifest = reader
        .attachment("__cerulion/flashback.json")
        .expect("attachment lookup")
        .expect("the capture manifest");
    let text = String::from_utf8(manifest.data).expect("utf-8");
    let doc: serde_json::Value =
        serde_json::from_str(&text).unwrap_or_else(|e| panic!("valid JSON: {e}\n{text}"));
    let h = &doc["handoff"];
    // THE oracle, and it is what the old "says none" was reaching for: the
    // manifest's count is the number of records THIS BAG HOLDS. Read off
    // `cfg.rings.len()` it would say 1; read off the retention it could say
    // anything; only the carried count can agree with the channel beside it.
    assert_eq!(
        h["trace"].as_str().expect("a trace verdict"),
        format!("carried: {carried_in_bag} scheduler-trace record(s)"),
        "the manifest must report the count the bag actually carries: {text}"
    );
    assert!(
        !h["trace"]
            .as_str()
            .expect("a trace verdict")
            .starts_with("none:"),
        "…and a capture that carries a trace never renders an ABSENCE cause: {text}"
    );
    assert_eq!(
        h["trace_rings_configured"],
        serde_json::json!(1),
        "…and the configured fact survives under a name that says what it is: {text}"
    );

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// The dashcam contract, driven by the RECORDER rather than by a unit test: a
/// capture directory already at its count cap is swept when a new capture lands,
/// and a PINNED capture survives.
///
/// The sweep runs after a capture is collected, so this needs a real capture to
/// trigger it — which is what makes it an e2e rather than a second copy of
/// `flashback_plane`'s own arm.
#[test]
fn a_new_capture_sweeps_the_directory_and_a_pinned_one_survives() {
    let mgr = make_manager(16);
    let topic = unique_topic("/fb/sweep");
    let dir = temp_dir("sweep");

    // Two stale captures already there; the first is PINNED.
    for name in ["old_a.mcap", "old_b.mcap"] {
        std::fs::write(dir.join(name), vec![0u8; 64]).expect("seed");
    }
    std::fs::write(dir.join("old_a.mcap.pin"), b"").expect("pin");

    let mut cfg = BagdConfig::new(unique_out("fb_sweep"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    let mut s = settings(&dir, true);
    // Room for TWO captures. With three present, the oldest UNPINNED goes.
    s.caps = RetentionCaps {
        max_bytes: u64::MAX,
        max_captures: 2,
    };
    cfg.flashback = Some(s);
    let ready = unique_ready_file("sweep");
    cfg.ready_file = Some(ready.clone());

    // The tap is OPEN-ONLY (it never creates a topic's services), so the
    // producer must exist before `Recorder::setup` runs.
    let mut pub_ = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    // THE fix for this arm's CI failure: it publishes for only
    // ~40 ms before asking, so on a loaded runner the request reached a
    // responder that did not exist yet, was delivered to nobody, and the wait
    // below burned its whole 20 s ceiling on a verdict that could never come.
    await_bagd_ready(&ready, "the retention-sweep arm");

    for seq in 0..4u32 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), b"s"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }
    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("sweep me"))
        .expect("request");
    assert!(
        await_condition(CAPTURE_DEADLINE, || requester
            .drain_outcomes(request_id)
            .iter()
            .any(|f| matches!(f.outcome, FlashbackOutcome::Finished { .. }))),
        "the capture must finish before the sweep can run"
    );
    // The sweep runs on the pass that COLLECTS the capture, which is the same
    // pass that publishes the verdict — so it may be one pass behind it.
    assert!(
        await_condition(CAPTURE_DEADLINE, || !dir.join("old_b.mcap").exists()),
        "the oldest UNPINNED capture must be evicted"
    );

    // The ABSENT half of the headline's `handoff` assertions, and
    // the ANTI-TAUTOLOGY for them: this recorder was handed NOTHING (no
    // attachments, no capture-plane tag, no ring), so every slot must name its
    // own cause. Without it, a renderer hardcoding `embedded` passes the
    // headline character for character over a bag that carries none of it.
    let fresh = captures(&dir)
        .into_iter()
        .find(|p| !p.ends_with("old_a.mcap"))
        .expect("the new capture");
    let reader = cerulion_bag::BagReader::open(&fresh).expect("open the capture");
    let manifest = reader
        .attachment("__cerulion/flashback.json")
        .expect("attachment lookup")
        .expect("the capture manifest");
    let text = String::from_utf8(manifest.data).expect("utf-8");
    let doc: serde_json::Value =
        serde_json::from_str(&text).unwrap_or_else(|e| panic!("valid JSON: {e}\n{text}"));
    let h = &doc["handoff"];
    for (slot, what) in [
        ("graph", "graph.yaml"),
        ("env", "env.json"),
        ("recorder", "recorder.json"),
        ("state_plane", "a capture-plane tag"),
    ] {
        assert!(
            h[slot]
                .as_str()
                .is_some_and(|s| s.starts_with("absent:") && s.len() > "absent:".len()),
            "a capture handed no {what} must say so BY CAUSE, not merely be missing it: {text}"
        );
    }
    assert!(
        h["trace"].as_str().is_some_and(|s| s.starts_with("none:")),
        "{text}"
    );

    shutdown.store(true, Ordering::Relaxed);
    let _ = join_bagd(handle, &shutdown, "the retention-sweep arm");

    assert!(
        dir.join("old_a.mcap").exists(),
        "a PINNED capture is never a candidate — that is the escape's whole point"
    );
    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// A SHUTDOWN must resolve an accepted capture, not
/// abandon it.
///
/// The sequence: a window-only recorder is shut down between the
/// `Accepted` verdict and the post window's end. Abandoning the capture there leaves
/// `terminal_outcomes=0, capture_bags=0` — no bag, and a requester waiting for a
/// verdict that can never come. This drives that sequence and asserts the
/// opposite.
///
/// The post window is deliberately LONG here (the shipped 15 s), because the
/// whole point is to stop while the capture is still recording; a short one
/// would let the ordinary due-path close it and the test would pass without the
/// shutdown path resolving anything.
#[test]
fn a_shutdown_mid_capture_still_writes_the_bag_and_answers_the_requester() {
    let mgr = make_manager(64);
    let topic = unique_topic("/fb/teardown");
    let dir = temp_dir("teardown");
    let mut cfg = BagdConfig::new(unique_out("fb_teardown"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    let mut s = settings(&dir, true);
    // The SHIPPED post window: this capture must still be recording when the
    // recorder is told to stop.
    s.policy.post_window_ns = 15_000 * 1_000_000;
    cfg.flashback = Some(s);
    let ready = unique_ready_file("teardown");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    // "…carrying the window it had" is an assertion about FRAMES, so the tap has
    // to exist before the eight below are published (this arm's CI
    // failure).
    await_bagd_ready(&ready, "the shutdown-mid-capture arm");

    for seq in 0..8u32 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), b"td"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request = CaptureRequest::manual("stop me mid-capture");
    let request_id = requester.request(&request).expect("request");
    let mut accepted = false;
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if matches!(frame.outcome, FlashbackOutcome::Accepted { .. }) {
                    accepted = true;
                }
            }
            if !accepted {
                let _ = requester.request_with_id(&request, request_id);
            }
            accepted
        }),
        "the capture must be ACCEPTED before the shutdown, or this test proves nothing"
    );

    // …and now stop, WHILE it is still recording.
    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the shutdown-mid-capture arm").expect("recorder");
    assert!(summary.bag_paths.is_empty(), "still window-only");

    // The capture was WRITTEN rather than discarded: the frames were already in
    // memory and somebody asked for them, so a shorter bag beats no bag.
    let bags = captures(&dir);
    assert_eq!(
        bags.len(),
        1,
        "a shutdown must WRITE the accepted capture, not abandon it — got {bags:?}"
    );
    let reader = cerulion_bag::BagReader::open(&bags[0]).expect("open the capture");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "and it must be FINALIZED, not left partial: {completeness:?}"
    );
    assert!(
        msgs.iter().any(|m| m.topic == topic),
        "…carrying the window it had"
    );

    // …and the requester was ANSWERED. Drained after the join, because the
    // verdict is published on the recorder's own shutdown path.
    let terminal = requester
        .drain_outcomes(request_id)
        .into_iter()
        .filter(|f| {
            matches!(
                f.outcome,
                FlashbackOutcome::Finished { .. } | FlashbackOutcome::Failed { .. }
            )
        })
        .count();
    assert!(
        terminal >= 1,
        "the requester must get a TERMINAL verdict — waiting forever for one that can never \
         come is the failure this arm excludes"
    );

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}

/// A recorder with NO flashback settings is byte-identical to a pre-flashback
/// one: it writes its bag, holds no window, and answers no request.
///
/// The anti-tautology control. Without it, every arm above is satisfied by a
/// recorder that captures unconditionally.
#[test]
fn a_recorder_with_no_flashback_settings_answers_nothing_and_writes_its_bag() {
    let mgr = make_manager(16);
    let topic = unique_topic("/fb/none");
    let out = unique_out("fb_none");
    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    assert!(cfg.flashback.is_none(), "the LIBRARY default is off");
    let ready = unique_ready_file("none");
    cfg.ready_file = Some(ready.clone());

    // The tap is OPEN-ONLY (it never creates a topic's services), so the
    // producer must exist before `Recorder::setup` runs.
    let mut pub_ = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    // THE fix for this arm's CI failure: it failed on "it
    // still records exactly as it did before flashback", which is
    // `summary.messages > 0` — every frame below had been published into a topic
    // whose tap did not exist yet, and a data-only tap has no back-fill.
    await_bagd_ready(&ready, "the no-flashback control arm");

    for seq in 0..5u32 {
        pub_.publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), b"n"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("nobody is listening"))
        .expect("request");
    // The claim here is an ABSENCE, so what it needs is not a longer wall but
    // EVIDENCE that the recorder really was running and really did decline to
    // answer. Two things supply it, and neither is a wall in units of anything
    // under test: the ready-file above proves `Recorder::setup` completed, and
    // the bag FILE appearing proves the drive loop ran passes and reached
    // `ensure_writer` — the same loop that would have serviced a trigger.
    assert!(
        wait_for_file(&out, CAPTURE_DEADLINE),
        "the recorder must have created its bag ({}) — until it has, 'nothing answered' says \
         only that nothing has run yet",
        out.display()
    );
    // …and only then a generous CEILING on the silence. `await_condition`
    // returning false IS the absence: load can make a verdict arrive later, so a
    // wider window can only make this arm stricter, never invert it.
    let answered = await_condition(ABSENCE_WINDOW, || {
        !requester.drain_outcomes(request_id).is_empty()
    });
    assert!(
        !answered,
        "a recorder with no flashback settings must not answer the trigger channel"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = join_bagd(handle, &shutdown, "the no-flashback control arm").expect("recorder");
    assert!(
        !summary.bag_paths.is_empty() && summary.messages > 0,
        "…and it still records exactly as it did before flashback"
    );
    std::fs::remove_file(&ready).ok();
}

/// An excluded topic is not in the window, and its sibling still is.
///
/// The lever the design names for buying window seconds back on a big-state
/// robot. Its parser is oracle-tested in `cerulion_core`, and every one of those
/// arms stays green if nothing WIRES it — the harvest could ignore the list, or
/// apply it to the wrong tap, or (the shape that would look most like working)
/// exclude everything. Only a run that publishes on BOTH topics and then READS
/// THE BAG can tell those apart.
///
/// The sibling is the anti-tautology half and it is in the same body: without
/// it, "the excluded topic is absent" is satisfied by a capture that carries
/// nothing at all.
#[test]
fn an_excluded_topic_is_absent_from_the_capture_while_its_sibling_is_carried() {
    let mgr = make_manager(64);
    let excluded = unique_topic("/fb/excl/cam");
    let kept = unique_topic("/fb/excl/imu");
    let dir = temp_dir("exclude");
    let mut cfg = BagdConfig::new(
        unique_out("fb_exclude"),
        vec![TapSpec::attach(&excluded), TapSpec::attach(&kept)],
    );
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    let mut s = settings(&dir, true);
    // A PREFIX pattern, so the arm drives the half of the grammar an exact name
    // would not: the excluded topic's unique suffix is not known to the pattern.
    let (parsed, complaint) =
        cerulion_core::flashback::parse_exclude_topics(Some(&format!("{excluded}*")));
    assert_eq!(complaint, None, "the fixture's own pattern must be valid");
    s.exclude_topics = parsed;
    cfg.flashback = Some(s);
    let ready = unique_ready_file("exclude");
    cfg.ready_file = Some(ready.clone());

    let mut pub_excluded = publisher(&mgr, &excluded, 256);
    let mut pub_kept = publisher(&mgr, &kept, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_cfg = cfg.clone();
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, rec_cfg, rec_shutdown));
    await_bagd_ready(&ready, "the exclude arm");

    // Both topics publish the SAME number of frames, so a capture that carried
    // the excluded one would be unmistakable.
    const PRE: u32 = 8;
    for seq in 0..PRE {
        pub_excluded
            .publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), b"cam"))
            .expect("publish excluded");
        pub_kept
            .publish_raw(&build_frame(0xABCD, seq, 1_000 + u64::from(seq), b"imu"))
            .expect("publish kept");
        std::thread::sleep(Duration::from_millis(10));
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("the exclude arm"))
        .expect("request");
    let mut finished = false;
    assert!(
        await_condition(CAPTURE_DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                    finished = true;
                }
            }
            finished
        }),
        "the recorder must report the capture FINISHED"
    );

    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the exclude arm").expect("recorder");

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    let reader = cerulion_bag::BagReader::open(&bags[0]).expect("open the capture");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(completeness.is_finalized(), "{completeness:?}");

    let kept_frames = msgs.iter().filter(|m| m.topic == kept).count();
    let excluded_frames = msgs.iter().filter(|m| m.topic == excluded).count();
    assert!(
        kept_frames > 0,
        "ANTI-TAUTOLOGY: the SIBLING must be carried, or 'the excluded topic is \
         absent' is satisfied by a capture that holds nothing at all"
    );
    assert_eq!(
        excluded_frames, 0,
        "an excluded topic must contribute NO frames to the window — got {excluded_frames} \
         against a sibling's {kept_frames}"
    );

    std::fs::remove_dir_all(&dir).ok();
    std::fs::remove_file(&ready).ok();
}
