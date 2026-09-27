// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end: a FLASHBACK CAPTURE carries the node state a
//! resume would begin from.
//!
//! Real `StateRingOwner` producer, real recorder, real trigger channel, real
//! finished MCAP read back. Isolated per-test SHM roots + unique ring tags, so
//! the file is parallel-safe (measured: 20/20 green at libtest's default
//! parallelism, and 12/12 under CI's own `--test-threads=1`).
//!
//! # The three rendezvous rules every arm here follows
//!
//! A new arm that skips any of them reproduces a failure this file has already
//! paid for once, and none is visible from the assertion that ends up red.
//!
//! * **Nothing may be published, and no wall may be started, before the
//!   recorder's READY-FILE appears.** `run_bagd` writes that sentinel the moment
//!   `Recorder::setup` returns — which is where the data-only TAPS attach and
//!   where the flashback plane's own clock begins — so before it, a frame lands
//!   in no queue at all (a tap requests no late-joiner history, so the frame is
//!   not late, it is GONE) and a wall started at `thread::spawn` measures a
//!   stretch of time the recorder has not lived through. Both failed on main CI:
//!   `a_capture_from_a_run_with_no_state_ring_...` failed at "a
//!   capture with no anchor still carries its frames" (its six frames published
//!   ~60 ms after the spawn, into a tap that did not exist), and
//!   `an_anchor_older_than_the_windows_reach_...` failed its `>= 1 ms past the
//!   floor` precondition — its 250 ms wall was spent INSIDE setup, so the
//!   checkpoint was harvested at recorder-clock ~0 and the distance it exists to
//!   create was never created. Anchoring both on the sentinel makes them
//!   LOWER bounds again, which load can only lengthen.
//! * **The recorder must be LISTENING before the trigger is published.**
//!   iceoryx2 pub/sub keeps no history for a subscriber that attaches later, so
//!   a request published before `Recorder::setup` opened its control subscriber
//!   is not delivered LATE — it is not delivered AT ALL, and the arm then burns
//!   its whole liveness ceiling for a reason nothing in the message names. Every
//!   arm that pushes state records rendezvouses on the recorder DRAINING the
//!   ring, which happens on the drive loop and therefore strictly after setup;
//!   the one arm with no ring at all RE-PUBLISHES under the same id until a
//!   verdict answers, which is what `FlashbackRequester::request_with_id` exists
//!   for and what `cerulion flashback` itself does. FOUND on `Test (Linux)` at
//!   b94bc6e56: one arm at the full 20 s ceiling, seven siblings at ~0.4 s,
//!   running sequentially.
//! * **Nothing variable may sit between the state push and the trigger.** The
//!   late-anchor arm's verdict is decided by exactly that gap against a 300 ms
//!   post window, so opening the requester inside it — `open_on_manager` CREATES
//!   iceoryx2 services and ports — was enough to invert the verdict under load.
//!   FOUND locally at an anchor 1184 ms past the floor against a 1200 ms
//!   boundary, ~1 run in 11 with this file's 8 arms in parallel.
//!
//! # What these arms prove that the unit tests cannot
//!
//! `anchor_window.rs` is oracle-tested in isolation, and every one of those arms
//! stays green if nothing WIRES it: the drive loop could stop draining the ring,
//! the harvester could never be attached to a `SendStateRing`, the selected
//! checkpoint could be dropped on the floor between `select_anchor` and the
//! `CaptureJob`. Those are the inert-shipping shapes, and only a run that pushes
//! real records onto a real ring and then READS THE BAG can see them.
//!
//! It is also the only place the two halves of the fix meet. The retention holds
//! RAW records precisely so a capture's `__cerulion/state` bytes are identical to
//! a recording's — a claim about bytes that only a byte comparison against
//! hand-built records can settle.
//!
//! # The over-claim arm is not a nicety
//!
//! A capture that carries an anchor is still NOT resimmable today: `bag play
//! --resim` derives its resume step from the scheduler trace, and a plain
//! `graph run` mints no trace ring. So one arm reads the manifest and
//! requires it to say so — the alternative is an operator reading "anchor: yes"
//! as "you can resim this", which is the over-claim this vocabulary exists to
//! prevent.
//!
//! # Injected constants, and only these
//!
//! The post window is injected at ~300 ms (the shipped 15 s would make this a
//! minute-long suite for a property that has nothing to do with the number), and
//! two arms inject the retention's BYTE CEILING because they are ABOUT that
//! ceiling. Every other constant is the shipped one, and every deadline is a
//! generous liveness ceiling in seconds rather than a wall stated in units of the
//! thing under test.

#![cfg(unix)]

mod common;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_bagd::{run_bagd, BagdConfig, FlashbackSettings, StateCoverage, TapSpec};
use cerulion_core::flashback::channel::{FlashbackOutcome, FlashbackRequester};
use cerulion_core::flashback::retention::RetentionCaps;
use cerulion_core::flashback::switch::TriggerPosture;
use cerulion_core::flashback::trigger::{CaptureRequest, TriggerPolicy};
use cerulion_core::state_ring::{
    encode_record, encode_skip_record, SkipCause, StateRecordHeader, StateRingOwner,
    RECORD_KIND_CHUNK_V2, RECORD_KIND_FINAL_V2, STATE_RECORD_FORMAT_VERSION, STATE_RECORD_PAYLOAD,
    STATE_RECORD_SIZE,
};

use common::{
    await_bagd_ready, await_condition, build_frame, join_bagd, make_manager, publisher, unique_out,
    unique_ready_file, unique_topic,
};

/// Shipped everywhere except the post window — see the module docs.
const POST_WINDOW_MS: u64 = 300;
/// Generous liveness ceilings. Load can only delay these, never invert them.
const DEADLINE: Duration = Duration::from_secs(20);
/// An arbitrary, stable schema hash for the hand-built data frames.
const HASH: u64 = 0x0C1E_0C1E_0C1E_0C1E;
/// The run every state record in this file carries.
const RUN: u64 = 0x0000_C1E5_0000_0002;
/// Far above what any arm pushes, so no arm is measuring a lap it did not mean.
const RING_RECORDS: u32 = 256;

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "anchor-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

fn unique_ring_tag(tag: &str) -> String {
    format!(
        "anchor_{tag}_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0)
    )
}

fn settings(dir: &Path, anchor_max_bytes: u64, window_only: bool) -> FlashbackSettings {
    settings_spanning(dir, anchor_max_bytes, window_only, Duration::from_secs(30))
}

fn settings_spanning(
    dir: &Path,
    anchor_max_bytes: u64,
    window_only: bool,
    window_span: Duration,
) -> FlashbackSettings {
    FlashbackSettings {
        window_span,
        window_max_bytes: 64 * 1024 * 1024,
        anchor_max_bytes,
        // These arms state a ceiling, so the basis is
        // `Env`. The anchor-first reserve may never grow past a number an
        // operator gave, which is what keeps an injected ceiling meaning what
        // the arm that injected it says it means.
        anchor_cap_basis: cerulion_core::flashback::CapBasis::Env,
        // The shipped default. These arms are about the ANCHOR
        // ceiling, so the trace one is deliberately generous — a ceiling that
        // bit here would change what the capture carries for a reason no arm in
        // this file is asking about.
        trace_max_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TRACE_MAX_MB * 1024 * 1024,
        dir: dir.to_path_buf(),
        label: "demo".into(),
        caps: RetentionCaps::default(),
        policy: TriggerPolicy {
            post_window_ns: POST_WINDOW_MS * 1_000_000,
            ..Default::default()
        },
        posture: TriggerPosture::default(),
        // WINDOW-ONLY selects WHICH THREAD is the state ring's single consumer:
        // the DRIVE LOOP when there is no continuous bag, the WRITER THREAD when
        // there is. Both feed the same retention, and both are driven here.
        // These arms are not about the exclude lever.
        exclude_topics: cerulion_core::flashback::ExcludeTopics::default(),

        window_only,
        // The shipped per-topic tap budget. These arms are
        // about the WINDOW, not the tap depth, so they take the default the
        // resolver would give them.
        tap_budget_bytes: cerulion_core::flashback::DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024,
    }
}

/// A whole anchor for `node_idx` at `step`: `parts - 1` full CHUNKs then a FINAL
/// carrying `tail_len` bytes.
///
/// Hand-built through `cerulion_core`'s own `encode_record`, so the read-back
/// comparison is against records this test decided AND against the production
/// record format — a framing change fails here rather than producing a bag full
/// of bytes no reader accepts.
fn anchor(node_idx: u32, step: u64, parts: u32, tail_len: usize) -> Vec<Vec<u8>> {
    anchor_on_rank(0, node_idx, step, parts, tail_len)
}

/// The same anchor, stamped with a WORKER RANK of the caller's choosing.
///
/// `StateRingProducer::push_record` asserts that a record's header rank matches
/// the ring's own, so a record for rank 1 has to be built for rank 1 rather than
/// re-stamped on the way in. The rank is the first parameter because it is the
/// ring the record belongs to, which is the outermost fact about it.
fn anchor_on_rank(
    rank: u32,
    node_idx: u32,
    step: u64,
    parts: u32,
    tail_len: usize,
) -> Vec<Vec<u8>> {
    let mut out = Vec::with_capacity(parts as usize);
    for part in 0..parts - 1 {
        out.push(
            encode_record(
                &StateRecordHeader {
                    run_id: RUN,
                    step,
                    node_idx,
                    part,
                    kind: RECORD_KIND_CHUNK_V2,
                    len: STATE_RECORD_PAYLOAD as u32,
                    rank,
                    format_version: STATE_RECORD_FORMAT_VERSION,
                },
                &vec![(0xA0 + node_idx as u8).wrapping_add(part as u8); STATE_RECORD_PAYLOAD],
            )
            .to_vec(),
        );
    }
    out.push(
        encode_record(
            &StateRecordHeader {
                run_id: RUN,
                step,
                node_idx,
                part: parts - 1,
                kind: RECORD_KIND_FINAL_V2,
                len: tail_len as u32,
                rank,
                format_version: STATE_RECORD_FORMAT_VERSION,
            },
            &vec![0x5A; tail_len],
        )
        .to_vec(),
    );
    out
}

/// Push `records` onto a ring, through a producer the caller MINTED ONCE.
///
/// The mint is the caller's because a state ring is SPSC and `producer()` is
/// mint-once — a second call answers `None`, so a helper that minted its own
/// would work exactly once and then silently push nothing.
fn push_records(producer: &mut cerulion_core::state_ring::StateRingProducer, records: &[Vec<u8>]) {
    for r in records {
        let mut buf = [0u8; STATE_RECORD_SIZE as usize];
        buf.copy_from_slice(r);
        producer.push_record(&buf);
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

/// What a refused capture left behind.
struct Refusal {
    dir: PathBuf,
    /// The reason the requester was told, verbatim.
    reason: String,
}

/// Drive a capture whose retention ceiling refuses every checkpoint, and collect
/// the REFUSAL.
///
/// A second harness rather than a flag on `capture_with_records`, because the two
/// wait for OPPOSITE things: that one blocks until a capture is `Finished` and
/// asserts a bag exists, and by design this path produces neither. A
/// shared harness would have to be told which verdict to expect, which is the
/// whole content of what these arms assert.
fn refused_capture_with_records(tag: &str, records: &[Vec<u8>], anchor_max_bytes: u64) -> Refusal {
    let mgr = make_manager(64);
    let topic = unique_topic("/fba/refused");
    let dir = temp_dir(tag);
    let ring_tag = unique_ring_tag(tag);

    let mut cfg = BagdConfig::new(
        unique_out(&format!("fba_{tag}")),
        vec![TapSpec::attach(&topic)],
    );
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    cfg.flashback = Some(settings(&dir, anchor_max_bytes, true));
    let ready = unique_ready_file("fba_refused");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, 256);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["alpha", "beta"])
        .expect("create the state ring");
    cfg.state_rings = vec![owner.name().to_string()];
    let mut producer = owner.producer().expect("the single producer");

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, &format!("the '{tag}' refusal driver"));

    // Frames as well as state: a refusal must be refusing a capture that had
    // something to carry, or the arm cannot tell "refused" from "nothing to do".
    push_records(&mut producer, records);
    for seq in 0..8u32 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        await_condition(DEADLINE, || {
            producer.free_records() == Some(u64::from(RING_RECORDS))
        }),
        "the recorder must drain the state ring before the trigger"
    );
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("an operator saw the wobble"))
        .expect("request");

    let mut reason = None;
    assert!(
        await_condition(DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if let FlashbackOutcome::Refused { reason: r, .. } = &frame.outcome {
                    reason = Some(r.clone());
                }
                assert!(
                    !matches!(frame.outcome, FlashbackOutcome::Finished { .. }),
                    "the project rule forbids finalizing a capture that cannot resim; this one \
                     was FINISHED: {:?}",
                    frame.outcome
                );
            }
            reason.is_some()
        }),
        "the recorder must REFUSE this capture over the trigger channel"
    );

    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, &format!("the '{tag}' refusal driver")).expect("recorder");
    std::fs::remove_file(&ready).ok();
    Refusal {
        dir,
        reason: reason.expect("a refusal reason"),
    }
}

/// The state records a finished capture carries, in file order.
fn state_records(bag: &Path) -> Vec<Vec<u8>> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "a reported capture must be FINALIZED, got {completeness:?}"
    );
    msgs.into_iter()
        .filter(|m| m.topic == cerulion_bag::STATE_TOPIC)
        .map(|m| m.data)
        .collect()
}

/// A capture's `__cerulion/state_coverage.json`, when it carries one.
fn state_coverage(bag: &Path) -> Option<StateCoverage> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let att = reader
        .attachment(cerulion_bagd::STATE_COVERAGE_ATTACHMENT)
        .expect("read attachments")?;
    Some(serde_json::from_slice(&att.data).expect("state_coverage.json parses"))
}

/// A capture's `__cerulion/flashback.json`.
fn flashback_manifest(bag: &Path) -> serde_json::Value {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let att = reader
        .attachment("__cerulion/flashback.json")
        .expect("read attachments")
        .expect("every capture carries its own manifest");
    serde_json::from_slice(&att.data).expect("flashback.json parses")
}

/// Everything one arm needs to drive a capture and read it back.
struct Harness {
    dir: PathBuf,
    bag: PathBuf,
    /// The CONTINUOUS bag, on a `--record` run. `None` in window-only mode,
    /// where there is none by definition.
    continuous: Option<PathBuf>,
}

/// Run a window-only recorder against a real state ring: push `records`, publish
/// `frames` data frames, then trigger a manual capture and read the bag.
///
/// The ORDER is the whole point of a black box and is the same order the frame
/// half's headline uses: everything is published BEFORE anybody asks for it, so
/// nothing in the capture got there by being requested.
fn capture_with_records(
    tag: &str,
    records: &[Vec<u8>],
    anchor_max_bytes: u64,
    window_only: bool,
) -> Harness {
    capture_with_records_after(tag, records, anchor_max_bytes, window_only, 0)
}

/// The same driver, with `warmups` captures triggered and finished BEFORE the one
/// the caller reads.
///
/// It exists for one property: a capture's `seq` counts from 0, so the FIRST
/// capture of a run stamps 0 onto every restore point, and an arm that asserts
/// the stamp against that capture's own `seq` is asserting `0 == 0`. Such an arm
/// passes against a stamp that was never written, which is the shape a mutant
/// measured here: forcing the per-rank `capture_seq` to a constant 0 left the
/// whole suite green. A warm-up capture moves the number off 0 and the same
/// assertion starts being able to fail.
///
/// The warm-ups select the SAME checkpoints, since a capture consumes nothing
/// from the retention, so everything else the caller asserts about the bag is
/// unchanged by them.
fn capture_with_records_after(
    tag: &str,
    records: &[Vec<u8>],
    anchor_max_bytes: u64,
    window_only: bool,
    warmups: usize,
) -> Harness {
    let mgr = make_manager(64);
    let topic = unique_topic("/fba/probe");
    let dir = temp_dir(tag);
    let ring_tag = unique_ring_tag(tag);

    let mut cfg = BagdConfig::new(
        unique_out(&format!("fba_{tag}")),
        vec![TapSpec::attach(&topic)],
    );
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    cfg.flashback = Some(settings(&dir, anchor_max_bytes, window_only));
    let ready = unique_ready_file("fba_driver");
    cfg.ready_file = Some(ready.clone());

    // The tap is OPEN-ONLY, so the producer must exist before `Recorder::setup`.
    let mut pub_ = publisher(&mgr, &topic, 256);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["alpha", "beta"])
        .expect("create the state ring");
    cfg.state_rings = vec![owner.name().to_string()];
    // MINTED ONCE, and held for the whole arm: a state ring is SPSC, so the
    // second `producer()` answers `None`. It is also the rendezvous below — the
    // producer is the only handle in this process that can see the consumer's
    // cursor without becoming a second consumer.
    let mut producer = owner.producer().expect("the single producer");

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    // …and the tap must EXIST before the frames below, or the frames-only arms
    // assert on a capture that carries nothing for a reason unrelated to what
    // they are named for (rendezvous rule 1).
    await_bagd_ready(&ready, &format!("the '{tag}' capture driver"));

    // The PRE-WINDOW: state and frames, both before any request.
    push_records(&mut producer, records);
    for seq in 0..8u32 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    // The recorder must have DRAINED the ring before the capture is triggered —
    // a capture that fired first would be frames-only for a reason that has
    // nothing to do with the code under test, and the arms below would be
    // measuring the harness. Waited on as a CONDITION (the records leaving the
    // ring, read off the producer's view of the consumer cursor), never as a
    // sleep, under a liveness ceiling load can delay but not invert.
    assert!(
        await_condition(DEADLINE, || {
            producer.free_records() == Some(u64::from(RING_RECORDS))
        }),
        "the recorder must drain the state ring — nothing else in this process consumes it"
    );

    // …and the drained checkpoints must be OLDER than the deadline this capture
    // will derive, or the arm is not asking what it is named for.
    //
    // The anchor deadline is `trigger − post_window`. These checkpoints
    // are admitted a few milliseconds after `push_records`, so triggering straight
    // after the drain puts BOTH of them INSIDE the post window — where `select`
    // correctly refuses the newest and falls back to the oldest, and the arm reads
    // the step-10 records while asserting the step-20 ones. MEASURED before this
    // wait existed: 1 failure in 3 runs IN ISOLATION, the two checkpoints landing
    // on either side of the boundary depending on whether the recorder harvested
    // them in one drive pass or two.
    //
    // A sleep is the right instrument here and a condition is not available: what
    // has to become true is that WALL TIME has passed, since the deadline is
    // defined against the trigger instant and nothing observable moves with it.
    // It is a LOWER bound, so load can only push the checkpoints further from the
    // boundary — never across it.
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    // The WARM-UPS, each finished before the next is asked for. Waiting for
    // `Finished` is what makes them separate captures rather than one: a request
    // that arrives while a capture is still open is COALESCED into it, and the
    // seq would not advance.
    //
    // A MANUAL request is never held by the refractory floor (that floor is
    // automatic-only, because a manual request is a deliberate act), so
    // back-to-back manual captures are the shipped behaviour rather than a
    // fixture trick.
    for n in 0..warmups {
        let warm_id = requester
            .request(&CaptureRequest::manual(
                "a warm-up so the next seq is not zero",
            ))
            .expect("request a warm-up capture");
        let mut warm_done = false;
        assert!(
            await_condition(DEADLINE, || {
                for frame in requester.drain_outcomes(warm_id) {
                    if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                        warm_done = true;
                    }
                }
                warm_done
            }),
            "the recorder must report warm-up capture {n} FINISHED"
        );
    }
    let request_id = requester
        .request(&CaptureRequest::manual("an operator saw the wobble"))
        .expect("request");

    let mut finished = false;
    assert!(
        await_condition(DEADLINE, || {
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
    let summary =
        join_bagd(handle, &shutdown, &format!("the '{tag}' capture driver")).expect("recorder");
    std::fs::remove_file(&ready).ok();
    let continuous = summary.bag_paths.first().cloned();
    assert_eq!(
        continuous.is_none(),
        window_only,
        "a window-only recorder writes NO continuous bag and a `--record` one writes exactly \
         one — the mode has to be REAL for the writer-thread arm to be driving what it says"
    );

    let bags = captures(&dir);
    assert_eq!(
        bags.len(),
        warmups + 1,
        "one bag per trigger, the warm-ups included, got {bags:?}"
    );
    // The LAST one, which is the capture the caller asked for: the file name
    // carries the capture's zero-padded seq after a non-decreasing stamp, so
    // sorted order is trigger order.
    let bag = bags[bags.len() - 1].clone();

    // PRECONDITION, asserted rather than assumed, and SCOPED to the captures that
    // embed: IF this capture carries an anchor at all, it must be one eligible for
    // the window it claims.
    //
    // Callers that assert the NEWEST checkpoint's records depend on that — `select`
    // serves the newest checkpoint AT OR BEFORE the deadline, so if the wait above
    // ever stops putting them there, this fails naming the reason instead of those
    // callers failing with a wall of bytes that differ for a reason nothing in
    // their body explains. It is deliberately NOT asserted unconditionally: the
    // ceiling and torn arms drive this same driver to a FRAMES-ONLY capture on
    // purpose, and demanding a fit from them would assert the opposite of what
    // they exist to prove.
    let manifest = flashback_manifest(&bag);
    if manifest["anchor"]["embedded"] == serde_json::json!(true) {
        assert_eq!(
            manifest["anchor"]["fit"],
            serde_json::json!("covers_the_claimed_window"),
            "the drained checkpoints must be older than `trigger − post_window`, or the \
             newest one is not the one `select` can serve: {manifest}"
        );
    }

    Harness {
        dir,
        bag,
        continuous,
    }
}

/// When the byte ceiling evicts PAST the anchor, the bag says
/// which side of it the gap is on.
///
/// # Why the pair, and why this arm
///
/// A resume re-executes from the anchor onward while the bag holds frames from
/// its ACHIEVED reach onward, and those two instants can fall either way round.
/// The earlier report measured `frames_before_anchor_ms` from the CLAIMED
/// floor, which is the one case it can never describe: with the ceiling biting,
/// the floor is frozen where the trigger put it while the frames behind it are
/// gone, so a floor-relative figure counts a stretch the bag does NOT carry as
/// recording the resume will skip — and the frames the resume actually NEEDS and
/// does not have go unmentioned entirely.
///
/// The fixture separates the two ceilings deliberately: the ANCHOR retention is
/// generous (so a checkpoint really is embedded) while the FRAME window is a few
/// frames wide (so its reach walks past that checkpoint). That combination is
/// what no other arm in this file produces, and it is the one the pair exists
/// for.
///
/// Measuring `frames_before_anchor_ms` from `floor_ns` again reports a
/// nonzero span before an anchor the bag does not reach, and fails here.
#[test]
fn an_anchor_older_than_the_windows_reach_reports_the_frames_the_bag_is_missing() {
    const PAYLOAD: usize = 2 * 1024;
    const CAP_BYTES: u64 = 8 * 1024;

    let mgr = make_manager(64);
    let topic = unique_topic("/fba/gap");
    let dir = temp_dir("gap");
    let ring_tag = unique_ring_tag("gap");

    let mut cfg = BagdConfig::new(unique_out("fba_gap"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    let mut s = settings(&dir, 64 * 1024 * 1024, true);
    // The ONE injected constant beyond the post window: a FRAME ceiling a few
    // frames wide. The anchor ceiling above stays generous, which is the whole
    // point — the checkpoint must survive while the frames around it do not.
    s.window_max_bytes = CAP_BYTES;
    cfg.flashback = Some(s);
    let ready = unique_ready_file("fba_gap");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, PAYLOAD as u32 + 64);
    let payload = vec![0xCDu8; PAYLOAD];
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["alpha", "beta"])
        .expect("create the state ring");
    cfg.state_rings = vec![owner.name().to_string()];
    let mut producer = owner.producer().expect("the single producer");

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the anchor-older-than-the-reach arm");

    // The checkpoint is pushed AFTER a short stretch of run, then drained before
    // any frame is published — so its stamp is provably older than everything
    // the window ends up holding, AND provably some distance past the capture's
    // floor (which saturates to zero on a run this short). Both halves matter:
    // the first is what puts the gap on the MISSING side, and the second is what
    // makes a floor-relative figure a visibly different number rather than a
    // rounding-to-zero coincidence.
    //
    // A sleep is the right instrument and a condition is not available: what has
    // to become true is that the recorder's own clock has advanced, and nothing
    // observable from here moves with it.
    //
    // ITS ORIGIN IS THE READY-FILE, NOT THE SPAWN, and that is the whole of its
    // load-safety (this arm's CI failure, at the `>= 1 ms`
    // precondition below). The recorder's flashback clock begins inside
    // `Recorder::setup`; a wall started at `thread::spawn` is spent partly — on
    // a loaded runner, ENTIRELY — before that clock exists, so the checkpoint
    // was harvested at recorder-clock ~0 and `taken_at_ns − floor_ns` rounded to
    // 0 ms. Started after the sentinel it is a true LOWER bound on the
    // recorder's OWN elapsed time, so load can only push the anchor FURTHER from
    // the floor.
    std::thread::sleep(Duration::from_millis(250));
    let mut records = anchor(0, 10, 1, 16);
    records.extend(anchor(1, 10, 1, 16));
    push_records(&mut producer, &records);
    assert!(
        await_condition(DEADLINE, || {
            producer.free_records() == Some(u64::from(RING_RECORDS))
        }),
        "the recorder must drain the state ring before the frames start"
    );

    // …then far more bytes than the window can hold, so its reach walks forward
    // past that checkpoint.
    let mut seq = 0u32;
    for _ in 0..30 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), &payload))
            .expect("publish");
        seq += 1;
        std::thread::sleep(Duration::from_millis(10));
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");
    let request_id = requester
        .request(&CaptureRequest::manual("the reach walked past the anchor"))
        .expect("request");

    // The inflow CONTINUES through the post window, which keeps the reach moving
    // while the capture records.
    let mut finished = false;
    for _ in 0..60 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), &payload))
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
            await_condition(DEADLINE, || {
                requester
                    .drain_outcomes(request_id)
                    .iter()
                    .any(|f| matches!(f.outcome, FlashbackOutcome::Finished { .. }))
            }),
            "the recorder must report the capture FINISHED"
        );
    }

    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the anchor-older-than-the-reach arm").expect("recorder");
    std::fs::remove_file(&ready).ok();

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1, "exactly one capture, got {bags:?}");
    let m = flashback_manifest(&bags[0]);

    // PRECONDITIONS, asserted rather than assumed: an anchor really is embedded,
    // and the frame window really did lose its reach to the ceiling. Either one
    // failing means the fixture stopped producing the shape this arm is named
    // for, which must read as a fixture failure rather than as a verdict.
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "the anchor ceiling is generous, so a checkpoint must survive: {m}"
    );
    let floor = m["floor_ns"].as_u64().expect("a floor");
    let achieved_from = m["achieved_from_ns"]
        .as_u64()
        .expect("this fixture publishes throughout, so the capture carries frames");
    assert!(
        achieved_from > floor,
        "the frame ceiling must have evicted past the floor ({achieved_from} vs {floor}): {m}"
    );

    let before = m["anchor"]["frames_before_anchor_ms"]
        .as_u64()
        .expect("an embedded anchor states how much of the bag predates it");
    let missing = m["anchor"]["frames_missing_after_anchor_ms"]
        .as_u64()
        .expect("…and how much of the anchor's forward span the bag is missing");
    assert!(
        missing > 0,
        "the anchor predates the bag's reach, so the resume needs frames this bag has not got \
         (missing {missing} ms): {m}"
    );
    // PRECONDITION for the mutation this arm exists to kill: the anchor must sit
    // a MEASURABLE distance past the floor, or a floor-relative figure would
    // round to the same 0 the correct one produces and the assertion below would
    // be satisfied by both.
    let taken_at = m["anchor"]["taken_at_ns"]
        .as_u64()
        .expect("an embedded anchor states when it was harvested");
    let anchor_ms_past_floor = taken_at.saturating_sub(floor) / 1_000_000;
    assert!(
        anchor_ms_past_floor >= 1,
        "the anchor must be at least a millisecond past the floor for this arm to discriminate \
         (it is {anchor_ms_past_floor} ms): {m}"
    );
    assert_eq!(
        before, 0,
        "…and NOTHING in the bag predates the anchor, so the other half is 0. A floor-relative \
         figure here would report {anchor_ms_past_floor} ms of recording that is not in this \
         bag: {m}"
    );

    std::fs::remove_dir_all(&dir).ok();
}

// ===========================================================================
// THE HEADLINE
// ===========================================================================

/// A capture carries the NEWEST checkpoint, its records byte-identical to what
/// crossed the ring, and a manifest that makes them readable.
///
/// Three claims in one body, each of which the others cannot stand in for:
///
/// * the RECORDS are the ring's own bytes (a re-encode would differ);
/// * the checkpoint is the NEWEST usable one, not the first the retention saw —
///   the inversion PR 1 named as its residual, and the reason this work exists;
/// * `state_coverage.json` is present with the `node_idx → node id` table, which
///   is what a reader needs to walk the state channel at all.
#[test]
fn a_capture_carries_the_newest_checkpoints_records_verbatim() {
    // Two checkpoints. Step 10 is a whole checkpoint (both nodes); step 20 is
    // the NEWER one, and is what the capture must carry.
    let mut older: Vec<Vec<u8>> = Vec::new();
    older.extend(anchor(0, 10, 3, 7));
    older.extend(anchor(1, 10, 1, 0));
    let mut newer: Vec<Vec<u8>> = Vec::new();
    newer.extend(anchor(0, 20, 2, 5));
    newer.extend(anchor(1, 20, 1, 3));

    let mut all = older.clone();
    all.extend(newer.clone());
    // ONE WARM-UP CAPTURE FIRST, so this capture's `seq` is 1 rather than 0 and
    // the identity assertion at the end of this arm can fail. See
    // `capture_with_records_after`: with the run's first capture the same
    // assertion reads 0 == 0 and passes against a stamp that was never written.
    let h = capture_with_records_after("head", &all, 64 * 1024 * 1024, true, 1);

    let got = state_records(&h.bag);
    assert_eq!(
        got, newer,
        "the capture must carry the NEWEST checkpoint's records VERBATIM — a \
         fill-then-skip ring would have served step 10, which is exactly the \
         inversion this closes"
    );
    // …and it is not merely "some records": the OLDER checkpoint's bytes must be
    // absent, so the arm cannot pass against a capture that dumped everything.
    for record in &older {
        assert!(
            !got.contains(record),
            "a capture carries ONE checkpoint, not the retention"
        );
    }

    // The manifest without which those records are unreadable.
    let cov = state_coverage(&h.bag).expect("a capture with an anchor carries state_coverage.json");
    assert_eq!(cov.version, cerulion_bagd::STATE_COVERAGE_VERSION);
    assert_eq!(
        cov.rings_declared, 1,
        "one ring contributed, so a restore engine can resolve node_idx without ambiguity"
    );
    assert!(
        !cov.attached_mid_run,
        "a capture writes WHOLE anchors from part 0 — claiming a mid-run attach would make a \
         reader DISCARD the head of the very anchor the capture exists to carry"
    );
    assert_eq!(cov.records, newer.len() as u64);
    assert_eq!(cov.nodes.len(), 2, "both nodes' anchors are named");
    assert_eq!(cov.nodes["alpha"].node_idx, Some(0));
    assert_eq!(cov.nodes["beta"].node_idx, Some(1));
    assert_eq!(cov.nodes["alpha"].anchors_complete, 1);
    assert_eq!(cov.nodes["beta"].anchors_complete, 1);
    assert!(
        cov.unattributed_indices.is_empty(),
        "every index resolved through the ring's manifest"
    );

    // The flashback manifest reports the SAME checkpoint, so the two artifacts
    // in one bag cannot disagree.
    let m = flashback_manifest(&h.bag);
    assert_eq!(m["anchor"]["embedded"], serde_json::json!(true));
    assert_eq!(m["anchor"]["step"], serde_json::json!(20));
    assert_eq!(m["anchor"]["run_id"], serde_json::json!(RUN));
    assert_eq!(m["anchor"]["nodes"], serde_json::json!(2));
    assert_eq!(m["anchor"]["complete"], serde_json::json!(2));
    assert_eq!(
        m["anchor"]["records"],
        serde_json::json!(newer.len()),
        "the manifest's record count must be what the bag actually holds"
    );

    // THE PER-RANK BLOCK, over the production path. This run has one rank, so
    // the block has exactly one entry and its numbers are the scalars above —
    // the k=1 identity the fold has by construction, asserted here against a
    // manifest a real recorder wrote rather than a hand-built report.
    let per_rank = m["anchor"]["per_rank"]
        .as_object()
        .expect("every embedded anchor block carries its set, keyed by rank");
    assert_eq!(per_rank.len(), 1, "a one-rank run writes one entry");
    let rank0 = &per_rank["0"];
    assert_eq!(rank0["step"], serde_json::json!(20));
    assert_eq!(rank0["nodes"], serde_json::json!(2));
    assert_eq!(rank0["complete"], serde_json::json!(2));
    assert_eq!(rank0["records"], serde_json::json!(newer.len()));
    assert_eq!(
        rank0["fit"], m["anchor"]["fit"],
        "one rank, so the entry's fit and the folded scalar are the same answer"
    );
    // The CAPTURE IDENTITY, and the NUMBER is written out here by hand.
    //
    // This is the second capture of the run, so both sides are 1 and neither is
    // the value a missing stamp reads. Against `m["seq"]` alone the assertion was
    // VACUOUS and a mutant proved it: forcing the per-rank `capture_seq` to a
    // constant 0 left this arm green, which can only mean `m["seq"]` read 0 too.
    // The literal is therefore the load-bearing half, and the equality against
    // `seq` stays beside it so the two halves of the manifest cannot disagree.
    assert_eq!(
        m["seq"],
        serde_json::json!(1),
        "PRECONDITION: the warm-up must have taken seq 0, or the stamp below is \
         asserted against the one value a missing stamp also reads: {m}"
    );
    assert_eq!(
        rank0["capture_seq"],
        serde_json::json!(1),
        "the restore point names the capture event that selected it, by number"
    );
    assert_eq!(
        rank0["capture_seq"], m["seq"],
        "…and that number is the one the manifest states at the top"
    );
    // …and the Q8 stamp's ANTI-VACUITY half over the production path: every
    // rank this run had contributed, so the block is present and EMPTY. Absent
    // would mean a reader cannot tell "no rank is missing" from "this recorder
    // does not say".
    assert_eq!(
        m["anchor"]["missing_ranks"],
        serde_json::json!([]),
        "a capture with every rank present says so"
    );

    std::fs::remove_dir_all(&h.dir).ok();
}

/// One rank's half of a MULTI-RANK run: the ring's own header rank, the node
/// manifest that ring declares, and the records that rank publishes.
struct RankPlan {
    /// The ring header's rank, which is also the rank every record it carries
    /// stamps (`StateRingProducer` takes it at the mint).
    rank: u32,
    /// This ring's node table. DIFFERENT lists per rank, so a `node_idx`
    /// resolved through the wrong rank's table names the wrong node.
    nodes: &'static [&'static str],
    /// What this rank publishes. EMPTY declares the ring and publishes nothing,
    /// which is the shape that puts the rank in the capture's shortfall.
    records: Vec<Vec<u8>>,
    /// Push these immediately BEFORE the trigger rather than a whole post window
    /// ahead of it, so this rank's checkpoint is stamped INSIDE the post window
    /// and its member's fit is the degraded one.
    inside_the_post_window: bool,
}

/// Every manifest a multi-rank run wrote, in trigger order.
struct MultiRankRun {
    dir: PathBuf,
    manifests: Vec<serde_json::Value>,
}

/// Drive a window-only recorder against ONE REAL STATE RING PER RANK and read
/// every capture's manifest back.
///
/// # Why this driver exists, measured rather than assumed
///
/// `capture_with_records` declares exactly one ring, and every other arm in this
/// file is therefore a k=1 run: the binary's own comment says so. The recorder's
/// aggregation fold (the `Embedded` block's summed counts, its latest instant,
/// its worst fit, its per-rank entries and its missing-rank stamp) is built at
/// the capture close and NOWHERE ELSE, so a k=1 run exercises every one of those
/// folds at the point where each is the identity. Seven mutations of that fold
/// survived the whole suite for exactly that reason. This driver is what makes
/// them fail.
///
/// # The ORDER, and what each step of it buys
///
/// * Ranks are drained ONE AT A TIME, in vector order, with the drain waited on
///   as a CONDITION between them. The fold takes the LATEST member by
///   `(taken_at_ns, step)`, and a rank whose records reach the recorder after its
///   predecessor's have all left the ring is harvested on a later drive pass and
///   therefore stamped later. So "the last plan in the vector is the latest
///   member" is a fact the driver establishes rather than a race it hopes for —
///   and where the arms also give the later rank the higher STEP, the tie-break
///   agrees, so the two keys cannot disagree.
/// * The pre-window ranks are then left alone for two post windows before the
///   trigger. The anchor deadline is `trigger − post_window`, so this is what
///   makes their members COVER the claimed window. It is a LOWER bound: load can
///   only move them further from the boundary, never across it.
/// * The post-window ranks are pushed LAST, with the trigger immediately after.
///   Their checkpoints are stamped inside the post window, which is what makes
///   their fit the degraded one. Load delays the drain, which moves the stamp
///   LATER and further INTO the band, so it cannot invert the verdict either.
///
/// Rendezvous rules 1 and 2 of the module docs are followed unchanged: nothing is
/// published and no wall starts before the ready-file, and every trigger is
/// published after the recorder has been seen draining a ring.
fn multi_rank_captures(tag: &str, plans: Vec<RankPlan>, captures_wanted: usize) -> MultiRankRun {
    let mgr = make_manager(64);
    let topic = unique_topic("/fba/krank");
    let dir = temp_dir(tag);

    let mut cfg = BagdConfig::new(
        unique_out(&format!("fba_{tag}")),
        vec![TapSpec::attach(&topic)],
    );
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    // A generous anchor ceiling: these arms are about the FOLD, and a ceiling
    // that bit would change which members the set has for a reason none of them
    // is asking about.
    cfg.flashback = Some(settings(&dir, 64 * 1024 * 1024, true));
    let ready = unique_ready_file(&format!("fba_{tag}"));
    cfg.ready_file = Some(ready.clone());

    // The tap is OPEN-ONLY, so the producer must exist before `Recorder::setup`.
    let mut pub_ = publisher(&mgr, &topic, 256);

    // ONE REAL RING PER RANK, created and HELD to the end of this function.
    // Declaring a name with no ring behind it would exercise the recorder's
    // degraded-open path instead of the multi-ring one, which is a different arm.
    let mut owners: Vec<cerulion_core::state_ring::StateRingOwner> = Vec::new();
    for plan in &plans {
        let ring_tag = unique_ring_tag(&format!("{tag}r{}", plan.rank));
        let owner = StateRingOwner::create(&ring_tag, RING_RECORDS, plan.rank, RUN, plan.nodes)
            .expect("create a rank's state ring");
        cfg.state_rings.push(owner.name().to_string());
        owners.push(owner);
    }
    // MINTED ONCE each and held: a state ring is SPSC, so a second `producer()`
    // answers `None`. Each is also its rank's rendezvous, being the only handle
    // in this process that can see that ring's consumer cursor.
    let mut producers: Vec<cerulion_core::state_ring::StateRingProducer> = owners
        .iter_mut()
        .map(|o| o.producer().expect("the single producer"))
        .collect();

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, &format!("the '{tag}' multi rank capture driver"));

    // The PRE-WINDOW frames, so the window has a floor and a reach to measure
    // the per-rank frame spans against.
    for seq in 0..8u32 {
        pub_.publish_raw(&build_frame(HASH, seq, 1_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    // THE PRE-WINDOW RANKS, one at a time with the drain waited on between them.
    for (i, plan) in plans.iter().enumerate() {
        if plan.inside_the_post_window {
            continue;
        }
        push_records(&mut producers[i], &plan.records);
        let rank = plan.rank;
        assert!(
            await_condition(DEADLINE, || {
                producers[i].free_records() == Some(u64::from(RING_RECORDS))
            }),
            "the recorder must drain rank {rank}'s state ring — nothing else in this process \
             consumes it"
        );
    }

    // …and their checkpoints must be OLDER than the deadline the capture derives,
    // or the members read the degraded fit and the arms are not asking what they
    // are named for. A sleep is the right instrument and a condition is not
    // available: what has to become true is that WALL TIME has passed, since the
    // deadline is defined against the trigger instant.
    std::thread::sleep(Duration::from_millis(POST_WINDOW_MS * 2));

    // Opened BEFORE the post-window pushes below. `open_on_manager` CREATES
    // iceoryx2 services and ports, and anything variable between a post-window
    // push and the trigger is competing with the post window's own budget — the
    // load inversion this file has already paid for once.
    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");

    // THE POST-WINDOW RANKS, last, with the trigger immediately after.
    for (i, plan) in plans.iter().enumerate() {
        if plan.inside_the_post_window {
            push_records(&mut producers[i], &plan.records);
        }
    }

    for n in 0..captures_wanted {
        let request_id = requester
            .request(&CaptureRequest::manual("an operator saw a rank wobble"))
            .expect("request");
        let mut finished = false;
        assert!(
            await_condition(DEADLINE, || {
                for frame in requester.drain_outcomes(request_id) {
                    if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                        finished = true;
                    }
                }
                finished
            }),
            "the recorder must report capture {n} FINISHED"
        );
    }

    // Every rank's records really did leave its ring — asserted AFTER the
    // captures, so the wait cannot widen the gap the post-window ranks depend on.
    for (i, plan) in plans.iter().enumerate() {
        let rank = plan.rank;
        assert!(
            await_condition(DEADLINE, || {
                producers[i].free_records() == Some(u64::from(RING_RECORDS))
            }),
            "rank {rank}'s records must have reached the recorder"
        );
    }

    shutdown.store(true, Ordering::Relaxed);
    join_bagd(
        handle,
        &shutdown,
        &format!("the '{tag}' multi rank capture driver"),
    )
    .expect("recorder");
    std::fs::remove_file(&ready).ok();
    drop(producers);
    drop(owners);

    let bags = captures(&dir);
    assert_eq!(
        bags.len(),
        captures_wanted,
        "one bag per trigger, got {bags:?}"
    );
    MultiRankRun {
        dir,
        manifests: bags.iter().map(|b| flashback_manifest(b)).collect(),
    }
}

/// THE k>1 FOLD, over the production path: the counts SUM over the ranks, the
/// step and the instant are the LATEST rank's, and each rank keeps its own
/// numbers beside them.
///
/// # Why this cannot be asserted at the unit level, and why k=1 cannot assert it
///
/// The fold is built in the recorder's capture close and nowhere else: no
/// function takes a set of members and returns the folded block, so there is
/// nothing a unit arm could call. And under k=1 every fold in it is the
/// IDENTITY — the sum over a one-member set is that member's number, the latest
/// of one member is that member, the worst of one fit is that fit — so a k=1
/// arm asserts each rule at the one input where every wrong rule agrees with the
/// right one. That is measured rather than argued: mutations replacing the sum
/// with the first member's count, and the latest member with the earliest,
/// survived this whole binary while it drove one rank only.
///
/// Every expected number here is HAND WRITTEN from the two ranks' inputs: rank 0
/// publishes two nodes at step 20 and rank 1 publishes one node at step 24, so
/// the sums are 3 and the latest rank's step is 24. No expectation is read back
/// off the thing under test.
#[test]
fn a_two_rank_capture_folds_the_counts_and_takes_the_latest_ranks_step() {
    let mut rank0: Vec<Vec<u8>> = Vec::new();
    rank0.extend(anchor_on_rank(0, 0, 20, 1, 6));
    rank0.extend(anchor_on_rank(0, 1, 20, 1, 6));
    let rank1 = anchor_on_rank(1, 0, 24, 1, 6);
    let r0_records = rank0.len();
    let r1_records = rank1.len();

    let run = multi_rank_captures(
        "fold",
        vec![
            RankPlan {
                rank: 0,
                nodes: &["alpha", "beta"],
                records: rank0,
                inside_the_post_window: false,
            },
            RankPlan {
                rank: 1,
                nodes: &["gamma"],
                records: rank1,
                inside_the_post_window: false,
            },
        ],
        1,
    );
    let m = &run.manifests[0];
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "both ranks anchored, so the capture embeds: {m}"
    );

    // THE SET, first, because every assertion below is about a fold over it.
    let per_rank = m["anchor"]["per_rank"]
        .as_object()
        .expect("every embedded anchor block carries its set, keyed by rank");
    assert_eq!(per_rank.len(), 2, "two rings, two entries: {m}");
    let r0 = &per_rank["0"];
    let r1 = &per_rank["1"];

    // EACH RANK'S OWN NUMBERS, hand written from what that rank published.
    assert_eq!(r0["step"], serde_json::json!(20));
    assert_eq!(r0["nodes"], serde_json::json!(2));
    assert_eq!(r0["complete"], serde_json::json!(2));
    assert_eq!(r0["records"], serde_json::json!(r0_records));
    assert_eq!(r1["step"], serde_json::json!(24));
    assert_eq!(r1["nodes"], serde_json::json!(1));
    assert_eq!(r1["complete"], serde_json::json!(1));
    assert_eq!(r1["records"], serde_json::json!(r1_records));

    // THE COUNTS, SUMMED over the set: 2 + 1 = 3, and every one of them
    // complete. ONE RANK'S count is 2 and the other's is 1, so neither is 3 and
    // the sum cannot be satisfied by publishing either member's number.
    assert_eq!(
        m["anchor"]["nodes"],
        serde_json::json!(3),
        "rank 0's two nodes plus rank 1's one, never one ring's count: {m}"
    );
    assert_eq!(m["anchor"]["complete"], serde_json::json!(3));
    assert_eq!(
        m["anchor"]["records"],
        serde_json::json!(r0_records + r1_records),
        "the record count is the whole cross rank checkpoint's: {m}"
    );

    // THE STEP, the LATEST rank's. Rank 1 published the higher step AND was
    // drained later, so both halves of the fold's key agree on it; the earliest
    // member's 20 would claim a resume from a step rank 1 never reached.
    assert_eq!(
        m["anchor"]["step"],
        serde_json::json!(24),
        "the later rank's step, not rank 0's 20: {m}"
    );
    // …and the INSTANT is read off the SAME member as the step, so the pair a
    // reader sees is a checkpoint that was really taken.
    assert_eq!(
        m["anchor"]["taken_at_ns"], r1["taken_at_ns"],
        "the folded instant is the latest member's own: {m}"
    );
    assert_ne!(
        r0["taken_at_ns"], r1["taken_at_ns"],
        "PRECONDITION: the two ranks were drained on different passes, or 'latest' is \
         asserted against two equal instants: {m}"
    );
    assert_eq!(m["anchor"]["run_id"], serde_json::json!(RUN));

    // THE STAMP's control: every declared rank contributed, so it is present and
    // EMPTY rather than absent.
    assert_eq!(
        m["anchor"]["missing_ranks"],
        serde_json::json!([]),
        "a capture with every rank present says so: {m}"
    );

    std::fs::remove_dir_all(&run.dir).ok();
}

/// THE FIT, twice over: the capture reports the WORST rank's, and each rank's
/// entry reports ITS OWN.
///
/// A capture covers the window it claims only if EVERY rank's member does, so
/// the scalar is the worst of them; and a reader who has to find out WHICH rank
/// fell short needs each entry to keep its own answer. The two rules are
/// independent and both are invisible at k=1, where there is one fit and it is
/// both.
///
/// The fixture is the only shape in this file that produces two DIFFERENT fits
/// from one capture: rank 0 is drained two post windows before the trigger, so
/// its member sits at or before the anchor deadline and COVERS; rank 1 is pushed
/// immediately before the trigger, so its member is stamped inside the post
/// window and is NEWER than the claimed window. Both wall arguments push the
/// safe way — load lengthens the sleep before the trigger (rank 0 further from
/// the boundary) and delays rank 1's drain (further into the band).
#[test]
fn a_two_rank_capture_reports_the_worst_fit_and_each_ranks_own() {
    let rank0 = anchor_on_rank(0, 0, 30, 1, 6);
    let rank1 = anchor_on_rank(1, 0, 34, 1, 6);

    let run = multi_rank_captures(
        "fits",
        vec![
            RankPlan {
                rank: 0,
                nodes: &["alpha"],
                records: rank0,
                inside_the_post_window: false,
            },
            RankPlan {
                rank: 1,
                nodes: &["gamma"],
                records: rank1,
                inside_the_post_window: true,
            },
        ],
        1,
    );
    let m = &run.manifests[0];
    let per_rank = m["anchor"]["per_rank"]
        .as_object()
        .expect("every embedded anchor block carries its set, keyed by rank");
    assert_eq!(per_rank.len(), 2, "two rings, two entries: {m}");

    // EACH RANK'S OWN FIT, two different answers written out by hand. An entry
    // carrying the FOLDED fit would read `newer_than_the_claimed_window` for
    // rank 0 as well, and a reader could not tell which rank fell short.
    assert_eq!(
        per_rank["0"]["fit"],
        serde_json::json!("covers_the_claimed_window"),
        "rank 0 was drained two post windows before the trigger: {m}"
    );
    assert_eq!(
        per_rank["1"]["fit"],
        serde_json::json!("newer_than_the_claimed_window"),
        "rank 1 was drained inside the post window: {m}"
    );
    assert_ne!(
        per_rank["0"]["fit"], per_rank["1"]["fit"],
        "PRECONDITION: this fixture exists to produce two different fits: {m}"
    );

    // THE SCALAR, the WORST of them. The BEST would publish a coverage claim
    // rank 1 cannot meet.
    assert_eq!(
        m["anchor"]["fit"],
        serde_json::json!("newer_than_the_claimed_window"),
        "the worst rank's fit, never the best: {m}"
    );

    std::fs::remove_dir_all(&run.dir).ok();
}

/// THE CAPTURE IDENTITY, per rank, across TWO captures of one run.
///
/// Every restore point names the capture event that selected it, so an operator
/// holding two bags can say which capture a given rank's state came from. The
/// number is stamped at SELECTION, and the same checkpoints selected by a second
/// capture therefore carry a different number.
///
/// Two captures rather than one, and the numbers written out by hand, because a
/// capture's `seq` counts from 0: a single capture's arm asserts the stamp
/// against 0, which is also what a stamp that was never written reads. Measured:
/// a mutation writing a constant 0 into every per-rank entry left this binary
/// green while it drove one capture only.
#[test]
fn two_captures_stamp_each_ranks_entry_with_its_own_captures_number() {
    let rank0 = anchor_on_rank(0, 0, 40, 1, 6);
    let rank1 = anchor_on_rank(1, 0, 44, 1, 6);

    let run = multi_rank_captures(
        "ident",
        vec![
            RankPlan {
                rank: 0,
                nodes: &["alpha"],
                records: rank0,
                inside_the_post_window: false,
            },
            RankPlan {
                rank: 1,
                nodes: &["gamma"],
                records: rank1,
                inside_the_post_window: false,
            },
        ],
        2,
    );
    assert_eq!(run.manifests.len(), 2);
    let first = &run.manifests[0];
    let second = &run.manifests[1];

    // The two captures' own numbers, by hand: a run's captures count from 0.
    assert_eq!(first["seq"], serde_json::json!(0));
    assert_eq!(
        second["seq"],
        serde_json::json!(1),
        "the second capture of the run: {second}"
    );

    for (m, seq) in [(first, 0u64), (second, 1u64)] {
        let per_rank = m["anchor"]["per_rank"]
            .as_object()
            .expect("every embedded anchor block carries its set, keyed by rank");
        assert_eq!(per_rank.len(), 2, "two rings, two entries: {m}");
        assert_eq!(
            per_rank["0"]["capture_seq"],
            serde_json::json!(seq),
            "rank 0's entry names capture {seq}: {m}"
        );
        assert_eq!(
            per_rank["1"]["capture_seq"],
            serde_json::json!(seq),
            "rank 1's entry names capture {seq}: {m}"
        );
    }

    // The CHECKPOINTS did not move between the two captures: a capture consumes
    // nothing from the retention, so what changed is the identity and only the
    // identity. Without this the arm above could pass against a second capture
    // that had selected something else entirely.
    assert_eq!(
        second["anchor"]["per_rank"]["0"]["step"],
        serde_json::json!(40)
    );
    assert_eq!(
        second["anchor"]["per_rank"]["1"]["step"],
        serde_json::json!(44)
    );
    assert_eq!(
        first["anchor"]["per_rank"]["0"]["taken_at_ns"],
        second["anchor"]["per_rank"]["0"]["taken_at_ns"],
        "the same checkpoint, so the same instant"
    );

    std::fs::remove_dir_all(&run.dir).ok();
}

/// A capture ONE RANK SHORT is WRITTEN, and it NAMES the rank, the cause and the
/// remedy.
///
/// The unit arm for this builds the report by hand; this one makes a real
/// recorder build it. Rank 1 declares a ring and publishes nothing, which is
/// what a rank whose plane was refused at arm time leaves behind, and the
/// capture has to carry rank 0's state rather than be discarded for rank 1's
/// absence.
///
/// The stamp is asserted as a WHOLE OBJECT against a hand written one, rank,
/// cause and remedy together. Asserting only that the block is non-empty would
/// pass for a stamp naming the wrong rank, and asserting the rank alone would
/// pass for one whose remedy points at the wrong knob.
#[test]
fn a_two_rank_capture_missing_one_rank_stamps_that_rank_in_the_bag() {
    let rank0 = anchor_on_rank(0, 0, 50, 1, 6);

    let run = multi_rank_captures(
        "short",
        vec![
            RankPlan {
                rank: 0,
                nodes: &["alpha"],
                records: rank0,
                inside_the_post_window: false,
            },
            RankPlan {
                rank: 1,
                nodes: &["gamma"],
                records: Vec::new(),
                inside_the_post_window: false,
            },
        ],
        1,
    );
    let m = &run.manifests[0];
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "one rank's absence does not discard the other's state: {m}"
    );

    let per_rank = m["anchor"]["per_rank"]
        .as_object()
        .expect("every embedded anchor block carries its set, keyed by rank");
    assert_eq!(
        per_rank.len(),
        1,
        "only the rank that anchored has an entry: {m}"
    );
    assert_eq!(per_rank["0"]["step"], serde_json::json!(50));

    // THE STAMP, whole and by hand.
    assert_eq!(
        m["anchor"]["missing_ranks"],
        serde_json::json!([{
            "rank": 1,
            "reason": "no_anchor_retained",
            "remedy": "this rank retained no checkpoint: it published no state ring, its \
                       plane was refused at arm time, or it had not reached its first anchor \
                       cadence when the capture was triggered",
        }]),
        "the capture names the rank it lacks, the cause and the remedy: {m}"
    );

    std::fs::remove_dir_all(&run.dir).ok();
}

/// THE OVER-CLAIM ARM: a capture that carries an anchor still says it cannot be
/// resimmed, and why.
///
/// `bag play --resim` derives its resume step from the scheduler trace
/// (`first_recorded_step − 1`) and refuses a mid-run bag with none — and a plain
/// `graph run` mints no trace ring at all. An operator reading
/// `embedded: true` as "you can resim this" would be reading a claim nothing in
/// the bag supports, so both facts ride the same block and the reason is named.
#[test]
fn a_capture_with_an_anchor_and_no_trace_says_it_is_not_resimmable() {
    let records = anchor(0, 30, 1, 9);
    let h = capture_with_records("overclaim", &records, 64 * 1024 * 1024, true);

    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "precondition: this capture really does carry state"
    );
    assert_eq!(
        m["anchor"]["resimmable"],
        serde_json::json!(false),
        "state without a trace is not resumable, and the bag must SAY so rather than leaving \
         an operator to infer it from `embedded`"
    );
    // The COUNT the verdict is computed from:
    // the manifest states it ONCE, inside `handoff`, so `resimmable` and the
    // `trace` verdict cannot disagree about whether this bag holds a trace.
    let trace = m["handoff"]["trace"]
        .as_str()
        .expect("every capture's handoff states a trace verdict");
    assert!(
        trace.contains("handed no trace ring"),
        "this fixture's recorder really was handed none, so the absence names THAT cause \
         rather than the rings-held-none-carried one: {trace}"
    );
    assert!(
        !trace.contains("carried:"),
        "…and it must not claim to carry any: {trace}"
    );
    let reason = m["anchor"]["resimmable_reason"]
        .as_str()
        .expect("a refusal states its reason");
    // **The mid-run attach re-anchored this.** The reason used to be required to name the
    // ISSUE, because a plain `graph run` minted no ring and the remedy was a
    // feature that did not exist. It exists: a multi-process run provisions
    // rings by default, so the actionable half is now WHERE the run recorded
    // what it did about them, and a tracker id in an operator's face would be a
    // shrug rather than the remedy.
    assert!(
        reason.contains("run.json"),
        "…naming where the run stated what it did about its rings, so the reason is \
         actionable rather than a shrug: {reason}"
    );
    // The state IS in the bag and readable — the capture is not resumable, which
    // is a different and weaker statement than "it carries nothing".
    assert_eq!(state_records(&h.bag), records);
    assert!(state_coverage(&h.bag).is_some());

    std::fs::remove_dir_all(&h.dir).ok();
}

/// THE ANTI-TAUTOLOGY ARM: a run with NO state ring produces a capture that says
/// it has no anchor, names WHICH absence it is, and carries no state artifacts.
///
/// Without it, every assertion above is satisfied by a recorder that embeds
/// something unconditionally.
#[test]
fn a_capture_from_a_run_with_no_state_ring_says_so_and_carries_nothing() {
    let mgr = make_manager(64);
    let topic = unique_topic("/fba/none");
    let dir = temp_dir("none");
    let mut cfg = BagdConfig::new(unique_out("fba_none"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    cfg.flashback = Some(settings(&dir, 64 * 1024 * 1024, true));
    let ready = unique_ready_file("fba_none");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, 256);
    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    // THE fix for this arm's CI failure at "a capture with no
    // anchor still carries its frames": with no state ring there is no drain to
    // rendezvous on, so the six frames below went out ~60 ms after the spawn and
    // on a loaded runner landed in a tap that did not exist yet.
    await_bagd_ready(&ready, "the no-state-ring arm");

    for seq in 0..6u32 {
        pub_.publish_raw(&build_frame(HASH, seq, 2_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }

    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");

    // RE-PUBLISHED while nothing has answered, which is this arm's whole
    // rendezvous — the same loop `cerulion flashback` runs, mirroring
    // `flashback_e2e_test`'s rate-cap arm.
    //
    // `FlashbackRequester::request_with_id`'s own docs state the rule: iceoryx2
    // pub/sub keeps no history for a subscriber that attaches later, so a request
    // published before the recorder's control subscriber existed is not delivered
    // LATE, it is not delivered AT ALL. The recorder opens that subscriber at the
    // END of `Recorder::setup` (after the taps), and this arm is the only one in
    // the file that reaches its trigger with nothing proving setup finished:
    // every other arm waits for the state ring to DRAIN, which happens on the
    // drive loop and therefore strictly afterwards. Here there is no state ring
    // at all — that is the point of the arm — so the trigger went out ~60 ms
    // after the recorder thread was spawned and, on a loaded runner, into a
    // service nobody had subscribed to.
    //
    // MEASURED, on `Test (Linux)` at b94bc6e56: the arm burned the full 20 s
    // ceiling while its seven siblings passed in ~0.4 s each, under
    // `--test-threads=1` — i.e. sequentially, with nothing to contend with. A
    // lost request is the only shape that produces that.
    //
    // The re-publish STOPS at the first verdict: further requests would COALESCE
    // into the running capture and extend it, which is the trigger gate working
    // and would delay the FINISHED this arm waits for.
    let request = CaptureRequest::manual("nothing to anchor");
    let request_id = requester.request(&request).expect("request");
    // Every verdict seen, so a run that is answered but never FINISHES fails
    // naming the verdict it got instead of as a silent twenty-second stare.
    let mut seen: Vec<String> = Vec::new();
    let mut finished = false;
    assert!(
        await_condition(DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                    finished = true;
                }
                seen.push(format!("{:?}", frame.outcome));
            }
            if seen.is_empty() {
                let _ = requester.request_with_id(&request, request_id);
            }
            !seen.is_empty()
        }),
        "no recorder ever answered the trigger channel — the request raced \
         `Recorder::setup` and the re-publish never reached a subscriber"
    );
    assert!(
        await_condition(DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                    finished = true;
                }
                seen.push(format!("{:?}", frame.outcome));
            }
            finished
        }),
        "the recorder must still capture — a run with no checkpoints still has a \
         black box (verdicts seen: {seen:?})"
    );
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the no-state-ring arm").expect("recorder");
    std::fs::remove_file(&ready).ok();

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1);
    let bag = &bags[0];

    let m = flashback_manifest(bag);
    assert_eq!(m["anchor"]["embedded"], serde_json::json!(false));
    assert_eq!(
        m["anchor"]["reason"],
        serde_json::json!("no_anchor_retained"),
        "a reader that cannot tell WHICH absence this is cannot act on either"
    );
    assert_eq!(m["anchor"]["resimmable"], serde_json::json!(false));
    assert!(
        state_records(bag).is_empty(),
        "no anchor means no state records"
    );
    assert!(
        state_coverage(bag).is_none(),
        "…and no state manifest either: an attachment claiming a checkpoint that is not there \
         is worse than its absence"
    );
    // The FRAMES are still in the bag — this is the frames-only degradation, and
    // it is still a black box.
    let reader = cerulion_bag::BagReader::open(bag).expect("open");
    let (msgs, _) = reader.recover_messages().expect("recover");
    assert!(
        msgs.iter().any(|msg| msg.topic == topic),
        "a capture with no anchor still carries its frames"
    );

    std::fs::remove_dir_all(&dir).ok();
}

/// The refusal rule, end to end: a robot whose checkpoints do not fit is
/// REFUSED, never handed a frames-only clip.
///
/// # Why this asserts that NOTHING is written
///
/// The opposite posture is conceivable: write the capture anyway, carrying its
/// frames and no anchor. The rule forbids it. Below one checkpoint generation
/// there is NO frames-only fallback, and an operator is never handed a dashcam
/// clip wearing the Flashback name. So this arm pins the refusal itself, and a
/// recorder that degrades to frames-only fails it.
///
/// The ceiling is injected because this arm is ABOUT the ceiling. It is set below
/// one checkpoint, so nothing can be retained at all.
#[test]
fn a_checkpoint_that_does_not_fit_the_ceiling_is_refused_never_written_frames_only() {
    // A three-record anchor against a ceiling of one record.
    let records = anchor(0, 40, 3, 11);
    let r = refused_capture_with_records("ceiling", &records, STATE_RECORD_SIZE as u64);

    // NOTHING was written. Not a frames-only bag, and — the half that catches a
    // refusal which forgot to release its reserved filename — not a zero-byte
    // one either, which `reserve_capture_path` creates to claim the name and
    // which the dashcam sweep and `cerulion bag info` would both take for a
    // capture.
    assert!(
        captures(&r.dir).is_empty(),
        "the project rule refuses the capture outright: no bag, not even the empty one \
         the filename reservation created: {:?}",
        captures(&r.dir)
    );

    // …and the operator is told the knob, not merely that something went wrong.
    assert!(
        r.reason
            .contains("cannot hold one whole checkpoint generation"),
        "the refusal must name the CONDITION: {}",
        r.reason
    );
    assert!(
        r.reason.contains("CERULION_FLASHBACK_"),
        "…and the exact knob that fixes it, which is the whole content of this \
         verdict: {}",
        r.reason
    );
    assert!(
        r.reason.contains("cerulion bag record"),
        "…and the watchable path for an operator who cannot raise it: {}",
        r.reason
    );

    std::fs::remove_dir_all(&r.dir).ok();
}

/// End to end: a capture whose only ceiling refusal was a NEW
/// anchor's FIRST record still names the CEILING.
///
/// The sibling arm above drives the refusal kind that had bytes to abandon. This
/// drives the kind that had none, which is the commoner one on a real robot: the
/// in-flight budget is GLOBAL across the anchors in flight, so a node whose first
/// record arrives while a SIBLING holds the budget is refused with nothing of its
/// own buffered. `drop_open` answers `false` for it, so a counter incremented
/// only inside that branch leaves the whole refusal invisible — and the capture
/// would report `no_anchor_retained` and send its operator to the arm gate when the
/// answer is `CERULION_FLASHBACK_ANCHOR_MAX_MB`.
///
/// The shape is exact rather than incidental: node 0 opens an anchor and never
/// closes it (one CHUNK record, no FINAL), which holds the whole one-record
/// budget, and node 1's first record then lands against a full ceiling. Nothing
/// completes, so the retention is empty for BOTH readings — which is the point:
/// the two differ only in WHICH absence they name, and that is what the operator
/// acts on.
#[test]
fn a_first_record_refused_at_a_full_ceiling_is_refused_as_the_ceiling_not_an_empty_retention() {
    // Non-FINAL first parts, so neither anchor can ever close.
    let held = anchor(0, 60, 2, 5).remove(0);
    let refused = anchor(1, 60, 2, 5).remove(0);
    // Room for exactly ONE record: node 0's part 0 fills it, and node 1's first
    // record is refused with nothing of its own held.
    let r = refused_capture_with_records("ceilfirst", &[held, refused], STATE_RECORD_SIZE as u64);

    // Same outcome as its sibling above. What this arm adds (and the
    // reason it is its own arm) is one discrimination: this shape's
    // refusal has nothing of its own buffered, so a recorder keyed on that
    // would classify it `no_anchor_retained` (the ARM GATE's answer)
    // rather than as the ceiling. That distinction decides whether the
    // capture is REFUSED at all, so getting it wrong writes a frames-only bag
    // the refusal rule forbids, a strictly louder consequence than the wrong word in
    // a manifest.
    assert!(
        captures(&r.dir).is_empty(),
        "the CEILING refused this run's second anchor outright, so the capture cannot \
         resim and must not be written: {:?}",
        captures(&r.dir)
    );
    assert!(
        r.reason
            .contains("cannot hold one whole checkpoint generation"),
        "…and it must be refused AS the ceiling: {}",
        r.reason
    );

    std::fs::remove_dir_all(&r.dir).ok();
}

/// A TORN anchor is never embedded: it can never be served, so a capture that
/// carried one would spend its ceiling on bytes no resume may use — and would
/// look, to a reader, exactly like a capture that carried a usable checkpoint.
///
/// The stream is torn by SKIPPING a part, which is the shape a lost record
/// really has.
#[test]
fn a_torn_anchor_is_never_embedded() {
    let whole = anchor(0, 50, 4, 6);
    // Parts 0 and 2: part 1 is missing, so the stream tears at part 2.
    let torn = vec![whole[0].clone(), whole[2].clone(), whole[3].clone()];
    let h = capture_with_records("torn", &torn, 64 * 1024 * 1024, true);

    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(false),
        "a torn anchor is refused, not served short"
    );
    assert!(state_records(&h.bag).is_empty());

    std::fs::remove_dir_all(&h.dir).ok();
}

/// THE SECOND CALL SITE: a `--record` recorder's capture carries an anchor too,
/// harvested by the WRITER THREAD.
///
/// The state ring is SPSC, so its single consumer is the drive loop in a
/// window-only recorder and the writer thread once a continuous bag exists. Every
/// arm above drives the FIRST of those; this drives the second, and it is the one
/// that could ship silently inert — the drive-loop half would keep every other
/// arm green while a `--record` robot's captures were permanently frames-only.
///
/// The one-artifact rule is what makes that unacceptable rather than a footnote:
/// `cerulion flashback` means ONE thing, so a `--record` run's capture must be
/// the same artifact a plain run's is.
#[test]
fn a_record_runs_capture_carries_an_anchor_harvested_by_the_writer_thread() {
    let mut newer: Vec<Vec<u8>> = Vec::new();
    newer.extend(anchor(0, 60, 2, 4));
    newer.extend(anchor(1, 60, 1, 8));
    let mut all = anchor(0, 55, 1, 2);
    all.extend(newer.clone());

    let h = capture_with_records("record", &all, 64 * 1024 * 1024, false);

    let got = state_records(&h.bag);
    assert_eq!(
        got, newer,
        "a `--record` recorder's capture must carry the newest checkpoint VERBATIM — the \
         harvest runs on the WRITER thread here, and nothing else in this file drives it"
    );
    let m = flashback_manifest(&h.bag);
    assert_eq!(m["anchor"]["embedded"], serde_json::json!(true));
    assert_eq!(m["anchor"]["step"], serde_json::json!(60));
    assert!(state_coverage(&h.bag).is_some());

    // …and the CONTINUOUS bag still carries EVERY record, which is what proves
    // the harvest did not take them from under it: the retention is a second
    // READER of one drain, never a second CONSUMER of the ring.
    let continuous = h
        .continuous
        .clone()
        .expect("a --record run writes a continuous bag");
    assert_eq!(
        state_records(&continuous),
        all,
        "the continuous recording must still hold every state record"
    );

    std::fs::remove_dir_all(&h.dir).ok();
}

/// Over a REAL ring on the `--record` path: a node the ring DECLARED
/// and that did not anchor at the captured step is NAMED in the capture's
/// coverage.
///
/// `AnchorWindow::select` is satisfied by ONE node's anchor, so a checkpoint is
/// whatever had arrived when the capture closed. A sibling whose records were
/// still in flight — or a rank that died — is simply absent, and a manifest
/// built only from the anchors present described a SMALLER GRAPH than the run
/// has and read as clean coverage of it. `StateCoverage::nodes` documents the
/// opposite contract for itself, and the RUN coverage path has always honoured
/// it; only the capture path did not.
///
/// Driven in `--record` mode ON PURPOSE, which is what makes this an arm about
/// the WIRING rather than about the builder. The state ring is SPSC: once the
/// bag exists the WRITER THREAD owns it, so at `close_capture` — which runs on
/// the drive loop — the ring's manifest is not reachable at all. The declared
/// table can only answer because it was recorded when the ring was OPENED and
/// held in the shared retention, which is precisely the design this pins. A
/// build that read `node_ids()` at capture time would pass every unit arm and
/// report a partial checkpoint as complete on exactly the robots that record.
///
/// The capture NAMES the missing node here and does not ESCALATE, and BOTH are
/// asserted. The recorder gates `NodesWithoutAnchor` on an observed arm word and
/// this harness publishes none, so the un-escalated reading is the correct one
/// — and saying so is what stops the arm being read as having forgotten it.
#[test]
fn a_declared_node_that_did_not_anchor_is_named_in_a_record_runs_capture() {
    // The ring declares `alpha` (index 0) and `beta` (index 1); only alpha
    // anchors, so beta is DUE at this step and absent.
    let records = anchor(0, 70, 2, 6);
    let h = capture_with_records("partial", &records, 64 * 1024 * 1024, false);

    let m = flashback_manifest(&h.bag);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "precondition: the capture really carries the partial checkpoint"
    );
    assert_eq!(
        m["anchor"]["nodes"],
        serde_json::json!(1),
        "precondition: ONE node's anchor is in the checkpoint"
    );

    let cov = state_coverage(&h.bag).expect("a capture with an anchor carries state_coverage.json");
    assert_eq!(
        cov.nodes.len(),
        2,
        "both DECLARED nodes are named — if the absent one is missing from \
         the manifest, nothing in the bag says the checkpoint is partial: {:?}",
        cov.nodes
    );
    let alpha = &cov.nodes["alpha"];
    assert_eq!(alpha.anchors_complete, 1);
    assert_eq!(alpha.node_idx, Some(0));
    let beta = &cov.nodes["beta"];
    assert_eq!(
        beta.anchors_complete, 0,
        "beta was declared by the ring and anchored at no step this capture carries"
    );
    assert_eq!(beta.bytes, 0, "an absent anchor accounts for no state");
    assert_eq!(
        beta.node_idx,
        Some(1),
        "its ring contributed this bag's records, so its index space is the state \
         channel's and the index is a fact a reader may use"
    );
    assert_eq!(
        beta.ring, alpha.ring,
        "one ring declared both, so the rows must agree about which ring that is"
    );
    assert_eq!(
        cov.nodes_without_anchor(),
        1,
        "the escalation term — what makes `is_incomplete` say so on an armed run"
    );
    // …and the OTHER half of that sentence, ASSERTED rather than left to be
    // inferred from its absence. This harness pushes records onto a real state
    // ring but publishes no arm word (by design the recorder arms nothing, it
    // READS the graph's word), so `observed_arm` is None — and the recorder gates
    // `NodesWithoutAnchor` on `armed.is_some()`, deliberately: a recorder that
    // merely drained whatever rings it was handed asked for no cadence and
    // failed at nothing, so it must not escalate a terminal line about a node
    // that was never due to anchor. The capture therefore NAMES the missing node
    // without CLAIMING the run fell short, which is the safe direction of the
    // two. The escalation itself is pinned where an arm word can be supplied —
    // the unit arm
    // `a_declared_node_that_did_not_anchor_is_named_and_makes_the_capture_incomplete`.
    assert!(
        cov.armed.is_none(),
        "precondition: this harness observes no arm word, which is what gates the \
         escalation asserted below"
    );
    assert!(
        !cov.is_incomplete(),
        "an UNARMED recording NAMES the node and does not ESCALATE — the arm-word gate. \
         If this ever fires, the harness has begun observing an arm word and this arm \
         should assert `is_incomplete()` instead of its negation: {:?}",
        cov.incomplete_reasons()
    );

    std::fs::remove_dir_all(&h.dir).ok();
}

/// Over a REAL ring: a checkpoint that lands too late to cover
/// the capture's claimed pre-window is reported as such, not passed off as
/// covering it.
///
/// The unit arm in `flashback_plane.rs` pins the arithmetic deterministically;
/// this pins that the arithmetic is WIRED — that the production `close_capture`
/// really passes the FROZEN floor and really renders the label it gets back.
///
/// # Why the wall cannot invert this arm
///
/// The window is shortened to 1.5 s (the arm is ABOUT the window arithmetic, so
/// the span is the thing under test) and the state records are pushed
/// immediately before the trigger, so the checkpoint is stamped inside the last
/// `post_window` before `T`. Load can only DELAY the drain, which moves the
/// stamp LATER — further into the degraded band, never out of it. The load
/// inversion is therefore structurally impossible here.
///
/// The INVARIANT is asserted as well, and it is wall-free: `fit` and the
/// anchor's own distance from the capture's floor (`taken_at_ns − floor_ns`, both
/// now stated in the manifest) are two renderings of one comparison,
/// so they must agree whatever the run's timing was.
#[test]
fn a_checkpoint_inside_the_post_window_is_labelled_newer_than_the_claimed_window() {
    const WINDOW_MS: u64 = 1_500;

    let mgr = make_manager(64);
    let topic = unique_topic("/fba/late");
    let dir = temp_dir("late");
    let ring_tag = unique_ring_tag("late");

    let mut cfg = BagdConfig::new(unique_out("fba_late"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    cfg.flashback = Some(settings_spanning(
        &dir,
        64 * 1024 * 1024,
        true,
        Duration::from_millis(WINDOW_MS),
    ));
    let ready = unique_ready_file("fba_late");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, 256);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["alpha", "beta"])
        .expect("create the state ring");
    cfg.state_rings = vec![owner.name().to_string()];
    let mut producer = owner.producer().expect("the single producer");

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    await_bagd_ready(&ready, "the checkpoint-inside-the-post-window arm");

    // Frames first, so the window has something to hold and its floor is real.
    //
    // The recorder must have been up for a WHOLE window before the trigger, or
    // `begin_capture`'s `now − span` SATURATES to zero and the claimed start
    // lands at `0 + (span − post)` — an instant every checkpoint of a short run
    // precedes, which makes the degraded arm unreachable for a reason that has
    // nothing to do with the rule. Asserted from the manifest below rather than
    // trusted, so a slow start fails on the PRECONDITION instead of inverting
    // the verdict.
    //
    // ANCHORED ON THE READY-FILE, not on the spawn: the recorder's own window
    // clock begins inside `Recorder::setup`, so a wall started before the
    // sentinel over-counts by however long setup took, and on a loaded runner
    // the precondition below is exactly what gives way (rendezvous rule 1).
    let armed_at = std::time::Instant::now();
    for seq in 0..8u32 {
        pub_.publish_raw(&build_frame(HASH, seq, 3_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }
    // GENEROUS: the recorder's own drive clock starts after its setup (tap
    // attach + schema wait), so this wall is an upper bound on a quantity the
    // test cannot read. Waiting longer only ever helps — it moves the floor off
    // zero — and a start slower than even this fails on the precondition below
    // rather than inverting the verdict.
    while armed_at.elapsed() < Duration::from_millis(WINDOW_MS + 1_500) {
        std::thread::sleep(Duration::from_millis(20));
    }

    // The REQUESTER is opened BEFORE the records are pushed, and that ordering is
    // the whole of this arm's load-safety.
    //
    // What decides the verdict is the gap between the recorder's DRAIN of the
    // state record and the TRIGGER: the checkpoint is stamped at the drain, the
    // capture's floor is frozen at `T − window_span`, so
    // `taken_at_ns − floor_ns = window_span − (T − drain)` and the anchor is
    // `newer_than_the_claimed_window` only while that gap stays under the post
    // window (300 ms here). Anything variable between the push and the trigger is
    // therefore competing with a 300 ms budget — and `open_on_manager` CREATES
    // iceoryx2 services and ports, which is exactly such a cost. MEASURED with it
    // inside the window, under this file's own 8 arms running in parallel: a run
    // landed at 1184 ms against a 1200 ms boundary and INVERTED the verdict — the
    // load-inversion class in this arm.
    //
    // Hoisted out, the gap is one `request` (a loan + a send) plus one drive-loop
    // pass — measured at under a millisecond in isolation, `before` reading 1500
    // on 5 consecutive runs — so the margin is the whole 300 ms, and the two
    // remaining forces both push the SAFE way: load delays the DRAIN, which moves
    // the stamp LATER and `before` HIGHER, further from the boundary.
    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");

    // …and the state LAST, with the trigger immediately after and the drain
    // rendezvous deliberately AFTER it (waiting for the drain first made the gap a
    // load-sensitive variable pushing the WRONG way).
    let records = anchor(0, 70, 1, 6);
    push_records(&mut producer, &records);

    let request_id = requester
        .request(&CaptureRequest::manual("a late anchor"))
        .expect("request");
    let mut finished = false;
    assert!(
        await_condition(DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                    finished = true;
                }
            }
            finished
        }),
        "the recorder must report the capture FINISHED"
    );
    // The records really did leave the ring — asserted AFTER the capture, so the
    // wait cannot widen the gap the arm is about.
    assert!(
        await_condition(DEADLINE, || {
            producer.free_records() == Some(u64::from(RING_RECORDS))
        }),
        "the recorder must drain the state ring"
    );
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(
        handle,
        &shutdown,
        "the checkpoint-inside-the-post-window arm",
    )
    .expect("recorder");
    std::fs::remove_file(&ready).ok();

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1);
    let m = flashback_manifest(&bags[0]);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "precondition: the checkpoint really was retained and embedded"
    );
    // PRECONDITION: the floor did not saturate. `span_ms` is `ended − floor`,
    // and `ended` is `T + post`, so a full window means it reaches at least
    // `window + post`. Below that the claimed start is meaningless and the arm
    // is measuring the harness.
    let span_ms = m["span_ms"].as_u64().expect("span");
    assert!(
        span_ms >= WINDOW_MS + POST_WINDOW_MS,
        "precondition: the recorder must have been up a whole window before the trigger \
         (span {span_ms} ms against a {WINDOW_MS} ms window + {POST_WINDOW_MS} ms post)"
    );
    assert_eq!(
        m["anchor"]["fit"],
        serde_json::json!("newer_than_the_claimed_window"),
        "a checkpoint taken inside the post window covers less than the capture claims. \
         With a deadline read off the CLOSE time this same capture \
         reports `covers_the_claimed_window`: {m}"
    );

    // The WALL-FREE invariant: `fit` and the anchor's own distance from the
    // capture's floor render the same comparison, so they agree however the run
    // was scheduled.
    //
    // This was re-pointed at `taken_at_ns − floor_ns` and it is the same
    // number it always read, now stated by the two fields it is a difference of.
    // `frames_before_anchor_ms` no longer serves: it is measured from what the
    // BAG carries (which is the plain reading of its own name), and this arm's
    // capture deliberately carries NO frames — its 8 pre-window frames age out
    // during the whole-window wait above, so a bag-relative figure is 0 for a
    // reason that has nothing to do with where the anchor sits.
    let taken_at = m["anchor"]["taken_at_ns"]
        .as_u64()
        .expect("an embedded anchor states when it was harvested");
    let floor_ns = m["floor_ns"].as_u64().expect("a floor");
    let before = taken_at.saturating_sub(floor_ns) / 1_000_000;
    let claimed_start_ms = WINDOW_MS - POST_WINDOW_MS;
    assert!(
        before > claimed_start_ms,
        "`newer_than_the_claimed_window` means the anchor sits past the claimed start \
         ({before} ms into a {WINDOW_MS} ms window whose claim begins at {claimed_start_ms} ms)"
    );

    // MARGIN, as a PRECONDITION rather than part of the verdict: with nothing
    // variable left between the push and the trigger the gap is sub-millisecond,
    // so a `before` anywhere near the boundary means a cost has crept back into
    // the window. Failing HERE converts a mystifying verdict inversion into an
    // attributable message. Deliberately generous — two thirds of the post
    // window, so it fires strictly BEFORE the verdict could flip, and the one
    // force load really applies (delaying the DRAIN, which raises `before`)
    // cannot trip it.
    let max_gap_ms = POST_WINDOW_MS * 2 / 3;
    assert!(
        before >= WINDOW_MS - max_gap_ms,
        "the drain-to-trigger gap grew to {} ms (budget {max_gap_ms} ms of a \
         {POST_WINDOW_MS} ms post window): something variable is inside the measured \
         window again — see this arm's ordering comment. \
         taken_at_ns−floor_ns={before} ms, window={WINDOW_MS} ms",
        WINDOW_MS.saturating_sub(before)
    );

    std::fs::remove_dir_all(&dir).ok();
}

/// A deliberately VOIDED node crosses a REAL ring, is retained, and is reported
/// apart from the complete anchors.
///
/// Added alongside the vector arms rather than instead of them: a
/// skip is the one anchor shape whose whole value is what it TELLS a reader, so
/// the claim worth pinning is that it survives the real drain, the real
/// retention and the real capture write — and that the manifest counts it as a
/// node with no state rather than as one with some.
#[test]
fn a_voided_node_is_retained_over_a_real_ring_and_counted_apart() {
    let mut records = anchor(0, 80, 1, 5);
    records.push(
        encode_skip_record(
            RUN,
            80,
            1,
            0,
            SkipCause::RecorderBehind,
            "the recorder has not drained enough of the state ring",
        )
        .to_vec(),
    );

    let h = capture_with_records("skip", &records, 64 * 1024 * 1024, true);

    assert_eq!(
        state_records(&h.bag),
        records,
        "both the complete anchor and the SKIP must reach the bag verbatim"
    );

    let m = flashback_manifest(&h.bag);
    assert_eq!(m["anchor"]["nodes"], serde_json::json!(2));
    assert_eq!(
        m["anchor"]["complete"],
        serde_json::json!(1),
        "a voided node is a node WITHOUT state, and the count that says a capture can be \
         resumed must not include it"
    );

    let cov = state_coverage(&h.bag).expect("state_coverage.json");
    assert_eq!(cov.nodes["alpha"].anchors_complete, 1);
    assert_eq!(cov.nodes["beta"].anchors_skipped, 1);
    assert_eq!(cov.nodes["beta"].anchors_complete, 0);
    assert_eq!(
        cov.nodes["beta"].bytes, 0,
        "a voided node carries no state bytes, over a real ring"
    );
    assert_eq!(
        cov.nodes["alpha"].bytes, 5,
        "…while the complete anchor accounts for its PAYLOAD, not its 512-byte record"
    );

    std::fs::remove_dir_all(&h.dir).ok();
}

/// Over a REAL ring: an EARLY capture — one triggered before a
/// whole window span has elapsed — does not claim coverage from a checkpoint
/// taken after its own trigger.
///
/// # Why this is not the sibling arm above
///
/// `a_checkpoint_inside_the_post_window_...` deliberately WAITS OUT a whole
/// window before triggering, so its floor does NOT saturate — and with an
/// un-saturated floor the old derivation `floor + (span - post)` and the
/// current one `T - post` are IDENTICAL (its unit twin asserts exactly that).
/// So that arm is structurally blind to this defect, and MEASURED: the
/// old derivation, once restored, still passes all eight arms of this file.
///
/// Here the recorder is triggered as early as it can be, so `T - window_span`
/// SATURATES to zero and the two derivations diverge:
///
/// * old derivation (broken): deadline `0 + (1500 - 300)` = 1200 ms after the floor —
///   LATER than the trigger itself, so an anchor taken at the trigger is
///   reported as covering the window BEFORE it.
/// * current derivation (correct): deadline `T - 300 ms`, so that same anchor is DEGRADED.
///
/// The saturation is asserted from the manifest rather than assumed, so a
/// recorder that started slower than expected fails on a PRECONDITION instead of
/// quietly becoming a duplicate of the sibling arm.
#[test]
fn an_early_capture_does_not_claim_coverage_from_an_anchor_taken_after_its_trigger() {
    const WINDOW_MS: u64 = 1_500;

    let mgr = make_manager(64);
    let topic = unique_topic("/fba/early");
    let dir = temp_dir("early");
    let ring_tag = unique_ring_tag("early");

    let mut cfg = BagdConfig::new(unique_out("fba_early"), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.status_period = None;
    cfg.discover_live = false;
    cfg.flashback = Some(settings_spanning(
        &dir,
        64 * 1024 * 1024,
        true,
        Duration::from_millis(WINDOW_MS),
    ));
    let ready = unique_ready_file("fba_early");
    cfg.ready_file = Some(ready.clone());

    let mut pub_ = publisher(&mgr, &topic, 256);
    let mut owner = StateRingOwner::create(&ring_tag, RING_RECORDS, 0, RUN, &["alpha", "beta"])
        .expect("create the state ring");
    cfg.state_rings = vec![owner.name().to_string()];
    let mut producer = owner.producer().expect("the single producer");

    let shutdown = Arc::new(AtomicBool::new(false));
    let rec_mgr = Arc::clone(&mgr);
    let rec_shutdown = Arc::clone(&shutdown);
    let handle = std::thread::spawn(move || run_bagd(rec_mgr, cfg, rec_shutdown));
    // Rule 1. It does NOT cost this arm its earliness — the sentinel lands at
    // the END of setup, i.e. at recorder-clock ~0, so triggering just after it
    // is as early as a request can be made; what it buys is that the four frames
    // below reach a tap that exists.
    await_bagd_ready(&ready, "the early-capture arm");

    // The REQUESTER is opened first, for the reason the sibling arm states: it
    // creates iceoryx2 services, and anything variable between the drain and the
    // trigger competes with the post window's budget.
    let requester = FlashbackRequester::open_on_manager(&mgr).expect("requester");

    // A few frames so the window holds something, then the state, then the
    // trigger — all as EARLY as the recorder will accept them, which is what
    // makes the floor saturate.
    for seq in 0..4u32 {
        pub_.publish_raw(&build_frame(HASH, seq, 4_000 + u64::from(seq), b"pre"))
            .expect("publish");
        std::thread::sleep(Duration::from_millis(10));
    }
    let records = anchor(0, 90, 1, 7);
    push_records(&mut producer, &records);

    let request_id = requester
        .request(&CaptureRequest::manual("an early fault"))
        .expect("request");
    let mut finished = false;
    assert!(
        await_condition(DEADLINE, || {
            for frame in requester.drain_outcomes(request_id) {
                if matches!(frame.outcome, FlashbackOutcome::Finished { .. }) {
                    finished = true;
                }
            }
            finished
        }),
        "the recorder must report the capture FINISHED"
    );
    assert!(
        await_condition(DEADLINE, || {
            producer.free_records() == Some(u64::from(RING_RECORDS))
        }),
        "the recorder must drain the state ring"
    );
    shutdown.store(true, Ordering::Relaxed);
    join_bagd(handle, &shutdown, "the early-capture arm").expect("recorder");
    std::fs::remove_file(&ready).ok();

    let bags = captures(&dir);
    assert_eq!(bags.len(), 1);
    let m = flashback_manifest(&bags[0]);
    assert_eq!(
        m["anchor"]["embedded"],
        serde_json::json!(true),
        "precondition: the checkpoint really was retained and embedded"
    );

    // PRECONDITION: the floor SATURATED. `span_ms` is `ended - floor`, and
    // `ended` is `T + post`; with a saturated floor that is `T + post` measured
    // from instant zero, which is strictly LESS than `window + post`. Above that
    // the floor did not saturate and this arm has silently become its sibling.
    let span_ms = m["span_ms"].as_u64().expect("span");
    assert!(
        span_ms < WINDOW_MS + POST_WINDOW_MS,
        "precondition: the capture must be EARLY enough that its floor saturates \
         (span {span_ms} ms against a {WINDOW_MS} ms window + {POST_WINDOW_MS} ms post)"
    );

    assert_eq!(
        m["anchor"]["fit"],
        serde_json::json!("newer_than_the_claimed_window"),
        "an anchor taken at the trigger of an EARLY capture cannot cover the window \
         before it. Under a `floor + (span - post)` deadline the saturated floor puts the \
         deadline LATER than the trigger, and this same capture reports \
         `covers_the_claimed_window`: {m}"
    );

    std::fs::remove_dir_all(&dir).ok();
}
