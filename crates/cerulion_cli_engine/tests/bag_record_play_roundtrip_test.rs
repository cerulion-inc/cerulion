// SPDX-License-Identifier: AGPL-3.0-only
//! The `bag record` → `bag play` ROUND TRIP over REAL iceoryx2.
//!
//! This is the acceptance test for the pair. `bag record` drives the production
//! `cerulion_bagd` recorder over live local topics; `bag play` reads the bag it
//! wrote and republishes it. The contract under test is that a frame survives
//! the whole loop BYTE-IDENTICALLY:
//!
//! ```text
//!   hand-built frame → publish_raw → SHM → bagd tap → MCAP
//!                    → bag play → SHM → subscriber
//! ```
//!
//! Every hop is checked against a HAND ORACLE recomputed from the frame's own
//! wire `sequence`, so no assertion compares the pipeline to itself.
//!
//! The recorder is a poller with no late-joiner history, so WHICH frames it
//! catches is genuinely non-deterministic (it depends on when its taps armed
//! relative to the publisher). That is real, documented behaviour, so the
//! oracles are shaped around it: the test asserts that everything captured is
//! correct and contiguous, never that a specific count was captured.
//!
//! Parallel-safe: per-test SHM roots + per-test topic names.

#![cfg(unix)]

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_cli_engine::bag_cmd::{self, PlayOptions, RecordOptions};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

const HASH_A: u64 = 0x0BAD_C0DE_0BAD_C0DE;
const HASH_B: u64 = 0x1234_5678_9ABC_DEF0;

/// Runaway guard for the background producer threads — a CAP, never a schedule.
///
/// Each of those threads OWNS the publishers it writes through, so when its loop
/// ends the topics' iceoryx2 services are released. It must therefore outlive
/// the recorder, and the only thing that may end it is the test's `publishing`
/// flag, which is cleared AFTER `bag_record_with_manager` returns.
///
/// A bound of 200 turns, at 5 ms a turn, ends the loop after ~1 s while the
/// recordings it covers block for 600-800 ms — leaving 200-400 ms for the
/// recorder to arm, learn schemas and open its taps. That is a machine-speed
/// assumption rather than a property of the code under test, and it fails on a
/// loaded runner: `--all` selects both live topics and then cannot
/// tap either, reporting `PublishSubscribeOpenError::DoesNotExist` for topics
/// that existed moments earlier, because the producer has already exited and
/// dropped its publishers. The oracles do not depend on it — the bound only stops the
/// publishers from disappearing mid-recording on a slow or loaded runner.
///
/// ~20 s at 5 ms a turn, which covers every recording in this file many times
/// over while still bounding a hang.
const PRODUCER_MAX_TURNS: u32 = 4_000;

fn unique() -> u64 {
    static N: AtomicU64 = AtomicU64::new(0);
    (std::process::id() as u64) << 20 | N.fetch_add(1, Ordering::Relaxed)
}

fn manager(tag: &str, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("playrt_{tag}_{}", unique()),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test")
}

/// The hand oracle: frame `seq` on a topic with `hash` is ALWAYS these bytes.
/// Recomputed at every comparison site from the sequence alone.
fn oracle_frame(hash: u64, seq: u32) -> Vec<u8> {
    // A payload that varies in BOTH content and length with the sequence, so a
    // truncation or an off-by-one frame boundary cannot pass.
    let payload: Vec<u8> = (0..(8 + (seq as usize % 5)))
        .map(|i| (seq as usize * 7 + i) as u8)
        .collect();
    let total = WireHeader::SIZE + payload.len();
    let header = WireHeader {
        schema_hash: hash,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        // Wire stamps 10 ms apart, which is also what `bag play` paces on.
        timestamp_ns: 2_000_000_000 + u64::from(seq) * 10_000_000,
    };
    let mut buf = vec![0u8; total];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(&payload);
    buf
}

/// Read a finalized bag back into `(topic, frame bytes)` in recorded order.
fn read_bag(path: &std::path::Path) -> Vec<(String, Vec<u8>)> {
    let reader = cerulion_bag::BagReader::open(path).expect("open bag");
    let mut walk = reader.user_frames().expect("the bag must be finalized");
    let mut out = Vec::new();
    while let Some((channel_id, span)) = walk.next_user_frame().expect("walk") {
        out.push((
            walk.topic(channel_id).to_string(),
            reader.frame(&span).to_vec(),
        ));
    }
    out
}

/// Assert a captured stream is a CONTIGUOUS run of the oracle: every frame
/// byte-identical to `oracle_frame(hash, seq)` for its own sequence, and the
/// sequences ascending by exactly one with no gap.
///
/// Contiguity is the real loss check — the recorder may start late (it has no
/// history) but must not DROP from the middle without saying so.
fn assert_contiguous_oracle_run(frames: &[Vec<u8>], hash: u64, what: &str) {
    assert!(!frames.is_empty(), "{what}: nothing was captured at all");
    let mut prev: Option<u32> = None;
    for (i, f) in frames.iter().enumerate() {
        let header = WireHeader::read_from_buf(f).unwrap_or_else(|| {
            panic!("{what}: frame {i} has no parseable wire header");
        });
        assert_eq!(
            f,
            &oracle_frame(hash, header.sequence),
            "{what}: frame {i} (wire sequence {}) is not byte-identical to the oracle",
            header.sequence
        );
        if let Some(p) = prev {
            assert_eq!(
                header.sequence,
                p + 1,
                "{what}: a frame was lost between wire sequence {p} and {} — the capture must \
                 be contiguous",
                header.sequence
            );
        }
        prev = Some(header.sequence);
    }
}

/// THE round trip: record live topics, then play the bag back, and prove the
/// frames survived both hops byte-identically.
#[test]
fn recorded_frames_play_back_byte_identical() {
    let id = unique();
    let topic_a = format!("/playrt/a/{id}");
    let topic_b = format!("/playrt/b/{id}");
    let dir = tempfile::tempdir().expect("tempdir");
    let out = dir.path().join("roundtrip.mcap");

    // ---- machine "robot": real publishers on a real SHM root ----------------
    let ix = cerulion_core::testing::iceoryx_test_config();
    let robot = manager("robot", ix.clone());
    let mut pub_a = robot
        .create_publisher(&topic_a, MaxSliceLen::const_new(4096), 0)
        .expect("publisher a");
    let mut pub_b = robot
        .create_publisher(&topic_b, MaxSliceLen::const_new(4096), 0)
        .expect("publisher b");

    // Publish continuously for the whole recording window. The recorder's taps
    // arm inside `bag_record`, so a fixed pre-published burst would be missed
    // entirely (a data-only tap is never sent late-joiner history).
    let publishing = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&publishing);
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while flag.load(Ordering::Relaxed) && seq < 400 {
            let _ = pub_a.publish_raw(&oracle_frame(HASH_A, seq));
            let _ = pub_b.publish_raw(&oracle_frame(HASH_B, seq));
            seq += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
        seq
    });

    // ---- record ------------------------------------------------------------
    let summary = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            topics: vec![topic_a.clone(), topic_b.clone()],
            out: out.clone(),
            duration: Some(Duration::from_millis(1_200)),
            schema_wait: Duration::from_millis(200),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect("record");

    publishing.store(false, Ordering::Relaxed);
    let published = producer.join().expect("producer thread");
    assert!(published > 0, "the producer must have published something");

    assert_eq!(
        summary.bag_paths,
        vec![out.clone()],
        "one file, no rotation"
    );
    assert!(
        summary.messages > 0,
        "the recorder captured nothing in a 1.2s window while a producer ran"
    );
    // The recorder learned the REAL wire hash for each channel from its first
    // frame — that is what makes the bag playable at all.
    let reader = cerulion_bag::BagReader::open(&out).expect("open");
    for (topic, hash) in [(&topic_a, HASH_A), (&topic_b, HASH_B)] {
        let ch = reader
            .channels()
            .expect("channels")
            .into_iter()
            .find(|c| &c.topic == topic)
            .unwrap_or_else(|| panic!("no channel for {topic}"));
        assert_eq!(
            ch.descriptor.expect("cerulion descriptor").schema_hash,
            hash,
            "the recorder must learn {topic}'s real wire schema hash"
        );
    }
    drop(reader);

    // ---- hop 1: what the bag holds is byte-identical to what was published --
    let recorded = read_bag(&out);
    for (topic, hash) in [(&topic_a, HASH_A), (&topic_b, HASH_B)] {
        let frames: Vec<Vec<u8>> = recorded
            .iter()
            .filter(|(t, _)| t == topic)
            .map(|(_, f)| f.clone())
            .collect();
        assert_contiguous_oracle_run(&frames, hash, &format!("recorded {topic}"));
    }

    // ---- hop 2: playing the bag republishes those exact bytes --------------
    // A SEPARATE SHM root, so the played frames cannot be the originals still
    // sitting in the robot's shared memory.
    let desk = manager("desk", cerulion_core::testing::iceoryx_test_config());
    let sub = desk
        .create_subscriber(&topic_a)
        .expect("pre-create so the subscriber is attached before playback");

    let running = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&running);
    let desk2 = Arc::clone(&desk);
    let bag_path = out.clone();
    let player = std::thread::spawn(move || {
        bag_cmd::bag_play_with_manager(
            &desk2,
            &bag_path,
            PlayOptions {
                rate: 1.0,
                // ONE pass, never a loop — see the collection loop below.
                repeat: false,
                topics: Vec::new(),
                ..Default::default()
            },
            flag,
            &mut Vec::new(),
        )
    });

    let recorded_a: Vec<Vec<u8>> = recorded
        .iter()
        .filter(|(t, _)| t == &topic_a)
        .map(|(_, f)| f.clone())
        .collect();
    // COLLECT ONE PASS. The player above runs with `repeat: false`, and that is
    // what makes the contiguity oracle below a sound claim rather than a race the
    // collector has to win.
    //
    // # The race a looping player creates, and why it is not a loss
    //
    // With `repeat: true` the player LAPS the bag. `want` is
    // `min(recorded_a.len(), 20)`, so whenever the bag holds fewer than 20 frames
    // — which under load is the normal case, because a starved producer thread
    // publishes fewer into the same 1.2 s window — `want == recorded_a.len()` and
    // the collector has to catch EVERY frame of one pass with ZERO misses. Miss
    // one and it keeps collecting into the NEXT pass, whose sequences start over,
    // and `assert_contiguous_oracle_run` reports the wrap as "a frame was lost".
    //
    // MEASURED, 20 reps under concurrent build load with the test
    // process demoted to macOS background QoS (`taskpolicy -b`): 17 of 20 FAIL,
    // and every single failure is that wrap — `6 -> 1`, `7 -> 1`, `7 -> 2`,
    // `8 -> 1`. Not one forward gap, i.e. the transport never actually drops
    // anything; the assertion fires on the player's own second lap.
    //
    // One pass makes a BACKWARDS sequence impossible by construction, so the
    // oracle keeps its full strength: a forward gap means a frame really was
    // lost between the player and a co-located subscriber, which is a genuine
    // failure rather than a harness artefact.
    let want = recorded_a.len().min(20);
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got: Vec<Vec<u8>> = Vec::new();
    while got.len() < want && Instant::now() < deadline {
        // SAMPLED BEFORE THE DRAIN, and the ordering is the whole guarantee. The
        // break below needs "the player was already done AND the queue is empty";
        // reading `is_finished()` AFTER the drain proves neither, because the
        // player is free to publish its final frame and exit in the window between
        // the drain returning empty and the flag being read — the loop then breaks
        // with the tail still queued and the exact-count oracle fails a CORRECT
        // run, claiming a frame was lost between the player and a co-located
        // subscriber. Finished-BEFORE plus an empty drain admits no such window:
        // everything the player would ever publish was published before the drain
        // ran, and the drain found nothing.
        //
        // SCOPE: a STRUCTURAL guarantee, not a measured failure. The window has
        // NOT been reproduced. A probe widening it to 20 ms does not fail this arm at
        // its 1.2 s record window — the collector reaches `want` (20) long
        // before a ~240-frame player finishes, so the finished-and-idle break is
        // not the exit that fires — and shrinking the record window to force the
        // small-bag shape produces a DIFFERENT failure ("delivered 0 of N") at a
        // similar rate under BOTH orderings, i.e. an artifact of the shortened
        // probe rather than this race. What the ordering rests on is that
        // finished-BEFORE-plus-empty-drain is a strictly stronger precondition
        // than finished-after, and it is the one the comment below asserts.
        let finished_before = player.is_finished();
        let before = got.len();
        let _ = sub.try_receive(|msg| {
            // CAPPED AT `want` BY CONSTRUCTION. `try_receive` invokes this closure
            // once per QUEUED frame, so a single call can deliver a burst — and an
            // unguarded push then OVERSHOOTS, which is not a harmless surplus: the
            // exact-count oracle below reads `got.len()`, and the overshoot is
            // larger the HEALTHIER the run (a quiet machine records ~240 frames, so
            // the player has a long tail to burst from, while under load the bag
            // is small and `want == recorded_a.len()` leaves nothing to overshoot
            // WITH). MEASURED without the cap: 10 of 12 reps FAIL on a QUIET machine at
            // `22 of 20` / `23 of 20`, while 20 of 20 pass under load — a test that only
            // fails when the machine is healthy.
            if got.len() < want {
                let header = msg.header();
                let payload = msg.payload();
                let mut buf = vec![0u8; WireHeader::SIZE + payload.len()];
                header.write_to_buf(&mut buf[..WireHeader::SIZE]);
                buf[WireHeader::SIZE..].copy_from_slice(payload);
                got.push(buf);
            }
        });
        // A single pass ENDS. Without this the loop would spend the whole 20 s
        // ceiling whenever the subscriber caught fewer than `want` — the player
        // has exhausted the bag and nothing more can arrive. The player being
        // finished is asked FIRST (see `finished_before` above), so a drain that
        // yields nothing only ends the loop once there is provably nothing left
        // to yield.
        if finished_before && got.len() == before {
            break;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    running.store(false, Ordering::Relaxed);
    let play = player.join().expect("player thread").expect("play");

    // THE COUNT IS THE ORACLE; the player finishing is only the RENDEZVOUS.
    //
    // `!got.is_empty()` was the only size check, and the finished-pass break
    // above can end the loop with FEWER than `want` frames in hand — so a
    // playback that lost its LAST frame satisfied every assertion below (a
    // contiguous run of N-1 frames is still contiguous, and every one of them is
    // still in the recorded set). One pass publishes each frame EXACTLY once and
    // the subscriber is attached before it starts, so `want` is not a hope: it is
    // what a lossless single pass delivers, and a short count is loss.
    assert_eq!(
        got.len(),
        want,
        "playing the recorded bag delivered {} of {want} frames on {topic_a} — the player \
         published each of them exactly once (`repeat: false`) into a subscriber that was \
         already attached, so a short count is a frame LOST between the player and a \
         co-located subscriber, not a pass that had not finished",
        got.len()
    );
    // …and the run must start at the bag's OWN FIRST FRAME. Contiguity is a
    // claim about the MIDDLE — `assert_contiguous_oracle_run` deliberately
    // tolerates a late start, because a live recorder has no history and may
    // legitimately begin mid-stream — so a `bag_play` regression that skips
    // the HEAD of the bag produces a shorter run that is still contiguous, still
    // all-oracle, and still entirely within the recorded set. The count cannot
    // see it either: `want` is `min(len, 20)` and a quiet machine records ~240
    // frames, so dropping the first few leaves 20 to collect. PLAYBACK is not a
    // live tap — the bag is a finished file and the player publishes it from the
    // start into an already-attached subscriber — so here the first frame out is
    // a fact, not a hope.
    assert_eq!(
        got[0],
        recorded_a[0],
        "playback started at wire sequence {:?}, but the bag's first frame on \
         {topic_a} is {:?} — the player skipped the HEAD of the bag. Contiguity \
         cannot see this (a short run is still contiguous) and neither can the \
         count (a ~240-frame bag has 20 to spare), which is why the head is \
         anchored separately.",
        WireHeader::read_from_buf(&got[0]).map(|h| h.sequence),
        WireHeader::read_from_buf(&recorded_a[0]).map(|h| h.sequence),
    );
    // Each played frame must be byte-identical to the ORACLE for its own wire
    // sequence — i.e. to what was originally published, not merely to what the
    // bag happens to hold.
    assert_contiguous_oracle_run(&got, HASH_A, "played /a");
    // And every played frame must be one the bag actually contains.
    for f in &got {
        assert!(
            recorded_a.contains(f),
            "a played frame is not in the recorded set: {:?}",
            WireHeader::read_from_buf(f).map(|h| h.sequence)
        );
    }
    assert!(play.total_injected() > 0);
    assert_eq!(
        play.topics.iter().map(|t| t.rejected).sum::<u64>(),
        0,
        "a bag this recorder wrote must never fail the player's own wire validation"
    );
}

/// `--all` records every live local topic and skips Cerulion's own internal
/// channels — the `ros2 bag record -a` behaviour, and the interim for a
/// robot whose graph-declared topic list misses its dynamic routes.
#[test]
fn all_records_every_live_topic_and_skips_internal_channels() {
    let id = unique();
    let topic_a = format!("/playall/a/{id}");
    let topic_b = format!("/playall/b/{id}");
    let dir = tempfile::tempdir().expect("tempdir");
    let out = dir.path().join("all.mcap");

    let robot = manager("all", cerulion_core::testing::iceoryx_test_config());
    let mut pub_a = robot
        .create_publisher(&topic_a, MaxSliceLen::const_new(1024), 0)
        .expect("publisher a");
    let mut pub_b = robot
        .create_publisher(&topic_b, MaxSliceLen::const_new(1024), 0)
        .expect("publisher b");

    let publishing = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&publishing);
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while flag.load(Ordering::Relaxed) && seq < PRODUCER_MAX_TURNS {
            let _ = pub_a.publish_raw(&oracle_frame(HASH_A, seq));
            let _ = pub_b.publish_raw(&oracle_frame(HASH_B, seq));
            seq += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let summary = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            all: true,
            out: out.clone(),
            duration: Some(Duration::from_millis(800)),
            schema_wait: Duration::from_millis(200),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect("record --all");
    publishing.store(false, Ordering::Relaxed);
    producer.join().expect("producer");

    let topics: Vec<&String> = summary.per_topic.keys().collect();
    assert!(
        topics.contains(&&topic_a) && topics.contains(&&topic_b),
        "--all must record every live topic, got {topics:?}"
    );
    assert!(
        !topics
            .iter()
            .any(|t| t.starts_with("/bagd/") || t.contains("__cerulion/")),
        "--all must never auto-select Cerulion's own internal channels, got {topics:?}"
    );
    assert!(summary.messages > 0);
}

/// `--exclude` narrows `--all`, and the excluded topic is absent from the bag
/// entirely (not merely empty).
#[test]
fn exclude_removes_a_topic_from_the_recording() {
    let id = unique();
    let keep = format!("/playex/keep/{id}");
    let drop_it = format!("/playex/drop/{id}");
    let dir = tempfile::tempdir().expect("tempdir");
    let out = dir.path().join("excluded.mcap");

    let robot = manager("excl", cerulion_core::testing::iceoryx_test_config());
    let mut pub_a = robot
        .create_publisher(&keep, MaxSliceLen::const_new(1024), 0)
        .expect("publisher keep");
    let mut pub_b = robot
        .create_publisher(&drop_it, MaxSliceLen::const_new(1024), 0)
        .expect("publisher drop");

    let publishing = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&publishing);
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while flag.load(Ordering::Relaxed) && seq < PRODUCER_MAX_TURNS {
            let _ = pub_a.publish_raw(&oracle_frame(HASH_A, seq));
            let _ = pub_b.publish_raw(&oracle_frame(HASH_B, seq));
            seq += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let summary = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            all: true,
            exclude: vec!["/drop/".to_string()],
            out: out.clone(),
            duration: Some(Duration::from_millis(600)),
            schema_wait: Duration::from_millis(150),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect("record");
    publishing.store(false, Ordering::Relaxed);
    producer.join().expect("producer");

    let topics: Vec<&String> = summary.per_topic.keys().collect();
    assert!(topics.contains(&&keep), "{topics:?}");
    assert!(
        !topics.contains(&&drop_it),
        "the excluded topic must not be a channel in the bag at all: {topics:?}"
    );
    // The bag itself agrees — the exclusion is not merely a summary artifact.
    let reader = cerulion_bag::BagReader::open(&out).expect("open");
    let channels: Vec<String> = reader
        .channels()
        .expect("channels")
        .into_iter()
        .map(|c| c.topic)
        .collect();
    assert!(
        !channels.contains(&drop_it),
        "excluded topic present in the bag's channel table: {channels:?}"
    );
}

/// The decision, end to end: naming a topic that is not LOCAL is a loud
/// refusal that says where it should be recorded — never a network fallback.
#[test]
fn a_non_local_topic_is_refused_and_never_reaches_the_network() {
    let robot = manager("nonlocal", cerulion_core::testing::iceoryx_test_config());
    let dir = tempfile::tempdir().expect("tempdir");
    let err = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            topics: vec!["/some/robot/topic".to_string()],
            out: dir.path().join("never.mcap"),
            duration: Some(Duration::from_millis(100)),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect_err("a non-local topic must be refused")
    .to_string();
    assert!(err.contains("/some/robot/topic"), "{err}");
    assert!(
        err.contains("never pulls a topic's frames across the network"),
        "{err}"
    );
    assert!(err.contains("ON the robot"), "{err}");
    assert!(
        !dir.path().join("never.mcap").exists(),
        "a refused recording must not leave a bag behind"
    );
}

/// Hold data-only taps on `topic` until one fails, and RETURN the holders —
/// the topic's subscriber slots are then exhausted, so the next attach fails
/// while the topic REMAINS visible to `list_topics`.
///
/// This is the realistic un-attachable shape: slots are finite
/// (`INTROSPECTION_SUBSCRIBER_HEADROOM` is 5, shared with the liveness
/// observer, vizd taps and the topic verbs), so a busy robot reaches it. The
/// other real cause — a producer exiting between the scan and the attach —
/// cannot be staged here, because releasing the service also removes the topic
/// from the scan, so the topic never reaches the preflight at all.
fn exhaust_subscriber_slots(
    manager: &Arc<TransportManager>,
    topic: &str,
) -> Vec<cerulion_core::transport::subscriber::DataOnlySubscriber> {
    let mut held = Vec::new();
    // Bounded: if the slots were unbounded the test would hang, so fail loudly.
    for _ in 0..256 {
        match manager.create_data_only_subscriber(topic) {
            Ok(sub) => held.push(sub),
            Err(_) => return held,
        }
    }
    panic!("subscriber slots for {topic} appear unbounded — cannot stage the refusal");
}

/// One un-attachable topic must cost THAT topic, not the whole
/// recording.
///
/// `run_bagd` attaches every tap with `?`, so without the preflight a single
/// topic that could not be tapped failed the ENTIRE run — an operator running
/// `-a` on a busy robot got NO BAG AT ALL rather than a bag missing one topic.
/// `ros2 bag record -a` degrades per-topic.
#[test]
fn an_unattachable_topic_is_excluded_and_the_rest_still_record() {
    let id = unique();
    let alive = format!("/playpf/alive/{id}");
    let doomed = format!("/playpf/doomed/{id}");
    let dir = tempfile::tempdir().expect("tempdir");
    let out = dir.path().join("degraded.mcap");

    let robot = manager("preflight", cerulion_core::testing::iceoryx_test_config());
    let mut pub_alive = robot
        .create_publisher(&alive, MaxSliceLen::const_new(1024), 0)
        .expect("publisher alive");
    let _pub_doomed = robot
        .create_publisher(&doomed, MaxSliceLen::const_new(1024), 0)
        .expect("publisher doomed");

    // Both topics are visible to the scan, and STAY visible…
    let live = robot.list_topics().expect("list");
    assert!(live.contains(&alive) && live.contains(&doomed), "{live:?}");
    // …but one has no subscriber slots left, so its tap cannot attach.
    let _hogs = exhaust_subscriber_slots(&robot, &doomed);
    assert!(
        robot.create_data_only_subscriber(&doomed).is_err(),
        "precondition: the doomed topic must be un-tappable"
    );
    assert!(
        robot.list_topics().expect("list").contains(&doomed),
        "precondition: it must STILL be listed — that is what makes it reach the preflight"
    );

    let publishing = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&publishing);
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while flag.load(Ordering::Relaxed) && seq < PRODUCER_MAX_TURNS {
            let _ = pub_alive.publish_raw(&oracle_frame(HASH_A, seq));
            seq += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let mut banner = Vec::new();
    let summary = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            all: true,
            out: out.clone(),
            duration: Some(Duration::from_millis(600)),
            schema_wait: Duration::from_millis(150),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut banner,
    )
    .expect("the recording must SURVIVE one un-attachable topic");
    publishing.store(false, Ordering::Relaxed);
    producer.join().expect("producer");

    // THE pin: a bag exists, and it holds the live topic.
    let topics: Vec<&String> = summary.record_health.topics.keys().collect();
    assert!(
        topics.contains(&&alive),
        "the attachable topic must still record: {topics:?}"
    );
    assert!(summary.messages > 0, "frames must have been captured");
    assert!(
        !topics.contains(&&doomed),
        "the un-attachable topic must not appear as a recorded channel: {topics:?}"
    );
    // And the excluded one is NAMED, not silently dropped.
    let banner = String::from_utf8(banner).expect("utf8");
    assert!(
        banner.contains("EXCLUDED") && banner.contains(&doomed),
        "the excluded topic must be named to the operator: {banner}"
    );
}

/// The complement: an EXPLICITLY named un-attachable topic is a HARD failure.
/// Silently dropping it would record a bag quietly missing what was asked for.
#[test]
fn an_explicitly_named_unattachable_topic_is_a_hard_failure() {
    let id = unique();
    let doomed = format!("/playpf/explicit/{id}");
    let dir = tempfile::tempdir().expect("tempdir");
    let out = dir.path().join("never.mcap");

    let robot = manager("preflight_x", cerulion_core::testing::iceoryx_test_config());
    let _pub_doomed = robot
        .create_publisher(&doomed, MaxSliceLen::const_new(1024), 0)
        .expect("publisher");
    let _hogs = exhaust_subscriber_slots(&robot, &doomed);

    let err = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            topics: vec![doomed.clone()],
            out: out.clone(),
            duration: Some(Duration::from_millis(200)),
            schema_wait: Duration::from_millis(50),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect_err("an explicitly named un-attachable topic must fail loudly")
    .to_string();
    assert!(err.contains(&doomed), "{err}");
    assert!(err.contains("named explicitly"), "{err}");
    assert!(err.contains("--all"), "the remedy must be named: {err}");
    assert!(
        !out.exists(),
        "a refused recording must leave no bag behind"
    );
}

/// The `record_coverage.json` attachment of a FINALIZED bag — the DURABLE
/// artifact, not the in-memory summary.
///
/// The two are the same value only if `finalize` actually serialised it (a run
/// whose writer was never created returns a summary and writes no attachment at
/// all), so an arm whose name says "manifest" has to read the file. Mirrors
/// `cerulion_bagd`'s own `discovery_e2e_test::read_coverage`.
fn read_coverage(out: &std::path::Path) -> cerulion_bagd::RecordCoverage {
    let reader = cerulion_bag::BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(cerulion_bagd::RECORD_COVERAGE_ATTACHMENT)
        .expect("read attachments")
        .expect("record_coverage.json is present in EVERY finalized bag");
    serde_json::from_slice(&att.data).expect("record_coverage.json parses")
}

/// `bag record` records ITS OWN mirror-provenance verdict into
/// the bag.
///
/// `--all` / `--regex` DERIVE the recorded topic set from the mirror registry —
/// they must, or auto-select would grab another robot's re-injected stream as
/// local — so that selection rests on the same windowed LISTEN that an earlier
/// investigation found can time out having heard nothing. The verb therefore runs the SAME
/// shared retry policy `bagd` does, and its verdict has to reach the durable
/// manifest: the verb sets `discover_live = false`, so the recorder never
/// gathers and would otherwise stamp `None` — NO CLAIM — over a decision that
/// was in fact made on possibly-timed-out evidence.
///
/// This arm pins the WIRING, which nothing else can see: every pure oracle
/// drives `resolve_mirror_snapshot` directly, and every `bag info` arm builds a
/// `RecordCoverage` by hand. Deleting `config.mirrors_established = …` compiles
/// clean and leaves both suites green.
///
/// The desk here has NO registry writer at all, so the gather takes the
/// zero-publisher fast path and settles — which is exactly why `Some(true)` is
/// the right oracle AND why it is a real assertion: that fast path is the
/// majority shipping shape, whether or not any other path can reach
/// `Settled`. `None` here means the verdict never crossed into the bag.
///
/// Two things this arm must do to earn its name. It reads the coverage
/// manifest from the FILE, not `summary.record_coverage` — the IN-MEMORY
/// object — because the manifest is written by a SEPARATE step
/// (`finalize` serialises that value into `RECORD_COVERAGE_ATTACHMENT`, guarded
/// on a writer having been created), so a run that returned a summary and wrote
/// no attachment would otherwise pass; its bagd sibling reads the file the
/// same way. And it drives `--all`, not `--topic`, because the reason the
/// verdict matters is `--all`/`--regex`, whose topic SET is derived from the
/// gather — which is also the path that actually consumes the snapshot.
#[test]
fn bag_record_stamps_its_own_mirror_verdict_into_the_coverage_manifest() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let robot = manager("verdict", ix);
    let tag = unique();
    let topic = format!("/verdict/{tag}");
    let out = std::env::temp_dir().join(format!("verdict_{tag}.mcap"));
    let _ = std::fs::remove_file(&out);

    let mut producer_port = robot
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 0)
        .expect("publisher");

    let publishing = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&publishing);
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while flag.load(Ordering::Relaxed) && seq < 400 {
            let _ = producer_port.publish_raw(&oracle_frame(HASH_A, seq));
            seq += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    // `--all`: the selection this verdict is ABOUT — its topic set is derived
    // from the mirror snapshot, which is why a timed-out gather is dangerous
    // here and why the bag has to record what that gather settled.
    let summary = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            all: true,
            out: out.clone(),
            duration: Some(Duration::from_millis(600)),
            schema_wait: Duration::from_millis(200),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect("record");

    publishing.store(false, Ordering::Relaxed);
    producer.join().expect("producer thread");

    assert!(
        summary.bag_paths.iter().any(|p| p == &out),
        "precondition: the recording really produced the bag this arm reads"
    );

    // Read the DURABLE artifact, not the in-memory summary: they are the same
    // value only if `finalize` actually serialised it.
    let coverage = read_coverage(&out);

    assert_eq!(
        coverage.mirrors_established,
        Some(true),
        "`bag record` derived its topic set from the mirror registry, so the bag must carry the \
         verdict that derivation rested on. `None` means the verb's own gather result never \
         reached `BagdConfig::mirrors_established` and the manifest makes no claim about a \
         decision it did make"
    );
    // The verb runs with discovery OFF, so this is NOT the recorder's own
    // gather speaking — which is the whole reason the caller has to hand its
    // verdict down.
    assert!(
        !coverage.enumerated,
        "precondition: `bag record` does not ask the recorder to enumerate, so the verdict above \
         can only have come from the VERB's gather"
    );
    assert!(
        coverage.tapped.contains_key(&topic),
        "precondition: `--all` really selected the live topic, so the snapshot this verdict \
         describes was genuinely consulted"
    );

    let _ = std::fs::remove_file(&out);
}

/// `bag record` hands its OWN run flag to the shared mirror
/// gather, so a run already asked to stop does not spend the retry budget — and
/// records that it therefore established nothing.
///
/// This test is the ONLY pin on the wiring: reverting it to a hardcoded `|| false`
/// compiles clean and leaves all 1038 lib tests and every other
/// arm in this file green (the same inert-wiring class as
/// the verdict pin above). Every cancellation oracle lives in the PURE
/// `resolve_mirror_snapshot` tests, which pass their own closure and so cannot
/// see what this call site hands it.
///
/// The oracle is the manifest verdict READ FROM THE BAG (as in the arm above,
/// never the in-memory summary). It discriminates cleanly on a desk
/// with no registry writer: cancelled BEFORE the first attempt yields
/// `Some(false)` (nothing was established, because nothing was asked), while
/// the hardcoded `|| false` runs attempt 1, takes the zero-publisher fast path,
/// and yields `Some(true)`. Note the POLARITY this catches too — `running` is
/// true-means-go, the inverse of bagd's shutdown flag, so a copied predicate
/// would invert the verdict on every healthy run.
#[test]
fn bag_record_hands_its_own_run_flag_to_the_mirror_gather() {
    let ix = cerulion_core::testing::iceoryx_test_config();
    let robot = manager("cancel", ix);
    let tag = unique();
    let topic = format!("/cancel/{tag}");
    let out = std::env::temp_dir().join(format!("cancel_{tag}.mcap"));
    let _ = std::fs::remove_file(&out);

    let mut producer_port = robot
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 0)
        .expect("publisher");
    for seq in 0..4u32 {
        let _ = producer_port.publish_raw(&oracle_frame(HASH_A, seq));
    }

    // Already asked to stop, before the verb runs.
    let summary = bag_cmd::bag_record_with_manager(
        Arc::clone(&robot),
        RecordOptions {
            topics: vec![topic.clone()],
            out: out.clone(),
            duration: Some(Duration::from_millis(200)),
            schema_wait: Duration::from_millis(100),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(false)),
        &mut Vec::new(),
    )
    .expect("record");

    assert!(
        summary.bag_paths.iter().any(|p| p == &out),
        "precondition: even a run asked to stop finalizes a bag, so the manifest below exists"
    );
    assert_eq!(
        read_coverage(&out).mirrors_established,
        Some(false),
        "a run asked to stop never asked the registry anything, so it established nothing. \
         Some(true) here means the gather ignored the caller's flag and ran its attempts \
         anyway (the hardcoded `|| false` this arm exists to catch)"
    );

    let _ = std::fs::remove_file(&out);
}
