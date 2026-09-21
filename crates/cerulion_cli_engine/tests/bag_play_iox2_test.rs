// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion bag play` end-to-end over REAL iceoryx2.
//!
//! Every test CRAFTS its bag with `cerulion_bag::BagWriter` from HAND-BUILT
//! wire frames, plays it through the production
//! [`bag_cmd::bag_play_with_manager`] over an isolated per-test SHM root, and
//! compares what a subscriber receives against the hand oracle — never against
//! a second run of the same code (the pre-existing tautology trap).
//!
//! The headline contract is BYTE-VERBATIM republication: a played frame must be
//! bit-for-bit what was recorded, wire `sequence` and `timestamp_ns` included.
//! That is what makes a bag a robot substitute — a player that re-stamped
//! frames would look live while lying about every timestamp a consumer reads.
//!
//! Parallel-safe: per-test SHM roots (`init_for_test`) + per-test topic names,
//! so no `#[serial]` and no `--test-threads=1`.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::bag_cmd::{self, PlayOptions};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::WireHeader;

/// A distinct schema hash per topic so a cross-topic mix-up cannot pass: the
/// player validates every frame against its channel's declared hash.
const HASH_A: u64 = 0x1122_3344_5566_7788;
const HASH_B: u64 = 0x99AA_BBCC_DDEE_FF00;

fn unique() -> u64 {
    static N: AtomicU64 = AtomicU64::new(0);
    // pid + counter: unique across concurrently-running test binaries too.
    (std::process::id() as u64) << 20 | N.fetch_add(1, Ordering::Relaxed)
}

fn manager(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("play_{tag}_{}", unique()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test")
}

/// Build a complete Cerulion wire frame BY HAND — the oracle every assertion
/// compares against. Recomputed from `(hash, seq, stamp, payload)` at every
/// call site, so no test ever compares a frame to itself.
fn frame(hash: u64, seq: u32, stamp_ns: u64, payload: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + payload.len();
    let header = WireHeader {
        schema_hash: hash,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: stamp_ns,
    };
    let mut buf = vec![0u8; total];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(payload);
    buf
}

/// One recorded message: `(topic, frame bytes)`. `log_time` is taken from the
/// frame's own header, exactly as `cerulion_bagd` stamps it.
struct Recorded {
    topic: String,
    bytes: Vec<u8>,
}

/// Write a finalized bag holding `frames`, in the given order.
fn write_bag(dir: &Path, name: &str, topics: &[(&str, u64)], frames: &[Recorded]) -> PathBuf {
    let path = dir.join(name);
    let schemas: Vec<TopicSchema> = topics
        .iter()
        .map(|(t, h)| TopicSchema {
            topic: (*t).to_string(),
            schema_name: "geometry_msgs/Vector3".to_string(),
            schema_hash: *h,
            wire_fixed_size: 24,
        })
        .collect();
    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &schemas).expect("create bag");
    // Payload buffers must outlive `write_chunk` (the zero-copy writev
    // contract), so `frames` is borrowed from outside the closure.
    w.write_chunk(|scope| {
        for f in frames {
            let ts = WireHeader::read_from_buf(&f.bytes)
                .expect("hand-built frames always parse")
                .timestamp_ns;
            let seq = WireHeader::read_from_buf(&f.bytes)
                .expect("parses")
                .sequence;
            scope.write_message(f.topic.as_str(), seq, ts, ts, &[&f.bytes[..]])?;
        }
        Ok(())
    })
    .expect("write chunk");
    w.finalize().expect("finalize");
    path
}

/// Drain everything a subscriber holds, rebuilding each FULL received frame.
///
/// `header()` is an owned parse of all 32 header bytes and `payload()` is the
/// zero-copy slice of everything after them, so their concatenation IS the
/// received frame — comparable byte-for-byte against what was recorded,
/// offset-table bytes included. (Same reconstruction the ingress tests
/// use; `read_from_buf`/`write_to_buf` are total inverses.)
fn drain(sub: &cerulion_core::CerulionSubscriber) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let _ = sub.try_receive(|msg| {
        let header = msg.header();
        let payload = msg.payload();
        let mut buf = vec![0u8; WireHeader::SIZE + payload.len()];
        header.write_to_buf(&mut buf[..WireHeader::SIZE]);
        buf[WireHeader::SIZE..].copy_from_slice(payload);
        out.push(buf);
    });
    out
}

/// Attach a subscriber and collect frames until `want` arrive or `budget`
/// elapses. Returns what actually arrived (the caller asserts the count, so a
/// shortfall fails with the real number rather than hanging).
fn collect(sub: &cerulion_core::CerulionSubscriber, want: usize, budget: Duration) -> Vec<Vec<u8>> {
    let deadline = Instant::now() + budget;
    let mut got = Vec::new();
    while got.len() < want && Instant::now() < deadline {
        got.extend(drain(sub));
        if got.len() < want {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    got.extend(drain(sub));
    got
}

/// Play `bag` LOOPING in a background thread and collect up to `want` frames
/// from `topic`.
///
/// # Why looping, and why a cyclic window
///
/// An iceoryx2 subscriber is CONNECTED to a publisher by the publisher's own
/// `update_connections()`, which runs at send time — so a subscriber that
/// exists before the first `send` can still miss the earliest frames while that
/// connection is established (the "connection warm-up" every delivery test in
/// this repo accounts for). That loss is TRANSPORT behaviour, not the player's,
/// so the two claims are asserted separately:
///
/// - the PLAYER's own contract — it published every frame of every pass — is
///   read off the summary counters, exactly, by the caller;
/// - the BYTE-IDENTITY contract is checked on whatever arrived, against the
///   oracle CYCLE, by [`assert_cyclic_window`]: every received frame must be
///   bit-for-bit its oracle counterpart, in order, with no gap.
///
/// Looping guarantees a non-empty window even when warm-up eats the first
/// frames, which one fast pass cannot.
fn play_looping_and_collect(
    manager: &Arc<TransportManager>,
    bag: &Path,
    topic: &str,
    want: usize,
    rate: f64,
) -> (Vec<Vec<u8>>, bag_cmd::PlaySummary) {
    let sub = manager
        .create_subscriber(topic)
        .expect("pre-create the topic so the subscriber is attached before playback");

    let running = Arc::new(AtomicBool::new(true));
    let mgr = Arc::clone(manager);
    let bag_path = bag.to_path_buf();
    let flag = Arc::clone(&running);
    let handle = std::thread::spawn(move || {
        let mut sink = Vec::new();
        bag_cmd::bag_play_with_manager(
            &mgr,
            &bag_path,
            PlayOptions {
                rate,
                repeat: true,
                ..Default::default()
            },
            flag,
            &mut sink,
        )
    });

    let got = collect(&sub, want, Duration::from_secs(20));
    running.store(false, Ordering::Relaxed);
    let summary = handle.join().expect("player thread").expect("play");
    (got, summary)
}

/// Play `bag` ONCE, synchronously, with no subscriber — for the arms that
/// assert on the player's own accounting rather than on delivery.
fn play_once(
    manager: &Arc<TransportManager>,
    bag: &Path,
    opts: PlayOptions,
) -> bag_cmd::PlaySummary {
    bag_cmd::bag_play_with_manager(
        manager,
        bag,
        opts,
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect("play")
}

/// Assert every received frame is BYTE-IDENTICAL to its counterpart in the
/// oracle cycle, in order, with no gap and no reordering.
///
/// Anchors on the first received frame's position in the cycle (absorbing the
/// transport warm-up loss) and then requires strict cyclic succession — so a
/// dropped, duplicated, reordered or mutated frame anywhere in the stream
/// fails.
fn assert_cyclic_window(got: &[Vec<u8>], oracle: &[Vec<u8>], what: &str) {
    assert!(
        !got.is_empty(),
        "{what}: nothing was received at all — the player published nothing, or the \
         subscriber never connected"
    );
    let n = oracle.len();
    let start = oracle
        .iter()
        .position(|f| f == &got[0])
        .unwrap_or_else(|| panic!("{what}: the first received frame matches NO oracle frame"));
    for (i, frame) in got.iter().enumerate() {
        let expect = &oracle[(start + i) % n];
        assert_eq!(
            frame,
            expect,
            "{what}: frame {i} (cycle position {}) is not byte-identical to the recorded frame",
            (start + i) % n
        );
    }
}

/// The first COMPLETE oracle cycle inside `got`, aligned to oracle position 0 —
/// a run-independent normalisation, so two runs with different warm-up loss are
/// directly comparable.
fn first_full_cycle(got: &[Vec<u8>], oracle: &[Vec<u8>], what: &str) -> Vec<Vec<u8>> {
    let n = oracle.len();
    let start = oracle
        .iter()
        .position(|f| f == &got[0])
        .unwrap_or_else(|| panic!("{what}: unanchored"));
    let offset = (n - start) % n;
    assert!(
        got.len() >= offset + n,
        "{what}: need a full cycle ({n} frames from index {offset}), got {}",
        got.len()
    );
    got[offset..offset + n].to_vec()
}

// ---------------------------------------------------------------------------

/// THE headline pin: what comes out of local SHM is bit-for-bit what went into
/// the bag — wire `sequence` and `timestamp_ns` included — against a hand
/// oracle recomputed from the same `(seq, stamp, payload)` tuples.
///
/// A player that re-stamped or re-sequenced frames would look completely
/// healthy to a subscriber while lying about every timestamp a consumer reads;
/// only a byte compare catches that.
#[test]
fn a_played_bag_republishes_byte_identical_frames() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/verbatim/{}", unique());
    // Deliberately NON-trivial stamps: not zero, not monotone-by-1, and a
    // sequence that does NOT start at 0 — a player that fabricated any of
    // these would still "work" without this oracle.
    let spec: Vec<(u32, u64, Vec<u8>)> = vec![
        (7, 1_000_000_000, vec![0xDE, 0xAD]),
        (8, 1_010_000_000, vec![0xBE, 0xEF, 0x01]),
        (9, 1_020_000_000, vec![0x11]),
    ];
    let recorded: Vec<Recorded> = spec
        .iter()
        .map(|(seq, ts, p)| Recorded {
            topic: topic.clone(),
            bytes: frame(HASH_A, *seq, *ts, p),
        })
        .collect();
    let bag = write_bag(dir.path(), "verbatim.mcap", &[(&topic, HASH_A)], &recorded);

    let mgr = manager("verbatim");
    // Rate 1.0 (the RECORDED pace) deliberately: a delivery oracle must not
    // outrun the polled subscriber. At a large rate the player floods the
    // 16-deep subscriber queue faster than the collector drains it and
    // iceoryx2 evicts (drop_oldest) — transport backpressure, not a player
    // defect, but indistinguishable from one at this assertion. `--rate` is
    // covered by `pacing_changes_when_not_what`.
    let (got, summary) = play_looping_and_collect(&mgr, &bag, &topic, spec.len() * 3, 1.0);

    // The oracle is rebuilt HERE from the spec, never read back from the bag.
    let oracle: Vec<Vec<u8>> = spec
        .iter()
        .map(|(seq, ts, p)| frame(HASH_A, *seq, *ts, p))
        .collect();
    assert_cyclic_window(&got, &oracle, "verbatim playback");
    assert_eq!(
        first_full_cycle(&got, &oracle, "verbatim playback"),
        oracle,
        "a full observed cycle must equal the recorded frames exactly"
    );
    // The PLAYER's own contract: every COMPLETED pass published every frame,
    // and the interrupted final pass contributed at most one more pass's worth.
    // (`passes` counts completed passes, and the run is stopped mid-pass.)
    let n = spec.len() as u64;
    assert!(
        summary.total_injected() >= summary.passes * n
            && summary.total_injected() < (summary.passes + 1) * n + n,
        "published {} frames over {} completed pass(es) of {n}",
        summary.total_injected(),
        summary.passes
    );
    assert_eq!(summary.topics.len(), 1);
    assert_eq!(summary.topics[0].rejected, 0);
    assert_eq!(summary.topics[0].failed, 0);
}

/// The DETERMINISM contract: two plays of one bag publish byte-identical frames
/// in an identical order. The order is the bag's file (recorded) order, which
/// is fixed by immutable bytes on disk, so it needs no sort and cannot tie.
#[test]
fn two_plays_of_one_bag_are_byte_identical() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/det/{}", unique());
    let recorded: Vec<Recorded> = (0..6u32)
        .map(|i| Recorded {
            topic: topic.clone(),
            bytes: frame(
                HASH_A,
                i + 3,
                500_000_000 + u64::from(i) * 7_000_000,
                &[i as u8; 4],
            ),
        })
        .collect();
    let bag = write_bag(dir.path(), "det.mcap", &[(&topic, HASH_A)], &recorded);
    let oracle: Vec<Vec<u8>> = (0..6u32)
        .map(|i| {
            frame(
                HASH_A,
                i + 3,
                500_000_000 + u64::from(i) * 7_000_000,
                &[i as u8; 4],
            )
        })
        .collect();

    let mut cycles = Vec::new();
    for tag in ["det1", "det2"] {
        let mgr = manager(tag);
        // Recorded pace — see the note in the verbatim test.
        let (got, _) = play_looping_and_collect(&mgr, &bag, &topic, oracle.len() * 3, 1.0);
        assert_cyclic_window(&got, &oracle, tag);
        cycles.push(first_full_cycle(&got, &oracle, tag));
    }
    // Anchored to the hand oracle on BOTH sides, so this is not a self-compare:
    // two runs agreeing on the WRONG bytes would fail the first two asserts.
    assert_eq!(cycles[0], oracle, "run 1 must equal the hand oracle");
    assert_eq!(cycles[1], oracle, "run 2 must equal the hand oracle");
    assert_eq!(cycles[0], cycles[1], "two plays must be byte-identical");
}

/// `--loop` replays the same frames again. The wire stamps REGRESS on the wrap
/// (the frames are verbatim, so their timestamps go backwards) — consumers read
/// that as a publisher clock-epoch reset, which is the documented behaviour and
/// is asserted here so it cannot change silently.
#[test]
fn looping_replays_the_same_frames_and_the_stamps_regress() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/loop/{}", unique());
    let recorded: Vec<Recorded> = (0..3u32)
        .map(|i| Recorded {
            topic: topic.clone(),
            bytes: frame(
                HASH_A,
                i,
                900_000_000 + u64::from(i) * 1_000_000,
                &[i as u8],
            ),
        })
        .collect();
    let bag = write_bag(dir.path(), "loop.mcap", &[(&topic, HASH_A)], &recorded);
    let oracle: Vec<Vec<u8>> = (0..3u32)
        .map(|i| {
            frame(
                HASH_A,
                i,
                900_000_000 + u64::from(i) * 1_000_000,
                &[i as u8],
            )
        })
        .collect();

    let mgr = manager("loop");
    let (got, summary) = play_looping_and_collect(&mgr, &bag, &topic, 9, 1.0);
    assert_cyclic_window(&got, &oracle, "looped playback");
    assert!(
        got.len() > oracle.len(),
        "--loop must replay the bag; got only {} frame(s) for a {}-frame bag",
        got.len(),
        oracle.len()
    );
    assert!(summary.passes >= 2, "passes = {}", summary.passes);

    // The wrap is where the stamps go backwards — the epoch-reset
    // class. Find it in what actually arrived and pin the regression.
    let stamps: Vec<u64> = got
        .iter()
        .map(|f| WireHeader::read_from_buf(f).expect("parses").timestamp_ns)
        .collect();
    let regressions = stamps.windows(2).filter(|w| w[1] < w[0]).count();
    assert!(
        regressions >= 1,
        "a --loop wrap must make the wire stamps REGRESS (frames are verbatim): {stamps:?}"
    );
}

/// A topic whose publisher slot is already held is refused BY NAME, and its
/// siblings still play. Silence here would be the worst outcome: the operator
/// would see a viewer showing one topic and never learn why the other is
/// missing.
#[test]
fn an_occupied_topic_is_refused_by_name_while_its_siblings_play() {
    let dir = tempfile::tempdir().expect("tempdir");
    let id = unique();
    let taken = format!("/play/taken/{id}");
    let free = format!("/play/free/{id}");
    let mut recorded = Vec::new();
    for i in 0..3u32 {
        recorded.push(Recorded {
            topic: taken.clone(),
            bytes: frame(HASH_A, i, 1_000 + u64::from(i), &[0xAA]),
        });
        recorded.push(Recorded {
            topic: free.clone(),
            bytes: frame(HASH_B, i, 1_000 + u64::from(i), &[0xBB, i as u8]),
        });
    }
    let bag = write_bag(
        dir.path(),
        "occupied.mcap",
        &[(&taken, HASH_A), (&free, HASH_B)],
        &recorded,
    );

    let mgr = manager("occupied");
    // A LIVE producer takes the slot first.
    let _incumbent = mgr
        .create_publisher(&taken, cerulion_core::wire::MaxSliceLen::const_new(256), 0)
        .expect("incumbent publisher");

    let running = Arc::new(AtomicBool::new(true));
    let mut banner = Vec::new();
    let summary = bag_cmd::bag_play_with_manager(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1_000.0,
            ..Default::default()
        },
        running,
        &mut banner,
    )
    .expect("the sibling topic must still play");

    assert_eq!(summary.refused.len(), 1, "refused: {:?}", summary.refused);
    assert_eq!(summary.refused[0].0, taken);
    assert!(
        summary.refused[0].1.contains("already publishing"),
        "the refusal must name the real cause: {}",
        summary.refused[0].1
    );
    // The refusal is on the operator's screen, not only in the summary struct.
    let banner = String::from_utf8(banner).expect("utf8");
    assert!(banner.contains("REFUSED"), "{banner}");
    assert!(banner.contains(&taken), "{banner}");
    // And the sibling really played, all of it.
    assert_eq!(summary.topics.len(), 1);
    assert_eq!(summary.topics[0].topic, free);
    assert_eq!(summary.topics[0].injected, 3);
}

/// Pacing changes WHEN, never WHAT: the same bag at two rates publishes the
/// same number of frames, and the slower rate genuinely takes longer — by an
/// amount the recorded span predicts.
#[test]
fn pacing_changes_when_not_what() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/pace/{}", unique());
    // 5 frames, 100 ms apart in recorded time = a 400 ms span.
    let recorded: Vec<Recorded> = (0..5u32)
        .map(|i| Recorded {
            topic: topic.clone(),
            bytes: frame(HASH_A, i, u64::from(i) * 100_000_000, &[i as u8]),
        })
        .collect();
    let bag = write_bag(dir.path(), "pace.mcap", &[(&topic, HASH_A)], &recorded);

    let mut walls = Vec::new();
    for (tag, rate) in [("pace_fast", 1_000.0), ("pace_slow", 10.0)] {
        let mgr = manager(tag);
        let summary = play_once(
            &mgr,
            &bag,
            PlayOptions {
                rate,
                ..Default::default()
            },
        );
        // WHAT is rate-invariant: every frame is published at either rate.
        assert_eq!(summary.total_injected(), 5, "{tag}");
        assert_eq!(summary.passes, 1, "{tag}");
        walls.push(summary.elapsed);
    }
    assert!(
        walls[1] > walls[0],
        "rate 10 ({:?}) must take longer than rate 1000 ({:?})",
        walls[1],
        walls[0]
    );
    // The slow leg's floor is ARITHMETIC, not a guess: 400 ms of recorded span
    // at 10x is 40 ms, and the schedule is anchored so it cannot finish sooner.
    assert!(
        walls[1] >= Duration::from_millis(30),
        "rate 10 over a 400ms span must take ~40ms, took {:?}",
        walls[1]
    );
}

/// Playback holds the topics open for its whole run and RELEASES them when the
/// transport goes away — a player that leaked its publisher would keep the
/// single-writer slot and lock out the real producer afterwards.
#[test]
fn teardown_releases_the_played_topics() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/teardown/{}", unique());
    let recorded = vec![Recorded {
        topic: topic.clone(),
        bytes: frame(HASH_A, 0, 42, &[1, 2, 3]),
    }];
    let bag = write_bag(dir.path(), "teardown.mcap", &[(&topic, HASH_A)], &recorded);

    let probe_cfg = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("play_teardown_{}", unique()),
            ..Default::default()
        },
        probe_cfg.clone(),
    )
    .expect("init_for_test");

    let running = Arc::new(AtomicBool::new(true));
    let mut sink = Vec::new();
    let summary = bag_cmd::bag_play_with_manager(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1_000.0,
            ..Default::default()
        },
        running,
        &mut sink,
    )
    .expect("play");
    assert_eq!(summary.total_injected(), 1);
    drop(mgr);

    // A fresh manager on the SAME SHM root must find nothing left.
    let probe = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("play_probe_{}", unique()),
            ..Default::default()
        },
        probe_cfg,
    )
    .expect("probe manager");
    assert!(
        probe.create_subscriber_open_only(&topic).is_err(),
        "the played topic's service must be released once the player's transport is dropped"
    );
}

/// A `--topics` name the bag does not carry is a LOUD error that names it AND
/// lists what the bag does hold — never a silently-empty playback.
#[test]
fn an_unknown_topic_filter_is_refused_and_lists_the_real_set() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/filter/{}", unique());
    let recorded = vec![Recorded {
        topic: topic.clone(),
        bytes: frame(HASH_A, 0, 1, &[9]),
    }];
    let bag = write_bag(dir.path(), "filter.mcap", &[(&topic, HASH_A)], &recorded);

    let mgr = manager("filter");
    let err = bag_cmd::bag_play_with_manager(
        &mgr,
        &bag,
        PlayOptions {
            topics: vec!["/nope".to_string()],
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect_err("an unknown topic must be refused")
    .to_string();
    assert!(err.contains("/nope"), "{err}");
    assert!(err.contains(&topic), "must list what the bag holds: {err}");
}

/// A bag whose recorder never finalized has no summary footer, so its frames
/// cannot be walked. That is refused LOUDLY with the reason and the remedy —
/// not played as an empty run that looks like the bag was empty.
#[test]
fn a_non_finalized_bag_is_refused_with_the_reason() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/torn/{}", unique());
    let path = write_torn_bag(dir.path(), &topic);

    let mgr = manager("torn");
    let err = bag_cmd::bag_play_with_manager(
        &mgr,
        &path,
        PlayOptions::default(),
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect_err("a non-finalized bag must be refused")
    .to_string();
    assert!(
        err.contains("not finalized"),
        "the refusal must name the state: {err}"
    );
    assert!(
        err.contains("bag info"),
        "the refusal must name the diagnostic verb: {err}"
    );
}

/// Write a bag and DROP the writer without finalizing — exactly what a KILLED
/// recorder leaves behind (no summary footer, so the frame walk cannot start).
fn write_torn_bag(dir: &Path, topic: &str) -> PathBuf {
    let path = dir.join("torn.mcap");
    let schemas = vec![TopicSchema {
        topic: topic.to_string(),
        schema_name: "geometry_msgs/Vector3".to_string(),
        schema_hash: HASH_A,
        wire_fixed_size: 24,
    }];
    let bytes = frame(HASH_A, 0, 5, &[7]);
    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &schemas).expect("create bag");
    w.write_chunk(|scope| scope.write_message(topic, 0, 5, 5, &[&bytes[..]]))
        .expect("write");
    // NO finalize: drop the writer, leaving a bag with no footer.
    drop(w);
    path
}

/// ACCEPTANCE: a bag holding TWO clock domains plays each at its
/// OWN recorded rate.
///
/// Domain A and domain B are both 5 frames 50 ms apart — a 200 ms recording —
/// but B's producer is on a clock 5 SECONDS ahead, and their frames interleave
/// in arrival order, which is exactly what a restarted producer, a netd mirror
/// re-injected from another machine, or two independently-started graphs
/// produce.
///
/// With ONE schedule for the whole file every interleave
/// point injects the 5 s offset — capped at `MAX_FRAME_GAP_NS`, itself 5 s —
/// instead of the topic's own 50 ms delta, so this 200 ms recording takes tens
/// of seconds. The discriminator is therefore orders of magnitude wide, not a
/// tight timing assertion: the per-domain path finishes in well under a second.
#[test]
fn a_two_clock_bag_plays_each_channel_at_its_own_recorded_rate() {
    let dir = tempfile::tempdir().expect("tempdir");
    let id = unique();
    let topic_a = format!("/play/dom_a/{id}");
    let topic_b = format!("/play/dom_b/{id}");

    // Interleaved in arrival order, as a recorder would have written them.
    let mut recorded = Vec::new();
    for i in 0..5u32 {
        recorded.push(Recorded {
            topic: topic_a.clone(),
            bytes: frame(HASH_A, i, u64::from(i) * 50_000_000, &[0xAA, i as u8]),
        });
        recorded.push(Recorded {
            topic: topic_b.clone(),
            // Same 50ms cadence, on a clock 5 SECONDS ahead.
            bytes: frame(
                HASH_B,
                i,
                5_000_000_000 + u64::from(i) * 50_000_000,
                &[0xBB, i as u8],
            ),
        });
    }
    let bag = write_bag(
        dir.path(),
        "domains.mcap",
        &[(&topic_a, HASH_A), (&topic_b, HASH_B)],
        &recorded,
    );

    let mgr = manager("domains");
    let started = Instant::now();
    let summary = play_once(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1.0,
            ..Default::default()
        },
    );
    let wall = started.elapsed();

    assert_eq!(
        summary.total_injected(),
        10,
        "every frame must be published"
    );
    // THE discriminator. Recorded span per domain is 200 ms; one file-wide
    // schedule takes ~20 s on this bag (four interleave points x a 5 s capped
    // gap, both directions).
    assert!(
        wall < Duration::from_secs(3),
        "a 200ms two-domain recording must play in ~200ms, took {wall:?} — that is the \
         cross-domain clock offset leaking into the schedule"
    );
    // And the ideal duration the player computed is per-domain, not summed.
    assert!(
        summary.scheduled_span_ns <= 300_000_000,
        "the run's ideal duration must be the longest DOMAIN's span (200ms), got {}ns",
        summary.scheduled_span_ns
    );
}

/// The single-channel common case: one producer, one timeline, and the schedule
/// tracks its recorded cadence.
#[test]
fn a_single_channel_bag_paces_at_its_recorded_rate() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/onedom/{}", unique());
    // 5 frames, 100 ms apart, ONE clock — a 400 ms recording.
    let recorded: Vec<Recorded> = (0..5u32)
        .map(|i| Recorded {
            topic: topic.clone(),
            bytes: frame(HASH_A, i, u64::from(i) * 100_000_000, &[i as u8]),
        })
        .collect();
    let bag = write_bag(dir.path(), "onedom.mcap", &[(&topic, HASH_A)], &recorded);

    let mgr = manager("onedom");
    let started = Instant::now();
    let summary = play_once(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1.0,
            ..Default::default()
        },
    );
    let wall = started.elapsed();
    assert_eq!(summary.total_injected(), 5);
    // `anchor + cumulative`: the anchor is the wall instant the channel's first
    // frame was reached, so the ideal duration is the recorded 400 ms plus that
    // — STRICTLY more than the recording, and never more than the run's own
    // wall. Both bounds are load-proof: a slow machine only makes the anchor and
    // the wall bigger, together. (A fixed 410 ms upper bound would assert
    // that this machine opens a bag in under 10 ms — a machine-speed claim
    // that a loaded runner falsifies.)
    assert!(
        summary.scheduled_span_ns > 400_000_000,
        "ideal duration must be the recorded 400ms plus the anchor offset, got {}",
        summary.scheduled_span_ns
    );
    assert_scheduled_span_is_not_inflated(&summary);
    assert!(
        wall >= Duration::from_millis(380) && wall < Duration::from_secs(3),
        "a 400ms single-domain recording must play in ~400ms, took {wall:?}"
    );
}

/// A HEALTHY run reports NO phantom lag. Before the first-frame anchor, every
/// run reported at least one frame "published behind schedule" — the walk-open
/// cost charged to frame 0 — and advised the operator to lower `--rate`.
///
/// # Why the bag holds exactly ONE frame
///
/// `slipped_frames == 0` is a claim about the PLAYER only for the FIRST frame
/// of a pass. That frame's schedule is seeded with the run origin, taken from
/// the SAME `elapsed` sample the frame is then paced against, so `schedule ==
/// elapsed` and the slip arm computes exactly 0 — at any load, on any machine.
///
/// Every LATER frame is a claim about the MACHINE. The schedule is ANCHORED, so
/// frame `k+1` is counted late exactly when frame `k`'s sleep overshoot plus
/// the loop's own per-frame work exceeds the channel's recorded delta. With
/// four frames 30 ms apart, one descheduling of the
/// player thread past 30 ms earns exactly ONE real slip — anchoring stops it
/// compounding, which is why the count is 1 rather than 3. A loaded runner
/// produces exactly that (`left: 1`); it reproduces on a 16-core machine at load
/// ~100 while an idle machine and a lightly loaded one stay green. Asserting a
/// whole-run zero asserts that this machine never loses a 30 ms slice — not
/// something the player can promise.
///
/// A one-frame bag has no cadence to miss: the whole run IS the frame that is
/// structurally immune, so every assertion below holds under any load, and the
/// unanchored schedule still fails it deterministically (schedule 0 against an
/// `elapsed` that has already advanced through the footer read, the frame walk
/// and the header parse — strictly positive work).
///
/// The multi-frame shape is not dropped. It is asserted on the claims that are
/// properties of the CODE rather than of the machine, in
/// [`a_multi_frame_run_anchors_its_schedule_at_the_first_frame`], which kills
/// strictly more call-site defects than a whole-run zero does.
#[test]
fn a_healthy_run_reports_no_slipped_frames_and_no_overrun() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/healthy/{}", unique());
    // ONE frame — a real shape (a latched one-shot topic), and the only shape
    // whose whole run is structurally immune to the machine's scheduling.
    let recorded = vec![Recorded {
        topic: topic.clone(),
        bytes: frame(HASH_A, 0, 30_000_000, &[0u8]),
    }];
    let bag = write_bag(dir.path(), "healthy.mcap", &[(&topic, HASH_A)], &recorded);

    let mgr = manager("healthy");
    let summary = play_once(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1.0,
            ..Default::default()
        },
    );
    assert_eq!(summary.total_injected(), 1);
    assert_eq!(
        summary.slipped_frames, 0,
        "a healthy run must report ZERO frames behind schedule — the first frame of a pass \
         is first by construction and cannot be late"
    );
    assert_eq!(summary.timeline_restarts, 0, "no restarts in a clean bag");
    assert_eq!(summary.wall_overrun_ns(), None, "and no overrun");
    // The anchor's own fingerprint, and the reason the assertion above holds:
    // the schedule starts at the wall instant the first frame was REACHED, not
    // at time zero. The recorded span of a one-frame bag is 0, so a zero ideal
    // duration IS the unanchored shape. Load can only make this bigger.
    assert!(
        summary.scheduled_span_ns > 0,
        "the schedule must be anchored at the instant the first frame was reached; a zero \
         ideal duration is the unanchored shape that charges the walk-open cost to frame 0"
    );
    assert_scheduled_span_is_not_inflated(&summary);
    let text = bag_cmd::render_play_summary(&summary);
    assert!(
        !text.contains("behind schedule"),
        "the summary must not advise lowering --rate on a healthy run: {text}"
    );
}

/// The MULTI-FRAME half of the healthy-run contract, asserted on what the
/// player controls rather than on what the machine happened to deliver.
///
/// `scheduled_span_ns` is `anchor + Σ(recorded delta / rate)`, so a STRICT
/// `> RECORDED_SPAN_NS` kills three call-site defects at once, and does so
/// LOAD-MONOTONELY — load can only make the anchor larger, never smaller, so
/// there is no runner slow enough to invert it:
///
/// * no run origin at all (the unanchored shape) — the anchor is 0 and the span
///   is EXACTLY the recorded span;
/// * `anchor_ns` never recorded — `unwrap_or(0)` gives the same equality;
/// * `clock.prev_stamp` never threaded back — every frame then looks like a
///   first frame, nothing advances the schedule, and the span collapses to the
///   anchor alone (a walk-open's worth of microseconds against the recorded
///   90 ms).
///
/// The upper bound is the run's own wall: the last frame is published at or
/// after the instant it was due, so a correct run ALWAYS ends at or after its
/// ideal duration. That makes it an exact, load-proof guard against an inflated
/// schedule (a double-counted `advance_ns` would put the ideal duration in the
/// future of a run that had already finished).
///
/// `slipped_frames` is deliberately NOT asserted here — over a multi-frame
/// timed run it is a statement about the machine, and its load-proof pin lives
/// in [`a_healthy_run_reports_no_slipped_frames_and_no_overrun`].
#[test]
fn a_multi_frame_run_anchors_its_schedule_at_the_first_frame() {
    /// Four frames 30 ms apart: three deltas.
    const RECORDED_SPAN_NS: u64 = 3 * 30_000_000;

    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/anchored/{}", unique());
    let recorded: Vec<Recorded> = (0..4u32)
        .map(|i| Recorded {
            topic: topic.clone(),
            bytes: frame(HASH_A, i, u64::from(i) * 30_000_000, &[i as u8]),
        })
        .collect();
    let bag = write_bag(dir.path(), "anchored.mcap", &[(&topic, HASH_A)], &recorded);

    let mgr = manager("anchored");
    let summary = play_once(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1.0,
            ..Default::default()
        },
    );
    assert_eq!(summary.total_injected(), 4);
    assert_eq!(summary.timeline_restarts, 0, "no restarts in a clean bag");
    assert!(
        summary.scheduled_span_ns > RECORDED_SPAN_NS,
        "the ideal duration must be the recorded {RECORDED_SPAN_NS}ns PLUS the anchor; got \
         {} — equal to the recording means no anchor, far below it means the per-channel \
         stamp was never threaded back",
        summary.scheduled_span_ns
    );
    assert_scheduled_span_is_not_inflated(&summary);
}

/// A run cannot finish BEFORE its last frame was due: every frame publishes at
/// or after its scheduled instant, and the wall is read after the last publish.
///
/// So `scheduled_span_ns <= elapsed` holds on every correct run at any load,
/// while a schedule inflated by a double-counted `advance_ns` puts the ideal
/// duration in the future of a run that has already returned.
fn assert_scheduled_span_is_not_inflated(summary: &bag_cmd::PlaySummary) {
    let wall_ns = summary.elapsed.as_nanos().min(u128::from(u64::MAX)) as u64;
    assert!(
        summary.scheduled_span_ns <= wall_ns,
        "the ideal duration ({}ns) cannot exceed the run's own wall ({wall_ns}ns) — the last \
         frame publishes at or after the instant it was due, so this is an inflated schedule",
        summary.scheduled_span_ns
    );
}

/// `--loop` must not fabricate a slipped frame per wrap. A wrap CLEARS every
/// channel clock and the run origin, so the new pass is structurally identical
/// to pass 1 — its first frame is anchored at the instant it was reached, not
/// charged the whole elapsed run — and the wrap itself is counted as the one
/// timeline restart it is (the bag's clock really does go backwards).
///
/// # Why the bag holds exactly ONE frame
///
/// Same reason as [`a_healthy_run_reports_no_slipped_frames_and_no_overrun`]:
/// only the FIRST frame of a pass is structurally immune to the machine's
/// scheduling, so a bag whose every played frame IS a pass's first frame is the
/// shape that makes `slipped_frames == 0` a claim about the player rather than
/// about the machine. With three frames 20 ms apart this test would count a real
/// slip whenever the runner loses a 20 ms slice (`left: 2` on a loaded runner,
/// reproducible on a 16-core machine at load ~100).
///
/// It is also the SHARPEST shape for the wrap: dropping the `run_origin_ns`
/// reset makes every pass after the first inherit pass 1's origin, so each of
/// their frames is charged the whole elapsed run and `slipped_frames` becomes
/// `passes - 1` — a deterministic kill at any load.
#[test]
fn a_loop_wrap_is_a_counted_restart_not_a_phantom_slip() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/play/wrap/{}", unique());
    let recorded = vec![Recorded {
        topic: topic.clone(),
        bytes: frame(HASH_A, 0, 20_000_000, &[0u8]),
    }];
    let bag = write_bag(dir.path(), "wrap.mcap", &[(&topic, HASH_A)], &recorded);

    let mgr = manager("wrap");
    let (_got, summary) = play_looping_and_collect(&mgr, &bag, &topic, 4, 1.0);
    assert!(summary.passes >= 2, "passes = {}", summary.passes);
    assert_eq!(
        summary.slipped_frames, 0,
        "a wrap must not be reported as the machine falling behind"
    );
    // Each completed wrap IS a timeline restart, and is counted as one.
    assert!(
        summary.timeline_restarts >= summary.passes - 1,
        "every wrap must be counted as a restart: {} restarts over {} passes",
        summary.timeline_restarts,
        summary.passes
    );
}

/// A restart on a FILTERED OR REFUSED channel is still
/// counted, and still re-anchors the domain it belongs to.
///
/// A skipped path that calls `pace_step` with a HARDCODED elapsed of 0 and
/// throws the result away has two consequences the operator feels: the re-anchor is
/// inert (`max(schedule, 0)` is always the schedule), so the phantom-lag defect
/// stays live whenever the regressing channel is the skipped one; and
/// `timeline_restarts` never counts it, so the summary offers no explanation
/// for the timing anomaly it causes.
///
/// Shape: A and B share ONE domain (B's regression is against ITSELF, not
/// across channels, so nothing splits them), `--topics` selects A only, and B's
/// producer restarts mid-bag.
#[test]
fn a_restart_on_a_filtered_channel_is_still_counted() {
    let dir = tempfile::tempdir().expect("tempdir");
    let id = unique();
    let played = format!("/play/kept/{id}");
    let filtered = format!("/play/skipped/{id}");

    let mut recorded = Vec::new();
    for (i, ts) in [0u64, 20_000_000, 40_000_000].iter().enumerate() {
        recorded.push(Recorded {
            topic: played.clone(),
            bytes: frame(HASH_A, i as u32, *ts, &[0xAA, i as u8]),
        });
    }
    // B's own frames, then B REGRESSES against itself — a producer restart.
    for (i, ts) in [60_000_000u64, 80_000_000, 1_000_000].iter().enumerate() {
        recorded.push(Recorded {
            topic: filtered.clone(),
            bytes: frame(HASH_B, i as u32, *ts, &[0xBB, i as u8]),
        });
    }
    recorded.push(Recorded {
        topic: played.clone(),
        bytes: frame(HASH_A, 3, 21_000_000, &[0xAA, 3]),
    });
    let bag = write_bag(
        dir.path(),
        "filtered_restart.mcap",
        &[(&played, HASH_A), (&filtered, HASH_B)],
        &recorded,
    );

    let mgr = manager("filtered_restart");
    let summary = play_once(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1.0,
            topics: vec![played.clone()],
            ..Default::default()
        },
    );
    // Only the selected topic played.
    assert_eq!(summary.topics.len(), 1);
    assert_eq!(summary.topics[0].topic, played);
    assert_eq!(summary.total_injected(), 4);
    // THE pin: the skipped channel's restart is visible to the operator.
    assert!(
        summary.timeline_restarts >= 1,
        "a restart on a filtered-out channel must still be COUNTED — it distorts the shared \
         domain's schedule either way, and the operator needs the counter that explains it"
    );
    let text = bag_cmd::render_play_summary(&summary);
    assert!(text.contains("stepped BACKWARDS"), "{text}");
}

/// Write a bag the way `cerulion_bagd` ACTUALLY writes one: TAP-GROUPED per
/// flush batch.
///
/// Both bagd write paths group by tap inside a chunk — inline
/// `for (tap_idx, tap) in taps.iter().enumerate() { for sample in &tap.held }`
/// (`cerulion_bagd/src/lib.rs:3211`) and threaded
/// `for (idx, samples) in batch.per_tap.iter().enumerate()` (`:1868`) — and
/// `tap.held` accumulates across drain passes until the flush interval (100 ms)
/// or the held budget. So at any real rate each tap contributes SEVERAL frames
/// per batch, and file order is per-tap arrival order, NOT global arrival order.
///
/// Every crafted bag elsewhere in this file writes ROUND-ROBIN, which is the one
/// ordering bagd never produces. That blindness is what this helper exists to
/// remove.
fn write_tap_grouped_bag(
    dir: &Path,
    name: &str,
    topics: &[(&str, u64)],
    frames_per_topic: usize,
    period_ns: u64,
    per_batch: usize,
) -> PathBuf {
    let path = dir.join(name);
    let schemas: Vec<TopicSchema> = topics
        .iter()
        .map(|(t, h)| TopicSchema {
            topic: (*t).to_string(),
            schema_name: "geometry_msgs/Vector3".to_string(),
            schema_hash: *h,
            wire_fixed_size: 24,
        })
        .collect();

    // Build the batches: batch b holds, for EACH topic in turn, the frames that
    // topic produced during that flush window.
    let mut batches: Vec<Vec<Recorded>> = Vec::new();
    let mut i = 0;
    while i < frames_per_topic {
        let mut batch = Vec::new();
        for (topic, hash) in topics {
            for k in 0..per_batch {
                let idx = i + k;
                if idx >= frames_per_topic {
                    break;
                }
                batch.push(Recorded {
                    topic: (*topic).to_string(),
                    bytes: frame(*hash, idx as u32, idx as u64 * period_ns, &[idx as u8]),
                });
            }
        }
        batches.push(batch);
        i += per_batch;
    }

    let mut w = BagWriter::create(&path, BagWriterConfig::default(), &schemas).expect("create bag");
    for batch in &batches {
        w.write_chunk(|scope| {
            for f in batch {
                let h = WireHeader::read_from_buf(&f.bytes).expect("parses");
                scope.write_message(
                    f.topic.as_str(),
                    h.sequence,
                    h.timestamp_ns,
                    h.timestamp_ns,
                    &[&f.bytes[..]],
                )?;
            }
            Ok(())
        })
        .expect("write chunk");
    }
    w.finalize().expect("finalize");
    path
}

/// A bag written the way bagd writes one — TAP-GROUPED, 3 topics, one
/// clock — must play at its RECORDED duration.
///
/// The earlier domain inference read every tap boundary as a backwards stamp and so
/// as a clock conflict, greedily 2-colouring the tap cycle into phantom
/// "independent CLOCK DOMAINS" on a single-clock bag. Each phantom domain's
/// schedule then re-anchored on the drop and re-climbed the NEXT tap's whole
/// within-batch span, so a K-tap domain played ~K times slower than recorded.
///
/// 3 topics x 60 frames at 10 ms = a 590 ms recording. The phantom-domain mechanism
/// stretches it by roughly the tap multiplicity; the assertion below is
/// deliberately generous so it fails on the defect and not on jitter.
#[test]
fn a_tap_grouped_bag_plays_at_its_recorded_duration() {
    let dir = tempfile::tempdir().expect("tempdir");
    let id = unique();
    let a = format!("/playtg/a/{id}");
    let b = format!("/playtg/b/{id}");
    let c = format!("/playtg/c/{id}");
    const FRAMES: usize = 60;
    const PERIOD_NS: u64 = 10_000_000; // 100 Hz
                                       // 100 Hz against a 100 ms flush window = 10 frames per tap per batch.
    let bag = write_tap_grouped_bag(
        dir.path(),
        "tapgrouped.mcap",
        &[(&a, HASH_A), (&b, HASH_B), (&c, HASH_A)],
        FRAMES,
        PERIOD_NS,
        10,
    );

    let recorded_span = Duration::from_nanos((FRAMES as u64 - 1) * PERIOD_NS);

    let mgr = manager("tapgrouped");
    let started = Instant::now();
    let summary = play_once(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1.0,
            ..Default::default()
        },
    );
    let wall = started.elapsed();

    assert_eq!(
        summary.total_injected(),
        (FRAMES * 3) as u64,
        "every frame must publish"
    );
    // THE pin, BOTH sides. An upper bound alone is satisfied by an INSTANT
    // DUMP — the backlog-dump failure mode — so the lower bound is what proves the
    // frames were actually paced rather than flushed.
    assert!(
        wall >= recorded_span.mul_f64(0.9),
        "a {recorded_span:?} recording must take about that long, took only {wall:?} — the \
         frames were DUMPED, not paced"
    );
    assert!(
        wall < recorded_span.mul_f64(1.5),
        "a {recorded_span:?} recording written the way bagd writes one must play in about \
         {recorded_span:?}, took {wall:?} — that is tap-grouped write batching being read as a \
         clock break"
    );
    // And a healthy single-clock bag must not accuse the MACHINE of lag. Raw
    // slip IS expected here and is structural: a tap-grouped bag is not in
    // global arrival order, so a later topic's frames are reached after the
    // earlier topics' have been paced — bounded by the recorder's flush window.
    // The discriminator is the wall OVERRUN, which must be absent.
    assert_eq!(
        summary.wall_overrun_ns(),
        None,
        "a tap-grouped single-clock bag must not overrun; scheduled {}ms vs wall {:?}",
        summary.scheduled_span_ns / 1_000_000,
        summary.elapsed
    );
    let text = bag_cmd::render_play_summary(&summary);
    if summary.slipped_frames > 0 {
        assert!(
            text.contains("STRUCTURAL, not lag on this machine"),
            "structural slip must NOT be reported as machine lag: {text}"
        );
        assert!(
            !text.contains("Lower --rate"),
            "and must not advise lowering --rate: {text}"
        );
    }
    assert_eq!(
        summary.timeline_restarts, 0,
        "write batching is not a timeline restart"
    );
    // MEASURED: the rate bound DOES bite here — the
    // later taps start behind and catch up — so demanding no fast-forward would
    // be wrong. What must hold is that the report never accuses the RECORDING:
    // it names both causes and stays an observation about this run.
    let text = bag_cmd::render_play_summary(&summary);
    if text.contains("fast-forward") {
        assert!(
            text.contains("not the recording"),
            "the fast-forward note must describe THIS RUN, not the recording: {text}"
        );
        assert!(
            text.contains("first flush was large"),
            "and must name write batching as a cause, not only a handover: {text}"
        );
    }
    // Whatever it reports, a healthy bag must not be told to lower --rate.
    assert!(!text.contains("Lower --rate"), "{text}");
}

/// Pins `bag info`: its damage-classification branch and its not-found /
/// unreadable paths.
#[test]
fn bag_info_reports_contents_and_classifies_damage() {
    let dir = tempfile::tempdir().expect("tempdir");
    let topic = format!("/playinfo/{}", unique());

    // A missing bag is refused by name, before anything else.
    let err = bag_cmd::bag_info(&dir.path().join("nope.mcap"), None)
        .expect_err("a missing bag must be refused")
        .to_string();
    assert!(err.contains("does not exist"), "{err}");

    // A FINALIZED bag renders its contents.
    let recorded: Vec<Recorded> = (0..4u32)
        .map(|i| Recorded {
            topic: topic.clone(),
            bytes: frame(HASH_A, i, u64::from(i) * 25_000_000, &[i as u8]),
        })
        .collect();
    let good = write_bag(dir.path(), "good.mcap", &[(&topic, HASH_A)], &recorded);
    let text = bag_cmd::bag_info(&good, None).expect("info");
    assert!(text.contains("state: finalized"), "{text}");
    assert!(text.contains(&topic), "{text}");
    assert!(text.contains("frames: 4 across 1 topic(s)"), "{text}");
    // 4 frames 25ms apart = a 75ms span.
    assert!(text.contains("span 0.075s"), "{text}");
    assert!(
        !text.contains("was never finalized"),
        "a healthy bag must not be flagged as damaged: {text}"
    );

    // A NON-finalized bag is CLASSIFIED, not merely rejected — that is the
    // branch `bag info` exists for, and `bag play` refuses these outright.
    let torn = write_torn_bag(dir.path(), &topic);
    let text = bag_cmd::bag_info(&torn, None).expect("info on a torn bag must still render");
    assert!(
        text.contains("NOT FINALIZED") || text.contains("TRUNCATED") || text.contains("TORN"),
        "the damage must be classified: {text}"
    );
    assert!(
        text.contains("was never finalized"),
        "and the operator told what it means: {text}"
    );
    assert!(
        text.contains("bag play` will refuse it"),
        "and what it costs them: {text}"
    );
}

/// A channel that STARTS LATE in the
/// recording must play at its recorded rate, not dump its backlog.
///
/// If every channel's schedule starts at the shared run origin, a channel
/// first REACHED at wall T carries a permanent deficit of `T - origin`. For the
/// tap-grouping case that deficit is genuine catch-up and must be carried. For a
/// channel that genuinely begins partway through the recording it is
/// catastrophic: MEASURED without the cap, a handover at the halfway point of a
/// 6 s bag flushes the second producer's 3 s of data in 0.6 ms (about 500 kHz)
/// while the summary prints "Every topic still plays at its recorded rate."
///
/// Shape: producer A owns the first half, producer B the second, with NO
/// overlap — exactly what a handover looks like in a bag.
#[test]
fn a_channel_that_starts_late_plays_at_its_rate_rather_than_dumping() {
    let dir = tempfile::tempdir().expect("tempdir");
    let id = unique();
    let early = format!("/playho/early/{id}");
    let late = format!("/playho/late/{id}");
    const HALF: usize = 40;
    const PERIOD_NS: u64 = 10_000_000; // 100 Hz

    let mut recorded = Vec::new();
    for i in 0..HALF {
        recorded.push(Recorded {
            topic: early.clone(),
            bytes: frame(HASH_A, i as u32, i as u64 * PERIOD_NS, &[0xAA]),
        });
    }
    for i in 0..HALF {
        let ts = (HALF + i) as u64 * PERIOD_NS;
        recorded.push(Recorded {
            topic: late.clone(),
            bytes: frame(HASH_B, i as u32, ts, &[0xBB]),
        });
    }
    let bag = write_bag(
        dir.path(),
        "handoff.mcap",
        &[(&early, HASH_A), (&late, HASH_B)],
        &recorded,
    );

    // Attach to the LATE channel before playback so its delivery is observable.
    let mgr = manager("handoff");
    let sub = mgr
        .create_subscriber(&late)
        .expect("pre-create the late topic");

    let running = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&running);
    let mgr2 = Arc::clone(&mgr);
    let bag_path = bag.clone();
    let player = std::thread::spawn(move || {
        bag_cmd::bag_play_with_manager(
            &mgr2,
            &bag_path,
            PlayOptions {
                rate: 1.0,
                ..Default::default()
            },
            flag,
            &mut Vec::new(),
        )
    });

    // Sample the late channel's arrival spread. A DUMP delivers everything in
    // one poll; a paced stream spreads across many.
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut polls_with_data = 0usize;
    let mut total = 0usize;
    let mut late_started = Instant::now();
    let mut seen_first = false;
    while total < HALF && Instant::now() < deadline {
        let got = drain(&sub);
        if !got.is_empty() {
            if !seen_first {
                seen_first = true;
                late_started = Instant::now();
            }
            polls_with_data += 1;
            total += got.len();
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    running.store(false, Ordering::Relaxed);
    let summary = player.join().expect("player").expect("play");

    assert!(total > 0, "the late channel delivered nothing");
    // THE pin: the late channel's HALF second of data must arrive spread over
    // many polls, not flushed in one. At 100 Hz over a 5 ms poll it is ~2 frames
    // per poll, so a paced stream needs many polls; a dump needs one.
    assert!(
        polls_with_data >= 5,
        "the late-starting channel's {total} frame(s) arrived in only {polls_with_data} poll(s) \
         — that is a BACKLOG DUMP, not bounded fast-forward"
    );
    // The catch-up is BOUNDED, so it cannot be instant — but the floor must be
    // derived from what the mechanism actually GUARANTEES, not from the content
    // length.
    //
    // A floor derived without the free window over-claims. The late channel
    // holds HALF a 100 Hz recording (40 frames,
    // 400ms of content), but `CATCHUP_FREE_DELTAS` = 16 means its final ~16
    // frames are within the free window and publish immediately. Only the
    // frames deeper than that are spaced at `PERIOD/CATCHUP_FACTOR` = 2.5ms, so
    // the guaranteed floor is about 24 x 2.5ms = 60ms, not 100ms. An 80ms
    // assertion passes only on ~20ms of debug-build overhead and would FAIL on a
    // faster machine (release build, quiet runner) — a latent flake, not a
    // defect.
    //
    // 55ms sits just under the derived 60ms so ordinary jitter cannot trip it,
    // while still failing decisively against a dump (which is sub-millisecond).
    let late_wall = late_started.elapsed();
    assert!(
        late_wall >= Duration::from_millis(55),
        "the fast-forward must be RATE-BOUNDED (~60ms guaranteed for the frames outside the \
         free window), took only {late_wall:?}"
    );
    // And the player must SAY it capped a late start rather than claiming
    // everything played at its recorded rate.
    assert_eq!(
        summary.catchup_channels, 1,
        "the fast-forwarding channel must be counted"
    );
    let text = bag_cmd::render_play_summary(&summary);
    assert!(text.contains("fast-forward"), "{text}");
    // It must describe THIS RUN, never make a claim about the recording.
    assert!(text.contains("not the recording"), "{text}");
}

/// A `--loop` wrap must not destroy the pacing structure.
///
/// Every channel regresses at the wrap, and re-anchoring each to `elapsed`
/// COLLAPSES the per-channel deficits that make pass 1 finish on time: from pass
/// 2 the first-batch tap groups then pace SEQUENTIALLY. MEASURED on an 8-tap
/// 100 Hz bag: 590 ms for pass 1, 1220 ms for every later pass (2.07x), with the
/// overrun accumulating without bound.
///
/// The existing loop arm is single-topic and therefore structurally blind: with
/// one channel there are no deficits to destroy.
#[test]
fn a_multi_topic_loop_wrap_keeps_every_pass_the_same_length() {
    let dir = tempfile::tempdir().expect("tempdir");
    let id = unique();
    let a = format!("/playlw/a/{id}");
    let b = format!("/playlw/b/{id}");
    let c = format!("/playlw/c/{id}");
    const FRAMES: usize = 30;
    const PERIOD_NS: u64 = 10_000_000;
    let bag = write_tap_grouped_bag(
        dir.path(),
        "loopwrap.mcap",
        &[(&a, HASH_A), (&b, HASH_B), (&c, HASH_A)],
        FRAMES,
        PERIOD_NS,
        10,
    );
    let recorded_span = Duration::from_nanos((FRAMES as u64 - 1) * PERIOD_NS);

    let mgr = manager("loopwrap");
    let running = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&running);
    let mgr2 = Arc::clone(&mgr);
    let bag_path = bag.clone();
    let player = std::thread::spawn(move || {
        bag_cmd::bag_play_with_manager(
            &mgr2,
            &bag_path,
            PlayOptions {
                rate: 1.0,
                repeat: true,
                ..Default::default()
            },
            flag,
            &mut Vec::new(),
        )
    });

    // Let it run several passes, then stop and check the per-pass wall.
    std::thread::sleep(recorded_span * 4);
    running.store(false, Ordering::Relaxed);
    let summary = player.join().expect("player").expect("play");

    assert!(
        summary.passes >= 2,
        "need at least two passes to compare; got {}",
        summary.passes
    );
    let per_pass = summary.elapsed.as_secs_f64() / summary.passes as f64;
    let recorded = recorded_span.as_secs_f64();
    // THE pin: every pass must cost about one recorded span. Collapsing the
    // per-channel deficits at the wrap makes pass 2+ cost about K times that.
    assert!(
        per_pass >= recorded * 0.9,
        "each --loop pass must cost about the recorded {recorded:.3}s, but {} pass(es) took \
         {:.3}s = {per_pass:.3}s each — the frames were DUMPED, not paced",
        summary.passes,
        summary.elapsed.as_secs_f64()
    );
    assert!(
        per_pass < recorded * 1.6,
        "each --loop pass must cost about the recorded {recorded:.3}s, but {} pass(es) took \
         {:.3}s = {per_pass:.3}s each — the wrap destroyed the pacing structure",
        summary.passes,
        summary.elapsed.as_secs_f64()
    );
    // And the run must not accumulate a phantom overrun across wraps.
    assert_eq!(
        summary.wall_overrun_ns(),
        None,
        "a looping healthy run must never accumulate an overrun"
    );
    // The fast-forward figure is a CHANNEL count, not an event count —
    // it must not grow with the number of passes. This bag has 3 channels, so
    // whatever it reports it can never exceed that however long the loop runs.
    assert!(
        summary.catchup_channels <= 3,
        "the fast-forward figure must be a CHANNEL count (<= 3 here), not an event count \
         accumulated across {} passes; got {}",
        summary.passes,
        summary.catchup_channels
    );
}

/// Only PLAYED channels may define the run's ideal duration.
///
/// A filtered (`--topics`) or refused channel's clock still advances — it has to,
/// so the rest keep their relative pacing — but counting it in
/// `scheduled_span_ns` let `play --topics /small` out of a long bag report the
/// LONG channel's duration as the ideal. A genuinely lagging run then showed no
/// overrun and was told "no action needed".
#[test]
fn only_played_channels_define_the_ideal_duration() {
    let dir = tempfile::tempdir().expect("tempdir");
    let id = unique();
    let small = format!("/playm1/small/{id}");
    let long = format!("/playm1/long/{id}");

    // `small` spans 30ms; `long` spans 2s. Interleaved so both clocks advance.
    let mut recorded = Vec::new();
    for i in 0..4u32 {
        recorded.push(Recorded {
            topic: small.clone(),
            bytes: frame(HASH_A, i, u64::from(i) * 10_000_000, &[0xAA]),
        });
        recorded.push(Recorded {
            topic: long.clone(),
            bytes: frame(HASH_B, i, u64::from(i) * 700_000_000, &[0xBB]),
        });
    }
    let bag = write_bag(
        dir.path(),
        "m1.mcap",
        &[(&small, HASH_A), (&long, HASH_B)],
        &recorded,
    );

    let mgr = manager("m1");
    let summary = play_once(
        &mgr,
        &bag,
        PlayOptions {
            rate: 1.0,
            topics: vec![small.clone()],
            ..Default::default()
        },
    );
    assert_eq!(summary.topics.len(), 1);
    assert_eq!(summary.topics[0].topic, small);
    // THE pin: the ideal duration is the SMALL channel's ~30ms, not the long
    // channel's 2.1s. A generous ceiling still separates them by ~70x.
    assert!(
        summary.scheduled_span_ns < 300_000_000,
        "the ideal duration must come from the PLAYED channel (~30ms), not the \
         filtered-out one (~2.1s); got {}ms",
        summary.scheduled_span_ns / 1_000_000
    );
}

// ---------------------------------------------------------------------------
// The K-sweep's LOAD-ADAPTIVE budget (pure helpers + one measurement
// helper). The helpers are oracle-tested by
// `the_k_sweep_budget_helpers_match_their_hand_oracles` below — hand vectors,
// never a second run of the same arithmetic.
// ---------------------------------------------------------------------------

/// The K-sweep fixture's recorded period: 100 Hz per tap.
const KSWEEP_PERIOD_NS: u64 = 10_000_000;

/// Frames per tap in the K-sweep fixture.
const KSWEEP_FRAMES: usize = 40;

/// Frames each tap contributes to ONE recorder flush window (`per_batch`) in
/// the K-sweep fixture — the `F` of the aggregate-ceiling arithmetic quoted on
/// the test itself, NOT the per-tap total.
const KSWEEP_BATCH: usize = 10;

/// The `--rate` at which the player's pacing COLLAPSES, so a run measures this
/// machine's raw per-frame publish throughput and nothing else.
///
/// Every scheduled delta becomes `delta / rate`, so at 1000x a 10 ms recorded
/// period schedules 10 us — under the per-frame publish cost, which means every
/// frame is already behind and `sleep_ns` is 0 throughout. The catch-up rate
/// bound shrinks with it (`advance / CATCHUP_FACTOR` = 2.5 us), so it never
/// binds either — which is what makes this baseline defect-IMMUNE: the
/// aggregate catch-up ceiling this test exists to catch cannot inflate it.
const UNPACED_RATE: f64 = 1000.0;

/// How many times this machine's OWN measured publish cost the paced run may
/// spend on top of the recorded span.
///
/// # Calibration, recomputed from the primary table
///
/// The number that matters is the ALLOWANCE/EXCESS margin — how much worse a
/// healthy run could get before the budget fails it — and it must be read off
/// the whole 36-measurement table (12 full-file runs x 3 K), not off the
/// per-K maxima of the excess/unpaced ratio. Those two diverge because the
/// `MIN_ALLOWANCE` floor decouples them at small K.
///
/// Worst healthy margins, MEASURED:
///
/// ```text
///                      K=3     K=8     K=20    overall
///  4.0 / 100ms        1.51x   2.50x   1.44x    1.44x     <- too thin
///  5.0 / 150ms        2.27x   3.13x   1.81x    1.81x     <- chosen
///  6.0 / 150ms        2.27x   3.75x   2.17x    2.17x     <- considered
/// ```
///
/// Dividing the budget by the per-K ratio maxima suggests "1.9-3.2x headroom"
/// at 4.0. That figure is wrong: it is not the real margin, and it ignores the
/// load-affected runs entirely.
/// The true worst margin at 4.0 is `1.44x` — thin enough that this test would
/// fail on a loaded runner.
///
/// What bounds the constant from ABOVE is the free-window-neutralised defect's
/// own excess/baseline ratio at K = 8, MEASURED at `14.14x / 11.75x / 13.76x`.
/// The MINIMUM is what matters (a multiplier at or above it absorbs the
/// defect), so at `5.0` the blindness margin is `11.75 / 5.0` = **2.35x**.
///
/// PROVENANCE: the table above is from the
/// CALIBRATION runs — a separate, deliberately load-mixed set — and
/// is NOT recomputable from the separate 12-run verification table.
/// Recomputing margins from that verification table alone gives, at the shipped
/// `5.0 / 150ms`: `3.03x` (K=3), `2.69x` (K=8), `3.46x` (K=20). Taking the
/// UNION worst of both datasets per column, the K=8 margin should be read as
/// `2.69x` (the verification table's iteration 8), not the `3.13x` above; K=3 and
/// K=20 stay at the calibration runs' `2.27x` / `1.81x` (their K=20 pair is
/// the disclosed `1.670x` load observation). The binding OVERALL figure is
/// unchanged either way: `1.81x`, K=20-bound.
///
/// `5.0 / 150ms` is therefore the balance point: `1.81x` of healthy margin
/// against `2.35x` of defect margin. `6.0` was considered and rejected — it
/// buys `0.36x` more healthy margin for `0.39x` less defect margin, and the
/// healthy side already has a second line of defence the defect side does not
/// (the three-window retry).
///
/// The headroom is not decoration: the paced run performs hundreds of
/// `sleep_interruptible` calls the unpaced run does not, and — the sharper
/// asymmetry — the catch-up rate bound is a PACED-ONLY cost by construction
/// (see the residual on the test), so a preemption can make the paced side pay
/// something the baseline structurally cannot.
const PUBLISH_BUDGET: f64 = 5.0;

/// The allowance never falls below this, however cheap the measured baseline.
///
/// K = 3 pushes only 120 frames, so its baseline is ~15-24 ms and a few ms of
/// ordinary jitter swings the RATIO hard — its worst measured excess/unpaced is
/// `3.37x`, which is 66 ms of excess over a 19.6 ms baseline, not a slow
/// machine. A floor puts the small-K budgets on an absolute footing instead of
/// a noisy denominator.
///
/// `150ms` rather than `100ms`: at 100 ms that same K = 3 run
/// has only `1.51x` of margin, and because it is FLOOR-bound raising
/// `PUBLISH_BUDGET` alone cannot help it. At 150 ms it has `2.27x`.
///
/// It is a FLOOR, never a cap: a genuinely expensive host still gets
/// `PUBLISH_BUDGET * unpaced`. And it is deliberately smaller than half the
/// recorded span (const-asserted below), so a floor-only budget admits at most
/// a `1.38x` run — still STRICTER than a flat `1.5x` ceiling.
const MIN_ALLOWANCE: Duration = Duration::from_millis(150);

/// Bounded window-health retry attempts per K. A transient
/// host stall almost never spans three independent windows; the defect is
/// deterministic and re-measures just as slow in every one.
const MAX_ATTEMPTS: usize = 3;

/// Is a paced run within the recorded span PLUS its allowance — `factor` times
/// the machine's own measured publish cost for the same frames, or
/// `min_allowance`, whichever is LARGER?
///
/// Contention inflates `unpaced` and `paced` together, so the verdict does not
/// invert on a loaded runner — which a fixed multiple of `recorded` does.
fn within_publish_budget(
    paced: Duration,
    recorded: Duration,
    unpaced: Duration,
    factor: f64,
    min_allowance: Duration,
) -> bool {
    let allowance = (factor * unpaced.as_secs_f64()).max(min_allowance.as_secs_f64());
    paced.as_secs_f64() <= recorded.as_secs_f64() + allowance
}

/// The aggregate catch-up ceiling's predicted load factor at `k` taps each
/// contributing `frames_per_round` frames to one recorder flush window.
///
/// The arithmetic is the one quoted on the test itself: a round of `k` taps
/// each contributing `F` frames costs `k·(F−1)·P/CATCHUP_FACTOR` of wall
/// against `F·P` of recorded time, so real-time playback needs
/// `k·(1 − 1/F) <= CATCHUP_FACTOR`. A value over 1.0 means the mechanism ALONE
/// makes real-time playback impossible at that `k` — so if the defect is live,
/// that `k` MUST be over budget. (At the fixture's `F` = `KSWEEP_BATCH` = 10
/// that is 0.675 / 1.8 / 4.5 for K = 3 / 8 / 20.)
fn defect_load_factor(k: usize, frames_per_round: usize) -> f64 {
    k as f64 * (1.0 - 1.0 / frames_per_round as f64) / bag_cmd::CATCHUP_FACTOR as f64
}

/// How one K settled, as the degrade gate reads it.
#[derive(Clone, Copy, Debug, PartialEq)]
struct KOutcome {
    /// Did some attempt meet the budget?
    met: bool,
    /// The paced wall's excess over the recorded span on the attempt that MET
    /// the budget (`Duration::ZERO` when the wall came in under the recorded
    /// span, or when nothing met it).
    excess: Duration,
}

/// How far ABOVE a monotone-in-K ceiling's own prediction the failing K's excess
/// must sit before a stall is the only explanation left.
///
/// MEASURED across the 12-run table: the healthy `E(20) / (E(8) * 20/8)` ratio
/// maxes at `2.12x` (median `0.86x`), and the synthetic single-K stall measures
/// `53x`. So `4.0` sits `1.89x` above the healthy maximum and 13x below the
/// stall.
///
/// **The healthy-run figure is NOT the number that governs.** This term is
/// consulted only at the DECISION POINT, where the K is already over budget, so
/// what matters is the disproportion threshold `B = 4.0 * predicted` against
/// the budget threshold `A = allowance`. MEASURED over the same table at
/// `4.0 / 100ms`, `B/A` ranges `0.97 .. 1.76` with median `1.19` — so
/// the LOUD band `(A, B]` is only ~20% of A wide, and on iteration 4 it is
/// **EMPTY** (`B < A`), meaning every over-budget K = 20 of that shape would
/// degrade rather than fail. Raising the budget makes that worse, not
/// better (`B/A` median falls to `0.96` at 5.0/150ms), because A grows while B
/// does not.
const STALL_DISPROPORTION: f64 = 4.0;

/// A failing K must ALSO be this many times over its own allowance before it
/// can be degraded.
///
/// This is what guarantees the loud band exists. `STALL_DISPROPORTION` alone
/// cannot: its threshold is keyed to the WITNESS's excess, which on a healthy
/// run is small and unrelated to the failing K's allowance, so the band it
/// leaves is whatever the arithmetic happens to give — measured as low as
/// ZERO. With this term the band is `(A, 2A]` by construction: **always
/// non-empty, exactly `A` wide, and load-adaptive**, so a MARGINALLY
/// over-budget K can never be degraded away no matter how small the witness.
///
/// Chosen over the alternative of flooring the PREDICTED excess at a fixed
/// absolute quantity (which also empties the empty-band case) because a fixed
/// floor stops working once a loaded host's allowance grows past it, while this
/// scales with the same quantity it is protecting.
///
/// It does not weaken the stall arm: MEASURED against the SHIPPED constants,
/// the synthetic stall's best excess is `5593.9ms` against an allowance of
/// `527.2ms`, i.e. `2A` = `1054.3ms` — over 5x clear. It does not weaken the
/// defect arm either: that is `STALL_DISPROPORTION`'s job, and a masked
/// witness's own large excess keeps `B` far above `2A`.
const GROSS_OVERAGE: f64 = 2.0;

/// May an over-budget measurement at `sweep[idx]` be declared NON-PROBATIVE?
///
/// THREE conditions, all required, because "met its budget" alone is a WEAKER
/// claim than the argument needs. The budget is load-adaptive, so on a slow
/// host a K = 8 that is plainly NOT playing in real time can still be recorded
/// `met` — and a partially-biting ceiling produces exactly that shape.
///
/// 1. Some SMALLER `k` — one the defect could not have left healthy, i.e. whose
///    [`defect_load_factor`] already exceeds 1 — met its budget. K = 8's factor
///    is 1.8, so a fully-biting ceiling cannot leave it real-time.
/// 2. The failing K's excess is DISPROPORTIONATE to what that witness's OWN
///    measured excess predicts under monotone-in-K scaling. The ceiling's cost
///    is `k(1-1/F)/CATCHUP_FACTOR`, i.e. linear in `k`, so a witness costing
///    `E` at `k` predicts `E * k_fail / k` at the failing K. An excess
///    `STALL_DISPROPORTION` times ABOVE that is explained by no monotone
///    mechanism.
/// 3. The failing K is GROSSLY over its own allowance ([`GROSS_OVERAGE`]), not
///    marginally. Without this the loud band can be empty — measured.
///
/// Condition 2 closes the load-masking path: if a slow host inflates the
/// witness's budget enough to record a LIVE defect as `met`, the witness's own
/// excess is correspondingly LARGE, the prediction is large, and the degrade
/// becomes unavailable. Condition 3 guarantees a marginal over-budget always
/// fails LOUD. They bind in different regimes and neither subsumes the other.
///
/// Contention can only push a smaller K's wall UP — out of `met`, and up its
/// own excess — so it can never MANUFACTURE this verdict; the failure direction
/// is a red, not a silent pass.
fn stall_is_the_only_explanation(
    idx: usize,
    sweep: &[usize],
    outcomes: &[KOutcome],
    failing_excess: Duration,
    failing_allowance: Duration,
    frames_per_round: usize,
) -> bool {
    // (3) A marginal over-budget is never non-probative, whatever the witness.
    if failing_excess.as_secs_f64() <= GROSS_OVERAGE * failing_allowance.as_secs_f64() {
        return false;
    }
    let failing_k = sweep[idx];
    sweep[..idx]
        .iter()
        .zip(&outcomes[..idx])
        .any(|(&k, outcome)| {
            if !outcome.met || defect_load_factor(k, frames_per_round) <= 1.0 {
                return false;
            }
            let predicted = outcome.excess.as_secs_f64() * failing_k as f64 / k as f64;
            failing_excess.as_secs_f64() > STALL_DISPROPORTION * predicted
        })
}

/// Does the sweep contain a K below `idx` that could serve as a degrade witness
/// at all — i.e. one the ceiling could not have left healthy?
///
/// Split out so the failure message can say what was ACTUALLY evaluated for
/// this K. At K = 3 and K = 8 there is no such peer, so no disproportion
/// verdict is ever computed for them and the message must not imply one.
fn degrade_witness_exists(
    idx: usize,
    sweep: &[usize],
    outcomes: &[KOutcome],
    frames_per_round: usize,
) -> bool {
    sweep[..idx]
        .iter()
        .zip(&outcomes[..idx])
        .any(|(&k, o)| o.met && defect_load_factor(k, frames_per_round) > 1.0)
}

/// One K-sweep measurement: the same bag played twice, moments apart, in the
/// same process — first with pacing collapsed (the machine's own publish cost),
/// then at `rate: 1.0` (the run under test).
///
/// Each half gets its OWN isolated SHM root, so neither can occupy the other's
/// single-writer publisher slot, and both pay their own service-creation cost
/// (which keeps the baseline a true ceiling for the paced run's setup).
fn measure_k_sweep_point(
    bag: &Path,
    k: usize,
    attempt: usize,
    expect_frames: u64,
) -> (Duration, Duration, bag_cmd::PlaySummary) {
    let base_mgr = manager(&format!("ksweep{k}base{attempt}"));
    let started = Instant::now();
    let base = play_once(
        &base_mgr,
        bag,
        PlayOptions {
            rate: UNPACED_RATE,
            ..Default::default()
        },
    );
    let unpaced = started.elapsed();
    // The baseline must have pushed the SAME frames, or it is not a cost
    // measurement for this K at all.
    assert_eq!(
        base.total_injected(),
        expect_frames,
        "K={k}: the unpaced baseline must publish every frame, or it is not a cost \
         measurement for this K at all"
    );

    let mgr = manager(&format!("ksweep{k}run{attempt}"));
    let started = Instant::now();
    let summary = play_once(
        &mgr,
        bag,
        PlayOptions {
            rate: 1.0,
            ..Default::default()
        },
    );
    (unpaced, started.elapsed(), summary)
}

/// Hand oracles for the helpers — vectors written out, never a second
/// run of the same arithmetic.
#[test]
fn the_k_sweep_budget_helpers_match_their_hand_oracles() {
    let recorded = Duration::from_millis(390);
    let ms = Duration::from_millis;
    // --- the FACTOR half (floor disabled, so each arm probes the factor) ---
    // A healthy quiet machine: 480ms paced, 110ms unpaced. Excess 90ms vs a 275ms
    // allowance.
    assert!(within_publish_budget(
        ms(480),
        recorded,
        ms(110),
        2.5,
        Duration::ZERO
    ));
    // The SAME machine 2.5x slower: BOTH halves inflate (excess 90ms -> 225ms,
    // baseline 110ms -> 275ms), so the verdict holds. This is the arm a fixed
    // ceiling cannot survive — 615/390 = 1.58x inverts a 1.5x gate while the
    // machine is behaving exactly as it does when quiet.
    assert!(within_publish_budget(
        ms(615),
        recorded,
        ms(275),
        2.5,
        Duration::ZERO
    ));
    // The defect: the paced half explodes while the baseline stays cheap.
    assert!(!within_publish_budget(
        ms(1700),
        recorded,
        ms(110),
        2.5,
        Duration::ZERO
    ));
    // Boundary, both sides: 390 + 2.5*100 = 640ms exactly.
    assert!(within_publish_budget(
        ms(640),
        recorded,
        ms(100),
        2.5,
        Duration::ZERO
    ));
    assert!(!within_publish_budget(
        ms(641),
        recorded,
        ms(100),
        2.5,
        Duration::ZERO
    ));
    // A zero-cost baseline with no floor grants nothing — the run must then fit
    // the recorded span itself, the strictest reading, not a free pass.
    assert!(!within_publish_budget(
        ms(400),
        recorded,
        Duration::ZERO,
        2.5,
        Duration::ZERO
    ));

    // --- the FLOOR half ---
    // The K = 3 shape: a 10ms baseline makes the factor term 25ms, which a
    // 90ms excess blows through. The SAME inputs pass once the floor lifts the
    // allowance to 100ms — the pair is what makes the floor load-bearing.
    assert!(!within_publish_budget(
        ms(480),
        recorded,
        ms(10),
        2.5,
        Duration::ZERO
    ));
    assert!(within_publish_budget(
        ms(480),
        recorded,
        ms(10),
        2.5,
        ms(100)
    ));
    // A FLOOR, never a cap: an expensive host still gets the full factor term
    // (687.5ms here), which the 100ms floor must not shrink.
    assert!(within_publish_budget(
        ms(615),
        recorded,
        ms(275),
        2.5,
        ms(100)
    ));
    // And the floor is not a blank cheque — 390 + 100 = 490ms is the whole of
    // it, so a run past that still fails on a cheap baseline.
    assert!(within_publish_budget(
        ms(490),
        recorded,
        ms(10),
        2.5,
        ms(100)
    ));
    assert!(!within_publish_budget(
        ms(491),
        recorded,
        ms(10),
        2.5,
        ms(100)
    ));

    // Load factors at F = 10, CATCHUP_FACTOR = 4: k * 0.9 / 4 — the same
    // arithmetic the test's own doc quotes (0.675 at K = 3, 1.8 at K = 8).
    assert!((defect_load_factor(3, 10) - 0.675).abs() < 1e-9);
    assert!((defect_load_factor(8, 10) - 1.8).abs() < 1e-9);
    assert!((defect_load_factor(20, 10) - 4.5).abs() < 1e-9);

    // The degrade gate. Sweep [3, 8, 20]: only K = 20 is ever degradable, and
    // only when K = 8 (load factor 1.8 > 1) met its budget AND its own excess
    // is far too small to explain K = 20's under monotone-in-K scaling.
    let sweep = [3usize, 8, 20];
    let met = |millis: u64| KOutcome {
        met: true,
        excess: ms(millis),
    };
    let unmet = KOutcome {
        met: false,
        excess: Duration::ZERO,
    };

    // A MEASURED stall: K = 8 healthy at 42ms, K = 20 at 5605ms. A ceiling
    // costing 42ms at K = 8 predicts 105ms at K = 20; 5605ms is 53x that.
    assert!(stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(42), unmet],
        ms(5605),
        ms(400),
        10
    ));
    // K = 8 over budget ⇒ the defect is NOT refuted, so K = 20 must fail loud.
    assert!(!stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), unmet, unmet],
        ms(5605),
        ms(400),
        10
    ));
    // THE LOAD-MASKING ARM. A slow host inflates K = 8's budget enough to
    // record a LIVE defect as `met` — but a masked defect carries a LARGE
    // excess (464ms, measured with the defect live), which predicts 1160ms at
    // K = 20, so the defect's own ~1365ms cannot clear 4x that and the degrade
    // is unavailable. Without the disproportion term this is a silent pass on
    // a live regression.
    assert!(!stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(464), unmet],
        ms(1365),
        ms(400),
        10
    ));
    // A PARTIAL regression of the same mechanism: K = 8 at 195ms (plainly not
    // real time, but inside a load-adaptive budget) predicts 487ms at K = 20,
    // which its 700ms does not exceed by 4x ⇒ fail loud, not degrade.
    assert!(!stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(195), unmet],
        ms(700),
        ms(300),
        10
    ));
    // Both sides of the disproportion threshold, holding everything else
    // fixed: 42ms at K = 8 predicts 105ms, so the boundary is 420ms.
    assert!(!stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(42), unmet],
        ms(420),
        ms(200),
        10
    ));
    assert!(stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(42), unmet],
        ms(421),
        ms(200),
        10
    ));
    // K = 8 itself can never degrade: its only smaller peer (K = 3, factor
    // 0.675) is one the defect is allowed to leave healthy — even when that
    // peer is healthy and the disproportion is enormous.
    assert!(!stall_is_the_only_explanation(
        1,
        &sweep,
        &[met(1), unmet, unmet],
        ms(5000),
        ms(200),
        10
    ));
    // Nor can the smallest K — there is no witness at all.
    assert!(!stall_is_the_only_explanation(
        0,
        &sweep,
        &[unmet, unmet, unmet],
        ms(5000),
        ms(200),
        10
    ));

    // --- which K can even HAVE a disproportion verdict computed ---
    // This decides what the failure message CLAIMS, and one of the four CI
    // firings was at K = 8 — the K whose message a human is most likely to
    // read. Nothing else in this file asserts it, because the branch only
    // chooses prose on an already-failing run.
    assert!(degrade_witness_exists(
        2,
        &sweep,
        &[met(20), met(40), unmet],
        10
    ));
    // K = 8's only smaller peer is K = 3, whose load factor is 0.675 — the
    // ceiling is ALLOWED to leave it healthy, so it witnesses nothing.
    assert!(!degrade_witness_exists(
        1,
        &sweep,
        &[met(20), unmet, unmet],
        10
    ));
    // The smallest K has no peer at all.
    assert!(!degrade_witness_exists(
        0,
        &sweep,
        &[unmet, unmet, unmet],
        10
    ));
    // A qualifying peer that did NOT meet its budget is not a witness either.
    assert!(!degrade_witness_exists(
        2,
        &sweep,
        &[met(20), unmet, unmet],
        10
    ));

    // --- the GROSS-OVERAGE term: the loud band is never empty ---
    // MEASURED, this is the shape that makes it empty: on iteration 4 of the
    // 12-run table K=20's allowance is 413ms while the disproportion
    // threshold from a 40ms K=8 excess is 400ms — B < A, so without this term EVERY
    // over-budget K=20 of that shape would degrade rather than fail.
    //
    // A MARGINAL over-budget (1.2x its allowance) fails loud even though
    // the witness is tiny and the disproportion term alone would license it...
    assert!(!stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(40), unmet],
        ms(496),
        ms(413),
        10
    ));
    // ...while the SAME witness and allowance with a grossly over-budget
    // excess still degrades (the term gates marginal cases only).
    assert!(stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(40), unmet],
        ms(5605),
        ms(413),
        10
    ));
    // Both sides of the gross-overage threshold at a fixed allowance: 2 x 200
    // = 400ms, and the disproportion bar (4 x 2.5 x 20 = 200ms) is clear of it
    // so this arm probes ONLY the new term.
    assert!(!stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(20), unmet],
        ms(400),
        ms(200),
        10
    ));
    assert!(stall_is_the_only_explanation(
        2,
        &sweep,
        &[met(20), met(20), unmet],
        ms(401),
        ms(200),
        10
    ));
}

// Shipped-constant drift guards, at COMPILE time. The oracle vectors above pass
// their own factor by hand, so without these a constant could drift with every
// arm still green.
//
// The baseline is defect-IMMUNE only while the catch-up spacing at
// `UNPACED_RATE` stays far under the per-frame publish cost (MEASURED at
// 91-108 us on a quiet M3 debug build). Two orders of magnitude of margin:
const _: () = assert!(
    KSWEEP_PERIOD_NS as f64 / UNPACED_RATE / (bag_cmd::CATCHUP_FACTOR as f64) < 10_000.0,
    "the unpaced baseline stops measuring publish cost once the catch-up bound approaches it"
);
// Real headroom over the measured 2.16x worst healthy ratio, and still well
// below the defect's own MINIMUM excess/baseline ratio at K = 8 (11.75x
// measured), which is what a budget multiplier would have to reach to absorb
// the defect.
const _: () = assert!(
    PUBLISH_BUDGET >= 3.0 && PUBLISH_BUDGET <= 6.0,
    "PUBLISH_BUDGET must keep real headroom without approaching the defect's own ratios"
);
// The floor must never on its own admit a run a flat 1.5x ceiling would
// fail: half the recorded span is 195ms, so any floor under it keeps a
// floor-only budget below 1.5x (at 150ms it tops out at 1.38x).
const _: () = assert!(
    MIN_ALLOWANCE.as_nanos() * 2 < ((KSWEEP_FRAMES as u128 - 1) * KSWEEP_PERIOD_NS as u128),
    "MIN_ALLOWANCE must stay under half the recorded span"
);
// A retry that cannot run twice is not a window-health retry.
const _: () = assert!(
    MAX_ATTEMPTS >= 2,
    "MAX_ATTEMPTS must allow a real re-measure"
);

/// The K-SWEEP. Every other tap-grouped fixture stops at 3 topics,
/// and 3 is exactly where the aggregate ceiling does not bite.
///
/// The player is single-threaded, so a per-frame catch-up spacing is an
/// AGGREGATE throughput ceiling: a round of `K` taps each contributing `F`
/// frames costs `K·(F−1)·P/4` of wall against `F·P` of recorded time, i.e.
/// real-time playback needs `Σ(1 − 1/F) ≤ 4`. At K=3 that is 0.675 (fine — which
/// is why every 3-topic fixture passes); at K=8 it is 1.8 and every channel falls
/// permanently behind, with the overrun branch telling a healthy machine to
/// lower `--rate`. A 75-topic robot bag measured 4.7x slow, and that is the
/// DEFAULT shape for `ros2 attach`.
///
/// The free window (`CATCHUP_FREE_DELTAS`) removes that ceiling by letting
/// routine flush-window deficits publish immediately. This arm sweeps K and
/// asserts a per-K CEILING — deliberately not literal flatness.
///
/// The wall does still grow slowly with K (MEASURED 1.067x / 1.150x / 1.237x at
/// K = 3 / 8 / 20), but that residual is the per-frame publish cost of a
/// debug build: K = 20 packs 800 frames into 390ms of recorded time. It is not
/// the aggregate catch-up ceiling, which grows far faster — the same sweep with
/// the free window removed measures 1.443x at K=3 and 2.190x at K=8.
///
/// The budget is MEASURED on this machine, not hard-coded.
///
/// A fixed `1.5x` ceiling inverts under load. MEASURED walls from four runs on
/// loaded macOS runners, with the play path and this crate unchanged
/// (wall / recorded span per K):
///
/// ```text
/// run 1   K=3 1.156  K=8 1.420  K=20 1.536   <- first over: K=20
/// run 2   K=3 1.121  K=8 1.196  K=20 1.719   <- first over: K=20
/// run 3   K=3 1.073  K=8 1.564  (never ran)  <- first over: K=8
/// run 4   K=3 1.098  K=8 1.233  K=20 1.535   <- first over: K=20
/// ```
///
/// Two details matter: runs 1 and 2 cross it at
/// `1.536x` and `1.719x`, not the same number twice; and
/// ONE run crosses it at **K = 8**, which the degrade arm below structurally cannot
/// cover. A wall assertion tight enough to catch the defect is tight enough for
/// a loaded runner to invert (the load-sensitivity class), so this test anchors the
/// oracle on a quantity load cannot fake rather than loosening the number.
///
/// The doc above already says the residual growth in K IS the per-frame publish
/// cost of a debug build, explicitly NOT the property under test. So the test
/// MEASURES that cost instead of budgeting for it: for each K it plays the SAME
/// bag twice moments apart in the same process — once with pacing COLLAPSED
/// (`UNPACED_RATE`), once at `rate: 1.0` — and requires
///
/// ```text
/// paced_wall - recorded_span  <=  max(PUBLISH_BUDGET * unpaced_wall, MIN_ALLOWANCE)
/// ```
///
/// MEASURED consequence, recomputed from the 12-run verification table
/// at `5.0 / 150ms`: the
/// effective ceilings are `1.38x` at K = 3 (floor-bound, identical in all 12
/// runs), `1.40x–1.64x` at K = 8, and `1.83x–2.04x` at K = 20. Against a
/// flat `1.5x`: STRICTER at K = 3 always; at K = 8 the ceiling
/// EXCEEDS `1.5x` on 5 of the 12 measured runs (worst `1.64x`); LOOSER at
/// K = 20 by design. That is the true trade — the measured budget buys
/// load-robustness by admitting, at the two larger Ks, some quiet-machine walls
/// a flat ceiling would refuse.
///
/// Contention inflates BOTH sides; the defect inflates only the paced one,
/// because at `UNPACED_RATE` the catch-up spacing is `own delta / rate /
/// CATCHUP_FACTOR` — microseconds, far under the per-frame publish cost — so it
/// never binds and the baseline stays a clean publish-throughput measurement
/// with the defect present or absent.
///
/// Two further defences, in order:
///
/// - a K over budget is RE-MEASURED in fresh temporal windows (the window-health
///   retry), up to `MAX_ATTEMPTS`, with per-attempt accounting printed;
/// - a K that is STILL over budget is declared NON-PROBATIVE (a `DEGRADE`
///   line, that K's wall claims skipped) only under the TWO conditions
///   [`stall_is_the_only_explanation`] documents: a smaller K the ceiling could
///   not have left healthy met its budget, AND the failing K's excess is
///   disproportionate to what that witness's own excess predicts under
///   monotone-in-K scaling. Everything load-safe still asserts unconditionally
///   (every frame published, plus the `0.9x` paced floor, which contention can
///   only make MORE true).
///
/// What fails this test: the
/// defect (`CATCHUP_FREE_DELTAS` = 0) fails at K = 8 across ALL THREE attempts,
/// which is also the proof the retry does not weaken the pin (a deterministic
/// defect re-measures just as slow in every fresh window) and that the degrade
/// cannot hide it (K = 8 is structurally non-degradable — its only smaller
/// peer, K = 3, has load factor 0.675).
///
/// # Residuals, stated because they are real holes
///
/// **A degrade is INVISIBLE in CI.** Both `DEGRADE` paths end in a
/// PASS, and libtest discards a passing test's captured output unless
/// `--nocapture` / `--show-output` is given, which CI does not pass
/// (`.github/workflows/ci.yml`). So a degraded run is a plain green check and
/// the explanation reaches nobody; the only way to see it is to re-run locally
/// with `-- --nocapture`. This is the load-sensitivity class, which
/// `graph_profile_iox2_test` documents identically. The print is NOT a
/// mitigation for the residual below, which is why that residual is stated
/// as a silent one.
///
/// - **the degrade arm**: a hypothetical NEW regression that bites only above
///   K = 8, survives every retry, AND is disproportionate enough to clear
///   `STALL_DISPROPORTION` would be degraded rather than failed — silently, per
///   the paragraph above. Narrowed but not closed by the monotone-scaling term.
/// - **K = 3 and K = 8 have NO degrade path at all** (no qualifying witness
///   below them), so their only protection against an ambient stall is the
///   measured budget. One of the four loaded runs above crosses `1.5x` at K = 8.
/// - **the catch-up rate bound is a PACED-ONLY cost, and the baseline
///   provably cannot cover it.** The whole re-anchor rests on contention
///   inflating both halves — but the `CATCHUP_FACTOR` spacing is `own delta /
///   rate / CATCHUP_FACTOR`, and the compile-time guard above proves that at
///   `UNPACED_RATE` it is 2.5us, i.e. the baseline can NEVER pay it. So a
///   preemption deep enough to push a channel past `CATCHUP_FREE_DELTAS` (16
///   deltas = 160ms at this fixture's 100 Hz) makes the paced run re-space its
///   backlog at 2.5ms a frame, producing the SAME ~1.8-2.2x K = 8 signature as
///   the real defect on a HEALTHY player, while the baseline stays flat. That
///   is a hard RED, and K = 8 has no degrade path — it may well be the actual
///   mechanism of the 1.564x K = 8 measurement above. The only shield is the
///   three-window retry, and the likely case is that it needs all three
///   ~430ms paced windows to escape a preemption of that depth. Closing it
///   properly means folding the player's own catch-up accounting into the
///   budget (it knows when slip exceeded the free window; `catchup_channels`
///   cannot serve, since the defect sets it too).
/// - **the overrun pair is not a wall detector.** `within_publish_budget`
///   divides out per-frame publish cost by construction, so it is blind to a
///   UNIFORM publish-cost regression (a new per-frame allocation in the play
///   path inflates both halves). The 250ms overrun assert
///   is implied by this run's
///   own paced excess and is a drift guard only. Nothing in this file covers
///   the uniform-cost class — it wants its own absolute-cost gate.
///   MEASURED margin, so the claim is not hand-waved: the player's own
///   `elapsed - scheduled_span_ns` is 1.7-4.3ms against the 250ms floor.
/// - **a stall landing on a BASELINE window** inflates that attempt's budget,
///   the masking direction. At `PUBLISH_BUDGET` = 5.0, masking K = 8's
///   measured defect wall needs a baseline of `>= 94.0ms` against the
///   `33.2-40.3ms` the defect build measures — `2.33x` over the WORST of
///   those three — and
///   the retry gives that draw three chances. It is REACHABLE: the 12-run
///   table has K = 8 baselines of 85.7ms and 123.3ms under ambient load. What
///   stops it being a silent pass is the disproportion term: a masked witness
///   carries a LARGE excess, so it cannot license a K = 20 degrade and the
///   defect surfaces there instead.
#[test]
fn the_wall_stays_under_a_ceiling_as_topic_count_grows() {
    const FRAMES: usize = KSWEEP_FRAMES;
    const PERIOD_NS: u64 = KSWEEP_PERIOD_NS;
    const SWEEP: [usize; 3] = [3, 8, 20]; // ASCENDING — the degrade gate reads smaller Ks
    let recorded_span = Duration::from_nanos((FRAMES as u64 - 1) * PERIOD_NS);

    // One throwaway play BEFORE the sweep. The first player run in a process
    // pays one-time costs (lazily-built schema walker, first iceoryx2 service
    // creation) that would otherwise land entirely in K = 3's baseline and make
    // that K's budget accidentally generous while every later K's is accurate.
    // Discarded — nothing is asserted on it.
    {
        let dir = tempfile::tempdir().expect("tempdir");
        let id = unique();
        let names: Vec<String> = (0..SWEEP[0]).map(|i| format!("/playw/{id}/t{i}")).collect();
        let specs: Vec<(&str, u64)> = names.iter().map(|n| (n.as_str(), HASH_A)).collect();
        let bag = write_tap_grouped_bag(
            dir.path(),
            "warm.mcap",
            &specs,
            FRAMES,
            PERIOD_NS,
            KSWEEP_BATCH,
        );
        let _ = play_once(
            &manager("ksweepwarm"),
            &bag,
            PlayOptions {
                rate: UNPACED_RATE,
                ..Default::default()
            },
        );
    }

    let mut outcomes = [KOutcome {
        met: false,
        excess: Duration::ZERO,
    }; SWEEP.len()];
    for (idx, k) in SWEEP.into_iter().enumerate() {
        let dir = tempfile::tempdir().expect("tempdir");
        let id = unique();
        let names: Vec<String> = (0..k).map(|i| format!("/playk/{id}/t{i}")).collect();
        let specs: Vec<(&str, u64)> = names
            .iter()
            .enumerate()
            .map(|(i, n)| (n.as_str(), if i % 2 == 0 { HASH_A } else { HASH_B }))
            .collect();
        let bag = write_tap_grouped_bag(
            dir.path(),
            "ksweep.mcap",
            &specs,
            FRAMES,
            PERIOD_NS,
            KSWEEP_BATCH,
        );

        // Window-health retry: a transient host stall does not
        // survive a fresh temporal window; the defect re-measures just as slow.
        let mut log: Vec<String> = Vec::new();
        let mut passed: Option<(Duration, Duration, f64, bag_cmd::PlaySummary)> = None;
        // The MINIMUM excess across attempts — the reading most favourable to
        // "this is a real ceiling", so the degrade gate is fed the hardest
        // number to call disproportionate — carried with the allowance of the
        // SAME attempt, since the gross-overage term compares the two.
        let mut best_excess = Duration::MAX;
        let mut best_allowance = Duration::ZERO;
        for attempt in 1..=MAX_ATTEMPTS {
            let (unpaced, paced, summary) =
                measure_k_sweep_point(&bag, k, attempt, (FRAMES * k) as u64);
            let ratio = paced.as_secs_f64() / recorded_span.as_secs_f64();
            let allowance =
                (PUBLISH_BUDGET * unpaced.as_secs_f64()).max(MIN_ALLOWANCE.as_secs_f64());
            let budget = recorded_span.as_secs_f64() + allowance;
            let excess = paced.saturating_sub(recorded_span);
            if excess < best_excess {
                best_excess = excess;
                best_allowance = Duration::from_secs_f64(allowance);
            }
            let line = format!(
                "K-sweep: K={k} attempt={attempt} wall={paced:?} ratio={ratio:.3} \
                 unpaced={unpaced:?} budget={budget:.3}s"
            );
            eprintln!("{line}");
            log.push(line);

            assert_eq!(
                summary.total_injected(),
                (FRAMES * k) as u64,
                "K={k}: every frame must publish"
            );
            // Load-SAFE in the right direction: contention only pushes a wall
            // UP, so a dumped run cannot hide behind a slow runner.
            assert!(
                ratio >= 0.9,
                "K={k}: took only {paced:?} ({ratio:.3}x) — the frames were DUMPED, not paced"
            );

            if within_publish_budget(paced, recorded_span, unpaced, PUBLISH_BUDGET, MIN_ALLOWANCE) {
                outcomes[idx] = KOutcome { met: true, excess };
                passed = Some((unpaced, paced, allowance, summary));
                break;
            }
        }

        let Some((_unpaced, paced, allowance, summary)) = passed else {
            // THE pin: the wall must stay flat in K. Without the free window it
            // grows roughly linearly once K passes ~4 (measured K=8 at 1.945x).
            //
            // The message must state only what was ACTUALLY evaluated for THIS
            // K. At K = 3 and K = 8 no smaller peer can serve as a witness, so
            // no disproportion verdict is ever computed for them — and one of
            // the four CI firings was at K = 8, i.e. the K whose message a
            // human is most likely to read.
            let why = if degrade_witness_exists(idx, &SWEEP, &outcomes, KSWEEP_BATCH) {
                format!(
                    "Its best excess was {best_excess:?} against an allowance of \
                     {best_allowance:?}, and that is NOT both {GROSS_OVERAGE}x over its own \
                     allowance AND {STALL_DISPROPORTION}x beyond what a monotone-in-K ceiling \
                     predicts from the smaller Ks"
                )
            } else {
                format!(
                    "Its best excess was {best_excess:?} against an allowance of \
                     {best_allowance:?}. No smaller K in this sweep can refute the ceiling here \
                     (a witness needs a load factor above 1, and the smallest such K is 8), so \
                     no disproportion verdict was computed — the budget is the whole verdict"
                )
            };
            assert!(
                stall_is_the_only_explanation(
                    idx,
                    &SWEEP,
                    &outcomes,
                    best_excess,
                    best_allowance,
                    KSWEEP_BATCH
                ),
                "K={k}: a {recorded_span:?} recording must play in about that long — plus this \
                 machine's OWN measured publish cost for {} frames — whatever the topic count, \
                 and it did not across {MAX_ATTEMPTS} attempt(s). {why}, so this is the \
                 single-threaded aggregate catch-up ceiling.\n  {}",
                FRAMES * k,
                log.join("\n  ")
            );
            eprintln!(
                "DEGRADE: K={k} stayed over its measured budget across {MAX_ATTEMPTS} \
                 attempt(s) (best excess {best_excess:?} vs allowance {best_allowance:?}, i.e. \
                 over {GROSS_OVERAGE}x) while a smaller K the ceiling could not have left \
                 healthy (K=8, cost {:.2}x real time) met its own budget with an excess \
                 {STALL_DISPROPORTION}x too small to explain it under monotone-in-K scaling. \
                 Treating this run as NON-PROBATIVE for K={k}; re-run on a quiescent host. NOTE: \
                 this line is DISCARDED by libtest in CI — the run is a plain green check.\n  {}",
                defect_load_factor(8, KSWEEP_BATCH),
                log.join("\n  ")
            );
            continue;
        };

        // The no-false-alarm claim ("a healthy bag is never told to lower
        // --rate") is decidable while THIS run's paced excess is under the
        // player's own overrun REPORT floor — because that excess is a strict
        // UPPER BOUND on what the player can report:
        //
        //   elapsed <= paced                      (`elapsed` starts after route
        //                                          creation, ends earlier)
        //   scheduled_span_ns >= recorded_span    (= anchor + 39 x 10ms)
        //   => elapsed - scheduled_span <= paced - recorded
        //
        // so under the gate the pair is IMPLIED and can never be a false red on
        // a loaded runner, while still running on essentially every real run.
        //
        // Keying it on `unpaced` instead would compare the wrong
        // quantity — the report fires on the PACED excess, and the two diverge
        // — so a run the budget had just approved could hard-fail on the next
        // line, un-retried and outside the degrade path.
        //
        // Keying it on the ALLOWANCE would be safe but would throw away
        // coverage: MEASURED, the player's own `elapsed - scheduled_span` is
        // 1.7-4.3ms against the 250ms floor (the `paced` excess of 19-65ms is
        // almost entirely route/publisher setup OUTSIDE the player's timed
        // window), while the allowance at K = 20 is already 253-263ms — so an
        // allowance gate skips the claim on every K = 20 run for a hazard
        // sitting ~60x away.
        //
        // What the pair still buys is a DRIFT GUARD tying this test's own
        // arithmetic to the player's overrun accounting: make
        // `scheduled_span_ns` under-count and the implication breaks and this
        // fires. It is deliberately not a wall detector any
        // more — see the residual on the test.
        let paced_excess = paced.saturating_sub(recorded_span);
        if (paced_excess.as_nanos() as u64) < bag_cmd::WALL_OVERRUN_REPORT_FLOOR_NS {
            let text = bag_cmd::render_play_summary(&summary);
            assert!(!text.contains("Lower --rate"), "K={k}: {text}");
            assert_eq!(summary.wall_overrun_ns(), None, "K={k}: {text}");
        } else {
            eprintln!(
                "DEGRADE: K={k}: this run's paced excess is {paced_excess:?}, at or over \
                 the player's {}ms overrun report floor — an overrun report here would be TRUE, \
                 so the no-false-alarm claim is skipped (allowance was {allowance:.3}s).",
                bag_cmd::WALL_OVERRUN_REPORT_FLOOR_NS / 1_000_000
            );
        }
    }
}

// ===========================================================================
// The BAG-TIME WINDOW (`--start-offset` / `--duration`)
// ===========================================================================
//
// These arms are the only tests that set a non-`None` `start_offset_ns` or
// `duration_bound_ns` on `PlayOptions`. Without them the two halves of one flag can
// disagree: a playback walk that excludes a frame at `> d` while the resim loop
// excludes one at `>= d`, with `docs/bag.md` documenting HALF-OPEN for both. The
// consequence is not a rounding difference — `bag play --duration 0`
// would republish the first frame of every channel while
// `bag play --resim all --duration 0` correctly executes nothing.
//
// The oracle throughout is `PlaySummary::topics[].injected`, a COUNT the player
// reports for what it validated and published. It is used rather than a
// subscriber drain because these arms are about WHICH frames the window admits,
// and a drain additionally measures transport warm-up (the sibling arms above
// carry the byte-verbatim republication contract).

/// Playback rate for the window arms.
///
/// The window is BAG TIME and the rate is WALL time, so the rate cannot change
/// which frames a bound admits — but at `1.0` these arms spend ~40 ms of wall
/// each pacing frames whose placement they do not assert, and this file is
/// PARALLEL-safe, so that wall lands on the sibling pacing arms
/// (`a_tap_grouped_bag_plays_at_its_recorded_duration` and its neighbours),
/// whose whole contract is a `< 1.5x recorded span` band. MEASURED: running
/// these four arms at `rate: 1.0` takes that binary from 6/6 green to 3/5 — the
/// load-sensitivity class, caused by this test rather than suffered by it. At 100x the
/// arms are effectively instant and assert exactly the same thing.
const WINDOW_RATE: f64 = 100.0;

/// One channel, five frames, 10 ms apart in BAG TIME, starting at 1 s.
fn window_fixture(dir: &Path, name: &str, topic: &str) -> PathBuf {
    const BASE_NS: u64 = 1_000_000_000;
    const SPACING_NS: u64 = 10_000_000;
    let frames: Vec<Recorded> = (0..5u32)
        .map(|i| Recorded {
            topic: topic.to_string(),
            bytes: frame(
                HASH_A,
                i,
                BASE_NS + u64::from(i) * SPACING_NS,
                &[i as u8; 24],
            ),
        })
        .collect();
    write_bag(dir, name, &[(topic, HASH_A)], &frames)
}

#[test]
fn a_zero_duration_bound_republishes_nothing() {
    // THE half-open boundary at its degenerate end. `--duration 0` asks for a
    // window that contains no bag time at all, and the only correct answer is
    // zero frames — which is also what the resim half executes.
    //
    // With `> d` instead of `>= d`, `bag_cmd`'s window admits each
    // channel's FIRST frame (`bag_elapsed == 0`), and this reads `injected: 1`.
    let dir = tempfile::tempdir().unwrap();
    let topic = format!("/win/zero{}", unique());
    let bag = window_fixture(dir.path(), "zero.mcap", &topic);
    let m = manager("winzero");
    let summary = play_once(
        &m,
        &bag,
        PlayOptions {
            duration_bound_ns: Some(0),
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    let played: u64 = summary.topics.iter().map(|t| t.injected).sum();
    assert_eq!(
        played, 0,
        "a zero-length window covers NOTHING: {:?}",
        summary.topics
    );
}

#[test]
fn a_duration_bound_is_half_open_on_both_sides_of_a_frame() {
    // The boundary pinned where it BITES, not only at zero. The channel's
    // frames sit at bag-elapsed 0, 10, 20, 30, 40 ms.
    //
    //  * a bound of EXACTLY 20 ms admits elapsed 0 and 10 and EXCLUDES 20 (the
    //    half-open rule), i.e. 2 frames;
    //  * a bound one nanosecond ABOVE 20 ms admits 20 too, i.e. 3.
    //
    // Both sides are asserted in ONE body, so "the bound does something" cannot
    // pass — only the exact placement of the edge does.
    //
    // A `> d` comparison makes the first arm read 3 (it admits the frame AT the
    // bound) while the second still reads 3, collapsing the pair.
    let dir = tempfile::tempdir().unwrap();
    let m = manager("winhalf");

    let topic_at = format!("/win/at{}", unique());
    let bag_at = window_fixture(dir.path(), "at.mcap", &topic_at);
    let at_bound = play_once(
        &m,
        &bag_at,
        PlayOptions {
            duration_bound_ns: Some(20_000_000),
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    let played_at: u64 = at_bound.topics.iter().map(|t| t.injected).sum();

    let topic_past = format!("/win/past{}", unique());
    let bag_past = window_fixture(dir.path(), "past.mcap", &topic_past);
    let past_bound = play_once(
        &m,
        &bag_past,
        PlayOptions {
            duration_bound_ns: Some(20_000_001),
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    let played_past: u64 = past_bound.topics.iter().map(|t| t.injected).sum();

    assert_eq!(
        played_at, 2,
        "a bound EXACTLY at a frame's elapsed excludes it (half-open): {:?}",
        at_bound.topics
    );
    assert_eq!(
        played_past, 3,
        "one nanosecond past that frame admits it: {:?}",
        past_bound.topics
    );
}

#[test]
fn a_start_offset_skips_the_prefix_and_the_bound_measures_from_it() {
    // `--start-offset` is `>=`-inclusive at ITS edge and the `--duration` bound
    // is measured FROM it, so the two compose into a window rather than two
    // independent cuts. Frames at 0, 10, 20, 30, 40 ms:
    //
    //  * offset 20 ms alone      => 20, 30, 40  (3 frames; the frame AT the
    //                               offset is IN — `bag_elapsed < from` skips)
    //  * offset 20 ms + 20 ms    => 20, 30      (2; 40 is at from+20, excluded)
    let dir = tempfile::tempdir().unwrap();
    let m = manager("winoff");

    let topic_a = format!("/win/off{}", unique());
    let bag_a = window_fixture(dir.path(), "off.mcap", &topic_a);
    let offset_only = play_once(
        &m,
        &bag_a,
        PlayOptions {
            start_offset_ns: Some(20_000_000),
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    assert_eq!(
        offset_only.topics.iter().map(|t| t.injected).sum::<u64>(),
        3,
        "the frame AT the offset is inside the window: {:?}",
        offset_only.topics
    );

    let topic_b = format!("/win/offdur{}", unique());
    let bag_b = window_fixture(dir.path(), "offdur.mcap", &topic_b);
    let both = play_once(
        &m,
        &bag_b,
        PlayOptions {
            start_offset_ns: Some(20_000_000),
            duration_bound_ns: Some(20_000_000),
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    assert_eq!(
        both.topics.iter().map(|t| t.injected).sum::<u64>(),
        2,
        "the bound is measured FROM the offset, half-open: {:?}",
        both.topics
    );
}

#[test]
fn a_looping_bound_re_applies_per_pass() {
    // `--loop` clears the per-channel window state at each pass, so the bound
    // means "the first D of the bag, EVERY time round" rather than "the first D
    // of the whole run" — which the second reading would make a `--loop` that
    // stops after one pass.
    //
    // The stop is a CONDITION, never a wall: a stop that sleeps 400 ms on the
    // TEST thread before starting the player and flips the flag from a timer
    // spawned beforehand loses to a >400 ms preemption between the two — ordinary
    // when four arms run in parallel behind a compile — which flips it before
    // playback begins and the arm reads `passes=0, elapsed=2.4µs`. This arm
    // pre-creates the subscriber (so nothing is missed), runs the player on its
    // own thread, and waits until enough frames have really been DELIVERED —
    // which load can delay but not invert — under a seconds-scale liveness
    // ceiling.
    let dir = tempfile::tempdir().unwrap();
    let topic = format!("/win/loop{}", unique());
    let bag = window_fixture(dir.path(), "loop.mcap", &topic);
    let m = manager("winloop");
    let sub = m
        .create_subscriber(&topic)
        .expect("pre-create the topic so the subscriber is attached before playback");

    let running = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&running);
    let mgr = Arc::clone(&m);
    let bag_path = bag.clone();
    let handle = std::thread::spawn(move || {
        bag_cmd::bag_play_with_manager(
            &mgr,
            &bag_path,
            PlayOptions {
                repeat: true,
                duration_bound_ns: Some(20_000_000),
                rate: WINDOW_RATE,
                ..PlayOptions::default()
            },
            flag,
            &mut Vec::new(),
        )
    });
    // 5 delivered frames is strictly more than two passes of the 2-frame
    // window, so reaching it PROVES the bound re-armed at least twice.
    let got = collect(&sub, 5, Duration::from_secs(20));
    running.store(false, Ordering::Relaxed);
    let summary = handle.join().expect("player thread").expect("play");
    assert!(
        got.len() >= 5,
        "the looping player must keep re-playing its window; delivered {} frame(s)",
        got.len()
    );

    let played: u64 = summary.topics.iter().map(|t| t.injected).sum();
    assert!(
        played >= 4,
        "at least two passes of the 2-frame window must have played, got {played}: \
         topics={:?} passes={} refused={:?} elapsed={:?}",
        summary.topics,
        summary.passes,
        summary.refused,
        summary.elapsed
    );
    // The bound holds WITHIN every pass: a pass admits at most the window's 2
    // frames, so the total can never exceed 2 per pass that ran. The `+ 2`
    // absorbs the pass that was in flight when the interrupt landed (`passes`
    // counts COMPLETED passes). A bound that leaked across passes would play
    // the whole 5-frame bag on the first pass and blow this ceiling.
    assert!(
        played <= 2 * (summary.passes + 1),
        "each of the {} completed pass(es) may inject at most the window's 2 frames, \
         got {played}: {:?}",
        summary.passes,
        summary.topics
    );
}

/// One channel whose PRODUCER RESTARTS mid-bag: four frames of a first epoch
/// 10 ms apart from 1 s, then `tail` frames of a SECOND epoch stamped far
/// BELOW the first.
///
/// That is the epoch reset as it lands in a recording — a restarted
/// worker's gating clock begins again near zero, so its frames carry stamps
/// under everything the topic recorded before. The sequence restarts with it,
/// because that is what a fresh publisher's commit counter does.
fn restarting_window_fixture(dir: &Path, name: &str, topic: &str, tail: u32) -> PathBuf {
    const EPOCH_A_NS: u64 = 1_000_000_000;
    const EPOCH_B_NS: u64 = 500_000_000;
    const SPACING_NS: u64 = 10_000_000;
    let mut frames: Vec<Recorded> = (0..4u32)
        .map(|i| Recorded {
            topic: topic.to_string(),
            bytes: frame(
                HASH_A,
                i,
                EPOCH_A_NS + u64::from(i) * SPACING_NS,
                &[i as u8; 24],
            ),
        })
        .collect();
    frames.extend((0..tail).map(|i| Recorded {
        topic: topic.to_string(),
        bytes: frame(
            HASH_A,
            i,
            EPOCH_B_NS + u64::from(i) * SPACING_NS,
            &[(100 + i) as u8; 24],
        ),
    }));
    write_bag(dir, name, &[(topic, HASH_A)], &frames)
}

#[test]
fn a_closed_duration_window_stays_closed_across_a_producer_restart() {
    // The bound is a comparison against `ts - first`, and `saturating_sub`
    // collapses a stamp BELOW `first` to 0 — which reads as "the very start of
    // the window" on a channel that has already run past its end. A stamp below
    // `first` is not a corrupt bag: it is the epoch reset (a restarted
    // producer's gating clock begins again near zero) recorded on whichever
    // topic it happened to, so `--duration` republished past the end the
    // operator asked for on exactly the runs an operator is most likely to be
    // inspecting.
    //
    // The fixture's first epoch sits at bag-elapsed 0, 10, 20, 30 ms and the
    // second at a stamp 500 ms BELOW the first frame's. Against a 20 ms bound:
    //
    //  * frames at elapsed 0 and 10 are inside  => 2
    //  * 20 and 30 are past it (half-open)      => the channel CLOSES
    //  * the restart's frames collapse to 0     => admitted without the latch, 4 total
    //
    // Dropping the `bag_window_closed` latch from `bag_cmd`'s window
    // makes this read 4.
    let dir = tempfile::tempdir().unwrap();
    let m = manager("winrestart");

    let topic = format!("/win/restart{}", unique());
    let bag = restarting_window_fixture(dir.path(), "restart.mcap", &topic, 2);
    let bounded = play_once(
        &m,
        &bag,
        PlayOptions {
            duration_bound_ns: Some(20_000_000),
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    assert_eq!(
        bounded.topics.iter().map(|t| t.injected).sum::<u64>(),
        2,
        "a window the bound CLOSED must stay closed — a regressing stamp is a \
         restart, not a fresh window: {:?}",
        bounded.topics
    );

    // ANTI-TAUTOLOGY: the same bag with NO bound plays all six frames, so the
    // count above is the WINDOW refusing them and not the fixture being
    // unplayable (a restart-detecting player that dropped the second epoch
    // outright would read 2 here too, and it must not).
    let topic_all = format!("/win/restartall{}", unique());
    let bag_all = restarting_window_fixture(dir.path(), "restartall.mcap", &topic_all, 2);
    let unbounded = play_once(
        &m,
        &bag_all,
        PlayOptions {
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    assert_eq!(
        unbounded.topics.iter().map(|t| t.injected).sum::<u64>(),
        6,
        "an unbounded run plays the restart's frames like any others: {:?}",
        unbounded.topics
    );
}

#[test]
fn a_restart_inside_an_open_duration_window_is_still_played() {
    // The latch is scoped to what the BOUND closed, and this is the arm that
    // says so: a bound WIDE enough to cover the whole first epoch never closes
    // the channel, so the restart's frames are admitted exactly as an unbounded
    // run admits them. Without it, "the window stays closed" would be
    // indistinguishable from "a regressing stamp is refused", which is a
    // different and wrong rule.
    //
    // Frames at elapsed 0, 10, 20, 30 ms under a 1 s bound: all four are in,
    // the channel never closes, and the restart's two frames (collapsing to
    // elapsed 0) are inside that same window.
    let dir = tempfile::tempdir().unwrap();
    let m = manager("winopen");
    let topic = format!("/win/restartopen{}", unique());
    let bag = restarting_window_fixture(dir.path(), "restartopen.mcap", &topic, 2);
    let wide = play_once(
        &m,
        &bag,
        PlayOptions {
            duration_bound_ns: Some(1_000_000_000),
            rate: WINDOW_RATE,
            ..PlayOptions::default()
        },
    );
    assert_eq!(
        wide.topics.iter().map(|t| t.injected).sum::<u64>(),
        6,
        "a window the bound never closed admits the restart's frames: {:?}",
        wide.topics
    );
}
