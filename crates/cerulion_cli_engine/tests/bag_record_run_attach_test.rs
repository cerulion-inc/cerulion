// SPDX-License-Identifier: AGPL-3.0-only
//! End to end: `cerulion bag record --run` attaches to a LIVE run.
//!
//! Everything else about the run attach is pinned by pure oracles, which structurally
//! cannot see the one thing this file exists for: whether the wiring between
//! `resolve_run_attach` and `Recorder::setup` actually reaches a REAL trace ring
//! at its LIVE cursor. A mid-run attach is not a lossy version of an at-zero one
//! — `TraceRingConsumer::open` starts at record 0 and `unread_regions` treats
//! `write - read > capacity` as `Overrun`, so a lapped ring is REFUSED outright.
//! The headline arm therefore laps its ring on purpose.
//!
//! The fixture is a `graph run` in miniature, built from production pieces: a
//! real `/__cerulion/runs` registry writer, a real run directory holding the
//! four artifacts, a real `TraceRingOwner` per rank, and a real publisher. The
//! recorder under test is `bag_record_with_manager` — the same entry
//! `cerulion bag record` calls.
//!
//! ISOLATED per-test SHM roots, so the registry gather sees exactly the run this
//! test published and nothing a sibling test or a live desk daemon is doing.
//!
//! LOAD DISCIPLINE. Every wait is a bounded CONDITION on a quantity load can
//! delay but not invert — a file appearing, a counter reaching a floor — with a
//! seconds-scale deadline and a panic that names what never happened. No arm
//! sleeps for a fixed settle, and no arm states a wall in units of anything it
//! is measuring.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_cli_engine::bag_cmd::{bag_record_with_manager, RecordOptions, RunTarget};
use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::trace_ring::{
    TraceRingOwner, TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_FIRE,
    RECORD_TYPE_STEP_BOUNDARY,
};
use cerulion_core::transport::run_registry::{RunHandle, RunRecord, RunState};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager, VirtualClock};

/// A stable hash for the hand-built frames — the value the bag's channel must
/// carry, and the one a hand oracle can therefore assert.
const HASH: u64 = 0x0981_0981_0981_0981;

/// The worker ring's capacity, in records. Small ON PURPOSE: the headline arm
/// pushes well past it so the ring LAPS, which is the state a mid-run attach
/// must survive and an at-zero one cannot.
const RING_CAP: u32 = 16;

/// The departure-ring sentinel rank.
const DEPARTURE_RANK: u32 = u32::MAX;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn nanos() -> u128 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos()
}

/// A per-run ring tag that fits macOS's 31-byte POSIX shm name limit.
///
/// Base-36 low-order nanos plus a process-local counter: short enough to open,
/// unique enough that a previous run's orphan is never re-opened.
fn short_tag(kind: &str) -> String {
    let mut n = (nanos() as u64) % 36u64.pow(7);
    let mut buf = String::new();
    for _ in 0..7 {
        let d = (n % 36) as u32;
        buf.push(char::from_digit(d, 36).expect("base 36"));
        n /= 36;
    }
    format!(
        "ra{kind}{buf}{}",
        COUNTER.fetch_add(1, Ordering::Relaxed) % 100
    )
}

fn unique(base: &str) -> String {
    format!(
        "{base}_{}_{}",
        nanos(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

/// Wait until `cond` holds, or fail naming what never happened.
///
/// A generous seconds-scale liveness ceiling against milliseconds of work: load
/// can make the condition arrive later, never make it arrive wrong.
fn await_condition(what: &str, deadline: Duration, mut cond: impl FnMut() -> bool) {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if cond() {
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("timed out after {deadline:?} waiting for: {what}");
}

/// A hand-built wire frame whose payload IS its sequence number, so a recorded
/// frame can be checked against an oracle computed from nothing but its index.
fn frame(seq: u32) -> Vec<u8> {
    let payload = seq.to_le_bytes();
    let header = WireHeader {
        schema_hash: HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 1_000 + u64::from(seq),
    };
    let mut buf = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(&payload);
    buf
}

fn fire(step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        record_type: RECORD_TYPE_FIRE,
        ..TraceRingRecord::default()
    }
}

fn boundary(step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        record_type: RECORD_TYPE_STEP_BOUNDARY,
        ..TraceRingRecord::default()
    }
}

fn departure(rank: u32) -> TraceRingRecord {
    TraceRingRecord {
        step: 0,
        record_type: RECORD_TYPE_DEPARTURE,
        node_idx: rank,
        ..TraceRingRecord::default()
    }
}

/// The live-run fixture: a registry writer, a run directory, and the rings the
/// run declares — assembled from the production types, not modelled.
struct LiveRun {
    _handle: RunHandle,
    _dir: tempfile::TempDir,
    run_id: u128,
}

/// Publish a run on `config`'s namespace and write its run directory.
///
/// `topics` is what the run's effective `graph.yaml` declares — the set
/// `--run` selects — and `rings` what its `run.json` declares.
fn publish_run(config: &iceoryx2::config::Config, topics: &[&str], rings: &[String]) -> LiveRun {
    publish_run_with_state_consumer(config, topics, rings, None)
}

/// `publish_run`, plus a hand-written `state_ring_consumer` value.
///
/// The state-consumer arms need a manifest that CLAIMS a standing window
/// recorder, and the only way to make a real one is to run a real graph — which
/// is `cerulion_cli`'s job, not this file's. What is under test HERE is the
/// reader and the gate it drives, so the manifest is hand-written exactly as
/// every other fact in this helper is. `None` writes NO key, which is both the
/// shape of a manifest that predates the key and the arm the legacy verdict covers.
fn publish_run_with_state_consumer(
    config: &iceoryx2::config::Config,
    topics: &[&str],
    rings: &[String],
    state_ring_consumer: Option<&str>,
) -> LiveRun {
    let dir = tempfile::tempdir().expect("run dir");
    let run_id: u128 = nanos() ^ 0x9810_0000_0000_0000;

    // The run's EFFECTIVE graph. `--run` resolves its topic set from exactly
    // this, so the `topic:` override is what makes the oracle a hand-written
    // name rather than a prefix the test also computes.
    let mut graph = String::from("name: rarun\nprefix: rarun\nnodes:\n");
    for (i, topic) in topics.iter().enumerate() {
        graph.push_str(&format!(
            "  - id: src{i}\n    type: src\n    outputs:\n      - name: out\n        \
             schema: std_msgs/String\n        topic: {topic}\n"
        ));
    }
    std::fs::write(dir.path().join("graph.yaml"), graph).expect("graph.yaml");
    std::fs::write(dir.path().join("env.json"), br#"{"PROBE":"ra"}"#).expect("env.json");
    std::fs::write(dir.path().join("recorder.json"), br#"{"arch":"test"}"#).expect("recorder.json");
    let ring_entries: Vec<serde_json::Value> = rings
        .iter()
        .enumerate()
        .map(|(i, tag)| serde_json::json!({ "tag": tag, "rank": i }))
        .collect();
    std::fs::write(dir.path().join("run.json"), {
        let mut manifest = serde_json::json!({
            "version": 1,
            "run_id": format!("0x{run_id:032x}"),
            "graph_name": "rarun",
            "rings": ring_entries,
        });
        if let (Some(obj), Some(state)) = (manifest.as_object_mut(), state_ring_consumer) {
            obj.insert(
                "state_ring_consumer".to_string(),
                serde_json::Value::String(state.to_string()),
            );
        }
        serde_json::to_vec_pretty(&manifest).expect("run.json")
    })
    .expect("run.json");

    let handle = RunHandle::publish_on_config(
        config,
        RunRecord {
            run_id,
            supervisor_pid: std::process::id(),
            run_started_at_ns: nanos() as u64,
            state: RunState::Live,
            graph_name: "rarun".to_string(),
            run_dir: dir.path().display().to_string(),
        },
    )
    .expect("publish the run on the registry");

    LiveRun {
        _handle: handle,
        _dir: dir,
        run_id,
    }
}

/// A bag's frames for `topic`, in order, as raw payload bytes.
///
/// Reads through `recover_messages`, the TORN-TAIL walk, rather than the
/// index-walking `messages()`. That is what makes it total over a bag in either
/// state: `messages()` needs the SUMMARY index, which exists only after
/// finalize, while `recover_messages` walks the chunks and returns whatever is
/// durable. Every caller here happens to read AFTER finalize — the arms assert
/// on the finished bag on purpose, since an open bag's frames only become
/// readable when a chunk closes — but the helper does not require it, and a
/// mid-run diagnostic can use it as-is.
///
/// (The mechanism works on an open bag; the callers here pass a finalized one.)
fn recorded_payloads(bag: &Path, topic: &str) -> Vec<Vec<u8>> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let Ok((msgs, _)) = reader.recover_messages() else {
        return Vec::new();
    };
    msgs.into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data[WireHeader::SIZE..].to_vec())
        .collect()
}

/// The bag's `record_coverage.json`, decoded.
fn read_coverage(bag: &Path) -> cerulion_bagd::RecordCoverage {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let a = reader
        .attachment(cerulion_bagd::RECORD_COVERAGE_ATTACHMENT)
        .expect("read attachments")
        .expect("record_coverage.json is in EVERY finalized bag");
    serde_json::from_slice(&a.data).expect("a decodable coverage manifest")
}

/// Every trace record in a finalized bag, in order.
fn recorded_trace(bag: &Path) -> Vec<TraceRingRecord> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let Ok((msgs, _)) = reader.recover_messages() else {
        return Vec::new();
    };
    msgs.into_iter()
        .filter(|m| m.topic == cerulion_bag::SCHEDULER_TRACE_TOPIC)
        .map(|m| {
            TraceRingRecord::from_bytes(m.data[..40].try_into().expect("a 40-byte trace record"))
        })
        .collect()
}

/// THE HEADLINE: a mid-run attach to a ring that has ALREADY LAPPED records the
/// trace from a COMPLETE step, and the departure ring passes through.
///
/// Three claims, none of which any pure arm can make:
///
/// 1. **The lapped ring is attachable at all.** The worker ring is pushed well
///    past its capacity before the recorder arms, so `open` would return
///    `Overrun` and the recording would fail outright.
/// 2. **The partial head step is discarded** — the armed gate drops the FIREs
///    whose boundary was lapped away, so the bag's trace begins at a boundary
///    and never at a headless fire.
/// 3. **The departure ring passes through** — its records reach the bag even
///    though it carries no boundary for a gate to open on. This is the decision
///    under test, driven through the production `ring_head_gate` rather than
///    asserted about it.
#[test]
fn a_mid_run_attach_records_a_lapped_ring_from_a_complete_step_and_keeps_departures() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_attach_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let topic = format!("/{}", unique("ra_attach"));
    // Ring TAGS, not resolved SHM names — the recorder resolves them through
    // `ring_shm_name`, exactly as `graph run --record` does, so this fixture
    // exercises that boundary rather than stepping around it. They are kept
    // short because a tag reaching `shm_open` unresolved is precisely the bug
    // that boundary prevents, and a SHORT tag makes that failure ENOENT rather
    // than macOS ENAMETOOLONG — one error kind instead of two.
    let worker_tag = short_tag("w");
    let dep_tag = short_tag("d");

    // ---- The run's rings, written BEFORE the recorder arms (the workers own
    // them; a recorder only ever attaches to rings that already exist).
    let mut worker_owner =
        TraceRingOwner::create(&worker_tag, RING_CAP, 0, &["src"]).expect("worker ring");
    let mut dep_owner =
        TraceRingOwner::create(&dep_tag, RING_CAP, DEPARTURE_RANK, &[]).expect("dep ring");
    // The owners stay alive for the whole arm: dropping one UNLINKS its SHM
    // name, and the recorder's mapping is what would then be reading a ring
    // nothing can find.
    let mut worker = worker_owner.producer().expect("the single producer");
    let mut dep = dep_owner.producer().expect("the single producer");

    // LAP the worker ring: 3x its capacity of a run the recorder will never see.
    // This is what makes the arm a mid-run attach rather than a late at-zero one.
    for step in 0..(RING_CAP as u64 * 3) {
        worker.push(&boundary(step));
        worker.push(&fire(step));
    }
    let lapped_through = worker.pushed();
    assert!(
        lapped_through > u64::from(RING_CAP),
        "the ring must have LAPPED before the attach, else this arm is an \
         at-zero attach wearing a mid-run name: pushed {lapped_through} into a \
         {RING_CAP}-record ring"
    );

    let run = publish_run(&ix, &[&topic], &[worker_tag.clone(), dep_tag.clone()]);

    // An UNDECLARED live producer. `--run`
    // derives its set from `config.nodes[].outputs[]` (the INFERRED universe,
    // already proven insufficient), so live-service discovery must be ON here
    // and this topic must reach the bag even though the run's graph never
    // mentions it. On the flagship `ros2 attach` shape that inference names four
    // topics while ~71 bridge routes stream unrecorded; this is that shape in
    // miniature.
    let undeclared = format!("/{}", unique("ra_undeclared"));
    let undeclared_pub = manager
        .create_publisher(&undeclared, MaxSliceLen::const_new(256), 0)
        .expect("an undeclared live producer");

    // ---- A live producer on the topic the run declares.
    //
    // STREAMING, not one-shot, and that is forced by the recorder's own shape
    // rather than chosen: its taps are listener-less `DataOnlySubscriber`s that
    // request NO late-joiner history, so a frame published before the tap
    // attaches is simply not in the bag — while the bag cannot be created until
    // a frame has taught the tap its schema. A one-shot publish sits on the
    // wrong side of that circle; a live run streams, so the fixture does too.
    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_topic = topic.clone();
    let produce_mgr = Arc::clone(&manager);
    let published = Arc::new(AtomicU64::new(0));
    let published_count = Arc::clone(&published);
    let producer = std::thread::spawn(move || {
        let mut undeclared_pub = undeclared_pub;
        let mut publisher = produce_mgr
            .create_publisher(&produce_topic, MaxSliceLen::const_new(256), 0)
            .expect("producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&frame(seq)).expect("publish");
            // The UNDECLARED producer streams too — a discovered tap requests no
            // late-joiner history, so a one-shot publish could land before the
            // rescan attaches and prove nothing.
            undeclared_pub.publish_raw(&frame(seq)).expect("publish");
            published_count.store(u64::from(seq) + 1, Ordering::Relaxed);
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    // ---- Arm the recorder on a helper thread.
    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique("ra_attach_bag")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    let recorder = std::thread::spawn(move || {
        let mut sink = Vec::new();
        bag_record_with_manager(
            manager,
            RecordOptions {
                run: Some(RunTarget::Sole),
                out: bag_path,
                ..RecordOptions::default()
            },
            stop,
            &mut sink,
        )
        .map(|s| (s, String::from_utf8_lossy(&sink).into_owned()))
    });

    // ---- The MID-RUN half: everything below happens while the recorder is
    // already attached, so it is exactly what a mid-run attach can see.
    //
    // The ring records must land AFTER the recorder has opened the ring at its
    // live cursor, or they would sit BEHIND that cursor and the arm would
    // measure nothing. The ring is opened in `Recorder::setup`, which runs
    // strictly before `ensure_writer`, so the bag file's appearance is a sound
    // (and observable) proxy for "the ring is attached".
    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    if recorder.is_finished() {
        panic!(
            "the recorder exited before creating a bag — its own error is the \
             failure: {:?}",
            recorder.join()
        );
    }

    // A PARTIAL head step first — two FIREs whose boundary is already lapped
    // away — then the first COMPLETE step. The gate must drop the former.
    const FIRST_COMPLETE_STEP: u64 = 900;
    worker.push(&fire(FIRST_COMPLETE_STEP - 1));
    worker.push(&fire(FIRST_COMPLETE_STEP - 1));
    worker.push(&boundary(FIRST_COMPLETE_STEP));
    worker.push(&fire(FIRST_COMPLETE_STEP));
    worker.push(&boundary(FIRST_COMPLETE_STEP + 1));
    dep.push(&departure(7));

    // Stop, and assert on the FINALIZED bag.
    //
    // Deliberately not a mid-run poll: an open bag's frames become readable only
    // once a chunk closes (4 MiB or the chunk time floor), so a wall-bounded
    // wait for them would be timing the flush cadence rather than the property
    // under test. Shutdown is the guarantee that needs no timing, and the
    // mechanism is TWO-SIDED rather than one loop: the drive loop drains the
    // TAPS until a pass yields nothing, while the trace RINGS are drained by
    // the WRITER THREAD, which owns the consumers and `writev`s straight out of
    // ring SHM (the drive loop's own `drain_rings` served the deleted inline
    // path and is gone). `finalize` joins that thread, so both tails are inside
    // the window — and the flag is flipped strictly AFTER the pushes above, so
    // everything written here is covered.
    running.store(false, Ordering::Relaxed);
    let (summary, stdout) = recorder
        .join()
        .expect("the recorder thread must not panic")
        .expect("the recording must succeed — a lapped ring must NOT Overrun");
    producing.store(false, Ordering::Relaxed);
    producer.join().expect("the producer thread must not panic");
    assert!(
        published.load(Ordering::Relaxed) > 0,
        "the fixture published nothing, so an empty bag would prove nothing"
    );

    let bag = summary.bag_paths.first().expect("one bag file").clone();

    // ---- CLAIM 1: the run was attached to, and its DECLARED topic selected.
    assert!(
        stdout.contains(&format!("0x{:032x}", run.run_id)),
        "the verb must report which run it attached to:\n{stdout}"
    );
    assert!(
        stdout.contains(&topic),
        "the recorded set must be the topic the RUN declares:\n{stdout}"
    );

    // ---- CLAIM 2: the frames are byte-identical to a hand oracle.
    let got = recorded_payloads(&bag, &topic);
    assert!(
        !got.is_empty(),
        "a mid-run attach to a live producer must record its frames; the \
         producer published {} and the bag holds none",
        published.load(Ordering::Relaxed)
    );
    // The payload IS the sequence, so each frame carries its own oracle: the
    // recorded window must be a CONTIGUOUS ASCENDING run of the stream. A
    // dropped, duplicated or reordered frame fails; the exact start is not
    // asserted, because where a mid-run attach begins is what it attaches to,
    // not something a test can fix without racing the recorder.
    let first = u32::from_le_bytes(
        got[0]
            .as_slice()
            .try_into()
            .expect("a 4-byte sequence payload"),
    );
    for (i, payload) in got.iter().enumerate() {
        assert_eq!(
            payload.as_slice(),
            first.wrapping_add(i as u32).to_le_bytes().as_slice(),
            "frame {i} of the recorded window must be sequence {}, so the window \
             is a contiguous run of the stream — a drop, a duplicate or a \
             reorder all fail here",
            first as usize + i
        );
    }

    // ---- CLAIM 3: the trace begins at a COMPLETE step.
    let trace = recorded_trace(&bag);
    assert!(!trace.is_empty(), "a declared ring must contribute a trace");
    let worker_records: Vec<&TraceRingRecord> = trace
        .iter()
        .filter(|r| r.record_type != RECORD_TYPE_DEPARTURE)
        .collect();
    assert!(
        !worker_records.is_empty(),
        "the worker ring must contribute records"
    );
    assert_eq!(
        worker_records[0].record_type, RECORD_TYPE_STEP_BOUNDARY,
        "the trace must OPEN on a step boundary — a headless FIRE at record 1 \
         makes the step skeleton wrong from its first record. Got {:?}",
        worker_records[0]
    );
    assert_eq!(
        worker_records[0].step, FIRST_COMPLETE_STEP,
        "and it must be the first COMPLETE step, not one of the two FIREs whose \
         boundary was lapped away"
    );
    // The EXACT record sequence, not merely
    // its head.
    //
    // `drain_rings` commits `n_consumed` (records READ) while writing
    // `n_records` (records ADMITTED after the head-step gate trimmed the
    // partial head). Committing the written count instead — which no
    // head-only test can see — under-advances the read cursor by
    // exactly the trimmed count, so the NEXT drain re-serves that suffix and
    // writes it AGAIN. The bag then carries a duplicated tail with a correct
    // head, which no head-only assertion can see. This is the whole reachable
    // worker trace, so a duplicate has nowhere to hide.
    let shape: Vec<(u32, u64)> = worker_records
        .iter()
        .map(|r| (r.record_type, r.step))
        .collect();
    assert_eq!(
        shape,
        vec![
            (RECORD_TYPE_STEP_BOUNDARY, FIRST_COMPLETE_STEP),
            (RECORD_TYPE_FIRE, FIRST_COMPLETE_STEP),
            (RECORD_TYPE_STEP_BOUNDARY, FIRST_COMPLETE_STEP + 1),
        ],
        "the worker trace must be EXACTLY the post-attach records that survive the gate — the two \
         headless FIREs of step {} dropped, everything after them written ONCE",
        FIRST_COMPLETE_STEP - 1
    );

    // ---- CLAIM 4 (the decision): the DEPARTURE ring passed through.
    let departures: Vec<&TraceRingRecord> = trace
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_DEPARTURE)
        .collect();
    assert_eq!(
        departures.len(),
        1,
        "the departure ring carries no STEP_BOUNDARY, so an ARMED gate would \
         discard its every record forever — the whole reason it is passed \
         through. Trace was: {:?}",
        trace
            .iter()
            .map(|r| (r.step, r.record_type))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        departures[0].node_idx, 7,
        "the departed rank must survive verbatim"
    );

    // ---- CLAIM 5: the bag says it attached mid-run, and carries the run.
    let reader = cerulion_bag::BagReader::open(&bag).expect("open");
    let run_json = reader
        .attachment("__cerulion/run.json")
        .expect("read attachments")
        .expect("a mid-run bag MUST carry its run identity");
    let doc: serde_json::Value = serde_json::from_slice(&run_json.data).expect("valid JSON");
    assert_eq!(doc["attached_mid_run"], true);
    assert_eq!(doc["run_id"], format!("0x{:032x}", run.run_id));
    for name in ["graph.yaml", "env.json", "__cerulion/recorder.json"] {
        assert!(
            reader.attachment(name).expect("read attachments").is_some(),
            "an attached bag must carry `{name}` from the run directory"
        );
    }

    // ---- CLAIM 6: the trace VERDICT is chosen from what the run
    // actually declared. The call-site `if rings.is_empty()` is pinned here
    // — the 15 pure arms cover the RENDER — and it is
    // reachable both ways on real shapes. This run declared rings, so:
    assert_eq!(
        doc["trace"],
        serde_json::Value::String(cerulion_cli_engine::bag_cmd::TRACE_FROM_ATTACH.to_string()),
        "a run that declared rings must take the FROM-ATTACH arm"
    );

    // ---- CLAIM 7: every DECLARED channel says it attached LATE.
    //
    // `open_tap` is handed `cfg.attached_mid_run` for exactly this, and
    // hardcoding it `false` survived the entire suite: no arm read the marker.
    // It is the per-channel half of `attached_mid_run` — a reader diffing this
    // bag against a from-step-0 one needs to know the head is missing.
    let coverage = read_coverage(&bag);
    let declared_tap = coverage
        .tapped
        .get(&topic)
        .unwrap_or_else(|| panic!("the declared topic must be tapped: {:?}", coverage.tapped));
    assert!(
        declared_tap.attached_late,
        "a mid-run attach covers its topic only from the attach point, and the channel must SAY so"
    );

    // ---- CLAIM 8: discovery ran, and it recorded the undeclared
    // producer. `--run` is an INFERRED universe (the discovery rule), so a
    // producer the run's declaration never named must still reach the bag.
    assert!(
        coverage.discovery_requested,
        "`--run` must request live-service discovery — its set is the static declaration an earlier \
         change proved insufficient"
    );
    assert!(
        coverage.tapped.contains_key(&undeclared),
        "an UNDECLARED live producer must be discovered and recorded; tapped set was {:?}",
        coverage.tapped.keys().collect::<Vec<_>>()
    );

    // ---- CLAIM 9: the networked schema rungs were armed.
    // `BagdConfig::new` leaves `schema_demand` at ZERO, so without the CLI
    // assembly threading it, the networked schema ladder is OFF for every `bag record` bag and
    // this flag reads `false`.
    assert!(
        coverage.schema_demand_requested,
        "an ATTACH recording must arm the schema-demand rungs — its channels are attach-mode by \
         construction and the run may carry types this desk never compiled"
    );

    let _ = std::fs::remove_file(&bag);
}

/// The CONTROL, and the reason the headline's claims are not vacuous: with NO
/// live run, the same verb records a plain topic recording with no run identity.
///
/// Without it, "the bag carries `run.json`" and "the trace opens on a boundary"
/// could both be satisfied by a recorder that always did those things.
#[test]
fn with_no_live_run_the_verb_records_standalone_and_says_so() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_standalone_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let topic = format!("/{}", unique("ra_standalone"));
    let mut publisher = manager
        .create_publisher(&topic, MaxSliceLen::const_new(256), 0)
        .expect("producer");
    publisher.publish_raw(&frame(0)).expect("publish");

    let out_path: PathBuf = std::env::temp_dir().join(format!("{}.mcap", unique("ra_standalone")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let topic_for_thread = topic.clone();
    let bag_path = out_path.clone();
    let recorder = std::thread::spawn(move || {
        let mut sink = Vec::new();
        bag_record_with_manager(
            manager,
            RecordOptions {
                // `--run` with nothing live falls back; the explicit topic is
                // what it falls back TO, and keeps this arm independent of the
                // declared-set path.
                run: Some(RunTarget::Sole),
                topics: vec![topic_for_thread],
                out: bag_path,
                ..RecordOptions::default()
            },
            stop,
            &mut sink,
        )
        .map(|s| (s, String::from_utf8_lossy(&sink).into_owned()))
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists(),
    );
    running.store(false, Ordering::Relaxed);
    let (summary, stdout) = recorder
        .join()
        .expect("no panic")
        .expect("a run-less machine must still record");

    assert!(
        stdout.contains("no live run"),
        "the fallback must be LOUD — a silently thinner bag is the whole thing \
         this notice exists to prevent:\n{stdout}"
    );
    let bag = summary.bag_paths.first().expect("one bag").clone();
    let reader = cerulion_bag::BagReader::open(&bag).expect("open");
    assert!(
        reader
            .attachment("__cerulion/run.json")
            .expect("read attachments")
            .is_none(),
        "a standalone recording must claim NO run identity"
    );
    assert!(
        reader
            .attachment("graph.yaml")
            .expect("read attachments")
            .is_none(),
        "and no graph it never attached to"
    );
    assert!(
        recorded_trace(&bag).is_empty(),
        "and no scheduler trace: it declared no rings"
    );

    // The NEGATIVE halves of the headline's per-channel claims, without which
    // each of them is satisfied by a recorder that always answers the same way.
    let coverage = read_coverage(&bag);
    let tap = coverage
        .tapped
        .get(&topic)
        .expect("the named topic must be tapped");
    assert!(
        !tap.attached_late,
        "a standalone recording did NOT attach to a run mid-flight, so its channel must not claim \
         it did"
    );
    assert!(
        !coverage.discovery_requested,
        "an EXPLICIT topic list is an operator's CHOSEN set — auto-adding the rest of the machine \
         underneath it would overrule them (the discovery rule)"
    );

    let _ = std::fs::remove_file(&bag);
}

/// **End to end:** a declared topic with NO producer costs that
/// topic, the live ones record, and the bag SAYS which was missing.
///
/// The pure arms pin `derive_run_topics`; this pins that the recorder is wired
/// to it. If the run-declared names were substituted into
/// `opts.topics` and reached the EXPLICIT branch, this exact shape — a
/// two-output graph with one node not yet publishing, i.e. the whole of
/// multi-process start-up and the permanent state of a `--peer-loss continue`
/// roster with a dead worker — would refuse the ENTIRE recording, with a remedy
/// ("record it ON the robot") that is already what the operator is doing.
#[test]
fn a_declared_topic_with_no_producer_does_not_refuse_the_whole_recording() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_degrade_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let live_topic = format!("/{}", unique("ra_deg_live"));
    let quiet_topic = format!("/{}", unique("ra_deg_quiet"));
    // The run DECLARES both; only one has a producer. Nothing ever creates a
    // service for the other — that is the condition under test, not a race.
    let run = publish_run(&ix, &[&live_topic, &quiet_topic], &[]);

    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_topic = live_topic.clone();
    let produce_mgr = Arc::clone(&manager);
    let producer = std::thread::spawn(move || {
        let mut publisher = produce_mgr
            .create_publisher(&produce_topic, MaxSliceLen::const_new(256), 0)
            .expect("producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&frame(seq)).expect("publish");
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique("ra_deg_bag")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    let recorder = std::thread::spawn(move || {
        let mut sink = Vec::new();
        bag_record_with_manager(
            manager,
            RecordOptions {
                run: Some(RunTarget::Sole),
                out: bag_path,
                ..RecordOptions::default()
            },
            stop,
            &mut sink,
        )
        .map(|s| (s, String::from_utf8_lossy(&sink).into_owned()))
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    if recorder.is_finished() {
        panic!(
            "the recorder exited instead of recording — one quiet DECLARED topic must never refuse \
             the run: {:?}",
            recorder.join()
        );
    }
    running.store(false, Ordering::Relaxed);
    let (summary, stdout) = recorder
        .join()
        .expect("no panic")
        .expect("a partially-live run must RECORD");
    producing.store(false, Ordering::Relaxed);
    producer.join().expect("no panic");

    let bag = summary.bag_paths.first().expect("one bag").clone();

    // The live topic recorded.
    assert!(
        !recorded_payloads(&bag, &live_topic).is_empty(),
        "the declared topic that IS producing must be in the bag"
    );

    // The quiet one is REPORTED — on stdout as it happens, and durably in the
    // manifest. The manifest half is the load-bearing one: a warn scrolls away,
    // and a bag that simply lacked the channel would claim by omission that the
    // run declared exactly what the bag contains (the discovery lesson, on the
    // selection side).
    assert!(
        stdout.contains(&quiet_topic),
        "the quiet declared topic must be named as it happens:\n{stdout}"
    );
    let coverage = read_coverage(&bag);
    assert_eq!(
        coverage.untapped.get(&quiet_topic),
        Some(&cerulion_bagd::UntappedReason::DeclaredNotLive),
        "the bag must record WHICH declared topic had no producer, and why; untapped was {:?}",
        coverage.untapped
    );
    assert!(
        !coverage.tapped.contains_key(&quiet_topic),
        "and it must not appear as a tapped channel"
    );

    // …and it is NOT escalated to a coverage GAP. A graph legitimately holds
    // outputs that have not fired, so counting one would make every ordinary
    // `--run` bag read INCOMPLETE — the exact over-reporting that trains an
    // operator to skim the number that matters.
    assert_eq!(
        coverage.gap_count(),
        0,
        "a declared-but-quiet output is accounted for, not a live producer missing from the bag"
    );

    // The run was really attached to (so this is the `--run` path, not a
    // standalone fallback that happened to record one topic).
    assert!(stdout.contains(&format!("0x{:032x}", run.run_id)));

    let _ = std::fs::remove_file(&bag);
}

/// **The other direction:** a run that declared NO rings gets the
/// no-rings verdict, not the from-attach one.
///
/// Paired with the headline's `TRACE_FROM_ATTACH` assertion, this pins the
/// call-site CHOICE (`if rings.is_empty()`) on both of its arms over real
/// shapes. Without run-declared rings only this arm is reachable — no manifest
/// ever carries rings — so the constant that names the OTHER outcome would be
/// unreachable prose.
#[test]
fn a_run_that_declared_no_rings_gets_the_no_rings_verdict() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_noring_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let topic = format!("/{}", unique("ra_noring"));
    let _run = publish_run(&ix, &[&topic], &[]);

    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_topic = topic.clone();
    let produce_mgr = Arc::clone(&manager);
    let producer = std::thread::spawn(move || {
        let mut publisher = produce_mgr
            .create_publisher(&produce_topic, MaxSliceLen::const_new(256), 0)
            .expect("producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&frame(seq)).expect("publish");
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique("ra_noring_bag")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    let recorder = std::thread::spawn(move || {
        bag_record_with_manager(
            manager,
            RecordOptions {
                run: Some(RunTarget::Sole),
                out: bag_path,
                ..RecordOptions::default()
            },
            stop,
            &mut Vec::new(),
        )
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    running.store(false, Ordering::Relaxed);
    let summary = recorder.join().expect("no panic").expect("must record");
    producing.store(false, Ordering::Relaxed);
    producer.join().expect("no panic");

    let bag = summary.bag_paths.first().expect("one bag").clone();
    let reader = cerulion_bag::BagReader::open(&bag).expect("open");
    let run_json = reader
        .attachment("__cerulion/run.json")
        .expect("read attachments")
        .expect("an attached bag carries its run identity");
    let doc: serde_json::Value = serde_json::from_slice(&run_json.data).expect("valid JSON");
    assert_eq!(
        doc["trace"],
        serde_json::Value::String(cerulion_cli_engine::bag_cmd::TRACE_NONE_NO_RINGS.to_string()),
        "a run whose manifest declared no rings must SAY that is why there is no trace, rather \
         than leave a reader to conclude the recorder lost one"
    );
    assert!(
        recorded_trace(&bag).is_empty(),
        "and the bag really must carry no trace, or the verdict is a lie"
    );

    let _ = std::fs::remove_file(&bag);
}

/// A DECLARED ring whose SHM segment is gone costs the TRACE, not
/// the recording.
///
/// The rings belong to another process, and its `TraceRingOwner::drop` unlinks
/// the SHM name the instant the run ends — so "the manifest names a ring that no
/// longer exists" is the ordinary way a recorder meets a run that is exiting,
/// not a corner. Refusing there threw away the frames too, which contradicts the
/// policy `read_run_artifacts` states one function away for the very same
/// directory: *an unreadable file is a fact to report, never a reason to abandon
/// a recording that is already the only copy of what is on the wire.*
///
/// The tag is never created at all, which is the same thing `shm_open` sees as a
/// ring that was unlinked — and it makes the arm deterministic rather than a
/// race against another process's teardown.
///
/// The asymmetry is deliberate and NOT tested here: on `graph run --record` the
/// rings are created by that process moments earlier, so a failed open is an
/// internal invariant violation and stays fatal. This arm is the ATTACH path.
#[test]
fn a_declared_ring_that_vanished_costs_the_trace_not_the_recording() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_ghostring_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let topic = format!("/{}", unique("ra_ghost"));
    // A ring TAG this test deliberately never creates. Short, so the failure is
    // a plain ENOENT rather than macOS's ENAMETOOLONG.
    let ghost_tag = short_tag("g");
    let _run = publish_run(&ix, &[&topic], std::slice::from_ref(&ghost_tag));

    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_topic = topic.clone();
    let produce_mgr = Arc::clone(&manager);
    let producer = std::thread::spawn(move || {
        let mut publisher = produce_mgr
            .create_publisher(&produce_topic, MaxSliceLen::const_new(256), 0)
            .expect("producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&frame(seq)).expect("publish");
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique("ra_ghost_bag")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    let recorder = std::thread::spawn(move || {
        bag_record_with_manager(
            manager,
            RecordOptions {
                run: Some(RunTarget::Sole),
                out: bag_path,
                ..RecordOptions::default()
            },
            stop,
            &mut Vec::new(),
        )
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    if recorder.is_finished() {
        panic!(
            "the recorder exited instead of recording — a vanished DECLARED ring \
             must cost its trace, never the frames: {:?}",
            recorder.join()
        );
    }
    running.store(false, Ordering::Relaxed);
    let summary = recorder
        .join()
        .expect("no panic")
        .expect("a vanished ring must NOT fail the recording");
    producing.store(false, Ordering::Relaxed);
    producer.join().expect("no panic");

    let bag = summary.bag_paths.first().expect("one bag").clone();
    assert!(
        !recorded_payloads(&bag, &topic).is_empty(),
        "the frames are the part that cannot be re-obtained later, so they must \
         be in the bag"
    );
    // …and the degrade is OBSERVABLE, not merely logged (Principle #3). A warn
    // scrolls away; this is the surface a caller can act on.
    let names: Vec<&str> = summary
        .rings_unavailable
        .iter()
        .map(|(n, _)| n.as_str())
        .collect();
    assert_eq!(
        names,
        vec![cerulion_core::shm_ring::ring_shm_name(&ghost_tag).as_str()],
        "the recorder must NAME the ring it could not open"
    );
    assert!(
        recorded_trace(&bag).is_empty(),
        "and carry no trace it never read"
    );

    // ---- The bag must SAY the trace is missing.
    //
    // The degrade costs the trace instead of the recording, but that alone is
    // not enough: the fact would reach `BagdSummary` (an in-process return
    // value) and a `warn!` (which scrolls away), and NO durable artifact.
    // Meanwhile `run.json`'s `trace` verdict is chosen when the bag is CREATED,
    // strictly before `Recorder::setup` opens a ring — so this bag claims the
    // trace was attached from the run's live cursor while holding ZERO trace
    // records, and only the coverage manifest contradicts it.
    //
    // Both surfaces are read IN ONE BODY on purpose: the point is not that the
    // manifest carries a field, it is that the manifest RECONCILES a claim the
    // frozen attachment cannot retract.
    let reader = cerulion_bag::BagReader::open(&bag).expect("open");
    let run_json = reader
        .attachment("__cerulion/run.json")
        .expect("read attachments")
        .expect("a mid-run bag MUST carry its run identity");
    let doc: serde_json::Value = serde_json::from_slice(&run_json.data).expect("valid JSON");
    assert_eq!(
        doc["trace"],
        serde_json::Value::String(cerulion_cli_engine::bag_cmd::TRACE_FROM_ATTACH.to_string()),
        "this is the CLAIM the coverage manifest exists to reconcile: the run DECLARED a ring, \
         so the verdict frozen at bag creation says the trace was attached — and \
         it is wrong, because the ring was gone by the time setup tried it"
    );

    let coverage = read_coverage(&bag);
    assert_eq!(
        coverage.rings_declared, 1,
        "the manifest must record how many rings were asked for, or a reader \
         cannot tell NO trace from a PARTIAL one"
    );
    let unavailable: Vec<&str> = coverage
        .rings_unavailable
        .keys()
        .map(String::as_str)
        .collect();
    assert_eq!(
        unavailable,
        vec![cerulion_core::shm_ring::ring_shm_name(&ghost_tag).as_str()],
        "the DURABLE artifact must name the ring, not only the in-process summary"
    );
    assert!(
        coverage.rings_unavailable.values().all(|e| !e.is_empty()),
        "and carry the transport's own error, which is what says WHY"
    );
    assert_eq!(coverage.rings_opened(), 0);
    assert!(coverage.trace_degraded());
    // A missing TRACE is deliberately NOT a coverage term: every arm of
    // `is_incomplete()` is about the PRODUCER picture or about whether what is
    // in the bag can be read, and this is neither. It takes its own line.
    //
    // Stated as a DIFFERENCE rather than an absolute, because this fixture's
    // frames carry a hand-built schema hash nothing can name, so the
    // descriptor term already holds — and `!is_incomplete()` would be asserting
    // something else entirely. What must be true is that clearing the ring
    // degrade changes NOTHING about the coverage verdict.
    let mut without_rings = coverage.clone();
    without_rings.rings_declared = 0;
    without_rings.rings_unavailable.clear();
    assert_eq!(
        coverage.is_incomplete(),
        without_rings.is_incomplete(),
        "the producers really are all in the bag; folding the trace into the \
         coverage verdict would make one head swallow the other's"
    );
    assert_eq!(coverage.gap_count(), 0, "and no producer is missing");

    // ---- …and `bag info` — the AUTHORITATIVE surface, since `run.json` cannot
    // be corrected in place — renders the outcome over this REAL manifest.
    let rendered = cerulion_cli_engine::bag_cmd::render_coverage(&coverage);
    assert!(
        rendered.contains("trace: NONE"),
        "zero of one ring opened must render NONE:\n{rendered}"
    );
    assert!(
        rendered.contains("run.json"),
        "and must reconcile itself with the frozen verdict:\n{rendered}"
    );

    // ---- COMPLETE is WITHHELD, and the assertion is written so the TRACE is
    // the reason.
    //
    // A bare `!rendered.contains("coverage:
    // COMPLETE")` here would be VACUOUS on this fixture: its frames
    // carry a hand-built schema hash nothing can name, so `schemas_unresolved`
    // already withholds COMPLETE for an unrelated reason and such an arm stays
    // GREEN even under a `drop && !coverage.trace_degraded()` change — only the
    // pure arm catches it, while this test's own name claims otherwise.
    //
    // Removing the confound at the fixture would mean publishing frames under a
    // resolvable built-in hash, which changes what the whole arm is recording
    // for the sake of one assertion. So it is removed AT THE ORACLE instead: the
    // schema term is cleared on a CLONE of the real manifest, leaving the trace
    // degrade as the only thing that can hold COMPLETE back. The clone is still
    // a real recorder-written manifest in every other field.
    let mut trace_only = coverage.clone();
    trace_only.schema_demand_requested = false;
    assert!(
        trace_only.trace_degraded() && trace_only.gap_count() == 0,
        "precondition: the clone must isolate the TRACE degrade, not remove it"
    );
    let rendered_trace_only = cerulion_cli_engine::bag_cmd::render_coverage(&trace_only);
    assert!(
        !rendered_trace_only.contains("coverage: COMPLETE"),
        "with the schema confound gone, the TRACE alone must still withhold \
         COMPLETE — that word is where an operator stops reading, and the trace \
         verdict sits below it:\n{rendered_trace_only}"
    );
    // …and the verdict is STATED rather than omitted: an absent
    // `coverage:` line already means "this caller opted out of discovery".
    assert!(
        rendered_trace_only.contains("coverage: NO GAPS"),
        "withholding COMPLETE must not withhold the verdict itself:\n{rendered_trace_only}"
    );
    // ANTI-TAUTOLOGY: clear the trace degrade too and COMPLETE must come back.
    // Without this, "COMPLETE is absent" is satisfied by a renderer that never
    // prints it, and the assertion above would pin nothing.
    let mut nothing_wrong = trace_only.clone();
    nothing_wrong.rings_declared = 0;
    nothing_wrong.rings_unavailable.clear();
    let rendered_clean = cerulion_cli_engine::bag_cmd::render_coverage(&nothing_wrong);
    assert!(
        rendered_clean.contains("coverage: COMPLETE"),
        "the TRACE was the only thing holding COMPLETE back, so clearing it must \
         restore the word — otherwise the pin above proves nothing:\n{rendered_clean}"
    );

    let _ = std::fs::remove_file(&bag);
}

/// `--exclude` survives DISCOVERY.
///
/// Two rules compose here. `--exclude` is meaningful on
/// the DECLARED half (`derive_run_topics` rule 3: an excluded topic is neither
/// recorded nor reported as missing) and, separately, live-service
/// discovery is ON for `--run` (the run-declared set is the INFERRED universe,
/// which is insufficient on its own). Each is right alone. Composed naively, the second
/// silently undoes the first: if `plan_discovery`'s only name filter is
/// `discovery_known`, which is seeded from the TAP set the exclusion has
/// already removed the topic from, the rescan re-finds the excluded topic
/// as an "undeclared live producer" and gives it a channel.
///
/// MEASURED in that shape: stdout announces ONE topic while the bag
/// holds the excluded one with 291 frames and `untapped = {}`.
///
/// Why this needs an e2e at all: every pure arm exercises ONE half. The
/// declared-side arms call `derive_run_topics` directly and never reach
/// discovery; the discovery arms call `plan_discovery` directly and never see a
/// `RecordOptions`. Only the real verb composes them.
#[test]
fn an_excluded_topic_stays_out_of_the_bag_even_though_discovery_is_on() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_exclude_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    // Two topics the run DECLARES, both with LIVE producers. The exclusion is
    // what must separate them — not liveness, and not the declaration.
    let keep = format!("/{}", unique("ra_keep"));
    let drop = format!("/{}", unique("ra_drop"));
    let _run = publish_run(&ix, &[&keep, &drop], &[]);

    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_mgr = Arc::clone(&manager);
    let (keep_topic, drop_topic) = (keep.clone(), drop.clone());
    let published = Arc::new(AtomicU64::new(0));
    let published_count = Arc::clone(&published);
    let producer = std::thread::spawn(move || {
        let mut keep_pub = produce_mgr
            .create_publisher(&keep_topic, MaxSliceLen::const_new(256), 0)
            .expect("keep producer");
        let mut drop_pub = produce_mgr
            .create_publisher(&drop_topic, MaxSliceLen::const_new(256), 0)
            .expect("drop producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            keep_pub.publish_raw(&frame(seq)).expect("publish");
            // The excluded topic STREAMS, and that is the whole fixture: a
            // one-shot publish could land before the rescan and prove nothing
            // about a filter the rescan is supposed to apply.
            drop_pub.publish_raw(&frame(seq)).expect("publish");
            published_count.store(u64::from(seq) + 1, Ordering::Relaxed);
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique("ra_exclude_bag")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    // The pattern matches the excluded topic's unique name and nothing else —
    // an over-broad pattern would take `/keep` out too and the arm would pass
    // for the wrong reason.
    let pattern = drop.trim_start_matches('/').to_string();
    let recorder = std::thread::spawn(move || {
        let mut sink = Vec::new();
        bag_record_with_manager(
            manager,
            RecordOptions {
                run: Some(RunTarget::Sole),
                out: bag_path,
                exclude: vec![pattern],
                ..RecordOptions::default()
            },
            stop,
            &mut sink,
        )
        .map(|s| (s, String::from_utf8_lossy(&sink).into_owned()))
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    if recorder.is_finished() {
        panic!(
            "the recorder exited before creating a bag — its own error is the \
             failure: {:?}",
            recorder.join()
        );
    }
    // Give the DISCOVERY RESCAN room to run: the defect is a rescan that
    // re-adds the excluded topic, so an arm that stops before one has
    // happened would pass against broken code. The condition is the
    // recorder's own settle floor expiring, observed as the bag existing plus a
    // producer that has kept streaming — never a wall in units of the 250 ms
    // cadence.
    let before = published.load(Ordering::Relaxed);
    await_condition(
        "the producer to stream past the recorder's discovery settle window",
        Duration::from_secs(30),
        || published.load(Ordering::Relaxed) > before + 60,
    );

    running.store(false, Ordering::Relaxed);
    let (summary, stdout) = recorder
        .join()
        .expect("the recorder thread must not panic")
        .expect("an excluded topic must not fail the recording");
    producing.store(false, Ordering::Relaxed);
    producer.join().expect("the producer thread must not panic");

    let bag = summary.bag_paths.first().expect("one bag file").clone();

    // ---- THE PIN: zero frames of the excluded topic, on a recording where
    // discovery genuinely ran.
    let dropped = recorded_payloads(&bag, &drop);
    assert!(
        dropped.is_empty(),
        "`--exclude` must keep the topic OUT of the bag; got {} frame(s) of \
         `{drop}` (without the exclusion filter in discovery, the rescan \
         re-adds the topic and 291 frames land here)",
        dropped.len()
    );
    // ANTI-TAUTOLOGY: the sibling topic IS recorded, so "no frames" is about
    // the exclusion rather than about a recorder that captured nothing.
    assert!(
        !recorded_payloads(&bag, &keep).is_empty(),
        "the un-excluded declared topic must still record — the producer \
         published {} frame(s)",
        published.load(Ordering::Relaxed)
    );
    assert!(
        stdout.contains(&keep) && !stdout.contains(&drop),
        "the verb must announce only what it records:\n{stdout}"
    );

    let coverage = read_coverage(&bag);
    // The SECOND anti-tautology, and the one that makes this arm probative:
    // discovery must have been ON. With it off the topic would be absent for a
    // reason that has nothing to do with `--exclude`, and this arm would
    // prove nothing.
    assert!(
        coverage.discovery_requested && coverage.enumerated,
        "this arm is only probative while discovery is ON and RAN: got \
         requested={} enumerated={}",
        coverage.discovery_requested,
        coverage.enumerated
    );
    assert!(
        !coverage.tapped.contains_key(&drop),
        "an excluded topic must get no channel at all: {:?}",
        coverage.tapped.keys().collect::<Vec<_>>()
    );

    // ---- ACCOUNTED FOR, not silent. `derive_run_topics`' rule 3 says an
    // excluded topic is not reported as MISSING, and this is not that report: a
    // row here says the recorder SAW a live producer and deliberately did not
    // tap it, exactly as `excluded_internal` and `remote_mirror` do. Dropping it
    // in silence would break the one claim `enumerated: true` makes.
    match coverage.untapped.get(&drop) {
        Some(cerulion_bagd::UntappedReason::ExcludedByRequest { pattern }) => {
            assert!(
                drop.contains(pattern.as_str()),
                "the row must name the pattern that matched, or an operator with \
                 several `--exclude`s cannot tell which one caught it: {pattern}"
            );
        }
        other => panic!(
            "the excluded live producer must be accounted for as \
             `excluded_by_request`; got {other:?}"
        ),
    }
    // …and it is NOT a coverage gap: the operator asked for it to be out, so a
    // recording that used `--exclude` for its intended purpose must not WARN.
    assert_eq!(
        coverage.gap_count(),
        0,
        "an operator's own exclusion is not a coverage gap: {:?}",
        coverage.untapped
    );
    // …stated as a DIFFERENCE for the reason the sibling ring arm documents:
    // this fixture's hand-built schema hash already trips the
    // descriptor term, so the claim that can be made — and the one that
    // matters — is that the exclusion ROW contributes nothing.
    let mut without_exclusion = coverage.clone();
    without_exclusion.untapped.remove(&drop);
    assert_eq!(
        coverage.is_incomplete(),
        without_exclusion.is_incomplete(),
        "a recording that used `--exclude` for its intended purpose must not be \
         escalated by its own exclusion"
    );

    let _ = std::fs::remove_file(&bag);
}

/// A run whose MANIFEST could not be read says the trace is **UNKNOWN** — never
/// that the run "declared no trace rings".
///
/// `RunArtifacts::rings` reads exclusively from `run.json`, so an unreadable
/// manifest yields an empty ring vector that is byte-identical to the one a
/// manifest genuinely declaring none produces. Keying the verdict on that vector
/// alone stamped a claim about the RUN — *its manifest declared no trace rings*
/// — onto a bag whose recorder never opened the manifest. Absence is a no-claim
/// everywhere else in this attachment (`run` is simply omitted when the manifest
/// is missing, per `a_missing_run_manifest_still_yields_the_recorders_own_identity`);
/// this was the one field that filled the hole with a cause instead.
///
/// It is not a corner. A run directory is removed by `RunDescriptor::drop`, and
/// an `Ending` run is DELIBERATELY still attachable (`an_ending_run_is_still_a_candidate`
/// — refusing it would make the verb's behaviour depend on a race with the run's
/// own teardown), so the recorder meets a half-removed directory on the ordinary
/// path.
///
/// **Nothing else corrects it, which is why the accuracy has to be in the string.**
/// The reconciliation built for a frozen `trace` verdict —
/// `record_coverage.json`'s `rings_unavailable` and `bag info`'s
/// `trace: NONE|PARTIAL` line — is gated on `RecordCoverage::trace_degraded()`,
/// i.e. `!rings_unavailable.is_empty()`. With no manifest there are ZERO declared
/// rings, so nothing is ever recorded as unavailable and that corrective line
/// never fires.
///
/// This arm exists SEPARATELY from the two pure oracles in `bag_cmd` because
/// those re-derive the selector expression and are therefore structurally blind
/// to a production call site that passes the wrong one — the inert-shipping
/// class. Only `run.json` is removed, so `graph.yaml` stays readable and the
/// default `--run` topic resolution is what runs (a missing `graph.yaml` is a
/// hard refusal one function earlier, a different path entirely).
#[test]
fn an_unreadable_run_manifest_says_the_trace_is_unknown_not_that_none_was_declared() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_nomanifest_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let topic = format!("/{}", unique("ra_nomanifest"));
    // Published WITH a ring declared, so the manifest we then delete is one that
    // had something to say. That is what makes the two wrong answers
    // distinguishable from the right one: a wrong answer claims the run
    // "declared no trace rings" about a manifest that declared one.
    let ring_tag = short_tag("nm");
    let run = publish_run(&ix, &[&topic], std::slice::from_ref(&ring_tag));

    // Remove ONLY the manifest — exactly what a reader sees part-way through
    // `RunDescriptor::drop`'s `remove_dir_all`. The other three artifacts stay,
    // which is also this arm's internal anti-tautology: if the recorder had
    // simply failed to read the whole directory, they would be missing too.
    let run_dir = {
        let gathered = cerulion_core::transport::run_registry::gather_runs_on_config(
            &ix,
            Duration::from_secs(5),
        )
        .expect("gather the run we just published");
        let rec = gathered
            .records
            .iter()
            .find(|r| r.run_id == run.run_id)
            .unwrap_or_else(|| panic!("the published run must be gatherable: {gathered:?}"));
        PathBuf::from(rec.run_dir.clone())
    };
    std::fs::remove_file(run_dir.join("run.json")).expect("remove the run manifest");
    assert!(
        run_dir.join("graph.yaml").exists(),
        "precondition: only the MANIFEST is gone — the topic set still resolves"
    );

    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_topic = topic.clone();
    let produce_mgr = Arc::clone(&manager);
    let producer = std::thread::spawn(move || {
        let mut publisher = produce_mgr
            .create_publisher(&produce_topic, MaxSliceLen::const_new(256), 0)
            .expect("producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&frame(seq)).expect("publish");
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique("ra_nomanifest_bag")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    let recorder = std::thread::spawn(move || {
        bag_record_with_manager(
            manager,
            RecordOptions {
                run: Some(RunTarget::Sole),
                out: bag_path,
                ..RecordOptions::default()
            },
            stop,
            &mut Vec::new(),
        )
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    running.store(false, Ordering::Relaxed);
    let summary = recorder
        .join()
        .expect("no panic")
        .expect("an unreadable manifest must cost the TRACE, never the recording");
    producing.store(false, Ordering::Relaxed);
    producer.join().expect("no panic");

    let bag = summary.bag_paths.first().expect("one bag").clone();
    let reader = cerulion_bag::BagReader::open(&bag).expect("open");
    let run_json = reader
        .attachment("__cerulion/run.json")
        .expect("read attachments")
        .expect("an attached bag carries its run identity even with no manifest");
    let doc: serde_json::Value = serde_json::from_slice(&run_json.data).expect("valid JSON");

    assert_eq!(
        doc["trace"],
        serde_json::Value::String(
            cerulion_cli_engine::bag_cmd::TRACE_UNKNOWN_NO_MANIFEST.to_string()
        ),
        "the recorder never read this run's manifest, so what it declared is UNKNOWN"
    );
    assert_ne!(
        doc["trace"],
        serde_json::Value::String(cerulion_cli_engine::bag_cmd::TRACE_NONE_NO_RINGS.to_string()),
        "THE discrimination: this manifest DECLARED a ring — saying the run \
         declared none states a fact about the run drawn from the recorder's own \
         failure to read it, and no other surface contradicts it"
    );
    assert!(
        doc["artifacts_unreadable"]["run.json"]
            .as_str()
            .is_some_and(|reason| !reason.is_empty()),
        "the bag must carry WHAT failed and WHY — a `warn!` scrolls away, the bag \
         does not. Got: {}",
        doc["artifacts_unreadable"]
    );
    // Absence stays NO-CLAIM: nothing invents a `run` object to fill the hole.
    assert!(
        doc.get("run").is_none() && doc.get("run_manifest_unparsed").is_none(),
        "a missing manifest is carried as an absence, not a fabrication"
    );
    // ANTI-TAUTOLOGY, in body: the OTHER three artifacts were readable and are
    // in the bag, so `artifacts_unreadable` names exactly what was lost rather
    // than the recorder having missed the directory wholesale.
    for present in ["graph.yaml", "env.json", "__cerulion/recorder.json"] {
        assert!(
            reader
                .attachment(present)
                .expect("read attachments")
                .is_some(),
            "{present} was readable and must still be attached — otherwise this \
             arm cannot tell a lost MANIFEST from a lost DIRECTORY"
        );
    }
    assert!(
        doc["artifacts_unreadable"].get("graph.yaml").is_none(),
        "only what actually failed may be named"
    );
    // Degrade, never fail: the frames are the part that cannot be obtained later.
    assert!(
        !recorded_payloads(&bag, &topic).is_empty(),
        "an unreadable manifest costs the TRACE, not the recording"
    );
    assert!(
        recorded_trace(&bag).is_empty(),
        "and there really is no trace, or the verdict is a lie"
    );

    let _ = std::fs::remove_file(&bag);
}

/// A run whose manifest was READ but will not PARSE says the trace is
/// **UNKNOWN** — never that the run "declared no trace rings".
///
/// The FOURTH state, and the one the three-way selector missed. Its siblings are
/// the manifest that is GONE (`an_unreadable_run_manifest_…`, where
/// `std::fs::read` fails) and the manifest that genuinely declared none; this is
/// the one in between — the read SUCCEEDS and the bytes carry no meaning — and
/// it fell through to [`TRACE_NONE_NO_RINGS`] because `run_manifest_ring_tags`
/// returns the same EMPTY vector for unparseable bytes as for a manifest that
/// declared none.
///
/// **The bag then contradicted itself inside ONE document.** `trace` claimed the
/// run's manifest declared no trace rings, while `run_manifest_unparsed` — added
/// by the same `render_attach_run_json` call, into the same JSON object —
/// recorded that the manifest could not be parsed at all. One artifact, two
/// answers, and the false one is the one an operator acts on. That is this
/// repo's one-run-two-answers class, in a single durable artifact rather than
/// across two log lines.
///
/// **It is reachable on the flagship path, not only on a corrupt disk.**
/// `run_dir::declare_run_rings` rewrites `run.json` IN PLACE — read, insert the
/// `rings` array, write back through `run_dir::write_artifact`, whose
/// `OpenOptions` carries `.truncate(true)`. The truncation lands at `open` and
/// the bytes at a later `write_all`, so between those two calls the manifest is
/// ZERO BYTES on disk. The recorder cannot be kept out of that window: `graph run`
/// publishes the run to the registry BEFORE the deployment dispatch, while the
/// ring declaration happens inside the record bring-up (between all-workers-READY
/// and the GO sentinel), so the run is discoverable — and therefore attachable —
/// for the whole of it. This arm models that window exactly, by writing the same
/// zero bytes the truncation leaves.
///
/// **Not a duplicate of its sibling, and `artifacts_unreadable` is what proves
/// it.** That key is what the unreadable arm's text points a reader at, and this
/// state does not produce one — `std::fs::read` returned `Ok`, so
/// `RunArtifacts::unreadable` is empty. Asserting it ABSENT is therefore the
/// discriminator between the two degraded states AND the no-dangling-pointer
/// property: a shortcut that reused [`TRACE_UNKNOWN_NO_MANIFEST`] here would send
/// the reader to a key that is not in the document.
///
/// Lives here rather than beside the pure oracles in `bag_cmd` for the reason
/// the sibling arm documents, which holds for this state too: a pure
/// arm re-derives the selector in its own body, so it is structurally blind to
/// the production call site passing the wrong one. Reverting to
/// `} else if false {` passes against a pure arm and fails only here.
#[test]
fn a_manifest_read_but_unparseable_says_unknown_not_that_none_was_declared() {
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ra_unparsed_test".into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let topic = format!("/{}", unique("ra_unparsed"));
    // Published WITH a ring declared, so the manifest we then truncate is one
    // that HAD something to say. That is what makes the wrong answer
    // distinguishable from the right one: a wrong answer claims the run
    // "declared no trace rings" about a manifest that declared one.
    let ring_tag = short_tag("up");
    let run = publish_run(&ix, &[&topic], std::slice::from_ref(&ring_tag));

    let run_dir = {
        let gathered = cerulion_core::transport::run_registry::gather_runs_on_config(
            &ix,
            Duration::from_secs(5),
        )
        .expect("gather the run we just published");
        let rec = gathered
            .records
            .iter()
            .find(|r| r.run_id == run.run_id)
            .unwrap_or_else(|| panic!("the published run must be gatherable: {gathered:?}"));
        PathBuf::from(rec.run_dir.clone())
    };
    // ZERO BYTES, not removed — the ONE line that separates this arm from its
    // sibling, and the exact state `write_artifact`'s `.truncate(true)` leaves
    // between its `open` and its `write_all`.
    let manifest = run_dir.join("run.json");
    std::fs::write(&manifest, b"").expect("truncate the run manifest in place");
    assert!(
        manifest.exists(),
        "precondition: the manifest must still EXIST — a removed one is the \
         sibling arm's state, and this arm would silently become a duplicate of it"
    );
    assert_eq!(
        std::fs::read(&manifest)
            .expect("the manifest must be READABLE")
            .len(),
        0,
        "precondition: the read must SUCCEED and yield nothing — that success is \
         what makes this state carry no `artifacts_unreadable` entry"
    );
    assert!(
        run_dir.join("graph.yaml").exists(),
        "precondition: only the MANIFEST was truncated — the topic set still resolves"
    );

    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_topic = topic.clone();
    let produce_mgr = Arc::clone(&manager);
    let producer = std::thread::spawn(move || {
        let mut publisher = produce_mgr
            .create_publisher(&produce_topic, MaxSliceLen::const_new(256), 0)
            .expect("producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&frame(seq)).expect("publish");
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique("ra_unparsed_bag")));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    let recorder = std::thread::spawn(move || {
        bag_record_with_manager(
            manager,
            RecordOptions {
                run: Some(RunTarget::Sole),
                out: bag_path,
                ..RecordOptions::default()
            },
            stop,
            &mut Vec::new(),
        )
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    running.store(false, Ordering::Relaxed);
    let summary = recorder
        .join()
        .expect("no panic")
        .expect("an unparseable manifest must cost the TRACE, never the recording");
    producing.store(false, Ordering::Relaxed);
    producer.join().expect("no panic");

    let bag = summary.bag_paths.first().expect("one bag").clone();
    let reader = cerulion_bag::BagReader::open(&bag).expect("open");
    let run_json = reader
        .attachment("__cerulion/run.json")
        .expect("read attachments")
        .expect("an attached bag carries its run identity even with an unparseable manifest");
    let doc: serde_json::Value = serde_json::from_slice(&run_json.data).expect("valid JSON");

    assert_eq!(
        doc["trace"],
        serde_json::Value::String(
            cerulion_cli_engine::bag_cmd::TRACE_UNKNOWN_UNPARSEABLE_MANIFEST.to_string()
        ),
        "the recorder read bytes it could not parse, so what this run declared is \
         UNKNOWN. Got: {}",
        doc["trace"]
    );
    assert_ne!(
        doc["trace"],
        serde_json::Value::String(cerulion_cli_engine::bag_cmd::TRACE_NONE_NO_RINGS.to_string()),
        "THE discrimination, and the state the three-way selector collapsed this \
         one onto: the manifest DECLARED a ring. Saying the run declared none \
         states a fact about the RUN drawn from bytes that parsed as nothing — \
         and says it BESIDE `run_manifest_unparsed` in the same document"
    );
    assert_ne!(
        doc["trace"],
        serde_json::Value::String(
            cerulion_cli_engine::bag_cmd::TRACE_UNKNOWN_NO_MANIFEST.to_string()
        ),
        "the two degraded states are NOT interchangeable: this one's read \
         SUCCEEDED, so the unreadable arm's text would point at an \
         `artifacts_unreadable` key that is not in this document"
    );

    // The EVIDENCE the verdict names must actually be here — the verdict's
    // pointer is only as good as the key it points at.
    assert_eq!(
        doc["run_manifest_unparsed"],
        serde_json::Value::String(String::new()),
        "the unparseable bytes are carried VERBATIM, and reading back exactly the \
         zero bytes we wrote is what proves the recorder met the truncation \
         window rather than some other failure. Got: {:?}",
        doc.get("run_manifest_unparsed")
    );
    // …and the key the SIBLING state's verdict names must NOT be, or the two are
    // indistinguishable and this arm is a duplicate.
    assert!(
        doc.get("artifacts_unreadable").is_none(),
        "the read SUCCEEDED, so nothing was unreadable — a row here would both \
         contradict `run_manifest_unparsed` and make this state look like its \
         sibling. Got: {:?}",
        doc.get("artifacts_unreadable")
    );
    // Absence stays NO-CLAIM: nothing invents a parsed `run` object to fill the
    // hole left by bytes that carried no meaning.
    assert!(
        doc.get("run").is_none(),
        "an unparseable manifest is carried as opaque text, not a fabricated shape"
    );

    // Degrade, never fail: the frames are the part that cannot be obtained later.
    assert!(
        !recorded_payloads(&bag, &topic).is_empty(),
        "an unparseable manifest costs the TRACE, not the recording"
    );
    assert!(
        recorded_trace(&bag).is_empty(),
        "and there really is no trace, or the verdict is a lie"
    );

    let _ = std::fs::remove_file(&bag);
}

/// `docs/bag.md`'s scheduler-trace section teaches the INTENT/OUTCOME split.
///
/// A doc drifts from the code when nothing reads it. A section that
/// presents exactly TWO cases and asserts the first unconditionally —
/// *"attaching to such a run picks the trace up from the attach point"* — hides
/// the third case the code goes to considerable lengths to report accurately: a run whose
/// directory is being removed as the recorder attaches. If `rings_unavailable`,
/// `rings_declared` and `artifacts_unreadable` appear NOWHERE in the file,
/// a reader who consults the doc and then opens
/// `__cerulion/run.json` — the two artifacts the doc points at — is given no
/// reason to know a second, authoritative artifact exists or to look for it.
///
/// A structural arm rather than a prose check, on the doc-pinning
/// principle: a doc that needs the code's comments to be correct is the defect,
/// and the only durable fix is a test that fails when the two diverge.
#[test]
fn the_docs_scheduler_trace_section_names_the_degraded_cases_and_their_artifacts() {
    let raw =
        std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/bag.md"))
            .expect("docs/bag.md must be readable from the crate root");
    // Whitespace-NORMALIZED, because the file is hard-wrapped prose: a phrase
    // this arm requires is routinely split across a line break (an unnormalized
    // match fails on exactly that, against a doc that DOES contain the sentence). A
    // reflow must not break a CONTENT pin — the property is what the section
    // says, not how it is wrapped.
    let doc = raw.split_whitespace().collect::<Vec<_>>().join(" ");

    // ANTI-TAUTOLOGY: the section must still be reachable, or every assertion
    // below is vacuous against a renamed or deleted heading.
    assert!(
        doc.contains("**Scheduler trace.**"),
        "the scheduler-trace section moved or was renamed — this arm is pinning \
         nothing until its anchor is updated"
    );

    for (token, why) in [
        (
            "rings_unavailable",
            "the manifest field that carries a vanished ring's outcome",
        ),
        (
            "rings_declared",
            "…which is only readable as a FRACTION of what was declared",
        ),
        (
            "artifacts_unreadable",
            "the field that carries an unreadable manifest's outcome — a case \
             `rings_unavailable` structurally cannot cover, since with no manifest \
             nothing is ever DECLARED unavailable",
        ),
        (
            "run_manifest_unparsed",
            "the field that carries an UNPARSEABLE manifest's outcome — the third \
             degraded case, and the one `artifacts_unreadable` structurally cannot \
             cover, since its read SUCCEEDED. A reader told only about \
             `artifacts_unreadable` goes looking for a key this bag does not have",
        ),
        (
            "trace: NONE",
            "the `bag info` verdict line an operator greps for",
        ),
        (
            "state_ring_consumer",
            "the run's OWN statement about whether anything is \
             already draining its per-rank state rings. It is what decides \
             whether this verb declines the state plane, so a reader who cannot \
             find it cannot tell an anchorless bag from a broken one",
        ),
        (
            "state_rings",
            "the verdict this verb writes into its own bag — the \
             key that says the state plane was DECLINED rather than swept and \
             found empty. Without it the two are byte-indistinguishable, since a \
             declined attach writes no `state_coverage.json` either",
        ),
        (
            "trace_rings",
            "§5.4: the run's OWN statement about which of the trace \
             states it is in. Without it a reader takes an empty `rings` list \
             for the one fact it happens to know about, and a run that DECLINED \
             rings is told the feature does not exist yet",
        ),
        (
            "declared_unavailable",
            "§5.4: the ranks whose declared ring was never created. \
             Tags are stamped BEFORE creation, so `rings` over-declares; a \
             reader told only about `rings` renders `from the attach point` for \
             a ring nothing could open",
        ),
        (
            "cannot be corrected in place",
            "WHY `run.json`'s frozen verdict cannot be trusted alone — without \
             this the reader has the fields but not the rule",
        ),
    ] {
        assert!(
            doc.contains(token),
            "docs/bag.md's scheduler-trace section must teach the INTENT/OUTCOME \
             split — missing {token:?} ({why}). A reader who learns only the \
             two-case model, then reads `run.json`'s frozen `trace`, is told the \
             trace is present when the bag holds none."
        );
    }
}

/// Make `tracing` callsite interest SAFE to capture from a
/// thread-scoped subscriber, and un-poison any callsite a sibling test already
/// killed.
///
/// # The defect this closes
///
/// Without it, `a_manifest_predating_the_key_is_swept_and_says_it_could_not_tell`
/// can fail under a parallel runner with `left: 0, right: 1` — the unknown-verdict warn is
/// not merely mis-filtered, it is ABSENT from the capture, while the same
/// verdict string is written into the bag from the same variable four
/// lines earlier. That is not a timing band and no wait can repair it.
///
/// `tracing` caches an `Interest` per CALLSITE, once, on the callsite's first
/// hit (`tracing_core::callsite::register`). The rebuild it does there asks
/// `DISPATCHERS.rebuilder()`, which returns `Rebuilder::JustOne` whenever at
/// most ONE dispatcher is registered — the initial state of an empty registry
/// included — and `JustOne::for_each` consults `dispatcher::get_default`, i.e.
/// **the registering thread's own default subscriber**. A thread with none gets
/// `NoSubscriber`, whose `register_callsite` returns `Interest::never()`, and
/// that `never` is CACHED: `tracing::warn!` gates on `!interest.is_never()`, so
/// the site is dead for the rest of the process.
///
/// This binary reproduces that exactly. The UNKNOWN arm's warn is one callsite
/// shared by four tests, and only the three state-consumer arms run under
/// [`ThreadLogs`]; `an_unreadable_run_manifest_…` and
/// `a_manifest_read_but_unparseable_…` reach the very same `warn!` on recorder
/// threads with NO thread-local subscriber. Whichever hits it first decides the
/// callsite's fate for the whole binary, and libtest runs them in parallel.
/// It also explains why only THIS arm can fail: the REFUSAL warn eight lines
/// up in `bag_cmd.rs` is a different callsite, reachable only from
/// `attach_under_state_consumer(.., Some("standing"))`, so it is always
/// registered under a capturing subscriber and can never be poisoned.
///
/// # Why a permanent GLOBAL subscriber is the remedy
///
/// Once one is registered, `get_default` on a thread with no scoped subscriber
/// yields IT rather than `NoSubscriber`, and it answers `enabled` with `true` —
/// so no thread can ever cache `never` for any callsite again, whatever the
/// interleaving. It discards everything, so it changes no output: the capture
/// still comes from the thread-scoped [`ThreadLogs`], which takes precedence.
///
/// The [`rebuild_interest_cache`](tracing::callsite::rebuild_interest_cache)
/// call is the other half, and it is not redundant: a callsite poisoned BEFORE
/// the global existed keeps its cached `never`, because `register` fires once
/// per callsite and will not fire again. The rebuild recomputes every already
/// registered callsite against the now-registered global. Together the two make
/// the outcome independent of which test wins the race — install first and
/// nothing can be poisoned, install late and the rebuild repairs it.
fn arm_tracing_capture() {
    /// Enables everything and records nothing.
    struct GlobalFloor;
    impl tracing::Subscriber for GlobalFloor {
        fn enabled(&self, _: &tracing::Metadata<'_>) -> bool {
            true
        }
        fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::Id {
            tracing::Id::from_u64(1)
        }
        fn record(&self, _: &tracing::Id, _: &tracing::span::Record<'_>) {}
        fn record_follows_from(&self, _: &tracing::Id, _: &tracing::Id) {}
        fn event(&self, _: &tracing::Event<'_>) {}
        fn enter(&self, _: &tracing::Id) {}
        fn exit(&self, _: &tracing::Id) {}
    }
    static ARMED: std::sync::Once = std::sync::Once::new();
    ARMED.call_once(|| {
        // A failure means somebody else already claimed the slot, which serves
        // the same purpose — the point is that SOME global exists, not that it
        // is ours.
        let _ = tracing::subscriber::set_global_default(GlobalFloor);
    });
    tracing::callsite::rebuild_interest_cache();
}

/// A `tracing` sink that records what ONE THREAD emits.
///
/// `#[traced_test]` is the usual pattern and cannot be used here: it installs a
/// GLOBAL subscriber and attributes captured lines to the test's own span, while
/// the line under test is emitted by `bag_record_with_manager` on a spawned
/// recorder thread, which inherits no span. Those lines would be captured and
/// then filtered out — a green test that proves nothing.
///
/// `with_default` scopes a subscriber to exactly the thread that installs it, so
/// wrapping the recorder closure captures its events and nothing else. Hand-
/// rolled because `tracing-subscriber` is not a dependency of this crate and one
/// assertion does not justify adding it.
struct ThreadLogs(std::sync::Arc<std::sync::Mutex<Vec<String>>>);

impl tracing::Subscriber for ThreadLogs {
    fn enabled(&self, _: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::Id {
        tracing::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::Id, _: &tracing::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        struct Msg(String);
        impl tracing::field::Visit for Msg {
            fn record_debug(&mut self, f: &tracing::field::Field, v: &dyn std::fmt::Debug) {
                if f.name() == "message" {
                    self.0.push_str(&format!("{v:?}"));
                } else {
                    self.0.push_str(&format!(" {}={v:?}", f.name()));
                }
            }
        }
        let mut msg = Msg(String::new());
        event.record(&mut msg);
        // The LEVEL is captured with the text: "the refusal is loud" is half the
        // claim, and a `debug!` carrying the same words would satisfy a
        // message-only assertion.
        self.0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(format!("{} {}", event.metadata().level(), msg.0));
    }
    fn enter(&self, _: &tracing::Id) {}
    fn exit(&self, _: &tracing::Id) {}
}

/// **The apparatus oracle for [`arm_tracing_capture`].**
///
/// Reproduces the callsite poisoning DETERMINISTICALLY, in the exact shape this
/// binary produces by accident: ONE capture live (so `tracing`'s dispatcher
/// registry reports `has_just_one`), and a sibling thread with NO subscriber
/// winning the race to a shared callsite's FIRST hit.
///
/// The interleave is forced with channels rather than sleeps, so it is a
/// property and not a timing band: the cold thread's hit is sequenced strictly
/// between the capture being installed and the capture's own emit. That
/// ordering is the whole hazard — a cold hit BEFORE the capture is harmless,
/// because `Dispatch::new` rebuilds every callsite's interest on registration,
/// which is exactly why running the two real tests serially does NOT reproduce
/// it and why only a genuinely parallel runner shows it.
///
/// Run this test alone (`--exact`) to see it. Measured, with `arm_tracing_capture`
/// deleted from both call sites:
///
/// ```text
/// assertion `left == right` failed: … Captured: []
///   left: 0
///  right: 1
/// ```
///
/// — the CAPTURE SAW NOTHING, which is the parallel-runner failure signature of
/// `a_manifest_predating_the_key_…` character for character. The cold
/// thread's `NoSubscriber` answers `Interest::never()` and that verdict is
/// cached for the process. With the same deletion the two real tests run
/// SERIALLY still pass, because `Dispatch::new` rebuilds every callsite's
/// interest on registration — which is why the defect needs a genuinely
/// parallel runner to show, and why reproducing it serially proves nothing.
#[test]
fn a_callsite_first_hit_from_an_uncaptured_thread_still_reaches_a_live_capture() {
    // ONE callsite, reached from both threads. A closure would still be one
    // callsite; a function makes that impossible to lose to a refactor.
    fn probe(marker: &str) {
        tracing::warn!(marker = %marker, "tracing-capture probe");
    }

    arm_tracing_capture();

    let logs = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let logs_w = std::sync::Arc::clone(&logs);
    let (go_tx, go_rx) = std::sync::mpsc::channel::<()>();
    let (done_tx, done_rx) = std::sync::mpsc::channel::<()>();

    let cold = std::thread::spawn(move || {
        go_rx.recv().expect("the capture to be installed");
        probe("cold");
        done_tx.send(()).expect("the capture to still be waiting");
    });
    let warm = std::thread::spawn(move || {
        tracing::subscriber::with_default(ThreadLogs(logs_w), || {
            go_tx.send(()).expect("the cold thread to be waiting");
            done_rx.recv().expect("the cold thread's hit");
            probe("warm");
        });
    });
    cold.join().expect("no panic on the cold thread");
    warm.join().expect("no panic on the capturing thread");

    let seen = logs.lock().unwrap_or_else(|e| e.into_inner()).clone();
    assert_eq!(
        seen.len(),
        1,
        "a callsite another thread hit first must still reach a live capture — \
         otherwise every `!logs.contains(..)` in this file is vacuous and every \
         `logs.lines().filter(..)` counts zero. Captured: {seen:?}"
    );
    assert!(
        seen[0].starts_with("WARN") && seen[0].contains("marker=warm"),
        "…and it must be the CAPTURING thread's emit, not the cold one's: {seen:?}"
    );
}

/// Drive one attach against a run whose manifest declares
/// `state_ring_consumer = state`, and hand back the bag's own answer.
///
/// The tuple is `(state_rings verdict, state_coverage present, trace records,
/// frames, bag info, recorder logs)` — the four things the state-consumer arms disagree
/// about, plus what `cerulion bag info` prints for the bag, plus what the
/// recorder thread emitted. The last two are the only surfaces an operator ever
/// sees: one in the file, one on the terminal at the moment it happens.
///
/// The log capture is THREAD-SCOPED (see `ThreadLogs`), so it sees the attach
/// decision only while that decision is taken on the recorder thread. If it ever
/// moves, the two `!logs.contains(..)` ABSENCE assertions would go vacuous —
/// but the two PRESENCE arms fail loudly first, so the pair degrades safely
/// rather than silently. `state_coverage`'s
/// presence is the BEHAVIOURAL half and is not a restatement of the verdict:
/// `state_coverage_seed` returns `None` exactly when the recorder was given no
/// ring, none failed, AND no discovery tag, so the attachment exists if and only
/// if the recorder really swept. A verdict string can be written by a renderer
/// that changed nothing; this cannot.
fn attach_under_state_consumer(
    tag: &str,
    state: Option<&str>,
) -> (String, bool, usize, usize, String, String) {
    // BEFORE anything in this arm can emit — see `arm_tracing_capture`.
    arm_tracing_capture();
    let ix = iceoryx_test_config();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("{tag}_mgr"),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        ix.clone(),
    )
    .expect("isolated transport");

    let topic = format!("/{}", unique(tag));
    // A real, LAPPED trace ring, so the refusal arm can prove it costs the STATE
    // plane and nothing else: a bag that lost its trace as well would satisfy a
    // verdict assertion just as happily.
    let ring_tag = short_tag(tag);
    // The owner stays alive for the whole arm: dropping it UNLINKS the SHM name
    // the recorder's mapping is reading.
    let mut ring = TraceRingOwner::create(&ring_tag, RING_CAP, 0, &["src"])
        .expect("create the run's trace ring");
    let mut producer = ring.producer().expect("the ring's one producer");
    // LAP the ring before the attach, so this is a genuine MID-RUN attach rather
    // than an at-zero one wearing the name: `open` refuses a lapped ring, and
    // `open_at_live` is what the attach path uses.
    for step in 0..(u64::from(RING_CAP) * 3) {
        producer.push(&boundary(step));
        producer.push(&fire(step));
    }
    let _run =
        publish_run_with_state_consumer(&ix, &[&topic], std::slice::from_ref(&ring_tag), state);

    let producing = Arc::new(AtomicBool::new(true));
    let produce_stop = Arc::clone(&producing);
    let produce_topic = topic.clone();
    let produce_mgr = Arc::clone(&manager);
    let frames = std::thread::spawn(move || {
        let mut publisher = produce_mgr
            .create_publisher(&produce_topic, MaxSliceLen::const_new(256), 0)
            .expect("producer");
        let mut seq = 0u32;
        while produce_stop.load(Ordering::Relaxed) {
            publisher.publish_raw(&frame(seq)).expect("publish");
            seq = seq.wrapping_add(1);
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let out_path = std::env::temp_dir().join(format!("{}.mcap", unique(tag)));
    let running = Arc::new(AtomicBool::new(true));
    let stop = Arc::clone(&running);
    let bag_path = out_path.clone();
    let logs = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let logs_w = std::sync::Arc::clone(&logs);
    let recorder = std::thread::spawn(move || {
        // Scoped to THIS thread — see `ThreadLogs`.
        tracing::subscriber::with_default(ThreadLogs(logs_w), || {
            bag_record_with_manager(
                manager,
                RecordOptions {
                    run: Some(RunTarget::Sole),
                    out: bag_path,
                    ..RecordOptions::default()
                },
                stop,
                &mut Vec::new(),
            )
        })
    });

    await_condition(
        "the recorder to create its bag",
        Duration::from_secs(30),
        || out_path.exists() || recorder.is_finished(),
    );
    if recorder.is_finished() {
        panic!(
            "the recorder exited before creating a bag — its own error is the failure: {:?}",
            recorder.join()
        );
    }
    // The trace records must land AFTER the recorder opened the ring at its LIVE
    // cursor, or they sit behind it and the arm measures nothing. The ring is
    // opened in `Recorder::setup`, strictly before `ensure_writer`, so the bag
    // file's appearance is a sound proxy for "the ring is attached".
    const FIRST_COMPLETE_STEP: u64 = 900;
    for step in FIRST_COMPLETE_STEP..(FIRST_COMPLETE_STEP + 3) {
        producer.push(&boundary(step));
        producer.push(&fire(step));
    }
    // No wait for the drain: the trace rings are drained by the WRITER THREAD,
    // which `finalize` joins, so everything pushed strictly BEFORE this flip is
    // inside the shutdown window. (A mid-run poll would time the chunk-flush
    // cadence instead — see the headline arm's note.)
    running.store(false, Ordering::Relaxed);
    let summary = recorder.join().expect("no panic").expect("must record");
    producing.store(false, Ordering::Relaxed);
    frames.join().expect("no panic");

    let bag = summary.bag_paths.first().expect("one bag").clone();
    let reader = cerulion_bag::BagReader::open(&bag).expect("open");
    let run_json = reader
        .attachment("__cerulion/run.json")
        .expect("read attachments")
        .expect("an attached bag carries its run identity");
    let doc: serde_json::Value = serde_json::from_slice(&run_json.data).expect("valid JSON");
    let verdict = doc["state_rings"]
        .as_str()
        .expect("the attach manifest states what it did about the state plane")
        .to_string();
    let swept = summary.state_coverage.is_some();
    let trace = recorded_trace(&bag).len();
    let payloads = recorded_payloads(&bag, &topic).len();
    // Read BEFORE the bag is removed, and through the shipped verb rather than
    // through `render_state_rings_section` directly: the unit arms in
    // `bag_cmd::state_rings_section` construct their `StateRingsReading`
    // by hand, so they stay green if `bag_info` stops pushing the block at all.
    let info = cerulion_cli_engine::bag_cmd::bag_info(&bag, None).expect("bag info must render");

    let _ = std::fs::remove_file(&bag);
    let logged = logs.lock().unwrap_or_else(|e| e.into_inner()).join("\n");
    (verdict, swept, trace, payloads, info, logged)
}

/// **The headline:** a run that reports a STANDING window
/// recorder makes the attach decline state-ring discovery — and cost the bag
/// nothing else.
///
/// This is the DEFAULT run shape, so an attach that ignores the key
/// is routinely the second consumer of a `Backpressure` ring whose cursor lives
/// in ONE shared header slot. The slower reader is then retired with an
/// `Overrun` it never asked for, and on the shape that matters the slower reader
/// can be the run's own black box.
///
/// Four claims, and the last two are what stop the refusal being an
/// over-correction: the manifest SAYS it declined and says why, the recorder
/// really swept nothing (`state_coverage` absent — see the helper), and the bag
/// still carries its scheduler TRACE and its FRAMES. Trace rings are `FailLoud`
/// with LOCAL cursors by design, so N readers is sound and the
/// refusal must not touch them.
#[test]
fn a_standing_recorder_makes_the_attach_decline_the_state_plane() {
    let (verdict, swept, trace, frames, info, logs) =
        attach_under_state_consumer("scstand", Some("standing"));

    assert_eq!(
        verdict,
        cerulion_cli_engine::bag_cmd::STATE_RINGS_REFUSED_STANDING,
        "the bag must SAY it declined, and why — a reader holding it has no other \
         way to learn its anchors are elsewhere"
    );
    assert!(
        !swept,
        "and must really have declined: `state_coverage` is written whenever the \
         recorder was given a ring, lost one, or held a discovery tag, so its \
         presence would mean the sweep ran anyway"
    );
    assert!(
        trace > 0,
        "the refusal is scoped to the STATE plane: a trace ring is FailLoud with \
         local cursors, so this bag must still carry its scheduler trace"
    );
    assert!(
        frames > 0,
        "…and its frames, which nothing about the state plane touches"
    );
    // THE OPERATOR SURFACE. A declined attach carries no `state_coverage.json`,
    // which is byte-for-byte what a recording that was never configured for
    // checkpoints looks like — so without this row `bag info` prints the two
    // identically and the decision above is durable but invisible.
    assert!(
        info.contains("state rings:") && info.contains("refused"),
        "`bag info` must PRINT the decision, or it is written into the bag and surfaced by \
         nothing:\n{info}"
    );
    // …and the operator is told AT THE MOMENT it happens, not only when they
    // later open the bag. The refusal NARROWS what this recording captures, so
    // silence at the terminal lets a `--run` attach come back without anchors
    // and say nothing about it until somebody reads the file.
    // COUNTED, not merely found. The site documents "ONE loud line"
    // (`bag_cmd.rs`, the refusal arm), and a `.find` is satisfied by the first
    // of any number — so a refactor that emitted the refusal once per tap, or
    // left an old call beside a new one, would keep this green while doubling
    // the noise on exactly the surface the line exists to keep readable.
    let refusals: Vec<&str> = logs
        .lines()
        .filter(|l| l.contains("bag record --run:") && l.contains("NOT sweeping for them"))
        .collect();
    assert_eq!(
        refusals.len(),
        1,
        "the attach must WARN that it declined the state plane, EXACTLY once. Logged:\n{logs}"
    );
    let refusal = refusals[0];
    // Asserted at WARN, not merely present: the same sentence at `debug!` is
    // filtered out by default, which is the same thing as not emitting it.
    assert!(
        refusal.starts_with("WARN"),
        "the refusal must be LOUD: {refusal}"
    );
    assert!(
        refusal.contains("exactly ONE consumer") && refusal.contains("cerulion flashback"),
        "…and say WHY it declined and where the anchors are instead: {refusal}"
    );
}

/// **The other direction:** a run that reports NO standing
/// consumer is swept in full.
///
/// The anti-tautology arm for the headline. Without it, "the attach declined"
/// is satisfied by an attach that declines unconditionally — which would cost
/// every `CERULION_FLASHBACK=off` run's mid-run recording its anchors to avoid
/// contending with a consumer that does not exist.
#[test]
fn a_run_with_no_standing_consumer_is_swept_as_before() {
    let (verdict, swept, trace, frames, info, logs) = attach_under_state_consumer(
        "scnone",
        Some("none: the Flashback capture plane is switched off for this run"),
    );

    assert_eq!(
        verdict,
        cerulion_cli_engine::bag_cmd::STATE_RINGS_FROM_ATTACH,
        "a run that says nobody is draining its state rings gets the from-attach \
         verdict, not the refusal"
    );
    assert!(
        swept,
        "and the recorder really swept: `state_coverage` is present exactly when it \
         was given a discovery tag"
    );
    assert!(trace > 0 && frames > 0, "everything else is unchanged");
    assert!(
        info.contains("state rings:") && info.contains("from the attach point"),
        "`bag info` prints the swept verdict too — printing only the refusal would make the \
         block itself a signal:\n{info}"
    );
    // THE ANTI-TAUTOLOGY HALF of the refusal warn asserted in the standing arm:
    // without it, "the attach warns when it declines" is satisfied by a build
    // that warns on every attach — which would train an operator to ignore the
    // line on exactly the runs it matters for.
    //
    // This run gave a READABLE answer, so it is also the one arm that must be
    // silent on both counts: no refusal, and no could-not-tell.
    assert!(
        !logs.contains("NOT sweeping for them"),
        "an attach that SWEPT must not warn that it declined:\n{logs}"
    );
    // The could-not-tell line is pinned by BOTH halves it is made of — its
    // constant message and the `state_rings` field the verdict rides in — so
    // this stays a real absence claim whichever half a regression reintroduces
    // (an interpolated message carrying no field, or a field beside a reworded
    // message). `state_rings=` is emitted at exactly one site in the tree.
    assert!(
        !logs.contains("could not tell whether this run's per-rank"),
        "…nor hedge its anchors at all, having read the run's own words:\n{logs}"
    );
    assert!(
        !logs.contains("state_rings="),
        "…and must carry no state-ring verdict field either — the field exists to \
         report a decision taken on no evidence, and this attach had evidence:\n{logs}"
    );
}

/// **The UNKNOWN arm:** a manifest that predates the key is
/// swept, and the bag says the decision was made on no evidence.
///
/// This is every run started by a build older than the key, so it is the arm
/// that decides whether the refusal is a narrowing or a breakage. It PROCEEDS, which is
/// the deliberate asymmetry: declining on absent evidence would cost a bag its
/// anchors to avoid a consumer nobody observed, while proceeding keeps
/// exactly the behaviour of a build without the key — the hazard is no worse, and the
/// bag records that nobody could tell.
///
/// It must NOT render the from-attach text, which is a positive claim about what
/// the run declared, on a manifest that declared nothing.
#[test]
fn a_manifest_predating_the_key_is_swept_and_says_it_could_not_tell() {
    let (verdict, swept, trace, frames, info, logs) = attach_under_state_consumer("sclegacy", None);

    assert_eq!(
        verdict,
        cerulion_cli_engine::bag_cmd::STATE_RINGS_UNKNOWN_LEGACY,
        "an absent key is UNKNOWN — a fact about the run — and must never be \
         rendered as the run having declared no consumer"
    );
    assert!(
        swept,
        "an unknown proceeds: the behaviour of a build without the key, exactly, rather than \
         a refusal made on no evidence"
    );
    assert!(trace > 0 && frames > 0, "everything else is unchanged");
    // The bag DOES carry a `state_rings` key — the attach wrote one, saying it
    // could not tell what the RUN decided. So this asserts the LEGACY verdict's
    // own text, not the bare `verdict: None` row: both render under a
    // `state rings: unknown` prefix, and matching the prefix alone would read as
    // though this arm covered a row it never reaches (that one needs a
    // bag that predates the key and is pinned by the unit arms instead).
    assert!(
        info.contains("state rings: unknown")
            && info.contains("carries no state-ring-consumer statement"),
        "`bag info` must say UNKNOWN rather than stay silent — silence here is exactly what a \
         recording that never checkpointed looks like:\n{info}"
    );
    // It proceeded on NO evidence, which is the deliberate choice — but an
    // inference, and the hazard it accepts (a lapped state ring whose loser can
    // be the run's own black box) is real. So it is LOUD at the terminal too,
    // not only in a file nobody opens until later.
    // Counted for the reason the standing arm's refusal is: one attach, one
    // line. A `.find` here would pass a build that reported the unknown once
    // per artifact it failed to read.
    let unknowns: Vec<&str> = logs
        .lines()
        .filter(|l| {
            l.contains("bag record --run: this attach could not tell whether this run's per-rank")
                && l.contains("state_rings=")
                && l.contains("unknown")
        })
        .collect();
    assert_eq!(
        unknowns.len(),
        1,
        "an attach that could not tell must SAY so at the terminal, EXACTLY once:\n{logs}"
    );
    let unknown = unknowns[0];
    assert!(
        unknown.starts_with("WARN"),
        "…and loudly, or it is the same as silence: {unknown}"
    );
    // …and the run's own verdict must ride the `state_rings` FIELD. Splicing this
    // datum into the message (`"bag record --run: {}"`) is what
    // `tracing_field_discipline_test` rejects, and it costs every consumer a
    // key: a JSON subscriber gets none, and an operator can only grep the prose.
    // Built from the exported const rather than retyped, so a reword of the
    // verdict cannot leave this arm matching a sentence the bag no longer says.
    assert!(
        unknown.contains(&format!(
            "state_rings={}",
            cerulion_cli_engine::bag_cmd::STATE_RINGS_UNKNOWN_LEGACY
        )),
        "the verdict belongs in the field, whole: {unknown}"
    );
    // …but it must NOT claim to have declined anything.
    assert!(
        !logs.contains("NOT sweeping for them"),
        "an attach that SWEPT must not warn that it declined:\n{logs}"
    );
}
