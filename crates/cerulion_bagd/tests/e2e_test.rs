// SPDX-License-Identifier: AGPL-3.0-only
//! In-process end-to-end tests for `cerulion_bagd::run_bagd`.
//! Each test builds an ISOLATED per-instance iceoryx2 transport
//! ([`common::make_manager`]) and drives `run_bagd` on a worker thread with an
//! `Arc<AtomicBool>` shutdown flag — the same flag the binary wires to
//! SIGINT/SIGTERM. Oracles are HAND-BUILT wire frames + hand-pushed trace
//! records (never a self-compare), and every bag is independently re-read with
//! the upstream `mcap` crate.

mod common;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_bag::{BagReader, TraceRingRecord, NONDETERMINISM_TOPIC, SCHEDULER_TRACE_TOPIC};
use cerulion_bagd::{
    run_bagd, BagdConfig, BagdError, BagdSummary, TapSpec, WriterStallGate,
    FROZEN_BURST_MARKER_THRESHOLD, RECORD_HEALTH_ATTACHMENT,
};
use cerulion_core::trace_ring::TraceRingOwner;
use cerulion_core::transport::TransportManager;

use common::*;

/// Spawn `run_bagd` on a worker thread.
fn spawn_bagd(
    mgr: Arc<TransportManager>,
    cfg: BagdConfig,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<Result<BagdSummary, BagdError>> {
    std::thread::spawn(move || run_bagd(mgr, cfg, shutdown))
}

/// A config with fast, test-friendly cadences + a ready-file, status off.
fn test_cfg(out: std::path::PathBuf, taps: Vec<TapSpec>, ready: std::path::PathBuf) -> BagdConfig {
    let mut cfg = BagdConfig::new(out, taps);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.ready_file = Some(ready);
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(500);
    cfg
}

/// Full recovered messages from a FINALIZED bag, filtered to `topic` (file
/// order == arrival order).
fn data_msgs_for(reader: &BagReader, topic: &str) -> Vec<cerulion_bag::BagMessage> {
    let (msgs, completeness) = reader.recover_messages().expect("recover_messages");
    assert!(
        completeness.is_finalized(),
        "bag must be Finalized, got {completeness:?}"
    );
    msgs.into_iter().filter(|m| m.topic == topic).collect()
}

/// Data (non-reserved) message payloads recovered from a bag, filtered to
/// `topic`.
fn data_frames_for(reader: &BagReader, topic: &str) -> Vec<Vec<u8>> {
    data_msgs_for(reader, topic)
        .into_iter()
        .map(|m| m.data)
        .collect()
}

/// Sorted channel descriptor tuples of a bag: `(topic, schema_name,
/// schema_hash, wire_fixed_size)` — the rotation-stability oracle shape.
fn channel_tuples(reader: &BagReader) -> Vec<(String, String, u64, u32)> {
    let mut out: Vec<(String, String, u64, u32)> = reader
        .channels()
        .expect("channels")
        .into_iter()
        .map(|c| {
            let d = c.descriptor.expect("cerulion descriptor");
            (c.topic, c.schema_name, d.schema_hash, d.wire_fixed_size)
        })
        .collect();
    out.sort();
    out
}

/// Drain every queued `/bagd/status` frame into parsed JSON values (bounded).
fn drain_status_json(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    out: &mut Vec<serde_json::Value>,
) {
    let mut held = Vec::new();
    loop {
        let n = sub.drain_owned(1, &mut held).expect("drain status");
        if n == 0 {
            break;
        }
        for sample in &held {
            let frame = sample.payload();
            let body = &frame[cerulion_core::wire::WireHeader::SIZE..];
            out.push(serde_json::from_slice(body).expect("status body parses as JSON"));
        }
        held.clear();
    }
}

// ============================================================
// Test 1 — e2e happy path: 2 topics + a trace ring, byte-exact.
// ============================================================

#[test]
fn e2e_happy_path_records_frames_and_trace_and_manifest() {
    const N: usize = 3;
    let mgr = make_manager(32);
    let topic_a = unique_topic("happyA");
    let topic_b = unique_topic("happyB");
    let out = unique_out("happy");
    let ready = unique_out("happy_ready");

    // Publishers create the services (default borrow budget) BEFORE the taps.
    let mut pub_a = publisher(&mgr, &topic_a, 512);
    let mut pub_b = publisher(&mgr, &topic_b, 512);

    // A trace ring the owner + producer feed — created WITH the
    // per-node input section so the manifest-attachment `inputs` pin below
    // exercises bagd reading it off the ring (the production recorder shape).
    let ring_tag = unique_ring_tag("happy");
    let mut owner = TraceRingOwner::create_with_inputs_publishers_and_capacities_or_degrade(
        &ring_tag,
        16,
        7,
        &["n0", "n1"],
        &[&[], &["inp_a", "inp_b"]],
        // And the additive PUBLISHER section, so the
        // `publishers` pin below exercises bagd reading it off the ring
        // (the production recorder shape).
        &[
            &[("out", 0x0102_0304_0506_0708_090A_0B0C_0D0E_0F10u128)],
            &[],
        ],
        // And the additive CAPACITY section — the per-stage
        // staging rims — so the `read_log_capacities` pin below exercises bagd
        // reading them off the ring, which is the production recorder shape.
        // `n1`'s two inputs each carry one BODY stage (role byte 0); the rims
        // are hand-chosen and DISTINCT so a stamp that collapsed the rows, or
        // read the wrong one, cannot pass.
        &[&[], &[(0, 0, 22), (1, 0, 130)]],
    )
    .expect("ring create");
    let mut producer = owner.producer().expect("producer");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    cfg.rings = vec![owner.name().to_string()];

    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    // Hand-build + publish 3 distinct frames per topic (interleaved).
    let mut expect_a: Vec<Vec<u8>> = Vec::new();
    let mut expect_b: Vec<Vec<u8>> = Vec::new();
    for i in 0..N {
        let fa = build_frame(
            0xA000 + i as u64,
            i as u32,
            1000 + i as u64,
            &vec![0xA0 + i as u8; 8 + i],
        );
        let fb = build_frame(
            0xB000 + i as u64,
            i as u32,
            2000 + i as u64,
            &vec![0xB0 + i as u8; 5 + i],
        );
        pub_a.publish_raw(&fa).expect("publish A");
        pub_b.publish_raw(&fb).expect("publish B");
        expect_a.push(fa);
        expect_b.push(fb);
    }

    // Push 4 known trace records. Each is pushed with the on-ring reserved = 0;
    // multi-process recording requires bagd to OVERWRITE reserved with
    // the ring's HEADER rank (7 here) when writing to the bag, so the oracle
    // carries reserved = 7 — an end-to-end proof the rank stamp survives verbatim
    // into the finalized bag.
    let mut expect_trace: Vec<TraceRingRecord> = Vec::new();
    for i in 0..4u64 {
        let rec = TraceRingRecord {
            step: i,
            fire_time_ns: 5000 + i,
            duration_ns: 10 + i,
            node_idx: (i % 2) as u32,
            global_level: 0,
            record_type: 1,
            reserved: 0,
        };
        producer.push(&rec);
        // Oracle carries the stamped rank (7), not the on-ring 0.
        expect_trace.push(TraceRingRecord { reserved: 7, ..rec });
    }

    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");

    assert_eq!(summary.messages, (2 * N) as u64, "all data frames recorded");
    assert_eq!(summary.ring_records, 4, "all trace records recorded");
    assert_eq!(summary.bag_paths, vec![out.clone()]);

    let reader = BagReader::open(&out).expect("open bag");

    // Byte-exact per-topic frames, in arrival order.
    assert_eq!(
        data_frames_for(&reader, &topic_a),
        expect_a,
        "topic A frames"
    );
    assert_eq!(
        data_frames_for(&reader, &topic_b),
        expect_b,
        "topic B frames"
    );

    // The no-wall-clock WRITE-path contract, pinned EXACTLY:
    // recorded MCAP sequence == the wire sequence and log_time == publish_time
    // == the hand-chosen wire timestamp_ns (1000+i on A, 2000+i on B). The
    // values are hand-picked oracles, so any wall-clock/fabricated stamp fails.
    for (i, m) in data_msgs_for(&reader, &topic_a).iter().enumerate() {
        assert_eq!(
            m.sequence, i as u32,
            "topic A msg {i}: sequence == wire seq"
        );
        assert_eq!(
            m.log_time,
            1000 + i as u64,
            "topic A msg {i}: log_time == wire timestamp_ns"
        );
        assert_eq!(
            m.publish_time, m.log_time,
            "topic A msg {i}: publish_time == log_time"
        );
    }
    for (i, m) in data_msgs_for(&reader, &topic_b).iter().enumerate() {
        assert_eq!(
            m.sequence, i as u32,
            "topic B msg {i}: sequence == wire seq"
        );
        assert_eq!(
            m.log_time,
            2000 + i as u64,
            "topic B msg {i}: log_time == wire timestamp_ns"
        );
        assert_eq!(
            m.publish_time, m.log_time,
            "topic B msg {i}: publish_time == log_time"
        );
    }

    // Scheduler trace == the pushed records, in order, with reserved stamped to
    // the ring's rank (7) — the provenance stamp applied by bagd.
    let trace = reader.scheduler_trace().expect("scheduler_trace");
    assert_eq!(
        trace, expect_trace,
        "trace records match pushed, with reserved stamped to the ring rank (7)"
    );

    // Manifest attachment present + correct.
    let att = reader
        .attachment("__cerulion/trace_manifest_rank7.json")
        .expect("attachments")
        .expect("manifest present");
    let json: serde_json::Value = serde_json::from_slice(&att.data).expect("manifest json");
    assert_eq!(json["rank"], 7);
    assert_eq!(json["node_ids"], serde_json::json!(["n0", "n1"]));
    // Bagd stamps the ring's per-node input section
    // into the manifest attachment's `inputs` key — the offline resolver for
    // a kind-6 record's input_idx (a NON-empty list for the consumer node,
    // an empty one for the input-less node).
    assert_eq!(
        json["inputs"],
        serde_json::json!({ "n0": [], "n1": ["inp_a", "inp_b"] }),
        "manifest `inputs` == the ring's input section, keyed by node id"
    );
    // The AUTHORITATIVE staging table plus the scalar SENTINEL.
    //
    // `read_log_capacities` carries one `[input_idx, role, capacity]` row per
    // STAGE — keyed on the pair, never on position, because an input under the
    // Separate or legacy-`Sync` discipline carries two stages that share an
    // index. The replay adopts these rims (decision F1) so both sides truncate
    // identically.
    //
    // The expected values are LITERALS, never the writer's own expression: a
    // changed stamp would otherwise sail through the one test that exists to
    // catch it. `n0` has no read stages (no wired inputs); `n1`'s two inputs
    // each carry one BODY stage (role byte 0) at the rim this graph derives.
    let caps = &json["read_log_capacities"];
    assert!(
        caps.is_object(),
        "the manifest carries the per-input staging table: {caps}"
    );
    assert_eq!(caps["n0"], serde_json::json!([]), "n0 wires no inputs");
    assert_eq!(
        caps["n1"],
        serde_json::json!([[0, 0, 22], [1, 0, 130]]),
        "the rows reach the bag VERBATIM from the ring's capacity section — \
         `[input_idx, role, capacity]`, with the two DISTINCT rims proving the \
         stamp did not collapse or transpose them"
    );
    // And the scalar is the VERSION SENTINEL now — not a capacity, and
    // deliberately `0` so a PRE-1487 binary compares it against its own linked
    // 320, stands the read log down loudly, and never claims a match it cannot
    // back. Literal, for the same reason as above.
    assert_eq!(
        json["read_log_capacity"],
        serde_json::json!(0),
        "manifest carries the sentinel, not a capacity"
    );
    // The sentinel and the table ride one condition. This rank
    // HAS stages, so both keys are present — the arm below covers the other
    // side of that condition, and the pair is what a source-only rank depends
    // on.
    assert!(
        json.get("read_log_capacities").is_some() && json.get("read_log_capacity").is_some(),
        "a rank WITH stages declares both keys: {json}"
    );
    // The additive `publishers` key — the ring's per-node
    // OUTPUT table, keyed by the publisher's raw id as 32 lowercase hex
    // digits (a `u128` has no JSON number type) and valued by
    // `[node, output]`. It is what turns a bagged producer annotation's
    // 64-bit token back into a name offline.
    assert_eq!(
        json["publishers"],
        serde_json::json!({ "0102030405060708090a0b0c0d0e0f10": ["n0", "out"] }),
        "manifest `publishers` == the ring's publisher section, keyed by hex id"
    );

    // Independent `mcap`-crate oracle: same file, same data payloads.
    let bytes = std::fs::read(&out).expect("read bag");
    let mut oracle_a: Vec<Vec<u8>> = Vec::new();
    let mut trace_count = 0usize;
    for item in mcap::MessageStream::new(&bytes).expect("mcap stream") {
        let m = item.expect("mcap message");
        if m.channel.topic == topic_a {
            oracle_a.push(m.data.into_owned());
        } else if m.channel.topic == SCHEDULER_TRACE_TOPIC {
            trace_count += 1;
        }
    }
    assert_eq!(oracle_a, expect_a, "mcap oracle topic A payloads match");
    assert_eq!(trace_count, 4, "mcap oracle trace record count");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Test 2 — pool-exhaustion cadence: borrow=3, hundreds of frames, no loss.
// ============================================================

#[test]
fn pool_exhaustion_cadence_records_all_frames_no_err() {
    const N: usize = 300;
    // Queue depth must hold all N so nothing drops at the transport layer.
    let mgr = make_manager(16);
    let topic = unique_topic("cadence");
    let out = unique_out("cadence");
    let ready = unique_out("cadence_ready");

    // Provision borrow=3 (held budget 2) + a deep queue.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 3, N + 16, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::with_capacity(N);
    for i in 0..N {
        let frame = build_frame(0xC0DE, i as u32, i as u64, &[(i % 251) as u8; 16]);
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }

    // Let bagd drain everything (budget-triggered flush keeps the pool free).
    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle
        .join()
        .expect("join")
        .expect("run_bagd Ok (no ExceedsMaxBorrows)");

    assert_eq!(
        summary.messages, N as u64,
        "every published frame recorded (no starvation)"
    );

    let reader = BagReader::open(&out).expect("open");
    let got = data_frames_for(&reader, &topic);
    assert_eq!(got.len(), N, "no frames lost");
    assert_eq!(got, expect, "frames byte-match in FIFO order");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Test 4 — ready-file appears only after taps armed.
// ============================================================

#[test]
fn ready_file_written_only_after_taps_armed() {
    let mgr = make_manager(16);
    let topic = unique_topic("ready");
    let out = unique_out("ready");
    let ready = unique_out("ready_sentinel");
    let _ = std::fs::remove_file(&ready);

    let _pubr = publisher(&mgr, &topic, 256);

    // Before run_bagd, the sentinel must NOT exist.
    assert!(!ready.exists(), "ready-file must not pre-exist");

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());

    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file must appear once the tap is armed"
    );

    shutdown.store(true, Ordering::Relaxed);
    let _ = handle.join().expect("join").expect("run_bagd Ok");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Test 5 — rotation: tiny size-cap → ≥2 files, concat stream == published.
// ============================================================

#[test]
fn rotation_produces_multiple_readable_files_concat_matches() {
    const N: usize = 12;
    const BODY: usize = 1024;
    let mgr = make_manager(16);
    let topic = unique_topic("rot");
    let out = unique_out("rot");
    let ready = unique_out("rot_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 2, N + 8, (BODY + 128) as u32);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.size_cap_bytes = Some(3000); // tiny — forces multiple rolls
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::with_capacity(N);
    for i in 0..N {
        let frame = build_frame(
            0x501,
            i as u32,
            i as u64,
            &[(i as u8).wrapping_add(1); BODY],
        );
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
        // Pace slightly so flush/rotation runs between frames.
        std::thread::sleep(Duration::from_millis(10));
    }

    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");

    assert!(
        summary.bag_paths.len() >= 2,
        "size cap must produce ≥2 files, got {:?}",
        summary.bag_paths
    );
    assert_eq!(summary.messages, N as u64, "all frames across all files");

    // Each file independently readable + Finalized; concat == published order.
    let mut concat: Vec<Vec<u8>> = Vec::new();
    for path in &summary.bag_paths {
        let reader = BagReader::open(path).expect("open rotation file");
        concat.extend(data_frames_for(&reader, &topic));
    }
    assert_eq!(
        concat, expect,
        "concatenated rotation streams == published order"
    );

    // EVERY rotation file's channel set (topic, schema_name,
    // schema_hash, wire_fixed_size) must be identical to file 1's, and the
    // tapped channel must carry the LEARNED wire hash (0x501 — every published
    // frame's schema_hash), never a hash-0 placeholder in a later file.
    let first_reader = BagReader::open(&summary.bag_paths[0]).expect("open first file");
    let first_channels = channel_tuples(&first_reader);
    let tapped = first_channels
        .iter()
        .find(|(t, _, _, _)| t == &topic)
        .expect("tapped channel in file 1");
    assert_eq!(tapped.1, "unknown", "attach-mode name");
    assert_eq!(tapped.2, 0x501, "file 1 carries the learned wire hash");
    assert_eq!(tapped.3, 0, "attach-mode wire_fixed_size");
    for path in &summary.bag_paths[1..] {
        let reader = BagReader::open(path).expect("open rotation file");
        assert_eq!(
            channel_tuples(&reader),
            first_channels,
            "rotation file {} must carry an IDENTICAL channel set to file 1",
            path.display()
        );
    }

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Test 6 — ring overrun → run_bagd Err(RingOverrun), bag NOT Finalized.
// ============================================================

#[test]
fn ring_overrun_fails_loud_and_leaves_bag_unfinalized() {
    let mgr = make_manager(16);
    let topic = unique_topic("overrun");
    let out = unique_out("overrun");
    let ready = unique_out("overrun_ready");

    // Deep borrow budget (16 -> held budget 15) so the pre-overrun data frames
    // are still HELD (not at_budget-flushed) when the ring error fires -- the
    // salvage assertion below needs held-but-unflushed frames at overrun.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 16, 32, 256);

    // A TINY (capacity 2) ring — the owner + producer will lap it.
    let ring_tag = unique_ring_tag("overrun");
    let mut owner = TraceRingOwner::create(&ring_tag, 2, 3, &["only"]).expect("ring create");
    let mut producer = owner.producer().expect("producer");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.rings = vec![owner.name().to_string()];
    cfg.schema_wait = Duration::from_millis(200);
    // Long interval: nothing flushes before the overrun, so the held frames
    // can only reach the bag through the choke-point salvage.
    cfg.flush_interval = Duration::from_secs(10);
    // Status ON so the TERMINAL failed/overrun frame is pinned.
    cfg.status_period = Some(Duration::from_millis(50));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    // Status subscriber connects BEFORE the failure so the terminal frame
    // (published at error time) is queued on it.
    let mut status_sub = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // Record a couple of data frames first: they are drained-and-HELD (deep
    // budget + 10s flush interval means no flush fires), so at overrun time
    // they exist ONLY in bagd's hands -- the salvage must write them.
    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..2u32 {
        let frame = build_frame(0x006E, i, i as u64, &[0x11; 32]);
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }
    settle();
    settle(); // frames drained into held; writer created (schema learned)

    // Now lap the ring hard: 64 records into a capacity-2 ring.
    for i in 0..64u64 {
        producer.push(&TraceRingRecord {
            step: i,
            fire_time_ns: i,
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: 1,
            reserved: 0,
        });
    }

    // run_bagd must return RingOverrun on its own (no shutdown needed).
    let result = handle.join().expect("join");
    match result {
        Err(BagdError::RingOverrun { records_lost, .. }) => {
            assert!(
                records_lost >= 1,
                "records_lost should be >0, got {records_lost}"
            );
        }
        other => panic!("expected RingOverrun, got {other:?}"),
    }

    // The file MUST exist — an `if` here would let the key
    // completeness check silently skip. Nothing
    // flushes before the overrun (deep budget + 10s interval — that is the
    // point): the file exists because the writer PRELUDE was written at
    // creation, and the held frames reach it via the salvage flush at error
    // time.
    assert!(
        out.exists(),
        "a bag file must exist: the writer was created (prelude on disk) and the salvage \
         flushed the held frames at error time"
    );
    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "overran bag must NOT be Finalized, got {completeness:?}"
    );
    // The held-but-never-interval-flushed frames were
    // SALVAGED into the truncated bag by the error choke point -- without the
    // salvage they would have been silently destroyed at Recorder drop.
    let frames: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(
        frames, expect,
        "the held pre-overrun frames must be SALVAGED into the truncated bag"
    );

    // The TERMINAL status frame reports the failure. Best-effort
    // publish, but in this test the status publisher exists and publish_raw is
    // synchronous, so the frame MUST be queued by the time join returned —
    // bounded drain, hard assertion (absence would only be tolerable if the
    // status publisher itself had failed to create, which this test's setup
    // makes impossible).
    let mut statuses: Vec<serde_json::Value> = Vec::new();
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    let terminal = loop {
        drain_status_json(&mut status_sub, &mut statuses);
        if let Some(t) = statuses
            .iter()
            .find(|s| s["state"].as_str() == Some("failed"))
        {
            break t.clone();
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no terminal failed status frame arrived; got {statuses:?}"
        );
        std::thread::sleep(Duration::from_millis(10));
    };
    assert_eq!(
        terminal["overrun"],
        serde_json::json!(true),
        "the overrun failure must set overrun=true in the terminal status"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Test 7 — attach-mode schema derivation: channel carries the wire hash,
//          name "unknown".
// ============================================================

#[test]
fn attach_mode_channel_carries_wire_hash_named_unknown() {
    const HASH: u64 = 0x0102_0304_0506_0708;
    let mgr = make_manager(16);
    let topic = unique_topic("attach");
    let out = unique_out("attach");
    let ready = unique_out("attach_ready");

    let mut pubr = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let frame = build_frame(HASH, 0, 42, &[0xEE; 12]);
    pubr.publish_raw(&frame).expect("publish");
    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let _ = handle.join().expect("join").expect("run_bagd Ok");

    let reader = BagReader::open(&out).expect("open");
    let chan = reader
        .channels()
        .expect("channels")
        .into_iter()
        .find(|c| c.topic == topic)
        .expect("tapped channel present");
    assert_eq!(
        chan.schema_name, "unknown",
        "attach-mode name is \"unknown\""
    );
    let desc = chan.descriptor.expect("cerulion descriptor");
    assert_eq!(
        desc.schema_hash, HASH,
        "channel carries the wire schema_hash"
    );
    assert_eq!(desc.wire_fixed_size, 0, "attach-mode wire_fixed_size is 0");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Test 8 — missing topic at tap → loud Err, exit non-zero, zero services.
// ============================================================

#[test]
fn missing_topic_tap_errors_and_creates_no_service() {
    let mgr = make_manager(16);
    // Never create a publisher for this topic → the service does not exist.
    let topic = unique_topic("missing");
    let out = unique_out("missing");

    let mut cfg = BagdConfig::new(out.clone(), vec![TapSpec::attach(&topic)]);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.status_period = None;
    cfg.schema_wait = Duration::from_millis(200);

    // run_bagd (in-line, no thread) must error at tap-open time.
    let shutdown = Arc::new(AtomicBool::new(false));
    let result = run_bagd(mgr.clone(), cfg, shutdown);
    assert!(
        matches!(result, Err(BagdError::Transport(_))),
        "missing topic must be a loud transport error, got {result:?}"
    );

    // open-only guarantees zero services created for the missing topic.
    assert_eq!(
        mgr.topic_subscriber_count(&topic),
        0,
        "no subscriber service should exist for the missing topic"
    );
    assert_eq!(
        mgr.topic_publisher_count(&topic),
        0,
        "no publisher service should exist for the missing topic"
    );

    // No bag file should have been created (setup failed before writer creation).
    assert!(
        !out.exists(),
        "no bag file on the missing-topic failure path"
    );
}

// ============================================================
// Test 9 — /bagd/status: JSON frames arrive with the expected fields,
//          counters are monotonic, and the bag does NOT tap its own status.
// ============================================================

#[test]
fn status_topic_publishes_json_and_is_not_recorded_in_the_bag() {
    const N: usize = 3;
    let mgr = make_manager(16);
    let topic = unique_topic("status");
    let out = unique_out("status");
    let ready = unique_out("status_ready");

    let mut pubr = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.status_period = Some(Duration::from_millis(50));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );

    // bagd's setup created the /bagd/status publisher BEFORE the ready-file, so
    // the service exists — attach a plain test subscriber on the SAME manager.
    let mut status_sub = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    settle();

    // Publish N data frames so the counters have something to count.
    for i in 0..N {
        let frame = build_frame(0x57A7, i as u32, i as u64, &[0x22; 16]);
        pubr.publish_raw(&frame).expect("publish");
    }

    // Collect status payloads until one shows all N messages flushed (bounded).
    // Values mid-run are racy (flush cadence vs status cadence), so the pin is
    // presence + type + MONOTONICITY, and the terminal "counted all N" state.
    let mut statuses: Vec<serde_json::Value> = Vec::new();
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    let mut held = Vec::new();
    'collect: while std::time::Instant::now() < deadline {
        // Default provisioning → small borrow budget; drain one, copy, release.
        let n = status_sub.drain_owned(1, &mut held).expect("drain status");
        if n == 0 {
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }
        for sample in &held {
            let frame = sample.payload();
            assert!(
                frame.len() > cerulion_core::wire::WireHeader::SIZE,
                "status frame must carry a JSON body past the wire header"
            );
            let body = &frame[cerulion_core::wire::WireHeader::SIZE..];
            let json: serde_json::Value =
                serde_json::from_slice(body).expect("status body parses as JSON");
            statuses.push(json.clone());
            if json["messages"].as_u64() == Some(N as u64) {
                held.clear();
                break 'collect;
            }
        }
        held.clear();
    }
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    assert_eq!(summary.messages, N as u64, "all data frames recorded");

    // The TERMINAL status frame. run_bagd publishes
    // state="finalized" right before returning Ok (publish_raw is synchronous,
    // so by join time it is queued on our subscriber) — drain everything that
    // arrived after the collect loop stopped and pin the LAST frame.
    drain_status_json(&mut status_sub, &mut statuses);
    assert!(!statuses.is_empty(), "no /bagd/status frame ever arrived");
    let last = statuses.last().expect("non-empty");
    assert_eq!(
        last["state"].as_str(),
        Some("finalized"),
        "the LAST status frame after a clean run must be the terminal \
         state=\"finalized\", got {last}"
    );
    assert_eq!(
        last["messages"].as_u64(),
        Some(N as u64),
        "the terminal status must count all {N} flushed messages, got {last}"
    );

    // Field presence + types on every captured status.
    let mut prev_messages = 0u64;
    let mut prev_bytes = 0u64;
    for s in &statuses {
        assert!(
            matches!(
                s["state"].as_str(),
                Some("recording") | Some("finalizing") | Some("finalized")
            ),
            "state must be recording|finalizing|finalized, got {s}"
        );
        assert!(s["bag_path"].is_string(), "bag_path must be a string: {s}");
        let m = s["messages"].as_u64().expect("messages is u64");
        let b = s["bytes"].as_u64().expect("bytes is u64");
        assert!(s["chunks"].is_u64(), "chunks must be u64: {s}");
        assert!(s["ring_records"].is_u64(), "ring_records must be u64: {s}");
        // The anomaly counters ride the status JSON too.
        assert_eq!(
            s["headerless"].as_u64(),
            Some(0),
            "headerless must be a present-and-zero u64 in this clean run: {s}"
        );
        assert_eq!(
            s["dropped_unwritten"].as_u64(),
            Some(0),
            "dropped_unwritten must be a present-and-zero u64 in this clean run: {s}"
        );
        assert!(
            s["per_topic"].is_object(),
            "per_topic must be an object: {s}"
        );
        assert_eq!(s["overrun"], serde_json::json!(false));
        // Monotonic counters across successive statuses.
        assert!(
            m >= prev_messages,
            "messages must be monotonic: {statuses:?}"
        );
        assert!(b >= prev_bytes, "bytes must be monotonic: {statuses:?}");
        prev_messages = m;
        prev_bytes = b;
    }
    // The terminal status names the bag file.
    assert_eq!(
        last["bag_path"].as_str(),
        Some(out.display().to_string().as_str()),
        "the terminal status must carry the bag path"
    );

    // The finalized bag must NOT contain /bagd/status as a channel (bagd never
    // taps its own status topic).
    let reader = BagReader::open(&out).expect("open bag");
    let channels = reader.channels().expect("channels");
    assert!(
        channels.iter().any(|c| c.topic == topic),
        "the tapped data topic must be a channel"
    );
    assert!(
        !channels
            .iter()
            .any(|c| c.topic == cerulion_bagd::STATUS_TOPIC),
        "/bagd/status must NOT be a bag channel: {channels:?}"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Test 10 — a silent sibling tap must NOT cost a speaking tap its
//           frames. A tap at its borrow budget before bag creation WAITS; the
//           burst is preserved in the topic's own SHM queue and recorded in
//           full once the writer force-creates.
// ============================================================

/// This test previously pinned the OPPOSITE contract — `dropped_unwritten ==
/// BURST - BUDGET`, i.e. the recorder drop-oldest-EVICTED 4 of these 6 frames.
/// That eviction probe was retired because waiting strictly dominates it
/// (see `Recorder::drain_taps`): evicting keeps at most a budget's worth of
/// frames and destroys the rest, while waiting keeps those AND everything the
/// topic's own provisioned queue holds. The scenario is byte-for-byte the one the old test
/// ran; only the oracle moved, and it moved from "4 frames destroyed" to "zero
/// frames destroyed" — a strictly stronger claim on identical stimulus.
///
/// Restoring the eviction arm makes this fail at 2 recorded / 4
/// dropped.
#[test]
#[tracing_test::traced_test]
fn a_silent_sibling_tap_does_not_cost_a_speaking_tap_its_frames() {
    const BURST: usize = 6;
    // Provisioned subscriber_max_borrowed_samples = 3 → held budget = 3-1 = 2,
    // i.e. the burst is 3x the budget: every frame past the 2nd depends on the
    // tap WAITING rather than evicting.
    const BUDGET: usize = 2;
    // ANTI-VACUITY: the burst must EXCEED the held budget, or the tap never
    // reaches the `room == 0` branch this test exists to pin and every
    // assertion below would hold for a recorder that still evicted.
    const _: () = assert!(BURST > BUDGET);
    // The topic's own queue is deep enough to hold the whole burst, so the
    // loss boundary this test probes is never reached (its overflow twin below
    // probes the other side of that boundary).
    const QUEUE_DEPTH: usize = BURST + 8;
    const _: () = assert!(QUEUE_DEPTH >= BURST);
    let mgr = make_manager(16);
    let topic_a = unique_topic("dropA");
    let topic_b = unique_topic("dropB");
    let out = unique_out("drop");
    let ready = unique_out("drop_ready");

    let mut pub_a = publisher_with_provisioning(&mgr, &topic_a, 3, QUEUE_DEPTH, 256);
    // Topic B's service must EXIST (open-only tap) but stays SILENT — the
    // attach-mode writer cannot be created until B's schema-wait expires.
    let _pub_b = publisher(&mgr, &topic_b, 256);

    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    // Long enough that the whole burst sits pre-writer for many ~10ms wait
    // cycles — the window in which the retired probe used to shred it.
    cfg.schema_wait = Duration::from_millis(1200);

    // Helper thread publishes the burst + flips shutdown; run_bagd runs INLINE
    // on THIS thread so any eviction warn would land inside `#[traced_test]`'s
    // captured span (a worker-thread warn would escape the test's span scope).
    let shutdown = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();
    let out_c = out.clone();
    let shutdown_c = shutdown.clone();
    let publisher_thread = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        for i in 0..BURST {
            let frame = build_frame(0xD809, i as u32, i as u64, &[(i as u8) + 1; 16]);
            pub_a.publish_raw(&frame).expect("publish burst");
        }
        // Outlive the schema-wait so the writer creates, then stop — WAITED FOR on
        // the writer's own observable rather than slept. The bag FILE is created
        // when the channel set closes and the writer comes up (the same signal
        // `ingress_build_e2e_test` reads to prove a ceiling released a held bag),
        // so this is the event the old fixed 2.2 s was a bet on. The injected
        // `schema_wait` above is UNCHANGED: the burst still sits pre-writer for
        // the whole 1.2 s grace, which is the stimulus this arm pins.
        assert!(
            wait_for_file(&out_c, LIVENESS_CEILING),
            "the writer never created the bag, so the burst never got past the \
             schema-wait window this arm exists to survive"
        );
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    publisher_thread.join().expect("publisher thread");

    // THE PIN: nothing is destroyed. The burst is 3x the held budget and sat
    // pre-writer for over a second, and every frame still reaches the bag.
    assert_eq!(
        summary.dropped_unwritten, 0,
        "a tap at its borrow budget must WAIT, not evict (got {})",
        summary.dropped_unwritten
    );
    assert_eq!(
        summary.frames_lost, 0,
        "the topic's queue is deeper than the burst, so nothing overflows either"
    );
    assert_eq!(
        summary.messages, BURST as u64,
        "every published frame must reach the bag"
    );
    assert_eq!(summary.per_topic.get(&topic_a), Some(&(BURST as u64)));
    assert_eq!(summary.per_topic.get(&topic_b), None, "B stayed silent");

    // NO log-absence predicate for the retired eviction warn. It would be
    // VACUOUS: that message no longer exists anywhere in the workspace, so
    // `!logs_contain(..)` is permanently true and cannot fail — not even under
    // a variant that restores the eviction, which drops frames with no
    // logging at all. `dropped_unwritten == 0` above is the real pin (that variant fails
    // exactly there, at 4 — deterministic here, unlike the overflow twin's
    // count, because the whole burst is published before the first drain and
    // the queue holds all of it, so the drops are exactly BURST - BUDGET), and
    // the byte-exact ordered oracle below catches an eviction that somehow
    // forgot to count itself.
    //
    // A clean run reports clean: the WITH-ANOMALIES escalation keys on exactly
    // the counters asserted zero above (its positive arm lives on the overflow
    // twin below and on the frames_lost tests).
    assert!(
        !logs_contain("WITH ANOMALIES"),
        "a run that lost nothing must not print the anomalies terminal line"
    );
    // Creation still force-creates on the schema-wait with B's placeholder —
    // waiting changed WHAT SURVIVES, never WHEN the bag is created.
    assert!(
        logs_contain("PLACEHOLDER schema"),
        "the silent tap must still be loudly named at writer creation"
    );

    // Byte-exact, in publish order — the head of the burst is the frame the
    // retired probe used to destroy first, so ordering is load-bearing here.
    let reader = BagReader::open(&out).expect("open bag");
    let frames = data_frames_for(&reader, &topic_a);
    assert_eq!(frames.len(), BURST);
    for (k, frame) in frames.iter().enumerate() {
        assert_eq!(
            frame[cerulion_core::wire::WireHeader::SIZE],
            (k + 1) as u8,
            "frame {k} must be the k-th PUBLISHED frame (nothing dropped, nothing reordered)"
        );
    }

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// Flip a `run_bagd` shutdown flag on Drop — INCLUDING on unwind.
///
/// The arms here run their stimulus on one thread and `run_bagd` on the other,
/// with the shutdown store as the stimulus thread's LAST statement. A stimulus
/// assertion that fires therefore panics BEFORE that store, `run_bagd` keeps
/// looping forever, and the arm HANGS instead of failing — which in CI is a
/// job-timeout cancellation with no attributable red (the class this repo
/// tracks separately as CI health), strictly worse than the assertion it
/// was trying to report. MEASURED while hardening this file: under a 16-way
/// spinner load at background QoS the pre-existing 5 s ready-file ceiling blew,
/// and the arm sat in `drive_loop` for ELEVEN MINUTES with its stimulus thread
/// already gone.
///
/// Holding one of these makes the flip unwind-safe, so the panic surfaces
/// through `join().expect(..)` carrying its own message. It does NOT replace the
/// explicit store on the happy path — that one still controls WHEN the run
/// stops, which several oracles depend on.
struct ShutdownOnDrop(Arc<AtomicBool>);

impl Drop for ShutdownOnDrop {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Relaxed);
    }
}

/// A LIVENESS backstop for the arms `ShutdownOnDrop` structurally
/// cannot reach — and it is a SECOND hang class, not a variant of the first.
///
/// [`ShutdownOnDrop`] covers arms whose stimulus runs on a spawned thread while
/// `run_bagd` runs INLINE: there the guard rides the stimulus closure and an
/// unwinding assertion flips the flag on its way out.
///
/// These arms are the INVERSE. `spawn_bagd` puts `run_bagd` on the worker, the
/// stimulus IS the main thread, and the run is expected to SELF-TERMINATE on an
/// injected fault (a lapped ring, a poisoned drain, a failed append) — so the
/// shutdown flag is never stored at all and `handle.join()` is unconditional. A
/// guard has nothing to guard: if the fault regresses and the run keeps going,
/// main blocks in `join()` forever. That is worse than a red — a wedged job
/// reads as "still running" and burns the whole CI budget with no attributable
/// failure (MEASURED on a real flake: a blown ceiling left an arm in `drive_loop`
/// for ELEVEN MINUTES).
///
/// So the flag is flipped by a watchdog instead, which turns the wedge back into
/// the arm's own `panic!("expected ...")`: the run finalizes, `join()` returns
/// `Ok`, and the `match` that was written to catch exactly this regression gets
/// to run. The ceiling is a liveness backstop and nothing else — every arm using
/// it completes in well under a second — so it can be enormously generous and
/// still never gate a healthy run.
///
/// Returned unjoined ON PURPOSE (the [`empty_taps_rejected_with_config_error`]
/// idiom this generalizes): joining would re-introduce the very wait it removes,
/// and a detached sleeper in a test process that is about to exit costs nothing.
#[must_use = "drop the handle explicitly so it reads as deliberately unjoined"]
fn shutdown_watchdog(shutdown: &Arc<AtomicBool>, after: Duration) -> JoinHandle<()> {
    let flag = shutdown.clone();
    std::thread::spawn(move || {
        std::thread::sleep(after);
        flag.store(true, Ordering::Relaxed);
    })
}

/// Block until a `/bagd/status` frame reports at least `want` ring
/// records CONSUMED — the recorder's own progress, not a wall.
///
/// The ring's read cursor is consumer-LOCAL (no shared header field), so a
/// producer-side test cannot see it directly; `ring_records` on the status feed
/// folds the writer thread's live counter and is the one place that number
/// surfaces mid-run. Load can only DELAY the frame, so the timeout is a liveness
/// backstop rather than the property.
fn await_status_ring_records(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    want: u64,
    timeout: Duration,
) -> bool {
    let mut seen: Vec<serde_json::Value> = Vec::new();
    let start = Instant::now();
    loop {
        drain_status_json(sub, &mut seen);
        if seen
            .iter()
            .any(|s| s["ring_records"].as_u64().is_some_and(|n| n >= want))
        {
            return true;
        }
        if start.elapsed() >= timeout {
            return false;
        }
        std::thread::sleep(FILE_POLL_INTERVAL);
    }
}

/// The shared liveness ceiling for waits that are NOT the property
/// under test.
///
/// Every wait it bounds is a CONDITION (a file appearing, a log line, a run
/// self-terminating), so load can only DELAY it — which is why the value is
/// deliberately enormous against the sub-second healthy paths it covers. A CI flake
/// MEASURED the cost of getting this wrong in the other direction: under a
/// 16-way spinner load at background QoS the file's pre-existing 5 s ready-file
/// ceilings blew while bagd's own log showed the ready file written moments
/// later, and the arm then WEDGED rather than failing.
const LIVENESS_CEILING: Duration = Duration::from_secs(30);

/// Drain every `/bagd/status` frame currently queued, folding the highest wire
/// `sequence` seen into `newest`.
///
/// A shallow subscriber queue may DROP frames, and that is harmless here for a
/// specific reason: the queue is drop-OLDEST, so the newest frame published is
/// never the one discarded — the maximum this reads is therefore the newest
/// status bagd had published when the drain found the queue empty, which is
/// exactly what [`await_drain_gate_quiesced`] reasons about.
fn fold_newest_status_seq(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    newest: &mut Option<u32>,
) {
    let mut held = Vec::new();
    loop {
        let n = sub.drain_owned(1, &mut held).expect("drain status");
        if n == 0 {
            break;
        }
        for sample in &held {
            let seq = cerulion_core::wire::WireHeader::read_from_buf(sample.payload())
                .expect("a /bagd/status frame carries a parseable wire header")
                .sequence;
            *newest = Some(newest.map_or(seq, |cur| cur.max(seq)));
        }
        held.clear();
    }
}

/// Block until no `drain_taps` pass that read the drain gate as OPEN
/// can still be in flight. The POSITIVE handshake a gate CLOSE needs, and the
/// one thing a `sleep` cannot give.
///
/// `fault_inject_tap_drain_gate` is read at the TOP of `drain_taps`, so a pass
/// that checked it a microsecond before the test flipped it to `false` keeps
/// draining for the rest of that pass — and its inner loop runs until the tap's
/// queue reports EMPTY, so a producer that starts bursting into that window is
/// drained at full speed. That is the very race the gate exists to remove,
/// re-created at the close.
///
/// `/bagd/status` closes it. The drive loop is SINGLE-THREADED and publishes its
/// status frame at the BOTTOM of a pass (after the drain, the flush and the
/// writer check), so given a caller that has ALREADY closed the gate:
///
///   * draining the status queue to EMPTY establishes `h` = the newest sequence
///     published as of that drain, which finished at a wall instant strictly
///     AFTER the close;
///   * `h + 1` was therefore published strictly after that instant; and
///   * `h + 2` belongs to a pass that STARTED after `h + 1`'s pass ENDED, i.e.
///     strictly after the close — so ITS gate read is the closed one, and since
///     passes are sequential no earlier pass can still be running.
///
/// (`None` — no status published yet when we drained — takes the same argument
/// one step earlier: the first frame is sequence 0, so the target is 1.)
///
/// Load can only DELAY the target frame; it cannot make a gate-open pass
/// reappear. The timeout is a liveness backstop, never the property under test.
fn await_drain_gate_quiesced(
    sub: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    timeout: Duration,
) -> bool {
    let mut newest: Option<u32> = None;
    fold_newest_status_seq(sub, &mut newest);
    let target = newest.map_or(1, |h| h + 2);
    let start = Instant::now();
    while start.elapsed() < timeout {
        fold_newest_status_seq(sub, &mut newest);
        if newest.is_some_and(|h| h >= target) {
            return true;
        }
        std::thread::sleep(FILE_POLL_INTERVAL);
    }
    false
}

/// The accounting half: waiting MOVED the loss boundary, it did not
/// abolish loss — and what is lost past that boundary is still COUNTED.
///
/// Same silent-sibling shape, but the burst is far larger than topic A's own
/// provisioned queue, so the queue itself drop-oldest-evicts. Two claims:
///
///  1. the loss boundary really IS the topic's provisioned queue depth — the
///     recorder's own counter stays 0 and what survives is bounded by the
///     queue, not by the tap's TWO-frame borrow budget; and
///  2. that loss is reported per topic as `frames_lost` in
///     `record_health.json` (the GapObserver's wire-sequence gap), with
///     the terminal line escalated — not a silent hole.
///
/// THE OVERFLOW IS NOW DETERMINISTIC. The arm used to publish the burst
/// with the recorder draining freely and simply require the producer to WIN
/// (`frames_lost > 0`). That is a race-outcome expectation, and it inverted: a
/// loaded CI run starved the producer thread, the recorder kept up, nothing
/// was lost, and the arm failed. The direction was the mirror image of the
/// earlier flakes, which needed the DRAINER to win — which is the
/// point: a bet on a race breaks in whichever direction it was placed.
///
/// The burst now commits with `fault_inject_tap_drain_gate` CLOSED — the seam
/// `tap_queue_overflow_detected_with_exact_counts_and_record_health` and the
/// prefix-loss harness already use — so the recorder receives NOTHING
/// while it runs and iceoryx2 keeps exactly the newest [`QUEUE_DEPTH`] samples.
/// The overflow is then a function of the provisioned queue depth ALONE, which
/// is what lets every count below be EXACT instead of `> 0`: a strictly
/// stronger oracle on the same stimulus. The loss is still REAL — the frames
/// between the seed and the survivors are genuinely reclaimed by iceoryx2 and
/// genuinely absent from the bag. Only the RACE is gone.
///
/// The two windows the gate does not itself cover are closed POSITIVELY, never
/// by a sleep:
///
///   * THE SEED must be staged before the burst reclaims it. Otherwise the
///     tap's first-ever observation carries a nonzero sequence, `GapObserver`
///     takes it as the BASELINE — indistinguishable, by construction, from a
///     tap that attached mid-stream — and the run reports `frames_lost=0` with
///     everything below inverted the same way the CI failure inverted it (that
///     blind spot is stated as a residual on `Recorder::drain_taps` and is what
///     `prefix_lost` exists to name). `bagd learned attach-mode
///     schema` is emitted AFTER the tap's inner drain loop broke on an EMPTY
///     read, so that ONE line proves both that the seed is staged and that the
///     queue is dry. Hence exactly ONE seed frame, where the earlier arm
///     used two: with one, "the baseline exists" and "the whole seed is staged"
///     are the SAME fact. A second frame would be staged by the next pass with
///     overwhelming probability and nothing would PROVE it, leaving the exact
///     oracle ambiguous between 392 and 393.
///   * THE CLOSE itself — see [`await_drain_gate_quiesced`].
///
/// The pre-writer window is preserved (it is what the `dropped_unwritten == 0`
/// pin needs: the retired eviction fired only while `learned_all()` was false),
/// but it no longer rests on a wall clock. `schema_wait` is raised far past the
/// run so nothing force-creates, and the SILENT sibling B is what holds the bag
/// un-created across the whole burst; B then speaks ONCE at the end, which is
/// what creates the writer — a condition, not a deadline.
#[test]
#[tracing_test::traced_test]
fn a_burst_past_the_topics_own_queue_depth_still_loses_and_still_says_so() {
    // Every wait below is bounded by the file-wide [`LIVENESS_CEILING`] — a
    // backstop, never a property under test: the healthy path takes ~0.3 s end
    // to end, so a ceiling that generous costs nothing and stops a
    // pathologically-loaded runner from reddening an arm whose every wait is a
    // CONDITION. (The constant was later hoisted to module scope; it was declared
    // here first, and every arm in the file now shares one value.)
    const QUEUE_DEPTH: usize = 8;
    const SEED: usize = 1;
    const BURST: usize = 400;
    const PUBLISHED: usize = SEED + BURST;
    // With the drain gated for the whole burst, iceoryx2 keeps exactly the
    // newest QUEUE_DEPTH samples and reclaims the rest, so both counts are
    // arithmetic rather than an outcome.
    const RECORDED: u64 = (SEED + QUEUE_DEPTH) as u64;
    const LOST: u64 = (BURST - QUEUE_DEPTH) as u64;
    // Anti-vacuity: the burst must overrun the queue, or nothing is lost and
    // both claims below would hold vacuously.
    const _: () = assert!(BURST > QUEUE_DEPTH);
    // ...and conservation must be a real cross-check, not an identity that
    // holds however the run went.
    const _: () = assert!(RECORDED + LOST == PUBLISHED as u64);
    let mgr = make_manager(16);
    let topic_a = unique_topic("ovfA");
    let topic_b = unique_topic("ovfB");
    let out = unique_out("ovf");
    let ready = unique_out("ovf_ready");

    let mut pub_a = publisher_with_provisioning(&mgr, &topic_a, 3, QUEUE_DEPTH, 256);
    let mut pub_b = publisher(&mgr, &topic_b, 256);

    let gate = Arc::new(AtomicBool::new(true)); // OPEN for the seed
    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());
    // Nothing may force-create: B's silence is what holds the bag un-created,
    // and B breaks that silence on purpose at the end. A deadline here would
    // put the creation instant back on the wall clock under load.
    cfg.schema_wait = Duration::from_secs(120);
    // The gate-close handshake reads this feed; the period is short so the two
    // status frames it needs cost tens of milliseconds, not seconds.
    cfg.status_period = Some(Duration::from_millis(20));

    let shutdown = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let gate_c = gate.clone();
    let mgr_c = mgr.clone();
    let out_c = out.clone();
    let publisher_thread = std::thread::spawn(move || {
        // A panicking stimulus must FAIL the arm, never hang it (see the type's
        // doc); the explicit store at the end still owns the happy path.
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        // bagd's setup created the /bagd/status publisher BEFORE the ready-file,
        // so the service exists — attach open-only on the SAME manager.
        let mut status_sub = mgr_c
            .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
            .expect("status subscriber");
        settle();

        // PHASE 1 — SEED, gate OPEN. One frame, drained and staged, so the
        // GapObserver's baseline is sequence 0 and the burst's gap lands
        // BETWEEN observed sequences instead of ahead of all of them.
        pub_a
            .publish_raw(&build_frame(0xD80A, 0, 0, &[1u8; 16]))
            .expect("publish seed");
        assert!(
            await_condition(LIVENESS_CEILING, || logs_contain(
                "bagd learned attach-mode schema"
            )),
            "the tap must stage the seed before the burst can reclaim it — this line is \
             emitted only after the tap's drain loop broke on an EMPTY read, so it proves \
             both halves at once (topic B is silent, so it is A's line or none)"
        );

        // PHASE 2 — STALL. Close the gate, then WAIT OUT any pass that read it
        // as open (see `await_drain_gate_quiesced`; a sleep here would be the
        // same bet this arm exists to remove, just a smaller one).
        gate_c.store(false, Ordering::Relaxed);
        assert!(
            await_drain_gate_quiesced(&mut status_sub, LIVENESS_CEILING),
            "the recorder must reach a pass that began AFTER the gate closed"
        );

        // PHASE 3 — BURST, gate CLOSED. Nothing is draining, so the topic's own
        // 8-slot queue keeps the newest 8 and iceoryx2 reclaims the other 392.
        for i in SEED..PUBLISHED {
            let frame = build_frame(0xD80A, i as u32, i as u64, &[(i % 251) as u8; 16]);
            pub_a.publish_raw(&frame).expect("publish burst");
        }
        assert!(
            !out_c.exists(),
            "precondition: the burst must sit in the PRE-WRITER window — that is the \
             window the retired eviction probe fired in, and it is what makes the \
             `dropped_unwritten == 0` pin below a real kill of a restored eviction"
        );

        // PHASE 4 — RESUME. Reopen the gate so the survivors drain, then let B
        // speak ONCE: with both attach schemas known the writer is created by
        // `learned_all()`, not by a deadline.
        gate_c.store(true, Ordering::Relaxed);
        pub_b
            .publish_raw(&build_frame(0xB0B0, 0, 0, &[2u8; 16]))
            .expect("publish B");
        assert!(
            await_condition(LIVENESS_CEILING, || out_c.exists()),
            "the writer must be created once every attach-mode schema is known"
        );
        settle();
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    publisher_thread.join().expect("publisher thread");

    // (1) The recorder itself destroys nothing — the queue did.
    assert_eq!(
        summary.dropped_unwritten, 0,
        "the recorder never evicts; overflow is the TOPIC's queue, not the tap's budget"
    );
    // (No log-absence predicate here either — see the sibling arm above for why
    // it would be vacuous. The counter is the pin; a restored eviction fails it at a nonzero
    // count, and the oracle is `== 0`, so any nonzero kills it.)
    // (2) The loss is COUNTED and ATTRIBUTED in the durable health report.
    assert_eq!(
        summary.frames_lost, LOST,
        "a {BURST}-frame burst into a {QUEUE_DEPTH}-slot queue with the drain gated loses \
         exactly the {LOST} frames the queue could not hold"
    );
    let health = summary
        .record_health
        .topics
        .get(&topic_a)
        .expect("topic A has a health entry");
    assert_eq!(
        health.frames_lost, LOST,
        "the loss must be ATTRIBUTED to the overflowing topic: {health:?}"
    );
    assert_eq!(
        health.gap_events, 1,
        "one contiguous reclaim ⇒ exactly one mid-stream gap: {health:?}"
    );
    assert_eq!(
        health.frames_recorded + health.frames_lost,
        PUBLISHED as u64,
        "conservation: recorded {} + lost {} == {PUBLISHED}",
        health.frames_recorded,
        health.frames_lost
    );
    // The seed proves the boundary is the QUEUE, not the borrow budget: more
    // frames survive than the tap could ever hold at once.
    assert_eq!(
        health.frames_recorded, RECORDED,
        "survivors are the seed plus a full queue's worth — not just the held seed, and \
         not the tap's borrow budget: {health:?}"
    );
    assert!(
        health.frames_recorded > SEED as u64,
        "survivors must include queue-resident frames, not just the held seed: {health:?}"
    );
    // Terminal line: a lossy run says so.
    assert!(
        logs_contain("WITH ANOMALIES"),
        "a lossy run must escalate its terminal line"
    );

    // Independent frame oracle: the bag holds the seed and then the burst's
    // TAIL, in order — so the loss is pinned by IDENTITY, not only by count.
    let reader = BagReader::open(&out).expect("open bag");
    let frames = data_frames_for(&reader, &topic_a);
    let seqs: Vec<u32> = frames
        .iter()
        .map(|f| {
            cerulion_core::wire::WireHeader::read_from_buf(f)
                .expect("recorded frame carries a wire header")
                .sequence
        })
        .collect();
    let mut expected: Vec<u32> = vec![0];
    expected.extend((PUBLISHED - QUEUE_DEPTH) as u32..PUBLISHED as u32);
    assert_eq!(
        seqs, expected,
        "seed + the newest {QUEUE_DEPTH} of the burst"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// REGRESSION PIN: the deterministic form of the CI flake.
///
/// The flake (main run 30439618784) was the happy-path recording coming back
/// with 5 of 6 frames. Root cause: topic A published, the recorder drained it
/// and learned A's schema, and on the very next pass A sat at its ONE-frame
/// held budget (default provisioning ⇒ `subscriber_max_borrowed_samples` 2)
/// while B's first frame was still microseconds away — so `learned_all()` was
/// false, the bag did not exist, and the eviction probe consumed A's second
/// frame and threw away its first. With interleaved publishes that state lasts
/// microseconds, which is why it took a loaded CI runner to hit it.
///
/// This arm makes that window WIDE instead of racing it: A speaks alone, the
/// recorder is given time to drain and learn, and only then does the rest
/// arrive. Under the retired probe this recorded 5 of 6 with A's HEAD frame
/// missing (MEASURED with the retired probe). The oracle is byte-exact and
/// ordered, so the head frame's loss is caught by identity, not just by count.
#[test]
fn a_learned_tap_keeps_its_head_frame_when_a_sibling_speaks_a_beat_later() {
    const N: usize = 3;
    let mgr = make_manager(32);
    let topic_a = unique_topic("headA");
    let topic_b = unique_topic("headB");
    let out = unique_out("head");
    let ready = unique_out("head_ready");

    // DEFAULT provisioning on both — the shipping shape, and the one that gives
    // a tap a pre-writer held budget of exactly ONE.
    let mut pub_a = publisher(&mgr, &topic_a, 512);
    let mut pub_b = publisher(&mgr, &topic_b, 512);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut expect_a: Vec<Vec<u8>> = Vec::new();
    let mut expect_b: Vec<Vec<u8>> = Vec::new();
    for i in 0..N {
        expect_a.push(build_frame(
            0xA000 + i as u64,
            i as u32,
            1000 + i as u64,
            &vec![0xA0 + i as u8; 8 + i],
        ));
        expect_b.push(build_frame(
            0xB000 + i as u64,
            i as u32,
            2000 + i as u64,
            &vec![0xB0 + i as u8; 5 + i],
        ));
    }

    // A speaks FIRST and ALONE. Two settles is ~5 drive-loop passes: more than
    // enough for the recorder to drain A0, learn A's schema, and then find A at
    // budget with B still silent — the exact state the probe fired in.
    pub_a.publish_raw(&expect_a[0]).expect("publish A0");
    settle();
    settle();

    // Now the rest arrives, A's next frame first — the frame that used to
    // trigger the eviction of A0.
    for frame in expect_a.iter().skip(1) {
        pub_a.publish_raw(frame).expect("publish A");
    }
    for frame in &expect_b {
        pub_b.publish_raw(frame).expect("publish B");
    }

    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");

    assert_eq!(
        summary.dropped_unwritten, 0,
        "the head frame must not be evicted for a sibling that speaks a beat later"
    );
    assert_eq!(summary.messages, (2 * N) as u64, "all frames recorded");

    // Byte-exact + ordered: the retired probe lost A's FIRST frame
    // specifically, which a count alone would catch but identity nails.
    let reader = BagReader::open(&out).expect("open bag");
    assert_eq!(
        data_frames_for(&reader, &topic_a),
        expect_a,
        "topic A frames, head included"
    );
    assert_eq!(
        data_frames_for(&reader, &topic_b),
        expect_b,
        "topic B frames"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Shutdown with a still-silent tap must force-create the
// writer IMMEDIATELY (no busy-spin until schema_wait, no drop-oldest eviction
// of held frames) and return promptly.
// ============================================================

#[test]
fn shutdown_with_silent_tap_flushes_held_promptly_without_drops() {
    // Budget 2 on A (borrow 3); publish exactly budget-many frames so the tap
    // sits AT budget; B never speaks, schema_wait is prohibitively long (10s).
    const BURST: usize = 2;
    let mgr = make_manager(16);
    let topic_a = unique_topic("sdA");
    let topic_b = unique_topic("sdB");
    let out = unique_out("sd");
    let ready = unique_out("sd_ready");

    let mut pub_a = publisher_with_provisioning(&mgr, &topic_a, 3, 16, 256);
    let _pub_b = publisher(&mgr, &topic_b, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    cfg.schema_wait = Duration::from_secs(10);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..BURST {
        let frame = build_frame(0x5D5D, i as u32, 700 + i as u64, &[(i as u8) + 9; 12]);
        pub_a.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }
    settle(); // frames drained into held (B still silent → no writer yet)

    // SIGINT-equivalent at ~t0 (far before the 10s schema_wait).
    let flip = std::time::Instant::now();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    let took = flip.elapsed();

    // Prompt: without the shutdown force-create the code busy-spins (and evicts) until schema_wait
    // elapses — 10s. Well under that bound proves the shutting force-create.
    assert!(
        took < Duration::from_secs(3),
        "shutdown must return promptly (writer force-created), took {took:?}"
    );
    assert_eq!(
        summary.dropped_unwritten, 0,
        "shutdown must NOT evict held frames"
    );
    assert_eq!(summary.messages, BURST as u64, "all held frames flushed");

    // All held frames landed in the FINALIZED bag; B is the placeholder.
    let reader = BagReader::open(&out).expect("open");
    assert_eq!(
        data_frames_for(&reader, &topic_a),
        expect,
        "held frames must be in the finalized bag"
    );
    let channels = channel_tuples(&reader);
    let b_chan = channels
        .iter()
        .find(|(t, _, _, _)| t == &topic_b)
        .expect("silent tap B still gets a channel");
    assert_eq!(
        (b_chan.1.as_str(), b_chan.2, b_chan.3),
        ("unknown", 0, 0),
        "the silent tap's channel is the hash-0 placeholder"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// A tap provisioned with subscriber_max_borrowed_samples == 1 RECORDS. This
// test asserted the exact inverse until the held-frame change, and the
// inversion IS the fix.
//
// A held frame used to pin its shared-memory slot until the disk write
// finished, so a tap's capacity was `max_borrowed - 1` — ZERO at a budget of 1,
// meaning the tap could never hold a sample and the topic would be silently
// absent from every bag. Arming refused it loudly rather than record nothing.
// That refusal was the small end of the defect that lost 17,720 frames on the
// Go2: the recorder's throughput was hostage to a number the PRODUCER chose,
// and a producer nobody provisioned for recording (a `ros2 attach` bridge route,
// any foreign publisher) gets the stock budget.
//
// Copy-at-drain removed the premise — `drain_taps` asks for one sample, copies
// its payload and drops the sample inside the loop, so exactly one borrow is
// live at a time and a budget of 1 records at full rate. The oracle is
// therefore the FRAMES, byte-for-byte, off a borrow-1 topic.
//
// The refusal survives at a budget of ZERO, which no iceoryx2 service can be
// created with — a fail-closed guard against a future provisioning change
// rather than a reachable operator error, which is why no arm here can reach it.
// ============================================================

#[test]
fn single_borrow_tap_records_at_full_rate() {
    const N: u32 = 6;
    let mgr = make_manager(16);
    let topic = unique_topic("borrow1");
    let out = unique_out("borrow1");
    let ready = unique_out("borrow1_ready");

    // borrow == 1: the LOWEST budget any service can carry, and the one arming
    // used to refuse outright. The receive queue is deep enough that the only
    // thing under test is the BORROW budget.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 1, 32, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared — a borrow-1 tap must ARM, not be refused"
    );
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..N {
        let frame = build_frame(0x0B01, i, 100 + i as u64, &[(i as u8) + 1; 24]);
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }
    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");

    // THE PIN: every frame from a topic no operator provisioned for recording,
    // byte-for-byte, with nothing destroyed on either loss ledger.
    let reader = BagReader::open(&out).expect("open");
    assert_eq!(
        data_frames_for(&reader, &topic),
        expect,
        "every frame from the borrow-1 topic must reach the bag"
    );
    assert_eq!(summary.messages, N as u64);
    assert_eq!(summary.frames_lost, 0, "no tap-queue overflow");
    assert_eq!(
        summary.dropped_unwritten, 0,
        "nothing drained-but-unwritten"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// A frame with an unparseable wire header is recorded WHOLE with
// fabricated seq/time 0, counted (summary + status) and warned once per topic.
// ============================================================

#[test]
#[tracing_test::traced_test]
fn headerless_frame_counted_warned_and_recorded_with_seq_zero() {
    const GOOD: usize = 3;
    let mgr = make_manager(16);
    let topic = unique_topic("headerless");
    let out = unique_out("headerless");
    let ready = unique_out("headerless_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 16, 32, 256);

    // run_bagd INLINE on this thread so the warn lands in the traced span.
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(300);
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let garbage: Vec<u8> = vec![0xBA; 10]; // 10 bytes < WireHeader::SIZE → headerless
    let garbage_c = garbage.clone();
    let publisher_thread = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        // One good frame FIRST (learns the schema), then the garbage frame,
        // then more good frames.
        let mut expect: Vec<Vec<u8>> = Vec::new();
        for i in 0..GOOD {
            let frame = build_frame(0x600D, i as u32, 900 + i as u64, &[0x33; 16]);
            pubr.publish_raw(&frame).expect("publish good");
            expect.push(frame.clone());
            if i == 0 {
                pubr.publish_raw(&garbage_c).expect("publish garbage");
            }
        }
        std::thread::sleep(Duration::from_millis(400));
        shutdown_c.store(true, Ordering::Relaxed);
        expect
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    let expect_good = publisher_thread.join().expect("publisher thread");

    assert_eq!(
        summary.headerless, 1,
        "exactly one headerless frame counted"
    );
    assert_eq!(
        summary.messages,
        (GOOD + 1) as u64,
        "good + garbage frames all recorded"
    );
    assert!(
        logs_contain("unparseable wire header"),
        "the headerless warn must fire"
    );

    // The garbage frame is in the bag WHOLE with fabricated seq/time 0
    // (documented observability-grade policy: record everything, hide
    // nothing, fabricate loudly).
    let reader = BagReader::open(&out).expect("open");
    let msgs = data_msgs_for(&reader, &topic);
    let garbage_msg = msgs
        .iter()
        .find(|m| m.data == garbage)
        .expect("the garbage frame must be recorded whole");
    assert_eq!(garbage_msg.sequence, 0, "fabricated sequence 0");
    assert_eq!(garbage_msg.log_time, 0, "fabricated log_time 0");
    assert_eq!(garbage_msg.publish_time, 0, "fabricated publish_time 0");
    // The good frames are intact alongside it.
    let good_frames: Vec<Vec<u8>> = msgs
        .iter()
        .filter(|m| m.data != garbage)
        .map(|m| m.data.clone())
        .collect();
    assert_eq!(good_frames, expect_good, "good frames unaffected");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// A mid-drain receive error SALVAGES already-held frames into the
// (un-finalized) bag before the error propagates.
// ============================================================

#[test]
fn tap_drain_error_salvages_held_frames_into_unfinalized_bag() {
    const PUBLISHED: usize = 5;
    const SURVIVE: usize = 3; // fault fires after 3 successful drains
    let mgr = make_manager(16);
    let topic = unique_topic("salvage");
    let out = unique_out("salvage");
    let ready = unique_out("salvage_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 16, 32, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_secs(10); // salvage must not depend on it
                                               // The test seam: tap 0's subscriber errors its next receive after 3
                                               // successfully drained samples (empty receives don't count down).
    cfg.fault_inject_tap_receive_after = Some((0, SURVIVE as u32));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut frames: Vec<Vec<u8>> = Vec::new();
    for i in 0..PUBLISHED {
        let frame = build_frame(0x5A17, i as u32, 40 + i as u64, &[(i as u8) + 1; 20]);
        pubr.publish_raw(&frame).expect("publish");
        frames.push(frame);
    }

    // run_bagd fails on its own once the fault fires.
    let result = handle.join().expect("join");
    assert!(
        matches!(result, Err(BagdError::Transport(_))),
        "the injected receive failure must propagate, got {result:?}"
    );

    // The 3 frames drained before the fault were SALVAGED: present in the bag,
    // which remains UN-finalized (the recording still failed).
    assert!(out.exists(), "the salvage must have created the bag file");
    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "a failed recording must NOT be finalized, got {completeness:?}"
    );
    let got: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(
        got,
        frames[..SURVIVE].to_vec(),
        "exactly the already-held frames must be salvaged, byte-identical"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// TapSpec::exact (exact mode): the writer is created
// immediately (no schema learning, no schema_wait), and the channel carries
// the EXACT name/hash/size instead of "unknown"/0.
// ============================================================

#[test]
fn exact_mode_tap_creates_writer_immediately_with_real_schema() {
    const N: usize = 3;
    const EXACT_HASH: u64 = 0xE4AC_7000_0000_0042;
    let mgr = make_manager(16);
    let topic = unique_topic("exact");
    let out = unique_out("exact");
    let ready = unique_out("exact_ready");

    let mut pubr = publisher(&mgr, &topic, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let exact = cerulion_bag::TopicSchema {
        topic: topic.clone(),
        schema_name: "geometry_msgs/Vector3".to_string(),
        schema_hash: EXACT_HASH,
        wire_fixed_size: 56,
    };
    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::exact(&topic, exact)],
        ready.clone(),
    );
    // Prohibitively long: exact mode must NOT wait for learning.
    cfg.schema_wait = Duration::from_secs(30);
    let started = std::time::Instant::now();
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    // Pin immediate writer creation independently of the shutdown
    // force-create -- the bag file (prelude written at creation) must exist
    // BEFORE any frame is published, far inside the 30s schema_wait.
    assert!(
        wait_for_file(&out, Duration::from_secs(2)),
        "exact mode must create the writer (bag file) without waiting for messages"
    );
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..N {
        let frame = build_frame(EXACT_HASH, i as u32, 3000 + i as u64, &[0x44; 24]);
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }
    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    let took = started.elapsed();

    // Writer creation + the whole run finished far inside the 30s schema_wait
    // — exact mode never waits to learn.
    assert!(
        took < Duration::from_secs(10),
        "exact mode must not wait out schema_wait, took {took:?}"
    );
    assert_eq!(summary.messages, N as u64);

    let reader = BagReader::open(&out).expect("open");
    assert_eq!(data_frames_for(&reader, &topic), expect, "frames flow");
    let channels = channel_tuples(&reader);
    let chan = channels
        .iter()
        .find(|(t, _, _, _)| t == &topic)
        .expect("exact channel");
    assert_eq!(
        chan.1, "geometry_msgs/Vector3",
        "EXACT schema name (not \"unknown\")"
    );
    assert_eq!(chan.2, EXACT_HASH, "EXACT schema hash");
    assert_eq!(chan.3, 56, "EXACT wire_fixed_size (not 0)");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// init_with_ix_config_json rejects malformed JSON loudly.
// ============================================================

#[test]
fn init_with_ix_config_json_rejects_malformed_json() {
    // The parse failure fires BEFORE any singleton/iceoryx2 touch, so this is
    // safe to run in-process alongside the isolated-manager tests.
    let result = cerulion_core::TransportManager::init_with_ix_config_json(
        cerulion_core::TransportConfig::default(),
        "definitely not json {",
    );
    match result {
        Err(cerulion_core::TransportError::InvalidTransportConfig { reason }) => {
            assert!(
                reason.contains("could not parse iceoryx2 Config from JSON"),
                "the reason must name the parse contract: {reason}"
            );
            assert!(
                reason.contains("expected"),
                "the reason must carry the serde parse message: {reason}"
            );
        }
        Err(other) => panic!("expected InvalidTransportConfig, got {other:?}"),
        Ok(_) => panic!("expected InvalidTransportConfig, got Ok(manager)"),
    }
}

// ============================================================
// An empty tap list is a loud config error (the library API
// has no clap `required = true` in front of it).
// ============================================================

#[test]
fn empty_taps_rejected_with_config_error() {
    let mgr = make_manager(16);
    let out = unique_out("notaps");
    let cfg = BagdConfig::new(out.clone(), Vec::new());
    let shutdown = Arc::new(AtomicBool::new(false));
    // A regressed rejection must FAIL the match below, not hang the suite. This
    // is the idiom that [`shutdown_watchdog`] was extracted from and then applied
    // to the eight sibling arms that had it MISSING; it stays here as the
    // original, now sharing that one implementation.
    let watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    let result = run_bagd(mgr, cfg, shutdown);
    drop(watchdog); // deliberately not joined -- see `shutdown_watchdog`
    assert!(
        matches!(result, Err(BagdError::Config(_))),
        "empty taps must be BagdError::Config, got {result:?}"
    );
    assert!(!out.exists(), "no bag file for a rejected config");
}

// ============================================================
// A failed /bagd/status publisher creation degrades to
// warn-and-continue: the recording itself must be unaffected.
// ============================================================

#[test]
#[tracing_test::traced_test]
fn status_publisher_conflict_warns_and_recording_continues() {
    const N: usize = 3;
    let mgr = make_manager(16);
    let topic = unique_topic("statconflict");
    let out = unique_out("statconflict");
    let ready = unique_out("statconflict_ready");

    let mut pubr = publisher(&mgr, &topic, 256);

    // Pre-create /bagd/status with max_publishers = 1 and OCCUPY the single
    // slot — bagd's status publisher port creation must fail.
    let mut squat_tc = mgr.default_topic_config();
    squat_tc.max_publishers = Some(1);
    let _squatter = mgr
        .create_publisher_with_topic_config(
            cerulion_bagd::STATUS_TOPIC,
            cerulion_core::wire::MaxSliceLen::const_new(1024),
            0,
            squat_tc,
        )
        .expect("squat the status topic");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.status_period = Some(Duration::from_millis(50)); // status REQUESTED

    // Inline run (traced span captures the warn); helper publishes + stops.
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let publisher_thread = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        let mut expect = Vec::new();
        for i in 0..N {
            let frame = build_frame(0x0057_A7C0, i as u32, 60 + i as u64, &[0x55; 16]);
            pubr.publish_raw(&frame).expect("publish");
            expect.push(frame);
        }
        std::thread::sleep(Duration::from_millis(300));
        shutdown_c.store(true, Ordering::Relaxed);
        expect
    });

    let summary =
        run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok despite status conflict");
    let expect = publisher_thread.join().expect("publisher thread");

    assert!(
        logs_contain("status publisher unavailable"),
        "the degraded-status warn must fire"
    );
    // The terminal summary restates the degradation (the t=0 warn
    // may be hours up-scroll on a real run).
    assert!(
        logs_contain("requested but UNAVAILABLE"),
        "the terminal log must restate the status degradation"
    );
    assert!(
        summary.status_unavailable,
        "the summary must carry status_unavailable"
    );
    assert_eq!(summary.messages, N as u64, "recording unaffected");
    let reader = BagReader::open(&out).expect("open");
    assert_eq!(data_frames_for(&reader, &topic), expect, "frames intact");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// The shutdown-boundary overflow case: a frame queued
// at SIGTERM on an at-budget tap (with a silent sibling blocking the writer)
// must land in the bag with ZERO evictions.
//
// ORDERING, stated precisely because a loose reading of it contradicts
// `drive_loop`'s own "Deliberately NO pre-drain force-create here": WITHIN a
// pass the drain runs FIRST and `ensure_writer` second (so a first frame
// queued at SIGTERM still contributes its learned schema). What this arm
// exercises is ACROSS passes — the shutdown pass force-creates the writer, and
// the NEXT pass's drain then has somewhere to put the queued frame, with the
// exit check refusing to end on a pass that flushed.
//
// There is no eviction probe to defend the boundary:
// a tap at `room == 0` waits unconditionally. This arm
// fails under a variant that restores one — see its
// `dropped_unwritten == 0` assertion, which that variant fails at 1.
// ============================================================

#[test]
fn shutdown_with_queued_overflow_frame_lands_without_eviction() {
    const BUDGET: usize = 2; // borrow 3 → held budget 2
    let mgr = make_manager(16);
    let topic_a = unique_topic("sdqA");
    let topic_b = unique_topic("sdqB");
    let out = unique_out("sdq");
    let ready = unique_out("sdq_ready");

    let mut pub_a = publisher_with_provisioning(&mgr, &topic_a, 3, 16, 256);
    let _pub_b = publisher(&mgr, &topic_b, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    cfg.schema_wait = Duration::from_secs(10);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    // Fill the tap exactly to budget (held = 2, writer absent — B is silent).
    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..BUDGET {
        let frame = build_frame(0x5D0F, i as u32, 800 + i as u64, &[(i as u8) + 3; 12]);
        pub_a.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }
    settle(); // both drained into held

    // SIGTERM FIRST, then the overflow frame — it is "queued at SIGTERM": any
    // in-flight pass that wakes on it finds the tap at `room == 0` and must
    // WAIT, leaving the frame in the queue for a post-flush pass.
    let flip = std::time::Instant::now();
    shutdown.store(true, Ordering::Relaxed);
    let overflow = build_frame(0x5D0F, BUDGET as u32, 800 + BUDGET as u64, &[0x77; 12]);
    pub_a.publish_raw(&overflow).expect("publish overflow");
    expect.push(overflow);

    let summary = handle.join().expect("join").expect("run_bagd Ok");
    let took = flip.elapsed();

    assert!(
        took < Duration::from_secs(3),
        "shutdown must stay prompt, took {took:?}"
    );
    assert_eq!(
        summary.dropped_unwritten, 0,
        "the queued overflow frame must NOT trigger a shutdown-boundary eviction"
    );
    assert_eq!(
        summary.messages,
        (BUDGET + 1) as u64,
        "the held frames AND the frame queued at SIGTERM must all be recorded"
    );

    let reader = BagReader::open(&out).expect("open");
    assert_eq!(
        data_frames_for(&reader, &topic_a),
        expect,
        "all frames (including the one queued at SIGTERM) byte-match in order"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// The learn-suppression gate: a topic whose first message
// arrives AFTER the bag was created keeps its hash-0 placeholder channel
// (registered channels are immutable), warns loudly once, and its frames are
// still recorded in full.
// ============================================================

#[test]
#[tracing_test::traced_test]
fn late_speaker_keeps_placeholder_channel_with_loud_warn() {
    let mgr = make_manager(16);
    let topic_a = unique_topic("lateA");
    let topic_b = unique_topic("lateB");
    let out = unique_out("late");
    let ready = unique_out("late_ready");

    let mut pub_a = publisher(&mgr, &topic_a, 256);
    let mut pub_b = publisher(&mgr, &topic_b, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    // Short: the writer force-creates with B's placeholder BEFORE B speaks.
    cfg.schema_wait = Duration::from_millis(300);

    // Inline run (the warn must land in the traced span); the helper thread
    // stages the timeline: A speaks early (learned), the writer force-creates
    // at 300ms with B's placeholder, THEN B speaks.
    let ready_c = ready.clone();
    let out_c = out.clone();
    let shutdown_c = shutdown.clone();
    let publisher_thread = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        let fa = build_frame(0xA11CE, 0, 10, &[0x21; 8]);
        pub_a.publish_raw(&fa).expect("publish A");
        // Wait out the schema grace so the writer exists (B = placeholder) —
        // waited for on the bag FILE, which appears exactly when the channel set
        // closes and the writer force-creates. That is the event the old fixed
        // 700 ms was a bet on, and the injected 300 ms grace above is unchanged.
        assert!(
            wait_for_file(&out_c, LIVENESS_CEILING),
            "the writer never force-created the bag, so B's frames would not be \
             'late' in the sense this arm pins"
        );
        // NOW B speaks for the first time — after bag creation.
        let mut fb: Vec<Vec<u8>> = Vec::new();
        for i in 0..2u32 {
            let frame = build_frame(0xB0B0, i, 20 + i as u64, &[0x42 + i as u8; 10]);
            pub_b.publish_raw(&frame).expect("publish B");
            fb.push(frame);
        }
        std::thread::sleep(Duration::from_millis(300));
        shutdown_c.store(true, Ordering::Relaxed);
        (fa, fb)
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    let (fa, fb) = publisher_thread.join().expect("publisher thread");

    // The suppression warn fired (once per tap; presence pinned here).
    assert!(
        logs_contain("channel remains the hash-0"),
        "the late-speaker suppression warn must fire"
    );
    // And the misleading 'learned' info did NOT fire for B after creation:
    // B's frames were recorded, so a learn would have logged its 0xB0B0 hash.
    assert!(
        !logs_contain("0x000000000000B0B0"),
        "no post-creation 'learned' log may name B's wire hash"
    );
    assert_eq!(
        summary.messages, 3,
        "A's frame + B's 2 late frames recorded"
    );

    let reader = BagReader::open(&out).expect("open");
    // B's channel is STILL the placeholder — immutable once registered.
    let channels = channel_tuples(&reader);
    let b_chan = channels
        .iter()
        .find(|(t, _, _, _)| t == &topic_b)
        .expect("B channel present");
    assert_eq!(
        (b_chan.1.as_str(), b_chan.2, b_chan.3),
        ("unknown", 0, 0),
        "a late speaker's channel remains the hash-0 placeholder"
    );
    // A's channel carries its learned hash (control arm).
    let a_chan = channels
        .iter()
        .find(|(t, _, _, _)| t == &topic_a)
        .expect("A channel present");
    assert_eq!(a_chan.2, 0xA11CE, "the early speaker's channel is learned");
    // B's frames are recorded IN FULL despite the placeholder channel.
    assert_eq!(
        data_frames_for(&reader, &topic_b),
        fb,
        "late frames are recorded whole on the placeholder channel"
    );
    assert_eq!(data_frames_for(&reader, &topic_a), vec![fa]);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// A tap whose first-ever frame is queued at SIGTERM
// must still get its schema LEARNED (the shutdown pass drains BEFORE the
// force-create): channel carries the real wire hash, no misleading
// "spoke AFTER bag creation" warn.
// ============================================================

#[test]
#[tracing_test::traced_test]
fn first_frame_queued_at_sigterm_still_learns_schema() {
    const B_HASH: u64 = 0x1EA2_2000_0000_0077;
    let mgr = make_manager(16);
    let topic_a = unique_topic("learnA");
    let topic_b = unique_topic("learnB");
    let out = unique_out("learn");
    let ready = unique_out("learn_ready");

    let mut pub_a = publisher_with_provisioning(&mgr, &topic_a, 3, 16, 256);
    let mut pub_b = publisher(&mgr, &topic_b, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(
        out.clone(),
        vec![TapSpec::attach(&topic_a), TapSpec::attach(&topic_b)],
        ready.clone(),
    );
    // Prohibitively long: only the SIGTERM force-create can create the writer.
    cfg.schema_wait = Duration::from_secs(10);
    // De-flake (pre-existing ~1/3 race): the drain gate holds B's first frame
    // undrained across its publish, and the gate REOPENS BEFORE the shutdown
    // flag is set — so under program order, any pass that observes shutdown
    // also sees an open gate and drains B first (a pass landing between the
    // two stores is a NORMAL pass that learns B early; schema_wait stays
    // prohibitive so only the SIGTERM force-create builds the writer either
    // way). Residual: two Relaxed stores on different atomics can in theory
    // be observed out of order on weak memory — theoretical-only; the
    // schedulable preemption race (a ~1/3 flake without the gate, and the shutdown-
    // before-gate ordering's placeholder bake) is closed.
    let drain_gate = Arc::new(AtomicBool::new(true));
    cfg.fault_inject_tap_drain_gate = Some(drain_gate.clone());

    // Inline run (traced span must capture — or rather, must NOT capture —
    // the suppression warn); the helper stages the flip-then-publish timeline.
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let drain_gate_c = drain_gate.clone();
    let publisher_thread = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        // A speaks (learned; held frames also keep the shutdown drain alive).
        let mut fa: Vec<Vec<u8>> = Vec::new();
        for i in 0..2u32 {
            let frame = build_frame(0xAAAA, i, 5 + i as u64, &[0x61; 8]);
            pub_a.publish_raw(&frame).expect("publish A");
            fa.push(frame);
        }
        settle(); // drained + held; B still COMPLETELY silent → no writer yet
                  // Close the drain gate, land B's first-ever frame in the tap
                  // queue (publish_raw is synchronous into SHM), reopen the
                  // gate, THEN flag shutdown — gate-before-shutdown order, so
                  // every shutdown-observing pass drains B (see the cfg note).
        drain_gate_c.store(false, Ordering::Relaxed);
        let fb = build_frame(B_HASH, 0, 99, &[0x62; 16]);
        pub_b.publish_raw(&fb).expect("publish B first frame");
        drain_gate_c.store(true, Ordering::Relaxed);
        shutdown_c.store(true, Ordering::Relaxed);
        (fa, fb)
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    let (fa, fb) = publisher_thread.join().expect("publisher thread");

    assert_eq!(
        summary.dropped_unwritten, 0,
        "no shutdown-boundary eviction"
    );
    assert_eq!(
        summary.messages, 3,
        "A's 2 frames + B's first frame recorded"
    );

    // B was LEARNED, not baked as a placeholder: no suppression warn fired...
    assert!(
        !logs_contain("channel remains the hash-0"),
        "a first frame queued at SIGTERM must be LEARNED, not warned as a late speaker"
    );
    // ...and the channel carries B's real wire hash.
    let reader = BagReader::open(&out).expect("open");
    let channels = channel_tuples(&reader);
    let b_chan = channels
        .iter()
        .find(|(t, _, _, _)| t == &topic_b)
        .expect("B channel present");
    assert_eq!(
        b_chan.2, B_HASH,
        "B's channel must carry the REAL wire schema_hash, not the hash-0 placeholder"
    );
    assert_eq!(data_frames_for(&reader, &topic_b), vec![fb]);
    assert_eq!(data_frames_for(&reader, &topic_a), fa);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Exactly-once across a mid-flush error: a tiny
// chunk_max_bytes forces durable auto-flushes INSIDE one flush call; the
// injected write failure then leaves a durable prefix + a discarded pending
// chunk. The salvage retry must write ONLY the never-persisted remainder —
// every frame lands in the bag exactly once.
// ============================================================

#[test]
fn flush_error_after_durable_chunks_salvages_each_frame_exactly_once() {
    const N: usize = 6;
    const BODY: usize = 64; // frame = 32 hdr + 64 = 96 B; ~127 B chunk body each
    let mgr = make_manager(16);
    let topic = unique_topic("exact1");
    let out = unique_out("exact1");
    let ready = unique_out("exact1_ready");

    // borrow 7 → held budget 6: the at_budget trigger fires the flush exactly
    // when ALL N frames are held (deterministic multi-chunk write set).
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 7, N + 8, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(300);
    cfg.flush_interval = Duration::from_secs(10); // only at_budget can flush
                                                  // ~127 B per message body: the 2nd message crosses 200 → durable chunks
                                                  // after messages #1 and #3; the fault fires before message #4.
    cfg.chunk_max_bytes = 200;
    cfg.fault_inject_flush_error_after_messages = Some(4);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..N {
        let frame = build_frame(0xE0_0E, i as u32, 50 + i as u64, &[(i as u8) + 1; BODY]);
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }

    // The at_budget flush fires with all 6 held, auto-flushes 2 durable
    // chunks (4 messages), then the injected failure hits → salvage.
    let result = handle.join().expect("join");
    assert!(
        matches!(result, Err(BagdError::Bag(_))),
        "the injected flush failure must propagate, got {result:?}"
    );

    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "a failed recording must NOT be finalized, got {completeness:?}"
    );
    let frames: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    // THE pin: every frame exactly once, in order — a double-write of the
    // durable prefix (the bug named in this test's header) or a lost remainder both fail this.
    assert_eq!(
        frames, expect,
        "each frame must appear in the salvaged bag EXACTLY once, in order"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// A status-degraded run that fails restates the
// degradation on the failure terminal path (its "failed" status frame was
// unpublishable).
// ============================================================

#[test]
#[tracing_test::traced_test]
fn status_conflict_restated_on_failing_run() {
    let mgr = make_manager(16);
    let topic = unique_topic("statfail");
    let out = unique_out("statfail");
    let ready = unique_out("statfail_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 16, 16, 256);

    // Squat /bagd/status (single publisher slot, occupied) → bagd degrades.
    let mut squat_tc = mgr.default_topic_config();
    squat_tc.max_publishers = Some(1);
    let _squatter = mgr
        .create_publisher_with_topic_config(
            cerulion_bagd::STATUS_TOPIC,
            cerulion_core::wire::MaxSliceLen::const_new(1024),
            0,
            squat_tc,
        )
        .expect("squat the status topic");

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.status_period = Some(Duration::from_millis(50)); // status REQUESTED
    cfg.schema_wait = Duration::from_secs(10);
    // Fail the run mid-drain after 2 successful receives.
    cfg.fault_inject_tap_receive_after = Some((0, 2));

    let ready_c = ready.clone();
    // This arm relies ENTIRELY on `fault_inject_tap_receive_after`
    // firing to end the run — it stores the shutdown flag nowhere at all — so the
    // guard below needs a handle to it. Cloned BEFORE `run_bagd` consumes the
    // original.
    let shutdown_c = shutdown.clone();
    let publisher_thread = std::thread::spawn(move || {
        // Nothing in this closure ever stores the shutdown flag, and the
        // publishes below are what ARM the injected failure — so an assertion that
        // panics here leaves `run_bagd` looping on the test thread forever. See
        // [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        for i in 0..3u32 {
            let frame = build_frame(0x0FA1, i, i as u64, &[0x11; 8]);
            pubr.publish_raw(&frame).expect("publish");
        }
    });

    // Inline run: the failure-arm restate must land in the traced span.
    let result = run_bagd(mgr.clone(), cfg, shutdown);
    publisher_thread.join().expect("publisher thread");
    assert!(
        matches!(result, Err(BagdError::Transport(_))),
        "the injected drain failure must propagate, got {result:?}"
    );

    assert!(
        logs_contain("status publisher unavailable"),
        "the setup degradation warn must fire"
    );
    // The FAILURE-arm restatement (distinct wording: the terminal "failed"
    // frame could not be published).
    assert!(
        logs_contain("could not be published"),
        "the failing terminal path must restate the status degradation"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Tap-queue overflow loss detection + record_health.json.
//
// The recorder drain gate stalls bagd DURING a mid-stream burst so a tiny tap
// queue overflows DETERMINISTICALLY: iceoryx2 reclaims the unread middle, the
// resumed drain observes an exact wire-sequence gap. run_bagd runs INLINE (the
// LOST-frames warn is a lib-crate event captured by `#[traced_test]`); the
// publisher + gate toggles run on a helper thread.
// ============================================================

const PROBE_HASH: u64 = 0x0597;

/// Read + parse the `record_health.json` attachment from a finalized bag.
fn read_record_health(out: &std::path::Path) -> cerulion_bagd::RecordHealth {
    let reader = BagReader::open(out).expect("open bag");
    let att = reader
        .attachment(cerulion_bagd::RECORD_HEALTH_ATTACHMENT)
        .expect("read attachments")
        .expect("record_health.json attachment present in EVERY finalized bag");
    assert_eq!(att.media_type, "application/json");
    serde_json::from_slice(&att.data).expect("record_health.json parses")
}

#[test]
#[tracing_test::traced_test]
fn tap_queue_overflow_detected_with_exact_counts_and_record_health() {
    // Tiny tap queue (= service ceiling) makes overflow reproducible; the drain
    // gate makes the drop DETERMINISTIC (bagd receives NOTHING during the burst,
    // so iceoryx2 keeps exactly the newest CEILING samples).
    const CEILING: usize = 4;
    const BURST: u32 = 20; // seq 1..=BURST published during the stall
                           // Baseline seq 0 (phase 1) is flushed first; the burst keeps its newest
                           // CEILING (seq 17..=20), so the resumed drain jumps 0 -> 17.
    const LOST: u64 = BURST as u64 - CEILING as u64; // 16
    const RECORDED: u64 = 1 + CEILING as u64; // seq 0 + newest 4

    let mgr = make_manager(16);
    let topic = unique_topic("loss");
    let out = unique_out("loss");
    let ready = unique_out("loss_ready");

    // Publisher OWNS the service at the tiny ceiling; the tap inherits it
    // open-only. Drop-oldest recycles pool slots, so the burst never exhausts
    // the pool despite the shallow ceiling. (The borrow budget bounds how many
    // samples the tap holds AT ONCE inside one drain; the tap now copies
    // and releases each one, so it no longer bounds how much the tap can stage.)
    let mut publisher = publisher_with_provisioning(&mgr, &topic, 3, CEILING, 256);

    let gate = Arc::new(AtomicBool::new(true)); // OPEN initially
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let gate_c = gate.clone();
    let worker = std::thread::spawn(move || {
        // A panicking stimulus must FAIL this arm, not hang it — the
        // shutdown store is this thread's last statement. See `ShutdownOnDrop`.
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        // First phase: baseline. Publish seq 0 and let bagd STAGE it (gate open) so
        // last_seq == 0 is established BEFORE the overflow. Staging is what
        // matters, not the flush: held frames are committed to the GapObserver
        // in drain order, so seq 0 is the baseline whenever that flush happens.
        //
        // This was a bare `sleep(300)`, which is the same bet on a race, placed
        // on the other side of the gate: a starved recorder leaves seq 0 in the
        // queue, the burst evicts it with the rest of the head, and the tap's
        // first-ever observation (seq 17) becomes the BASELINE, taking every
        // count below to 0. The gate makes the OVERFLOW deterministic; only a
        // condition can make the baseline deterministic. `bagd learned
        // attach-mode schema` is emitted after this tap's inner drain loop broke
        // on an EMPTY read, so it proves seq 0 is staged (single attach tap ⇒ it
        // is this topic's line or none).
        publisher
            .publish_raw(&build_frame(PROBE_HASH, 0, 0, &[1u8; 16]))
            .expect("publish seq 0");
        assert!(
            await_condition(Duration::from_secs(10), || logs_contain(
                "bagd learned attach-mode schema"
            )),
            "the tap must stage the baseline frame before the burst can reclaim it"
        );

        // Second phase: STALL. Close the gate + settle so at least one gated pass
        // runs while the queue is still empty, THEN burst 1..=BURST. bagd
        // receives nothing → the queue keeps the newest CEILING, reclaims the
        // middle.
        gate_c.store(false, Ordering::Relaxed);
        settle();
        for i in 1..=BURST {
            publisher
                .publish_raw(&build_frame(PROBE_HASH, i, i as u64, &[i as u8; 16]))
                .expect("publish burst");
        }
        std::thread::sleep(Duration::from_millis(150)); // gate closed → bagd idle

        // Third phase: RESUME. Open the gate, let bagd drain the survivors, detect
        // the gap, flush, then stop.
        gate_c.store(true, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(300));
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    worker.join().expect("worker thread");

    // EXACT loss accounting (deterministic — the gate removes the race).
    assert_eq!(summary.frames_lost, LOST, "aggregate frames_lost");
    let th = summary
        .record_health
        .topics
        .get(&topic)
        .expect("topic in record_health");
    assert_eq!(th.frames_lost, LOST, "per-topic frames_lost");
    assert_eq!(th.gap_events, 1, "exactly one mid-stream gap");
    assert!(!th.multi_publisher, "single-writer topic");
    assert_eq!(
        th.frames_recorded, RECORDED,
        "seq 0 + newest CEILING survive"
    );

    // The record-time LOUD warn fired (kills the mid-run blind spot) + the
    // terminal WITH-ANOMALIES escalation.
    assert!(
        logs_contain("bagd tap LOST frames"),
        "the first-gap loud warn must fire at record time"
    );
    assert!(
        logs_contain("WITH ANOMALIES"),
        "a lossy run's terminal line must escalate"
    );

    // The attachment is inside the FINALIZED bag and matches the summary.
    let disk = read_record_health(&out);
    assert_eq!(disk, summary.record_health, "attachment == summary health");
    assert_eq!(disk.version, cerulion_bagd::RECORD_HEALTH_VERSION);
    assert_eq!(disk.dropped_unwritten, 0, "no unwritten drops here");

    // Independent frame oracle: the survivors on disk are seq 0 then the newest
    // CEILING of the burst (fill bytes 1, then 17..=20).
    let reader = BagReader::open(&out).expect("open bag");
    let frames = data_frames_for(&reader, &topic);
    assert_eq!(frames.len() as u64, RECORDED, "recorded frame count");
    let fills: Vec<u8> = frames
        .iter()
        .map(|f| f[cerulion_core::wire::WireHeader::SIZE])
        .collect();
    assert_eq!(fills, vec![1, 17, 18, 19, 20], "survivor fill bytes");

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

#[test]
#[tracing_test::traced_test]
fn healthy_recording_writes_all_zero_record_health() {
    // Generous ceiling + no gate: every frame drains, no overflow → the
    // attachment is PRESENT and all-zero (the always-present control).
    const N: u32 = 8;
    let mgr = make_manager(16);
    let topic = unique_topic("ok");
    let out = unique_out("ok");
    let ready = unique_out("ok_ready");

    let mut publisher = publisher_with_provisioning(&mgr, &topic, 3, 16, 256);
    let cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let worker = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(wait_for_file(&ready_c, LIVENESS_CEILING), "ready");
        settle();
        // Publish consecutively at a modest pace — the deep queue holds them and
        // bagd drains each; no overflow, no gap.
        for i in 0..N {
            publisher
                .publish_raw(&build_frame(PROBE_HASH, i, i as u64, &[(i as u8) + 1; 16]))
                .expect("publish");
            std::thread::sleep(Duration::from_millis(15));
        }
        std::thread::sleep(Duration::from_millis(200));
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    worker.join().expect("worker");

    assert_eq!(summary.frames_lost, 0, "healthy run loses nothing");
    let th = summary.record_health.topics.get(&topic).expect("topic");
    assert_eq!(th.frames_lost, 0);
    assert_eq!(th.gap_events, 0);
    assert!(!th.multi_publisher);
    assert_eq!(th.frames_recorded, N as u64, "every frame recorded");

    // Healthy terminal + no loss warn.
    assert!(
        !logs_contain("bagd tap LOST frames"),
        "no loss warn on a healthy run"
    );
    assert!(
        !logs_contain("WITH ANOMALIES"),
        "no anomaly escalation on a healthy run"
    );

    // Always-present all-zero attachment.
    let disk = read_record_health(&out);
    assert_eq!(disk, summary.record_health);
    assert_eq!(disk.version, cerulion_bagd::RECORD_HEALTH_VERSION);
    assert_eq!(disk.dropped_unwritten, 0);
    let dth = disk.topics.get(&topic).expect("topic health on disk");
    assert_eq!(dth.frames_lost, 0);
    assert_eq!(dth.gap_events, 0);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

#[test]
#[tracing_test::traced_test]
fn multi_publisher_tap_disables_gap_detection_honestly() {
    // The SAME overflow stimulus as the lossy test, but the tap is flagged
    // multi_publisher (a `/tf`-style topic) → gap detection is DISABLED and the
    // health entry reports multi_publisher:true + frames_lost 0 (the
    // interleaved sequence streams are NOT single-writer loss). Anti-tautology:
    // an unflagged tap on this exact stimulus WOULD report loss (the test above).
    const CEILING: usize = 4;
    const BURST: u32 = 20;

    let mgr = make_manager(16);
    let topic = unique_topic("tf");
    let out = unique_out("tf");
    let ready = unique_out("tf_ready");

    let mut publisher = publisher_with_provisioning(&mgr, &topic, 3, CEILING, 256);

    let gate = Arc::new(AtomicBool::new(true));
    // The multi_publisher flag rides on the tap descriptor (graph stamps it from
    // `multi_publisher_topics`; here set directly).
    let tap = TapSpec {
        topic: topic.clone(),
        exact: None,
        multi_publisher: true,
    };
    let mut cfg = test_cfg(out.clone(), vec![tap], ready.clone());
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let gate_c = gate.clone();
    let worker = std::thread::spawn(move || {
        // A panicking stimulus must FAIL this arm, not hang it — the
        // shutdown store is this thread's last statement. See `ShutdownOnDrop`.
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(wait_for_file(&ready_c, LIVENESS_CEILING), "ready");
        settle();
        publisher
            .publish_raw(&build_frame(PROBE_HASH, 0, 0, &[1u8; 16]))
            .expect("publish seq 0");
        // A CONDITION, not a `sleep(300)` — see the sibling arm above.
        // Gap detection is disabled here, so `frames_lost` cannot invert; what a
        // lost baseline WOULD invert is the anti-vacuity oracle below, which
        // requires seq 0's fill byte at the head of the survivors.
        assert!(
            await_condition(Duration::from_secs(10), || logs_contain(
                "bagd learned attach-mode schema"
            )),
            "the tap must stage the baseline frame before the burst can reclaim it"
        );
        gate_c.store(false, Ordering::Relaxed);
        settle();
        for i in 1..=BURST {
            publisher
                .publish_raw(&build_frame(PROBE_HASH, i, i as u64, &[i as u8; 16]))
                .expect("publish burst");
        }
        std::thread::sleep(Duration::from_millis(150));
        gate_c.store(true, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(300));
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    worker.join().expect("worker");

    // Detection disabled → reported ZERO loss (no false positive on the interleave).
    assert_eq!(
        summary.frames_lost, 0,
        "multi-publisher topic reports no loss"
    );
    let th = summary.record_health.topics.get(&topic).expect("topic");
    assert!(th.multi_publisher, "the health entry marks multi_publisher");
    assert!(
        !th.sequence_anomaly,
        "a DECLARED multi tap must never be marked as a runtime anomaly"
    );
    assert_eq!(th.frames_lost, 0);
    assert_eq!(th.gap_events, 0);

    // No loss warn fired (the flag suppresses detection entirely) — and no
    // runtime-anomaly disable either (detection was never armed).
    assert!(
        !logs_contain("bagd tap LOST frames"),
        "multi-publisher taps must NOT emit the loss warn"
    );
    assert!(
        !logs_contain("DISABLING sequence-gap detection"),
        "a declared multi tap never runtime-disables (detection never armed)"
    );

    // Find-pass F6 (anti-vacuity): the overflow GENUINELY happened —
    // the recorded survivors are seq 0 plus ONLY the newest CEILING of the
    // burst (fill bytes 1, then 17..=20), proving frames vanished on the wire
    // while the health reports 0/0 + the flag. A regression that
    // suppresses the OVERFLOW itself (rather than the detection) fails here.
    assert_eq!(
        th.frames_recorded,
        1 + CEILING as u64,
        "seq 0 + the newest CEILING burst frames survive"
    );
    let reader = BagReader::open(&out).expect("open bag");
    let frames = data_frames_for(&reader, &topic);
    let fills: Vec<u8> = frames
        .iter()
        .map(|f| f[cerulion_core::wire::WireHeader::SIZE])
        .collect();
    assert_eq!(
        fills,
        vec![1, 17, 18, 19, 20],
        "the burst's head must have been genuinely reclaimed (drop-oldest \
         survivors only) — otherwise the 0/0 health report is vacuous"
    );

    // The attachment serializes multi_publisher:true (and NO sequence_anomaly).
    let disk = read_record_health(&out);
    assert_eq!(disk, summary.record_health);
    let dth = disk.topics.get(&topic).expect("topic on disk");
    assert!(dth.multi_publisher);
    assert!(!dth.sequence_anomaly);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Find-pass F2 — the RUNTIME disable is LOUD and reported
// DISTINCTLY: a single-writer tap that first counts a REAL gap (loud LOST
// warn) then observes a backward jump must warn at the disable site, zero
// the counts, and mark `sequence_anomaly` (NOT `multi_publisher`) in the
// attachment.
// ============================================================

#[test]
#[tracing_test::traced_test]
fn runtime_nonmonotonic_disable_is_loud_and_marked_sequence_anomaly() {
    let mgr = make_manager(16);
    let topic = unique_topic("anom");
    let out = unique_out("anom");
    let ready = unique_out("anom_ready");

    // Raw publish_raw frames with HAND-CHOSEN sequences (the publisher is just
    // a frame pipe here): 0,1 → baseline; 5 → REAL gap (lost 3, loud warn);
    // 2 → backward jump (non-single-writer evidence) → runtime disable.
    let mut publisher = publisher_with_provisioning(&mgr, &topic, 3, 16, 256);
    let cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let worker = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(wait_for_file(&ready_c, LIVENESS_CEILING), "ready");
        settle();
        // First phase: baseline 0,1 — flushed (test_cfg flush_interval = 20ms).
        for i in 0..2u32 {
            publisher
                .publish_raw(&build_frame(PROBE_HASH, i, i as u64, &[1u8; 16]))
                .expect("publish baseline");
        }
        std::thread::sleep(Duration::from_millis(250));
        // Second phase: seq 5 — a REAL gap; its commit fires the loud LOST warn.
        publisher
            .publish_raw(&build_frame(PROBE_HASH, 5, 5, &[2u8; 16]))
            .expect("publish gap frame");
        std::thread::sleep(Duration::from_millis(250));
        // Third phase: seq 2 — BACKWARD (non-monotone) → runtime disable, counts
        // zeroed, anomaly warn. Committed in a LATER chunk than the gap so
        // the "prior loud warn superseded" story is exercised end-to-end.
        publisher
            .publish_raw(&build_frame(PROBE_HASH, 2, 6, &[3u8; 16]))
            .expect("publish backward frame");
        std::thread::sleep(Duration::from_millis(250));
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    worker.join().expect("worker");

    // BOTH warns fired, in story order: the gap was loud when counted, and
    // the later disable LOUDLY explained the reversal to 0/0.
    assert!(
        logs_contain("bagd tap LOST frames"),
        "the real gap must fire the loud LOST warn before the disable"
    );
    assert!(
        logs_contain("DISABLING sequence-gap detection"),
        "the runtime disable must be LOUD (F2 — a silent 0/0 reversal after \
         a loud LOST warn is the exact blind spot this kills)"
    );

    // The attachment: counts ZEROED, and the disable attributed to
    // a RUNTIME anomaly — distinctly NOT the declared multi_publisher flag.
    let th = summary.record_health.topics.get(&topic).expect("topic");
    assert!(
        th.sequence_anomaly,
        "the runtime disable must be surfaced as sequence_anomaly"
    );
    assert!(
        !th.multi_publisher,
        "an undeclared topic must NOT be reported as declared multi-publisher"
    );
    assert_eq!(
        th.frames_lost, 0,
        "prior gap counts zeroed as unattributable"
    );
    assert_eq!(th.gap_events, 0);
    assert_eq!(summary.frames_lost, 0);
    assert_eq!(th.frames_recorded, 4, "all 4 frames recorded whole");

    // Round-trips through the bag attachment.
    let disk = read_record_health(&out);
    assert_eq!(disk, summary.record_health);
    let dth = disk.topics.get(&topic).expect("topic on disk");
    assert!(dth.sequence_anomaly && !dth.multi_publisher);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// Find-pass F3 — the salvage path must NOT count the durably
// committed prefix as a false gap: a mid-flush failure AFTER durable
// auto-flushed chunks commits the prefix (advancing last_seq over it); the
// salvage re-flush of the CONSECUTIVE remainder must stay gap-free (no
// false LOST warn on the error-recovery path).
// ============================================================

#[test]
#[tracing_test::traced_test]
fn salvage_reflush_after_durable_prefix_counts_no_false_gap() {
    const BODY: usize = 64; // frame = 32 hdr + 64 = 96 B; ~127 B chunk body each
    let mgr = make_manager(16);
    let topic = unique_topic("salv");
    let out = unique_out("salv");
    let ready = unique_out("salv_ready");

    // borrow 9 → held budget 8 (phase 2's 5 frames all drain in one pass).
    let mut publisher = publisher_with_provisioning(&mgr, &topic, 9, 16, 256);

    let gate = Arc::new(AtomicBool::new(true));
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.fault_inject_tap_drain_gate = Some(gate.clone());
    // Tiny chunks force durable auto-flushes INSIDE phase 2's flush call;
    // the CUMULATIVE fault (3) then fires mid-flush: phase 1 wrote seq 0
    // (count 1), phase 2 writes seqs 1,2 (count 3, auto-flushed durable at
    // ~254 B > 200) and faults before seq 3 → commit_durable_prefix(2) must
    // advance last_seq 0 → 2, so the salvage re-flush of seqs 3,4,5 is
    // CONSECUTIVE (without that advance, salvage would seed gap detection
    // from the stale 0 and log a false loud warn).
    cfg.chunk_max_bytes = 200;
    cfg.fault_inject_flush_error_after_messages = Some(3);

    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let gate_c = gate.clone();
    let worker = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(wait_for_file(&ready_c, LIVENESS_CEILING), "ready");
        settle();
        // First phase: seq 0 alone — a SUCCESSFUL flush establishes last_seq = 0
        // (1 message < the cumulative fault threshold 3).
        publisher
            .publish_raw(&build_frame(PROBE_HASH, 0, 0, &[1u8; BODY]))
            .expect("publish seq 0");
        std::thread::sleep(Duration::from_millis(300));
        // Second phase: STALL the drain, publish seqs 1..=5 CONSECUTIVELY, then
        // resume — one drain pass holds all 5, one flush call writes them,
        // hitting the durable-auto-flush + injected-fault geometry above.
        gate_c.store(false, Ordering::Relaxed);
        settle();
        for i in 1..=5u32 {
            publisher
                .publish_raw(&build_frame(
                    PROBE_HASH,
                    i,
                    i as u64,
                    &[(i as u8) + 1; BODY],
                ))
                .expect("publish phase 2");
        }
        std::thread::sleep(Duration::from_millis(150));
        gate_c.store(true, Ordering::Relaxed);
        // The flush fault aborts the run on its own; no shutdown flip needed,
        // but set it as a belt-and-suspenders bound.
        std::thread::sleep(Duration::from_millis(500));
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let result = run_bagd(mgr.clone(), cfg, shutdown);
    worker.join().expect("worker");
    assert!(
        matches!(result, Err(BagdError::Bag(_))),
        "the injected flush failure must propagate, got {result:?}"
    );

    // This test pins that the stream was CONSECUTIVE 0..=5 throughout, so NO
    // loss warn may fire — without the durable-prefix advance, the salvage
    // re-flush would seed gap detection from the STALE last_seq (0) and
    // falsely warn "LOST frames" for the durably-committed seqs 1,2.
    assert!(
        !logs_contain("bagd tap LOST frames"),
        "a consecutive stream must never produce a loss warn on the salvage \
         path — the durable prefix must advance last_seq (F3)"
    );
    assert!(
        !logs_contain("DISABLING sequence-gap detection"),
        "nor a false runtime-anomaly disable"
    );

    // Exactly-once integrity still holds alongside that accounting (crib of
    // the exactly-once mid-flush-error pin): every frame appears once, in order.
    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "a failed recording must NOT be finalized"
    );
    let seqs: Vec<u32> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| {
            cerulion_core::wire::WireHeader::read_from_buf(&m.data[..32])
                .expect("header")
                .sequence
        })
        .collect();
    assert_eq!(
        seqs,
        vec![0, 1, 2, 3, 4, 5],
        "every frame exactly once, in order, across the durable prefix + salvage"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================================
// The DEDICATED WRITER THREAD: a filesystem stall no longer starves
// the tap drain. All four tests provision borrow == RECORDING_SUBSCRIBER_MAX_
// BORROWED (22), which is how `graph run --record` provisions a recorded topic.
// That number no longer SELECTS anything — the writer thread is the
// only write path, at any borrow budget — so it is now shipping-shape fidelity
// rather than a switch.
// ============================================================================

/// The recording-provisioned borrow budget `graph run --record` applies
/// (mirrors `RECORDING_SUBSCRIBER_MAX_BORROWED`).
const REC_BORROW: usize = 22;
/// An arbitrary stable schema hash for the stall frames.
const STALL_HASH: u64 = 0x6510_6510_6510_6510;

/// Whether the (still-open, un-finalized) bag already carries a data message.
///
/// With `chunk_max_bytes = 1` every written message closes its own chunk, so
/// this is an OBSERVABLE of "the writer wrote it" from outside the recorder.
fn bag_has_messages(out: &std::path::Path) -> bool {
    BagReader::open(out).is_ok_and(|r| r.recover_messages().is_ok_and(|(m, _)| !m.is_empty()))
}

/// The writer thread's injected-panic message (`WriterCore::write_batch`'s
/// panic seam). The panic-suppressing hook below matches on it so it silences
/// ONLY that deliberate unwind.
const WRITER_PANIC_SEAM_MSG: &str = "fault injection: bagd writer thread panic";

/// Wait (bounded) until `pred` holds; returns whether it did.
fn wait_until(deadline: Duration, mut pred: impl FnMut() -> bool) -> bool {
    let end = Instant::now() + deadline;
    while Instant::now() < end {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    pred()
}

/// Drive ONE stall scenario end to end and return the summary + the count of
/// data frames recovered from the finalized bag.
///
/// Shape: engage a sustained writer stall, publish a seed frame + wait for the
/// write path to enter the stall, then burst `burst` frames PACED so the drain
/// (poll ~10ms) can keep the tap queue drained. Release, shut down, finalize.
///
/// The `force_inline` A/B control was dropped with the inline path itself.
fn run_stall_scenario(label: &str, queue_depth: usize, burst: u32) -> (BagdSummary, usize) {
    let mgr = make_manager(16);
    let topic = unique_topic(label);
    let out = unique_out(label);
    let ready = unique_out(&format!("{label}_ready"));

    // Publisher OWNS the service at borrow 22 + a shallow-ish receive queue:
    // shallow enough that a drain which STOPPED for the stall would overflow it.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, queue_depth, 256);

    let gate = Arc::new(WriterStallGate::default());
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    // Engage the stall BEFORE the first flush, then publish the seed so the
    // first flush blocks in the write path.
    gate.engaged.store(true, Ordering::Relaxed);
    pubr.publish_raw(&build_frame(STALL_HASH, 0, 0, &[1u8; 16]))
        .expect("publish seed");
    assert!(
        wait_until(Duration::from_secs(3), || gate
            .entered
            .load(Ordering::Relaxed)),
        "the write path must ENTER the stall (a flush blocked)"
    );

    // Burst DURING the stall, paced > the drain poll so the threaded drain keeps
    // the tap queue drained; the inline (blocked) drain cannot.
    for i in 1..=burst {
        pubr.publish_raw(&build_frame(STALL_HASH, i, i as u64, &[i as u8; 16]))
            .expect("publish burst");
        std::thread::sleep(Duration::from_millis(25));
    }

    // Release the stall, let the writer catch up, then shut down + finalize.
    gate.engaged.store(false, Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(400));
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join bagd").expect("run_bagd Ok");

    let reader = BagReader::open(&out).expect("open bag");
    let recovered = data_frames_for(&reader, &topic).len();

    // The bag is left in place for the caller to inspect (record_health etc.);
    // the caller cleans it up via `summary.bag_paths`.
    let _ = std::fs::remove_file(&ready);
    (summary, recovered)
}

// ------------------------------------------------------------------
// Test 1 (HEADLINE) — the drain keeps draining THROUGH a stalled writer, and a
// sustained stall costs nothing but counted, attributed loss.
//
// This test's shape was REWRITTEN, not its subject. It used to be an A/B: the
// same graph run twice, once with `fault_inject_force_inline_writer`, asserting
// the concurrent drain lost strictly less than the blocked one. There is no
// inline path left to compare against — a run against ITSELF would have kept
// the name and proved nothing — so the property is pinned DIRECTLY instead:
// with a PATHOLOGICALLY shallow tap queue (Q=8) and a writer stalled for the
// whole burst, the drain still runs, defer-not-drop retains the in-flight
// batches, and every published frame is either IN the bag or counted in
// `frames_lost`. Nothing is `dropped_unwritten`, and nothing vanishes.
//
// The strict-improvement claim the A/B used to make now lives where it can
// still be measured: `defer_not_drop_loses_zero_where_drop_newest_loses` runs
// the real A/B on the seam that still has two arms.
// ------------------------------------------------------------------
#[test]
fn stall_keeps_the_drain_running_and_every_frame_is_accounted() {
    const Q: usize = 8;
    const BURST: u32 = 24;
    const PUBLISHED: u64 = BURST as u64 + 1; // + the seed (seq 0)

    let (summary, recovered) = run_stall_scenario("stall_threaded", Q, BURST);
    assert_eq!(
        summary.dropped_unwritten, 0,
        "defer-not-drop — a transient channel-full never drops (got {})",
        summary.dropped_unwritten
    );
    // Conservation: every published frame is either recorded or lost to counted
    // tap-queue overflow (frames_lost) — nothing vanishes unaccounted.
    assert_eq!(
        summary.messages + summary.frames_lost + summary.dropped_unwritten,
        PUBLISHED,
        "conservation — recorded {} + frames_lost {} + dropped {} == {PUBLISHED}",
        summary.messages,
        summary.frames_lost,
        summary.dropped_unwritten
    );
    // ANTI-VACUITY: a drain that stopped for the stall would be pinned at the
    // handful of frames the queue absorbed before it filled. The burst is paced
    // over ~600 ms against a 10 ms drain poll, so a LIVE drain sees most of it.
    assert!(
        summary.messages > Q as u64,
        "the drain must keep running THROUGH the stall — a blocked drain cannot \
         exceed the {Q}-deep tap queue (recorded {})",
        summary.messages
    );
    assert_eq!(
        recovered as u64, summary.messages,
        "recovered bag frames == summary.messages"
    );
    cleanup(&summary.bag_paths[0]);
}

/// Defer-not-drop: drive a channel-full scenario (a sustained writer
/// stall fills the bounded drain→writer channel) end to end and return the
/// summary + the count of data frames recovered from the finalized bag.
/// `force_drop` selects the drop-newest behavior (the A/B control) via
/// the fault-inject seam; `false` is the production DEFER path. A deep receive
/// queue (256) isolates the channel-full path as the ONLY loss mechanism (no tap
/// overflow), so the ONLY difference between the two arms is defer-vs-drop.
fn run_channel_full_scenario(label: &str, force_drop: bool) -> (BagdSummary, usize) {
    const Q: usize = 256;
    const BURST: u32 = 24;

    let mgr = make_manager(16);
    let topic = unique_topic(label);
    let out = unique_out(label);
    let ready = unique_out(&format!("{label}_ready"));
    let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, Q, 256);

    let gate = Arc::new(WriterStallGate::default());
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());
    cfg.fault_inject_force_drop_on_channel_full = force_drop;

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    // Engage a SUSTAINED stall, publish the seed so the first flush blocks the
    // writer, then burst the rest — the bounded channel fills and stays full.
    gate.engaged.store(true, Ordering::Relaxed);
    pubr.publish_raw(&build_frame(STALL_HASH, 0, 0, &[1u8; 16]))
        .expect("publish seed");
    assert!(
        wait_until(Duration::from_secs(3), || gate
            .entered
            .load(Ordering::Relaxed)),
        "the writer must ENTER the stall"
    );
    for i in 1..=BURST {
        pubr.publish_raw(&build_frame(STALL_HASH, i, i as u64, &[i as u8; 16]))
            .expect("publish burst");
        std::thread::sleep(Duration::from_millis(25));
    }
    // Release; let the writer catch up (defer arm flushes its retained frames),
    // then shut down + finalize.
    gate.engaged.store(false, Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(400));
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join bagd").expect("run_bagd Ok");

    let reader = BagReader::open(&out).expect("open bag");
    let recovered = data_frames_for(&reader, &topic).len();
    let _ = std::fs::remove_file(&ready);
    (summary, recovered)
}

// ------------------------------------------------------------------
// Test 2 (HEADLINE, defer-not-drop) — the LOAD-BEARING A/B: on a
// transient writer stall that fills the bounded drain→writer channel, the
// production DEFER path loses ZERO (frames retained in their taps, flushed once
// the writer catches up), while the drop-newest control (fault-inject
// force-drop) loses > 0 on the IDENTICAL graph + IDENTICAL stall. Mutation-
// verified: reverting flush_threaded to drop-newest makes the defer arm's
// `dropped_unwritten == 0` assertion fail (it becomes the drop arm's count).
// ------------------------------------------------------------------
#[test]
fn defer_not_drop_loses_zero_where_drop_newest_loses() {
    const PUBLISHED: u64 = 24 + 1; // BURST + the seed (seq 0)

    // DEFER (production default): a transient channel-full stall never drops —
    // every published frame is RETAINED and eventually recorded.
    let (defer, defer_recovered) = run_channel_full_scenario("chanfull_defer", false);
    assert_eq!(
        defer.frames_lost, 0,
        "defer: deep queue → zero tap-overflow loss"
    );
    assert_eq!(
        defer.dropped_unwritten, 0,
        "defer-not-drop loses ZERO on a transient channel-full stall (got {})",
        defer.dropped_unwritten
    );
    assert_eq!(
        defer.messages, PUBLISHED,
        "defer: every published frame reached the finalized bag"
    );
    assert_eq!(
        defer_recovered as u64, PUBLISHED,
        "defer: recovered bag frames == every published frame"
    );

    // FORCE-DROP (the drop-newest control / mutation): the SAME stall drops the
    // excess between drain and writer.
    let (drop, _drop_recovered) = run_channel_full_scenario("chanfull_drop", true);
    assert!(
        drop.dropped_unwritten > 0,
        "force-drop control: the drop-newest path loses frames on the identical \
         stall (got {})",
        drop.dropped_unwritten
    );
    assert_eq!(
        drop.messages + drop.dropped_unwritten,
        PUBLISHED,
        "force-drop: conservation (recorded {} + dropped {} == {PUBLISHED})",
        drop.messages,
        drop.dropped_unwritten
    );

    // The load-bearing contrast: identical graph + identical stall — only
    // defer-vs-drop differs, and only defer loses zero (Principle #6).
    assert!(
        defer.dropped_unwritten < drop.dropped_unwritten,
        "defer must strictly beat drop-newest (defer {} < drop {})",
        defer.dropped_unwritten,
        drop.dropped_unwritten
    );

    cleanup(&defer.bag_paths[0]);
    cleanup(&drop.bag_paths[0]);
}

/// The renamed counter's DURABLE surface carries a NONZERO value under
/// the NEW key.
///
/// The A/B above reads `BagdSummary::dropped_unwritten`, which
/// `finalize_threaded` sums straight off the taps — so it never touches
/// `build_record_health`, and hardcoding THAT sum to a literal `0` survived all
/// 154 bagd tests. The bag attachment is what `cerulion replay` actually reads
/// (`replay_cmd::read_record_health` is its only reader in the CLI engine — NOT
/// `bag info`, which renders the scan and the coverage manifest), and nothing
/// asserted it at any value but zero: every other arm in the repo runs a
/// healthy recording, where a stamp that reports nothing and a stamp that
/// reports correctly are the same bytes.
///
/// The force-drop seam is the one path that produces a nonzero count on a run
/// that still FINALIZES (both production causes — a latched write error and a
/// dead writer — also fail the finalize, so no bag is written at all), which is
/// why it exists and why it is the only way to reach this assertion.
///
/// Asserted on the RAW JSON, not the typed field: the wire key is the contract,
/// and a typed round trip would pass under the old spelling via the serde
/// `alias`. The precondition (`summary.dropped_unwritten > 0`) fails LOUDLY if
/// the seam ever stops dropping, so the arm cannot go vacuously green.
///
/// Hardcoding `build_record_health`'s `dropped_unwritten` sum to `0`
/// fails this — the stamp reports `0` while the run really dropped frames.
#[test]
fn the_unwritten_drop_count_reaches_the_durable_health_stamp_under_its_new_key() {
    let (summary, _recovered) = run_channel_full_scenario("chanfull_stamp", true);

    // PRECONDITION, not the pin: without a real drop the assertions below are
    // satisfied by a stamp that reports nothing.
    assert!(
        summary.dropped_unwritten > 0,
        "the force-drop seam must actually drop frames, or this arm proves nothing"
    );

    let reader = BagReader::open(&summary.bag_paths[0]).expect("open bag");
    let att = reader
        .attachment(RECORD_HEALTH_ATTACHMENT)
        .expect("attachment lookup")
        .expect("record_health.json present in a finalized bag");
    let health: serde_json::Value =
        serde_json::from_slice(&att.data).expect("record_health parses");

    assert_eq!(
        health["dropped_unwritten"].as_u64(),
        Some(summary.dropped_unwritten),
        "the DURABLE stamp must carry the run's real count, not a zero: {health}"
    );
    assert!(
        health.get("dropped_pre_writer").is_none(),
        "one quantity, one key — the retired spelling must not also be written: {health}"
    );
    assert_eq!(
        health["version"].as_u64(),
        Some(u64::from(cerulion_bagd::RECORD_HEALTH_VERSION)),
        "the stamp declares the version whose shape it actually has"
    );

    cleanup(&summary.bag_paths[0]);
}

// ------------------------------------------------------------------
// Test 2b — DEFER breadcrumb + record_health ACCURACY: the production defer path
// emits its `bagd DEFERRING a flush` breadcrumb (once), does NOT emit the old
// `DROPPING drained frames` warn, and finalizes with an all-zero-drop
// record_health. `run_bagd` runs INLINE on THIS thread so the breadcrumb lands
// in the `#[traced_test]` span (a worker-thread event would escape it — same
// pattern as `pre_writer_budget_overflow`).
// ------------------------------------------------------------------
#[test]
#[tracing_test::traced_test]
fn channel_full_defers_and_breadcrumbs_not_drops() {
    const Q: usize = 256;
    const BURST: u32 = 24;
    const PUBLISHED: u64 = BURST as u64 + 1;

    let mgr = make_manager(16);
    let topic = unique_topic("stall_defer");
    let out = unique_out("stall_defer");
    let ready = unique_out("stall_defer_ready");
    let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, Q, 256);

    let gate = Arc::new(WriterStallGate::default());
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();
    let gate_c = gate.clone();
    let shutdown_c = shutdown.clone();
    let control = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        gate_c.engaged.store(true, Ordering::Relaxed);
        pubr.publish_raw(&build_frame(STALL_HASH, 0, 0, &[1u8; 16]))
            .expect("publish seed");
        assert!(
            wait_until(Duration::from_secs(3), || gate_c
                .entered
                .load(Ordering::Relaxed)),
            "the writer must ENTER the stall"
        );
        for i in 1..=BURST {
            pubr.publish_raw(&build_frame(STALL_HASH, i, i as u64, &[i as u8; 16]))
                .expect("publish burst");
            std::thread::sleep(Duration::from_millis(25));
        }
        gate_c.engaged.store(false, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(400));
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    control.join().expect("control thread");

    // No tap overflow (deep queue) AND no channel-full drop (defer-not-drop).
    assert_eq!(
        summary.frames_lost, 0,
        "no tap-queue overflow with a deep queue"
    );
    assert_eq!(
        summary.dropped_unwritten, 0,
        "defer-not-drop: a transient channel-full stall drops NOTHING (got {})",
        summary.dropped_unwritten
    );
    assert_eq!(
        summary.messages, PUBLISHED,
        "every published frame was recorded (recorded {}, published {PUBLISHED})",
        summary.messages
    );
    // The DEFER breadcrumb fired (once); the old DROPPING warn did NOT.
    assert!(
        logs_contain("bagd DEFERRING a flush"),
        "the defer path must emit its breadcrumb"
    );
    assert!(
        !logs_contain("bagd DROPPING drained frames"),
        "the defer path must NOT emit the old channel-full drop warn"
    );
    // The bag FINALIZED cleanly with an all-zero-drop record_health.
    let health = read_record_health(&out);
    assert_eq!(
        health.dropped_unwritten, 0,
        "record_health carries zero drops on the defer path"
    );
    let th = health.topics.values().next().expect("one topic in health");
    assert_eq!(
        th.frames_lost, 0,
        "record_health per-topic frames_lost == 0"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ------------------------------------------------------------------
// Test 3 — QUIET-RUN correctness: a no-stall run records the FULL frame stream,
// in order, byte-identical to a HAND ORACLE + an all-zero record_health.
//
// The oracle used to be a second run through the inline path. With one
// write path left, the surviving — and always stronger — half is the hand
// oracle, which is what makes this a cross-check rather than a self-compare.
// ------------------------------------------------------------------
#[test]
fn quiet_run_records_the_full_stream_in_order() {
    fn run_quiet(label: &str) -> (BagdSummary, Vec<Vec<u8>>) {
        const N: u32 = 24;
        let mgr = make_manager(16);
        let topic = unique_topic(label);
        let out = unique_out(label);
        let ready = unique_out(&format!("{label}_ready"));
        let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, 64, 256);

        let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
        cfg.schema_wait = Duration::from_millis(200);

        let shutdown = Arc::new(AtomicBool::new(false));
        let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
        assert!(wait_for_file(&ready, LIVENESS_CEILING), "ready-file");
        settle();
        // Paced so no flush is ever contended — a genuinely quiet recording.
        for i in 0..N {
            pubr.publish_raw(&build_frame(
                STALL_HASH,
                i,
                100 + i as u64,
                &[(i as u8) + 1; 16],
            ))
            .expect("publish");
            std::thread::sleep(Duration::from_millis(10));
        }
        std::thread::sleep(Duration::from_millis(200));
        shutdown.store(true, Ordering::Relaxed);
        let summary = handle.join().expect("join").expect("run_bagd Ok");
        let reader = BagReader::open(&out).expect("open");
        let frames = data_frames_for(&reader, &topic);
        cleanup(&out);
        let _ = std::fs::remove_file(&ready);
        (summary, frames)
    }

    let (summary, frames) = run_quiet("quiet_threaded");

    // The full stream reached the bag, in order, against a HAND oracle.
    let expect: Vec<Vec<u8>> = (0..24u32)
        .map(|i| build_frame(STALL_HASH, i, 100 + i as u64, &[(i as u8) + 1; 16]))
        .collect();
    assert_eq!(
        frames, expect,
        "every frame recorded, in order (hand oracle)"
    );
    assert_eq!(summary.messages, 24, "recorded all frames");
    assert_eq!(summary.frames_lost, 0, "quiet run loses nothing");
    assert_eq!(summary.dropped_unwritten, 0, "quiet run drops nothing");
    // All-zero health on a healthy run.
    assert_eq!(
        summary.record_health.dropped_unwritten, 0,
        "healthy run: all-zero record_health"
    );
}

// ------------------------------------------------------------------
// Test 4 — SHUTDOWN ordering under a stall: SIGINT mid-stall finalizes cleanly
// once the stall clears — no lost tail beyond the reported accounting; the bag is
// Finalized with the manifest + record_health present.
// ------------------------------------------------------------------
#[test]
fn shutdown_mid_stall_finalizes_cleanly_with_absorbed_frames() {
    const N: u32 = 6;
    let mgr = make_manager(16);
    let topic = unique_topic("stall_shutdown");
    let out = unique_out("stall_shutdown");
    let ready = unique_out("stall_shutdown_ready");
    // Deep queue so nothing is lost to tap-overflow — the only accounting here
    // is channel-full drops, all counted.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, 64, 256);

    let gate = Arc::new(WriterStallGate::default());
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.flush_interval = Duration::from_millis(20);
    cfg.schema_wait = Duration::from_millis(200);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, LIVENESS_CEILING), "ready-file");
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::new();
    // Engage the stall, then publish N frames (paced) — the first lands in the
    // writer's in-flight batch (which stalls), the next in the channel, and the
    // rest are DEFERRED (defer-not-drop: retained in the tap, not dropped)
    // then flushed once the stall clears at shutdown. Conservation still holds
    // (recorded + dropped == N) with dropped == 0 on this transient stall.
    gate.engaged.store(true, Ordering::Relaxed);
    for i in 0..N {
        let frame = build_frame(STALL_HASH, i, 200 + i as u64, &[(i as u8) + 1; 12]);
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
        std::thread::sleep(Duration::from_millis(25));
    }
    assert!(
        wait_until(Duration::from_secs(3), || gate
            .entered
            .load(Ordering::Relaxed)),
        "the writer must have entered the stall"
    );

    // SIGINT-equivalent WHILE the writer is still stalled.
    shutdown.store(true, Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(100));
    // Release the stall so the shutdown's blocking tail-flush + finalize can
    // complete (a truly stuck disk would hang here — same as any writev).
    gate.engaged.store(false, Ordering::Relaxed);

    let summary = handle.join().expect("join").expect("run_bagd Ok");

    // Clean shutdown: the bag is FINALIZED (data_frames_for asserts it), the
    // recorded frames are an in-order PREFIX of what was published (no
    // reordering / no lost-tail beyond the reported drop accounting), and
    // conservation holds: recorded + dropped == published, frames_lost == 0.
    let reader = BagReader::open(&out).expect("open");
    let frames = data_frames_for(&reader, &topic); // asserts Finalized
    assert_eq!(summary.frames_lost, 0, "deep queue → no tap overflow");
    assert_eq!(
        summary.messages + summary.dropped_unwritten,
        N as u64,
        "conservation: recorded {} + dropped {} == {N}",
        summary.messages,
        summary.dropped_unwritten
    );
    assert!(
        summary.messages >= 1,
        "at least the absorbed frames were finalized"
    );
    assert_eq!(
        frames.len() as u64,
        summary.messages,
        "bag frames == messages"
    );
    assert_eq!(
        frames,
        expect[..frames.len()],
        "recorded frames are an in-order prefix of the published stream (no lost tail \
         beyond the reported channel-full drop accounting)"
    );
    // manifest/health path: record_health attachment present in the finalized bag.
    let health = read_record_health(&out);
    assert_eq!(health.dropped_unwritten, summary.dropped_unwritten);

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================================
// Zero-copy trace records: the writer thread `writev`s straight
// over the trace ring's committed SHM span(s) (wrap = two iovecs, never a
// copy), advancing the ring read cursor only AFTER the write lands. The rank
// stamp is an iovec substitution (record[0..36] from SHM + 4 owned rank bytes),
// byte-identical to the copy path's `rec.reserved = rank`.
// ============================================================================

/// Hand-build the `k`-th trace record for the rider tests (distinct fields so
/// any reorder/tear shows).
fn rider_record(k: u64) -> TraceRingRecord {
    TraceRingRecord {
        step: k,
        fire_time_ns: 9000 + k,
        duration_ns: 77 + k,
        node_idx: (k % 3) as u32,
        global_level: (k % 2) as u32,
        record_type: 1,
        reserved: 0, // producer always pushes 0; bagd stamps the ring rank
    }
}

// ------------------------------------------------------------------
// Rider test 1 (STRONGEST PIN) — the zero-copy trace channel equals a HAND
// ORACLE: every record, in push order, framed as seq 0..N with
// log_time == publish_time == fire_time_ns and `reserved` stamped to the ring's
// header rank (7).
//
// The second arm was the same input through the COPYING inline path.
// That path is gone, and re-running the identical path would have compared this
// run to itself — the oracle is what carried the pin, and it is kept whole.
// ------------------------------------------------------------------
#[test]
fn zero_copy_trace_is_byte_identical_to_the_oracle() {
    const N_REC: u64 = 6;
    const RANK: u32 = 7;

    fn run_one(label: &str) -> (BagdSummary, Vec<cerulion_bag::BagMessage>) {
        let mgr = make_manager(16);
        let topic = unique_topic(label);
        let out = unique_out(label);
        let ready = unique_out(&format!("{label}_ready"));
        let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, 32, 256);

        let ring_tag = unique_ring_tag(label);
        let mut owner =
            TraceRingOwner::create(&ring_tag, 16, RANK, &["n0", "n1", "n2"]).expect("ring create");
        let mut producer = owner.producer().expect("producer");

        let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
        cfg.rings = vec![owner.name().to_string()];
        cfg.schema_wait = Duration::from_millis(200);

        let shutdown = Arc::new(AtomicBool::new(false));
        let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
        assert!(wait_for_file(&ready, LIVENESS_CEILING), "ready-file");
        settle();

        // One data frame (learns the schema, creates the bag) + the records.
        pubr.publish_raw(&build_frame(0x651A, 0, 42, &[9u8; 8]))
            .expect("publish seed");
        for k in 0..N_REC {
            producer.push(&rider_record(k));
        }
        // Generous absorb window (flush cadence 20ms); a missed record would
        // fail the count assert below, never pass silently.
        std::thread::sleep(Duration::from_millis(400));
        shutdown.store(true, Ordering::Relaxed);
        let summary = handle.join().expect("join").expect("run_bagd Ok");

        let reader = BagReader::open(&out).expect("open bag");
        let msgs = data_msgs_for(&reader, SCHEDULER_TRACE_TOPIC);
        let _ = std::fs::remove_file(&ready);
        cleanup(&out);
        (summary, msgs)
    }

    let (summary, msgs) = run_one("zc_trace_threaded");

    // Hand oracle: every record, in push order, with reserved stamped to 7 —
    // framed as seq 0..N and log_time == publish_time == fire_time_ns.
    assert_eq!(summary.ring_records, N_REC, "recorded all records");
    assert_eq!(msgs.len() as u64, N_REC);
    for (k, m) in msgs.iter().enumerate() {
        let expect = TraceRingRecord {
            reserved: RANK,
            ..rider_record(k as u64)
        };
        assert_eq!(
            m.data,
            expect.as_bytes().to_vec(),
            "record {k}: zero-copy payload bytes == hand oracle (rank-stamped)"
        );
        assert_eq!(m.sequence, k as u32, "record {k}: trace seq");
        assert_eq!(
            m.log_time,
            9000 + k as u64,
            "record {k}: log_time == fire_time_ns"
        );
        assert_eq!(m.publish_time, m.log_time, "record {k}: publish_time");
    }
}

// ------------------------------------------------------------------
// Rider test 2 — deterministic WRAP-AROUND: a capacity-8 ring is consumed to
// cursor 5, then (with the writer stalled so no partial drain can split the
// region) 6 more records are pushed — the committed region [5..11) wraps the
// ring end and MUST be handed as two iovec spans, never a copy. All 11 records
// land in push order, rank-stamped.
// ------------------------------------------------------------------
#[test]
fn wrapped_ring_span_lands_in_order_via_two_iovec_spans() {
    const RANK: u32 = 3;
    let mgr = make_manager(16);
    let topic = unique_topic("zc_wrap");
    let out = unique_out("zc_wrap");
    let ready = unique_out("zc_wrap_ready");
    let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, 32, 256);

    let ring_tag = unique_ring_tag("zc_wrap");
    let mut owner = TraceRingOwner::create(&ring_tag, 8, RANK, &["a"]).expect("ring create");
    let mut producer = owner.producer().expect("producer");

    let gate = Arc::new(WriterStallGate::default());
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.rings = vec![owner.name().to_string()];
    cfg.schema_wait = Duration::from_millis(200);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());
    // The live status feed is this arm's only window onto the
    // recorder's RING progress. The ring's read cursor is CONSUMER-LOCAL (there
    // is no shared header field a producer-side test could read), so
    // `ring_records` on `/bagd/status` — which folds the writer thread's live
    // counter — is the one observable that says "the writer consumed them".
    cfg.status_period = Some(Duration::from_millis(20));

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, LIVENESS_CEILING), "ready-file");
    settle();
    let mut status_sub = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");

    // Bag created (writer spawned) once the seed frame learns the schema.
    pubr.publish_raw(&build_frame(0x651B, 0, 7, &[1u8; 8]))
        .expect("publish seed");
    assert!(
        wait_until(LIVENESS_CEILING, || out.exists()),
        "bag file must exist (writer thread spawned)"
    );

    // First phase: 5 records, which the writer must CONSUME (cursor → 5) before
    // phase 2 pushes 6 more into the capacity-8 ring.
    //
    // A fixed 400 ms sleep used to stand here. Its failure mode was loud
    // (an unconsumed phase 1 makes phase 2 overrun and `run_bagd` returns
    // `RingOverrun`), so this was never a silent pass — but it was still a WALL
    // bet on a ~20 ms flush cadence, and a later flake MEASURED macOS background-QoS
    // timer coalescing charging a nominal 150 ms as 1100-1696 ms. A bet with
    // that little headroom is a latent CI red whatever it fails as.
    for k in 0..5u64 {
        producer.push(&rider_record(k));
    }
    assert!(
        await_status_ring_records(&mut status_sub, 5, LIVENESS_CEILING),
        "the writer must CONSUME phase 1 before phase 2 laps the capacity-8 ring"
    );

    // Second phase: stall the writer BEFORE its drain (maybe_stall precedes
    // drain_slices), push 6 more — committed region [5..11) wraps the ring end
    // (slots 5..8 + 0..3). The post-release drain sees the WHOLE region as one
    // wrapped pair of spans — deterministic two-iovec-span coverage.
    //
    // CONFIRM the stall, which four sibling arms do and this one did
    // not. Engaging the gate alone leaves a real race: a `write_batch` that
    // passed `maybe_stall` microseconds EARLIER runs on, so it can reach
    // `drain_slices` while the pushes below are still landing and take the
    // region in PIECES — the wrap is then never handed as one wrapped pair, the
    // arm's whole subject goes untested, and every assertion still passes.
    //
    // Two details are load-bearing and each was found by RUNNING the naive form:
    //
    //  * `entered` is RESET first. `maybe_stall` sets it on EVERY pass whether
    //    or not the gate is engaged, so phase 1's writes have already latched
    //    it and waiting on the un-reset flag is the vacuous check this replaces.
    //  * the writer needs WORK to reach `maybe_stall` at all — batches are
    //    handed only when something is pending — so one data frame is published
    //    to summon it. Without that the wait simply expires (MEASURED: 30.2 s).
    //    The frame is unasserted, exactly like the seed above.
    gate.entered.store(false, Ordering::Relaxed);
    gate.engaged.store(true, Ordering::Relaxed);
    pubr.publish_raw(&build_frame(0x651B, 1, 8, &[2u8; 8]))
        .expect("publish stall summons");
    assert!(
        wait_until(LIVENESS_CEILING, || gate.entered.load(Ordering::Relaxed)),
        "the writer must be INSIDE the stall before the wrapping region is pushed"
    );
    for k in 5..11u64 {
        producer.push(&rider_record(k));
    }
    gate.engaged.store(false, Ordering::Relaxed);
    assert!(
        await_status_ring_records(&mut status_sub, 11, LIVENESS_CEILING),
        "the released writer must consume the WRAPPED region"
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    assert_eq!(summary.ring_records, 11, "all 11 records recorded");

    let reader = BagReader::open(&out).expect("open bag");
    let trace = reader.scheduler_trace().expect("scheduler_trace");
    let expect: Vec<TraceRingRecord> = (0..11u64)
        .map(|k| TraceRingRecord {
            reserved: RANK,
            ..rider_record(k)
        })
        .collect();
    assert_eq!(
        trace, expect,
        "all 11 records in push order across the wrap, rank-stamped"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ------------------------------------------------------------------
// Rider test 3 — the IN-FLIGHT-HOLD overrun arm (constraint (a)): while the
// writer holds drained-but-uncommitted ring slices, the producer laps the held
// span. The post-writev `commit` re-validation detects the lap and the run
// fails with the ring's EXACT `records_lost` accounting — counted and loud,
// never a silent torn read; the bag is left un-finalized.
// ------------------------------------------------------------------
#[test]
fn producer_lap_during_inflight_hold_fails_loud_with_exact_records_lost() {
    const CAP: u32 = 8;
    let mgr = make_manager(16);
    let topic = unique_topic("zc_hold");
    let out = unique_out("zc_hold");
    let ready = unique_out("zc_hold_ready");
    let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, 32, 256);

    let ring_tag = unique_ring_tag("zc_hold");
    let mut owner = TraceRingOwner::create(&ring_tag, CAP, 0, &["a"]).expect("ring create");
    let mut producer = owner.producer().expect("producer");

    let gate = Arc::new(WriterStallGate::default());
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.rings = vec![owner.name().to_string()];
    cfg.schema_wait = Duration::from_millis(200);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, LIVENESS_CEILING), "ready-file");
    settle();

    pubr.publish_raw(&build_frame(0x651C, 0, 7, &[1u8; 8]))
        .expect("publish seed");
    assert!(
        wait_until(Duration::from_secs(3), || out.exists()),
        "bag file must exist (writer thread spawned)"
    );

    // Engage the RING-HOLD gate (stalls AFTER drain_slices, BEFORE writev),
    // then push: the writer drains a span and holds the read cursor at 0.
    gate.ring_hold_engaged.store(true, Ordering::Relaxed);
    for k in 0..3u64 {
        producer.push(&rider_record(k));
    }
    assert!(
        wait_until(Duration::from_secs(3), || gate
            .ring_hold_entered
            .load(Ordering::Relaxed)),
        "the writer must be holding drained-but-uncommitted ring slices"
    );

    // Lap the HELD span: 10 more pushes → write_cursor 13, read_cursor 0 —
    // 13 - 0 > 8, and slots 0..2 (the held span) were overwritten mid-hold.
    for k in 3..13u64 {
        producer.push(&rider_record(k));
    }
    // Release: the writer writevs (possibly torn bytes — irrelevant, the bag
    // will not finalize) then `commit` re-validates and detects the lap.
    gate.ring_hold_engaged.store(false, Ordering::Relaxed);

    // The failure latches; the drive loop aborts and finalize surfaces it.
    shutdown.store(true, Ordering::Relaxed);
    let result = handle.join().expect("join");
    match result {
        Err(BagdError::RingOverrun { records_lost, .. }) => {
            // EXACT accounting: write 13, read 0, capacity 8 → 5 lost.
            assert_eq!(
                records_lost, 5,
                "records_lost must be the ring's exact lap arithmetic (13 - 8)"
            );
        }
        other => panic!("expected RingOverrun from the in-flight-hold lap, got {other:?}"),
    }
    // The bag must read back NOT finalized (truncated) — replay refuses it.
    let reader = BagReader::open(&out).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "a torn-trace recording must NOT be finalized, got {completeness:?}"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================================
// The writer thread DYING mid-run, by a latched write error
// or a panic. Both drive the same drain-side accounting: frames
// handed off to a DEAD writer must be counted `dropped_unwritten` (and
// NOT folded as `messages`), the run must return Err LOUDLY, and the bag must
// read back un-finalized. Threaded path (borrow 22); a deep receive queue
// isolates the loss to the dead writer (no tap-queue overflow).
// ============================================================================

/// Drive a threaded recording where the writer thread dies mid-run and return
/// `run_bagd`'s result + the terminal "failed" `/bagd/status` frame (the counter
/// telemetry — the run returns Err, so there is no `BagdSummary`).
///
/// `n` frames go into a DEEP receive queue in THREE deliberately-separated
/// batches, so that the last one is provably handed off AFTER the writer has
/// failed. See the phase block below for why that separation is mechanical
/// rather than timed. `write_error_after` arms the write-error origin;
/// `panic` arms the writer panic.
fn run_writer_death_scenario(
    label: &str,
    n: u32,
    write_error_after: Option<u64>,
    panic: bool,
) -> (
    Result<BagdSummary, BagdError>,
    serde_json::Value,
    std::path::PathBuf,
) {
    let mgr = make_manager(16);
    let topic = unique_topic(label);
    let out = unique_out(label);
    let ready = unique_out(&format!("{label}_ready"));
    // Deep receive queue (256): the drain never overflows the tap — the ONLY
    // loss mechanism is the dead writer, so dropped_unwritten is attributable.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, REC_BORROW, 256, 256);

    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.flush_interval = Duration::from_millis(10);
    cfg.schema_wait = Duration::from_millis(200);
    // One chunk per message: this is what makes "the writer wrote batch one"
    // readable from OUTSIDE the recorder (stage 1 below polls the bag itself).
    cfg.chunk_max_bytes = 1;
    // Status ON so the terminal "failed" frame carries the drain-side counters.
    cfg.status_period = Some(Duration::from_millis(20));
    cfg.fault_inject_flush_error_after_messages = write_error_after;
    cfg.fault_inject_writer_panic = panic;
    // The write-error fault starts DISARMED and is opened only after
    // this harness has OBSERVED the recorder record something (see below). The
    // panic seam is unaffected.
    let armed = std::sync::Arc::new(AtomicBool::new(false));
    if write_error_after.is_some() {
        cfg.fault_inject_write_error_armed = Some(armed.clone());
    }
    // The writer STALL gate, engaged from the very first
    // batch. It is what turns this scenario from a race into a sequence — see
    // the phase block below.
    let stall = Arc::new(WriterStallGate::default());
    stall.engaged.store(true, Ordering::Relaxed);
    cfg.fault_inject_writer_stall_gate = Some(stall.clone());

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(wait_for_file(&ready, LIVENESS_CEILING), "ready-file");
    // Connect the status subscriber BEFORE the failure so the terminal frame
    // (published at error time) is queued on it.
    let mut status_sub = mgr
        .create_data_only_subscriber(cerulion_bagd::STATUS_TOPIC)
        .expect("status subscriber");
    // ONE accumulating buffer for every status wait below. `drain_status_json`
    // CONSUMES from the subscriber, so a wait with its own throwaway vec can
    // swallow a frame a later wait needs -- including the terminal one.
    let mut statuses: Vec<serde_json::Value> = Vec::new();
    settle();

    // THE SEQUENCE. Both things this scenario must produce
    // pull in opposite directions, and every earlier attempt to reconcile them
    // was a WALL BET that CI eventually collected on.
    //
    // The consuming tests assert BOTH that frames were written BEFORE the
    // failure (their anti-vacuity arm) and that frames handed off AFTER it are
    // counted `dropped_unwritten` (the property under test). The first half had
    // already been made observable, but the second half stayed a bet: phase two
    // was PACED at 10 ms per frame so the burst would span several drain passes
    // and some would land after the latch.
    //
    // That bet lost on macOS CI (`threaded_write_error_...` at `dropped >= 1`).
    // Pacing cannot be made safe by widening, because the failure is not "too
    // early" -- it is the drive loop EXITING. `drive_loop` returns the moment it
    // observes `failed`, and its pass order is drain -> flush -> check, so a
    // pass that finds an empty queue exits having dropped nothing. Under macOS
    // background-QoS coalescing the publisher's sleeps stretch far past the
    // recorder's poll, which is exactly a stream of empty passes.
    //
    // So the pacing is gone and the ordering is now MECHANICAL. Two structural
    // facts carry it, and neither is a duration:
    //
    //   * `RECORDING_WRITER_CHANNEL_BOUND` is 1. With one batch held by a
    //     stalled writer and one queued behind it, the NEXT hand-off cannot be
    //     sent at all -- it DEFERS, and its frames stay held on the drain side.
    //   * the writer's failure fires on the batch it is STALLED on. Both seams
    //     sit immediately after `maybe_stall()` in `write_batch`, so releasing
    //     the gate runs straight into the failure with NO intervening dequeue.
    //
    // That second point is what a first cut of this rewrite got wrong, and the
    // `taskpolicy -b` runs found it: if the writer fails on a batch it has to
    // DEQUEUE first, the channel frees for a moment before the failure lands,
    // and a drain retry inside that window hands the last batch to a
    // still-writable writer -- the frames fold as `messages` and `dropped` is 0
    // (MEASURED: 2 failures in 20 under background QoS). Failing on the STALLED
    // batch closes it: the channel still holds batch 3 at the instant of the
    // failure, so batch 4 provably cannot have been handed off before it.
    //
    // Every wait below is a condition on an observable -- the gate's `entered`
    // flag, the bag's own contents, the status feed's counters.

    // STAGE 1 -- batch 1 is ONE frame, so the drain cannot split it, and it is
    // driven all the way to DURABLE. Its purpose is `written_total >= 1`: both
    // seams require it, which is what makes the FAILING batch a later batch,
    // i.e. one the writer can be stalled on.
    pubr.publish_raw(&build_frame(STALL_HASH, 0, 0, &[0u8; 16]))
        .expect("publish batch one");
    {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !stall.entered.load(Ordering::Relaxed) {
            assert!(
                Instant::now() < deadline,
                "the writer never reached the stall gate -- it never took a batch"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }
    stall.engaged.store(false, Ordering::Relaxed);
    // `chunk_max_bytes = 1` closes a chunk per message, so "the writer wrote
    // batch one" is readable from OUTSIDE the recorder. Nothing else has been
    // published, so a durable batch 1 also means the writer is now blocked in
    // `recv` -- which is what stage 2 relies on.
    {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !bag_has_messages(&out) {
            assert!(
                Instant::now() < deadline,
                "batch one never became durable, so the writer is not provably past it"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    // STAGE 2 -- batch 2 is the batch that FAILS, and the writer stalls ON it.
    // Re-arming the gate BEFORE publishing is what makes that certain: the
    // writer cannot reach `write_batch` until this frame exists.
    stall.entered.store(false, Ordering::Relaxed);
    stall.engaged.store(true, Ordering::Relaxed);
    armed.store(true, Ordering::Relaxed);
    pubr.publish_raw(&build_frame(STALL_HASH, 1, 1, &[1u8; 16]))
        .expect("publish batch two");
    {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !stall.entered.load(Ordering::Relaxed) {
            assert!(
                Instant::now() < deadline,
                "the writer never stalled on batch two -- the failing batch is not the held one"
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    // STAGE 3 -- batch 3 FILLS the channel. Its hand-off is counted, and that
    // count is the observable proving the channel is now full.
    pubr.publish_raw(&build_frame(STALL_HASH, 2, 2, &[2u8; 16]))
        .expect("publish batch three");
    {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            drain_status_json(&mut status_sub, &mut statuses);
            if statuses
                .iter()
                .any(|s| s["messages"].as_u64().is_some_and(|m| m >= 3))
            {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "recorder never reported three handed-off frames, so the drain->writer channel \
                 was never filled; status frames seen: {statuses:?}"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    // STAGE 4 -- batch 4 is everything else. The drain picks it up and its
    // hand-off DEFERS on the full channel, so these frames stay held on the
    // drain side and cannot reach the writer until batch 3 is dequeued -- which
    // happens only after the failure.
    for i in 3..n {
        pubr.publish_raw(&build_frame(STALL_HASH, i, i as u64, &[i as u8; 16]))
            .expect("publish batch four");
    }
    // Release: straight into the failure, with the channel still full.
    stall.engaged.store(false, Ordering::Relaxed);
    // The run ENDS ON ITS OWN: `drive_loop` returns as soon as it observes the
    // writer's `failed` latch, and `run_bagd` then finalizes to an `Err`. So
    // wait for that rather than sleeping a fixed window and forcing shutdown —
    // a forced shutdown races the failure, and winning that race would flush
    // batch 3 through a still-writable writer (counting it as `messages`, which
    // is the very confusion this scenario exists to rule out).
    //
    // The PANIC seam is the exception and needs the second arm: an unwinding
    // writer never reaches the `failed` latch (`writer_gone`, the Disconnected
    // send, is the only reliable signal there), so its drive loop keeps running.
    // Its batch-3 hand-off is decided the same way regardless — the channel is
    // already disconnected — and the recorder REPORTS that decision on its
    // status feed, so `dropped_unwritten >= 1` is the observable that says the
    // scenario is complete and a shutdown can no longer change the outcome.
    //
    // `shutdown` after the deadline is a SAFETY NET for a run that never fails
    // at all; such a run fails the assertions below regardless, and this only
    // stops it hanging the suite.
    {
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            if handle.is_finished() {
                break;
            }
            drain_status_json(&mut status_sub, &mut statuses);
            if statuses
                .iter()
                .any(|s| s["dropped_unwritten"].as_u64().is_some_and(|d| d >= 1))
            {
                break;
            }
            if Instant::now() >= deadline {
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        shutdown.store(true, Ordering::Relaxed);
    }
    let result = handle.join().expect("join bagd");

    // The terminal "failed" status frame (published on run_bagd's error arm).
    let deadline = Instant::now() + Duration::from_secs(5);
    let terminal = loop {
        drain_status_json(&mut status_sub, &mut statuses);
        if let Some(t) = statuses
            .iter()
            .rev()
            .find(|s| s["state"].as_str() == Some("failed"))
        {
            break t.clone();
        }
        assert!(
            Instant::now() < deadline,
            "no terminal failed status frame arrived; got {statuses:?}"
        );
        std::thread::sleep(Duration::from_millis(10));
    };

    // The caller reads the (un-finalized) bag back, then cleans up.
    let _ = std::fs::remove_file(&ready);
    (result, terminal, out)
}

// ------------------------------------------------------------------
// The THREADED WRITE-ERROR origin: a mid-run write_chunk failure
// on the writer thread latches, the run returns Err LOUDLY, the bag is NOT
// finalized, and every frame handed off AFTER the error is counted
// dropped_unwritten (never folded as messages). Swallowing
// the write_chunk error (`let _ = ...`) makes this test FAIL (the run would
// finalize Ok).
// ------------------------------------------------------------------
#[test]
fn threaded_write_error_fails_loud_and_counts_post_error_frames_dropped() {
    const N: u32 = 40;
    // The injected write error is ARMED BY THE HARNESS, only after it
    // has OBSERVED the recorder record at least one message on its status feed
    // (see `run_writer_death_scenario`). `Some(1)` therefore means "error on the
    // next message once the window is open", not "error on message 1" — the
    // `messages >= 1` arm below is a consequence of the arming, not a race.
    let (result, terminal, out) = run_writer_death_scenario("threaded_werr", N, Some(1), false);

    // (1) finalize returns Err LOUDLY — the injected BagError propagates as
    // BagdError::Bag with the fault-injection text.
    match &result {
        Err(BagdError::Bag(e)) => {
            let msg = e.to_string();
            assert!(
                msg.contains("fault injection")
                    && msg.contains("writer-thread chunk write failure"),
                "the writer-thread write error must surface loudly, got: {msg}"
            );
        }
        other => panic!(
            "expected Err(BagdError::Bag(..)) from the writer-thread write error, got {other:?}"
        ),
    }

    // (4) the terminal "failed" status telemetry reflects the same accounting.
    assert_eq!(
        terminal["state"].as_str(),
        Some("failed"),
        "the terminal status frame is 'failed'"
    );
    let messages = terminal["messages"].as_u64().expect("messages u64");
    let dropped = terminal["dropped_unwritten"]
        .as_u64()
        .expect("dropped_unwritten u64");
    assert_eq!(
        terminal["frames_lost"].as_u64(),
        Some(0),
        "deep queue: no tap-queue overflow — all loss is the dead writer"
    );

    // (3) THE DEAD-WRITER ACCOUNTING PIN: frames handed off AFTER the writer errored are counted
    // dropped_unwritten, NOT messages. The errored writer keeps draining the
    // channel fast, so those post-error hand-offs SUCCEED the send — the ONLY
    // reason they are dropped is the drain-side `failed`-flag check. Without it
    // they would fold as `messages` and `dropped` would be ~0.
    assert!(
        dropped >= 1,
        "post-error hand-offs must be counted dropped_unwritten; got {dropped}"
    );
    // Some frames were written before the error (bounded hand oracle, not a
    // self-compare): >= 1 (at least one real tap write) and both counts are a
    // subset of what was published — no double-count.
    assert!(
        messages >= 1,
        "at least one frame was written before the error; got {messages}"
    );
    assert!(
        messages + dropped <= N as u64,
        "recorded {messages} + dropped {dropped} must not exceed the {N} published"
    );

    // (2) the bag reads back NOT finalized (a failed recording is truncated).
    let reader = BagReader::open(&out).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "a failed recording must NOT be finalized, got {completeness:?}"
    );
    cleanup(&out);
}

// ------------------------------------------------------------------
// The writer-thread PANIC path: a mid-run panic disconnects the
// channel; the drain-side dead-writer branch folds every later (Disconnected)
// hand-off as dropped_unwritten, the join-Err maps to
// Config("bagd writer thread panicked"), and the bag is NOT finalized. A no-op
// panic hook is installed for the duration so the writer's unwind does not spew
// a backtrace to CI stderr.
// ------------------------------------------------------------------
#[test]
fn threaded_writer_panic_surfaces_config_error_and_counts_dropped() {
    const N: u32 = 40;
    // Suppress the writer thread's panic backtrace (serial test — --test-threads=1).
    // RAII guard, scope: run_writer_death_scenario itself contains
    // panicking asserts (ready-file timeout, join expect). If one of THOSE
    // fires, its message is swallowed regardless (the no-op hook is installed
    // at panic time — hooks run before unwinding), and per the rule
    // below the no-op hook then LEAKS for the rest of this serial binary —
    // the price of not aborting. The guard's restore only ever runs on the
    // success path, via the explicit drop below.
    //
    // The `thread::panicking()` check is load-bearing:
    // `std::panic::set_hook` PANICS when called from a panicking thread, and std
    // raises that as a NON-UNWINDING panic, i.e. `abort()`. Restoring
    // unconditionally in `Drop` therefore turns the very unwind this guard exists
    // to survive (a panicking assert inside run_writer_death_scenario) into SIGABRT,
    // killing the whole test binary and losing the failing test's name. On the
    // unwind path we deliberately LEAK the no-op hook instead.
    //
    // ⚠️ COST, stated because it is easy to miss: `set_hook` is
    // PROCESS-GLOBAL. This binary runs its tests in PARALLEL, so for as long
    // as the no-op hook is installed
    // EVERY OTHER TEST IN THE PROCESS also loses its panic message — a sibling
    // failing in this window reports a bare `panicked at ...` with no assertion
    // text, or (on the leak path above) for the rest of the run. If you are
    // debugging a message-less panic anywhere in this file, suspect this first,
    // and re-run the suspect test alone (`--exact`) to get its text back.
    //
    // The window is already as narrow as it can be — installed immediately
    // before `run_writer_death_scenario` and dropped immediately after, because
    // the noise it suppresses is that call's DELIBERATE writer-thread panic.
    // Narrowing further is not possible: the panic happens on a thread the
    // scenario owns, and Rust has no per-thread hook.
    type PanicHook = Box<dyn Fn(&std::panic::PanicHookInfo<'_>) + Sync + Send>;
    struct PanicHookGuard(Option<Arc<PanicHook>>);
    impl Drop for PanicHookGuard {
        fn drop(&mut self) {
            if std::thread::panicking() {
                return;
            }
            if let Some(prev) = self.0.take() {
                // Drop OUR hook first — it holds the other `Arc` clone, and
                // only once it is gone can the original Box be recovered.
                drop(std::panic::take_hook());
                if let Ok(original) = Arc::try_unwrap(prev) {
                    std::panic::set_hook(original);
                }
            }
        }
    }
    let hook_guard = {
        // FILTERING, not silencing. The hook is
        // process-global and this binary runs its tests in PARALLEL, so a
        // blanket no-op swallowed CONCURRENT tests' panic messages for as long
        // as it was installed — a failing sibling reported a bare `panicked
        // at ...` with no assertion text. That is not hypothetical: a real CI
        // failure arrived exactly that way, with the whole diagnostic
        // gone. The hook now suppresses EXACTLY the deliberate writer-thread
        // fault and forwards everything else to the previous hook.
        let prev = Arc::new(std::panic::take_hook());
        let inner = Arc::clone(&prev);
        std::panic::set_hook(Box::new(move |info| {
            let payload = info.payload();
            let msg = payload
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_default();
            if !msg.contains(WRITER_PANIC_SEAM_MSG) {
                inner(info);
            }
        }));
        // Restore by handing back the ORIGINAL hook: dropping our filtering hook
        // first releases its `Arc` clone, so `try_unwrap` recovers the Box.
        PanicHookGuard(Some(prev))
    };
    let (result, terminal, out) = run_writer_death_scenario("threaded_panic", N, None, true);
    drop(hook_guard);

    // (1) finalize surfaces the panicked-writer error (the join-Err mapping).
    match &result {
        Err(BagdError::Config(s)) => assert_eq!(
            s, "bagd writer thread panicked",
            "the join-Err must map to the exact Config panic message"
        ),
        other => panic!(
            "expected Err(BagdError::Config(\"bagd writer thread panicked\")), got {other:?}"
        ),
    }

    // (3) post-panic hand-offs are counted dropped_unwritten via the dead-writer
    // (Disconnected) branch.
    assert_eq!(
        terminal["state"].as_str(),
        Some("failed"),
        "the terminal status frame is 'failed'"
    );
    let dropped = terminal["dropped_unwritten"]
        .as_u64()
        .expect("dropped_unwritten u64");
    assert!(
        dropped >= 1,
        "post-panic hand-offs must be counted dropped_unwritten; got {dropped}"
    );
    assert_eq!(
        terminal["frames_lost"].as_u64(),
        Some(0),
        "deep queue: no tap-queue overflow"
    );

    // (2) the bag is un-finalized.
    let reader = BagReader::open(&out).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "a panicked-writer recording must NOT be finalized, got {completeness:?}"
    );
    cleanup(&out);
}

// ============================================================
// Frozen-timestamp-burst marker e2e.
//
// Reproduces the storm shape at the tap: a run of consecutive frames
// sharing ONE timestamp_ns (a gating-clock stall / catch-up burst). bagd must
// emit EXACTLY ONE marker onto the reserved __cerulion/nondeterminism channel,
// and that marker must be INVISIBLE to the replay reader (Principle #7 — replay
// skips all __cerulion/* channels).
// ============================================================

#[test]
fn frozen_timestamp_burst_emits_one_marker_and_replay_ignores_it() {
    // A deep-borrow (>= RECORDING_SUBSCRIBER_MAX_BORROWED = 22) tap so the
    // THREADED writer path (where the marker detector lives) engages, and a deep
    // receive buffer so the burst does not lap the tap (no frames_lost — we want
    // the FROZEN-TS signal, not a real gap).
    let mgr = make_manager(128);
    let topic = unique_topic("frozenburst");
    let out = unique_out("frozenburst");
    let ready = unique_out("frozenburst_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 22, 128, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    // The frozen instant (mirrors the forensics' t=33990241976). BURST frames
    // ALL carry it; the tail frames carry distinct timestamps so the run breaks
    // and the tracker proves it does not double-fire.
    const FROZEN_TS: u64 = 33_990_241_976;
    const BURST: u32 = 40; // > threshold (32); < the ~880 real storm depth.
    const TAIL: u32 = 3;
    let total = BURST + TAIL;
    for i in 0..BURST {
        let frame = build_frame(0x5A0A, i, FROZEN_TS, &[(i as u8) ^ 0x5A; 16]);
        pubr.publish_raw(&frame).expect("publish burst frame");
        // Let the writer drain periodically so the burst is spread across
        // batches (the tracker persists across batches — the real-run shape).
        if i % 7 == 6 {
            settle();
        }
    }
    // Tail: distinct timestamps break the run (and prove no second marker).
    for j in 0..TAIL {
        let seq = BURST + j;
        let frame = build_frame(0x5A0A, seq, FROZEN_TS + 1000 + j as u64, &[0x11; 16]);
        pubr.publish_raw(&frame).expect("publish tail frame");
    }

    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");

    // (1) EXACTLY ONE marker was emitted (writer-side truth).
    assert_eq!(
        summary.frozen_burst_markers, 1,
        "one frozen-burst marker per burst (summary)"
    );
    // No frame lost — the burst was a FROZEN-TS signal, not a real gap.
    assert_eq!(summary.frames_lost, 0, "deep buffer: no tap-queue overflow");
    assert_eq!(
        summary.messages, total as u64,
        "every data frame recorded (marker is on a reserved channel, not counted)"
    );

    let reader = BagReader::open(&out).expect("open bag");

    // (2) The marker record is on the __cerulion/nondeterminism channel, and
    //     its JSON payload is the self-describing burst descriptor (oracle on
    //     the parsed fields — never a self-compare).
    let markers = data_msgs_for(&reader, NONDETERMINISM_TOPIC);
    assert_eq!(
        markers.len(),
        1,
        "exactly one nondeterminism record (the burst marker)"
    );
    let v: serde_json::Value =
        serde_json::from_slice(&markers[0].data).expect("marker payload is JSON");
    assert_eq!(v["marker"], "frozen_timestamp_burst");
    assert_eq!(v["topic"], topic);
    assert_eq!(v["timestamp_ns"], FROZEN_TS);
    assert_eq!(
        v["run_len_at_mark"], FROZEN_BURST_MARKER_THRESHOLD,
        "the marker fires AT the threshold, so run_len_at_mark == threshold — a \
         FLOOR, never the true burst depth (the marker is not re-stamped at burst end)"
    );
    assert!(
        v.get("run_len").is_none(),
        "the over-claiming `run_len` key is gone (renamed to run_len_at_mark)"
    );
    assert_eq!(v["threshold"], FROZEN_BURST_MARKER_THRESHOLD);
    // The TRUE burst depth is derivable from the bag, NOT from the marker: count
    // the consecutive user frames on this topic that share the marker's frozen
    // `timestamp_ns` (their log_time). That count is the real depth (BURST = 40
    // here, > the run_len_at_mark floor of 32) — proving the true-depth derivation
    // path the payload doc points at.
    let frozen_depth = data_msgs_for(&reader, &topic)
        .iter()
        .filter(|m| m.log_time == FROZEN_TS)
        .count();
    assert_eq!(
        frozen_depth, BURST as usize,
        "true burst depth ({BURST}) is recoverable from frames sharing timestamp_ns, \
         and exceeds the run_len_at_mark floor ({})",
        FROZEN_BURST_MARKER_THRESHOLD
    );
    // The marker's log_time time-aligns with the burst.
    assert_eq!(
        markers[0].log_time, FROZEN_TS,
        "marker log_time == the frozen timestamp (time-aligns with the loss window)"
    );

    // (3) All user data frames are present + healthy; the marker did NOT enter
    //     the user-topic stream.
    let user_frames = data_frames_for(&reader, &topic);
    assert_eq!(
        user_frames.len(),
        total as usize,
        "all {total} user frames recorded, none displaced by the marker"
    );

    // (4) Principle #7 replay-tolerance PIN: the replay reader's USER-frame walk
    //     (reader.user_frames(), the exact seam replay compares against) SKIPS
    //     all __cerulion/* channels, so the marker is INVISIBLE to replay.
    let mut walk = reader.user_frames().expect("user_frames walk");
    let mut user_topics: Vec<String> = Vec::new();
    while let Some((cid, _span)) = walk.next_user_frame().expect("next_user_frame") {
        user_topics.push(walk.topic(cid).to_string());
    }
    assert!(
        !user_topics.iter().any(|t| t.starts_with("__cerulion/")),
        "the user-frame walk must exclude every reserved channel (incl. the marker)"
    );
    assert_eq!(
        user_topics.iter().filter(|t| *t == &topic).count(),
        total as usize,
        "the user-frame walk yields exactly the {total} data frames"
    );

    // (5) record_health.json (B1): the topic's health carries the reconciliation
    //     high-water marks and is HEALTHY (frames_lost 0, no baseline reset — the
    //     frozen burst alone is not a loss). Parsed as JSON (back-compat shape).
    let health_att = reader
        .attachment(RECORD_HEALTH_ATTACHMENT)
        .expect("attachment lookup")
        .expect("record_health.json present in a finalized bag");
    let health: serde_json::Value =
        serde_json::from_slice(&health_att.data).expect("record_health parses");
    let th = &health["topics"][&topic];
    assert_eq!(th["frames_recorded"].as_u64(), Some(total as u64));
    assert_eq!(
        th["frames_lost"].as_u64(),
        Some(0),
        "no loss on a frozen burst"
    );
    assert_eq!(th["first_seq"].as_u64(), Some(0), "B1 first_seq populated");
    assert_eq!(
        th["last_seq"].as_u64(),
        Some((total - 1) as u64),
        "B1 last_seq high-water mark populated"
    );
    // Healthy → the probe fields are OMITTED (additive/back-compat).
    assert!(
        th.get("baseline_resets_after_first").is_none(),
        "healthy tap omits baseline_resets_after_first: {th}"
    );
    assert!(
        th.get("gap_detection_disabled_reason").is_none(),
        "armed tap omits gap_detection_disabled_reason: {th}"
    );

    cleanup(&out);
}

// ============================================================
// SUSTAINED-RATE zero-loss: a LIVE recorder's drain
//   must not be rate-clamped below the topic's publish rate. A clamped drain
//   back-fills the tap's SHM queue until it drop-oldest-EVICTS — recorded-frame
//   LOSS (Principle #6) on a topic the recorder is nominally recording. The
//   target shapes sustain ~1 kHz (moveit2) to ~1.7 kHz (humanoid).
//
// THE MARGIN BELOW WAS WIDENED. Both halves were
// MEASURED against this head, not reasoned:
//
// (a) Single-fault isolation. `drive_loop`'s backlog-aware pacing alone
//     (collapse `made_progress => None` so `next_sleep` is always
//     `Some(WAIT_TICK)`) does not drop frames here (2.70 s, all 6000
//     frames recorded). Neither does one drain REQUEST per pass alone
//     (`break` after the first `drain_owned`): also all 6000 frames
//     recorded. The copy-at-drain loop is why: it runs `drain_owned`
//     until the queue reports EMPTY, so a 10 ms-paced pass still clears
//     everything that accumulated during the sleep, and the pacing
//     branch alone is not a throughput cap. Only the COMBINATION (one
//     request per pass AND an unconditional sleep) drops frames:
//     `Some(1227)` against a published `Some(6000)`. Isolating either
//     fault alone is a real coverage gap in the drain path and is NOT
//     closed here — see the residual note on the arm.
//
// (b) THE LOAD MARGIN, which is why this arm was flagged. The oracle is an
//     EXACT full count over a stream the producer emits regardless of how the
//     recorder is scheduled, so the arm inverts the moment the DRIVE LOOP is
//     starved for longer than the queue can cover — the mirror image of
//     an earlier flake, whose producer starvation made a "must lose frames" arm lose
//     nothing. That budget is `BUFFER / publish_rate` and nothing else, so it is
//     a function of QUEUE DEPTH (the one lever load cannot move) and the fix is
//     to raise it. MEASURED rather than derived, by dosing the DRIVE LOOP with a
//     single CONTIGUOUS stall through `fault_inject_tap_drain_gate` and bisecting
//     (a DUTY CYCLE is the wrong dose and was tried first: 50 ms of stall every
//     100 ms — 1.41 s cumulative — loses nothing, because the recorder catches
//     up between windows and the queue only ever has to hold ONE window):
//
//         BUFFER=1024, 6000 frames:   410 ms PASS,  610 ms FAIL (356 lost)
//         BUFFER=8192, 12000 frames: 3000 ms PASS, 4000 ms FAIL (388 lost)
//
//     i.e. the budget moved from ~0.46 s to ~3.7 s — 8x — and the 610 ms stall
//     that REDDENS the old geometry passes the new one outright. (The observed
//     publish rate is ~2.2 kHz, not the nominal 3 kHz: the 10 ms inter-batch
//     sleeps run long. Both boundaries match `BUFFER / observed rate`.) The
//     stream lengthens with the queue so a combined-fault regression still has
//     time to overflow it, and the two startup races are replaced by CONDITIONS
//     (below)
//     so none of that budget is spent before frame 0.
//
//     Scope: a rate comparison cannot be gated away the way that arm's
//     overflow could. "The drain keeps up with a live producer" IS a race by
//     construction; pacing the producer on the recorder's own progress would
//     make loss impossible and remove this arm's power to catch that
//     regression. So this stays a
//     margin, and the margin is now large, measured and written down.
// ============================================================

#[test]
fn sustained_rate_records_without_backlog_loss() {
    // Provision the topic at the recording borrow budget with a bounded SHM
    // queue depth (BUFFER). (That budget used to also SELECT the writer
    // thread; there is one write path now, so it is kept here only to model the
    // shipping `graph run --record` provisioning.)
    //
    // BUFFER IS THE WHOLE LOAD MARGIN, which is why it moved. It is the
    // number of frames the topic's own queue can hold while the recorder is not
    // draining, so `BUFFER / rate` is EXACTLY how long the drive loop may be
    // starved before this arm's exact oracle inverts through no fault of the
    // code. MEASURED by bisection against a single contiguous injected stall:
    // 1024 slots survived 410 ms and died at 610 ms; 8192 survives 3000 ms and
    // dies at 4000 ms. That is the one lever load cannot move — a deeper queue
    // is a deeper queue whoever the scheduler favours — which is why it is the
    // lever pulled instead of loosening the oracle.
    const BORROW: usize = cerulion_core::transport::RECORDING_SUBSCRIBER_MAX_BORROWED; // 22
    const BUFFER: usize = 8192;
    const SEED: usize = 1;
    // The stream must outlast a combined-fault regression's fill time, or
    // raising BUFFER would buy the margin by giving up the ability to catch
    // it: a deeper queue delays that regression's overflow too. 12 k frames
    // (~5.5 s of publishing) leaves it no way out — MEASURED at 8593 of
    // 12001 recorded, a 3408-frame shortfall (a re-run against the merged
    // tree recorded 8588, a 3413-frame shortfall: the exact figure is a
    // single observation of a
    // race-shaped oracle and moves by a handful of frames run to run, which is
    // why the arm asserts the FULL count rather than a threshold near it).
    const BURST: usize = 12_000;
    const PUBLISHED: usize = SEED + BURST;
    const BATCH: usize = 30; // ~3 kHz at the 10ms inter-batch sleep below

    let mgr = make_manager(BUFFER);
    let topic = unique_topic("sustained");
    let out = unique_out("sustained");
    let ready = unique_out("sustained_ready");

    // Publisher creates the service FIRST (borrow + deep buffer); the tap opens
    // it open-only and inherits the ceiling.
    let mut publisher = publisher_with_provisioning(&mgr, &topic, BORROW, BUFFER, 256);

    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    // Frequent flush so held budget is released every cycle (the fixed drain
    // relies on the flush to free room for the next pass); short schema_wait is
    // irrelevant here (a single tap learns + creates the writer on pass 1).
    cfg.flush_interval = Duration::from_millis(10);

    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());

    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );

    // START THE STREAM FROM A KNOWN STATE, on CONDITIONS rather than on
    // `settle()`. The ready file says the taps are ARMED, not that this tap has
    // drained anything — so a `settle()` here bets a fixed sleep on schema
    // learning and writer creation, and every millisecond that bet loses is
    // spent out of the 2.73 s margin above before frame 0 is even published.
    // One SEED frame plus a wait for the bag FILE closes both: the writer is
    // created only once a tap's schema is known, so the file appearing proves
    // the seed was drained AND the recorder is past setup.
    publisher
        .publish_raw(&build_frame(0x5605, 0, 0, &[0u8; 16]))
        .expect("publish seed frame");
    assert!(
        await_condition(LIVENESS_CEILING, || out.exists()),
        "the recorder must learn the seed's schema and create the bag before the burst"
    );

    // Sustained publish ABOVE the buggy cap. Distinct incrementing wire
    // sequences let the tap's gap detector count any drop-oldest eviction as
    // `frames_lost`.
    for i in SEED..PUBLISHED {
        let frame = build_frame(0x5605, i as u32, i as u64, &[(i & 0xff) as u8; 16]);
        publisher
            .publish_raw(&frame)
            .expect("publish sustained frame");
        if (i + 1) % BATCH == 0 {
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    // Let the tail fully drain, then stop.
    settle();
    settle();
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");

    // ZERO-LOSS oracle: every published frame is recorded. Under a
    // combined-fault regression the clamped drain back-fills the
    // 8192-deep queue mid-run and this collapses by thousands with
    // frames_lost > 0.
    assert_eq!(
        summary.per_topic.get(&topic),
        Some(&(PUBLISHED as u64)),
        "every published frame must be recorded (the live drain keeps up with the \
         {PUBLISHED}-frame sustained stream); a shortfall means drain throughput was \
         clamped and the SHM queue drop-oldest-evicted"
    );
    assert_eq!(
        summary.frames_lost, 0,
        "no wire-sequence gaps: the tap SHM queue never drop-oldest-evicted"
    );
    assert_eq!(
        summary.dropped_unwritten, 0,
        "the single tap learns its schema + creates the writer on pass 1 — no unwritten drops"
    );

    // Independently re-read the finalized bag: the recovered frame count matches.
    let reader = BagReader::open(&out).expect("open bag");
    let frames = data_frames_for(&reader, &topic);
    assert_eq!(
        frames.len(),
        PUBLISHED,
        "the finalized bag must contain every published frame"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// The CHUNK TIME FLOOR, pinned on BOTH sides of its threshold.
//
// A chunk now spans many drain cycles, and until it CLOSES none of it is on
// disk: the open chunk is the bag's bounded crash-loss unit. At 4 MiB that is
// ~1.4 s of the measured Go2 firehose, but minutes of a low-rate recording — so
// `CHUNK_TIME_FLOOR` closes one after ~1 s regardless of size. It must cost
// nothing at high rates, where the SIZE threshold is meant to fire first.
//
// Both arms read the bag WHILE THE RUN IS STILL GOING, which is the only way to
// tell a chunk that closed on its own from one `finalize` closed for it.
//
// LOAD DISCIPLINE, stated per assertion rather than claimed for the section
// (a deadline-shaped assertion is not load-invariant: an `elapsed <
// CHUNK_TIME_FLOOR` DEADLINE on the high-rate arm would be FALSE, since a
// runner slow enough to spend the floor merely PUBLISHING inverts a
// deadline no matter how correct the code is):
//
//   * every wait for CONTENT is a bounded wait, so load can only DELAY it;
//   * the low-rate arm's cadence CEILING divides by a wall measured from
//     outside the recorder, so load only loosens it;
//   * the high-rate arm attributes its close to SIZE by COUNTING chunks, which
//     is a function of the BYTES the test publishes and of nothing else. Load
//     cannot move it in either direction.
//
// No assertion here is a deadline.
// ============================================================

/// LOW RATE: a recording nowhere near `chunk_max_bytes` still lands on disk,
/// because the TIME floor closed its chunk — and closes it AT MOST once per
/// floor, not once per drain cycle.
///
/// Both bounds are load-bearing and neither can stand alone:
///
/// * the LOWER bound (content is readable mid-run) is what `CHUNK_TIME_FLOOR`
///   buys. Dropping `write_batch`'s `last_chunk_flush.elapsed() >=
///   CHUNK_TIME_FLOOR` arm makes this wait expire — nothing reaches disk until
///   finalize.
/// * the UPPER bound (the CADENCE) is what the RE-ANCHOR buys, and without it
///   the floor degenerates into exactly the regression it exists to
///   remove. `last_chunk_flush` is only ever re-armed by `open_chunk_bytes() ==
///   0`; delete that and `elapsed() >= CHUNK_TIME_FLOOR` is true forever after
///   the first second, so EVERY batch closes a chunk again: the
///   one-chunk-per-drain-cycle shape the bench measured at 360 chunks
///   against 4. The lower bound cannot see it (a run that flushes constantly
///   still lands content mid-run, sooner if anything).
///
/// The upper bound is arithmetic, not a timing guess: a time-triggered close
/// RE-ANCHORS the clock, so closes are >= `CHUNK_TIME_FLOOR` apart by
/// construction; the SIZE trigger is asserted below to be unreachable here; and
/// `finalize` closes the last one. The wall is measured from OUTSIDE the
/// recorder (before spawn, after join) so it strictly exceeds the recorder's
/// own lifetime — load can only LENGTHEN it, which only LOOSENS the bound. It
/// therefore cannot flake in the direction that matters.
///
/// **The CADENCE is closed from below too**, the half that was deferred at first.
/// The two bounds above are on different quantities: "content landed at all"
/// and "closes are not more often than a floor". Between them sits a whole
/// class nothing could see — a floor that still fires, just far too rarely. A
/// floor widened to 2x passes both today: content lands at ~2 s against a 5 s
/// budget, and half as many chunks is comfortably under a ceiling. That matters
/// because the open chunk is the bag's bounded CRASH-LOSS unit, and its
/// documented bound is one floor.
///
/// The missing side is a COUNT floor, derived from this test's own control flow
/// rather than from a wall:
///
///   * `landed` returns only AFTER the first time close, so the recorder has
///     been live for >= 1 floor at that point;
///   * the publisher then keeps streaming for a further `STREAM_FLOORS` floors
///     before `stop`;
///   * so closes at 1..=`STREAM_FLOORS` floors all fall strictly inside the
///     window, and `finalize` adds one more.
///
/// Hence `>= STREAM_FLOORS`, one close dropped for boundary safety. This is
/// load-SAFE in the way a wall-divided bound is not: load LENGTHENS the window
/// and can only ADD closes, never remove them. `STREAM_FLOORS` is 5 rather than
/// the 2 it needs to merely exist, because the bound has to separate correct
/// code from a 2x-widened floor with margin on BOTH sides. MEASURED on this
/// desk: correct code produces **6** against a floor of 5; a 2x-widened floor
/// lands `landed` at ~2 floors and then closes at 2/4/6 for **4**. That costs
/// ~3 s of test time and buys the only assertion in the repo that a chunk's age
/// is bounded from below as well as above.
#[test]
fn a_low_rate_recording_reaches_disk_on_the_chunk_time_floor() {
    /// Floors the ~10 Hz stream keeps running for AFTER the first close is
    /// observed.
    ///
    /// This was widened from 5 and SPLIT from the asserted minimum,
    /// which used to BE it. MEASURED: correct code closed 6 chunks against a
    /// required 5, and the doc's own record of the 2x-widened-floor variant is 4
    /// — a margin of exactly ONE on each side, on an assertion whose comment
    /// claimed "load can only ADD closes to it". That claim is FALSE in the
    /// direction that matters: a TIME close needs a BATCH to have been handed to
    /// the writer, so a floor window in which a starved drain hands over nothing
    /// closes nothing, and starvation REMOVES closes. Streaming for more floors
    /// than are demanded is what buys the slack, and it is nearly free in suite
    /// wall — this arm is not the critical path.
    const STREAM_FLOORS: u64 = 9;
    /// Closes DEMANDED, deliberately below [`STREAM_FLOORS`]. MEASURED at this
    /// window: correct code closes 10 and the 2x-widened-floor variant closes 6,
    /// so the arm carries THREE floors of slack against a starved drain (it had
    /// one) while still failing that variant. The variant-side margin is
    /// deliberately left where it was rather than split with the correct side —
    /// the defect being fixed is the LATENT RED, i.e. correct code
    /// failing under load, and buying variant margin here would spend exactly
    /// that.
    const MIN_CLOSES: u64 = 7;
    const _: () = assert!(MIN_CLOSES < STREAM_FLOORS);
    const BODY: usize = 16;
    let mgr = make_manager(16);
    let topic = unique_topic("timefloorlow");
    let out = unique_out("timefloorlow");
    let ready = unique_out("timefloorlow_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 4, 64, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(200);
    // The production 4 MiB: this recording cannot come within three orders of
    // magnitude of it, so the SIZE trigger is provably not what closes a chunk.
    cfg.chunk_max_bytes = cerulion_bag::DEFAULT_CHUNK_MAX_BYTES;
    // The cadence bound's wall. Started BEFORE the recorder exists, so it
    // strictly exceeds the drive loop's own lifetime (the clock the floor is
    // enforced against) — a strictly generous denominator.
    let run_started = Instant::now();
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    // ~10 Hz for as long as the assertion polls. A publisher thread keeps the
    // stream going while the oracle runs, so the floor is measured against a
    // LIVE recording rather than a quiesced one.
    let stop = Arc::new(AtomicBool::new(false));
    let stop_c = stop.clone();
    let pub_thread = std::thread::spawn(move || {
        let mut i = 0u32;
        while !stop_c.load(Ordering::Relaxed) {
            pubr.publish_raw(&build_frame(0x71F0, i, i as u64, &[(i as u8) + 1; BODY]))
                .expect("publish");
            i += 1;
            std::thread::sleep(Duration::from_millis(100));
        }
        i
    });

    // THE PIN: a chunk closes on the floor, so the bag has content long before
    // anything asks it to finalize. The budget is 5x the floor — generous
    // enough that a loaded runner cannot fail it, tight enough that "never
    // until finalize" cannot pass it.
    let landed = wait_until(Duration::from_secs(5), || {
        BagReader::open(&out).is_ok_and(|r| {
            r.recover_messages()
                .is_ok_and(|(msgs, _)| msgs.iter().any(|m| m.topic == topic))
        })
    });
    assert!(
        landed,
        "a low-rate recording must reach disk on the CHUNK TIME FLOOR — nothing was \
         readable while the run was still going"
    );

    // Keep the ~10 Hz stream running well past the first close, so BOTH cadence
    // assertions below have several floors to be wrong about. Without this the
    // run ends moments after the first chunk lands and a per-batch flusher, a
    // per-floor flusher and a once-in-a-while flusher are all within a chunk or
    // two of each other.
    std::thread::sleep(Duration::from_millis(
        STREAM_FLOORS * cerulion_bagd::CHUNK_TIME_FLOOR_MS,
    ));

    stop.store(true, Ordering::Relaxed);
    let published = pub_thread.join().expect("publisher thread");
    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    let wall = run_started.elapsed();

    // ANTI-TAUTOLOGY: prove the SIZE trigger could not have been what fired.
    // This is also the cadence bound's premise — with the size trigger provably
    // out of reach, every close but the last is a TIME close.
    assert!(
        summary.bytes < cerulion_bag::DEFAULT_CHUNK_MAX_BYTES as u64,
        "the whole recording ({} B) must sit under one chunk's size threshold, or the \
         size trigger — not the floor — is what closed the chunk",
        summary.bytes
    );
    assert_eq!(
        summary.messages, published as u64,
        "every published frame still reaches the bag"
    );

    // THE CADENCE PIN (the re-anchor's oracle). Time closes are >= one floor
    // apart, so at most `ceil(wall / floor)` of them fit; `finalize` closes the
    // open chunk for one more.
    let max_chunks = (wall.as_millis() as u64).div_ceil(cerulion_bagd::CHUNK_TIME_FLOOR_MS) + 1;
    assert!(
        summary.chunks <= max_chunks,
        "the time floor must bound the AGE of the open chunk, not fire on every batch: \
         {} chunks over a {wall:?} run, where a {} ms floor admits at most {max_chunks} \
         (a per-batch flusher is the one-chunk-per-drain-cycle regression)",
        summary.chunks,
        cerulion_bagd::CHUNK_TIME_FLOOR_MS
    );

    // THE CADENCE PIN, OTHER SIDE. Derived from this test's own fixed
    // window rather than from the wall, so a slow runner cannot move the
    // THRESHOLD. It CAN move the observed count, though, and downward: a time
    // close needs a batch handed to the writer, so a floor window a starved
    // drain spends handing over nothing closes nothing. Hence the demanded
    // minimum sits two floors BELOW the streaming window instead of
    // equalling it. See the constants for the measured values on both sides.
    let min_chunks = MIN_CLOSES;
    assert!(
        summary.chunks >= min_chunks,
        "the time floor must actually FIRE at its cadence, not merely eventually: {} chunks \
         over a run that streamed for at least {STREAM_FLOORS} floors past its first close, \
         where a {} ms floor owes at least {min_chunks} (a floor that fires far too rarely \
         still lands content and still passes a ceiling — it just leaves the open chunk, \
         the bag's crash-loss unit, older than its documented bound)",
        summary.chunks,
        cerulion_bagd::CHUNK_TIME_FLOOR_MS
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// HIGH RATE: the SIZE threshold still fires first, well inside the time floor.
///
/// Removing `write_message`'s size-triggered auto-flush leaves the
/// floor as the only trigger — the mid-run wait expires AND the chunk count
/// collapses to what one sub-second run's floor can produce.
///
/// The size attribution is a COUNT, not a deadline. This run publishes
/// `N * (32 + BODY)` bytes against a `CHUNK_CAP`-byte chunk, so a size-closing
/// writer must produce about `bytes / CHUNK_CAP` chunks; the time floor could
/// contribute at most one or two over the whole (sub-second, then shutdown)
/// run. The count is fixed by the bytes the test publishes, so a loaded runner
/// cannot move it — where the previous `elapsed < CHUNK_TIME_FLOOR` assertion
/// was a deadline load could invert outright.
#[test]
fn a_high_rate_recording_closes_its_chunk_on_size_well_inside_the_time_floor() {
    const BODY: usize = 256;
    const N: u32 = 64; // 64 * (32 + 256) B = 18 KiB, ~4.5x the chunk cap below.
    const CHUNK_CAP: usize = 4096;
    let mgr = make_manager(16);
    let topic = unique_topic("timefloorhigh");
    let out = unique_out("timefloorhigh");
    let ready = unique_out("timefloorhigh_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 4, 128, 512);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(200);
    cfg.chunk_max_bytes = CHUNK_CAP;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let started = Instant::now();
    for i in 0..N {
        pubr.publish_raw(&build_frame(0x71F1, i, i as u64, &[(i as u8) + 1; BODY]))
            .expect("publish");
    }

    // Content lands mid-run — a chunk closed on its own rather than at
    // `finalize`. A bounded wait for CONTENT, generous enough that load only
    // delays it. (This alone does NOT attribute the close to SIZE: the floor
    // would eventually land content too. The count below is the attribution.)
    let landed = wait_until(Duration::from_secs(5), || {
        BagReader::open(&out).is_ok_and(|r| {
            r.recover_messages()
                .is_ok_and(|(msgs, _)| msgs.iter().any(|m| m.topic == topic))
        })
    });
    assert!(
        landed,
        "a recording {}x the chunk cap must close a chunk mid-run (nothing readable after \
         {:?})",
        (N as usize * (32 + BODY)) / CHUNK_CAP,
        started.elapsed()
    );

    shutdown.store(true, Ordering::Relaxed);
    let summary = handle.join().expect("join").expect("run_bagd Ok");
    assert_eq!(summary.messages, N as u64, "every frame recorded");

    // THE SIZE ATTRIBUTION: a byte-derived FLOOR on the chunk count. The
    // recorded body is at least `N * BODY` bytes (the wire header adds more, and
    // MCAP message framing more still), so a writer closing on `CHUNK_CAP` owes
    // at least this many chunks. One allowance is subtracted for the final
    // partial chunk. The time floor could not have produced them: this whole run
    // is one publish loop plus a shutdown, far short of that many floors.
    let min_size_chunks = ((N as usize * BODY) / CHUNK_CAP) as u64 - 1;
    assert!(
        summary.chunks >= min_size_chunks,
        "the chunks must have closed on SIZE: {} B of payload against a {CHUNK_CAP} B chunk \
         owes at least {min_size_chunks} chunks, got {} (the time floor cannot fire that \
         often in one publish loop)",
        N as usize * BODY,
        summary.chunks
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// A RING lap does not cost the TAPS their held frames.
//
// The recorder's two halves fail independently: a trace ring can be lapped by
// its producer while the disk, the bag and every tap are perfectly healthy. The
// run must still END (the ring's records are irrecoverable and the bag stays
// un-finalized), but the frames the recorder has ALREADY CONSUMED off the
// iceoryx2 queues — which nothing else can recover — must still land.
//
// That needs the writer to stay WRITABLE across the lap. `failed` says "the run
// must stop"; `WriterHandle::writable` says "the writer can still write", and
// only a BAG write error clears it. Collapsing the two makes `do_write` refuse
// every later batch AND makes `flush_threaded` book the finalize-time hand-off
// as `dropped_unwritten` — for frames a reader can plainly see in the bag.
//
// The window is built deterministically with the writer STALL gate rather than
// raced: the stall sits at the top of `write_batch`, before the ring drain, so
// while it is engaged the drain keeps draining, the bounded drain→writer channel
// fills, and defer-not-drop leaves the remainder HELD. Lapping the ring
// during the stall means the lap is detected on release with frames still held.
// ============================================================

/// `let ring_only = false;` in `writer_thread_main::do_write` (i.e.
/// treating a lap as a bag write error) fails this — the held frames are
/// refused, counted `dropped_unwritten`, and absent from the bag.
#[test]
fn a_ring_lap_still_lets_the_held_frames_land() {
    const SEED: u32 = 1;
    const BURST: u32 = 40;
    let mgr = make_manager(16);
    let topic = unique_topic("laphold");
    let out = unique_out("laphold");
    let ready = unique_out("laphold_ready");

    // Deep receive queue: the ONLY thing under test is what happens to frames
    // the recorder is HOLDING, never tap-queue overflow.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 8, 256, 256);

    let ring_tag = unique_ring_tag("laphold");
    let mut owner = TraceRingOwner::create(&ring_tag, 2, 5, &["only"]).expect("ring create");
    let mut producer = owner.producer().expect("producer");

    let gate = Arc::new(WriterStallGate::default());
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.rings = vec![owner.name().to_string()];
    cfg.schema_wait = Duration::from_millis(200);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    // Seed: learns the schema and spawns the writer thread.
    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..SEED {
        let frame = build_frame(0x1AB0, i, i as u64, &[(i as u8) + 1; 32]);
        pubr.publish_raw(&frame).expect("publish seed");
        expect.push(frame);
    }
    assert!(
        wait_until(Duration::from_secs(3), || out.exists()),
        "the writer thread must have created the bag"
    );

    // STALL the writer, then burst: the drain keeps draining, the bounded
    // channel fills, and defer-not-drop leaves the remainder HELD drain-side.
    gate.engaged.store(true, Ordering::Relaxed);
    for i in SEED..SEED + BURST {
        let frame = build_frame(0x1AB0, i, i as u64, &[(i as u8) + 1; 32]);
        pubr.publish_raw(&frame).expect("publish burst");
        expect.push(frame);
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        wait_until(Duration::from_secs(3), || gate
            .entered
            .load(Ordering::Relaxed)),
        "the writer must be inside the stall"
    );

    // Lap the capacity-2 ring while the writer is held: the lap is detected on
    // release, with tap frames still held drain-side.
    for i in 0..64u64 {
        producer.push(&TraceRingRecord {
            step: i,
            fire_time_ns: i,
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: 1,
            reserved: 0,
        });
    }
    gate.engaged.store(false, Ordering::Relaxed);

    // The lap ends the run on its own.
    let result = handle.join().expect("join");
    match result {
        Err(BagdError::RingOverrun { records_lost, .. }) => {
            assert!(records_lost >= 1, "records_lost > 0, got {records_lost}");
        }
        other => panic!("expected RingOverrun, got {other:?}"),
    }

    // THE PIN: the ring's records are gone and the bag is un-finalized, but
    // every TAP frame the recorder consumed is in it, in order.
    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "an overran bag must NOT be Finalized, got {completeness:?}"
    );
    let frames: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(
        frames, expect,
        "every frame the recorder consumed off the queue must reach the truncated bag — \
         a lapped trace RING says nothing about the taps"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// An append failure must not destroy an EARLIER batch's frames.
//
// A chunk spans many drain cycles now, so the open chunk normally holds frames
// from batches the drain was already told were accepted. Discarding the
// pending chunk on an append failure only made sense when it held raw
// pointers into `held` samples about to be released. Now the arena owns
// copies and `write_message` never half-appends, so discarding only throws
// away frames that have nothing to do with the failure.
//
// The geometry is exact: a chunk cap far above the whole recording means NOTHING
// auto-flushes, so batch one's frames are still in the OPEN chunk when batch
// two's append fails.
// ============================================================

/// `writer.discard_pending_chunk()` in place of `writer.flush_chunk()`
/// on `write_batch`'s error path fails this — batch one vanishes and only the
/// re-written batch two survives.
#[test]
fn an_append_failure_keeps_the_open_chunks_earlier_batch() {
    const FIRST: u32 = 3;
    const SECOND: u32 = 4;
    const BODY: usize = 32;
    let mgr = make_manager(16);
    let topic = unique_topic("openchunk");
    let out = unique_out("openchunk");
    let ready = unique_out("openchunk_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 8, 64, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(200);
    cfg.flush_interval = Duration::from_millis(20);
    // Far above the whole recording: nothing may auto-flush, so batch one's
    // frames are still OPEN when batch two fails. (The time floor is 1 s and
    // the two bursts are 200 ms apart, so it cannot close one either.)
    cfg.chunk_max_bytes = cerulion_bag::DEFAULT_CHUNK_MAX_BYTES;
    // Fire mid-batch-two: FIRST + 2 messages accepted, then the failure.
    cfg.fault_inject_flush_error_after_messages = Some(FIRST as u64 + 2);
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut expect: Vec<Vec<u8>> = Vec::new();
    for i in 0..FIRST {
        let frame = build_frame(0x0C40, i, 10 + i as u64, &[(i as u8) + 1; BODY]);
        pubr.publish_raw(&frame).expect("publish batch one");
        expect.push(frame);
    }
    // Let batch one flush ON ITS OWN — it must be a SEPARATE batch, and its
    // frames must still be in the open chunk when batch two arrives.
    std::thread::sleep(Duration::from_millis(200));
    for i in FIRST..FIRST + SECOND {
        let frame = build_frame(0x0C40, i, 10 + i as u64, &[(i as u8) + 1; BODY]);
        pubr.publish_raw(&frame).expect("publish batch two");
        expect.push(frame);
    }

    let result = handle.join().expect("join");
    assert!(
        matches!(result, Err(BagdError::Bag(_))),
        "the injected append failure must propagate, got {result:?}"
    );

    // THE PIN: batch one is untouched by batch two's failure, and batch two's
    // own remainder was retried — every frame, exactly once, in order.
    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "a failed recording must NOT be finalized, got {completeness:?}"
    );
    let frames: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(
        frames, expect,
        "an append failure must close the open chunk, not discard it — an EARLIER \
         batch's frames are not the failing batch's to destroy"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// The other side of that recovery: when the CLOSE ITSELF fails.
///
/// `write_batch`'s error path has three frame-destroying branches, and until now
/// NONE of them had a test caller — the only fault seam faults an APPEND, which
/// leaves the arena writable, so `flush_chunk` always succeeded and the three
/// `... are LOST` branches were unreachable from any test in the repo. That is a
/// bad place to be unexercised: the FIRST of them destroys the open chunk, which
/// now holds frames from batches the drain was already told were
/// accepted (exactly what the sibling test above exists to protect).
///
/// It is reachable now through `BagWriter::fault_inject_flush_failures_for_test`
/// (bagd writes over the real `FileSink`, so a flush failure has no other
/// origin). The geometry is the sibling's, plus a one-shot flush failure:
///
///   1. batch one lands in the OPEN chunk (nothing auto-flushes);
///   2. batch two's append faults at its first message;
///   3. the recovery tries to CLOSE the open chunk — and that fails.
///
/// WHAT IS PINNED, stated as the trade it is rather than as a good outcome:
/// batch one is DESTROYED, deliberately (a chunk whose `writev` failed cannot be
/// made durable, so the code discards it rather than leaving it for a later
/// flush), and batch two is STILL SALVAGED into a fresh chunk. The loss is only
/// acceptable because the run then DIES with the bag un-finalized — no reader can
/// mistake this bag for a complete recording — and because the branch says so
/// out loud (`could not close the open chunk after a write error — its frames
/// are LOST`).
///
/// The log line itself is not asserted here: it is emitted from the WRITER
/// thread, which `#[traced_test]` cannot reach even with `run_bagd` inline (the
/// capture is scoped to the test's span and bagd spawns its writer beneath it).
/// The OUTCOME is the pin.
///
/// Dropping the branch's `discard_pending_chunk()` fails this — the
/// un-discarded chunk survives, the retry appends batch two to it, and the next
/// flush (which succeeds, the fault being one-shot) lands BOTH batches.
#[test]
fn a_failed_close_after_an_append_failure_still_salvages_the_failing_batch() {
    const FIRST: u32 = 3;
    const SECOND: u32 = 4;
    const BODY: usize = 32;
    let mgr = make_manager(16);
    let topic = unique_topic("closefail");
    let out = unique_out("closefail");
    let ready = unique_out("closefail_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 8, 64, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(200);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.chunk_max_bytes = cerulion_bag::DEFAULT_CHUNK_MAX_BYTES;
    // Fault batch two's FIRST message, so its whole burst is the remainder the
    // salvage retry has to re-write.
    cfg.fault_inject_flush_error_after_messages = Some(FIRST as u64);
    // ONE flush failure: the recovery's close. The retry's own close then
    // succeeds, which is what makes "batch two survived" observable at all.
    cfg.fault_inject_flush_chunk_failures = 1;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    for i in 0..FIRST {
        pubr.publish_raw(&build_frame(
            0x0C41,
            i,
            10 + i as u64,
            &[(i as u8) + 1; BODY],
        ))
        .expect("publish batch one");
    }
    // Batch one must be its OWN batch, and still OPEN when batch two arrives.
    std::thread::sleep(Duration::from_millis(200));
    let mut batch_two: Vec<Vec<u8>> = Vec::new();
    for i in FIRST..FIRST + SECOND {
        let frame = build_frame(0x0C41, i, 10 + i as u64, &[(i as u8) + 1; BODY]);
        pubr.publish_raw(&frame).expect("publish batch two");
        batch_two.push(frame);
    }

    let result = handle.join().expect("join");
    assert!(
        matches!(result, Err(BagdError::Bag(_))),
        "the injected append failure must still propagate, got {result:?}"
    );

    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "the loss is only tolerable because the bag is NOT finalized, got {completeness:?}"
    );
    let frames: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(
        frames, batch_two,
        "a chunk whose close FAILED is discarded (batch one is gone, deliberately and \
         loudly), but the failing batch is still retried into a fresh chunk — the \
         recovery must not lose the frames it CAN still write"
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

// ============================================================
// The other TWO frame-destroying branches of `write_batch`'s error
// path, which were known to be reachable by nobody and left that way.
//
// The recovery closes the open chunk, then retries the failing batch's
// un-accepted remainder into a fresh one. THREE things can go wrong and each is
// a different `... are LOST` branch:
//
//   1. the CLOSE fails                     — pinned by the sibling above;
//   2. the retry SUCCEEDS, its close fails — this section, arm one;
//   3. the retry ITSELF fails              — this section, arm two.
//
// Neither was reachable with the original seam: the flush fault is a
// bare COUNT and the error path flushes twice in a row, so it could only ever
// fail the FIRST close. `fault_inject_flush_chunk_skip` exists for exactly
// these two shapes and for nothing else — arm one fails the SALVAGE close, arm
// two fails the auto-flush the retry reaches from inside `write_message`.
//
// WHAT THE ORACLE IS, and why it is not the log line. Both branches emit an
// `error!` from the WRITER thread, which `#[traced_test]` cannot reach (the
// capture is scoped to the test's span and bagd spawns its writer beneath it —
// the sibling above says the same). So the pin is the BAG: batch one, the
// durable prefix the recovery closed, must be present exactly once and in
// order; the failing batch must be absent; the bag must not be finalized.
//
// AND THAT IS ALSO THE PRECONDITION. Both arms are configured so that a seam
// which did NOT reach its branch leaves the failing batch IN the bag:
//
//   * arm one — if the skip were ignored, the FIRST close fails instead and
//     batch one is destroyed (the batch-one assertion fires); if the skip
//     swallowed the failure, the salvage close succeeds and batch two lands
//     (the batch-two assertion fires);
//   * arm two — if the retry's auto-flush did not fail, the retry completes and
//     batch two lands, exactly as the earlier salvage test asserts; and if
//     the branch kept its arena, the writer's shutdown close lands the partial
//     prefix it had already appended.
//
// So neither arm can pass vacuously: an unreached branch fails it loudly rather
// than quietly agreeing.
//
// The two arms produce the SAME bag, deliberately — the recorder's promise is
// about frames, and both branches keep the same one. What tells them apart is
// the CONFIGURATION, which is why they are two tests and not one.
// ============================================================

/// ARM ONE — branch 2: the salvage retry writes, and its own close FAILS.
///
/// Geometry: batch one lands in the open chunk; batch two's append faults at
/// its first message; the recovery CLOSES batch one (the skip lets this one
/// through, so the durable prefix survives); the retry re-appends batch two
/// into a fresh chunk; that chunk's close fails, and its frames go.
///
/// Dropping `fault_inject_flush_chunk_skip` from the writer wiring
/// (so the count fails the FIRST close) fails this — batch one is destroyed and
/// batch two survives, i.e. it degenerates into the sibling test above.
#[test]
fn a_salvaged_remainder_that_cannot_be_flushed_is_lost_while_the_prefix_survives() {
    const FIRST: u32 = 3;
    const SECOND: u32 = 4;
    const BODY: usize = 32;
    let mgr = make_manager(16);
    let topic = unique_topic("salvflush");
    let out = unique_out("salvflush");
    let ready = unique_out("salvflush_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 8, 64, 256);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(200);
    cfg.flush_interval = Duration::from_millis(20);
    // The production cap: nothing here comes near it, so no chunk closes on
    // SIZE and the only content-bearing flushes are the recovery's two.
    cfg.chunk_max_bytes = cerulion_bag::DEFAULT_CHUNK_MAX_BYTES;
    cfg.fault_inject_flush_error_after_messages = Some(FIRST as u64);
    // Let the recovery's CLOSE through; fail the SALVAGE close behind it.
    cfg.fault_inject_flush_chunk_skip = 1;
    cfg.fault_inject_flush_chunk_failures = 1;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut batch_one: Vec<Vec<u8>> = Vec::new();
    for i in 0..FIRST {
        let frame = build_frame(0x0C42, i, 10 + i as u64, &[(i as u8) + 1; BODY]);
        pubr.publish_raw(&frame).expect("publish batch one");
        batch_one.push(frame);
    }
    // Batch one must be its OWN batch, and still OPEN when batch two arrives.
    std::thread::sleep(Duration::from_millis(200));
    for i in FIRST..FIRST + SECOND {
        pubr.publish_raw(&build_frame(
            0x0C42,
            i,
            10 + i as u64,
            &[(i as u8) + 1; BODY],
        ))
        .expect("publish batch two");
    }

    let result = handle.join().expect("join");
    assert!(
        matches!(result, Err(BagdError::Bag(_))),
        "the injected append failure must still propagate, got {result:?}"
    );

    assert_prefix_survived_and_batch_two_lost(
        &out,
        &topic,
        &batch_one,
        "the recovery CLOSED the durable prefix before retrying, so a salvage close that \
         fails may only cost the frames IT was carrying — batch one is not the salvage's \
         to destroy (if batch one is missing the skip was ignored and the FIRST close \
         failed; if batch two is present the salvage close never failed at all)",
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// ARM TWO — branch 3: the salvage retry PARTIALLY writes, then FAILS.
///
/// The first cut of this arm reached the branch and could not see what it does.
/// It kept the append fault ARMED across the retry, so the retry re-faulted at
/// exactly the position it had resumed from and appended NOTHING — the pending
/// chunk was EMPTY at `discard_pending_chunk()`, making the discard a no-op no
/// bag-level oracle could distinguish. That is STRUCTURAL, not a choice of
/// constants: `skip` is derived from what the writer ACCEPTED, so a
/// position-keyed fault always fires at the first un-skipped message.
///
/// The retry therefore fails a DIFFERENT way — the way a failing disk actually
/// produces it. `write_message` appends to the arena and THEN auto-flushes at
/// `chunk_max_bytes`, so a flush failure mid-retry returns `Err` with the
/// already-appended messages sitting in the arena:
///
///   1. batch one (3 SMALL frames) lands in the open chunk, well under the cap;
///   2. batch two's append faults at its FIRST message (one-shot, so the retry
///      is free to proceed);
///   3. the recovery CLOSES batch one — the skip lets this one through;
///   4. the retry re-appends batch two's LARGE frames; the FIRST already crosses
///      the cap (`BIG_BODY >= CHUNK_CAP`), so its auto-flush is the next
///      content-bearing flush and it FAILS — whatever way the drain happened to
///      split batch two.
///
/// So the retry leaves a NON-EMPTY pending chunk, and the discard is the only
/// thing standing between those frames and the bag — the writer's shutdown
/// `close_open_chunk()` flushes a pending chunk even after a latched error.
///
/// Dropping `discard_pending_chunk()` from the salvage-retry-failed
/// branch fails this — the shutdown close lands the retry's partial prefix and
/// batch two is no longer absent.
#[test]
fn a_salvage_retry_that_fails_partway_discards_what_it_had_written() {
    const FIRST: u32 = 3;
    const SECOND: u32 = 4;
    // Batch one is small so it cannot approach the cap on its own; batch two is
    // LARGE so its second frame crosses it during the retry.
    const SMALL_BODY: usize = 32;
    // >= CHUNK_CAP on its own, so ANY non-empty retry auto-flushes on its FIRST
    // frame. At 2048 the arm depended on the retry carrying at least two frames:
    // a drain that split batch two 1+3 would leave the retry under the cap, the
    // SALVAGE CLOSE would fail instead, and the test would exercise branch 2
    // while staying green (both branches discard, and the bag is identical). The
    // branch pin is now interleaving-independent.
    const BIG_BODY: usize = 4096;
    const CHUNK_CAP: usize = 4096;
    let mgr = make_manager(16);
    let topic = unique_topic("salvretry");
    let out = unique_out("salvretry");
    let ready = unique_out("salvretry_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 8, 64, (BIG_BODY + 128) as u32);

    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(200);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.chunk_max_bytes = CHUNK_CAP;
    cfg.fault_inject_flush_error_after_messages = Some(FIRST as u64);
    // Let the recovery CLOSE through; fail the next content-bearing flush,
    // which the retry reaches from INSIDE `write_message`'s auto-flush.
    cfg.fault_inject_flush_chunk_skip = 1;
    cfg.fault_inject_flush_chunk_failures = 1;
    let handle = spawn_bagd(mgr.clone(), cfg, shutdown.clone());
    // This run is expected to SELF-TERMINATE on its injected fault, so
    // nothing in this arm ever stores the shutdown flag and the `join()` below is
    // unconditional. A regressed fault would therefore WEDGE the suite instead of
    // failing the `match` written to catch it; the watchdog turns that back into
    // an attributable red. See [`shutdown_watchdog`].
    let _watchdog = shutdown_watchdog(&shutdown, LIVENESS_CEILING);
    assert!(
        wait_for_file(&ready, LIVENESS_CEILING),
        "ready-file never appeared"
    );
    settle();

    let mut batch_one: Vec<Vec<u8>> = Vec::new();
    for i in 0..FIRST {
        let frame = build_frame(0x0C43, i, 10 + i as u64, &[(i as u8) + 1; SMALL_BODY]);
        pubr.publish_raw(&frame).expect("publish batch one");
        batch_one.push(frame);
    }
    // Batch one must be its OWN batch, and still OPEN when batch two arrives.
    std::thread::sleep(Duration::from_millis(200));
    for i in FIRST..FIRST + SECOND {
        pubr.publish_raw(&build_frame(
            0x0C43,
            i,
            10 + i as u64,
            &[(i as u8) + 1; BIG_BODY],
        ))
        .expect("publish batch two");
    }

    let result = handle.join().expect("join");
    assert!(
        matches!(result, Err(BagdError::Bag(_))),
        "the failed retry must propagate the ORIGINAL write error, got {result:?}"
    );

    assert_prefix_survived_and_batch_two_lost(
        &out,
        &topic,
        &batch_one,
        "a retry that fails partway must DISCARD what it had already appended — those \
         frames are the failing batch's own, and the writer still closes its open chunk \
         at shutdown, so an un-discarded arena would land them in a bag whose run FAILED \
         (any batch-two frame present means the salvage branch kept its arena)",
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}

/// The shared oracle for both error-path arms: the durable prefix is in the bag
/// EXACTLY ONCE and in order, the failing batch is absent entirely (no partial
/// write, no duplicate), and the bag is NOT finalized — which is the only thing
/// that makes the loss tolerable, since no reader can mistake it for complete.
fn assert_prefix_survived_and_batch_two_lost(
    out: &std::path::Path,
    topic: &str,
    batch_one: &[Vec<u8>],
    why: &str,
) {
    let reader = BagReader::open(out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        !completeness.is_finalized(),
        "the loss is only tolerable because the bag is NOT finalized, got {completeness:?}"
    );
    let frames: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(frames, batch_one, "{why}");
}

// ============================================================
// STAGING FULL is DROP-AND-COUNT, never block.
//
// The one named policy for the batching writer: when the recorder's
// own staging budget is spent under a stalled disk, the drain STOPS for that
// topic. It must never block (recorder slowness must not pressure the live
// plane) and never evict (eviction is retired), so the frames pile up in the
// TOPIC's own queue and whatever that queue drops is counted, per topic, as
// `frames_lost`.
//
// Built with the writer STALL gate, which is what a stalled disk looks like
// from the drain's side: the writer never takes a batch, so `held` only grows.
// ============================================================

/// Neutralising `drain_taps`' `held.byte_len() >=
/// RECORDING_TAP_STAGING_MAX_BYTES` arm fails this — the drain keeps copying
/// into unbounded recorder memory, so the topic's queue never overflows, no
/// warn fires and `staging_full_passes` stays 0.
#[test]
#[tracing_test::traced_test]
fn a_stalled_writer_stops_the_drain_at_the_staging_budget_and_counts_it() {
    // 64 KiB bodies: 4 MiB of staging is ~64 frames, so the budget is reachable
    // in a burst rather than a benchmark.
    const BODY: usize = 64 * 1024;
    const QUEUE_DEPTH: usize = 32;
    const BURST: usize = 200;
    let mgr = make_manager(16);
    let topic = unique_topic("stagefull");
    let out = unique_out("stagefull");
    let ready = unique_out("stagefull_ready");

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 4, QUEUE_DEPTH, (BODY + 64) as u32);

    let gate = Arc::new(WriterStallGate::default());
    let shutdown = Arc::new(AtomicBool::new(false));
    let mut cfg = test_cfg(out.clone(), vec![TapSpec::attach(&topic)], ready.clone());
    cfg.schema_wait = Duration::from_millis(200);
    cfg.flush_interval = Duration::from_millis(20);
    cfg.fault_inject_writer_stall_gate = Some(gate.clone());

    // `run_bagd` runs on THIS thread so the drain-side STAGING FULL warn lands
    // inside `#[traced_test]`'s captured span (a worker-thread warn escapes it).
    let ready_c = ready.clone();
    let shutdown_c = shutdown.clone();
    let gate_c = gate.clone();
    let stimulus = std::thread::spawn(move || {
        // This closure's shutdown store is its LAST statement, so any
        // assertion below would panic BEFORE it and leave `run_bagd` looping on the
        // test thread forever — a wedge, not a red. See [`ShutdownOnDrop`].
        let _stop = ShutdownOnDrop(shutdown_c.clone());
        assert!(
            wait_for_file(&ready_c, LIVENESS_CEILING),
            "ready-file never appeared"
        );
        settle();
        // Seed: learns the schema so the writer thread exists to be stalled.
        pubr.publish_raw(&build_frame(0x57A6, 0, 0, &[1u8; 16]))
            .expect("publish seed");
        settle();
        settle();
        // STALL the disk, then burst far past the staging budget.
        gate_c.engaged.store(true, Ordering::Relaxed);
        let body = vec![0xA5u8; BODY];
        for i in 1..=BURST {
            pubr.publish_raw(&build_frame(0x57A6, i as u32, i as u64, &body))
                .expect("publish burst");
            std::thread::sleep(Duration::from_millis(2));
        }
        std::thread::sleep(Duration::from_millis(200));
        gate_c.engaged.store(false, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(400));
        shutdown_c.store(true, Ordering::Relaxed);
    });

    let started = Instant::now();
    let summary = run_bagd(mgr.clone(), cfg, shutdown).expect("run_bagd Ok");
    let wall = started.elapsed();
    stimulus.join().expect("stimulus thread");

    // (1) NEVER BLOCK: the recorder ran to a clean shutdown. The stall is
    // released by the stimulus, so a drain that BLOCKED on the staging budget
    // would still finish — what a block would show up as is a run that took
    // the stall's whole length to make progress. The generous ceiling is a
    // liveness backstop, not the pin.
    //
    // It was raised from 30 s, because a wall its own comment calls "not
    // the pin" must not be the thing that reddens the arm — and this one was.
    // MEASURED: under `taskpolicy -b` plus 33 CPU spinners (an ~85x-slowdown
    // regime) this arm's ~1.5 s healthy path took 39.7 s and failed HERE, at
    // line (1), with both real pins below intact. The two pins are what
    // discriminate a blocked drain: it would leave `staging_full_passes` at 0
    // and print no STAGING FULL warn, neither of which is a function of the
    // wall. So the ceiling is now sized like the backstop it claims to be.
    assert!(
        wall < LIVENESS_CEILING * 10,
        "the recorder must never wedge on a full staging budget (took {wall:?})"
    );

    // (2) THE PIN: the drain STOPPED for this topic and said so.
    assert!(
        logs_contain("bagd STAGING FULL"),
        "a stalled writer must drive the tap to its staging budget and warn"
    );
    let health = summary
        .record_health
        .topics
        .get(&topic)
        .expect("topic health entry");
    assert!(
        health.staging_full_passes > 0,
        "the stop must be COUNTED, not only logged: {health:?}"
    );

    // (3) NEVER EVICT: the recorder destroyed nothing of its own. Whatever was
    // lost was lost in the TOPIC's queue, and is attributed there.
    assert_eq!(
        summary.dropped_unwritten, 0,
        "the recorder never evicts its own staged frames"
    );
    assert_eq!(
        health.frames_recorded + health.frames_lost,
        BURST as u64 + 1,
        "conservation: recorded {} + lost {} == the whole burst",
        health.frames_recorded,
        health.frames_lost
    );

    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
}
