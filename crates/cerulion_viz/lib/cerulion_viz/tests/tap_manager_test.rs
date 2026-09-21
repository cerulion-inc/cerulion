// SPDX-License-Identifier: AGPL-3.0-only
//! The `TapManager` runtime attach/detach + drain→dispatch e2e
//! over REAL iceoryx2 (per-test isolated SHM root — parallel-safe, no `#[serial]`).
//!
//! Every frame is a HAND-BUILT `geometry_msgs/Vector3` wire frame published by a
//! REAL producer (Principle #13: no fake data) — so the tap output is checked
//! BYTE-EXACT against the exact bytes published (never a self-compare), and the
//! worker render is checked against the known `Vector3 → three Scalar plots`
//! oracle (`sink_dispatch_test::vector3_infers_to_three_scalars`).
//!
//! The `VizLogWorker` owns the memory `RecordingStream` DIRECTLY (not the
//! process-global `crate::stream`), so these tests never touch the global stream
//! and stay isolated; num_msgs is measured as a DELTA from a post-setup baseline,
//! robust to the process-global scene-setup once-guards.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::codegen::FrameWalker;
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::SinkState;
use cerulion_viz::tap_manager::{AttachOutcome, TapManager, WakeMode};
use cerulion_viz::worker::{InputFrames, VizLogWorker};
use native_ros2_messages::geometry_msgs::Vector3;
use tracing_test::traced_test;

/// The maximum frames a single `poll` drains per tap (well above any test's
/// publish count — every test drains its whole queue in one poll).
const POLL_MAX: usize = 1024;

/// Build a full `geometry_msgs/Vector3` wire frame (3 f64, fixed-only, no
/// variable fields → an empty offset table). Byte-exact + deterministic, so it
/// doubles as the tap-output oracle.
fn build_vector3_frame(seq: u32, ts: u64, v: [f64; 3]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    for x in v {
        payload.extend_from_slice(&x.to_le_bytes());
    }
    let header = WireHeader {
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        // No variable fields → the (empty) offset table sits right past the fixed
        // section.
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A fresh isolated transport (per-test SHM root → parallel-safe).
fn isolated_transport(node_name: &str) -> Arc<TransportManager> {
    let clock = Arc::new(VirtualClock::new());
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node_name.to_string(),
            clock,
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated transport")
}

/// A fresh in-memory Rerun sink; returns the stream, a clone for flushing (the
/// worker MOVES its copy), and the storage handle for `num_msgs`.
fn memory_sink(
    recording_id: &str,
) -> (
    rerun::RecordingStream,
    rerun::RecordingStream,
    rerun::sink::MemorySinkStorage,
) {
    let (rec, storage) = rerun::RecordingStreamBuilder::new("tap_manager")
        .recording_id(recording_id)
        .memory()
        .expect("memory sink");
    let flush = rec.clone();
    (rec, flush, storage)
}

// ---------------------------------------------------------------------------
// Test 1 — attach a live topic, poll, render (byte-exact tap oracle + render).
// ---------------------------------------------------------------------------

#[test]
fn attach_live_topic_drains_byte_exact_and_renders() {
    let mgr = isolated_transport("attach_live");
    let topic = "/perception/velocity";

    // The service must EXIST before the open-only tap attaches, so create the
    // producer first — but publish NOTHING yet (a data-only tap has no history:
    // frames before attach would drop). Then attach, THEN publish.
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("producer attaches");

    let mut tm = TapManager::new();
    let outcome = tm
        .attach(&mgr, topic, None, WakeMode::Timer)
        .expect("attach live topic");
    // The route key is the WHOLE topic (leading/trailing `/` trimmed),
    // not its last segment — that is what makes the render entity per-topic
    // unique.
    assert_eq!(
        outcome,
        AttachOutcome::Attached {
            topic: topic.to_string(),
            route_key: "perception/velocity".to_string(),
        }
    );
    assert!(tm.contains(topic));
    assert_eq!(tm.len(), 1);

    // Publish THREE distinct hand-built frames (the drain-all + order oracle).
    let oracle: Vec<Vec<u8>> = (0..3)
        .map(|i| build_vector3_frame(i as u32, 1_000 * (i as u64 + 1), [i as f64, 2.0, 3.0]))
        .collect();
    for frame in &oracle {
        publisher.publish_raw(frame).expect("publish_raw Vector3");
    }

    // Worker owns the memory sink + the builtin walker (knows Vector3).
    let (rec, flush, storage) = memory_sink("attach_live");
    let mut worker =
        VizLogWorker::spawn(rec, builtin_walker(), SinkState::new()).expect("spawn worker");
    // Baseline AFTER the worker's scene setup (once-guards) is flushed.
    worker.sync();
    flush.flush_blocking().expect("flush baseline");
    let baseline = storage.num_msgs();

    // Poll drains ALL three queued frames, keyed by the route key, BYTE-EXACT and
    // IN ORDER (a latest-only read would keep only the last).
    let batch = tm.poll(POLL_MAX);
    assert_eq!(batch.len(), 1, "one attached tap yielded frames");
    assert_eq!(
        batch[0].name, "perception/velocity",
        "keyed by the route key (the whole topic)"
    );
    assert_eq!(
        batch[0].frames, oracle,
        "the tap drained EXACTLY the published frames, in order (byte-exact)"
    );

    // Hand the batch to the worker → each Vector3 shape-infers to three Scalar
    // plots. num_msgs counts CHUNKS; Rerun's micro-batcher compacts the three
    // frames' same-entity rows within one flush into three chunks (x/y/z) — the
    // tap oracle above is what proves all three frames flowed; this proves they
    // RENDERED (were not unknown-hash-skipped).
    worker.try_enqueue(batch);
    worker.sync();
    flush.flush_blocking().expect("flush render");
    assert_eq!(
        storage.num_msgs() - baseline,
        3,
        "a Vector3 renders as three Scalar plots (x/y/z)"
    );

    drop(worker);
}

// ---------------------------------------------------------------------------
// Test — attach is idempotent (no second tap, scarce introspection slots).
// ---------------------------------------------------------------------------

#[test]
fn attach_twice_opens_exactly_one_tap() {
    let mgr = isolated_transport("idempotent");
    let topic = "/robot/odom";
    let _publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("producer attaches");

    let mut tm = TapManager::new();
    let first = tm
        .attach(&mgr, topic, None, WakeMode::Timer)
        .expect("first attach");
    assert!(matches!(first, AttachOutcome::Attached { .. }));

    // The second attach opens NO second tap (idempotent) — the introspection
    // slot budget is scarce.
    let second = tm
        .attach(&mgr, topic, None, WakeMode::Timer)
        .expect("second attach");
    assert_eq!(
        second,
        AttachOutcome::AlreadyAttached {
            topic: topic.to_string(),
        }
    );
    assert_eq!(tm.len(), 1, "still exactly one tap");
    assert_eq!(tm.list(), vec![topic.to_string()]);
}

// ---------------------------------------------------------------------------
// Test 2 — detach frees the tap (a fresh tap on the same topic then succeeds).
// ---------------------------------------------------------------------------

#[test]
fn detach_frees_the_introspection_slot() {
    let mgr = isolated_transport("detach_frees");
    // A single-subscriber-slot topic: while the TapManager holds its tap, no
    // other tap can attach; after detach, the slot is free again.
    let topic = "/lidar/points";
    let mut config = mgr.default_topic_config();
    config.max_subscribers = Some(1);
    let _publisher = mgr
        .create_publisher_with_topic_config(topic, MaxSliceLen::const_new(1 << 16), 0, config)
        .expect("producer attaches");

    let mut tm = TapManager::new();
    tm.attach(&mgr, topic, None, WakeMode::Timer)
        .expect("attach");
    // With the only slot taken, a fresh raw tap is refused.
    assert!(
        mgr.create_data_only_subscriber(topic).is_err(),
        "the single subscriber slot is taken by the TapManager tap"
    );

    // Detach releases the slot.
    assert!(tm.detach(topic), "detach removed the tap");
    assert!(!tm.contains(topic));
    assert!(tm.is_empty());

    // A fresh tap on the SAME topic now succeeds — the slot was released (so a
    // re-created producer service is reachable again).
    mgr.create_data_only_subscriber(topic)
        .expect("a fresh tap attaches after detach freed the slot");

    // Detaching an unknown topic is a no-op, not an error.
    assert!(!tm.detach("/never/attached"));
}

// ---------------------------------------------------------------------------
// Test 3 — attach a missing topic surfaces the actionable error (no phantom).
// ---------------------------------------------------------------------------

#[test]
fn attach_missing_topic_surfaces_actionable_error() {
    let mgr = isolated_transport("missing");
    let mut tm = TapManager::new();

    // No producer ever created this topic's service → the open-only tap fails
    // LOUDLY (not a silent skip).
    let err = tm
        .attach(&mgr, "/nonexistent/topic", None, WakeMode::Timer)
        .expect_err("attaching a missing topic must error");
    let msg = err.to_string();
    assert!(
        msg.contains("does not exist"),
        "the actionable 'topic does not exist' error must surface, got: {msg}"
    );
    // The failed attach left NO phantom entry — the topic stays re-attachable.
    assert!(!tm.contains("/nonexistent/topic"));
    assert!(tm.is_empty());
}

// ---------------------------------------------------------------------------
// Test 4 — slot exhaustion surfaces the no-free-slot error (no panic).
// ---------------------------------------------------------------------------

#[test]
fn slot_exhaustion_surfaces_no_free_slot_error_no_panic() {
    let mgr = isolated_transport("slot_exhaust");
    let topic = "/camera/image";
    // Provision the topic with EXACTLY the introspection headroom's worth of
    // subscriber slots, then fill them all with raw taps so the TapManager
    // attach is one too many — refused. DERIVED from the production constant
    // (a literal `4` went on passing after the liveness observer raised the headroom to 5,
    // while quietly no longer modelling the headroom it claims to mirror).
    const SLOTS: usize = cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM;
    let mut config = mgr.default_topic_config();
    config.max_subscribers = Some(SLOTS);
    let _publisher = mgr
        .create_publisher_with_topic_config(topic, MaxSliceLen::const_new(1 << 16), 0, config)
        .expect("producer attaches");

    // Hold every raw tap ALIVE so they keep their slots.
    let _held: Vec<_> = (0..SLOTS)
        .map(|_| {
            mgr.create_data_only_subscriber(topic)
                .expect("fill an introspection slot")
        })
        .collect();

    let mut tm = TapManager::new();
    let err = tm
        .attach(&mgr, topic, None, WakeMode::Timer)
        .expect_err("one tap past the provisioned slots must be refused, not panic");
    let msg = err.to_string();
    assert!(
        msg.contains("subscriber slots are attached"),
        "the actionable 'no free introspection slot' error must surface, got: {msg}"
    );
    // The failed attach left NO phantom entry.
    assert!(!tm.contains(topic));
    assert!(tm.is_empty());
}

// ---------------------------------------------------------------------------
// Test 6 — SwapWalker: an unknown-hash topic renders ONLY after the swap.
// ---------------------------------------------------------------------------

#[test]
fn swap_walker_makes_an_unknown_hash_topic_render() {
    // Worker starts with an EMPTY walker (knows no schema at all), so a Vector3
    // frame is unknown-hash → NOT rendered. This needs no transport — it pins the
    // worker's SwapWalker handling directly.
    let (empty_walker, _warn) = FrameWalker::new(Vec::new());
    let (rec, flush, storage) = memory_sink("swap_walker");
    let mut worker =
        VizLogWorker::spawn(rec, empty_walker, SinkState::new()).expect("spawn worker");

    let frame = build_vector3_frame(0, 1_000, [1.0, 2.0, 3.0]);
    let one_frame = || {
        vec![InputFrames {
            name: "velocity".to_string(),
            frames: vec![frame.clone()],
        }]
    };

    // Baseline after scene setup.
    worker.sync();
    flush.flush_blocking().expect("flush baseline");
    let baseline = storage.num_msgs();

    // BEFORE the swap: the empty walker cannot decode the hash → nothing renders.
    worker.try_enqueue(one_frame());
    worker.sync();
    flush.flush_blocking().expect("flush pre-swap");
    assert_eq!(
        storage.num_msgs() - baseline,
        0,
        "an unknown-hash frame renders nothing before the walker knows the schema"
    );

    // Swap in the builtin walker (knows Vector3). FIFO on the channel → the next
    // batch decodes against the new walker.
    worker.swap_walker(builtin_walker());

    // AFTER the swap: the SAME frame now decodes → three Scalar plots.
    let after_swap_baseline = storage.num_msgs();
    worker.try_enqueue(one_frame());
    worker.sync();
    flush.flush_blocking().expect("flush post-swap");
    assert_eq!(
        storage.num_msgs() - after_swap_baseline,
        3,
        "after the swap the same frame renders as three Scalar plots"
    );

    drop(worker);
}

// ---------------------------------------------------------------------------
// Test 7 — poll is deterministic (byte-identical across runs == hand oracle).
// ---------------------------------------------------------------------------

#[test]
fn poll_output_is_deterministic_byte_identical() {
    // Two independent publish→poll cycles yield byte-identical tap output, both
    // equal to the hand-built oracle (Principle #7-flavoured; not a self-compare
    // — both are anchored to the SAME hand oracle).
    let oracle: Vec<Vec<u8>> = (0..3)
        .map(|i| {
            build_vector3_frame(
                i as u32,
                500 * (i as u64 + 1),
                [i as f64, i as f64 + 0.5, 7.0],
            )
        })
        .collect();

    let run_once = |node: &str, topic: &str| -> Vec<Vec<u8>> {
        let mgr = isolated_transport(node);
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
            .expect("producer attaches");
        let mut tm = TapManager::new();
        tm.attach(&mgr, topic, None, WakeMode::Timer)
            .expect("attach");
        for frame in &oracle {
            publisher.publish_raw(frame).expect("publish_raw");
        }
        // A couple of polls in case delivery lags the first (each drains all
        // queued frames; the first non-empty poll owns them).
        let mut collected = Vec::new();
        for _ in 0..3 {
            for input in tm.poll(POLL_MAX) {
                collected.extend(input.frames);
            }
            if !collected.is_empty() {
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        collected
    };

    let a = run_once("det_a", "/det/vel");
    let b = run_once("det_b", "/det/vel");
    assert_eq!(a, oracle, "run A drains exactly the hand oracle");
    assert_eq!(b, oracle, "run B drains exactly the hand oracle");
    assert_eq!(a, b, "two runs are byte-identical");
}

// ---------------------------------------------------------------------------
// Test 8 — the NON-FATAL drain-error path: a mid-drain
// receive fault forwards the partial fill via the Err-ARM forward, the tap
// SURVIVES, and the next poll delivers the rest. Kills the mutations: the Err-arm
// dropping the partial (the byte-exact f0 assert fails), dropping the tap (the
// survive + next-poll-delivers asserts fail).
//
// `after = 1` is deliberate: with the default iceoryx2 borrow budget (2) the
// ERRORING drain_owned chunk itself pushes f0 into scratch and THEN errors on the
// next receive, so f0 rides the Err-arm's partial forward (NOT the success-path
// forward). `after = 2` would deliver f0,f1 through a fully-successful chunk and
// leave the erroring chunk's scratch empty — the Err-arm forward would be untested
// and a "drop the Err-arm forward" mutation would slip through.
// ---------------------------------------------------------------------------

#[test]
fn tap_survives_drain_fault_forwards_partial_then_next_poll_delivers() {
    let mgr = isolated_transport("drain_fault");
    let topic = "/sensor/accel";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("producer attaches");

    let mut tm = TapManager::new();
    tm.attach(&mgr, topic, None, WakeMode::Timer)
        .expect("attach live topic");

    // FIVE distinct hand-built frames (the partial-forward oracle; no fake data).
    let oracle: Vec<Vec<u8>> = (0..5)
        .map(|i| build_vector3_frame(i as u32, 1_000 * (i as u64 + 1), [i as f64, 2.0, 3.0]))
        .collect();
    for frame in &oracle {
        publisher.publish_raw(frame).expect("publish_raw Vector3");
    }

    // Arm the tap to FAIL its receive after draining exactly ONE frame. The poll
    // pushes f0 into scratch then the next receive errors — NON-FATAL: the Err-arm
    // forwards that partial fill (byte-exact) and the tap stays attached.
    tm.fault_inject_tap_receive_after(topic, 1);
    let batch = tm.poll(POLL_MAX);
    assert_eq!(
        batch.len(),
        1,
        "the faulting poll still yields the partial fill via the Err-arm forward"
    );
    assert_eq!(
        batch[0].name, "sensor/accel",
        "keyed by the route key (the whole topic)"
    );
    assert_eq!(
        batch[0].frames,
        oracle[..1].to_vec(),
        "the Err-arm forwarded EXACTLY the frame drained before the fault (f0), byte-exact"
    );
    // The tap SURVIVED the non-fatal error (never dropped).
    assert!(
        tm.contains(topic),
        "the tap stays attached after a drain error"
    );
    assert_eq!(tm.len(), 1, "the drain error left the tap set unchanged");

    // The fault cleared itself on firing → the NEXT poll succeeds and delivers
    // the remaining frames (f1..f4) byte-exact — the tap recovered.
    let batch2 = tm.poll(POLL_MAX);
    assert_eq!(batch2.len(), 1, "the recovered poll delivers the rest");
    assert_eq!(
        batch2[0].frames,
        oracle[1..].to_vec(),
        "the surviving tap delivers the remaining frames on the next poll (byte-exact)"
    );
}

// ---------------------------------------------------------------------------
// Test 9 — the drain-error warn is FLOOD-LATCHED: N failing
// polls emit EXACTLY one warn + (N-1) debug, and the first successful poll emits
// ONE recovery info. Pins the comment-vs-mechanism claim. Kills the mutation:
// reverting to an un-latched per-poll `warn!` yields N warns + 0 debug (fails),
// dropping the recovery `on_decoded` yields 0 recovery info (fails).
//
// The ONLY `#[traced_test]` in this binary — `tracing_test`'s capture is a
// process-global buffer, so a second traced test emitting the SAME substrings
// would contaminate the counts (the `drop_latch_log_test.rs` own-binary pattern).
// All log-discipline assertions are folded into this one body.
// ---------------------------------------------------------------------------

#[traced_test]
#[test]
fn drain_error_is_flood_latched_warn_once_then_debug_then_recovery_info() {
    const N: usize = 4;
    let mgr = isolated_transport("drain_latch_log");
    let topic = "/telemetry/state";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::const_new(1 << 16), 0)
        .expect("producer attaches");
    let mut tm = TapManager::new();
    tm.attach(&mgr, topic, None, WakeMode::Timer)
        .expect("attach live topic");

    // Three frames that stay buffered through the failing regime (each failing
    // poll errors on the FIRST receive, so it drains nothing).
    let oracle: Vec<Vec<u8>> = (0..3)
        .map(|i| build_vector3_frame(i as u32, 100 * (i as u64 + 1), [i as f64, 1.0, 2.0]))
        .collect();
    for frame in &oracle {
        publisher.publish_raw(frame).expect("publish_raw Vector3");
    }

    // N failing polls: re-arm `after = 0` before each so every poll's first
    // receive errors immediately (drains nothing → the frames stay queued).
    for _ in 0..N {
        tm.fault_inject_tap_receive_after(topic, 0);
        let batch = tm.poll(POLL_MAX);
        assert!(
            batch.is_empty(),
            "an immediate-fault poll drains nothing (frames stay queued)"
        );
        assert!(tm.contains(topic), "the tap survives every drain error");
    }

    // The healing poll (no fault re-armed) drains the three buffered frames
    // byte-exact AND heals the regime with one recovery info.
    let healed = tm.poll(POLL_MAX);
    assert_eq!(
        healed.len(),
        1,
        "the healed poll delivers the buffered frames"
    );
    assert_eq!(
        healed[0].frames, oracle,
        "the surviving tap delivers all buffered frames once it recovers (byte-exact)"
    );

    logs_assert(|lines: &[&str]| {
        let warns = lines
            .iter()
            .filter(|l| l.contains("viz tap drain failed"))
            .count();
        let debugs = lines
            .iter()
            .filter(|l| l.contains("viz tap drain still failing"))
            .count();
        let recoveries = lines
            .iter()
            .filter(|l| l.contains("viz tap drain recovered"))
            .count();
        if warns != 1 {
            return Err(format!("expected exactly ONE loud drain warn, got {warns}"));
        }
        if debugs != N - 1 {
            return Err(format!(
                "expected exactly {} suppressed debug repeats, got {debugs}",
                N - 1
            ));
        }
        if recoveries != 1 {
            return Err(format!(
                "expected exactly ONE recovery info, got {recoveries}"
            ));
        }
        Ok(())
    });
}

// ---------------------------------------------------------------------------
// The optional wake listener.
//
// The oracle is the PRODUCER's own live event-listener count
// (`event_listener_count_for_test`), which is the data-only-tap event-level proof run
// in reverse: there a listener's ABSENCE was the property; here its presence is
// the cost, and the count is the only log-independent way to see it.
//
// The publisher is `create_ingress_publisher` — netd's own desk-side mirror
// publisher, the exact producer a `WakeMode::Listener` tap observes in
// production.
// ---------------------------------------------------------------------------

/// A `WakeMode::Timer` tap registers ZERO event listeners — byte-identical to
/// every tap before the wake path landed — while a `WakeMode::Listener` tap on the SAME topic
/// raises the producer's count by exactly one, and detach gives it back.
///
/// Mutation oracle: opening a listener under `WakeMode::Timer` fails the first
/// assertion; never opening one under `WakeMode::Listener` fails the second.
#[test]
fn timer_mode_registers_no_listener_while_listener_mode_registers_exactly_one() {
    let mgr = isolated_transport("wake_modes");
    let topic = "/tap/modes";
    let mut publisher = mgr
        .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
        .expect("netd's mirror-publisher shape");
    let base = publisher.event_listener_count_for_test();

    // TIMER: the data-only-tap shape, unchanged.
    let mut tm = TapManager::new();
    tm.attach(&mgr, topic, None, WakeMode::Timer)
        .expect("timer attach");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base,
        "a WakeMode::Timer tap must register ZERO event listeners — it is the \
         pre-wake-path tap, byte for byte"
    );
    assert!(
        !tm.has_wake(topic),
        "a Timer tap reports no wake (Principle #3: the cost is observable)"
    );
    assert!(
        tm.wake_sources().is_empty(),
        "a Timer-only manager offers a drain loop nothing to block on"
    );
    tm.detach(topic);

    // LISTENER: exactly one, and only one.
    let mut tm = TapManager::new();
    tm.attach(&mgr, topic, None, WakeMode::Listener)
        .expect("listener attach");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base + 1,
        "a WakeMode::Listener tap registers EXACTLY ONE event listener"
    );
    assert!(tm.has_wake(topic), "and reports it");
    assert_eq!(
        tm.wake_sources().len(),
        1,
        "the drain loop is offered exactly that one source"
    );
    assert_eq!(tm.wake_sources()[0].topic(), topic);

    // Detach releases it, so the producer's own notify-elision gate can re-arm.
    tm.detach(topic);
    publisher
        .publish_raw(&build_vector3_frame(0, 1, [0.0; 3]))
        .expect("publish drives the producer's event sweep");
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base,
        "detaching the tap released its wake listener"
    );
}

/// The frame stream is BYTE-IDENTICAL between the two modes. The wake changes
/// only WHEN a caller would look; it must never change WHAT it finds.
///
/// Each leg publishes the same hand-built oracle against a fresh tap on its own
/// topic, so this is a comparison against written-down bytes four times over,
/// never a self-compare of one run against another.
///
/// **The wake path runs it on the LOCAL producer shape too.** If only
/// a mirror ever took a `Listener`, the parity claim would only ever
/// have to hold for `create_ingress_publisher`. A local GRAPH publisher is a
/// different producer — `SingleWriter` provisioning, an elision-armable notify
/// gate — and it is now the shape `attach_local` opens a listener against, so
/// the sibling claim has to hold there or the local wake changed the data.
#[test]
fn the_frame_stream_is_byte_identical_between_wake_modes() {
    let mgr = isolated_transport("wake_parity");
    let oracle: Vec<Vec<u8>> = (0..4)
        .map(|i| build_vector3_frame(i as u32, 100 * (i as u64 + 1), [i as f64, 7.5, -1.0]))
        .collect();

    // (producer shape, topic, mode) — the two modes on BOTH producer classes.
    let mut drained: Vec<(&str, Vec<Vec<u8>>)> = Vec::new();
    for (shape, topic, mode) in [
        ("mirror", "/parity/mirror/timer", WakeMode::Timer),
        ("mirror", "/parity/mirror/listener", WakeMode::Listener),
        ("local", "/parity/local/timer", WakeMode::Timer),
        ("local", "/parity/local/listener", WakeMode::Listener),
    ] {
        let mut publisher = match shape {
            "mirror" => mgr
                .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
                .expect("mirror publisher"),
            _ => mgr
                .create_publisher_with_topic_config(
                    topic,
                    MaxSliceLen::const_new(1 << 16),
                    0,
                    graph_topic_config(&mgr, 2),
                )
                .expect("graph-shaped local publisher"),
        };
        let mut tm = TapManager::new();
        tm.attach(&mgr, topic, None, mode).expect("attach");
        for frame in &oracle {
            publisher.publish_raw(frame).expect("publish");
        }
        let batch = tm.poll_detailed(POLL_MAX);
        assert_eq!(batch.len(), 1, "{topic}: one tap yielded frames");
        assert_eq!(
            batch[0].frames, oracle,
            "{topic}: the tap drained EXACTLY the published frames, in order — the \
             wake is a sibling of the tap, not a change to it"
        );
        drained.push((shape, batch[0].frames.clone()));
    }
    // And the four legs agree with each other as well as with the oracle: the
    // mode does not change the bytes, and neither does the producer class.
    for (shape, frames) in &drained {
        assert_eq!(
            *frames, drained[0].1,
            "{shape}: every (producer shape x wake mode) leg must drain the same bytes"
        );
    }
}

/// A wake that cannot be opened is NOT an attach failure: the tap attaches in
/// Timer mode, keeps delivering, and says so LOUDLY.
///
/// The failure is induced the way a real one arises — the topic's event service
/// is at its listener ceiling, so `create_wake_listener` is refused while
/// `create_data_only_subscriber` (which never touches the event service) is not.
///
/// **The wake path extends this to the LOCAL producer shape.** The degrade arm
/// against `create_ingress_publisher` (a mirror) alone covers only one
/// class that asks for a wake. A genuine GRAPH-shaped
/// producer — `SingleWriter` provisioning, the shape whose event budget is the
/// tight one (see the port-budget arm below) — asks too, so it gets the same
/// coverage. Both halves run in ONE body against one oracle: the degrade rule is
/// one rule, and splitting it would let the two drift.
#[test]
#[traced_test]
fn a_wake_that_cannot_be_opened_degrades_the_tap_to_the_timer_loudly() {
    let mgr = isolated_transport("wake_degrade");

    // (a) the MIRROR shape (netd's desk-side publisher), and (b) the LOCAL GRAPH
    // shape — a `SingleWriter` topic provisioned exactly as the graph runtime
    // provisions one, which is the class the wake path bills.
    let mirror_topic = "/tap/degrade/mirror";
    let local_topic = "/tap/degrade/local";
    let mut producers: Vec<(&str, cerulion_core::transport::publisher::CerulionPublisher)> = vec![
        (
            mirror_topic,
            mgr.create_ingress_publisher(mirror_topic, MaxSliceLen::const_new(1 << 16))
                .expect("mirror publisher"),
        ),
        (
            local_topic,
            mgr.create_publisher_with_topic_config(
                local_topic,
                MaxSliceLen::const_new(1 << 16),
                0,
                graph_topic_config(&mgr, 2),
            )
            .expect("graph-shaped local publisher"),
        ),
    ];

    for (topic, publisher) in producers.iter_mut() {
        // Fill the event service's listener slots with raw wake listeners until
        // it refuses one more, and HOLD them, so the tap's own wake cannot be
        // opened. The bound is well above any plausible ceiling, so a raised
        // default fails the precondition loudly instead of looping forever.
        let mut hogs = Vec::new();
        let mut ceiling_reached = false;
        for _ in 0..512 {
            match mgr.create_wake_listener(topic) {
                Ok(l) => hogs.push(l),
                Err(_) => {
                    ceiling_reached = true;
                    break;
                }
            }
        }
        assert!(
            ceiling_reached,
            "{topic}: precondition — the event service must have a listener \
             ceiling to exhaust"
        );

        // The TAP still attaches — the wake is an optimisation, the frames are not.
        let mut tm = TapManager::new();
        tm.attach(&mgr, topic, None, WakeMode::Listener)
            .expect("a wake failure must NOT fail the attach");
        assert!(
            !tm.has_wake(topic),
            "{topic}: the wake was refused, so the tap reports it has none"
        );

        // And it delivers, byte-exact, on the timer path.
        let oracle = build_vector3_frame(0, 42, [1.0, 2.0, 3.0]);
        publisher.publish_raw(&oracle).expect("publish");
        let batch = tm.poll_detailed(POLL_MAX);
        assert_eq!(batch.len(), 1, "{topic}: one tap yielded frames");
        assert_eq!(
            batch[0].frames,
            vec![oracle],
            "{topic}: a degraded tap delivers exactly the same frames, one poll \
             interval later"
        );
        drop(hogs);
    }

    // LOUD, never silent: the operator is told which topic lost its wake and
    // what it costs — on BOTH shapes (the two loops each degraded once).
    assert!(logs_contain("viz tap attached WITHOUT its wake listener"));
}

/// A GRAPH-shaped topic's event service, provisioned exactly as the graph
/// runtime provisions one: `SingleWriter`, `in_graph_subscribers` consumers plus
/// [`INTROSPECTION_SUBSCRIBER_HEADROOM`] spare data slots.
fn graph_topic_config(
    mgr: &TransportManager,
    in_graph_subscribers: usize,
) -> cerulion_core::transport::TopicServiceConfig {
    cerulion_core::transport::TopicServiceConfig::for_topology(
        mgr.default_topic_config(),
        /* max_consumer_depth  */ 4,
        in_graph_subscribers,
        cerulion_core::transport::PublisherProvisioning::SingleWriter,
        /* history_size        */ 0,
        /* extra_event_listeners */ 0,
    )
}

/// **The wake path, THE PORT-BUDGET CHECK.** A wake is a listener on the
/// topic's EVENT service, so raising local topics onto the wake path spends a
/// resource that is provisioned, finite, and — on a graph-owned topic —
/// TIGHTER than on a mirror. This arm measures whether it can run out.
///
/// The arithmetic, from `TopicServiceConfig::for_topology` +
/// `open_topic_services`, for a `SingleWriter` topic:
///
/// ```text
/// data subscriber slots = in_graph_subscribers + INTROSPECTION_SUBSCRIBER_HEADROOM
/// event listener slots  = max_subscribers + max_publishers + extra_event_listeners
///                       = (in_graph + HEADROOM) + 1 + 0
/// ```
///
/// The publisher itself holds ONE of those listeners (its `SubscriberConnected`
/// channel), which leaves exactly `data subscriber slots` free — one per
/// possible data reader. A vizd tap is a `DataOnlySubscriber` (1 data slot, 0
/// listeners) plus a `WakeSource` (0 data slots, 1 listener), so **every tap
/// that can exist at all can also hold a wake**: the data cap is reached first,
/// by construction, and the wake can never be the binding constraint.
///
/// That is the claim, and it is asserted rather than argued: `data_slots`
/// tap+wake PAIRS all attach, the NEXT tap is refused on the DATA half, and one
/// further bare listener is refused on the event half — i.e. the two budgets run
/// out together, with the data one first.
///
/// MEASURED numbers are printed so a change in either formula shows up as a
/// diff in the run log, not only as a failed assertion.
#[test]
fn a_graph_topics_event_budget_admits_one_wake_for_every_tap_it_can_hold() {
    use cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM;

    const IN_GRAPH: usize = 2;
    let expected_slots = IN_GRAPH + INTROSPECTION_SUBSCRIBER_HEADROOM;

    let mgr = isolated_transport("wake_budget");
    let topic = "/tap/budget";
    let _publisher = mgr
        .create_publisher_with_topic_config(
            topic,
            MaxSliceLen::const_new(1 << 16),
            0,
            graph_topic_config(&mgr, IN_GRAPH),
        )
        .expect("graph-shaped local publisher");

    // Every data slot the topic offers, taken as a tap+wake PAIR — the exact
    // shape `attach_local` now opens.
    let mut taps = Vec::new();
    let mut wakes = Vec::new();
    for i in 0..expected_slots {
        taps.push(
            mgr.create_data_only_subscriber(topic)
                .unwrap_or_else(|e| panic!("tap {i} of {expected_slots} must attach: {e}")),
        );
        wakes.push(
            mgr.create_wake_listener(topic)
                .unwrap_or_else(|e| panic!("wake {i} of {expected_slots} must attach: {e}")),
        );
    }
    println!(
        "wake path [port budget] graph topic with in_graph={IN_GRAPH}: \
         {expected_slots} tap+wake pairs attached (headroom \
         {INTROSPECTION_SUBSCRIBER_HEADROOM})"
    );

    // THE CLAIM: the DATA half runs out first. A vizd tap that cannot exist
    // cannot want a wake, so the wake is never the binding constraint.
    let over_tap = mgr.create_data_only_subscriber(topic);
    assert!(
        over_tap.is_err(),
        "the topic must be at its DATA subscriber cap after {expected_slots} taps \
         — otherwise this arm is not measuring the budget it claims to"
    );

    // And the event half is exhausted at exactly the same point (the publisher's
    // own listener is the +1), so no slack is being silently relied on.
    let over_wake = mgr.create_wake_listener(topic);
    assert!(
        over_wake.is_err(),
        "the event budget is `data slots + 1` and the publisher holds the +1, so \
         a further listener must be refused: an extra one means the two budgets \
         are NOT running out together and the arithmetic in this arm's docs is \
         wrong"
    );

    // Dropping one pair gives BOTH slots back — the bill is held only while a
    // human is watching, which is the whole of the wake path's cost argument.
    taps.pop();
    wakes.pop();
    assert!(
        mgr.create_data_only_subscriber(topic).is_ok(),
        "detaching a tap releases its data slot"
    );
    assert!(
        mgr.create_wake_listener(topic).is_ok(),
        "and dropping its wake releases the listener — the producer's notify-elision \
         gate re-arms within one publish"
    );
}

/// `demote_to_timer` drops the wake and leaves the tap delivering — the lever
/// the wedge fallback pulls.
#[test]
fn demoting_a_tap_to_the_timer_releases_its_listener_and_keeps_it_delivering() {
    let mgr = isolated_transport("wake_demote");
    let topic = "/tap/demote";
    let mut publisher = mgr
        .create_ingress_publisher(topic, MaxSliceLen::const_new(1 << 16))
        .expect("mirror publisher");
    let base = publisher.event_listener_count_for_test();

    let mut tm = TapManager::new();
    tm.attach(&mgr, topic, None, WakeMode::Listener)
        .expect("attach");
    assert_eq!(publisher.event_listener_count_for_test(), base + 1);

    assert!(
        tm.demote_to_timer(topic),
        "demoting a tap that HAS a wake reports that it dropped one"
    );
    assert!(!tm.has_wake(topic));
    assert!(tm.wake_sources().is_empty());
    assert!(
        !tm.demote_to_timer(topic),
        "demoting again reports nothing was dropped (idempotent, never an error)"
    );
    assert!(
        !tm.demote_to_timer("/never/attached"),
        "demoting an unattached topic is a no-op, never a panic"
    );

    // The tap survived the demotion and still delivers, byte-exact.
    let oracle = build_vector3_frame(0, 9, [4.0, 5.0, 6.0]);
    publisher.publish_raw(&oracle).expect("publish");
    let batch = tm.poll_detailed(POLL_MAX);
    assert_eq!(batch.len(), 1);
    assert_eq!(batch[0].frames, vec![oracle]);
    assert_eq!(
        publisher.event_listener_count_for_test(),
        base,
        "the demotion released the listener, so the producer stops paying for it"
    );
}

/// `wake_sources` is ordered like `list` — deterministically sorted — so a drain
/// loop's fired INDEX maps back to a topic without a second lookup order.
#[test]
fn wake_sources_are_returned_in_the_same_sorted_order_as_list() {
    let mgr = isolated_transport("wake_order");
    // Attached in REVERSE-alphabetical order, so a sorted result cannot be an
    // insertion-order accident.
    let topics = ["/order/c", "/order/a", "/order/b", "/order/d"];
    let _publishers: Vec<_> = topics
        .iter()
        .map(|t| {
            mgr.create_ingress_publisher(t, MaxSliceLen::const_new(1 << 16))
                .expect("mirror publisher")
        })
        .collect();

    let mut tm = TapManager::new();
    // `/order/b` is deliberately a TIMER tap: the wake vector must be the
    // sorted subset that HAS wakes, not the sorted tap list with a hole in it.
    for topic in topics {
        let mode = if topic.ends_with("/b") {
            WakeMode::Timer
        } else {
            WakeMode::Listener
        };
        tm.attach(&mgr, topic, None, mode).expect("attach");
    }

    assert_eq!(
        tm.list(),
        vec![
            "/order/a".to_string(),
            "/order/b".to_string(),
            "/order/c".to_string(),
            "/order/d".to_string(),
        ]
    );
    let sources = tm.wake_sources();
    let wake_topics: Vec<&str> = sources.iter().map(|s| s.topic()).collect();
    assert_eq!(
        wake_topics,
        vec!["/order/a", "/order/c", "/order/d"],
        "the wake vector is the SORTED subset that has wakes — the timer tap \
         contributes nothing rather than a gap"
    );
}
