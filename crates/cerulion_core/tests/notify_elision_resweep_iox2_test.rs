// SPDX-License-Identifier: AGPL-3.0-only
//! The LIVE-LOOP BOUNDARY re-check for the notify-elision
//! self-heal, end-to-end over real iceoryx2 (`GraphRuntime::build_for_test`,
//! per-test SHM root; no fake data, Principle #13).
//!
//! # The failure this guards against
//!
//! The per-publish self-heal resumes notifies the instant a FOREIGN listener
//! attaches — but it is evaluated ONLY inside `notify_sent_sample`, i.e. ONLY
//! when the producer publishes through the `OutputProxy` path. On the DDS-bridge
//! attach graph the affected producers publish OFF that path: a raw wire frame
//! via `AnyPublisher::publish_raw` (which by contract does NOT notify), or they
//! go quiescent. Either way, with only the per-publish check, a `topic hz` that
//! attaches afterward blocks on the
//! topic's event listener FOREVER — the elided/off-gate publisher never rings
//! it, even though iceoryx2's `send` already put the frames in its SHM queue
//! (`send` auto-updates its subscriber connections).
//!
//! # The boundary re-check
//!
//! The runtime re-checks the elision gate on every owned publisher at the
//! live-loop boundary (`pump_history_all`, the ≤250 ms cadence that already
//! services quiescent publishers' late-joiner history), so a foreign
//! LISTENER-full subscriber gets notifies flowing again within one heartbeat
//! REGARDLESS of the producer's publish cadence — a structural boundary, not a
//! timer. No-op unless elision is armed AND a foreign listener is present, so
//! the elision win and the data-only-tap invariant are untouched.
//!
//! # SCOPE: the LISTENER-driven ingress class only
//!
//! This heals a listener-driven consumer (`topic hz`/`echo`, a listener-full
//! sink) that blocks on a topic's event listener — the PRIMARY symptom
//! (`topic hz /lowstate` blocked forever). It does NOT causally heal the
//! poll-paced GATEWAY / `remoted` EGRESS: those drain listener-less
//! `DataOnlySubscriber`s in bare poll loops (`gateway.rs::run` +
//! `tap.drain_owned`; `remoted/tap.rs`) that never wait on a `SentSample`, so
//! they are STRUCTURALLY notify-immune and `CERULION_NOTIFY_ELISION` on/off
//! cannot change what they drain. An egress stall on that path has
//! a SEPARATE root cause (demand/liveliness/queue/live-
//! loop-throttle) — the boundary re-check does not touch the egress data path.
//!
//! # What this file pins
//!
//! A graph with a producer that publishes OFF the notify gate (`publish_raw`) +
//! an in-graph consumer (so elision arms). A late foreign LISTENER-full
//! subscriber (the `topic hz` shape, on a SEPARATE manager over the same SHM
//! root) blocks on its event listener with an EMPTY queue; the producer then
//! publishes one off-gate frame and the runtime drives ONE `pump_history_all`
//! boundary pass. The subscriber must WAKE and receive that frame within a
//! BOUNDED window well under its own long wait timeout — proving the boundary
//! re-check delivered the wake the per-publish path never would. With the resweep
//! reverted the subscriber only wakes on its own 2 s timeout, blowing the
//! bounded window (the negative control). Hand oracle on the frame's stamped
//! `timestamp_ns`, never a self-compare.
//!
//! `#[serial]` — the foreign subscriber runs on a second manager over the same
//! SHM root; `build_for_test` is per-test-SHM-root, but the blocking-wait timing
//! assertion prefers an unshared machine.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test notify_elision_resweep_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::{RealClock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, MacroPolicy, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wake::WakeSet;
use cerulion_core::wire::WireHeader;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Publishes driven through the elision-armed gate before the silent
/// window, so the "one notify per FRAME" half of the cost model is measured
/// against a real number rather than asserted at zero traffic.
const FRAMES: u32 = 6;
/// Silent boundary passes — a live loop takes this boundary on EVERY
/// iteration, so this stands in for ~200 quiet ~1 kHz passes. Without the debt gate EVERY one
/// of them would fire a real `SentSample`.
const SILENT_PASSES: u32 = 200;

/// The producer's blocking-wait timeout on the foreign subscriber. Large so the
/// negative control (no boundary wake) is DISTINGUISHABLE from the resweep path.
const WAIT_TIMEOUT: Duration = Duration::from_secs(2);
/// The BOUNDED window the resweep must deliver within. Well above the ~150 ms the
/// main thread settles before publishing, well below `WAIT_TIMEOUT`.
const BOUNDED_WINDOW: Duration = Duration::from_millis(1000);
/// The main thread lets the foreign subscriber block on an EMPTY queue before
/// publishing, so the frame arrives DURING the blocking wait (needs the wake).
const SETTLE: Duration = Duration::from_millis(150);

/// Build a structurally-valid raw wire frame carrying only a stamped
/// `timestamp_ns` (the hand oracle) — an all-fixed, header-only Vector3 frame
/// (`total_size == WireHeader::SIZE`, no payload, no offset table). The foreign
/// subscriber reads `msg.header().timestamp_ns` back.
fn build_raw_frame(sequence: u32, timestamp_ns: u64) -> Vec<u8> {
    let mut header = WireHeader::new(Vector3::SCHEMA_HASH, sequence, timestamp_ns);
    header.total_size = WireHeader::SIZE as u32;
    let mut buf = vec![0u8; WireHeader::SIZE];
    header.write_to_buf(&mut buf);
    buf
}

/// Build the graph: a Period producer that publishes a raw frame via
/// `publish_raw` (OFF the notify gate — the raw-ingress shape) each fire,
/// stamping `timestamp_ns = fire_count`, plus a Period consumer that drains the
/// topic (so the graph has a real in-graph, listener-owning consumer and
/// elision arms). Returns `(runtime, producer_fire_count, topic)`.
fn build_offgate_graph(prefix: &str) -> (GraphRuntime, Arc<AtomicU64>, String) {
    let fire_count = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "resweep".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "offgate_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "draining_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();

    // Producer: Period @ 10ms, publishes a raw frame via publish_raw each fire.
    let fc = Arc::clone(&fire_count);
    let producer_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let producer = ClosureNodeEntry::new(producer_info, move |ctx| {
        let seq = fc.fetch_add(1, Ordering::Relaxed) + 1;
        // `timestamp_ns = seq` IS the hand oracle the foreign subscriber reads.
        let frame = build_raw_frame(seq as u32, seq);
        let pubr = ctx
            .publisher_mut("out")
            .expect("producer publisher 'out' must be wired");
        // OFF the notify gate: publish_raw never calls notify_sent_sample, so the
        // per-publish self-heal never runs — exactly the raw-ingress path.
        pubr.publish_raw(&frame)?;
        Ok(())
    })
    .with_label("offgate_producer");
    factories.insert("producer".to_string(), Box::new(producer));

    // Consumer: Period @ 10ms, drains the topic (arms a real in-graph listener).
    let consumer_info = NodeInfo::from_names(vec!["inp".to_string()], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        if let Some(sub) = ctx.subscriber("inp") {
            let _ = sub.try_receive(|_msg| {});
        }
        Ok(())
    })
    .with_label("draining_consumer");
    factories.insert("consumer".to_string(), Box::new(consumer));

    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build off-gate graph");
    let topic = format!("/{prefix}/producer/out");
    (runtime, fire_count, topic)
}

/// Build the on-gate graph: a Period producer publishing through `loan_proxy` +
/// `OutputProxy::Drop` — the ON-GATE graph-publisher shape notify elision arms,
/// i.e. the producer a locally-viewed Studio topic actually has — plus a
/// draining Period consumer so elision arms against a real in-graph listener.
/// Returns `(runtime, producer_fire_count, topic)`.
fn build_ongate_graph(prefix: &str) -> (GraphRuntime, Arc<AtomicU64>, String) {
    let fire_count = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "resweep_ongate".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "ongate_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "draining_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();

    let fc = Arc::clone(&fire_count);
    let producer_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let producer = ClosureNodeEntry::new(producer_info, move |ctx| {
        let seq = fc.fetch_add(1, Ordering::Relaxed) + 1;
        // ON the gate: the loaned proxy's Drop runs `notify_sent_sample`, so the
        // elision decision is taken per publish — the only local publish
        // path that notifies at all.
        let pubr = ctx
            .publisher_mut("out")
            .expect("producer publisher 'out' must be wired");
        let mut out = pubr.loan_proxy::<Vector3>()?;
        out.x = seq as f64;
        Ok(())
    })
    .with_label("ongate_producer");
    factories.insert("producer".to_string(), Box::new(producer));

    let consumer_info = NodeInfo::from_names(vec!["inp".to_string()], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        if let Some(sub) = ctx.subscriber("inp") {
            let _ = sub.try_receive(|_msg| {});
        }
        Ok(())
    })
    .with_label("draining_consumer");
    factories.insert("consumer".to_string(), Box::new(consumer));

    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build on-gate graph");
    let topic = format!("/{prefix}/producer/out");
    (runtime, fire_count, topic)
}

/// A vizd `WakeSource` on an elision-armed graph publisher costs
/// ONE notify per FRAME — never one per live-loop pass.
///
/// # Why this arm exists (and why it lives beside the boundary re-check pin)
///
/// vizd puts a `WakeSource` — a listener on the topic's
/// event service — on every LOCALLY-viewed Studio topic, whose producer is a
/// graph publisher, i.e. the one class `arm_notify_elision` covers. Its cost is
/// "one `sendto` per frame", and that holds only because `resweep_notify_elision`
/// is gated on an outstanding publish debt: a resweep that fires a real `SentSample`
/// on EVERY boundary pass in which the producer has not published fires on every
/// `live_step` (`pump_history_all`), not on a timer. Measured without the
/// gate: 198 notifies for 2 frames, and — because the notify also
/// reaches the topic's OWN in-graph consumer listener on the live WaitSet — the
/// graph's live loop woke ITSELF and free-ran, 9/s → 136/s with a live producer.
///
/// # What it pins, and why each half is load-bearing
///
/// * The wake genuinely UN-ARMS notify elision (`notify_elided_count` stops
///   moving the instant it attaches, having moved before) — this is the class
///   the whole wake suite is structurally blind to, since every producer
///   in it is a bare `create_publisher` that `arm_notify_elision` never touches.
///   It is also the wake source's stated bill, asserted rather than argued.
/// * The BOUNDARY fires ZERO times across `SILENT_PASSES` — the storm.
///   Load cannot fake this: it is an exact count over a hand-driven boundary,
///   with no wall anywhere.
/// * Delivery is byte-identical to a hand oracle throughout, so the gate is
///   proven to change WHEN a consumer wakes and nothing else (Principle #7).
///
/// Restoring a
/// `notified_since_boundary` trigger fires 200 boundary notifies where 0 are
/// required — one per silent pass, MEASURED, not derived; `let elide = true`
/// (the gate never opening for a foreign listener) fails the un-arm
/// assertion at 12 elided vs 6.
#[test]
#[serial]
fn a_wake_on_an_armed_graph_publisher_costs_one_notify_per_frame_not_one_per_pass() {
    let (mut runtime, fire_count, topic) = build_ongate_graph("resweep_wake");

    assert!(
        runtime.notify_elision_expected_for_test(&topic).is_some(),
        "elision must be armed on the graph-owned producer topic (the class a \
         locally-viewed Studio topic is in)"
    );

    // PHASE 0 — no viewer. Elision engages, and the boundary has nothing to say.
    for _ in 0..FRAMES {
        runtime.step(Duration::from_millis(10));
        runtime.pump_history_all_for_test();
    }
    let elided_unwatched = runtime.notify_elided_count_for_test(&topic);
    assert_eq!(
        elided_unwatched, FRAMES as u64,
        "un-watched: every publish elides (the elision win the wake spends)"
    );
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        0,
        "un-watched: no foreign listener ⇒ the boundary announces nothing"
    );

    // PHASE 1 — a vizd tap attaches. This is the REAL production wake source
    // (`TransportManager::create_wake_listener`, exactly what
    // `TapManager::attach(.., WakeMode::Listener)` opens), on a SECOND manager
    // over the same SHM root — the desk-daemon shape.
    let cfg = runtime
        .test_transport()
        .expect("build_for_test parks a test transport")
        .iox_config();
    let viewer_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "vizd".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cfg,
    )
    .expect("second manager on the same SHM root");
    let wake = viewer_mgr
        .create_wake_listener(&topic)
        .expect("a vizd wake source attaches to the live graph topic");

    // PHASE 2 — publish while watched. The gate opens (live > expected), so each
    // publish notifies DIRECTLY and the boundary stays silent: the documented
    // one-`sendto`-per-frame model.
    let before_frames = fire_count.load(Ordering::Relaxed);
    for _ in 0..FRAMES {
        runtime.step(Duration::from_millis(10));
        runtime.pump_history_all_for_test();
    }
    let published_while_watched = fire_count.load(Ordering::Relaxed) - before_frames;
    assert_eq!(
        published_while_watched, FRAMES as u64,
        "precondition: the producer really fired once per step while watched"
    );
    assert_eq!(
        runtime.notify_elided_count_for_test(&topic),
        elided_unwatched,
        "THE BILL: a vizd wake UN-ARMS elision — every watched publish notifies for \
         itself (the elided counter stops moving). Anti-tautology: it moved to {} \
         before the wake attached.",
        elided_unwatched
    );

    // PHASE 3 — THE STORM ARM. The producer says nothing; the live loop keeps
    // taking its boundary. Without the debt gate each of these fires a real notify into the
    // graph's own WaitSet.
    for _ in 0..SILENT_PASSES {
        runtime.pump_history_all_for_test();
    }
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        0,
        "{FRAMES} watched publishes + {SILENT_PASSES} silent \
         boundary passes must fire ZERO boundary notifies — the per-publish path \
         announced every frame, and a quiet pass has nothing to announce"
    );

    // PHASE 3b: **THE DEMOTER-INVERSE ARM (the storm's second consequence).**
    // The count above is the producer's side of the storm; this is the VIEWER's.
    // vizd's `WakeDemoter` drops a wake that FIRES `WAKE_DEMOTE_STREAK` (64)
    // times running without delivering, so a boundary firing on every silent pass
    // makes an IDLE locally-viewed topic demote its OWN wake within ~64 passes of
    // a ~1 kHz loop — and when that topic later starts streaming it is back on
    // the 16 ms poll floor the wake source exists to remove. Nothing above can
    // see that: the demoter counts FIRES, not notifies.
    //
    // So ask the wake source itself, through the same `WakeSet` vizd's drain loop
    // blocks on. Zero fires across a window ~3x the streak ⇒ no streak can ever
    // accumulate on an idle topic, and the demotion arithmetic is pinned by
    // vizd's own pure oracles.
    let mut set = WakeSet::new(1).expect("a wake set over the viewer's source");
    // QUIESCE first: phase 2's publishes legitimately notified this listener and
    // those notifications are still queued (a real drain loop would have consumed
    // them draining the frames). A fire owed to a real frame is not the defect —
    // the defect is a fire on a pass where nothing was published.
    let mut drained_pending = 0_usize;
    for _ in 0..64 {
        if set
            .wait(&[&wake], Duration::from_millis(1))
            .expect("wake wait")
            .is_empty()
        {
            break;
        }
        drained_pending += 1;
    }
    assert!(
        drained_pending >= 1,
        "precondition (anti-vacuity): the watched publishes really did ring this \
         listener, so the zero below is a measured silence rather than a source \
         that never fires at all"
    );

    let mut fires = 0_usize;
    for _ in 0..SILENT_PASSES {
        runtime.pump_history_all_for_test();
        // A short, NON-zero timeout: an empty slice would not block at all, and a
        // fired source is a QUEUED notification — load can delay it, never erase
        // it, so this is a count assertion rather than a wall.
        if !set
            .wait(&[&wake], Duration::from_millis(1))
            .expect("wake wait")
            .is_empty()
        {
            fires += 1;
        }
    }
    assert_eq!(
        fires, 0,
        "an IDLE watched topic's wake must never fire: {SILENT_PASSES} silent \
         boundary passes fired it {fires} times, which is how a viewer would \
         DEMOTE its own wake (64 fired-but-empty waits) and lose the instant path \
         for the stream that came later"
    );

    // FIREWALL — the wake changes WHEN a consumer is told, never WHAT flowed. The
    // producer published 1..=2N contiguously across both phases.
    assert_eq!(
        fire_count.load(Ordering::Relaxed),
        (FRAMES * 2) as u64,
        "hand oracle: one publish per step across both phases, nothing lost or doubled"
    );

    // PHASE 4 — the LATE-ATTACH guarantee survives the resweep. Drop the viewer (the
    // gate re-arms), publish so a frame elides, then re-attach: the boundary must
    // announce that ONE debt exactly once, and then go quiet again.
    drop(wake);
    runtime.step(Duration::from_millis(10));
    runtime.pump_history_all_for_test();
    assert_eq!(
        runtime.notify_elided_count_for_test(&topic),
        elided_unwatched + 1,
        "viewer gone: the gate re-arms and the publish elides again"
    );
    let resweeps_before_reattach = runtime.notify_resweep_count_for_test(&topic);
    let _wake2 = viewer_mgr
        .create_wake_listener(&topic)
        .expect("a second vizd wake attaches after the elided publish");
    runtime.pump_history_all_for_test();
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        resweeps_before_reattach + 1,
        "LATE ATTACH: the frame elided before this listener existed, so the boundary \
         MUST announce it — exactly once"
    );
    for _ in 0..SILENT_PASSES {
        runtime.pump_history_all_for_test();
    }
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        resweeps_before_reattach + 1,
        "…and the announcement SPENT the debt: {SILENT_PASSES} further silent \
         passes add nothing"
    );
}

/// **The debt gate's ONE behavioural narrowing, stated as a contract:
/// the late-attach heal is per un-announced FRAME, not per
/// LISTENER.**
///
/// The debt is spent by the FIRST announcing boundary, so a SECOND wake source
/// attaching AFTER that boundary is never resweep-woken for the elided frame its
/// own tap already holds. A resweep that fired on every silent pass would
/// happen to catch that listener too — this arm records
/// what the debt-gated resweep trades away, and the bound on what it does NOT.
///
/// Both taps are connected BEFORE the publish, so iceoryx2 delivers the frame to
/// BOTH queues at `send()` time — which is what makes the claim narrow rather
/// than alarming: viewer B's frame is not lost, it is sitting in B's own SHM
/// queue, and a vizd-class consumer drains it on its timeout fallback (one poll
/// interval). What B loses is the WAKE, i.e. latency, bounded by the consumer's
/// own fallback. The flagship single-viewer path is untouched, because that
/// viewer attaches before its first announcing boundary.
///
/// Restoring a silent-pass trigger makes B's
/// wake fire, which fails this arm — that is the whole point of pinning it, so
/// the day the policy changes back it is a deliberate act with a test to move.
#[test]
#[serial]
fn a_second_wake_attaching_after_the_announcement_rides_the_timeout_not_the_boundary() {
    let (mut runtime, _fire_count, topic) = build_ongate_graph("resweep_wake_second");

    let cfg = runtime
        .test_transport()
        .expect("build_for_test parks a test transport")
        .iox_config();
    let viewer_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "two_viewers".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cfg,
    )
    .expect("second manager on the same SHM root");

    // BOTH taps connect BEFORE the publish — a data-only tap registers NO
    // listener, so the gate is still armed and the publish still
    // elides, while iceoryx2 delivers the frame to both queues at send time.
    let mut tap_a = viewer_mgr
        .create_data_only_subscriber(&topic)
        .expect("viewer A's tap");
    let mut tap_b = viewer_mgr
        .create_data_only_subscriber(&topic)
        .expect("viewer B's tap");

    runtime.step(Duration::from_millis(10));
    runtime.pump_history_all_for_test();
    assert_eq!(
        runtime.notify_elided_count_for_test(&topic),
        1,
        "precondition: with only data-only taps attached the publish ELIDES — \
         that elided frame is the debt the boundary owes somebody"
    );
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        0,
        "precondition: nobody is listening yet, so the boundary announces nothing \
         and the debt SURVIVES"
    );

    // Viewer A attaches its wake. The next boundary announces the debt — to
    // every listener that exists AT THAT MOMENT, which is A alone.
    let wake_a = viewer_mgr
        .create_wake_listener(&topic)
        .expect("viewer A's wake");
    runtime.pump_history_all_for_test();
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        1,
        "the late-attach heal fires for viewer A"
    );

    // Viewer B attaches AFTER the announcement. The debt is spent.
    let wake_b = viewer_mgr
        .create_wake_listener(&topic)
        .expect("viewer B's wake");
    for _ in 0..SILENT_PASSES {
        runtime.pump_history_all_for_test();
    }
    assert_eq!(
        runtime.notify_resweep_count_for_test(&topic),
        1,
        "THE NARROWING: the announcement is per un-announced FRAME, not per \
         LISTENER — viewer B's attach does not re-announce a frame already \
         announced, and {SILENT_PASSES} further passes add nothing"
    );

    // A was woken; B was not. Both are read through the same `WakeSet` a vizd
    // drain loop blocks on, so this is the viewer-facing consequence, not an
    // internal counter.
    let mut set = WakeSet::new(2).expect("a wake set over both viewers' sources");
    assert!(
        !set.wait(&[&wake_a], Duration::from_millis(1))
            .expect("wake wait")
            .is_empty(),
        "viewer A's wake FIRED — the anti-tautology half, without which 'B did \
         not fire' is satisfied by a boundary that woke nobody at all"
    );
    assert!(
        set.wait(&[&wake_b], Duration::from_millis(1))
            .expect("wake wait")
            .is_empty(),
        "viewer B's wake did NOT fire — it attached after the announcement"
    );

    // AND THE BOUND: B lost the WAKE, never the FRAME. Its tap was connected
    // before the publish, so the frame is in B's own queue and its next timeout
    // drain (≤ one poll interval for a vizd-class consumer) collects it.
    let mut out = Vec::new();
    let n_b = tap_b.drain_owned(8, &mut out).expect("viewer B drains");
    assert_eq!(
        n_b, 1,
        "viewer B's tap HOLDS the elided frame — the narrowing costs a wake, \
         which is latency, and never a frame"
    );
    out.clear();
    let n_a = tap_a.drain_owned(8, &mut out).expect("viewer A drains");
    assert_eq!(n_a, 1, "viewer A holds the same one frame (control)");
}

/// The outcome a foreign-subscriber thread reports back.
struct ForeignOutcome {
    /// The `timestamp_ns` of the frame it received (`None` = nothing received).
    received_ts: Option<u64>,
    /// How long its blocking wait took (from wait entry to wake).
    elapsed: Duration,
}

#[test]
#[serial]
fn boundary_resweep_wakes_a_blocked_foreign_subscriber_within_a_bounded_window() {
    let (mut runtime, fire_count, topic) = build_offgate_graph("resweep_boundary");

    // Elision must be armed on the producer's topic (default ON). Off-gate
    // publishing means the per-publish self-heal never runs — only the boundary
    // re-check can heal a late foreign listener.
    assert!(
        runtime.notify_elision_expected_for_test(&topic).is_some(),
        "elision must be armed on the graph-owned producer topic"
    );

    // WARMUP: drive the producer BEFORE the foreign subscriber attaches, so the
    // late joiner (history_size 0) gets NONE of these frames — its queue is empty
    // when it starts to block.
    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }

    // A SECOND manager on the SAME SHM root — the real `topic hz` shape (a
    // separate process opening a listener-full subscriber).
    let cfg = runtime
        .test_transport()
        .expect("build_for_test parks a test transport")
        .iox_config();
    let foreign_mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "resweep_topic_hz".into(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cfg,
    )
    .expect("second manager on the same SHM root");

    let entered_wait = Arc::new(AtomicBool::new(false));
    let entered_t = Arc::clone(&entered_wait);
    let topic_t = topic.clone();

    let handle = std::thread::spawn(move || {
        let sub = foreign_mgr
            .create_subscriber_open_only(&topic_t)
            .expect("foreign topic-hz subscriber attaches (listener-full)");
        // Clear any startup frames so the measured wait begins on an EMPTY queue
        // (a non-blocking drain).
        let _ = sub.wait_for_message(Duration::from_millis(1), |_m| {});

        // Signal we are ABOUT to block, then do the measured blocking wait.
        entered_t.store(true, Ordering::Release);
        let start = Instant::now();
        let mut received_ts: Option<u64> = None;
        let _ = sub.wait_for_message(WAIT_TIMEOUT, |m| {
            received_ts = Some(m.header().timestamp_ns);
        });
        ForeignOutcome {
            received_ts,
            elapsed: start.elapsed(),
        }
    });

    // Wait for the subscriber to signal it is about to block, then SETTLE so it
    // is genuinely parked in the listener wait (past its empty pre-drain) before
    // the test publishes. The frame therefore arrives DURING the blocking wait — the
    // only way to receive it is a WAKE.
    let deadline = Instant::now() + Duration::from_secs(5);
    while !entered_wait.load(Ordering::Acquire) {
        assert!(Instant::now() < deadline, "foreign subscriber never armed");
        std::thread::sleep(Duration::from_millis(2));
    }
    std::thread::sleep(SETTLE);

    // Publish ONE off-gate frame (a step fires the Period producer, which
    // publish_raw's the frame — no notify), then drive ONE boundary pass. The
    // resweep sees the foreign listener (live > expected) and fires the wake.
    let before = fire_count.load(Ordering::Relaxed);
    runtime.step(Duration::from_millis(10));
    let oracle_ts = fire_count.load(Ordering::Relaxed);
    assert_eq!(
        oracle_ts,
        before + 1,
        "the measured step fired the producer exactly once"
    );
    // THE RESWEEP: the boundary re-check runs here (live-loop cadence). Without it
    // the frame sits un-woken in the foreign subscriber's queue until its own
    // 2 s timeout.
    runtime.pump_history_all_for_test();

    // On the debt semantics: the producer
    // published OFF the gate, so `publish_raw` recorded an UN-ANNOUNCED FRAME
    // and notified nobody — the boundary therefore has a debt to announce and
    // FIRES. The trigger suppresses redundant notifies but NEVER disables the
    // genuine quiescent/off-gate heal. (The active-producer skip side is
    // `notify_elision_iox2_test::boundary_resweep_does_not_double_notify_an_active_producer`,
    // where every publish notifies for itself and clears the debt.)
    assert!(
        runtime.notify_resweep_count_for_test(&topic) >= 1,
        "off-gate publish (no per-publish notify) ⇒ the boundary sweep must FIRE the heal"
    );

    let outcome = handle.join().expect("foreign subscriber thread panicked");

    // Hand oracle: it received EXACTLY the off-gate frame we published (its
    // stamped timestamp_ns == the producer's fire count at that step).
    assert_eq!(
        outcome.received_ts,
        Some(oracle_ts),
        "the foreign subscriber must receive the off-gate frame (hand oracle on \
         stamped timestamp_ns)"
    );
    // THE BOUNDED WINDOW: it woke via the boundary notify, NOT its own 2 s
    // timeout. WITH the resweep a no-op this elapsed climbs to
    // ~WAIT_TIMEOUT and the assertion fails — the negative control.
    assert!(
        outcome.elapsed < BOUNDED_WINDOW,
        "the foreign subscriber must wake via the boundary re-check within the \
         bounded window ({:?}), not its own {:?} timeout — got {:?}",
        BOUNDED_WINDOW,
        WAIT_TIMEOUT,
        outcome.elapsed
    );
}
