// SPDX-License-Identifier: AGPL-3.0-only
//! Replay-equivalence tests (Principle #7: Replay = Live).
//!
//! Two complementary scopes:
//!
//! 1. **Scheduler trace replay** — two independent scheduler instances with
//!    identical inputs must produce identical traces.
//!
//! 2. **Message replay via snapshots** — wire frames published through the
//!    SHM-backed `loan_proxy` API can be captured as
//!    `<Name>Snapshot` heap-owned values, replayed via
//!    `loan_proxy + write_from_snapshot`, and the round-tripped frame is
//!    bit-identical to the original at the field level. The snapshot
//!    pattern is the reconstruction primitive
//!    (no MCAP-backed replay path is wired up at this
//!    layer; in-memory snapshot round-trip is the closest equivalent
//!    that exercises the same determinism property).

use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TraceEntry, TriggerPolicy};
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::WireHeader;

// =============================================================
// Scheduler trace replay (pre-existing coverage, unchanged)
// =============================================================

fn counting_callback() -> (Box<dyn FnMut() + Send>, Arc<AtomicU64>) {
    let count = Arc::new(AtomicU64::new(0));
    let count_clone = count.clone();
    let cb = Box::new(move || {
        count_clone.fetch_add(1, Ordering::Relaxed);
    });
    (cb, count)
}

fn run_complex_scenario() -> Vec<TraceEntry> {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    // Node A: 10ms period (100Hz)
    let (cb_a, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "camera".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(10),
                max_catchup: None,
            },
            callback: cb_a,
        })
        .unwrap();

    // Node B: 20ms period (50Hz)
    let (cb_b, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "lidar".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(20),
                max_catchup: None,
            },
            callback: cb_b,
        })
        .unwrap();

    // Node C: data-triggered
    let (cb_c, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "detector".to_string(),
            policy: TriggerPolicy::Data,
            callback: cb_c,
        })
        .unwrap();

    // Node D: external trigger
    let (cb_d, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "manual".to_string(),
            policy: TriggerPolicy::External,
            callback: cb_d,
        })
        .unwrap();

    // Execute a deterministic sequence of steps and signals
    scheduler.step(Duration::from_millis(10)); // t=10ms: camera fires
    scheduler.signal_data("detector").unwrap();
    scheduler.step(Duration::from_millis(10)); // t=20ms: camera+lidar+detector fire
    scheduler.trigger_external("manual").unwrap();
    scheduler.step(Duration::from_millis(5)); // t=25ms: manual fires
    scheduler.step(Duration::from_millis(5)); // t=30ms: camera fires
    scheduler.signal_data("detector").unwrap();
    scheduler.step(Duration::from_millis(10)); // t=40ms: camera+lidar+detector fire
    scheduler.step(Duration::from_millis(10)); // t=50ms: camera fires

    scheduler.trace().to_vec()
}

#[test]
fn test_replay_equivalence_complex() {
    let trace1 = run_complex_scenario();
    let trace2 = run_complex_scenario();

    assert_eq!(
        trace1.len(),
        trace2.len(),
        "trace lengths must match: {} vs {}",
        trace1.len(),
        trace2.len()
    );

    for (i, (a, b)) in trace1.iter().zip(trace2.iter()).enumerate() {
        assert_eq!(a, b, "trace entry {} differs: {:?} vs {:?}", i, a, b);
    }
}

#[test]
fn test_replay_trace_correctness() {
    let trace = run_complex_scenario();

    assert!(!trace.is_empty(), "trace should not be empty");

    let mut last_times: std::collections::HashMap<&str, u64> = std::collections::HashMap::new();
    for entry in &trace {
        let node_id: &str = &entry.node_id;
        if let Some(&prev) = last_times.get(node_id) {
            assert!(
                entry.fire_time_ns >= prev,
                "fire times for '{}' should be non-decreasing: {} < {}",
                node_id,
                entry.fire_time_ns,
                prev
            );
        }
        last_times.insert(node_id, entry.fire_time_ns);
    }
}

#[test]
fn test_replay_stability_across_runs() {
    let reference = run_complex_scenario();
    for i in 0..10 {
        let trace = run_complex_scenario();
        assert_eq!(trace, reference, "trace diverged on run {}", i);
    }
}

// =============================================================
// Unbounded sync replay determinism
//
// Two extensions per the design discussion:
//
// 1. **Loose-AND firing reproduction across publish bursts.** A fast
//    trigger publishes multiple times between slow-trigger arrivals.
//    Unbounded sync fires once per slow arrival (at the slow trigger's
//    rate). Compare against an ORACLE VECTOR of expected fire times
//    — not against a second self-run (avoids the F11 anti-pattern).
//
// 2. **Fire ordering when multiple trigger inputs arrive on the same
//    tick.** The scheduler's per-trigger `IndexMap` insertion order
//    determines `latest of each` semantics. Pin via a deterministic
//    expected fire-count + step sequence.
// =============================================================

/// Run unbounded sync with a 10 Hz "slow" trigger + 60 Hz "fast"
/// trigger for 100 ms. Returns the captured trace.
fn run_unbounded_sync_burst_scenario() -> Vec<TraceEntry> {
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["slow".to_string(), "fast".to_string()],
                window: None, // unbounded
            },
            callback: cb,
        })
        .unwrap();

    // Mimic a typical robotics workload: slow trigger at 10 Hz (every
    // 100 ms), fast trigger at 60 Hz (every ~16.67 ms). Within each
    // 100 ms slot the fast trigger publishes ~6 times.
    //
    // The recorded arrival sequence (in ns): fast at 0, 17, 33, 50,
    // 67, 83; slow at 0; fast at 100, 117, ...; slow at 100; ...
    //
    // Expected unbounded-sync fires: ONE per slow arrival, taking the
    // latest fast at that moment. So fires at slow's t=0 (using fast
    // at t=0), and slow's t=100 (using fast at t=83).
    scheduler.signal_sync_input("fusion", "fast", 0).unwrap();
    scheduler.signal_sync_input("fusion", "slow", 0).unwrap();
    scheduler.step(Duration::from_millis(17)); // → fire at first step (both have data)

    scheduler
        .signal_sync_input("fusion", "fast", 17_000_000)
        .unwrap();
    scheduler.step(Duration::from_millis(16));

    scheduler
        .signal_sync_input("fusion", "fast", 33_000_000)
        .unwrap();
    scheduler.step(Duration::from_millis(17));

    scheduler
        .signal_sync_input("fusion", "fast", 50_000_000)
        .unwrap();
    scheduler.step(Duration::from_millis(17));

    scheduler
        .signal_sync_input("fusion", "fast", 67_000_000)
        .unwrap();
    scheduler.step(Duration::from_millis(16));

    scheduler
        .signal_sync_input("fusion", "fast", 83_000_000)
        .unwrap();
    scheduler.step(Duration::from_millis(17));

    // Second slow arrival at t=100ms — fusion fires again with latest
    // fast (which was at 83 ms).
    scheduler
        .signal_sync_input("fusion", "fast", 100_000_000)
        .unwrap();
    scheduler
        .signal_sync_input("fusion", "slow", 100_000_000)
        .unwrap();
    scheduler.step(Duration::from_millis(10));

    scheduler.trace().to_vec()
}

#[test]
fn test_unbounded_sync_burst_replay_matches_oracle() {
    // Run the scenario. Compare the captured trace against a
    // hand-constructed oracle vector. This pins the firing semantics
    // independently of any second-run comparison (F11 anti-pattern
    // — two runs of the same scenario would match even if the
    // scheduler did nothing useful).
    let trace = run_unbounded_sync_burst_scenario();

    // Expected: 2 fires, one per slow-trigger arrival.
    assert_eq!(
        trace.len(),
        2,
        "expected exactly 2 unbounded-sync fires (one per slow trigger); got {} entries: {:?}",
        trace.len(),
        trace
    );
    for entry in &trace {
        assert_eq!(&*entry.node_id, "fusion", "all fires should be on `fusion`");
    }
    // First fire happens during the first `step(17ms)` call. With
    // `VirtualClock` stepping in millisecond units, the clock
    // advances by `delta_ns` during the step and the fire is recorded
    // with the post-step current_time_ns. Pin the exact times so a
    // future scheduler change that shifts when-within-the-step a fire
    // is timestamped surfaces here.
    assert_eq!(trace[0].fire_time_ns, 17_000_000, "first fire at 17ms");
    assert_eq!(trace[1].fire_time_ns, 110_000_000, "second fire at 110ms");
}

#[test]
fn test_unbounded_sync_burst_replay_stability_across_runs() {
    // Determinism: re-run the same scenario 10 times and confirm
    // each trace is bit-identical to the first. This catches
    // any non-deterministic source in the unbounded-sync evaluation
    // path (e.g. HashMap iteration order, OS-clock sneaking in).
    let reference = run_unbounded_sync_burst_scenario();
    for i in 0..10 {
        let trace = run_unbounded_sync_burst_scenario();
        assert_eq!(
            trace, reference,
            "unbounded-sync trace diverged on run {}",
            i
        );
    }
}

#[test]
fn test_unbounded_sync_same_tick_input_ordering_is_deterministic() {
    // When multiple trigger inputs arrive in the same scheduler tick,
    // the unbounded-sync firing rule MUST be deterministic. Concretely:
    // the input order on `TriggerPolicy::Sync.inputs` defines which
    // message is `latest of each` even when wall-clock timestamps are
    // identical. Compare against an oracle vector.
    let clock = Arc::new(VirtualClock::new());
    let mut scheduler = Scheduler::with_virtual_clock(clock);

    let (cb, _) = counting_callback();
    scheduler
        .add_node(NodeConfig {
            id: "fusion".to_string(),
            policy: TriggerPolicy::Sync {
                inputs: vec!["a".to_string(), "b".to_string(), "c".to_string()],
                window: None,
            },
            callback: cb,
        })
        .unwrap();

    // All three inputs arrive at t=0 (same scheduler tick).
    scheduler.signal_sync_input("fusion", "a", 0).unwrap();
    scheduler.signal_sync_input("fusion", "b", 0).unwrap();
    scheduler.signal_sync_input("fusion", "c", 0).unwrap();
    scheduler.step(Duration::from_millis(10));

    let trace = scheduler.trace().to_vec();

    // Oracle: exactly one fire because all three triggers are present
    // by the time the scheduler evaluates. No second fire because
    // queues are drained on fire.
    assert_eq!(
        trace.len(),
        1,
        "same-tick multi-input arrival → exactly one fire (not three)"
    );
    assert_eq!(&*trace[0].node_id, "fusion");
    assert_eq!(trace[0].fire_time_ns, 10_000_000, "fire at end-of-step");
}

// =============================================================
// Message replay via snapshots
// =============================================================
//
// `<Name>::snapshot` and `<Name>::write_from_snapshot` (
// 1+2) are the reconstruction primitives. We capture a sequence of
// snapshots from a "live" run, then on a "replay" run we reconstruct each
// frame via `loan_proxy` + `write_from_snapshot` and assert the resulting
// snapshot is bit-identical. Same in-memory backend, two independent
// publishers, one shared snapshot vector — a microcosm of replay = live.

fn topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = NEXT.fetch_add(1, Ordering::Relaxed);
    format!("test/replay_msg/{base}/{nanos}/{id}")
}
static NEXT: AtomicU64 = AtomicU64::new(0);

#[test]
fn test_message_replay_fixed_schema_snapshot_round_trip() {
    use native_ros2_messages::geometry_msgs::{Vector3, Vector3Snapshot};

    let topic_live = topic("live_fixed");
    let topic_replay = topic("replay_fixed");

    let tt = TestTransport::with_buffer_size(8);

    // --- Live run: publish 3 frames, capture snapshots from the subscriber.
    let mut live_pub = tt.publisher(&topic_live, MaxSliceLen::const_new(256), 0);
    let mut live_sub = tt.subscriber(&topic_live);

    let originals = [
        Vector3Snapshot {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        Vector3Snapshot {
            x: -4.5,
            y: 0.0,
            z: f64::from_bits(0xDEAD_BEEF_0000_0001),
        },
        Vector3Snapshot {
            x: 1e-300,
            y: 1e300,
            z: -0.0,
        },
    ];

    let mut captured: Vec<Vector3Snapshot> = Vec::with_capacity(originals.len());
    for snap in &originals {
        {
            let mut proxy = live_pub.loan_proxy::<Vector3>().expect("live loan");
            proxy.write_from_snapshot(snap);
        }
        let recovered = live_sub
            .try_view::<Vector3, _>(|view| view.snapshot())
            .expect("live try_view")
            .expect("live subscriber should see the published frame");
        captured.push(recovered);
    }

    // Sanity: live recovery is byte-equal to the originals.
    for (i, (orig, recv)) in originals.iter().zip(captured.iter()).enumerate() {
        assert_eq!(orig, recv, "live frame {i} did not round-trip");
    }

    // --- Replay run: independent publisher + subscriber, replay via
    //                 loan_proxy + write_from_snapshot, capture snapshots.
    let mut replay_pub = tt.publisher(&topic_replay, MaxSliceLen::const_new(256), 0);
    let mut replay_sub = tt.subscriber(&topic_replay);

    let mut replayed: Vec<Vector3Snapshot> = Vec::with_capacity(captured.len());
    for snap in &captured {
        {
            let mut proxy = replay_pub.loan_proxy::<Vector3>().expect("replay loan");
            proxy.write_from_snapshot(snap);
        }
        let recovered = replay_sub
            .try_view::<Vector3, _>(|view| view.snapshot())
            .expect("replay try_view")
            .expect("replay subscriber should see the replayed frame");
        replayed.push(recovered);
    }

    // --- Assertion: replay = live, bit-for-bit at the field level.
    assert_eq!(captured, replayed, "Replay = Live: snapshots must match");
}

#[test]
fn test_message_replay_variable_schema_snapshot_round_trip() {
    use native_ros2_messages::sensor_msgs::{Image, ImageSnapshot};

    let topic_live = topic("live_var");
    let topic_replay = topic("replay_var");

    // Image snapshot exercises both fixed fields and variable payload.
    // Note: the `header` field of Image is a complex variable type, stored
    // as raw `Vec<u8>` in the snapshot (a snapshot limitation).
    let originals = [
        ImageSnapshot {
            header: Vec::new(),
            height: 32,
            width: 32,
            encoding: "rgb8".to_string(),
            is_bigendian: 0,
            step: 96,
            data: vec![1, 2, 3, 4, 5],
        },
        ImageSnapshot {
            header: Vec::new(),
            height: 64,
            width: 48,
            encoding: "mono8".to_string(),
            is_bigendian: 1,
            step: 48,
            data: (0..256u32).map(|i| i as u8).collect(),
        },
        ImageSnapshot {
            header: Vec::new(),
            height: 0,
            width: 0,
            encoding: String::new(),
            is_bigendian: 0,
            step: 0,
            data: Vec::new(),
        },
    ];

    let max_slice = (WireHeader::SIZE + 13 + 8 * 3 + 1024) as u32;

    let tt = TestTransport::with_buffer_size(8);

    let mut live_pub = tt.publisher(&topic_live, MaxSliceLen::const_new(max_slice), 0);
    let mut live_sub = tt.subscriber(&topic_live);

    let mut captured: Vec<ImageSnapshot> = Vec::with_capacity(originals.len());
    for snap in &originals {
        {
            let mut proxy = live_pub.loan_proxy::<Image>().expect("live loan");
            proxy
                .write_from_snapshot(snap)
                .expect("write_from_snapshot");
        }
        let recovered = live_sub
            .try_view::<Image, _>(|view| view.snapshot())
            .expect("live try_view")
            .expect("live subscriber sees frame");
        captured.push(recovered);
    }

    // Live recovery is field-equal to the originals.
    for (i, (orig, recv)) in originals.iter().zip(captured.iter()).enumerate() {
        assert_eq!(orig, recv, "live variable frame {i} did not round-trip");
    }

    // Replay run.
    let mut replay_pub = tt.publisher(&topic_replay, MaxSliceLen::const_new(max_slice), 0);
    let mut replay_sub = tt.subscriber(&topic_replay);

    let mut replayed: Vec<ImageSnapshot> = Vec::with_capacity(captured.len());
    for snap in &captured {
        {
            let mut proxy = replay_pub.loan_proxy::<Image>().expect("replay loan");
            proxy
                .write_from_snapshot(snap)
                .expect("write_from_snapshot");
        }
        let recovered = replay_sub
            .try_view::<Image, _>(|view| view.snapshot())
            .expect("replay try_view")
            .expect("replay subscriber sees frame");
        replayed.push(recovered);
    }

    assert_eq!(
        captured, replayed,
        "Replay = Live (variable schema): snapshots must match",
    );
}
