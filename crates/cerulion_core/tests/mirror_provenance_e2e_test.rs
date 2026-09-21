// SPDX-License-Identifier: AGPL-3.0-only
//! The mirror-provenance registration e2e — a desk re-injector
//! that mirrors a remote topic into local SHM registers `(canonical topic, origin
//! robot)` over the `/__cerulion/mirrors` control service, and a separate reader
//! (the `topic list` process) gathers the live snapshot.
//!
//! Cribs `ingress_injection_seam_test.rs` / `network_ingress_test.rs`: REAL
//! iceoryx2 over PER-TEST SHM roots. Two `TransportManager`s share ONE root — a
//! WRITER (the re-injector: creates the real local-SHM mirror via
//! `create_ingress_injector`, then `register_mirror_provenance`) and a READER (the
//! `topic list` process: `gather_mirror_provenance`). Both are `network: None`
//! (the registration path opens NO zenoh session). Hand oracles (never a
//! self-compare). Parallel-safe (per-test SHM roots; NO `#[serial]`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::mirror_registry::MirrorRecord;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

/// A manager on the supplied shared SHM root (so a writer + reader can meet).
fn manager_on(root: &iceoryx2::config::Config, name: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("mirror_prov_{name}"),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init_for_test")
}

/// Gather until the reader observes `want` (a live writer republishes every
/// ~150ms; the in-process pub↔sub connection takes a pass or two), or fail loudly
/// after a bounded wall budget. Returns the observed snapshot.
fn gather_until(
    reader: &TransportManager,
    want: &[MirrorRecord],
    budget: Duration,
) -> Vec<MirrorRecord> {
    let deadline = Instant::now() + budget;
    let mut last = Vec::new();
    while Instant::now() < deadline {
        last = reader
            .gather_mirror_provenance(Duration::from_millis(250))
            .expect("gather");
        if last == want {
            return last;
        }
    }
    last
}

/// The headline e2e: a re-injector creates a real local mirror + registers its
/// provenance; a SEPARATE reader on the shared root gathers exactly the
/// `(topic → robot)` record (hand oracle). Determinism: a second gather is
/// identical.
#[test]
fn reinjector_registers_provenance_reader_gathers_it() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let topic = format!("/utlidar/robot_odom_{id}");
    let robot = "ubuntu";

    let writer = manager_on(&root, &format!("writer_{id}"));
    let reader = manager_on(&root, &format!("reader_{id}"));

    // The actual re-inject flow: create the local-SHM mirror publisher, THEN
    // record its provenance (the vizd / connectd shape).
    let _injector = writer
        .create_ingress_injector(&topic, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("create the local mirror publisher");
    assert!(
        writer
            .register_mirror_provenance(&topic, robot)
            .expect("register provenance"),
        "the first registration is genuinely new"
    );

    let want = vec![MirrorRecord {
        topic: topic.clone(),
        origin_robot: robot.to_string(),
    }];
    let got = gather_until(&reader, &want, Duration::from_secs(5));
    assert_eq!(
        got, want,
        "the reader gathers exactly the registered provenance"
    );

    // Determinism: a REPEATED bounded gather returns the IDENTICAL snapshot
    // (content identity, not single-shot timing). This must use the SAME
    // bounded-retry oracle as the first gather — a single 250ms shot pins
    // "the writer's 150ms republish thread lands a record within one 250ms
    // window", which is a shared-VM timing property, NOT content identity: a
    // fresh reader has no buffered history at connect (iceoryx2 0.9.1
    // late-joiner delivery needs the publisher's next republish tick), so on a
    // loaded macOS CI VM a descheduled republish thread misses the window and
    // returns empty. Production `topic list` uses the 600ms
    // `MIRROR_GATHER_WINDOW`; the test's 250ms-per-attempt + 5s retry budget
    // converges to the same hand oracle deterministically (early-exits fast
    // here since the writer is still live and republishing).
    let again = gather_until(&reader, &want, Duration::from_secs(5));
    assert_eq!(
        again, want,
        "a repeated bounded gather yields the identical snapshot"
    );
}

/// EXPIRE-ON-DROP over real cross-manager SHM: the reader sees the provenance
/// while the writer is live, then sees NOTHING once the writer (its registry +
/// republish pump) drops — the mirror's provenance has expired (the no-publisher
/// fast path returns empty).
#[test]
fn provenance_expires_when_the_reinjector_drops() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let topic = format!("/lf/lowstate_{id}");
    let robot = "orin";

    let reader = manager_on(&root, &format!("reader_{id}"));
    let writer = manager_on(&root, &format!("writer_{id}"));
    let _injector = writer
        .create_ingress_injector(&topic, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("create the local mirror publisher");
    writer
        .register_mirror_provenance(&topic, robot)
        .expect("register provenance");

    let want = vec![MirrorRecord {
        topic: topic.clone(),
        origin_robot: robot.to_string(),
    }];
    assert_eq!(
        gather_until(&reader, &want, Duration::from_secs(5)),
        want,
        "the reader sees the live mirror's provenance"
    );

    // Drop the re-injector (process/writer exit): registry + pump + mirror
    // publisher all torn down.
    drop(_injector);
    drop(writer);

    // Now a fresh gather must converge to EMPTY (bounded retry for iceoryx2 port
    // cleanup after the drop).
    let empty = gather_until(&reader, &[], Duration::from_secs(5));
    assert!(
        empty.is_empty(),
        "after the re-injector drops, the mirror's provenance has expired: {empty:?}"
    );
}

/// The no-mirror desk fast path: a reader on a root with NO live mirror writer
/// gathers EMPTY (the `number_of_publishers == 0` instant return) AND pays no
/// gather window. Hand oracle + timing bound: even with a large 2 s window
/// requested, the no-publisher fast path returns well under 100 ms (a generous CI
/// margin — the real cost is a service open + one dynamic-config read).
#[test]
fn no_mirror_desk_gathers_empty_instantly() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let reader = manager_on(&root, &format!("reader_{id}"));
    let started = Instant::now();
    let got = reader
        .gather_mirror_provenance(Duration::from_secs(2))
        .expect("gather");
    let elapsed = started.elapsed();
    assert!(
        got.is_empty(),
        "a desk with no mirror gathers nothing: {got:?}"
    );
    assert!(
        elapsed < Duration::from_millis(100),
        "the no-publisher fast path must not pay the gather window (elapsed {elapsed:?}, \
         requested 2s window)"
    );
}

/// Two topics from ONE re-injector (a robot streaming several topics) both gather,
/// distinctly attributed. Hand oracle over the sorted snapshot.
#[test]
fn two_mirrored_topics_from_one_reinjector_both_gather() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let topic_a = format!("/utlidar/robot_odom_{id}");
    let topic_b = format!("/lf/lowstate_{id}");
    let robot = "ubuntu";

    let writer = manager_on(&root, &format!("writer_{id}"));
    let reader = manager_on(&root, &format!("reader_{id}"));
    let _inj_a = writer
        .create_ingress_injector(&topic_a, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("mirror a");
    let _inj_b = writer
        .create_ingress_injector(&topic_b, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("mirror b");
    writer
        .register_mirror_provenance(&topic_a, robot)
        .expect("register a");
    writer
        .register_mirror_provenance(&topic_b, robot)
        .expect("register b");

    // gather_from_node dedups + returns sorted by canonical topic; `/lf/...` sorts
    // before `/utlidar/...`.
    let mut want = vec![
        MirrorRecord {
            topic: topic_b.clone(),
            origin_robot: robot.to_string(),
        },
        MirrorRecord {
            topic: topic_a.clone(),
            origin_robot: robot.to_string(),
        },
    ];
    want.sort_by(|x, y| x.topic.cmp(&y.topic));
    assert_eq!(
        gather_until(&reader, &want, Duration::from_secs(5)),
        want,
        "both mirrored topics gather, distinctly attributed"
    );
}

/// The public API refuses an empty origin robot LOUDLY (a mirror must be
/// attributed) and an over-long robot identity — the validation floor at the
/// registration entry. Hand oracle on the error class.
#[test]
fn register_mirror_provenance_refuses_empty_and_over_long_robot() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let id = unique_id();
    let writer = manager_on(&root, &format!("writer_{id}"));
    let topic = format!("/x_{id}");

    let empty = writer.register_mirror_provenance(&topic, "");
    assert!(
        empty.is_err(),
        "an empty origin robot is refused (a mirror must be attributed)"
    );
    let over_long = "r".repeat(1024);
    assert!(
        writer
            .register_mirror_provenance(&topic, &over_long)
            .is_err(),
        "an over-long robot identity is refused before it enters the map"
    );
    // A well-formed registration still succeeds after the refusals (the mutex was
    // not poisoned by the validation Errs — they return before touching it).
    assert!(writer
        .register_mirror_provenance(&topic, "ubuntu")
        .expect("valid registration succeeds"));
}
