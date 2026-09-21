// SPDX-License-Identifier: AGPL-3.0-only
//! The ingress-build PROGRESS channel over REAL iceoryx2.
//!
//! The recorder-side policy is oracle-tested in `cerulion_bagd::ingress_hold`
//! and driven end to end in `cerulion_bagd/tests/ingress_build_e2e_test.rs`.
//! What lives HERE is the wire half — the properties that are about transport
//! rather than about a recording, and that the recorder's arms cannot see:
//!
//! * the PRODUCING seam is real. Progress is broadcast by
//!   [`TransportManager::create_ingress_publisher`] itself, so every bridge —
//!   including one already vendored into a user's workspace — reports without
//!   any change of its own. An inert seam fails the first arm.
//! * the REPUBLISH BELT is load-bearing. A reader that opens AFTER routes were
//!   created must still learn the current total; the channel requests no
//!   late-joiner history, so only the periodic re-send can deliver it. Nothing
//!   in the recorder's arms distinguishes a working belt from a missing one,
//!   because there the reader is always up first.
//! * a MALFORMED frame is counted and skipped, never wedging the drain.
//! * two producing PROCESSES fold into one machine-wide total.
//!
//! Per-test SHM roots via `init_for_test`, so these are parallel-safe and carry
//! no `#[serial]`.

use std::time::{Duration, Instant};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::ingress_build::{
    encode_record, IngressBuildProgress, INGRESS_BUILD_RECORD_LEN, INGRESS_BUILD_REPUBLISH_INTERVAL,
};
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{TransportConfig, VirtualClock};

/// A liveness ceiling for every convergence loop below. Stated in seconds, not
/// in units of the interval under test — a bound that scales with the thing
/// being measured is the wall-in-its-own-units mistake.
const CONVERGE_DEADLINE: Duration = Duration::from_secs(10);

fn manager(name: &str) -> std::sync::Arc<TransportManager> {
    let config = TransportConfig {
        node_name: name.into(),
        clock: std::sync::Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 16,
        network: None,
    };
    TransportManager::init_for_test(config, iceoryx_test_config()).expect("init_for_test")
}

/// A process-unique canonical topic.
fn topic(base: &str) -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "/{base}/{}/{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// Drain until `want` holds or the deadline passes; returns whether it held.
fn converge(
    reader: &mut cerulion_core::transport::ingress_build::IngressBuildReader,
    progress: &mut IngressBuildProgress,
    want: impl Fn(&IngressBuildProgress) -> bool,
) -> bool {
    let start = Instant::now();
    loop {
        reader.drain_into(progress);
        if want(progress) {
            return true;
        }
        if start.elapsed() >= CONVERGE_DEADLINE {
            return false;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Creating a runtime ingress route BROADCASTS progress — from the production
/// seam, with no bridge-side call of any kind.
///
/// This is the no-inert-shipping arm. If the report were dropped from
/// `create_ingress_publisher`, or moved somewhere only a future bridge would
/// call, this fails — and every already-vendored `dds_bridge` would go on
/// silently racing the recorder.
#[test]
fn creating_runtime_ingress_routes_broadcasts_the_running_count() {
    let mgr = manager("broadcast");
    let mut reader = mgr
        .create_ingress_build_reader()
        .expect("open progress reader");
    let mut progress = IngressBuildProgress::new();

    assert_eq!(
        mgr.ingress_routes_created(),
        0,
        "a manager that has bridged nothing must have opened no channel and counted nothing"
    );

    let mut keep = Vec::new();
    for i in 0..3 {
        keep.push(
            mgr.create_ingress_publisher(&topic(&format!("route{i}")), MaxSliceLen::const_new(64))
                .expect("create_ingress_publisher"),
        );
    }
    assert_eq!(mgr.ingress_routes_created(), 3);

    assert!(
        converge(&mut reader, &mut progress, |p| p.total_routes() == 3),
        "the reader must converge to the producer's running total, got {}",
        progress.total_routes()
    );
    assert_eq!(progress.writers_seen(), 1, "one producing process");
    // The fold's baseline rule means the FIRST record heard is banked, so at
    // most two of the three can be advances — a floor, since delivery may
    // coalesce.
    assert!(progress.advances() <= 2, "a baseline is never an advance");

    // The writer identity a reader folds by is the one this manager reports.
    assert!(
        mgr.ingress_build_writer_id().is_some(),
        "the channel is open, so its identity is observable (Principle #3)"
    );
    assert!(
        mgr.ingress_build_pump_active(),
        "the republish belt starts on the first route"
    );
}

/// A creation-triggered record reports ~0 recency; a belt rebroadcast reports a
/// GROWING one. That difference is the whole cold-boot discriminator.
///
/// A receiver cannot compute this: by value and by timing, a settled plane's
/// republished `98` is indistinguishable from a plane that just created its 98th
/// route. The WRITER can, because it knows when it last created something — so
/// it says so, and one record is enough to answer "is anybody still building?".
///
/// This arm is what stops the belt from becoming a re-send of the previous
/// frame's BYTES: the recency has to be recomputed at send time, on whichever
/// thread is sending, or a rebroadcast would keep claiming the freshness of the
/// creation that preceded it — forever.
#[test]
fn a_creation_reports_zero_recency_while_the_belt_reports_a_growing_one() {
    let mgr = manager("recency");
    let mut reader = mgr
        .create_ingress_build_reader()
        .expect("open progress reader");
    let mut progress = IngressBuildProgress::new();

    let _route = mgr
        .create_ingress_publisher(&topic("recency"), MaxSliceLen::const_new(64))
        .expect("create_ingress_publisher");

    // THE creation-triggered frame — the FIRST record this writer ever sends,
    // delivered synchronously inside `create_ingress_publisher` a full republish
    // interval before the belt's first tick. It must ALREADY carry the claim:
    // asking the first frame rather than "whichever frame first carries one" is
    // load-bearing, because a writer that stamped its creation AFTER sending
    // would emit route 1's frame with the never-created sentinel and be rescued
    // 250 ms later by the belt — masking, on the cold-boot path, exactly the
    // arming delay this field exists to remove.
    let first_pass = drain_first_record(&mut reader, &mut progress);
    let first = first_pass.freshest_creation.expect(
        "route 1's OWN frame must carry a recency claim — a writer that stamps its creation after \
         sending leaves the first frame saying 'never created', so nothing arms until the belt \
         catches up (or, if the belt failed to spawn, until route 2)",
    );
    assert!(
        first < INGRESS_BUILD_REPUBLISH_INTERVAL,
        "a creation-triggered record must report ~0 recency, got {first:?}"
    );

    // Now let the belt run WITHOUT creating anything, and require the reported
    // recency to have GROWN past a full interval.
    std::thread::sleep(INGRESS_BUILD_REPUBLISH_INTERVAL * 4);
    let later = drain_first_record(&mut reader, &mut progress)
        .freshest_creation
        .expect("a belt rebroadcast carries the claim too");
    assert!(
        later > first && later >= INGRESS_BUILD_REPUBLISH_INTERVAL,
        "a belt rebroadcast must report a GROWING recency (first {first:?}, later {later:?}) — \
         if it re-sent the creation frame's bytes it would claim to be fresh forever, and a \
         settled robot would hold every recorder that ever armed against it"
    );
    // ...and the total did NOT move, so recency is carrying information the
    // count cannot.
    assert_eq!(progress.total_routes(), 1);
    assert_eq!(progress.advances(), 0);
}

/// Drain until a pass folds at least one record; return THAT pass's outcome.
///
/// Deliberately keyed on `folded`, not on `freshest_creation`: a helper that
/// skipped records carrying no claim would silently wait for a later frame and
/// hide a writer whose FIRST frame said nothing.
fn drain_first_record(
    reader: &mut cerulion_core::transport::ingress_build::IngressBuildReader,
    progress: &mut IngressBuildProgress,
) -> cerulion_core::transport::ingress_build::IngressBuildDrain {
    let start = Instant::now();
    loop {
        let out = reader.drain_into(progress);
        if out.folded > 0 {
            return out;
        }
        assert!(
            start.elapsed() < CONVERGE_DEADLINE,
            "no record arrived at all"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// A reader that opens AFTER the routes were created still learns the total.
///
/// THE republish-belt arm. The service requests no late-joiner history, so a
/// subscriber attaching after a send receives nothing from that send — only the
/// periodic re-send can deliver the current total. Nothing is created after the
/// reader opens, so a converged total is proof the belt ran.
///
/// This is what makes the recorder's BASELINE rule work on a settled robot: the
/// recorder hears the standing total, banks it, and correctly declines to treat
/// it as motion. Without the belt it would hear nothing at all and would then
/// misread the NEXT route as a first sighting rather than an advance.
#[test]
fn a_reader_that_opens_late_still_learns_the_standing_total() {
    let mgr = manager("late_reader");
    let mut keep = Vec::new();
    for i in 0..2 {
        keep.push(
            mgr.create_ingress_publisher(&topic(&format!("pre{i}")), MaxSliceLen::const_new(64))
                .expect("create_ingress_publisher"),
        );
    }
    assert_eq!(mgr.ingress_routes_created(), 2);

    // Opened AFTER every send. Only the belt can serve it.
    let mut reader = mgr
        .create_ingress_build_reader()
        .expect("open progress reader");
    let mut progress = IngressBuildProgress::new();
    assert!(
        converge(&mut reader, &mut progress, |p| p.total_routes() == 2),
        "a late reader must converge within a few republish intervals \
         ({INGRESS_BUILD_REPUBLISH_INTERVAL:?}); got {}",
        progress.total_routes()
    );
    assert_eq!(
        progress.advances(),
        0,
        "everything this reader heard was a re-send of a total that never moved — a settled \
         plane must produce NO motion, or every recording against a built robot would be held"
    );
}

/// A malformed frame is counted and skipped; the drain keeps working.
///
/// The channel is a dedicated control service that should never carry foreign
/// frames, so a frame that does not decode is evidence of something wrong and
/// must be visible (Principle #3) — and must not be able to wedge the reader
/// that a recording's coverage depends on.
#[test]
fn a_malformed_frame_is_counted_and_skipped_without_wedging_the_drain() {
    let mgr = manager("malformed");
    let mut reader = mgr
        .create_ingress_build_reader()
        .expect("open progress reader");
    let mut progress = IngressBuildProgress::new();
    assert_eq!(reader.malformed(), 0);

    // Three shapes the decoder must refuse: wrong length, bad magic, bad
    // version. Sent through the real publisher, so these are real wire frames.
    let good = encode_record(7, 1, 0);
    let mut bad_magic = good;
    bad_magic[1] ^= 0xFF;
    let mut bad_version = good;
    bad_version[2] = bad_version[2].wrapping_add(1);
    for bytes in [
        &good[..INGRESS_BUILD_RECORD_LEN - 1],
        &bad_magic[..],
        &bad_version[..],
    ] {
        assert!(
            mgr.send_raw_ingress_build_for_test(bytes)
                .expect("raw send"),
            "the harness must actually put the frame on the wire"
        );
    }

    let start = Instant::now();
    while reader.malformed() < 3 && start.elapsed() < CONVERGE_DEADLINE {
        reader.drain_into(&mut progress);
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(reader.malformed(), 3, "every bad frame must be COUNTED");
    assert_eq!(
        progress.writers_seen(),
        0,
        "and none of them may enter the fold — a corrupt frame must not invent a producer"
    );

    // The drain still works afterwards: a real route lands.
    let _route = mgr
        .create_ingress_publisher(&topic("after_bad"), MaxSliceLen::const_new(64))
        .expect("create_ingress_publisher");
    assert!(
        converge(&mut reader, &mut progress, |p| p.total_routes() == 1),
        "a malformed frame must not wedge the drain"
    );
}

/// Two producing PROCESSES fold into one machine-wide total.
///
/// A multi-process graph runs one worker per group, and each worker that opens
/// ingress routes writes its own record; a `cerulion-netd` sharing the machine
/// is a third. The recorder holds on the MACHINE's motion, so the fold has to
/// sum them and count them as distinct writers.
#[test]
fn two_producing_managers_fold_into_one_machine_wide_total() {
    // Distinct managers on the SAME SHM root, so this is two writers on one
    // service — the shape a multi-process graph produces.
    let ix = iceoryx_test_config();
    let make = |name: &str| {
        TransportManager::init_for_test(
            TransportConfig {
                node_name: name.into(),
                clock: std::sync::Arc::new(VirtualClock::new()),
                subscriber_buffer_size: 16,
                network: None,
            },
            ix.clone(),
        )
        .expect("init_for_test")
    };
    let a = make("fold_a");
    let b = make("fold_b");

    let mut reader = a.create_ingress_build_reader().expect("open reader");
    let mut progress = IngressBuildProgress::new();

    let mut keep = Vec::new();
    keep.push(
        a.create_ingress_publisher(&topic("fold_a0"), MaxSliceLen::const_new(64))
            .expect("a route"),
    );
    for i in 0..2 {
        keep.push(
            b.create_ingress_publisher(&topic(&format!("fold_b{i}")), MaxSliceLen::const_new(64))
                .expect("b route"),
        );
    }

    assert!(
        converge(&mut reader, &mut progress, |p| p.total_routes() == 3
            && p.writers_seen() == 2),
        "expected 3 routes across 2 writers, got {} across {}",
        progress.total_routes(),
        progress.writers_seen()
    );
    assert_ne!(
        a.ingress_build_writer_id(),
        b.ingress_build_writer_id(),
        "two processes must not fold into one entry — their counts would overwrite each other"
    );
}
