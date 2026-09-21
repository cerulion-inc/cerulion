// SPDX-License-Identifier: AGPL-3.0-only
//! Round-trip tests for the SHM-backed `OutputProxy<T>` / `InputView<T>` API.
//!
//! There is ONE transport backend, iceoryx2 (there is no in-process heap
//! backend). Every arm runs `CerulionPublisher::loan_proxy` → `OutputProxy::Drop`
//! (publish) → `CerulionSubscriber::try_view` → `InputView::Deref` to the
//! SHM-backed `<Name>Shm` reader. That one path is reached two ways:
//!
//! - via the process-global singleton `TransportManager` (the production path);
//! - via `TestTransport` — the same iceoryx2 ports on an isolated per-test SHM
//!   root. The two `*_in_process` test names below mean "same process, own SHM
//!   root", NOT a second backend.
//!
//! Both setups are exercised for a fixed-size schema (`Vector3`) and for a
//! variable-length schema (`std_msgs::String`) to validate:
//!
//! - The `WireHeader::total_size` is finalized correctly on drop.
//! - Variable-field writes land in the offset-table region and round-trip
//!   through the reader's typed accessor.
//! - The proxy drops without panicking even when the user never explicitly
//!   sends.
//!
//! These tests touch the iceoryx2 singleton, so the parent `cargo test`
//! invocation must run them with `--test-threads=1`.

use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, line_level};
use cerulion_core::wire::MaxSliceLen;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime};
use tracing_test::traced_test;

use cerulion_core::testing::TestTransport;
use cerulion_core::transport::TransportManager;

// Graph-level imports for the NodeHandle-observability e2e.
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{MacroPolicy, NodeEntry, NodeInfo, OutputMeta};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3 as ProducerVec3;
use native_ros2_messages::std_msgs::String as RosString;

/// Monotonic counter so parallel-process tests don't collide.
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/proxy/{}/{}/{}", base, nanos, id)
}

// ============================================================
// iceoryx2 round-trips
// ============================================================

#[test]
fn proxy_round_trip_fixed_schema_iceoryx2() {
    use native_ros2_messages::geometry_msgs::Vector3;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("fixed_iox2");
    // 32 (header) + 24 (Vector3) = 56; round up for safety.
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Loan a proxy and write fields directly into SHM via the SHM-backed type.
    {
        let mut proxy = publisher
            .loan_proxy::<Vector3>()
            .expect("loan_proxy should succeed for fixed schema");
        proxy.x = 1.5;
        proxy.y = 2.5;
        proxy.z = -3.5;
        // Drop publishes.
    }

    // Read back via the subscriber.
    let observed = subscriber
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("try_view should not error");
    let observed = observed.expect("subscriber should have one sample available");
    assert!((observed.0 - 1.5).abs() < f64::EPSILON);
    assert!((observed.1 - 2.5).abs() < f64::EPSILON);
    assert!((observed.2 - -3.5).abs() < f64::EPSILON);
}

#[test]
fn proxy_round_trip_variable_schema_iceoryx2() {
    use native_ros2_messages::std_msgs::String as RosString;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("var_iox2");
    // 32 (header) + 0 (fixed) + 8 (offset table) + 64 (room for the string) = 104.
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(1024))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    {
        let mut proxy = publisher
            .loan_proxy::<RosString>()
            .expect("loan_proxy should succeed for variable schema");
        proxy
            .set_data("hello proxy")
            .expect("set_data within max_slice_len");
    }

    let observed = subscriber
        .try_view::<RosString, _>(|view| view.data().expect("utf-8").to_owned())
        .expect("try_view should not error");
    let observed = observed.expect("subscriber should have one sample available");
    assert_eq!(observed, "hello proxy");
}

#[test]
fn proxy_drop_without_send_does_not_panic_iceoryx2() {
    use native_ros2_messages::geometry_msgs::Vector3;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("drop_no_panic");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");

    // Loan and drop without writing — for fixed schemas all_variables_written()
    // is vacuously true, so the message is published with default zero fields.
    drop(
        publisher
            .loan_proxy::<Vector3>()
            .expect("loan_proxy should succeed"),
    );
    // No panic. Sequence advances; subsequent loans still work.
    drop(
        publisher
            .loan_proxy::<Vector3>()
            .expect("second loan should succeed after the first drop"),
    );
}

// ============================================================
// Round-trips over an isolated per-test SHM root (`TestTransport`)
//
// Same iceoryx2 backend as above; the `_in_process` suffix means these
// run independently of the process-global singleton, on their own SHM root.
// ============================================================

#[test]
fn proxy_round_trip_fixed_schema_in_process() {
    use native_ros2_messages::geometry_msgs::Vector3;

    let topic = unique_topic("fixed_inproc");
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let mut subscriber = tt.subscriber(&topic);

    {
        let mut proxy = publisher
            .loan_proxy::<Vector3>()
            .expect("loan_proxy in-process should succeed");
        proxy.x = 7.0;
        proxy.y = 8.0;
        proxy.z = 9.0;
    }

    let observed = subscriber
        .try_view::<Vector3, _>(|view| (view.x, view.y, view.z))
        .expect("try_view should not error");
    let observed = observed.expect("in-process subscriber should have a sample");
    assert!((observed.0 - 7.0).abs() < f64::EPSILON);
    assert!((observed.1 - 8.0).abs() < f64::EPSILON);
    assert!((observed.2 - 9.0).abs() < f64::EPSILON);
}

#[test]
fn proxy_round_trip_variable_schema_in_process() {
    use native_ros2_messages::std_msgs::String as RosString;

    let topic = unique_topic("var_inproc");
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(1024), 0);
    let mut subscriber = tt.subscriber(&topic);

    {
        let mut proxy = publisher
            .loan_proxy::<RosString>()
            .expect("loan_proxy in-process should succeed");
        proxy
            .set_data("in-process round trip")
            .expect("set_data within max_slice_len");
    }

    let observed = subscriber
        .try_view::<RosString, _>(|view| view.data().expect("utf-8").to_owned())
        .expect("try_view should not error");
    let observed = observed.expect("in-process subscriber should have a sample");
    assert_eq!(observed, "in-process round trip");
}

// ============================================================
// Error paths
// ============================================================

#[test]
fn loan_proxy_rejects_undersized_buffer_for_variable_schema() {
    use cerulion_core::TransportError;
    use native_ros2_messages::std_msgs::String as RosString;

    let topic = unique_topic("undersized");
    // 32 (header) + 0 (fixed) + 8 (offset table) = 40 bytes minimum for std_msgs::String.
    // 32 is the smallest legal MaxSliceLen (newtype lower bound = WireHeader::SIZE);
    // 32 < 40 so loan_proxy must still reject as MaxSliceLenRequired.
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher(&topic, MaxSliceLen::const_new(32), 0);
    let result = publisher.loan_proxy::<RosString>();
    match result {
        Err(TransportError::MaxSliceLenRequired { .. }) => {}
        Err(other) => panic!("expected MaxSliceLenRequired, got error {:?}", other),
        Ok(_) => panic!("expected MaxSliceLenRequired, got Ok"),
    };
}

#[test]
fn try_view_rejects_schema_mismatch() {
    use cerulion_core::TransportError;
    use native_ros2_messages::geometry_msgs::{Point, Vector3};

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("schema_mismatch");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publish a Vector3 (fixed, 3 × f64).
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
    }

    // Try to read as Point (different schema → different SCHEMA_HASH).
    match subscriber.try_view::<Point, _>(|_view| ()) {
        Err(TransportError::SchemaMismatch { .. }) => {}
        other => panic!("expected SchemaMismatch, got {:?}", other),
    }
}

// ============================================================
// spin_view_until_seq
// ============================================================

#[test]
fn spin_view_until_seq_returns_first_matching_sample() {
    use native_ros2_messages::geometry_msgs::Vector3;
    use std::time::Duration;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("spin_match");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publisher's first COMMIT stamps WireHeader.sequence = 0 (the
    // sequence is consumed at commit, not loan).
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 4.0;
        proxy.y = 5.0;
        proxy.z = 6.0;
    }

    let observed = subscriber
        .spin_view_until_seq::<Vector3, _>(0, Duration::from_secs(1), |view| {
            (view.x, view.y, view.z)
        })
        .expect("spin_view_until_seq should not error");
    let observed = observed.expect("expected sample within timeout");
    assert!((observed.0 - 4.0).abs() < f64::EPSILON);
    assert!((observed.1 - 5.0).abs() < f64::EPSILON);
    assert!((observed.2 - 6.0).abs() < f64::EPSILON);
}

#[test]
fn spin_view_until_seq_skips_stale_samples() {
    use native_ros2_messages::geometry_msgs::Vector3;
    use std::time::Duration;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("spin_skip_stale");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Publish three samples; sequences 0, 1, 2.
    for i in 0..3 {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = i as f64;
        proxy.y = 0.0;
        proxy.z = 0.0;
    }

    // Ask for sequence >= 2 — older samples must be skipped, latest returned.
    let observed = subscriber
        .spin_view_until_seq::<Vector3, _>(2, Duration::from_secs(1), |view| view.x)
        .expect("spin_view_until_seq should not error");
    let observed = observed.expect("expected sample within timeout");
    assert!((observed - 2.0).abs() < f64::EPSILON);
}

#[test]
fn spin_view_until_seq_times_out_when_no_publisher() {
    use native_ros2_messages::geometry_msgs::Vector3;
    use std::time::Duration;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("spin_timeout");
    // Create publisher so the iceoryx2 service exists, but never publish.
    let _publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // 50 ms timeout — should return Ok(None), not error.
    let observed = subscriber
        .spin_view_until_seq::<Vector3, _>(0, Duration::from_millis(50), |_view| 42u32)
        .expect("spin_view_until_seq should not error on timeout");
    assert!(observed.is_none(), "expected timeout, got Some");
}

// ============================================================
// Sequence stamped at COMMIT, not loan — a loan that never
// publishes burns NO sequence number, so published streams are gap-free
// and downstream wire-seq gap detectors (bagd frames_lost, drop_oldest
// eviction) count only REAL losses.
// ============================================================

/// Drain every pending frame, returning `(sequence, x)` per frame in
/// arrival order — the wire-level oracle surface (exactly what bagd's
/// gap detector reads). `x` is Vector3's first fixed field at frame
/// bytes `[32..40]` (f64 LE, after the 32-byte WireHeader).
fn drain_seq_and_x(
    subscriber: &mut cerulion_core::transport::subscriber::CerulionSubscriber,
) -> Vec<(u32, f64)> {
    let mut out = Vec::new();
    // `try_receive` drains the whole queue (FIFO), handing each frame's parsed
    // WireHeader + body to the callback. `msg.payload()` is the body AFTER the
    // 32-byte header, so Vector3's first fixed field `x` is body bytes [0..8].
    subscriber
        .try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().expect("x field bytes"));
            out.push((msg.header().sequence, x));
        })
        .expect("try_receive");
    out
}

/// HEADLINE: commit → DISCARD (the macro early-exit
/// class, simulated via the same `__cer_defer_publish` hook the macro
/// preamble calls) → commit. The two PUBLISHED frames must carry
/// CONSECUTIVE sequences (0, 1 — no gap), and the discarded payload must
/// not appear.
///
/// Restoring a loan-time `fetch_add` in
/// `loan_proxy` (and reverting the Drop commit-stamp) makes this fail
/// with sequences `[0, 2]` — the exact phantom-gap signature bagd
/// mis-books as a lost frame.
#[test]
fn discarded_loan_burns_no_sequence_committed_stream_is_gapless() {
    use native_ros2_messages::geometry_msgs::Vector3;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("seq_commit_discard");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(256))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Commit #1 → sequence 0.
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan 1");
        proxy.x = 1.0;
    }
    // DISCARD: defer-publish then drop — the macro tick preamble's exact
    // early-exit surface (loan released, nothing sent, NO sequence burned).
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan 2");
        proxy.x = 99.0; // must never be observed
        proxy.__cer_defer_publish();
    }
    // Commit #2 → sequence 1 (a loan-time stamp would give 2 — the phantom gap).
    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan 3");
        proxy.x = 2.0;
    }

    // The HEADLINE wire-level assert runs FIRST so a regression fails
    // with the phantom-gap signature itself, not a downstream symptom.
    let frames = drain_seq_and_x(&mut subscriber);
    assert_eq!(
        frames,
        vec![(0, 1.0), (1, 2.0)],
        "published frames must carry CONSECUTIVE sequences with the \
         discarded payload absent — a (0, _), (2, _) result is the \
         loan-time-stamp bug (phantom gap)"
    );

    // Accessor semantics: two commits consumed exactly two
    // numbers; the discard consumed none.
    assert_eq!(
        publisher.sequence(),
        2,
        "sequence() must reflect COMMITS only (2 commits, 1 discard)"
    );
}

/// The all-variables-gate discard class (a variable-schema proxy dropped
/// with a declared variable field unwritten — the discard
/// surface) must also burn no sequence: commit → unwritten-field discard
/// → commit yields consecutive sequences on the wire.
#[test]
fn unwritten_variable_field_discard_burns_no_sequence() {
    use native_ros2_messages::std_msgs::String as RosString;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("seq_commit_unwritten");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(1024))
        .expect("create publisher");
    let subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    {
        let mut proxy = publisher.loan_proxy::<RosString>().expect("loan 1");
        proxy.set_data("first").expect("set_data");
    }
    // Discard: `data` never written → the Drop all-variables gate skips
    // the publish (loud error!) — and must NOT consume a sequence.
    {
        let _proxy = publisher.loan_proxy::<RosString>().expect("loan 2");
    }
    {
        let mut proxy = publisher.loan_proxy::<RosString>().expect("loan 3");
        proxy.set_data("second").expect("set_data");
    }

    assert_eq!(publisher.sequence(), 2, "2 commits, 1 gate-discard");

    let mut seqs = Vec::new();
    let mut datas = Vec::new();
    // `try_receive` drains the whole queue, handing each frame's parsed header +
    // body to the callback (borrow released per frame).
    subscriber
        .try_receive(|msg| {
            seqs.push(msg.header().sequence);
            // String payload: offset table entry then bytes; read via the
            // typed view is simpler — but the frame is already drained, so
            // assert on the header stream only (the payload identity is
            // covered by the fixed-schema headline above).
            datas.push(msg.payload().len());
        })
        .expect("try_receive");
    assert_eq!(
        seqs,
        vec![0, 1],
        "gate-discarded loan must not burn a sequence (a loan-time stamp gives [0, 2])"
    );
    assert_eq!(datas.len(), 2, "exactly the two committed frames");
}

// ============================================================
// End-to-end proof of the discard flood-latch's
// RECOVERY info! + re-arm over REAL iceoryx2 transport. The pure latch is
// oracle-tested in `output_discard_latch_test.rs`; the loud-first + debug-
// suppressed drop-site mapping is pinned by the cdylib stderr subprocess
// test — but the recovery `info!` (`OutputProxy::Drop`'s steady-state
// send-success site) and its re-arm are exercised by NO other integration test.
// This drives a real publisher through a sustained discard regime (≥2
// discards so `suppressed > 0`), a healing complete publish, and a
// fresh discard, asserting on the captured `tracing` stream with a hand
// oracle (count of each message), never a self-compare.
// ============================================================

/// A variable-schema (`std_msgs::String`) proxy dropped WITHOUT `set_data`
/// trips the all-variables discard gate; a `set_data` + drop is a complete
/// publish. Sequence: discard·discard·discard → complete → discard.
///
/// Expected `tracing` stream (hand oracle):
/// - exactly 2 loud `error!` (regime-head discard #1, then the re-armed
///   post-recovery discard) — NOT 5, proving the flood-latch downgraded the
///   sustained middle discards;
/// - exactly 2 `debug!` "discard suppressed" (the 2nd and 3rd discards of the
///   first regime, `suppressed` 1 then 2);
/// - exactly 1 `info!` "output recovered" carrying `suppressed_count=2` (the
///   total downgraded in the closed regime);
/// - the complete publish actually DELIVERED (`try_view` reads the healed
///   payload — the recovery is a real send, not just a log).
#[test]
#[traced_test]
fn output_discard_recovery_info_and_rearm_e2e_iceoryx2() {
    use native_ros2_messages::std_msgs::String as RosString;

    let mgr = TransportManager::get_or_init().expect("init");
    let topic = unique_topic("discard_recovery");
    let mut publisher = mgr
        .create_publisher_simple(&topic, MaxSliceLen::const_new(1024))
        .expect("create publisher");
    let mut subscriber = mgr.create_subscriber(&topic).expect("create subscriber");

    // Regime 1: three discards (drop without set_data). #1 → error!,
    // #2 → debug{suppressed:1}, #3 → debug{suppressed:2}.
    for _ in 0..3 {
        let _proxy = publisher
            .loan_proxy::<RosString>()
            .expect("loan_proxy discard");
        // No set_data — the all-variables gate discards on drop.
    }

    // Complete publish → send-success recovery site: info! "output
    // recovered" with suppressed_count=2, then the latch re-arms.
    {
        let mut proxy = publisher
            .loan_proxy::<RosString>()
            .expect("loan_proxy complete");
        proxy
            .set_data("healed")
            .expect("set_data within max_slice_len");
    }

    // Fresh discard AFTER recovery → error! again (re-armed), NOT debug.
    {
        let _proxy = publisher
            .loan_proxy::<RosString>()
            .expect("loan_proxy re-armed discard");
    }

    // The healing publish really delivered (hand oracle, not a self-compare).
    let observed = subscriber
        .try_view::<RosString, _>(|view| view.data().expect("utf-8").to_owned())
        .expect("try_view should not error")
        .expect("the complete publish must have delivered one frame");
    assert_eq!(
        observed, "healed",
        "recovery is a real send, not just a log"
    );

    // Principle #3 queryability pin: the UNCONDITIONAL per-port discard
    // counter equals the hand-oracle discard count (3 in regime 1 + 1 re-armed
    // = 4), INDEPENDENT of log level — the 2 sustained discards were downgraded
    // to `debug!` (only 2 `error!`s above), yet all 4 are counted here. This is
    // the signal a persistently-broken node stays queryable through even when
    // its head `error!` has scrolled away.
    assert_eq!(
        publisher.output_discard_count(),
        4,
        "output_discard_count must count EVERY discard (loud + debug-suppressed), \
         not just the ones that logged loudly"
    );

    // Hand-oracle counts over the captured tracing stream.
    //
    // RELEASE-SAFE (re-stated): the "discard suppressed" count
    // observes `debug!` events, which `release_max_level_info` compiles out
    // under `--release`, so its expectation goes through `debug_lines_expected`
    // — the gate the discipline walk requires; the profile is not a property a
    // test may assume. What carries the contract in release is level-free: the
    // never-loud twin below and the unconditional `output_discard_count()`
    // counter above, which pins the same discards without a log level.
    logs_assert(|lines: &[&str]| {
        // Level-free twin: a suppressed discard repeat must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| {
                    line_level(l) == Some(level) && (l.contains("OutputProxy discard suppressed"))
                })
                .count();
            if loud != 0 {
                return Err(format!(
                    "a suppressed discard repeat was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // The loud heads, matched WITH their level token AND against the
        // level-free total of the same marker: a head demoted to `warn!`/`info!`
        // is not a loud error!, and a second copy of it at another level is not
        // one either.
        let errors = count_at_exclusively(
            lines,
            "ERROR",
            &["dropped without writing all declared variable fields"],
        )?;
        let debugs = count_at_exclusively(lines, "DEBUG", &["OutputProxy discard suppressed"])?;
        let recoveries = count_at_exclusively(lines, "INFO", &["OutputProxy: output recovered"])?;
        if errors != 2 {
            return Err(format!(
                "expected exactly 2 loud discard error!s (regime head + re-armed), got {errors} \
                 — a flood (5) means the latch never downgraded; <2 means no re-arm"
            ));
        }
        let want_debugs = debug_lines_expected(2);
        if debugs != want_debugs {
            return Err(format!(
                "expected exactly {want_debugs} debug 'discard suppressed' (2nd+3rd of regime 1), got {debugs}"
            ));
        }
        if recoveries != 1 {
            return Err(format!(
                "expected exactly 1 'output recovered' info!, got {recoveries}"
            ));
        }
        Ok(())
    });
    // Assert the RECOVERY line ITSELF carries suppressed_count=2, via a
    // per-line predicate — NOT a whole-log `logs_contain("suppressed_count=2")`.
    // The sustained `debug!` line in the same regime (the 3rd discard) ALSO
    // carries `suppressed_count=2`, so a bare `logs_contain` would pass even if
    // the recovery info! logged the WRONG count. Requiring BOTH the "output
    // recovered" marker AND `suppressed_count=2` on ONE line makes the assert
    // recovery-count-unique (a wrong-recovery-count regression fails).
    logs_assert(|lines: &[&str]| {
        let recovery_with_count = count_at_exclusively(
            lines,
            "INFO",
            &["OutputProxy: output recovered", "suppressed_count=2"],
        )?;
        if recovery_with_count != 1 {
            return Err(format!(
                "expected exactly 1 'output recovered' line carrying suppressed_count=2 \
                 (the closed regime's two downgraded discards), got {recovery_with_count}"
            ));
        }
        Ok(())
    });
}

// ============================================================
// NodeHandle-level (off-thread operator) observability of the
// per-output discard count. The `OutputDiscardLatch` lives on the per-port
// `CerulionPublisher` — reachable only from the node's OWN tick code (via
// `AnyPublisher::output_discard_count`). An operator reads a `NodeHandle`, which
// cannot reach the publisher, so the Principle #3 "observable independent of
// execution" claim holds only because the runtime shares the port's
// discard counter into the node's `NodeHandle` per-output map. This test drives
// a real graph (`build_for_test`) through a discard regime and reads the count
// back through `NodeHandle::output_discard_count` — the same surface the sibling
// `promise_within_iox2_test` reads `promise_within_missed_count` from.
// ============================================================

/// A `std_msgs::String` producer that DISCARDS its variable output on every fire
/// except one healing publish at fire index 3 — the graph-level analogue of the
/// raw-publisher regime in `output_discard_recovery_info_and_rearm_e2e_iceoryx2`.
/// It counts its OWN intended discards into a shared atomic (the HAND ORACLE —
/// NOT a self-compare against the counter under test).
struct DiscardingProducer {
    context: Option<NodeContext>,
    fire_n: u64,
    discards_attempted: Arc<AtomicU64>,
}

impl DiscardingProducer {
    fn new(discards_attempted: Arc<AtomicU64>) -> Self {
        Self {
            context: None,
            fire_n: 0,
            discards_attempted,
        }
    }
}

impl NodeEntry for DiscardingProducer {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::with_meta(
            Vec::new(),
            vec![OutputMeta::new(
                "out".to_string(),
                <RosString as ShmMessage>::SCHEMA_HASH,
                <RosString as ShmMessage>::MAX_SLICE_LEN,
            )],
        )
        .with_policy(MacroPolicy::Period { period_ms: 5 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        let n = self.fire_n;
        self.fire_n += 1;
        // One complete publish at fire index 3 heals the regime (proving the
        // count does NOT reset on recovery); every other fire discards.
        let heal = n == 3;
        if let Some(ctx) = self.context.as_mut() {
            if let Some(pubr) = ctx.publisher_mut("out") {
                let mut proxy = pubr.loan_proxy::<RosString>()?;
                if heal {
                    proxy.set_data("healed")?;
                } else {
                    // Drop WITHOUT writing the `data` variable field → the
                    // all-variables discard gate trips on drop.
                    self.discards_attempted.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        Ok(())
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        Ok(())
    }
}

#[test]
#[traced_test]
fn output_discard_count_is_observable_through_node_handle_e2e() {
    let discards_attempted = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "output_discard_nodehandle_test".to_string(),
        prefix: "odn".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "prod".to_string(),
            node_type: "discarding".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "std_msgs/String".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod".to_string(),
        Box::new(DiscardingProducer::new(Arc::clone(&discards_attempted))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build discarding-producer graph");
    // 8 × 5 ms steps → the Period(5 ms) node fires ~8 times (one heal at fire 3).
    for _ in 0..8 {
        runtime.step(Duration::from_millis(5));
    }

    let attempted = discards_attempted.load(Ordering::Relaxed);
    // Anti-tautology: the apparatus actually moved (the node fired + discarded).
    assert!(
        attempted > 0,
        "the producer must have fired and discarded at least once (got 0 — Period \
         node never fired?)"
    );

    // THE PIN: the off-thread operator reads the per-output discard count through
    // the NodeHandle — the same value the node's own tick would read via
    // `AnyPublisher::output_discard_count`. It equals the node's OWN discard tally
    // (hand oracle), INDEPENDENT of log level (most discards were `debug!`-
    // downgraded by the flood latch), and did NOT reset on the mid-run heal.
    let handle = runtime
        .node_handle("prod")
        .expect("prod node handle exists after build");
    assert_eq!(
        handle.output_discard_count("out"),
        attempted,
        "NodeHandle::output_discard_count must equal the node's actual discard \
         count — the Principle #3 off-thread observability the runtime wires"
    );

    // An unregistered / never-discarded output name reads 0, never a panic.
    assert_eq!(
        handle.output_discard_count("nonexistent"),
        0,
        "an unknown output name reads 0 (nothing discarded), never panics"
    );
}

// ============================================================
// NodeHandle-level (off-thread operator) observability of the
// per-output UNDELIVERED-NOTIFY count. Exactly the discard-count shape above,
// for the undelivered-notify signal: the `NotifyDeliveryLatch` lives on the per-port
// `CerulionPublisher`, reachable only from the node's OWN tick code (via
// `AnyPublisher::notify_undelivered_count`). An operator reads a `NodeHandle`,
// which cannot reach the publisher, and the iceoryx2 log-level default (correctly)
// silences iceoryx2's own per-publish complaint about this exact condition, so
// without this wiring a sustained degraded wake path is invisible to anything
// but a `debug!`. This test drives a real graph whose output topic carries a
// FOREIGN listener nobody drains (a `create_subscriber_open_only` handle held
// and never received from — the shape a wedged `topic hz` / a stalled sibling
// process presents) and reads the count back through
// `NodeHandle::notify_undelivered_count`.
// ============================================================

/// A `Vector3` producer that publishes a complete frame on every fire and
/// records, at the START of each tick, the count its OWN
/// `AnyPublisher::notify_undelivered_count` reports — the node-local view the
/// NodeHandle value is cross-checked against (two views, one count).
struct NotifyingProducer {
    context: Option<NodeContext>,
    node_local_count: Arc<AtomicU64>,
}

impl NodeEntry for NotifyingProducer {
    // The producer's schema, named once here (the file's other tests import it
    // per-fn inside their own module scopes).

    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::with_meta(
            Vec::new(),
            vec![OutputMeta::new(
                "out".to_string(),
                <ProducerVec3 as ShmMessage>::SCHEMA_HASH,
                <ProducerVec3 as ShmMessage>::MAX_SLICE_LEN,
            )],
        )
        .with_policy(MacroPolicy::Period { period_ms: 1 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        if let Some(ctx) = self.context.as_mut() {
            if let Some(pubr) = ctx.publisher_mut("out") {
                // Read BEFORE publishing, so this reflects every notify issued
                // so far and nothing from this fire.
                self.node_local_count
                    .store(pubr.notify_undelivered_count(), Ordering::Relaxed);
                let mut proxy = pubr.loan_proxy::<ProducerVec3>()?;
                proxy.x = 1.0;
                proxy.y = 2.0;
                proxy.z = 3.0;
            }
        }
        Ok(())
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        Ok(())
    }
}

/// Publishes to issue while trying to saturate the foreign listener's event
/// socket. Sized well past any plausible `AF_UNIX SOCK_DGRAM` capacity for
/// 8-byte datagrams (crib: `notify_self_drain_iox2_test::SATURATING_NOTIFIES`);
/// the loop breaks as soon as the counter moves, so the full bound is only paid
/// on a platform where saturation never happens (which fails loudly below).
const NOTIFY_SATURATING_STEPS: usize = 20_000;

#[test]
fn notify_undelivered_count_is_observable_through_node_handle_e2e() {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let topic = format!("/nodehandle/{nanos}");

    // An ISOLATED transport we also hold, so the test can attach the foreign
    // listener to the very same SHM root the graph publishes on.
    let clock = Arc::new(VirtualClock::new());
    let transport = TransportManager::init_for_test(
        cerulion_core::transport::TransportConfig {
            node_name: format!("nodehandle_{nanos}"),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated transport");

    let node_local_count = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "notify_undelivered_nodehandle_test".to_string(),
        prefix: "nun".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "prod".to_string(),
            node_type: "notifying".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                // Absolute override so the test can attach its foreign listener
                // to a name it knows exactly.
                topic: Some(topic.clone()),
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod".to_string(),
        Box::new(NotifyingProducer {
            context: None,
            node_local_count: Arc::clone(&node_local_count),
        }),
    );
    let mut runtime = GraphRuntime::build(config, factories, &transport, clock)
        .expect("build notifying-producer graph");

    // THE STIMULUS: a foreign listener on the producer's topic that NOBODY ever
    // drains. Held for the whole run — dropping it would deregister the listener
    // and heal the condition.
    let _wedged = transport
        .create_subscriber_open_only(&topic)
        .expect("foreign subscriber on the producer's topic");

    let mut steps = 0_usize;
    for _ in 0..NOTIFY_SATURATING_STEPS {
        runtime.step(Duration::from_millis(1));
        steps += 1;
        // Stop as soon as the condition is real — keeps the test fast and the
        // log volume bounded.
        if runtime
            .node_handle("prod")
            .expect("prod handle")
            .notify_undelivered_count("out")
            > 0
        {
            break;
        }
    }

    let observed = runtime
        .node_handle("prod")
        .expect("prod node handle exists after build")
        .notify_undelivered_count("out");
    assert!(
        observed > 0,
        "after {steps} publishes with a foreign listener that is never drained, its \
         AF_UNIX event socket must be full and notifies must start failing — got 0, so \
         either the platform's socket is unexpectedly unbounded or the \
         NodeHandle wiring is not hooked up"
    );

    // THE PIN: the off-thread NodeHandle value and the node's OWN
    // `AnyPublisher::notify_undelivered_count` are ONE count. The node records
    // its view at tick START, so one more step (whose own publish happens after
    // the read) makes the two directly comparable.
    runtime.step(Duration::from_millis(1));
    let handle = runtime
        .node_handle("prod")
        .expect("prod node handle exists after build");
    assert_eq!(
        node_local_count.load(Ordering::Relaxed),
        observed,
        "NodeHandle::notify_undelivered_count must be the same count the node's own \
         AnyPublisher::notify_undelivered_count reports — the runtime installs one \
         shared anchor on both sides (Principle #3)"
    );

    // Monotonic, never reset: the condition is still present, so it keeps
    // growing rather than clearing.
    assert!(
        handle.notify_undelivered_count("out") >= observed,
        "the unconditional total must never decrease"
    );

    // An unregistered output name reads 0, never a panic.
    assert_eq!(
        handle.notify_undelivered_count("nonexistent"),
        0,
        "an unknown output name reads 0 (nothing undelivered), never panics"
    );
}
