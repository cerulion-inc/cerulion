// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The schema-hash-mismatch flood latch at the PRODUCTION `rmw_take`
//! call site (`rmw_cerulion/src/api/pubsub.rs`).
//!
//! A wire frame whose `schema_hash` does not match the subscription's type is
//! DROPPED. That is a version SKEW between the two ends, so it fails EVERY
//! frame until somebody redeploys — and earlier each drop emitted a BARE
//! per-frame `warn!`. On a 100 Hz topic that is ~100 lines/s ≈ 860 MB/day, the
//! exact class that once filled a 234 GB robot disk and that the earlier
//! latches already fixed for their own arms.
//!
//! The suppression policy is `cerulion_core`'s shared `FailureRegimeLatch`,
//! oracle-tested in `cerulion_core/tests/failure_regime_latch_test.rs`; the
//! four `cerulion_core` service arms are pinned in `service_test.rs`. THIS
//! file pins the rmw topic-take arm end-to-end through the real C ABI.
//!
//! Its OWN binary, not an arm of `rmw_e2e_test.rs`, because `#[traced_test]`
//! installs a GLOBAL tracing subscriber: any sibling test that brings the rmw
//! runtime up first takes that slot and the capture then panics
//! (`SetGlobalDefaultError`). Same shape and same reason as
//! `rmw_transient_local_ceiling_test.rs`, where every test is `#[traced_test]`
//! too.
//!
//! ⚠️ Shares the iceoryx2 SHM singleton — run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_schema_mismatch_test -- --test-threads=1
//! ```

use cerulion_core::testing::{count_at, count_at_exclusively, debug_lines_expected, never_loud};
use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;
// no-env-filter so the capture reaches the `cerulion_core` target (the
// schema-mismatch reporter's crate), not just this test crate.
use tracing_test::traced_test;

// =====================================================================
// Fixtures (cribbed from rmw_e2e_test.rs, exactly as
// rmw_transient_local_ceiling_test.rs does)
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn member(
    name: &str,
    type_id: u8,
    offset: u32,
) -> ffi::rosidl_typesupport_introspection_c__MessageMember {
    ffi::rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr(name),
        type_id_: type_id,
        offset_: offset,
        ..Default::default()
    }
}

fn make_message_ts(
    namespace: &str,
    name: &str,
    size_of: usize,
    members: Vec<ffi::rosidl_typesupport_introspection_c__MessageMember>,
) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr(namespace),
            message_name_: cstr(name),
            member_count_: members.len() as u32,
            size_of_: size_of,
            members_: members.as_ptr(),
            ..Default::default()
        },
    ));
    let ts = ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    };
    Box::leak(Box::new(ts))
}

#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CPoint {
    x: f64,
    y: f64,
    z: f64,
}

/// Unique TYPE NAME per call, so schema hashes never collide across runs
/// against the global SHM singleton — and so two typesupports with the SAME
/// layout can carry DIFFERENT hashes, which is exactly the skew under test.
fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        "rmw_mismatch__msg",
        unique,
        std::mem::size_of::<CPoint>(),
        vec![
            member("x", ROS_TYPE_DOUBLE, 0),
            member("y", ROS_TYPE_DOUBLE, 8),
            member("z", ROS_TYPE_DOUBLE, 16),
        ],
    )
}

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    nanos ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
}

unsafe fn setup_node(
    name: &str,
) -> (
    *mut ffi::rmw_context_t,
    *mut ffi::rmw_node_t,
    Box<ffi::rmw_init_options_t>,
) {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null(), "node creation failed");
    (context, node, options)
}

fn default_qos() -> ffi::rmw_qos_profile_t {
    ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth: 8,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    }
}

// =====================================================================
// The pins
// =====================================================================

/// Substring unique to the LOUD (`warn!`) arm of the mismatch report.
const HASH_LOUD: &str = "dropping a frame whose wire schema hash does not match";
/// Substring unique to the SUPPRESSED (`debug!`) arm.
const HASH_SUPPRESSED: &str = "schema-hash mismatch suppressed";
/// Substring unique to the RECOVERY (`info!`) arm.
const HASH_RECOVERY: &str = "schema hashes match again";
/// Substring unique to the LOUD (`error!`) arm of the DECODE report.
const DECODE_LOUD: &str = "dropping a frame the bridge could not decode";
/// Substring unique to the SUPPRESSED (`debug!`) arm of the decode report.
const DECODE_SUPPRESSED: &str = "decode failure suppressed";
/// Substring unique to the DECADE RE-ANNOUNCEMENT (`error!`) arm of the decode
/// report — the arm that exists precisely to survive a filter hiding `debug!`.
const DECODE_STILL: &str = "decode failures are STILL dropping every frame";

/// Read the subscription's hash-mismatch counter — the log-level-independent
/// Principle #3 signal.
///
/// This cast is the ONLY way to reach it: the rmw C ABI is standardized, so no
/// accessor can be added for rclcpp/rclpy. That is precisely why the shared
/// latch re-announces an open regime at each decade of the running total — the
/// log is a ROS user's whole window onto the condition.
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn hash_mismatch_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    cerulion_core::transport::failure_regime_latch::lock_regime_latch(&data.hash_mismatches)
        .total_failures()
}

/// Read the subscription's -failure counter (the sibling latch).
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn decode_failure_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    data.decode_failures
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .total_failures()
}

/// The wire `schema_hash` this subscription's bridge expects.
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn expected_schema_hash(subscription: *const ffi::rmw_subscription_t) -> u64 {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    data.bridge.schema_hash()
}

/// The CERULION topic name behind an rmw subscription (the fully-qualified
/// ROS name itself — the mapping is the identity) — what a raw
/// `cerulion_core` publisher must open.
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn cerulion_topic(subscription: *const ffi::rmw_subscription_t) -> String {
    let data = &*((*subscription).data as *const rmw_cerulion::runtime::SubscriptionData);
    data.topic.clone()
}

/// A hand-built wire frame: 32-byte header + `payload_len` zero bytes.
///
/// `total_size` must equal the published slice length exactly, or the
/// subscriber's own bounds check drops the frame BEFORE the hash gate and
/// neither latch moves.
fn raw_frame(schema_hash: u64, sequence: u32, payload_len: usize) -> Vec<u8> {
    use cerulion_core::wire::WireHeader;
    let total = WireHeader::SIZE + payload_len;
    let header = WireHeader {
        schema_hash,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence,
        timestamp_ns: 0,
    };
    let mut frame = vec![0u8; total];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame
}

/// The hash-mismatch latch at the PRODUCTION `rmw_take` call site (`api/pubsub.rs`).
///
/// A version-skewed peer publishes a type this subscription does not expect:
/// the hashes disagree, so EVERY frame is dropped until somebody redeploys.
///
/// Hand oracles: N=6 skewed frames ⇒ exactly ONE loud head + 5 suppressed
/// repeats + an UNCONDITIONAL counter of 6; then a matching frame ⇒ exactly
/// one recovery line carrying the SUPPRESSED count (5, not 6 — the loud head
/// was never suppressed) and a delivered message; then a fresh skew is loud
/// again (the re-arm). `taken` stays false throughout the regime — a dropped
/// frame must never be reported as taken.
///
/// The skew is built the way a real one arises: two typesupports with the same
/// layout and DIFFERENT type names, so the schema hashes differ while the
/// frames stay structurally publishable on one topic.
#[test]
#[serial]
#[traced_test]
fn wrong_hash_takes_are_loud_once_counted_always_and_recover() {
    const SKEWED_FRAMES: usize = 6;
    unsafe {
        let suffix = unique_suffix();
        // Same layout, different type NAME ⇒ different schema hash.
        let ours = point_ts(&format!("MismatchOurs{suffix}"));
        let skewed = point_ts(&format!("MismatchSkewed{suffix}"));
        let (_, node, _opts) = setup_node(&format!("node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/skew/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert_eq!(
            hash_mismatch_count(subscription),
            0,
            "a fresh subscription must start clean"
        );

        // The skewed peer.
        let bad_publisher = rmw_create_publisher(node, skewed, topic.as_ptr(), &qos, &pub_opts);
        assert!(!bad_publisher.is_null());

        for i in 0..SKEWED_FRAMES {
            let msg = CPoint {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                rmw_publish(
                    bad_publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let mut out = CPoint::default();
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK,
                "a hash mismatch is a DROP, not a take failure"
            );
            assert!(!taken, "a wrong-hash frame must never be reported as taken");
            assert_eq!(out, CPoint::default(), "the out buffer must be untouched");
        }
        assert_eq!(
            hash_mismatch_count(subscription),
            SKEWED_FRAMES as u64,
            "the counter is UNCONDITIONAL — it must count the debug-suppressed \
             repeats too, or a persistently skewed subscription is invisible at \
             RUST_LOG=error"
        );

        // Recovery: the operator redeploys, and a matching publisher takes
        // over the topic. The skewed publisher is destroyed first so the two
        // are never live at once — the rmw subscription path leaves
        // `max_publishers` at iceoryx2's default of 2, so this is a
        // FIDELITY choice (one writer per deployment), not a cap the
        // transport would have enforced.
        assert_eq!(rmw_destroy_publisher(node, bad_publisher), RMW_RET_OK);
        let good_publisher = rmw_create_publisher(node, ours, topic.as_ptr(), &qos, &pub_opts);
        assert!(!good_publisher.is_null());
        let healed = CPoint {
            x: 42.0,
            y: -1.0,
            z: 7.5,
        };
        assert_eq!(
            rmw_publish(
                good_publisher,
                &healed as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut out = CPoint::default();
        let mut taken = false;
        assert_eq!(
            rmw_take(
                subscription,
                &mut out as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken, "the matching frame must be delivered");
        assert_eq!(out, healed);
        assert_eq!(
            hash_mismatch_count(subscription),
            SKEWED_FRAMES as u64,
            "recovery must NEVER reset the running total"
        );

        // Re-armed: a fresh skew is loud again.
        assert_eq!(rmw_destroy_publisher(node, good_publisher), RMW_RET_OK);
        let bad_again = rmw_create_publisher(node, skewed, topic.as_ptr(), &qos, &pub_opts);
        assert!(!bad_again.is_null());
        let msg = CPoint::default();
        assert_eq!(
            rmw_publish(
                bad_again,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let mut taken = true;
        assert_eq!(
            rmw_take(
                subscription,
                &mut out as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(!taken);
        assert_eq!(hash_mismatch_count(subscription), SKEWED_FRAMES as u64 + 1);

        logs_assert(|lines: &[&str]| {
            let warns = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
            never_loud(lines, HASH_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
            if warns != 2 {
                return Err(format!(
                    "expected exactly 2 WARN loud heads (the {SKEWED_FRAMES}-frame regime, \
                     then the re-armed one) — a per-frame warn would give {}, got {warns}",
                    SKEWED_FRAMES + 1
                ));
            }
            let want_debugs = debug_lines_expected(SKEWED_FRAMES - 1);
            if debugs != want_debugs {
                return Err(format!(
                    "expected {want_debugs} DEBUG suppressed repeats, got {debugs}"
                ));
            }
            if recoveries != 1 {
                return Err(format!(
                    "expected exactly 1 INFO recovery line, got {recoveries}"
                ));
            }
            // A variant that emits the suppressed arm at `warn!` keeps
            // every count above intact while suppression does nothing.
            let leaked = count_at(lines, "WARN", HASH_SUPPRESSED);
            if leaked != 0 {
                return Err(format!(
                    "the suppressed arm must be DEBUG, found {leaked} at WARN"
                ));
            }
            let rec = lines
                .iter()
                .find(|l| l.contains(HASH_RECOVERY))
                .ok_or("no recovery line")?;
            if !rec.contains(&format!("suppressed_count={}", SKEWED_FRAMES - 1)) {
                return Err(format!(
                    "recovery must report the {} SUPPRESSED (not all {SKEWED_FRAMES}): {rec}",
                    SKEWED_FRAMES - 1
                ));
            }
            // Operators grep a topic drop by `topic=`, never a generic `name=`
            // — on the RECOVERY line as much as the head, since watching one
            // key must show the regime both open and close.
            if !rec.contains("topic=/rmw_mismatch/skew/") {
                return Err(format!("recovery must log under `topic=`: {rec}"));
            }
            let head = lines
                .iter()
                .find(|l| l.contains(HASH_LOUD))
                .ok_or("no loud head")?;
            if !head.contains("topic=/rmw_mismatch/skew/") {
                return Err(format!("loud head must log under `topic=`: {head}"));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, bad_again), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// ANTI-TAUTOLOGY control for the test above: a MATCHED pub/sub pair emits
/// none of the lines and leaves the counter at exactly zero. Without
/// it, every "exactly N" assertion above would still hold if the reporter
/// also fired on matching frames.
#[test]
#[serial]
#[traced_test]
fn a_matched_pub_sub_pair_is_silent_and_counts_zero() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("MismatchQuiet{suffix}"));
        let (_, node, _opts) = setup_node(&format!("quiet_node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/quiet/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        for i in 0..6 {
            let msg = CPoint {
                x: i as f64,
                y: 1.0,
                z: 2.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            let mut out = CPoint::default();
            let mut taken = false;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            assert!(taken);
            assert_eq!(out, msg);
        }

        assert_eq!(hash_mismatch_count(subscription), 0);
        assert_eq!(decode_failure_count(subscription), 0);
        logs_assert(|lines: &[&str]| {
            let any = lines
                .iter()
                .filter(|l| {
                    l.contains(HASH_LOUD)
                        || l.contains(HASH_SUPPRESSED)
                        || l.contains(HASH_RECOVERY)
                        || l.contains(DECODE_LOUD)
                        || l.contains(DECODE_SUPPRESSED)
                })
                .count();
            if any == 0 {
                Ok(())
            } else {
                Err(format!(
                    "a matched pair must emit no hash-mismatch / decode-failure lines, got {any}"
                ))
            }
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The SEPARATENESS of the hash latch from the decode
/// latch, at the production `rmw_take` site.
///
/// `SubscriptionData` deliberately carries TWO latches: a hash mismatch (the
/// two ends disagree on the TYPE — redeploy from the same schemas) and a
/// decode failure (they agree on the type and disagree on the FRAMING —
/// redeploy from the same build) are different conditions with different
/// remedies, and one open regime must never swallow the other's loud head.
/// Until this arm, that design claim had no test: merging the two latches left
/// the whole suite green.
///
/// Built from RAW wire frames on the subscription's own iceoryx2 topic,
/// because the two conditions need frames a real rmw publisher cannot produce:
/// a payload that carries the RIGHT hash but is too short for the bridge
/// layout (24 bytes of fixed section for a 3×f64 message).
///
/// Routing the decode-failure report through `data.hash_mismatches`
/// (one latch for both conditions) fails this test twice — the head is
/// downgraded to a suppressed repeat by the decode regime already open, and
/// `decode_failure_count` never leaves 0.
#[test]
#[serial]
#[traced_test]
fn an_open_decode_regime_does_not_swallow_the_hash_mismatch_head() {
    const DECODE_FRAMES: usize = 3;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("MismatchSep{suffix}"));
        let (_, node, _opts) = setup_node(&format!("sep_node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/sep/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let expected = expected_schema_hash(subscription);
        let cer_topic = cerulion_topic(subscription);
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let mut raw_pub = rt
            .transport
            .create_publisher_simple(&cer_topic, msl)
            .expect("raw publisher on the subscription's topic");

        // A 3×f64 message needs 24 bytes of fixed section; 8 is far short, so
        // the bridge's unflatten refuses AFTER the hash gate has passed.
        const SHORT_PAYLOAD: usize = 8;

        let take_one = || {
            let mut out = CPoint::default();
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            taken
        };

        // Phase A — open a DECODE regime: right hash, unusable payload.
        for seq in 0..DECODE_FRAMES {
            raw_pub
                .publish_raw(&raw_frame(expected, seq as u32, SHORT_PAYLOAD))
                .expect("publish short frame");
            assert!(!take_one(), "an undecodable frame must never be taken");
        }
        assert_eq!(decode_failure_count(subscription), DECODE_FRAMES as u64);
        assert_eq!(
            hash_mismatch_count(subscription),
            0,
            "a frame whose hash MATCHED must not touch the hash counter"
        );

        // Phase B — with that regime OPEN, a hash mismatch must still be LOUD.
        let wrong = expected ^ 0xA5A5_A5A5_A5A5_A5A5;
        raw_pub
            .publish_raw(&raw_frame(wrong, DECODE_FRAMES as u32, SHORT_PAYLOAD))
            .expect("publish skewed frame");
        assert!(!take_one());
        assert_eq!(hash_mismatch_count(subscription), 1);
        assert_eq!(
            decode_failure_count(subscription),
            DECODE_FRAMES as u64,
            "a wrong-hash frame returns before the decode arm — the decode \
             counter must not move"
        );

        // Phase C — the decode regime was never closed by phase B, so the next
        // undecodable frame is a suppressed repeat, NOT a fresh loud head.
        raw_pub
            .publish_raw(&raw_frame(
                expected,
                DECODE_FRAMES as u32 + 1,
                SHORT_PAYLOAD,
            ))
            .expect("publish short frame");
        assert!(!take_one());
        assert_eq!(decode_failure_count(subscription), DECODE_FRAMES as u64 + 1);

        logs_assert(|lines: &[&str]| {
            let decode_heads = count_at_exclusively(lines, "ERROR", &[DECODE_LOUD])?;
            never_loud(lines, DECODE_SUPPRESSED)?;
            let decode_debugs = count_at_exclusively(lines, "DEBUG", &[DECODE_SUPPRESSED])?;
            let hash_heads = count_at_exclusively(lines, "WARN", &[HASH_LOUD])?;
            never_loud(lines, HASH_SUPPRESSED)?;
            let hash_debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
            let hash_recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
            if decode_heads != 1 {
                return Err(format!(
                    "expected exactly 1 ERROR decode head over {} undecodable frames, \
                     got {decode_heads}",
                    DECODE_FRAMES + 1
                ));
            }
            let want_decode_debugs = debug_lines_expected(DECODE_FRAMES);
            if decode_debugs != want_decode_debugs {
                return Err(format!(
                    "expected {want_decode_debugs} DEBUG decode repeats, got {decode_debugs}"
                ));
            }
            if hash_heads != 1 {
                return Err(format!(
                    "an OPEN decode regime must not swallow the hash mismatch's loud \
                     head: expected 1 WARN, got {hash_heads}"
                ));
            }
            if hash_debugs != 0 {
                return Err(format!(
                    "the single hash mismatch must be the LOUD head, not a repeat of \
                     somebody else's regime: got {hash_debugs} DEBUG lines"
                ));
            }
            if hash_recoveries != 0 {
                return Err(format!(
                    "a lone hash mismatch re-arms SILENTLY, got {hash_recoveries} \
                     recovery lines"
                ));
            }
            Ok(())
        });

        drop(raw_pub);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The DECADE RE-ANNOUNCEMENT of the decode latch, at the production
/// `rmw_take` site — the arm the rmw sites need MOST.
///
/// `SubscriptionData::decode_failures` sits behind an opaque `*mut c_void` the
/// STANDARDIZED rmw C ABI hands to rclcpp/rclpy, so no accessor can be added
/// and the only reader is the cast above. A ROS user's whole window onto a
/// subscription dropping every frame is therefore the LOG, which is why an open
/// regime re-announces loudly at each power of ten instead of going silent
/// after one line.
///
/// That arm pins only the hash reporter's twin: every other
/// drive in the suite stops at 4 failures while the first decade boundary is
/// 10, so this `error!` was reachable by no assertion and free to be a
/// `debug!` — which would put it back under exactly the filter it exists to
/// survive.
///
/// Hand oracle over 10 undecodable frames in ONE open regime: 1 `ERROR` head +
/// 8 `DEBUG` repeats + 1 `ERROR` re-announcement carrying `total_failures=10`
/// and the UNCHANGED `suppressed=8`. Changing `error!` to `debug!` on that arm
/// fails the ERROR triple AND the DEBUG-negative guard.
///
/// Same RAW-frame stimulus as the separateness arm: the right hash with a
/// payload below the bridge's 24-byte fixed section is a shape no real rmw
/// publisher can produce, so it must be hand-built.
#[test]
#[serial]
#[traced_test]
fn an_open_decode_regime_re_announces_at_the_decade_at_error() {
    /// One full decade of undecodable frames — the 10th crosses the boundary.
    const DECODE_FRAMES: usize = 10;
    /// A 3×f64 message needs 24 bytes of fixed section; 8 is far short, so the
    /// bridge's unflatten refuses AFTER the hash gate has passed.
    const SHORT_PAYLOAD: usize = 8;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("DecodeDecade{suffix}"));
        let (_, node, _opts) = setup_node(&format!("decade_node_{suffix}"));

        let topic = CString::new(format!("/rmw_mismatch/decade/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let expected = expected_schema_hash(subscription);
        let cer_topic = cerulion_topic(subscription);
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let mut raw_pub = rt
            .transport
            .create_publisher_simple(&cer_topic, msl)
            .expect("raw publisher on the subscription's topic");

        for seq in 0..DECODE_FRAMES {
            raw_pub
                .publish_raw(&raw_frame(expected, seq as u32, SHORT_PAYLOAD))
                .expect("publish short frame");
            let mut out = CPoint::default();
            let mut taken = true;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut out as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
            assert!(!taken, "an undecodable frame must never be taken");
        }
        assert_eq!(decode_failure_count(subscription), DECODE_FRAMES as u64);
        assert_eq!(
            hash_mismatch_count(subscription),
            0,
            "every frame carried the RIGHT hash — the hash counter must not move"
        );

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[DECODE_LOUD])?;
            let still = count_at_exclusively(lines, "ERROR", &[DECODE_STILL])?;
            never_loud(lines, DECODE_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[DECODE_SUPPRESSED])?;
            let want_debugs = debug_lines_expected(8);
            if (heads, still, debugs) != (1, 1, want_debugs) {
                return Err(format!(
                    "expected (1 ERROR head, 1 ERROR decade re-announcement, {want_debugs} DEBUG \
                     repeats) over {DECODE_FRAMES} undecodable frames, got \
                     ({heads}, {still}, {debugs})"
                ));
            }
            if count_at(lines, "DEBUG", DECODE_STILL) != 0 {
                return Err(
                    "the re-announcement must be LOUD — it is the operator's only \
                     window at the rmw sites, where the counter is unreachable"
                        .to_string(),
                );
            }
            let line = lines
                .iter()
                .find(|l| l.contains(DECODE_STILL))
                .ok_or("no re-announcement line")?;
            for needle in ["total_failures=10", "suppressed=8"] {
                if !line.contains(needle) {
                    return Err(format!("re-announcement is missing {needle}: {line}"));
                }
            }
            // The reporters used to log a GENERIC `name=`
            // while the arms three lines away logged `topic=`, so one
            // subscription's two failure conditions could not be followed with
            // one query. Every decode line on a subscription now carries the
            // same key as its hash-mismatch twin.
            let head = lines
                .iter()
                .find(|l| l.contains(DECODE_LOUD))
                .ok_or("no decode head")?;
            for l in [head, line] {
                if !l.contains("topic=/rmw_mismatch/decade/") {
                    return Err(format!("decode line must log under `topic=`: {l}"));
                }
            }
            Ok(())
        });

        drop(raw_pub);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
