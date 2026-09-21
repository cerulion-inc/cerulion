// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! A TRANSIENT_LOCAL rmw publisher provisions the subscriber buffer
//! ceiling UP so its history
//! request is SATISFIABLE, instead of clamping history down or silencing the
//! (correct) cerulion_core warn. rclpy's /rosout asks TRANSIENT_LOCAL
//! depth 1000 → rmw clamps history to 16 → without the raise the core warns
//! "history exceeds 75% of the subscriber buffer ceiling" on EVERY rclpy
//! process. The publisher raises the ceiling to `(history*4).div_ceil(3)` (the
//! smallest `c` with `history <= c*3/4`), so the warn stays quiet AND
//! late-joiners can hold the full history.
//!
//! ⚠️ Shares the iceoryx2 SHM singleton + the process-global registry —
//! run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_transient_local_ceiling_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::collections::BTreeSet;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;
use tracing_test::traced_test;

const WARN_SUBSTR: &str = "exceeds 75%";

/// The shared leading phrase of BOTH
/// create-time depth warns (publisher and subscription). Each site carries its
/// own literal below this prefix, so a message edit at one site is still
/// visible here.
const DEPTH_WARN_MARKER: &str = "requested QoS depth differs from the depth Cerulion provisioned";

/// The transport's default subscriber queue depth — the depth an rmw
/// SUBSCRIPTION really gets, since `create_subscriber` forwards the defaults
/// and the caller's `depth` is never applied to the queue.
///
/// HAND-WRITTEN, not read back from the transport: reading
/// `subscriber_buffer_size()` would ask the production code to confirm its own
/// answer. `the_transport_default_is_still_the_hand_written_oracle` is the
/// drift guard that fails loudly if the constant moves.
const PROVISIONED_DEFAULT_DEPTH: usize = 16;

/// The subscriber-SLOT ceiling a TRANSIENT_LOCAL depth-1000 publisher
/// provisions: `(16 * 4).div_ceil(3)` for the CLAMPED history 16, hand-computed.
///
/// This is a slot BUDGET, NOT the depth endpoint info reports — the
/// retention-depth refinement separated the two. It stays pinned exactly where it
/// belongs, by the require-N probes in
/// [`transient_local_depth_1000_clamps_history_and_reports_the_provisioned_depth`],
/// which are byte-unchanged by that decision.
const PROVISIONED_CLAMPED_CEILING: usize = 22;

/// The RETENTION a TRANSIENT_LOCAL depth-1000 publisher is provisioned, and so
/// the depth its endpoint info reports (retention-depth reporting rule): the clamped
/// history `1000.clamp(1, 16)`, hand-computed.
///
/// Deliberately NOT spelled as [`PROVISIONED_DEFAULT_DEPTH`] even though both
/// are 16 today: one is a publisher's clamped history and the other is the
/// transport's default subscriber buffer. They answer different questions and
/// either can move without the other.
const PROVISIONED_CLAMPED_HISTORY: usize = 16;

/// A TRANSIENT_LOCAL depth whose ask IS its retention (`5.clamp(1, 16) == 5`) —
/// the decision's second worked case, and the shape that makes a create QUIET.
const MATCHING_TL_DEPTH: usize = 5;

// =====================================================================
// Log-capture helpers (crib `rmw_publish_reject_test.rs`)
//
// Every predicate matches the LEVEL TOKEN as well as the message: a text-only
// filter passes a variant that demotes the warn to `debug!`, so the
// operator never sees the clamp while the message text stays intact.
// =====================================================================

const LEVELS: [&str; 5] = ["TRACE", "DEBUG", "INFO", "WARN", "ERROR"];

/// The LEVEL token of one captured line, or `None` if its header carries none.
///
/// `tracing-test` renders `<timestamp> <LEVEL> <span>: <target>: <message>`,
/// and the SPAN is the test function's own name — so a bare substring search
/// for "WARN" can be satisfied by a field value or a renamed test. Read whole
/// whitespace tokens out of the header instead.
fn line_level(line: &str) -> Option<&'static str> {
    let header = line.split(": ").next().unwrap_or(line);
    header
        .split_whitespace()
        .find_map(|token| LEVELS.into_iter().find(|level| *level == token))
}

/// Count captured lines carrying BOTH the level token and the marker.
fn count_at(lines: &[&str], level: &str, marker: &str) -> usize {
    lines
        .iter()
        .filter(|l| line_level(l) == Some(level) && l.contains(marker))
        .count()
}

/// The first captured line at `level` carrying `marker`, for arms asserting on
/// a line's FIELDS rather than counting lines.
fn find_at<'a>(lines: &[&'a str], level: &str, marker: &str) -> Option<&'a str> {
    lines
        .iter()
        .copied()
        .find(|l| line_level(l) == Some(level) && l.contains(marker))
}

/// True iff `line` carries the structured FIELD `key=value` as a whole
/// WHITESPACE TOKEN, never as a substring.
///
/// Load-bearing: a substring predicate makes `requested_depth=1` satisfied by
/// `requested_depth=1000`, and `topic=` satisfied by any prefixed key. Only for
/// fields whose value contains no whitespace (every numeric field, and the
/// topic name).
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let needle = format!("{key}={value}");
    line.split_whitespace().any(|token| token == needle)
}

// =====================================================================
// Fixtures (cribbed from rmw_e2e_test.rs)
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

fn point_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        "rmw_tl__msg",
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

fn transient_local_qos(depth: usize) -> ffi::rmw_qos_profile_t {
    ffi::rmw_qos_profile_t {
        history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth,
        reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        durability: ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
        lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
        liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
        avoid_ros_namespace_conventions: false,
    }
}

fn volatile_qos(depth: usize) -> ffi::rmw_qos_profile_t {
    let mut q = transient_local_qos(depth);
    q.durability = ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    q
}

// A REAL malloc-backed rcutils_allocator_t (crib rmw_e2e_test) — the
// depth-1000 test reads the endpoint info back through the caller
// allocator.
extern "C" {
    fn malloc(size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}
unsafe extern "C" fn fixture_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    malloc(size)
}
unsafe extern "C" fn fixture_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}
fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(fixture_allocate);
    a.deallocate = Some(fixture_deallocate);
    a
}

// =====================================================================
// Positive CONTROL: the exact warn IS captured on the un-raised path.
// =====================================================================

/// Anti-tautology guard for the negative tests below: create a publisher
/// on the RAW cerulion_core transport with history 16 at the DEFAULT
/// buffer ceiling (16) — the un-raised path — and confirm the
/// `#[traced_test]` subscriber DOES capture cerulion_core's "exceeds 75%"
/// warn. If this fails, the warn moved / capture is broken, and the
/// negative assertions are meaningless.
#[test]
#[traced_test]
#[serial]
fn control_unraised_history_16_fires_the_core_warn() {
    let suffix = unique_suffix();
    let rt = rmw_cerulion::runtime::runtime().expect("runtime");
    let topic = format!("rmw_tl_ctl_{suffix}");
    let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
    // Default buffer is 16; history 16 > 16*3/4 = 12 → the warn fires.
    let _pub = rt
        .transport
        .create_publisher(&topic, msl, 16)
        .expect("core publisher");
    assert!(
        logs_contain(WARN_SUBSTR),
        "the un-raised path MUST fire the core warn (else the negative tests are tautological)"
    );
}

// =====================================================================
// (i) The fix: a raised TRANSIENT_LOCAL publisher does NOT warn.
// =====================================================================

/// THE raise pin: reverting the ceiling raise
/// (`.max((history*4).div_ceil(3))` → `.max(0)`) fails exactly this
/// test (the un-raised ceiling 16 trips the 75% warn at history 16).
///
/// RENAMED to say which warn it owns: the cerulion_core 75%
/// warn. (Under the earlier slot-depth reporting rule this publisher also fired the create-time
/// DEPTH warn — asked 16, provisioned the ceiling 22 — which is what forced the
/// rename. The retention-depth refinement reports RETENTION instead, so `16 == 16`
/// and it is quiet on both; the name stays, because scoping a negative
/// assertion to the warn it actually asserts about is right either way.)
#[test]
#[traced_test]
#[serial]
fn transient_local_depth_16_publisher_does_not_fire_the_core_75_percent_warn() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlA{suffix}"));
        let (_, node, _opts) = setup_node(&format!("tl_ceil_node_{suffix}"));
        let topic = CString::new(format!("/rmw_tl/a/{suffix}")).expect("topic");
        let qos = transient_local_qos(16);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        assert!(
            !logs_contain(WARN_SUBSTR),
            "raised ceiling (16 → 22) must keep the 75% warn quiet"
        );
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// (v) The headline motivating case: rclpy /rosout asks TRANSIENT_LOCAL
// depth 1000 (no other test passes depth > 16,
// so a clamp-widening regression would escape them: the raise formula scales
// with history, keeping the warn quiet at ANY clamp value).
// =====================================================================

/// Pins all three halves of the depth-1000 pipeline:
/// (a) no 75% warn;
/// (b) the clamp actually BIT — the created service's provisioned
///     ceiling is exactly 22 = (16*4).div_ceil(3) for the CLAMPED
///     history 16, NOT 1334 = (1000*4).div_ceil(3) for the raw depth.
///     Observed behaviorally via iceoryx2 open requirements (crib
///     `topic_buffer_sizing_test`'s require-N pattern): an opener
///     requiring ceiling 22 attaches while ones requiring 23 and 1334
///     are rejected by iceoryx2 itself — 22-attach + 23-reject pins
///     EXACTLY 22; the 1334 probe names the unclamped regression value;
/// (c) **PROVISIONED-DEPTH REPORTING.** Endpoint info reports the depth
///     Cerulion actually PROVISIONED, and that depth is
///     RETENTION: the clamped history 16, not the requested 1000 and not
///     the subscriber-slot ceiling 22.
///     Echoing the ask would make the ONE surface that
///     can show the clamp repeat the request instead — `depth:
///     1000` on an endpoint that retains 16 is a promise the transport
///     does not keep; reporting 22 would be a different promise it does not
///     keep either (nothing retains 22 frames). The (a)/(b) halves keep the 22
///     ceiling pinned exactly where it belongs — behaviorally,
///     against the live service — while (c) pins the number ROS is told.
#[test]
#[traced_test]
#[serial]
fn transient_local_depth_1000_clamps_history_and_reports_the_provisioned_depth() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlK{suffix}"));
        let (_, node, _opts) = setup_node(&format!("tl_kilo_node_{suffix}"));
        let topic = CString::new(format!("/rmw_tl/kilo/{suffix}")).expect("topic");
        let qos = transient_local_qos(1000);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // (a) The raise keeps the core warn quiet on the headline ask.
        assert!(
            !logs_contain(WARN_SUBSTR),
            "depth 1000 (clamped to history 16, ceiling 22) must not warn"
        );

        // (b) Require-N probes against the LIVE service. Cerulion's own
        // pre-check passes (buffer == cfg ceiling), so a rejection
        // genuinely comes from the iceoryx2 open-time at-least
        // verification against the ceiling the rmw publisher CREATED.
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let cer_topic = format!("/rmw_tl/kilo/{suffix}");
        let probe = |ceiling: usize| {
            let mut cfg = rt.transport.default_topic_config();
            cfg.subscriber_max_buffer_size = ceiling;
            rt.transport
                .create_subscriber_with_buffers(&cer_topic, cfg, ceiling)
        };
        assert!(
            probe(22).is_ok(),
            "an opener requiring ceiling 22 must attach — the raise applied \
             to the CLAMPED history (16 → 22)"
        );
        assert!(
            probe(23).is_err(),
            "an opener requiring 23 must be rejected — with the 22-attach \
             this pins the provisioned ceiling at EXACTLY 22"
        );
        assert!(
            probe(1334).is_err(),
            "the UNCLAMPED raise value ((1000*4).div_ceil(3) = 1334) must \
             NOT be provisioned — the clamp bit before the raise"
        );

        // (c) endpoint info reports the depth Cerulion RETAINS —
        // the clamped history 16, cross-checked against (ii)'s late-joiner
        // delivery arm, which receives exactly 16 frames against a hand
        // oracle. Explicitly NOT the ceiling 22 the probes above just
        // measured: that is a slot budget, and asserting they DIFFER here is
        // what stops the two numbers being conflated again.
        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        assert_eq!(
            (*arr.info_array).qos_profile.depth,
            PROVISIONED_CLAMPED_HISTORY,
            "endpoint info must report the depth Cerulion RETAINS \
             ({PROVISIONED_CLAMPED_HISTORY} — the clamped history), never an echo of the \
             requested 1000 and never the subscriber-slot ceiling \
             {PROVISIONED_CLAMPED_CEILING} (by design)"
        );
        assert_ne!(
            PROVISIONED_CLAMPED_HISTORY, PROVISIONED_CLAMPED_CEILING,
            "the retention and the slot ceiling must stay DISTINCT oracles — if they \
             ever coincide this arm stops discriminating between them"
        );
        assert_eq!(
            (*arr.info_array).qos_profile.durability,
            ffi::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// The create-time depth warning contract.
//
// The decided surface is TWO things — endpoint info serves the provisioned
// depth (arm (c) above), and a create whose ask differs from what it got
// says so LOUDLY, naming both numbers. The arms below own the second half.
// =====================================================================

/// Drift guard for [`PROVISIONED_DEFAULT_DEPTH`].
///
/// The subscription arms hand-write 16 as their oracle rather than reading it
/// back from the transport (which would be the production code confirming its
/// own answer). If the transport default ever moves, fail HERE — naming the
/// constant — instead of failing those arms with a mystifying number.
#[test]
#[traced_test]
#[serial]
fn the_transport_default_is_still_the_hand_written_oracle() {
    let rt = rmw_cerulion::runtime::runtime().expect("runtime");
    assert_eq!(
        rt.transport.subscriber_buffer_size(),
        PROVISIONED_DEFAULT_DEPTH,
        "the transport's default subscriber buffer moved; update \
         PROVISIONED_DEFAULT_DEPTH and re-check the subscription arms"
    );
}

/// THE warn pin: a clamped TRANSIENT_LOCAL depth-1000 publisher warns EXACTLY
/// once at create, at WARN, naming the topic AND both numbers.
///
/// The field assertions go through [`has_field`] (whole-token `key=value`), so
/// `requested_depth=1000` cannot be satisfied by a prose mention and
/// `provisioned_depth=22` cannot be satisfied by `provisioned_depth=220`.
#[test]
#[traced_test]
#[serial]
fn a_clamped_depth_warns_at_create_naming_both_numbers() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlW{suffix}"));
        let (_, node, _opts) = setup_node(&format!("tl_warn_node_{suffix}"));
        let ros_topic = format!("/rmw_tl/warn/{suffix}");
        let cer_topic = ros_topic.to_string();
        let topic = CString::new(ros_topic.clone()).expect("topic");
        let qos = transient_local_qos(1000);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        logs_assert(|lines: &[&str]| {
            let warns = count_at(lines, "WARN", DEPTH_WARN_MARKER);
            if warns != 1 {
                return Err(format!(
                    "expected EXACTLY 1 WARN depth line at create, got {warns} \
                     (0 = the warn is gone or was demoted below WARN; >1 = it fires per something)"
                ));
            }
            let line = find_at(lines, "WARN", DEPTH_WARN_MARKER).expect("counted above");
            if !has_field(line, "topic", &cer_topic) {
                return Err(format!("the warn must name topic={cer_topic}; got: {line}"));
            }
            if !has_field(line, "requested_depth", "1000") {
                return Err(format!(
                    "the warn must name requested_depth=1000; got: {line}"
                ));
            }
            if !has_field(
                line,
                "provisioned_depth",
                &PROVISIONED_CLAMPED_HISTORY.to_string(),
            ) {
                return Err(format!(
                    "the warn must name provisioned_depth={PROVISIONED_CLAMPED_HISTORY} \
                     (the RETENTION — not the slot ceiling \
                     {PROVISIONED_CLAMPED_CEILING}); got: {line}"
                ));
            }
            // The earlier slot-depth diagnostic also carried `retained_frames`, because under
            // the ceiling reading neither decided number answered "how many old
            // frames does a late joiner get?". `provisioned_depth` IS that
            // number now, so the field is gone — two spellings of one value on
            // one line is a surface an operator has to be told to ignore.
            if line.contains("retained_frames") {
                return Err(format!(
                    "`retained_frames` duplicates provisioned_depth and must not be re-added; got: {line}"
                ));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The NO-WARN control: a publisher whose ask IS what it gets creates silently
/// — and the decision's second worked case (TRANSIENT_LOCAL depth 5 ⇒ 5).
///
/// The forbidden-line half is PAIRED, in this same body and over this same
/// capture, with a publisher that MUST warn — without it "no depth warn fired"
/// is satisfied by a build in which the warn does not exist at all, and every
/// assertion in the arm above would still need its own separate proof that the
/// warn is CONDITIONAL rather than unconditional.
///
/// The quiet publisher is TRANSIENT_LOCAL depth 5, not the VOLATILE depth-16
/// the earlier slot-depth version used. That swap is load-bearing even though VOLATILE
/// is also quiet again under the "scope to real clamps" decision: a VOLATILE
/// publisher is quiet because of the DURABILITY GATE, so it could not
/// distinguish "the ask matched" from "the gate suppressed it". A
/// TRANSIENT_LOCAL depth 5 reaches the comparison and passes it, which is the
/// property this arm exists to pin. It also carries the reported-depth
/// assertion, so "quiet" and "reported == requested" are ONE fact in one body
/// rather than two arms that could drift apart.
#[test]
#[traced_test]
#[serial]
fn a_matching_depth_creates_without_a_depth_warn() {
    unsafe {
        let suffix = unique_suffix();
        let (_, node, _opts) = setup_node(&format!("tl_quiet_node_{suffix}"));
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();

        // TRANSIENT_LOCAL depth 5 ⇒ history `5.clamp(1, 16)` = 5, which IS what
        // this publisher asked for (retention-depth reporting rule: TL depth-5 → 5).
        let quiet_ts = point_ts(&format!("TlQ{suffix}"));
        let quiet_ros = format!("/rmw_tl/quiet/{suffix}");
        let quiet_cer = quiet_ros.to_string();
        let quiet_topic = CString::new(quiet_ros).expect("topic");
        let quiet_qos = transient_local_qos(MATCHING_TL_DEPTH);
        let quiet_pub =
            rmw_create_publisher(node, quiet_ts, quiet_topic.as_ptr(), &quiet_qos, &pub_opts);
        assert!(!quiet_pub.is_null());

        // The other half of "matching": endpoint info reports the 5 it asked
        // for BECAUSE 5 is what it retains, not because the surface echoes.
        let mut allocator = malloc_allocator();
        let mut quiet_arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(
                node,
                &mut allocator,
                quiet_topic.as_ptr(),
                false,
                &mut quiet_arr
            ),
            RMW_RET_OK
        );
        assert_eq!(quiet_arr.size, 1);
        assert_eq!(
            (*quiet_arr.info_array).qos_profile.depth,
            MATCHING_TL_DEPTH,
            "a TRANSIENT_LOCAL depth-{MATCHING_TL_DEPTH} publisher retains \
             {MATCHING_TL_DEPTH} and must report it — reporting the slot ceiling \
             ({PROVISIONED_DEFAULT_DEPTH} here) would over-claim by {over} frames \
             (by design)",
            over = PROVISIONED_DEFAULT_DEPTH - MATCHING_TL_DEPTH
        );

        // The paired positive: a clamped publisher on a DIFFERENT topic.
        let loud_ts = point_ts(&format!("TlL{suffix}"));
        let loud_ros = format!("/rmw_tl/loud/{suffix}");
        let loud_cer = loud_ros.to_string();
        let loud_topic = CString::new(loud_ros).expect("topic");
        let loud_qos = transient_local_qos(1000);
        let loud_pub =
            rmw_create_publisher(node, loud_ts, loud_topic.as_ptr(), &loud_qos, &pub_opts);
        assert!(!loud_pub.is_null());

        logs_assert(move |lines: &[&str]| {
            let warns = count_at(lines, "WARN", DEPTH_WARN_MARKER);
            if warns != 1 {
                return Err(format!(
                    "expected EXACTLY 1 WARN depth line across BOTH creates, got {warns} \
                     (2 = the warn fires unconditionally; 0 = it never fires)"
                ));
            }
            let line = find_at(lines, "WARN", DEPTH_WARN_MARKER).expect("counted above");
            if !has_field(line, "topic", &loud_cer) {
                return Err(format!(
                    "the one warn must name the CLAMPED topic={loud_cer}, not the matching one; got: {line}"
                ));
            }
            if line.contains(&quiet_cer) {
                return Err(format!(
                    "the matching-depth topic {quiet_cer} must not appear on a depth warn; got: {line}"
                ));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, loud_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, quiet_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The SUBSCRIPTION half, where the decision is loudest: an rmw subscription's
/// requested depth is not applied to its queue AT ALL, so
/// endpoint info reports the transport default it really got — and the create
/// warns, naming both numbers.
///
/// Reporting the request here would be the echo the decision kills, on the one
/// endpoint class where the request changed literally nothing.
#[test]
#[traced_test]
#[serial]
fn a_subscription_reports_the_provisioned_default_depth_and_warns() {
    unsafe {
        const ASKED: usize = 200;
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlS{suffix}"));
        let (_, node, _opts) = setup_node(&format!("tl_sub_node_{suffix}"));
        let ros_topic = format!("/rmw_tl/sub/{suffix}");
        let cer_topic = ros_topic.to_string();
        let topic = CString::new(ros_topic).expect("topic");
        let qos = volatile_qos(ASKED);
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_subscriptions_info_by_topic(
                node,
                &mut allocator,
                topic.as_ptr(),
                false,
                &mut arr
            ),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        assert_eq!(
            (*arr.info_array).qos_profile.depth,
            PROVISIONED_DEFAULT_DEPTH,
            "a subscription's endpoint info must report the queue depth it really got \
             ({PROVISIONED_DEFAULT_DEPTH}), never an echo of the {ASKED} it asked for and \
             that was never applied"
        );
        // The non-depth axes are UNCHANGED by this decision — they still carry
        // the requested values, pending the getter rebuild.
        assert_eq!(
            (*arr.info_array).qos_profile.durability,
            ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE
        );

        logs_assert(move |lines: &[&str]| {
            let warns = count_at(lines, "WARN", DEPTH_WARN_MARKER);
            if warns != 1 {
                return Err(format!(
                    "expected EXACTLY 1 WARN depth line at subscription create, got {warns}"
                ));
            }
            let line = find_at(lines, "WARN", DEPTH_WARN_MARKER).expect("counted above");
            if !has_field(line, "topic", &cer_topic) {
                return Err(format!("the warn must name topic={cer_topic}; got: {line}"));
            }
            if !has_field(line, "requested_depth", &ASKED.to_string()) {
                return Err(format!(
                    "the warn must name requested_depth={ASKED}; got: {line}"
                ));
            }
            if !has_field(
                line,
                "provisioned_depth",
                &PROVISIONED_DEFAULT_DEPTH.to_string(),
            ) {
                return Err(format!(
                    "the warn must name provisioned_depth={PROVISIONED_DEFAULT_DEPTH}; got: {line}"
                ));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// (iii) VOLATILE → no history, no ceiling raise, no 75% warn.
// =====================================================================

/// RENAMED to say which warn it owns: the cerulion_core 75% one,
/// which is the only warn this arm asserts about. (A VOLATILE publisher is now
/// quiet on the create-time DEPTH warn too — that is
/// [`a_volatile_publisher_reports_the_zero_it_retains_and_stays_quiet`]'s
/// business, and it proves the silence by PAIRING it with a publisher that must
/// warn, which a bare `..._does_not_warn` here could not do.)
#[test]
#[traced_test]
#[serial]
fn volatile_publisher_does_not_fire_the_core_75_percent_warn() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlV{suffix}"));
        let (_, node, _opts) = setup_node(&format!("tl_vol_node_{suffix}"));
        let topic = CString::new(format!("/rmw_tl/v/{suffix}")).expect("topic");
        let qos = volatile_qos(16); // depth 16 but VOLATILE → history 0
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        assert!(
            !logs_contain(WARN_SUBSTR),
            "VOLATILE keeps history 0 — the warn is guarded by history_size > 0"
        );
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// **The VOLATILE adjudication, made a recorded decision rather than a
/// side effect.** The retention-depth reporting rule names TL depth-1000 → 16 and
/// TL depth-5 → 5 explicitly and says "VOLATILE → its real retention"; the code
/// answers that question with `history = 0` for any non-TRANSIENT_LOCAL
/// durability (`api/pubsub.rs`), and cerulion_core confirms what 0 MEANS: a
/// `history_size == 0` service "retained nothing", so a late joiner "receives
/// only post-match samples (the DDS VOLATILE contract)"
/// (`transport/publisher.rs`, the `SentHistory` suppression gate).
///
/// So endpoint info reports 0 — **and the create stays QUIET**
/// (warnings are scoped to real clamps). Retention 0 is what the caller's own
/// durability policy MEANS, not a divergence from their ask, and there is
/// nothing for them to act on; a bare `requested != provisioned` warn would put
/// a line on every create of the ordinary ROS path (rclcpp's default QoS is
/// `KeepLast(10)` + VOLATILE) saying only "you chose VOLATILE". This arm owns
/// BOTH halves of that decision, which is why they share one body: the reported
/// number and the silence are one decision.
///
/// The forbidden-line half is PAIRED, in this same capture, with a publisher
/// that MUST warn — otherwise "no depth warn fired" is satisfied by a build in
/// which the warn does not exist at all. The paired positive is the decision's
/// OTHER named case, the degenerate `TRANSIENT_LOCAL` depth 0 → 1 raise, so
/// the pin doubles as the clamp's FLOOR-side oracle (its ceiling side is
/// `a_clamped_depth_warns_at_create_naming_both_numbers`).
#[test]
#[traced_test]
#[serial]
fn a_volatile_publisher_reports_the_zero_it_retains_and_stays_quiet() {
    unsafe {
        const ASKED: usize = 10; // rclcpp's default depth
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlZ{suffix}"));
        let (_, node, _opts) = setup_node(&format!("tl_vol0_node_{suffix}"));
        let ros_topic = format!("/rmw_tl/vol0/{suffix}");
        let cer_topic = ros_topic.to_string();
        let topic = CString::new(ros_topic).expect("topic");
        let qos = volatile_qos(ASKED);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        assert_eq!(
            (*arr.info_array).qos_profile.depth,
            0,
            "a VOLATILE publisher retains NOTHING for a late joiner, so the depth it \
             reports is 0 — never the {ASKED} it asked for, and never the transport's \
             {PROVISIONED_DEFAULT_DEPTH}-slot ceiling (by design)"
        );
        // The durability axis is what EXPLAINS the 0 — a reader seeing depth 0
        // beside VOLATILE has the whole story, which is why 0 is accurate here
        // rather than merely small.
        assert_eq!(
            (*arr.info_array).qos_profile.durability,
            ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE
        );

        // The paired positive: the decision's other named case, a TRANSIENT_LOCAL
        // depth 0 RAISED to the clamp floor 1 — a real clamp of the caller's
        // own number, so it must be loud.
        const FLOOR_ASKED: usize = 0;
        const FLOOR_PROVISIONED: usize = 1;
        let loud_ts = point_ts(&format!("TlF{suffix}"));
        let loud_ros = format!("/rmw_tl/floor/{suffix}");
        let loud_cer = loud_ros.to_string();
        let loud_topic = CString::new(loud_ros).expect("topic");
        let loud_qos = transient_local_qos(FLOOR_ASKED);
        let loud_pub =
            rmw_create_publisher(node, loud_ts, loud_topic.as_ptr(), &loud_qos, &pub_opts);
        assert!(!loud_pub.is_null());

        logs_assert(move |lines: &[&str]| {
            let warns = count_at(lines, "WARN", DEPTH_WARN_MARKER);
            if warns != 1 {
                return Err(format!(
                    "expected EXACTLY 1 WARN depth line across BOTH creates, got {warns} \
                     (2 = the VOLATILE create warned, i.e. the durability gate is gone and \
                     every stock ROS publisher now logs; 0 = the warn never fires at all, \
                     so the quiet half proves nothing)"
                ));
            }
            let line = find_at(lines, "WARN", DEPTH_WARN_MARKER).expect("counted above");
            if !has_field(line, "topic", &loud_cer) {
                return Err(format!(
                    "the one warn must name the CLAMPED topic={loud_cer}, not the VOLATILE \
                     one; got: {line}"
                ));
            }
            if line.contains(&cer_topic) {
                return Err(format!(
                    "the VOLATILE topic {cer_topic} must not appear on a depth warn — \
                     retention 0 is what VOLATILE means, not a clamp; got: {line}"
                ));
            }
            // The floor raise is a REAL clamp and names both numbers.
            if !has_field(line, "requested_depth", &FLOOR_ASKED.to_string()) {
                return Err(format!(
                    "the warn must name requested_depth={FLOOR_ASKED}; got: {line}"
                ));
            }
            if !has_field(line, "provisioned_depth", &FLOOR_PROVISIONED.to_string()) {
                return Err(format!(
                    "the warn must name provisioned_depth={FLOOR_PROVISIONED} (the clamp \
                     FLOOR); got: {line}"
                ));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, loud_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// **The OPENED-SERVICE case.** A
/// publisher does not always get the history it asked for: iceoryx2's
/// open-time verification on `.history_size(N)` is AT-LEAST (existing < required
/// fails), so a request of 5 attaches happily to a service already created at
/// 16 — and the port it hands back is sized from the SERVICE's static config
/// (`iceoryx2-0.9.1/src/port/publisher.rs`: `Queue::new(static_config
/// .history_size)`), not from the request. The publisher then really retains
/// 16. Reporting the request would understate what late joiners receive, which
/// is the exact inversion of the principle these arms pin.
///
/// Reachable, and this arm builds the shape it comes from: a deeper publisher
/// created the service and DIED (its port dropped, its slot free) while a
/// subscriber holds the service open — a restart onto a live topic. The rmw
/// publisher that attaches next asked for 5.
///
/// The oracle is BEHAVIORAL and independent of the reported number: a late
/// joiner is drained and its frame count is what "really retains" MEANS. The
/// arm asserts the count first (so a transport change makes it fail as a
/// measurement, not as a mystifying constant), then requires endpoint info to
/// agree with it. Asserting `> SHALLOW` as well is what stops the pair being
/// satisfied by a build where both numbers are 5.
#[test]
#[traced_test]
#[serial]
fn a_publisher_that_opened_a_deeper_service_reports_the_depth_it_really_retains() {
    unsafe {
        const DEEP: usize = 16;
        const SHALLOW: usize = 5;
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlD{suffix}"));
        let (context, node, _opts) = setup_node(&format!("tl_deep_node_{suffix}"));
        let ros_topic = format!("/rmw_tl/deep/{suffix}");
        let cer_topic = ros_topic.to_string();
        let topic = CString::new(ros_topic).expect("topic");

        // (1) A deeper publisher CREATES the service at history DEEP, then
        // dies — while a subscriber keeps the service (and its static config)
        // alive. This is a producer restart onto a live topic.
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let deep_pub = rt
            .transport
            .create_publisher(&cer_topic, msl, DEEP)
            .expect("deep core publisher");
        let _holder = rt
            .transport
            .create_subscriber_open_only(&cer_topic)
            .expect("service holder");
        drop(deep_pub);

        // (2) The rmw publisher asks for SHALLOW and OPENS that service.
        let qos = transient_local_qos(SHALLOW);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(
            !publisher.is_null(),
            "a shallower TRANSIENT_LOCAL request must ATTACH to a deeper existing \
             service (iceoryx2 open verification is at-least) — if this ever starts \
             refusing, this whole arm is unreachable and should be revisited"
        );

        // (3) Publish DEEP frames into that history, before any subscriber.
        for i in 0..DEEP {
            let msg = CPoint {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
        }

        // (4) MEASURE the retention: a late joiner drains what the pump gives.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let mut received: BTreeSet<u64> = BTreeSet::new();
        for _ in 0..80 {
            let mut sub_ptrs = [(*subscription).data];
            let mut subs = ffi::rmw_subscriptions_t {
                subscriber_count: 1,
                subscribers: sub_ptrs.as_mut_ptr(),
            };
            let timeout = ffi::rmw_time_t {
                sec: 0,
                nsec: 50_000_000,
            };
            let _ = rmw_wait(
                &mut subs,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &timeout,
            );
            loop {
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
                if !taken {
                    break;
                }
                received.insert(out.x as u64);
            }
            if received.len() >= DEEP {
                break;
            }
        }
        let retained = received.len();
        assert_eq!(
            retained, DEEP,
            "the port is sized from the SERVICE's history ({DEEP}), not the request \
             ({SHALLOW}) — a late joiner really receives {DEEP} frames"
        );
        assert!(
            retained > SHALLOW,
            "the measured retention must EXCEED the request, or this arm cannot tell \
             a truthful report from an echo of the ask"
        );

        // (5) Endpoint info must agree with the measurement.
        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        assert_eq!(
            (*arr.info_array).qos_profile.depth,
            retained,
            "endpoint info must report the depth this publisher REALLY retains \
             ({retained}, measured above), not the {SHALLOW} it requested — reporting \
             the request understates what late joiners receive (Principle #3)"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// **The VOLATILE-on-a-deeper-service case.** One could
/// argue a VOLATILE publisher that attaches to a service someone created with
/// native history must still report 0, because 0 is "the promised" retention.
///
/// MEASURED, and it is not: such a publisher really does deliver its retained
/// frames to a late joiner. Every gate on the Cerulion side suppresses the
/// SentHistory *wake*, never the *delivery* — `deliver_history` runs
/// `update_connections()` (which is what pushes the frames) BEFORE the
/// `history_size == 0` early return, the rmw pump (`runtime::
/// pump_publisher_events`) calls it on every registered publisher with no
/// `has_history()` gate, and iceoryx2's own history queue exists whenever the
/// SERVICE's static history is nonzero (`port/publisher.rs`: `Queue::new(
/// static_config.history_size)`), so every `send` pushes into it.
///
/// So reporting 0 here would be the report-vs-reality divergence that
/// #3810891438 was raised to fix, pointed the other way: the row would promise
/// a late joiner nothing while the transport hands it a full backlog. This arm
/// exists so that claim is a MEASUREMENT rather than an argument — it counts
/// the frames first and only then asserts what endpoint info must say.
///
/// A FRESH VOLATILE publisher still reports 0, because iceoryx2's own default
/// `publisher_history_size` is 0 and Cerulion arms `.history_size()` only for a
/// nonzero request — pinned by
/// [`a_volatile_publisher_reports_the_zero_it_retains_and_stays_quiet`],
/// which is the arm that owns the decided VOLATILE value. The two together say
/// the rule precisely: VOLATILE reports the retention it HAS, which is normally
/// zero and is not zero when it inherited a deeper service.
#[test]
#[traced_test]
#[serial]
fn a_volatile_publisher_on_a_deeper_service_reports_what_it_really_delivers() {
    unsafe {
        const DEEP: usize = 16;
        const ASKED: usize = 10; // rclcpp's default depth, VOLATILE
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlV2{suffix}"));
        let (context, node, _opts) = setup_node(&format!("tl_vdeep_node_{suffix}"));
        let ros_topic = format!("/rmw_tl/vdeep/{suffix}");
        let cer_topic = ros_topic.to_string();
        let topic = CString::new(ros_topic).expect("topic");

        // A TRANSIENT_LOCAL-depth service exists and its creator has died,
        // while a subscriber holds it open.
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let msl = cerulion_core::wire::MaxSliceLen::try_new(4096).expect("slice len");
        let deep_pub = rt
            .transport
            .create_publisher(&cer_topic, msl, DEEP)
            .expect("deep core publisher");
        let _holder = rt
            .transport
            .create_subscriber_open_only(&cer_topic)
            .expect("service holder");
        drop(deep_pub);

        // A VOLATILE rmw publisher attaches and publishes BEFORE any late
        // joiner exists — under the DDS VOLATILE contract those frames are
        // supposed to be unreachable to a later subscriber.
        let qos = volatile_qos(ASKED);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        for i in 0..DEEP {
            let msg = CPoint {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
        }

        // MEASURE what a late joiner actually gets.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let ws = rmw_create_wait_set(context, 8);
        let mut received: BTreeSet<u64> = BTreeSet::new();
        for _ in 0..80 {
            let mut sub_ptrs = [(*subscription).data];
            let mut subs = ffi::rmw_subscriptions_t {
                subscriber_count: 1,
                subscribers: sub_ptrs.as_mut_ptr(),
            };
            let timeout = ffi::rmw_time_t {
                sec: 0,
                nsec: 50_000_000,
            };
            let _ = rmw_wait(
                &mut subs,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &timeout,
            );
            loop {
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
                if !taken {
                    break;
                }
                received.insert(out.x as u64);
            }
            if received.len() >= DEEP {
                break;
            }
        }
        let delivered = received.len();

        // THE MEASUREMENT. A VOLATILE request did not make the retention zero:
        // the service's history queue is what the port got, and it delivered.
        assert_eq!(
            delivered, DEEP,
            "a VOLATILE publisher that INHERITED a depth-{DEEP} service really \
             delivers its backlog to a late joiner — if this ever measures 0, the \
             delivery IS gated on the request after all and endpoint info should \
             report 0 here"
        );

        // ...so reporting 0 would promise a late joiner nothing while the
        // transport hands it {DEEP} frames — the same report-vs-reality
        // divergence #3810891438 fixed, pointed the other way.
        let mut allocator = malloc_allocator();
        let mut arr: ffi::rmw_topic_endpoint_info_array_t = std::mem::zeroed();
        assert_eq!(
            rmw_get_publishers_info_by_topic(node, &mut allocator, topic.as_ptr(), false, &mut arr),
            RMW_RET_OK
        );
        assert_eq!(arr.size, 1);
        assert_eq!(
            (*arr.info_array).qos_profile.depth,
            delivered,
            "endpoint info must report the {delivered} frames a late joiner MEASURABLY \
             receives, not the 0 a VOLATILE request would suggest"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// (iv) depth 1 (the /tf_static class) → no raise needed, none applied.
// =====================================================================

/// RENAMED for the same reason as the depth-16 arm: to say which
/// warn it owns, the cerulion_core 75% one. (Under the earlier slot-depth reporting rule a
/// depth-1 publisher was reported at the default-16 ceiling and so DID fire the
/// create-time depth warn; the retention-depth refinement reports the retained 1, so
/// it is quiet on both.)
#[test]
#[traced_test]
#[serial]
fn transient_local_depth_1_needs_no_raise_and_does_not_fire_the_core_75_percent_warn() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("Tl1{suffix}"));
        let (_, node, _opts) = setup_node(&format!("tl_one_node_{suffix}"));
        let topic = CString::new(format!("/rmw_tl/one/{suffix}")).expect("topic");
        // history 1: (1*4).div_ceil(3) = 2, .max(16) = 16 — no raise applied,
        // and 1 <= 16*3/4 = 12, so no warn either.
        let qos = transient_local_qos(1);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        assert!(!logs_contain(WARN_SUBSTR));
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// (ii) Behavioral proof: a late joiner receives ALL 16 history frames.
// =====================================================================

/// Pins TRANSIENT_LOCAL history DELIVERY correctness through the
/// rmw_wait pump (Principle #6, no data loss): a depth-16 publisher
/// publishes 16 frames BEFORE any subscriber exists, then a late joiner
/// receives ALL 16 against the hand oracle {0..15} — no loss, no
/// duplication (cribs `transient_local_history_reaches_late_joiner_via_
/// wait_pump`, extended to the full-history batch).
///
/// SCOPE: this test is independent
/// of the ceiling raise — with the raise reverted it still
/// passes, because in a quiet system 16 history frames deliver fine
/// into an un-raised ceiling-16 buffer (the warn's eviction risk needs
/// live traffic this test deliberately doesn't have). The RAISE is
/// pinned by the warn tests instead (reverting the
/// raise fails `transient_local_depth_16_publisher_does_not_warn`);
/// THIS test pins pump delivery.
#[test]
#[traced_test]
#[serial]
fn late_joiner_receives_all_16_history_frames() {
    unsafe {
        const N: usize = 16;
        let suffix = unique_suffix();
        let ts = point_ts(&format!("TlH{suffix}"));
        let (context, node, _opts) = setup_node(&format!("tl_hist_node_{suffix}"));
        let topic = CString::new(format!("/rmw_tl/hist/{suffix}")).expect("topic");
        let qos = transient_local_qos(N);
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // Publish 16 distinct frames BEFORE the subscriber exists → they
        // land in native history.
        for i in 0..N {
            let msg = CPoint {
                x: i as f64,
                y: 0.0,
                z: 0.0,
            };
            assert_eq!(
                rmw_publish(
                    publisher,
                    &msg as *const _ as *const c_void,
                    std::ptr::null_mut()
                ),
                RMW_RET_OK
            );
        }

        // Late joiner; the publisher is idle from here — only the pump
        // delivers.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        let ws = rmw_create_wait_set(context, 8);
        let mut received: BTreeSet<u64> = BTreeSet::new();
        for _ in 0..80 {
            let mut sub_ptrs = [(*subscription).data];
            let mut subs = ffi::rmw_subscriptions_t {
                subscriber_count: 1,
                subscribers: sub_ptrs.as_mut_ptr(),
            };
            let timeout = ffi::rmw_time_t {
                sec: 0,
                nsec: 50_000_000,
            };
            let _ = rmw_wait(
                &mut subs,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                ws,
                &timeout,
            );
            // Drain everything the pump delivered this cycle.
            loop {
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
                if !taken {
                    break;
                }
                // Full payload integrity (also reads every field): the
                // frames carried x = index, y = z = 0.
                assert_eq!(out.y, 0.0);
                assert_eq!(out.z, 0.0);
                received.insert(out.x as u64);
            }
            if received.len() >= N {
                break;
            }
        }

        // Hand oracle: exactly {0, 1, .., 15} — the full history, no loss,
        // no duplication.
        let expected: BTreeSet<u64> = (0..N as u64).collect();
        assert_eq!(
            received, expected,
            "late joiner must receive every history frame via the wait pump"
        );

        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
