// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The FIRST end-to-end harness for `rmw_publish_serialized_message`
//! (`rmw_cerulion/src/api/misc.rs`), and the flood latch on its two
//! frame-REJECT arms.
//!
//! # Coverage of the serialization family
//!
//! This harness is the only test that calls `rmw_publish_serialized_message`, and
//! the serialization family around it is thinner than it looks. Of its three
//! siblings only `rmw_take_serialized_message` has arms (in `rmw_e2e_test.rs`,
//! via itself and `..._with_info`); `rmw_serialize` has its only test caller
//! in this file's [`serialize`] helper; and `rmw_deserialize` has NONE —
//! including the null-buffer guard cited below as the precedent for the one on
//! the publish path, which is therefore itself unpinned. The entry point that
//! ACCEPTS a caller-supplied wire frame and validates it against a publisher's
//! type is covered here alone, so both of its reject arms and its
//! sequence/timestamp re-stamp rest on this file.
//!
//! # The flood shape
//!
//! The function rejects a frame on TWO adjacent arms — a malformed wire header
//! and a schema-hash mismatch — and each was a BARE per-call `error!`. It is
//! the per-MESSAGE publish entry point: rosbag-class callers (`ros2 bag play`,
//! any serialized republisher) hit it once per frame, so a sustained skew
//! floods at frame rate. That is the disk-fill class (a per-publish
//! `warn!` filled a 234 GB robot disk) and exactly what the shared latch killed on the
//! TAKE side. Each arm now rides its own `FailureRegimeLatch`.
//!
//! # Two latches, not one
//!
//! A malformed header (the bytes are not a Cerulion frame) and a hash mismatch
//! (they are a frame, of the WRONG type) are different conditions with
//! different remedies, so sharing a regime would let either mask the other's
//! loud head — pinned by
//! [`an_open_header_regime_does_not_swallow_the_hash_reject_head`].
//!
//! Its OWN binary, not an arm of `rmw_e2e_test.rs`, because `#[traced_test]`
//! installs a GLOBAL tracing subscriber: any sibling test that brings the rmw
//! runtime up first takes that slot and the capture then panics
//! (`SetGlobalDefaultError`). Same shape and same reason as
//! `rmw_schema_mismatch_test.rs` and `rmw_transient_local_ceiling_test.rs`.
//!
//! ⚠️ Shares the iceoryx2 SHM singleton — run serial:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_publish_reject_test -- --test-threads=1
//! ```

use cerulion_core::testing::{
    count_at, count_at_exclusively, debug_level_compiled_in, debug_lines_expected, line_level,
    lines_at_exclusively, never_loud,
};
use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};

use rmw_cerulion::ffi::{self, RMW_RET_INVALID_ARGUMENT, RMW_RET_OK};
use rmw_cerulion::*;
// no-env-filter so the capture reaches every target, not just this test crate.
//
// EVERY test in this binary carries `#[traced_test]`, including the ones that
// assert nothing about logs. The macro's `set_global_default(...).expect(..)`
// runs once per binary, and `runtime()` installs its OWN stderr subscriber
// with `try_init` the first time the rmw runtime comes up — so a NON-traced
// test running first would take the global slot and make the first traced test
// panic with `SetGlobalDefaultError`. Tagging them all keeps the traced
// subscriber first, whatever order libtest picks.
use tracing_test::traced_test;

// =====================================================================
// Fixtures (cribbed from rmw_e2e_test.rs / rmw_schema_mismatch_test.rs)
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;

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

/// A REAL malloc-backed `rcutils_allocator_t` — rcl always passes a functional
/// allocator; a zeroed one (all-`None` fn pointers) only exercises BAD_ALLOC.
fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(fixture_allocate);
    a.deallocate = Some(fixture_deallocate);
    a
}

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
        "rmw_reject__msg",
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
// Serialized-message helpers
// =====================================================================

/// An owned `rmw_serialized_message_t` — freed on `Drop` through the same
/// malloc allocator rmw filled it with, so no arm leaks its frame.
struct Serialized(ffi::rmw_serialized_message_t);

impl Serialized {
    /// An empty message rmw will allocate into (the rcl calling convention).
    fn empty() -> Self {
        let mut m: ffi::rmw_serialized_message_t = unsafe { std::mem::zeroed() };
        m.allocator = malloc_allocator();
        Self(m)
    }

    /// A message wrapping a HAND-BUILT byte string — the shape a bag player
    /// hands us when its frame did not come from this `rmw_serialize`.
    fn from_bytes(bytes: &[u8]) -> Self {
        assert!(!bytes.is_empty(), "use `null_buffer` for the empty case");
        let mut m = Self::empty();
        unsafe {
            let buf = malloc(bytes.len()) as *mut u8;
            assert!(!buf.is_null(), "fixture malloc failed");
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), buf, bytes.len());
            m.0.buffer = buf;
            m.0.buffer_length = bytes.len();
            m.0.buffer_capacity = bytes.len();
        }
        m
    }

    /// An UNINITIALISED serialized message: NULL buffer, zero length — exactly
    /// what `rcutils_get_zero_initialized_uint8_array()` yields, so a caller
    /// that forgets to fill it hands us this.
    fn null_buffer() -> Self {
        Self::empty()
    }

    fn as_ptr(&self) -> *const ffi::rmw_serialized_message_t {
        &self.0
    }

    fn bytes(&self) -> &[u8] {
        if self.0.buffer.is_null() {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.0.buffer as *const u8, self.0.buffer_length) }
    }
}

impl Drop for Serialized {
    fn drop(&mut self) {
        if !self.0.buffer.is_null() {
            unsafe { free(self.0.buffer as *mut c_void) };
        }
    }
}

/// Serialize `msg` against `ts` through the REAL `rmw_serialize` — the only
/// producer of a frame `rmw_publish_serialized_message` is contracted to accept.
unsafe fn serialize(msg: &CPoint, ts: *const ffi::rosidl_message_type_support_t) -> Serialized {
    let mut out = Serialized::empty();
    assert_eq!(
        rmw_serialize(msg as *const _ as *const c_void, ts, &mut out.0),
        RMW_RET_OK,
        "rmw_serialize must produce a frame"
    );
    assert!(out.0.buffer_length >= cerulion_core::wire::WireHeader::SIZE);
    out
}

/// Take one message, returning `(taken, value, publication_sequence_number)`.
///
/// # Safety
/// `subscription` must be a live subscription created by this implementation.
unsafe fn take_one(subscription: *const ffi::rmw_subscription_t) -> (bool, CPoint, u64) {
    let mut out = CPoint::default();
    let mut taken = true;
    let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
    assert_eq!(
        rmw_take_with_info(
            subscription,
            &mut out as *mut _ as *mut c_void,
            &mut taken,
            &mut info,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    (taken, out, info.publication_sequence_number)
}

// =====================================================================
// Publish-reject latch oracles
// =====================================================================

/// Substring unique to the LOUD (`error!`) arm of the MALFORMED-HEADER report.
const HEADER_LOUD: &str = "REJECTING a serialized frame whose wire header is malformed";
/// Substring unique to its DECADE RE-ANNOUNCEMENT (`error!`) arm.
const HEADER_STILL: &str = "malformed serialized frames are STILL being rejected";
/// Substring unique to its SUPPRESSED (`debug!`) arm.
const HEADER_SUPPRESSED: &str = "malformed-header reject suppressed";
/// Substring unique to its RECOVERY (`info!`) arm.
const HEADER_RECOVERY: &str = "serialized frames carry a well-formed wire header again";

/// Substring unique to the LOUD (`error!`) arm of the SCHEMA-HASH report.
const HASH_LOUD: &str = "REJECTING a serialized frame whose wire schema hash does not match";
/// Substring unique to its DECADE RE-ANNOUNCEMENT (`error!`) arm.
const HASH_STILL: &str = "schema-hash rejects are STILL refusing every serialized frame";
/// Substring unique to its SUPPRESSED (`debug!`) arm.
const HASH_SUPPRESSED: &str = "schema-hash reject suppressed";
/// Substring unique to its RECOVERY (`info!`) arm.
const HASH_RECOVERY: &str = "serialized frames match the publisher type again";

/// Every level token `tracing` can render, so a line's level can be read out
/// rather than searched for.
/// The first captured line at `level` carrying `marker`, for the arms that
/// assert on a line's FIELDS rather than counting lines.
///
/// Use [`lines_at`] instead wherever a test produces MORE THAN ONE such line
/// and the field under assertion could differ between them — the
/// unfilled-message arm drives two publishers, and inspecting only
/// the first head's `buffer_len` would miss the second.
fn find_at<'a>(lines: &[&'a str], level: &str, marker: &str) -> Option<&'a str> {
    lines
        .iter()
        .copied()
        .find(|l| line_level(l) == Some(level) && l.contains(marker))
}

/// True iff `line` carries the structured FIELD `key=value` — as a whole
/// whitespace token, not as a substring.
///
/// The distinction is load-bearing and cost an earlier revision a false pass. The
/// malformed-header message names the unfilled case in PROSE, including the
/// literal ``(`buffer_len=0`)``, so a `line.contains("buffer_len=0")` predicate
/// is satisfied by every head ever emitted — regardless of what the FIELD says.
/// That is precisely how the first attempt at the "every head carries the
/// diagnostic" fix still passed against the `input.buffer_capacity` regression
/// it was written to catch. Tokenising fixes it: the prose occurrence is
/// ``(`buffer_len=0`)``, backticks and parens included, which is not the token
/// `buffer_len=0`.
///
/// Only for fields whose VALUE contains no whitespace (every numeric field, and
/// the topic name) — `kind="serialized message"` would need quoting-aware
/// parsing and is asserted by other means.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let needle = format!("{key}={value}");
    line.split_whitespace().any(|token| token == needle)
}

/// The publisher's malformed-header reject counter — the
/// log-level-independent Principle #3 signal.
///
/// This cast is the ONLY way a ROS caller's counterpart could reach it: the rmw
/// C ABI is standardized, so rclcpp/rclpy see an opaque `*mut c_void` and no
/// accessor can be added for them. That is exactly why the shared latch
/// re-announces an open regime at each decade — the log is their whole window.
///
/// # Safety
/// `publisher` must be a live publisher created by this implementation.
unsafe fn header_reject_count(publisher: *const ffi::rmw_publisher_t) -> u64 {
    let data = &*((*publisher).data as *const rmw_cerulion::runtime::PublisherData);
    data.malformed_header_reject_count()
}

/// The publisher's schema-hash reject counter (the sibling latch).
///
/// # Safety
/// `publisher` must be a live publisher created by this implementation.
unsafe fn hash_reject_count(publisher: *const ffi::rmw_publisher_t) -> u64 {
    let data = &*((*publisher).data as *const rmw_cerulion::runtime::PublisherData);
    data.schema_hash_reject_count()
}

// =====================================================================
// The behavioral floor (no latch involved)
// =====================================================================

/// HAPPY PATH: a frame produced by `rmw_serialize` publishes through
/// `rmw_publish_serialized_message` and reaches a subscriber INTACT.
///
/// Hand oracles (never a self-compare): three distinct `CPoint`s go in and the
/// same three come out, in order, and each carries the wire SEQUENCE the
/// publish site re-stamped — 0, 1, 2. The re-stamp is load-bearing and was
/// untested: `rmw_serialize` stamps 0/0 into every frame it builds, so
/// republishing a bag would otherwise put a stream of seq-0 frames on the wire.
#[test]
#[serial]
#[traced_test]
fn serialized_publish_round_trips_and_restamps_the_sequence() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("RejectHappy{suffix}"));
        let (_, node, _opts) = setup_node(&format!("happy_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/happy/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let oracle = [
            CPoint {
                x: 1.5,
                y: -2.5,
                z: 3.25,
            },
            CPoint {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
            CPoint {
                x: -0.125,
                y: 0.0,
                z: 99.5,
            },
        ];

        for (i, expected) in oracle.iter().enumerate() {
            let frame = serialize(expected, ts);
            // `rmw_serialize` stamps sequence 0 into EVERY frame it builds —
            // the value the publish site must overwrite.
            let stamped = cerulion_core::wire::WireHeader::read_from_buf(frame.bytes())
                .expect("serialized frame carries a header");
            assert_eq!(stamped.sequence, 0, "rmw_serialize always stamps seq 0");

            assert_eq!(
                rmw_publish_serialized_message(publisher, frame.as_ptr(), std::ptr::null_mut()),
                RMW_RET_OK
            );
            let (taken, got, seq) = take_one(subscription);
            assert!(taken, "a well-formed frame must be delivered");
            assert_eq!(got, *expected, "the payload must survive the round trip");
            assert_eq!(
                seq, i as u64,
                "the publish site must RE-STAMP the wire sequence — a bag \
                 replayed through here would otherwise publish seq 0 forever"
            );
        }

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A frame too short to hold the 32-byte wire header is REJECTED with
/// `RMW_RET_INVALID_ARGUMENT` and nothing reaches the subscriber.
///
/// Hand oracle: the subscriber sees NO message at all after the reject, and
/// still delivers the very next well-formed frame — a rejected frame must not
/// wedge the publisher.
#[test]
#[serial]
#[traced_test]
fn a_malformed_header_is_rejected_and_nothing_is_published() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("RejectShort{suffix}"));
        let (_, node, _opts) = setup_node(&format!("short_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/short/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // A TRUNCATED frame: 16 bytes where the header alone needs 32.
        let truncated = Serialized::from_bytes(&[0xABu8; 16]);
        assert_eq!(
            rmw_publish_serialized_message(publisher, truncated.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT,
            "a buffer too short for the wire header is not a Cerulion frame"
        );
        let (taken, _, _) = take_one(subscription);
        assert!(!taken, "a rejected frame must never reach the wire");

        // The publisher still works — a reject is per-frame, not terminal.
        let healthy = CPoint {
            x: 7.0,
            y: 8.0,
            z: 9.0,
        };
        let frame = serialize(&healthy, ts);
        assert_eq!(
            rmw_publish_serialized_message(publisher, frame.as_ptr(), std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (taken, got, seq) = take_one(subscription);
        assert!(taken);
        assert_eq!(got, healthy);
        assert_eq!(
            seq, 0,
            "a REJECTED frame must not burn a wire sequence number"
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// A well-formed frame serialized against a DIFFERENT type is REJECTED with
/// `RMW_RET_INVALID_ARGUMENT` and nothing reaches the subscriber.
///
/// The skew is built the way a real one arises: two typesupports with the same
/// layout and DIFFERENT type names, so the schema hashes differ while the
/// frames stay structurally publishable on one topic. This is the shape a bag
/// recorded against an older message definition has.
#[test]
#[serial]
#[traced_test]
fn a_schema_hash_mismatch_is_rejected_and_nothing_is_published() {
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("RejectOurs{suffix}"));
        let skewed = point_ts(&format!("RejectSkewed{suffix}"));
        let (_, node, _opts) = setup_node(&format!("skew_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/skew/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ours, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let msg = CPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let wrong_type = serialize(&msg, skewed);
        let right_type = serialize(&msg, ours);
        // Same layout, same length — ONLY the schema hash differs. Without
        // this the test could pass on a length check instead of the hash gate.
        assert_eq!(
            wrong_type.bytes().len(),
            right_type.bytes().len(),
            "the two frames must differ ONLY in their schema hash"
        );
        let wrong_hash = cerulion_core::wire::WireHeader::read_from_buf(wrong_type.bytes())
            .expect("header")
            .schema_hash;
        let right_hash = cerulion_core::wire::WireHeader::read_from_buf(right_type.bytes())
            .expect("header")
            .schema_hash;
        assert_ne!(wrong_hash, right_hash, "the fixture must really be skewed");

        assert_eq!(
            rmw_publish_serialized_message(publisher, wrong_type.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT,
            "a wrong-typed frame must be refused BEFORE it reaches the topic"
        );
        let (taken, _, _) = take_one(subscription);
        assert!(!taken, "a rejected frame must never reach the wire");

        // The matching frame still publishes.
        assert_eq!(
            rmw_publish_serialized_message(publisher, right_type.as_ptr(), std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (taken, got, _) = take_one(subscription);
        assert!(taken);
        assert_eq!(got, msg);

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The null-argument guards, which no test reached before this harness.
#[test]
#[serial]
#[traced_test]
fn null_arguments_are_rejected() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("RejectNull{suffix}"));
        let (_, node, _opts) = setup_node(&format!("null_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/null/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let msg = CPoint::default();
        let frame = serialize(&msg, ts);
        assert_eq!(
            rmw_publish_serialized_message(std::ptr::null(), frame.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(
            rmw_publish_serialized_message(publisher, std::ptr::null(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );

        // An UNINITIALISED serialized message (NULL buffer, zero length).
        // `std::slice::from_raw_parts` over a null pointer is UB even at
        // length zero, so the publish site builds the empty slice by hand
        // rather than calling it; `rmw_deserialize` has had a null guard from
        // the start and the publish site had none.
        //
        // MEASURED, not argued: deleting that handling makes this arm ABORT the
        // whole test process (SIGABRT, "unsafe precondition(s) violated:
        // slice::from_raw_parts requires the pointer to be aligned and
        // non-null"), because a debug build checks that precondition. Only the
        // RETURN CODE is asserted here; that an unfilled message is also a
        // LATCHED, counted malformed-header reject — the same treatment its
        // indistinguishable zero-LENGTH twin gets — is
        // `an_unfilled_serialized_message_is_a_latched_malformed_header_reject`.
        let uninit = Serialized::null_buffer();
        assert_eq!(
            rmw_publish_serialized_message(publisher, uninit.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// The flood latch on the two reject arms
// =====================================================================

/// The reject latch at the PRODUCTION `rmw_publish_serialized_message` site, MALFORMED
/// HEADER arm.
///
/// A bag player feeding frames this implementation cannot parse rejects EVERY
/// frame until the source is re-recorded — the earlier site emitted one
/// `error!` per call for as long as that lasted.
///
/// Hand oracles: N=6 truncated frames ⇒ exactly ONE loud head + 5 suppressed
/// repeats + an UNCONDITIONAL counter of 6; then a well-formed frame ⇒ exactly
/// one recovery line carrying the SUPPRESSED count (5, not 6 — the loud head
/// was never suppressed), the frame actually published, and the counter NOT
/// reset; then a fresh truncated frame is loud again (the re-arm).
#[test]
#[serial]
#[traced_test]
fn malformed_header_rejects_are_loud_once_counted_always_and_recover() {
    const BAD_FRAMES: usize = 6;
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("RejectHdrRegime{suffix}"));
        let (_, node, _opts) = setup_node(&format!("hdr_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/hdr/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        assert_eq!(
            header_reject_count(publisher),
            0,
            "a fresh publisher must start clean"
        );

        // 16 bytes where the header alone needs 32 — a truncated frame.
        let truncated = Serialized::from_bytes(&[0x11u8; 16]);
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(publisher, truncated.as_ptr(), std::ptr::null_mut()),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(
            header_reject_count(publisher),
            BAD_FRAMES as u64,
            "the counter is UNCONDITIONAL — it must count the debug-suppressed \
             repeats too, or a publisher rejecting everything is invisible at \
             RUST_LOG=error"
        );
        assert_eq!(
            hash_reject_count(publisher),
            0,
            "a frame that never reached the hash gate must not touch its counter"
        );

        // Recovery: the operator re-records, and well-formed frames arrive.
        let healed = CPoint {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        };
        let good = serialize(&healed, ts);
        assert_eq!(
            rmw_publish_serialized_message(publisher, good.as_ptr(), std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (taken, got, _) = take_one(subscription);
        assert!(taken, "the recovered frame must actually be published");
        assert_eq!(got, healed);
        assert_eq!(
            header_reject_count(publisher),
            BAD_FRAMES as u64,
            "recovery must NEVER reset the running total"
        );

        // Re-armed: a fresh truncated frame is loud again.
        assert_eq!(
            rmw_publish_serialized_message(publisher, truncated.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(header_reject_count(publisher), BAD_FRAMES as u64 + 1);

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[HEADER_LOUD])?;
            never_loud(lines, HEADER_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[HEADER_SUPPRESSED])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[HEADER_RECOVERY])?;
            if heads != 2 {
                return Err(format!(
                    "expected exactly 2 ERROR loud heads (the {BAD_FRAMES}-frame regime, \
                     then the re-armed one) — a per-frame error! would give {}, got {heads}",
                    BAD_FRAMES + 1
                ));
            }
            let want_debugs = debug_lines_expected(BAD_FRAMES - 1);
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
            // The headline regression: the suppressed arm emitted at `error!` keeps
            // every count above intact while suppression does nothing.
            let leaked = count_at(lines, "ERROR", HEADER_SUPPRESSED);
            if leaked != 0 {
                return Err(format!(
                    "the suppressed arm must be DEBUG, found {leaked} at ERROR"
                ));
            }
            let rec = find_at(lines, "INFO", HEADER_RECOVERY).ok_or("no INFO recovery line")?;
            if !has_field(rec, "suppressed_count", &(BAD_FRAMES - 1).to_string()) {
                return Err(format!(
                    "recovery must report the {} SUPPRESSED (not all {BAD_FRAMES}): {rec}",
                    BAD_FRAMES - 1
                ));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The reject latch at the PRODUCTION site, SCHEMA-HASH arm — same contract, its own
/// latch.
///
/// A bag recorded against an older message definition replays wrong-typed
/// frames at frame rate; earlier each one was an `error!`.
///
/// Hand oracles: N=6 skewed frames ⇒ 1 loud head + 5 suppressed repeats +
/// counter 6; then a matching frame ⇒ one recovery carrying
/// `suppressed_count=5` and an actual publish; then a fresh skew is loud again.
#[test]
#[serial]
#[traced_test]
fn schema_hash_rejects_are_loud_once_counted_always_and_recover() {
    const BAD_FRAMES: usize = 6;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("RejectHashOurs{suffix}"));
        let skewed = point_ts(&format!("RejectHashSkewed{suffix}"));
        let (_, node, _opts) = setup_node(&format!("hash_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/hash/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        let subscription = rmw_create_subscription(node, ours, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ours, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        assert_eq!(hash_reject_count(publisher), 0);

        let msg = CPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let wrong_type = serialize(&msg, skewed);
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(
                    publisher,
                    wrong_type.as_ptr(),
                    std::ptr::null_mut()
                ),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(
            hash_reject_count(publisher),
            BAD_FRAMES as u64,
            "the counter is UNCONDITIONAL"
        );
        assert_eq!(
            header_reject_count(publisher),
            0,
            "a well-formed header must never touch the malformed-header counter"
        );

        // Recovery: a frame serialized against the publisher's own type.
        let right_type = serialize(&msg, ours);
        assert_eq!(
            rmw_publish_serialized_message(publisher, right_type.as_ptr(), std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (taken, got, _) = take_one(subscription);
        assert!(taken);
        assert_eq!(got, msg);
        assert_eq!(
            hash_reject_count(publisher),
            BAD_FRAMES as u64,
            "recovery must NEVER reset the running total"
        );

        // Re-armed.
        assert_eq!(
            rmw_publish_serialized_message(publisher, wrong_type.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(hash_reject_count(publisher), BAD_FRAMES as u64 + 1);

        logs_assert(|lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[HASH_LOUD])?;
            never_loud(lines, HASH_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
            let recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
            if heads != 2 {
                return Err(format!(
                    "expected exactly 2 ERROR loud heads — a per-frame error! would give \
                     {}, got {heads}",
                    BAD_FRAMES + 1
                ));
            }
            let want_debugs = debug_lines_expected(BAD_FRAMES - 1);
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
            if count_at(lines, "ERROR", HASH_SUPPRESSED) != 0 {
                return Err("the suppressed arm must be DEBUG".to_string());
            }
            let rec = find_at(lines, "INFO", HASH_RECOVERY).ok_or("no INFO recovery line")?;
            if !has_field(rec, "suppressed_count", &(BAD_FRAMES - 1).to_string()) {
                return Err(format!(
                    "recovery must report the {} SUPPRESSED, not all {BAD_FRAMES}: {rec}",
                    BAD_FRAMES - 1
                ));
            }
            // Both hashes are the diagnosis — they name which type the
            // publisher expected and which one the frame carried.
            let head = find_at(lines, "ERROR", HASH_LOUD).ok_or("no ERROR loud head")?;
            for needle in ["expected_hash=0x", "actual_hash=0x"] {
                if !head.contains(needle) {
                    return Err(format!("loud head is missing {needle}: {head}"));
                }
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// SEPARATENESS — the design claim that the two reject conditions ride
/// DIFFERENT latches, driven in BOTH directions.
///
/// "Your bytes are not a frame" and "your frame is the wrong type" have
/// different remedies (re-record the source vs re-record against the current
/// definition), so one open regime must never swallow the other's loud head.
/// Merging them leaves every other arm in this file green — which is exactly
/// why this arm exists.
///
/// Phases, all against hand oracles:
///
/// * **A** — 3 truncated frames open a HEADER regime (head + 2 suppressed).
/// * **B** — a hash-skewed frame. Its header PARSES, so it CLOSES the header
///   regime (one recovery, `suppressed_count=2`) and its hash reject must be a
///   LOUD head, not a repeat of somebody else's regime.
/// * **C** — the reverse direction: another truncated frame. It returns before
///   the hash gate, so the hash counter must not move, and the header regime
///   (closed in B) re-opens LOUD.
/// * **D** — another skewed frame. Nothing closed the HASH regime (C never
///   reached that gate), so this must be a SUPPRESSED repeat, not a second
///   head. That half is what proves the regimes are genuinely independent
///   rather than merely both-loud.
#[test]
#[serial]
#[traced_test]
fn an_open_header_regime_does_not_swallow_the_hash_reject_head() {
    const HEADER_FRAMES: usize = 3;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("RejectSepOurs{suffix}"));
        let skewed = point_ts(&format!("RejectSepSkewed{suffix}"));
        let (_, node, _opts) = setup_node(&format!("sep_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/sep/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ours, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let truncated = Serialized::from_bytes(&[0x22u8; 16]);
        let msg = CPoint {
            x: -1.0,
            y: -2.0,
            z: -3.0,
        };
        let wrong_type = serialize(&msg, skewed);

        // Phase A — open a HEADER regime.
        for _ in 0..HEADER_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(publisher, truncated.as_ptr(), std::ptr::null_mut()),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(header_reject_count(publisher), HEADER_FRAMES as u64);
        assert_eq!(hash_reject_count(publisher), 0);

        // Phase B — with that regime open, a hash reject must STILL be loud.
        assert_eq!(
            rmw_publish_serialized_message(publisher, wrong_type.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(hash_reject_count(publisher), 1);
        assert_eq!(
            header_reject_count(publisher),
            HEADER_FRAMES as u64,
            "a parseable header is not a header reject"
        );

        // Phase C — reverse: a truncated frame must not touch the hash latch.
        assert_eq!(
            rmw_publish_serialized_message(publisher, truncated.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(header_reject_count(publisher), HEADER_FRAMES as u64 + 1);
        assert_eq!(
            hash_reject_count(publisher),
            1,
            "a malformed frame returns BEFORE the hash gate — that counter must \
             not move"
        );

        // Phase D — the hash regime was never closed, so this is a repeat.
        assert_eq!(
            rmw_publish_serialized_message(publisher, wrong_type.as_ptr(), std::ptr::null_mut()),
            RMW_RET_INVALID_ARGUMENT
        );
        assert_eq!(hash_reject_count(publisher), 2);

        logs_assert(|lines: &[&str]| {
            let header_heads = count_at_exclusively(lines, "ERROR", &[HEADER_LOUD])?;
            never_loud(lines, HEADER_SUPPRESSED)?;
            let header_debugs = count_at_exclusively(lines, "DEBUG", &[HEADER_SUPPRESSED])?;
            let header_recoveries = count_at_exclusively(lines, "INFO", &[HEADER_RECOVERY])?;
            let hash_heads = count_at_exclusively(lines, "ERROR", &[HASH_LOUD])?;
            never_loud(lines, HASH_SUPPRESSED)?;
            let hash_debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
            let hash_recoveries = count_at_exclusively(lines, "INFO", &[HASH_RECOVERY])?;
            if hash_heads != 1 {
                return Err(format!(
                    "an OPEN header regime must not swallow the hash reject's loud head: \
                     expected 1 ERROR, got {hash_heads}"
                ));
            }
            let want_hash_debugs = debug_lines_expected(1);
            if hash_debugs != want_hash_debugs {
                return Err(format!(
                    "phase D must be a SUPPRESSED repeat of the still-open hash regime \
                     (a malformed frame in between cannot close it): expected {want_hash_debugs} DEBUG, \
                     got {hash_debugs}"
                ));
            }
            if hash_recoveries != 0 {
                return Err(format!(
                    "no frame ever passed the hash gate, so there is nothing to recover: \
                     got {hash_recoveries}"
                ));
            }
            if header_heads != 2 {
                return Err(format!(
                    "expected 2 ERROR header heads (phase A, then phase C after B closed \
                     the regime), got {header_heads}"
                ));
            }
            let want_header_debugs = debug_lines_expected(HEADER_FRAMES - 1);
            if header_debugs != want_header_debugs {
                return Err(format!(
                    "expected {want_header_debugs} DEBUG header repeats, got {header_debugs}"
                ));
            }
            if header_recoveries != 1 {
                return Err(format!(
                    "the skewed frame's header PARSED, which closes the header regime \
                     exactly once: got {header_recoveries}"
                ));
            }
            let rec =
                find_at(lines, "INFO", HEADER_RECOVERY).ok_or("no INFO header recovery line")?;
            if !has_field(rec, "suppressed_count", &(HEADER_FRAMES - 1).to_string()) {
                return Err(format!("header recovery must report 2 suppressed: {rec}"));
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The DECADE RE-ANNOUNCEMENT of the MALFORMED-HEADER latch.
///
/// `PublisherData`'s counters sit behind an opaque `*mut c_void` the
/// STANDARDIZED rmw C ABI hands to rclcpp/rclpy, so no accessor can be added
/// and a ROS user's whole window onto a publisher rejecting every frame is the
/// LOG. An open regime therefore re-announces loudly at each power of ten
/// instead of going silent after one line — and that arm must be pinned per
/// SITE, because every other drive in this file stops at 7 rejects while the
/// first decade boundary is 10, leaving the `error!` reachable by no assertion
/// and free to be a `debug!` (exactly the hole found on the take side).
///
/// Hand oracle over 10 truncated frames in ONE open regime: 1 `ERROR` head +
/// 8 `DEBUG` repeats + 1 `ERROR` re-announcement carrying `total_failures=10`
/// and the UNCHANGED `suppressed=8`.
///
/// The re-announcement must also carry the HEAD's diagnostic field set
/// (`buffer_len` + `header_size`) and the `topic=` key. Both are asserted here
/// as well as in their own tests, deliberately: this line is read by the
/// operator who MISSED the head, so it is the one most likely to be grepped
/// alone and the one where a thinner field set costs the most.
#[test]
#[serial]
#[traced_test]
fn an_open_header_reject_regime_re_announces_at_the_decade_at_error() {
    const BAD_FRAMES: usize = 10;
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("RejectHdrDecade{suffix}"));
        let (_, node, _opts) = setup_node(&format!("hdr_decade_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/hdrdec/{suffix}")).expect("topic");
        let expected_topic = format!("/rmw_reject/hdrdec/{suffix}");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        // 16 bytes where the header alone needs 32. The needle below derives
        // from this constant, so the oracle cannot drift from the fixture.
        const TRUNCATED_LEN: usize = 16;
        let truncated = Serialized::from_bytes(&[0x33u8; TRUNCATED_LEN]);
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(publisher, truncated.as_ptr(), std::ptr::null_mut()),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(header_reject_count(publisher), BAD_FRAMES as u64);
        assert_eq!(hash_reject_count(publisher), 0);

        logs_assert(move |lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[HEADER_LOUD])?;
            let still = count_at_exclusively(lines, "ERROR", &[HEADER_STILL])?;
            never_loud(lines, HEADER_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[HEADER_SUPPRESSED])?;
            let want_debugs = debug_lines_expected(8);
            if (heads, still, debugs) != (1, 1, want_debugs) {
                return Err(format!(
                    "expected (1 ERROR head, 1 ERROR decade re-announcement, {want_debugs} DEBUG \
                     repeats) over {BAD_FRAMES} rejects, got ({heads}, {still}, {debugs})"
                ));
            }
            if count_at(lines, "DEBUG", HEADER_STILL) != 0 {
                return Err(
                    "the re-announcement must be LOUD — it is the operator's only window \
                     at the rmw sites, where the counter is unreachable"
                        .to_string(),
                );
            }
            let line =
                find_at(lines, "ERROR", HEADER_STILL).ok_or("no ERROR re-announcement line")?;
            // `buffer_len` + `header_size` are the head's diagnosis; the line
            // that SUBSTITUTES for a missed head must carry them too. Matched
            // as FIELDS, not substrings — the message text names
            // ``(`buffer_len=0`)`` in prose (see `has_field`).
            let size = cerulion_core::wire::WireHeader::SIZE.to_string();
            let truncated = TRUNCATED_LEN.to_string();
            for (key, value) in [
                ("total_failures", "10"),
                ("suppressed", "8"),
                ("buffer_len", truncated.as_str()),
                ("header_size", size.as_str()),
                ("topic", expected_topic.as_str()),
            ] {
                if !has_field(line, key, value) {
                    return Err(format!(
                        "re-announcement is missing the field {key}={value}: {line}"
                    ));
                }
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The DECADE RE-ANNOUNCEMENT of the SCHEMA-HASH latch — the second of the two
/// `StillFailing` emissions this change ships.
///
/// Pinned separately from its header twin ON PURPOSE: they are independent
/// `tracing` call sites, so a level regression in one is invisible to the
/// other's oracle. The take-side latch at first shipped three such emissions and pinned one;
/// both unpinned emissions passed the whole suite even when broken.
///
/// Hand oracle over 10 skewed frames: 1 `ERROR` head + 8 `DEBUG` repeats + 1
/// `ERROR` re-announcement carrying `total_failures=10` / `suppressed=8`, the
/// head's diagnostic HASH PAIR, and the `topic=` key — for the same reason as
/// its header twin: this is the line the operator who missed the head reads.
#[test]
#[serial]
#[traced_test]
fn an_open_hash_reject_regime_re_announces_at_the_decade_at_error() {
    const BAD_FRAMES: usize = 10;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("RejectHashDecOurs{suffix}"));
        let skewed = point_ts(&format!("RejectHashDecSkewed{suffix}"));
        let (_, node, _opts) = setup_node(&format!("hash_decade_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/hashdec/{suffix}")).expect("topic");
        let expected_topic = format!("/rmw_reject/hashdec/{suffix}");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ours, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let msg = CPoint {
            x: 0.5,
            y: 0.25,
            z: 0.125,
        };
        let wrong_type = serialize(&msg, skewed);
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(
                    publisher,
                    wrong_type.as_ptr(),
                    std::ptr::null_mut()
                ),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(hash_reject_count(publisher), BAD_FRAMES as u64);
        assert_eq!(
            header_reject_count(publisher),
            0,
            "every frame carried a well-formed header"
        );

        logs_assert(move |lines: &[&str]| {
            let heads = count_at_exclusively(lines, "ERROR", &[HASH_LOUD])?;
            let still = count_at_exclusively(lines, "ERROR", &[HASH_STILL])?;
            never_loud(lines, HASH_SUPPRESSED)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[HASH_SUPPRESSED])?;
            let want_debugs = debug_lines_expected(8);
            if (heads, still, debugs) != (1, 1, want_debugs) {
                return Err(format!(
                    "expected (1 ERROR head, 1 ERROR decade re-announcement, {want_debugs} DEBUG \
                     repeats) over {BAD_FRAMES} rejects, got ({heads}, {still}, {debugs})"
                ));
            }
            if count_at(lines, "DEBUG", HASH_STILL) != 0 {
                return Err("the re-announcement must be LOUD".to_string());
            }
            let line =
                find_at(lines, "ERROR", HASH_STILL).ok_or("no ERROR re-announcement line")?;
            for (key, value) in [
                ("total_failures", "10"),
                ("suppressed", "8"),
                ("topic", expected_topic.as_str()),
            ] {
                if !has_field(line, key, value) {
                    return Err(format!(
                        "re-announcement is missing the field {key}={value}: {line}"
                    ));
                }
            }
            // Value PREFIXES (the hashes are run-unique), so substring here.
            for needle in ["expected_hash=0x", "actual_hash=0x"] {
                if !line.contains(needle) {
                    return Err(format!("re-announcement is missing {needle}: {line}"));
                }
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The per-site FIELD-NAME contract: EVERY line logs the topic under
/// `topic=`, never a generic `name=`.
///
/// Operators grep a reject by that key, so it must appear on the lines that
/// show a regime OPEN, the lines that show it PERSIST — both the suppressed
/// repeat and the decade RE-ANNOUNCEMENT — and the lines that show it CLOSE,
/// for BOTH conditions. `publish_reject_latch` has no site enum (there is
/// exactly one serialized-publish site in rmw), so the contract is satisfied by
/// construction — and this arm is what keeps "by construction" true.
///
/// It therefore drives each condition PAST the decade boundary (10 rejects, not
/// 2). Stopping at 2 and enumerating six markers would leave the
/// two `StillFailing` arms — whose `topic=` is written at a DIFFERENT `tracing`
/// call site from the head's — reachable by no key assertion in the whole
/// binary: mutating both to `name=` would pass every test. So all EIGHT line
/// kinds are enumerated here, each matched on its LEVEL as well as its message.
#[test]
#[serial]
#[traced_test]
fn reject_lines_log_the_topic_under_topic() {
    // Past the first decade boundary, so the two `StillFailing` arms emit.
    const BAD_FRAMES: usize = 10;
    unsafe {
        let suffix = unique_suffix();
        let ours = point_ts(&format!("RejectFieldOurs{suffix}"));
        let skewed = point_ts(&format!("RejectFieldSkewed{suffix}"));
        let (_, node, _opts) = setup_node(&format!("field_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/field/{suffix}")).expect("topic");
        // The Cerulion topic IS the ROS name (the mapping is the identity) —
        // what the reporters log and what an operator greps for.
        let expected_topic = format!("/rmw_reject/field/{suffix}");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ours, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        let msg = CPoint {
            x: 9.0,
            y: 9.0,
            z: 9.0,
        };
        let truncated = Serialized::from_bytes(&[0x44u8; 16]);
        let wrong_type = serialize(&msg, skewed);
        let right_type = serialize(&msg, ours);

        // Open a header regime and drive it past the decade (head, suppressed
        // repeats, one re-announcement), then a skewed frame closes it — a
        // recovery line exists because repeats were suppressed — and opens the
        // hash regime, which is driven past its own decade the same way, then a
        // matching frame closes that one.
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(publisher, truncated.as_ptr(), std::ptr::null_mut()),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(
                    publisher,
                    wrong_type.as_ptr(),
                    std::ptr::null_mut()
                ),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(
            rmw_publish_serialized_message(publisher, right_type.as_ptr(), std::ptr::null_mut()),
            RMW_RET_OK
        );
        // The drive really did reach every arm (the oracle below would
        // otherwise be vacuous in the direction that matters).
        assert_eq!(header_reject_count(publisher), BAD_FRAMES as u64);
        assert_eq!(hash_reject_count(publisher), BAD_FRAMES as u64);

        logs_assert(move |lines: &[&str]| {
            // Level-free twin for the two DEBUG kinds below.
            never_loud(lines, HEADER_SUPPRESSED)?;
            never_loud(lines, HASH_SUPPRESSED)?;
            // All EIGHT line kinds this module can emit, at the level each is
            // contracted to use.
            for (level, marker) in [
                ("ERROR", HEADER_LOUD),
                ("ERROR", HEADER_STILL),
                ("DEBUG", HEADER_SUPPRESSED),
                ("INFO", HEADER_RECOVERY),
                ("ERROR", HASH_LOUD),
                ("ERROR", HASH_STILL),
                ("DEBUG", HASH_SUPPRESSED),
                ("INFO", HASH_RECOVERY),
            ] {
                // A DEBUG kind has no line at all where `debug!` is compiled out.
                if level == "DEBUG" && !debug_level_compiled_in() {
                    continue;
                }
                let line = find_at(lines, level, marker)
                    .ok_or_else(|| format!("no {level} line for {marker}"))?;
                if !has_field(line, "topic", &expected_topic) {
                    return Err(format!(
                        "every reject-latch line must log the topic under the FIELD `topic=`, \
                         never a generic `name=` — operators grep by it: {line}"
                    ));
                }
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// ANTI-TAUTOLOGY control: a healthy serialized publisher emits NONE of the
/// reject-latch lines and leaves BOTH counters at exactly zero.
///
/// Scope, stated narrowly because the R0 wording overclaimed: this does NOT
/// rescue the "exactly N" arms above (those publish accepted frames too, so a
/// reporter firing on acceptance would inflate their counts and fail them). It
/// covers the case they cannot see — a publisher that NEVER rejects anything.
/// The regime arms all open a regime first, so every line they count has a
/// reject somewhere upstream of it; nothing there distinguishes "reports a
/// reject" from "reports every frame, and rejects are all we fed it". This arm
/// is the one that says a clean bag replay is SILENT and counts zero, which is
/// also the whole promise the flood fix makes to a healthy robot.
#[test]
#[serial]
#[traced_test]
fn a_healthy_serialized_publisher_is_silent_and_counts_zero() {
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("RejectQuiet{suffix}"));
        let (_, node, _opts) = setup_node(&format!("quiet_node_{suffix}"));

        let topic = CString::new(format!("/rmw_reject/quiet/{suffix}")).expect("topic");
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
            let frame = serialize(&msg, ts);
            assert_eq!(
                rmw_publish_serialized_message(publisher, frame.as_ptr(), std::ptr::null_mut()),
                RMW_RET_OK
            );
            let (taken, got, _) = take_one(subscription);
            assert!(taken);
            assert_eq!(got, msg);
        }

        assert_eq!(header_reject_count(publisher), 0);
        assert_eq!(hash_reject_count(publisher), 0);
        logs_assert(|lines: &[&str]| {
            let any = lines
                .iter()
                .filter(|l| {
                    l.contains(HEADER_LOUD)
                        || l.contains(HEADER_STILL)
                        || l.contains(HEADER_SUPPRESSED)
                        || l.contains(HEADER_RECOVERY)
                        || l.contains(HASH_LOUD)
                        || l.contains(HASH_STILL)
                        || l.contains(HASH_SUPPRESSED)
                        || l.contains(HASH_RECOVERY)
                })
                .count();
            if any == 0 {
                Ok(())
            } else {
                Err(format!(
                    "a healthy serialized publisher must emit no reject-latch lines, got {any}"
                ))
            }
        });

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// An UNFILLED serialized message is a zero-byte frame, and takes the ordinary
/// latched malformed-header arm: LOUD once, counted always, suppressed after.
///
/// An uninitialised `rmw_serialized_message_t` carries a NULL buffer, and
/// `std::slice::from_raw_parts` over a null pointer is UB even at length zero,
/// so the publish site builds the empty slice by hand instead of calling it.
/// That is the whole of the UB fix. The CONDITION is deliberately NOT
/// special-cased, and this arm pins that decision, because the
/// alternative (an early silent `RMW_RET_INVALID_ARGUMENT`) is silently WORSE than
/// having no check at all:
///
/// * With no null check at all this call is UB on an
///   unfilled message. A DEBUG build aborts on the `from_raw_parts`
///   precondition — that half is MEASURED (see the mutation note below). A
///   RELEASE build compiles the check out, and the empty slice is EXPECTED to
///   reach the malformed-header `error!`; that half is what the UB should do,
///   not something anyone measured, since reasoning about a UB path is not
///   evidence. It is still the only behaviour observed in practice, so an
///   early SILENT return would at best replace a loud condition
///   with nothing.
/// * The identical caller mistake arrives in TWO shapes: a NULL buffer, and a
///   non-null buffer of length zero (asserted here as well). Only the first is
///   distinguishable at the guard, so branching on it would give one shape a
///   loud head plus a counter and the other silence — for the same bug.
///
/// Hand oracle, per publisher: 3 unfilled publishes ⇒ 1 `ERROR` head + 2
/// `DEBUG` repeats + counter 3, with the hash counter untouched. EVERY head
/// (both publishers') must carry `buffer_len=0` — the diagnosis that tells an
/// operator "you sent nothing" apart from "you sent a truncated frame" — and
/// `header_size`, the number it is nothing against.
///
/// **Assert it on every head, not the first.** Shape 2 is the
/// ONLY fixture in this file whose `buffer_capacity` differs from its
/// `buffer_length` (4 vs 0), so it is the only place a `bytes.len()` →
/// `input.buffer_capacity` slip at the call site is observable — and with the
/// arm reading just the FIRST head (the NULL publisher's, whose capacity is
/// also 0) that slip leaves all twelve tests green. MEASURED, not argued: under
/// this arm it fails with `buffer_len=4`.
///
/// Deleting the null handling ABORTS this test binary outright
/// in a debug build (SIGABRT on the `from_raw_parts` precondition check) —
/// which is also the proof it fixes real UB rather than tidying a style point.
#[test]
#[serial]
#[traced_test]
fn an_unfilled_serialized_message_is_a_latched_malformed_header_reject() {
    const BAD_FRAMES: usize = 3;
    unsafe {
        let suffix = unique_suffix();
        let ts = point_ts(&format!("RejectNullGuard{suffix}"));
        let (_, node, _opts) = setup_node(&format!("nullguard_node_{suffix}"));

        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();

        let null_topic = CString::new(format!("/rmw_reject/nullguard/{suffix}")).expect("topic");
        let null_pub = rmw_create_publisher(node, ts, null_topic.as_ptr(), &qos, &pub_opts);
        assert!(!null_pub.is_null());

        // Shape 1: NULL buffer (`rcutils_get_zero_initialized_uint8_array()`).
        let uninit = Serialized::null_buffer();
        assert!(
            uninit.bytes().is_empty(),
            "the fixture must really carry no bytes"
        );
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(null_pub, uninit.as_ptr(), std::ptr::null_mut()),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(
            header_reject_count(null_pub),
            BAD_FRAMES as u64,
            "an unfilled message is a zero-byte frame: too short to hold the wire \
             header, counted like any other malformed frame"
        );
        assert_eq!(
            hash_reject_count(null_pub),
            0,
            "it never reached the hash gate"
        );

        // Shape 2: a non-null, zero-LENGTH buffer — the same caller mistake in
        // the form the null check cannot see. It must be indistinguishable.
        let empty_topic =
            CString::new(format!("/rmw_reject/nullguard_empty/{suffix}")).expect("topic");
        let empty_pub = rmw_create_publisher(node, ts, empty_topic.as_ptr(), &qos, &pub_opts);
        assert!(!empty_pub.is_null());
        // A CAPACITY that differs from the LENGTH is the whole point of this
        // shape: it is the only fixture in the file where reading the wrong one
        // at the call site is observable in the log.
        const ZERO_LEN_CAPACITY: usize = 4;
        let mut zero_len = Serialized::from_bytes(&[0x55u8; ZERO_LEN_CAPACITY]);
        zero_len.0.buffer_length = 0;
        assert!(!zero_len.0.buffer.is_null() && zero_len.bytes().is_empty());
        assert_eq!(
            zero_len.0.buffer_capacity, ZERO_LEN_CAPACITY,
            "the capacity/length divergence is what makes this shape probative"
        );
        for _ in 0..BAD_FRAMES {
            assert_eq!(
                rmw_publish_serialized_message(empty_pub, zero_len.as_ptr(), std::ptr::null_mut()),
                RMW_RET_INVALID_ARGUMENT
            );
        }
        assert_eq!(
            header_reject_count(empty_pub),
            BAD_FRAMES as u64,
            "a zero-LENGTH buffer is the same caller mistake as a NULL one and must \
             be reported identically — routing them apart is what this pins"
        );
        assert_eq!(hash_reject_count(empty_pub), 0);

        logs_assert(|lines: &[&str]| {
            // One regime per publisher, so both shapes are counted together.
            never_loud(lines, HEADER_SUPPRESSED)?;
            let heads = lines_at_exclusively(lines, "ERROR", &[HEADER_LOUD])?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[HEADER_SUPPRESSED])?;
            let want_debugs = debug_lines_expected(2 * (BAD_FRAMES - 1));
            if (heads.len(), debugs) != (2, want_debugs) {
                return Err(format!(
                    "expected (2 ERROR heads — one per publisher, {want_debugs} DEBUG repeats), \
                     got ({}, {debugs})",
                    heads.len()
                ));
            }
            // EVERY head, not the first: only shape 2 has capacity != length,
            // so inspecting one head cannot see a length-vs-capacity slip. And
            // by FIELD, not substring: the message text itself names
            // ``(`buffer_len=0`)`` in prose, so a `contains` predicate here is
            // satisfied by every head no matter what the field says.
            let size = cerulion_core::wire::WireHeader::SIZE.to_string();
            for head in &heads {
                if !has_field(head, "buffer_len", "0") {
                    return Err(format!(
                        "EVERY head must carry the FIELD `buffer_len=0` — that is what \
                         tells an operator they sent NOTHING rather than a truncated \
                         frame, and a head reporting the buffer's CAPACITY instead of \
                         its LENGTH says `buffer_len=4` here: {head}"
                    ));
                }
                if !has_field(head, "header_size", &size) {
                    return Err(format!(
                        "the head must carry `header_size={size}` — `buffer_len` alone \
                         does not say what it fell short OF: {head}"
                    ));
                }
            }
            if count_at(lines, "ERROR", HASH_LOUD) != 0 {
                return Err("a zero-byte frame never reaches the hash gate".to_string());
            }
            Ok(())
        });

        assert_eq!(rmw_destroy_publisher(node, null_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, empty_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
