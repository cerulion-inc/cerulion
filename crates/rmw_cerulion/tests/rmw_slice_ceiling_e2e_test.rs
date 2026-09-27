// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! Slice-ceiling bridge-half e2e: the per-type slice ceiling GOVERNS the real
//! negotiated iceoryx2 buffer on the rmw path — not merely that a function
//! returns a number. The oracle is the LOAN VERDICT: a publisher's SHM pool
//! is created at `initial_max_slice_len = the resolved ceiling`
//! (`AllocationStrategy::Static` — no rounding), so an over-ceiling
//! `rmw_publish` fails its loan (`RMW_RET_ERROR`) while an under-ceiling one
//! succeeds and DELIVERS. Under the earlier blanket (128 MiB) every
//! arm's over-ceiling publish would succeed, which is what makes each verdict
//! a pin that actually discriminates rather than a tautology.
//!
//! ONE `#[test]` body, its OWN binary, on purpose (three process-global
//! reasons, each fatal to a multi-test layout):
//!
//! * `CERULION_RMW_SLICE_CEILING` is read ONCE per process (`OnceLock`) at
//!   the first `rmw_create_publisher` — a second test could never see a
//!   different env value, so every env-dependent arm must share one value,
//!   set before the first creation (the `chunk_c_ffi` ordered-fold pattern);
//! * `#[traced_test]` takes the process-global subscriber slot BEFORE
//!   `runtime()`'s `try_init` (the `rmw_schema_mismatch_test` precedent);
//! * iceoryx2 shared memory is a process singleton.
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_slice_ceiling_e2e_test -- --test-threads=1
//! ```

use cerulion_core::testing::{count_at_exclusively, debug_lines_expected, line_level};
use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use tracing_test::traced_test;

use rmw_cerulion::ffi::introspection_cpp::{CppMessageMember, CppMessageMembers};
use rmw_cerulion::ffi::{self, RMW_RET_ERROR, RMW_RET_OK};
use rmw_cerulion::*;

// =====================================================================
// Hand-built typesupport fixtures (the rmw_e2e_test.rs harness shape)
// =====================================================================

const ROS_TYPE_DOUBLE: u8 = 2;
const ROS_TYPE_STRING: u8 = 16;

extern "C" {
    fn malloc(size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
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
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

/// The variable wire shape every arm shares: `{ id: f64, label: string }`.
#[repr(C)]
struct CLabeled {
    id: f64,
    label: CRosString,
}

fn labeled_ts(namespace: &str, name: &str) -> *const ffi::rosidl_message_type_support_t {
    make_message_ts(
        namespace,
        name,
        std::mem::size_of::<CLabeled>(),
        vec![
            member("id", ROS_TYPE_DOUBLE, 0),
            member("label", ROS_TYPE_STRING, 8),
        ],
    )
}

#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct CPoint {
    x: f64,
    y: f64,
    z: f64,
}

/// A `label` of `len` filler bytes, malloc-backed (freed by the caller).
unsafe fn labeled_msg(id: f64, len: usize) -> CLabeled {
    let buf = malloc(len + 1) as *mut u8;
    assert!(!buf.is_null(), "fixture malloc failed");
    std::ptr::write_bytes(buf, b'a', len);
    *buf.add(len) = 0;
    CLabeled {
        id,
        label: CRosString {
            data: buf,
            size: len,
            capacity: len + 1,
        },
    }
}

unsafe fn free_labeled(msg: CLabeled) {
    free(msg.label.data as *mut c_void);
}

/// Publish a `CLabeled` with a `len`-byte label and return the verdict.
unsafe fn publish_labeled(publisher: *const ffi::rmw_publisher_t, id: f64, len: usize) -> i32 {
    let msg = labeled_msg(id, len);
    let ret = rmw_publish(
        publisher,
        &msg as *const _ as *const c_void,
        std::ptr::null_mut(),
    );
    free_labeled(msg);
    ret
}

fn unique_suffix() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64
}

// =====================================================================
// C++ introspection fixtures (the cpp_bridge_test.rs shape): a fake
// `std::vector<f64>` — repr(C) header + Rust accessor fns; the bridge
// only ever touches it through the function-pointer contract.
// =====================================================================

#[repr(C)]
struct FakeVecF64 {
    data: *mut f64,
    len: usize,
    cap: usize,
}

/// The C++ wire shape every cpp arm shares: `{ data: f64[] }` — one
/// unbounded primitive sequence, so the payload size is `8 × elems`.
#[repr(C)]
struct CppSeqMsg {
    data: FakeVecF64,
}

unsafe extern "C" fn vecf64_size(field: *const c_void) -> usize {
    (*(field as *const FakeVecF64)).len
}
unsafe extern "C" fn vecf64_get_const(field: *const c_void, idx: usize) -> *const c_void {
    let v = &*(field as *const FakeVecF64);
    v.data.add(idx) as *const c_void
}
unsafe extern "C" fn vecf64_get(field: *mut c_void, idx: usize) -> *mut c_void {
    let v = &mut *(field as *mut FakeVecF64);
    v.data.add(idx) as *mut c_void
}
unsafe extern "C" fn vecf64_resize(field: *mut c_void, size: usize) {
    let v = &mut *(field as *mut FakeVecF64);
    // Test fixture: leak old storage (process-lifetime fixtures).
    let mut storage = vec![0f64; size.max(1)];
    v.data = storage.as_mut_ptr();
    v.len = size;
    v.cap = size;
    std::mem::forget(storage);
}

/// A C++ introspection typesupport for [`CppSeqMsg`] under `namespace`
/// VERBATIM — the point is exactly that the bridge accepts any
/// namespace spelling, so the caller picks `pkg::msg` or `pkg__msg`.
fn cpp_seq_ts(namespace: &str, name: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(Box::new([CppMessageMember {
        name_: cstr("data"),
        type_id_: ROS_TYPE_DOUBLE,
        string_upper_bound_: 0,
        members_: std::ptr::null(),
        is_key_: false,
        is_array_: true,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: 0,
        default_value_: std::ptr::null(),
        size_function: Some(vecf64_size),
        get_const_function: Some(vecf64_get_const),
        get_function: Some(vecf64_get),
        fetch_function: None,
        assign_function: None,
        resize_function: Some(vecf64_resize),
        #[cfg(cerulion_has_is_rosidl_buffer)]
        is_rosidl_buffer_: false,
    }]));
    let mm = Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr(namespace),
        message_name_: cstr(name),
        member_count_: 1,
        size_of_: std::mem::size_of::<CppSeqMsg>(),
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: None,
        fini_function: None,
    }));
    let ts = ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    };
    Box::leak(Box::new(ts))
}

/// Publish a [`CppSeqMsg`] carrying `elems` f64s and return the verdict.
unsafe fn publish_cpp_seq(publisher: *const ffi::rmw_publisher_t, elems: usize) -> i32 {
    let mut storage = vec![0f64; elems.max(1)];
    let msg = CppSeqMsg {
        data: FakeVecF64 {
            data: storage.as_mut_ptr(),
            len: elems,
            cap: elems,
        },
    };
    rmw_publish(
        publisher,
        &msg as *const _ as *const c_void,
        std::ptr::null_mut(),
    )
}

/// Panic-safe env manipulation (the repo's `EnvVarGuard` pattern):
/// removes the var on drop, panicking test or not. NOTE: the parsed set
/// outlives the guard by design — the production `OnceLock` caches the first
/// read for the process lifetime, which is exactly what this binary pins.
struct EnvVarGuard {
    key: &'static str,
}

impl EnvVarGuard {
    fn set(key: &'static str, value: &str) -> Self {
        std::env::set_var(key, value);
        Self { key }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.key);
    }
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

/// Every arm of the bridge contract, ordered in ONE body (see the
/// module docs for why). Ceilings under test: the env narrows the
/// SMALL-tier `tf2_msgs/TFMessage` to 1 KiB (override BEATS table — code
/// that checks the table before the override serves the 256 KiB tier here
/// and the 100 KiB probe flips verdict), `nav_msgs/Path` rides its untouched MEDIUM tier
/// (4 MiB — code that never consults the lookup serves the 128 MiB blanket
/// and the 8 MiB probe flips verdict), a user type gets an env ceiling the
/// table never heard of, an absent-everywhere type keeps the blanket, and a
/// FIXED type ignores an override that would otherwise refuse its every
/// publish.
#[test]
#[serial]
#[traced_test]
fn the_resolved_ceiling_governs_real_negotiated_buffers_end_to_end() {
    let suffix = unique_suffix();
    // Set BEFORE the first rmw_create_publisher: the production OnceLock
    // parses this exactly once for the process.
    //   - tf2_msgs/TFMessage:1024   narrows a table-LISTED type (SMALL tier)
    //   - nonsense                  malformed sibling (must warn ONCE, and
    //                               must not disturb any other entry)
    //   - rmw_slc/Narrow{S}:1024    a user type the table never lists
    //   - rmw_slc/Fixed{S}:32       names a FIXED type (must be ignored)
    //   - rmw_slc/Fixed2{S}:32      a SECOND fixed type (its own loud head)
    let env_value = format!(
        "tf2_msgs/TFMessage:1024,nonsense,rmw_slc/Narrow{suffix}:1024,rmw_slc/Fixed{suffix}:32,rmw_slc/Fixed2{suffix}:32"
    );
    let _guard = EnvVarGuard::set("CERULION_RMW_SLICE_CEILING", &env_value);

    unsafe {
        let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
        let node = rmw_create_node(context, cstr(&format!("slc_node_{suffix}")), cstr("/"));
        assert!(!node.is_null(), "node creation failed");

        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        // ── ARM 1: env override BEATS the table on a LISTED type ─────────
        // `tf2_msgs/TFMessage` is SMALL-tier (256 KiB); the env narrows it to
        // 1 KiB. A 100 KiB label sits UNDER the tier and OVER the override,
        // so its verdict separates the two precedences exactly: correct code
        // refuses it, code with inverted precedence (table first) accepts it.
        let tf_ts = labeled_ts("tf2_msgs__msg", "TFMessage");
        let tf_topic = CString::new(format!("/slc/tf/{suffix}")).expect("topic");
        let tf_pub = rmw_create_publisher(node, tf_ts, tf_topic.as_ptr(), &qos, &pub_opts);
        assert!(!tf_pub.is_null(), "TFMessage publisher creation failed");
        assert_eq!(
            publish_labeled(tf_pub, 1.0, 100 * 1024),
            RMW_RET_ERROR,
            "100 KiB must exceed the 1 KiB env override (under the SMALL tier \
             alone it would fit — an accepted publish means the table beat the env)"
        );
        assert_eq!(
            publish_labeled(tf_pub, 2.0, 64),
            RMW_RET_OK,
            "an under-override publish must still work"
        );

        // ── ARM 2: the table tier governs a type the env does not name ───
        // `nav_msgs/Path` is MEDIUM-tier (4 MiB). An 8 MiB label must be
        // refused (under the earlier blanket it fits in 128 MiB — an
        // accepted publish means the bridge never consulted the lookup), and
        // a 1 MiB label must pass (also proving neither the malformed
        // `nonsense` entry nor the other types' overrides leaked onto it).
        let path_ts = labeled_ts("nav_msgs__msg", "Path");
        let path_topic = CString::new(format!("/slc/path/{suffix}")).expect("topic");
        let path_pub = rmw_create_publisher(node, path_ts, path_topic.as_ptr(), &qos, &pub_opts);
        assert!(!path_pub.is_null(), "Path publisher creation failed");
        assert_eq!(
            publish_labeled(path_pub, 3.0, 8 * 1024 * 1024),
            RMW_RET_ERROR,
            "8 MiB must exceed the 4 MiB MEDIUM tier (an accepted publish \
             means the bridge fell back to the 128 MiB blanket)"
        );
        assert_eq!(
            publish_labeled(path_pub, 4.0, 1024 * 1024),
            RMW_RET_OK,
            "1 MiB must fit the MEDIUM tier untouched by the other entries"
        );

        // ── ARM 3: env ceiling on a user type + DELIVERY under it ────────
        // `rmw_slc/Narrow{S}` is unknown to the table; the env caps it at
        // 1 KiB. The over-ceiling publish is refused, and a small message
        // still DELIVERS through a real subscription (the ceiling changed
        // the bound, not the data path).
        let narrow_ts = labeled_ts("rmw_slc__msg", &format!("Narrow{suffix}"));
        let narrow_topic = CString::new(format!("/slc/narrow/{suffix}")).expect("topic");
        let narrow_sub =
            rmw_create_subscription(node, narrow_ts, narrow_topic.as_ptr(), &qos, &sub_opts);
        assert!(!narrow_sub.is_null(), "Narrow subscription creation failed");
        let narrow_pub =
            rmw_create_publisher(node, narrow_ts, narrow_topic.as_ptr(), &qos, &pub_opts);
        assert!(!narrow_pub.is_null(), "Narrow publisher creation failed");
        assert_eq!(
            publish_labeled(narrow_pub, 5.0, 4096),
            RMW_RET_ERROR,
            "4 KiB must exceed the 1 KiB env ceiling on a table-unknown type"
        );
        assert_eq!(
            publish_labeled(narrow_pub, 6.0, 8),
            RMW_RET_OK,
            "a small publish under the env ceiling must succeed"
        );
        let mut out = CLabeled {
            id: 0.0,
            label: CRosString {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        };
        let mut taken = false;
        assert_eq!(
            rmw_take(
                narrow_sub,
                &mut out as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken, "the under-ceiling message must be DELIVERED");
        assert_eq!(out.id, 6.0, "delivered payload must be the accepted frame");
        assert_eq!(out.label.size, 8);
        free(out.label.data as *mut c_void);

        // ── ARM 4: absent from BOTH env and table ⇒ blanket, unchanged ───
        // `rmw_slc/Free{S}`: a 4 KiB publish (refused on Narrow above) is
        // accepted here and DELIVERS — byte-identical fallback to today's
        // 128 MiB blanket, undisturbed by the malformed sibling.
        let free_ts = labeled_ts("rmw_slc__msg", &format!("Free{suffix}"));
        let free_topic = CString::new(format!("/slc/free/{suffix}")).expect("topic");
        let free_sub = rmw_create_subscription(node, free_ts, free_topic.as_ptr(), &qos, &sub_opts);
        assert!(!free_sub.is_null(), "Free subscription creation failed");
        let free_pub = rmw_create_publisher(node, free_ts, free_topic.as_ptr(), &qos, &pub_opts);
        assert!(!free_pub.is_null(), "Free publisher creation failed");
        assert_eq!(
            publish_labeled(free_pub, 7.0, 4096),
            RMW_RET_OK,
            "an env/table-absent type must keep the blanket (the same 4 KiB \
             the Narrow arm refused)"
        );
        let mut out2 = CLabeled {
            id: 0.0,
            label: CRosString {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        };
        let mut taken2 = false;
        assert_eq!(
            rmw_take(
                free_sub,
                &mut out2 as *mut _ as *mut c_void,
                &mut taken2,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken2, "the blanket-path message must be DELIVERED");
        assert_eq!(out2.id, 7.0);
        assert_eq!(out2.label.size, 4096);
        free(out2.label.data as *mut c_void);

        // ── ARM 5: an override naming a FIXED type is ignored, and the
        // ignored-override warn is once-per-TYPE, not per creation ────────
        // `rmw_slc/Fixed{S}` (3 doubles, exact frame 32 + 24 = 56 B) is
        // named in the env at 32 B. If the override were APPLIED, this
        // 56-byte frame could never loan and the publish would fail — its
        // success is the behavioral proof the override is ignored. THREE
        // publishers of the type are created (distinct topics) so the warn
        // pin below can require 1 loud head + 2 suppressed repeats (a
        // bare per-creation warn emits 3 loud
        // lines); a SECOND fixed type then gets its own loud head.
        let fixed_ts = make_message_ts(
            "rmw_slc__msg",
            &format!("Fixed{suffix}"),
            std::mem::size_of::<CPoint>(),
            vec![
                member("x", ROS_TYPE_DOUBLE, 0),
                member("y", ROS_TYPE_DOUBLE, 8),
                member("z", ROS_TYPE_DOUBLE, 16),
            ],
        );
        let fixed_topic = CString::new(format!("/slc/fixed/{suffix}")).expect("topic");
        let fixed_pub = rmw_create_publisher(node, fixed_ts, fixed_topic.as_ptr(), &qos, &pub_opts);
        assert!(!fixed_pub.is_null(), "Fixed publisher creation failed");
        let fixed_topic_b = CString::new(format!("/slc/fixedb/{suffix}")).expect("topic");
        let fixed_pub_b =
            rmw_create_publisher(node, fixed_ts, fixed_topic_b.as_ptr(), &qos, &pub_opts);
        assert!(!fixed_pub_b.is_null(), "second Fixed publisher failed");
        let fixed_topic_c = CString::new(format!("/slc/fixedc/{suffix}")).expect("topic");
        let fixed_pub_c =
            rmw_create_publisher(node, fixed_ts, fixed_topic_c.as_ptr(), &qos, &pub_opts);
        assert!(!fixed_pub_c.is_null(), "third Fixed publisher failed");
        let fixed2_ts = make_message_ts(
            "rmw_slc__msg",
            &format!("Fixed2{suffix}"),
            std::mem::size_of::<CPoint>(),
            vec![
                member("x", ROS_TYPE_DOUBLE, 0),
                member("y", ROS_TYPE_DOUBLE, 8),
                member("z", ROS_TYPE_DOUBLE, 16),
            ],
        );
        let fixed2_topic = CString::new(format!("/slc/fixed2/{suffix}")).expect("topic");
        let fixed2_pub =
            rmw_create_publisher(node, fixed2_ts, fixed2_topic.as_ptr(), &qos, &pub_opts);
        assert!(!fixed2_pub.is_null(), "Fixed2 publisher creation failed");
        let pt = CPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        assert_eq!(
            rmw_publish(
                fixed_pub,
                &pt as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK,
            "a fixed type's exact 56-byte frame must publish — failure here \
             means the 32-byte override was APPLIED instead of ignored"
        );

        // ── ARM 6: determinism — a second same-type creation resolves
        // identically (same process, same parsed set, same verdicts) ──────
        let narrow2_topic = CString::new(format!("/slc/narrow2/{suffix}")).expect("topic");
        let narrow2_pub =
            rmw_create_publisher(node, narrow_ts, narrow2_topic.as_ptr(), &qos, &pub_opts);
        assert!(!narrow2_pub.is_null(), "second Narrow publisher failed");
        assert_eq!(
            publish_labeled(narrow2_pub, 8.0, 4096),
            RMW_RET_ERROR,
            "a second publisher of the same type must resolve the same ceiling"
        );
        assert_eq!(publish_labeled(narrow2_pub, 9.0, 8), RMW_RET_OK);

        // ── Arm 7: the C-style `pkg__msg` C++
        // namespace resolves the ENV OVERRIDE — a `::`-only split
        // passes `tf2_msgs__msg` through verbatim, the qualified name becomes
        // `tf2_msgs__msg/TFMessage`, and BOTH the override and the table
        // miss (128 MiB blanket): this 4 KiB publish would be ACCEPTED past the
        // 1 KiB canonical override. Here it is refused. ─────────────────
        let cpp_dunder_tf = cpp_seq_ts("tf2_msgs__msg", "TFMessage");
        let cpp_tf_topic = CString::new(format!("/slc/cpptf/{suffix}")).expect("topic");
        let cpp_tf_pub =
            rmw_create_publisher(node, cpp_dunder_tf, cpp_tf_topic.as_ptr(), &qos, &pub_opts);
        assert!(
            !cpp_tf_pub.is_null(),
            "cpp dunder TFMessage publisher failed"
        );
        assert_eq!(
            publish_cpp_seq(cpp_tf_pub, 512), // 4 KiB of f64s
            RMW_RET_ERROR,
            "a `pkg__msg`-spelled C++ publisher must honor the canonical env \
             override (accepted = the namespace passed through unnormalized \
             and fell to the blanket)"
        );
        assert_eq!(
            publish_cpp_seq(cpp_tf_pub, 8), // 64 B
            RMW_RET_OK,
            "an under-override cpp publish must still work"
        );

        // ── ARM 8: the `pkg__msg` C++ namespace resolves the
        // TABLE TIER — `nav_msgs__msg`/`Path` must ride MEDIUM (4 MiB), not
        // the blanket: 8 MiB refused, 1 MiB accepted. ─────────────────────
        let cpp_dunder_path = cpp_seq_ts("nav_msgs__msg", "Path");
        let cpp_path_topic = CString::new(format!("/slc/cpppath/{suffix}")).expect("topic");
        let cpp_path_pub = rmw_create_publisher(
            node,
            cpp_dunder_path,
            cpp_path_topic.as_ptr(),
            &qos,
            &pub_opts,
        );
        assert!(!cpp_path_pub.is_null(), "cpp dunder Path publisher failed");
        assert_eq!(
            publish_cpp_seq(cpp_path_pub, 1024 * 1024), // 8 MiB of f64s
            RMW_RET_ERROR,
            "a `pkg__msg`-spelled C++ publisher must honor the table tier \
             (accepted = it fell to the 128 MiB blanket)"
        );
        assert_eq!(
            publish_cpp_seq(cpp_path_pub, 128 * 1024), // 1 MiB
            RMW_RET_OK,
            "1 MiB must fit the MEDIUM tier under the dunder spelling"
        );

        // ── ARM 9 (control): the conventional `pkg::msg` C++
        // namespace behaves exactly as before the normalization — the 1 KiB
        // override still governs. ─────────────────────────────────────────
        let cpp_colon_tf = cpp_seq_ts("tf2_msgs::msg", "TFMessage");
        let cpp_ctf_topic = CString::new(format!("/slc/cppctf/{suffix}")).expect("topic");
        let cpp_ctf_pub =
            rmw_create_publisher(node, cpp_colon_tf, cpp_ctf_topic.as_ptr(), &qos, &pub_opts);
        assert!(
            !cpp_ctf_pub.is_null(),
            "cpp colon TFMessage publisher failed"
        );
        assert_eq!(
            publish_cpp_seq(cpp_ctf_pub, 12800), // 100 KiB — under the tier, over the override
            RMW_RET_ERROR,
            "the `pkg::msg` control must keep honoring the env override"
        );
        assert_eq!(publish_cpp_seq(cpp_ctf_pub, 8), RMW_RET_OK);

        // ── WARN PINS ────────────────────────────────────────────────────
        // (a) parse-once IS the flood regime: ELEVEN publishers were created
        //     above, but the malformed `nonsense` entry warned EXACTLY once
        //     (a per-creation re-parse would have warned per publisher);
        // (b) the log-flood pin: three creations of the
        //     first fixed type produced EXACTLY one loud IGNORED head + two
        //     DEBUG suppressed repeats (a bare per-creation warn emits 3
        //     loud lines), and the SECOND fixed type got its own loud head.
        logs_assert(|lines: &[&str]| {
            // The ONE shared body for the level match (read from the line
            // HEADER). The absence sweep below keeps this non-exclusive form:
            // the exclusive one would REFUSE at a level this marker never uses
            // rather than read 0. Every positive count uses
            // `count_at_exclusively`, which also pairs the level-free total.
            let with = |level: &str, needle: &str, entry: &str| {
                lines
                    .iter()
                    .filter(|l| {
                        line_level(l) == Some(level) && l.contains(needle) && l.contains(entry)
                    })
                    .count()
            };
            let malformed = count_at_exclusively(lines, "WARN", &["entry SKIPPED", "nonsense"])?;
            if malformed != 1 {
                return Err(format!(
                    "the malformed entry must warn EXACTLY once per process \
                     (parse-once), got {malformed}"
                ));
            }
            let fixed_name = format!("rmw_slc/Fixed{suffix}");
            let fixed_heads =
                count_at_exclusively(lines, "WARN", &["override is IGNORED", &fixed_name])?;
            if fixed_heads != 1 {
                return Err(format!(
                    "3 creations of one fixed overridden type must produce \
                     EXACTLY 1 loud head (a bare per-creation warn produces 3), \
                     got {fixed_heads}"
                ));
            }
            // Level-free, and FIRST: a suppressed repeat must never be LOUD.
            // The exclusive DEBUG count below refuses a loud copy too, but with
            // a generic message; this arm names the condition.
            for level in ["WARN", "INFO", "ERROR"] {
                let n = with(level, "suppressed repeat", &fixed_name);
                if n != 0 {
                    return Err(format!(
                        "a suppressed repeat for the repeated fixed type was emitted at \
                         {level} ({n} line(s))"
                    ));
                }
            }
            let fixed_suppressed =
                count_at_exclusively(lines, "DEBUG", &["suppressed repeat", &fixed_name])?;
            let want_fixed_suppressed = debug_lines_expected(2);
            if fixed_suppressed != want_fixed_suppressed {
                return Err(format!(
                    "expected {want_fixed_suppressed} DEBUG suppressed repeats for the repeated fixed \
                     type, got {fixed_suppressed}"
                ));
            }
            let fixed2_heads = count_at_exclusively(
                lines,
                "WARN",
                &["override is IGNORED", &format!("rmw_slc/Fixed2{suffix}")],
            )?;
            if fixed2_heads != 1 {
                return Err(format!(
                    "a second fixed type must get its OWN loud head, got {fixed2_heads}"
                ));
            }
            Ok(())
        });

        // Teardown.
        assert_eq!(rmw_destroy_publisher(node, tf_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, path_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, narrow_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, narrow2_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, free_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, fixed_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, fixed_pub_b), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, fixed_pub_c), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, fixed2_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, cpp_tf_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, cpp_path_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, cpp_ctf_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, narrow_sub), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, free_sub), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
