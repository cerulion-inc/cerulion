// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! C-ABI level: the `--adopt-take` PLAIN take
//! through the exact `extern "C"` surface rcl calls, over REAL iceoryx2
//! shared memory — the heap hook stood in by a FAKE with a real SEGMENT
//! REGISTRY installed through the `test-seams` hook seam
//! (`heaphook::TestHookGuard`). The fake implements the take-side contract
//! (exact-range registration, overlap refusal, containment lookup on free,
//! the ONE process-global release callback) as plain Rust state, and
//! `simulate_free` plays the app's `free()`: it looks the pointer up by
//! CONTAINMENT (interior pointers included), removes the registration, and
//! fires the REAL production release callback with the registration's
//! cookie — byte-for-byte the hook's classify-then-callback sequence. What
//! the fake CANNOT prove — the real `LD_PRELOAD`ed interposed `free` — is
//! the Linux arm (`rmw_adopt_take_linux_test.rs`, needs a Linux/GNU host).
//!
//! Oracles are hand-built, never self-compares: published values are
//! recomputed from the oracle structs; the zero-copy claim is pinned by
//! POINTER IDENTITY (the taken message's sequence `data` IS the registered
//! range start, `capacity == size`) plus a LIVENESS proof (the bytes stay
//! the FIRST frame's after a SECOND publish — a held sample's slot cannot
//! be reclaimed); the copy control's registry stays empty. Counters are
//! the `AdoptStats` atomics (Principle #3, log-independent).
//!
//! ⚠️ iceoryx2 shared memory is a process singleton, the fake hook and the
//! adopt env gate are process-global — run with `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_adopt_take_test -- --test-threads=1
//! ```
//!
//! Two arms self-re-exec THIS binary as a subprocess (`--exact <child>
//! --ignored`) to capture the rmw's stderr `tracing` output — the
//! level-token-matched once-warn and the destroy/shutdown proof lines
//! cannot be captured in-process (the rmw runtime installs its own fmt
//! subscriber at first init).

use serial_test::serial;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use rmw_cerulion::adopt_take::{
    bad_env_value_warned, live_context_count, missing_preload_warn_count, registered_stats_count,
    AdoptStats, AdoptedSample, ADOPT_TAKE_BUDGET_ENV, ADOPT_TAKE_ENV, RMW_ADOPT_TAKE_BORROW_BUDGET,
};
use rmw_cerulion::ffi::introspection_cpp::{
    rmw_cerulion_cppstring_construct, rmw_cerulion_cppstring_destruct, rmw_cerulion_cppstring_view,
    rmw_cerulion_vector_u8_capacity, rmw_cerulion_vector_u8_construct, rmw_cerulion_vector_u8_data,
    rmw_cerulion_vector_u8_size, CppMessageMember, CppMessageMembers, VecTriplet,
};
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::heaphook::{
    HookApi, HookReleaseCallback, TestHookGuard, RC_ERR_BAD_ARG, RC_ERR_NOT_ARMED, RC_ERR_OVERLAP,
    RC_ERR_UNKNOWN, RC_OK,
};
use rmw_cerulion::runtime::SubscriptionData;
use rmw_cerulion::*;

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_UINT32: u8 = 12;
const ROS_TYPE_STRING: u8 = 16;

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
    /// `pub(crate)` in the crate on purpose; the fixture binds the compiled
    /// shim symbol itself (the `rmw_shadow_take_test` precedent).
    fn rmw_cerulion_vector_u8_assign(v: *mut c_void, data: *const u8, len: usize);
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

static UNIQUE: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> u64 {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64;
    nanos ^ UNIQUE.fetch_add(1, Ordering::Relaxed)
}

/// Panic-safe env override: restores (or removes) the prior value on drop.
struct EnvVarGuard {
    key: &'static str,
    prior: Option<std::ffi::OsString>,
}

impl EnvVarGuard {
    fn set(key: &'static str, value: &str) -> Self {
        let prior = std::env::var_os(key);
        std::env::set_var(key, value);
        Self { key, prior }
    }

    fn unset(key: &'static str) -> Self {
        let prior = std::env::var_os(key);
        std::env::remove_var(key);
        Self { key, prior }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        match &self.prior {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

// =====================================================================
// The FAKE heap hook: the take-side segment contract as plain Rust state
// =====================================================================

#[derive(Clone, Copy, Debug)]
struct FakeSeg {
    start: usize,
    len: usize,
    cookie: usize,
}

#[derive(Default)]
struct FakeState {
    segments: Vec<FakeSeg>,
    release_cb: HookReleaseCallback,
    /// 1-based register-call index at which to refuse with RC_ERR_OVERLAP
    /// (the "pre-register the target range" seam, expressed as a
    /// fail-at-N arm — the target address is unknowable before the take).
    fail_register_at: Option<usize>,
    /// Refuse `set_release_callback`. With the hook resolved and the type
    /// forgeable this is the ONE remaining way the grant fails to
    /// construct, and it is the branch's only reachable driver.
    fail_set_cb: bool,
    register_calls: usize,
    unregister_calls: usize,
    set_cb_calls: usize,
}

fn fake() -> &'static Mutex<FakeState> {
    static FAKE: OnceLock<Mutex<FakeState>> = OnceLock::new();
    FAKE.get_or_init(|| Mutex::new(FakeState::default()))
}

fn reset_fake() {
    let mut s = fake().lock().expect("fake");
    *s = FakeState::default();
}

fn fake_segments() -> Vec<FakeSeg> {
    fake().lock().expect("fake").segments.clone()
}

fn arm_register_failure_at(call: usize) {
    fake().lock().expect("fake").fail_register_at = Some(call);
}

fn arm_set_callback_failure() {
    fake().lock().expect("fake").fail_set_cb = true;
}

fn set_cb_calls() -> usize {
    fake().lock().expect("fake").set_cb_calls
}

/// Play the app's `free(ptr)`: containment lookup (interior pointers
/// match, exactly like the real hook's `classify`), registry removal, then
/// the stored release callback fired OUTSIDE the registry lock with the
/// freed address + the registration's cookie — the hook's own sequence.
fn simulate_free(ptr: usize) {
    let (cb, seg) = {
        let mut s = fake().lock().expect("fake");
        let pos = s
            .segments
            .iter()
            .position(|seg| ptr >= seg.start && ptr < seg.start + seg.len)
            .unwrap_or_else(|| panic!("simulate_free({ptr:#x}): no registered range contains it"));
        let seg = s.segments.swap_remove(pos);
        (s.release_cb, seg)
    };
    let cb = cb.expect("a release callback must be installed before any free");
    // SAFETY: the production callback's contract — one cookie per
    // registration, fired at most once (the removal above guarantees it).
    unsafe { cb(ptr as *mut c_void, seg.cookie) };
}

// Window entries: the take path never drives them — inert NOT_ARMED / OK.
unsafe extern "C" fn f_disarm() -> i32 {
    RC_ERR_NOT_ARMED
}
unsafe extern "C" fn f_escape() -> i32 {
    RC_ERR_NOT_ARMED
}
unsafe extern "C" fn f_range(_p: *const c_void, _l: usize) -> i32 {
    RC_ERR_NOT_ARMED
}

unsafe extern "C" fn f_register_segment(start: *mut c_void, len: usize, cookie: usize) -> i32 {
    if start.is_null() || len == 0 {
        return RC_ERR_BAD_ARG;
    }
    let mut s = fake().lock().expect("fake");
    s.register_calls += 1;
    if s.fail_register_at == Some(s.register_calls) {
        return RC_ERR_OVERLAP;
    }
    let (a, b) = (start as usize, start as usize + len);
    if s.segments
        .iter()
        .any(|seg| a < seg.start + seg.len && seg.start < b)
    {
        return RC_ERR_OVERLAP;
    }
    s.segments.push(FakeSeg {
        start: a,
        len,
        cookie,
    });
    RC_OK
}

unsafe extern "C" fn f_unregister_segment(start: *mut c_void) -> i32 {
    let mut s = fake().lock().expect("fake");
    s.unregister_calls += 1;
    match s
        .segments
        .iter()
        .position(|seg| seg.start == start as usize)
    {
        Some(pos) => {
            s.segments.swap_remove(pos);
            RC_OK
        }
        None => RC_ERR_UNKNOWN,
    }
}

unsafe extern "C" fn f_set_release_callback(cb: HookReleaseCallback) -> i32 {
    let mut s = fake().lock().expect("fake");
    s.set_cb_calls += 1;
    if s.fail_set_cb {
        // The callback is NOT stored: a hook that refuses the handshake
        // must not be left holding one.
        return RC_ERR_UNKNOWN;
    }
    s.release_cb = cb;
    RC_OK
}

/// A callback to install when a test only needs one to EXIST — it stands in
/// for the real `adopt_release_callback` in the "was a callback left behind?"
/// question, and is never invoked by those arms.
unsafe extern "C" fn noop_release_callback(_ptr: *mut c_void, _cookie: usize) {}

/// The fake hook: the window entries answer NOT_ARMED (this suite never
/// arms one) and the three registry entries are the real fake.
/// Everything else spreads from `HookApi::inert()`'s inert `RC_OK`
/// stubs, so widening the struct is one edit there, not six here.
fn fake_hook_api() -> HookApi {
    HookApi {
        disarm_window: f_disarm,
        window_escape: f_escape,
        window_range_test: f_range,
        register_segment: f_register_segment,
        unregister_segment: f_unregister_segment,
        set_release_callback: f_set_release_callback,
        ..HookApi::inert()
    }
}

// =====================================================================
// C fixture: LaserScan-shaped — TWO forgeable f32 sequences + a string
// =====================================================================

#[repr(C)]
struct CRosString {
    data: *mut u8,
    size: usize,
    capacity: usize,
}

#[repr(C)]
struct CF32Seq {
    data: *mut f32,
    size: usize,
    capacity: usize,
}

#[repr(C)]
struct CScan {
    angle_min: f32,
    angle_max: f32,
    ranges: CF32Seq,
    intensities: CF32Seq,
    frame_id: CRosString,
}

unsafe extern "C" fn scan_init(
    msg: *mut c_void,
    _init: ffi::rosidl_runtime_c__message_initialization,
) {
    let m = &mut *(msg as *mut CScan);
    m.frame_id.data = calloc(1, 1) as *mut u8;
    m.frame_id.size = 0;
    m.frame_id.capacity = 1;
}

unsafe extern "C" fn scan_fini(msg: *mut c_void) {
    let m = &mut *(msg as *mut CScan);
    for p in [
        m.frame_id.data as *mut c_void,
        m.ranges.data as *mut c_void,
        m.intensities.data as *mut c_void,
    ] {
        if !p.is_null() {
            free(p);
        }
    }
    m.frame_id.data = std::ptr::null_mut();
    m.ranges.data = std::ptr::null_mut();
    m.intensities.data = std::ptr::null_mut();
}

fn c_member(
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

fn c_sequence(
    name: &str,
    type_id: u8,
    offset: u32,
) -> ffi::rosidl_typesupport_introspection_c__MessageMember {
    let mut m = c_member(name, type_id, offset);
    m.is_array_ = true;
    m
}

fn scan_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(
        vec![
            c_member("angle_min", ROS_TYPE_FLOAT, 0),
            c_member("angle_max", ROS_TYPE_FLOAT, 4),
            c_sequence(
                "ranges",
                ROS_TYPE_FLOAT,
                std::mem::offset_of!(CScan, ranges) as u32,
            ),
            c_sequence(
                "intensities",
                ROS_TYPE_FLOAT,
                std::mem::offset_of!(CScan, intensities) as u32,
            ),
            c_member(
                "frame_id",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CScan, frame_id) as u32,
            ),
        ]
        .into_boxed_slice(),
    );
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_adopt__msg"),
            message_name_: cstr(unique),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CScan>(),
            members_: members.as_ptr(),
            init_function: Some(scan_init),
            fini_function: Some(scan_fini),
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

/// A FIXED-only typesupport (nothing forgeable) — the gate-matrix arm.
fn fixed_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(vec![c_member("x", ROS_TYPE_FLOAT, 0)].into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_adopt__msg"),
            message_name_: cstr(unique),
            member_count_: 1,
            size_of_: 4,
            members_: members.as_ptr(),
            init_function: None,
            fini_function: None,
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

struct ScanOracle {
    angle_min: f32,
    angle_max: f32,
    ranges: Vec<f32>,
    intensities: Vec<f32>,
    frame_id: &'static str,
}

fn scan_oracle(n_ranges: usize, n_intensities: usize, seed: f32) -> ScanOracle {
    ScanOracle {
        angle_min: -1.5707964 + seed,
        angle_max: 1.5707964 + seed,
        ranges: (0..n_ranges)
            .map(|i| seed + 0.5 + i as f32 * 0.01)
            .collect(),
        intensities: (0..n_intensities)
            .map(|i| seed + (i % 7) as f32 * 10.0)
            .collect(),
        frame_id: "laser",
    }
}

/// The `CScan` value for an oracle, plus the three heap buffers its members
/// borrow (the caller frees them after the publish call returns).
unsafe fn scan_value(o: &ScanOracle) -> (CScan, [*mut c_void; 3]) {
    let rdata = calloc(o.ranges.len().max(1), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(o.ranges.as_ptr(), rdata, o.ranges.len());
    let idata = calloc(o.intensities.len().max(1), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(o.intensities.as_ptr(), idata, o.intensities.len());
    let sdata = calloc(o.frame_id.len() + 1, 1) as *mut u8;
    std::ptr::copy_nonoverlapping(o.frame_id.as_ptr(), sdata, o.frame_id.len());
    let msg = CScan {
        angle_min: o.angle_min,
        angle_max: o.angle_max,
        ranges: CF32Seq {
            data: if o.ranges.is_empty() {
                std::ptr::null_mut()
            } else {
                rdata
            },
            size: o.ranges.len(),
            capacity: o.ranges.len(),
        },
        intensities: CF32Seq {
            data: if o.intensities.is_empty() {
                std::ptr::null_mut()
            } else {
                idata
            },
            size: o.intensities.len(),
            capacity: o.intensities.len(),
        },
        frame_id: CRosString {
            data: sdata,
            size: o.frame_id.len(),
            capacity: o.frame_id.len() + 1,
        },
    };
    (
        msg,
        [
            rdata as *mut c_void,
            idata as *mut c_void,
            sdata as *mut c_void,
        ],
    )
}

unsafe fn publish_scan(publisher: *const ffi::rmw_publisher_t, o: &ScanOracle) {
    let (msg, owned) = scan_value(o);
    assert_eq!(
        rmw_publish(
            publisher,
            &msg as *const _ as *const c_void,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    for p in owned {
        free(p);
    }
}

/// Publish one scan through `rmw_publish_serialized_message` with
/// the `ranges` offset-table entry REWRITTEN to aim inside the fixed
/// section — the placement `forge_placement` refuses (`off < data_floor`),
/// so that member is served by COPY and counted in
/// `ForgeOutcome::below_floor`, which is what OPENS the forge-fallback
/// regime. `intensities` is left well placed, so the take still ADOPTS (a
/// non-zero forged mask) instead of degrading to the whole-message copy
/// arm. Crib: `rmw_loan_refusal_latch_test.rs::publish_serialized_blob`.
///
/// The table is payload-relative at `layout.fixed_size` (`type_bridge.rs`'s
/// `table_base`), which for this typesupport is the two `f32`s; the entry
/// this patches is asserted to BE the ranges entry first, so a layout change
/// fails the test's own premise instead of silently patching other bytes.
unsafe fn publish_scan_below_floor(
    publisher: *const ffi::rmw_publisher_t,
    ts: *const ffi::rosidl_message_type_support_t,
    o: &ScanOracle,
) {
    const FIXED_SIZE: usize = 8; // angle_min: f32, angle_max: f32
    let (msg, owned) = scan_value(o);
    let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
    ser.allocator = scan_serialize_allocator();
    assert_eq!(
        rmw_serialize(&msg as *const _ as *const c_void, ts, &mut ser),
        RMW_RET_OK
    );
    {
        let frame = std::slice::from_raw_parts_mut(ser.buffer, ser.buffer_length);
        let entry = cerulion_core::wire::WireHeader::SIZE + FIXED_SIZE;
        let len = u32::from_le_bytes(frame[entry + 4..entry + 8].try_into().expect("len"));
        assert_eq!(
            len as usize,
            o.ranges.len() * 4,
            "premise: the entry at the table base must be `ranges` — a layout change must \
             fail HERE, not silently patch some other field's bytes"
        );
        // offset 0 = `angle_min`'s own bytes: inside the fixed section, i.e.
        // below the data floor. Length 4 keeps it one whole f32 element.
        frame[entry..entry + 4].copy_from_slice(&0u32.to_le_bytes());
        frame[entry + 4..entry + 8].copy_from_slice(&4u32.to_le_bytes());
    }
    assert_eq!(
        rmw_publish_serialized_message(publisher, &ser, std::ptr::null_mut()),
        RMW_RET_OK
    );
    for p in owned {
        free(p);
    }
}

unsafe extern "C" fn ser_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    calloc(size.max(1), 1)
}
unsafe extern "C" fn ser_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}
unsafe extern "C" fn ser_reallocate(
    ptr: *mut c_void,
    size: usize,
    _state: *mut c_void,
) -> *mut c_void {
    extern "C" {
        fn realloc(p: *mut c_void, n: usize) -> *mut c_void;
    }
    realloc(ptr, size.max(1))
}
unsafe extern "C" fn ser_zero_allocate(n: usize, size: usize, _state: *mut c_void) -> *mut c_void {
    calloc(n.max(1), size.max(1))
}

fn scan_serialize_allocator() -> ffi::rcutils_allocator_t {
    ffi::rcutils_allocator_t {
        allocate: Some(ser_allocate),
        deallocate: Some(ser_deallocate),
        reallocate: Some(ser_reallocate),
        zero_allocate: Some(ser_zero_allocate),
        state: std::ptr::null_mut(),
    }
}

/// Cleanup for a message taken from a BELOW-FLOOR frame: `ranges` is a real
/// heap COPY (the placement the forge refuses) while `intensities` is
/// FORGED, so the two halves belong to different owners — the copy to the
/// allocator (`scan_fini`), the forged range to the hook.
unsafe fn free_below_floor_scan(mut msg: CScan) {
    if !msg.intensities.data.is_null() {
        simulate_free(msg.intensities.data as usize);
        msg.intensities.data = std::ptr::null_mut();
    }
    scan_fini(&mut msg as *mut CScan as *mut c_void);
}

unsafe fn c_string_of(s: &CRosString) -> String {
    String::from_utf8_lossy(std::slice::from_raw_parts(s.data, s.size)).into_owned()
}

// =====================================================================
// Shared scaffolding
// =====================================================================

unsafe fn setup_node_with_context(name: &str) -> (*mut ffi::rmw_context_t, *mut ffi::rmw_node_t) {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null(), "node creation failed");
    (context, node)
}

unsafe fn setup_node(name: &str) -> *mut ffi::rmw_node_t {
    setup_node_with_context(name).1
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

/// Node + subscription + publisher on one unique topic (subscription FIRST
/// so ITS create-leg borrow floor mints the service).
unsafe fn setup_pair(
    ts: *const ffi::rosidl_message_type_support_t,
    tag: &str,
    suffix: u64,
) -> (
    *mut ffi::rmw_node_t,
    *mut ffi::rmw_publisher_t,
    *mut ffi::rmw_subscription_t,
) {
    let node = setup_node(&format!("{tag}_node_{suffix}"));
    let topic = CString::new(format!("/rmw_adopt/{tag}/{suffix}")).expect("topic");
    let qos = default_qos();
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    assert!(!subscription.is_null(), "subscription creation failed");
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
    assert!(!publisher.is_null(), "publisher creation failed");
    (node, publisher, subscription)
}

unsafe fn teardown(
    node: *mut ffi::rmw_node_t,
    publisher: *mut ffi::rmw_publisher_t,
    subscription: *mut ffi::rmw_subscription_t,
) {
    assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
    assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
    assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
}

unsafe fn sub_data(subscription: *const ffi::rmw_subscription_t) -> &'static SubscriptionData {
    &*((*subscription).data as *const SubscriptionData)
}

/// The subscription's schema-hash-mismatch total. `take_adopted`
/// carries its OWN copy of that gate — a separate call site from the
/// copying take's — so the adopted path needs its own reading of it.
unsafe fn hash_mismatch_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    cerulion_core::transport::failure_regime_latch::lock_regime_latch(
        &sub_data(subscription).hash_mismatches,
    )
    .total_failures()
}

/// The subscription's decode-failure total. The pre-write entry gate
/// reports through this latch, so it is the ARRIVAL witness: without it,
/// "the caller's message is unchanged" is equally true of a frame that never
/// reached the subscriber at all.
unsafe fn decode_failure_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    sub_data(subscription)
        .decode_failures
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .total_failures()
}

unsafe fn adopt_stats(subscription: *const ffi::rmw_subscription_t) -> Arc<AdoptStats> {
    Arc::clone(
        &sub_data(subscription)
            .adopt
            .as_ref()
            .expect("subscription must be adopt-armed")
            .stats,
    )
}

fn stat(a: &AdoptStats) -> (u64, u64, u64, u64, u64, u64) {
    (
        a.takes.load(Ordering::Relaxed),
        a.adopted_takes.load(Ordering::Relaxed),
        a.releases.load(Ordering::Relaxed),
        a.outstanding.load(Ordering::Relaxed),
        a.fallbacks.load(Ordering::Relaxed),
        a.budget_refusals.load(Ordering::Relaxed),
    )
}

/// A plain take into a freshly-initialized CScan.
unsafe fn take_scan(
    subscription: *const ffi::rmw_subscription_t,
) -> (bool, Box<CScan>, ffi::rmw_message_info_t) {
    let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
    scan_init(&mut *msg as *mut _ as *mut c_void, 0);
    let mut taken = false;
    let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
    let ret = rmw_take_with_info(
        subscription,
        &mut *msg as *mut _ as *mut c_void,
        &mut taken,
        &mut info,
        std::ptr::null_mut(),
    );
    assert_eq!(ret, RMW_RET_OK, "rmw_take_with_info must not error");
    (taken, msg, info)
}

/// Release an adopted CScan the way the app's fini would under the real
/// hook: each forged member's `data` is "freed" through the fake (firing
/// the REAL release callback), then nulled so the fixture's real `fini`
/// frees only the heap-owned string.
/// An all-empty forged frame must leave a
/// sequence header the ordinary rosidl `fini` can destroy — a NULL data
/// pointer with zero size and capacity. Both bridges write the EMPTY
/// header for an empty entry rather than aiming at the zero-length
/// shared-memory slice, which is what makes handing the REAL headers to
/// `scan_fini` safe instead of erasing them first.
unsafe fn assert_destructor_safe_empty(msg: &CScan) {
    for (name, seq) in [("ranges", &msg.ranges), ("intensities", &msg.intensities)] {
        assert!(
            seq.data.is_null() && seq.size == 0 && seq.capacity == 0,
            "an empty forged `{name}` must be the destructor-safe EMPTY header, not a \
             shared-memory pointer fini would free: data={:p} size={} capacity={}",
            seq.data,
            seq.size,
            seq.capacity
        );
    }
}

unsafe fn free_adopted_scan(mut msg: CScan) {
    // Takes the struct BY VALUE (callers deref-move out of their Box): the
    // forged pointers ride the FIELDS, so the struct's own address is
    // irrelevant here (clippy: boxed_local).
    for seq in [&mut msg.ranges as *mut CF32Seq, &mut msg.intensities] {
        let seq = &mut *seq;
        if !seq.data.is_null() {
            simulate_free(seq.data as usize);
            seq.data = std::ptr::null_mut();
        }
    }
    scan_fini(&mut msg as *mut CScan as *mut c_void);
}

/// The held sample's payload address range,
/// reached by deref'ing a registration's cookie back to its
/// `Arc<AdoptedSample>` target. Every forged range must lie INSIDE it —
/// the oracle a copy-to-heap-and-register-the-copy implementation cannot
/// pass (its registered copy is heap memory, not the sample).
unsafe fn held_sample_range(seg: &FakeSeg) -> std::ops::Range<usize> {
    // SAFETY: every cookie the adoption branch mints is
    // `Arc::into_raw(Arc<AdoptedSample>)`, alive while its registration is
    // live in the fake registry (the clone is consumed only by a release).
    let adopted = &*(seg.cookie as *const AdoptedSample);
    let payload = adopted.payload();
    let start = payload.as_ptr() as usize;
    start..start + payload.len()
}

fn assert_scan_values(msg: &CScan, o: &ScanOracle) {
    assert_eq!(msg.angle_min.to_bits(), o.angle_min.to_bits());
    assert_eq!(msg.angle_max.to_bits(), o.angle_max.to_bits());
    unsafe {
        assert_eq!(c_string_of(&msg.frame_id), o.frame_id);
    }
    for (name, seq, want) in [
        ("ranges", &msg.ranges, &o.ranges),
        ("intensities", &msg.intensities, &o.intensities),
    ] {
        assert_eq!(seq.size, want.len(), "{name}.size");
        if want.is_empty() {
            // The COMPLETE empty header — `{NULL, 0, 0}` — never just the
            // pointer (a `{NULL, 0, capacity > 0}` header must fail here).
            assert_eq!(
                (seq.data.is_null(), seq.size, seq.capacity),
                (true, 0, 0),
                "{name}: empty is the complete null header {{NULL, 0, 0}}"
            );
            continue;
        }
        let seen = unsafe { std::slice::from_raw_parts(seq.data, seq.size) };
        for (i, (a, b)) in seen.iter().zip(want.iter()).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "{name}[{i}]");
        }
    }
}

// =====================================================================
// Arm 1 — pointer identity + liveness (C bridge) + the copy control
// =====================================================================

/// HEADLINE: under env + Active fake hook, a PLAIN take
/// serves both forgeable sequences IN PLACE — each `data` pointer IS a
/// registered range's start with the exact byte length and
/// `capacity == size` — the copied members (fixed floats, string) are
/// exact, the sample stays held (`outstanding == 1`) and its bytes stay
/// the FIRST frame's after a SECOND publish (a held slot cannot be
/// reclaimed). Frees release: one callback per range, last one drops the
/// sample.
#[test]
#[serial]
fn adopted_take_serves_shm_in_place_with_exact_registered_ranges() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Adopt{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "a1", suffix);
        let stats = adopt_stats(subscription);
        assert_eq!(
            sub_data(subscription).adopt.as_ref().unwrap().budget,
            RMW_ADOPT_TAKE_BORROW_BUDGET,
            "default budget"
        );

        let o1 = scan_oracle(32, 16, 0.0);
        publish_scan(publisher, &o1);
        let (taken, msg1, info) = take_scan(subscription);
        assert!(taken, "a published frame must be takeable");
        assert!(info.source_timestamp > 0, "wire timestamp reaches info");

        // Pointer identity: each forged member IS a registered range.
        let segs = fake_segments();
        assert_eq!(segs.len(), 2, "one exact registration per forged member");
        for (name, seq) in [("ranges", &msg1.ranges), ("intensities", &msg1.intensities)] {
            let reg = segs
                .iter()
                .find(|s| s.start == seq.data as usize)
                .unwrap_or_else(|| panic!("{name}.data must be a registered range start"));
            assert_eq!(reg.len, seq.size * 4, "{name}: exact byte length");
            assert_eq!(seq.capacity, seq.size, "{name}: capacity == size");
        }
        assert_scan_values(&msg1, &o1);
        // The registered ranges lie inside the held
        // sample's payload — read back through the cookie, so a
        // copy-to-heap-and-register-the-copy implementation fails here.
        for seg in &segs {
            let held = held_sample_range(seg);
            assert!(
                held.contains(&seg.start) && held.contains(&(seg.start + seg.len - 1)),
                "registered range {:#x}+{} must lie inside the held sample {held:#x?}",
                seg.start,
                seg.len
            );
        }
        assert_eq!(
            segs[0].cookie, segs[1].cookie,
            "both members are served by ONE held sample (clones of one Arc)"
        );
        // The string is a COPY (heap), never a registered range.
        assert!(
            !segs.iter().any(|s| (msg1.frame_id.data as usize) >= s.start
                && (msg1.frame_id.data as usize) < s.start + s.len),
            "the string copy must not alias a registered range"
        );
        assert_eq!(stat(&stats), (1, 1, 0, 1, 0, 0));

        // Liveness: a second publish + take must NOT disturb msg1's bytes —
        // the held sample pins its slot.
        let o2 = scan_oracle(32, 16, 100.0);
        publish_scan(publisher, &o2);
        let (taken2, msg2, _) = take_scan(subscription);
        assert!(taken2);
        assert_scan_values(&msg1, &o1);
        assert_scan_values(&msg2, &o2);
        assert_eq!(stat(&stats), (2, 2, 0, 2, 0, 0));

        // The app frees: one release per range; each sample releases on its
        // LAST range's free.
        free_adopted_scan(*msg1);
        assert_eq!(stat(&stats), (2, 2, 2, 1, 0, 0), "msg1's sample released");
        free_adopted_scan(*msg2);
        assert_eq!(stat(&stats), (2, 2, 4, 0, 0, 0), "all samples released");
        assert!(
            fake_segments().is_empty(),
            "no registration outlives its free"
        );

        teardown(node, publisher, subscription);
    }
}

/// An EMPTY forgeable entry is forged to the null header and registers
/// NOTHING (its free is already a rosidl no-op); the non-empty sibling
/// still adopts and holds the sample alone.
#[test]
#[serial]
fn empty_forged_entry_registers_nothing_and_the_sibling_holds_the_sample() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("AdoptE{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "a1b", suffix);
        let stats = adopt_stats(subscription);

        let o = scan_oracle(24, 0, 0.5);
        publish_scan(publisher, &o);
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken);
        assert_scan_values(&msg, &o);
        let segs = fake_segments();
        assert_eq!(segs.len(), 1, "only the non-empty member registers");
        assert_eq!(segs[0].start, msg.ranges.data as usize);
        assert_eq!(stat(&stats), (1, 1, 0, 1, 0, 0));

        free_adopted_scan(*msg);
        assert_eq!(stat(&stats), (1, 1, 1, 0, 0, 0));
        teardown(node, publisher, subscription);
    }
}

/// The COPY CONTROL: same frame, same hook installed, NO env — the adopt
/// state never constructs, the registry stays empty, the bytes are served
/// by the plain path (heap copies the fixture's fini frees normally).
#[test]
#[serial]
fn copy_control_without_the_env_is_the_plain_path() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::unset(ADOPT_TAKE_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Ctrl{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "ctrl", suffix);
        assert!(
            sub_data(subscription).adopt.is_none(),
            "no env ⇒ no adopt state"
        );

        let o = scan_oracle(32, 16, 1.0);
        publish_scan(publisher, &o);
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken);
        assert_scan_values(&msg, &o);
        assert!(
            fake_segments().is_empty(),
            "the copy path must register nothing"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

// =====================================================================
// Arm 3 — the gate matrix (the witness is load-bearing)
// =====================================================================

/// The sentence the missing-preload warn alone emits. Filtering the child's
/// stderr on `ADOPT_TAKE_ENV` instead matched a strict SUPERSET — that name
/// is a PREFIX of `CERULION_RMW_ADOPT_TAKE_BUDGET`, carried by the
/// budget-parse warn and the WARN-level budget-exhaustion refusals, and it
/// is also named verbatim by `warn_bad_env_value_once`. THIS child emits
/// none of those today, so narrowing the head filter is prophylactic — but
/// the old predicate was doing a second job, "no OTHER warn in this child
/// names the gate variable", so that half is asserted separately rather
/// than dropped.
const MISSING_PRELOAD_WARN_SUBSTR: &str = "is set but the Cerulion heap hook is not ACTIVE";

/// env + hook + forgeable ⇒ armed; hook + no env ⇒ not armed; env + hook +
/// FIXED type ⇒ not armed (nothing adoptable); the budget env override is
/// honored at create.
#[test]
#[serial]
fn gate_matrix_arms_exactly_env_plus_grant_plus_forgeable() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();

        // hook + no env ⇒ None, zero adopt machinery.
        {
            let _env = EnvVarGuard::unset(ADOPT_TAKE_ENV);
            let ts = scan_ts(&format!("GmA{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, "gma", suffix);
            assert!(sub_data(subscription).adopt.is_none());
            teardown(node, publisher, subscription);
        }
        // env + hook + forgeable ⇒ armed, with the env-overridden budget.
        {
            let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
            let _bud = EnvVarGuard::set(ADOPT_TAKE_BUDGET_ENV, "7");
            let ts = scan_ts(&format!("GmB{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, "gmb", suffix);
            let state = sub_data(subscription)
                .adopt
                .as_ref()
                .expect("env + grant + forgeable must arm");
            assert_eq!(state.budget, 7, "budget env override is CREATE-time");
            teardown(node, publisher, subscription);
        }
        // env + hook + FIXED-only type ⇒ None (nothing adoptable; the plain
        // path already serves it).
        {
            let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
            let ts = fixed_ts(&format!("GmFix{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, "gmf", suffix);
            assert!(sub_data(subscription).adopt.is_none());
            teardown(node, publisher, subscription);
        }
        // A class sweep found this: every arm in the tree sets the gate to
        // "1" or leaves it unset, and the strict parse is driven only
        // through `classify_gate_value` DIRECTLY — so nothing proved
        // `env_armed()` consults it, and a variant reading the variable's
        // mere PRESENCE armed adoption on "0" with the whole suite green.
        // Both non-arming shapes now reach the classifier through the
        // PRODUCTION entry point. `adopt.is_none()` is the load-bearing
        // pin: the hook IS installed and the type IS forgeable, so the ENV
        // branch is the only route to `None` here.
        //
        // The second assertion is what makes the two shapes DIFFERENT
        // rather than a repeat: an unusable value must WARN and a
        // legitimate `0` must stay quiet. Without it, dropping
        // `warn_bad_env_value_once` from the unrecognized arm turns it
        // into a SILENT disarm — the exact silent inference the strict
        // parse exists to prevent — while adding one to the `0` arm puts
        // noise on a legitimate setting; both variants keep
        // `adopt.is_none()` true. The latch is one-way and process-global,
        // so this also RECORDS that the "yes" leg burns it for the rest of
        // this binary rather than leaving a future once-latch pin
        // order-dependently vacuous.
        for value in ["0", "yes"] {
            let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, value);
            let ts = scan_ts(&format!("GmOff{value}{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, &format!("gmoff{value}"), suffix);
            assert!(
                sub_data(subscription).adopt.is_none(),
                "`{value}` must not arm through the production entry point"
            );
            assert_eq!(
                bad_env_value_warned(),
                value == "yes",
                "`{value}` must warn iff it is unusable"
            );
            teardown(node, publisher, subscription);
        }
        // The budget's strict parse through `budget_from_env()`, pinned on
        // BOTH sides of the clamp: `0` is refused and the DEFAULT served,
        // while `1` — the legitimate minimum — is honoured. One side alone
        // leaves `n > 1` alive, which silently turns a requested floor of 1
        // into 16. What a dropped clamp actually costs is not a take
        // failure: `0` reaches `create_borrow_floor`, and
        // `effective_create_borrow_floor` discards any floor at or below
        // the iceoryx2 default of 2, so the service is created at that
        // default instead of the budget — premature refusals under load,
        // behind an ARMED line reporting `budget=0`.
        for (value, expected) in [("0", RMW_ADOPT_TAKE_BORROW_BUDGET), ("1", 1)] {
            let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
            let _bud = EnvVarGuard::set(ADOPT_TAKE_BUDGET_ENV, value);
            let ts = scan_ts(&format!("GmBud{value}{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, &format!("gmbud{value}"), suffix);
            let state = sub_data(subscription)
                .adopt
                .as_ref()
                .expect("still armed — only the BUDGET value varies");
            assert_eq!(
                state.budget, expected,
                "budget `{value}` must resolve to {expected}"
            );
            teardown(node, publisher, subscription);
        }
    }
}

/// `take_adopted` carries its
/// OWN copy of the schema-hash gate — a SEPARATE call site from the
/// copying take's, which `rmw_schema_mismatch_test.rs` covers — and no other
/// arm drives it, so deleting it leaves every other test in the tree green while a
/// version-skewed publisher's frame is FORGED into the caller's message.
/// A skewed frame on an adopt-armed subscription must be refused exactly
/// as the copy path refuses it: frame consumed, `taken` FALSE, the hash-mismatch
/// latch counting, nothing adopted, and — the part unique to this path —
/// nothing registered with the hook, because the gate returns before the
/// forge. The MATCHING pair on a sibling topic in the same body is the
/// anti-tautology half: the two typesupports differ ONLY in their message
/// name, so the layout and the published frame are identical and the hash
/// is the only moving part.
#[test]
#[serial]
fn a_hash_skewed_frame_is_refused_on_the_adopted_path_before_anything_is_forged() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let node = setup_node(&format!("hashskew_node_{suffix}"));
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();

        // The SKEWED pair: same layout, different message NAME.
        let skew_topic = CString::new(format!("/rmw_adopt/hashskew/{suffix}")).expect("topic");
        let skew_sub = rmw_create_subscription(
            node,
            scan_ts(&format!("HashSkewSub{suffix}")),
            skew_topic.as_ptr(),
            &qos,
            &sub_opts,
        );
        assert!(!skew_sub.is_null(), "subscription creation failed");
        let skew_pub = rmw_create_publisher(
            node,
            scan_ts(&format!("HashSkewPub{suffix}")),
            skew_topic.as_ptr(),
            &qos,
            &pub_opts,
        );
        assert!(!skew_pub.is_null(), "publisher creation failed");
        let skew_stats = adopt_stats(skew_sub);

        // The MATCHING pair: one typesupport for both ends.
        let ok_topic = CString::new(format!("/rmw_adopt/hashok/{suffix}")).expect("topic");
        let ok_ts = scan_ts(&format!("HashOk{suffix}"));
        let ok_sub = rmw_create_subscription(node, ok_ts, ok_topic.as_ptr(), &qos, &sub_opts);
        assert!(!ok_sub.is_null(), "subscription creation failed");
        let ok_pub = rmw_create_publisher(node, ok_ts, ok_topic.as_ptr(), &qos, &pub_opts);
        assert!(!ok_pub.is_null(), "publisher creation failed");
        let ok_stats = adopt_stats(ok_sub);

        assert_eq!(hash_mismatch_count(skew_sub), 0, "nothing counted yet");

        let o = scan_oracle(16, 8, 5.0);
        publish_scan(skew_pub, &o);
        let (taken, mut skewed_msg, _) = take_scan(skew_sub);
        assert!(!taken, "a hash-skewed frame must NOT be delivered");
        assert_eq!(
            hash_mismatch_count(skew_sub),
            1,
            "the ADOPTED path's own hash gate counted it"
        );
        assert!(
            fake_segments().is_empty(),
            "the gate returns BEFORE the forge — nothing may be registered with the hook"
        );
        assert_eq!(
            stat(&skew_stats),
            (0, 0, 0, 0, 0, 0),
            "the gate returns before every adopt counter, the served/fallback ones included"
        );

        // Anti-tautology: the identical stimulus with a matching hash is
        // adopted and delivered, so the refusal above is the HASH and not
        // the fixture, the topic, or the arming.
        publish_scan(ok_pub, &o);
        let (taken, mut ok_msg, _) = take_scan(ok_sub);
        assert!(taken, "a matching frame is still delivered");
        assert_scan_values(&ok_msg, &o);
        assert_eq!(hash_mismatch_count(ok_sub), 0, "and counts no mismatch");
        assert_eq!(stat(&ok_stats).1, 1, "and really took the ADOPTED path");

        scan_fini(&mut *skewed_msg as *mut _ as *mut c_void);
        // Release the adopted members before the sub goes away.
        simulate_free(ok_msg.ranges.data as usize);
        simulate_free(ok_msg.intensities.data as usize);
        ok_msg.ranges.data = std::ptr::null_mut();
        ok_msg.intensities.data = std::ptr::null_mut();
        scan_fini(&mut *ok_msg as *mut _ as *mut c_void);
        assert_eq!(rmw_destroy_publisher(node, skew_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, skew_sub), RMW_RET_OK);
        assert_eq!(rmw_destroy_publisher(node, ok_pub), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, ok_sub), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Memory safety: the adopt reuse pre-pass
/// must free a forgeable member only when it OWNS an allocation. This is
/// the C bridge's half of the same guard the C++ twin carries, and it is
/// the leg that runs on every host — the C++ behavioural arm is gated on
/// that host's `std::vector` layout probe enabling forging.
///
/// The hazard is minted by the forge itself: it writes `capacity = size`,
/// so an EMPTY sequence leaves a live SHM address behind a capacity of 0.
/// A zero-length range registers nothing with the hook, so freeing that
/// pointer reaches the real `free` with a shared-memory address. The
/// pre-pass only meets a previous frame's triplet when the caller REUSES
/// a message, which is exactly what a well-behaved ROS node does — so the
/// arm takes twice into the SAME message, with empty sequences both
/// times, which is an empty `/scan` published twice.
///
/// Under the `data != null` guard this frees an SHM address: glibc and
/// macOS libc both abort on a pointer that is not one of theirs, so a
/// variant that skips the guard kills the binary rather than changing a value.
#[test]
#[serial]
fn an_empty_frame_reused_across_takes_is_not_freed_by_the_pre_pass() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ReuseEmpty{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "reuseempty", suffix);

        // ONE message, taken into twice — the reuse the pre-pass exists
        // for. `scan_init` runs once, as a real caller's does.
        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg as *mut _ as *mut c_void, 0);
        let empty = scan_oracle(0, 0, 7.0);
        for round in 0..2 {
            publish_scan(publisher, &empty);
            let mut taken = false;
            let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
            let ret = rmw_take_with_info(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                &mut info,
                std::ptr::null_mut(),
            );
            assert_eq!(ret, RMW_RET_OK, "round {round}");
            assert!(taken, "round {round}: the empty frame is served");
        }
        // The second take's pre-pass met the first take's forged triplet.
        // NOTE the limit of this leg: freeing an SHM address does
        // not reliably abort, so what
        // it pins is reachability and correctness of the served frame, not
        // the invalid free. The arm below pins the free itself, using an
        // address whose release DOES abort.
        // Reaching here at all is the pin; the values say the forge really
        // did leave a non-owning, non-null pointer for it to meet.
        assert!(
            fake_segments().is_empty(),
            "an empty frame registers nothing, so there is nothing for a free to reach"
        );
        assert_scan_values(&msg, &empty);
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

/// Memory safety: the C bridge's release
/// gate, driven directly with an address whose release is DETECTABLE.
///
/// The end-to-end arm above cannot serve as the mutation kill: its
/// non-owning pointer is a shared-memory address, and freeing one of
/// those does not reliably abort, so reverting the `data != null` check
/// survives it. A pointer into the MIDDLE of a live heap block does
/// abort, on glibc and on macOS alike, so that is what this hands the
/// pre-pass: capacity 0 (owns nothing) with a non-null `data`, which is
/// exactly the shape the forge leaves behind for an empty sequence.
#[test]
#[serial]
fn the_c_bridge_pre_pass_frees_only_a_sequence_that_owns_storage() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("OwnGate{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "owngate", suffix);
        let bridge = &sub_data(subscription).bridge;

        // NON-OWNING: a live mid-block address with capacity 0.
        let mut live: Vec<f32> = vec![1.5, 2.5, 3.5, 4.5];
        let mid = live.as_mut_ptr().add(1);
        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        msg.ranges.data = mid;
        msg.ranges.size = 0;
        msg.ranges.capacity = 0;
        bridge.release_forgeable_members(&mut *msg as *mut _ as *mut c_void);
        assert!(
            msg.ranges.data.is_null() && msg.ranges.capacity == 0,
            "the sequence is cleared, released or not"
        );
        assert_eq!(
            live,
            vec![1.5, 2.5, 3.5, 4.5],
            "the block the non-owning sequence pointed into must be untouched"
        );

        // OWNING: a real allocation with a non-zero capacity is released.
        let owned: Box<[f32]> = vec![9.0f32; 4].into_boxed_slice();
        let begin = Box::into_raw(owned) as *mut f32;
        let mut msg2: Box<CScan> = Box::new(std::mem::zeroed());
        msg2.ranges.data = begin;
        msg2.ranges.size = 4;
        msg2.ranges.capacity = 4;
        bridge.release_forgeable_members(&mut *msg2 as *mut _ as *mut c_void);
        assert!(
            msg2.ranges.data.is_null(),
            "an owning sequence is released and cleared"
        );
        teardown(node, publisher, subscription);
    }
}

/// A subscription destroyed while an
/// adopted sample is still held must not have its counters detached.
///
/// Retiring unconditionally is wrong: folding a snapshot into the
/// retired total and dropping the registry entry leaves the `Arc` shared with
/// every registered range's release cookie, so the app's later frees keep
/// updating an object nothing aggregates — shutdown then reports a stale
/// `released`/`outstanding` pair and can take the outstanding-sample
/// branch on samples that have in fact been freed. Such an entry stays
/// registered and is aggregated LIVE, which is also why nothing is double
/// counted: an entry is either folded once and removed, or summed from the
/// registry, never both.
#[test]
#[serial]
fn a_subscription_destroyed_with_samples_held_keeps_its_counters_live() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let before = registered_stats_count();
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("HeldDestroy{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "helddestroy", suffix);
        let stats = adopt_stats(subscription);

        publish_scan(publisher, &scan_oracle(16, 8, 3.0));
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken);
        assert_eq!(stat(&stats).3, 1, "one sample outstanding");
        assert_eq!(registered_stats_count(), before + 1);

        // Destroy while the sample is HELD.
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(
            registered_stats_count(),
            before + 1,
            "an entry whose samples are still held must STAY registered — retiring it \
             detaches the counters the app's later frees update"
        );

        // The app frees; the still-registered counters must move.
        let releases_before = stat(&stats).2;
        simulate_free(msg.ranges.data as usize);
        simulate_free(msg.intensities.data as usize);
        msg.ranges.data = std::ptr::null_mut();
        msg.intensities.data = std::ptr::null_mut();
        assert_eq!(
            stat(&stats).2,
            releases_before + 2,
            "frees after destroy still count"
        );
        assert_eq!(
            stat(&stats).3,
            0,
            "and the sample is released — the value shutdown would read"
        );
        // Staying registered is the right
        // answer only while the samples are held. The drain is the second
        // moment retirement can become possible, and without a retirement
        // path there a process that destroys subscriptions while holding
        // samples grows the registry without bound, which is the very leak
        // the create/destroy arm pins for the ordinary order. The entry
        // must leave here.
        assert_eq!(
            registered_stats_count(),
            before,
            "the last held sample has drained — the destroyed subscription's entry \
             must now LEAVE the registry"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The process-global stats registry holds a
/// strong `Arc` per successful adopt subscription; if it never released one,
/// a process that cycles subscriptions would grow it without bound. A
/// create/destroy loop must leave its length where it started.
#[test]
#[serial]
fn cycling_subscriptions_does_not_grow_the_stats_registry() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let before = registered_stats_count();
        let suffix = unique_suffix();
        for round in 0..4 {
            let ts = scan_ts(&format!("Cycle{round}{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, &format!("cycle{round}"), suffix);
            assert!(
                sub_data(subscription).adopt.is_some(),
                "round {round} must actually arm, or the registry was never touched"
            );
            assert_eq!(
                registered_stats_count(),
                before + 1,
                "round {round}: exactly this subscription is registered while it lives"
            );
            teardown(node, publisher, subscription);
            assert_eq!(
                registered_stats_count(),
                before,
                "round {round}: destroy must retire it again"
            );
        }
    }
}

/// A frame that registers nothing must not
/// close a registration-failure regime. An all-empty forged mask asks the
/// hook for nothing, so it has tested nothing — reporting it clean would
/// re-arm the latch and make the NEXT genuine failure a fresh loud head
/// instead of the suppressed repeat it is. On a topic whose registrations
/// are failing, an empty `/scan` would do that once per frame, which is
/// the flood the latch exists to prevent.
#[test]
#[serial]
fn an_empty_frame_cannot_close_a_registration_failure_regime() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("EmptyClean{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "emptyclean", suffix);
        let adopt_state = || sub_data(subscription).adopt.as_ref().expect("armed");
        // 1. Open the regime: the first registration is refused.
        arm_register_failure_at(1);
        publish_scan(publisher, &scan_oracle(16, 8, 1.0));
        let (taken, mut failed_msg, _) = take_scan(subscription);
        assert!(
            taken,
            "a registration failure still serves the frame by copy"
        );
        scan_fini(&mut *failed_msg as *mut _ as *mut c_void);
        assert!(
            adopt_state().registration_regime_open(),
            "the registration-failure regime must be OPEN"
        );
        assert_eq!(adopt_state().registration_failure_count(), 1);

        // 2. THE PIN: an empty-sequence frame registers nothing, so it
        // cannot report a recovery. The total is unchanged either way —
        // only the regime bit can tell a false recovery from none.
        publish_scan(publisher, &scan_oracle(0, 0, 2.0));
        let (taken, mut empty_msg, _) = take_scan(subscription);
        assert!(taken, "the empty frame is still served");
        // The pointers are not erased
        // before `fini`. Overwriting them removed the only check that an
        // empty forged entry is destructor-safe — a bridge that left a
        // shared-memory pointer in one, or failed to un-forge it, was
        // hidden and the arm still passed. Both bridges write the EMPTY
        // header for an empty entry, so the state is ASSERTED and `fini`
        // is handed the real headers.
        assert_destructor_safe_empty(&empty_msg);
        scan_fini(&mut *empty_msg as *mut _ as *mut c_void);
        assert!(
            adopt_state().registration_regime_open(),
            "a frame that registered NOTHING must leave the regime OPEN"
        );

        // 3. Anti-tautology: a frame that DOES register closes it, so the
        // bit above is not simply stuck.
        publish_scan(publisher, &scan_oracle(16, 8, 3.0));
        let (taken, mut good_msg, _) = take_scan(subscription);
        assert!(taken);
        assert_eq!(fake_segments().len(), 2, "this one really registered");
        assert!(
            !adopt_state().registration_regime_open(),
            "a REAL registration closes the regime"
        );
        simulate_free(good_msg.ranges.data as usize);
        simulate_free(good_msg.intensities.data as usize);
        good_msg.ranges.data = std::ptr::null_mut();
        good_msg.intensities.data = std::ptr::null_mut();
        scan_fini(&mut *good_msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

/// CHILD: the grant's OWN refusal branch — a resolved hook that
/// answers `set_release_callback` with an error. It runs in a CHILD because
/// a refused grant takes the same no-grant path as a missing hook and
/// therefore BURNS the process-global missing-preload once-latch, which
/// `env_without_a_hook_stays_on_the_copy_path_with_one_warn` asserts is
/// pristine (`before == 0`) — and that guard is right: "fires at the first
/// armed create" is observable only by the first test in a process to
/// trigger it, so a delta oracle there would go vacuous rather than fail.
/// Sorting alphabetically ahead of it, this arm broke it. A child gets a
/// fresh process and a fresh latch.
#[test]
#[ignore]
fn child_hook_refusing_the_release_callback_stays_on_the_copy_path() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    arm_set_callback_failure();
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("GrantRefuse{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "grantrefuse", suffix);
        assert!(
            sub_data(subscription).adopt.is_none(),
            "a refused handshake must NOT arm adoption"
        );
        assert!(
            set_cb_calls() >= 1,
            "the hook was actually asked — this is the refusal branch, not an earlier gate"
        );
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "and no callback is left installed"
        );

        let o = scan_oracle(16, 8, 9.0);
        publish_scan(publisher, &o);
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken, "the copy path still serves the frame");
        assert_scan_values(&msg, &o);
        assert!(
            fake_segments().is_empty(),
            "nothing may be registered when the grant never constructed"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

/// The grant's OWN refusal
/// branch. Nothing else drives it, so deleting the `rc != RC_OK` check leaves the
/// rest of the suite green while every take runs the adoption path with NO release
/// callback installed: the hook would forge pointers into samples nothing
/// could ever release.
///
/// `adopt.is_none()` alone does NOT say the refusal is why — no hook, an
/// unarmed env and a non-forgeable type all produce it, and none of them
/// reaches this call. The child asserts `set_cb_calls`, so the hook really
/// was asked and really said no; the parent asserts the refusal is LOUD,
/// level-matched, because an operator whose hook rejects the handshake has
/// no other way to learn that every take silently kept the copy path.
#[test]
#[serial]
fn a_hook_that_refuses_the_release_callback_leaves_every_take_on_the_copy_path() {
    let (status, err) = run_child(
        "child_hook_refusing_the_release_callback_stays_on_the_copy_path",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    let refusal = err
        .lines()
        .find(|l| l.contains("WARN") && l.contains("refused set_release_callback"))
        .unwrap_or_else(|| panic!("the refused handshake must be LOUD; stderr:\n{err}"));
    assert!(
        refusal.contains("the grant is NOT constructed"),
        "and must say adoption is OFF, not merely that a call failed: {refusal}"
    );
    assert!(
        refusal.contains("every take keeps the copy path"),
        "and must name the consequence: {refusal}"
    );
}

/// The EMPTY-SEQUENCE adopted
/// take — a forged mask with ZERO registered ranges. Ordinary robot
/// traffic (an empty `/scan`, an empty `/plan`), and pinned by nothing else:
/// the only sibling arm drives the REGISTRATION-FAILURE path, which
/// reaches its counters through a different branch entirely. Production
/// calls this shape out as correct by design — "with ZERO ranges
/// registered (every forged entry empty) this releases the sample
/// immediately" — so the pin is that the sample really is released AT ONCE
/// rather than pinned for the life of a message the app will never free
/// through the hook: `outstanding` back to 0 with `releases` still 0,
/// because no registration existed for a release callback to fire on.
///
/// This is NOT the `outcome.forged == 0` arm the deferral list described,
/// and the correction matters: an empty sequence still SETS the mask (this
/// arm measured it). `forged == 0` needs every forgeable member to sit
/// below the frame's data floor — `offset_table_offset + offset_table_bytes`
/// — which is a defensive gate against a malformed or hostile frame whose
/// entry points into the fixed section or the table, not something a
/// well-formed publisher can emit. Reaching it needs hand-built raw frames
/// that still pass the hash gate and decode, so it stays deferred with a
/// corrected cost.
#[test]
#[serial]
fn an_empty_sequence_take_forges_nothing_and_releases_the_sample_at_once() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("NoForge{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "noforge", suffix);
        let stats = adopt_stats(subscription);

        let empty = scan_oracle(0, 0, 4.0);
        publish_scan(publisher, &empty);
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken, "an empty-sequence frame is still SERVED");
        assert_scan_values(&msg, &empty);
        assert!(
            fake_segments().is_empty(),
            "every forged entry is empty, so NOTHING may be registered with the hook"
        );
        assert_eq!(
            stat(&stats),
            (1, 1, 0, 0, 0, 0),
            "served and adopted, but the sample is released AT ONCE (outstanding 0) and no \
             release callback fired (releases 0) — there was no registration to fire one"
        );
        // The counters and the served
        // label answer one question — did this take pay a payload copy?
        // An all-empty frame copies nothing, so `adopted_takes` counts it
        // and `fallbacks` does not, and the label must be `Adopted` to
        // match. Labelling it `Copied` while leaving both
        // counters as they are here would let the summary read
        // `adopted == takes` while the diagnostic says a copy happened.
        // The `Copied` side of the same rule is pinned by the budget
        // child, whose copy-served recovery asserts BOTH `served=copied`
        // and `fallbacks == 1`.
        assert_eq!(
            stat(&stats).4,
            0,
            "no fallback is counted, so no copy was paid — the label must agree"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);

        // Anti-tautology in the same body, with the same subscription: a
        // frame that DOES carry data registers its members and HOLDS the
        // sample, so the zeros above describe this frame's shape and not a
        // subscription that had stopped adopting.
        let full = scan_oracle(16, 8, 4.0);
        publish_scan(publisher, &full);
        let (taken, mut msg2, _) = take_scan(subscription);
        assert!(taken);
        assert_scan_values(&msg2, &full);
        assert_eq!(
            fake_segments().len(),
            2,
            "both members registered when there IS something to register"
        );
        assert_eq!(
            stat(&stats),
            (2, 2, 0, 1, 0, 0),
            "and THIS sample is held — the empty one really did release early"
        );
        simulate_free(msg2.ranges.data as usize);
        simulate_free(msg2.intensities.data as usize);
        msg2.ranges.data = std::ptr::null_mut();
        msg2.intensities.data = std::ptr::null_mut();
        scan_fini(&mut *msg2 as *mut _ as *mut c_void);
        assert_eq!(stat(&stats).3, 0, "and released when the app freed it");
        teardown(node, publisher, subscription);
    }
}

/// env WITHOUT any hook (macOS resolves structurally Absent — no
/// TestHookGuard installed) ⇒ the grant never constructs, the take stays
/// the copy path, and the misconfiguration warn fires EXACTLY ONCE across
/// repeated creates (the log-independent counter; the level-token-matched
/// stderr pin is the subprocess arm below). This is the arm the
/// grant-from-env-alone variant fails.
#[test]
#[serial]
fn env_without_a_hook_stays_on_the_copy_path_with_one_warn() {
    reset_fake();
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    unsafe {
        let before = missing_preload_warn_count();
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("NoHook{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "nohook", suffix);
        assert!(
            sub_data(subscription).adopt.is_none(),
            "no grant ⇒ no adopt state (the witness is load-bearing)"
        );
        assert_eq!(
            missing_preload_warn_count(),
            1,
            "the misconfiguration warn fires at the first armed create"
        );
        assert_eq!(before, 0, "no earlier test may have burned the once-latch");

        // A second armed create: still copy path, still ONE warn.
        let ts2 = scan_ts(&format!("NoHook2{suffix}"));
        let topic = CString::new(format!("/rmw_adopt/nohook2/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let sub2 = rmw_create_subscription(node, ts2, topic.as_ptr(), &qos, &sub_opts);
        assert!(!sub2.is_null());
        assert!(sub_data(sub2).adopt.is_none());
        assert_eq!(missing_preload_warn_count(), 1, "once-latched");

        // The copy path really serves frames (byte oracle).
        let o = scan_oracle(8, 4, 2.0);
        publish_scan(publisher, &o);
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken);
        assert_scan_values(&msg, &o);
        assert!(fake_segments().is_empty());
        scan_fini(&mut *msg as *mut _ as *mut c_void);

        assert_eq!(rmw_destroy_subscription(node, sub2), RMW_RET_OK);
        teardown(node, publisher, subscription);
    }
}

// =====================================================================
// Arm 4 — registration-failure fallback (byte-equal copy, no leak)
// =====================================================================

/// A registration failure on the SECOND member rolls the take back to the
/// copy path: the already-registered sibling is UNREGISTERED (the leak
/// oracle the skip-sibling-unregister variant fails), the message is
/// un-forged and the masked members are copied byte-equal to the hand
/// oracle, the sample is released, the fallback is counted + latched —
/// and the take still SUCCEEDS.
#[test]
#[serial]
fn registration_failure_falls_back_to_a_byte_equal_copy_and_leaks_nothing() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("RegF{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "regf", suffix);
        let stats = adopt_stats(subscription);
        arm_register_failure_at(2);

        let o = scan_oracle(16, 8, 3.0);
        publish_scan(publisher, &o);
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken, "the fallback take still succeeds");
        assert_scan_values(&msg, &o);
        assert!(
            fake_segments().is_empty(),
            "the registered sibling must be rolled back — nothing may leak"
        );
        {
            let s = fake().lock().expect("fake");
            assert_eq!(s.register_calls, 2, "both members attempted");
            assert_eq!(s.unregister_calls, 1, "exactly the sibling unregistered");
        }
        assert_eq!(
            stat(&stats),
            (1, 0, 0, 0, 1, 0),
            "served by copy: fallback counted, nothing adopted, sample released"
        );
        assert_eq!(
            sub_data(subscription)
                .adopt
                .as_ref()
                .unwrap()
                .registration_failure_count(),
            1,
            "latched once"
        );
        // The copies are REAL heap allocations — the ordinary fini owns them.
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

/// CHILD: one registration failure on the SECOND member, so the
/// fallback diagnostic describes a PARTIAL failure — one sibling registered
/// before the break.
#[test]
#[ignore]
fn child_registration_fallback_reports_the_partial_count() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("RegFDiag{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "regfdiag", suffix);
        arm_register_failure_at(2);
        publish_scan(publisher, &scan_oracle(16, 8, 5.0));
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken, "the fallback take still succeeds");
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

/// The fallback diagnostic must
/// report how many siblings were registered BEFORE the failure.
///
/// `registered_before_failure` is the field that separates a PARTIAL
/// failure — some ranges took, then one was refused, which is the signature
/// of a stale leaked registration — from a wholly bad range set. An earlier
/// fix added an `inner_state.adopt_registered.clear()` to the rollback and left
/// the report reading `.len()` afterwards, so the field read 0 on every
/// occurrence and the distinction was gone. The regression is visible in
/// our own gate logs: the same test's line carried
/// `registered_before_failure=1` before that fix and `=0` after it, and no
/// assertion anywhere noticed.
///
/// Asserted as a whole `key=value` token: `registered_before_failure=1` is
/// a substring of `registered_before_failure=12`, and the sibling
/// `ranges_total` is pinned in the same line so a reporter that swapped the
/// two arguments cannot pass.
#[test]
#[serial]
fn the_registration_fallback_reports_the_partial_registered_count() {
    let (status, err) = run_child(
        "child_registration_fallback_reports_the_partial_count",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    let line = err
        .lines()
        .find(|l| l.contains("WARN") && l.contains("adopt-take segment registration FAILED"))
        .unwrap_or_else(|| panic!("no fallback warn; stderr:\n{err}"));
    assert!(
        has_field(line, "registered_before_failure", "1"),
        "the FIRST member registered before the second was refused, so the report must say \
         1 — a 0 here erases the partial/total distinction the field exists for: {line}"
    );
    assert!(
        has_field(line, "ranges_total", "2"),
        "and both ranges were attempted: {line}"
    );
}

/// The configured adoption
/// budget is what bounds retention, whatever the transport was provisioned
/// at.
///
/// The create floor is `max(budget, loaned-take
/// budget)` so a small adopt budget cannot shrink the independent
/// loaned-take path.
/// That is right for provisioning and says nothing about retention: if the
/// only thing bounding retained samples were the transport
/// refusing to lend more, a budget of 2 against a service minted at 4
/// could hold FOUR SHM samples. Provisioning and retention are different
/// questions.
///
/// Budget 2 on a fresh service is the discriminating case: the floor is 4,
/// so the transport would happily lend a third and fourth.
#[test]
#[serial]
fn the_configured_budget_bounds_retention_not_the_provisioned_floor() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::set(ADOPT_TAKE_BUDGET_ENV, "2");
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Retain{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "retain", suffix);
        let stats = adopt_stats(subscription);
        {
            let state = sub_data(subscription).adopt.as_ref().expect("armed");
            assert_eq!(state.budget, 2, "the operator asked for 2");
            assert!(
                state.effective_budget >= 4,
                "and the SERVICE is provisioned higher for the loaned path (got {}) — which \
                 is exactly what must not become a retention allowance",
                state.effective_budget
            );
        }

        for i in 0..4 {
            publish_scan(publisher, &scan_oracle(8, 4, i as f32));
        }
        let mut held = Vec::new();
        for _ in 0..2 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(taken, "takes within the CONFIGURED budget serve");
            held.push(msg);
        }
        assert_eq!(stat(&stats).3, 2, "two retained");

        // THE PIN: the third take is SERVED — by copy — and retains nothing.
        // Over-budget degrades to a copy rather than refusing. The
        // budget is about how many samples may be RETAINED, and a copy
        // retains none; refusing instead would drop a frame we are already
        // holding and tell the caller nothing was taken.
        let fallbacks_before = stat(&stats).4;
        let adopted_before = stat(&stats).1;
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(
            taken,
            "over the retention budget the frame is still DELIVERED — by copy, not refused"
        );
        assert_eq!(
            stat(&stats).3,
            2,
            "and nothing new is retained: the CONFIGURED budget of 2 still binds, whatever \
             the provisioned floor of 4 would allow"
        );
        assert_eq!(
            stat(&stats).1,
            adopted_before,
            "it did not count as an adopted take"
        );
        assert_eq!(
            stat(&stats).4,
            fallbacks_before + 1,
            "it counted as a copy fallback"
        );
        // The copy is the caller's own heap memory — `fini` owns it.
        scan_fini(&mut *msg as *mut _ as *mut c_void);

        // Adoption RESUMES once a sample is freed — the documented self-heal.
        free_adopted_scan(*held.pop().expect("one held"));
        let adopted_before = stat(&stats).1;
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken, "freeing one lets the next take serve");
        assert_eq!(
            stat(&stats).1,
            adopted_before + 1,
            "and it ADOPTS again — the degrade is not sticky"
        );
        held.push(msg);

        for msg in held {
            free_adopted_scan(*msg);
        }
        teardown(node, publisher, subscription);
    }
}

/// A small adopt budget must not
/// shrink the floor the loaned-take path depends on.
///
/// Arming adopt-take requires a forgeable sequence, and `can_loan_take` is
/// `can_loan || forge_count > 0` — so an adopt-armed subscription is ALWAYS
/// loan-capable too, stays advertised as such, and keeps a shadow pool sized
/// for the loaned-take budget. The create-floor branch was an `else if`, so
/// the adopt budget REPLACED that floor instead of composing with it: a
/// `CERULION_RMW_ADOPT_TAKE_BUDGET` of 1-3 minted the service at 2-3 borrows
/// (1 and 2 are discarded as not exceeding the iceoryx2 default) and the
/// independent `rmw_take_loaned_message` path on the same subscription then
/// failed `ExceedsMaxBorrows` one or two loans early.
///
/// `effective_budget` is the EFFECTIVE service capacity, so it reads the
/// provisioning rather than the request — which is exactly the distinction
/// this bug lives in.
#[test]
#[serial]
fn a_small_adopt_budget_does_not_shrink_the_loaned_take_floor() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    // 2 is the interesting value: it is a legal budget AND it is exactly the
    // iceoryx2 default, so a floor of 2 is discarded and the service would be
    // minted at 2 — below the loaned path's own budget of 4.
    let _bud = EnvVarGuard::set(ADOPT_TAKE_BUDGET_ENV, "2");
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("FloorCompose{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "floorc", suffix);
        let data = sub_data(subscription);
        let state = data.adopt.as_ref().expect("armed");
        assert_eq!(state.budget, 2, "the adopt budget is honoured as requested");
        assert!(
            state.effective_budget >= 4,
            "the service must still be minted at the loaned-take floor (got {}) — this \
             subscription is loan-capable too, and its shadow pool is sized for it",
            state.effective_budget
        );
        teardown(node, publisher, subscription);
    }
}

/// A panic
/// in the registration-RECOVERY report must not escape the rollback window.
///
/// With `report_adopt_registration_clean` on the adopted path BEFORE the
/// guarded tail, the registrations are LIVE and their `Arc`
/// cookies hold the sample, so a panic there — a host `tracing` subscriber's
/// `on_event`, the same hazard class the report-window seam exists for —
/// takes `ForgedMessageGuard::drop`'s un-forge and never reaches
/// `withdraw_adopt_registrations`. The sample is never released and one
/// borrow-budget slot is lost for the life of the process; `budget` such
/// panics exhaust the subscription for good.
///
/// The seam fires beside that reporter and travels with it, so moving the
/// report back outside the window moves the seam too and fails this arm.
#[test]
#[serial]
fn a_panic_in_the_registration_report_still_withdraws_the_registrations() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("RegRep{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "regrep", suffix);
        let stats = adopt_stats(subscription);

        publish_scan(publisher, &scan_oracle(16, 8, 31.0));
        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg as *mut _ as *mut c_void, 0);
        let fired_before = rmw_cerulion::test_seams::adopt_registration_report_panics_fired();
        let mut taken = false;
        let ret = {
            let _seam = rmw_cerulion::test_seams::AdoptRegistrationReportPanicGuard::arm();
            rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            )
        };
        assert_eq!(
            rmw_cerulion::test_seams::adopt_registration_report_panics_fired(),
            fired_before + 1,
            "the registration-report seam must have fired — otherwise this arm proves nothing"
        );
        assert_eq!(ret, rmw_cerulion::ffi::RMW_RET_ERROR);
        assert!(!taken);

        // The caller's message is fini-safe...
        assert!(
            msg.ranges.data.is_null() && msg.intensities.data.is_null(),
            "the unwind un-forges the caller's message"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);

        // ...AND the registrations are gone with it. Leaving them is the
        // permanent leak: the app has no pointer to free, so nothing would
        // ever retire those cookies or return the borrow slot.
        assert!(
            fake_segments().is_empty(),
            "the registrations must be WITHDRAWN on the unwind (got {} still registered)",
            fake_segments().len()
        );
        assert_eq!(
            stat(&stats).3,
            0,
            "and the sample is RELEASED — one leaked slot per panic exhausts the budget"
        );
        // A take that unwinds counts
        // NOTHING, anywhere. The counters are transactional with delivery —
        // they all move in one place, after the report window has succeeded —
        // so this path, which returns `RMW_RET_ERROR` with `taken = false`,
        // must leave every one of them at zero.
        //
        // Two half-measures each satisfy one half of this and break the other.
        // Bumping `adopted_takes` in its branch and `takes`
        // inside the window lets a panic between them leave `adopted > takes`,
        // inverting the invariant the summary and the bench citability gate
        // rest on. Bumping `takes` first makes them consistent
        // and consistently WRONG: all three then count a delivery that never
        // happened against a counter documented as "successful takes". Only
        // counting after the window satisfies both.
        //
        // The liveness half — that these counters are not simply dead — is
        // `two_runs_of_the_adopted_take_match_the_hand_oracle`, which asserts
        // they reach (3, 3, 0, 3, 0, 0) on the success path in this same
        // binary. It is deliberately NOT re-proved here with a second take:
        // an `ffi_guard` panic marks the entity wedged by design, so a re-take
        // on THIS subscription is refused before it can touch a counter.
        assert_eq!(
            stat(&stats),
            (0, 0, 0, 0, 0, 0),
            "a take that unwound must contribute nothing: \
             (takes, adopted, releases, outstanding, fallbacks, refusals)"
        );

        teardown(node, publisher, subscription);
    }
}

/// A panic in the reporting tail on
/// the COPY-FALLBACK path must not empty the members the rollback just
/// filled.
///
/// The two paths need OPPOSITE things from the same unwind, which is what
/// made this easy to get wrong. On the ADOPTED path the members aim into
/// the held sample, so the unwind MUST empty them or the caller's `fini`
/// frees shared memory. On the fallback path the guard has already stood
/// down and `copy_forged_members` has refilled those members with real
/// `malloc`'d buffers — emptying them there does not free anything, it
/// drops the only pointer to each one. An earlier tail called
/// `unforge_now()` unconditionally, so every tail panic after a
/// registration failure leaked a buffer set.
///
/// The oracle is the message itself: the copied pointers must SURVIVE the
/// unwind with their bytes intact, and `fini` must then own them — which is
/// also why this arm ends by running `scan_fini` over them rather than
/// leaking them itself.
#[test]
#[serial]
fn a_tail_panic_after_a_registration_failure_keeps_the_copied_buffers() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("TailCopy{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "tailcopy", suffix);
        let stats = adopt_stats(subscription);
        // Fail the SECOND registration: the first sibling registers, then the
        // rollback withdraws it and serves the frame by copy.
        arm_register_failure_at(2);

        let o = scan_oracle(16, 8, 21.0);
        publish_scan(publisher, &o);
        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg as *mut _ as *mut c_void, 0);
        let fired_before = rmw_cerulion::test_seams::adopt_report_window_panics_fired();
        let mut taken = false;
        let ret = {
            let _seam = rmw_cerulion::test_seams::AdoptReportWindowPanicGuard::arm();
            rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            )
        };
        assert_eq!(
            rmw_cerulion::test_seams::adopt_report_window_panics_fired(),
            fired_before + 1,
            "the report-window seam must have fired — otherwise this arm proves nothing"
        );
        assert_eq!(ret, rmw_cerulion::ffi::RMW_RET_ERROR);
        assert!(!taken);

        // THE PIN: the rollback's own buffers are still there. A second,
        // ungated un-forge would have replaced these with EMPTY headers,
        // and nothing would ever free them.
        assert!(
            !msg.ranges.data.is_null(),
            "the copied `ranges` buffer must survive the unwind — emptying its header \
             here frees nothing and drops the only pointer to it"
        );
        assert!(
            !msg.intensities.data.is_null(),
            "and likewise `intensities`"
        );
        // ...and they are the COPY, byte-equal to the hand oracle, not an
        // SHM address: the sample was released by the rollback.
        assert_scan_values(&msg, &o);
        assert!(
            fake_segments().is_empty(),
            "the registration rollback still withdrew its sibling"
        );
        assert_eq!(stat(&stats).3, 0, "and the sample is released, not held");

        // The ordinary `fini` owns the copies — this is the free that
        // emptying the members would make impossible.
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

// =====================================================================
// Arm 5 — budget: refuse, latch discipline, self-heal
// =====================================================================

/// Retaining budget-many adopted messages refuses the NEXT take with
/// `taken = false` and RMW_RET_OK (never an error), counts + latches it
/// (unconditional total grows per refusal), and SELF-HEALS the moment the
/// app frees one adopted message.
#[test]
#[serial]
fn budget_refusal_is_taken_false_latched_and_self_heals_on_free() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    // 4 > SUBSCRIBER_MAX_BORROWED_HELD (3), so the created borrow cap is
    // exactly the requested budget — deterministic refusal at 4 held.
    let _bud = EnvVarGuard::set(ADOPT_TAKE_BUDGET_ENV, "4");
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Budget{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "bud", suffix);
        let stats = adopt_stats(subscription);
        let data = sub_data(subscription);
        assert_eq!(data.adopt.as_ref().unwrap().budget, 4);
        // We created the service, so the effective
        // capacity IS the requested floor.
        assert_eq!(
            data.adopt.as_ref().unwrap().effective_budget,
            4,
            "a floor-created service's effective budget equals the request"
        );

        for i in 0..5 {
            publish_scan(publisher, &scan_oracle(8, 4, i as f32));
        }
        let mut held = Vec::new();
        for i in 0..4 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(taken, "take {i} within the budget must serve");
            held.push(msg);
        }
        assert_eq!(stat(&stats).3, 4, "four samples outstanding");

        let refusals_before = data.loan_refusal_count();
        let (taken, msg5, _) = take_scan(subscription);
        assert!(!taken, "past the budget: taken=false, never an error");
        drop(msg5);
        let (taken, msg6, _) = take_scan(subscription);
        assert!(!taken, "still refused while everything is retained");
        drop(msg6);
        assert_eq!(stat(&stats).5, 2, "each refusal counted");
        assert_eq!(
            data.loan_refusal_count(),
            refusals_before + 2,
            "the latch total is unconditional (head + suppressed repeat alike)"
        );

        // Self-heal: free ONE adopted message ⇒ the next take serves.
        free_adopted_scan(*held.remove(0));
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken, "freeing one adopted message recovers the take");
        held.push(msg);

        for msg in held {
            free_adopted_scan(*msg);
        }
        assert_eq!(stat(&stats).3, 0);
        teardown(node, publisher, subscription);
    }
}

// =====================================================================
// Arm 6 — destroy with outstanding: warn-and-LEAVE, frees still release
// =====================================================================

/// Destroying the subscription with adopted samples outstanding LEAVES the
/// registrations in place (reclaiming would dangle the app's pointers) —
/// and a free AFTER destroy still fires the release callback and drops the
/// sample (the machinery is subscription-independent by construction).
#[test]
#[serial]
fn destroy_with_outstanding_leaves_registrations_and_later_frees_release() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Destroy{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "dst", suffix);
        let stats = adopt_stats(subscription); // survives destroy (Arc)

        publish_scan(publisher, &scan_oracle(16, 8, 4.0));
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken);
        assert_eq!(stat(&stats), (1, 1, 0, 1, 0, 0));

        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(
            fake_segments().len(),
            2,
            "destroy must LEAVE the registrations (never reclaim)"
        );
        assert_eq!(stat(&stats).3, 1, "the sample is still held after destroy");

        // The app frees after destroy — releases still fire, no crash.
        free_adopted_scan(*msg);
        assert_eq!(stat(&stats), (1, 1, 2, 0, 0, 0));
        assert!(fake_segments().is_empty());

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// Arm 9 — multi-member reverse-order + interior-pointer free
// =====================================================================

/// Two forged members freed in REVERSE declaration order: the first free
/// releases ONE clone (the sample stays held — the delete-one-Arc-clone
/// variant dies here), the sibling's bytes stay valid, and the SECOND free
/// through an INTERIOR pointer still resolves by containment to the right
/// cookie and releases the sample.
#[test]
#[serial]
fn reverse_order_and_interior_frees_release_by_cookie() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Rev{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "rev", suffix);
        let stats = adopt_stats(subscription);

        let o = scan_oracle(32, 16, 5.0);
        publish_scan(publisher, &o);
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken);

        // Free the SECOND-declared member first.
        simulate_free(msg.intensities.data as usize);
        msg.intensities.data = std::ptr::null_mut();
        assert_eq!(
            stat(&stats),
            (1, 1, 1, 1, 0, 0),
            "one release, sample STILL held by the sibling's clone"
        );
        // The sibling still reads the sample.
        let seen = std::slice::from_raw_parts(msg.ranges.data, msg.ranges.size);
        for (i, (a, b)) in seen.iter().zip(o.ranges.iter()).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "ranges[{i}] after sibling free");
        }
        // Interior pointer: one element in — containment resolves the cookie.
        simulate_free(msg.ranges.data as usize + 4);
        msg.ranges.data = std::ptr::null_mut();
        assert_eq!(stat(&stats), (1, 1, 2, 0, 0, 0), "last free releases");

        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

// =====================================================================
// Arm 8 — the C++ bridge twin of arm 1
// =====================================================================

#[repr(C, align(16))]
struct StringSlot([u8; 32]);
#[repr(C, align(8))]
struct VecU8Slot([usize; 3]);

/// A minimal C++ fixture: `width` (fixed), `encoding` (std::string),
/// `data` (REAL std::vector<uint8_t> — the forgeable member).
#[repr(C, align(16))]
struct CppFrame {
    width: u32,
    encoding: StringSlot,
    data: VecU8Slot,
}

unsafe extern "C" fn cframe_init(msg: *mut c_void, _init: u32) {
    let m = &mut *(msg as *mut CppFrame);
    m.width = 0;
    rmw_cerulion_cppstring_construct(
        m.encoding.0.as_mut_ptr() as *mut c_void,
        std::ptr::null(),
        0,
    );
    rmw_cerulion_vector_u8_construct(m.data.0.as_mut_ptr() as *mut c_void);
}

unsafe extern "C" fn cframe_fini(msg: *mut c_void) {
    let m = &mut *(msg as *mut CppFrame);
    rmw_cerulion_cppstring_destruct(m.encoding.0.as_mut_ptr() as *mut c_void);
    // The vector slot is destructed by the caller AFTER it has dealt with a
    // forged triplet (the test nulls it before fini, exactly as the un-forge
    // or the app's own free would have) — destruct on the all-null triplet
    // deallocates nothing.
    use rmw_cerulion::ffi::introspection_cpp::rmw_cerulion_vector_u8_destruct;
    rmw_cerulion_vector_u8_destruct(m.data.0.as_mut_ptr() as *mut c_void);
}

unsafe extern "C" fn vecu8_size(field: *const c_void) -> usize {
    rmw_cerulion_vector_u8_size(field)
}
unsafe extern "C" fn vecu8_get_const(field: *const c_void, idx: usize) -> *const c_void {
    rmw_cerulion_vector_u8_data(field).add(idx) as *const c_void
}
unsafe extern "C" fn vecu8_get(field: *mut c_void, idx: usize) -> *mut c_void {
    (rmw_cerulion_vector_u8_data(field) as *mut u8).add(idx) as *mut c_void
}

fn cpp_member(name: &str, type_id: u8, offset: u32) -> CppMessageMember {
    CppMessageMember {
        name_: cstr(name),
        type_id_: type_id,
        string_upper_bound_: 0,
        members_: std::ptr::null(),
        is_key_: false,
        is_array_: false,
        array_size_: 0,
        is_upper_bound_: false,
        offset_: offset,
        default_value_: std::ptr::null(),
        size_function: None,
        get_const_function: None,
        get_function: None,
        fetch_function: None,
        assign_function: None,
        resize_function: None,
    }
}

fn cframe_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let mut data = cpp_member(
        "data",
        ROS_TYPE_UINT8,
        std::mem::offset_of!(CppFrame, data) as u32,
    );
    data.is_array_ = true;
    data.size_function = Some(vecu8_size);
    data.get_const_function = Some(vecu8_get_const);
    data.get_function = Some(vecu8_get);
    let members = Box::leak(
        vec![
            cpp_member("width", ROS_TYPE_UINT32, 0),
            cpp_member(
                "encoding",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CppFrame, encoding) as u32,
            ),
            data,
        ]
        .into_boxed_slice(),
    );
    let mm = Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr("rmw_adopt::msg"),
        message_name_: cstr(unique),
        member_count_: members.len() as u32,
        size_of_: std::mem::size_of::<CppFrame>(),
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: Some(cframe_init),
        fini_function: Some(cframe_fini),
    }));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

unsafe fn cpp_string_of(slot: *const c_void) -> String {
    let mut data: *const c_char = std::ptr::null();
    let mut len = 0usize;
    rmw_cerulion_cppstring_view(slot, &mut data, &mut len);
    String::from_utf8_lossy(std::slice::from_raw_parts(data as *const u8, len)).into_owned()
}

/// The C++ twin of arm 1: a plain
/// take of a REAL `std::vector<uint8_t>` type under the grant writes a
/// FORGED triplet — `begin` IS the registered range's start,
/// `capacity == size` (measured through the compiled libstdc++ shim, not
/// a Rust mirror) — the string copies, and the free releases the sample.
#[test]
#[serial]
fn cpp_twin_adopts_the_vector_in_place_and_the_free_releases() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = cframe_ts(&format!("CppAdopt{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "cpp", suffix);
        let stats = adopt_stats(subscription);

        // Publish a real C++ message through the C++ bridge.
        let payload: Vec<u8> = (0..96u32)
            .map(|i| (i.wrapping_mul(2654435761) >> 24) as u8)
            .collect();
        let mut src: Box<CppFrame> = Box::new(std::mem::zeroed());
        cframe_init(&mut *src as *mut _ as *mut c_void, 0);
        src.width = 640;
        rmw_cerulion_cppstring_destruct(src.encoding.0.as_mut_ptr() as *mut c_void);
        rmw_cerulion_cppstring_construct(
            src.encoding.0.as_mut_ptr() as *mut c_void,
            "rgb8".as_ptr() as *const c_char,
            4,
        );
        rmw_cerulion_vector_u8_assign(
            src.data.0.as_mut_ptr() as *mut c_void,
            payload.as_ptr(),
            payload.len(),
        );
        assert_eq!(
            rmw_publish(
                publisher,
                &*src as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        cframe_fini(&mut *src as *mut _ as *mut c_void);

        // Take into a fresh C++ message.
        let mut msg: Box<CppFrame> = Box::new(std::mem::zeroed());
        cframe_init(&mut *msg as *mut _ as *mut c_void, 0);
        let mut taken = false;
        assert_eq!(
            rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken);

        assert_eq!(msg.width, 640);
        assert_eq!(
            cpp_string_of(msg.encoding.0.as_ptr() as *const c_void),
            "rgb8"
        );
        // The vector reads correctly THROUGH the real libstdc++ accessors —
        // and aims at the registered range with capacity == size.
        let vslot = msg.data.0.as_ptr() as *const c_void;
        assert_eq!(rmw_cerulion_vector_u8_size(vslot), payload.len());
        assert_eq!(rmw_cerulion_vector_u8_capacity(vslot), payload.len());
        let begin = rmw_cerulion_vector_u8_data(vslot) as usize;
        let segs = fake_segments();
        assert_eq!(segs.len(), 1);
        assert_eq!(segs[0].start, begin, "vector begin IS the registered range");
        assert_eq!(segs[0].len, payload.len());
        let seen = std::slice::from_raw_parts(begin as *const u8, payload.len());
        assert_eq!(seen, payload.as_slice());
        // The registered range lies inside the held
        // sample's payload (via the cookie) — never a heap copy.
        let held = held_sample_range(&segs[0]);
        assert!(
            held.contains(&segs[0].start) && held.contains(&(segs[0].start + segs[0].len - 1)),
            "the forged vector must aim inside the held sample"
        );
        assert_eq!(stat(&stats), (1, 1, 0, 1, 0, 0));

        // The app frees the vector's buffer (what ~vector / a growth does).
        simulate_free(begin);
        assert_eq!(stat(&stats), (1, 1, 1, 0, 0, 0));
        // Hand the slot back to a destructible state (the all-null triplet),
        // exactly what a real freed-and-emptied vector holds.
        std::ptr::write_unaligned(
            msg.data.0.as_mut_ptr() as *mut VecTriplet,
            VecTriplet::EMPTY,
        );
        cframe_fini(&mut *msg as *mut _ as *mut c_void);

        teardown(node, publisher, subscription);
    }
}

// =====================================================================
// Subprocess arms — stderr level-token pins (warn text + proof lines)
// =====================================================================

/// Strip ANSI SGR escape sequences (`ESC [ … m`) — the child's fmt
/// subscriber colorizes even a piped stderr, and `takes=1` must match the
/// FIELD, not fight the styling bytes.
fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' {
            for e in chars.by_ref() {
                if e == 'm' {
                    break;
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// Bounded child wait: spawn with piped stderr, wait up to 120 s (fresh
/// iceoryx2 init in the child is seconds-class), SIGKILL + fail on
/// overrun.
fn run_child(child_test: &str, envs: &[(&str, &str)]) -> (std::process::ExitStatus, String) {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = std::process::Command::new(exe);
    cmd.args([child_test, "--exact", "--ignored", "--nocapture"])
        .env("CER_RMW_ADOPT_TAKE_CHILD", "1")
        .env("RUST_LOG", "rmw_cerulion=info")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped());
    for (k, v) in envs {
        cmd.env(k, v);
    }
    let mut child = cmd.spawn().expect("spawn child");
    let mut stderr = child.stderr.take().expect("stderr piped");
    let reader = std::thread::spawn(move || {
        use std::io::Read;
        let mut buf = String::new();
        let _ = stderr.read_to_string(&mut buf);
        strip_ansi(&buf)
    });
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    let status = loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => break status,
            None if std::time::Instant::now() > deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!("child {child_test} exceeded the 120s deadline");
            }
            None => std::thread::sleep(std::time::Duration::from_millis(50)),
        }
    };
    let err = reader.join().expect("stderr reader");
    (status, err)
}

/// CHILD (runs only under the parent's re-exec): two armed creates with NO
/// hook — the parent asserts the once-warn on stderr.
#[test]
#[ignore]
fn child_env_without_preload_two_creates() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    unsafe {
        let suffix = unique_suffix();
        let node = setup_node(&format!("warn_child_{suffix}"));
        for tag in ["one", "two"] {
            let ts = scan_ts(&format!("Warn{tag}{suffix}"));
            let topic = CString::new(format!("/rmw_adopt/warn/{tag}/{suffix}")).expect("topic");
            let qos = default_qos();
            let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
            let sub = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
            assert!(!sub.is_null());
            assert!(sub_data(sub).adopt.is_none());
            assert_eq!(rmw_destroy_subscription(node, sub), RMW_RET_OK);
        }
    }
}

/// The stderr pin (level-token-matched):
/// env-without-preload emits EXACTLY ONE `WARN` line naming the env var,
/// the missing preload, and the DIRECT-launch remedy — across TWO armed
/// creates — and never an "ARMED" line. The remedy half follows the
/// launcher-refusal rule: the launcher flag always refuses, so a line that
/// named it would point a reader at a dead end.
#[test]
#[serial]
fn env_without_preload_warn_is_once_level_matched_and_names_the_fix() {
    let (status, err) = run_child(
        "child_env_without_preload_two_creates",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    // A class sweep found this: `ADOPT_TAKE_ENV` is a strict PREFIX of
    // `CERULION_RMW_ADOPT_TAKE_BUDGET`, so filtering on the env NAME also
    // matches the budget-parse warn, the WARN-level budget-exhaustion
    // refusals, and `warn_bad_env_value_once` — the same whole-token
    // discipline the numeric fields use. The head is
    // located by the sentence this warn alone emits.
    let warn_lines: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("WARN") && l.contains(MISSING_PRELOAD_WARN_SUBSTR))
        .collect();
    assert_eq!(
        warn_lines.len(),
        1,
        "exactly one once-latched warn across two creates; stderr:\n{err}"
    );
    let line = warn_lines[0];
    assert!(
        line.contains(ADOPT_TAKE_ENV),
        "names the env var that asked for adoption: {line}"
    );
    assert!(
        line.contains("libcerulion_heaphook.so"),
        "names the missing preload: {line}"
    );
    // The remedy has two requirements. First: the
    // launcher flag cannot stage the hook for a `ros2`-spawned node any more
    // (it refuses), so naming `--adopt-take` alone would send the reader
    // straight back to a verb that always exits 69. Second:
    // the remedy is PLATFORM-dependent, because the hook interposes glibc's
    // malloc/free — on a non-Linux/GNU host there is nothing to preload and
    // a preload instruction would be impossible to follow.
    //
    // BOTH arms are asserted; neither host skips. Each checks its own
    // positive AND the other arm's distinguishing phrase as a negative, so a
    // remedy that shipped the wrong arm for the host fails here rather than
    // reading plausibly.
    if cfg!(all(target_os = "linux", target_env = "gnu")) {
        assert!(
            line.contains("LD_PRELOAD"),
            "on Linux/GNU, names the variable the hook must be preloaded under: {line}"
        );
        assert!(
            line.contains("DIRECTLY"),
            "on Linux/GNU, says the node must be launched directly: {line}"
        );
        assert!(
            !line.contains("Linux/GNU-ONLY"),
            "and must NOT serve the unsupported-host arm here: {line}"
        );
    } else {
        assert!(
            line.contains("Linux/GNU-ONLY"),
            "off Linux/GNU, says adoption is unavailable on this host: {line}"
        );
        assert!(
            line.contains("nothing to preload on THIS host"),
            "off Linux/GNU, says there is nothing to preload: {line}"
        );
        assert!(
            !line.contains("LD_PRELOAD=<lib dir>"),
            "and must NOT hand an impossible preload command to a host with no hook: {line}"
        );
    }
    // ...and the job the OLD, over-broad filter was also doing, kept
    // rather than dropped: no OTHER warn in this child may name the gate
    // variable. Without it, routing the healthy `Some("1")` arm through
    // `warn_bad_env_value_once` as well — a warn that names
    // `CERULION_RMW_ADOPT_TAKE` and fires in this child — passes every
    // other assertion here.
    let env_named: Vec<&str> = err
        .lines()
        .filter(|l| {
            l.contains("WARN") && l.contains(ADOPT_TAKE_ENV) && !l.contains(ADOPT_TAKE_BUDGET_ENV)
        })
        .collect();
    assert_eq!(
        env_named.len(),
        1,
        "no OTHER warn may name the gate env var; stderr:\n{err}"
    );
    assert!(
        !err.contains("adopt-take ARMED"),
        "no subscription may arm without the grant; stderr:\n{err}"
    );
}

/// CHILD: full adopted lifecycle with an outstanding sample at destroy AND
/// at shutdown — emits both summary proof lines + both warns.
#[test]
#[ignore]
fn child_summary_lines_with_outstanding() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Sum{suffix}"));
        let (context, node) = setup_node_with_context(&format!("sum_child_{suffix}"));
        let topic = CString::new(format!("/rmw_adopt/sum/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        publish_scan(publisher, &scan_oracle(8, 4, 6.0));
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken);
        // Deliberately NOT freed — outstanding at destroy and shutdown.
        std::mem::forget(msg);

        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_shutdown(context), RMW_RET_OK);
        // The shutdown arm must have CLEARED the release callback.
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "rmw_shutdown with outstanding must clear the release callback"
        );
    }
}

/// CHILD: destroy while a sample is HELD, then free it, then
/// shut down — the drained entry retires, so the shutdown aggregate can
/// only be right if its counts were FOLDED on the way out.
#[test]
#[ignore]
fn child_summary_after_a_drained_destroy_retires() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Drain{suffix}"));
        let (context, node) = setup_node_with_context(&format!("drain_child_{suffix}"));
        let topic = CString::new(format!("/rmw_adopt/drain/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());

        publish_scan(publisher, &scan_oracle(8, 4, 6.0));
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken);

        // Destroy while HELD, then let the app free — the drain is what
        // makes the entry retirable.
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        simulate_free(msg.ranges.data as usize);
        simulate_free(msg.intensities.data as usize);
        msg.ranges.data = std::ptr::null_mut();
        msg.intensities.data = std::ptr::null_mut();
        assert_eq!(
            registered_stats_count(),
            0,
            "the drained entry must have retired before shutdown reads the registry"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        assert_eq!(rmw_shutdown(context), RMW_RET_OK);
    }
}

/// The counts of a subscription that
/// retires on the DRAIN (destroyed while held, freed afterwards) must
/// still reach the shutdown aggregate — and exactly once.
///
/// This is the half `a_subscription_destroyed_with_samples_held_keeps_its_counters_live`
/// cannot see. That arm reads the registry; this one reads what shutdown
/// PRINTS, which is the only place the fold is observable. Three
/// regressions land on it and nowhere else: removing the entry without
/// folding leaves an empty registry and no retired totals, so the
/// aggregate summary is not printed at all; folding without removing
/// double counts every field; and folding twice does the same.
#[test]
#[serial]
fn a_drained_destroy_folds_its_counts_into_the_shutdown_aggregate_exactly_once() {
    let (status, err) = run_child(
        "child_summary_after_a_drained_destroy_retires",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    let summaries: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("adopt-take summary"))
        .collect();
    // One at destroy (the subscription's own numbers, sample still held)
    // and one at shutdown (the aggregate, read after the free).
    assert_eq!(
        summaries.len(),
        2,
        "a destroy summary and a shutdown aggregate; a shutdown that finds neither a \
         live entry nor retired totals prints NOTHING, which is what dropping the fold \
         looks like; stderr:\n{err}"
    );
    let shutdown = summaries[1];
    assert!(shutdown.contains("INFO"), "aggregate is INFO: {shutdown}");
    for (key, value) in [
        ("takes", "1"),
        ("adopted", "1"),
        ("released", "2"),
        ("outstanding", "0"),
        ("fallbacks", "0"),
        ("budget_refusals", "0"),
    ] {
        assert!(
            has_field(shutdown, key, value),
            "field `{key}={value}` missing as a whole token — a doubled value is an \
             entry that was folded AND left registered: {shutdown}"
        );
    }
    // The frees landed before shutdown, so nothing is outstanding and the
    // leak warn must be absent (the anti-vacuity half: a run where the
    // free never reached the counters would trip it).
    assert!(
        !err.contains("rmw_shutdown with adopted samples outstanding"),
        "nothing is outstanding at shutdown; stderr:\n{err}"
    );
}

/// The proof lines, end to end on stderr: the DESTROY summary (with
/// topic + the six stable field spellings), the destroy warn, the SHUTDOWN
/// aggregate summary, and the shutdown leak-and-count warn — level tokens
/// matched.
#[test]
#[serial]
fn summary_proof_lines_carry_the_stable_fields_at_destroy_and_shutdown() {
    let (status, err) = run_child(
        "child_summary_lines_with_outstanding",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    let summaries: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("adopt-take summary"))
        .collect();
    assert_eq!(
        summaries.len(),
        2,
        "one destroy summary + one shutdown aggregate; stderr:\n{err}"
    );
    for line in &summaries {
        assert!(line.contains("INFO"), "summary is INFO: {line}");
        // Whole-token matches: a substring `takes=1` is satisfied
        // by `takes=10`; `has_field` matches `key=value` as one token.
        for (key, value) in [
            ("takes", "1"),
            ("adopted", "1"),
            ("released", "0"),
            ("outstanding", "1"),
            ("fallbacks", "0"),
            ("budget_refusals", "0"),
        ] {
            assert!(
                has_field(line, key, value),
                "field `{key}={value}` missing as a whole token: {line}"
            );
        }
    }
    let destroy_warn = err
        .lines()
        .find(|l| l.contains("WARN") && l.contains("destroyed with adopted samples outstanding"))
        .unwrap_or_else(|| panic!("destroy warn missing; stderr:\n{err}"));
    assert!(
        has_field(destroy_warn, "outstanding", "1"),
        "{destroy_warn}"
    );
    let shutdown_warn = err
        .lines()
        .find(|l| l.contains("WARN") && l.contains("rmw_shutdown with adopted samples outstanding"))
        .unwrap_or_else(|| panic!("shutdown warn missing; stderr:\n{err}"));
    assert!(
        has_field(shutdown_warn, "outstanding", "1"),
        "{shutdown_warn}"
    );
}

// =====================================================================
// Context lifecycle, panic window, registry
// leak
// =====================================================================

/// CHILD: a FINAL shutdown with NOTHING outstanding — the
/// ordinary, healthy shape — must still clear the release callback.
#[test]
#[ignore]
fn child_final_shutdown_with_nothing_outstanding_clears_the_callback() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ClearClean{suffix}"));
        let (context, node) = setup_node_with_context(&format!("clearclean_{suffix}"));
        let topic = CString::new(format!("/rmw_adopt/clearclean/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let stats = adopt_stats(subscription);

        // Adopt, then FREE — the app behaved perfectly.
        publish_scan(publisher, &scan_oracle(8, 4, 9.0));
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken);
        free_adopted_scan(*msg);
        assert_eq!(stat(&stats).3, 0, "nothing outstanding");
        // The grant installed the callback, and it is still installed here.
        assert!(
            fake().lock().expect("fake").release_cb.is_some(),
            "precondition: the grant installed the callback"
        );

        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_shutdown(context), RMW_RET_OK);
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "a FINAL shutdown must clear the callback even with nothing outstanding"
        );
    }
}

/// Stale callback: a final shutdown clears
/// the process-global release callback on EVERY final shutdown, not only
/// when adopted samples are still outstanding.
///
/// A clear that lives inside the summary's `outstanding > 0` arm lets
/// the healthy shape — adopt, free everything, shut down — leave a callback
/// installed pointing into this module after the last context was gone.
/// Nothing about shutdown guarantees the module stays mapped, and the hook
/// would still invoke it for any range that remained registered. The
/// clear is cheap and sound at any time; it is the outstanding-sample LEAK,
/// not the clear, that the summary's arm is really about.
///
/// A child process, because the callback is process-global and clearing it
/// would leak into every sibling test in this binary.
#[test]
#[serial]
fn a_final_shutdown_clears_the_callback_even_with_nothing_outstanding() {
    let (status, err) = run_child(
        "child_final_shutdown_with_nothing_outstanding_clears_the_callback",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
}

/// CHILD: a callback is installed and the live set is EMPTY, then
/// an init fails — the rollback is the last member out.
#[test]
#[ignore]
fn child_rollback_empties_the_live_set_and_clears_the_callback() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        assert_eq!(
            live_context_count(),
            0,
            "a fresh process: the failing init is the LAST member out"
        );
        // The state an interleaved shutdown leaves behind: a callback the
        // previous shutdown declined to clear because it was not final then.
        let api = rmw_cerulion::heaphook::active_hook().expect("the fake hook is installed");
        assert_eq!((api.set_release_callback)(Some(noop_release_callback)), 0);
        assert!(
            fake().lock().expect("fake").release_cb.is_some(),
            "precondition: a callback is installed"
        );

        let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        let fired_before = rmw_cerulion::test_seams::init_announce_panics_fired();
        let ret = {
            let _seam = rmw_cerulion::test_seams::InitAnnouncePanicGuard::arm();
            rmw_init(&*options, context)
        };
        assert_eq!(
            rmw_cerulion::test_seams::init_announce_panics_fired(),
            fired_before + 1,
            "the announce seam must have fired — otherwise this arm proves nothing"
        );
        assert_eq!(ret, rmw_cerulion::ffi::RMW_RET_ERROR);
        assert_eq!(live_context_count(), 0, "the rollback removed its entry");

        // THE PIN: with nobody left, the callback must be gone. Leaving it
        // is a call into a possibly-unmapped module on the next free.
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "a rollback that empties the live set must clear the release callback"
        );
        Box::leak(options);
    }
}

/// A rollback that empties the live set is the last
/// context leaving, and must clear the process-global release callback.
///
/// A failed `rmw_init` that removes its live-set entry, and only
/// that, misses a reachable order: an older context shuts down while
/// this one is PROVISIONALLY in the set (it is inserted before the caller's
/// struct is written), so that shutdown correctly decides it is not final
/// and KEEPS the callback — a surviving context's frees still need it. Then
/// this init fails. The process is left with NO contexts and a callback
/// still pointing into this library, and a later free of a still-registered
/// range calls an address that may since have been unmapped.
///
/// The production path needs two contexts interleaving, which no
/// single-threaded harness can stage. What it produces is a STATE —
/// callback installed, and the failing init is the last member out — and
/// that state is what the child constructs directly, by installing the
/// callback through the fake hook's own setter. The property under test is
/// the rollback's, not the interleave's.
///
/// A CHILD process, because the live-context set is process-global and this
/// arm needs it EMPTY: sibling tests in this binary leak contexts they never
/// shut down, so the precondition cannot hold in-process.
#[test]
#[serial]
fn a_rollback_that_empties_the_live_set_clears_the_release_callback() {
    let (status, err) = run_child(
        "child_rollback_empties_the_live_set_and_clears_the_callback",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
}

/// An `rmw_init` that fails after
/// registering the context must leave no lifecycle state behind.
///
/// `rmw_init` inserts the context into the adopt-take live set BEFORE
/// writing the caller's struct (so a context somehow already live is
/// refused with the struct untouched), and then runs work that can fail in
/// a way this module does not control: the post-registration `warn!`/`info!`
/// go to the HOST's subscriber, and `install_tracing` deliberately lets the
/// host's win. `ffi_guard` turns such a panic into `RMW_RET_ERROR` — and
/// the entry, plus the written context, must not survive it. The caller has
/// been told the init failed, so it will never call `rmw_shutdown`; the
/// orphaned entry is the one that decides whether the LAST shutdown may
/// clear the process-global release callback, so one failed init could
/// suppress that clear for the life of the process.
///
/// The panic is injected by a seam rather than by a real panicking
/// subscriber: `install_tracing` hands the slot to the host, and no
/// in-process harness here can install a foreign subscriber that panics on
/// one specific event. The seam reproduces the CONTROL FLOW the rollback
/// has to survive, which is what is being pinned.
#[test]
#[serial]
fn a_panicking_announce_leaves_no_lifecycle_state_behind() {
    unsafe {
        let before = live_context_count();
        let fired_before = rmw_cerulion::test_seams::init_announce_panics_fired();
        let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));

        let ret = {
            let _seam = rmw_cerulion::test_seams::InitAnnouncePanicGuard::arm();
            rmw_init(&*options, context)
        };
        assert_eq!(
            rmw_cerulion::test_seams::init_announce_panics_fired(),
            fired_before + 1,
            "the announce seam must have fired — otherwise this arm proves nothing"
        );
        assert_eq!(
            ret,
            rmw_cerulion::ffi::RMW_RET_ERROR,
            "the failed init reports failure"
        );
        assert_eq!(
            live_context_count(),
            before,
            "a FAILED init must leave no live-set entry — nothing will ever shut it down"
        );
        // ...and the caller's struct is back to the zeroed shape `rmw_init`
        // requires on entry, so a RETRY is accepted rather than refused as
        // already-initialized.
        assert!(
            (*context).implementation_identifier.is_null(),
            "the failed init must not leave the caller's context written"
        );
        assert_eq!(
            rmw_init(&*options, context),
            RMW_RET_OK,
            "and the retry succeeds — the rollback left a reusable context"
        );
        assert_eq!(rmw_shutdown(context), RMW_RET_OK);
        Box::leak(options);
    }
}

/// CHILD: TWO live rmw contexts, adopted samples outstanding — an
/// intermediate `rmw_shutdown` must KEEP the process-global release
/// callback (a release must still land through it), and only the FINAL
/// context's shutdown may clear it.
#[test]
#[ignore]
fn child_two_contexts_keep_the_callback_until_last() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("TwoCtx{suffix}"));
        // Context 1 owns the node + pair; context 2 is a second LIVE
        // context (rmw_init only — liveness is what the refcount tracks).
        let (ctx1, node) = setup_node_with_context(&format!("twoctx_{suffix}"));
        let mut options2: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options2, allocator), RMW_RET_OK);
        let ctx2: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options2, ctx2), RMW_RET_OK);
        Box::leak(options2);

        let topic = CString::new(format!("/rmw_adopt/twoctx/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let stats = adopt_stats(subscription);

        publish_scan(publisher, &scan_oracle(8, 4, 7.0));
        publish_scan(publisher, &scan_oracle(8, 4, 8.0));
        let (taken1, msg1, _) = take_scan(subscription);
        let (taken2, msg2, _) = take_scan(subscription);
        assert!(taken1 && taken2);
        assert_eq!(stat(&stats).3, 2, "two samples outstanding");

        // INTERMEDIATE shutdown (ctx2): the callback must be KEPT.
        assert_eq!(rmw_shutdown(ctx2), RMW_RET_OK);
        assert!(
            fake().lock().expect("fake").release_cb.is_some(),
            "an intermediate context's shutdown must KEEP the release callback"
        );
        // ...and a release still LANDS through it.
        free_adopted_scan(*msg1);
        assert_eq!(
            stat(&stats).2,
            2,
            "the kept callback must still release (two ranges freed)"
        );
        assert_eq!(stat(&stats).3, 1, "one sample still outstanding");

        // FINAL shutdown (ctx1) with outstanding > 0: NOW it clears.
        std::mem::forget(msg2);
        assert_eq!(rmw_shutdown(ctx1), RMW_RET_OK);
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "the final context's shutdown must clear the release callback"
        );

        // The lifecycle-race recovery arm: a context that becomes live
        // AFTER the final clear re-installs the callback at its first
        // armed create (the grant install is serialized with the clear
        // under the ONE lifecycle lock, so whichever order they land in,
        // a live context always ends up with a working callback) — and
        // its adopted samples release normally.
        let mut options3: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        assert_eq!(rmw_init_options_init(&mut *options3, allocator), RMW_RET_OK);
        let ctx3: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options3, ctx3), RMW_RET_OK);
        Box::leak(options3);
        let node3 = rmw_create_node(ctx3, cstr(&format!("twoctx3_{suffix}")), cstr("/"));
        assert!(!node3.is_null());
        let ts3 = scan_ts(&format!("TwoCtx3{suffix}"));
        let topic3 = CString::new(format!("/rmw_adopt/twoctx3/{suffix}")).expect("topic");
        let sub3 = rmw_create_subscription(node3, ts3, topic3.as_ptr(), &qos, &sub_opts);
        assert!(!sub3.is_null());
        assert!(
            fake().lock().expect("fake").release_cb.is_some(),
            "an armed create in a fresh context must RE-INSTALL the callback"
        );
        let pub3 = rmw_create_publisher(node3, ts3, topic3.as_ptr(), &qos, &pub_opts);
        assert!(!pub3.is_null());
        let stats3 = adopt_stats(sub3);
        publish_scan(pub3, &scan_oracle(4, 2, 12.0));
        let (taken3, msg3, _) = take_scan(sub3);
        assert!(taken3, "the fresh context adopts normally");
        free_adopted_scan(*msg3);
        assert_eq!(
            stat(&stats3).2,
            2,
            "the re-installed callback releases the fresh context's samples"
        );
        // Its own final shutdown clears again (msg2's sample is still the
        // outstanding one).
        assert_eq!(rmw_shutdown(ctx3), RMW_RET_OK);
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "the fresh context's final shutdown clears again"
        );
    }
}

/// With two live contexts and adopted
/// samples outstanding, shutting down ONE context keeps the process-global
/// release callback armed (the surviving context's frees must still
/// release — the level-matched KEPT warn says so), and only the LAST
/// shutdown runs the clear-and-leak arm. The in-child asserts carry the
/// callback-state oracle; the stderr pins carry the loudness.
#[test]
#[serial]
fn intermediate_shutdown_keeps_the_callback_and_the_last_one_clears() {
    let (status, err) = run_child(
        "child_two_contexts_keep_the_callback_until_last",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    // A class sweep found this: these were `find`s, so an extra CLEARED warn
    // — the exact damage a variant that clears on an INTERMEDIATE shutdown
    // does — was invisible. The child's shape fixes both counts: it shuts
    // down ctx2 (intermediate ⇒ KEPT), then ctx1 (final ⇒ CLEARED), then
    // ctx3 (its own final ⇒ CLEARED again — reached only because the
    // deliberately forgotten `msg2` keeps the process-global outstanding
    // total above zero; BOTH warns are gated on that registry-wide total,
    // so freeing it would make this count 1 with the shape unchanged).
    let kept: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("WARN") && l.contains("the release callback is KEPT"))
        .collect();
    assert_eq!(
        kept.len(),
        1,
        "exactly one intermediate shutdown keeps it; stderr:\n{err}"
    );
    assert!(
        kept[0].contains("OTHER rmw contexts are still live"),
        "{}",
        kept[0]
    );
    let cleared: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("WARN") && l.contains("the release callback is CLEARED"))
        .collect();
    assert_eq!(
        cleared.len(),
        2,
        "exactly the two FINAL shutdowns clear it; stderr:\n{err}"
    );
    // Both, not just the first: a wrong `outstanding` on any clear after
    // the head would otherwise be invisible, and the second line is also
    // what pins that the forgotten sample is still counted across the
    // re-init cycle.
    for line in &cleared {
        assert!(has_field(line, "outstanding", "1"), "{line}");
    }
}

/// CHILD: `rmw_shutdown` is idempotent per context.
/// Two live contexts with adopted samples outstanding; the SECOND context
/// is shut down TWICE. The repeat is the rmw.h no-op (`RMW_RET_OK`,
/// nothing decremented): the live set still holds context 1, the
/// process-global release callback stays installed, a release still lands
/// through it, and context 1 still ADOPTS. With the lifecycle kept as an
/// UNKEYED count the repeat would decrement it to zero and clear the
/// callback under a live context, routing its later frees onto the hook's
/// no-callback path. The two other edges of the same state machine ride
/// along, each per rmw.h: `rmw_init` on an already-initialized context and
/// `rmw_context_fini` on a context that was never shut down both refuse
/// with `RMW_RET_INVALID_ARGUMENT` and leave the context untouched, while
/// fini AFTER shutdown zeroes it.
#[test]
#[ignore]
fn child_repeat_shutdown_of_one_context_is_a_no_op() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("RepeatCtx{suffix}"));
        let (ctx1, node) = setup_node_with_context(&format!("repeatctx_{suffix}"));
        let mut options2: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options2, allocator), RMW_RET_OK);
        let ctx2: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options2, ctx2), RMW_RET_OK);
        assert_eq!(live_context_count(), 2, "two contexts live");

        // rmw.h: `rmw_init` on an already-initialized context refuses, and
        // counts nothing.
        assert_eq!(
            rmw_init(&*options2, ctx2),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "rmw_init on an already-initialized context is INVALID_ARGUMENT"
        );
        assert_eq!(
            live_context_count(),
            2,
            "a refused re-init must count nothing"
        );
        // The assertion
        // above is answered by the rmw.h "already initialized" check —
        // `ctx2` carries our identifier, so the identity-keyed LIVE-SET
        // guard beneath it is never reached, and EITHER guard would be
        // individually deletable with the suite green. This is the shape
        // the live-set guard exists for, and the only one that reaches it:
        // the caller its production comment names, who re-zeroes a LIVE
        // struct by hand, so the rmw.h check passes and only the live set
        // knows the context is already running. `rmw_context_t` is `Copy`,
        // so the struct is put back verbatim and the lifecycle arithmetic
        // below is untouched. The restore is deliberately not RAII: a
        // failing assert panics before it and leaves `ctx2` zeroed with a
        // live-set entry orphaned, which is harmless ONLY because this
        // body runs solely under a `run_child` re-exec with `--exact`, so
        // the process dies with the panic and every `live_context_count()`
        // assertion in the file lives inside that one child. The same
        // pattern outside a child would poison the binary.
        let saved = *ctx2;
        std::ptr::write_bytes(ctx2, 0, 1);
        assert_eq!(
            rmw_init(&*options2, ctx2),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "a re-zeroed LIVE context is refused by the identity-keyed live set"
        );
        assert!(
            (*ctx2).implementation_identifier.is_null(),
            "the refused init wrote nothing into the caller's struct"
        );
        assert_eq!(
            live_context_count(),
            2,
            "the live-set refusal must count nothing either"
        );
        *ctx2 = saved;
        Box::leak(options2);

        let topic = CString::new(format!("/rmw_adopt/repeatctx/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let stats = adopt_stats(subscription);

        publish_scan(publisher, &scan_oracle(8, 4, 7.0));
        publish_scan(publisher, &scan_oracle(8, 4, 8.0));
        let (taken1, msg1, _) = take_scan(subscription);
        let (taken2, msg2, _) = take_scan(subscription);
        assert!(taken1 && taken2);
        assert_eq!(stat(&stats).3, 2, "two samples outstanding");
        let installs_before = fake().lock().expect("fake").set_cb_calls;
        assert!(
            installs_before >= 1,
            "the armed create installed the callback"
        );

        // rmw.h: fini on a context that was never shut down refuses and
        // leaves it UNTOUCHED (and its live-set entry intact).
        assert_eq!(
            rmw_context_fini(ctx1),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "rmw_context_fini on a live (un-shut-down) context is INVALID_ARGUMENT"
        );
        assert!(
            !(*ctx1).implementation_identifier.is_null(),
            "a refused fini leaves the context untouched"
        );
        assert_eq!(live_context_count(), 2, "a refused fini orphans nothing");

        // THE PIN: context 2 shut down TWICE. The repeat is a no-op — the
        // count stays at context 1 alone, and the callback it still needs
        // is KEPT.
        assert_eq!(rmw_shutdown(ctx2), RMW_RET_OK);
        assert_eq!(
            rmw_shutdown(ctx2),
            RMW_RET_OK,
            "a repeat shutdown of an already-shut-down context is the rmw.h no-op"
        );
        assert_eq!(
            live_context_count(),
            1,
            "the repeat must not decrement past context 2's own removal"
        );
        assert!(
            fake().lock().expect("fake").release_cb.is_some(),
            "the repeat shutdown must not clear the callback context 1's frees still need"
        );
        assert_eq!(
            fake().lock().expect("fake").set_cb_calls,
            installs_before,
            "neither of context 2's shutdowns touched the callback"
        );
        // fini AFTER shutdown: the contract's happy path — zeroed in place.
        assert_eq!(rmw_context_fini(ctx2), RMW_RET_OK);
        assert!(
            (*ctx2).implementation_identifier.is_null(),
            "fini after shutdown zeroes the context"
        );
        assert_eq!(
            live_context_count(),
            1,
            "fini of a shut-down context changes no count"
        );

        // ...and a release still LANDS through the kept callback.
        free_adopted_scan(*msg1);
        assert_eq!(
            stat(&stats).2,
            2,
            "the kept callback must still release (two ranges freed)"
        );
        assert_eq!(stat(&stats).3, 1, "one sample still outstanding");

        // ...and context 1 still ADOPTS.
        publish_scan(publisher, &scan_oracle(8, 4, 9.0));
        let (taken3, msg3, _) = take_scan(subscription);
        assert!(taken3, "context 1 still takes after the repeat shutdown");
        assert_eq!(
            stat(&stats).1,
            3,
            "context 1 still ADOPTS after the repeat shutdown (three adopted takes)"
        );
        assert_eq!(stat(&stats).3, 2, "two samples outstanding again");

        // FINAL shutdown (ctx1) with outstanding > 0: NOW it clears — once.
        // A repeat of the final shutdown is the same no-op: the clear arm
        // does not run a second time (the install/clear call count holds).
        std::mem::forget(msg2);
        std::mem::forget(msg3);
        assert_eq!(rmw_shutdown(ctx1), RMW_RET_OK);
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "the final context's shutdown must clear the release callback"
        );
        assert_eq!(
            live_context_count(),
            0,
            "no context live after the final shutdown"
        );
        let installs_after_clear = fake().lock().expect("fake").set_cb_calls;
        assert_eq!(
            installs_after_clear,
            installs_before + 1,
            "exactly one clear"
        );
        assert_eq!(
            rmw_shutdown(ctx1),
            RMW_RET_OK,
            "a repeat final shutdown is the no-op"
        );
        assert_eq!(live_context_count(), 0);
        assert_eq!(
            fake().lock().expect("fake").set_cb_calls,
            installs_after_clear,
            "the repeat final shutdown must not run the clear arm again"
        );
        assert_eq!(rmw_context_fini(ctx1), RMW_RET_OK);
        assert!((*ctx1).implementation_identifier.is_null());
    }
}

/// `rmw_shutdown` is idempotent per context — with
/// two contexts live and adopted samples outstanding, shutting the second
/// one down TWICE leaves the count at 1, the process-global release
/// callback installed, and the first context adopting (the in-child
/// asserts carry that oracle). The stderr pins carry the no-op half: the
/// intermediate KEPT warn and the final CLEARED warn each appear EXACTLY
/// once — a repeat that re-ran the summary would print a second KEPT line
/// even with the callback intact — plus the refused-fini warn.
#[test]
#[serial]
fn a_repeat_shutdown_of_one_context_is_a_no_op_and_keeps_the_callback() {
    let (status, err) = run_child(
        "child_repeat_shutdown_of_one_context_is_a_no_op",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    let kept = err
        .lines()
        .filter(|l| l.contains("WARN") && l.contains("the release callback is KEPT"))
        .count();
    assert_eq!(
        kept, 1,
        "exactly one KEPT warn — a repeat shutdown must not re-run the summary; stderr:\n{err}"
    );
    let cleared = err
        .lines()
        .filter(|l| l.contains("WARN") && l.contains("the release callback is CLEARED"))
        .count();
    assert_eq!(
        cleared, 1,
        "exactly one CLEARED warn — the final clear runs once; stderr:\n{err}"
    );
    let refused_fini = err
        .lines()
        .filter(|l| {
            l.contains("WARN")
                && l.contains("rmw_context_fini on a context that was never shut down")
        })
        .count();
    assert_eq!(refused_fini, 1, "the refused fini is loud; stderr:\n{err}");
}

/// Whole-whitespace-token `key=value` match — never a substring.
fn has_field(line: &str, key: &str, value: &str) -> bool {
    let needle = format!("{key}={value}");
    line.split_whitespace().any(|token| token == needle)
}

/// CHILD: the adopted-budget refusal's
/// SUPPRESSED repeats stay diagnostically specific. Budget 4, four adopted
/// messages retained, then TEN refused takes — the head, the eight `debug!`
/// repeats and the decade re-announcement must EACH carry
/// `kind=adopted_budget_exhausted`, `budget=4` and the remedy (free adopted
/// messages, or raise CERULION_RMW_ADOPT_TAKE_BUDGET). Repeats that
/// took the shared generic arm would drop the remedy from the normal
/// retry path. The parent runs this child at RUST_LOG=rmw_cerulion=debug.
#[test]
#[ignore]
fn child_adopted_budget_repeats_keep_their_diagnostic() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("BudgetRep{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "budrep", suffix);
        let stats = adopt_stats(subscription);
        assert_eq!(sub_data(subscription).adopt.as_ref().unwrap().budget, 4);
        for i in 0..5 {
            publish_scan(publisher, &scan_oracle(8, 4, i as f32));
        }
        let mut held = Vec::new();
        for i in 0..4 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(taken, "take {i} within the budget must serve");
            held.push(msg);
        }
        for _ in 0..10 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(!taken, "refused while everything is retained");
            drop(msg);
        }
        assert_eq!(stat(&stats).5, 10, "ten refusals counted");
        for msg in held {
            free_adopted_scan(*msg);
        }
        teardown(node, publisher, subscription);
    }
}

/// With the child driven past the decade at
/// debug, every line of the adopted-budget regime — the WARN head, the eight
/// DEBUG repeats, the WARN decade re-announcement — carries the kind, the
/// budget and the remedy; no repeat falls to the generic
/// `loaned take refused (suppressed repeat)` line.
#[test]
#[serial]
fn adopted_budget_suppressed_repeats_keep_the_kind_budget_and_remedy() {
    let (status, err) = run_child(
        "child_adopted_budget_repeats_keep_their_diagnostic",
        &[
            (ADOPT_TAKE_ENV, "1"),
            (ADOPT_TAKE_BUDGET_ENV, "4"),
            ("RUST_LOG", "rmw_cerulion=debug"),
        ],
    );
    assert!(status.success(), "child failed:\n{err}");
    let lines: Vec<&str> = err.lines().collect();
    let heads: Vec<&str> = lines
        .iter()
        .copied()
        .filter(|l| l.contains("WARN") && l.contains("adopted take refused —"))
        .collect();
    assert_eq!(heads.len(), 1, "exactly one loud head; stderr:\n{err}");
    // DEBUG-count discipline: this is a DEBUG COUNT,
    // and `release_max_level_info` compiles those lines out — so the
    // expectation is a function of what was compiled in, and the release
    // contract is pinned LEVEL-FREE beside it. Without that twin a
    // `debug!`→`warn!` promotion passes in release, where the gated count
    // reads 0 and the only assertion is skipped.
    const ADOPTED_SUPPRESSED: &str = "adopted take refused (suppressed repeat)";
    cerulion_core::testing::never_loud(&lines, ADOPTED_SUPPRESSED)
        .unwrap_or_else(|e| panic!("{e}; stderr:\n{err}"));
    let repeats =
        cerulion_core::testing::lines_at_exclusively(&lines, "DEBUG", &[ADOPTED_SUPPRESSED])
            .unwrap_or_else(|e| panic!("{e}; stderr:\n{err}"));
    let want_repeats = cerulion_core::testing::debug_lines_expected(8);
    assert_eq!(
        repeats.len(),
        want_repeats,
        "10 refusals − head − decade = 8 adopted-specific DEBUG repeats (0 when \
         `debug!` is compiled out); stderr:\n{err}"
    );
    let stills: Vec<&str> = lines
        .iter()
        .copied()
        .filter(|l| l.contains("WARN") && l.contains("adopted take STILL refused"))
        .collect();
    assert_eq!(
        stills.len(),
        1,
        "exactly one decade re-announcement; stderr:\n{err}"
    );
    // The generic line is a DEBUG-only marker too, so it gets the same
    // level-free twin: the zero below says no adopted-budget repeat FELL to
    // it, and `never_loud` says that if one ever does, a promotion to
    // warn!/info!/error! cannot hide behind a release build where the
    // DEBUG line does not exist at all.
    const GENERIC_SUPPRESSED: &str = "loaned take refused (suppressed repeat)";
    cerulion_core::testing::never_loud(&lines, GENERIC_SUPPRESSED)
        .unwrap_or_else(|e| panic!("{e}; stderr:\n{err}"));
    assert_eq!(
        lines
            .iter()
            .filter(|l| l.contains(GENERIC_SUPPRESSED))
            .count(),
        0,
        "no adopted-budget repeat may fall to the generic line; stderr:\n{err}"
    );
    for line in heads.iter().chain(repeats.iter()).chain(stills.iter()) {
        assert!(
            has_field(line, "kind", "adopted_budget_exhausted"),
            "kind rides every line: {line}"
        );
        assert!(
            has_field(line, "budget", "4"),
            "budget rides every line: {line}"
        );
        assert!(
            line.contains("CERULION_RMW_ADOPT_TAKE_BUDGET"),
            "the remedy rides every line: {line}"
        );
    }
    let still = stills[0];
    assert!(
        has_field(still, "total_failures", "10") && has_field(still, "suppressed_count", "8"),
        "the decade line carries the running total and the suppressed count: {still}"
    );
}

/// Memory safety: a C++ message whose
/// forgeable vector the CALLER filled BEFORE the first adopted take — a
/// genuine `std::vector<uint8_t>` buffer from `operator new`, 4 KiB of
/// it, through the real libstdc++ shim. The adopting take's reuse
/// pre-pass must release that buffer through the C++ pair (the shim's
/// `::operator delete`) and then forge the triplet over the slot: the
/// triplet aims at the registered SHM range with `capacity == size`, the
/// bytes are the published payload, and the app's free releases the
/// sample. A `free()`-based release cannot be told apart from
/// `operator delete` behaviourally on a libc-backed `operator new` (both
/// return the block), which is why the structural pin below is the
/// discriminating check; this arm proves the pre-pass runs on a caller-owned
/// buffer without a crash or a leak of the take.
#[test]
#[serial]
fn cpp_twin_releases_a_caller_filled_vector_before_the_first_adopted_take() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = cframe_ts(&format!("CppReuse{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "cppreuse", suffix);
        let stats = adopt_stats(subscription);

        let payload: Vec<u8> = (0..64u32).map(|i| (i * 7 + 3) as u8).collect();
        let mut src: Box<CppFrame> = Box::new(std::mem::zeroed());
        cframe_init(&mut *src as *mut _ as *mut c_void, 0);
        src.width = 320;
        rmw_cerulion_vector_u8_assign(
            src.data.0.as_mut_ptr() as *mut c_void,
            payload.as_ptr(),
            payload.len(),
        );
        assert_eq!(
            rmw_publish(
                publisher,
                &*src as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        cframe_fini(&mut *src as *mut _ as *mut c_void);

        // The caller's message arrives with its vector ALREADY holding a
        // real heap buffer (operator new, via the shim's assign) — the
        // shape rclcpp produces when it reuses an initialized message.
        let filler: Vec<u8> = vec![0xa5; 4096];
        let mut msg: Box<CppFrame> = Box::new(std::mem::zeroed());
        cframe_init(&mut *msg as *mut _ as *mut c_void, 0);
        let vslot = msg.data.0.as_mut_ptr() as *mut c_void;
        rmw_cerulion_vector_u8_assign(vslot, filler.as_ptr(), filler.len());
        let caller_begin = rmw_cerulion_vector_u8_data(vslot) as usize;
        assert_eq!(rmw_cerulion_vector_u8_capacity(vslot), filler.len());
        assert_ne!(caller_begin, 0, "the caller's vector really owns a buffer");

        let mut taken = false;
        assert_eq!(
            rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        assert!(taken, "the take adopts into the reused message");
        assert_eq!(msg.width, 320);
        // The triplet was forged OVER the released buffer: it now aims at
        // the registered SHM range, capacity == size, contents published.
        assert_eq!(rmw_cerulion_vector_u8_size(vslot), payload.len());
        assert_eq!(rmw_cerulion_vector_u8_capacity(vslot), payload.len());
        let begin = rmw_cerulion_vector_u8_data(vslot) as usize;
        assert_ne!(
            begin, caller_begin,
            "the forged triplet must not alias the caller's released buffer"
        );
        let segs = fake_segments();
        assert_eq!(segs.len(), 1, "exactly one range registered");
        assert_eq!(segs[0].start, begin, "vector begin IS the registered range");
        assert_eq!(segs[0].len, payload.len());
        let held = held_sample_range(&segs[0]);
        assert!(
            held.contains(&begin) && held.contains(&(begin + payload.len() - 1)),
            "the forged range lies inside the held sample"
        );
        let seen = std::slice::from_raw_parts(begin as *const u8, payload.len());
        assert_eq!(seen, payload.as_slice());
        assert_eq!(stat(&stats).3, 1, "one sample outstanding");

        // The app's free of the forged pointer releases the sample; the
        // slot is nulled before fini exactly as the app's own free leaves it.
        simulate_free(begin);
        msg.data.0 = [0usize; 3];
        assert_eq!(stat(&stats).3, 0, "released");
        cframe_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

/// The same memory-safety class — the structural pin
/// and discriminating check: the C++ twin's reuse pre-pass releases a
/// caller's vector buffer ONLY through the shim's `::operator delete`
/// entry (`rmw_cerulion_vector_pod_release`) and never through libc
/// `free`. Behaviour cannot distinguish the two on a libc-backed
/// `operator new` (both return the block), so the source is the pin: the
/// function body is walked on its own, comments outside it do not count,
/// and the shim's implementation is walked too.
#[test]
fn the_cpp_twin_releases_caller_vectors_through_operator_delete_never_libc_free() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let bridge = std::fs::read_to_string(root.join("src/type_bridge_cpp.rs"))
        .expect("read src/type_bridge_cpp.rs");
    let start = bridge
        .find("pub unsafe fn release_forgeable_members(")
        .expect("the C++ twin defines release_forgeable_members");
    let body = &bridge[start..];
    let end = body.find("\n    }\n").expect("fn body closes") + 7;
    let body = &body[..end];
    assert!(
        body.contains("rmw_cerulion_vector_pod_release("),
        "the C++ twin must release through the shim's operator-delete entry; body:\n{body}"
    );
    assert!(
        !body.contains("libc_free") && !body.contains("free("),
        "the C++ twin must never free a std::vector buffer with libc free; body:\n{body}"
    );
    // Memory safety: it must also release
    // only when the vector OWNS an allocation. A non-null `begin` does
    // not imply one — the forge itself writes `end_of_storage == begin`
    // for an empty entry, leaving a live SHM address with no allocation
    // behind it and no hook registration to release it. The behavioural
    // arm for this lives in `cpp_bridge_test.rs` but is gated on the
    // host's std::vector layout probe enabling forging, so the source is
    // the pin that runs everywhere — the same reason the operator-delete
    // property above is pinned here.
    assert!(
        body.contains("owns_storage()"),
        "the C++ twin must gate its release on the vector's ownership state; body:\n{body}"
    );
    assert!(
        !body.contains("t.begin != 0"),
        "and never on a non-null begin, which owns nothing by itself; body:\n{body}"
    );
    let shim = std::fs::read_to_string(root.join("shim/cppstring_shim.cpp"))
        .expect("read shim/cppstring_shim.cpp");
    let start = shim
        .find("void rmw_cerulion_vector_pod_release(")
        .expect("the shim defines rmw_cerulion_vector_pod_release");
    let impl_ = &shim[start..];
    let impl_ = &impl_[..impl_.find('}').expect("fn body closes") + 1];
    assert!(
        impl_.contains("::operator delete(begin)"),
        "the shim's release must be ::operator delete; got:\n{impl_}"
    );
}

/// A panic inside the forge window
/// (after `unflatten_forged` installed SHM pointers in the caller's
/// message, before any registration) must leave the message fini-safe —
/// the `ForgedMessageGuard` un-forges on unwind, `ffi_guard` reports
/// `RMW_RET_ERROR`, the sample releases, and nothing is registered. The
/// subscription is then WEDGED by the crate's poisoned-mutex policy
/// (loud errors, never torn state) while the process stays healthy.
#[test]
#[serial]
fn a_panic_in_the_forge_window_leaves_the_message_fini_safe() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("PanicWin{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "pwin", suffix);
        let stats = adopt_stats(subscription);

        publish_scan(publisher, &scan_oracle(16, 8, 9.0));
        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg as *mut _ as *mut c_void, 0);
        let fired_before = rmw_cerulion::test_seams::adopt_forge_window_panics_fired();
        let mut taken = false;
        let ret = {
            let _seam = rmw_cerulion::test_seams::AdoptForgeWindowPanicGuard::arm();
            rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            )
        };
        assert_eq!(
            rmw_cerulion::test_seams::adopt_forge_window_panics_fired(),
            fired_before + 1,
            "the seam must have fired (the error is attributable)"
        );
        assert_eq!(
            ret,
            rmw_cerulion::ffi::RMW_RET_ERROR,
            "ffi_guard converts the panic"
        );
        assert!(!taken);

        // THE guard oracle: the caller's forgeable headers are EMPTY — no
        // dangling SHM pointer survives the unwind (this is the assert a
        // variant that removes the guard fails, with a live SHM address here).
        assert!(
            msg.ranges.data.is_null(),
            "ranges must be un-forged on unwind (got {:p})",
            msg.ranges.data
        );
        assert!(
            msg.intensities.data.is_null(),
            "intensities must be un-forged on unwind (got {:p})",
            msg.intensities.data
        );
        assert!(
            fake_segments().is_empty(),
            "the panic fired before any registration — none may survive"
        );
        assert_eq!(stat(&stats).3, 0, "the sample's Arc unwound and released");
        // fini-safe FOR REAL: the fixture's fini frees only the heap string.
        scan_fini(&mut *msg as *mut _ as *mut c_void);

        // The subscription is wedged by the poisoned-lock policy — a loud
        // error, never silence, never torn state.
        publish_scan(publisher, &scan_oracle(4, 2, 10.0));
        let mut msg2: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg2 as *mut _ as *mut c_void, 0);
        let mut taken2 = false;
        assert_eq!(
            rmw_take(
                subscription,
                &mut *msg2 as *mut _ as *mut c_void,
                &mut taken2,
                std::ptr::null_mut(),
            ),
            rmw_cerulion::ffi::RMW_RET_ERROR,
            "the poisoned subscription fails loudly"
        );
        assert!(!taken2);
        scan_fini(&mut *msg2 as *mut _ as *mut c_void);

        // The PROCESS is fine: a fresh pair adopts normally.
        let ts2 = scan_ts(&format!("PanicWin2{suffix}"));
        let (node2, publisher2, subscription2) = setup_pair(ts2, "pwin2", suffix);
        let o = scan_oracle(8, 4, 11.0);
        publish_scan(publisher2, &o);
        let (taken3, msg3, _) = take_scan(subscription2);
        assert!(
            taken3,
            "a fresh subscription adopts normally after the panic"
        );
        assert_scan_values(&msg3, &o);
        free_adopted_scan(*msg3);

        teardown(node2, publisher2, subscription2);
        teardown(node, publisher, subscription);
    }
}

/// A subscription create that fails after
/// the adopt gate armed must not grow the process-global stats registry
/// (otherwise every failed armed create leaks one Arc forever). The
/// failure is forced deterministically: the service is pre-created with
/// `max_subscribers = 1` and the one slot occupied by a native subscriber,
/// so the rmw's own subscriber-port creation exhausts the slots and the
/// create returns null — well after `arm_for_create` ran.
#[test]
#[serial]
fn a_failed_create_after_arming_registers_no_stats() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("RegLeak{suffix}"));
        let topic_str = format!("/rmw_adopt/regleak/{suffix}");
        let node = setup_node(&format!("regleak_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");

        // Pre-create the service capped at ONE subscriber and occupy it.
        let mut cfg = rt.transport.default_topic_config();
        cfg.max_subscribers = Some(1);
        let native = rt
            .transport
            .create_subscriber_with_buffers(&topic_str, cfg, rt.transport.subscriber_buffer_size())
            .expect("native capped subscriber");

        let before = registered_stats_count();
        let topic = CString::new(topic_str.clone()).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let failed = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(failed.is_null(), "the slot-exhausted create must fail");
        assert_eq!(
            registered_stats_count(),
            before,
            "a FAILED create must not grow the stats registry"
        );

        // Anti-vacuity: free the slot — the same create now succeeds and
        // registers exactly one entry.
        drop(native);
        let ok = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!ok.is_null(), "with the slot free the create succeeds");
        assert_eq!(
            registered_stats_count(),
            before + 1,
            "a successful armed create registers exactly one entry"
        );
        assert_eq!(rmw_destroy_subscription(node, ok), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

// =====================================================================
// The report-window unwind and the effective budget
// =====================================================================

/// A panic after the registrations completed but before the
/// function returns (the report window — the guard now stands down LAST)
/// must still un-forge the caller's message. The registrations themselves
/// stay (their cookies hold the sample — a bounded, counted leak on a
/// subscription the panic just wedged), and the later frees still release.
#[test]
#[serial]
fn a_panic_in_the_report_window_still_unforges_the_message() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("PanicRep{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "prep", suffix);
        let stats = adopt_stats(subscription);

        publish_scan(publisher, &scan_oracle(16, 8, 13.0));
        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg as *mut _ as *mut c_void, 0);
        let fired_before = rmw_cerulion::test_seams::adopt_report_window_panics_fired();
        let mut taken = false;
        let ret = {
            let _seam = rmw_cerulion::test_seams::AdoptReportWindowPanicGuard::arm();
            rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            )
        };
        assert_eq!(
            rmw_cerulion::test_seams::adopt_report_window_panics_fired(),
            fired_before + 1,
            "the report-window seam must have fired"
        );
        assert_eq!(ret, rmw_cerulion::ffi::RMW_RET_ERROR);
        assert!(!taken);

        // THE oracle: the guard was still armed through the reporting
        // tail, so the unwind un-forged the message — no SHM pointer
        // survives behind the error return (reverting the disarm order
        // leaves live SHM addresses here).
        assert!(
            msg.ranges.data.is_null(),
            "ranges must be un-forged on a report-window unwind (got {:p})",
            msg.ranges.data
        );
        assert!(
            msg.intensities.data.is_null(),
            "intensities must be un-forged on a report-window unwind (got {:p})",
            msg.intensities.data
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);

        // The un-forge and the
        // registration withdrawal are ONE rollback, and this arm asserts
        // both halves of it. Leaving the registrations could be justified as
        // "the app's later frees still release" — but the un-forge two
        // assertions above is exactly what leaves the app with NO pointer to
        // any registered range, and the caller got RMW_RET_ERROR besides. The
        // frees that reasoning relies on are played BY THIS TEST, which no
        // application can do: nothing would ever release that sample,
        // and its SHM borrow slot would stay pinned for the process's life.
        // The rollback is complete at the point of unwind.
        assert!(
            fake_segments().is_empty(),
            "the registrations must be WITHDRAWN on the unwind — the caller cannot free \
             ranges it has no pointer to (got {} still registered)",
            fake_segments().len()
        );
        assert_eq!(
            stat(&stats).3,
            0,
            "and the sample is RELEASED, not pinned behind cookies nobody can retire"
        );

        teardown(node, publisher, subscription);
    }
}

/// CHILD: against a pre-existing service at
/// the iceoryx2 default borrow capacity (2), retain TWO adopted samples and
/// take a third — the refusal must NAME the effective budget 2, never the
/// requested floor 16. Driven as a child so the `warn!` is captured (the
/// rmw runtime installs its own subscriber at first init).
#[test]
#[ignore]
fn child_pre_existing_service_refusal_names_the_effective_budget() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("EffBudRef{suffix}"));
        let topic_str = format!("/rmw_adopt/effbudref/{suffix}");
        let node = setup_node(&format!("effbudref_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let native = rt
            .transport
            .create_subscriber_with_buffers(
                &topic_str,
                rt.transport.default_topic_config(),
                rt.transport.subscriber_buffer_size(),
            )
            .expect("native default subscriber");
        let topic = CString::new(topic_str.clone()).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(
            !subscription.is_null(),
            "open of the pre-existing service succeeds"
        );
        let state = sub_data(subscription).adopt.as_ref().expect("armed");
        assert_eq!(state.effective_budget, 2);
        assert_eq!(state.budget, RMW_ADOPT_TAKE_BORROW_BUDGET);
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(
            !publisher.is_null(),
            "a publisher on the pre-existing service"
        );
        let stats = adopt_stats(subscription);
        for i in 0..3 {
            publish_scan(publisher, &scan_oracle(8, 4, i as f32));
        }
        let mut held = Vec::new();
        for i in 0..2 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(
                taken,
                "take {i} within the EFFECTIVE budget of 2 must serve"
            );
            held.push(msg);
        }
        let (taken, msg3, _) = take_scan(subscription);
        assert!(
            !taken,
            "the third take is past the effective budget: refused"
        );
        drop(msg3);
        // A SECOND refusal, so the regime has something SUPPRESSED: the
        // latch stays silent on recovery when only the loud head fired
        // (the deliberate lone-failure silent re-arm), so a single refusal
        // could never exercise the recovery wording at all.
        let (taken, msg4, _) = take_scan(subscription);
        assert!(!taken, "still past the budget: refused again");
        drop(msg4);
        assert_eq!(stat(&stats).5, 2, "both budget refusals counted");
        for msg in held {
            free_adopted_scan(*msg);
        }
        // The recovery after an
        // adopted-budget refusal. With the samples freed the next take
        // serves, closing the regime — and the line it emits must speak
        // the adopted vocabulary, not the loaned one.
        publish_scan(publisher, &scan_oracle(8, 4, 99.0));
        let (taken, recovered_msg, _) = take_scan(subscription);
        assert!(taken, "with the budget freed the next adopted take serves");
        free_adopted_scan(*recovered_msg);

        // A second regime, closed by a take
        // that serves by COPY. The label must follow the outcome, not the
        // path: this take pays a full copy, so reporting it `adopted`
        // would tell an operator the zero-copy path was working on a
        // frame that was copied.
        for i in 0..4 {
            publish_scan(publisher, &scan_oracle(8, 4, 200.0 + i as f32));
        }
        let mut held2 = Vec::new();
        for _ in 0..2 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(taken, "within the effective budget again");
            held2.push(msg);
        }
        for _ in 0..2 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(!taken, "past the budget again: refused");
            drop(msg);
        }
        for msg in held2 {
            free_adopted_scan(*msg);
        }
        // The next take's registrations fail, so it falls back to a copy.
        // `fail_register_at` is an index into the RUNNING register-call
        // count, not a per-take one, so it has to be aimed at the next
        // call rather than at 1 — earlier takes have already consumed
        // several.
        let next_register_call = fake().lock().expect("fake").register_calls + 1;
        arm_register_failure_at(next_register_call);
        publish_scan(publisher, &scan_oracle(8, 4, 250.0));
        let (taken, mut copied_msg, _) = take_scan(subscription);
        assert!(taken, "the copy fallback still serves the frame");
        assert_eq!(
            stat(&stats).4,
            1,
            "and it really fell back — otherwise the recovery below would be an \
             adopted take and the label pin would be testing nothing"
        );
        // Served by COPY, so the sequences are real heap allocations the
        // ordinary fini owns — NOT forged SHM addresses.
        scan_fini(&mut *copied_msg as *mut _ as *mut c_void);

        // A third regime, closed by an
        // all-empty frame. That take pays no copy — it copies nothing and
        // holds nothing — so `adopted_takes` counts it, `fallbacks` does
        // not, and the label must agree: `served=adopted`. Saying
        // `copied` there while leaving both counters as they are lets the
        // summary read `adopted == takes` while the diagnostic says
        // a copy happened.
        //
        // The QUEUE ORDER is load-bearing: a
        // refused take does NOT consume its frame, so any non-empty frame
        // still queued is what the recovering take serves — and it adopts
        // normally, which would make this arm pass under the very variant it
        // exists to catch. The empty frame is published LAST, after the
        // two the takes will hold, so it is the one still queued when the
        // budget frees up.
        // DRAIN first: a refused take does not consume its frame, so the
        // earlier regimes leave frames queued and the "empty frame is
        // last" plan only holds on an empty queue. With nothing held, a
        // take that returns false means the queue is empty — a refusal
        // cannot happen at zero outstanding.
        loop {
            let (taken, msg, _) = take_scan(subscription);
            if !taken {
                drop(msg);
                break;
            }
            free_adopted_scan(*msg);
        }
        publish_scan(publisher, &scan_oracle(8, 4, 300.0));
        publish_scan(publisher, &scan_oracle(8, 4, 301.0));
        publish_scan(publisher, &scan_oracle(0, 0, 400.0));
        let mut held3 = Vec::new();
        for _ in 0..2 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(taken, "the two non-empty frames are taken and held");
            held3.push(msg);
        }
        for _ in 0..2 {
            let (taken, msg, _) = take_scan(subscription);
            assert!(
                !taken,
                "past the budget a third time — the EMPTY frame stays queued"
            );
            drop(msg);
        }
        for msg in held3 {
            free_adopted_scan(*msg);
        }
        let fallbacks_before = stat(&stats).4;
        let adopted_before = stat(&stats).1;
        let (taken, mut empty_msg, _) = take_scan(subscription);
        assert!(taken, "the empty frame serves and closes the regime");
        assert!(
            empty_msg.ranges.size == 0 && empty_msg.intensities.size == 0,
            "and it really IS the empty frame — otherwise this arm tests a normal adoption"
        );
        assert_eq!(
            stat(&stats).4,
            fallbacks_before,
            "an all-empty frame pays no copy, so no fallback is counted — the label must \
             say the same thing"
        );
        assert_eq!(
            stat(&stats).1,
            adopted_before + 1,
            "...and it IS counted as an adopted take, which is the other half of the rule"
        );
        // The pointers are not erased
        // before `fini`. Overwriting them removed the only check that an
        // empty forged entry is destructor-safe — a bridge that left a
        // shared-memory pointer in one, or failed to un-forge it, was
        // hidden and the arm still passed. Both bridges write the EMPTY
        // header for an empty entry, so the state is ASSERTED and `fini`
        // is handed the real headers.
        assert_destructor_safe_empty(&empty_msg);
        scan_fini(&mut *empty_msg as *mut _ as *mut c_void);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        drop(native);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The pre-existing-service arm below pins
/// only the STORED effective budget; this one exercises the refusal PATH
/// and asserts the emitted diagnostic names `budget=2` — the effective
/// capacity — as a whole token, and never the requested `budget=16`. A
/// variant that reports the requested floor fails here.
#[test]
#[serial]
fn a_pre_existing_smaller_service_refusal_names_the_effective_budget_not_the_request() {
    let (status, err) = run_child(
        "child_pre_existing_service_refusal_names_the_effective_budget",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    // The child drives two refusals, and a
    // `.find()` would check only the first — a second diagnostic with the
    // WRONG budget, or with the requested floor back in it, would be invisible.
    // EVERY emitted refusal is checked, and the count is pinned to what
    // the latch's policy actually produces at this log level: one loud
    // head, the repeat suppressed to `debug!` and filtered out by the
    // child's `RUST_LOG=rmw_cerulion=info`.
    let refusals: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("adopted take refused"))
        .collect();
    assert!(
        !refusals.is_empty(),
        "at least the loud head must be emitted; stderr:\n{err}"
    );
    // The child opens TWO regimes (the second closed by a copy-served
    // take), and the latch re-arms on recovery — so exactly one loud head
    // PER REGIME, never one per refusal. The suppressed repeat inside each
    // regime is `debug!` and filtered out by the child's
    // `RUST_LOG=rmw_cerulion=info`.
    let loud: Vec<&&str> = refusals.iter().filter(|l| l.contains("WARN")).collect();
    assert_eq!(
        loud.len(),
        3,
        "one LOUD head per regime, not per refusal; got {refusals:#?}"
    );
    for refusal in &refusals {
        assert!(
            has_field(refusal, "kind", "adopted_budget_exhausted"),
            "every refusal names its kind: {refusal}"
        );
        assert!(
            has_field(refusal, "budget", "2"),
            "every refusal must name the EFFECTIVE budget 2: {refusal}"
        );
        assert!(
            !has_field(refusal, "budget", "16"),
            "no refusal may report the requested floor 16: {refusal}"
        );
        // This field was renamed: one `outstanding_loans=`
        // could not say WHICH of the two borrow-holding paths held the
        // budget, and the adopt line was advising a free for borrows that
        // might all be loans. This child holds two ADOPTED messages and no
        // loan, so the pair is the discriminating assertion — and it is the
        // `AdoptedOnly` side of the classification, whose `LoanedOnly` twin
        // is pinned by `a_refusal_held_by_loans_names_the_loans_and_not_the_adopted_messages`.
        assert!(
            has_field(refusal, "adopted_outstanding", "2"),
            "every refusal names how many ADOPTED messages are held: {refusal}"
        );
        assert!(
            has_field(refusal, "loaned_outstanding", "0"),
            "...and how many LOANS are, which is what makes the remedy right: {refusal}"
        );
        assert!(
            has_field(refusal, "kind", "adopted_budget_exhausted"),
            "adopted messages alone hold the budget here, so the kind operators already \
             grep for is the one that must appear: {refusal}"
        );
        assert!(
            refusal.contains("CERULION_RMW_ADOPT_TAKE_BUDGET")
                || refusal.contains("free adopted messages"),
            "every refusal carries a remedy: {refusal}"
        );
    }
    // The recovery line must not tell an
    // operator who saw `kind=adopted_budget_exhausted` that the LOANED
    // take recovered — that points them at a different API (and at
    // `rmw_return_loaned_message_from_subscription`, which is not how an
    // adopted message is released). Both take paths share one
    // subscription latch, so the reporter is TOLD which take served.
    let recovery = err
        .lines()
        .find(|l| l.contains("INFO") && l.contains("recovered"))
        .unwrap_or_else(|| panic!("recovery line missing; stderr:\n{err}"));
    assert!(
        recovery.contains("adopted take recovered"),
        "the recovery must speak the ADOPTED vocabulary: {recovery}"
    );
    assert!(
        has_field(recovery, "served", "adopted"),
        "and carry it as a field an operator can grep: {recovery}"
    );
    assert!(
        !recovery.contains("loaned take recovered"),
        "never the loaned wording: {recovery}"
    );
    // The second recovery closed a regime
    // with a COPY-served take, and must say so. Labelling it `adopted`
    // reports the zero-copy path as working on a frame that was copied.
    let recoveries: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("INFO") && l.contains("recovered"))
        .collect();
    assert!(
        recoveries.len() >= 2,
        "at least the adopted and copy-served recoveries; stderr:\n{err}"
    );
    let copied = recoveries[1];
    assert!(
        has_field(copied, "served", "copied"),
        "a copy-served take must be labelled copied: {copied}"
    );
    assert!(
        copied.contains("served by COPY"),
        "and say so in words an operator reads: {copied}"
    );
    assert!(
        !has_field(copied, "served", "adopted"),
        "never adopted: {copied}"
    );
    // The third recovery was closed by an
    // ALL-EMPTY frame, which pays no copy — `adopted_takes` counts it and
    // `fallbacks` does not — so its label must be `adopted`. The counters
    // and the label answer one question; this is the arm that would catch
    // them disagreeing again.
    assert_eq!(
        recoveries.len(),
        3,
        "three regimes were opened and closed; stderr:\n{err}"
    );
    let empty = recoveries[2];
    assert!(
        has_field(empty, "served", "adopted"),
        "an all-empty frame pays no copy, so it is an adopted take: {empty}"
    );
    assert!(
        !has_field(empty, "served", "copied"),
        "never copied — nothing was copied: {empty}"
    );
}

/// Against a pre-existing service created at
/// the iceoryx2 default borrow capacity, the adopt state carries the
/// service's EFFECTIVE budget (what `ExceedsMaxBorrows` really binds at)
/// beside the requested create floor — the refusal diagnostics report the
/// effective one, so the operator's remedy math is right.
#[test]
#[serial]
fn a_pre_existing_smaller_service_reports_its_effective_budget() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("EffBud{suffix}"));
        let topic_str = format!("/rmw_adopt/effbud/{suffix}");
        let node = setup_node(&format!("effbud_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");

        // Pre-create the service with the DEFAULT config (no borrow floor —
        // the iceoryx2 default of 2); keep the native subscriber alive so
        // the service exists when the rmw opens it.
        let native = rt
            .transport
            .create_subscriber_with_buffers(
                &topic_str,
                rt.transport.default_topic_config(),
                rt.transport.subscriber_buffer_size(),
            )
            .expect("native default subscriber");

        let topic = CString::new(topic_str.clone()).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let sub = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!sub.is_null(), "open of the pre-existing service succeeds");
        let state = sub_data(sub).adopt.as_ref().expect("armed");
        assert_eq!(
            state.budget, RMW_ADOPT_TAKE_BORROW_BUDGET,
            "the REQUESTED floor is unchanged"
        );
        assert_eq!(
            state.effective_budget, 2,
            "the EFFECTIVE budget is the pre-existing service's real capacity \
             (the iceoryx2 default), not the create floor"
        );

        assert_eq!(rmw_destroy_subscription(node, sub), RMW_RET_OK);
        drop(native);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// CHILD: the over-retention take must say NOTHING about the
/// below-floor regime. Drives, in one subscription: two below-floor frames
/// (regime OPEN, two samples retained) → one take at the retention limit
/// (the over-retention COPY arm) → a free → one more below-floor frame.
#[test]
#[ignore]
fn child_over_retention_take_does_not_close_the_below_floor_regime() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("FloorRegime{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "floorregime", suffix);
        let stats = adopt_stats(subscription);
        let o = scan_oracle(16, 8, 5.0);

        // 1 + 2: below-floor frames OPEN the regime (loud head, then a
        // suppressed repeat) and RETAIN two samples — `intensities` is well
        // placed, so each take still adopts.
        let mut held = Vec::new();
        for round in 0..2 {
            publish_scan_below_floor(publisher, ts, &o);
            let (taken, msg, _) = take_scan(subscription);
            assert!(
                taken,
                "round {round}: a below-floor entry is served, never refused"
            );
            assert!(
                !msg.intensities.data.is_null(),
                "round {round}: `intensities` must still FORGE, or the take degrades to the \
                 whole-message copy arm and never opens the regime this test needs"
            );
            held.push(msg);
        }
        assert_eq!(
            stat(&stats).1,
            2,
            "both below-floor takes ADOPTED (only `ranges` was copied)"
        );
        assert_eq!(
            stat(&stats).3,
            2,
            "…and both samples are retained — the take below is AT the limit"
        );

        // 3: at the retention limit ⇒ the OVER-RETENTION copy arm. It decodes
        // with the plain `unflatten` and never looks at any entry's
        // placement, so its `below_floor == 0` is NOT MEASURED — reporting it
        // as a clean forge would close the regime opened above.
        let fallbacks_before = stat(&stats).4;
        publish_scan(publisher, &o);
        let (taken, mut msg3, _) = take_scan(subscription);
        assert!(
            taken,
            "over the retention budget the frame is still DELIVERED"
        );
        assert_eq!(
            stat(&stats).4,
            fallbacks_before + 1,
            "…by copy: it counted as a fallback"
        );
        assert_eq!(stat(&stats).3, 2, "…and retained nothing new");
        scan_fini(&mut *msg3 as *mut _ as *mut c_void);

        // Free one adopted message so the next take is UNDER the limit and
        // reaches the forge reporters at all.
        let first = held.remove(0);
        free_below_floor_scan(*first);
        assert_eq!(stat(&stats).3, 1, "one sample released");

        // 4: below-floor again. The regime opened at step 1 is still open, so
        // this is a SUPPRESSED repeat — a fresh loud head here means step 3
        // re-armed the latch.
        publish_scan_below_floor(publisher, ts, &o);
        let (taken, msg4, _) = take_scan(subscription);
        assert!(taken, "the fourth take is served");
        free_below_floor_scan(*msg4);
        for msg in held {
            free_below_floor_scan(*msg);
        }
        teardown(node, publisher, subscription);
        // The parent pipes STDERR and nulls stdout, so the completion
        // marker must go to stderr or it can never be observed.
        eprintln!("CHILD_FLOOR_REGIME_OK");
    }
}

/// A take
/// served by the OVER-RETENTION copy arm must not report a forge verdict it
/// never measured.
///
/// `ForgeOutcome::default()` carries `below_floor == 0`, so a copy arm that
/// reports it falls through into `report_forge_clean` — `observe_success` on the
/// below-floor latch, which CLOSES an open regime, re-arms it, and prints a
/// recovery claiming "frames place their sequences at or above the data
/// floor again". That is a statement about the PRODUCER's wire layout, made
/// by a take that decoded with the plain copying `unflatten` and never
/// looked at a single entry's placement. On a topic whose publisher really
/// does place a sequence below the floor, every over-retention take emits
/// that false recovery and re-arms, so the next genuinely-forging take
/// prints a fresh LOUD head instead of the suppressed repeat it is — a
/// flood in exactly the regime the latch exists to suppress, alternating at
/// frame rate for as long as the app sits at its retention limit.
///
/// The oracle is the LATCH STATE, which only the log can witness: the
/// unconditional counter moves identically either way (a recovery never
/// touches `total_failures`), which is why this is a child arm and not an
/// in-process counter assertion like its siblings. At `RUST_LOG=info` the
/// loud head and the recovery are visible and the suppressed repeat is not,
/// so both halves of the contract are readable: ONE head for the whole run,
/// and NO recovery.
#[test]
#[serial]
fn an_over_retention_take_does_not_close_the_below_floor_regime() {
    let (status, err) = run_child(
        "child_over_retention_take_does_not_close_the_below_floor_regime",
        &[(ADOPT_TAKE_ENV, "1"), (ADOPT_TAKE_BUDGET_ENV, "2")],
    );
    assert!(status.success(), "child failed:\n{err}");
    assert!(
        err.contains("CHILD_FLOOR_REGIME_OK"),
        "the child must have reached its end — the marker goes to stderr because the \
         parent nulls the child's stdout, and an `|| !err.is_empty()` escape here would \
         make this assertion vacuous:\n{err}"
    );
    let heads: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("WARN") && l.contains("forged loaned take fell back to COPYING"))
        .collect();
    assert_eq!(
        heads.len(),
        1,
        "the below-floor regime opens ONCE and stays open: a second loud head means the \
         over-retention take re-armed the latch. heads={heads:#?}\nstderr:\n{err}"
    );
    let recoveries: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("forged loaned take recovered"))
        .collect();
    assert!(
        recoveries.is_empty(),
        "no take in this run measured a well-placed frame through the FORGE, so nothing may \
         claim the producer recovered. recoveries={recoveries:#?}\nstderr:\n{err}"
    );
}

/// A decided property: the adopted take is
/// DETERMINISTIC — two runs of the same sequence produce the same counters
/// and register the same byte ranges.
///
/// Principle #7 is the reason this arm exists at all: the adopted path adds
/// a second way for a take to end (adopt vs. copy vs. refuse) and a piece of
/// per-subscription state (`outstanding`) that survives a take, so "the same
/// stimulus twice" is no longer obviously the same execution. If a
/// borrow-budget race, an allocator address, or a queue-depth accident could
/// steer one run down the copy arm and the next down the forge arm, the
/// counters would say so — and the bench numbers this path exists to earn
/// would be sampling two different implementations.
///
/// The ranges are compared as offsets RELATIVE TO THE FRAME BODY — the
/// sample's payload base plus the 32-byte `WireHeader`, i.e. the base the
/// offset table itself is written against (`take_adopted` decodes
/// `&owned.payload()[WireHeader::SIZE..]`). The absolute SHM address
/// legitimately differs between runs (a different pool slot); the
/// body-relative offset is what the wire format fixes, and is therefore what
/// determinism means here.
///
/// NOT a self-compare: run 1 is anchored to a HAND oracle computed from the
/// wire layout — `data_floor` is the fixed section (two `f32`s) plus the
/// offset table (three variable members × 8 bytes) = 32, `ranges` is the
/// first variable region at the floor and `intensities` follows it, each
/// `len = count × 4` — and run 2 is compared to the same oracle. Two runs
/// agreeing with each other proves nothing if both drifted together.
#[test]
#[serial]
fn two_runs_of_the_adopted_take_match_the_hand_oracle() {
    const FIXED_SECTION: usize = 2 * 4; // angle_min, angle_max
    const OFFSET_TABLE: usize = 3 * 8; // ranges, intensities, frame_id
    const DATA_FLOOR: usize = FIXED_SECTION + OFFSET_TABLE;
    const N_RANGES: usize = 16;
    const N_INTENSITIES: usize = 8;
    const TAKES: usize = 3;

    // Per adopted take, in registration order: the two forgeable members,
    // laid out back to back from the data floor.
    let per_take: [(usize, usize); 2] = [
        (DATA_FLOOR, N_RANGES * 4),
        (DATA_FLOOR + N_RANGES * 4, N_INTENSITIES * 4),
    ];
    let want_ranges: Vec<(usize, usize)> = (0..TAKES).flat_map(|_| per_take).collect();
    // Cross-check the hand arithmetic against the ONE definition of the floor
    // (`WireLayout::data_floor` = fixed section + offset table), so a layout
    // change fails here rather than being blessed into the oracle above.
    assert_eq!(
        DATA_FLOOR,
        FIXED_SECTION + OFFSET_TABLE,
        "the data floor is the fixed section plus one 8-byte offset-table entry per \
         variable member (ranges, intensities, frame_id)"
    );
    // takes, adopted, releases, outstanding, fallbacks, budget_refusals —
    // three takes, all adopted, nothing freed yet, nothing refused.
    let want_stats = (TAKES as u64, TAKES as u64, 0, TAKES as u64, 0, 0);

    let mut runs = Vec::new();
    for run in 0..2 {
        reset_fake();
        let _hook = TestHookGuard::install(fake_hook_api());
        let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
        let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
        unsafe {
            let suffix = unique_suffix();
            let ts = scan_ts(&format!("Det{run}x{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, "det", suffix);
            let stats = adopt_stats(subscription);
            let o = scan_oracle(N_RANGES, N_INTENSITIES, 3.0);

            let mut held = Vec::new();
            for take in 0..TAKES {
                publish_scan(publisher, &o);
                let (taken, msg, _) = take_scan(subscription);
                assert!(taken, "run {run} take {take}");
                assert_scan_values(&msg, &o);
                held.push(msg);
            }

            // Every registration, as (offset, len) from its own frame's BODY
            // base — the absolute address is a pool slot and is allowed to
            // differ. `AdoptedSample::payload()` is the whole frame, so the
            // body starts one `WireHeader` in; that is the base the offset
            // table encodes against, and the base the hand oracle is computed
            // in.
            let ranges: Vec<(usize, usize)> = fake_segments()
                .iter()
                .map(|seg| {
                    let payload = held_sample_range(seg);
                    assert!(
                        seg.start >= payload.start && seg.start + seg.len <= payload.end,
                        "run {run}: every registered range lies inside its held sample"
                    );
                    let body = payload.start + cerulion_core::wire::WireHeader::SIZE;
                    assert!(
                        seg.start >= body,
                        "run {run}: a forged range must lie in the BODY, never in the header"
                    );
                    (seg.start - body, seg.len)
                })
                .collect();
            runs.push((stat(&stats), ranges));

            for msg in held {
                free_adopted_scan(*msg);
            }
            teardown(node, publisher, subscription);
        }
    }

    for (run, (stats, ranges)) in runs.iter().enumerate() {
        assert_eq!(
            *stats, want_stats,
            "run {run}: counters must match the hand oracle \
             (takes, adopted, releases, outstanding, fallbacks, refusals)"
        );
        assert_eq!(
            *ranges, want_ranges,
            "run {run}: the registered ranges must match the hand oracle, as offsets from \
             each sample's own payload base"
        );
    }
    assert_eq!(
        runs[0], runs[1],
        "and the two runs must agree with each other, not only with the oracle"
    );
}

/// CHILD: a subscription that OUTLIVES the final `rmw_shutdown`
/// must stop adopting. Own process, because the clear only runs for the FINAL
/// context and sibling tests in this binary leak contexts.
#[test]
#[ignore]
fn child_take_after_the_callback_is_cleared_does_not_adopt() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("PostShut{suffix}"));
        let (context, node) = setup_node_with_context(&format!("postshut_{suffix}"));
        let topic = CString::new(format!("/rmw_adopt/postshut/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let stats = adopt_stats(subscription);
        let o = scan_oracle(8, 4, 9.0);

        // Baseline: while the callback is installed the take ADOPTS.
        publish_scan(publisher, &o);
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken, "the pre-shutdown take is served");
        assert_scan_values(&msg, &o);
        assert_eq!(
            stat(&stats),
            (1, 1, 0, 1, 0, 0),
            "precondition: the pre-shutdown take adopted and retained"
        );
        let registered_before = fake_segments().len();
        assert!(
            registered_before > 0,
            "precondition: ranges were registered"
        );
        free_adopted_scan(*msg);
        // `releases` counts callback firings — one per REGISTERED RANGE freed,
        // and this type forges two members, so it is derived from the
        // registration count rather than written as a literal.
        let before_shutdown = stat(&stats);
        assert_eq!(
            before_shutdown,
            (1, 1, registered_before as u64, 0, 0, 0),
            "precondition: one adopted take, freed: \
             (takes, adopted, releases, outstanding, fallbacks, refusals)"
        );

        // The FINAL shutdown clears the callback — WITHOUT destroying the
        // subscription, which rmw and rcl do not require and an executor
        // still spinning does not do.
        assert_eq!(rmw_shutdown(context), RMW_RET_OK);
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "precondition: the final shutdown cleared the callback"
        );

        // THE PIN: the still-alive subscription must serve by COPY. Adopting
        // here registers ranges the hook can never call back for, so the app's
        // free leaks the cookie and pins one borrow + pool slot per take for
        // the life of the process.
        let registered_after_clear = fake_segments().len();
        publish_scan(publisher, &o);
        let (taken, mut msg2, _) = take_scan(subscription);
        assert!(taken, "the post-shutdown take is still SERVED — by copy");
        assert_scan_values(&msg2, &o);
        assert_eq!(
            fake_segments().len(),
            registered_after_clear,
            "no new registration may be made once nothing can release it"
        );
        // Every adopt counter is FROZEN: the degraded take does not fall back
        // WITHIN the adoption branch, it leaves that branch entirely and is
        // served by the copying take, which owns no adopt counters at all.
        // `fallbacks` is documented as adoption-branch takes served by copy,
        // and this take never entered it. So the counters are not the record
        // of the degrade — the once-per-process warn the parent pins is.
        assert_eq!(
            stat(&stats),
            before_shutdown,
            "a take after the callback was cleared leaves the adoption branch, so every \
             adopt counter is unchanged: (takes, adopted, releases, outstanding, fallbacks, \
             refusals)"
        );
        // Both members are real heap copies now, so the ordinary fini owns them.
        scan_fini(&mut *msg2 as *mut _ as *mut c_void);
        // The parent pipes STDERR and nulls stdout, so the completion
        // marker must go to stderr or it can never be observed.
        eprintln!("CHILD_POST_SHUTDOWN_OK");
    }
}

/// The adoption branch is gated on the
/// release callback being installed NOW, not on the grant taken at create.
///
/// `AdoptTakeGrant` proves the heap hook was live when the SUBSCRIPTION was
/// created. `context_ended` clears the process-global release callback on the
/// FINAL context's `rmw_shutdown`, and neither rmw nor rcl guarantees every
/// subscription is destroyed first — an executor still spinning, or node
/// destructors running after `rclcpp::shutdown()`, both take afterwards. Such
/// a take must not forge and register ranges with a hook that has no callback
/// to call: the app's later `free()` takes the hook's documented no-callback
/// path (`release_without_callback`), which LEAKS the range rather than
/// wrongly freeing it, so the `Arc<AdoptedSample>` cookie is never reclaimed,
/// `outstanding` never drains, and one shared-memory borrow plus its
/// publisher-pool slot is pinned for the life of the process — per take, until
/// the retention budget is permanently gone.
///
/// The child owns the counter and registry oracles (it panics, and the parent
/// requires a clean exit); the parent additionally pins the operator-facing
/// half, which only the log can witness: the degrade is announced EXACTLY once
/// per process, not once per take.
#[test]
#[serial]
fn a_take_after_the_release_callback_is_cleared_serves_by_copy() {
    let (status, err) = run_child(
        "child_take_after_the_callback_is_cleared_does_not_adopt",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    assert!(
        err.contains("CHILD_POST_SHUTDOWN_OK"),
        "the child must have reached its end:\n{err}"
    );
    let degrades: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("WARN") && l.contains("adopt-take degraded to the copying take"))
        .collect();
    assert_eq!(
        degrades.len(),
        1,
        "the degrade is announced once per PROCESS — a per-take line would flood a \
         shutting-down robot. degrades={degrades:#?}\nstderr:\n{err}"
    );
}

/// A shutdown that lands between the
/// dispatch's callback check and the registrations must not leave those
/// registrations behind.
///
/// Gating the adoption branch on the callback being installed leaves a
/// check-then-register window, and closing it does not need
/// "a mutex on the hot path". This is the
/// arm that pins the mechanism: the take re-reads the callback EPOCH after its
/// registrations land and, if it changed, withdraws them and serves by copy
/// through the rollback the registration-failure arm already owns. No lock,
/// and the rollback runs only on the rare loss.
///
/// The race is driven DETERMINISTICALLY by a seam that clears the callback
/// inside the forge window — the exact interleaving a concurrent
/// `rmw_shutdown` produces. A thread-racing test would be flaky and would
/// prove the property only on the runs that happened to lose.
///
/// `unregister_segment` is the caller-side half of the hook contract and needs
/// no callback, and the app cannot have freed anything yet because the take
/// has not returned its message — which is why the withdrawal is sound
/// exactly here.
#[test]
#[serial]
fn a_shutdown_racing_the_registrations_withdraws_them_and_serves_by_copy() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ShutRace{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "shutrace", suffix);
        let stats = adopt_stats(subscription);
        let o = scan_oracle(8, 4, 13.0);

        let fired_before = rmw_cerulion::test_seams::adopt_forge_window_clears_fired();
        publish_scan(publisher, &o);
        let (taken, mut msg, _) = {
            let _race = rmw_cerulion::test_seams::AdoptForgeWindowClearGuard::arm();
            take_scan(subscription)
        };
        assert_eq!(
            rmw_cerulion::test_seams::adopt_forge_window_clears_fired(),
            fired_before + 1,
            "the race seam must have fired — otherwise this arm proves nothing"
        );

        // The frame is still SERVED, by copy, with the right bytes.
        assert!(
            taken,
            "losing the race degrades the take, it never drops the frame"
        );
        assert_scan_values(&msg, &o);

        // THE PIN: nothing is left registered with a hook that can no longer
        // release it, and the sample is back.
        assert!(
            fake_segments().is_empty(),
            "the registrations made inside the race must be WITHDRAWN (got {} still \
             registered) — leaving them is the permanent leak: the app has no pointer \
             that can release them and the SHM borrow is pinned for the life of the \
             process",
            fake_segments().len()
        );
        assert_eq!(
            stat(&stats),
            (1, 0, 0, 0, 1, 0),
            "the take counted as a FALLBACK, not an adoption, and retained nothing: \
             (takes, adopted, releases, outstanding, fallbacks, refusals)"
        );

        // Both members are real heap copies now, so the ordinary fini owns
        // them — the same ending the registration-failure fallback has.
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

/// A concurrent `rmw_create_subscription`
/// must be INVISIBLE to an in-flight adopted take.
///
/// `AdoptTakeGrant::try_acquire` runs at every subscription create and
/// re-installs the SAME callback pointer. Bumping the epoch
/// unconditionally there makes that no-op re-install look exactly like a
/// shutdown to a take already past its dispatch check: the post-registration
/// re-read differs, and the take withdraws perfectly good registrations and
/// serves a copy. Node bring-up creates subscriptions while other
/// subscriptions are taking, so this is not a corner — it is a zero-copy path
/// that silently degrades whenever the graph changes.
///
/// The epoch moves only on a genuine transition (none → installed), so a
/// re-install cannot be mistaken for a teardown. The seam drives the
/// interleaving deterministically inside the forge window; the POSITIVE
/// CONTROL that a real clear still aborts the take is its sibling,
/// `a_shutdown_racing_the_registrations_withdraws_them_and_serves_by_copy`,
/// which must keep passing — without it this arm could be satisfied by an
/// epoch check that never fires at all.
#[test]
#[serial]
fn a_concurrent_create_during_a_take_does_not_abort_the_adoption() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Reinstall{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "reinstall", suffix);
        let stats = adopt_stats(subscription);
        let o = scan_oracle(8, 4, 17.0);

        let fired_before = rmw_cerulion::test_seams::adopt_forge_window_reinstalls_fired();
        // The vacuous-seam class:
        // the seam's own counter proves only that the seam RAN, not that it
        // re-installed anything. With `reinstall_release_callback_for_seam`
        // stubbed to a no-op the arm would still pass, because the callback
        // installed at subscription-create time already permits adoption — so
        // the thing under test would never be driven. The fake's
        // `set_release_callback` count is what proves a real re-install
        // happened INSIDE the take.
        let set_cb_before = set_cb_calls();
        publish_scan(publisher, &o);
        let (taken, msg, _) = {
            let _reinstall = rmw_cerulion::test_seams::AdoptForgeWindowReinstallGuard::arm();
            take_scan(subscription)
        };
        assert_eq!(
            rmw_cerulion::test_seams::adopt_forge_window_reinstalls_fired(),
            fired_before + 1,
            "the re-install seam must have fired — otherwise this arm proves nothing"
        );
        assert!(
            set_cb_calls() > set_cb_before,
            "…and it must have really RE-INSTALLED the callback: the seam firing is not the \
             same claim, and a no-op seam leaves this arm proving only that a take with an \
             already-installed callback adopts (got {} calls, unchanged from {set_cb_before})",
            set_cb_calls()
        );

        assert!(taken, "the take is served");
        assert_scan_values(&msg, &o);
        // THE PIN: it ADOPTED. A rollback here would still serve the right
        // bytes, so the values alone cannot see this — the counters and the
        // live registrations are what distinguish zero-copy from a copy.
        assert_eq!(
            stat(&stats),
            (1, 1, 0, 1, 0, 0),
            "a no-op re-install must not abort the adoption: \
             (takes, adopted, releases, outstanding, fallbacks, refusals)"
        );
        assert!(
            !fake_segments().is_empty(),
            "…and its registrations are still live, held by the app"
        );
        assert!(
            rmw_cerulion::adopt_take::release_callback_epoch() != 0,
            "the callback is still installed — the re-install was a no-op, not a teardown"
        );

        free_adopted_scan(*msg);
        teardown(node, publisher, subscription);
    }
}

/// CHILD: the three shapes that decide whether a take can lose its
/// registrations — live context, clear-after-return, and every take after.
#[test]
#[ignore]
fn child_lost_registration_window_is_shutdown_only_and_bounded() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("WindowShape{suffix}"));
        let (context, node) = setup_node_with_context(&format!("windowshape_{suffix}"));
        let topic = CString::new(format!("/rmw_adopt/windowshape/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let stats = adopt_stats(subscription);
        let o = scan_oracle(8, 4, 29.0);

        // SHAPE 1 — context ALIVE. A take that completes here never loses its
        // registrations: they release normally when the app frees.
        publish_scan(publisher, &o);
        let (taken, msg1, _) = take_scan(subscription);
        assert!(taken);
        let per_take = fake_segments().len();
        assert!(per_take > 0, "precondition: the take registered ranges");
        free_adopted_scan(*msg1);
        assert!(
            fake_segments().is_empty(),
            "a take under a LIVE context is fully releasable — nothing is stranded"
        );
        assert_eq!(stat(&stats).3, 0, "…and its sample is back");
        assert_eq!(
            stat(&stats).2,
            per_take as u64,
            "one release per registered range"
        );

        // SHAPE 2 — THE RESIDUAL, driven at the exact instant it
        // matters: the clear lands AFTER the post-registration epoch re-read and
        // BEFORE the take returns. Nothing downstream of the last check can
        // notice, because the app takes delivery when `rmw_take` returns.
        let fired_before = rmw_cerulion::test_seams::adopt_report_window_clears_fired();
        publish_scan(publisher, &o);
        let (taken, msg2, _) = {
            let _race = rmw_cerulion::test_seams::AdoptReportWindowClearGuard::arm();
            take_scan(subscription)
        };
        assert_eq!(
            rmw_cerulion::test_seams::adopt_report_window_clears_fired(),
            fired_before + 1,
            "the post-check seam must have fired — otherwise this arm proves nothing"
        );
        assert!(
            taken,
            "the take still reports success — that is what makes it a residual"
        );
        assert_scan_values(&msg2, &o);
        assert!(
            fake().lock().expect("fake").release_cb.is_none(),
            "precondition: the clear landed"
        );
        assert_eq!(
            fake_segments().len(),
            per_take,
            "THE RESIDUAL, measured: this take's registrations outlive the callback that \
             would release them — a known and accepted limitation. A clear \
             one step EARLIER is caught and rolled back \
             (a_shutdown_racing_the_registrations_withdraws_them_and_serves_by_copy); past \
             this point there is no check left to add, only a lock or a deferred teardown, \
             and both were rejected."
        );
        assert_eq!(stat(&stats).3, 1, "…and its sample is pinned");
        // The same loss arrives from a plain post-RETURN `rmw_shutdown`; that
        // shape is the final-shutdown contract and is pinned by
        // `child_summary_lines_with_outstanding`. Shutting down here now is a
        // no-op for the callback (already cleared) and makes the child
        // end with the context closed.
        assert_eq!(rmw_shutdown(context), RMW_RET_OK);

        // SHAPE 3 — BOUNDED. Every later take is degraded by the dispatch gate,
        // so the stranded set cannot grow: the window is one take wide, not a
        // leak that compounds for as long as the app keeps taking.
        publish_scan(publisher, &o);
        let (taken, mut msg3, _) = take_scan(subscription);
        assert!(taken, "later takes are still SERVED — by copy");
        assert_scan_values(&msg3, &o);
        assert_eq!(
            fake_segments().len(),
            per_take,
            "no NEW registration after the callback is gone — the loss is bounded to the \
             takes already in flight, it does not compound"
        );
        assert_eq!(stat(&stats).3, 1, "…and no new sample is pinned");
        scan_fini(&mut *msg3 as *mut _ as *mut c_void);

        // `msg2` is deliberately NOT freed: its forged pointers have no release
        // path, which is the residual itself. The contract leaks it to process
        // exit, and `simulate_free` would panic on the absent callback — the
        // fake refusing to invent one is the right behaviour.
        std::mem::forget(msg2);
        // The parent pipes STDERR and nulls stdout.
        eprintln!("CHILD_WINDOW_SHAPE_OK");
    }
}

/// The window
/// in which an adopted take can lose its registrations is SHUTDOWN-ONLY and
/// BOUNDED — which is the claim accepting the limitation rests on, so
/// it is pinned rather than asserted in prose.
///
/// Three shapes, one arm, because the claim is about which of them can strand
/// a registration:
///
/// 1. context ALIVE — a take that completes here is fully releasable; the
///    app's free releases every range. No window.
/// 2. the clear lands after the take's LAST CHECK — driven at the exact
///    instant of the loss (past the post-registration epoch re-read,
///    before the return). Its
///    registrations outlive the callback. THE RESIDUAL: the
///    app takes delivery when `rmw_take` returns, so no check inside the take
///    closes this, only moves it — which the dispatch gate and the epoch
///    re-read each did, one window apart. A plain post-RETURN shutdown loses
///    the same way and is the contract pinned by
///    (`child_summary_lines_with_outstanding`). rcl stops executors before
///    `rmw_shutdown`, so a well-formed shutdown has no take in flight.
/// 3. every take AFTER that — degraded to a copy by the dispatch gate, so
///    nothing new is registered. This is what makes the loss BOUNDED to the
///    takes already in flight instead of compounding for as long as the app
///    keeps taking, and it is the half that can actually catch a broken variant.
///
/// A clear landing DURING a take is the fourth shape and is CAUGHT, not lost —
/// `a_shutdown_racing_the_registrations_withdraws_them_and_serves_by_copy`.
#[test]
#[serial]
fn the_lost_registration_window_is_shutdown_only_and_bounded() {
    let (status, err) = run_child(
        "child_lost_registration_window_is_shutdown_only_and_bounded",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    assert!(
        err.contains("CHILD_WINDOW_SHAPE_OK"),
        "the child must have reached its end:\n{err}"
    );
    assert!(
        err.contains("adopt-take degraded to the copying take"),
        "the degrade that BOUNDS the window is announced, so an operator can see why the \
         zero-copy path stopped:\n{err}"
    );
}

// =====================================================================
// The residual-window arms
// =====================================================================

/// The parts of a `CScan` that carry information, read WITHOUT touching the
/// struct's padding (reading uninitialised padding bytes as `u8` would be
/// UB, and this oracle exists to be trustworthy).
///
/// Every member that a decode would overwrite is here: the two fixed floats
/// by their exact bits, and each container by its `{data, size, capacity}`
/// triplet — which is what "the caller's message was not written" has to
/// mean for a message whose members are pointers.
#[derive(Debug, PartialEq, Eq)]
struct ScanImage {
    angle_min_bits: u32,
    angle_max_bits: u32,
    ranges: (usize, usize, usize),
    intensities: (usize, usize, usize),
    frame_id: (usize, usize, usize),
}

fn scan_image(m: &CScan) -> ScanImage {
    ScanImage {
        angle_min_bits: m.angle_min.to_bits(),
        angle_max_bits: m.angle_max.to_bits(),
        ranges: (m.ranges.data as usize, m.ranges.size, m.ranges.capacity),
        intensities: (
            m.intensities.data as usize,
            m.intensities.size,
            m.intensities.capacity,
        ),
        frame_id: (
            m.frame_id.data as usize,
            m.frame_id.size,
            m.frame_id.capacity,
        ),
    }
}

/// Take into a message the CALLER already owns and has filled — the reuse
/// pattern the adopt path exists to support, and the only shape in which a
/// clobber is observable at all.
unsafe fn take_into(subscription: *const ffi::rmw_subscription_t, msg: &mut CScan) -> bool {
    let mut taken = false;
    let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
    assert_eq!(
        rmw_take_with_info(
            subscription,
            msg as *mut CScan as *mut c_void,
            &mut taken,
            &mut info,
            std::ptr::null_mut(),
        ),
        RMW_RET_OK,
        "a malformed frame is DROPPED, never turned into an error return"
    );
    taken
}

/// Publish a frame whose first offset-table entry points PAST the end of the
/// payload — the malformed-frame class `read_var_entry` refuses.
///
/// Built the way `publish_scan_below_floor` builds its frame: serialize a
/// real message through the production `rmw_serialize`, then patch the one
/// table entry, so everything else about the frame (header, hash, fixed
/// section, the other entries) is exactly what a real producer emits and the
/// refusal can only be the entry's doing. The LENGTH is left whole, so this
/// is the OFFSET class and not a ragged element count.
unsafe fn publish_scan_with_unresolvable_entry(
    publisher: *const ffi::rmw_publisher_t,
    ts: *const ffi::rosidl_message_type_support_t,
    o: &ScanOracle,
) {
    const FIXED_SIZE: usize = 8; // angle_min: f32, angle_max: f32
    let (msg, owned) = scan_value(o);
    let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
    ser.allocator = scan_serialize_allocator();
    assert_eq!(
        rmw_serialize(&msg as *const _ as *const c_void, ts, &mut ser),
        RMW_RET_OK
    );
    {
        let frame = std::slice::from_raw_parts_mut(ser.buffer, ser.buffer_length);
        let entry = cerulion_core::wire::WireHeader::SIZE + FIXED_SIZE;
        let len = u32::from_le_bytes(frame[entry + 4..entry + 8].try_into().expect("len"));
        assert_eq!(
            len as usize,
            o.ranges.len() * 4,
            "premise: the entry at the table base must be `ranges` — a layout change must \
             fail HERE, not silently patch some other field's bytes"
        );
        let past_end = u32::try_from(ser.buffer_length).expect("frame length fits u32") + 4096;
        frame[entry..entry + 4].copy_from_slice(&past_end.to_le_bytes());
    }
    assert_eq!(
        rmw_publish_serialized_message(publisher, &ser, std::ptr::null_mut()),
        RMW_RET_OK
    );
    for p in owned {
        free(p);
    }
}

/// Publish a frame whose first offset-table entry declares a length that is
/// NOT a whole number of elements — the other malformation class the gate
/// refuses, and the one a `None` `elem_size` at the call site would silently
/// drop (the pure verdict cannot see which stride its caller chose).
///
/// The OFFSET is left alone, so this is the LENGTH class and not the offset
/// one its sibling helper covers.
unsafe fn publish_scan_with_ragged_length(
    publisher: *const ffi::rmw_publisher_t,
    ts: *const ffi::rosidl_message_type_support_t,
    o: &ScanOracle,
) {
    const FIXED_SIZE: usize = 8; // angle_min: f32, angle_max: f32
    let (msg, owned) = scan_value(o);
    let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
    ser.allocator = scan_serialize_allocator();
    assert_eq!(
        rmw_serialize(&msg as *const _ as *const c_void, ts, &mut ser),
        RMW_RET_OK
    );
    {
        let frame = std::slice::from_raw_parts_mut(ser.buffer, ser.buffer_length);
        let entry = cerulion_core::wire::WireHeader::SIZE + FIXED_SIZE;
        let len = u32::from_le_bytes(frame[entry + 4..entry + 8].try_into().expect("len"));
        assert_eq!(
            len as usize,
            o.ranges.len() * 4,
            "premise: the entry at the table base must be `ranges` — a layout change must \
             fail HERE, not silently patch some other field's bytes"
        );
        assert!(
            len >= 1,
            "premise: the ragged length must stay non-negative"
        );
        frame[entry + 4..entry + 8].copy_from_slice(&(len - 1).to_le_bytes());
    }
    assert_eq!(
        rmw_publish_serialized_message(publisher, &ser, std::ptr::null_mut()),
        RMW_RET_OK
    );
    for p in owned {
        free(p);
    }
}

/// A frame whose offset-table entry cannot be resolved is
/// refused with the caller's message BYTE-UNTOUCHED.
///
/// Both decodes on this path walk the type's members writing as they go and
/// bail at the first entry they cannot read, so a caller legally REUSING one
/// message across takes was left holding a chimera — some members from the
/// new frame, the rest from the one it had — on a call that reported nothing
/// taken. The pre-write gate answers the same question read-only first.
///
/// The oracle is the message's own image (both floats by bits, all three
/// container triplets) PLUS its contents, taken before and after; the frame
/// carries deliberately DIFFERENT values, so a decode that ran is visible.
/// The anti-tautology half is in the same body: a well-formed frame through
/// the identical apparatus must CHANGE that image, otherwise "unchanged"
/// would be a statement about the probe rather than about the gate.
///
/// The filed remedy for this item was a scratch decode plus a member swap.
/// It is not what shipped and cannot be: a rosidl C++ message holds real
/// `std::string` objects, which libstdc++ makes non-trivially-relocatable,
/// so moving a decoded message's bytes into the caller's would corrupt every
/// string member on the platform ROS 2 ships (`type_bridge_cpp` records the
/// same constraint from the other side and reaches those members only
/// through the compiled shim).
#[test]
#[serial]
fn a_frame_with_an_unresolvable_entry_leaves_the_callers_message_untouched() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Item12{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "item12", suffix);
        let stats = adopt_stats(subscription);

        // A caller-owned message with content of its OWN: every member is
        // heap-owned, so a clobbering take would legally free and overwrite
        // all of it — nothing here is a shared-memory address.
        let held = scan_oracle(3, 2, 11.0);
        let (msg_val, owned) = scan_value(&held);
        let mut msg = Box::new(msg_val);
        let before = scan_image(&msg);

        // TWO malformation classes, one after the other, into the SAME
        // caller-owned message: an entry whose offset lies outside the
        // payload, and a primitive sequence whose declared length is not a
        // whole number of elements. The second is the class a call site that
        // passed no stride would silently stop refusing — the pure verdict
        // cannot see which stride its caller chose.
        for (what, publish) in [
            (
                "an entry offset outside the payload",
                publish_scan_with_unresolvable_entry
                    as unsafe fn(
                        *const ffi::rmw_publisher_t,
                        *const ffi::rosidl_message_type_support_t,
                        &ScanOracle,
                    ),
            ),
            ("a ragged element count", publish_scan_with_ragged_length),
        ] {
            let failures_before = decode_failure_count(subscription);
            // The frame's values differ from the held message's in every member.
            publish(publisher, ts, &scan_oracle(4, 3, 99.0));

            assert!(
                !take_into(subscription, &mut msg),
                "{what}: a malformed frame must not be delivered"
            );
            assert_eq!(
                scan_image(&msg),
                before,
                "{what}: THE pin — the frame was refused BEFORE anything was written, so the \
                 caller's message is the one it had, not a chimera of its own members and \
                 the frame's"
            );
            // Load-bearing beside the image, not decoration: an allocator that
            // returned the SAME address for a same-sized member would leave the
            // image identical while the CONTENTS changed.
            assert_scan_values(&msg, &held);
            // The ARRIVAL witness. Without it every assertion above is equally
            // true of a frame that never reached the subscriber, and the arm
            // would pass having proven nothing.
            assert_eq!(
                decode_failure_count(subscription),
                failures_before + 1,
                "{what}: the frame REACHED the subscriber and was refused by the pre-write \
                 gate — it is not merely absent"
            );
            assert_eq!(
                stat(&stats),
                (0, 0, 0, 0, 0, 0),
                "{what}: nothing was taken, adopted, released, or counted as a fallback"
            );
            assert!(
                fake_segments().is_empty(),
                "{what}: and nothing reached the hook"
            );
        }
        // The gate sits ahead of the reuse pre-pass too, so the caller still
        // owns every buffer it came in with — freeing them here is safe, and
        // a double free would abort if the take had released them.
        for p in owned {
            free(p);
        }

        // ANTI-TAUTOLOGY: the same probe, the same reuse pattern, a WELL-FORMED
        // frame. The image must change — otherwise the assertion above is
        // about the probe, not about the gate.
        let (msg2_val, _owned2) = scan_value(&held);
        let mut msg2 = Box::new(msg2_val);
        let before2 = scan_image(&msg2);
        let good = scan_oracle(4, 3, 99.0);
        publish_scan(publisher, &good);
        assert!(
            take_into(subscription, &mut msg2),
            "the control frame is well formed and must be delivered"
        );
        assert_ne!(
            scan_image(&msg2),
            before2,
            "the probe must be able to SEE a take that wrote — otherwise the unchanged \
             assertion above proves nothing"
        );
        assert_scan_values(&msg2, &good);
        assert_eq!(
            stat(&stats).1,
            1,
            "and the control was really ADOPTED, so the gate lets a well-formed frame all \
             the way through rather than merely not refusing it"
        );
        assert_eq!(
            decode_failure_count(subscription),
            2,
            "a well-formed frame does not move the decode-failure counter — which is what \
             makes the two increments above attributable to the refusals"
        );
        // The take freed `_owned2`'s three buffers itself (the forgeable
        // members through the reuse pre-pass, the string inside its own
        // assign), so only the adopted pointers are left to release.
        free_adopted_scan(*msg2);
        teardown(node, publisher, subscription);
    }
}

/// A service whose borrow ceiling cannot carry adoption is
/// not ARMED for it — and the subscription still delivers, by copy.
///
/// A caller reusing one message needs two borrow units at the instant of a
/// take: the one its previously-adopted sample still pins, and the one the
/// receive consumes. The release that frees the first runs INSIDE the take,
/// after the receive, so at a ceiling of 1 that receive is refused and the
/// release never runs — a permanent wedge whose documented self-heal ("free
/// an adopted message") is unreachable, because for such a caller the free
/// IS the refused call. No check inside the take can pre-empt it: the
/// receive is what consumes the borrow. So the gate is at CREATE.
///
/// Reachability is not assumed — it is BUILT here. This crate's own create
/// leg asks for `max(budget, 4)`, but the OPEN leg is requirement-free by
/// design, so the shape exists only when a FOREIGN creator minted the
/// service first; the test mints one through `cerulion_core` and holds it.
/// The control pair on an ordinary topic is in the same body, so a gate that
/// simply disabled adoption everywhere fails it.
#[test]
#[serial]
fn a_service_below_the_borrow_minimum_is_not_adopt_armed_and_still_delivers() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        // The node comes FIRST: it is what brings the rmw runtime (and with
        // it the transport) up, and the foreign service has to be minted on
        // that same manager.
        let node = setup_node(&format!("item13_node_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");

        // The foreign creator: a borrow ceiling of ONE.
        let cer_topic = format!("/rmw_adopt/item13/{suffix}");
        let mut cfg = rt.transport.default_topic_config();
        cfg.publisher_provisioning = cerulion_core::transport::PublisherProvisioning::External;
        cfg.subscriber_max_borrowed_samples = Some(1);
        let depth = cfg.subscriber_max_buffer_size;
        let pin = rt
            .transport
            .create_subscriber_with_buffers(&cer_topic, cfg, depth)
            .expect("the borrow-1 service must be creatable — it is this arm's premise");
        assert_eq!(
            pin.max_borrowed_samples(),
            1,
            "PREMISE: the foreign service must really be minted at a ceiling of ONE — the \
             create path composes borrow floors, and a silently raised ceiling would make \
             every assertion below vacuous. This is the value production reads."
        );

        let ts = scan_ts(&format!("Item13{suffix}"));
        let topic = CString::new(cer_topic.clone()).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(
            !subscription.is_null(),
            "the subscription must OPEN the pre-existing borrow-1 service, not refuse it"
        );
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null(), "publisher creation failed");
        assert!(
            sub_data(subscription).adopt.is_none(),
            "adoption must NOT be armed on a service that cannot lend a held sample and an \
             incoming receive at the same time"
        );

        // It still DELIVERS — the degrade is to the copying take, not to
        // nothing.
        let o = scan_oracle(3, 2, 4.0);
        publish_scan(publisher, &o);
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(taken, "the copying take serves the frame");
        assert_scan_values(&msg, &o);
        assert!(
            fake_segments().is_empty(),
            "and the copying take registers nothing with the hook"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        teardown(node, publisher, subscription);
        drop(pin);

        // CONTROL: the same type, the same env, an ORDINARY service — armed.
        let suffix2 = unique_suffix();
        let ts2 = scan_ts(&format!("Item13ok{suffix2}"));
        let (node2, publisher2, subscription2) = setup_pair(ts2, "item13ok", suffix2);
        assert!(
            sub_data(subscription2).adopt.is_some(),
            "the gate must key on the SERVICE's ceiling — a normal service still arms"
        );
        let o2 = scan_oracle(3, 2, 6.0);
        publish_scan(publisher2, &o2);
        let (taken2, msg2, _) = take_scan(subscription2);
        assert!(taken2);
        assert_scan_values(&msg2, &o2);
        free_adopted_scan(*msg2);
        teardown(node2, publisher2, subscription2);
    }
}

/// CHILD (runs only under the parent's re-exec): six malformed frames then a
/// well-formed one, so the parent can count what reached stderr.
#[test]
#[ignore]
fn child_malformed_frames_are_flood_latched() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Flood{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "flood", suffix);
        let bad = scan_oracle(4, 3, 3.0);
        for _ in 0..6 {
            publish_scan_with_unresolvable_entry(publisher, ts, &bad);
            let (taken, mut msg, _) = take_scan(subscription);
            assert!(!taken, "every malformed frame is refused");
            scan_fini(&mut *msg as *mut _ as *mut c_void);
        }
        assert_eq!(
            decode_failure_count(subscription),
            6,
            "the counter is UNCONDITIONAL — it moves on every refusal, whatever the log did"
        );
        // A good frame closes the regime.
        let good = scan_oracle(3, 2, 8.0);
        publish_scan(publisher, &good);
        let (taken, msg, _) = take_scan(subscription);
        assert!(taken, "the healthy frame is served");
        assert_eq!(
            decode_failure_count(subscription),
            6,
            "...and recovery does NOT reset the running total"
        );
        free_adopted_scan(*msg);
        teardown(node, publisher, subscription);
        eprintln!("CHILD_FLOOD_OK");
    }
}

/// The malformed-frame refusal is
/// FLOOD-LATCHED, not one line per frame.
///
/// A stale or hand-rolled producer emits a malformed frame EVERY frame, so an
/// unlatched line here is one warning per frame at frame rate — the
/// disk-fill class, and on top of the latched report it would have duplicated
/// that report's own loud head and decade re-announcements. A bare `warn!`
/// beside the latched reporter, kept to preserve the member attribution,
/// would do exactly that: a real need met the
/// wrong way. The attribution rides the latched line instead.
///
/// The oracle is the shape, and it is LEVEL-MATCHED: six refusals
/// must produce exactly ONE loud head (the suppressed repeats are `debug!`,
/// so at `RUST_LOG=info` they are absent) while the unconditional counter
/// reads 6, and the frame that heals the regime must produce exactly ONE
/// recovery line without resetting that counter. A text-only assertion would
/// pass a reporter that kept emitting at `error!`, which is the variant this
/// arm exists to kill.
#[test]
#[serial]
fn malformed_frames_are_latched_to_one_loud_head_not_one_line_per_frame() {
    let (status, err) = run_child(
        "child_malformed_frames_are_flood_latched",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    assert!(
        err.contains("CHILD_FLOOD_OK"),
        "the child must have reached its end:\n{err}"
    );
    // Every LOUD line this reporter can emit, matched by its MODULE and level
    // rather than by the head's own text. Filtering on the head's wording
    // cannot see a SUPPRESSED-repeat arm promoted to `error!` — which is the
    // whole failure this test exists to catch, and which a text-scoped filter
    // lets through.
    let loud: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("ERROR") && l.contains("rmw_cerulion::decode_failure_latch"))
        .collect();
    assert_eq!(
        loud.len(),
        1,
        "SIX malformed frames must produce exactly ONE loud line from the decode-failure \
         reporter — more is the disk-fill class this latch exists to prevent (whichever arm \
         emitted them), none means the operator was told nothing. Got {}:\n{err}",
        loud.len()
    );
    let head = loud[0];
    assert!(
        head.contains("refusing a frame before decoding it"),
        "...and the one loud line is the HEAD, not a repeat that was promoted: {head}"
    );
    assert!(
        has_field(head, "var_idx", "0"),
        "the head names WHICH member is malformed — the attribution a bare warn would \
         carry: {head}"
    );
    assert!(
        has_field(head, "reason", "entry_out_of_bounds"),
        "...and HOW: {head}"
    );
    assert!(
        !err.contains("malformed variable entry in wire frame"),
        "the un-latched per-frame line must be gone entirely, not merely quieter:\n{err}"
    );
    let recoveries: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("INFO") && l.contains("decode") && l.contains("recover"))
        .collect();
    assert_eq!(
        recoveries.len(),
        1,
        "the healing frame closes the regime exactly once. Got {}:\n{err}",
        recoveries.len()
    );
    assert!(
        has_field(recoveries[0], "suppressed_count", "5"),
        "and the recovery reports what the operator MISSED (6 refusals, 1 loud): {}",
        recoveries[0]
    );
}

/// CHILD (runs only under the parent's re-exec): a subscription on a service
/// whose borrow ceiling cannot carry adoption. The parent reads the
/// non-arming warn off stderr.
#[test]
#[ignore]
fn child_borrow_ceiling_too_small_says_why() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let node = setup_node(&format!("item13w_node_{suffix}"));
        let rt = rmw_cerulion::runtime::runtime().expect("runtime");
        let cer_topic = format!("/rmw_adopt/item13w/{suffix}");
        let mut cfg = rt.transport.default_topic_config();
        cfg.publisher_provisioning = cerulion_core::transport::PublisherProvisioning::External;
        cfg.subscriber_max_borrowed_samples = Some(1);
        let depth = cfg.subscriber_max_buffer_size;
        let pin = rt
            .transport
            .create_subscriber_with_buffers(&cer_topic, cfg, depth)
            .expect("the borrow-1 service must be creatable");
        assert_eq!(pin.max_borrowed_samples(), 1, "PREMISE: ceiling of ONE");
        let ts = scan_ts(&format!("Item13W{suffix}"));
        let topic = CString::new(cer_topic).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());
        assert!(sub_data(subscription).adopt.is_none());
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
        drop(pin);
        eprintln!("CHILD_CEILING_WARN_OK");
    }
}

/// The non-arming is ANNOUNCED, with the numbers and the
/// remedy.
///
/// This warn is the only thing that tells a ROS user their zero-copy path
/// silently became a copying one — the rmw counters sit behind the
/// standardized C ABI, so the log is their whole window. Without this arm a
/// variant that keeps `adopt = None` and deletes the line passes the suite
/// and ships a silent degrade, which is the exact defect class the house
/// rules forbid.
///
/// The ANTI-TAUTOLOGY half is the control: an ordinary service must NOT
/// print it, and must print the affirmative ARMED line instead — which also
/// pins that the two claims are mutually exclusive (an affirmative line
/// emitted before the gate could retract it would break that).
#[test]
#[serial]
fn the_borrow_ceiling_refusal_says_why_and_an_ordinary_service_does_not() {
    let (status, err) = run_child(
        "child_borrow_ceiling_too_small_says_why",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    assert!(
        err.contains("CHILD_CEILING_WARN_OK"),
        "the child must have reached its end:\n{err}"
    );
    let line = err
        .lines()
        .find(|l| l.contains("WARN") && l.contains("adopt-take NOT armed"))
        .unwrap_or_else(|| panic!("the non-arming was silent; stderr:\n{err}"));
    assert!(
        has_field(line, "effective_budget", "1"),
        "the line must name the service's REAL ceiling: {line}"
    );
    assert!(
        has_field(line, "minimum", "2"),
        "...and the minimum it fell short of: {line}"
    );
    assert!(
        line.contains("served by the COPYING take"),
        "...and what serves the subscription instead: {line}"
    );
    assert!(
        !err.contains("adopt-take ARMED"),
        "a subscription that was NOT armed must never have claimed it was — the affirmative \
         line is made only after this gate passes:\n{err}"
    );

    // CONTROL: an ordinary service arms, says so, and does not print the refusal.
    let (status2, err2) = run_child(
        "child_registration_fallback_reports_the_partial_count",
        &[(ADOPT_TAKE_ENV, "1")],
    );
    assert!(status2.success(), "control child failed:\n{err2}");
    assert!(
        !err2.contains("adopt-take NOT armed"),
        "an ordinary service must not print the ceiling refusal:\n{err2}"
    );
    assert!(
        err2.contains("adopt-take ARMED"),
        "...and must make the affirmative claim, so the assertion above is about the GATE \
         and not about a line nobody ever prints:\n{err2}"
    );
}

/// CHILD (runs only under the parent's re-exec): the whole borrow budget is
/// held by outstanding LOANED takes when the adopted take's receive is
/// refused. The parent reads the refusal line off stderr.
#[test]
#[ignore]
fn child_loaned_borrows_hold_the_budget_at_the_adopt_refusal() {
    if std::env::var("CER_RMW_ADOPT_TAKE_CHILD").is_err() {
        eprintln!("child body invoked without the parent harness — skipping");
        return;
    }
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("Item10{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "item10", suffix);
        let stats = adopt_stats(subscription);
        let o = scan_oracle(3, 2, 2.0);
        for _ in 0..6 {
            publish_scan(publisher, &o);
        }
        // Hold the WHOLE borrow budget as loans. With the adopt budget set
        // to 1 the service is minted at `max(1, RMW_TAKE_LOAN_BORROW_BUDGET)`
        // = 4, which is also the shadow pool's capacity — so four loaned
        // takes take every unit and none of them is an adopted sample.
        let mut loans: Vec<*mut c_void> = Vec::new();
        for _ in 0..4 {
            let mut p: *mut c_void = std::ptr::null_mut();
            let mut taken = false;
            assert_eq!(
                rmw_take_loaned_message(subscription, &mut p, &mut taken, std::ptr::null_mut()),
                RMW_RET_OK
            );
            assert!(
                taken,
                "each loaned take must be served — the budget is not spent yet"
            );
            loans.push(p);
        }
        // The adopted take's receive now fails `ExceedsMaxBorrows`.
        let (taken, mut msg, _) = take_scan(subscription);
        assert!(!taken, "the receive is refused, so nothing is delivered");
        assert_eq!(
            stats.budget_refusals.load(Ordering::Relaxed),
            1,
            "exactly one refusal"
        );
        assert_eq!(
            stats.outstanding.load(Ordering::Relaxed),
            0,
            "and NO adopted message is held — every borrow is a loan, which is the whole \
             point of this arm"
        );
        scan_fini(&mut *msg as *mut _ as *mut c_void);
        // Now the MIXED shape: give ONE borrow back and let an adopted take
        // have it, so both counters are nonzero — the only state in which
        // either single-sided remedy would be half wrong.
        //
        // Exactly one adopted message, not two: the env budget is 1, so
        // `retention_limit = min(budget, effective) = 1` and a SECOND adopted
        // take would be served by the over-retention COPY arm and retain
        // nothing. (Asking for two here measures one — the
        // budget that makes the loaned-only shape reachable is the same
        // budget that caps retention at one.)
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, loans.remove(0)),
            RMW_RET_OK
        );
        let (taken_adopt, msg_adopt, _) = take_scan(subscription);
        assert!(taken_adopt, "the freed borrow lets an adopted take through");
        assert_eq!(
            stats.outstanding.load(Ordering::Relaxed),
            1,
            "one adopted sample is held, beside three loans"
        );
        let (taken_mixed, mut msg_mixed, _) = take_scan(subscription);
        assert!(
            !taken_mixed,
            "the budget is spent again: 3 loans + 1 adopted"
        );
        assert_eq!(
            stats.budget_refusals.load(Ordering::Relaxed),
            2,
            "the second refusal"
        );
        scan_fini(&mut *msg_mixed as *mut _ as *mut c_void);
        free_adopted_scan(*msg_adopt);
        for p in loans {
            assert_eq!(
                rmw_return_loaned_message_from_subscription(subscription, p),
                RMW_RET_OK
            );
        }
        teardown(node, publisher, subscription);
        eprintln!("CHILD_LOANED_REFUSAL_OK");
    }
}

/// When LOANS hold the borrow budget, the adopted take's
/// refusal says so — and does not advise a release the consumer cannot make.
///
/// An adopt-armed type is always `can_loan_take` too, so outstanding loaned
/// takes consume the SAME `subscriber_max_borrowed_samples` budget adopted
/// samples do. A refusal that reports `kind=adopted_budget_exhausted`
/// and tells the operator to free adopted messages whichever side holds the
/// borrows — advice that does nothing when every outstanding borrow is a
/// loan, on a line operators grep BY KIND.
///
/// Fields are asserted as whole `key=value` tokens (`loaned_outstanding=4`
/// is a substring of `loaned_outstanding=40`), and the remedy is pinned in
/// BOTH directions: it must name the return call, and it must NOT tell this
/// consumer to free adopted messages it does not have.
#[test]
#[serial]
fn a_refusal_held_by_loans_names_the_loans_and_not_the_adopted_messages() {
    let (status, err) = run_child(
        "child_loaned_borrows_hold_the_budget_at_the_adopt_refusal",
        &[(ADOPT_TAKE_ENV, "1"), (ADOPT_TAKE_BUDGET_ENV, "1")],
    );
    assert!(status.success(), "child failed:\n{err}");
    assert!(
        err.contains("CHILD_LOANED_REFUSAL_OK"),
        "the child must have reached its end:\n{err}"
    );
    let line = err
        .lines()
        .find(|l| l.contains("WARN") && l.contains("adopted take refused"))
        .unwrap_or_else(|| panic!("no adopt refusal warn; stderr:\n{err}"));
    assert!(
        has_field(line, "kind", "loaned_borrows_exhausted"),
        "the kind must name who actually holds the budget — `adopted_budget_exhausted` here \
         sends an operator to the wrong API: {line}"
    );
    assert!(
        has_field(line, "adopted_outstanding", "0"),
        "no adopted message is held: {line}"
    );
    assert!(
        has_field(line, "loaned_outstanding", "4"),
        "four loans are: {line}"
    );
    assert!(
        line.contains("rmw_return_loaned_message_from_subscription"),
        "the remedy must name the call that actually releases a unit here: {line}"
    );
    assert!(
        !line.contains("free adopted messages (their fini"),
        "and must NOT prescribe freeing adopted messages this consumer does not hold — that \
         was the defect: {line}"
    );

    // The MIXED holder, from the same child's second refusal: both counters
    // nonzero, so neither single-sided remedy is the whole truth and the line
    // must name both. Without this arm `Both` is a variant nothing drives.
    let mixed = err
        .lines()
        .filter(|l| l.contains("adopted take") && l.contains("refused"))
        .find(|l| has_field(l, "kind", "mixed_borrows_exhausted"))
        .unwrap_or_else(|| panic!("no mixed-holder refusal; stderr:\n{err}"));
    assert!(
        has_field(mixed, "adopted_outstanding", "1") && has_field(mixed, "loaned_outstanding", "3"),
        "the mixed line must carry BOTH counts: {mixed}"
    );
    assert!(
        mixed.contains("rmw_return_loaned_message_from_subscription")
            && mixed.contains("free adopted messages"),
        "...and name BOTH releases, since either one frees a unit: {mixed}"
    );
}
