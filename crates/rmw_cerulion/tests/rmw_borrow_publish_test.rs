// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The windowed borrow, C-ABI level: `rmw_borrow_loaned_message` →
//! (simulated) fill → `rmw_publish_loaned_message` for an UNBOUNDED
//! primitive-sequence type, over REAL iceoryx2 shared memory — the
//! windowed-borrow spine end to end, with the heap hook stood in by a
//! FAKE installed through the `test-seams` hook seam
//! (`heaphook::TestHookGuard`). The fake implements the hook's window
//! contract (per-thread arm/disarm, bump-cursor range test, retire
//! recording) as plain Rust state, and the tests act as BOTH the
//! launcher-preloaded hook and the stock fill: `fake_bump` hands out
//! tail addresses exactly as the interposed `malloc` would, and the test
//! writes the sequence bytes there before aiming the rosidl header at
//! them — which is byte-for-byte the state a real bump-backed
//! `resize`/`assign` leaves. What the fake CANNOT prove — a real
//! `LD_PRELOAD`ed libstdc++ fill bumping through the real interposer —
//! is the Linux container arm.
//!
//! Oracles are hand-built: the published frame is taken back RAW
//! (`rmw_take_serialized_message` — the "cerulion" serialized form IS
//! the wire frame) and compared byte-for-byte against a hand-assembled
//! expected payload; zero-copy vs copy is asserted through the
//! Principle-#3 counters on `PublisherData`
//! (`borrow_adopted_count`/`borrow_copied_count`/`borrow_degrade_count`)
//! — the frame bytes alone cannot discriminate an adopted publish from a
//! copy that landed at the same offsets, the counters can.
//!
//! ⚠️ iceoryx2 shared memory is a process singleton AND the fake hook is
//! process-global — run with `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_borrow_publish_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::collections::HashMap;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};
use std::thread::ThreadId;

use cerulion_core::wire::WireHeader;
use rmw_cerulion::ffi::{self, RMW_RET_BAD_ALLOC, RMW_RET_OK, RMW_RET_UNSUPPORTED};
use rmw_cerulion::heaphook::{
    HookApi, TestHookGuard, RC_ERR_ALREADY_ARMED, RC_ERR_NOT_ARMED, RC_OK,
};
use rmw_cerulion::runtime::PublisherData;
use rmw_cerulion::*;

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_STRING: u8 = 16;

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn align_up(v: usize, a: usize) -> usize {
    (v + a - 1) & !(a - 1)
}

// =====================================================================
// The FAKE heap hook: the window contract as plain per-thread state
// =====================================================================

#[derive(Clone, Copy)]
struct FakeWindow {
    base: usize,
    cursor: usize,
    limit: usize,
}

#[derive(Default)]
struct FakeState {
    windows: HashMap<ThreadId, FakeWindow>,
    /// Every `retire_slot(base)` the rmw issued — the retire-on-reuse
    /// observable.
    retired: Vec<usize>,
}

fn fake() -> &'static Mutex<FakeState> {
    static FAKE: OnceLock<Mutex<FakeState>> = OnceLock::new();
    FAKE.get_or_init(|| Mutex::new(FakeState::default()))
}

fn reset_fake() {
    let mut s = fake().lock().expect("fake");
    s.windows.clear();
    s.retired.clear();
}

unsafe extern "C" fn f_arm(base: *mut c_void, limit: *mut c_void) -> i32 {
    let me = std::thread::current().id();
    let mut s = fake().lock().expect("fake");
    if s.windows.contains_key(&me) {
        return RC_ERR_ALREADY_ARMED;
    }
    s.windows.insert(
        me,
        FakeWindow {
            base: base as usize,
            cursor: base as usize,
            limit: limit as usize,
        },
    );
    RC_OK
}

unsafe extern "C" fn f_disarm() -> i32 {
    let me = std::thread::current().id();
    match fake().lock().expect("fake").windows.remove(&me) {
        Some(_) => 0, // no escape latched
        None => RC_ERR_NOT_ARMED,
    }
}

unsafe extern "C" fn f_escape() -> i32 {
    let me = std::thread::current().id();
    if fake().lock().expect("fake").windows.contains_key(&me) {
        0
    } else {
        RC_ERR_NOT_ARMED
    }
}

unsafe extern "C" fn f_range(ptr: *const c_void, len: usize) -> i32 {
    let me = std::thread::current().id();
    match fake().lock().expect("fake").windows.get(&me) {
        Some(w) => {
            let p = ptr as usize;
            i32::from(p >= w.base && p + len <= w.cursor)
        }
        None => RC_ERR_NOT_ARMED,
    }
}

unsafe extern "C" fn f_retire(base: *mut c_void) -> i32 {
    fake().lock().expect("fake").retired.push(base as usize);
    RC_OK
}

unsafe extern "C" fn f_counter(kind: u32) -> u64 {
    // Distinct per-kind values so a counter read can be attributed.
    match kind {
        0..=5 => 100 + kind as u64,
        _ => u64::MAX,
    }
}

/// A STALE v2 hook's counter surface: same ABI version, but
/// built before kinds 4/5 existed — it answers its documented unknown-index
/// sentinel (`u64::MAX`) for them. The mixed-deployment shape the sentinel
/// rule degrades gracefully for.
unsafe extern "C" fn f_counter_stale_v2(kind: u32) -> u64 {
    match kind {
        0..=3 => 100 + kind as u64,
        _ => u64::MAX,
    }
}

/// The fake hook: the window entries are the real fake. The segment
/// registry entries this suite never drives spread from
/// `HookApi::inert()`'s inert `RC_OK` stubs.
fn fake_hook_api() -> HookApi {
    HookApi {
        arm_window: f_arm,
        disarm_window: f_disarm,
        window_escape: f_escape,
        window_range_test: f_range,
        retire_slot: f_retire,
        counter: f_counter,
        ..HookApi::inert()
    }
}

/// The fill's allocator stand-in: bump `len` bytes at `align` out of the
/// CURRENT thread's fake window — what the interposed `malloc` does under
/// an armed window. `None` past the tail (the real hook would escape to
/// the heap).
fn fake_bump(len: usize, align: usize) -> Option<usize> {
    let me = std::thread::current().id();
    let mut s = fake().lock().expect("fake");
    let w = s.windows.get_mut(&me)?;
    let aligned = align_up(w.cursor, align.max(1));
    let end = aligned.checked_add(len)?;
    if end > w.limit {
        return None;
    }
    w.cursor = end;
    Some(aligned)
}

fn fake_window_of_current_thread() -> Option<FakeWindow> {
    let me = std::thread::current().id();
    fake().lock().expect("fake").windows.get(&me).copied()
}

// =====================================================================
// The unbounded fixture — LaserScan-shaped (crib of the seal tests)
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
    #[cfg(cerulion_has_is_rosidl_buffer)]
    is_rosidl_buffer: bool,
    #[cfg(cerulion_has_is_rosidl_buffer)]
    owns_rosidl_buffer: bool,
}

#[repr(C)]
struct CScanish {
    angle_min: f32,
    ranges: CF32Seq,
    frame_id: CRosString,
}

unsafe extern "C" fn scanish_init(
    msg: *mut c_void,
    _init: ffi::rosidl_runtime_c__message_initialization,
) {
    let m = &mut *(msg as *mut CScanish);
    m.frame_id.data = calloc(1, 1) as *mut u8;
    m.frame_id.size = 0;
    m.frame_id.capacity = 1;
}

unsafe extern "C" fn scanish_fini(msg: *mut c_void) {
    let m = &mut *(msg as *mut CScanish);
    if !m.frame_id.data.is_null() {
        free(m.frame_id.data as *mut c_void);
        m.frame_id.data = std::ptr::null_mut();
    }
    if !m.ranges.data.is_null() {
        free(m.ranges.data as *mut c_void);
        m.ranges.data = std::ptr::null_mut();
    }
}

fn scanish_ts(type_name: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = vec![
        {
            let mut m = ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("angle_min"),
                type_id_: ROS_TYPE_FLOAT,
                offset_: 0,
                ..Default::default()
            };
            m.is_array_ = false;
            m
        },
        {
            let mut m = ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("ranges"),
                type_id_: ROS_TYPE_FLOAT,
                offset_: std::mem::offset_of!(CScanish, ranges) as u32,
                ..Default::default()
            };
            m.is_array_ = true;
            m.array_size_ = 0;
            m.is_upper_bound_ = false;
            m
        },
        ffi::rosidl_typesupport_introspection_c__MessageMember {
            name_: cstr("frame_id"),
            type_id_: ROS_TYPE_STRING,
            offset_: std::mem::offset_of!(CScanish, frame_id) as u32,
            ..Default::default()
        },
    ];
    let members = Box::leak(members.into_boxed_slice());
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("borrow_e2e__msg"),
            message_name_: cstr(type_name),
            member_count_: members.len() as u32,
            size_of_: std::mem::size_of::<CScanish>(),
            members_: members.as_ptr(),
            init_function: Some(scanish_init),
            fini_function: Some(scanish_fini),
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

/// Geometry oracle (must agree with `BorrowSlotGeometry` for CScanish):
/// fixed section 4, table 2×8, floor 20, tail at the struct rounded to 16.
const C_FIXED: usize = 4;
const C_FLOOR: usize = C_FIXED + 16;
fn tail_off() -> usize {
    align_up(std::mem::size_of::<CScanish>().max(C_FLOOR), 16)
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

unsafe extern "C" fn alloc_fn(size: usize, _state: *mut c_void) -> *mut c_void {
    calloc(1, size)
}
unsafe extern "C" fn dealloc_fn(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}
fn malloc_allocator() -> ffi::rcutils_allocator_t {
    let mut a: ffi::rcutils_allocator_t = unsafe { std::mem::zeroed() };
    a.allocate = Some(alloc_fn);
    a.deallocate = Some(dealloc_fn);
    a
}

/// One pub + sub pair for the fixture type on a unique topic.
unsafe fn setup_pair(
    tag: &str,
) -> (
    *mut ffi::rmw_publisher_t,
    *mut ffi::rmw_subscription_t,
    *const PublisherData,
) {
    let suffix = unique_suffix();
    let ts = scanish_ts(&format!("Scan{tag}{suffix}"));
    let (_, node, _opts) = setup_node(&format!("bw_{tag}_{suffix}"));
    let topic = CString::new(format!("/borrow_e2e/{tag}/{suffix}")).expect("topic");
    let qos = default_qos();
    let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    assert!(!subscription.is_null());
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
    assert!(!publisher.is_null());
    // Also keep the typesupport + node pointers (and the ROS topic) for
    // later C-ABI calls.
    BORROW_TS.with(|t| t.set(ts as usize));
    BORROW_NODE.with(|t| t.set(node as usize));
    BORROW_TOPIC.with(|t| *t.borrow_mut() = format!("/borrow_e2e/{tag}/{suffix}"));
    let pdata = (*publisher).data as *const PublisherData;
    (publisher, subscription, pdata)
}

thread_local! {
    static BORROW_TS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    static BORROW_NODE: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    static BORROW_TOPIC: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
}
fn ts_ptr() -> *const ffi::rosidl_message_type_support_t {
    BORROW_TS.with(|t| t.get()) as *const _
}
fn node_ptr() -> *mut ffi::rmw_node_t {
    BORROW_NODE.with(|t| t.get()) as *mut _
}
/// The ROS topic `setup_pair` last minted (identity-mapped to the Cerulion
/// name — `runtime::ros_topic_to_cerulion`).
fn ros_topic() -> String {
    BORROW_TOPIC.with(|t| t.borrow().clone())
}

/// The topic's LIVE publisher-port count as the transport sees
/// it (`Some(n)`; `None` = the probe itself failed). After a leaking
/// destroy this must stay at the leaked publisher's port — dropping
/// the `PublisherData` would deregister the port and read
/// `Some(0)` while the leaked samples keep the port's on-disk tag alive
/// (the dead-node sweep then wedges forever on that orphan tag).
fn live_publisher_ports() -> Option<u32> {
    let cerulion_topic = rmw_cerulion::runtime::ros_topic_to_cerulion(&ros_topic())
        .expect("a fully-qualified ROS name maps verbatim");
    rmw_cerulion::runtime::runtime()
        .expect("rmw runtime")
        .transport
        .topic_publisher_count_checked(&cerulion_topic)
}

/// Take the newest frame RAW; returns (header, payload bytes).
unsafe fn take_raw(subscription: *const ffi::rmw_subscription_t) -> (WireHeader, Vec<u8>) {
    let mut serialized: ffi::rmw_serialized_message_t = std::mem::zeroed();
    serialized.allocator = malloc_allocator();
    let mut taken = false;
    assert_eq!(
        rmw_take_serialized_message(
            subscription,
            &mut serialized,
            &mut taken,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    assert!(taken, "a published frame must be takeable");
    let frame = std::slice::from_raw_parts(serialized.buffer, serialized.buffer_length).to_vec();
    free(serialized.buffer as *mut c_void);
    let header = WireHeader::read_from_buf(&frame).expect("frame carries a header");
    assert_eq!(
        header.total_size as usize,
        frame.len(),
        "serialized = frame"
    );
    (header, frame[WireHeader::SIZE..].to_vec())
}

/// Hand-assembled expected payload for an adopted/windowless Scanish frame
/// whose ranges landed at `ranges_off` and string right after.
fn expected_payload(
    total: usize,
    angle_min: f32,
    ranges: &[f32],
    ranges_off: usize,
    frame_id: &str,
    frame_id_off: usize,
) -> Vec<u8> {
    let mut e = vec![0u8; total];
    e[0..4].copy_from_slice(&angle_min.to_le_bytes());
    e[C_FIXED..C_FIXED + 4].copy_from_slice(&(ranges_off as u32).to_le_bytes());
    e[C_FIXED + 4..C_FIXED + 8].copy_from_slice(&((ranges.len() * 4) as u32).to_le_bytes());
    e[C_FIXED + 8..C_FIXED + 12].copy_from_slice(&(frame_id_off as u32).to_le_bytes());
    e[C_FIXED + 12..C_FIXED + 16].copy_from_slice(&(frame_id.len() as u32).to_le_bytes());
    for (i, r) in ranges.iter().enumerate() {
        e[ranges_off + 4 * i..ranges_off + 4 * i + 4].copy_from_slice(&r.to_le_bytes());
    }
    e[frame_id_off..frame_id_off + frame_id.len()].copy_from_slice(frame_id.as_bytes());
    e
}

/// Fill the borrowed struct the way a bump-backed stock fill leaves it:
/// sequence bytes bumped into the window, string on the heap.
/// Returns the PAYLOAD-RELATIVE offset the bump placed the sequence at —
/// the bump aligns the ABSOLUTE address to 16 (as glibc `malloc` does), so
/// the first placement is `align_up(payload + tail_off, 16) - payload`,
/// which is `tail_off + 8` whenever the payload base is 8-but-not-16
/// aligned (the iceoryx2 chunk shape). Oracles derive placements from this
/// return, never from the frame under test.
unsafe fn fill_adopted(msg: *mut c_void, angle: f32, ranges: &[f32], frame_id: &str) -> usize {
    let m = &mut *(msg as *mut CScanish);
    m.angle_min = angle;
    let dst = fake_bump(ranges.len() * 4, 16).expect("fill fits the window");
    std::ptr::copy_nonoverlapping(ranges.as_ptr(), dst as *mut f32, ranges.len());
    m.ranges = CF32Seq {
        data: dst as *mut f32,
        size: ranges.len(),
        capacity: ranges.len(),
        #[cfg(cerulion_has_is_rosidl_buffer)]
        is_rosidl_buffer: false,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        owns_rosidl_buffer: false,
    };
    set_heap_string(&mut m.frame_id, frame_id);
    dst - msg as usize
}

/// Fill with HEAP-backed sequence storage (the escaped shape).
unsafe fn fill_escaped(msg: *mut c_void, angle: f32, ranges: &[f32], frame_id: &str) {
    let m = &mut *(msg as *mut CScanish);
    m.angle_min = angle;
    let dst = calloc(ranges.len().max(1), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(ranges.as_ptr(), dst, ranges.len());
    m.ranges = CF32Seq {
        data: dst,
        size: ranges.len(),
        capacity: ranges.len(),
        #[cfg(cerulion_has_is_rosidl_buffer)]
        is_rosidl_buffer: false,
        #[cfg(cerulion_has_is_rosidl_buffer)]
        owns_rosidl_buffer: false,
    };
    set_heap_string(&mut m.frame_id, frame_id);
}

unsafe fn set_heap_string(s: &mut CRosString, value: &str) {
    if !s.data.is_null() {
        free(s.data as *mut c_void);
    }
    s.data = calloc(value.len() + 1, 1) as *mut u8;
    std::ptr::copy_nonoverlapping(value.as_ptr(), s.data, value.len());
    s.size = value.len();
    s.capacity = value.len() + 1;
}

// =====================================================================
// The arms
// =====================================================================

#[test]
#[serial]
fn windowed_borrow_adopts_the_fill_and_publishes_zero_copy() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, subscription, pdata) = setup_pair("adopt");
        assert!(
            (*publisher).can_loan_messages,
            "an unbounded type with an Active hook must offer the borrow"
        );

        let ranges = [1.5f32, -2.25, 3.0, 1.0e-3];
        let frame_id = "lidar_link";
        let tail = tail_off();

        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_OK
        );
        assert!(!msg.is_null());
        // The window was armed over the slot tail (geometry oracle), and
        // the message was CONSTRUCTED: the init'd string.
        let w = fake_window_of_current_thread().expect("window armed at borrow");
        assert_eq!(
            w.base,
            msg as usize + tail,
            "window base = payload + tail_off"
        );
        assert!(w.limit > w.base, "a real fill tail was reserved");
        let m = &*(msg as *const CScanish);
        assert!(
            !m.frame_id.data.is_null() && m.frame_id.size == 0,
            "the typesupport init ran in the slot"
        );

        let ranges_off = fill_adopted(msg, -0.75, &ranges, frame_id);
        // The bump's documented placement rule (absolute 16-alignment).
        assert_eq!(
            msg as usize + ranges_off,
            align_up(msg as usize + tail, 16),
            "first bump lands at the 16-aligned window base"
        );
        assert_eq!(
            rmw_publish_loaned_message(publisher, msg, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert!(
            fake_window_of_current_thread().is_none(),
            "publish disarms the window"
        );

        // Principle-#3 counters are the zero-copy oracle (the frame bytes
        // alone cannot discriminate adopt from copy).
        assert_eq!((*pdata).borrow_adopted_count(), 1, "fully adopted");
        assert_eq!((*pdata).borrow_copied_count(), 0);
        assert_eq!((*pdata).borrow_degrade_count(), 0);

        // The frame on the wire: gap-frame shape, byte-exact.
        let (header, payload) = take_raw(subscription);
        assert_eq!(header.sequence, 0, "first publish");
        assert_eq!(header.offset_table_offset as usize, C_FIXED);
        assert_eq!(header.offset_table_count, 2);
        assert!(header.timestamp_ns > 0);
        let frame_id_off = ranges_off + ranges.len() * 4;
        let total = frame_id_off + frame_id.len();
        assert_eq!(payload.len(), total);
        let expected = expected_payload(total, -0.75, &ranges, ranges_off, frame_id, frame_id_off);
        assert_eq!(payload, expected, "whole payload vs hand oracle");

        // A second identical borrow+fill publishes to ITS OWN hand oracle
        // (header seq/ts differ by design; the slot's absolute alignment
        // decides the placements, so each frame is pinned to the oracle
        // derived from its own slot).
        let mut msg2: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg2),
            RMW_RET_OK
        );
        let ranges_off2 = fill_adopted(msg2, -0.75, &ranges, frame_id);
        assert_eq!(
            rmw_publish_loaned_message(publisher, msg2, std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (header2, payload2) = take_raw(subscription);
        assert_eq!(header2.sequence, 1, "sequences are gap-free");
        let frame_id_off2 = ranges_off2 + ranges.len() * 4;
        let expected2 = expected_payload(
            frame_id_off2 + frame_id.len(),
            -0.75,
            &ranges,
            ranges_off2,
            frame_id,
            frame_id_off2,
        );
        assert_eq!(payload2, expected2, "second frame vs its own hand oracle");
        assert_eq!((*pdata).borrow_adopted_count(), 2);
    }
}

#[test]
#[serial]
fn escaped_fill_publishes_a_correct_copy_loudly_counted() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, subscription, pdata) = setup_pair("esc");
        let ranges = [9.0f32, 8.0, 7.0];

        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_OK
        );
        // The fill escaped the window entirely (heap storage).
        fill_escaped(msg, 0.5, &ranges, "esc_frame");
        assert_eq!(
            rmw_publish_loaned_message(publisher, msg, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert_eq!((*pdata).borrow_adopted_count(), 0);
        assert_eq!((*pdata).borrow_copied_count(), 1, "the escape paid a copy");
        assert_eq!(
            (*pdata).borrow_degrade_count(),
            1,
            "the degrade is latched + counted"
        );

        // The frame is CORRECT — the same bytes the copy path serves.
        let (_, payload) = take_raw(subscription);
        let tail = tail_off();
        let expected = expected_payload(
            tail + ranges.len() * 4 + "esc_frame".len(),
            0.5,
            &ranges,
            tail,
            "esc_frame",
            tail + ranges.len() * 4,
        );
        assert_eq!(payload, expected);
    }
}

#[test]
#[serial]
fn second_borrow_on_one_thread_is_windowless_and_both_publish_correctly() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, subscription, pdata) = setup_pair("two");
        let tail = tail_off();

        // Borrow A arms the thread's window.
        let mut a: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut a),
            RMW_RET_OK
        );
        let wa = fake_window_of_current_thread().expect("A armed");
        assert_eq!(wa.base, a as usize + tail);

        // Borrow B: the thread's window is taken — WINDOWLESS, loudly.
        let mut b: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut b),
            RMW_RET_OK
        );
        assert_eq!(
            (*pdata).borrow_degrade_count(),
            1,
            "the windowless borrow is latched at borrow time"
        );
        let still = fake_window_of_current_thread().expect("A's window untouched");
        assert_eq!(
            (still.base, still.limit),
            (wa.base, wa.limit),
            "borrow B must never disarm A's live window"
        );

        // Fill + publish B (heap fill — windowless): correct copy.
        let rb = [4.0f32, 5.0];
        fill_escaped(b, 2.0, &rb, "b");
        assert_eq!(
            rmw_publish_loaned_message(publisher, b, std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (_, payload_b) = take_raw(subscription);
        assert_eq!(
            payload_b,
            expected_payload(tail + 8 + 1, 2.0, &rb, tail, "b", tail + 8)
        );
        assert!(
            fake_window_of_current_thread().is_some(),
            "B's publish must not disarm A's window either"
        );

        // Fill + publish A (bump fill): adopted.
        let ra = [1.0f32];
        let a_off = fill_adopted(a, 3.0, &ra, "a");
        assert_eq!(
            rmw_publish_loaned_message(publisher, a, std::ptr::null_mut()),
            RMW_RET_OK
        );
        let (_, payload_a) = take_raw(subscription);
        assert_eq!(
            payload_a,
            expected_payload(a_off + 4 + 1, 3.0, &ra, a_off, "a", a_off + 4)
        );
        assert_eq!((*pdata).borrow_adopted_count(), 1);
        assert_eq!((*pdata).borrow_copied_count(), 1);
    }
}

#[test]
#[serial]
fn return_releases_the_loan_and_slot_reuse_retires_the_quarantine() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, _subscription, pdata) = setup_pair("ret");
        let tail = tail_off();

        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_OK
        );
        let tb1 = msg as usize + tail;
        assert_eq!(
            rmw_return_loaned_message_from_publisher(publisher, msg),
            RMW_RET_OK
        );
        assert!(
            fake_window_of_current_thread().is_none(),
            "return disarms the window"
        );
        assert!(
            fake().lock().expect("fake").retired.is_empty(),
            "the quarantine extent must OUTLIVE the loan (retire waits for reuse)"
        );

        // The next borrow: if the pool re-issues the same slot, the old
        // extent is retired BEFORE re-arming; a fresh slot leaves it
        // outstanding (both valid outcomes of retire-on-reuse).
        let mut msg2: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg2),
            RMW_RET_OK
        );
        let tb2 = msg2 as usize + tail;
        let retired = fake().lock().expect("fake").retired.clone();
        if tb2 == tb1 {
            assert_eq!(retired, vec![tb1], "reused slot ⇒ old extent retired");
        } else {
            assert!(
                retired.is_empty(),
                "fresh slot ⇒ the old extent still awaits ITS slot's reuse"
            );
        }
        // Clean finish: publish the second borrow.
        fill_adopted(msg2, 1.0, &[2.0f32], "r");
        assert_eq!(
            rmw_publish_loaned_message(publisher, msg2, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert_eq!((*pdata).borrow_adopted_count(), 1);
    }
}

#[test]
#[serial]
fn without_a_hook_the_type_keeps_the_copy_path_and_borrow_is_unsupported() {
    reset_fake();
    // NO TestHookGuard: the process handshake resolves Absent (the hook is
    // a Linux LD_PRELOAD payload; none is preloaded into a test binary).
    unsafe {
        let (publisher, subscription, pdata) = setup_pair("nohook");
        assert!(
            !(*publisher).can_loan_messages,
            "no hook ⇒ the copy-path surface exactly"
        );
        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_UNSUPPORTED
        );
        assert!(msg.is_null());
        assert_eq!(
            (*pdata).borrow_degrade_count(),
            0,
            "a degrade, not a latch storm"
        );

        // The plain publish path is untouched — the always-sound floor.
        let ranges = [6.5f32, 6.0];
        let mut m: CScanish = std::mem::zeroed();
        scanish_init(&mut m as *mut _ as *mut c_void, 0);
        m.angle_min = -1.0;
        let heap = calloc(2, 4) as *mut f32;
        std::ptr::copy_nonoverlapping(ranges.as_ptr(), heap, 2);
        m.ranges = CF32Seq {
            data: heap,
            size: 2,
            capacity: 2,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        };
        set_heap_string(&mut m.frame_id, "plain");
        assert_eq!(
            rmw_publish(
                publisher,
                &m as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        scanish_fini(&mut m as *mut _ as *mut c_void);
        let (_, payload) = take_raw(subscription);
        // The plain flatten packs tightly at the floor.
        assert_eq!(
            payload,
            expected_payload(
                C_FLOOR + 8 + 5,
                -1.0,
                &ranges,
                C_FLOOR,
                "plain",
                C_FLOOR + 8
            )
        );
    }
}

#[test]
#[serial]
fn destroying_with_a_cross_thread_armed_window_leaks_the_slot_never_releases_it() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, _subscription, _pdata) = setup_pair("xleak");
        let ts_addr = ts_ptr() as usize;
        let pub_addr = publisher as usize;
        // Borrow on a SPAWNED thread T: T's window arms over the slot.
        let msg_addr = std::thread::spawn(move || {
            let mut msg: *mut c_void = std::ptr::null_mut();
            assert_eq!(
                rmw_borrow_loaned_message(
                    pub_addr as *const ffi::rmw_publisher_t,
                    ts_addr as *const ffi::rosidl_message_type_support_t,
                    &mut msg,
                ),
                RMW_RET_OK
            );
            msg as usize
        })
        .join()
        .expect("borrow thread");
        assert_eq!(
            fake().lock().expect("fake").windows.len(),
            1,
            "T's window is armed"
        );
        // Wrong-thread RETURN from main ⇒ the slot is orphan-HELD (T's
        // window cannot be disarmed from here).
        assert_eq!(
            rmw_return_loaned_message_from_publisher(publisher, msg_addr as *mut c_void),
            RMW_RET_OK
        );
        // DESTROY on main: releasing the held slot would let the pool
        // recycle it under T's still-armed window (the stale-window
        // use-after-free class) — it must be LEAKED instead, counted.
        let before = rmw_cerulion::borrow_destroy_leak_count();
        assert_eq!(
            rmw_destroy_publisher(node_ptr(), publisher as *mut ffi::rmw_publisher_t),
            RMW_RET_OK
        );
        assert_eq!(
            rmw_cerulion::borrow_destroy_leak_count(),
            before + 1,
            "the cross-thread-armed slot is leaked, never released"
        );
        assert_eq!(
            fake().lock().expect("fake").windows.len(),
            1,
            "T's window is untouched — nobody else may disarm it"
        );
        // The leaked slot's PUBLISHER stays registered too — the
        // forgotten sample pins the port's on-disk tag, and a deregistered
        // port with a live tag wedges every later dead-node sweep of this
        // process's node (dropping the `PublisherData` here makes this
        // read `Some(0)`; the cross-process half of that same check lives in
        // `rmw_leak_at_destroy_registry_test`).
        assert_eq!(
            live_publisher_ports(),
            Some(1),
            "the leaked publisher's iceoryx2 port must STAY registered on the topic"
        );
        // The documented cost, pinned exactly: rmw topics are provisioned at
        // iceoryx2's default `max_publishers = 2` (`default_topic_config`
        // leaves the cap unset and declares `External`, so no single-writer
        // pre-check fires), so a re-create on the same ROS topic still
        // succeeds ONLY while the topic's other slot is free — here it is
        // (the leaked port holds one of the two) — and it takes that slot;
        // the NEXT is refused with a NULL publisher
        // (`ExceedsMaxSupportedPublishers` inside the transport, logged as
        // `publisher creation failed`). With another live publisher already
        // on the topic, the FIRST re-create would be the refused one.
        let topic = CString::new(ros_topic()).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let second = rmw_create_publisher(node_ptr(), ts_ptr(), topic.as_ptr(), &qos, &pub_opts);
        assert!(
            !second.is_null(),
            "with the topic's other slot free, one re-create on the leaked topic still \
             succeeds (it takes the second of the two slots)"
        );
        assert_eq!(
            live_publisher_ports(),
            Some(2),
            "the leaked port + the re-created port both hold slots"
        );
        let third = rmw_create_publisher(node_ptr(), ts_ptr(), topic.as_ptr(), &qos, &pub_opts);
        assert!(
            third.is_null(),
            "a third publisher is refused (NULL): the leaked port holds one of the topic's \
             two slots for the life of this process"
        );
        // A CLEAN destroy releases ITS slot; the leaked port's stays held.
        assert_eq!(rmw_destroy_publisher(node_ptr(), second), RMW_RET_OK);
        assert_eq!(
            live_publisher_ports(),
            Some(1),
            "the re-created publisher's clean destroy released its slot; the leaked \
             port's slot is held for the life of the process"
        );
    }
}

#[test]
#[serial]
fn hook_counters_are_readable_through_the_consumer_and_absent_without_a_hook() {
    reset_fake();
    {
        let _hook = TestHookGuard::install(fake_hook_api());
        // The consumer read the shutdown line surfaces (the hook's
        // counters' first shipped reader) — against the fake's distinct
        // per-kind table.
        let got = rmw_cerulion::heaphook::hook_counters().expect("active hook");
        assert_eq!(
            got,
            [
                ("release_without_callback", Some(100)),
                ("bootstrap_exhausted", Some(101)),
                ("pre_resolution_leak", Some(102)),
                ("quarantine_noop_frees", Some(103)),
                ("tombstone_hits", Some(104)),
                ("atfork_interval_leaks", Some(105)),
            ]
        );
        // The shutdown surface runs the same read (its log line is the
        // operator half; the read path is the pin here).
        rmw_cerulion::heaphook::log_hook_counters();
    }
    assert!(
        rmw_cerulion::heaphook::hook_counters().is_none(),
        "no hook ⇒ no counters claimed (never fabricated zeros)"
    );
}

// Mixed-deployment counter sentinel: an OLDER v2 hook — a stale
// `.so` built before counter kinds 4/5 existed — answers its documented
// unknown-index sentinel (`u64::MAX`) for them while serving kinds 0-3
// normally. The consumer must treat MAX as "this hook does not serve this
// kind" and OMIT those kinds from its report (`None` per kind; the log line
// renders `unavailable`) while kinds 0-3 still render — never log MAX
// (18446744073709551615) as a real tombstone/atfork count. Reverting
// `hook_counters` to pass the raw u64 through makes kinds 4/5 read
// `Some(u64::MAX)` → the oracle goes red.
#[test]
#[serial]
fn a_stale_v2_hooks_unserved_counter_kinds_degrade_to_unavailable() {
    reset_fake();
    let _hook = TestHookGuard::install(HookApi {
        counter: f_counter_stale_v2,
        ..fake_hook_api()
    });
    let got = rmw_cerulion::heaphook::hook_counters().expect("active hook");
    assert_eq!(
        got,
        [
            ("release_without_callback", Some(100)),
            ("bootstrap_exhausted", Some(101)),
            ("pre_resolution_leak", Some(102)),
            ("quarantine_noop_frees", Some(103)),
            ("tombstone_hits", None),
            ("atfork_interval_leaks", None),
        ],
        "kinds the stale hook serves still render; the sentinel kinds are \
         omitted, never reported as a u64::MAX count"
    );
    // The operator half — the log line rendering the sentinel kinds as
    // `unavailable`, never a raw u64::MAX — is pinned by
    // `rmw_heaphook_counter_log_test.rs` (its own `#[traced_test]` binary,
    // the rmw_schema_mismatch_test precedent; THIS binary installs no
    // tracing subscriber, so calling the fn here would assert nothing).
}

#[test]
#[serial]
fn wrong_thread_publish_copies_holds_the_slot_and_the_borrow_thread_self_heals() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, subscription, pdata) = setup_pair("xthr");
        let tail = tail_off();

        // Borrow + bump-fill on THIS thread.
        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_OK
        );
        let ranges = [7.0f32, 7.5];
        fill_adopted(msg, 4.0, &ranges, "xt");

        // Publish from ANOTHER thread: must still ship a correct frame
        // (copied — the borrow thread's window cannot be disarmed there).
        let pub_addr = publisher as usize;
        let msg_addr = msg as usize;
        let ret = std::thread::spawn(move || {
            let publisher = pub_addr as *const ffi::rmw_publisher_t;
            rmw_publish_loaned_message(publisher, msg_addr as *mut c_void, std::ptr::null_mut())
        })
        .join()
        .expect("publisher thread");
        assert_eq!(ret, RMW_RET_OK);
        assert_eq!((*pdata).borrow_copied_count(), 1, "wrong thread ⇒ copy");
        assert_eq!((*pdata).borrow_degrade_count(), 1);
        let (_, payload) = take_raw(subscription);
        // The wrong-thread fallback FLATTENS: a tight frame packed at the
        // data floor (the same bytes the plain publish serves) — not the
        // tail-placed gap-frame shape.
        assert_eq!(
            payload,
            expected_payload(C_FLOOR + 8 + 2, 4.0, &ranges, C_FLOOR, "xt", C_FLOOR + 8)
        );
        assert!(
            fake_window_of_current_thread().is_some(),
            "the borrow thread's window is still armed (nobody else may disarm it)"
        );

        // The borrow thread SELF-HEALS on its next borrow: the stale window
        // is disarmed, the held slot released, and a fresh window armed.
        let mut msg2: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg2),
            RMW_RET_OK
        );
        let w = fake_window_of_current_thread().expect("fresh window armed");
        assert_eq!(w.base, msg2 as usize + tail, "the NEW loan owns the window");
        fill_adopted(msg2, 5.0, &[1.25f32], "ok");
        assert_eq!(
            rmw_publish_loaned_message(publisher, msg2, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert_eq!((*pdata).borrow_adopted_count(), 1, "zero-copy restored");
    }
}

/// Lifecycle: an orphaned slot charges
/// the publisher's loan budget, so the owner-thread self-heal must run
/// BEFORE the fresh loan is requested — run after it, a budget-exhausted borrow
/// fails `BAD_ALLOC` before the heal can ever run, and the stale window
/// plus its slot are held forever. The arm builds exactly that state:
/// one orphan (owner = MAIN) plus three held loans = the full budget of 4.
#[test]
#[serial]
fn a_budget_exhausted_borrow_self_heals_by_releasing_the_orphan_first() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, subscription, pdata) = setup_pair("xbud");

        // (a) Borrow on MAIN (the owner window arms here), then RETURN it
        //     from a spawned thread — wrong-thread ⇒ the slot is
        //     orphan-HELD against MAIN, still charging the loan budget.
        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_OK
        );
        let pub_addr = publisher as usize;
        let msg_addr = msg as usize;
        std::thread::spawn(move || {
            assert_eq!(
                rmw_return_loaned_message_from_publisher(
                    pub_addr as *const ffi::rmw_publisher_t,
                    msg_addr as *mut c_void,
                ),
                RMW_RET_OK
            );
        })
        .join()
        .expect("return thread");
        assert!(
            fake_window_of_current_thread().is_some(),
            "MAIN's window is still armed over the orphaned slot"
        );
        // The wrong-thread RETURN is a slot-hold lifecycle event, never a
        // publish degrade: nothing was published or copied, so the
        // publish-degrade latch must not move — it rides its own counter.
        assert_eq!(
            (*pdata).borrow_degrade_count(),
            0,
            "a return-only event must not count as a publish degrade"
        );
        assert_eq!(
            (*pdata).borrow_wrong_thread_return_count(),
            1,
            "the wrong-thread return rides its own lifecycle counter"
        );

        // (b) Fill the REST of the budget with three HELD loans on another
        //     thread (the first arms that thread's window; the next two
        //     degrade windowless — all three charge the budget), then the
        //     anti-vacuity half: a FOURTH borrow there really fails.
        let ts_addr = ts_ptr() as usize;
        std::thread::spawn(move || {
            let publisher = pub_addr as *const ffi::rmw_publisher_t;
            let ts = ts_addr as *const ffi::rosidl_message_type_support_t;
            for _ in 0..3 {
                let mut m: *mut c_void = std::ptr::null_mut();
                assert_eq!(rmw_borrow_loaned_message(publisher, ts, &mut m), RMW_RET_OK);
            }
            let mut m: *mut c_void = std::ptr::null_mut();
            assert_eq!(
                rmw_borrow_loaned_message(publisher, ts, &mut m),
                RMW_RET_BAD_ALLOC,
                "anti-vacuity: the budget really is exhausted (orphan + 3 held)"
            );
        })
        .join()
        .expect("budget thread");

        // (c) MAIN borrows again at the exhausted budget: the self-heal
        //     must release MAIN's orphan (disarming its stale window)
        //     BEFORE the loan, which is exactly what makes the loan
        //     possible. With the heal after the loan: BAD_ALLOC here, heal never runs.
        let mut healed: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut healed),
            RMW_RET_OK,
            "the owner-thread borrow self-heals by releasing its orphan first"
        );
        // The healed borrow is fully usable end to end: a FRESH window
        // armed (the stale one was disarmed by the sweep), fill adopts,
        // publish ships the byte-exact gap frame.
        let ranges = [3.25f32, -1.5];
        let ranges_off = fill_adopted(healed, 2.0, &ranges, "bh");
        assert_eq!(
            rmw_publish_loaned_message(publisher, healed, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert_eq!((*pdata).borrow_adopted_count(), 1, "healed borrow adopts");
        let (_header, payload) = take_raw(subscription);
        let fid_off = ranges_off + ranges.len() * 4;
        let expected = expected_payload(fid_off + 2, 2.0, &ranges, ranges_off, "bh", fid_off);
        assert_eq!(payload, expected, "healed frame vs hand oracle");
    }
}

/// Memory safety: a panic that POISONS
/// the publisher's loan bookkeeping must not turn destroy into a slot
/// release — a `lock_unpoisoned` refusal that skips the whole
/// teardown block lets the `Box` drop return every held slot to the pool
/// while a borrow window is still armed over one (the stale-window
/// use-after-free class), with ZERO counter evidence.
///
/// The held loan is deliberately borrowed on the DESTROY thread:
/// a cross-thread loan leaks on the ORDINARY arm too, so it cannot
/// discriminate the poisoned discipline — an implementation that ignores
/// the poisoned flag but handles cross-thread leaks would pass. An
/// OWN-thread windowed loan flips outcome with the flag: unpoisoned it
/// would fini + release (counter flat); poisoned it must be LEAKED
/// (trust nothing logically) — and "trust nothing" includes the
/// ownership inference, so the thread's armed window is PRESERVED (no
/// disarm rides torn tables: a panic can leave an armed window absent
/// from them or an entry stale). The poison-safe diagnostic accessor is
/// pinned here too (it must answer, not abort, on a poisoned publisher).
#[test]
#[serial]
fn a_poisoned_publisher_still_leaks_armed_slots_at_destroy() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, _subscription, pdata) = setup_pair("xpoi");

        // Borrow on THIS thread (the destroy thread) and HOLD the loan:
        // our window arms over the slot.
        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_OK
        );
        assert!(
            fake_window_of_current_thread().is_some(),
            "the destroy thread's window is armed"
        );

        // POISON the loan bookkeeping: a panic while holding the inner
        // mutex (the state any panicking borrow/seal leaves behind).
        let pdata_addr = pdata as usize;
        let poisoner = std::thread::spawn(move || {
            let data = &*(pdata_addr as *const PublisherData);
            let _guard = data.inner.lock().expect("not yet poisoned");
            panic!("poison the publisher's loan bookkeeping");
        })
        .join();
        assert!(poisoner.is_err(), "the poisoner must have panicked");
        assert!(
            (*pdata).inner.lock().is_err(),
            "precondition: the mutex is really poisoned"
        );
        // The test-seam diagnostic must ANSWER on a poisoned publisher
        // (poison-safe `into_inner`), never abort the reader — and the
        // capacities are structurally valid data, so the reserve still
        // reads true.
        let (pend_cap, _, _) = (*pdata).loan_bookkeeping_capacities();
        assert!(
            pend_cap >= 4,
            "the diagnostic accessor answers (poison-safely) after a panic"
        );

        // DESTROY from the owning thread: the poisoned arm must LEAK the
        // own-thread windowed loan (counted) — the unpoisoned
        // classification would have fini'd + released it, counter flat —
        // and must PRESERVE the armed window (no disarm rides torn
        // tables). If the whole block is skipped, the counter
        // stays flat, and the slot goes back to the pool under the
        // armed window.
        let before = rmw_cerulion::borrow_destroy_leak_count();
        assert_eq!(
            rmw_destroy_publisher(node_ptr(), publisher as *mut ffi::rmw_publisher_t),
            RMW_RET_OK
        );
        assert_eq!(
            rmw_cerulion::borrow_destroy_leak_count(),
            before + 1,
            "the poisoned path leaks (and counts) even an OWN-thread armed \
             slot — the flag, not the thread test, must drive the leak"
        );
        assert!(
            fake_window_of_current_thread().is_some(),
            "the poisoned arm PRESERVES the armed window — ownership \
             inference from torn tables must not drive a disarm"
        );
        // The poisoned leak arm keeps the publisher's port
        // registered too (the leak decision, not the poison, drives it).
        assert_eq!(
            live_publisher_ports(),
            Some(1),
            "the leaked publisher's iceoryx2 port must STAY registered on the topic"
        );
    }
}

/// Mirror of the pending-loan poisoned arm for the ORPHAN drain's own
/// `poisoned` disjunct: an
/// OWN-thread orphan flips outcome with the flag exactly like an
/// own-thread pending loan — unpoisoned destroy would release it (drop,
/// counter flat), the poisoned discipline must LEAK it (trust nothing
/// logically), counted, with the thread's armed window PRESERVED (no
/// disarm rides torn tables).
#[test]
#[serial]
fn a_poisoned_publishers_own_thread_orphan_leaks_at_destroy_not_releases() {
    reset_fake();
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        let (publisher, _subscription, pdata) = setup_pair("xpor");

        // Build an OWN-thread orphan: borrow on THIS thread (window arms
        // here), then RETURN it from a spawned thread — wrong-thread ⇒
        // the slot is orphan-HELD against THIS thread, window still armed.
        let mut msg: *mut c_void = std::ptr::null_mut();
        assert_eq!(
            rmw_borrow_loaned_message(publisher, ts_ptr(), &mut msg),
            RMW_RET_OK
        );
        let pub_addr = publisher as usize;
        let msg_addr = msg as usize;
        std::thread::spawn(move || {
            assert_eq!(
                rmw_return_loaned_message_from_publisher(
                    pub_addr as *const ffi::rmw_publisher_t,
                    msg_addr as *mut c_void,
                ),
                RMW_RET_OK
            );
        })
        .join()
        .expect("return thread");
        assert!(
            fake_window_of_current_thread().is_some(),
            "this thread's window is still armed over the orphaned slot"
        );

        // POISON the loan bookkeeping.
        let pdata_addr = pdata as usize;
        let poisoner = std::thread::spawn(move || {
            let data = &*(pdata_addr as *const PublisherData);
            let _guard = data.inner.lock().expect("not yet poisoned");
            panic!("poison the publisher's loan bookkeeping");
        })
        .join();
        assert!(poisoner.is_err(), "the poisoner must have panicked");
        assert!(
            (*pdata).inner.lock().is_err(),
            "precondition: the mutex is really poisoned"
        );

        // DESTROY from the orphan's owner thread: the poisoned arm must
        // LEAK the own-thread orphan (counted) — unpoisoned it would
        // have been dropped (released), counter flat — and must PRESERVE
        // the armed window (no disarm rides torn tables).
        let before = rmw_cerulion::borrow_destroy_leak_count();
        assert_eq!(
            rmw_destroy_publisher(node_ptr(), publisher as *mut ffi::rmw_publisher_t),
            RMW_RET_OK
        );
        assert_eq!(
            rmw_cerulion::borrow_destroy_leak_count(),
            before + 1,
            "the poisoned path leaks (and counts) even an OWN-thread \
             orphan — the flag, not the thread test, must drive the leak"
        );
        assert!(
            fake_window_of_current_thread().is_some(),
            "the poisoned arm PRESERVES the armed window — ownership \
             inference from torn tables must not drive a disarm"
        );
        // The orphan drain's poisoned leak arm keeps the
        // publisher's port registered too.
        assert_eq!(
            live_publisher_ports(),
            Some(1),
            "the leaked publisher's iceoryx2 port must STAY registered on the topic"
        );
    }
}
