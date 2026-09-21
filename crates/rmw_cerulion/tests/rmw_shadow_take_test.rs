// SPDX-License-Identifier: AGPL-3.0-only
//! Real transport: the FORGED loaned take through the
//! EXACT `extern "C"` surface rcl calls, over REAL iceoryx2 shared memory
//! — the sample is HELD across the C ABI and the shadow's forged
//! `std::vector` / rosidl sequence aims into it.
//!
//! ⚠️ iceoryx2 shared memory is a process singleton and this binary owns a
//! `#[global_allocator]` probe — run with `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_shadow_take_test -- --test-threads=1
//! ```
//!
//! Hand oracles throughout. The headline pin is an ADDRESS-RANGE oracle —
//! the forged `data()` pointer lies inside the held sample's own bytes
//! (`SubscriptionInner::pending_takes[..].sample.payload()`), which is what
//! "zero copies" means and what a copy could not fake (a copy is heap, and
//! the heap is not the sample). The allocation probe pins the OTHER half of
//! the cost claim, narrowly scoped: the rmw's own take path allocates
//! NOTHING on the Rust heap at steady state (shadow recycled, pending-take
//! table pre-reserved). It cannot see the copying take's libc / C++
//! allocations — those never go through Rust's global allocator — so it is
//! not offered as the zero-copy proof; the address oracle is.
//!
//! The read-only contract is pinned in a CHILD PROCESS: a write through the
//! forged payload pointer must terminate it by signal (SIGSEGV on Linux,
//! SIGBUS on macOS — iceoryx2 opens subscriber data segments
//! `AccessMode::Read`), never return. A child that exits normally means the
//! contract is false, which is exactly what should fail loudly here.

#![cfg(unix)]

use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use rmw_cerulion::ffi::introspection_cpp::{
    rmw_cerulion_cppstring_construct, rmw_cerulion_cppstring_destruct, rmw_cerulion_cppstring_view,
    rmw_cerulion_vector_u8_capacity, rmw_cerulion_vector_u8_construct, rmw_cerulion_vector_u8_data,
    rmw_cerulion_vector_u8_destruct, rmw_cerulion_vector_u8_size, CppMessageMember,
    CppMessageMembers,
};
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::runtime::SubscriptionData;
use rmw_cerulion::*;

// =====================================================================
// Allocation probe (Rust global allocator only — see the module docs)
// =====================================================================

struct CountingAllocator {
    inner: System,
    count: AtomicU64,
    bytes: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            count: AtomicU64::new(0),
            bytes: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }
    fn enable(&self) {
        self.count.store(0, Ordering::SeqCst);
        self.bytes.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    /// (allocations, bytes) requested since `enable`.
    fn disable(&self) -> (u64, u64) {
        self.enabled.store(false, Ordering::SeqCst);
        (
            self.count.load(Ordering::SeqCst),
            self.bytes.load(Ordering::SeqCst),
        )
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            self.count.fetch_add(1, Ordering::Relaxed);
            self.bytes
                .fetch_add(layout.size() as u64, Ordering::Relaxed);
        }
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            self.count.fetch_add(1, Ordering::Relaxed);
            self.bytes.fetch_add(new_size as u64, Ordering::Relaxed);
        }
        unsafe { self.inner.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

// =====================================================================
// Shared scaffolding (crib of rmw_e2e_test.rs)
// =====================================================================

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
    /// `pub(crate)` in the crate on purpose; a fixture binds it itself.
    fn rmw_cerulion_vector_u8_assign(v: *mut c_void, data: *const u8, len: usize);
}

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_UINT8: u8 = 8;
const ROS_TYPE_UINT32: u8 = 12;
const ROS_TYPE_INT32: u8 = 13;
const ROS_TYPE_STRING: u8 = 16;
const ROS_TYPE_MESSAGE: u8 = 18;

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

unsafe fn setup_node(name: &str) -> *mut ffi::rmw_node_t {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let node = rmw_create_node(context, cstr(name), cstr("/"));
    assert!(!node.is_null(), "node creation failed");
    node
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

/// Node + subscription + publisher on one topic.
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
    let topic = CString::new(format!("/rmw_shadow/{tag}/{suffix}")).expect("topic");
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

unsafe fn take_loaned(
    subscription: *const ffi::rmw_subscription_t,
) -> (ffi::rmw_ret_t, bool, *mut c_void, ffi::rmw_message_info_t) {
    let mut taken = false;
    let mut out: *mut c_void = std::ptr::null_mut();
    let mut info: ffi::rmw_message_info_t = std::mem::zeroed();
    let ret = rmw_take_loaned_message_with_info(
        subscription,
        &mut out,
        &mut taken,
        &mut info,
        std::ptr::null_mut(),
    );
    (ret, taken, out, info)
}

unsafe fn sub_data(subscription: *const ffi::rmw_subscription_t) -> &'static SubscriptionData {
    &*((*subscription).data as *const SubscriptionData)
}

/// The held sample's address range for the ONE outstanding loan keyed by
/// `key` — the zero-copy oracle.
unsafe fn held_sample_range(
    subscription: *const ffi::rmw_subscription_t,
    key: *mut c_void,
) -> std::ops::Range<usize> {
    let data = sub_data(subscription);
    let inner = data.inner.lock().unwrap_or_else(|e| e.into_inner());
    let take = inner
        .pending_takes
        .iter()
        .find(|t| t.key == key as usize)
        .expect("the loaned pointer must be an outstanding take");
    let payload = take.sample.payload();
    let start = payload.as_ptr() as usize;
    start..start + payload.len()
}

unsafe fn outstanding(subscription: *const ffi::rmw_subscription_t) -> usize {
    sub_data(subscription)
        .inner
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .pending_takes
        .len()
}

unsafe fn shadows_built_and_free(subscription: *const ffi::rmw_subscription_t) -> (usize, usize) {
    let inner = sub_data(subscription)
        .inner
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    (inner.shadows.built(), inner.shadows.free_count())
}

unsafe fn loan_has_shadow(subscription: *const ffi::rmw_subscription_t, key: *mut c_void) -> bool {
    let inner = sub_data(subscription)
        .inner
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    inner
        .pending_takes
        .iter()
        .find(|t| t.key == key as usize)
        .map(|t| t.shadow.is_some())
        .expect("outstanding take")
}

unsafe fn decode_failure_count(subscription: *const ffi::rmw_subscription_t) -> u64 {
    sub_data(subscription)
        .decode_failures
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .total_failures()
}

// =====================================================================
// C++ fixture: sensor_msgs/Image-shaped, REAL std::string + std::vector
// =====================================================================

#[repr(C, align(16))]
struct StringSlot([u8; 32]);
#[repr(C, align(8))]
struct VecU8Slot([usize; 3]);

#[repr(C)]
#[derive(Clone, Copy, Default, PartialEq, Debug)]
struct CppTime {
    sec: i32,
    nanosec: u32,
}

#[repr(C, align(16))]
struct CppHeader {
    stamp: CppTime,
    frame_id: StringSlot,
}

/// `header, height, width, encoding, is_bigendian, step, data` — the
/// `sensor_msgs/Image` member list, over real C++ containers.
#[repr(C, align(16))]
struct CppImage {
    header: CppHeader,
    height: u32,
    width: u32,
    encoding: StringSlot,
    is_bigendian: u8,
    step: u32,
    data: VecU8Slot,
}

static CPP_FINI_CALLS: AtomicU64 = AtomicU64::new(0);

unsafe extern "C" fn image_init(msg: *mut c_void, _init: u32) {
    let m = &mut *(msg as *mut CppImage);
    m.header.stamp = CppTime::default();
    rmw_cerulion_cppstring_construct(
        m.header.frame_id.0.as_mut_ptr() as *mut c_void,
        std::ptr::null(),
        0,
    );
    m.height = 0;
    m.width = 0;
    rmw_cerulion_cppstring_construct(
        m.encoding.0.as_mut_ptr() as *mut c_void,
        std::ptr::null(),
        0,
    );
    m.is_bigendian = 0;
    m.step = 0;
    rmw_cerulion_vector_u8_construct(m.data.0.as_mut_ptr() as *mut c_void);
}

unsafe extern "C" fn image_fini(msg: *mut c_void) {
    CPP_FINI_CALLS.fetch_add(1, Ordering::SeqCst);
    let m = &mut *(msg as *mut CppImage);
    rmw_cerulion_cppstring_destruct(m.header.frame_id.0.as_mut_ptr() as *mut c_void);
    rmw_cerulion_cppstring_destruct(m.encoding.0.as_mut_ptr() as *mut c_void);
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

fn cpp_members(
    ns: &str,
    name: &str,
    size_of: usize,
    members: Vec<CppMessageMember>,
    init: Option<unsafe extern "C" fn(*mut c_void, u32)>,
    fini: Option<unsafe extern "C" fn(*mut c_void)>,
) -> *const CppMessageMembers {
    let members = Box::leak(members.into_boxed_slice());
    Box::leak(Box::new(CppMessageMembers {
        message_namespace_: cstr(ns),
        message_name_: cstr(name),
        member_count_: members.len() as u32,
        size_of_: size_of,
        has_any_key_member_: false,
        members_: members.as_ptr(),
        init_function: init,
        fini_function: fini,
    }))
}

fn cpp_ts(members: *const CppMessageMembers) -> *const ffi::rosidl_message_type_support_t {
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_cpp"),
        data: members as *const c_void,
        ..Default::default()
    }))
}

/// The Image typesupport: unique type name per test so schema hashes never
/// collide across runs against the global SHM singleton.
fn image_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let time = cpp_members(
        "builtin_interfaces::msg",
        "Time",
        std::mem::size_of::<CppTime>(),
        vec![
            cpp_member("sec", ROS_TYPE_INT32, 0),
            cpp_member("nanosec", ROS_TYPE_UINT32, 4),
        ],
        None,
        None,
    );
    let mut stamp = cpp_member("stamp", ROS_TYPE_MESSAGE, 0);
    stamp.members_ = cpp_ts(time);
    let header = cpp_members(
        "std_msgs::msg",
        "Header",
        std::mem::size_of::<CppHeader>(),
        vec![
            stamp,
            cpp_member(
                "frame_id",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CppHeader, frame_id) as u32,
            ),
        ],
        None,
        None,
    );
    let mut hdr = cpp_member("header", ROS_TYPE_MESSAGE, 0);
    hdr.members_ = cpp_ts(header);
    let mut data = cpp_member(
        "data",
        ROS_TYPE_UINT8,
        std::mem::offset_of!(CppImage, data) as u32,
    );
    data.is_array_ = true;
    data.size_function = Some(vecu8_size);
    data.get_const_function = Some(vecu8_get_const);
    data.get_function = Some(vecu8_get);
    cpp_ts(cpp_members(
        "rmw_shadow::msg",
        unique,
        std::mem::size_of::<CppImage>(),
        vec![
            hdr,
            cpp_member(
                "height",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CppImage, height) as u32,
            ),
            cpp_member(
                "width",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CppImage, width) as u32,
            ),
            cpp_member(
                "encoding",
                ROS_TYPE_STRING,
                std::mem::offset_of!(CppImage, encoding) as u32,
            ),
            cpp_member(
                "is_bigendian",
                ROS_TYPE_UINT8,
                std::mem::offset_of!(CppImage, is_bigendian) as u32,
            ),
            cpp_member(
                "step",
                ROS_TYPE_UINT32,
                std::mem::offset_of!(CppImage, step) as u32,
            ),
            data,
        ],
        Some(image_init),
        Some(image_fini),
    ))
}

unsafe fn cpp_string_of(slot: *const c_void) -> String {
    let mut data: *const c_char = std::ptr::null();
    let mut len = 0usize;
    rmw_cerulion_cppstring_view(slot, &mut data, &mut len);
    String::from_utf8_lossy(std::slice::from_raw_parts(data as *const u8, len)).into_owned()
}

/// The hand oracle: one Image value.
struct ImageOracle {
    stamp: CppTime,
    frame_id: &'static str,
    height: u32,
    width: u32,
    encoding: &'static str,
    is_bigendian: u8,
    step: u32,
    data: Vec<u8>,
}

fn image_oracle(payload_len: usize, seed: u32) -> ImageOracle {
    ImageOracle {
        stamp: CppTime {
            sec: 1_700_000_000 + seed as i32,
            nanosec: 123_456_789,
        },
        frame_id: "camera_optical_frame",
        height: 1080,
        width: 1920,
        encoding: "rgb8",
        is_bigendian: 0,
        step: 5760,
        data: (0..payload_len as u32)
            .map(|i| (i.wrapping_mul(2654435761).wrapping_add(seed) >> 24) as u8)
            .collect(),
    }
}

/// A source message built from the oracle over REAL containers; freed by
/// `image_fini` + `Box` drop.
unsafe fn make_image(o: &ImageOracle) -> Box<CppImage> {
    let mut m: Box<CppImage> = Box::new(std::mem::zeroed());
    image_init(&mut *m as *mut _ as *mut c_void, 0);
    m.header.stamp = o.stamp;
    rmw_cerulion_cppstring_destruct(m.header.frame_id.0.as_mut_ptr() as *mut c_void);
    rmw_cerulion_cppstring_construct(
        m.header.frame_id.0.as_mut_ptr() as *mut c_void,
        o.frame_id.as_ptr() as *const c_char,
        o.frame_id.len(),
    );
    m.height = o.height;
    m.width = o.width;
    rmw_cerulion_cppstring_destruct(m.encoding.0.as_mut_ptr() as *mut c_void);
    rmw_cerulion_cppstring_construct(
        m.encoding.0.as_mut_ptr() as *mut c_void,
        o.encoding.as_ptr() as *const c_char,
        o.encoding.len(),
    );
    m.is_bigendian = o.is_bigendian;
    m.step = o.step;
    rmw_cerulion_vector_u8_assign(
        m.data.0.as_mut_ptr() as *mut c_void,
        o.data.as_ptr(),
        o.data.len(),
    );
    m
}

unsafe fn publish_image(publisher: *const ffi::rmw_publisher_t, o: &ImageOracle) {
    let mut msg = make_image(o);
    assert_eq!(
        rmw_publish(
            publisher,
            &*msg as *const _ as *const c_void,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    image_fini(&mut *msg as *mut _ as *mut c_void);
}

/// Assert a loaned CppImage against the oracle: copied members exact,
/// the forged `data` aiming INSIDE the held sample with capacity == size.
unsafe fn assert_image_loan(
    subscription: *const ffi::rmw_subscription_t,
    loaned: *mut c_void,
    o: &ImageOracle,
) {
    let img = &*(loaned as *const CppImage);
    assert_eq!(
        img.header.stamp, o.stamp,
        "nested fixed member copied exactly"
    );
    assert_eq!(
        cpp_string_of(img.header.frame_id.0.as_ptr() as *const c_void),
        o.frame_id,
        "nested string copied into the shadow"
    );
    assert_eq!((img.height, img.width), (o.height, o.width));
    assert_eq!(
        cpp_string_of(img.encoding.0.as_ptr() as *const c_void),
        o.encoding
    );
    assert_eq!((img.is_bigendian, img.step), (o.is_bigendian, o.step));
    let vec = img.data.0.as_ptr() as *const c_void;
    let data_ptr = rmw_cerulion_vector_u8_data(vec) as usize;
    let range = held_sample_range(subscription, loaned);
    assert!(
        range.contains(&data_ptr) && range.contains(&(data_ptr + o.data.len() - 1)),
        "data() must aim INSIDE the held SHM sample ({data_ptr:#x} not in {range:x?})"
    );
    assert!(
        !range.contains(&(loaned as usize)),
        "the loaned object itself (the shadow) is NOT in shared memory"
    );
    assert_eq!(rmw_cerulion_vector_u8_size(vec), o.data.len());
    assert_eq!(
        rmw_cerulion_vector_u8_capacity(vec),
        o.data.len(),
        "capacity == size: growth must reallocate, never write past the frame"
    );
    assert_eq!(
        std::slice::from_raw_parts(data_ptr as *const u8, o.data.len()),
        &o.data[..],
        "payload bytes read through the SHM alias"
    );
}

/// After a return the shadow is retained by the pool and un-forged: its
/// vector reads as the empty triplet through the C++ runtime itself.
unsafe fn assert_shadow_unforged(loaned: *mut c_void) {
    let img = &*(loaned as *const CppImage);
    let vec = img.data.0.as_ptr() as *const c_void;
    assert!(
        rmw_cerulion_vector_u8_data(vec).is_null(),
        "un-forged data() is null"
    );
    assert_eq!(rmw_cerulion_vector_u8_size(vec), 0);
    assert_eq!(rmw_cerulion_vector_u8_capacity(vec), 0);
}

// =====================================================================
// C fixture: LaserScan-shaped, two forged float32[] + a string
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

static C_FINI_CALLS: AtomicU64 = AtomicU64::new(0);

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
    C_FINI_CALLS.fetch_add(1, Ordering::SeqCst);
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
            message_namespace_: cstr("rmw_shadow__msg"),
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

struct ScanOracle {
    angle_min: f32,
    angle_max: f32,
    ranges: Vec<f32>,
    intensities: Vec<f32>,
    frame_id: &'static str,
}

fn scan_oracle(n: usize) -> ScanOracle {
    ScanOracle {
        angle_min: -1.5707964,
        angle_max: 1.5707964,
        ranges: (0..n).map(|i| 0.5 + i as f32 * 0.01).collect(),
        intensities: (0..n).map(|i| (i % 7) as f32 * 10.0).collect(),
        frame_id: "laser",
    }
}

unsafe fn publish_scan(publisher: *const ffi::rmw_publisher_t, o: &ScanOracle) {
    let rdata = calloc(o.ranges.len(), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(o.ranges.as_ptr(), rdata, o.ranges.len());
    let idata = calloc(o.intensities.len(), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(o.intensities.as_ptr(), idata, o.intensities.len());
    let sdata = calloc(o.frame_id.len() + 1, 1) as *mut u8;
    std::ptr::copy_nonoverlapping(o.frame_id.as_ptr(), sdata, o.frame_id.len());
    let msg = CScan {
        angle_min: o.angle_min,
        angle_max: o.angle_max,
        ranges: CF32Seq {
            data: rdata,
            size: o.ranges.len(),
            capacity: o.ranges.len(),
        },
        intensities: CF32Seq {
            data: idata,
            size: o.intensities.len(),
            capacity: o.intensities.len(),
        },
        frame_id: CRosString {
            data: sdata,
            size: o.frame_id.len(),
            capacity: o.frame_id.len() + 1,
        },
    };
    assert_eq!(
        rmw_publish(
            publisher,
            &msg as *const _ as *const c_void,
            std::ptr::null_mut()
        ),
        RMW_RET_OK
    );
    free(rdata as *mut c_void);
    free(idata as *mut c_void);
    free(sdata as *mut c_void);
}

unsafe fn c_string_of(s: &CRosString) -> String {
    String::from_utf8_lossy(std::slice::from_raw_parts(s.data, s.size)).into_owned()
}

unsafe fn assert_scan_loan(
    subscription: *const ffi::rmw_subscription_t,
    loaned: *mut c_void,
    o: &ScanOracle,
) {
    let s = &*(loaned as *const CScan);
    assert_eq!(s.angle_min.to_bits(), o.angle_min.to_bits());
    assert_eq!(s.angle_max.to_bits(), o.angle_max.to_bits());
    assert_eq!(c_string_of(&s.frame_id), o.frame_id);
    let range = held_sample_range(subscription, loaned);
    for (name, seq, want) in [
        ("ranges", &s.ranges, &o.ranges),
        ("intensities", &s.intensities, &o.intensities),
    ] {
        let p = seq.data as usize;
        assert!(
            range.contains(&p),
            "{name}.data must aim inside the held sample"
        );
        assert_eq!(p % 4, 0, "{name}: forged f32* is 4-aligned");
        assert_eq!(seq.size, want.len(), "{name}.size");
        assert_eq!(seq.capacity, want.len(), "{name}.capacity == size");
        let seen = std::slice::from_raw_parts(seq.data, seq.size);
        for (i, (a, b)) in seen.iter().zip(want.iter()).enumerate() {
            assert_eq!(a.to_bits(), b.to_bits(), "{name}[{i}]");
        }
    }
    assert!(
        !range.contains(&(s.frame_id.data as usize)),
        "the string is a shadow-owned copy"
    );
}

// =====================================================================
// Tests
// =====================================================================

/// HEADLINE: an Image-shaped C++ message taken as a loan is served through
/// a shadow whose `data()` aims INTO the held iceoryx2 sample — zero payload
/// copies — with every copied member exact; the return un-forges the
/// shadow and the next take REUSES it with no Rust-heap allocation.
#[test]
#[serial]
fn cpp_image_loaned_take_aims_data_into_the_held_sample_and_recycles_the_shadow() {
    unsafe {
        let suffix = unique_suffix();
        let ts = image_ts(&format!("ImgA{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "img", suffix);
        assert!(
            (*subscription).can_loan_messages,
            "Image-shaped type must be take-loanable"
        );
        assert!(
            !(*publisher).can_loan_messages,
            "the publish-side loan stays fixed-only (a borrow must hand out a whole SHM struct)"
        );
        assert_eq!(
            shadows_built_and_free(subscription),
            (0, 0),
            "nothing built before a take"
        );

        let o1 = image_oracle(1 << 20, 1);
        publish_image(publisher, &o1);
        let (ret, taken, loaned, info) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert!(!loaned.is_null());
        assert!(
            loan_has_shadow(subscription, loaned),
            "a forged take is served by a shadow"
        );
        assert_eq!(shadows_built_and_free(subscription), (1, 0));
        assert_image_loan(subscription, loaned, &o1);
        // message_info is the wire header of that same held sample.
        {
            let data = sub_data(subscription);
            let inner = data.inner.lock().unwrap_or_else(|e| e.into_inner());
            let header = inner.pending_takes[0]
                .sample
                .wire_header()
                .expect("held frame header");
            assert_eq!(info.source_timestamp, header.timestamp_ns as i64);
            assert_eq!(info.publication_sequence_number, u64::from(header.sequence));
            assert_eq!(info.publication_sequence_number, 0, "first publish");
        }

        // Return: un-forged BEFORE the sample is released, shadow retained.
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, loaned),
            RMW_RET_OK
        );
        assert_eq!(outstanding(subscription), 0);
        assert_eq!(
            shadows_built_and_free(subscription),
            (1, 1),
            "the shadow is recycled, not freed"
        );
        assert_shadow_unforged(loaned);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, loaned),
            ffi::RMW_RET_INVALID_ARGUMENT,
            "a second return of the same pointer is an unknown loan"
        );

        // Steady state: the SAME shadow serves the next take, and the rmw's
        // own take + return path allocates nothing on the Rust heap.
        let o2 = image_oracle(1 << 20, 2);
        publish_image(publisher, &o2);
        ALLOCATOR.enable();
        let (ret, taken, loaned2, _) = take_loaned(subscription);
        let ret_back = rmw_return_loaned_message_from_subscription(subscription, loaned2);
        let (allocs, bytes) = ALLOCATOR.disable();
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_eq!(loaned2, loaned, "the recycled shadow is handed out again");
        assert_eq!(ret_back, RMW_RET_OK);
        assert_eq!(
            shadows_built_and_free(subscription),
            (1, 1),
            "no second shadow was built"
        );
        assert_eq!(
            (allocs, bytes),
            (0, 0),
            "steady-state forged take + return must not allocate on the Rust heap"
        );
        // The second frame's bytes were the ones served (re-take to check
        // the oracle against the held sample of THAT take).
        publish_image(publisher, &o2);
        let (ret, taken, loaned3, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_image_loan(subscription, loaned3, &o2);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, loaned3),
            RMW_RET_OK
        );

        // Empty queue: taken false, no pointer, no shadow consumed.
        let (ret, taken, none, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(!taken);
        assert!(none.is_null());
        assert_eq!(shadows_built_and_free(subscription), (1, 1));

        teardown(node, publisher, subscription);
    }
}

/// The C typesupport path forges rosidl `{data, size, capacity}` headers:
/// Class sweep after the adopted-path fixes: a panic in the loaned take's
/// SERVE REPORT must not strand the loan.
///
/// The loan is pushed into `pending_takes` — the sample borrowed, its slot
/// consumed — and only THEN does `report_loan_served` run, with
/// `*loaned_message` and `*taken` still unwritten. A panic there (a host
/// `tracing` subscriber's `on_event`) made `ffi_guard` return
/// `RMW_RET_ERROR` while the loan stayed tracked and the caller never
/// received the pointer it would need for
/// `rmw_return_loaned_message_from_subscription`: one borrow slot pinned per
/// panic, for the life of the process.
///
/// The same shape as the ADOPTED path's panic arms, covered here as one
/// class. The invariant is theirs: on failure
/// the caller holds nothing of ours and we hold nothing of the caller's.
#[test]
#[serial]
fn a_panic_in_the_loaned_serve_report_releases_the_loan() {
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ServeRep{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "serverep", suffix);
        let before = outstanding(subscription);

        publish_scan(publisher, &scan_oracle(64));
        let fired_before = rmw_cerulion::test_seams::loaned_serve_report_panics_fired();
        let (ret, taken, loaned, _) = {
            let _seam = rmw_cerulion::test_seams::LoanedServeReportPanicGuard::arm();
            take_loaned(subscription)
        };
        assert_eq!(
            rmw_cerulion::test_seams::loaned_serve_report_panics_fired(),
            fired_before + 1,
            "the serve-report seam must have fired — otherwise this arm proves nothing"
        );
        assert_eq!(ret, ffi::RMW_RET_ERROR, "the panic surfaces as an error");
        assert!(!taken, "and nothing is handed out");
        assert!(loaned.is_null(), "the out-param is never written");

        // THE PIN: the loan is GONE, not stranded. Nothing else could ever
        // return it — the caller has no pointer to hand back.
        assert_eq!(
            outstanding(subscription),
            before,
            "the unwind must release the loan; a stranded one pins its borrow slot for the \
             life of the process"
        );

        teardown(node, publisher, subscription);

        // ANTI-TAUTOLOGY on a FRESH pair, deliberately not this one: a
        // caught panic marks the entity wedged BY DESIGN ("entity wedged by
        // an earlier panic; failing call"), so re-taking on the same
        // subscription would fail for a reason that has nothing to do with
        // the rollback. A fresh pair shows the seam, not the harness, is
        // what produced the outcome above.
        let suffix2 = unique_suffix();
        let ts2 = scan_ts(&format!("ServeRepOk{suffix2}"));
        let (node2, publisher2, subscription2) = setup_pair(ts2, "serverepok", suffix2);
        publish_scan(publisher2, &scan_oracle(64));
        let (ret, taken, loaned, _) = take_loaned(subscription2);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken && !loaned.is_null());
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription2, loaned),
            RMW_RET_OK
        );
        teardown(node2, publisher2, subscription2);
    }
}

/// TWO `float32[]` members aim into the held sample, 4-aligned, and the
/// return re-points both at `{NULL, 0, 0}`.
#[test]
#[serial]
fn c_scan_loaned_take_forges_both_sequences_and_unforges_on_return() {
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ScanA{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "scan", suffix);
        assert!((*subscription).can_loan_messages);

        let o = scan_oracle(1081);
        publish_scan(publisher, &o);
        let (ret, taken, loaned, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_scan_loan(subscription, loaned, &o);

        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, loaned),
            RMW_RET_OK
        );
        let s = &*(loaned as *const CScan);
        assert!(s.ranges.data.is_null() && s.intensities.data.is_null());
        assert_eq!((s.ranges.size, s.ranges.capacity), (0, 0));
        assert_eq!((s.intensities.size, s.intensities.capacity), (0, 0));

        // Reuse: same shadow, second frame exact.
        let o2 = scan_oracle(360);
        publish_scan(publisher, &o2);
        let (ret, taken, loaned2, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_eq!(loaned2, loaned);
        assert_scan_loan(subscription, loaned2, &o2);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, loaned2),
            RMW_RET_OK
        );
        teardown(node, publisher, subscription);
    }
}

/// Budget: the shadow pool is the loan budget (4). With four forged loans
/// outstanding the fifth take is REFUSED — RMW_RET_ERROR, `taken` false,
/// no pointer, the queued frame NOT consumed, the refusal COUNTED — and
/// returning one loan serves that surviving frame through the recycled
/// shadow (FIFO intact).
#[test]
#[serial]
fn cpp_shadow_pool_exhaustion_is_refused_before_consuming_a_frame_and_recovers() {
    unsafe {
        let suffix = unique_suffix();
        let ts = image_ts(&format!("ImgB{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "imgb", suffix);
        let oracles: Vec<ImageOracle> = (0..5).map(|i| image_oracle(4096, 10 + i)).collect();
        for o in &oracles {
            publish_image(publisher, o);
        }
        let mut held = Vec::new();
        for (i, o) in oracles.iter().take(4).enumerate() {
            let (ret, taken, p, _) = take_loaned(subscription);
            assert_eq!(ret, RMW_RET_OK, "take {i} inside the budget");
            assert!(taken);
            assert_image_loan(subscription, p, o);
            held.push(p);
        }
        assert_eq!(shadows_built_and_free(subscription), (4, 0));
        assert_eq!(sub_data(subscription).loan_refusal_count(), 0);

        let (ret, taken, p5, _) = take_loaned(subscription);
        assert_eq!(
            ret,
            ffi::RMW_RET_ERROR,
            "the fifth concurrent forged loan is refused"
        );
        assert!(!taken);
        assert!(p5.is_null());
        assert_eq!(outstanding(subscription), 4);
        assert_eq!(
            shadows_built_and_free(subscription),
            (4, 0),
            "no fifth shadow is built"
        );
        assert_eq!(
            sub_data(subscription).loan_refusal_count(),
            1,
            "the refusal is counted unconditionally"
        );

        // Return ONE: the surviving fifth frame is served through the
        // recycled shadow, oracle-exact.
        let first = held.remove(0);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, first),
            RMW_RET_OK
        );
        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK, "returning a loan recovers the take");
        assert!(taken);
        assert_eq!(p, first, "the freed shadow is the one reused");
        assert_image_loan(subscription, p, &oracles[4]);
        assert_eq!(
            sub_data(subscription).loan_refusal_count(),
            1,
            "a served take never counts"
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        for p in held {
            assert_eq!(
                rmw_return_loaned_message_from_subscription(subscription, p),
                RMW_RET_OK
            );
        }
        assert_eq!(shadows_built_and_free(subscription), (4, 4));
        teardown(node, publisher, subscription);
    }
}

/// Eligibility through the C ABI: a string-only type is NOT take-loanable
/// (nothing to forge), every loan verb is UNSUPPORTED; the copying take
/// still works. Kills a predicate that admits any variable type.
#[test]
#[serial]
fn cpp_string_only_type_is_not_take_loanable_through_the_c_abi() {
    #[repr(C, align(16))]
    struct CppStringMsg {
        data: StringSlot,
    }
    unsafe extern "C" fn init(msg: *mut c_void, _i: u32) {
        rmw_cerulion_cppstring_construct(msg, std::ptr::null(), 0);
    }
    unsafe extern "C" fn fini(msg: *mut c_void) {
        rmw_cerulion_cppstring_destruct(msg);
    }
    unsafe {
        let suffix = unique_suffix();
        let ts = cpp_ts(cpp_members(
            "rmw_shadow::msg",
            &format!("Str{suffix}"),
            std::mem::size_of::<CppStringMsg>(),
            vec![cpp_member("data", ROS_TYPE_STRING, 0)],
            Some(init),
            Some(fini),
        ));
        let (node, publisher, subscription) = setup_pair(ts, "str", suffix);
        assert!(!(*subscription).can_loan_messages);
        assert!(!(*publisher).can_loan_messages);
        let (ret, taken, out, _) = take_loaned(subscription);
        assert_eq!(ret, ffi::RMW_RET_UNSUPPORTED);
        assert!(!taken);
        assert!(out.is_null());
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, 0x8 as *mut c_void),
            ffi::RMW_RET_UNSUPPORTED
        );
        assert_eq!(shadows_built_and_free(subscription), (0, 0));
        teardown(node, publisher, subscription);
    }
}

/// Control: a FIXED type's loaned take is unchanged — the SHM pointer
/// itself, no shadow built, no shadow on the take.
#[test]
#[serial]
fn fixed_type_take_still_hands_out_the_shm_pointer_without_a_shadow() {
    #[repr(C)]
    #[derive(Clone, Copy, PartialEq, Debug)]
    struct CPoint {
        x: f64,
        y: f64,
        z: f64,
    }
    unsafe {
        let suffix = unique_suffix();
        let members = Box::leak(
            vec![
                c_member("x", 2, 0),
                c_member("y", 2, 8),
                c_member("z", 2, 16),
            ]
            .into_boxed_slice(),
        );
        let mm = Box::leak(Box::new(
            ffi::rosidl_typesupport_introspection_c__MessageMembers {
                message_namespace_: cstr("rmw_shadow__msg"),
                message_name_: cstr(&format!("Pt{suffix}")),
                member_count_: 3,
                size_of_: std::mem::size_of::<CPoint>(),
                members_: members.as_ptr(),
                ..Default::default()
            },
        ));
        let ts: *const ffi::rosidl_message_type_support_t =
            Box::leak(Box::new(ffi::rosidl_message_type_support_t {
                typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
                data: mm as *const _ as *const c_void,
                ..Default::default()
            }));
        let (node, publisher, subscription) = setup_pair(ts, "pt", suffix);
        assert!((*subscription).can_loan_messages);
        assert!(
            (*publisher).can_loan_messages,
            "a fixed type still borrows on the publish side"
        );
        let msg = CPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        assert_eq!(
            rmw_publish(
                publisher,
                &msg as *const _ as *const c_void,
                std::ptr::null_mut()
            ),
            RMW_RET_OK
        );
        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert!(
            !loan_has_shadow(subscription, p),
            "a fixed type's take has no shadow"
        );
        assert_eq!(shadows_built_and_free(subscription), (0, 0));
        let range = held_sample_range(subscription, p);
        assert!(
            range.contains(&(p as usize)),
            "the loaned pointer IS the SHM payload"
        );
        assert_eq!(*(p as *const CPoint), msg);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        teardown(node, publisher, subscription);
    }
}

/// Destroy with an outstanding FORGED loan: the shadow is un-forged, its
/// `fini` (the C++ destructor) runs over an EMPTY vector, the sample is
/// released — no abort, no leak — and the topic stays healthy.
#[test]
#[serial]
fn destroying_a_subscription_with_a_forged_loan_outstanding_unforges_then_finis() {
    unsafe {
        let suffix = unique_suffix();
        let ts = image_ts(&format!("ImgD{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "imgd", suffix);
        let o = image_oracle(65536, 3);
        publish_image(publisher, &o);
        publish_image(publisher, &o);
        let (ret, taken, p1, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        let (ret, taken, p2, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_ne!(p1, p2);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p2),
            RMW_RET_OK
        );
        assert_eq!(shadows_built_and_free(subscription), (2, 1));

        // One loan outstanding, one shadow idle: destroy runs fini on BOTH
        // (the outstanding one after un-forging) — counted, never a crash.
        CPP_FINI_CALLS.store(0, Ordering::SeqCst);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(
            CPP_FINI_CALLS.load(Ordering::SeqCst),
            2,
            "every built shadow is destroyed exactly once"
        );

        // Healthy afterwards.
        let topic = CString::new(format!("/rmw_shadow/imgd/{suffix}")).expect("topic");
        let qos = default_qos();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let sub2 = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!sub2.is_null());
        let o2 = image_oracle(128, 4);
        publish_image(publisher, &o2);
        let (ret, taken, p, _) = take_loaned(sub2);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_image_loan(sub2, p, &o2);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(sub2, p),
            RMW_RET_OK
        );
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, sub2), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// Determinism: two independent runs of the same message through the forged
/// take observe the SAME values — each asserted against the hand oracle,
/// never against each other.
#[test]
#[serial]
fn two_runs_of_the_forged_take_match_the_hand_oracle() {
    unsafe {
        let o = image_oracle(777, 9);
        for run in 0..2 {
            let suffix = unique_suffix();
            let ts = image_ts(&format!("ImgR{run}x{suffix}"));
            let (node, publisher, subscription) = setup_pair(ts, "imgr", suffix);
            publish_image(publisher, &o);
            let (ret, taken, p, _) = take_loaned(subscription);
            assert_eq!(ret, RMW_RET_OK, "run {run}");
            assert!(taken);
            assert_image_loan(subscription, p, &o);
            assert_eq!(
                rmw_return_loaned_message_from_subscription(subscription, p),
                RMW_RET_OK
            );
            teardown(node, publisher, subscription);
        }
    }
}

/// A same-hash frame whose forged entry is NOT element-aligned (a bag player
/// can put one on the wire through `rmw_publish_serialized_message`) is
/// REFUSED: consumed, dropped, counted on the decode latch, `taken` false,
/// no pointer, the shadow back in the pool — and a clean frame afterwards
/// is loaned normally.
#[test]
#[serial]
fn c_misaligned_forge_target_on_the_wire_is_refused_and_counted() {
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ScanM{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "scanm", suffix);
        let o = scan_oracle(8);
        // Serialize a scan through the real rmw_serialize, then shift the
        // `ranges` entry by one byte (in bounds, whole elements).
        let rdata = calloc(o.ranges.len(), 4) as *mut f32;
        std::ptr::copy_nonoverlapping(o.ranges.as_ptr(), rdata, o.ranges.len());
        let idata = calloc(o.intensities.len(), 4) as *mut f32;
        std::ptr::copy_nonoverlapping(o.intensities.as_ptr(), idata, o.intensities.len());
        let sdata = calloc(o.frame_id.len() + 1, 1) as *mut u8;
        std::ptr::copy_nonoverlapping(o.frame_id.as_ptr(), sdata, o.frame_id.len());
        let msg = CScan {
            angle_min: o.angle_min,
            angle_max: o.angle_max,
            ranges: CF32Seq {
                data: rdata,
                size: o.ranges.len(),
                capacity: o.ranges.len(),
            },
            intensities: CF32Seq {
                data: idata,
                size: o.intensities.len(),
                capacity: o.intensities.len(),
            },
            frame_id: CRosString {
                data: sdata,
                size: o.frame_id.len(),
                capacity: o.frame_id.len() + 1,
            },
        };
        let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
        ser.allocator = malloc_allocator();
        assert_eq!(
            rmw_serialize(&msg as *const _ as *const c_void, ts, &mut ser),
            RMW_RET_OK
        );
        let bytes = std::slice::from_raw_parts_mut(ser.buffer, ser.buffer_length);
        let fixed_size = 8usize; // angle_min + angle_max
        let entry = cerulion_core::wire::WireHeader::SIZE + fixed_size; // ranges = entry 0
        let off = u32::from_le_bytes(bytes[entry..entry + 4].try_into().unwrap());
        let len = u32::from_le_bytes(bytes[entry + 4..entry + 8].try_into().unwrap());
        assert_eq!(len as usize, o.ranges.len() * 4);
        bytes[entry..entry + 4].copy_from_slice(&(off + 1).to_le_bytes());
        bytes[entry + 4..entry + 8].copy_from_slice(&(len - 4).to_le_bytes());
        assert_eq!(
            rmw_publish_serialized_message(publisher, &ser, std::ptr::null_mut()),
            RMW_RET_OK
        );
        assert_eq!(decode_failure_count(subscription), 0);
        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(!taken, "a misaligned forge target must not be loaned");
        assert!(p.is_null());
        assert_eq!(
            decode_failure_count(subscription),
            1,
            "counted on the framing latch"
        );
        assert_eq!(
            shadows_built_and_free(subscription),
            (1, 1),
            "the shadow went back to the pool"
        );
        assert_eq!(outstanding(subscription), 0);

        // Control: a clean frame is loaned normally afterwards.
        publish_scan(publisher, &o);
        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_scan_loan(subscription, p, &o);
        assert_eq!(decode_failure_count(subscription), 1);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        free(rdata as *mut c_void);
        free(idata as *mut c_void);
        free(sdata as *mut c_void);
        teardown(node, publisher, subscription);
    }
}

unsafe extern "C" fn fixture_allocate(size: usize, _state: *mut c_void) -> *mut c_void {
    calloc(size.max(1), 1)
}
unsafe extern "C" fn fixture_deallocate(ptr: *mut c_void, _state: *mut c_void) {
    free(ptr)
}
unsafe extern "C" fn fixture_reallocate(
    ptr: *mut c_void,
    size: usize,
    _state: *mut c_void,
) -> *mut c_void {
    extern "C" {
        fn realloc(p: *mut c_void, n: usize) -> *mut c_void;
    }
    realloc(ptr, size)
}
unsafe extern "C" fn fixture_zero_allocate(
    n: usize,
    size: usize,
    _state: *mut c_void,
) -> *mut c_void {
    calloc(n.max(1), size.max(1))
}

fn malloc_allocator() -> ffi::rcutils_allocator_t {
    ffi::rcutils_allocator_t {
        allocate: Some(fixture_allocate),
        deallocate: Some(fixture_deallocate),
        reallocate: Some(fixture_reallocate),
        zero_allocate: Some(fixture_zero_allocate),
        state: std::ptr::null_mut(),
    }
}

// =====================================================================
// The read-only contract, pinned in a child process
// =====================================================================

const CHILD_ENV: &str = "RMW_SHADOW_TAKE_WRITE_CHILD";

/// The CHILD: take a forged loan and WRITE through `data()`. Must die by
/// signal — `#[ignore]`d so it only ever runs when the parent spawns it.
#[test]
#[ignore = "child arm of `a_write_through_the_forged_payload_faults_loudly`; spawned by the parent"]
fn forged_write_child() {
    if std::env::var_os(CHILD_ENV).is_none() {
        return;
    }
    unsafe {
        let suffix = unique_suffix();
        let ts = image_ts(&format!("ImgW{suffix}"));
        let (_node, publisher, subscription) = setup_pair(ts, "imgw", suffix);
        let o = image_oracle(4096, 5);
        publish_image(publisher, &o);
        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        // PROVE the write below is the forged one: a null loan or a pointer
        // outside the held sample would fault too, and prove nothing. The
        // parent requires the marker AND the signal.
        assert!(!p.is_null(), "child: the loaned pointer must be non-null");
        let img = &*(p as *const CppImage);
        let data = rmw_cerulion_vector_u8_data(img.data.0.as_ptr() as *const c_void) as *mut u8;
        let range = held_sample_range(subscription, p);
        assert!(
            range.contains(&(data as usize)),
            "child: data() must lie inside the held sample before the write"
        );
        {
            use std::io::Write as _;
            eprintln!("FORGED-WRITE-REACHED data={:#x}", data as usize);
            std::io::stderr().flush().expect("flush marker");
        }
        // The write that must fault: the mapping is read-only.
        std::ptr::write_volatile(data, 0xFF);
        // Unreachable on a correct transport; make it LOUD if reached. The
        // test then returns normally and the child exits 0 — which the
        // parent treats as the contract being false.
        eprintln!("WRITE-THROUGH-FORGED-POINTER-SUCCEEDED");
    }
}

/// A write through the forged payload pointer terminates the process by
/// signal (SIGSEGV / SIGBUS) — loud, never silent, never another
/// subscriber's view. A child that returns normally means the subscriber
/// mapping is writable, which would falsify the documented contract; a child
/// that dies WITHOUT first printing the reached-marker (a null loan, a pointer
/// outside the sample, a panic) fails too — any fault is not evidence.
#[test]
#[serial]
fn a_write_through_the_forged_payload_faults_loudly() {
    use std::os::unix::process::ExitStatusExt;
    let exe = std::env::current_exe().expect("current_exe");
    let out = std::process::Command::new(exe)
        .args([
            "--exact",
            "forged_write_child",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD_ENV, "1")
        .output()
        .expect("spawn child");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("FORGED-WRITE-REACHED"),
        "the child must REACH the forged write (non-null loan, data() inside the held \
         sample) before it dies — a fault anywhere else proves nothing:\n{stderr}"
    );
    assert!(
        !stderr.contains("WRITE-THROUGH-FORGED-POINTER-SUCCEEDED"),
        "the child wrote into the subscriber mapping without faulting:\n{stderr}"
    );
    let signal = out.status.signal();
    // Per-platform values via libc (the state_dontfork_test convention): the
    // old hand-defined pair claimed to be portable but bound SIGBUS to 10,
    // which on Linux is SIGUSR1 (Linux SIGBUS is 7) — so a stray SIGUSR1
    // would have been accepted as proof of the read-only contract and a real
    // Linux SIGBUS misclassified as a wrong exit.
    assert!(
        matches!(signal, Some(s) if s == libc::SIGSEGV || s == libc::SIGBUS),
        "child must die by SIGSEGV/SIGBUS, got status {:?} (signal {signal:?})\n{stderr}",
        out.status
    );
}

// =====================================================================
// PLACEMENT over the wire: the data floor (the FrameWalker's rule)
// =====================================================================

/// Serialize one scan through the real `rmw_serialize` (buffers freed by the
/// caller through `free_scan_buffers`).
unsafe fn serialized_scan(
    o: &ScanOracle,
    ts: *const ffi::rosidl_message_type_support_t,
) -> (ffi::rmw_serialized_message_t, [*mut c_void; 3]) {
    let rdata = calloc(o.ranges.len(), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(o.ranges.as_ptr(), rdata, o.ranges.len());
    let idata = calloc(o.intensities.len(), 4) as *mut f32;
    std::ptr::copy_nonoverlapping(o.intensities.as_ptr(), idata, o.intensities.len());
    let sdata = calloc(o.frame_id.len() + 1, 1) as *mut u8;
    std::ptr::copy_nonoverlapping(o.frame_id.as_ptr(), sdata, o.frame_id.len());
    let msg = CScan {
        angle_min: o.angle_min,
        angle_max: o.angle_max,
        ranges: CF32Seq {
            data: rdata,
            size: o.ranges.len(),
            capacity: o.ranges.len(),
        },
        intensities: CF32Seq {
            data: idata,
            size: o.intensities.len(),
            capacity: o.intensities.len(),
        },
        frame_id: CRosString {
            data: sdata,
            size: o.frame_id.len(),
            capacity: o.frame_id.len() + 1,
        },
    };
    let mut ser: ffi::rmw_serialized_message_t = std::mem::zeroed();
    ser.allocator = malloc_allocator();
    assert_eq!(
        rmw_serialize(&msg as *const _ as *const c_void, ts, &mut ser),
        RMW_RET_OK
    );
    (
        ser,
        [
            rdata as *mut c_void,
            idata as *mut c_void,
            sdata as *mut c_void,
        ],
    )
}

unsafe fn free_scan_buffers(bufs: [*mut c_void; 3]) {
    for b in bufs {
        free(b);
    }
}

/// A same-hash frame whose `ranges` entry is aimed BELOW the data floor —
/// at payload offset 0, the fixed section's own eight bytes — is served, not
/// refused: the take succeeds, `ranges` is a heap COPY (outside the held
/// sample) of exactly the bytes the entry designates (`angle_min`,
/// `angle_max`), `intensities` is still forged, the fallback is COUNTED,
/// and the copied-into shadow is DESTROYED on return rather than recycled
/// (`fini` frees the copy; the pool builds a fresh one). The natural frame
/// afterwards places `ranges` exactly AT the floor and is forged.
#[test]
#[serial]
fn c_entry_below_the_data_floor_on_the_wire_is_copied_counted_and_the_shadow_retired() {
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ScanF{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "scanf", suffix);
        let o = scan_oracle(6);
        let fixed_size = 8usize; // angle_min + angle_max
        let data_floor = fixed_size + 3 * 8; // + three offset-table entries
        let entry = cerulion_core::wire::WireHeader::SIZE + fixed_size; // ranges = entry 0

        let (ser, bufs) = serialized_scan(&o, ts);
        let bytes = std::slice::from_raw_parts_mut(ser.buffer, ser.buffer_length);
        bytes[entry..entry + 4].copy_from_slice(&0u32.to_le_bytes());
        bytes[entry + 4..entry + 8].copy_from_slice(&8u32.to_le_bytes());
        assert_eq!(
            rmw_publish_serialized_message(publisher, &ser, std::ptr::null_mut()),
            RMW_RET_OK
        );
        free_scan_buffers(bufs);
        assert_eq!(sub_data(subscription).forge_fallback_count(), 0);

        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken, "a below-floor entry is served, not refused");
        assert!(!p.is_null());
        let s = &*(p as *const CScan);
        let range = held_sample_range(subscription, p);
        assert!(
            !range.contains(&(s.ranges.data as usize)),
            "ranges was COPIED: its header must not alias the held sample"
        );
        assert_eq!((s.ranges.size, s.ranges.capacity), (2, 2));
        let seen = std::slice::from_raw_parts(s.ranges.data, 2);
        assert_eq!(
            seen[0].to_bits(),
            o.angle_min.to_bits(),
            "the bytes the entry designates"
        );
        assert_eq!(seen[1].to_bits(), o.angle_max.to_bits());
        assert!(
            range.contains(&(s.intensities.data as usize)),
            "intensities (at/above the floor) is still forged into the sample"
        );
        assert_eq!(s.intensities.size, o.intensities.len());
        assert_eq!(c_string_of(&s.frame_id), o.frame_id);
        assert_eq!(
            sub_data(subscription).forge_fallback_count(),
            1,
            "the fallback is counted unconditionally"
        );
        assert_eq!(
            decode_failure_count(subscription),
            0,
            "not a framing failure"
        );
        assert_eq!(shadows_built_and_free(subscription), (1, 0));

        // Return: the tainted shadow is destroyed (fini frees the copy), not
        // recycled — the pool is empty again and the next take builds anew.
        C_FINI_CALLS.store(0, Ordering::SeqCst);
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        assert_eq!(
            C_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "the copied-into shadow was retired"
        );
        assert_eq!(shadows_built_and_free(subscription), (0, 0));

        // Control: the natural frame places `ranges` exactly AT the floor and
        // is forged; the counter does not move.
        let (clean, bufs) = serialized_scan(&o, ts);
        let cbytes = std::slice::from_raw_parts(clean.buffer, clean.buffer_length);
        let off = u32::from_le_bytes(cbytes[entry..entry + 4].try_into().unwrap()) as usize;
        assert_eq!(
            off, data_floor,
            "the writer's first entry sits exactly AT the data floor"
        );
        assert_eq!(
            rmw_publish_serialized_message(publisher, &clean, std::ptr::null_mut()),
            RMW_RET_OK
        );
        free_scan_buffers(bufs);
        let (ret, taken, p2, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_scan_loan(subscription, p2, &o);
        assert_eq!(
            sub_data(subscription).forge_fallback_count(),
            1,
            "a forged take never counts"
        );
        assert_eq!(
            shadows_built_and_free(subscription),
            (1, 0),
            "a fresh shadow was built"
        );
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p2),
            RMW_RET_OK
        );
        assert_eq!(
            shadows_built_and_free(subscription),
            (1, 1),
            "a clean shadow is recycled"
        );
        teardown(node, publisher, subscription);
    }
}

/// A below-floor `ranges` entry (copied) followed by a malformed `frame_id`
/// entry on the SAME frame: the take is refused (framing latch), nothing
/// forged survives, and the shadow that now owns the copy is RETIRED rather
/// than recycled — the next take builds a fresh one (a
/// recycled copy-holding shadow leaks its buffer under the next forge).
#[test]
#[serial]
fn c_copied_then_malformed_frame_retires_the_shadow_instead_of_recycling_it() {
    unsafe {
        let suffix = unique_suffix();
        let ts = scan_ts(&format!("ScanR{suffix}"));
        let (node, publisher, subscription) = setup_pair(ts, "scanr", suffix);
        let o = scan_oracle(5);
        let fixed_size = 8usize;
        let entry = cerulion_core::wire::WireHeader::SIZE + fixed_size; // ranges = entry 0
        let (ser, bufs) = serialized_scan(&o, ts);
        let bytes = std::slice::from_raw_parts_mut(ser.buffer, ser.buffer_length);
        bytes[entry..entry + 4].copy_from_slice(&0u32.to_le_bytes());
        bytes[entry + 4..entry + 8].copy_from_slice(&8u32.to_le_bytes());
        // frame_id = entry 2: a length past the frame.
        bytes[entry + 20..entry + 24].copy_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(
            rmw_publish_serialized_message(publisher, &ser, std::ptr::null_mut()),
            RMW_RET_OK
        );
        free_scan_buffers(bufs);

        C_FINI_CALLS.store(0, Ordering::SeqCst);
        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(!taken, "a malformed frame is refused");
        assert!(p.is_null());
        assert_eq!(decode_failure_count(subscription), 1);
        assert_eq!(
            shadows_built_and_free(subscription),
            (0, 0),
            "the shadow that copied before the failure is RETIRED, not recycled"
        );
        assert_eq!(
            C_FINI_CALLS.load(Ordering::SeqCst),
            1,
            "its fini freed the copy"
        );
        assert_eq!(outstanding(subscription), 0);

        // A clean frame afterwards builds a fresh shadow and is served.
        publish_scan(publisher, &o);
        let (ret, taken, p, _) = take_loaned(subscription);
        assert_eq!(ret, RMW_RET_OK);
        assert!(taken);
        assert_scan_loan(subscription, p, &o);
        assert_eq!(shadows_built_and_free(subscription), (1, 0));
        assert_eq!(
            rmw_return_loaned_message_from_subscription(subscription, p),
            RMW_RET_OK
        );
        assert_eq!(
            shadows_built_and_free(subscription),
            (1, 1),
            "a clean shadow is recycled"
        );
        teardown(node, publisher, subscription);
    }
}

// =====================================================================
// The RESOLVER calls the seams-bypass warn seam
// =====================================================================

/// `era::emit_cpp_bypass_warn` is pinned in its own binary; that the
/// resolver CALLS it on every C++ typesupport resolve under the
/// `test-seams` bypass was not — deleting the call in
/// `cpp_typesupport_gate` killed nothing. This file is a resolver-routed
/// C++ surface (`setup_pair` registers `rosidl_typesupport_introspection_cpp`
/// handles through the C ABI), so the counter must advance here: once per
/// resolve, and a pair resolves at least twice (subscription + publisher).
/// A generated lane has no bypass and must advance it not at all.
#[test]
#[serial]
fn the_resolver_calls_the_seams_bypass_warn_on_every_cpp_typesupport_resolve() {
    unsafe {
        let suffix = unique_suffix();
        let ts = image_ts(&format!("ImgBypass{suffix}"));
        // `setup_pair`'s two creations, with the counter read BETWEEN them
        // (one aggregate delta of 2 is satisfied by
        // a resolver that warns twice for the subscription and never for
        // the publisher, so each creation is its own exact pin).
        let node = setup_node(&format!("bypass_node_{suffix}"));
        let topic = CString::new(format!("/rmw_shadow/bypass/{suffix}")).expect("topic");
        let qos = default_qos();
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        let before = rmw_cerulion::era::cpp_bypass_warns_fired();
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null(), "subscription creation failed");
        let after_subscription = rmw_cerulion::era::cpp_bypass_warns_fired();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null(), "publisher creation failed");
        let after_publisher = rmw_cerulion::era::cpp_bypass_warns_fired();
        teardown(node, publisher, subscription);
        let per_creation = if rmw_cerulion::era::bindings_source()
            == rmw_cerulion::era::BindingsSource::Vendored
        {
            // EXACTLY one per creation (the total is exact,
            // and so is each side).
            1
        } else {
            // A generated lane has no vendored bypass.
            0
        };
        assert_eq!(
            after_subscription - before,
            per_creation,
            "the subscription's C++ typesupport resolve must call the bypass warn \
             {per_creation} time(s)"
        );
        assert_eq!(
            after_publisher - after_subscription,
            per_creation,
            "the publisher's C++ typesupport resolve must call the bypass warn \
             {per_creation} time(s)"
        );
    }
}
