// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The WINDOWED PUBLISH path performs ZERO heap
//! allocations at steady state — the permanent regression guard for the
//! per-publisher [`SealScratch`] (plan items, placement offsets,
//! offset-table entries, the off-slot head build, the owned-encode arena)
//! and the create-time loan-bookkeeping reserves. Any future change that
//! sneaks an allocation back onto `rmw_publish_loaned_message`'s windowed
//! path (a fresh `Vec`, a `format!`, a boxed error) fails here.
//!
//! # Scope — the PUBLISH half, deliberately
//!
//! The measured window covers `rmw_publish_loaned_message` end to end:
//! wrong-thread guard, cursor bisection, disarm, seal (plan + commit),
//! wire-header stamp, iceoryx2 send, quarantine bookkeeping, counters.
//! The BORROW half cannot carry a zero-alloc counter pin: the typesupport
//! `init_function` allocates on the heap BY DESIGN (its containers belong
//! where `fini` frees them), and the FAKE hook's own bookkeeping (a
//! HashMap arm per thread) allocates in ways the real interposer does
//! not. The borrow-side loan-path reserve is pinned STRUCTURALLY instead
//! (`loan_bookkeeping_capacities` — all three vectors hold the loan
//! budget from CREATE, before any borrow).
//!
//! # Why a separate binary?
//!
//! `#[global_allocator]` is process-wide; the counter is additionally
//! THREAD-scoped (the `shm_ring_zero_alloc_test` pattern): background
//! threads — libtest machinery, platform TLS, the rmw event pump —
//! allocate at unpredictable times and are OUT of contract. A real
//! allocation introduced into the publish path still fails: it happens on
//! the measured thread. One `#[serial]` test body (the fold-ordered
//! pattern) so no sibling test thread exists during a measured window.
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_borrow_zero_alloc_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::HashMap;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};
use std::thread::ThreadId;

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::heaphook::{
    HookApi, TestHookGuard, RC_ERR_ALREADY_ARMED, RC_ERR_NOT_ARMED, RC_OK,
};
use rmw_cerulion::runtime::PublisherData;
use rmw_cerulion::*;

// ---------------------------------------------------------------------------
// Counting allocator (thread-scoped; counts alloc + alloc_zeroed + realloc —
// `Vec` growth reallocates, and `vec![0; n]` zero-allocates, so counting
// `alloc` alone would miss both regression shapes)
// ---------------------------------------------------------------------------

thread_local! {
    /// `true` only on the thread that called [`CountingAllocator::enable`].
    /// MUST be const-init with a non-`Drop` payload: a lazy TLS init or a
    /// registered destructor touched from inside `GlobalAlloc::alloc`
    /// would RECURSE into the allocator.
    static MEASURED_THREAD: Cell<bool> = const { Cell::new(false) };
}

struct CountingAllocator {
    inner: System,
    count: AtomicU64,
    enabled: AtomicBool,
}

impl CountingAllocator {
    const fn new() -> Self {
        Self {
            inner: System,
            count: AtomicU64::new(0),
            enabled: AtomicBool::new(false),
        }
    }
    fn enable(&self) {
        MEASURED_THREAD.set(true);
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        MEASURED_THREAD.set(false);
        self.count.load(Ordering::SeqCst)
    }
    fn tally(&self) {
        if self.enabled.load(Ordering::Relaxed) {
            // `try_with`, never `with`: during thread teardown TLS is
            // inaccessible and `with` would panic INSIDE the allocator.
            let measured = MEASURED_THREAD.try_with(Cell::get).unwrap_or(false);
            if measured {
                self.count.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.tally();
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        self.tally();
        unsafe { self.inner.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        self.tally();
        unsafe { self.inner.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

// ---------------------------------------------------------------------------
// Fixture: the fake heap hook + LaserScan-shaped unbounded C type (crib of
// `rmw_borrow_publish_test.rs`, trimmed to what the alloc pin needs)
// ---------------------------------------------------------------------------

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

#[derive(Clone, Copy)]
struct FakeWindow {
    base: usize,
    cursor: usize,
    limit: usize,
}

#[derive(Default)]
struct FakeState {
    windows: HashMap<ThreadId, FakeWindow>,
    retired: Vec<usize>,
}

fn fake() -> &'static Mutex<FakeState> {
    static FAKE: OnceLock<Mutex<FakeState>> = OnceLock::new();
    FAKE.get_or_init(|| Mutex::new(FakeState::default()))
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
        Some(_) => 0,
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
    // Swap-remove-by-value instead of the sibling suite's append-only log:
    // the retire fires at BORROW time (outside the measured windows), but
    // an unbounded Vec here would still be one warmup-defeating growth
    // source too many in an alloc-measuring binary.
    let mut s = fake().lock().expect("fake");
    if let Some(pos) = s.retired.iter().position(|&b| b == base as usize) {
        s.retired.swap_remove(pos);
    }
    s.retired.push(base as usize);
    RC_OK
}

/// The fake hook: the window entries are the real fake; the entries this
/// alloc-measuring suite never drives spread from `HookApi::inert()`'s
/// inert `RC_OK` stubs.
fn fake_hook_api() -> HookApi {
    HookApi {
        arm_window: f_arm,
        disarm_window: f_disarm,
        window_escape: f_escape,
        window_range_test: f_range,
        retire_slot: f_retire,
        ..HookApi::inert()
    }
}

/// Bump `len` bytes at `align` out of the current thread's fake window —
/// what the interposed `malloc` does under an armed window.
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
            message_namespace_: cstr("borrow_za__msg"),
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

unsafe fn set_heap_string(s: &mut CRosString, value: &str) {
    if !s.data.is_null() {
        free(s.data as *mut c_void);
    }
    s.data = calloc(value.len() + 1, 1) as *mut u8;
    std::ptr::copy_nonoverlapping(value.as_ptr(), s.data, value.len());
    s.size = value.len();
    s.capacity = value.len() + 1;
}

/// Bump-fill the borrowed struct (sequence into the window, string on the
/// libc heap — `calloc` never touches the Rust global allocator).
unsafe fn fill_adopted(msg: *mut c_void, angle: f32, ranges: &[f32], frame_id: &str) {
    let m = &mut *(msg as *mut CScanish);
    m.angle_min = angle;
    let dst = fake_bump(ranges.len() * 4, 16).expect("fill fits the window");
    std::ptr::copy_nonoverlapping(ranges.as_ptr(), dst as *mut f32, ranges.len());
    m.ranges = CF32Seq {
        data: dst as *mut f32,
        size: ranges.len(),
        capacity: ranges.len(),
    };
    set_heap_string(&mut m.frame_id, frame_id);
}

// ---------------------------------------------------------------------------
// The one test body (fold-ordered): reserve pin → warmup → measured →
// counter-bites control
// ---------------------------------------------------------------------------

const WARMUP_CYCLES: usize = 128;
const MEASURED_CYCLES: usize = 64;

#[test]
#[serial]
fn windowed_publish_is_zero_alloc_at_steady_state() {
    let _hook = TestHookGuard::install(fake_hook_api());
    unsafe {
        // Setup (unmeasured): runtime, node, one publisher of the
        // unbounded fixture type.
        let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
        let node = rmw_create_node(context, cstr("bza_node"), cstr("/"));
        assert!(!node.is_null());
        let ts = scanish_ts("ScanZa");
        let topic = CString::new(format!("/borrow_za/{}", std::process::id())).expect("topic");
        let qos = ffi::rmw_qos_profile_t {
            history: ffi::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth: 8,
            reliability: ffi::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability: ffi::RMW_QOS_POLICY_DURABILITY_VOLATILE,
            deadline: ffi::rmw_time_t { sec: 0, nsec: 0 },
            lifespan: ffi::rmw_time_t { sec: 0, nsec: 0 },
            liveliness: ffi::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            liveliness_lease_duration: ffi::rmw_time_t { sec: 0, nsec: 0 },
            avoid_ros_namespace_conventions: false,
        };
        let pub_opts: ffi::rmw_publisher_options_t = std::mem::zeroed();
        let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
        assert!(!publisher.is_null());
        let pdata = (*publisher).data as *const PublisherData;
        assert!((*publisher).can_loan_messages, "windowed surface is on");

        // --- Step 1: the loan-bookkeeping reserve, on a COLD publisher
        //     (before ANY borrow) — the structural half of the
        //     no-allocation-on-loan guarantee. The borrow path itself
        //     cannot carry a counter pin (typesupport init allocates by
        //     design), so the reserve is asserted directly.
        let (pending_cap, quarantine_cap, orphan_cap) = (*pdata).loan_bookkeeping_capacities();
        for (name, cap) in [
            ("pending_loans", pending_cap),
            ("quarantined_tails", quarantine_cap),
            ("orphaned_loans", orphan_cap),
        ] {
            assert!(
                cap >= 4,
                "{name} must be reserved to the loan budget at CREATE \
                 (got capacity {cap}) — an in-budget borrow's push must \
                 never allocate on the loan path"
            );
        }

        // --- Step 2: warmup (unmeasured) — pool slot rotation grows the
        //     quarantine bookkeeping to its steady footprint, the seal
        //     scratch reaches its retained capacity, first-connect and
        //     lazy transport work all happen here.
        let ranges = [1.5f32, -2.25, 3.0];
        for i in 0..WARMUP_CYCLES {
            let mut msg: *mut c_void = std::ptr::null_mut();
            assert_eq!(
                rmw_borrow_loaned_message(publisher, ts, &mut msg),
                RMW_RET_OK
            );
            fill_adopted(msg, i as f32, &ranges, "za");
            assert_eq!(
                rmw_publish_loaned_message(publisher, msg, std::ptr::null_mut()),
                RMW_RET_OK
            );
        }
        let adopted_before = (*pdata).borrow_adopted_count();

        // --- Step 3: the measured steady state — the counter is open
        //     ONLY across `rmw_publish_loaned_message` (borrow + fill are
        //     out of contract, see the module doc), every cycle.
        let mut total_allocs = 0u64;
        for i in 0..MEASURED_CYCLES {
            let mut msg: *mut c_void = std::ptr::null_mut();
            assert_eq!(
                rmw_borrow_loaned_message(publisher, ts, &mut msg),
                RMW_RET_OK
            );
            fill_adopted(msg, i as f32, &ranges, "za");
            ALLOCATOR.enable();
            let ret = rmw_publish_loaned_message(publisher, msg, std::ptr::null_mut());
            total_allocs += ALLOCATOR.disable();
            assert_eq!(ret, RMW_RET_OK);
        }
        assert_eq!(
            total_allocs, 0,
            "the windowed publish path must perform ZERO heap allocations \
             at steady state ({MEASURED_CYCLES} publishes measured)"
        );
        // Anti-vacuity: the measured cycles really took the zero-copy
        // windowed path (a pin over a silently degraded path proves
        // nothing about the seal machinery).
        assert_eq!(
            (*pdata).borrow_adopted_count(),
            adopted_before + MEASURED_CYCLES as u64,
            "every measured publish adopted"
        );

        // --- Step 4: the counter still bites (a broken probe would make
        //     step 3 vacuous).
        ALLOCATOR.enable();
        let poke: Box<u64> = Box::new(41);
        let sanity = ALLOCATOR.disable();
        assert_eq!(*poke, 41);
        assert!(
            sanity >= 1,
            "sanity: a deliberate Box allocation must be counted (got {sanity})"
        );
    }
}
