// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! The ADOPTED plain take's steady-state
//! Rust-heap allocation contract, pinned with a counting global allocator
//! (the `rmw_borrow_zero_alloc_test` discipline — own binary, because the
//! `#[global_allocator]` is process-wide).
//!
//! The contract:
//!
//! - an ADOPTED take costs EXACTLY ONE Rust-heap allocation — the
//!   `Arc<AdoptedSample>` — and that one is STRUCTURAL: the sample needs
//!   shared ownership whose clones (the registration cookies) outlive the
//!   call on whatever thread the app frees from, which is exactly `Arc`'s
//!   contract. A pooled refcount block would re-implement Arc's
//!   cross-thread release race; if the per-message allocation ever measures
//!   as a cost, a slab is the amortization path. Everything else the
//!   branch needs per take (the forged-range walk Vec, the
//!   registration rollback ledger) lives as reusable scratch on
//!   `SubscriptionInner`, cleared and refilled under the one mutex;
//! - a COPY-PATH take (no adopt gate) costs ZERO Rust-heap allocations.
//!
//! Scope (the `rmw_shadow_take_test` caveat): the probe counts the
//! RUST global allocator only. The copy path's sequence/string copies go
//! through libc `malloc` (rosidl-compatible by design) and a C++ fill
//! through `operator new` — neither is visible here, and neither is
//! claimed. What this binary pins is the RMW's own bookkeeping cost.
//!
//! ⚠️ iceoryx2 SHM is a process singleton, the fake hook and the env gate
//! are process-global, and the allocator window must not see a sibling
//! test's thread — run with `--test-threads=1`:
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_adopt_zero_alloc_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};

use rmw_cerulion::adopt_take::{AdoptedSample, ADOPT_TAKE_BUDGET_ENV, ADOPT_TAKE_ENV};
use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::heaphook::{
    HookApi, HookReleaseCallback, TestHookGuard, RC_ERR_BAD_ARG, RC_ERR_NOT_ARMED, RC_ERR_OVERLAP,
    RC_ERR_UNKNOWN, RC_OK,
};
use rmw_cerulion::runtime::SubscriptionData;
use rmw_cerulion::*;

const ROS_TYPE_FLOAT: u8 = 1;
const ROS_TYPE_STRING: u8 = 16;

extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

// =====================================================================
// Counting allocator (Rust global allocator only — see the module docs)
// =====================================================================

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
        self.count.store(0, Ordering::SeqCst);
        self.enabled.store(true, Ordering::SeqCst);
    }
    /// Allocations requested since `enable`.
    fn disable(&self) -> u64 {
        self.enabled.store(false, Ordering::SeqCst);
        self.count.load(Ordering::SeqCst)
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { self.inner.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { self.inner.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if self.enabled.load(Ordering::Relaxed) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { self.inner.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator::new();

// =====================================================================
// Minimal fake hook: segment registry + the release callback
// =====================================================================

#[derive(Clone, Copy)]
struct FakeSeg {
    start: usize,
    len: usize,
    cookie: usize,
}

#[derive(Default)]
struct FakeState {
    segments: Vec<FakeSeg>,
    release_cb: HookReleaseCallback,
}

fn fake() -> &'static Mutex<FakeState> {
    static FAKE: OnceLock<Mutex<FakeState>> = OnceLock::new();
    FAKE.get_or_init(|| Mutex::new(FakeState::default()))
}

/// Play the app's `free(ptr)` — containment lookup, removal, then the REAL
/// production release callback with the registration's cookie.
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
    let cb = cb.expect("a release callback must be installed");
    // SAFETY: one cookie per registration, consumed exactly once.
    unsafe { cb(ptr as *mut c_void, seg.cookie) };
}

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
    fake().lock().expect("fake").release_cb = cb;
    RC_OK
}

/// The fake hook: NOT_ARMED windows (no borrow window here) plus the
/// real segment registry; the rest spreads from `HookApi::inert()`'s
/// inert `RC_OK` stubs.
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
// The C fixture (crib of rmw_adopt_take_test — two forgeable f32 seqs)
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
struct CScan {
    angle_min: f32,
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

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn scan_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let mut ranges = ffi::rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr("ranges"),
        type_id_: ROS_TYPE_FLOAT,
        offset_: std::mem::offset_of!(CScan, ranges) as u32,
        ..Default::default()
    };
    ranges.is_array_ = true;
    let mut intensities = ffi::rosidl_typesupport_introspection_c__MessageMember {
        name_: cstr("intensities"),
        type_id_: ROS_TYPE_FLOAT,
        offset_: std::mem::offset_of!(CScan, intensities) as u32,
        ..Default::default()
    };
    intensities.is_array_ = true;
    let members = Box::leak(
        vec![
            ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("angle_min"),
                type_id_: ROS_TYPE_FLOAT,
                offset_: 0,
                ..Default::default()
            },
            ranges,
            intensities,
            ffi::rosidl_typesupport_introspection_c__MessageMember {
                name_: cstr("frame_id"),
                type_id_: ROS_TYPE_STRING,
                offset_: std::mem::offset_of!(CScan, frame_id) as u32,
                ..Default::default()
            },
        ]
        .into_boxed_slice(),
    );
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_adopt_za__msg"),
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

// =====================================================================
// Harness
// =====================================================================

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

fn unique_suffix() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("clock")
        .as_nanos() as u64
}

unsafe fn setup_pair(
    tag: &str,
    suffix: u64,
) -> (
    *mut ffi::rmw_node_t,
    *mut ffi::rmw_publisher_t,
    *mut ffi::rmw_subscription_t,
) {
    let mut options: Box<ffi::rmw_init_options_t> = Box::new(std::mem::zeroed());
    let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
    assert_eq!(rmw_init_options_init(&mut *options, allocator), RMW_RET_OK);
    let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
    assert_eq!(rmw_init(&*options, context), RMW_RET_OK);
    Box::leak(options);
    let node = rmw_create_node(context, cstr(&format!("{tag}_{suffix}")), cstr("/"));
    assert!(!node.is_null());
    let ts = scan_ts(&format!("Za{tag}{suffix}"));
    let topic = CString::new(format!("/rmw_adopt_za/{tag}/{suffix}")).expect("topic");
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
    let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
    let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
    assert!(!subscription.is_null());
    let publisher = rmw_create_publisher(node, ts, topic.as_ptr(), &qos, &pub_opts);
    assert!(!publisher.is_null());
    (node, publisher, subscription)
}

unsafe fn publish_scan(publisher: *const ffi::rmw_publisher_t, n: usize, seed: f32) {
    let rdata = calloc(n, 4) as *mut f32;
    let idata = calloc(n, 4) as *mut f32;
    for i in 0..n {
        *rdata.add(i) = seed + i as f32;
        *idata.add(i) = seed - i as f32;
    }
    let sdata = calloc(6, 1) as *mut u8;
    std::ptr::copy_nonoverlapping("laser".as_ptr(), sdata, 5);
    let msg = CScan {
        angle_min: seed,
        ranges: CF32Seq {
            data: rdata,
            size: n,
            capacity: n,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        intensities: CF32Seq {
            data: idata,
            size: n,
            capacity: n,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            is_rosidl_buffer: false,
            #[cfg(cerulion_has_is_rosidl_buffer)]
            owns_rosidl_buffer: false,
        },
        frame_id: CRosString {
            data: sdata,
            size: 5,
            capacity: 6,
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

/// Free an ADOPTED message's forged members through the fake hook (the
/// REAL release callback runs) and `fini` the rest — outside any measured
/// window.
unsafe fn free_adopted(msg: &mut CScan) {
    for seq in [&mut msg.ranges as *mut CF32Seq, &mut msg.intensities] {
        let seq = &mut *seq;
        if !seq.data.is_null() {
            simulate_free(seq.data as usize);
            seq.data = std::ptr::null_mut();
        }
    }
    scan_fini(msg as *mut CScan as *mut c_void);
}

// =====================================================================
// The arms
// =====================================================================

const WARMUP: usize = 8;
const MEASURED: usize = 6;

/// HEADLINE: after warm-up, each ADOPTED take performs
/// EXACTLY ONE Rust-heap allocation — the structural `Arc<AdoptedSample>`
/// — across `MEASURED` consecutive takes. A variant that restores
/// per-take pooled-scratch Vecs fails this with ~3-4× the count.
#[test]
#[serial]
fn an_adopted_take_costs_exactly_one_rust_allocation_the_arc() {
    {
        let mut s = fake().lock().expect("fake");
        *s = FakeState::default();
    }
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let (node, publisher, subscription) = setup_pair("arc", suffix);

        // Warm-up: grow the subscription's adopt scratch, the fake hook's
        // segment Vec (held simultaneously so its capacity covers the
        // measured window's 2×MEASURED live ranges), iceoryx2 connections
        // and the latches. All freed before measuring.
        for i in 0..WARMUP {
            publish_scan(publisher, 16, i as f32);
        }
        let mut warm: Vec<Box<CScan>> = (0..WARMUP)
            .map(|_| {
                let mut m: Box<CScan> = Box::new(std::mem::zeroed());
                scan_init(&mut *m as *mut _ as *mut c_void, 0);
                m
            })
            .collect();
        for m in warm.iter_mut() {
            let mut taken = false;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut **m as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut(),
                ),
                RMW_RET_OK
            );
            assert!(taken, "warm-up take must serve");
        }
        for m in warm.iter_mut() {
            free_adopted(m);
        }
        drop(warm);

        // The measured window covers ONLY the takes: frames pre-published,
        // messages pre-initialized, frees afterwards.
        for i in 0..MEASURED {
            publish_scan(publisher, 16, 100.0 + i as f32);
        }
        let mut msgs: Vec<Box<CScan>> = (0..MEASURED)
            .map(|_| {
                let mut m: Box<CScan> = Box::new(std::mem::zeroed());
                scan_init(&mut *m as *mut _ as *mut c_void, 0);
                m
            })
            .collect();

        ALLOCATOR.enable();
        for m in msgs.iter_mut() {
            let mut taken = false;
            let ret = rmw_take(
                subscription,
                &mut **m as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            );
            assert_eq!(ret, RMW_RET_OK);
            assert!(taken, "measured take must serve");
        }
        let allocs = ALLOCATOR.disable();
        assert_eq!(
            allocs, MEASURED as u64,
            "an adopted take must cost EXACTLY one Rust allocation (the \
             structural Arc<AdoptedSample>) — got {allocs} over {MEASURED} takes"
        );

        // Sanity: they really adopted (two live ranges per held message).
        assert_eq!(
            fake().lock().expect("fake").segments.len(),
            2 * MEASURED,
            "every measured take must have adopted"
        );
        // Counting ranges is not
        // enough. An implementation that copied each sequence to a libc heap
        // buffer and registered THOSE would register the same two ranges per
        // take and still pass the allocation count — the copies are made by
        // the C allocator, which this Rust probe does not see — while losing
        // the zero-copy retention the whole mode exists for. So each
        // registered range must lie INSIDE the sample its cookie names, which
        // a registered heap copy cannot satisfy. Same oracle as the sibling
        // adopt test's pointer-identity arm.
        for seg in &fake().lock().expect("fake").segments {
            // SAFETY: every cookie the adoption branch mints is
            // `Arc::into_raw(Arc<AdoptedSample>)`, alive while its
            // registration is live in the fake registry.
            let adopted = &*(seg.cookie as *const AdoptedSample);
            let payload = adopted.payload();
            let base = payload.as_ptr() as usize;
            assert!(
                seg.start >= base && seg.start + seg.len <= base + payload.len(),
                "a registered range must ALIAS the held sample, not a copy of it: \
                 range {:#x}..{:#x} is outside the sample payload {:#x}..{:#x}",
                seg.start,
                seg.start + seg.len,
                base,
                base + payload.len()
            );
        }
        for m in msgs.iter_mut() {
            free_adopted(m);
        }
        drop(msgs);

        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

/// The COPY-PATH control: with the gate unarmed the plain take performs
/// ZERO Rust-heap allocations at steady state (its payload copies are libc
/// `malloc`, invisible here by design and not claimed — see the module
/// docs).
#[test]
#[serial]
fn a_copy_path_take_costs_zero_rust_allocations() {
    let _env = EnvVarGuard::unset(ADOPT_TAKE_ENV);
    unsafe {
        let suffix = unique_suffix();
        let (node, publisher, subscription) = setup_pair("copy", suffix);

        let mut msg: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *msg as *mut _ as *mut c_void, 0);
        for i in 0..WARMUP {
            publish_scan(publisher, 16, i as f32);
            let mut taken = false;
            assert_eq!(
                rmw_take(
                    subscription,
                    &mut *msg as *mut _ as *mut c_void,
                    &mut taken,
                    std::ptr::null_mut(),
                ),
                RMW_RET_OK
            );
            assert!(taken);
        }

        for i in 0..MEASURED {
            publish_scan(publisher, 16, 200.0 + i as f32);
        }
        ALLOCATOR.enable();
        for _ in 0..MEASURED {
            let mut taken = false;
            let ret = rmw_take(
                subscription,
                &mut *msg as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            );
            assert_eq!(ret, RMW_RET_OK);
            assert!(taken, "measured copy take must serve");
        }
        let allocs = ALLOCATOR.disable();
        assert_eq!(
            allocs, 0,
            "the copy-path take must cost zero RUST allocations at steady state"
        );

        scan_fini(&mut *msg as *mut _ as *mut c_void);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}

unsafe fn sub_data(subscription: *const ffi::rmw_subscription_t) -> &'static SubscriptionData {
    &*((*subscription).data as *const SubscriptionData)
}

/// Hot path: the adoption scratch — the
/// forged-range walk and the registration rollback ledger — is RESERVED
/// at subscription create to the type's forgeable-member count, so the
/// FIRST adopted take grows neither Vec; the headline arm above pins the
/// steady state, this one pins the first frame. Capacity is the oracle
/// (deterministic; a first-frame allocation count would also see the
/// transport's and the latches' one-time first-use allocations, which the
/// headline arm warms past). A variant using `Vec::new()` fails at the first
/// assertion (0 < 2) and again after the take (grown to 2).
#[test]
#[serial]
fn the_adopt_scratch_is_reserved_at_create_so_the_first_take_grows_nothing() {
    {
        let mut s = fake().lock().expect("fake");
        *s = FakeState::default();
    }
    let _hook = TestHookGuard::install(fake_hook_api());
    let _env = EnvVarGuard::set(ADOPT_TAKE_ENV, "1");
    let _bud = EnvVarGuard::unset(ADOPT_TAKE_BUDGET_ENV);
    unsafe {
        let suffix = unique_suffix();
        let (node, publisher, subscription) = setup_pair("cap", suffix);
        let data = sub_data(subscription);
        let forgeable = data.bridge.forged_sequence_count();
        assert_eq!(forgeable, 2, "the scan fixture has two forgeable sequences");
        let (ranges_cap, ledger_cap) = {
            let inner = data.inner.lock().expect("inner");
            (
                inner.adopt_ranges.capacity(),
                inner.adopt_registered.capacity(),
            )
        };
        assert!(
            ranges_cap >= forgeable && ledger_cap >= forgeable,
            "the scratch must be reserved at create to the forgeable-member count \
             (ranges cap {ranges_cap}, ledger cap {ledger_cap}, need {forgeable})"
        );

        // The FIRST adopted take.
        publish_scan(publisher, 16, 1.0);
        let mut m: Box<CScan> = Box::new(std::mem::zeroed());
        scan_init(&mut *m as *mut _ as *mut c_void, 0);
        let mut taken = false;
        assert_eq!(
            rmw_take(
                subscription,
                &mut *m as *mut _ as *mut c_void,
                &mut taken,
                std::ptr::null_mut(),
            ),
            RMW_RET_OK
        );
        assert!(taken, "the first take must adopt");
        assert_eq!(
            fake().lock().expect("fake").segments.len(),
            forgeable,
            "both forgeable members were adopted on the first take"
        );
        let (ranges_after, ledger_after) = {
            let inner = data.inner.lock().expect("inner");
            (
                inner.adopt_ranges.capacity(),
                inner.adopt_registered.capacity(),
            )
        };
        assert_eq!(
            (ranges_after, ledger_after),
            (ranges_cap, ledger_cap),
            "the first adopted take must not grow the scratch"
        );
        free_adopted(&mut m);
        assert_eq!(rmw_destroy_publisher(node, publisher), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
