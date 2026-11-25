// SPDX-License-Identifier: AGPL-3.0-only
// The crate root is `#![cfg(unix)]`, so on a non-unix target this test
// file must compile to NOTHING or its `use rmw_cerulion::…` items vanish.
#![cfg(unix)]
//! `CERULION_RMW_PERIOD_PARK` costs `rmw_wait` no heap allocation: the switch is read once,
//! when `rmw_create_wait_set` builds the wait set, and every wait applies the value kept on
//! the wait set. A wait that read the variable itself would allocate its value (a `String`)
//! on every call while it is set, so the same wait set's waits would count more allocations
//! with the variable set than unset. Here they count exactly the same with it unset, `0` and
//! a near miss, on the measured thread.
//!
//! The publisher-event pump's first pass runs before any window is measured. The pump runs
//! inside `rmw_wait` once per 20 ms of wall clock from `rmw_init`, and its first pass takes the
//! publisher registry's lock for the first time; where `std`'s `Mutex` is pthread-backed
//! (macOS) that first lock allocates the mutex, one allocation in whichever window the pass
//! lands in. Every later pass, with no publisher registered, allocates nothing.
//!
//! Where the park runs (a Linux build that carries it, on a CPU with a monitor-wait
//! primitive), the test also pins the read-once rule through `predicting_waits`: a wait set
//! created with the variable unset keeps running the park after `0` is set, and a wait set
//! created after that runs none.
//!
//! # Why a separate binary?
//!
//! `#[global_allocator]` is process-wide; the counter is additionally THREAD-scoped (the
//! `rmw_borrow_zero_alloc_test` pattern): background threads (libtest machinery) allocate at
//! unpredictable times and are out of contract. One `#[serial]` test
//! body, so no sibling test thread exists during a measured window.
//!
//! ```bash
//! cargo test -p rmw_cerulion --test rmw_period_park_switch_alloc_test -- --test-threads=1
//! ```

use serial_test::serial;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::test_seams::EnvVarGuard;
use rmw_cerulion::*;

// ---------------------------------------------------------------------------
// Counting allocator (thread-scoped; counts alloc + alloc_zeroed + realloc)
// ---------------------------------------------------------------------------

thread_local! {
    /// `true` only on the thread that called [`CountingAllocator::enable`]. Const-init with a
    /// non-`Drop` payload: a lazy TLS init touched from inside `GlobalAlloc::alloc` would
    /// recurse into the allocator.
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
            // `try_with`, never `with`: during thread teardown TLS is inaccessible and `with`
            // would panic inside the allocator.
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
// Fixture: one f64 message type (crib of `rmw_wait_event_test.rs`)
// ---------------------------------------------------------------------------

const ROS_TYPE_DOUBLE: u8 = 2;
const VAR: &str = "CERULION_RMW_PERIOD_PARK";
/// Waits per measured window: enough that one allocation per wait could not hide.
const WAITS: u64 = 64;

fn cstr(s: &str) -> *const c_char {
    CString::new(s).expect("cstr").into_raw()
}

fn f64_ts(unique: &str) -> *const ffi::rosidl_message_type_support_t {
    let members = Box::leak(Box::new([
        ffi::rosidl_typesupport_introspection_c__MessageMember {
            name_: cstr("x"),
            type_id_: ROS_TYPE_DOUBLE,
            offset_: 0,
            ..Default::default()
        },
    ]));
    let mm = Box::leak(Box::new(
        ffi::rosidl_typesupport_introspection_c__MessageMembers {
            message_namespace_: cstr("rmw_ppa__msg"),
            message_name_: cstr(unique),
            member_count_: 1,
            size_of_: std::mem::size_of::<f64>(),
            members_: members.as_ptr(),
            ..Default::default()
        },
    ));
    Box::leak(Box::new(ffi::rosidl_message_type_support_t {
        typesupport_identifier: cstr("rosidl_typesupport_introspection_c"),
        data: mm as *const _ as *const c_void,
        ..Default::default()
    }))
}

/// One zero-timeout wait on `subscription` (nothing is ever published, so it times out).
unsafe fn wait_once(subscription: *mut ffi::rmw_subscription_t, ws: *mut ffi::rmw_wait_set_t) {
    let mut slot = [(*subscription).data];
    let mut subs = ffi::rmw_subscriptions_t {
        subscriber_count: 1,
        subscribers: slot.as_mut_ptr(),
    };
    let zero = ffi::rmw_time_t { sec: 0, nsec: 0 };
    let ret = rmw_wait(
        &mut subs,
        std::ptr::null_mut(),
        std::ptr::null_mut(),
        std::ptr::null_mut(),
        std::ptr::null_mut(),
        ws,
        &zero,
    );
    assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "nothing was published");
}

/// Returns once the publisher-event pump has run a pass, so its first pass, which allocates
/// the registry's lock on its first use where that lock is pthread-backed, is behind every
/// measured window. The pump claims a pass once per 20 ms of wall clock and says whether it
/// ran.
fn run_a_publisher_event_pump_pass() {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !rmw_cerulion::runtime::pump_publisher_events() {
        assert!(
            Instant::now() < deadline,
            "the publisher-event pump ran no pass in 5 s: the runtime is not initialized"
        );
        std::thread::yield_now();
    }
}

/// Allocations on this thread over [`WAITS`] waits, after a warm-up that fills every reused
/// buffer.
unsafe fn allocs_over_waits(
    subscription: *mut ffi::rmw_subscription_t,
    ws: *mut ffi::rmw_wait_set_t,
) -> u64 {
    for _ in 0..8 {
        wait_once(subscription, ws);
    }
    ALLOCATOR.enable();
    for _ in 0..WAITS {
        wait_once(subscription, ws);
    }
    ALLOCATOR.disable()
}

#[test]
#[serial]
fn the_period_park_switch_costs_rmw_wait_no_allocation_and_is_read_at_wait_set_creation() {
    // Restored when the guards drop. The two other flags that turn the park off are unset so
    // the read-once arm below measures this switch alone.
    let _unset = EnvVarGuard::unset(VAR);
    let _monitor = EnvVarGuard::unset("CERULION_MONITOR_WAIT");
    let _event = EnvVarGuard::unset("CERULION_RMW_EVENT_WAIT");
    unsafe {
        let mut options: ffi::rmw_init_options_t = std::mem::zeroed();
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&options, context), RMW_RET_OK);
        let node = rmw_create_node(context, cstr("pp_switch_alloc"), cstr("/"));
        assert!(!node.is_null());
        let suffix = std::process::id();
        let ts = f64_ts(&format!("PpAlloc{suffix}"));
        let topic = CString::new(format!("/rmw_ppa/{suffix}")).expect("topic");
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
        let sub_opts: ffi::rmw_subscription_options_t = std::mem::zeroed();
        // No publisher: the subscription's wait maps the topic's doorbell page by creating it,
        // so where the park runs every call takes the period-learned park, and the
        // publisher-event pump, which runs on a 20 ms wall-clock cadence, has no publisher to
        // pump, so once its first pass has run no measured window depends on whether it came
        // due.
        let subscription = rmw_create_subscription(node, ts, topic.as_ptr(), &qos, &sub_opts);
        assert!(!subscription.is_null());

        // One wait set, created with the variable unset; the variable then changes under it.
        let ws = rmw_create_wait_set(context, 8);
        assert!(!ws.is_null());
        run_a_publisher_event_pump_pass();
        let unset = allocs_over_waits(subscription, ws);
        let off_guard = EnvVarGuard::set(VAR, "0");
        let off = allocs_over_waits(subscription, ws);
        let near_miss_guard = EnvVarGuard::set(VAR, "on");
        let near_miss = allocs_over_waits(subscription, ws);
        drop(near_miss_guard);
        // The counter counts: every wait clones its subscription's topic name, and one box is one.
        assert!(
            unset >= WAITS,
            "{WAITS} waits counted only {unset} allocations: the counter is not counting"
        );
        ALLOCATOR.enable();
        let poke = std::hint::black_box(Box::new(41u64));
        assert!(ALLOCATOR.disable() >= 1, "a Box counted no allocation");
        drop(poke);
        assert_eq!(
            (off, near_miss),
            (unset, unset),
            "the waits of one wait set must count the same allocations over {WAITS} waits \
             whatever {VAR} holds (unset {unset}, 0 {off}, near miss {near_miss}): a wait \
             that reads the variable allocates its value on every call"
        );

        #[cfg(cerulion_period_park)]
        {
            let runs =
                cfg!(target_os = "linux") && cerulion_core::monitor_wait::monitor_wait_available();
            let data = &*((*ws).data as *const WaitSetData);
            let before = data.predicting_waits.load(Ordering::Relaxed);
            wait_once(subscription, ws);
            assert_eq!(
                data.predicting_waits.load(Ordering::Relaxed) - before,
                u64::from(runs),
                "a wait set created with {VAR} unset keeps its value after 0 is set"
            );
            let later = rmw_create_wait_set(context, 8);
            assert!(!later.is_null());
            wait_once(subscription, later);
            let later_data = &*((*later).data as *const WaitSetData);
            assert_eq!(
                later_data.predicting_waits.load(Ordering::Relaxed),
                0,
                "a wait set created after 0 is set runs no learned park"
            );
            assert_eq!(rmw_destroy_wait_set(later), RMW_RET_OK);
        }

        drop(off_guard);
        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
        assert_eq!(rmw_destroy_subscription(node, subscription), RMW_RET_OK);
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
}
