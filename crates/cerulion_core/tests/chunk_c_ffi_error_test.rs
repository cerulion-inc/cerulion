// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end test that the rich
//! NodeError message threads from a real cdylib through
//! `DylibNodeEntry::tick` and surfaces in the `TransportError`
//! variant the loader returns.
//!
//! A numeric FFI return code alone gives the loader nothing beyond a
//! generic class label ("logic error", "panic", etc.).
//! So the cdylib stashes the actual `NodeError::Display`
//! string in a thread-local, the loader pulls it via
//! `cerulion_take_last_error`, and the message lands in the
//! `TransportError::NodeError { reason: ... }` field that callers
//! actually log / display.
//!
//! This test loads the dedicated `test_node_failing_cdylib` fixture
//! (whose `tick` always returns
//! `NodeError::Logic("simulated failure for chunk C runtime_error_2 test")`),
//! init's it, ticks it once, and asserts the loader's error contains
//! that exact message text.

// Tests in this file mutate `CER_FAIL_MODE`
// (process-global env var) and rely on the cdylib's `NODES` static
// (process-local via `dlopen` refcounting on macOS/Linux). Both are
// process-global state — concurrent execution races. The `#[serial]`
// attribute from `serial_test` serializes all marked tests within
// this binary so the bodies run one at a time.
//
// `#[serial]` provides mutex-based serialization but does NOT
// guarantee FIFO acquisition order. Tests whose correctness depends
// on running in a specific order (e.g., `cdylib_tick_lifecycle_codes_1_and_2`
// below: code 1 must precede code 2 because phase 2 poisons NODES)
// must combine the ordered assertions into a single `#[test]` body.
// `#[serial]` only protects against arbitrary interleaving; combining
// is what protects against scheduling-induced reordering.

use cerulion_core::wire::MaxSliceLen;
use serial_test::serial;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::SystemTime;

use cerulion_core::clock::RealClock;
use cerulion_core::graph::node::{
    AnyPublisher, AnySubscriber, DylibNodeEntry, NodeContext, NodeEntry, ShutdownSignal,
};
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/chunk_c/{base}/{nanos}/{id}")
}

/// Locate the `test_node_failing_cdylib` shared library produced by the
/// workspace build. The fixture is declared as a workspace member, so
/// `cargo test` triggers the cdylib build automatically.
fn find_failing_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_failing_cdylib")
}

/// A deferred fix: combined lifecycle test for FFI code 1
/// (logic error) and code 2 (panic).
///
/// Originally split into `cdylib_tick_failure_surfaces_rich_error_message`
/// and `cdylib_tick_panic_surfaces_code_2`. The two tests are
/// alphabetically adjacent (`tick_failure` < `tick_panic`) and
/// share the cdylib's process-local `NODES` static (via `dlopen`
/// refcounting). Under `cargo test --workspace`'s default parallel
/// scheduling, the `#[serial]` mutex serialized the test BODIES but
/// did NOT guarantee FIFO acquisition order — a sufficiently-loaded
/// scheduler could pre-empt `tick_failure` with `tick_panic`,
/// poisoning NODES and causing the next `tick_failure` run (in the
/// same binary process) to fail at init with "NODES mutex poisoned".
///
/// Combining the two contracts into a single `#[test]` body makes
/// the phase 1 → phase 2 ordering an in-body invariant — libtest's
/// scheduler can never reorder phases within one test.
///
/// First phase (code 1, logic error): tick_logic mode. init succeeds;
/// tick returns Err(Logic) with the rich message; shutdown succeeds
/// cleanly. NODES is healthy at end of phase 1.
///
/// Second phase (code 2, panic): tick_panic mode. init succeeds; tick
/// panics → catch_unwind returns code 2 with "panic caught by
/// catch_unwind" message; the macro's NODES guard drops during
/// unwind, poisoning NODES. Shutdown after this returns code 3
/// (poisoned) — we discard the result because the panic itself is
/// the assertion.
#[test]
#[serial]
fn cdylib_tick_lifecycle_codes_1_and_2() {
    // ===== phase 1: code 1 — Logic error (tick_logic mode) =====
    //
    // Defensive: ensure a sibling test didn't leak CER_FAIL_MODE.
    // The default fixture mode (Logic error) is what this phase
    // depends on. `#[serial]` makes the leak window narrower but
    // not impossible (if a future panic-poisoning test runs first
    // and skips its env remove via panic, leak survives mutex
    // release).
    std::env::remove_var("CER_FAIL_MODE");
    let path = find_failing_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("phase 1: load failing cdylib");

    let topic = unique_topic("out_logic");
    let tt = TestTransport::with_buffer_size(8);
    let out_pub = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let _out_sub = tt.subscriber(&topic);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(out_pub));
    let subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    let ctx = NodeContext::with_runtime_env(
        publishers,
        subscribers,
        Arc::new(RealClock),
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    node.init(ctx)
        .expect("phase 1: init must succeed (fixture only fails on tick)");

    let err = node
        .tick()
        .expect_err("phase 1: tick must propagate the simulated failure");
    let msg = err.to_string();
    assert!(
        msg.contains("simulated failure for chunk C runtime_error_2 test"),
        "phase 1: tick error must include the rich NodeError message threaded \
         through cerulion_take_last_error; got:\n  {msg}"
    );
    assert!(
        msg.contains("tick failed:"),
        "phase 1: message must carry the cdylib's `tick failed:` prefix; got:\n  {msg}"
    );

    // Second tick must produce a fresh rich error (consumption-then-
    // refill cycle through LAST_ERROR).
    let err2 = node
        .tick()
        .expect_err("phase 1: second tick must also fail");
    assert!(
        err2.to_string()
            .contains("simulated failure for chunk C runtime_error_2 test"),
        "phase 1: second tick must produce a fresh rich error message; got:\n  {err2}"
    );

    // Shutdown succeeds; NODES is restored to a healthy state for
    // phase 2's fresh init call.
    node.shutdown().expect("phase 1: shutdown");

    // ===== phase 2: code 2 — panic in tick (tick_panic mode) =====
    //
    // This phase mutates `CER_FAIL_MODE` and POISONS NODES at the end.
    // It MUST run after phase 1 — combining the two phases in
    // one test body guarantees this regardless of scheduler whims.
    std::env::set_var("CER_FAIL_MODE", "tick_panic");
    let mut node = DylibNodeEntry::load(&path).expect("phase 2: load");

    let topic = unique_topic("out_panic");
    let tt2 = TestTransport::with_buffer_size(8);
    let out_pub = tt2.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let _out_sub = tt2.subscriber(&topic);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(out_pub));
    let ctx = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        Arc::new(RealClock),
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    node.init(ctx)
        .expect("phase 2: init must succeed when CER_FAIL_MODE=tick_panic (only tick panics)");

    let err = node
        .tick()
        .expect_err("phase 2: tick must propagate panic via FFI code 2");
    let msg = err.to_string();
    assert!(
        msg.contains("panic caught by catch_unwind"),
        "phase 2: tick error must be the FFI code-2 'panic caught by catch_unwind' message; got: {msg}"
    );

    // Discard shutdown result: NODES is poisoned by the panic-unwind
    // above, so cerulion_node_shutdown will return code 3. That's
    // tested separately in `chunk_c_ffi_codes_3_4_test.rs`'s
    // lifecycle test; here we just confirm code 2 was caught.
    let _ = node.shutdown();
    std::env::remove_var("CER_FAIL_MODE");

    // NODES is now poisoned. Any subsequent test that loads the same
    // cdylib in this binary will see poisoned NODES on its first
    // `cerulion_node_init` call. The other tests in this file
    // (`cdylib_init_failure_*`, `cdylib_shutdown_*`,
    // `cdylib_double_shutdown_*`) sort
    // alphabetically BEFORE `cdylib_tick_lifecycle_codes_1_and_2`,
    // so under `#[serial]` + libtest's alphabetical-queue worker
    // model they run first. Combine them into this lifecycle if a
    // future flake report indicates the alphabetical-bias assumption
    // is breaking.
}

/// FFI init failure path.
///
/// Set `CER_FAIL_MODE=init_fail` so the cdylib's `init` returns Err.
/// `cerulion_node_init` returns handle 0 + populates `LAST_ERROR`.
/// Loader's `init` must surface the rich message via the `take_last_error`
/// path.
#[test]
#[serial]
fn cdylib_init_failure_surfaces_rich_error_message() {
    std::env::set_var("CER_FAIL_MODE", "init_fail");
    let path = find_failing_cdylib();
    let mut node = DylibNodeEntry::load(&path).expect("load");

    let topic = unique_topic("out");
    let tt = TestTransport::with_buffer_size(8);
    let out_pub = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let _out_sub = tt.subscriber(&topic);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    publishers.insert("out".to_string(), AnyPublisher::Ipc(out_pub));
    let ctx = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        Arc::new(RealClock),
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );

    let err = node
        .init(ctx)
        .expect_err("init must fail with CER_FAIL_MODE=init_fail");
    let msg = err.to_string();
    assert!(
        msg.contains("simulated init failure"),
        "init error must include the user's NodeError message; got: {msg}"
    );
    assert!(
        msg.contains("init failed:"),
        "init error must carry the cdylib's `init failed:` prefix; got: {msg}"
    );

    std::env::remove_var("CER_FAIL_MODE");
}

/// `cerulion_node_shutdown` with a missing handle
/// must return code 4 + populate `LAST_ERROR`, mirroring
/// `cerulion_node_tick`. Returning 0 (silent success) would let a
/// buggy loader sending a stale or never-registered handle see "yep,
/// shut down ✓" and never diagnose the miswiring.
///
/// This test calls the FFI symbols directly via libloading to bypass
/// the host-side `DylibNodeEntry::shutdown` wrapper (which goes through
/// the registered handle path). Init is never called, so handle 1 is
/// guaranteed not to exist in the cdylib's NODES map.
#[test]
#[serial]
fn cdylib_shutdown_missing_handle_returns_code_4() {
    // The macro-generated cdylib is the cleanest fixture for this test
    // — we only need a working cerulion_node_shutdown export that
    // hasn't seen any prior init.
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib");

    let lib = unsafe { libloading::Library::new(path) }.expect("load cdylib");
    let shutdown_fn: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("cerulion_node_shutdown export");
    let take_last_error_fn: libloading::Symbol<unsafe extern "C" fn() -> *mut std::ffi::c_char> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("cerulion_take_last_error export");
    let free_error_fn: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_char)> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("cerulion_free_error export");

    // Stale handle: nothing has been init'd, so any non-zero handle is missing.
    let stale_handle: u64 = 99_999;
    let code = unsafe { shutdown_fn(stale_handle) };
    assert_eq!(
        code, 4,
        "cerulion_node_shutdown(stale) must return code 4 (handle not found); got {code}"
    );

    // LAST_ERROR must carry the rich diagnostic.
    let raw = unsafe { take_last_error_fn() };
    assert!(
        !raw.is_null(),
        "cerulion_take_last_error must return non-null after a code-4 shutdown"
    );
    let msg = unsafe { std::ffi::CStr::from_ptr(raw) }
        .to_string_lossy()
        .into_owned();
    unsafe { free_error_fn(raw) };

    assert!(
        msg.contains("cerulion_node_shutdown"),
        "LAST_ERROR must identify the FFI entrypoint; got: {msg}"
    );
    assert!(
        msg.contains(&stale_handle.to_string()),
        "LAST_ERROR must include the stale handle id; got: {msg}"
    );
    assert!(
        msg.contains("not found"),
        "LAST_ERROR must indicate the handle was not registered; got: {msg}"
    );
}

/// Idempotent shutdown: a second shutdown of a real
/// handle must also return code 4. After init+shutdown, the handle is
/// removed from the NODES map; a subsequent shutdown of the same handle
/// must be code 4, not silently 0.
#[test]
#[serial]
fn cdylib_double_shutdown_second_returns_code_4() {
    std::env::remove_var("CER_FAIL_MODE");
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib");

    let lib = unsafe { libloading::Library::new(path) }.expect("load cdylib");
    let init_fn: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_void) -> u64> =
        unsafe { lib.get(b"cerulion_node_init") }.expect("cerulion_node_init export");
    let shutdown_fn: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("cerulion_node_shutdown export");
    let take_last_error_fn: libloading::Symbol<unsafe extern "C" fn() -> *mut std::ffi::c_char> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("cerulion_take_last_error export");
    let free_error_fn: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_char)> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("cerulion_free_error export");

    // Prepare a real NodeContext and pass it to init (the macro consumes
    // the Box::into_raw on its first action, so we must not drop ours).
    let topic = unique_topic("dbl_shutdown_out");
    let tt = TestTransport::with_buffer_size(8);
    let out_pub = tt.publisher(&topic, MaxSliceLen::const_new(256), 0);
    let _out_sub = tt.subscriber(&topic);
    let mut publishers: IndexMap<String, AnyPublisher> = IndexMap::new();
    // The macro fixture's `#[output] cmd_out: Vector3` declares the
    // port name as `cmd_out` (the prior comment + insert used `cmd_vel`
    // which was wrong; this test never reaches tick so the mismatch was
    // invisible, but the wiring should still be correct for future
    // readers + any extension of this test that calls tick).
    publishers.insert("cmd_out".to_string(), AnyPublisher::Ipc(out_pub));
    let ctx = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        Arc::new(RealClock),
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    let ctx_ptr = Box::into_raw(Box::new(ctx)) as *mut std::ffi::c_void;

    let handle = unsafe { init_fn(ctx_ptr) };
    assert_ne!(handle, 0, "init must succeed");

    let code1 = unsafe { shutdown_fn(handle) };
    assert_eq!(code1, 0, "first shutdown of valid handle must return 0");
    let code2 = unsafe { shutdown_fn(handle) };
    assert_eq!(
        code2, 4,
        "second shutdown of the same handle must return code 4; got {code2}"
    );

    // Also assert the LAST_ERROR diagnostic
    // for the second shutdown carries the new "already shut down or
    // never registered" wording. A regression that left code 4 returning
    // but reverted the LAST_ERROR text would otherwise pass the prior
    // assertions silently.
    let raw = unsafe { take_last_error_fn() };
    assert!(
        !raw.is_null(),
        "take_last_error must return non-null after a code-4 shutdown"
    );
    let msg = unsafe { std::ffi::CStr::from_ptr(raw) }
        .to_string_lossy()
        .into_owned();
    unsafe { free_error_fn(raw) };
    assert!(
        msg.contains("cerulion_node_shutdown"),
        "LAST_ERROR must identify the FFI entrypoint; got: {msg}"
    );
    assert!(
        msg.contains(&handle.to_string()),
        "LAST_ERROR must include the stale handle id; got: {msg}"
    );
    assert!(
        msg.contains("already shut down or never registered"),
        "LAST_ERROR must contain the new diagnostic wording; got: {msg}"
    );

    // Third shutdown of the same handle: still missing, still code 4
    // (idempotent). Locks the contract that the handle stays absent
    // from the map after first shutdown — would catch a regression
    // that re-inserted the handle on shutdown for any reason.
    let code3 = unsafe { shutdown_fn(handle) };
    assert_eq!(code3, 4, "third shutdown must remain code 4; got {code3}");
}

/// Edge handle values for
/// `cerulion_node_shutdown`. Locks that handle 0 and u64::MAX are
/// not special-cased; both go through the same `remove(&handle)`
/// path and return 4 + LAST_ERROR.
#[test]
#[serial]
fn cdylib_shutdown_handle_zero_returns_code_4() {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib");

    let lib = unsafe { libloading::Library::new(path) }.expect("load cdylib");
    let shutdown_fn: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("cerulion_node_shutdown export");

    // Handle 0 — the same missing-handle path as a stale handle.
    assert_eq!(unsafe { shutdown_fn(0) }, 4, "shutdown(0) must return 4");
    // u64::MAX — boundary value never produced by NEXT_HANDLE.fetch_add.
    assert_eq!(
        unsafe { shutdown_fn(u64::MAX) },
        4,
        "shutdown(u64::MAX) must return 4"
    );
}

/// Consecutive stale-handle
/// shutdowns must overwrite (not accumulate) LAST_ERROR. The cdylib's
/// thread-local `LAST_ERROR` should drop the prior CString allocation
/// when overwritten — this test exercises that path by issuing two
/// stale shutdowns and reading LAST_ERROR after the second to confirm
/// it carries the SECOND handle id, not the first.
#[test]
#[serial]
fn cdylib_shutdown_consecutive_stale_overwrites_last_error() {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib");

    let lib = unsafe { libloading::Library::new(path) }.expect("load cdylib");
    let shutdown_fn: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("cerulion_node_shutdown export");
    let take_last_error_fn: libloading::Symbol<unsafe extern "C" fn() -> *mut std::ffi::c_char> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("cerulion_take_last_error export");
    let free_error_fn: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_char)> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("cerulion_free_error export");

    let first_handle: u64 = 11_111;
    let second_handle: u64 = 22_222;
    assert_eq!(unsafe { shutdown_fn(first_handle) }, 4);
    assert_eq!(unsafe { shutdown_fn(second_handle) }, 4);

    // LAST_ERROR must carry the SECOND handle's diagnostic, not the first.
    let raw = unsafe { take_last_error_fn() };
    assert!(!raw.is_null());
    let msg = unsafe { std::ffi::CStr::from_ptr(raw) }
        .to_string_lossy()
        .into_owned();
    unsafe { free_error_fn(raw) };
    assert!(
        msg.contains(&second_handle.to_string()),
        "LAST_ERROR must reflect second (overwriting) shutdown; got: {msg}"
    );
    assert!(
        !msg.contains(&first_handle.to_string()),
        "LAST_ERROR must not retain first shutdown's handle id (would indicate accumulation, not overwrite); got: {msg}"
    );
}
