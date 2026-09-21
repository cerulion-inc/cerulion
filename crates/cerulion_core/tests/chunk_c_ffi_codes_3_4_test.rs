// SPDX-License-Identifier: AGPL-3.0-only
//! Defensive end-to-end coverage for FFI codes 3 (NODES
//! mutex poisoned) and 4 (handle not found) on `cerulion_node_tick`,
//! exercised by raw libloading (bypassing `DylibNodeEntry`).
//!
//! Why test these defensively:
//!
//! The cdylib FFI surface is public — a future raw-libloading
//! consumer (custom loader, language binding, etc.) may pass a stale
//! handle or hit the poison cascade. Both codes are unreachable via
//! the normal `DylibNodeEntry` flow:
//!
//! - Code 4 is unreachable because `DylibNodeEntry::tick` only ever
//!   sends its own valid handle to the FFI.
//! - Code 3 is unreachable via `DylibNodeEntry::tick` directly
//!   (a panic in tick still returns `Err(TransportError)` to the
//!   host, surfacing as code 2 to the caller), but the cdylib's
//!   NODES mutex DOES get poisoned by the unwind. Any subsequent
//!   FFI call on the same loaded cdylib hits the poison check at
//!   the top of `cerulion_node_tick` / `_init` / `_shutdown` and
//!   returns code 3 (or 0 sentinel for init) with `LAST_ERROR =
//!   "<entry>: NODES mutex poisoned"`. This test exercises that
//!   cascade explicitly so the contract is locked.
//!
//! ## Why raw libloading
//!
//! `DylibNodeEntry` is the host-side wrapper around the FFI symbols.
//! Loading the cdylib directly via `libloading::Library` lets these
//! tests assert on the raw `i32` return codes and the `LAST_ERROR`
//! C-string contents — the contract that ALL FFI consumers see, not
//! just the loader Cerulion ships.
//!
//! ## Concurrency: `#[serial]` + combined lifecycle
//!
//! Tests in this file mutate `CER_FAIL_MODE` (process-global env
//! var) and depend on the cdylib's `NODES` static, which is
//! process-local: on macOS/Linux, `dlopen` refcounts so every
//! `libloading::Library::new(path)` within the same binary sees
//! the same `NODES`. Without serialization, concurrent tests
//! would race on both surfaces.
//!
//! Each `#[test]` here carries `#[serial]` (from the `serial_test`
//! crate), which serializes all marked tests within this binary
//! via a process-global mutex. Cross-file isolation comes for
//! free from cargo's per-binary process boundary — each
//! `tests/*.rs` runs in its own process.
//!
//! **Important caveat.** `#[serial]` provides mutex-based
//! serialization but does NOT guarantee FIFO acquisition order.
//! When tests have correctness dependencies on which one runs
//! first (e.g., code-4 assertions must precede the panic-poison
//! phase, because poisoning leaves `NODES` permanently `Err` for
//! every subsequent FFI call in this binary), `#[serial]` alone
//! is insufficient.
//!
//! For that case we fold ALL ordered assertions into a single
//! `#[test]` body — `cdylib_ffi_codes_3_and_4_full_lifecycle` —
//! whose 5 sequential phases — (a) code-4 missing handles,
//! (b) init + first tick → code 2 (panic-poisons NODES),
//! (c) second tick → code 3, (d) third tick → code 3 persists,
//! (e) cascade through init + shutdown under poison — run in
//! fixed order regardless of how libtest schedules other tests. A design that
//! splits codes 3 and 4 into separate `#[test]` functions and
//! relies on libtest's alphabetical scheduling (`cargo test --
//! --help`: "By default, the tests are run in alphabetical
//! order") flakes under loaded parallel runs because of the
//! non-FIFO mutex acquisition.
//!
//! Independent tests (e.g. `env_var_guard_cleans_up_on_panic`,
//! `libtest_help_pins_documented_flags_and_defaults`) carry
//! `#[serial]` for env-mutation / subprocess isolation only;
//! they have no ordering dependency on the lifecycle test.

use cerulion_core::clock::RealClock;
use cerulion_core::graph::node::{AnyPublisher, AnySubscriber, NodeContext, ShutdownSignal};
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use serial_test::serial;
use std::ffi::CStr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::SystemTime;

// Tests in this file mutate `CER_FAIL_MODE`
// (process-global env var). Each test is `#[serial]` so they run
// one at a time within this binary. See `chunk_c_ffi_error_test.rs`
// for the rationale.

/// RAII guard that removes a process-global env var on drop.
///
/// Pairs with `set_var` at the top of any test that mutates env
/// state so cleanup runs even if an assertion panics mid-test.
/// Without this, a panic between `set_var` and the closing
/// `remove_var` leaks the env state to the next test in this
/// binary — `#[serial]` serializes the test bodies but does NOT
/// reset env between them.
struct EnvVarGuard {
    name: &'static str,
}

impl EnvVarGuard {
    fn set(name: &'static str, value: &str) -> Self {
        std::env::set_var(name, value);
        Self { name }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.name);
    }
}

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("test/{base}/{nanos}/{id}")
}

fn find_failing_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_failing_cdylib")
}

/// Read + free `LAST_ERROR` via the cdylib's own free function.
/// Asserts non-null and returns the message text.
unsafe fn take_last_error(
    take: &libloading::Symbol<'_, unsafe extern "C" fn() -> *mut std::ffi::c_char>,
    free: &libloading::Symbol<'_, unsafe extern "C" fn(*mut std::ffi::c_char)>,
    context: &str,
) -> String {
    let raw = unsafe { take() };
    assert!(
        !raw.is_null(),
        "cerulion_take_last_error must return non-null in {context}"
    );
    let msg = unsafe { CStr::from_ptr(raw) }
        .to_string_lossy()
        .into_owned();
    unsafe { free(raw) };
    msg
}

/// Combined FFI code 3 + code 4 contract test (single body).
///
/// All cdylib FFI assertions run in ONE test body to eliminate
/// cross-test ordering concerns. Splitting them into separate tests
/// (which we tried initially) flaked because:
///
/// - On macOS/Linux, `dlopen` refcounts and re-opening the same
///   shared library returns the SAME loaded image. The cdylib's
///   `NODES: Mutex<...>` static is therefore shared across every
///   `libloading::Library::new(path)` call within the same process.
/// - The panic-poison phase poisons NODES for the rest of the
///   process. If a sibling test in the same binary that asserts
///   `code == 4` happens to acquire the `#[serial]` mutex AFTER
///   the panic-poison test, it sees NODES already poisoned and
///   gets `code == 3` instead — flake.
/// - libtest's alphabetical scheduling is best-effort; the
///   `#[serial]` mutex-acquisition order is NOT guaranteed FIFO.
///
/// Combining all FFI assertions into one `#[test]` body makes the
/// phase ordering an in-body guarantee: phases run sequentially
/// regardless of how libtest schedules other tests.
///
/// ## Phases
///
///   phase 1: code 4 — boundary handles (0, u64::MAX) + 99_999 stale.
///            NODES is fresh; assertions hold unconditionally.
///   phase 2: init + first tick → code 2 (panic caught); NODES
///            gets poisoned via guard Drop during unwind.
///   phase 3: second tick → code 3 (NODES poisoned).
///   phase 4: third tick → still code 3 (poison persists, no
///            auto-recovery).
///   phase 5: cascade — init under poison → 0 sentinel + entrypoint-
///            specific LAST_ERROR; shutdown under poison → code 3 +
///            entrypoint-specific LAST_ERROR.
///
/// Mirrors `chunk_c_ffi_error_test.rs::cdylib_shutdown_handle_zero_returns_code_4`
/// (for the code-4 boundary phase) and pins the full code-3 cascade
/// the macro generates at `cerulion_macros/src/codegen.rs:983-1113`.
#[test]
#[serial]
fn cdylib_ffi_codes_3_and_4_full_lifecycle() {
    // Set CER_FAIL_MODE=tick_panic for the whole test. In phase 1 the
    // code-4 assertions hit the missing-handle branch of
    // `cerulion_node_tick` BEFORE the fixture's `fail_mode()` is
    // consulted, so the env value is irrelevant for that phase.
    // Then phase 2 uses it to trigger the panic that poisons NODES.
    // `EnvVarGuard` ensures cleanup even if any assertion panics.
    let _env = EnvVarGuard::set("CER_FAIL_MODE", "tick_panic");

    let path = find_failing_cdylib();
    let lib = unsafe { libloading::Library::new(&path) }.expect("load failing cdylib");

    let init_fn: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_void) -> u64> =
        unsafe { lib.get(b"cerulion_node_init") }.expect("cerulion_node_init export");
    let tick_fn: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> =
        unsafe { lib.get(b"cerulion_node_tick") }.expect("cerulion_node_tick export");
    let shutdown_fn: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> =
        unsafe { lib.get(b"cerulion_node_shutdown") }.expect("cerulion_node_shutdown export");
    // The new ABI-v7 pump entrypoint participates in the
    // same NODES-poison cascade as tick/init/shutdown.
    let pump_history_fn: libloading::Symbol<unsafe extern "C" fn(u64) -> i32> =
        unsafe { lib.get(b"cerulion_node_pump_history") }
            .expect("cerulion_node_pump_history export");
    let take_last_error_fn: libloading::Symbol<unsafe extern "C" fn() -> *mut std::ffi::c_char> =
        unsafe { lib.get(b"cerulion_take_last_error") }.expect("cerulion_take_last_error export");
    let free_error_fn: libloading::Symbol<unsafe extern "C" fn(*mut std::ffi::c_char)> =
        unsafe { lib.get(b"cerulion_free_error") }.expect("cerulion_free_error export");

    // ===== phase 1: code 4 — missing handles (NODES is fresh) =====
    //
    // Boundary values + 99_999 stale: all three must return code 4
    // with a `LAST_ERROR` payload identifying the entrypoint and the
    // "not found" diagnostic. The handle id itself is intentionally
    // NOT asserted in the LAST_ERROR substring check — the macro
    // currently formats it with `{}`, but a future tweak to `{:#}`
    // or hex would break a brittle substring without changing the
    // contract. We assert only the structural components (entrypoint
    // name + "not found"). Three distinct handle values together
    // pin the "missing-handle path is uniform" contract.
    //
    // Handle 0 is the FFI init-failed sentinel — a buggy raw-FFI
    // consumer that ignores `init`'s return value and tick-loops on
    // 0 is the most likely real-world miswiring. `u64::MAX` is a
    // boundary value never produced by `NEXT_HANDLE.fetch_add`.
    // `99_999` is well above any plausible `NEXT_HANDLE` value
    // assigned during this test's later init call. None of these
    // values can be a valid handle.
    for &(label, handle) in &[
        ("zero (init-failed sentinel)", 0u64),
        ("u64::MAX", u64::MAX),
        ("99_999 (stale)", 99_999_u64),
    ] {
        let code = unsafe { tick_fn(handle) };
        assert_eq!(
            code, 4,
            "phase 1: tick({label}) must return code 4 (handle not found); got {code}"
        );
        let msg = unsafe {
            take_last_error(
                &take_last_error_fn,
                &free_error_fn,
                &format!("phase 1 tick({label})"),
            )
        };
        assert!(
            msg.contains("cerulion_node_tick"),
            "phase 1: LAST_ERROR must identify the FFI entrypoint for tick({label}); got: {msg}"
        );
        assert!(
            msg.contains("not found"),
            "phase 1: LAST_ERROR must indicate the handle was not registered for tick({label}); got: {msg}"
        );
    }

    // ===== phase 2: init + panic-tick → code 2 (poisons NODES) =====
    //
    // Build a NodeContext for `cerulion_node_init`. The fixture's
    // `#[output] out: Vector3` declares the port name `out`.
    let topic = unique_topic("tick_panic_poison");
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
    // The cdylib's `cerulion_node_init` reconstructs the Box at
    // `codegen.rs:997` (`let ctx = unsafe { *Box::from_raw(ctx_ptr) };`),
    // destructuring it into a stack-local `ctx` value before any
    // fallible step. From that point on there is no Box — only a
    // moved-around `NodeContext` value. On the success path it
    // moves into the entry stashed in NODES; on the user-init-Err
    // path the `entry` (now owning `ctx`) drops at function-end;
    // on the NODES-already-poisoned path (not reached here because
    // NODES is fresh — phase 1 only attempted `tick`, which does
    // NOT poison on missing handle) the same `entry` drops because
    // reconstruction precedes the lock check. So `Box::into_raw`
    // here is NOT a leak on its own — it cleanly transfers
    // ownership to the cdylib in every path.
    //
    // The genuine leak is downstream: once NODES is poisoned by the
    // first tick's panic-unwind, neither `cerulion_node_shutdown`
    // nor the cdylib's static destructors can remove the entry from
    // NODES — the lock is frozen Err for the rest of the process.
    // The NodeContext (and its iceoryx2 `CerulionPublisher`, etc.)
    // therefore leaks for the remainder of this test binary's life.
    // We accept this leak: (a) the test process exits seconds later,
    // (b) calling `cerulion_node_shutdown(handle)` after poisoning
    // would just return code 3 without freeing anything, and (c)
    // the alternative (don't poison NODES) defeats the test.
    let ctx_ptr = Box::into_raw(Box::new(ctx)) as *mut std::ffi::c_void;

    let handle = unsafe { init_fn(ctx_ptr) };
    assert_ne!(
        handle, 0,
        "phase 2: init must succeed (NODES is fresh after phase 1, which only \
         exercised missing-handle paths that do not touch the lock state)"
    );

    // First tick — panic caught by catch_unwind, returns code 2.
    // The NODES guard drops during unwind, poisoning the mutex.
    let code_panic = unsafe { tick_fn(handle) };
    assert_eq!(
        code_panic, 2,
        "phase 2: first tick must return code 2 (panic caught by catch_unwind); got {code_panic}"
    );
    let msg_panic =
        unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 2 code-2 tick") };
    assert!(
        msg_panic.contains("panic caught by catch_unwind"),
        "phase 2: code-2 LAST_ERROR must be the panic-catch message; got: {msg_panic}"
    );

    // ===== phase 3: second tick → code 3 (NODES is now poisoned) =====
    let code_poison = unsafe { tick_fn(handle) };
    assert_eq!(
        code_poison, 3,
        "phase 3: second tick must return code 3 (NODES mutex poisoned by phase 2 panic-unwind); got {code_poison}"
    );
    let msg_poison =
        unsafe { take_last_error(&take_last_error_fn, &free_error_fn, "phase 3 code-3 tick") };
    assert!(
        msg_poison.contains("cerulion_node_tick"),
        "phase 3: code-3 LAST_ERROR must identify the FFI entrypoint; got: {msg_poison}"
    );
    assert!(
        msg_poison.contains("NODES mutex poisoned"),
        "phase 3: code-3 LAST_ERROR must mention the NODES mutex poison; got: {msg_poison}"
    );

    // ===== phase 4: third tick → still code 3 (no auto-recovery) =====
    //
    // Locks the contract that NODES does NOT auto-recover between
    // calls. A future change that silently "heals" the poisoned
    // mutex (e.g., by calling `clear_poison` or replacing NODES)
    // would flip this assertion.
    let code_persist = unsafe { tick_fn(handle) };
    assert_eq!(
        code_persist, 3,
        "phase 4: third tick must remain code 3 (poison persists across calls); got {code_persist}"
    );

    // ===== phase 5: cascade through init + shutdown under poison =====
    //
    // Every NODES-touching FFI entrypoint must detect the poison and
    // use its OWN entrypoint identifier in `LAST_ERROR`. A future
    // refactor that consolidates the poison check across entrypoints
    // could otherwise silently drop a per-entry identifier; the
    // assertions below catch that regression.
    //
    // codegen.rs:997 consumes the Box BEFORE the NODES lock check,
    // so we need a fresh stub ctx for this call (the original ctx
    // was consumed by the successful phase 2 init above).
    let stub_ctx = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        Arc::new(RealClock),
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    let stub_ctx_ptr = Box::into_raw(Box::new(stub_ctx)) as *mut std::ffi::c_void;
    let init_under_poison = unsafe { init_fn(stub_ctx_ptr) };
    assert_eq!(
        init_under_poison, 0,
        "phase 5: init under poison must return 0 (sentinel); got {init_under_poison}"
    );
    let init_msg = unsafe {
        take_last_error(
            &take_last_error_fn,
            &free_error_fn,
            "phase 5 init under poison",
        )
    };
    assert!(
        init_msg.contains("cerulion_node_init"),
        "phase 5: init-under-poison LAST_ERROR must identify the init entrypoint; got: {init_msg}"
    );
    assert!(
        init_msg.contains("NODES mutex poisoned"),
        "phase 5: init-under-poison LAST_ERROR must mention NODES poison; got: {init_msg}"
    );

    let shutdown_under_poison = unsafe { shutdown_fn(handle) };
    assert_eq!(
        shutdown_under_poison, 3,
        "phase 5: shutdown under poison must return code 3; got {shutdown_under_poison}"
    );
    let sd_msg = unsafe {
        take_last_error(
            &take_last_error_fn,
            &free_error_fn,
            "phase 5 shutdown under poison",
        )
    };
    assert!(
        sd_msg.contains("cerulion_node_shutdown"),
        "phase 5: shutdown-under-poison LAST_ERROR must identify the shutdown entrypoint; got: {sd_msg}"
    );
    assert!(
        sd_msg.contains("NODES mutex poisoned"),
        "phase 5: shutdown-under-poison LAST_ERROR must mention NODES poison; got: {sd_msg}"
    );

    // ===== phase 5: pump_history under poison =====
    //
    // The ABI-v7 `cerulion_node_pump_history` entrypoint must participate in
    // the SAME poison cascade as tick/init/shutdown: code 3 + a LAST_ERROR
    // that names ITS OWN entrypoint. Pins that the macro generated the pump's
    // poison branch (a refactor consolidating the poison check could otherwise
    // drop the per-entry identifier for the newest entrypoint).
    let pump_under_poison = unsafe { pump_history_fn(handle) };
    assert_eq!(
        pump_under_poison, 3,
        "phase 5: pump_history under poison must return code 3; got {pump_under_poison}"
    );
    let pump_msg = unsafe {
        take_last_error(
            &take_last_error_fn,
            &free_error_fn,
            "phase 5 pump_history under poison",
        )
    };
    assert!(
        pump_msg.contains("cerulion_node_pump_history"),
        "phase 5: pump-under-poison LAST_ERROR must identify the pump_history entrypoint; got: {pump_msg}"
    );
    assert!(
        pump_msg.contains("NODES mutex poisoned"),
        "phase 5: pump-under-poison LAST_ERROR must mention NODES poison; got: {pump_msg}"
    );

    // `_env` drops here, removing CER_FAIL_MODE even if any of the
    // phase assertions above had panicked.
}

/// Prove `EnvVarGuard` cleans up via Drop
/// even when the test body panics.
///
/// The whole point of `EnvVarGuard` is panic safety: a future edit
/// that adds an assertion between `set_var` and the natural Drop
/// must not leak the env var to subsequent tests. This test pins
/// that contract by deliberately panicking inside the guard's
/// scope (via `catch_unwind`) and then asserting the env var is
/// unset after.
///
/// Uses a distinct probe env var name (`CER_FAIL_MODE_TEST_GUARD_PROBE`)
/// so this test does not interact with the cdylib's `CER_FAIL_MODE`
/// reading. `#[serial]` still serializes against the other tests
/// in the file because POSIX `setenv` is not thread-safe — a
/// concurrent setenv from a sibling test could race against the
/// `EnvVarGuard::set` here.
#[test]
#[serial]
fn env_var_guard_cleans_up_on_panic() {
    const PROBE: &str = "CER_FAIL_MODE_TEST_GUARD_PROBE";

    // Defensive: ensure no leftover from a prior run.
    std::env::remove_var(PROBE);
    assert!(
        std::env::var(PROBE).is_err(),
        "precondition: probe env var must be unset at test entry"
    );

    let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _env = EnvVarGuard::set(PROBE, "set_value");
        // Confirm the set actually took effect — otherwise the
        // Drop assertion below is meaningless.
        assert_eq!(
            std::env::var(PROBE).as_deref(),
            Ok("set_value"),
            "EnvVarGuard::set must have set the probe env var"
        );
        // Force a panic so the guard's Drop must run via unwind,
        // not via normal scope exit.
        panic!("intentional panic to exercise EnvVarGuard::Drop on unwind");
    }))
    .is_err();
    assert!(
        panicked,
        "catch_unwind closure must have caught the intentional panic"
    );

    // Post-condition: env var must be unset because Drop ran during
    // the unwind. If `EnvVarGuard::Drop` ever changes from
    // `remove_var` to a fallible operation that can short-circuit,
    // this assertion catches the regression.
    assert!(
        std::env::var(PROBE).is_err(),
        "EnvVarGuard::Drop must have removed {PROBE:?} during panic unwind; \
         it is still set, meaning the guard's panic-safety contract is broken"
    );
}

/// Oracle test pinning the libtest
/// `--help` text + flags that the docs and CI commands cite.
///
/// Multiple places in this repo depend on libtest's documented
/// surface:
///
/// 1. **File header at lines ~63-65** — the "Concurrency:
///    `#[serial]` + combined lifecycle" section cites the
///    alphabetical-default sentence verbatim when explaining why
///    a split-test design flakes.
/// 2. **`AGENTS.md` testing section** — the serial-test commands
///    pass `-- --test-threads=1` to control intra-binary parallelism.
/// 3. **`.github/workflows/ci.yml` serial test jobs** — same
///    `--test-threads=1` invocations.
///
/// If a future rustc renames, removes, or reformats any of these
/// flags, our citations rot silently. This test invokes the current
/// test binary's libtest with `--help`, normalizes whitespace, and
/// asserts each cited substring is present. A failure here lists
/// which substring is missing — the maintainer updates BOTH this
/// test AND the corresponding doc/CI references in lockstep.
///
/// The substring set is intentionally small (3 pins). Each pin is
/// a verbatim fragment from `cargo test -- --help`. Pins 1-2 each
/// serve as canary for a specific doc/CI reference elsewhere in
/// this repo; pin 3 (`--include-ignored`) is forward-looking — a
/// sentinel for libtest CLI surface reshuffles that would also
/// affect any future use of that flag. Adding more pins here costs
/// little; the test runs in milliseconds.
#[test]
#[serial]
fn libtest_help_pins_documented_flags_and_defaults() {
    let exe = std::env::current_exe().expect("current_exe");
    let output = std::process::Command::new(&exe)
        .arg("--help")
        .output()
        .expect("invoke own test binary with --help");
    assert!(
        output.status.success(),
        "test binary --help must exit 0; got status {:?}, stderr: {}",
        output.status,
        String::from_utf8_lossy(&output.stderr),
    );
    let text = String::from_utf8_lossy(&output.stdout);

    // libtest's --help line-wraps the paragraph at ~80 columns, so a
    // substring like "Use --shuffle to run the tests in random order."
    // is split across a newline. Collapse all whitespace runs to
    // single spaces before matching so the test pins the words, not
    // the wrapping.
    let normalized: String = text.split_whitespace().collect::<Vec<_>>().join(" ");

    let pins: &[(&str, &str)] = &[
        // Pin 1: the alphabetical-default sentence cited in this
        // file's header at lines ~63-65.
        (
            "alphabetical default + --shuffle (cited in file header)",
            "By default, the tests are run in alphabetical order. \
             Use --shuffle to run the tests in random order.",
        ),
        // Pin 2: the `--test-threads n_threads` flag cited in
        // `AGENTS.md` testing section and `.github/workflows/ci.yml`
        // serial-test commands. Note the placeholder is lowercase
        // `n_threads` in this rustc's libtest — verbatim per output
        // of `cargo test -- --help`.
        (
            "--test-threads flag (cited in CLAUDE.md + ci.yml)",
            "--test-threads n_threads",
        ),
        // Pin 3: the `--include-ignored` flag, which is the
        // documented escape hatch for any test marked `#[ignore]`.
        // We don't currently use it but the CLI surface is part of
        // the documented contract; if libtest reshuffles, this
        // catches it.
        (
            "--include-ignored flag (documented test-runner contract)",
            "--include-ignored",
        ),
    ];

    let mut failures: Vec<String> = Vec::new();
    for (label, substring) in pins {
        if !normalized.contains(substring) {
            failures.push(format!("  - {label}: missing substring {substring:?}"));
        }
    }
    assert!(
        failures.is_empty(),
        "libtest --help no longer contains expected substrings; \
         our citations need updating. Missing:\n{}\n\n\
         Whitespace-normalized --help (truncated at 2KB):\n---\n{}\n---",
        failures.join("\n"),
        normalized.chars().take(2048).collect::<String>(),
    );
}
