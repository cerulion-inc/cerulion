// SPDX-License-Identifier: AGPL-3.0-only
//! Regression pin: the cdylib-local stderr tracing subscriber
//! (`install_cdylib_stderr_tracing`, called from the macro-generated
//! `cerulion_node_init`) makes node-side `tracing` events host-visible.
//!
//! # The bug this pins
//!
//! A cdylib statically links its OWN copy of `cerulion_core` + `tracing`, so its
//! `tracing` global dispatcher is a distinct static from the host binary's. The
//! host initializing ITS subscriber does nothing for the cdylib's static, so
//! every `tracing::error!`/`warn!`/`info!` emitted by node-side code — including
//! cerulion_core's loud-by-design `OutputProxy` discard error ("OutputProxy
//! dropped without writing all declared variable fields; skipping publish") —
//! dispatched to a no-op. Production dylib graphs ran silently broken. The
//! stopgap installs an `fmt` subscriber (respecting the env-snapshot RUST_LOG)
//! on the cdylib's own static at node init, so those events land on stderr.
//!
//! # Why a subprocess
//!
//! libtest CAPTURES its own process's stdout/stderr and offers no API to read it
//! back, so a test cannot assert on the very stderr it is producing. This
//! re-execs THIS test binary (`current_exe --exact subprocess_child_discard_probe
//! --ignored --nocapture`) as a child, pipes the CHILD's stderr, drives a
//! 1-node (or 2-node) graph over real iceoryx2 (`build_for_test`) that loads the
//! `test_node_discard_probe_cdylib` fixture, and asserts on the captured stderr.
//! The child installs NO host subscriber and uses NO `#[traced_test]` — the pin
//! is that the CDYLIB's own subscriber writes to the child process stderr.
//!
//! Oracle: the literal error/warn strings (NOT a self-compare). Test 1 FAILS
//! without the subscriber (the black hole — nothing reaches stderr). Portable: runs on
//! macOS AND Linux CI (no `cfg(linux)` gate).
//!
//! # The 8 pins
//!
//! 1. `RUST_LOG=info` surfaces the discard error EXACTLY ONCE across the firing
//!    steps (the discard error is now FLOOD-LATCHED — first occurrence
//!    is loud at `error!`, sustained occurrences downgrade to `debug!` and are
//!    filtered out at info) AND the user warn.
//! 2. `RUST_LOG=off` suppresses the discard error, the user warn, AND the info
//!    probe — the env-snapshot spec is genuinely applied (anti-tautology).
//! 3. Absent `RUST_LOG` defaults to EXACTLY info-level visibility: discard
//!    error + user warn + info probe present, debug probe ABSENT.
//! 4. `CER_DISCARD_MODE=healthy` (all fields written): NO discard error, user
//!    warn still present — the error is tied to the unwritten-field condition.
//! 5. TWO instances of one cdylib init twice without panic + exactly ONE
//!    install breadcrumb (single-install semantics). Counted under an EXPLICIT
//!    `RUST_LOG`, because the breadcrumb prints only when one is set.
//! 6. An unparseable `RUST_LOG` falls back to "info" LOUDLY: the fallback
//!    marker appears in the breadcrumb and the discard error stays visible.
//! 7. `RUST_LOG=debug` shows the flood-latch's TWO levels: the loud
//!    first `error!` (exactly once) AND the sustained `debug!` suppressed lines
//!    (>= 1) — the once-per-regime error + downgraded-repeat contract e2e.
//!    (RECOVERY — the `info!` re-arm — cannot be exercised here: the fixture's
//!    discard mode is fixed per run, so it never heals; recovery is pinned in
//!    the pure `output_discard_latch_test.rs` oracle vectors.)
//! 8. Absent `RUST_LOG` prints NO breadcrumb at all, while the node-side logs
//!    still reach stderr (the positive anchor): the breadcrumb names an
//!    effective filter only when the user supplied one, so a default run
//!    carries no per-cdylib announcement line.
//!
//! # Running (needs the fixture built first; `#[serial]` — each parent test
//! spawns a child that builds over the iceoryx2 SHM singleton)
//!
//! ```bash
//! # The fixture must be built in the SAME profile as the test binary — the
//! # `debug!`-gated oracles compare the host's static level against what the
//! # child emits, and a sibling-profile fixture is refused, never silently used.
//! cargo build -p test_node_discard_probe_cdylib
//! cargo test -p cerulion_core --test cdylib_tracing_stopgap_test -- --test-threads=1
//! cargo build --release -p test_node_discard_probe_cdylib
//! cargo test -p cerulion_core --release --test cdylib_tracing_stopgap_test -- --test-threads=1
//! ```

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::testing::{debug_level_compiled_in, line_level};
use indexmap::IndexMap;
use serial_test::serial;

// --- Child re-exec protocol (env switches) --------------------------------

/// Set to "1" ONLY on the spawned child invocation, so a bare `-- --ignored`
/// full run of `subprocess_child_discard_probe` no-ops instead of false-firing.
const ENV_CHILD: &str = "CER_TRACING_STOPGAP_CHILD";
/// Number of fixture instances the child wires into its graph (default 1).
const ENV_INSTANCES: &str = "CER_TRACING_STOPGAP_INSTANCES";
/// Consumed by the FIXTURE (`test_node_discard_probe_cdylib`): "healthy" ⇒ write
/// all variable fields (no discard error); anything else ⇒ leave fields unwritten.
const ENV_DISCARD_MODE: &str = "CER_DISCARD_MODE";

/// The fixture is `period_ms = 10`; step the virtual clock by the period so the
/// node fires (and publishes) every step.
const STEP: Duration = Duration::from_millis(10);
/// Firing steps the child runs. > 1 so the discard error is emitted repeatedly.
const CHILD_STEPS: usize = 5;
/// Hard cap on the child's runtime — SIGKILL past this so a hung child (e.g. a
/// wedged iceoryx2 init) never blocks CI. Generous: cold iceoryx2 init + 5 steps.
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);

// --- Oracle strings --------------------------------------------------------

/// cerulion_core's loud `OutputProxy::drop` discard error (transport/output_proxy.rs).
/// This is now the FIRST-of-regime `error!` message ONLY; the
/// downgraded sustained lines use [`SUPPRESSED_DEBUG`] (a distinct string that
/// does NOT contain this substring), so `matches(DISCARD_ERROR).count()`
/// isolates error-level occurrences under any filter.
const DISCARD_ERROR: &str = "OutputProxy dropped without writing all declared variable fields";
/// The flood-latch's SUSTAINED `debug!` line (transport/output_proxy.rs).
/// Only visible under a `debug` filter; deliberately shares no substring with
/// [`DISCARD_ERROR`] so the two levels are separately countable.
const SUPPRESSED_DEBUG: &str = "OutputProxy discard suppressed";
/// The fixture's per-tick USER `tracing::warn!` — pins that user tick logs surface.
const USER_WARN: &str = "discard probe tick fired";
/// The fixture's per-tick USER `tracing::info!` — pins the info side of the
/// default filter (a default mutated to "warn"/"error" loses this line).
const INFO_PROBE: &str = "discard probe info probe";
/// The fixture's per-tick USER `tracing::debug!` — its ABSENCE under the default
/// pins the filter at exactly "info" (a default mutated to "debug"/"trace"
/// shows this line).
const DEBUG_PROBE: &str = "discard probe debug probe";
/// The installer's one-per-cdylib startup breadcrumb (prefix — the `(filter: ...)`
/// suffix varies by arm).
const BREADCRUMB: &str = "cerulion cdylib tracing: node-side logs -> stderr";
/// The breadcrumb's filter description when an unparseable RUST_LOG fell back.
const FALLBACK_MARKER: &str = "info (fallback: RUST_LOG spec unparseable)";
/// libtest / std panic prefix — its ABSENCE proves the two-instance init is clean.
const PANIC_MARKER: &str = "panicked at";

/// Lines of the child's stderr carrying `marker` at `level`, PANICKING if a
/// line carrying the marker sits at any other level.
///
/// The ONE body, `cerulion_core::testing::count_at_exclusively`, over this
/// stream: the level is part of the predicate on purpose (the discard error is
/// the latch's loud first-of-regime `error!`, and a bare substring
/// count would still read 1 if it were demoted to `warn!`/`info!`), and the
/// level-free total is paired with it so a SECOND copy at another level cannot
/// pass either. Sound on this stream because the cdylib subscriber writes with
/// `with_ansi(false)` — no colour escapes wrap the level token, and the shared
/// helper reads it from the line HEADER.
///
/// The panic direction rests on that shape: this is the child's RAW stderr, not
/// `tracing-test`'s capture, so a line carrying the marker without a parseable
/// `<ts> <LEVEL> <target>:` header (a wrapped line, a panic backtrace echoing
/// the message) would refuse rather than count. Every caller here asks for
/// (`"ERROR"`, `DISCARD_ERROR`), whose DEBUG twin deliberately shares no
/// substring with it (see the const's own doc).
fn count_at(stderr: &str, level: &str, marker: &str) -> usize {
    let lines: Vec<&str> = stderr.lines().collect();
    cerulion_core::testing::count_at_exclusively(&lines, level, &[marker])
        .unwrap_or_else(|e| panic!("{e}\n--- child stderr ---\n{stderr}"))
}

// --- Fixture locator -------------------------------------------------------

/// SAME profile as this test binary, never a sibling-profile fallback: the
/// `debug!`-gated oracles below compare the HOST's static level against what the
/// CHILD emits, and a release test that quietly loaded the debug fixture would
/// skip the suppressed-line requirement while the child still emits it. The
/// locator refuses loudly, naming the build, when this profile's fixture is
/// absent.
fn discard_probe_cdylib_path() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib_same_profile("test_node_discard_probe_cdylib")
}

/// Loud precondition: fail with the exact build command if the fixture is absent.
fn assert_fixture_built() {
    let path = discard_probe_cdylib_path();
    assert!(
        path.exists(),
        "discard-probe test cdylib not found at {path:?}. Run \
         `cargo build -p test_node_discard_probe_cdylib` first — under the SAME profile as \
         this test (`--release` for a `cargo test --release` run); a sibling-profile fixture \
         is refused, never silently used."
    );
}

// --- Child graph builder + entrypoint --------------------------------------

/// A pure-source graph of `instances` fixture nodes, each publishing its own
/// `image_out` topic (id-derived, so no single-writer collision). No consumer —
/// observability is the child's stderr, not the payload.
fn build_probe_graph(instances: usize) -> TransportResult<GraphRuntime> {
    let path = discard_probe_cdylib_path();
    let mut nodes = Vec::new();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for i in 0..instances {
        let id = format!("probe{i}");
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: id.clone(),
            node_type: "discard_probe".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "image_out".to_string(),
                schema: "Image".to_string(),
                max_slice_len: Some(4096),
                history_size: 0,
                topic: None,
            }],
        });
        factories.insert(id, Box::new(DylibNodeEntry::load(&path)?));
    }
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "discard_probe".to_string(),
        prefix: "stopgap".to_string(),
        nodes,
    };
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8)
}

/// Re-invoked as a subprocess by the parent tests (`--exact ... --ignored`). A
/// no-op pass when `CER_TRACING_STOPGAP_CHILD` is unset (so a bare `-- --ignored` run of this
/// binary doesn't false-fire). Builds + steps the probe graph, then exits 0.
#[test]
#[ignore]
// P12 exemption, scoped to this fn rather than the file: this is the body of a
// SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
// code IS the channel the parent reads its verdict from. The ban stays armed for
// every other line in this binary, which is the half a file-wide allow gave up.
#[allow(clippy::disallowed_methods)]
fn subprocess_child_discard_probe() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let instances = std::env::var(ENV_INSTANCES)
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(1)
        .max(1);
    // `.expect` failures panic → libtest marks failed → non-zero exit, caught by
    // the parent's `.success()` assert.
    let mut rt = build_probe_graph(instances).expect("build discard-probe graph");
    for _ in 0..CHILD_STEPS {
        rt.step(STEP);
    }
    drop(rt);
    // Explicit clean exit so libtest's own stdout summary / late bookkeeping
    // cannot perturb the exit status the parent reads.
    std::process::exit(0);
}

// --- Parent spawn + capture ------------------------------------------------

struct ChildOutcome {
    success: bool,
    stderr: String,
}

/// Spawn the child with a controlled env, capture its stderr, bounded-wait for
/// exit (SIGKILL on timeout). `rust_log = None` REMOVES RUST_LOG from the child
/// env (also clearing any the CI harness carries); `Some(v)` sets it.
fn run_child(rust_log: Option<&str>, discard_mode: Option<&str>, instances: usize) -> ChildOutcome {
    assert_fixture_built();

    let exe = std::env::current_exe().expect("current_exe for child re-exec");
    let mut cmd = std::process::Command::new(exe);
    cmd.args([
        "--exact",
        "subprocess_child_discard_probe",
        "--ignored",
        "--nocapture",
    ]);
    cmd.env(ENV_CHILD, "1");
    cmd.env(ENV_INSTANCES, instances.to_string());
    match rust_log {
        Some(v) => {
            cmd.env("RUST_LOG", v);
        }
        None => {
            cmd.env_remove("RUST_LOG");
        }
    }
    match discard_mode {
        Some(m) => {
            cmd.env(ENV_DISCARD_MODE, m);
        }
        None => {
            cmd.env_remove(ENV_DISCARD_MODE);
        }
    }
    cmd.stdin(std::process::Stdio::null());
    // stdout carries libtest's own summary (nulled — noise); stderr carries the
    // cdylib subscriber output we assert on.
    cmd.stdout(std::process::Stdio::null());
    cmd.stderr(std::process::Stdio::piped());

    let mut child = cmd.spawn().expect("spawn discard-probe child process");

    // Drain stderr on a helper thread so a full pipe can never wedge the child.
    let mut stderr_pipe = child.stderr.take().expect("child stderr is piped");
    let (tx, rx) = std::sync::mpsc::channel();
    let drain = std::thread::spawn(move || {
        use std::io::Read;
        let mut buf = String::new();
        // A read error (or non-UTF-8 stderr) is the HARNESS failing: it must
        // not ship an empty capture that every absence assertion accepts. The
        // panic drops `tx`, and the parent's `expect` on the receive fires.
        stderr_pipe
            .read_to_string(&mut buf)
            .expect("read the child's stderr to EOF");
        let _ = tx.send(buf);
    });

    // Bounded wait: poll try_wait, SIGKILL + reap at the deadline.
    let deadline = std::time::Instant::now() + CHILD_TIMEOUT;
    let status = loop {
        match child.try_wait().expect("try_wait on discard-probe child") {
            Some(status) => break status,
            None => {
                if std::time::Instant::now() >= deadline {
                    let _ = child.kill();
                    break child.wait().expect("reap killed discard-probe child");
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        }
    };

    // A capture timeout is the HARNESS failing, never "the child printed nothing":
    // an empty string would satisfy every absence assertion in this file.
    let stderr = rx
        .recv_timeout(Duration::from_secs(10))
        .expect("child stderr capture timed out — the harness's drain thread, not the child");
    let _ = drain.join();

    ChildOutcome {
        success: status.success(),
        stderr,
    }
}

// --- Parent tests ----------------------------------------------------------

/// THE regression guard: with RUST_LOG=info the cdylib subscriber routes the
/// OutputProxy discard error AND the user warn to the process stderr. With
/// no subscriber in the cdylib's tracing static, stderr carries neither.
///
/// The discard error is FLOOD-LATCHED. Under `info` it appears
/// EXACTLY ONCE across the firing steps — the loud first-of-regime `error!`; the
/// sustained repeats downgrade to `debug!` (filtered out at info). This `== 1`
/// IS the contract (a per-tick error would make it `>= 2`);
/// the two-level flood-latch behavior is pinned under a `debug` filter by
/// `debug_filter_shows_first_error_and_suppressed_debug_lines` below, and the
/// recovery/re-arm by the pure `output_discard_latch_test.rs`.
#[test]
#[serial]
fn discard_error_is_host_visible_on_stderr_with_rust_log_info() {
    let out = run_child(Some("info"), None, 1);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    assert!(
        count_at(&out.stderr, "ERROR", DISCARD_ERROR) >= 1,
        "RUST_LOG=info must surface the cdylib OutputProxy discard error on stderr, \
         at ERROR (the black-hole regression). stderr:\n{}",
        out.stderr
    );
    // Flood-latch: the loud `error!` fires ONCE per discard regime. The
    // fixture discards every tick (one unbroken regime across CHILD_STEPS), so
    // the error appears EXACTLY once; the sustained repeats are `debug!` and
    // filtered at info. A regression to per-tick error! (unlatched) shows
    // CHILD_STEPS occurrences and fails here; a never-error regression shows 0.
    let occurrences = out.stderr.matches(DISCARD_ERROR).count();
    assert_eq!(
        occurrences, 1,
        "the discard error must be flood-latched to exactly ONE loud error! across \
         {CHILD_STEPS} steps (one discard regime), got {occurrences}. stderr:\n{}",
        out.stderr
    );
    // That one line is the loud `error!` HEAD: matched with its level token, so
    // a head demoted to `warn!`/`info!` — still one line, same text — fails here
    // instead of passing as "flood-latched".
    let at_error = count_at(&out.stderr, "ERROR", DISCARD_ERROR);
    assert_eq!(
        at_error, 1,
        "the one discard line must carry the ERROR level token (the loud \
         first-of-regime error!), got {at_error} at ERROR. stderr:\n{}",
        out.stderr
    );
    // The debug-level suppressed line must NOT appear at info (it is only
    // emitted at debug) — proves the sustained repeats were downgraded, not
    // dropped or kept loud.
    assert!(
        !out.stderr.contains(SUPPRESSED_DEBUG),
        "at info the downgraded (debug) suppressed line must be filtered out. stderr:\n{}",
        out.stderr
    );
    assert!(
        out.stderr.contains(USER_WARN),
        "RUST_LOG=info must surface the node's own tick warn on stderr. stderr:\n{}",
        out.stderr
    );
}

/// Flood-latch, both levels: under `RUST_LOG=debug` the discard error
/// still fires EXACTLY ONCE (the loud first-of-regime `error!`), and the
/// sustained repeats now surface as `debug!` suppressed lines (>= 1). This is
/// the e2e proof the repeats are DOWNGRADED (not dropped): at info they were
/// invisible (test 1), at debug they reappear at debug level. The `error` count
/// staying 1 while `debug` lines accumulate kills both a per-tick-error
/// regression (would show >1 error) and a never-suppress regression (would show
/// 0 debug lines).
#[test]
#[serial]
fn debug_filter_shows_first_error_and_suppressed_debug_lines() {
    let out = run_child(Some("debug"), None, 1);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    let errors = out.stderr.matches(DISCARD_ERROR).count();
    assert_eq!(
        errors, 1,
        "even at debug the loud error! fires ONCE per regime (flood-latched), \
         got {errors}. stderr:\n{}",
        out.stderr
    );
    // …and that one line carries the ERROR level token (the level is the contract).
    let at_error = count_at(&out.stderr, "ERROR", DISCARD_ERROR);
    assert_eq!(
        at_error, 1,
        "the one discard line must be the loud error! head, got {at_error} at ERROR. \
         stderr:\n{}",
        out.stderr
    );
    let suppressed = out.stderr.matches(SUPPRESSED_DEBUG).count();
    // Level-free twin: a suppressed line never carries a LOUD level token.
    let loud = out
        .stderr
        .lines()
        .filter(|l| {
            l.contains(SUPPRESSED_DEBUG) && matches!(line_level(l), Some("WARN" | "INFO" | "ERROR"))
        })
        .count();
    assert_eq!(
        loud, 0,
        "a suppressed discard line was emitted LOUDLY. stderr:\n{}",
        out.stderr
    );
    // The child cdylib is built in this binary's profile: where `debug!` is
    // compiled out there, no suppressed line can exist at any filter.
    assert!(
        !debug_level_compiled_in() || suppressed >= 1,
        "the sustained discards must surface as downgraded debug! lines under a \
         debug filter (>= 1 across {CHILD_STEPS} steps), got {suppressed}. stderr:\n{}",
        out.stderr
    );
}

/// Anti-tautology control for test 1: RUST_LOG=off proves the env-snapshot spec
/// is genuinely applied to the cdylib subscriber — the apparatus is not just
/// always-noisy. Neither the discard error nor the user warn appears.
///
/// The breadcrumb is the POSITIVE anchor: `RUST_LOG` is SET here, so it fires
/// on install regardless of the filter's level (`off` included), and this test
/// cannot pass vacuously on a child that never loaded the cdylib (an
/// all-negative body would go green on an empty stderr).
#[test]
#[serial]
fn rust_log_off_suppresses_node_side_logs() {
    let out = run_child(Some("off"), None, 1);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    assert!(
        out.stderr.contains(BREADCRUMB),
        "the install breadcrumb must appear even under RUST_LOG=off (positive \
         anchor: the child really loaded the cdylib and installed). stderr:\n{}",
        out.stderr
    );
    assert!(
        !out.stderr.contains(DISCARD_ERROR),
        "RUST_LOG=off must suppress the cdylib discard error. stderr:\n{}",
        out.stderr
    );
    assert!(
        !out.stderr.contains(USER_WARN),
        "RUST_LOG=off must suppress the node's own tick warn. stderr:\n{}",
        out.stderr
    );
    assert!(
        !out.stderr.contains(INFO_PROBE),
        "RUST_LOG=off must suppress the node's info probe. stderr:\n{}",
        out.stderr
    );
}

/// With RUST_LOG absent the cdylib subscriber defaults to EXACTLY "info": the
/// discard error, user warn, and info probe are visible; the debug probe is NOT.
/// A default mutated to "warn"/"error" loses the info probe; one mutated to
/// "debug"/"trace" shows the debug probe.
///
/// The debug-probe-ABSENT arm relies on the debug test profile: under release,
/// `tracing`'s `release_max_level_info` would compile the debug probe out
/// anyway, making the absence assert vacuously true — harmless, since this test
/// family runs in debug CI.
#[test]
#[serial]
fn rust_log_unset_defaults_to_info_and_shows_error() {
    let out = run_child(None, None, 1);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    assert!(
        count_at(&out.stderr, "ERROR", DISCARD_ERROR) >= 1,
        "absent RUST_LOG must default to \"info\" and surface the discard error at \
         ERROR. stderr:\n{}",
        out.stderr
    );
    assert!(
        out.stderr.contains(USER_WARN),
        "absent RUST_LOG (default \"info\") must surface the user warn. stderr:\n{}",
        out.stderr
    );
    assert!(
        out.stderr.contains(INFO_PROBE),
        "absent RUST_LOG (default \"info\") must surface the info probe — a \
         default mutated to \"warn\"/\"error\" fails here. stderr:\n{}",
        out.stderr
    );
    assert!(
        !out.stderr.contains(DEBUG_PROBE),
        "absent RUST_LOG (default \"info\") must NOT surface the debug probe — a \
         default mutated to \"debug\"/\"trace\" fails here. stderr:\n{}",
        out.stderr
    );
}

/// Healthy control: writing ALL variable fields publishes cleanly — no discard
/// error — while the user warn still appears. Proves the error is tied to the
/// unwritten-field condition, not to load/init.
#[test]
#[serial]
fn healthy_mode_emits_no_discard_error() {
    let out = run_child(Some("info"), Some("healthy"), 1);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    assert!(
        !out.stderr.contains(DISCARD_ERROR),
        "healthy mode writes all variable fields → no discard error. stderr:\n{}",
        out.stderr
    );
    assert!(
        out.stderr.contains(USER_WARN),
        "healthy mode still ticks → the user warn must appear (apparatus alive). \
         stderr:\n{}",
        out.stderr
    );
}

/// Single-install pin: TWO instances of the SAME cdylib in one graph each call
/// `cerulion_node_init` (→ `install_cdylib_stderr_tracing`); the second install
/// is a no-op via the `Once` guard. The graph builds + steps without panic, the
/// discard error still surfaces, and EXACTLY ONE breadcrumb appears. The child
/// runs under an EXPLICIT `RUST_LOG=info` on purpose: the breadcrumb prints
/// only when a spec is set (pin 8 is the unset twin), so counting it needs one.
///
/// Scope of the `count() == 1` pin: it pins single-install SEMANTICS,
/// not the `Once` symbol itself — with the `Once` deleted, the second call's
/// `set_global_default` would fail (default already set) and emit no second
/// breadcrumb, so the count stays 1. The `Once` additionally prevents that
/// redundant second `EnvFilter` build + failed-install attempt; no stronger
/// external observable distinguishes the two, so the behavioral half is the pin.
#[test]
#[serial]
fn two_instances_from_one_cdylib_init_twice_without_panic() {
    let out = run_child(Some("info"), None, 2);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    assert!(
        count_at(&out.stderr, "ERROR", DISCARD_ERROR) >= 1,
        "two instances must still surface the discard error at ERROR. stderr:\n{}",
        out.stderr
    );
    assert!(
        !out.stderr.contains(PANIC_MARKER),
        "installing the cdylib subscriber twice must not panic (Once-guarded). \
         stderr:\n{}",
        out.stderr
    );
    let breadcrumbs = out.stderr.matches(BREADCRUMB).count();
    assert_eq!(
        breadcrumbs, 1,
        "two inits of one cdylib must install (and announce) the subscriber \
         exactly once, got {breadcrumbs} breadcrumbs. stderr:\n{}",
        out.stderr
    );
}

/// A typo'd-but-UNPARSEABLE RUST_LOG must fall back to "info" LOUDLY:
/// `EnvFilter::try_new("foo=notalevel")` errs (the directive's level token
/// "notalevel" fails `LevelFilter::from_str` — verified against the vendored
/// tracing-subscriber 0.3.22 `Directive::parse`), so the installer eprintln!s
/// the parse failure, installs the "info" fallback (discard error visible), and
/// the breadcrumb names the fallback filter.
#[test]
#[serial]
fn rust_log_unparseable_falls_back_to_info_loudly() {
    let out = run_child(Some("foo=notalevel"), None, 1);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    assert!(
        out.stderr.contains(FALLBACK_MARKER),
        "an unparseable RUST_LOG must announce the info fallback in the \
         breadcrumb. stderr:\n{}",
        out.stderr
    );
    assert!(
        count_at(&out.stderr, "ERROR", DISCARD_ERROR) >= 1,
        "the info fallback must keep the discard error visible at ERROR. stderr:\n{}",
        out.stderr
    );
    assert!(
        out.stderr.contains(BREADCRUMB),
        "a successful (fallback) install must emit the breadcrumb. stderr:\n{}",
        out.stderr
    );
}

/// Pin 8: under the DEFAULT (RUST_LOG absent) the install prints NO breadcrumb.
/// The breadcrumb names an effective filter the user might not expect, which
/// can only happen when the user supplied one; with none supplied the filter
/// is the documented `info`, and one line per cdylib on every run is what a
/// first user learns to skim past. Positive anchor: the node-side logs still
/// reach stderr (the discard error at ERROR, the info probe), so the child
/// really loaded the cdylib and installed the subscriber; an empty stderr
/// would fail here instead of passing the absence check vacuously. The
/// explicit-`RUST_LOG` arms (pins 5 and 6) keep counting the breadcrumb, so
/// the two halves together pin "set: announced, unset: silent".
#[test]
#[serial]
fn rust_log_unset_prints_no_breadcrumb() {
    let out = run_child(None, None, 1);
    assert!(out.success, "child must exit 0; stderr:\n{}", out.stderr);
    assert!(
        count_at(&out.stderr, "ERROR", DISCARD_ERROR) >= 1,
        "positive anchor: the default install must still route the discard error to \
         stderr at ERROR. stderr:\n{}",
        out.stderr
    );
    assert!(
        out.stderr.contains(INFO_PROBE),
        "positive anchor: the default install must still surface the info probe. \
         stderr:\n{}",
        out.stderr
    );
    let breadcrumbs = out.stderr.matches(BREADCRUMB).count();
    assert_eq!(
        breadcrumbs, 0,
        "a default run (RUST_LOG absent) must print NO install breadcrumb, got \
         {breadcrumbs}. stderr:\n{}",
        out.stderr
    );
}
