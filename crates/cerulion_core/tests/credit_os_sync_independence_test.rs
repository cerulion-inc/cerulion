// SPDX-License-Identifier: AGPL-3.0-only
//! The credit plane's macOS os_sync tier is INDEPENDENT of the
//! barrier's kill switch.
//!
//! This pins a real coupling, not a hypothetical one. `credit_wake_word_
//! primitive_available` used to delegate the backend question to
//! `barrier::wake_word_block_primitive_available`, whose macOS arm is
//! `os_sync_active()` — and that ANDs in `CERULION_BARRIER_OS_SYNC`. So
//! `CERULION_BARRIER_OS_SYNC=0` silently disabled the CREDIT tier as well,
//! while `USER_API.md` promised the two switches were independent. The runtime
//! park path (`park_wait_credit`) had the SAME call and the same bug.
//!
//! **Why a subprocess.** Both switches are `OnceLock`-cached on first read, so
//! one process can observe exactly one combination. Each arm therefore re-execs
//! this binary with the env it wants and reads the answer back off stdout —
//! the same shape `cdylib_tracing_stopgap_test` and `iox2_log_level_test` use.
//!
//! **Why macOS-only.** `CERULION_CREDIT_OS_SYNC` and the whole os_sync tier are
//! macOS concepts; on Linux the credit word rides a shared futex and the
//! predicate is unconditionally true, so there would be nothing to observe.
#![cfg(target_os = "macos")]

use std::process::Command;

thread_local! {
    /// The most recent child's `park_wait_credit` verdict, so an arm can assert
    /// the RUNTIME decision as well as the predicate without changing the
    /// helper's return type.
    static PARKWAIT: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

/// The runtime park decision observed by the last `availability_with` call.
fn last_parkwait() -> bool {
    PARKWAIT.with(|c| c.get()).expect("a child ran")
}

const CHILD: &str = "child_prints_credit_primitive_availability";

/// Re-exec this test binary with `envs` applied and read the child's verdict.
fn availability_with(envs: &[(&str, &str)]) -> bool {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args(["--exact", CHILD, "--ignored", "--nocapture"]);
    // Clear BOTH switches first so an arm only ever sets what it names — an
    // inherited value from the parent's environment would silently decide the
    // answer for it.
    cmd.env_remove("CERULION_BARRIER_OS_SYNC");
    cmd.env_remove("CERULION_CREDIT_OS_SYNC");
    for (k, v) in envs {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("spawn child");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout
        .lines()
        .find(|l| l.starts_with("AVAIL="))
        .unwrap_or_else(|| panic!("child printed no AVAIL= line; stdout was:\n{stdout}"));
    let verdict = match line.trim_end() {
        "AVAIL=true" => true,
        "AVAIL=false" => false,
        other => panic!("unparseable child verdict {other:?}"),
    };
    let pw = stdout
        .lines()
        .find(|l| l.starts_with("PARKWAIT="))
        .unwrap_or_else(|| panic!("child printed no PARKWAIT= line; stdout was:\n{stdout}"));
    // FAIL CLOSED, exactly as the AVAIL verdict above does. A bare
    // `== "PARKWAIT=true"` coerces ANY other text — a truncated line, a renamed
    // marker, a child that panicked after printing half of it — to `false`,
    // which silently SATISFIES the disabled-switch assertions below. A verdict
    // parse that cannot fail turns those arms into decoration.
    let parked = match pw.trim_end() {
        "PARKWAIT=true" => true,
        "PARKWAIT=false" => false,
        other => {
            panic!("unparseable child PARKWAIT verdict {other:?}; full child stdout was:\n{stdout}")
        }
    };
    PARKWAIT.with(|c| c.set(Some(parked)));
    // Permanent evidence that each arm really spawned a child and read a real
    // answer. A subprocess oracle that silently stopped spawning would
    // otherwise pass in milliseconds and look identical to a healthy run — the
    // vacuous-pass class this file exists to avoid, not to reproduce.
    eprintln!("  ARM env={envs:?} -> AVAIL={verdict} PARKWAIT={parked}");
    verdict
}

/// The child half: report BOTH of this process's answers for the CURRENT env.
///
/// `AVAIL` is the availability PREDICATE. `PARKWAIT` is the RUNTIME park
/// decision — `park_wait_credit` returns whether it actually performed a kernel
/// wait, and its macOS arm has its own tier gate. Read-2 caught that probing
/// only `AVAIL` left the runtime site inert: restoring
/// `barrier::os_sync_active()` inside `park_wait_credit` leaves every arm
/// passing, even though that site is exactly half of the coupling this file pins.
///
/// The word is per-process (pid-scoped name) and nothing else touches it, so
/// the wait always times out — `true` means "a kernel wait was performed",
/// which is the bit under test, not "a wake arrived".
#[test]
#[ignore]
fn child_prints_credit_primitive_availability() {
    println!(
        "AVAIL={}",
        cerulion_core::credit::credit_wake_word_primitive_available()
    );
    let ns = format!("indep_{}", std::process::id());
    let id = cerulion_core::credit::credit_edge_id("/indep/data", "consumer", "inp");
    let w = cerulion_core::credit::MappedCredit::create_owned(&ns, &id, 4)
        .expect("the child owns its own credit word");
    let snap = w.wake_seq_snapshot();
    let performed = w.park_wait_credit(snap, std::time::Duration::from_millis(2));
    println!("PARKWAIT={performed}");
}

/// THE pin. Four arms, both directions, against hand-written expectations.
///
/// Restoring either `crate::barrier::os_sync_active()` (in
/// `park_wait_credit`) or `crate::barrier::wake_word_block_primitive_available()`
/// (in the predicate) fails arm 2 — that is the coupled code this test
/// exists to refuse.
#[test]
fn prc_the_credit_os_sync_tier_ignores_the_barrier_kill_switch() {
    // ARM 1 — CONTROL / anti-vacuity. With neither switch set the host must
    // report the tier AVAILABLE, otherwise this box has no usable os_sync
    // backend (macOS < 14.4, or the errno latch tripped) and every other arm
    // would read `false` for a reason that has nothing to do with the switches.
    let baseline = availability_with(&[]);
    if !baseline {
        eprintln!(
            "SKIP: no usable os_sync backend on this host (macOS < 14.4 or latched); \
             the switch-independence arms cannot be distinguished from an absent backend"
        );
        return;
    }

    // ARM 2 — THE HEADLINE, and the one that fails on the coupled code.
    // Disabling the BARRIER's tier says nothing about the CREDIT plane.
    assert!(
        availability_with(&[("CERULION_BARRIER_OS_SYNC", "0")]),
        "CERULION_BARRIER_OS_SYNC=0 must NOT disable the credit tier — the two \
         switches are documented as independent, and the credit gate must read \
         the shared BACKEND fact rather than the barrier's DECISION"
    );

    assert!(
        last_parkwait(),
        "CERULION_BARRIER_OS_SYNC=0 must NOT stop `park_wait_credit` performing its \
         kernel wait either — this is the RUNTIME half of the coupling, and probing \
         only the availability predicate left a mutant on this site alive"
    );

    // ARM 3 — the credit switch does decide, on its own.
    assert!(
        !availability_with(&[("CERULION_CREDIT_OS_SYNC", "0")]),
        "CERULION_CREDIT_OS_SYNC=0 must disable the credit tier"
    );

    assert!(
        !last_parkwait(),
        "CERULION_CREDIT_OS_SYNC=0 must also stop the runtime park's kernel wait"
    );

    // ARM 4 — and it decides regardless of the barrier's value, so arm 3 cannot
    // be passing for the wrong reason.
    assert!(
        !availability_with(&[
            ("CERULION_CREDIT_OS_SYNC", "0"),
            ("CERULION_BARRIER_OS_SYNC", "1"),
        ]),
        "CERULION_CREDIT_OS_SYNC=0 must disable the credit tier even with the \
         barrier tier explicitly ENABLED"
    );

    // ARM 5 — explicit `1` on both is the same as unset: enabled.
    assert!(
        availability_with(&[
            ("CERULION_CREDIT_OS_SYNC", "1"),
            ("CERULION_BARRIER_OS_SYNC", "0"),
        ]),
        "CERULION_CREDIT_OS_SYNC=1 with the barrier tier OFF is the exact \
         combination reported by the review: it must read AVAILABLE"
    );
}
