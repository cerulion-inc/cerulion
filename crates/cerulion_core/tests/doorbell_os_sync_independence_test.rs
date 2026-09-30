// SPDX-License-Identifier: AGPL-3.0-only
//! The doorbell's macOS os_sync tier is INDEPENDENT of every sibling plane's
//! kill switch, and `CERULION_DOORBELL_OS_SYNC` is the only one that decides it.
//!
//! This pins a coupling that has already shipped once on the plane next door.
//! `credit_wake_word_primitive_available` used to ask the BARRIER for the
//! backend question, and the barrier's macOS arm ANDs in
//! `CERULION_BARRIER_OS_SYNC`, so disabling the barrier's tier silently
//! disabled the credit plane's, while the API page promised the two switches
//! were independent. The doorbell is the fourth switch on one backend and makes
//! the same promise in `docs/user-api.md`, so it carries the same pin: a gate
//! that asked `barrier::os_sync_active()` instead of its own switch passes every
//! other test in the tree.
//!
//! **Both halves.** `AVAIL` is the availability PREDICATE
//! (`wake_word_block_primitive_available`). `PARKWAIT` is the RUNTIME decision:
//! whether the guard's kernel block really ran. The credit file's second read
//! found that probing only the predicate left a mutant alive on the runtime
//! site, because the two consult the gate separately.
//!
//! **Why a subprocess.** Every switch is `OnceLock`-cached on first read, so one
//! process can observe exactly one combination. Each arm re-execs this binary
//! with the env it wants and reads the answer off stdout, the shape
//! `credit_os_sync_independence_test` uses.
//!
//! **Why macOS-only.** The wake word and its switch are macOS concepts; off
//! macOS `wake_word_block_primitive_available` is a compile-time `false` and
//! there is nothing to observe.
#![cfg(target_os = "macos")]

use std::process::Command;

thread_local! {
    /// The most recent child's kernel-block verdict, so an arm can assert the
    /// RUNTIME decision as well as the predicate.
    static PARKWAIT: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

/// The runtime block decision observed by the last [`availability_with`] call.
fn last_parkwait() -> bool {
    PARKWAIT.with(|c| c.get()).expect("a child ran")
}

const CHILD: &str = "child_prints_doorbell_primitive_availability";

/// Every os_sync switch in the tree. Cleared before each arm so an arm only ever
/// sets what it names: an inherited value from the invoking shell would decide
/// the answer for it and the test would pass for the wrong reason.
const ALL_SWITCHES: &[&str] = &[
    "CERULION_DOORBELL_OS_SYNC",
    "CERULION_BARRIER_OS_SYNC",
    "CERULION_CREDIT_OS_SYNC",
    "CERULION_PARK_OS_SYNC",
];

/// Re-exec this test binary with `envs` applied and read the child's verdicts.
fn availability_with(envs: &[(&str, &str)]) -> bool {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args(["--exact", CHILD, "--ignored", "--nocapture"]);
    for k in ALL_SWITCHES {
        cmd.env_remove(k);
    }
    for (k, v) in envs {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("spawn child");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout
        .lines()
        .find(|l| l.starts_with("AVAIL="))
        .unwrap_or_else(|| panic!("child printed no AVAIL= line; stdout was:\n{stdout}"));
    // FAIL CLOSED on an unparseable verdict. A bare equality test would coerce a
    // truncated line, a renamed marker or a child that panicked half way to
    // `false`, which SATISFIES every disabled-switch assertion below and turns
    // those arms into decoration.
    let verdict = match line.trim_end() {
        "AVAIL=true" => true,
        "AVAIL=false" => false,
        other => panic!("unparseable child verdict {other:?}; full stdout was:\n{stdout}"),
    };
    let pw = stdout
        .lines()
        .find(|l| l.starts_with("PARKWAIT="))
        .unwrap_or_else(|| panic!("child printed no PARKWAIT= line; stdout was:\n{stdout}"));
    let parked = match pw.trim_end() {
        "PARKWAIT=true" => true,
        "PARKWAIT=false" => false,
        other => {
            panic!("unparseable child PARKWAIT verdict {other:?}; full stdout was:\n{stdout}")
        }
    };
    PARKWAIT.with(|c| c.set(Some(parked)));
    // Permanent evidence that each arm really spawned a child and read a real
    // answer: a subprocess oracle that stopped spawning would otherwise pass in
    // milliseconds and look identical to a healthy run.
    eprintln!("  ARM env={envs:?} -> AVAIL={verdict} PARKWAIT={parked}");
    verdict
}

/// The child half: report BOTH of this process's answers for the CURRENT env.
///
/// The page is per-process (pid-scoped namespace) and nothing rings it, so the
/// block always reaches its timeout. `PARKWAIT=true` therefore means "a kernel
/// wait really ran", which is the bit under test, not "a wake arrived".
#[test]
#[ignore]
fn child_prints_doorbell_primitive_availability() {
    println!(
        "AVAIL={}",
        cerulion_core::doorbell::wake_word_block_primitive_available()
    );
    let ns = format!("dbindep_{}", std::process::id());
    let bell = cerulion_core::doorbell::Doorbell::open_owned(&ns, "/indep/data")
        .expect("the child owns its own doorbell page");
    let guard = cerulion_core::doorbell::ParkedDoorbellGuard::enter(&bell);
    let outcome = guard.wait(std::time::Duration::from_millis(2));
    println!(
        "PARKWAIT={}",
        outcome == cerulion_core::monitor_wait::AddrParkOutcome::Parked
    );
}

/// THE pin. Every arm against a hand-written expectation, and the expectation
/// comes from the ENV the arm set rather than from the predicate it is testing.
#[test]
fn dbi_the_doorbell_os_sync_tier_ignores_every_sibling_kill_switch() {
    // ARM 1, CONTROL and anti-vacuity. With no switch set the host must report
    // the tier AVAILABLE, otherwise this box has no usable os_sync backend
    // (macOS < 14.4, or the errno latch tripped) and every later arm would read
    // `false` for a reason that has nothing to do with the switches.
    let baseline = availability_with(&[]);
    if !baseline {
        eprintln!(
            "SKIP: no usable os_sync backend on this host (macOS < 14.4 or latched); \
             the switch-independence arms cannot be distinguished from an absent backend"
        );
        return;
    }
    assert!(
        last_parkwait(),
        "the control arm must also perform the kernel block, or the runtime half \
         of every later assertion is vacuous"
    );

    // ARM 2, THE HEADLINE. No sibling switch may decide this plane.
    for sibling in [
        "CERULION_BARRIER_OS_SYNC",
        "CERULION_CREDIT_OS_SYNC",
        "CERULION_PARK_OS_SYNC",
    ] {
        assert!(
            availability_with(&[(sibling, "0")]),
            "{sibling}=0 must NOT disable the doorbell tier: the switches are \
             documented as independent, so this gate must read the shared BACKEND \
             fact and its OWN switch, never a sibling's decision"
        );
        assert!(
            last_parkwait(),
            "{sibling}=0 must NOT stop the guard's kernel block either - probing \
             only the predicate left a mutant alive on the runtime site of the \
             plane next door"
        );
    }

    // ARM 3, the doorbell's own switch does decide, on both halves.
    assert!(
        !availability_with(&[("CERULION_DOORBELL_OS_SYNC", "0")]),
        "CERULION_DOORBELL_OS_SYNC=0 must disable the doorbell tier"
    );
    assert!(
        !last_parkwait(),
        "CERULION_DOORBELL_OS_SYNC=0 must also stop the guard's kernel block"
    );

    // ARM 4, and it decides regardless of a sibling's value, so arm 3 cannot be
    // passing because something else happened to be off.
    assert!(
        !availability_with(&[
            ("CERULION_DOORBELL_OS_SYNC", "0"),
            ("CERULION_BARRIER_OS_SYNC", "1"),
        ]),
        "CERULION_DOORBELL_OS_SYNC=0 must disable this tier even with a sibling \
         tier explicitly ENABLED"
    );

    // ARM 5, explicit `1` is the same as unset, with every sibling off. This is
    // the exact combination the coupled code answers wrong.
    assert!(
        availability_with(&[
            ("CERULION_DOORBELL_OS_SYNC", "1"),
            ("CERULION_BARRIER_OS_SYNC", "0"),
            ("CERULION_CREDIT_OS_SYNC", "0"),
            ("CERULION_PARK_OS_SYNC", "0"),
        ]),
        "CERULION_DOORBELL_OS_SYNC=1 with every sibling tier OFF must read \
         AVAILABLE"
    );
    assert!(
        last_parkwait(),
        "and the kernel block must run in that combination too"
    );
}
