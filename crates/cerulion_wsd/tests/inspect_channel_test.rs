#![cfg(unix)]
//! The `--inspect-node` child's document-channel isolation.
//!
//! Two layers, because neither alone covers the contract:
//!
//! * a SELF-RE-EXEC probe of [`cerulion_wsd::inspect::isolate_document_channel`]
//!   — the CHILD (this test binary, re-run with `--exact`) calls it, then prints
//!   chatter to stdout the way a library's load-time code would, spawns an
//!   exec'd grandchild, and writes the document through the returned handle. The
//!   parent asserts the chatter reached stderr, the document is the tail of
//!   stdout, and the grandchild did NOT inherit the document channel. (libtest
//!   prints its own harness lines to stdout BEFORE the child's test body runs,
//!   which is why the document assertion is on the tail.) A second child arm
//!   closes fd 2 first and reports the refusal by exit code;
//! * the PRODUCTION binary (`cerulion-wsd --inspect-node <lib>`) run on a
//!   prebuilt fixture cdylib. Without it, a variant that writes the info JSON with
//!   `print!` instead of through the document handle stays green — the probe
//!   above never runs `main.rs`.
//!
//! The two REFUSAL arms are pinned differently, because only one of them is
//! reachable from a spawn at all. Rust's runtime re-opens `/dev/null` over a
//! descriptor closed at exec, so a spawn with fd 2 closed documents the library
//! normally and never reaches the fd-2 arm (that is itself asserted, by
//! `a_closed_stderr_at_exec_never_reaches_the_refusal_because_the_runtime_reopens_it`):
//! a variant that reports that refusal on the closed descriptor is caught only by
//! the structural pin `the_fd2_refusal_arm_reports_on_fd_1_never_on_stderr`.
//! Its sibling, the SURGERY refusal, needs a free descriptor and so IS reachable
//! from a plain spawn under an exhausted fd table —
//! `an_exhausted_fd_table_reaches_the_surgery_refusal_on_stderr_with_an_empty_document`
//! sweeps for that budget and pins the arm behaviourally.
//!
//! The `#[ignore]`d `inspect_channel_*_child` fns are the probe's own children,
//! env-gated on `CER_WSD_INSPECT_CHANNEL_CHILD`; they are harnesses, not box
//! tests, and returning immediately without that env var is what makes a
//! `-- --ignored` sweep harmless.

use std::io::Write as _;
use std::os::fd::AsRawFd as _;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_cli_engine::node_inspector::{InProcessInspector, NodeInspector as _};
use cerulion_core::graph::node::DylibNodeEntry;
use cerulion_wsd::inspect::IsolationRefusal;

const CHILD_ENV: &str = "CER_WSD_INSPECT_CHANNEL_CHILD";
/// The chatter/CLOEXEC arm.
const CHATTER_ARM: &str = "chatter";
/// The fd-2-closed refusal arm.
const CLOSED_STDERR_ARM: &str = "closed_stderr";

/// Distinct exit codes: a panic in a child would reach the parent as a bare
/// non-zero status among many, and the closed-stderr child cannot explain
/// itself on stderr at all — so each outcome gets a number.
const EXIT_NOT_CLOEXEC: i32 = 44;
const EXIT_REFUSED_NAMING_FD2: i32 = 42;
const EXIT_ISOLATED_ANYWAY: i32 = 43;
const EXIT_REFUSED_FOR_ANOTHER_REASON: i32 = 45;

/// How long the exec'd grandchild lives, and how long the parent gives the
/// document pipe to reach EOF after the child exits.
///
/// The margin is deliberately huge in BOTH directions: a loaded box can only
/// DELAY the healthy path (a false FAIL, never a false pass), and a wedged child's
/// hold is a hard floor no load can shorten.
const GRANDCHILD_HOLD_SECS: u64 = 10;
const DOCUMENT_RELEASE_BUDGET: Duration = Duration::from_secs(5);

/// The child tells the parent its grandchild's pid on stderr so the parent can
/// reap it once the timing has been measured (the healthy path returns long
/// before `sleep` would exit on its own).
const SLEEPER_PID_MARKER: &str = "CER_WSD_SLEEPER_PID=";

/// The fixture cdylib the production arms inspect.
const FIXTURE_CRATE: &str = "test_node_cdylib";

// ───────────────────────────────────────────────────────────────────────────
// The self-re-exec probe
// ───────────────────────────────────────────────────────────────────────────

#[test]
#[ignore = "self-re-exec child: run by isolation_routes_load_time_chatter_to_stderr..."]
fn inspect_channel_chatter_child() {
    if std::env::var(CHILD_ENV).ok().as_deref() != Some(CHATTER_ARM) {
        return;
    }
    let mut document = cerulion_wsd::inspect::isolate_document_channel().expect("isolate");

    // (a) The close-on-exec property, asserted directly on the descriptor. The
    // timing check below is the behavioral half; this one names the cause.
    // SAFETY: F_GETFD on a descriptor we own only reads its flags.
    let flags = unsafe { libc::fcntl(document.as_raw_fd(), libc::F_GETFD) };
    if flags < 0 || flags & libc::FD_CLOEXEC == 0 {
        // SAFETY: _exit(2) with a status; nothing to flush that matters.
        unsafe { libc::_exit(EXIT_NOT_CLOEXEC) }
    }

    // (b) An exec'd grandchild, spawned BEFORE the document is written, the way
    // a library constructor's vendor helper would be. It must NOT inherit the
    // document descriptor: if it does, the parent's document pipe stays open
    // for the grandchild's whole life and `output()` blocks on it.
    //
    // Its own 0/1/2 go to /dev/null ON PURPOSE. In THIS process fd 1 and fd 2
    // both point at the parent's STDERR pipe (that is what the isolation did),
    // so a grandchild inheriting them would hold that pipe open in BOTH arms
    // and the timing would stop discriminating. The document descriptor is the
    // only fd whose inheritance the property under test governs.
    // NOT waited on here, deliberately: waiting would release the descriptor
    // under test before the parent could observe whether it was inherited, and
    // this process leaves through `libc::exit` a few lines below anyway. The
    // PARENT reaps it by pid once the timing has been measured.
    #[allow(clippy::zombie_processes)]
    let sleeper = Command::new("sleep")
        .arg(GRANDCHILD_HOLD_SECS.to_string())
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn the exec'd grandchild");
    eprintln!("{SLEEPER_PID_MARKER}{}", sleeper.id());

    // What a library's load-time code does: Rust and C stdio, both to "stdout".
    println!("chatter from a load-time constructor");
    // SAFETY: printf to the C stdout stream with a plain literal.
    unsafe {
        libc::printf(c"buffered C chatter\n".as_ptr());
    }
    document.write_all(b"DOC").expect("write document");
    document.flush().expect("flush document");
    // Exit through libc so C stdio flushes (as a real child does at exit).
    // SAFETY: exit(3) with a status.
    unsafe { libc::exit(0) }
}

#[test]
#[ignore = "self-re-exec child: run by a_closed_stderr_is_refused_before_the_document_fd_is_taken"]
fn inspect_channel_closed_stderr_child() {
    if std::env::var(CHILD_ENV).ok().as_deref() != Some(CLOSED_STDERR_ARM) {
        return;
    }
    // SAFETY: close(2) on our own stderr. Nothing below writes to it; every
    // outcome leaves through `_exit` with a code, and the diagnosis rides
    // stdout — which the refusal must leave untouched, and which is the whole
    // point of the arm.
    unsafe { libc::close(2) };
    let code = match cerulion_wsd::inspect::isolate_document_channel() {
        Ok(_) => EXIT_ISOLATED_ANYWAY,
        Err(error) => {
            println!("REFUSAL={error}");
            let _ = std::io::stdout().flush();
            // The refusal must come from the fd-2 CHECK, not from `dup2(2, 1)`
            // failing EBADF after the document descriptor was already taken:
            // both are `Err`, and the KIND is what tells them apart. The
            // message is printed for the parent's diagnosis only — it is not
            // what the decision reads, here or in `main.rs`.
            if error.refusal() == IsolationRefusal::StderrUnusable {
                EXIT_REFUSED_NAMING_FD2
            } else {
                EXIT_REFUSED_FOR_ANOTHER_REASON
            }
        }
    };
    // SAFETY: _exit(2) with a status.
    unsafe { libc::_exit(code) }
}

/// Run one `#[ignore]`d child arm and return its finished output.
fn run_child_arm(arm: &str, test_name: &str) -> std::process::Output {
    let exe = std::env::current_exe().expect("current exe");
    Command::new(exe)
        .args(["--exact", test_name, "--ignored", "--nocapture"])
        .env(CHILD_ENV, arm)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("spawn child")
}

#[test]
fn isolation_routes_load_time_chatter_to_stderr_and_keeps_the_document_alone() {
    let started = Instant::now();
    let out = run_child_arm(CHATTER_ARM, "inspect_channel_chatter_child");
    let elapsed = started.elapsed();
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();

    // Reap the grandchild now that the timing is measured; in the healthy path
    // it is still sleeping. (Where it has already exited this is a no-op — the
    // pid cannot have been recycled inside its own lifetime.) Done BEFORE the
    // assertions so a failing arm still cleans up, and reported AFTER them so
    // an arm that never got as far as spawning fails under its own name.
    let sleeper_pid = stderr
        .lines()
        .find_map(|line| line.strip_prefix(SLEEPER_PID_MARKER))
        .and_then(|pid| pid.trim().parse::<i32>().ok())
        .filter(|pid| *pid > 1);
    if let Some(pid) = sleeper_pid {
        // SAFETY: kill(2) on a pid this process spawned a grandchild for.
        unsafe { libc::kill(pid, libc::SIGKILL) };
    }

    assert_ne!(
        out.status.code(),
        Some(EXIT_NOT_CLOEXEC),
        "the document descriptor is NOT close-on-exec — an exec'd grandchild inherits \
         the parent's document pipe (stderr: {stderr:?})"
    );
    assert!(out.status.success(), "child failed: {stderr}");
    assert!(
        sleeper_pid.is_some(),
        "the child did not report its grandchild's pid, so the timing assertion below \
         would be vacuous: {stderr:?}"
    );
    assert!(
        stdout.ends_with("DOC"),
        "the document must be the tail of stdout, got: {stdout:?}"
    );
    assert!(
        !stdout.contains("chatter"),
        "load-time chatter reached the document channel: {stdout:?}"
    );
    assert!(
        stderr.contains("chatter from a load-time constructor")
            && stderr.contains("buffered C chatter"),
        "chatter must land on stderr: {stderr:?}"
    );
    // The behavioral half of the close-on-exec pin: the document pipe reached
    // EOF while a `sleep GRANDCHILD_HOLD_SECS` the child exec'd was still
    // running, so that grandchild never held a copy of it.
    assert!(
        elapsed < DOCUMENT_RELEASE_BUDGET,
        "the document channel was still open {elapsed:?} after the child exited — an exec'd \
         grandchild inherited it (it holds for {GRANDCHILD_HOLD_SECS}s); the document \
         descriptor must be close-on-exec"
    );
}

#[test]
fn a_closed_stderr_is_refused_before_the_document_fd_is_taken() {
    let out = run_child_arm(CLOSED_STDERR_ARM, "inspect_channel_closed_stderr_child");
    let stdout = String::from_utf8_lossy(&out.stdout);
    match out.status.code() {
        Some(EXIT_REFUSED_NAMING_FD2) => {}
        Some(EXIT_ISOLATED_ANYWAY) => panic!(
            "isolation SUCCEEDED with stderr closed: fd 1 was redirected onto a closed \
             descriptor and load-time output has nowhere to go ({stdout})"
        ),
        Some(EXIT_REFUSED_FOR_ANOTHER_REASON) => panic!(
            "the refusal did not come from the fd-2 check — the document descriptor was \
             taken first and `dup2(2, 1)` failed afterwards ({stdout})"
        ),
        other => panic!("closed-stderr child exited with {other:?}; stdout: {stdout}"),
    }
}

// ───────────────────────────────────────────────────────────────────────────
// The production `cerulion-wsd --inspect-node` path
// ───────────────────────────────────────────────────────────────────────────

/// The prebuilt fixture cdylib. Resolved from THIS test binary's own location
/// (cargo puts integration tests in `<target>/<profile>/deps/`), so it is right
/// under any `CARGO_TARGET_DIR`; the sibling profile is tolerated, mirroring
/// `cerulion_core::testing::find_fixture_cdylib`.
///
/// A MISSING FIXTURE IS A FAILURE, not a skip, and that is this repo's
/// convention — `find_fixture_cdylib` panics naming the build command. It was a
/// skip here, and the skip was invisible: libtest DISCARDS a passing test's
/// stderr, so on a lane that never built the fixture both arms below printed
/// `... ok` and ran nothing. A cdylib is loaded with `libloading` at runtime,
/// so no `cargo test -p cerulion_wsd` can pull it in as a dependency; CI's
/// `crate-tests` job builds it in the step immediately before, and a bare desk
/// run gets the recipe out of this panic. Same reasoning for `current_exe` and
/// its parent: a harness that cannot locate itself is a broken harness, never
/// a reason to report two green pins.
fn fixture_cdylib() -> PathBuf {
    let exe = std::env::current_exe().expect("the test binary must know its own path");
    let dir = exe
        .parent()
        .expect("the test binary must live in a directory")
        .to_path_buf();
    let profile = if dir.file_name() == Some(std::ffi::OsStr::new("deps")) {
        dir.parent()
            .expect("cargo's `deps` directory must live under a profile directory")
            .to_path_buf()
    } else {
        dir
    };
    let file = cerulion_cli_engine::graph_cmd::cdylib_name(FIXTURE_CRATE);
    // The SAME profile as this test binary, and only that one: a release
    // fixture must not satisfy a debug run (or the reverse), or the arm would
    // inspect an artifact built under different settings from the binary it
    // is paired with — the repo's fixture contract
    // (`cerulion_core::testing::find_fixture_cdylib_same_profile`, not used
    // here because it sits behind `test-helpers`, a feature a dev-dependency
    // would unify into this crate's own build).
    let candidate = profile.join(&file);
    if candidate.exists() {
        return candidate;
    }
    panic!(
        "fixture cdylib `{file}` not found at {} — run `cargo build -p {FIXTURE_CRATE}` \
         first, under the SAME CARGO_TARGET_DIR and profile as this test (a sibling \
         profile's copy is deliberately not accepted). This arm runs the SHIPPED binary \
         and cannot be skipped: libtest discards a passing test's stderr, so a skip \
         would report green while proving nothing",
        candidate.display()
    );
}

fn inspect_node_output(fixture: &std::path::Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion-wsd"))
        .arg("--inspect-node")
        .arg(fixture)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("run cerulion-wsd --inspect-node")
}

#[test]
fn the_production_child_writes_the_whole_document_and_only_the_document() {
    let fixture = fixture_cdylib();
    let out = inspect_node_output(&fixture);
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    assert!(
        out.status.success(),
        "cerulion-wsd --inspect-node failed: {} / {stderr}",
        out.status
    );
    let stdout = String::from_utf8(out.stdout).expect("the document is UTF-8");

    // Oracle: this process loads the SAME library through cerulion_core and
    // asks for the same JSON. Two independent paths (a subprocess running the
    // shipped binary vs an in-process load), never the child against itself.
    let oracle_json = DylibNodeEntry::load(&fixture)
        .expect("load the fixture in-process")
        .info_json()
        .expect("fixture info JSON");
    assert!(
        !stdout.is_empty(),
        "the child produced NO document — the info JSON did not reach the document \
         channel (stderr: {stderr:?})"
    );
    assert_eq!(
        stdout, oracle_json,
        "the child's WHOLE stdout must be the info document, byte for byte"
    );

    // Anti-vacuity: the equality above must not be two empty or two junk
    // strings — what crossed really is an info document.
    let parsed: serde_json::Value =
        serde_json::from_str(&stdout).expect("the document must be JSON");
    assert!(
        parsed.get("inputs").is_some() && parsed.get("outputs").is_some(),
        "the document must be a node info document: {stdout}"
    );

    // And the parsed form agrees with what the ENGINE's own in-process
    // inspector — the seam the CLI uses — reports for the same library.
    let from_child = DylibNodeEntry::parse_info_json_labeled(stdout.trim(), "probe")
        .expect("the child's document must parse with the parent's parser");
    let from_engine = InProcessInspector
        .inspect(&fixture, "probe")
        .expect("engine in-process inspection");
    assert_eq!(
        format!("{from_child:?}"),
        format!("{from_engine:?}"),
        "the subprocess document and the in-process inspection must describe the same node"
    );
}

#[test]
fn a_failing_load_explains_on_stderr_and_leaves_the_document_channel_empty() {
    let dir = std::env::temp_dir().join(format!(
        "cerulion-wsd-inspect-notalib-{}-{}",
        std::process::id(),
        line!()
    ));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let not_a_library = dir.join(cerulion_cli_engine::graph_cmd::cdylib_name("not_a_node"));
    std::fs::write(&not_a_library, b"this is not a shared object\n").expect("write");

    let out = inspect_node_output(&not_a_library);
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    let _ = std::fs::remove_dir_all(&dir);

    assert!(
        !out.status.success(),
        "a file that is not a shared object must not inspect cleanly (stdout: {stdout:?})"
    );
    // The chatter this path really produces is the child's OWN diagnosis, and
    // it is loud: it must be on stderr, and the document channel must carry
    // nothing at all — a partial document is worse than none.
    assert!(
        stderr.contains("--inspect-node"),
        "the load failure must be explained on stderr: {stderr:?}"
    );
    assert!(
        stdout.is_empty(),
        "the document channel must stay empty when the load fails: {stdout:?}"
    );
}

#[test]
fn a_closed_stderr_at_exec_never_reaches_the_refusal_because_the_runtime_reopens_it() {
    let fixture = fixture_cdylib();
    // Anti-vacuity FIRST: prove `2>&-` really closes fd 2 in this shell (as
    // opposed to `2>/dev/null`, which leaves it open) — a shell that ignored it
    // would make everything below prove nothing.
    let probe = Command::new("/bin/sh")
        .arg("-c")
        .arg("exec 2>&-; printf x >&2")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("run the closed-stderr probe");
    assert!(
        !probe.status.success() && probe.stderr.is_empty() && probe.stdout.is_empty(),
        "`2>&-` did not close fd 2 in /bin/sh, so this arm proves nothing: {probe:?}"
    );

    let wsd = env!("CARGO_BIN_EXE_cerulion-wsd");
    let lib = fixture.to_str().expect("fixture path is UTF-8");
    assert!(
        !wsd.contains('\'') && !lib.contains('\''),
        "the shell wrapper below single-quotes these paths"
    );
    let out = Command::new("/bin/sh")
        .arg("-c")
        .arg(format!("exec '{wsd}' --inspect-node '{lib}' 2>&-"))
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("run cerulion-wsd under a closed stderr");
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();

    // MEASURED, and it is why the fd-1 report in `main.rs` is pinned on the
    // source rather than here: Rust's runtime opens `/dev/null` over any of
    // fd 0/1/2 that is closed at startup, so a stderr closed AT EXEC never
    // reaches the refusal arm — the child documents the library normally. If a
    // future toolchain drops that sanitization this arm fails, which is the
    // signal that the refusal path became reachable from a spawn.
    assert!(
        out.status.success(),
        "the runtime re-opens a closed fd 2, so the child should still document the \
         library; it exited {} with stdout {stdout:?}",
        out.status
    );
    let oracle_json = DylibNodeEntry::load(&fixture)
        .expect("load the fixture in-process")
        .info_json()
        .expect("fixture info JSON");
    assert_eq!(
        stdout, oracle_json,
        "a closed-at-exec stderr must not disturb the document channel"
    );
}

/// The SURGERY refusal, driven out of the shipped binary by a plain spawn.
///
/// The sibling structural pins say the fd-2 arm cannot be reached this way
/// (the runtime re-opens a closed fd 2). That reasoning does NOT carry to the
/// other arm: `F_DUPFD_CLOEXEC` needs a free descriptor, and an exhausted fd
/// table has none, so `ulimit -n` reaches it — which means this arm's contract
/// (exit 1, the explanation on STDERR, and the document channel left EMPTY) is
/// something a user can actually observe, and must be pinned behaviourally
/// rather than only in the source.
///
/// The window is SWEPT, never hardcoded: it sits wherever the runtime's own
/// descriptor appetite leaves exactly no spare, which moves with the tokio
/// version (`ulimit -n 9` on this desk, tokio 1.53 — below it the runtime
/// cannot build and panics, above it the surgery succeeds and the LOAD fails
/// instead). The sweep stops at the first budget that gets past isolation, so
/// its cost is a handful of spawns, and the transition from refusal to
/// past-isolation is itself the proof that the budget — not a permanently
/// broken binary — is what produced the refusals.
///
/// If NO budget in the sweep reaches the refusal (a platform whose runtime
/// takes exactly as many descriptors as the surgery needs would do it), this
/// arm has asserted nothing about the behaviour it exists to pin. On an
/// interactive desk it says so on stderr and returns — visible under
/// `cargo test -- --nocapture`, which is where a person is looking. On CI it
/// PANICS instead, because there a returning test is a green tick and the
/// stderr line is discarded. That is the one early return here, and it is that
/// case only.
#[test]
fn an_exhausted_fd_table_reaches_the_surgery_refusal_on_stderr_with_an_empty_document() {
    /// The refusal `main.rs` prints for `IsolationRefusal::SurgeryFailed`.
    const REFUSAL: &str = "could not separate the document channel from stdout";
    /// Every message the child prints about its own work starts with this, the
    /// refusal included — so "past isolation" is `!REFUSAL && CHILD_PREFIX`, and
    /// a runtime panic (which names neither) is neither.
    const CHILD_PREFIX: &str = "--inspect-node";
    /// fds 0/1/2 are the floor; 64 is far above any window a runtime that can
    /// start at all could occupy.
    const SWEEP_MIN: u32 = 3;
    const SWEEP_MAX: u32 = 64;

    let wsd = env!("CARGO_BIN_EXE_cerulion-wsd");
    // A path that cannot be loaded: past the isolation the child fails at
    // `dlopen`, so no fixture is needed and no run can produce a document.
    // (A run that DID produce one would fail the empty-stdout assertion, which
    // is the direction that matters.)
    let lib = "/nonexistent/cerulion-wsd-fd-sweep.so";
    assert!(
        !wsd.contains('\'') && !lib.contains('\''),
        "the shell wrapper below single-quotes these paths"
    );

    let mut refusals = 0usize;
    let mut reached_isolation = false;
    let mut trace: Vec<String> = Vec::new();
    for budget in SWEEP_MIN..=SWEEP_MAX {
        let out = Command::new("/bin/sh")
            .arg("-c")
            .arg(format!(
                "ulimit -n {budget}; exec '{wsd}' --inspect-node '{lib}'"
            ))
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .expect("run cerulion-wsd under a reduced fd budget");
        let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
        let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
        trace.push(format!(
            "ulimit -n {budget}: status {}, stdout {stdout:?}, stderr {:?}",
            out.status,
            // The panic banner is `\nthread 'main' (..) panicked at <loc>:\n<msg>`:
            // the HEADER alone names a source line and not the cause, so the
            // trace carries the first two non-blank lines (header | message).
            stderr
                .lines()
                .filter(|l| !l.trim().is_empty())
                .take(2)
                .collect::<Vec<_>>()
                .join(" | ")
        ));

        // A refusal is RECOGNISED on either channel and then required to be on
        // the right one. Recognising it on stderr alone would let the defect
        // this arm exists to catch — the refusal routed onto fd 1, into the
        // parent's document pipe — read as "no refusal at this budget" and
        // fall through to the loud no-op below.
        let combined = format!("{stdout}{stderr}");
        if combined.contains(REFUSAL) {
            refusals += 1;
            // THE contract of this arm, on a run a user can reproduce.
            assert_eq!(
                out.status.code(),
                Some(1),
                "a refused isolation must exit 1, not panic or succeed (ulimit -n {budget}): \
                 {stderr:?}"
            );
            assert!(
                stderr.contains(REFUSAL),
                "the surgery refusal must be explained on STDERR — stderr is perfectly good \
                 on this arm, and fd 1 is still the parent's DOCUMENT pipe (ulimit -n \
                 {budget}): stdout {stdout:?}, stderr {stderr:?}"
            );
            assert!(
                stdout.is_empty(),
                "the refusal must not reach the DOCUMENT channel — fd 1 is still the \
                 parent's document pipe at that point (ulimit -n {budget}): {stdout:?}"
            );
            assert!(
                stderr.contains(CHILD_PREFIX) && stderr.contains(lib),
                "the refusal on stderr must name the child and the library it was asked \
                 about (ulimit -n {budget}): {stderr:?}"
            );
            continue;
        }
        if combined.contains(CHILD_PREFIX) {
            // The surgery succeeded and the LOAD failed instead: we are past
            // the window, and everything below it has been asserted.
            reached_isolation = true;
            assert!(
                stdout.is_empty(),
                "a failing load must leave the document channel empty (ulimit -n {budget}): \
                 {stdout:?}"
            );
            break;
        }
        // Below the window the process cannot even get as far as this arm, and
        // that is tolerated — but on the SIGNATURE that shape actually has,
        // never on "anything that is not exit 1". MEASURED by running this
        // sweep with both channels PIPED, as it runs here (tokio 1.53, macOS
        // 26.0.1, /bin/sh = bash 3.2): budget 3 dies on SIGABRT out of dyld,
        // budgets 4-8 exit 101 from the failed runtime build, budget 9 is the
        // refusal. At 4-8 the exit code AND the text are two signatures of
        // ONE shape, neither alone "the whole": through a pipe the runtime's
        // panic banner does arrive (250-324 bytes: `Failed building the
        // Runtime: … Too many open files` at 4/5/8, tokio's `failed to create
        // UnixStream: … Too many open files` at 6/7 — same shape, two texts),
        // but its FIRST line is blank and its second is only the `panicked at
        // <file>:<line>` header — reading only
        // `lines().next()` would report those budgets as `stderr ""`; reading
        // the header alone would point at an unrelated source line, which
        // is why it now records header AND message. Both arms stay admitted (a tokio bump that
        // stops printing must not need this comment updated), and so is exit
        // 127, which is what ld.so reports on Linux when it cannot open a
        // shared object.
        //
        // Anything ELSE that neither refuses nor reaches the isolation is a
        // refusal that told nobody, and this is the assertion that says so: a
        // `SurgeryFailed` arm mutated to exit silently (any code but 1, with
        // nothing on either channel) would otherwise be read as "below the
        // window" at every budget and then leave `refusals == 0`.
        const EMFILE: &str = "Too many open files";
        let below_window = out.status.code().is_none()
            || matches!(out.status.code(), Some(101) | Some(127))
            || stderr.contains(EMFILE);
        assert!(
            below_window,
            "a run that carried neither the refusal nor the child's own output must be \
             from BELOW the window — signal-killed, or exit 101/127, or an explicit \
             {EMFILE:?} — and this one was none of those (ulimit -n {budget}): status \
             {}, stdout {stdout:?}, stderr {stderr:?}",
            out.status
        );
    }

    if refusals == 0 {
        let report = format!(
            "no fd budget in {SWEEP_MIN}..={SWEEP_MAX} reached the surgery refusal on this \
             platform, so the behavioural half of this arm did not run (the routing is still \
             pinned structurally by the_fd2_refusal_arm_reports_on_fd_1_never_on_stderr). \
             Sweep trace:\n{}",
            trace.join("\n")
        );
        // A skip is a PASS and libtest DISCARDS a passing test's stderr, so on
        // a lane the line below is invisible: this arm would report green
        // having asserted nothing about the behaviour it exists to pin. The
        // window can genuinely vanish (a runtime whose descriptor appetite
        // peaks above its steady state leaves no budget where the runtime
        // builds AND the surgery has no spare), and on an interactive desk
        // saying so under `--nocapture` is the right response — but on CI the
        // vanishing IS the news, so it fails there. Same rule, same reason, as
        // `cerulion_hygiene`'s `skip_as_root`.
        assert!(
            std::env::var_os("CI").is_none(),
            "FD SWEEP, and CI is set: {report}"
        );
        eprintln!("FD SWEEP: {report}");
        return;
    }
    assert!(
        reached_isolation,
        "the sweep saw {refusals} refusal(s) but never a budget that got PAST the isolation, \
         so the fd budget is not what produced them — the binary may be refusing \
         unconditionally. Sweep trace:\n{}",
        trace.join("\n")
    );
}

#[test]
fn the_production_child_isolates_the_document_channel_before_it_loads_the_library() {
    // Structural, deliberately: no fixture cdylib in this repo writes to fd 1
    // from a load-time constructor, so a variant that isolates the channel AFTER
    // `DylibNodeEntry::load` produces byte-identical output on every library we
    // can point it at. The ordering is the whole guarantee, so it is pinned on
    // the source instead of left unpinned.
    let main_rs = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/main.rs");
    let source = std::fs::read_to_string(&main_rs).expect("read main.rs");
    let code = code_only(&source);

    let isolate = code
        .find("isolate_document_channel()")
        .expect("main.rs must call isolate_document_channel()");
    let load = code
        .find("DylibNodeEntry::load(")
        .expect("main.rs must load the library with DylibNodeEntry::load()");
    assert!(
        isolate < load,
        "main.rs must isolate the document channel BEFORE loading the library — anything \
         the library prints while loading would otherwise land on the document channel"
    );
    assert!(
        code.contains("document\n            .write_all(json.as_bytes())"),
        "the info JSON must be written through the isolated document handle, never printed"
    );
}

#[test]
fn the_fd2_refusal_arm_reports_on_fd_1_never_on_stderr() {
    // Structural for a MEASURED reason, not for convenience: Rust's runtime
    // opens `/dev/null` over a closed fd 0/1/2 at startup (pinned by
    // `a_closed_stderr_at_exec_never_reaches_the_refusal_because_the_runtime_reopens_it`),
    // so no spawn can drive the production binary into THIS arm — the fd-2
    // one. Its sibling is a different story: the surgery arm is reachable from
    // a plain spawn under an exhausted fd table, and is pinned behaviourally
    // by `an_exhausted_fd_table_reaches_the_surgery_refusal_on_stderr_with_an_empty_document`.
    // Both still have to be right — the fd-2 arm fires when stderr is unusable, where an
    // `eprintln!` PANICS on the write error and costs exit 101 with an empty
    // stderr and no cause at all; the OTHER arm leaves stderr perfectly good
    // and fd 1 still the parent's DOCUMENT pipe, so reporting there would push
    // a diagnostic onto the one channel the isolation exists to keep clean.
    //
    // The pin is on the HANDLE, not only the macro: `writeln!(io::stderr(), …)`
    // and `io::stderr().write_all(…)` are both "not an `eprintln!`" while
    // reporting on exactly the descriptor the arm just found unusable.
    let main_rs = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/main.rs");
    let source = std::fs::read_to_string(&main_rs).expect("read main.rs");
    let code = code_only(&source);
    let stderr_unusable = code
        .find("IsolationRefusal::StderrUnusable =>")
        .expect("main.rs must route the refusal on `IsolationRefusal::StderrUnusable`");
    let surgery_failed = code
        .find("IsolationRefusal::SurgeryFailed =>")
        .expect("main.rs must route the refusal on `IsolationRefusal::SurgeryFailed`");
    assert!(
        stderr_unusable < surgery_failed,
        "the two arms must be found in declaration order for the slices below to be \
         each other's complement"
    );
    let fd1_arm = &code[stderr_unusable..surgery_failed];
    assert!(
        fd1_arm.contains("std::io::stdout()"),
        "the fd-2 refusal must be written to fd 1 by NAME — the one channel known open: \
         {fd1_arm}"
    );
    assert!(
        !fd1_arm.contains("stderr()"),
        "the fd-2 refusal must not name stderr at all — that is the descriptor it just \
         found unusable: {fd1_arm}"
    );
    assert!(
        fd1_arm.contains("writeln!"),
        "the fd-2 refusal must write its explanation, not swallow it: {fd1_arm}"
    );
    assert!(
        !fd1_arm.contains("eprintln!"),
        "`eprintln!` in the fd-2 arm PANICS on the write error, costing exit 101 with an \
         empty stderr: {fd1_arm}"
    );

    // The complement, and it is the half a one-armed pin loses: a refusal that
    // did NOT find stderr unusable must be explained THERE, never on fd 1.
    // Bounded at the shared `ExitCode::FAILURE` that ends the function, so the
    // slice cannot run on into unrelated code.
    let tail = &code[surgery_failed..];
    let end = tail
        .find("ExitCode::FAILURE")
        .expect("the refusal reporter must end by failing");
    let fd2_arm = &tail[..end];
    assert!(
        fd2_arm.contains("eprintln!"),
        "a refusal that leaves stderr usable must be explained on stderr: {fd2_arm}"
    );
    assert!(
        !fd2_arm.contains("stdout()"),
        "reporting a surgery failure on fd 1 writes a diagnostic into the parent's \
         DOCUMENT pipe, which is still what fd 1 is at that point: {fd2_arm}"
    );
}

/// `source` with `//` line comments and (nesting) `/* */` block comments
/// removed, so a walk cannot be satisfied — or defeated — by prose. String
/// literals are deliberately not modelled; the tokens above appear in no
/// literal in `main.rs`, which the anti-tautology `expect`s above enforce by
/// failing if a token is missing altogether.
fn code_only(source: &str) -> String {
    let bytes = source.as_bytes();
    let mut out = String::with_capacity(source.len());
    let mut i = 0;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 && bytes[i..].starts_with(b"//") {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
        } else if bytes[i..].starts_with(b"/*") {
            depth += 1;
            i += 2;
        } else if depth > 0 && bytes[i..].starts_with(b"*/") {
            depth -= 1;
            i += 2;
        } else {
            if depth == 0 {
                out.push(source[i..].chars().next().expect("char boundary"));
            }
            i += source[i..]
                .chars()
                .next()
                .map(char::len_utf8)
                .expect("char boundary");
            continue;
        }
    }
    out
}
