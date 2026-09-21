//! The ORPHAN VERDICT, over real processes.
//!
//! `ChildGuard`'s no-orphan verdict is the harness's headline guarantee, and
//! without this file it has ZERO coverage: `mp_support/mod.rs` carries no `#[test]`
//! at all, so an implementation returning `Clean { checked: noted.len() }` unconditionally
//! passes the entire suite — every e2e arm asserts `assert_clean()` on a run that
//! genuinely leaked nothing, which such an implementation satisfies perfectly.
//!
//! Producing a genuine orphan inside a PASSING e2e arm is a contradiction (the
//! arm would have to leak a worker on purpose), so the verdict half is driven
//! here instead: real child processes whose liveness this file controls exactly,
//! with the pid set injected through `note_workers_explicit`. The DISCOVERY half
//! — `worker_pids_of`'s `pgrep` against a real supervisor — stays covered by the
//! e2e arms, which is the only place a real `run-worker` subtree exists.
//!
//! Its OWN binary rather than a `#[cfg(test)]` module inside `mp_support`:
//! that module is `mod mp_support;`-included by fourteen e2e binaries, so an
//! inline test would be compiled and RUN fourteen times.
#![cfg(unix)]

use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

mod mp_support;
use mp_support::*;

/// A `sleep` child this file owns, killed and REAPED on drop.
///
/// Reaping matters more than killing here: `pid_is_gone` asks `kill(pid, 0)`,
/// which SUCCEEDS for a zombie, so an un-reaped corpse reads as alive and would
/// make a "gone" assertion hang on its own teardown.
struct Sleeper(std::process::Child);

impl Sleeper {
    fn new(secs: &str) -> Self {
        Self(
            Command::new("sleep")
                .arg(secs)
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
                .expect("spawn a sleep child"),
        )
    }
    fn pid(&self) -> u32 {
        self.0.id()
    }
    /// Kill AND reap, so the pid genuinely disappears.
    fn kill_and_reap(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

impl Drop for Sleeper {
    fn drop(&mut self) {
        self.kill_and_reap();
    }
}

/// A guard wrapping a child of our own, so the guard is real without needing a
/// supervisor. `single_process` is the matching constructor: a `sleep` leads no
/// group and owns no subtree.
fn guard_over_a_sleeper() -> (ChildGuard, u32) {
    let child = Command::new("sleep")
        .arg("30")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn the guarded child");
    let pid = child.id();
    (ChildGuard::single_process(child), pid)
}

/// A LIVE noted worker is `Leaked`, and it takes the grace to say so.
#[test]
fn a_noted_worker_that_is_still_alive_is_reported_leaked_after_the_grace() {
    let (mut guard, _) = guard_over_a_sleeper();
    let mut orphan = Sleeper::new("30");
    guard.note_workers_explicit(vec![orphan.pid()]);

    let started = Instant::now();
    let verdict = guard.orphan_verdict();
    let waited = started.elapsed();

    assert_eq!(
        verdict,
        OrphanVerdict::Leaked(vec![orphan.pid()]),
        "a noted worker still alive after the grace is a LEAK, and the verdict \
         must name its pid"
    );
    // The grace is SPENT before declaring — a verdict that fired instantly would
    // call every clean teardown a leak (a just-killed child stays visible until
    // it is reaped).
    assert!(
        waited >= Duration::from_secs(1),
        "the verdict must spend its grace before declaring a leak, took {waited:?}"
    );
    orphan.kill_and_reap();
}

/// The same noted worker, REAPED, is `Clean` — and promptly.
#[test]
fn a_noted_worker_that_is_gone_is_clean() {
    let (mut guard, _) = guard_over_a_sleeper();
    let mut worker = Sleeper::new("30");
    let pid = worker.pid();
    guard.note_workers_explicit(vec![pid]);
    worker.kill_and_reap();

    let started = Instant::now();
    let verdict = guard.orphan_verdict();
    assert_eq!(
        verdict,
        OrphanVerdict::Clean { checked: 1 },
        "a noted worker that is gone is Clean, and `checked` reports how many \
         pids the verdict actually looked at"
    );
    // ANTI-TAUTOLOGY for the arm above: the clean path returns on the FIRST
    // poll, so the 1 s floor there is measuring the grace and not this file's
    // own overhead.
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "a clean verdict must not wait out the grace"
    );
}

/// A worker that exits INSIDE the grace is `Clean`, not `Leaked`.
///
/// The condition is asserted, never a wall: the arm proves the grace LOOP
/// re-polls rather than sampling once, which is what separates a shutdown still
/// in progress from a real leak.
#[test]
fn a_worker_that_exits_inside_the_grace_is_clean_not_leaked() {
    let (mut guard, _) = guard_over_a_sleeper();
    let mut worker = Sleeper::new("30");
    let pid = worker.pid();
    guard.note_workers_explicit(vec![pid]);

    // Alive when the verdict starts; gone a few hundred ms in.
    let handle = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(250));
        worker.kill_and_reap();
    });
    let verdict = guard.orphan_verdict();
    handle.join().expect("reaper thread");

    assert_eq!(
        verdict,
        OrphanVerdict::Clean { checked: 1 },
        "a worker that exits INSIDE the grace is a teardown still finishing, not \
         a leak — the verdict must re-poll, not sample once"
    );
}

/// Nobody noted anything ⇒ `NotChecked`, which is deliberately NOT `Clean`.
#[test]
fn a_guard_that_noted_nothing_reports_not_checked() {
    let (guard, _) = guard_over_a_sleeper();
    assert_eq!(
        guard.orphan_verdict(),
        OrphanVerdict::NotChecked,
        "absence of evidence is not evidence of absence"
    );

    // An EMPTY noted set is the same claim: a `single_process` guard and a run
    // that aborted before spawning both legitimately have no workers.
    let (mut empty, _) = guard_over_a_sleeper();
    empty.note_workers_explicit(Vec::new());
    assert_eq!(empty.orphan_verdict(), OrphanVerdict::NotChecked);
}

/// `assert_clean` panics on `Leaked` — and passes on the other two.
#[test]
fn assert_clean_panics_on_leaked_and_passes_otherwise() {
    // The arms that must PASS, stated first so a panic in either is attributed
    // here rather than to the `catch_unwind` below.
    OrphanVerdict::Clean { checked: 3 }.assert_clean();
    OrphanVerdict::NotChecked.assert_clean();

    let leaked = OrphanVerdict::Leaked(vec![4242, 4243]);
    let err = std::panic::catch_unwind(|| leaked.assert_clean())
        .expect_err("assert_clean must PANIC on a Leaked verdict");
    let msg = err
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| err.downcast_ref::<&str>().map(|s| (*s).to_string()))
        .unwrap_or_default();
    // The pids are the actionable half — an operator kills what the line names.
    assert!(
        msg.contains("4242") && msg.contains("4243"),
        "the panic must NAME the leaked pids, got {msg:?}"
    );
    assert!(
        msg.contains("ORPHANED"),
        "and say what happened, got {msg:?}"
    );
}

/// A guard whose "worker" subtree appears LATE, over a real process group.
///
/// The pgrep the guard uses is `pgrep -P <child> -f run-worker`, so the fixture
/// spawns a group leader that — after a delay — forks a CHILD whose command line
/// carries that needle. A copied script name is used rather than `exec -a`, which
/// is a bash builtin: a `/bin/sh` that is dash would not have it.
/// The token `worker_pids_of` greps for in a child's argv.
///
/// Named here because two things must agree about it: the worker script's FILE
/// NAME (which is what puts the needle on the exec'd child) and the assertion
/// below that the supervisor's own argv does NOT carry it.
const WORKER_NEEDLE: &str = "run-worker";

fn guard_with_a_late_worker(
    dir: &std::path::Path,
) -> (ChildGuard, std::path::PathBuf, std::process::ChildStdin) {
    let worker = dir.join(WORKER_NEEDLE);
    let ready = dir.join("worker-arrived");
    // NO `exec`: an `exec sleep 30` REPLACES this process and the "run-worker"
    // name is lost from its command line, so `pgrep -f run-worker` then matched
    // only the PARENT (whose argv carries the script path) and never a child.
    // Keeping the script process alive is what puts the needle on a child.
    //
    // The worker PUBLISHES ITS OWN ARRIVAL before it settles, and the order is
    // what makes the file a sound witness: the `exec` that gives this process its
    // matchable argv happens BEFORE the script's first line runs, so the file
    // existing implies the pid is already matchable. The witness is conservative
    // in the right direction, never ahead of the thing it stands for.
    std::fs::write(
        &worker,
        "#!/bin/sh\n: > \"$CER_WORKER_ARRIVED\"\nsleep 30\n",
    )
    .expect("write the worker script");
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&worker, std::fs::Permissions::from_mode(0o755)).unwrap();
    }
    // THE NEEDLE MUST NOT APPEAR IN THE SUPERVISOR'S OWN ARGV, so the worker path
    // and the witness path travel in the ENVIRONMENT rather than in `-c`.
    //
    // MEASURED, and it is why this arm was flaky: `worker_pids_of` matches on
    // argv, and a child between `fork()` and `execve()` still carries its
    // PARENT's argv. While the supervisor's own command line was
    // `sh -c 'sleep 0.4; "<dir>/run-worker" & wait'` it CONTAINED the needle, so
    // the child forked for `sleep 0.4` matched it for the microseconds before it
    // exec'd — then exec'd away and exited at t~0.4. A first poll landing in that
    // window cached `Some([doomed_pid])`, and since `note_workers` re-notes only
    // while the set is `None`, nothing ever corrected it: the verdict later
    // checked a pid that was long gone and read `Clean { checked: 1 }`. Instrumented
    // on a loaded machine, that was 2 failures in 24 runs.
    //
    // With the paths in the environment the supervisor's argv is
    // `read _go; "$CER_WORKER" & wait` — no pre-exec child can inherit a matching
    // command line, so the window is closed BY CONSTRUCTION rather than by timing.
    // Every other `spawn_group_leader` fixture in this tree was checked: their
    // supervisor is the real `cerulion` binary (`graph run <graph> ...`), whose
    // argv never contains `run-worker`, so none of them can reach this shape.
    // "LATE" IS A CAUSAL FACT, NOT A TIMER.
    //
    // This spawned the worker behind `sleep 0.4` and called that "late". A sleep
    // is a wall-clock stand-in for causality: if the parent is descheduled past
    // 400 ms after the spawn — routinely, on a contended runner — the worker is
    // already up before the first poll runs, the cached-verdict path is never
    // exercised, and the arm passes having tested nothing. That is the same class
    // of defect this arm exists to pin, so it cannot be the way the arm is set up.
    //
    // The supervisor BLOCKS on a GO it reads from its stdin, and the parent
    // writes that GO only AFTER the first poll has returned. The ordering is
    // structural rather than asserted after the fact: the worker cannot exist
    // before the write, because the process that spawns it is parked in `read`
    // until then. There is no timing anywhere in the arm.
    let mut cmd = Command::new("sh");
    cmd.arg("-c")
        .arg("read _go; \"$CER_WORKER\" & wait")
        .env("CER_WORKER", &worker)
        .env("CER_WORKER_ARRIVED", &ready)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::null());

    // THE INVARIANT, ASSERTED OVER THE COMMAND'S REAL ARGV rather than over the
    // string a caller happened to build it from.
    //
    // `worker_pids_of` greps for `WORKER_NEEDLE` in a child's argv, and a child
    // between `fork()` and `execve()` still carries its PARENT's argv — so the
    // moment the supervisor's own command line contains the needle, any child it
    // forks becomes matchable in that pre-exec instant and a first poll can cache
    // a doomed pid.
    //
    // This guard first asserted `!script.contains(..)` on a local `&str`, which
    // was UNFAILABLE in the way that matters: it bound to the variable, not to
    // what `Command` carries, so restoring the interpolated path directly in
    // `.arg(format!(..))` put the needle back into the argv while the assertion
    // stayed true. Reading `get_program()` + `get_args()` binds it to the
    // artefact `sh` actually receives, whatever it was built from — and it runs
    // BEFORE the spawn, so a regression is caught without ever starting a child.
    for arg in std::iter::once(cmd.get_program()).chain(cmd.get_args()) {
        let rendered = arg.to_string_lossy();
        assert!(
            !rendered.contains(WORKER_NEEDLE),
            "the supervisor's own argv must not contain `{WORKER_NEEDLE}` — a \
             pre-exec child inherits it and `worker_pids_of` would match a pid that \
             is about to exec away and exit; pass the path through the ENVIRONMENT \
             instead. The offending argument was: {rendered}"
        );
    }
    let mut guard =
        ChildGuard::spawn_group_leader(&mut cmd).expect("spawn the late-worker group leader");
    let go = guard
        .stdin
        .take()
        .expect("the supervisor's stdin was piped");
    (guard, ready, go)
}

/// A worker that appears AFTER the first poll still reaches the verdict.
///
/// THE ARM THE SLEEPER PINS CANNOT BE. Those drive `guard_over_a_sleeper()`,
/// which is `single_process` — so `note_workers` takes its `Vec::new()` branch,
/// `pids` is ALWAYS empty, and the guard under test (`if !pids.is_empty()`) is
/// never executed at all: reverting it leaves those arms green. This one runs a real
/// `spawn_group_leader` whose subtree arrives late, and drives the CACHED path
/// (`try_wait_noting`, which notes only while the set is `None`) rather than
/// calling `note_workers` directly — calling it directly re-discovers regardless
/// of caching and would pass under the revert too.
#[test]
fn a_late_appearing_worker_is_noted_through_the_cached_path() {
    let dir = tempfile::tempdir().unwrap();
    let (mut guard, arrived, mut go) = guard_with_a_late_worker(dir.path());

    // POSITIVE CONTROL, so the late-appearance path is exercised rather than
    // assumed: the worker must genuinely not be up yet at the first poll. Without
    // this the arm would still pass against a fixture whose worker started at
    // t≈0, and it would then be pinning nothing about caching.
    assert!(
        !arrived.exists(),
        "positive control: the worker must not have arrived before the first poll — \
         its supervisor is parked in `read` until this test writes GO, so an arrival \
         here would mean the GO gate is not holding and the arm would be pinning nothing"
    );

    // FIRST poll at t≈0: the subtree does not exist yet, so nothing may be cached.
    let _ = guard.try_wait_noting().expect("try_wait");

    // ASSERT ON THE GUARD'S OWN STATE, not on a second `pgrep`. The old
    // precondition here was `worker_pids_of(..).is_empty()`, which re-ran the same
    // transient a moment later — by which time a doomed pre-exec child had already
    // exec'd away — so it read empty and PASSED while the bad note sat in the
    // cache. `NotChecked` is the thing this arm actually depends on, and a cached
    // `Some` never reverts to `None`, so it cannot be missed by a late look.
    assert_eq!(
        guard.orphan_verdict(),
        OrphanVerdict::NotChecked,
        "the first poll must note NOTHING: an empty set is deliberately not cached, \
         and a NON-empty one here would be a pid that does not belong to the worker"
    );

    // GO. Everything above happened with the supervisor parked in `read`, so the
    // worker provably did not exist for any of it. Releasing it here is what makes
    // "appeared after the first poll" a fact about ORDER rather than about how
    // long a sleep happened to be.
    use std::io::Write as _;
    go.write_all(b"go\n").expect("release the supervisor");
    go.flush().expect("flush the GO");
    drop(go);

    // RENDEZVOUS ON THE WORKER'S OWN EVIDENCE — a file it creates once and never
    // removes. A pid set from `pgrep` is a TRANSIENT (see the fixture's note); a
    // file that exists is monotone, so no poll can miss it by arriving late.
    let deadline = Instant::now() + Duration::from_secs(10);
    while !arrived.exists() {
        assert!(Instant::now() < deadline, "the late worker never arrived");
        std::thread::sleep(Duration::from_millis(20));
    }
    let workers = worker_pids_of(guard.id());
    assert_eq!(workers.len(), 1, "one late worker, got {workers:?}");

    // A LATER poll through the same cached path must now find it. Under the
    // reverted fix the empty set is already cached and this notes nothing.
    let _ = guard.try_wait_noting().expect("try_wait");
    assert_eq!(
        guard.orphan_verdict(),
        OrphanVerdict::Leaked(workers.clone()),
        "a worker that appeared after the first poll must reach the verdict — \
         `NotChecked` here means the empty note was cached"
    );

    // And it is `Clean` once that worker is gone.
    for pid in &workers {
        send_signal(*pid, libc::SIGKILL);
    }
    let deadline = Instant::now() + Duration::from_secs(10);
    while !workers.iter().all(|p| pid_is_gone(*p)) {
        assert!(Instant::now() < deadline, "the worker never exited");
        std::thread::sleep(Duration::from_millis(50));
    }
    assert_eq!(
        guard.orphan_verdict(),
        OrphanVerdict::Clean { checked: 1 },
        "the same noted set reads Clean once the worker is gone"
    );
}

/// A `single_process` guard stays `NotChecked` — correct by design.
///
/// Stated as its own arm so the `NotChecked` above can only mean "the empty note
/// was cached", never "this shape always reads NotChecked".
#[test]
fn a_single_process_guard_stays_not_checked() {
    let (mut guard, _) = guard_over_a_sleeper();
    let _ = guard.try_wait_noting().expect("try_wait");
    assert_eq!(
        guard.orphan_verdict(),
        OrphanVerdict::NotChecked,
        "a guard with no subtree has nothing to check — absence of evidence"
    );
}

/// `finish()` tears the guarded child down and returns the same verdict.
#[test]
fn finish_tears_down_the_child_and_returns_the_verdict() {
    let (mut guard, guarded_pid) = guard_over_a_sleeper();
    let mut worker = Sleeper::new("30");
    let pid = worker.pid();
    guard.note_workers_explicit(vec![pid]);
    worker.kill_and_reap();

    assert_eq!(guard.finish(), OrphanVerdict::Clean { checked: 1 });
    assert!(
        pid_is_gone(guarded_pid),
        "finish() must tear the guarded child down, pid {guarded_pid} survived"
    );
}

/// A REAPED child is never signalled again — neither as a pid nor as a pgid.
///
/// The hazard this pins is not a tidiness point. `finish()` and `Drop` are both
/// routinely reached AFTER `wait_bounded()` has reaped the child — the ordinary
/// shape of a passing arm — and a `teardown()` that ran `kill(-pgid, SIGTERM)` (a
/// group leader) or `kill(pid, SIGTERM)` (a single process) UNCONDITIONALLY
/// before it looked would misfire. Once reaped, that pid is free for the kernel
/// to reissue, so the signal lands on whatever process holds it now; on a shared
/// machine that is somebody else's work. `teardown()`'s SIGKILL arm carries the
/// same rule, so the two arms must agree.
///
/// BOTH constructors are driven, because both arms carry the guard and a test
/// that covered one would leave the other free to regress. The oracle is the
/// guard's `signals_sent()` counter rather than an intercepted `kill(2)`: an
/// LD_PRELOAD interceptor proves the same thing and only on Linux.
#[test]
fn a_reaped_child_is_never_signalled_on_teardown() {
    for leader in [false, true] {
        // `true` exits at once, so the wait below genuinely reaps it.
        let mut cmd = Command::new("true");
        cmd.stdout(Stdio::null()).stderr(Stdio::null());
        let mut guard = if leader {
            ChildGuard::spawn_group_leader(&mut cmd).expect("spawn the group leader")
        } else {
            ChildGuard::single_process(cmd.spawn().expect("spawn the child"))
        };

        // THE PRECONDITION, asserted rather than assumed: the child really is
        // reaped before teardown runs. Without this the test could pass by
        // never having had a live child at all.
        let status = guard
            .wait_bounded(Duration::from_secs(10))
            .expect("the child must exit within the bound");
        assert!(status.success(), "`true` should exit 0, got {status:?}");
        assert_eq!(
            guard.signals_sent(),
            0,
            "waiting must not signal anything (leader={leader})"
        );

        let _ = guard.finish();
        assert_eq!(
            guard.signals_sent(),
            0,
            "teardown after a REAP must send NO signal — the pid/pgid may have \
             been reissued to an unrelated process (leader={leader})"
        );
    }
}

/// The anti-tautology control: an UNREAPED child still gets its signal.
///
/// Without this arm the test above is satisfied by a `teardown()` that signals
/// nothing ever, which would silently disarm the whole harness — every e2e arm
/// would stop killing its supervisor and leak instead.
#[test]
fn an_unreaped_child_is_still_signalled_on_teardown() {
    for leader in [false, true] {
        let mut cmd = Command::new("sleep");
        cmd.arg("30").stdout(Stdio::null()).stderr(Stdio::null());
        let mut guard = if leader {
            ChildGuard::spawn_group_leader(&mut cmd).expect("spawn the group leader")
        } else {
            ChildGuard::single_process(cmd.spawn().expect("spawn the child"))
        };
        let pid = guard.id();

        // NOT waited on: the child is alive and unreaped when teardown runs.
        assert_eq!(
            guard.signals_sent(),
            0,
            "nothing sent yet (leader={leader})"
        );
        let _ = guard.finish();
        assert!(
            guard.signals_sent() >= 1,
            "an unreaped child MUST still be signalled, or teardown leaks it \
             (leader={leader})"
        );
        assert!(
            pid_is_gone(pid),
            "the unreaped child must actually be torn down, pid {pid} survived \
             (leader={leader})"
        );
    }
}
